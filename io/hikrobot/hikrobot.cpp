#include "hikrobot.hpp"

#include <libusb-1.0/libusb.h>

#include "tools/logger.hpp"

using namespace std::chrono_literals;

namespace io
{
HikRobot::HikRobot(std::string sn, double exposure_us, double gain, const std::string & vid_pid, bool flip, bool mirror)
: CameraBase(sn), exposure_us_(exposure_us), gain_(gain), queue_(1), daemon_quit_(false), vid_(-1), pid_(-1), flip_(flip), mirror_(mirror)
{
  set_vid_pid(vid_pid);
  if (libusb_init(NULL)) tools::logger()->warn("Unable to init libusb!");

  daemon_thread_ = std::thread{[this] {
    tools::logger()->info("HikRobot's daemon thread started.");

    capture_start();

    while (!daemon_quit_) {
      std::this_thread::sleep_for(100ms);

      if (capturing_) continue;

      capture_stop();
      reset_usb();
      capture_start();
    }

    capture_stop();

    tools::logger()->info("HikRobot's daemon thread stopped.");
  }};
}

HikRobot::~HikRobot()
{
  daemon_quit_ = true;
  if (daemon_thread_.joinable()) daemon_thread_.join();
  tools::logger()->info("HikRobot destructed.");
}

void HikRobot::read(cv::Mat & img, std::chrono::steady_clock::time_point & timestamp)
{
  CameraData data;
  queue_.pop(data);

  img = data.img;
  timestamp = data.timestamp;
}

bool HikRobot::try_read(cv::Mat & img, std::chrono::steady_clock::time_point & timestamp)
{
  CameraData data;
  bool read_full =  queue_.try_pop(data);

  if(read_full) {
    img = data.img;
    timestamp = data.timestamp;
    last_read_t = data.timestamp;
  }  
  return read_full;
}

bool HikRobot::ChoiceCamrea(MV_CC_DEVICE_INFO** pDeviceInfo, unsigned char* sn, size_t& cameraIndex){
    for(size_t i = 0; i < nDeviceNum; i++){
        std::cout<<"pDeviceInfo "<<i<<": "<<pDeviceInfo[i]->SpecialInfo.stUsb3VInfo.chSerialNumber<<std::endl;
        bool wl = true;
        for(int j  = 0; sn[j]!='\0';j++){
            if(sn[j] != pDeviceInfo[i]->SpecialInfo.stUsb3VInfo.chSerialNumber[j]) {
                wl = false;
                break;
            }
        }
        if(wl) {
            cameraIndex = i;
            return true;
        }
    }
    return false;
}

void HikRobot::capture_start()
{
  capturing_ = false;
  capture_quit_ = false;

  unsigned int ret;

  MV_CC_DEVICE_INFO_LIST device_list;

  ret = MV_CC_EnumDevices(MV_USB_DEVICE, &device_list);//枚举设备数量
  if(ret != MV_OK) {
      tools::logger()->warn("hik EnumDevices failed");
      return;
  }
  if(device_list.nDeviceNum == 0) {
    tools::logger()->warn("设备数量为0");
      return;
  }
  this->nDeviceNum = device_list.nDeviceNum;
  
  size_t cameraIndex = 0;
  bool exist = ChoiceCamrea(device_list.pDeviceInfo, (unsigned char*)camera_sn_.c_str(), cameraIndex);
  std::cout<<"camrea exist "<<exist<<std::endl;
  if(false){ // 注意：你原代码这里是 if(false)，保留你的原逻辑
    tools::logger()->warn("不存在hik相机 {}",camera_sn_);
    return;
  }

  ret = MV_CC_EnumDevices(MV_USB_DEVICE, &device_list);
  if (ret != MV_OK) {
    tools::logger()->warn("MV_CC_EnumDevices failed: {:#x}", ret);
    return;
  }

  if (device_list.nDeviceNum == 0) {
    tools::logger()->warn("Not found camera!");
    return;
  }

  ret = MV_CC_CreateHandle(&handle_, device_list.pDeviceInfo[cameraIndex]);
  if (ret != MV_OK) {
    tools::logger()->warn("MV_CC_CreateHandle failed: {:#x}", ret);
    return;
  }
  // MV_CC_SetGrabStrategy(handle_,MV_GrabStrategy_LatestImagesOnly);
  ret = MV_CC_OpenDevice(handle_);
  if (ret != MV_OK) {
    tools::logger()->warn("MV_CC_OpenDevice failed: {:#x}", ret);
    return;
  }

  set_enum_value("BalanceWhiteAuto", MV_BALANCEWHITE_AUTO_CONTINUOUS);
  set_enum_value("ExposureAuto", MV_EXPOSURE_AUTO_MODE_OFF);
  set_enum_value("GainAuto", MV_GAIN_MODE_OFF);
  set_float_value("ExposureTime", exposure_us_);

  MVCC_FLOATVALUE gainRange;
  MV_CC_GetFloatValue(handle_,"AutoGainUpperLimit", &gainRange);//获取增益值范围
  set_float_value("Gain", gain_*gainRange.fMax );
  MV_CC_SetFrameRate(handle_, 220);

  ret = MV_CC_StartGrabbing(handle_);
  if (ret != MV_OK) {
    tools::logger()->warn("MV_CC_StartGrabbing failed: {:#x}", ret);
    return;
  }

  capture_thread_ = std::thread{[this] {
    tools::logger()->info("HikRobot's capture thread started.");

    capturing_ = true;

    MV_FRAME_OUT raw;
    MV_CC_PIXEL_CONVERT_PARAM cvt_param;

    while (!capture_quit_) {
      // 删除了原有的 is_paused_ 锁死机制 (pause_cv_.wait)

      if (is_paused_) {
          std::unique_lock<std::mutex> lock(pause_mutex_);
          // 线程在这里完全停滞，CPU占用绝对 0%，直到 resume() 中调用 notify_all 唤醒它
          this->queue_.clear();
          pause_cv_.wait(lock, [this]() { return !is_paused_.load(); });
      }

      // tools::logger()->info("q.size=={}", this->queue_.size());

      unsigned int ret;
      unsigned int nMsec = 100;

      // 1. 获取图像缓存
      ret = MV_CC_GetImageBuffer(handle_, &raw, nMsec);
      
      if (ret != MV_OK) {
        tools::logger()->warn("MV_CC_GetImageBuffer failed: {:#x} 海康相机无法读取到图像", ret);
        break; // 真实的获取失败，跳出循环，让 daemon_thread_ 触发重连
      }


      // 3. 正常处理逻辑（非休眠状态下执行）
      auto timestamp = std::chrono::steady_clock::now();
      cv::Mat img(cv::Size(raw.stFrameInfo.nWidth, raw.stFrameInfo.nHeight), CV_8U, raw.pBufAddr);

      cvt_param.nWidth = raw.stFrameInfo.nWidth;
      cvt_param.nHeight = raw.stFrameInfo.nHeight;
      cvt_param.pSrcData = raw.pBufAddr;
      cvt_param.nSrcDataLen = raw.stFrameInfo.nFrameLen;
      cvt_param.enSrcPixelType = raw.stFrameInfo.enPixelType;
      cvt_param.pDstBuffer = img.data;
      cvt_param.nDstBufferSize = img.total() * img.elemSize();
      cvt_param.enDstPixelType = PixelType_Gvsp_BGR8_Packed;

      const auto & frame_info = raw.stFrameInfo;
      auto pixel_type = frame_info.enPixelType;
      cv::Mat dst_image;
      const static std::unordered_map<MvGvspPixelType, cv::ColorConversionCodes> type_map = {
        {PixelType_Gvsp_BayerGR8, cv::COLOR_BayerGR2RGB},
        {PixelType_Gvsp_BayerRG8, cv::COLOR_BayerRG2RGB},
        {PixelType_Gvsp_BayerGB8, cv::COLOR_BayerGB2RGB},
        {PixelType_Gvsp_BayerBG8, cv::COLOR_BayerBG2RGB}};
      
      cv::cvtColor(img, dst_image, type_map.at(pixel_type));
      img = dst_image;
      
      // 翻转和镜像
      if (flip_) {
          cv::flip(img, img, 0); // 垂直翻转
      }
      if (mirror_) {
          cv::flip(img, img, 1); // 水平镜像
      }

      queue_.push({img, timestamp});

      // 4. 正常处理完毕后，释放缓存块
      ret = MV_CC_FreeImageBuffer(handle_, &raw);
      if (ret != MV_OK) {
        tools::logger()->warn("MV_CC_FreeImageBuffer failed: {:#x}", ret);
        break;
      }
    }

    capturing_ = false;
    tools::logger()->info("HikRobot's capture thread stopped.");
  }};
}

void HikRobot::capture_stop()
{
  capture_quit_ = true;
  if (capture_thread_.joinable()) capture_thread_.join();

  unsigned int ret;

  ret = MV_CC_StopGrabbing(handle_);
  if (ret != MV_OK) {
    tools::logger()->warn("MV_CC_StopGrabbing failed: {:#x}", ret);
    return;
  }

  ret = MV_CC_CloseDevice(handle_);
  if (ret != MV_OK) {
    tools::logger()->warn("MV_CC_CloseDevice failed: {:#x}", ret);
    return;
  }

  ret = MV_CC_DestroyHandle(handle_);
  if (ret != MV_OK) {
    tools::logger()->warn("MV_CC_DestroyHandle failed: {:#x}", ret);
    return;
  }
}

void HikRobot::set_float_value(const std::string & name, double value)
{
  unsigned int ret;

  ret = MV_CC_SetFloatValue(handle_, name.c_str(), value);

  if (ret != MV_OK) {
    tools::logger()->warn("MV_CC_SetFloatValue(\"{}\", {}) failed: {:#x}", name, value, ret);
    return;
  }
}

void HikRobot::set_enum_value(const std::string & name, unsigned int value)
{
  unsigned int ret;

  ret = MV_CC_SetEnumValue(handle_, name.c_str(), value);

  if (ret != MV_OK) {
    tools::logger()->warn("MV_CC_SetEnumValue(\"{}\", {}) failed: {:#x}", name, value, ret);
    return;
  }
}

void HikRobot::set_vid_pid(const std::string & vid_pid)
{
  auto index = vid_pid.find(':');
  if (index == std::string::npos) {
    tools::logger()->warn("Invalid vid_pid: \"{}\"", vid_pid);
    return;
  }

  auto vid_str = vid_pid.substr(0, index);
  auto pid_str = vid_pid.substr(index + 1);

  try {
    vid_ = std::stoi(vid_str, 0, 16);
    pid_ = std::stoi(pid_str, 0, 16);
  } catch (const std::exception &) {
    tools::logger()->warn("Invalid vid_pid: \"{}\"", vid_pid);
  }
}

void HikRobot::reset_usb() const
{
  if (vid_ == -1 || pid_ == -1) return;

  auto handle = libusb_open_device_with_vid_pid(NULL, vid_, pid_);
  if (!handle) {
    tools::logger()->warn("Unable to open usb!");
    return;
  }

  if (libusb_reset_device(handle))
    tools::logger()->warn("Unable to reset usb!");
  else
    tools::logger()->info("Reset usb successfully :)");

  libusb_close(handle);
}

void HikRobot::pause() {
    this->is_paused_ = true; 
    this->queue_.clear();
    // 不再向硬件发送 MV_CC_StopGrabbing
}

void HikRobot::resume() {
    this->is_paused_ = false; 
    // 不再发送硬件指令或通知条件变量
    // MV_CC_ClearImageBuffer(handle_);
    // MV_CC_FreeImageBuffer(handle_, );
    if(!is_paused_) pause_cv_.notify_all(); // 唤醒正在沉睡的线程
}

}  // namespace io