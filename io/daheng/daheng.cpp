// 实现文件
#include "daheng.hpp"
#include <chrono>
#include <functional>
#include <vector>

namespace io
{

#define DAHENG_CHECK(status, msg) \
    if (status != GX_STATUS_SUCCESS) { \
        tools::logger()->error("[Daheng] 相机错误信息 [{}] at {}: {}", status, msg, #status); \
        return false; \
    }

// 新增：枚举设备并确认相机存在
bool DahengCamera::enum_and_check_camera()
{
    uint32_t device_num = 0;
    GX_STATUS status = GXUpdateDeviceList(&device_num, 1000); // 1秒超时
    if (status != GX_STATUS_SUCCESS || device_num == 0) {
        tools::logger()->warn("[Daheng] 大恒相机设备数量为0或者sdk错误  error: {:#x}", status);
        return false;
    }

    // 枚举所有设备
    GX_DEVICE_BASE_INFO* pDeviceList = new GX_DEVICE_BASE_INFO[device_num];
    size_t size = device_num * sizeof(GX_DEVICE_BASE_INFO);
    status = GXGetAllDeviceBaseInfo(pDeviceList, &size);
    
    if (status == GX_STATUS_SUCCESS) {
        for (uint32_t i = 0; i < device_num; ++i) {
            if (camera_sn_ == pDeviceList[i].szSN) {
                tools::logger()->info("[Daheng] 找到目标相机 SN={}", pDeviceList[i].szSN);
                delete[] pDeviceList;
                return true;
            }
        }
        tools::logger()->warn("[Daheng] 在 {} 个设备中未找到目标相机 SN {} ", device_num, camera_sn_ );
    } else {
        tools::logger()->error("[Daheng] 枚举摄像头设备失败: {:#x}", status);
    }
    
    delete[] pDeviceList;
    return false;
}


DahengCamera::DahengCamera(std::string camera_sn, 
                            double exposure_us, 
                            double gain, 
                            double gamma,
                            bool flip,
                            bool mirror
                        )
    : CameraBase(camera_sn),
      exposure_us_(exposure_us), 
      gain_(gain), 
      gamma_(gamma),
      queue_(1),
      flip_(flip), mirror_(mirror)
{
    // tools::logger()->info("[Daheng] 初始化大恒相机SDk...");
    
    DahengCamera::initSDK();
    sdk_initialized_ = true;
    
    std::this_thread::sleep_for(std::chrono::milliseconds(500)); // 给设备准备时间
    

    daemon_thread_ = std::thread([this](){

        // 关键修复：先枚举确认相机存在，等待1秒让设备准备好
        bool camera_enum = enum_and_check_camera();
        if (!camera_enum) {
            tools::logger()->warn("[Daheng] 初始相机检查失败，将在守护线程中重试...");
        }

        if (open_camera()) { /* 初次连接 */ }
        while (!daemon_quit_) {
            std::this_thread::sleep_for(std::chrono::milliseconds(500));

            if(!camera_enum) {
               camera_enum =  enum_and_check_camera();
               continue;
            }

            if ((!capture_quit_ && hDevice == nullptr) || !capturing_) {
                tools::logger()->warn("[Daheng] SN={} 正在尝试从重新连接...", this->camera_sn_);
                capture_stop();                  // 安全停止采集线程
                close_camera();                  // 关闭旧设备
                stop_collecting_num = 0;
                capture_quit_ = false;

                if (open_camera()) {
                    tools::logger()->info("[Daheng] 相机连接成功。");
                    capturing_ = true;
                } else {
                    std::this_thread::sleep_for(std::chrono::seconds(1));
                }
            }
        }
        capture_stop();
        close_camera();
    });
    
    tools::logger()->info("[Daheng] 大恒相机封装已初始化。");
}




DahengCamera::~DahengCamera()
{
    tools::logger()->info("[Daheng] 大恒相机析构函数已调用。");
    
    daemon_quit_ = true;
    if (daemon_thread_.joinable()) {
        daemon_thread_.join();
    }
    
    capture_stop();
    close_camera();
    
    if (sdk_initialized_) {
        GXCloseLib();
        sdk_initialized_ = false;
    }
}



bool DahengCamera::capture_stop()
{
    // 修复：正确设置退出标志
    capture_quit_ = true;
    
    if (capture_thread_.joinable()) {
        capture_thread_.join();
        tools::logger()->debug("[Daheng] 采集线程joined.");
    }
    
    if (hDevice != nullptr) {
        GX_STATUS status = GXSendCommand(hDevice, GX_COMMAND_ACQUISITION_STOP);
        if (status != GX_STATUS_SUCCESS) {
            tools::logger()->warn("[Daheng] 未能停止采集: {:#x}", status);
        }
    }
    
    // 安全释放资源
    if(frameData.pImgBuf != nullptr){
        free(frameData.pImgBuf);
        frameData.pImgBuf = nullptr;
    }
    if(pRaw8Buffer != nullptr){
        free(pRaw8Buffer);
        pRaw8Buffer = nullptr;
    }
    if(pMirrorBuffer != nullptr){
        free(pMirrorBuffer);
        pMirrorBuffer = nullptr;
    }
    if(pRGBframeData != nullptr){
        free(pRGBframeData);
        pRGBframeData = nullptr;
    }
    if(pGammaLut != nullptr){
        delete[] static_cast<int*>(pGammaLut);
        pGammaLut = nullptr;
    }
    
    capturing_ = false;
    tools::logger()->info("[Daheng]");
    return true;
}



bool DahengCamera::close_camera()
{
    if (hDevice == nullptr) {
        return true; // 已经关闭
    }
    
    GX_STATUS status = GXCloseDevice(hDevice);
    if (status != GX_STATUS_SUCCESS) {
        tools::logger()->error("Failed to close device: {:#x}", status);
        return false;
    }
    
    hDevice = nullptr;
    tools::logger()->info("Camera closed.");
    return true;
}


void DahengCamera::read(cv::Mat & img, std::chrono::steady_clock::time_point & timestamp)
{
  CameraData data;
  queue_.pop(data);

  img = data.img;
  timestamp = data.timestamp;
}

bool DahengCamera::try_read(cv::Mat & img, std::chrono::steady_clock::time_point & timestamp)
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


bool DahengCamera::initialize_camera()
{
    if (capture_quit_) {
        tools::logger()->warn("Camera already initializing/running!");
        return true;
    }
    
    if (!sdk_initialized_) {
        tools::logger()->error("SDK not initialized!");
        return false;
    }
    
    // 确保设备列表已更新
    if (!enum_and_check_camera()) {
        return false;
    }
    
    open_content_ = camera_sn_;  // 复制字符串

    GX_OPEN_PARAM open_param;
    open_param.openMode = GX_OPEN_SN;
    open_param.accessMode = GX_ACCESS_EXCLUSIVE;
    open_param.pszContent = &open_content_[0];  // 使用成员变量的指针
    
    GX_STATUS status = GXOpenDevice(&open_param, &hDevice);
    if (status != GX_STATUS_SUCCESS) {
        tools::logger()->warn("Failed to open daheng camera {}: {:#x}", camera_sn_, status);
        hDevice = nullptr;
        return false;
    }
    
    // 获取图像尺寸
    int64_t nPayLoadSize = 0;
    status = GXGetInt(hDevice, GX_INT_PAYLOAD_SIZE, &nPayLoadSize);
    DAHENG_CHECK(status, "GetPayloadSize");
    
    // 分配缓冲区
    frameData.pImgBuf = malloc(static_cast<size_t>(nPayLoadSize));
    pRaw8Buffer = malloc(nPayLoadSize);
    pMirrorBuffer = malloc(nPayLoadSize * 3);
    pRGBframeData = malloc(nPayLoadSize * 3);
    
    // 获取格式
    GXGetEnum(hDevice, GX_ENUM_PIXEL_FORMAT, &PixelFormat);
    GXGetEnum(hDevice, GX_ENUM_PIXEL_COLOR_FILTER, &ColorFilter);
    
    // 设置连续采集模式
    status = GXSetEnum(hDevice, GX_ENUM_ACQUISITION_MODE, GX_ACQ_MODE_CONTINUOUS);
    DAHENG_CHECK(status, "SetAcquisitionMode");
    
    // // 关闭自动光源预设
    // status = GXSetEnum(hDevice, GX_ENUM_LIGHT_SOURCE_PRESET, GX_LIGHT_SOURCE_PRESET_OFF);
    // DAHENG_CHECK(status, "SetLightSource");
    
    // 设置白平衡
    GXSetEnum(hDevice, GX_ENUM_BALANCE_WHITE_AUTO,
              auto_white_balance_ ? GX_BALANCE_WHITE_AUTO_CONTINUOUS : GX_BALANCE_WHITE_AUTO_OFF);
    
    // 设置曝光
    status = GXSetFloat(hDevice, GX_FLOAT_EXPOSURE_TIME, exposure_us_);
    DAHENG_CHECK(status, "SetExposure");
    
    // 设置增益
    GXSetEnum(hDevice, GX_ENUM_GAIN_SELECTOR, GX_GAIN_SELECTOR_ALL);
    GX_FLOAT_RANGE gainRange;
    GXGetFloatRange(hDevice, GX_FLOAT_GAIN, &gainRange);
    double actual_gain = gainRange.dMin + (gainRange.dMax - gainRange.dMin) * gain_;
    status = GXSetFloat(hDevice, GX_FLOAT_GAIN, actual_gain);
    DAHENG_CHECK(status, "SetGain");
    
    // 设置Gamma
    GXSetBool(hDevice, GX_BOOL_GAMMA_ENABLE, true);
    status = GXSetEnum(hDevice, GX_ENUM_GAMMA_MODE, GX_GAMMA_SELECTOR_USER);
    if (status == GX_STATUS_SUCCESS) { 
        status = GXSetFloat(hDevice, GX_FLOAT_GAMMA, gamma_);
        DAHENG_CHECK(status, "SetGammaValue");
        // 计算Gamma LUT
        int nLutLength = 0;
        VxInt32 DXStatus = DxGetGammatLut(gamma_, nullptr, &nLutLength);
        if (DXStatus == DX_OK && nLutLength > 0) {
            pGammaLut = new int[nLutLength];
            DXStatus = DxGetGammatLut(gamma_, pGammaLut, &nLutLength);
            if (DXStatus != DX_OK) {
                delete[] pGammaLut;
                pGammaLut = nullptr;
            }
        }
    }
    else{
        tools::logger()->info("Daheng Camera Set Gamma Mode Error: {:#x}", status);
    }

    

    
    // 启动采集
    status = GXSendCommand(hDevice, GX_COMMAND_ACQUISITION_START);
    DAHENG_CHECK(status, "AcquisitionStart");
    
    // 修复：正确设置状态标志
    capture_quit_ = false;  // false = 运行中
    capturing_ = true;
    
    tools::logger()->info("Daheng camera initialized and started successfully.");
    
        // 启动采集线程
    capture_thread_ = std::thread([&]() {
        tools::logger()->info("Capture thread started.");
        while (!capture_quit_) {
            
            if (is_paused_) {
                std::unique_lock<std::mutex> lock(pause_mutex_);
                // 线程在这里完全停滞，CPU占用绝对 0%，直到 resume() 中调用 notify_all 唤醒它
                this->queue_.clear();
                pause_cv_.wait(lock, [this]() { return !is_paused_.load(); });
            }

            cv::Mat frame = getFrame();

            // if (is_paused_.load()) {
            // stop_collecting_num = 0; // 假装还在正常工作，防止触发重连
            // std::this_thread::sleep_for(std::chrono::milliseconds(5)); // 稍微休眠，防止死循环吃满单核 CPU
            // continue;
            // }
        
            if (!frame.empty()) {
                CameraData data;
                data.img = frame;
                data.timestamp = std::chrono::steady_clock::now();
                queue_.push(data);
                // tools::logger()->info("[daheng] 相机采集到数据");
                capturing_ = true;
                stop_collecting_num = 0;
            } else {
                std::this_thread::sleep_for(std::chrono::milliseconds(5)); // 避免空转
                stop_collecting_num++;
                if(stop_collecting_num > 10) 
                {
                    stop_collecting_num = 11;
                    capturing_ = false;
                    tools::logger()->debug("[daheng] 相机sn = {}读取不到数据", this->camera_sn_);
                }
                // tools::logger()->debug("[daheng] 相机读取不到数据");

            }
        }
        tools::logger()->info("[Daheng] sn = {}采集线程已停止", this->camera_sn_);
    });
    
    return true;
}




bool DahengCamera::open_camera()
{
    if (hDevice != nullptr) {
        tools::logger()->warn("[Daheng] 相机sn = {}已打开", this->camera_sn_);
        return true;
    }
    return initialize_camera();
}

cv::Mat DahengCamera::getFrame() {
    if (GXGetImage(hDevice, &frameData, 100) == GX_STATUS_SUCCESS) {//在开始采集之后，通过此接口可以直接获取图像，注意此接口不能与回调采集方式混用。

        if (frameData.nStatus == 0) {


            ProcessData(frameData.pImgBuf, pRaw8Buffer, pRGBframeData, frameData.nWidth, frameData.nHeight,
                        (int) PixelFormat, mirror_ ? 2 : 4, flip_, mirror_);
            cv::Mat src(cv::Size(frameData.nWidth, frameData.nHeight), CV_8UC3, pRGBframeData);
            return src.clone();//这里不做深拷贝的话多相机imshow画面会出现横杠
        }
    }

    return {};
}

void DahengCamera::ProcessData(void *pImageBuf, void *pImageRaw8Buf, void *pImageRGBBuf, int nImageWidth, int nImageHeight,
                        int nPixelFormat, int nPixelColorFilter, bool flip , bool mirror ) {
        
        
    
    switch (nPixelFormat) {
        //DxImageMirror:该函数为产生一个与原图像在水平方向或者垂直方向相对称的镜像图像,输入图像为 8 位的 Raw 图像或 8 位的黑白图像。
        //DxRaw16toRaw8:该函数将 Raw16 图像（实际位数为 16 位，有效位为 10 位或者 12 位）转换成 Raw8 位图像（实际位数和有效位数都是 8 位）。
        //DxRaw8toRGB24:该函数用于将 Bayer 图像转换为 RGB 图像。bFlip 是否翻转；true，将图像进行上下翻转；FALSE，不翻转
        case GX_PIXEL_FORMAT_BAYER_GR12:
        case GX_PIXEL_FORMAT_BAYER_RG12:
        case GX_PIXEL_FORMAT_BAYER_GB12:
        case GX_PIXEL_FORMAT_BAYER_BG12:
            if (mirror) {
                DxImageMirror(pImageBuf, pMirrorBuffer, nImageWidth, nImageHeight, HORIZONTAL_MIRROR);
                DxRaw16toRaw8(pMirrorBuffer, pImageRaw8Buf, nImageWidth, nImageHeight, DX_BIT_4_11);
                DxRaw8toRGB24(pImageRaw8Buf, pImageRGBBuf, nImageWidth, nImageHeight, RAW2RGB_NEIGHBOUR,
                                DX_PIXEL_COLOR_FILTER(nPixelColorFilter), flip);
            } else {
                DxRaw16toRaw8(pImageBuf, pImageRaw8Buf, nImageWidth, nImageHeight, DX_BIT_4_11);
                DxRaw8toRGB24(pImageRaw8Buf, pImageRGBBuf, nImageWidth, nImageHeight, RAW2RGB_NEIGHBOUR,
                                DX_PIXEL_COLOR_FILTER(nPixelColorFilter), flip);
            }
            break;

        case GX_PIXEL_FORMAT_BAYER_GR10:
        case GX_PIXEL_FORMAT_BAYER_RG10:
        case GX_PIXEL_FORMAT_BAYER_GB10:
        case GX_PIXEL_FORMAT_BAYER_BG10:
            if (mirror) {
                DxImageMirror(pImageBuf, pMirrorBuffer, nImageWidth, nImageHeight, HORIZONTAL_MIRROR);
                DxRaw16toRaw8(pMirrorBuffer, pImageRaw8Buf, nImageWidth, nImageHeight, DX_BIT_2_9);
                DxRaw8toRGB24(pImageRaw8Buf, pImageRGBBuf, nImageWidth, nImageHeight, RAW2RGB_NEIGHBOUR,
                                DX_PIXEL_COLOR_FILTER(nPixelColorFilter), flip);
            } else {
                DxRaw16toRaw8(pImageBuf, pImageRaw8Buf, nImageWidth, nImageHeight, DX_BIT_2_9);
                DxRaw8toRGB24(pImageRaw8Buf, pImageRGBBuf, nImageWidth, nImageHeight, RAW2RGB_NEIGHBOUR,
                                DX_PIXEL_COLOR_FILTER(nPixelColorFilter), flip);
            }
            break;

        case GX_PIXEL_FORMAT_BAYER_GR8:
        case GX_PIXEL_FORMAT_BAYER_RG8:
        case GX_PIXEL_FORMAT_BAYER_GB8:
        case GX_PIXEL_FORMAT_BAYER_BG8:
            if (mirror) {
                DxImageMirror(pImageBuf, pMirrorBuffer, nImageWidth, nImageHeight, HORIZONTAL_MIRROR);
                DxRaw8toRGB24(pMirrorBuffer, pImageRGBBuf, nImageWidth, nImageHeight, RAW2RGB_NEIGHBOUR,
                                DX_PIXEL_COLOR_FILTER(nPixelColorFilter), flip); //RAW2RGB_ADAPTIVE
            } else {
                DxRaw8toRGB24(pImageBuf, pImageRGBBuf, nImageWidth, nImageHeight, RAW2RGB_NEIGHBOUR,
                                DX_PIXEL_COLOR_FILTER(nPixelColorFilter), flip); //RAW2RGB_ADAPTIVE
            }
            break;

        case GX_PIXEL_FORMAT_MONO12:
        case GX_PIXEL_FORMAT_MONO10:
            if (mirror) {
                DxRaw16toRaw8(pMirrorBuffer, pImageRaw8Buf, nImageWidth, nImageHeight, DX_BIT_4_11);//DxIma16toRaw8
                DxRaw8toRGB24(pImageRaw8Buf, pImageRGBBuf, nImageWidth, nImageHeight, RAW2RGB_NEIGHBOUR,
                                DX_PIXEL_COLOR_FILTER(NONE), flip);
            } else {
                DxRaw16toRaw8(pImageBuf, pImageRaw8Buf, nImageWidth, nImageHeight, DX_BIT_4_11);
                DxRaw8toRGB24(pImageRaw8Buf, pImageRGBBuf, nImageWidth, nImageHeight, RAW2RGB_NEIGHBOUR,
                                DX_PIXEL_COLOR_FILTER(NONE), flip);
            }
            break;

        case GX_PIXEL_FORMAT_MONO8:
            if (mirror) {
                DxImageMirror(pImageBuf, pMirrorBuffer, nImageWidth, nImageHeight, HORIZONTAL_MIRROR);
                DxRaw8toRGB24(pMirrorBuffer, pImageRGBBuf, nImageWidth, nImageHeight, RAW2RGB_NEIGHBOUR,
                                DX_PIXEL_COLOR_FILTER(NONE), flip);
            } else {
                DxRaw8toRGB24(pImageBuf, pImageRGBBuf, nImageWidth, nImageHeight, RAW2RGB_NEIGHBOUR,
                                DX_PIXEL_COLOR_FILTER(NONE), flip);
            }
            break;

        default:
            break;
    }
}

void DahengCamera::pause() {
    is_paused_ = true; // 设置暂停标志位
    
    if (hDevice != nullptr) {
        // GXSendCommand(hDevice, GX_COMMAND_ACQUISITION_STOP);
    }
}

void DahengCamera::resume() {
    is_paused_ = false; // 清除暂停标志位
    this->queue_.clear();
    if (hDevice != nullptr) {
        // GXSendCommand(hDevice, GX_COMMAND_ACQUISITION_START);
    }
    if(!is_paused_) pause_cv_.notify_all(); // 唤醒正在沉睡的线程
}


}  // namespace io