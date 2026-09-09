#ifndef IO__HIKROBOT_HPP
#define IO__HIKROBOT_HPP

#include <atomic>
#include <chrono>
#include <opencv2/opencv.hpp>
#include <string>
#include <thread>

#include "MvCameraControl.h"
#include "io/camera.hpp"
#include "tools/thread_safe_queue.hpp"

namespace io
{
class HikRobot : public CameraBase
{
public:
  HikRobot(std::string sn, double exposure_us, double gain, const std::string & vid_pid, bool flip, bool mirror);
  ~HikRobot() override;
  void read(cv::Mat & img, std::chrono::steady_clock::time_point & timestamp) override;
  bool try_read(cv::Mat & img, std::chrono::steady_clock::time_point & timestamp) override;
  void clear_camera_frame_buffer() override
  {
    queue_.clear();
    if (handle_ != nullptr) MV_CC_ClearImageBuffer(handle_);
  }

private:
  struct CameraData
  {
    cv::Mat img;
    std::chrono::steady_clock::time_point timestamp;
  };
  
  size_t nDeviceNum = 0;//当前设备数量
  double exposure_us_;
  double gain_;
  bool flip_ = false;// 垂直翻转
  bool mirror_ = false;// 水平镜像

  std::thread daemon_thread_;
  std::atomic<bool> daemon_quit_;

  void * handle_ = nullptr;
  std::thread capture_thread_;
  
  std::atomic<bool> capture_quit_;
  tools::ThreadSafeQueue<CameraData, true> queue_;

  int vid_, pid_;

  bool ChoiceCamrea(MV_CC_DEVICE_INFO** pDeviceInfo, 
                  unsigned char* sn, 
                  size_t& cameraIndex);
  void capture_start();
  void capture_stop();

  void set_float_value(const std::string & name, double value);
  void set_enum_value(const std::string & name, unsigned int value);

  void set_vid_pid(const std::string & vid_pid);
  void reset_usb() const;

  void pause() override;
  void resume() override;

  std::mutex pause_mutex_;
  std::condition_variable pause_cv_;
  
};

}  // namespace io

#endif  // IO__HIKROBOT_HPP
