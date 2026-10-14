#ifndef IO__CAMERA_HPP
#define IO__CAMERA_HPP


#include <atomic> 
#include <chrono>
#include <memory>
#include <opencv2/opencv.hpp>
#include <string>

namespace io
{
class CameraBase
{
public:
  int64_t sensorWidth = -1, sensorHeight = -1; //相机分辨率
  std::atomic<bool> is_paused_{false};
  std::atomic<bool> capturing_{false};// 相机正常运行
  std::chrono::steady_clock::time_point last_read_t;
  std::string camera_sn_;

  /** @brief 销毁相机基类 */
  virtual ~CameraBase() = default;
  /** @brief 构造相机基类 @param sn 相机序列号 */
  CameraBase(const std::string& sn) : camera_sn_(sn) {};
  /** @brief 阻塞读取一帧图像 @param img 输出图像 @param timestamp 输出采集时间戳 */
  virtual void read(cv::Mat & img, std::chrono::steady_clock::time_point & timestamp) = 0;
  /** @brief 尝试读取一帧图像 @param img 输出图像 @param timestamp 输出采集时间戳 @return 成功读取时返回 true */
  virtual bool try_read(cv::Mat & img, std::chrono::steady_clock::time_point & timestamp) = 0;

  /** @brief 暂停图像采集 */
  virtual void pause() {} //停止
  /** @brief 恢复图像采集 */
  virtual void resume() {} //开启

  /** @brief 清空相机内部帧缓冲区 */
  virtual void clear_camera_frame_buffer() = 0;
};

class Camera
{
public:
  std::string main_and_secondary = "main"; //是否是主相机
  std::chrono::microseconds timestamp_offset = std::chrono::microseconds(0); //时间戳偏移量
  cv::Mat img_gamma_lut;
  double  img_gamma = 1.0;
  double img_gamma_shadow_offset = 0.04;
  double img_gamma_luma_denoise_sigma = 0.7;
  double img_gamma_chroma_denoise_sigma = 1.0;


  /** @brief 根据配置文件创建具体相机实例 @param config_path YAML 配置文件路径 */
  Camera(const std::string & config_path);

  /** @brief 初始化已配置相机厂商的 SDK */
  static void initSDK();
  /** @brief 阻塞读取一帧图像 @param img 输出图像 @param timestamp 输出驱动采集时刻的时间戳（不含 timestamp_offset，由上层按需补偿） */
  void read(cv::Mat & img, std::chrono::steady_clock::time_point & timestamp);
  /** @brief 尝试读取一帧图像 @param img 输出图像 @param timestamp 输出驱动采集时刻的时间戳（不含 timestamp_offset，由上层按需补偿） @return 成功读取时返回 true */
  bool try_read(cv::Mat & img, std::chrono::steady_clock::time_point & timestamp);

  /** @brief 暂停底层相机采集 */
  void pause() { if(camera_) camera_->pause(); }
  /** @brief 恢复底层相机采集 */
  void resume() { if(camera_) camera_->resume(); }
  /** @brief 查询相机是否暂停 @return 暂停时返回 true */
  bool is_paused()  {   
    if(camera_) return camera_->is_paused_;
    else return false;
  }
  /** @brief 获取最近一次成功读取的时间 @return 最近读取时间戳 */
  std::chrono::steady_clock::time_point get_last_read_t( ){
    return camera_->last_read_t;
  }
  /** @brief 查询底层相机是否正在采集 @return 正在采集时返回 true */
  bool get_capturing(){
    return this->camera_->capturing_.load();
  }
  /** @brief 获取相机序列号 @return 序列号字符串 */
  std::string get_camera_sn(){
    return camera_->camera_sn_;
  }
  /** @brief 清空底层相机帧缓冲区 */
  void clear_camera_frame_buffer() {camera_->clear_camera_frame_buffer();};
  

private:
  /** @brief 对相机图像执行亮度降噪和暗部保护 Gamma */
  void process_image(cv::Mat & img) const;

  std::unique_ptr<CameraBase> camera_;
};

}  // namespace io

#endif  // IO__CAMERA_HPP
