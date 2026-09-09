#ifndef AUTO_AIM__YOLO_HPP
#define AUTO_AIM__YOLO_HPP

#include <chrono>

#include <opencv2/opencv.hpp>
#include <thread>

#include "armor.hpp"

namespace auto_aim
{
struct YOLOFrameData
{
  int detect_color = 0;     
  std::list<Armor> armors;
  Eigen::Quaterniond gimbal_q;
  std::chrono::steady_clock::time_point timestamp;
  cv::Mat frame;
  bool is_empty ;

  YOLOFrameData():is_empty(true){};

  YOLOFrameData(
    cv::Mat frame_, Eigen::Quaterniond gimbal_q_ = Eigen::Quaterniond(1,1,1,1),
    std::chrono::steady_clock::time_point timestamp_ = {})
  : gimbal_q(gimbal_q_), timestamp(timestamp_), frame(frame_), is_empty(false){};
};

class YOLOBase
{
public:
  virtual std::list<Armor> detect(const cv::Mat & img, int frame_count) = 0;
  virtual YOLOFrameData detect(YOLOFrameData frame_data, int frame_count = -1) = 0;
  virtual std::list<Armor> postprocess(
    double scale, cv::Mat & output, const cv::Mat & bgr_img, int frame_count) = 0;
};

class YOLO
{
public:
  YOLO(const std::string & config_path, bool debug = true);

  std::list<Armor> detect(const cv::Mat & img, int frame_count = -1);

  YOLOFrameData detect(YOLOFrameData frame_data, int frame_count = -1);

  std::list<Armor> postprocess(
    double scale, cv::Mat & output, const cv::Mat & bgr_img, int frame_count);

private:
  std::unique_ptr<YOLOBase> yolo_;
};

}  // namespace auto_aim

#endif  // AUTO_AIM__YOLO_HPP
