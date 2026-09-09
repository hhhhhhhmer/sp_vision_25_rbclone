#ifndef AUTO_BUFF__YOLO11_BUFF_HPP
#define AUTO_BUFF__YOLO11_BUFF_HPP
#include <yaml-cpp/yaml.h>

#include <memory>
#include <opencv2/opencv.hpp>

namespace auto_buff
{
inline const std::vector<std::string> class_names = {
  "inactive_target", "inactive_fan", "rune_center"};

enum RuneClass : int
{
  INACTIVE_TARGET = 0,
  INACTIVE_FAN = 1,
  RUNE_CENTER = 2
};

class YOLO11_BUFF
{
public:
  struct Object
  {
    cv::Rect_<float> rect;
    int label;
    float prob;
    std::vector<cv::Point2f> kpt;
    std::vector<float> kpt_conf;
  };

  YOLO11_BUFF(const std::string & config);
  ~YOLO11_BUFF();

  YOLO11_BUFF(const YOLO11_BUFF &) = delete;
  YOLO11_BUFF & operator=(const YOLO11_BUFF &) = delete;
  YOLO11_BUFF(YOLO11_BUFF &&) = delete;
  YOLO11_BUFF & operator=(YOLO11_BUFF &&) = delete;

  // 使用NMS，用来获取多个框
  std::vector<Object> get_multicandidateboxes(cv::Mat & image);

  // 寻找置信度最高的框
  std::vector<Object> get_onecandidatebox(cv::Mat & image);

private:
  struct Backend;
  std::unique_ptr<Backend> backend_;

  static constexpr int NUM_CLASSES = 3;
  static constexpr int NUM_POINTS = 4;
  static constexpr int KPT_DIMS = 3;

  float confidence_threshold_ = 0.7f;
  float keypoint_threshold_ = 0.3f;
  float iou_threshold_ = 0.4f;

  std::vector<Object> infer_and_decode(cv::Mat & image);
  std::vector<Object> decode(
    const float * output, int output_rows, int output_cols, float inverse_scale,
    const cv::Size & image_size) const;
  void draw_objects(cv::Mat & image, const std::vector<Object> & objects, double elapsed_s) const;
};
}  // namespace auto_buff
#endif
