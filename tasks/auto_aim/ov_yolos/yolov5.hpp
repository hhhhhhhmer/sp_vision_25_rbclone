#ifndef AUTO_AIM__YOLOV5_HPP
#define AUTO_AIM__YOLOV5_HPP

#include <array>
#include <list>
#include <opencv2/opencv.hpp>
#include <openvino/openvino.hpp>
#include <string>
#include <vector>

#include "async_pipeline.hpp"
#include "tasks/auto_aim/armor.hpp"
#include "tasks/auto_aim/detector.hpp"
#include "tasks/auto_aim/yolo.hpp"

namespace auto_aim
{
class YOLOV5 : public YOLOBase
{
public:
  YOLOV5(const std::string & config_path, bool debug);

  std::list<Armor> detect(const cv::Mat & bgr_img, int frame_count) override;

  YOLOFrameData detect(YOLOFrameData frame_data, int frame_count) override;
  std::list<Armor> postprocess(
    double scale, cv::Mat & output, const cv::Mat & bgr_img, int frame_count) override;

private:
  static constexpr int kInputWidth = 640;
  static constexpr int kInputHeight = 640;
  static constexpr int kOutputValues = 22;
  static constexpr int kColorCount = 4;
  static constexpr int kClassCount = 9;

  std::string device_, model_path_;
  std::string save_path_, debug_path_;
  bool debug_, use_roi_, use_traditional_;

  float nms_threshold_;
  float score_threshold_;
  double min_confidence_;

  ov::Core core_;
  ov::CompiledModel compiled_model_;
  ov::InferRequest sync_infer_request_;
  OpenVINOAsyncPipeline async_pipeline_;

  cv::Rect roi_;
  cv::Point2f offset_;
  cv::Mat tmp_img_;

  Detector detector_;
  friend class MultiThreadDetector;

  bool check_name(const Armor & armor) const;
  bool check_type(const Armor & armor) const;

  cv::Mat inference_image(const cv::Mat & raw_img);
  cv::Point2f get_center_norm(const cv::Mat & bgr_img, const cv::Point2f & center) const;

  std::list<Armor> parse(double scale, cv::Mat & output, const cv::Mat & bgr_img, int frame_count);
  Armor make_armor(
    int color_id, int class_id, float confidence, const cv::Rect & box,
    const std::vector<cv::Point2f> & keypoints) const;

  static int argmax(const float * values, int count);
  static float sigmoid(float value) noexcept;

  void save(const Armor & armor) const;
  void draw_detections(const cv::Mat & img, const std::list<Armor> & armors, int frame_count) const;
};

}  // namespace auto_aim

#endif  // AUTO_AIM__YOLOV5_HPP
