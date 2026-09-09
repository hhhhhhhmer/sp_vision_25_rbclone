#include "yolo.hpp"

#include <yaml-cpp/yaml.h>

#ifdef TENSOR_RT_MAKE
#include "trt_yolos/trt_yolo_0708.hpp"
#include "trt_yolos/trt_yolo_0526.hpp"
#endif

#ifdef OPENVINO_MAKE
#include "ov_yolos/yolo11.hpp"
#include "ov_yolos/yolov5.hpp"
#include "ov_yolos/yolov8.hpp"
#endif


namespace auto_aim
{
YOLO::YOLO(const std::string & config_path, bool debug)
{
  auto yaml = YAML::LoadFile(config_path);
  auto yolo_name = yaml["yolo_name"].as<std::string>();

  #ifdef OPENVINO_MAKE
  if (yolo_name == "yolov8") {
    yolo_ = std::make_unique<YOLOV8>(config_path, debug);
  }

  else if (yolo_name == "yolo11") {
    yolo_ = std::make_unique<YOLO11>(config_path, debug);
  }

  else if (yolo_name == "yolov5") {
    yolo_ = std::make_unique<YOLOV5>(config_path, debug);
  }
  else if (yolo_name == "ov_0526") {
    yolo_ = std::make_unique<YOLOV5>(config_path, debug);
  }


  else {
    throw std::runtime_error("Unknown yolo name: " + yolo_name + "!");
  }

  #endif

  #ifdef TENSOR_RT_MAKE
  if (yolo_name == "trt_0708") {
      yolo_ = std::make_unique<TensorRTYolo0708>(config_path, debug);
  }
  else if (yolo_name == "trt_0526") {          // 新增分支
      yolo_ = std::make_unique<TensorRTYolo0526>(config_path, debug);
  }

  else {
    throw std::runtime_error("Unknown yolo name: " + yolo_name + "!");
  }
  #endif
}

std::list<Armor> YOLO::detect(const cv::Mat & img, int frame_count)
{
  return yolo_->detect(img, frame_count);
}

YOLOFrameData YOLO::detect(YOLOFrameData frame_data, int frame_count){
  return yolo_->detect(frame_data, frame_count);
}

std::list<Armor> YOLO::postprocess(
  double scale, cv::Mat & output, const cv::Mat & bgr_img, int frame_count)
{
  return yolo_->postprocess(scale, output, bgr_img, frame_count);
}

}  // namespace auto_aim
