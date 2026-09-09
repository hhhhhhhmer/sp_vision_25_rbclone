#ifndef AUTO_AIM__OV_YOLOS__ASYNC_PIPELINE_HPP
#define AUTO_AIM__OV_YOLOS__ASYNC_PIPELINE_HPP

#include <array>
#include <stdexcept>

#include <opencv2/opencv.hpp>
#include <openvino/openvino.hpp>

#include "tasks/auto_aim/yolo.hpp"

namespace auto_aim
{

class OpenVINOAsyncPipeline
{
public:
  void init(ov::CompiledModel & compiled_model, int input_width, int input_height)
  {
    input_width_ = input_width;
    input_height_ = input_height;

    for (auto & slot : slots_) {
      slot.infer_request = compiled_model.create_infer_request();
    }
  }

  template <typename Parser>
  YOLOFrameData detect(
    const cv::Mat & bgr_img, const YOLOFrameData & frame_data, int frame_count, Parser parser)
  {
    submit(bgr_img, frame_data, frame_count);

    if (pending_count_ < static_cast<int>(slots_.size())) {
      return YOLOFrameData();
    }

    auto & finished = slots_[next_result_];
    finished.infer_request.wait();

    auto output_tensor = finished.infer_request.get_output_tensor();
    auto output_shape = output_tensor.get_shape();
    if (output_shape.size() < 3) {
      throw std::runtime_error("Unexpected OpenVINO YOLO output shape");
    }

    cv::Mat output(
      static_cast<int>(output_shape[1]), static_cast<int>(output_shape[2]), CV_32F,
      output_tensor.data());

    YOLOFrameData result = finished.frame_data;
    result.armors = parser(finished.scale, output, finished.frame_data.frame, finished.frame_count);
    result.is_empty = false;

    finished.pending = false;
    pending_count_--;
    next_result_ = (next_result_ + 1) % slots_.size();

    return result;
  }

private:
  struct Slot
  {
    ov::InferRequest infer_request;
    cv::Mat input;
    YOLOFrameData frame_data;
    int frame_count = -1;
    double scale = 1.0;
    bool pending = false;
  };

  void submit(const cv::Mat & bgr_img, const YOLOFrameData & frame_data, int frame_count)
  {
    auto & slot = slots_[next_submit_];
    if (slot.pending) {
      throw std::runtime_error("OpenVINO async slot is still pending");
    }

    auto x_scale = static_cast<double>(input_height_) / bgr_img.rows;
    auto y_scale = static_cast<double>(input_width_) / bgr_img.cols;
    slot.scale = std::min(x_scale, y_scale);
    auto h = static_cast<int>(bgr_img.rows * slot.scale);
    auto w = static_cast<int>(bgr_img.cols * slot.scale);

    slot.input = cv::Mat(input_height_, input_width_, CV_8UC3, cv::Scalar(0, 0, 0));
    auto input_roi = cv::Rect(0, 0, w, h);
    cv::resize(bgr_img, slot.input(input_roi), {w, h});

    slot.frame_data = frame_data;
    slot.frame_data.frame = frame_data.frame.clone();
    slot.frame_data.is_empty = false;
    slot.frame_count = frame_count;

    ov::Tensor input_tensor(
      ov::element::u8,
      {1, static_cast<size_t>(input_height_), static_cast<size_t>(input_width_), 3},
      slot.input.data);
    slot.infer_request.set_input_tensor(input_tensor);
    slot.infer_request.start_async();

    slot.pending = true;
    pending_count_++;
    next_submit_ = (next_submit_ + 1) % slots_.size();
  }

  std::array<Slot, 2> slots_;
  int input_width_ = 0;
  int input_height_ = 0;
  size_t next_submit_ = 0;
  size_t next_result_ = 0;
  int pending_count_ = 0;
};

}  // namespace auto_aim

#endif  // AUTO_AIM__OV_YOLOS__ASYNC_PIPELINE_HPP
