#include "yolo11_buff.hpp"

#include <algorithm>
#include <array>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

#include "tools/logger.hpp"

#if defined(SP_AUTO_BUFF_OPENVINO)
#include <openvino/core/preprocess/pre_post_process.hpp>
#include <openvino/openvino.hpp>
#elif defined(SP_AUTO_BUFF_TENSORRT)
#include <NvInfer.h>
#include <NvInferPlugin.h>
#include <cuda_runtime_api.h>
#include "trt_yolo11_buff_kernel.h"
#else
#error "YOLO11_BUFF requires SP_AUTO_BUFF_OPENVINO or SP_AUTO_BUFF_TENSORRT"
#endif

namespace auto_buff
{
namespace
{
cv::Rect clip_rect(const cv::Rect & rect, const cv::Size & size)
{
  return rect & cv::Rect(0, 0, size.width, size.height);
}

cv::Point2f rect_center(const cv::Rect_<float> & rect)
{
  return {rect.x + rect.width * 0.5f, rect.y + rect.height * 0.5f};
}

cv::Scalar color_for_label(int label)
{
  static const std::array<cv::Scalar, 3> colors = {
    cv::Scalar(0, 220, 255), cv::Scalar(255, 160, 0), cv::Scalar(80, 255, 80)};
  if (label < 0 || label >= static_cast<int>(colors.size())) return {255, 255, 255};
  return colors[label];
}

float top_left_letterbox(
  const cv::Mat & input, cv::Mat & output, const cv::Size & network_size)
{
  const float scale = std::min(
    network_size.height / static_cast<float>(input.rows),
    network_size.width / static_cast<float>(input.cols));
  const cv::Matx23f transform{scale, 0.0f, 0.0f, 0.0f, scale, 0.0f};
  cv::warpAffine(
    input, output, transform, network_size, cv::INTER_LINEAR, cv::BORDER_CONSTANT,
    cv::Scalar(0, 0, 0));
  return 1.0f / scale;
}

#if defined(SP_AUTO_BUFF_TENSORRT)
class TensorRTLogger : public nvinfer1::ILogger
{
public:
  void log(Severity severity, const char * message) noexcept override
  {
    if (severity <= Severity::kWARNING) std::fprintf(stderr, "[TensorRT] %s\n", message);
  }
};

TensorRTLogger g_trt_logger;

void check_cuda(cudaError_t status, const char * operation)
{
  if (status != cudaSuccess) {
    throw std::runtime_error(
      std::string(operation) + " failed: " + cudaGetErrorString(status));
  }
}

size_t tensor_volume(const nvinfer1::Dims & dims)
{
  size_t volume = 1;
  for (int i = 0; i < dims.nbDims; ++i) {
    if (dims.d[i] <= 0) throw std::runtime_error("Dynamic TensorRT shapes are not supported");
    volume *= static_cast<size_t>(dims.d[i]);
  }
  return volume;
}
#endif
}  // namespace

struct YOLO11_BUFF::Backend
{
  struct Result
  {
    const float * data;
    int rows;
    int cols;
    float inverse_scale;
  };

#if defined(SP_AUTO_BUFF_OPENVINO)
  explicit Backend(const std::string & model_path)
  {
    auto model = core.read_model(model_path);
    ov::preprocess::PrePostProcessor ppp(model);
    ppp.input()
      .tensor()
      .set_element_type(ov::element::u8)
      .set_layout("NHWC")
      .set_color_format(ov::preprocess::ColorFormat::BGR);
    ppp.input()
      .preprocess()
      .convert_element_type(ov::element::f32)
      .convert_color(ov::preprocess::ColorFormat::RGB)
      .scale({255.0, 255.0, 255.0});
    ppp.input().model().set_layout("NCHW");

    model = ppp.build();
    compiled_model =
      core.compile_model(model, "CPU", ov::hint::performance_mode(ov::hint::PerformanceMode::LATENCY));
    infer_request = compiled_model.create_infer_request();
    input_tensor = infer_request.get_input_tensor();
  }

  Result infer(const cv::Mat & image)
  {
    const ov::Shape input_shape = input_tensor.get_shape();
    if (input_shape.size() != 4 || input_shape[3] != 3) {
      throw std::runtime_error("Expected an NHWC OpenVINO input tensor");
    }

    cv::Mat letterboxed;
    const float inverse_scale = top_left_letterbox(
      image, letterboxed,
      cv::Size(static_cast<int>(input_shape[2]), static_cast<int>(input_shape[1])));
    std::memcpy(
      input_tensor.data<uint8_t>(), letterboxed.data,
      letterboxed.total() * letterboxed.elemSize());

    infer_request.infer();
    output_tensor = infer_request.get_output_tensor();
    const ov::Shape output_shape = output_tensor.get_shape();
    if (output_shape.size() != 3 || output_shape[0] != 1) {
      throw std::runtime_error("Expected a [1, channels, candidates] OpenVINO output");
    }
    return {
      output_tensor.data<const float>(), static_cast<int>(output_shape[1]),
      static_cast<int>(output_shape[2]), inverse_scale};
  }

  ov::Core core;
  ov::CompiledModel compiled_model;
  ov::InferRequest infer_request;
  ov::Tensor input_tensor;
  ov::Tensor output_tensor;
#elif defined(SP_AUTO_BUFF_TENSORRT)
  explicit Backend(const std::string & engine_path)
  {
    try {
      std::ifstream file(engine_path, std::ios::binary | std::ios::ate);
      if (!file) throw std::runtime_error("Cannot open TensorRT engine: " + engine_path);
      const std::streamsize size = file.tellg();
      if (size <= 0) throw std::runtime_error("TensorRT engine is empty: " + engine_path);
      file.seekg(0, std::ios::beg);
      std::vector<char> serialized(static_cast<size_t>(size));
      if (!file.read(serialized.data(), size)) {
        throw std::runtime_error("Failed to read TensorRT engine: " + engine_path);
      }

      initLibNvInferPlugins(&g_trt_logger, "");
      runtime = nvinfer1::createInferRuntime(g_trt_logger);
      if (runtime == nullptr) throw std::runtime_error("Failed to create TensorRT runtime");
      engine = runtime->deserializeCudaEngine(serialized.data(), serialized.size());
      if (engine == nullptr) throw std::runtime_error("Failed to deserialize TensorRT engine");
      context = engine->createExecutionContext();
      if (context == nullptr) throw std::runtime_error("Failed to create TensorRT context");

      for (int i = 0; i < engine->getNbIOTensors(); ++i) {
        const char * name = engine->getIOTensorName(i);
        const auto mode = engine->getTensorIOMode(name);
        if (mode == nvinfer1::TensorIOMode::kINPUT) {
          if (!input_name.empty()) throw std::runtime_error("Expected exactly one engine input");
          input_name = name;
        } else {
          if (!output_name.empty()) throw std::runtime_error("Expected exactly one engine output");
          output_name = name;
        }
      }
      if (input_name.empty() || output_name.empty()) {
        throw std::runtime_error("TensorRT engine must have one input and one output");
      }
      if (
        engine->getTensorDataType(input_name.c_str()) != nvinfer1::DataType::kFLOAT ||
        engine->getTensorDataType(output_name.c_str()) != nvinfer1::DataType::kFLOAT) {
        throw std::runtime_error("TensorRT engine input and output must use FP32 I/O tensors");
      }

      const nvinfer1::Dims input_dims = engine->getTensorShape(input_name.c_str());
      const nvinfer1::Dims output_dims = engine->getTensorShape(output_name.c_str());
      if (
        input_dims.nbDims != 4 || input_dims.d[0] != 1 || input_dims.d[1] != 3 ||
        output_dims.nbDims != 3 || output_dims.d[0] != 1) {
        throw std::runtime_error(
          "Expected engine tensors [1,3,H,W] and [1,channels,candidates]");
      }
      input_height = input_dims.d[2];
      input_width = input_dims.d[3];
      output_rows = output_dims.d[1];
      output_cols = output_dims.d[2];
      input_bytes = tensor_volume(input_dims) * sizeof(float);
      output_bytes = tensor_volume(output_dims) * sizeof(float);

      check_cuda(
        cudaStreamCreateWithFlags(&stream, cudaStreamNonBlocking),
        "cudaStreamCreateWithFlags");
      check_cuda(cudaMalloc(&input_device, input_bytes), "cudaMalloc(input)");
      check_cuda(cudaMalloc(&output_device, output_bytes), "cudaMalloc(output)");
      check_cuda(
        cudaHostAlloc(
          reinterpret_cast<void **>(&output_host), output_bytes, cudaHostAllocDefault),
        "cudaHostAlloc(output)");
      if (!context->setTensorAddress(input_name.c_str(), input_device)) {
        throw std::runtime_error("Failed to bind TensorRT input tensor");
      }
      if (!context->setTensorAddress(output_name.c_str(), output_device)) {
        throw std::runtime_error("Failed to bind TensorRT output tensor");
      }

      // Trigger TensorRT/CUDA lazy loading during initialization instead of
      // stalling the first frame in the control loop.
      check_cuda(cudaMemsetAsync(input_device, 0, input_bytes, stream), "cudaMemsetAsync(warmup)");
      if (!context->enqueueV3(stream)) throw std::runtime_error("TensorRT warmup enqueueV3 failed");
      check_cuda(cudaStreamSynchronize(stream), "cudaStreamSynchronize(warmup)");
    } catch (...) {
      release();
      throw;
    }
  }

  ~Backend() { release(); }

  Result infer(const cv::Mat & image)
  {
    if (image.type() != CV_8UC3) {
      throw std::runtime_error("TensorRT buff backend expects a CV_8UC3 BGR image");
    }
    const float scale = std::min(
      input_height / static_cast<float>(image.rows),
      input_width / static_cast<float>(image.cols));
    const float inverse_scale = 1.0f / scale;
    const std::size_t source_bytes =
      static_cast<std::size_t>(image.rows) * image.cols * image.elemSize();
    if (source_bytes > image_capacity) {
      if (image_device != nullptr) {
        check_cuda(cudaFree(image_device), "cudaFree(image)");
        image_device = nullptr;
        image_capacity = 0;
      }
      check_cuda(
        cudaMalloc(reinterpret_cast<void **>(&image_device), source_bytes), "cudaMalloc(image)");
      image_capacity = source_bytes;
    }

    check_cuda(
      cudaMemcpy2DAsync(
        image_device, image.cols * image.elemSize(), image.data, image.step,
        image.cols * image.elemSize(), image.rows, cudaMemcpyHostToDevice, stream),
      "cudaMemcpy2DAsync(image)");
    launch_yolo11_buff_preprocess(
      image_device, image.cols, image.rows, image.cols * image.elemSize(),
      static_cast<float *>(input_device), input_width, input_height, scale, stream);
    check_cuda(cudaGetLastError(), "launch_yolo11_buff_preprocess");
    if (!context->enqueueV3(stream)) throw std::runtime_error("TensorRT enqueueV3 failed");
    check_cuda(
      cudaMemcpyAsync(
        output_host, output_device, output_bytes, cudaMemcpyDeviceToHost, stream),
      "cudaMemcpyAsync(output)");
    check_cuda(cudaStreamSynchronize(stream), "cudaStreamSynchronize");
    return {output_host, output_rows, output_cols, inverse_scale};
  }

  void release() noexcept
  {
    if (output_device != nullptr) cudaFree(output_device);
    if (input_device != nullptr) cudaFree(input_device);
    if (image_device != nullptr) cudaFree(image_device);
    if (output_host != nullptr) cudaFreeHost(output_host);
    if (stream != nullptr) cudaStreamDestroy(stream);
    delete context;
    delete engine;
    delete runtime;
    output_device = nullptr;
    input_device = nullptr;
    image_device = nullptr;
    output_host = nullptr;
    stream = nullptr;
    context = nullptr;
    engine = nullptr;
    runtime = nullptr;
  }

  nvinfer1::IRuntime * runtime = nullptr;
  nvinfer1::ICudaEngine * engine = nullptr;
  nvinfer1::IExecutionContext * context = nullptr;
  cudaStream_t stream = nullptr;
  void * input_device = nullptr;
  void * output_device = nullptr;
  unsigned char * image_device = nullptr;
  std::size_t image_capacity = 0;
  std::string input_name;
  std::string output_name;
  int input_height = 0;
  int input_width = 0;
  int output_rows = 0;
  int output_cols = 0;
  size_t input_bytes = 0;
  size_t output_bytes = 0;
  float * output_host = nullptr;
#endif
};

YOLO11_BUFF::YOLO11_BUFF(const std::string & config)
{
  const auto yaml = YAML::LoadFile(config);
  if (yaml["buff_confidence_threshold"]) {
    confidence_threshold_ = yaml["buff_confidence_threshold"].as<float>();
  }
  if (yaml["buff_keypoint_threshold"]) {
    keypoint_threshold_ = yaml["buff_keypoint_threshold"].as<float>();
  }
  if (yaml["buff_iou_threshold"]) iou_threshold_ = yaml["buff_iou_threshold"].as<float>();

#if defined(SP_AUTO_BUFF_OPENVINO)
  const std::string model_path = yaml["model"].as<std::string>();
  backend_ = std::make_unique<Backend>(model_path);
#elif defined(SP_AUTO_BUFF_TENSORRT)
  const YAML::Node path_node = yaml["buff_engine_path"];
  if (!path_node) {
    throw std::runtime_error("TensorRT buff backend requires 'buff_engine_path' in the config");
  }
  backend_ = std::make_unique<Backend>(path_node.as<std::string>());
#endif
}

YOLO11_BUFF::~YOLO11_BUFF() = default;

std::vector<YOLO11_BUFF::Object> YOLO11_BUFF::get_multicandidateboxes(cv::Mat & image)
{
  return infer_and_decode(image);
}

std::vector<YOLO11_BUFF::Object> YOLO11_BUFF::get_onecandidatebox(cv::Mat & image)
{
  auto objects = infer_and_decode(image);
  if (objects.empty()) return {};
  auto best = std::max_element(objects.begin(), objects.end(), [](const Object & a, const Object & b) {
    return a.prob < b.prob;
  });
  return {*best};
}

std::vector<YOLO11_BUFF::Object> YOLO11_BUFF::infer_and_decode(cv::Mat & image)
{
  const int64 start = cv::getTickCount();
  if (image.empty()) {
    tools::logger()->warn("Empty img!, camera drop!");
    return {};
  }

  const Backend::Result result = backend_->infer(image);
  auto objects = decode(
    result.data, result.rows, result.cols, result.inverse_scale, image.size());
  const double elapsed_s =
    (cv::getTickCount() - start) / static_cast<double>(cv::getTickFrequency());
  draw_objects(image, objects, elapsed_s);
  return objects;
}

std::vector<YOLO11_BUFF::Object> YOLO11_BUFF::decode(
  const float * output, int output_rows, int output_cols, float inverse_scale,
  const cv::Size & image_size) const
{
  const int expected_rows = 4 + NUM_CLASSES + NUM_POINTS * KPT_DIMS;
  if (output_rows != expected_rows) {
    throw std::runtime_error(
      "Unexpected YOLO11 buff output channels: " + std::to_string(output_rows) +
      ", expected " + std::to_string(expected_rows));
  }

  const cv::Mat detections(output_rows, output_cols, CV_32F, const_cast<float *>(output));
  std::vector<cv::Rect> boxes;
  std::vector<float> confidences;
  std::vector<int> labels;
  std::vector<std::vector<cv::Point2f>> keypoints;
  std::vector<std::vector<float>> keypoint_confidences;

  for (int candidate = 0; candidate < detections.cols; ++candidate) {
    int label = -1;
    float score = -std::numeric_limits<float>::max();
    for (int cls = 0; cls < NUM_CLASSES; ++cls) {
      const float class_score = detections.at<float>(4 + cls, candidate);
      if (class_score > score) {
        score = class_score;
        label = cls;
      }
    }
    if (score < confidence_threshold_) continue;

    const float center_x = detections.at<float>(0, candidate);
    const float center_y = detections.at<float>(1, candidate);
    const float width = detections.at<float>(2, candidate);
    const float height = detections.at<float>(3, candidate);
    cv::Rect box(
      static_cast<int>((center_x - width * 0.5f) * inverse_scale),
      static_cast<int>((center_y - height * 0.5f) * inverse_scale),
      static_cast<int>(width * inverse_scale), static_cast<int>(height * inverse_scale));
    box = clip_rect(box, image_size);
    if (box.empty()) continue;

    std::vector<cv::Point2f> candidate_keypoints;
    std::vector<float> candidate_keypoint_confidences;
    const int keypoint_offset = 4 + NUM_CLASSES;
    for (int point = 0; point < NUM_POINTS; ++point) {
      candidate_keypoints.emplace_back(
        detections.at<float>(keypoint_offset + point * KPT_DIMS, candidate) * inverse_scale,
        detections.at<float>(keypoint_offset + point * KPT_DIMS + 1, candidate) * inverse_scale);
      candidate_keypoint_confidences.push_back(
        detections.at<float>(keypoint_offset + point * KPT_DIMS + 2, candidate));
    }

    boxes.push_back(box);
    confidences.push_back(score);
    labels.push_back(label);
    keypoints.push_back(std::move(candidate_keypoints));
    keypoint_confidences.push_back(std::move(candidate_keypoint_confidences));
  }

  std::vector<int> kept_indexes;
  for (int cls = 0; cls < NUM_CLASSES; ++cls) {
    std::vector<cv::Rect> class_boxes;
    std::vector<float> class_confidences;
    std::vector<int> class_indexes;
    for (size_t i = 0; i < labels.size(); ++i) {
      if (labels[i] != cls) continue;
      class_boxes.push_back(boxes[i]);
      class_confidences.push_back(confidences[i]);
      class_indexes.push_back(static_cast<int>(i));
    }

    std::vector<int> nms_indexes;
    cv::dnn::NMSBoxes(
      class_boxes, class_confidences, confidence_threshold_, iou_threshold_, nms_indexes);
    for (int index : nms_indexes) kept_indexes.push_back(class_indexes[index]);
  }
  std::sort(kept_indexes.begin(), kept_indexes.end(), [&](int lhs, int rhs) {
    return confidences[lhs] > confidences[rhs];
  });

  std::vector<Object> objects;
  objects.reserve(kept_indexes.size());
  for (int index : kept_indexes) {
    objects.push_back(
      {boxes[index], labels[index], confidences[index], std::move(keypoints[index]),
       std::move(keypoint_confidences[index])});
  }
  return objects;
}

void YOLO11_BUFF::draw_objects(
  cv::Mat & image, const std::vector<Object> & objects, double elapsed_s) const
{
  for (const Object & object : objects) {
    const cv::Scalar color = color_for_label(object.label);
    cv::rectangle(image, object.rect, color, 1, cv::LINE_8);
    const std::string label =
      class_names[object.label] + ":" + cv::format("%.2f", object.prob);
    int baseline = 0;
    const cv::Size text_size =
      cv::getTextSize(label, cv::FONT_HERSHEY_SIMPLEX, 0.45, 1, &baseline);
    const cv::Point text_origin(
      static_cast<int>(object.rect.x), std::max(12, static_cast<int>(object.rect.y) - 4));
    cv::rectangle(
      image,
      cv::Rect(
        text_origin.x, text_origin.y - text_size.height, text_size.width,
        text_size.height + baseline),
      color, cv::FILLED);
    cv::putText(
      image, label, text_origin, cv::FONT_HERSHEY_SIMPLEX, 0.45, cv::Scalar(0, 0, 0), 1,
      cv::LINE_AA);

    for (int point = 0; point < NUM_POINTS; ++point) {
      const cv::Scalar point_color = object.kpt_conf[point] >= keypoint_threshold_
                                       ? color
                                       : cv::Scalar(80, 80, 80);
      cv::circle(image, object.kpt[point], 2, point_color, -1, cv::LINE_AA);
      cv::putText(
        image, std::to_string(point), object.kpt[point] + cv::Point2f(4, -4),
        cv::FONT_HERSHEY_SIMPLEX, 0.35, point_color, 1, cv::LINE_AA);
    }
    cv::circle(image, rect_center(object.rect), 2, color, -1, cv::LINE_AA);
  }

  const double fps = elapsed_s > 0.0 ? 1.0 / elapsed_s : 0.0;
  cv::putText(
    image, cv::format("FPS: %.2f", fps), cv::Point(20, 40), cv::FONT_HERSHEY_PLAIN, 2.0,
    cv::Scalar(255, 0, 0), 2, cv::LINE_8);
}
}  // namespace auto_buff
