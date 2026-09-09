#include "yolov5.hpp"

#include <fmt/chrono.h>
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cctype>
#include <cmath>
#include <filesystem>
#include <limits>
#include <stdexcept>
#include <utility>

#include "tools/img_tools.hpp"
#include "tools/logger.hpp"

namespace auto_aim
{
namespace
{
constexpr std::array<Color, 4> MODEL_COLORS = {
  Color::red, Color::blue, Color::extinguish, Color::purple};
constexpr std::array<ArmorName, 9> MODEL_NAMES = {
  ArmorName::sentry, ArmorName::one,  ArmorName::two,  ArmorName::three, ArmorName::four,
  ArmorName::five,   ArmorName::outpost, ArmorName::base, ArmorName::base};
constexpr std::array<ArmorType, 9> MODEL_TYPES = {
  ArmorType::small, ArmorType::big,   ArmorType::small,
  ArmorType::small, ArmorType::small, ArmorType::small,
  ArmorType::small, ArmorType::small, ArmorType::big};
constexpr float BIG_ARMOR_RATIO_THRESHOLD = 3.0F;

std::string uppercase(std::string value)
{
  std::transform(value.begin(), value.end(), value.begin(), [](unsigned char c) {
    return static_cast<char>(std::toupper(c));
  });
  return value;
}

bool has_device(const std::vector<std::string> & available_devices, const std::string & device)
{
  return std::any_of(
    available_devices.begin(), available_devices.end(), [&device](const std::string & available) {
      return available == device || available.rfind(device + ".", 0) == 0;
    });
}

std::string available_devices_text(const std::vector<std::string> & available_devices)
{
  std::string result;
  for (const auto & device : available_devices) {
    if (!result.empty()) result += ", ";
    result += device;
  }
  return result.empty() ? "none" : result;
}

std::string select_device(ov::Core & core, const std::string & configured_device)
{
  const auto requested = uppercase(configured_device);
  const auto available = core.get_available_devices();
  if (requested == "AUTO") {
    if (has_device(available, "GPU")) return "GPU";
    if (has_device(available, "CPU")) return "CPU";
  } else if (requested == "CPU" || requested == "GPU") {
    if (has_device(available, requested)) return requested;
  } else if (requested.rfind("GPU.", 0) == 0) {
    if (std::find(available.begin(), available.end(), requested) != available.end()) return requested;
  } else {
    throw std::runtime_error(
      "Unsupported YOLOV5 device '" + configured_device + "'; use CPU, GPU, GPU.n or AUTO");
  }

  throw std::runtime_error(
    "Requested YOLOV5 device '" + configured_device + "' is unavailable; available devices: " +
    available_devices_text(available));
}
}  // namespace

YOLOV5::YOLOV5(const std::string & config_path, bool debug)
: debug_(debug), detector_(config_path, false)
{
  auto yaml = YAML::LoadFile(config_path);

  const auto yolo_name = yaml["yolo_name"].as<std::string>();
  const bool is_0526 = yolo_name == "ov_0526";
  const auto model_path_key = is_0526 ? "ov_0526_model_path" : "yolov5_model_path";
  model_path_ = yaml[model_path_key].as<std::string>();
  const auto configured_device = yaml["device"].as<std::string>();
  device_ = select_device(core_, configured_device);
  min_confidence_ = yaml["min_confidence"].as<double>();
  score_threshold_ = yaml["yolo_score_threshold"].as<float>(is_0526 ? 0.65F : 0.7F);
  nms_threshold_ = yaml["yolo_nms_threshold"].as<float>(is_0526 ? 0.45F : 0.3F);
  int x = 0, y = 0, width = 0, height = 0;
  x = yaml["roi"]["x"].as<int>();
  y = yaml["roi"]["y"].as<int>();
  width = yaml["roi"]["width"].as<int>();
  height = yaml["roi"]["height"].as<int>();
  use_roi_ = yaml["use_roi"].as<bool>();
  use_traditional_ = yaml["use_traditional"].as<bool>();
  roi_ = cv::Rect(x, y, width, height);
  offset_ = cv::Point2f(x, y);

  save_path_ = "imgs";
  std::filesystem::create_directory(save_path_);
  auto model = core_.read_model(model_path_);
  const auto input_shape = model->input().get_shape();
  const auto output_shape = model->output().get_shape();
  if (
    input_shape != ov::Shape{1, 3, kInputHeight, kInputWidth} || output_shape.size() != 3 ||
    output_shape[0] != 1 || output_shape[2] != kOutputValues) {
    throw std::runtime_error(
      "YOLOV5 expects input [1,3,640,640] and output [1,candidates,22]");
  }

  const auto model_input_type = model->input().get_element_type();
  ov::preprocess::PrePostProcessor ppp(model);
  auto & input = ppp.input();

  input.tensor()
    .set_element_type(ov::element::u8)
    .set_shape({1, kInputHeight, kInputWidth, 3})
    .set_layout("NHWC")
    .set_color_format(ov::preprocess::ColorFormat::BGR);

  input.model().set_layout("NCHW");

  input.preprocess()
    .convert_element_type(model_input_type)
    .convert_color(ov::preprocess::ColorFormat::RGB)
    .scale(255.0);
  ppp.output().tensor().set_element_type(ov::element::f32);

  model = ppp.build();
  ov::AnyMap compile_properties = {
    {ov::hint::performance_mode.name(), ov::hint::PerformanceMode::LATENCY}};
  if (device_.rfind("GPU", 0) == 0) {
    compile_properties[ov::hint::inference_precision.name()] = ov::element::f16;
  }
  compiled_model_ = core_.compile_model(model, device_, compile_properties);
  sync_infer_request_ = compiled_model_.create_infer_request();
  async_pipeline_.init(compiled_model_, kInputWidth, kInputHeight);

  const auto execution_devices = compiled_model_.get_property(ov::execution_devices);
  tools::logger()->info(
    "YOLOV5 model={} configured_device={} selected_device={} execution_device={} input_type={} "
    "candidates={}",
    model_path_, configured_device, device_, available_devices_text(execution_devices),
    model_input_type.get_type_name(), output_shape[1]);
}

cv::Mat YOLOV5::inference_image(const cv::Mat & raw_img)
{
  if (!use_roi_) return raw_img;

  if (roi_.width == -1) roi_.width = raw_img.cols - roi_.x;
  if (roi_.height == -1) roi_.height = raw_img.rows - roi_.y;
  const cv::Rect image_bounds(0, 0, raw_img.cols, raw_img.rows);
  if (roi_.width <= 0 || roi_.height <= 0 || (roi_ & image_bounds) != roi_) {
    throw std::runtime_error("YOLOV5 ROI is outside the input image");
  }
  offset_ = roi_.tl();
  return raw_img(roi_);
}

std::list<Armor> YOLOV5::detect(const cv::Mat & raw_img, int frame_count)
{
  if (raw_img.empty()) {
    tools::logger()->warn("Empty img!, camera drop!");
    return std::list<Armor>();
  }

  const cv::Mat bgr_img = inference_image(raw_img);

  auto x_scale = static_cast<double>(kInputHeight) / bgr_img.rows;
  auto y_scale = static_cast<double>(kInputWidth) / bgr_img.cols;
  auto scale = std::min(x_scale, y_scale);
  auto h = static_cast<int>(bgr_img.rows * scale);
  auto w = static_cast<int>(bgr_img.cols * scale);

  auto input = cv::Mat(kInputHeight, kInputWidth, CV_8UC3, cv::Scalar(0, 0, 0));
  auto roi = cv::Rect(0, 0, w, h);
  cv::resize(bgr_img, input(roi), {w, h});
  ov::Tensor input_tensor(
    ov::element::u8, {1, kInputHeight, kInputWidth, 3}, input.data);

  sync_infer_request_.set_input_tensor(input_tensor);
  sync_infer_request_.infer();

  auto output_tensor = sync_infer_request_.get_output_tensor();
  auto output_shape = output_tensor.get_shape();
  if (
    output_shape.size() != 3 || output_shape[0] != 1 ||
    output_shape[2] != kOutputValues) {
    throw std::runtime_error("YOLOV5 expects an OpenVINO output shaped [1, candidates, 22]");
  }
  cv::Mat output(
    static_cast<int>(output_shape[1]), static_cast<int>(output_shape[2]), CV_32F,
    output_tensor.data<float>());

  return parse(scale, output, raw_img, frame_count);
}

YOLOFrameData YOLOV5::detect(YOLOFrameData frame_data, int frame_count)
{
  if (frame_data.frame.empty()) {
    tools::logger()->warn("Empty img!, camera drop!");
    return YOLOFrameData();
  }

  const cv::Mat bgr_img = inference_image(frame_data.frame);

  return async_pipeline_.detect(
    bgr_img, frame_data, frame_count,
    [this](double scale, cv::Mat & output, const cv::Mat & img, int finished_frame_count) {
      return parse(scale, output, img, finished_frame_count);
    });
}

std::list<Armor> YOLOV5::parse(
  double scale, cv::Mat & output, const cv::Mat & bgr_img, int frame_count)
{
  if (output.cols != kOutputValues) {
    throw std::runtime_error("YOLOV5 decoder expects 22 values per candidate");
  }

  if (output.type() != CV_32F || !std::isfinite(scale) || scale <= 0.0) {
    throw std::runtime_error("YOLOV5 decoder received invalid output data");
  }

  const int source_width = use_roi_ ? roi_.width : bgr_img.cols;
  const int source_height = use_roi_ ? roi_.height : bgr_img.rows;
  if (source_width <= 0 || source_height <= 0) {
    throw std::runtime_error("YOLOV5 decoder received an empty source image");
  }

  // Model output: TL, BL, BR, TR, objectness, red/blue/gray/purple, G/1/2/3/4/5/O/Bs/Bb.
  std::vector<int> color_ids, num_ids;
  std::vector<float> confidences;
  std::vector<cv::Rect> boxes;
  std::vector<std::vector<cv::Point2f>> armors_key_points;
  for (int r = 0; r < output.rows; r++) {
    const float * row = output.ptr<float>(r);
    const float score = sigmoid(row[8]);

    if (!std::isfinite(score) || score < score_threshold_) continue;

    const int color_id = argmax(row + 9, kColorCount);
    const int class_id = argmax(row + 13, kClassCount);
    if (color_id < 0 || class_id < 0 || color_id >= 2) continue;

    std::vector<cv::Point2f> armor_key_points;
    armor_key_points.reserve(4);
    constexpr std::array<int, 4> clockwise_indices = {0, 3, 2, 1};
    bool valid_keypoints = true;
    for (const int point_index : clockwise_indices) {
      const float model_x = row[point_index * 2];
      const float model_y = row[point_index * 2 + 1];
      if (!std::isfinite(model_x) || !std::isfinite(model_y)) {
        valid_keypoints = false;
        break;
      }
      armor_key_points.emplace_back(
        std::clamp(static_cast<float>(model_x / scale), 0.0F, source_width - 1.0F),
        std::clamp(static_cast<float>(model_y / scale), 0.0F, source_height - 1.0F));
    }
    if (!valid_keypoints) continue;

    float min_x = armor_key_points[0].x;
    float max_x = armor_key_points[0].x;
    float min_y = armor_key_points[0].y;
    float max_y = armor_key_points[0].y;

    for (int i = 1; i < armor_key_points.size(); i++) {
      if (armor_key_points[i].x < min_x) min_x = armor_key_points[i].x;
      if (armor_key_points[i].x > max_x) max_x = armor_key_points[i].x;
      if (armor_key_points[i].y < min_y) min_y = armor_key_points[i].y;
      if (armor_key_points[i].y > max_y) max_y = armor_key_points[i].y;
    }
    if (
      max_x - min_x < 1.0F || max_y - min_y < 1.0F ||
      std::abs(cv::contourArea(armor_key_points)) < 1.0) {
      continue;
    }

    const int left = std::clamp(static_cast<int>(std::floor(min_x)), 0, source_width - 1);
    const int top = std::clamp(static_cast<int>(std::floor(min_y)), 0, source_height - 1);
    const int right = std::clamp(static_cast<int>(std::ceil(max_x)), left + 1, source_width);
    const int bottom = std::clamp(static_cast<int>(std::ceil(max_y)), top + 1, source_height);
    cv::Rect rect(left, top, right - left, bottom - top);

    color_ids.emplace_back(color_id);
    num_ids.emplace_back(class_id);
    boxes.emplace_back(rect);
    confidences.emplace_back(score);
    armors_key_points.emplace_back(armor_key_points);
  }

  std::vector<int> indices;
  cv::dnn::NMSBoxes(boxes, confidences, score_threshold_, nms_threshold_, indices);

  std::list<Armor> armors;
  for (const auto & i : indices) {
    armors.emplace_back(make_armor(
      color_ids[i], num_ids[i], confidences[i], boxes[i], armors_key_points[i]));
  }

  tmp_img_ = bgr_img;
  for (auto it = armors.begin(); it != armors.end();) {
    if (!check_name(*it)) {
      it = armors.erase(it);
      continue;
    }

    if (!check_type(*it)) {
      it = armors.erase(it);
      continue;
    }
    // 使用传统方法二次矫正角点
    if (use_traditional_) detector_.detect(*it, bgr_img);

    it->center_norm = get_center_norm(bgr_img, it->center);
    ++it;
  }

  if (debug_) draw_detections(bgr_img, armors, frame_count);

  return armors;
}

Armor YOLOV5::make_armor(
  int color_id, int class_id, float confidence, const cv::Rect & box,
  const std::vector<cv::Point2f> & keypoints) const
{
  Armor armor = use_roi_ ? Armor(color_id, class_id, confidence, box, keypoints, offset_)
                         : Armor(color_id, class_id, confidence, box, keypoints);

  armor.color = MODEL_COLORS.at(color_id);
  armor.name = MODEL_NAMES.at(class_id);
  armor.type = MODEL_TYPES.at(class_id);
  armor.class_id = class_id;

  // Infantry labels do not encode small/big armor, so use the landmark geometry.
  if (class_id >= 3 && class_id <= 5) {
    armor.type = armor.ratio >= BIG_ARMOR_RATIO_THRESHOLD ? ArmorType::big : ArmorType::small;
  }
  if (use_roi_) {
    armor.box.x += static_cast<int>(offset_.x);
    armor.box.y += static_cast<int>(offset_.y);
  }
  return armor;
}

int YOLOV5::argmax(const float * values, int count)
{
  int best_index = -1;
  float best_value = -std::numeric_limits<float>::infinity();
  for (int i = 0; i < count; ++i) {
    if (std::isfinite(values[i]) && values[i] > best_value) {
      best_value = values[i];
      best_index = i;
    }
  }
  return best_index;
}

bool YOLOV5::check_name(const Armor & armor) const
{
  auto name_ok = armor.name != ArmorName::not_armor;
  auto confidence_ok = armor.confidence > min_confidence_;

  // 保存不确定的图案，用于神经网络的迭代
  // if (name_ok && !confidence_ok) save(armor);

  return name_ok && confidence_ok;
}

bool YOLOV5::check_type(const Armor & armor) const
{
  if (armor.name == ArmorName::one) return armor.type == ArmorType::big;
  if (
    armor.name == ArmorName::two || armor.name == ArmorName::sentry ||
    armor.name == ArmorName::outpost) {
    return armor.type == ArmorType::small;
  }
  return true;
}

cv::Point2f YOLOV5::get_center_norm(const cv::Mat & bgr_img, const cv::Point2f & center) const
{
  auto h = bgr_img.rows;
  auto w = bgr_img.cols;
  return {center.x / w, center.y / h};
}

void YOLOV5::draw_detections(
  const cv::Mat & img, const std::list<Armor> & armors, int frame_count) const
{
  auto detection = img.clone();
  tools::draw_text(detection, fmt::format("[{}]", frame_count), {10, 30}, {255, 255, 255});
  for (const auto & armor : armors) {
    auto info = fmt::format(
      "{:.2f} {} {} {}", armor.confidence, COLORS[armor.color], ARMOR_NAMES[armor.name],
      ARMOR_TYPES[armor.type]);
    tools::draw_points(detection, armor.points, {0, 255, 0});
    tools::draw_text(detection, info, armor.center, {0, 255, 0});
  }

  if (use_roi_) {
    cv::Scalar green(0, 255, 0);
    cv::rectangle(detection, roi_, green, 2);
  }
  cv::resize(detection, detection, {}, 0.5, 0.5);  // 显示时缩小图片尺寸
  cv::imshow("detection", detection);
}

void YOLOV5::save(const Armor & armor) const
{
  auto file_name = fmt::format("{:%Y-%m-%d_%H-%M-%S}", std::chrono::system_clock::now());
  auto img_path = fmt::format("{}/{}_{}.jpg", save_path_, armor.name, file_name);
  cv::imwrite(img_path, tmp_img_);
}

float YOLOV5::sigmoid(float value) noexcept
{
  if (value >= 0.0F) return 1.0F / (1.0F + std::exp(-value));
  const float exp_value = std::exp(value);
  return exp_value / (1.0F + exp_value);
}

std::list<Armor> YOLOV5::postprocess(
  double scale, cv::Mat & output, const cv::Mat & bgr_img, int frame_count)
{
  return parse(scale, output, bgr_img, frame_count);
}

}  // namespace auto_aim
