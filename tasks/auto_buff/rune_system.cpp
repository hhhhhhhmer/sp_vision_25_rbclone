#include "rune_system.hpp"

#include <cmath>
#include <filesystem>
#include <stdexcept>

#include <opencv2/calib3d.hpp>

#include "detector/rune_detector.hpp"
#include "adapter/pose_adapter.hpp"
#include "PowerRuneProcessor.hpp"
#include "RuneDecisionModule.hpp"
#include "PnPVariable.hpp"
#include "common/PowerRuneDiagnostics.hpp"
#include "json.hpp"
#include "tools/logger.hpp"

#ifndef AUTO_BUFF_DEFAULT_CONFIG_PATH
#error "AUTO_BUFF_DEFAULT_CONFIG_PATH must be defined by CMake"
#endif

namespace auto_buff
{
namespace
{
constexpr float kMinimumBulletSpeed = 10.0f;
constexpr float kMaximumBulletSpeed = 35.0f;

double elapsed_ms(std::chrono::steady_clock::time_point begin)
{
  return std::chrono::duration<double, std::milli>(
    std::chrono::steady_clock::now() - begin).count();
}

bool valid_color(EnemyColor color)
{
  return color == EnemyColor::RED || color == EnemyColor::BLUE;
}

std::vector<cv::Point2f> project_rp_point(
  const CameraPose & camera_pose, const Eigen::Vector3d & point_in_rp)
{
  const Eigen::Vector3d point_in_camera =
    camera_pose.R_car_from_camera.transpose() *
    (point_in_rp - camera_pose.t_car_from_camera);
  if (!point_in_camera.allFinite() || point_in_camera.z() <= 1e-6) return {};
  std::vector<cv::Point3d> object_points{{
    point_in_camera.x(), point_in_camera.y(), point_in_camera.z()}};
  std::vector<cv::Point2d> projected;
  cv::projectPoints(
    object_points, cv::Vec3d::all(0.0), cv::Vec3d::all(0.0),
    J_POWER_RUNE.camera_matrix(), J_POWER_RUNE.distortion(), projected);
  std::vector<cv::Point2f> image_points;
  image_points.reserve(projected.size());
  for (const cv::Point2d & point : projected) image_points.emplace_back(point);
  return image_points;
}
}  // namespace

class RuneSystem::Impl
{
public:
  explicit Impl(const std::string & robot_config_path)
  {
    J_POWER_RUNE.initialize(AUTO_BUFF_DEFAULT_CONFIG_PATH, robot_config_path);
    update_power_rune_camera_calibration();
    default_bullet_speed_ =
      static_cast<double>(J_POWER_RUNE.config_["rune_ballistic_model"]["bullet_flying_speed"]);
    detector_ = std::make_unique<RuneDetector>();
  }

  void create_core(int width, int height)
  {
    J_POWER_RUNE.set_image_size(width, height);
    update_power_rune_camera_calibration();
    decision_ = std::make_unique<RuneDecisionModule>();
    processor_ = std::make_unique<PowerRuneProcessor>(*decision_);
    core_width_ = width;
    core_height_ = height;
  }

  void reset_core()
  {
    PowerRuneDiagnostics::instance().clear();
    if (core_width_ > 0 && core_height_ > 0) create_core(core_width_, core_height_);
    else {
      processor_.reset();
      decision_.reset();
    }
  }

  RuneCommand process(
    const cv::Mat & image, RuneTimestamp capture_time, const Eigen::Quaterniond & imu_q,
    BuffMode mode, EnemyColor enemy_color, float bullet_speed)
  {
    has_active_state_ = true;
    const auto total_begin = std::chrono::steady_clock::now();
    debug_ = {};
    debug_.frame_sequence = ++frame_sequence_;

    if (last_mode_ && *last_mode_ != mode) {
      last_mode_ = mode;
      reset_core();
      debug_.failure_reason = RuneFailureReason::MODE_CHANGED;
      debug_.total_ms = elapsed_ms(total_begin);
      return {};
    }
    last_mode_ = mode;

    if (image.empty() || image.type() != CV_8UC3) {
      debug_.failure_reason = RuneFailureReason::EMPTY_IMAGE;
      debug_.total_ms = elapsed_ms(total_begin);
      return {};
    }
    if (!valid_color(enemy_color)) {
      debug_.failure_reason = RuneFailureReason::INVALID_COLOR;
      debug_.total_ms = elapsed_ms(total_begin);
      return {};
    }
    if (!imu_q.coeffs().allFinite() || imu_q.norm() < 1e-6) {
      debug_.failure_reason = RuneFailureReason::INVALID_POSE;
      debug_.total_ms = elapsed_ms(total_begin);
      return {};
    }
    if (!processor_ || image.cols > core_width_ || image.rows > core_height_) {
      create_core(std::max(image.cols, core_width_), std::max(image.rows, core_height_));
    }

    std::vector<RuneDetection> detections;
    const auto detection_begin = std::chrono::steady_clock::now();
    try {
      detections = detector_->detect(image, mode);
    } catch (const std::exception & error) {
      tools::logger()->error("[RuneSystem] inference failed: {}", error.what());
      debug_.failure_reason = RuneFailureReason::INFERENCE_ERROR;
      debug_.detection_ms = elapsed_ms(detection_begin);
      debug_.total_ms = elapsed_ms(total_begin);
      return {};
    }
    debug_.detection_ms = elapsed_ms(detection_begin);
    debug_.detections.reserve(detections.size());
    for (const auto & detection : detections) {
      debug_.detections.push_back({
        detection.keypoints, detection.keypoint_confidences, detection.class_id,
        detection.confidence});
    }
    if (detections.empty()) {
      debug_.failure_reason = RuneFailureReason::DETECTION_EMPTY;
      debug_.total_ms = elapsed_ms(total_begin);
      return {};
    }

    const bool bullet_speed_valid =
      std::isfinite(bullet_speed) && bullet_speed >= kMinimumBulletSpeed &&
      bullet_speed <= kMaximumBulletSpeed;
    J_POWER_RUNE.set_bullet_speed(bullet_speed_valid ? bullet_speed : default_bullet_speed_);

    power_rune::RuneInput input;
    input.is_big_rune = mode == BuffMode::BIG;
    input.ori_mat = image;
    input.camera_pose = detail::make_rp_camera_pose(imu_q);
    input.timestamp = capture_time;
    input.target_color = static_cast<int>(enemy_color);
    input.nn_rune_infos.reserve(detections.size());
    for (const auto & detection : detections) {
      input.nn_rune_infos.push_back({
        cv::Point(cvRound(detection.keypoints[0].x), cvRound(detection.keypoints[0].y)),
        cv::Point(cvRound(detection.keypoints[1].x), cvRound(detection.keypoints[1].y)),
        cv::Point(cvRound(detection.keypoints[3].x), cvRound(detection.keypoints[3].y)),
        cv::Point(cvRound(detection.keypoints[4].x), cvRound(detection.keypoints[4].y)),
        cv::Point(cvRound(detection.keypoints[2].x), cvRound(detection.keypoints[2].y)),
        detection.class_id});
    }

    const auto core_begin = std::chrono::steady_clock::now();
    try {
      processor_->process_power_rune(input);
    } catch (const std::exception & error) {
      tools::logger()->error("[RuneSystem] RP core failed: {}", error.what());
      debug_.failure_reason = RuneFailureReason::CORE_NOT_READY;
      debug_.core_ms = elapsed_ms(core_begin);
      debug_.total_ms = elapsed_ms(total_begin);
      return {};
    }
    debug_.core_ms = elapsed_ms(core_begin);

    const PowerRuneCoreDebugState & core_debug = processor_->debug_state();
    debug_.armor_contours = core_debug.armor_contours;
    debug_.light_arm_contours = core_debug.light_arm_contours;
    debug_.center_contours = core_debug.center_contours;
    debug_.current_reprojection = core_debug.current_reprojection;
    debug_.reprojection_error = core_debug.reprojection_error;
    debug_.uv_pose_active = core_debug.uv_pose_active;
    debug_.uv_reprojection_error = core_debug.uv_reprojection_error;
    debug_.uv_nis = core_debug.uv_nis;
    debug_.uv_rejected_blades = core_debug.uv_rejected_blades;
    debug_.pnp_used = core_debug.pnp_used;
    debug_.keypoint_reprojection_error = core_debug.keypoint_reprojection_error;
    debug_.phase = core_debug.phase;
    debug_.continuous_phase = core_debug.continuous_phase;
    debug_.angular_velocity = core_debug.angular_velocity;
    debug_.big_rune_parameters = core_debug.big_rune_parameters;
    debug_.big_rune_model_ready = core_debug.big_rune_model_ready;
    if (!core_debug.produced_target) {
      debug_.failure_reason = RuneFailureReason::CORE_NOT_READY;
      debug_.total_ms = elapsed_ms(total_begin);
      return {};
    }

    const power_rune::RuneSendData rp_output =
      decision_->get_send_data(mode == BuffMode::BIG);
    if (decision_->predicted_armor_center()) {
      debug_.predicted_reprojection =
        project_rp_point(input.camera_pose, *decision_->predicted_armor_center());
    }
    RuneCommand command;
    command.found = rp_output.is_find_buff != 0;
    command.fire = command.found && rp_output.is_enable_fire != 0 && bullet_speed_valid;
    if (command.found) {
      command.yaw = -rp_output.yaw + static_cast<float>(J_POWER_RUNE.yaw_offset_rad());
      command.pitch = rp_output.pitch + static_cast<float>(J_POWER_RUNE.pitch_offset_rad());
    }
    if (!std::isfinite(command.yaw) || !std::isfinite(command.pitch)) {
      command = {};
      debug_.failure_reason = RuneFailureReason::INVALID_OUTPUT;
    } else if (!bullet_speed_valid) {
      debug_.failure_reason = RuneFailureReason::INVALID_BULLET_SPEED;
      if (++invalid_speed_log_counter_ % 100 == 1) {
        tools::logger()->warn(
          "[RuneSystem] invalid bullet speed {}, aiming with fallback {} and inhibiting fire",
          bullet_speed, default_bullet_speed_);
      }
    } else if (!command.found) {
      debug_.failure_reason = RuneFailureReason::CORE_NOT_READY;
    } else {
      debug_.failure_reason = RuneFailureReason::NONE;
    }
    debug_.found = command.found;
    debug_.fire = command.fire;
    debug_.total_ms = elapsed_ms(total_begin);
    return command;
  }

  std::unique_ptr<RuneDetector> detector_;
  std::unique_ptr<RuneDecisionModule> decision_;
  std::unique_ptr<PowerRuneProcessor> processor_;
  std::optional<BuffMode> last_mode_;
  RuneDebugSnapshot debug_;
  uint64_t frame_sequence_ = 0;
  uint64_t invalid_speed_log_counter_ = 0;
  bool has_active_state_ = false;
  int core_width_ = 0;
  int core_height_ = 0;
  double default_bullet_speed_ = 24.5;
};

RuneSystem::RuneSystem(const std::string & robot_config_path)
: impl_(std::make_unique<Impl>(robot_config_path))
{
}

RuneSystem::~RuneSystem() = default;

RuneCommand RuneSystem::process(
  const cv::Mat & image, std::chrono::steady_clock::time_point capture_time,
  const Eigen::Quaterniond & imu_q, BuffMode mode, EnemyColor enemy_color,
  float bullet_speed)
{
  return impl_->process(image, capture_time, imu_q, mode, enemy_color, bullet_speed);
}

void RuneSystem::reset()
{
  if (!impl_->has_active_state_) return;
  impl_->last_mode_.reset();
  impl_->reset_core();
  impl_->debug_ = {};
  impl_->has_active_state_ = false;
}

const RuneDebugSnapshot & RuneSystem::debug_snapshot() const
{
  return impl_->debug_;
}
}  // namespace auto_buff
