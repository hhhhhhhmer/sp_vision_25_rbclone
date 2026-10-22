#ifndef AUTO_BUFF__RUNE_SYSTEM_HPP
#define AUTO_BUFF__RUNE_SYSTEM_HPP

#include <array>
#include <chrono>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <Eigen/Geometry>
#include <opencv2/core.hpp>

namespace auto_buff
{
enum class BuffMode { SMALL, BIG };
enum class EnemyColor : uint8_t { RED = 0, BLUE = 1 };

enum class RuneFailureReason
{
  NONE,
  MODE_CHANGED,
  EMPTY_IMAGE,
  INVALID_COLOR,
  INVALID_POSE,
  DETECTION_EMPTY,
  CORE_NOT_READY,
  INVALID_OUTPUT,
  INVALID_BULLET_SPEED,
  INFERENCE_ERROR
};

struct RuneCommand
{
  bool found = false;
  bool fire = false;
  float yaw = 0.0f;
  float pitch = 0.0f;
};

struct RuneDebugDetection
{
  std::array<cv::Point2f, 5> keypoints{};
  std::array<float, 5> keypoint_confidences{};
  int class_id = 0;
  float confidence = 0.0f;
};

struct RuneDebugSnapshot
{
  uint64_t frame_sequence = 0;
  RuneFailureReason failure_reason = RuneFailureReason::CORE_NOT_READY;
  std::vector<RuneDebugDetection> detections;
  std::vector<std::vector<cv::Point>> armor_contours;
  std::vector<std::vector<cv::Point>> light_arm_contours;
  std::vector<std::vector<cv::Point>> center_contours;
  std::vector<cv::Point2f> current_reprojection;
  std::vector<cv::Point2f> predicted_reprojection;
  std::optional<double> reprojection_error;
  bool uv_pose_active = false;
  std::optional<double> uv_reprojection_error;
  double uv_nis = 0.0;
  int uv_rejected_blades = 0;
  bool pnp_used = false;
  std::optional<double> keypoint_reprojection_error;
  std::optional<double> phase;
  std::optional<double> continuous_phase;
  std::optional<double> angular_velocity;
  std::array<double, 5> big_rune_parameters{};
  bool big_rune_model_ready = false;
  bool found = false;
  bool fire = false;
  double detection_ms = 0.0;
  double core_ms = 0.0;
  double total_ms = 0.0;
};

class RuneSystem
{
public:
  explicit RuneSystem(const std::string & robot_config_path);
  ~RuneSystem();

  RuneSystem(const RuneSystem &) = delete;
  RuneSystem & operator=(const RuneSystem &) = delete;

  RuneCommand process(
    const cv::Mat & image,
    std::chrono::steady_clock::time_point capture_time,
    const Eigen::Quaterniond & imu_q,
    BuffMode mode,
    EnemyColor enemy_color,
    float bullet_speed);

  void reset();
  const RuneDebugSnapshot & debug_snapshot() const;

private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};
}  // namespace auto_buff

#endif
