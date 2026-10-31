#ifndef AUTO_AIM__CENTER_ACCELERATION_ESTIMATOR_HPP
#define AUTO_AIM__CENTER_ACCELERATION_ESTIMATOR_HPP

#include <Eigen/Dense>
#include <chrono>
#include <cstddef>
#include <deque>
#include <limits>

namespace auto_aim
{

struct CenterAccelerationEstimatorConfig
{
  // 保留开关，默认关闭：当前实现在平移靶上净亏（加速段有效、脉冲后过冲）。
  // 需要时在 YAML 里显式设置 center_accel_ff_enabled: true。
  bool enabled = false;
  double window_seconds = 0.25;
  std::size_t min_samples = 6;
  double min_span_seconds = 0.12;
  double ema_alpha = 0.2;
  double max_acceleration = 6.0;
  double max_jerk = 30.0;
  double max_fit_rmse = 0.08;
  double stale_timeout_seconds = 0.15;
};

/** @brief Fits planar center acceleration from timestamped posterior positions. */
class CenterAccelerationEstimator
{
public:
  using TimePoint = std::chrono::steady_clock::time_point;

  explicit CenterAccelerationEstimator(
    CenterAccelerationEstimatorConfig config = CenterAccelerationEstimatorConfig{});

  void reset();
  void add_sample(TimePoint time, const Eigen::Vector2d & position);

  /** @return filtered acceleration, or zero when disabled, invalid, or stale */
  Eigen::Vector2d acceleration(TimePoint time) const;
  bool valid(TimePoint time) const;

  std::size_t sample_count() const { return samples_.size(); }
  double last_fit_rmse() const { return last_fit_rmse_; }

private:
  struct Sample
  {
    TimePoint time;
    Eigen::Vector2d position;
  };

  CenterAccelerationEstimatorConfig config_;
  std::deque<Sample> samples_;
  Eigen::Vector2d acceleration_ = Eigen::Vector2d::Zero();
  bool estimate_valid_ = false;
  double last_fit_rmse_ = std::numeric_limits<double>::infinity();
  TimePoint last_fit_time_{};
  bool has_last_fit_time_ = false;

  void fit();
  void invalidate_estimate();
};

}  // namespace auto_aim

#endif  // AUTO_AIM__CENTER_ACCELERATION_ESTIMATOR_HPP
