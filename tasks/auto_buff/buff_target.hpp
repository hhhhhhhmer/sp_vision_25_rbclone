#ifndef AUTO_BUFF__TARGET_HPP
#define AUTO_BUFF__TARGET_HPP

#include <Eigen/Dense>
#include <deque>
#include <opencv2/opencv.hpp>
#include <optional>
#include <string>
#include <vector>
#include <memory>

#include "buff_detector.hpp"
#include "buff_type.hpp"
#include "tools/extended_kalman_filter.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/plotter.hpp"
#include "tools/ransac_sine_fitter.hpp"

namespace auto_buff
{
class PhaseDirectionTracker
{
public:
  void rebase(double phase, bool preserve_direction);
  void shift_reference(double delta);
  void update(double phase);
  void reset();
  int direction() const { return direction_; }
  bool ready() const { return direction_ != 0; }

private:
  bool has_last_phase_ = false;
  double last_phase_ = 0.0;
  int direction_ = 0;
  int score_ = 0;
  int reverse_candidate_direction_ = 0;
  int reverse_confirm_count_ = 0;
  std::deque<double> deltas_;
  std::deque<int> votes_;
};

/// Target 基类

class Target
{
public:
  Target();
  virtual void get_target(
    const std::optional<PowerRune> & p,
    std::chrono::steady_clock::time_point & timestamp) = 0;  // 纯虚函数

  void get_target(
    const std::vector<PowerRune> & observations,
    std::chrono::steady_clock::time_point & timestamp);

  virtual void predict(double dt) = 0;  // 纯虚函数

  Eigen::Vector3d point_buff2world(const Eigen::Vector3d & point_in_buff) const;

  virtual Eigen::Matrix3d rotation_buff2world() const;

  bool is_unsolve() const;

  bool can_control() const;

  bool prediction_ready() const;

  TargetReadiness readiness() const { return readiness_; }

  bool is_blind() const;

  bool can_fire(std::chrono::steady_clock::time_point now) const;

  int reset_count() const { return reset_count_; }

  virtual void reset();

  Eigen::VectorXd ekf_x() const;

  double spd = 0;  //调试用

  virtual std::unique_ptr<Target> clone() const = 0;

protected:
  virtual void init(double nowtime, const PowerRune & p) = 0;  // 纯虚函数

  virtual void update(double nowtime, const PowerRune & p) = 0;  // 纯虚函数

  double relative_time(std::chrono::steady_clock::time_point timestamp);

  bool predict_without_measurement(std::chrono::steady_clock::time_point timestamp);

  void record_measurement(
    const PowerRune & p, std::chrono::steady_clock::time_point timestamp);

  double update_plane_basis(const PowerRune & p, bool initialize);

  double measure_phase(const PowerRune & p, double reference) const;

  Eigen::VectorXd x0_;
  Eigen::MatrixXd P0_;
  Eigen::MatrixXd A_;
  Eigen::MatrixXd Q_;
  Eigen::MatrixXd H_;
  Eigen::MatrixXd R_;
  tools::ExtendedKalmanFilter ekf_;
  double lasttime_ = 0;
  bool first_in_;
  bool unsolvable_;
  TargetReadiness readiness_ = TargetReadiness::LOST;
  int last_track_id_ = -1;
  bool blind_ = false;
  bool has_start_timestamp_ = false;
  bool has_measurement_timestamp_ = false;
  bool has_full_observation_timestamp_ = false;
  std::chrono::steady_clock::time_point start_timestamp_{};
  std::chrono::steady_clock::time_point last_measurement_timestamp_{};
  std::chrono::steady_clock::time_point last_full_observation_timestamp_{};
  BuffPoseQuality last_pose_quality_ = BuffPoseQuality::FULL_8_POINT;
  int reset_count_ = 0;
  int innovation_reject_count_ = 0;
  bool has_plane_basis_ = false;
  Eigen::Vector3d plane_normal_{1.0, 0.0, 0.0};
  Eigen::Vector3d plane_normal_sum_{0.0, 0.0, 0.0};
  double plane_normal_weight_ = 0.0;
  Eigen::Vector3d phase_zero_axis_{0.0, 0.0, 1.0};
  Eigen::Vector3d phase_quarter_axis_{0.0, -1.0, 0.0};
  bool has_pending_phase_recovery_ = false;
  double pending_phase_recovery_ = 0.0;
  int pending_phase_recovery_hits_ = 0;
};

/// SmallTarget子类

class SmallTarget : public Target
{
public:
  SmallTarget();

  using Target::get_target;

  void get_target(
    const std::optional<PowerRune> & p, std::chrono::steady_clock::time_point & timestamp) override;

  void predict(double dt) override;

  std::unique_ptr<Target> clone() const override { return std::make_unique<SmallTarget>(*this); }

  void reset() override;

private:
  void init(double nowtime, const PowerRune & p) override;

  void update(double nowtime, const PowerRune & p) override;

  int small_prediction_roll_direction() const;

  bool has_stable_small_prediction_direction() const;

  const double SMALL_W = CV_PI / 3;
  // const double SMALL_W = 0;
  PhaseDirectionTracker phase_direction_;
};

/// BigTarget子类

class BigTarget : public Target
{
public:
  BigTarget();

  using Target::get_target;

  void get_target(
    const std::optional<PowerRune> & p, std::chrono::steady_clock::time_point & timestamp) override;

  void predict(double dt) override;

  std::unique_ptr<Target> clone() const override { return std::make_unique<BigTarget>(*this); }

  void reset() override;

private:
  struct PhaseSample
  {
    double time = 0.0;
    double phase = 0.0;
    double weight = 1.0;
  };

  void init(double nowtime, const PowerRune & p) override;

  void update(double nowtime, const PowerRune & p) override;

  void clear_speed_samples(bool clear_fitter);

  std::optional<double> estimate_window_speed() const;

  void add_speed_sample(double nowtime, double observed_phase, const PowerRune & p);

  tools::RansacSineFitter spd_fitter_;
  PhaseDirectionTracker phase_direction_;

  double fit_spd_ = 1.1775;
  double fit_blend_ = 0.0;
  double last_accepted_speed_time_ = 0.0;
  double last_fitter_sample_time_ = -1.0;
  double pause_speed_samples_until_ = 0.0;
  int speed_model_direction_ = 0;
  bool has_speed_center_ = false;
  Eigen::Vector3d last_speed_center_{0.0, 0.0, 0.0};
  std::deque<PhaseSample> phase_samples_;
  std::deque<double> accepted_speed_samples_;
};

}  // namespace auto_buff
#endif
