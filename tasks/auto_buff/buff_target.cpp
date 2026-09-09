#include "buff_target.hpp"

#include <algorithm>
#include <cmath>
#include <fmt/core.h>

namespace auto_buff
{
namespace
{
int configured_small_buff_direction()
{
  if (SMALL_BUFF_DIRECTION > 0) return 1;
  if (SMALL_BUFF_DIRECTION < 0) return -1;
  return 0;
}

int small_buff_direction(int auto_direction)
{
  const int configured_direction = configured_small_buff_direction();
  if (configured_direction != 0) return configured_direction;
  if (auto_direction > 0) return 1;
  if (auto_direction < 0) return -1;
  return 0;
}

int signum(int value)
{
  if (value > 0) return 1;
  if (value < 0) return -1;
  return 0;
}

double unwrap_near(double angle, double reference)
{
  return angle + std::round((reference - angle) / CV_2PI) * CV_2PI;
}

double phase_noise_multiplier(const PowerRune & p)
{
  if (p.pose_quality == BuffPoseQuality::PARTIAL_4_POINT) return 10.0 / 6.0;
  if (p.pose_quality == BuffPoseQuality::PARTIAL_5_POINT) return 2.0;
  return 1.0;
}

double median(std::vector<double> values)
{
  if (values.empty()) return 0.0;
  const size_t middle = values.size() / 2;
  std::nth_element(values.begin(), values.begin() + middle, values.end());
  double result = values[middle];
  if (values.size() % 2 == 0) {
    std::nth_element(values.begin(), values.begin() + middle - 1, values.end());
    result = 0.5 * (result + values[middle - 1]);
  }
  return result;
}
}  // namespace

void PhaseDirectionTracker::rebase(double phase, bool preserve_direction)
{
  has_last_phase_ = true;
  last_phase_ = phase;
  reverse_candidate_direction_ = 0;
  reverse_confirm_count_ = 0;
  if (!preserve_direction) {
    deltas_.clear();
    votes_.clear();
    direction_ = 0;
    score_ = 0;
  } else if (direction_ != 0) {
    score_ = direction_ * 6;
  }
}

void PhaseDirectionTracker::reset()
{
  has_last_phase_ = false;
  last_phase_ = 0.0;
  direction_ = 0;
  score_ = 0;
  reverse_candidate_direction_ = 0;
  reverse_confirm_count_ = 0;
  deltas_.clear();
  votes_.clear();
}

void PhaseDirectionTracker::shift_reference(double delta)
{
  if (has_last_phase_) last_phase_ += delta;
}

void PhaseDirectionTracker::update(double phase)
{
  if (!has_last_phase_) {
    rebase(phase, true);
    return;
  }

  const double delta = phase - last_phase_;
  last_phase_ = phase;

  constexpr double min_direction_delta = CV_PI / 900.0;
  constexpr double max_direction_delta = CV_PI / 5.0;
  constexpr double confirm_window_delta = CV_PI / 120.0;
  constexpr int direction_window = 8;
  const int min_window_samples = std::max(2, BUFF_DIRECTION_CONFIRM_INTERVALS);
  const int confirm_vote_margin = min_window_samples;
  const int confirm_score = min_window_samples;
  constexpr int score_limit = 30;

  if (std::abs(delta) <= min_direction_delta || std::abs(delta) >= max_direction_delta) return;

  const int sample_direction = delta > 0.0 ? 1 : -1;
  deltas_.push_back(delta);
  votes_.push_back(sample_direction);
  while (static_cast<int>(deltas_.size()) > direction_window) {
    deltas_.pop_front();
    votes_.pop_front();
  }
  score_ = std::clamp(score_ + sample_direction, -score_limit, score_limit);

  double window_delta_sum = 0.0;
  int window_vote_sum = 0;
  for (size_t i = 0; i < deltas_.size(); ++i) {
    window_delta_sum += deltas_[i];
    window_vote_sum += votes_[i];
  }

  int window_direction = 0;
  if (
    static_cast<int>(deltas_.size()) >= min_window_samples &&
    std::abs(window_delta_sum) >= confirm_window_delta &&
    std::abs(window_vote_sum) >= confirm_vote_margin) {
    const int delta_direction = window_delta_sum > 0.0 ? 1 : -1;
    const int vote_direction = signum(window_vote_sum);
    if (delta_direction == vote_direction) window_direction = delta_direction;
  }

  if (window_direction == 0) {
    reverse_candidate_direction_ = 0;
    reverse_confirm_count_ = 0;
    return;
  }

  if (direction_ == 0) {
    if (signum(score_) == window_direction && std::abs(score_) >= confirm_score) {
      direction_ = window_direction;
      reverse_candidate_direction_ = 0;
      reverse_confirm_count_ = 0;
    }
    return;
  }
  // The rune does not reverse in one mode session. Opposite votes are treated as outliers.
}

/// Target

Target::Target() : first_in_(true), unsolvable_(true) {};

void Target::get_target(
  const std::vector<PowerRune> & observations,
  std::chrono::steady_clock::time_point & timestamp)
{
  if (observations.empty()) {
    get_target(std::nullopt, timestamp);
    return;
  }
  const auto primary = std::find_if(observations.begin(), observations.end(), [](const PowerRune & p) {
    return p.primary;
  });
  if (primary == observations.end()) {
    get_target(std::nullopt, timestamp);
    return;
  }
  get_target(*primary, timestamp);
}

void Target::reset()
{
  first_in_ = true;
  unsolvable_ = true;
  readiness_ = TargetReadiness::LOST;
  last_track_id_ = -1;
  blind_ = false;
  has_start_timestamp_ = false;
  has_measurement_timestamp_ = false;
  has_full_observation_timestamp_ = false;
  has_plane_basis_ = false;
  plane_normal_sum_.setZero();
  plane_normal_weight_ = 0.0;
  has_pending_phase_recovery_ = false;
  pending_phase_recovery_hits_ = 0;
  reset_count_++;
}

Eigen::Vector3d Target::point_buff2world(const Eigen::Vector3d & point_in_buff) const
{
  if (unsolvable_) return Eigen::Vector3d(0, 0, 0);
  const Eigen::Vector3d center_in_world =
    tools::ypd2xyz(Eigen::Vector3d(ekf_.x[0], ekf_.x[2], ekf_.x[3]));
  return rotation_buff2world() * point_in_buff + center_in_world;
}

Eigen::Matrix3d Target::rotation_buff2world() const
{
  if (!has_plane_basis_) {
    return tools::rotation_matrix(Eigen::Vector3d(ekf_.x[4], 0.0, ekf_.x[5]));
  }

  Eigen::Vector3d z =
    std::cos(ekf_.x[5]) * phase_zero_axis_ + std::sin(ekf_.x[5]) * phase_quarter_axis_;
  z.normalize();
  Eigen::Vector3d x = plane_normal_.normalized();
  Eigen::Vector3d y = z.cross(x).normalized();
  x = y.cross(z).normalized();

  Eigen::Matrix3d rotation;
  rotation.col(0) = x;
  rotation.col(1) = y;
  rotation.col(2) = z;
  return rotation;
}

bool Target::is_unsolve() const { return unsolvable_; }

bool Target::can_control() const { return readiness_ != TargetReadiness::LOST; }

bool Target::prediction_ready() const { return readiness_ == TargetReadiness::PREDICTING; }

bool Target::is_blind() const { return blind_; }

bool Target::can_fire(std::chrono::steady_clock::time_point now) const
{
  if (
    !prediction_ready() || unsolvable_ || blind_ || !has_full_observation_timestamp_ ||
    last_pose_quality_ != BuffPoseQuality::FULL_8_POINT) {
    return false;
  }
  return tools::delta_time(now, last_full_observation_timestamp_) <=
         BUFF_FIRE_FULL_OBSERVATION_MAX_AGE_S;
}

double Target::relative_time(std::chrono::steady_clock::time_point timestamp)
{
  if (!has_start_timestamp_) {
    start_timestamp_ = timestamp;
    has_start_timestamp_ = true;
  }
  return tools::delta_time(timestamp, start_timestamp_);
}

bool Target::predict_without_measurement(std::chrono::steady_clock::time_point timestamp)
{
  if (first_in_ || !has_measurement_timestamp_) {
    unsolvable_ = true;
    readiness_ = TargetReadiness::LOST;
    blind_ = true;
    return false;
  }

  const double measurement_age = tools::delta_time(timestamp, last_measurement_timestamp_);
  if (measurement_age > BUFF_TRACK_RETENTION_S) {
    reset();
    blind_ = true;
    return false;
  }
  if (measurement_age > BUFF_BLIND_TIMEOUT_S) {
    unsolvable_ = true;
    readiness_ = TargetReadiness::LOST;
    blind_ = true;
    return false;
  }

  const double nowtime = relative_time(timestamp);
  predict(std::max(0.0, nowtime - lasttime_));
  lasttime_ = nowtime;
  unsolvable_ = false;
  blind_ = true;
  return true;
}

void Target::record_measurement(
  const PowerRune & p, std::chrono::steady_clock::time_point timestamp)
{
  has_measurement_timestamp_ = true;
  last_measurement_timestamp_ = timestamp;
  last_pose_quality_ = p.pose_quality;
  if (p.pose_quality == BuffPoseQuality::FULL_8_POINT) {
    has_full_observation_timestamp_ = true;
    last_full_observation_timestamp_ = timestamp;
  }
  blind_ = false;
}

Eigen::VectorXd Target::ekf_x() const { return ekf_.x; }

/// SmallTarget

SmallTarget::SmallTarget() : Target() {}

void SmallTarget::reset()
{
  Target::reset();
  phase_direction_.reset();
}

double Target::update_plane_basis(const PowerRune & p, bool initialize)
{
  Eigen::Vector3d normal = p.plane_normal_in_world;
  if (!normal.allFinite() || normal.norm() < 1e-6) {
    if (has_plane_basis_) return 0.0;
    normal = Eigen::Vector3d::UnitX();
  }
  normal.normalize();

  if (has_plane_basis_) {
    if (normal.dot(plane_normal_) < 0.0) normal = -normal;
  } else if (normal.dot(p.xyz_in_world) > 0.0) {
    // PnP平面法向存在正负二义性，固定为朝向相机的一侧。
    normal = -normal;
  }

  Eigen::Vector3d previous_radial = Eigen::Vector3d::Zero();
  double previous_phase = 0.0;
  if (has_plane_basis_ && ekf_.x.size() >= 6) {
    previous_phase = ekf_.x[5];
    previous_radial = std::cos(previous_phase) * phase_zero_axis_ +
                      std::sin(previous_phase) * phase_quarter_axis_;
  }

  if (!has_plane_basis_ || initialize) {
    plane_normal_sum_ = normal;
    plane_normal_weight_ = 1.0;
    plane_normal_ = normal;
  } else {
    if (p.pose_quality != BuffPoseQuality::FULL_8_POINT) return 0.0;
    plane_normal_sum_ += normal;
    plane_normal_weight_ += 1.0;
    normal = plane_normal_sum_.normalized();
    plane_normal_ = normal;
  }

  Eigen::Vector3d zero_axis = Eigen::Vector3d::UnitZ() - normal.z() * normal;
  if (zero_axis.norm() < 0.1) {
    zero_axis = Eigen::Vector3d::UnitX() - normal.x() * normal;
  }
  zero_axis.normalize();
  Eigen::Vector3d quarter_axis = normal.cross(zero_axis).normalized();

  phase_zero_axis_ = zero_axis;
  phase_quarter_axis_ = quarter_axis;
  has_plane_basis_ = true;

  if (previous_radial.norm() > 1e-6) {
    const double reexpressed = std::atan2(
      previous_radial.dot(phase_quarter_axis_), previous_radial.dot(phase_zero_axis_));
    ekf_.x[5] = unwrap_near(reexpressed, previous_phase);
    return ekf_.x[5] - previous_phase;
  }
  return 0.0;
}

double Target::measure_phase(const PowerRune & p, double reference) const
{
  Eigen::Vector3d radial = p.blade_xyz_in_world - p.xyz_in_world;
  if (!radial.allFinite() || radial.norm() < 1e-6 || !has_plane_basis_) return reference;
  radial.normalize();
  const double wrapped =
    std::atan2(radial.dot(phase_quarter_axis_), radial.dot(phase_zero_axis_));
  return unwrap_near(wrapped, reference);
}

// void SmallTarget::get_target(
//   const std::optional<PowerRune> & p, std::chrono::steady_clock::time_point & timestamp)
// {
//   static int lost_cn = 0;
//   static std::chrono::steady_clock::time_point start_timestamp = timestamp;
//   auto time_gap = tools::delta_time(timestamp, start_timestamp);

//   // 1. 处理视觉丢帧（触发盲推或彻底丢失）
//   if (!p.has_value()) {
//     lost_cn++;
//     // 如果丢失超过容忍阈值(6帧)或系统尚未初始化，放弃解算
//     if (lost_cn > 5 || first_in_) {
//       unsolvable_ = true;
//       first_in_ = true;
//       // tools::logger()->debug("[Target] 小符丢失过久，停止盲推");
//     } else {
//       // 短暂丢失，触发盲推机制
//       predict(time_gap - lasttime_); // 纯推演卡尔曼先验状态，不引入测量更新
//       lasttime_ = time_gap;          // 更新时间戳，供下次计算 dt 使用
//       unsolvable_ = false;           // 保持可解算状态，让 Aimer 继续下发指令
//       tools::logger()->debug("[Target] 小符盲推中，连续丢失帧数: {}", lost_cn);
//     }
//     return; // 盲推结束，直接返回
//   }

//   // 2. 正常识别到目标，重置丢失计数
//   lost_cn = 0;

//   // 3. 冷启动初始化
//   if (first_in_) {
//     unsolvable_ = true;
//     init(time_gap, p.value());
//     first_in_ = false;
//   }

//   // 4. 正常卡尔曼滤波测量更新
//   unsolvable_ = false;
//   update(time_gap, p.value());

//   // 5. 处理滤波器发散
//   if (std::abs(ekf_.x[6]) > SMALL_W + CV_PI / 18 || std::abs(ekf_.x[6]) < SMALL_W - CV_PI / 18) {
//     unsolvable_ = true;
//     tools::logger()->debug("[Target] 小符角度发散spd: {:.2f}", ekf_.x[6] * 180 / CV_PI);
//     first_in_ = true;
//     return;
//   }
// }

void SmallTarget::get_target(
  const std::optional<PowerRune> & p, std::chrono::steady_clock::time_point & timestamp)
{
  if (
    p.has_value() && has_measurement_timestamp_ &&
    tools::delta_time(timestamp, last_measurement_timestamp_) > BUFF_TRACK_RETENTION_S) {
    reset();
  }
  const double time_gap = relative_time(timestamp);

  if (!p.has_value()) {
    predict_without_measurement(timestamp);
    return;
  }

  auto obs = p.value();
  if (first_in_ && obs.pose_quality != BuffPoseQuality::FULL_8_POINT) {
    unsolvable_ = true;
    return;
  }

  // init
  if (first_in_) {
    init(time_gap, obs);
    first_in_ = false;
    last_track_id_ = obs.track_id;
    record_measurement(obs, timestamp);
    readiness_ = has_stable_small_prediction_direction() ? TargetReadiness::PREDICTING
                                                         : TargetReadiness::TRACKING;
    unsolvable_ = false;
    return;
  }

  std::string reset_reason;
  bool reset_track = false;
  if (obs.track_id >= 0 && last_track_id_ >= 0 && obs.track_id != last_track_id_) {
    if (obs.pose_quality != BuffPoseQuality::FULL_8_POINT) {
      tools::logger()->debug("[Target] 小符拒绝部分观测换轨");
      predict_without_measurement(timestamp);
      return;
    }

    const int old_track_id = last_track_id_;
    update_plane_basis(obs, false);
    const double switched_phase = measure_phase(obs, ekf_.x[5]);
    ekf_.x[5] = switched_phase;
    ekf_.P.row(5).setZero();
    ekf_.P.col(5).setZero();
    ekf_.P(5, 5) = 0.01;
    phase_direction_.rebase(switched_phase, true);
    lasttime_ = time_gap;
    last_track_id_ = obs.track_id;
    readiness_ = has_stable_small_prediction_direction() ? TargetReadiness::PREDICTING
                                                         : TargetReadiness::TRACKING;
    unsolvable_ = false;
    record_measurement(obs, timestamp);
    tools::logger()->debug(
      "[Target] 小符切换跟踪 {}->{}, 保留中心状态", old_track_id, obs.track_id);
    return;
  } else {
    const double observed_phase = measure_phase(obs, ekf_.x[5]);
    const double quality_scale = std::sqrt(std::max(1.0, obs.measurement_noise_scale));
    const double phase_error = std::abs(observed_phase - ekf_.x[5]);
    if (phase_error > CV_PI / 9.0 * quality_scale) {
      const bool consistent_recovery =
        has_pending_phase_recovery_ &&
        std::abs(observed_phase - pending_phase_recovery_) <= CV_PI / 12.0;
      pending_phase_recovery_hits_ = consistent_recovery ? pending_phase_recovery_hits_ + 1 : 1;
      pending_phase_recovery_ = observed_phase;
      has_pending_phase_recovery_ = true;
      if (pending_phase_recovery_hits_ >= 2) {
        ekf_.x[5] = observed_phase;
        ekf_.P.row(5).setZero();
        ekf_.P.col(5).setZero();
        ekf_.P(5, 5) = 0.02;
        phase_direction_.rebase(observed_phase, true);
        lasttime_ = time_gap;
        last_track_id_ = obs.track_id;
        record_measurement(obs, timestamp);
        readiness_ = has_stable_small_prediction_direction() ? TargetReadiness::PREDICTING
                                                             : TargetReadiness::TRACKING;
        unsolvable_ = false;
        has_pending_phase_recovery_ = false;
        pending_phase_recovery_hits_ = 0;
        tools::logger()->debug("[Target] 小符连续观测重锚相位");
        return;
      }
      reset_reason = fmt::format("phase_err {:.1f}deg", phase_error * 57.3);
      reset_track = true;
    }
  }
  if (reset_track) {
    tools::logger()->debug("[Target] 小符拒绝异常观测: {}", reset_reason);
    predict_without_measurement(timestamp);
    return;
  }
  innovation_reject_count_ = 0;
  has_pending_phase_recovery_ = false;
  pending_phase_recovery_hits_ = 0;

  // kalman update
  unsolvable_ = false;
  update(time_gap, obs);
  last_track_id_ = obs.track_id;
  record_measurement(obs, timestamp);

  // 处理发散
  if (
    std::abs(ekf_.x[6]) > 1e-6 &&
    (std::abs(ekf_.x[6]) > SMALL_W + CV_PI / 18 ||
     std::abs(ekf_.x[6]) < SMALL_W - CV_PI / 18)) {
    unsolvable_ = true;
    readiness_ = TargetReadiness::LOST;
    tools::logger()->debug("[Target] 小符角度发散spd: {:.2f}", ekf_.x[6] * 180 / CV_PI);
    first_in_ = true;
    return;
  }

  readiness_ = has_stable_small_prediction_direction() ? TargetReadiness::PREDICTING
                                                       : TargetReadiness::TRACKING;
  unsolvable_ = false;
}

void SmallTarget::predict(double dt)
{
  dt = std::max(0.0, dt);
  ekf_.x[6] = small_prediction_roll_direction() * SMALL_W;

  // R中心在世界系中近似静止。保留一个强阻尼yaw速度，只用于吸收短时平台平移和外参误差。
  const double center_velocity_decay = std::exp(-dt / 0.10);
  // clang-format off
  A_ << 1.0,  dt, 0.0, 0.0, 0.0, 0.0, 0.0, // R_yaw
        0.0, center_velocity_decay, 0.0, 0.0, 0.0, 0.0, 0.0, // R_v_yaw
        0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, // R_pitch
        0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, // R_dis
        0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, // yaw
        0.0, 0.0, 0.0, 0.0, 0.0, 1.0,  dt, // roll
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0; // spd

  // 小符角速度固定；中心、平面朝向只允许缓慢变化。
  const double v1 = 0.0002;
  auto a = dt * dt * dt * dt / 4;
  auto b = dt * dt * dt / 2;
  auto c = dt * dt;
  Q_ << a * v1 + 1e-6, b * v1, 0.0, 0.0, 0.0, 0.0, 0.0,
        b * v1, c * v1, 0.0, 0.0, 0.0, 0.0, 0.0,
           0.0,    0.0, 1e-6, 0.0, 0.0, 0.0, 0.0,
           0.0,    0.0, 0.0, 1e-3, 0.0, 0.0, 0.0,
           0.0,    0.0, 0.0, 0.0, 1e-7, 0.0, 0.0,
           0.0,    0.0, 0.0, 0.0, 0.0, 1e-6, 0.0,
           0.0,    0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
  // clang-format on 
  auto f = [&](const Eigen::VectorXd & x) -> Eigen::VectorXd {
    Eigen::VectorXd x_prior = A_ * x;
    x_prior[0] = tools::limit_rad(x_prior[0]);
    x_prior[2] = tools::limit_rad(x_prior[2]);
    x_prior[4] = tools::limit_rad(x_prior[4]);
    return x_prior;
  };
  ekf_.predict(A_, Q_, f);
  ekf_.x[6] = small_prediction_roll_direction() * SMALL_W;
}

void SmallTarget::init(double nowtime, const PowerRune & p)
{
  // 初始化内部变量
  lasttime_ = nowtime;

  // 初始状态协方差矩阵
  x0_.resize(7);
  P0_.resize(7, 7);
  A_.resize(7, 7);
  Q_.resize(7, 7);
  H_.resize(7, 7);//z x
  R_.resize(7, 7);//z z
  // [R_yaw]
  // [v_R_yaw]
  // [R_pitch]
  // [R_dis]
  // [yaw]
  // [angle/row]
  // [spd]   w=CV_PI/6

  update_plane_basis(p, !has_plane_basis_);
  const double phase_reference = ekf_.x.size() >= 6 ? ekf_.x[5] : 0.0;
  const double observed_phase = measure_phase(p, phase_reference);
  const double plane_yaw = std::atan2(plane_normal_.y(), plane_normal_.x());

  // clang-format off
  // 初始状态
  x0_ << p.ypd_in_world[0], 0.0, p.ypd_in_world[1], p.ypd_in_world[2],
         plane_yaw, observed_phase,
         SMALL_W * small_prediction_roll_direction();
  // 初始状态协方差矩阵
  P0_ << 0.01,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,
          0.0, 0.01,  0.0,  0.0,  0.0,  0.0,  0.0,
          0.0,  0.0, 0.01,  0.0,  0.0,  0.0,  0.0,
          0.0,  0.0,  0.0, 0.25,  0.0,  0.0,  0.0,
          0.0,  0.0,  0.0,  0.0, 0.04,  0.0,  0.0,
          0.0,  0.0,  0.0,  0.0,  0.0, 0.04,  0.0,
          0.0,  0.0,  0.0,  0.0,  0.0,  0.0, 1e-4;
  // 状态转移矩阵
  // A_ 
  // 过程噪声协方差矩阵                            //// 调整
  // Q_ 
  // 测量方程矩阵
  // H_
  // 测量噪声协方差矩阵                            //// 调整
  // R_

  // clang-format on

  // 防止夹角求和出现异常值
  auto x_add = [](const Eigen::VectorXd & a, const Eigen::VectorXd & b) -> Eigen::VectorXd {
    Eigen::VectorXd c = a + b;
    c[0] = tools::limit_rad(c[0]);
    c[2] = tools::limit_rad(c[2]);
    c[4] = tools::limit_rad(c[4]);
    return c;
  };
  // 创建扩展卡尔曼滤波器对象
  ekf_ = tools::ExtendedKalmanFilter(x0_, P0_, x_add);
  phase_direction_.rebase(observed_phase, false);
}

void SmallTarget::update(double nowtime, const PowerRune & p)
{
  // [R_yaw]     angle0
  // [v_R_yaw]
  // [R_pitch]   angle2
  // [R_dis]
  // [yaw]       angle4
  // [angle/row] angle5
  // [spd]   w=CV_PI/6
  const Eigen::VectorXd & R_ypd = p.ypd_in_world;  // R
  const double basis_phase_shift = update_plane_basis(p, false);
  phase_direction_.shift_reference(basis_phase_shift);
  double observed_phase = measure_phase(p, ekf_.x[5]);
  if (p.pose_quality == BuffPoseQuality::FULL_8_POINT) phase_direction_.update(observed_phase);

  ekf_.x[6] = SMALL_W * small_prediction_roll_direction();

  // 预测下一个状态
  predict(nowtime - lasttime_);
  observed_phase = unwrap_near(observed_phase, ekf_.x[5]);
  const double plane_yaw = std::atan2(plane_normal_.y(), plane_normal_.x());

  auto z_subtract = [](const Eigen::VectorXd & a, const Eigen::VectorXd & b) -> Eigen::VectorXd {
    Eigen::VectorXd c = a - b;
    c[0] = tools::limit_rad(c[0]);
    c[1] = tools::limit_rad(c[1]);
    if (c.size() == 5) c[3] = tools::limit_rad(c[3]);
    c[c.size() - 1] = tools::limit_rad(c[c.size() - 1]);
    return c;
  };

  const bool use_plane_measurement = p.pose_quality == BuffPoseQuality::FULL_8_POINT;
  double center_scale = p.measurement_noise_scale;
  const double phase_scale = center_scale * phase_noise_multiplier(p);
  if (use_plane_measurement) {
    Eigen::MatrixXd H{
      {1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
      {0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0},
      {0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0},
      {0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0},
      {0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0}};
    Eigen::MatrixXd R = Eigen::MatrixXd::Zero(5, 5);
    R.diagonal() << 0.0001 * center_scale, 0.0001 * center_scale, 0.04 * center_scale,
      0.01 * center_scale, 0.0001 * phase_scale;
    Eigen::VectorXd z{{R_ypd[0], R_ypd[1], R_ypd[2], plane_yaw, observed_phase}};
    ekf_.update(z, H, R, z_subtract);
  } else {
    Eigen::MatrixXd H{
      {1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
      {0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0},
      {0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0},
      {0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0}};
    Eigen::MatrixXd R = Eigen::MatrixXd::Zero(4, 4);
    R.diagonal() << 0.0001 * center_scale, 0.0001 * center_scale, 0.04 * center_scale,
      0.0001 * phase_scale;
    Eigen::VectorXd z{{R_ypd[0], R_ypd[1], R_ypd[2], observed_phase}};
    ekf_.update(z, H, R, z_subtract);
  }

  ekf_.x[6] = SMALL_W * small_prediction_roll_direction();

  // 更新lasttime
  lasttime_ = nowtime;
  return;
}

int SmallTarget::small_prediction_roll_direction() const
{
  return small_buff_direction(phase_direction_.direction());
}

bool SmallTarget::has_stable_small_prediction_direction() const
{
  return configured_small_buff_direction() != 0 || phase_direction_.ready();
}

/// BigTarget

BigTarget::BigTarget() : Target(), spd_fitter_(100, 0.25, 1.884, 2.000) {}

void BigTarget::reset()
{
  Target::reset();
  phase_direction_.reset();
  clear_speed_samples(true);
  pause_speed_samples_until_ = 0.0;
  speed_model_direction_ = 0;
  has_speed_center_ = false;
}

void BigTarget::clear_speed_samples(bool clear_fitter)
{
  phase_samples_.clear();
  last_fitter_sample_time_ = -1.0;
  if (!clear_fitter) return;

  accepted_speed_samples_.clear();
  spd_fitter_.clear();
  fit_spd_ = 1.1775;
  fit_blend_ = 0.0;
  last_accepted_speed_time_ = 0.0;
}

std::optional<double> BigTarget::estimate_window_speed() const
{
  if (phase_samples_.size() < 3) return std::nullopt;
  if (phase_samples_.back().time - phase_samples_.front().time < BUFF_BIG_SPEED_MIN_SPAN_S) {
    return std::nullopt;
  }

  double weight_sum = 0.0;
  double mean_time = 0.0;
  double mean_phase = 0.0;
  for (const auto & sample : phase_samples_) {
    weight_sum += sample.weight;
    mean_time += sample.weight * sample.time;
    mean_phase += sample.weight * sample.phase;
  }
  if (weight_sum < 1e-9) return std::nullopt;
  mean_time /= weight_sum;
  mean_phase /= weight_sum;

  double numerator = 0.0;
  double denominator = 0.0;
  for (const auto & sample : phase_samples_) {
    const double centered_time = sample.time - mean_time;
    numerator += sample.weight * centered_time * (sample.phase - mean_phase);
    denominator += sample.weight * centered_time * centered_time;
  }
  if (denominator < 1e-9) return std::nullopt;
  return numerator / denominator;
}

void BigTarget::add_speed_sample(double nowtime, double observed_phase, const PowerRune & p)
{
  const bool valid_quality =
    p.pose_quality == BuffPoseQuality::FULL_8_POINT &&
    p.center_source == RuneCenterSource::DETECTED && p.reprojection_error <= 4.5 &&
    p.prediction_error <= 8.0 / 57.3 && nowtime >= pause_speed_samples_until_;
  if (!valid_quality) {
    phase_samples_.clear();
    return;
  }

  const int direction = phase_direction_.direction();
  if (speed_model_direction_ != 0 && direction != 0 && speed_model_direction_ != direction) {
    clear_speed_samples(true);
  }
  if (direction != 0) speed_model_direction_ = direction;

  if (has_speed_center_ && (p.xyz_in_world - last_speed_center_).norm() > 0.15) {
    clear_speed_samples(true);
  }
  last_speed_center_ = p.xyz_in_world;
  has_speed_center_ = true;

  if (!phase_samples_.empty() && nowtime - phase_samples_.back().time > 0.3) {
    clear_speed_samples(true);
  }

  phase_samples_.push_back(
    {nowtime, observed_phase, 1.0 / std::max(1.0, p.measurement_noise_scale)});
  while (static_cast<int>(phase_samples_.size()) > BUFF_BIG_SPEED_PHASE_WINDOW) {
    phase_samples_.pop_front();
  }

  const auto estimated_signed_speed = estimate_window_speed();
  if (!estimated_signed_speed.has_value()) return;
  if (direction != 0 && estimated_signed_speed.value() * direction <= 0.0) return;

  const double speed = std::abs(estimated_signed_speed.value());
  if (speed < 0.35 || speed > 2.25) return;

  if (accepted_speed_samples_.size() >= 3) {
    const std::vector<double> samples(accepted_speed_samples_.begin(), accepted_speed_samples_.end());
    const double sample_median = median(samples);
    std::vector<double> deviations;
    deviations.reserve(samples.size());
    for (const double value : samples) deviations.push_back(std::abs(value - sample_median));
    const double mad = median(std::move(deviations));
    const double limit = std::max(0.25, 3.0 * 1.4826 * mad);
    if (std::abs(speed - sample_median) > limit) return;
  }

  const double speed_dt =
    last_accepted_speed_time_ > 0.0 ? nowtime - last_accepted_speed_time_ : 0.12;
  const double alpha = 1.0 - std::exp(-std::max(speed_dt, 0.0) / 0.12);
  fit_spd_ = std::clamp((1.0 - alpha) * fit_spd_ + alpha * speed, 0.35, 2.25);
  last_accepted_speed_time_ = nowtime;

  accepted_speed_samples_.push_back(speed);
  while (accepted_speed_samples_.size() > 7) accepted_speed_samples_.pop_front();

  if (last_fitter_sample_time_ < 0.0 || nowtime - last_fitter_sample_time_ >= 0.015) {
    spd_fitter_.add_data(nowtime, speed);
    spd_fitter_.fit();
    last_fitter_sample_time_ = nowtime;
  }
}

// void BigTarget::get_target(
//   const std::optional<PowerRune> & p, std::chrono::steady_clock::time_point & timestamp)
// {
//   static int lost_cn = 0;
//   static std::chrono::steady_clock::time_point start_timestamp = timestamp;
//   auto time_gap = tools::delta_time(timestamp, start_timestamp);

//   // 1. 处理视觉丢帧（触发盲推或彻底丢失）
//   if (!p.has_value()) {
//     lost_cn++;
//     // 如果丢失超过容忍阈值(6帧)或系统尚未初始化，放弃解算
//     if (lost_cn > 60 || first_in_) {
//       unsolvable_ = true;
//       first_in_ = true;
//       // tools::logger()->debug("[Target] 大符丢失过久，停止盲推");
//     } else {
//       // 短暂丢失，触发盲推机制
//       predict(time_gap - lasttime_); // 纯推演卡尔曼先验状态
//       lasttime_ = time_gap;          // 更新时间戳
//       unsolvable_ = false;           // 保持可解算状态
//       tools::logger()->debug("[Target] 大符盲推中，连续丢失帧数: {}", lost_cn);
//     }
//     return; // 盲推结束，直接返回
//   }

//   // 2. 正常识别到目标，重置丢失计数
//   lost_cn = 0;

//   // 3. 冷启动初始化
//   if (first_in_) {
//     unsolvable_ = true;
//     init(time_gap, p.value());
//     first_in_ = false;
//   }

//   // 4. 正常卡尔曼滤波测量更新
//   unsolvable_ = false;
//   update(time_gap, p.value());

//   // 5. 处理滤波器发散
//   if (
//     ekf_.x[7] > 1.045 * 1.5 || ekf_.x[7] < 0.78 / 1.5 || ekf_.x[8] > 2.0 * 1.5 ||
//     ekf_.x[8] < 1.884 / 1.5) {
//     unsolvable_ = true;
//     tools::logger()->debug("[Target] 大符角度发散a: {:.2f}b:{:.2f}", ekf_.x[7], ekf_.x[8]);
//     first_in_ = true;
//     return;
//   }
// }

void BigTarget::get_target(
  const std::optional<PowerRune> & p, std::chrono::steady_clock::time_point & timestamp)
{
  if (
    p.has_value() && has_measurement_timestamp_ &&
    tools::delta_time(timestamp, last_measurement_timestamp_) > BUFF_TRACK_RETENTION_S) {
    reset();
  }
  const double time_gap = relative_time(timestamp);
  if (!p.has_value()) {
    predict_without_measurement(timestamp);
    return;
  }

  auto obs = p.value();
  if (first_in_ && obs.pose_quality != BuffPoseQuality::FULL_8_POINT) {
    unsolvable_ = true;
    return;
  }
  if (first_in_) {
    unsolvable_ = true;
    init(time_gap, obs);
    first_in_ = false;
    last_track_id_ = obs.track_id;
    record_measurement(obs, timestamp);
    readiness_ = TargetReadiness::TRACKING;
    unsolvable_ = false;
    return;
  }

  if (obs.track_id >= 0 && last_track_id_ >= 0 && obs.track_id != last_track_id_) {
    if (obs.pose_quality != BuffPoseQuality::FULL_8_POINT) {
      tools::logger()->debug("[Target] 大符拒绝部分观测换轨");
      predict_without_measurement(timestamp);
      return;
    }

    const int old_track_id = last_track_id_;
    update_plane_basis(obs, false);
    const double switched_phase = measure_phase(obs, ekf_.x[5]);
    const bool legal_slot_switch =
      obs.slot_offset != 0 && std::abs(obs.slot_offset) <= 2 &&
      obs.slot_residual <= 12.0 / 57.3;
    ekf_.x[5] = switched_phase;
    ekf_.P.row(5).setZero();
    ekf_.P.col(5).setZero();
    ekf_.P(5, 5) = 0.01;
    phase_direction_.rebase(switched_phase, true);
    clear_speed_samples(!legal_slot_switch);
    pause_speed_samples_until_ = time_gap + 0.1;
    lasttime_ = time_gap;
    last_track_id_ = obs.track_id;
    readiness_ = phase_direction_.ready() ? TargetReadiness::PREDICTING
                                         : TargetReadiness::TRACKING;
    unsolvable_ = false;
    record_measurement(obs, timestamp);
    tools::logger()->debug(
      "[Target] 大符切换跟踪 {}->{}, 保留中心和平面状态", old_track_id, obs.track_id);
    return;
  }

  const double observed_phase = measure_phase(obs, ekf_.x[5]);
  const double quality_scale = std::sqrt(std::max(1.0, obs.measurement_noise_scale));
  const double phase_error = std::abs(observed_phase - ekf_.x[5]);
  if (phase_error > CV_PI / 6.0 * quality_scale) {
    const bool consistent_recovery =
      has_pending_phase_recovery_ &&
      std::abs(observed_phase - pending_phase_recovery_) <= CV_PI / 10.0;
    pending_phase_recovery_hits_ = consistent_recovery ? pending_phase_recovery_hits_ + 1 : 1;
    pending_phase_recovery_ = observed_phase;
    has_pending_phase_recovery_ = true;
    if (pending_phase_recovery_hits_ >= 2) {
      ekf_.x[5] = observed_phase;
      ekf_.P.row(5).setZero();
      ekf_.P.col(5).setZero();
      ekf_.P(5, 5) = 0.02;
      phase_direction_.rebase(observed_phase, true);
      lasttime_ = time_gap;
      last_track_id_ = obs.track_id;
      record_measurement(obs, timestamp);
      readiness_ = phase_direction_.ready() ? TargetReadiness::PREDICTING
                                           : TargetReadiness::TRACKING;
      unsolvable_ = false;
      has_pending_phase_recovery_ = false;
      pending_phase_recovery_hits_ = 0;
      tools::logger()->debug("[Target] 大符连续观测重锚相位");
      return;
    }
    tools::logger()->debug(
      "[Target] 大符拒绝异常连续相位: {:.1f}deg", phase_error * 57.3);
    predict_without_measurement(timestamp);
    return;
  }

  has_pending_phase_recovery_ = false;
  pending_phase_recovery_hits_ = 0;
  update(time_gap, obs);
  last_track_id_ = obs.track_id;
  record_measurement(obs, timestamp);
  readiness_ = phase_direction_.ready() ? TargetReadiness::PREDICTING
                                       : TargetReadiness::TRACKING;
  unsolvable_ = false;
}

void BigTarget::predict(double dt)
{
  dt = std::max(0.0, dt);
  const bool fit_ready = spd_fitter_.ready(
    60, BUFF_BIG_FIT_MIN_SPAN_S, BUFF_BIG_FIT_MIN_INLIER_RATIO, BUFF_BIG_FIT_MAX_RMS);
  const auto fit = spd_fitter_.best_result_;
  const double t = lasttime_ + dt;
  const int direction = phase_direction_.direction();
  double speed_magnitude = std::clamp(fit_spd_, 0.0, 2.1);
  double phase_delta = speed_magnitude * dt;
  if (fit_ready && fit.omega > 1e-6) {
    const double fitted_speed = std::clamp(
      spd_fitter_.sine_function(t, fit.A, fit.omega, fit.phi, fit.C), 0.0, 2.1);
    const double fitted_phase_delta =
      -fit.A / fit.omega * std::cos(fit.omega * t + fit.phi) +
      fit.A / fit.omega * std::cos(fit.omega * lasttime_ + fit.phi) + fit.C * dt;
    speed_magnitude = (1.0 - fit_blend_) * speed_magnitude + fit_blend_ * fitted_speed;
    phase_delta = (1.0 - fit_blend_) * phase_delta + fit_blend_ * fitted_phase_delta;
  }

  A_.setIdentity(10, 10);
  A_(0, 1) = dt;
  A_(5, 6) = dt;
  Q_.setZero(10, 10);
  Q_(0, 0) = 1e-6;
  Q_(1, 1) = 1e-5;
  Q_(2, 2) = 1e-6;
  Q_(3, 3) = 1e-3;
  Q_(4, 4) = 1e-7;
  Q_(5, 5) = 1e-4;
  Q_(6, 6) = 0.02;

  auto f = [&](const Eigen::VectorXd & x) -> Eigen::VectorXd {
    Eigen::VectorXd x_prior = x;
    x_prior[0] = tools::limit_rad(x_prior[0] + dt * x_prior[1]);
    x_prior[2] = tools::limit_rad(x_prior[2]);
    x_prior[4] = tools::limit_rad(x_prior[4]);
    x_prior[5] += direction * phase_delta;
    x_prior[6] = direction * speed_magnitude;
    if (fit_ready) {
      x_prior[7] = fit.A;
      x_prior[8] = fit.omega;
      x_prior[9] = tools::limit_rad(fit.phi);
    }
    return x_prior;
  };
  ekf_.predict(A_, Q_, f);
  lasttime_ = t;
}

void BigTarget::init(double nowtime, const PowerRune & p)
{
  // 初始化内部变量
  lasttime_ = nowtime;
  unsolvable_ = true;
  fit_spd_ = 1.1775;
  fit_blend_ = 0.0;
  last_accepted_speed_time_ = 0.0;
  last_fitter_sample_time_ = -1.0;
  pause_speed_samples_until_ = 0.0;
  speed_model_direction_ = 0;
  has_speed_center_ = false;
  clear_speed_samples(true);
  update_plane_basis(p, !has_plane_basis_);
  const double phase_reference = ekf_.x.size() >= 6 ? ekf_.x[5] : 0.0;
  const double observed_phase = measure_phase(p, phase_reference);
  const double plane_yaw = std::atan2(plane_normal_.y(), plane_normal_.x());

  // 初始状态协方差矩阵
  x0_.resize(10);
  P0_.resize(10, 10);
  A_.resize(10, 10);
  Q_.resize(10, 10);
  H_.resize(5, 10);
  R_.resize(5, 5);

  // [R_yaw]
  // [v_R_yaw]
  // [R_pitch]
  // [R_dis]
  // [yaw]
  // [angle/row]
  // [spd]       角速度 a*sin(wt) + 2.09 - a
  // [a]         0.78-1.045
  // [w]         1.884-2.000
  // [fi]

  // clang-format off
  x0_ << p.ypd_in_world[0], 0.0, p.ypd_in_world[1], p.ypd_in_world[2],
         plane_yaw, observed_phase, 0.0, 0.9125, 1.942, 0.0;
  P0_.setZero();
  P0_.diagonal() << 0.01, 0.01, 0.01, 0.25, 0.04, 0.04, 0.25, 0.1, 0.1, 0.5;
  // clang-format on

  // 防止夹角求和出现异常值
  auto x_add = [](const Eigen::VectorXd & a, const Eigen::VectorXd & b) -> Eigen::VectorXd {
    Eigen::VectorXd c = a + b;
    c[0] = tools::limit_rad(c[0]);
    c[2] = tools::limit_rad(c[2]);
    c[4] = tools::limit_rad(c[4]);
    c[9] = tools::limit_rad(c[9]);
    return c;
  };
  // 创建扩展卡尔曼滤波器对象
  ekf_ = tools::ExtendedKalmanFilter(x0_, P0_, x_add);
  phase_direction_.rebase(observed_phase, false);
}

void BigTarget::update(double nowtime, const PowerRune & p)
{
  // [R_yaw]
  // [v_R_yaw]
  // [R_pitch]
  // [R_dis]
  // [yaw]
  // [angle/row] 角度
  // [spd]       角速度 a*sin(wt) + 2.09 - a
  // [a]         0.78-1.045
  // [w]         1.884-2.000
  // [fi]
  const Eigen::VectorXd & R_ypd = p.ypd_in_world;  // R
  const double basis_phase_shift = update_plane_basis(p, false);
  phase_direction_.shift_reference(basis_phase_shift);
  for (auto & sample : phase_samples_) sample.phase += basis_phase_shift;
  double observed_phase = measure_phase(p, ekf_.x[5]);
  if (p.pose_quality == BuffPoseQuality::FULL_8_POINT) phase_direction_.update(observed_phase);
  add_speed_sample(nowtime, observed_phase, p);

  const double state_dt = nowtime - lasttime_;
  const bool fit_ready = spd_fitter_.ready(
    60, BUFF_BIG_FIT_MIN_SPAN_S, BUFF_BIG_FIT_MIN_INLIER_RATIO, BUFF_BIG_FIT_MAX_RMS);
  if (fit_ready) {
    fit_blend_ = std::min(
      1.0, fit_blend_ + std::max(state_dt, 0.0) / std::max(BUFF_BIG_FIT_BLEND_S, 1e-3));
  } else {
    fit_blend_ = 0.0;
  }
  predict(state_dt);
  observed_phase = unwrap_near(observed_phase, ekf_.x[5]);
  const double plane_yaw = std::atan2(plane_normal_.y(), plane_normal_.x());

  auto z_subtract = [](const Eigen::VectorXd & a, const Eigen::VectorXd & b) -> Eigen::VectorXd {
    Eigen::VectorXd c = a - b;
    c[0] = tools::limit_rad(c[0]);
    c[1] = tools::limit_rad(c[1]);
    if (c.size() == 5) c[3] = tools::limit_rad(c[3]);
    c[c.size() - 1] = tools::limit_rad(c[c.size() - 1]);
    return c;
  };

  const bool use_plane_measurement = p.pose_quality == BuffPoseQuality::FULL_8_POINT;
  double center_scale = p.measurement_noise_scale;
  const double phase_scale = center_scale * phase_noise_multiplier(p);
  if (use_plane_measurement) {
    Eigen::MatrixXd H{
      {1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
      {0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
      {0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
      {0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0},
      {0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0}};
    Eigen::MatrixXd R = Eigen::MatrixXd::Zero(5, 5);
    R.diagonal() << 0.0001 * center_scale, 0.0001 * center_scale, 0.04 * center_scale,
      0.01 * center_scale, 0.0004 * phase_scale;
    Eigen::VectorXd z{{R_ypd[0], R_ypd[1], R_ypd[2], plane_yaw, observed_phase}};
    ekf_.update(z, H, R, z_subtract);
  } else {
    Eigen::MatrixXd H{
      {1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
      {0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
      {0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
      {0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0}};
    Eigen::MatrixXd R = Eigen::MatrixXd::Zero(4, 4);
    R.diagonal() << 0.0001 * center_scale, 0.0001 * center_scale, 0.04 * center_scale,
      0.0004 * phase_scale;
    Eigen::VectorXd z{{R_ypd[0], R_ypd[1], R_ypd[2], observed_phase}};
    ekf_.update(z, H, R, z_subtract);
  }

  if (fit_ready) {
    const auto & fit = spd_fitter_.best_result_;
    ekf_.x[7] = fit.A;
    ekf_.x[8] = fit.omega;
    ekf_.x[9] = tools::limit_rad(fit.phi);
  }
  spd = ekf_.x[6];

  lasttime_ = nowtime;
  unsolvable_ = false;
}
}  // namespace auto_buff
