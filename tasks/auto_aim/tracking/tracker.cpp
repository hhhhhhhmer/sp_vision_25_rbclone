#include "tracker.hpp"

#include <yaml-cpp/yaml.h>

#include <tuple>
#include <cmath>
#include <limits>
#include <stdexcept>

#include "enemy_color_policy.hpp"
#include "../geometry/solver.hpp"
#include "tools/logger.hpp"
#include "tools/fft.hpp"
#include "tools/math_tools.hpp"


namespace auto_aim
{
namespace
{
/** @brief 计算装甲板归一化中心到图像中心的距离 @param armor 装甲板 @return 中心距离；坐标无效时返回正无穷 */
double normalized_center_distance(const Armor & armor)
{
  if (!std::isfinite(armor.center_norm.x) || !std::isfinite(armor.center_norm.y) ||
      armor.center_norm.x < 0.0F || armor.center_norm.y < 0.0F) {
    return std::numeric_limits<double>::infinity();
  }
  return cv::norm(armor.center_norm - cv::Point2f(0.5F, 0.5F));
}

/** @brief 按装甲板到图像中心的距离原地排序 @param armors 装甲板列表 */
void sort_by_image_center(std::list<Armor> & armors)
{
  armors.sort([](const Armor & a, const Armor & b) {
    return normalized_center_distance(a) < normalized_center_distance(b);
  });
}

template <typename T>
T optional_value(const YAML::Node & yaml, const char * key, const T & fallback)
{
  return yaml[key] ? yaml[key].as<T>() : fallback;
}

CenterAccelerationEstimatorConfig read_center_acceleration_config(const YAML::Node & yaml)
{
  CenterAccelerationEstimatorConfig config;
  config.enabled = optional_value(yaml, "center_accel_ff_enabled", config.enabled);
  config.window_seconds = optional_value(yaml, "center_accel_window_s", config.window_seconds);
  config.min_samples = optional_value(yaml, "center_accel_min_samples", config.min_samples);
  config.min_span_seconds =
    optional_value(yaml, "center_accel_min_span_s", config.min_span_seconds);
  config.ema_alpha = optional_value(yaml, "center_accel_ema_alpha", config.ema_alpha);
  config.max_acceleration = optional_value(yaml, "center_accel_max_mps2", config.max_acceleration);
  config.max_jerk = optional_value(yaml, "center_accel_max_jerk_mps3", config.max_jerk);
  config.max_fit_rmse = optional_value(yaml, "center_accel_max_fit_rmse_m", config.max_fit_rmse);
  config.stale_timeout_seconds =
    optional_value(yaml, "center_accel_stale_timeout_s", config.stale_timeout_seconds);
  return config;
}
}  // namespace

Tracker::Tracker(const std::string & config_path, Solver * solver)
: Tracker(config_path, static_cast<IArmorPoseSolver *>(solver))
{
}

Tracker::Tracker(const std::string & config_path, IArmorPoseSolver * solver)
: solver_{solver},
  detect_count_(0),
  temp_lost_count_(0),
  state_{"lost"},
  pre_state_{"lost"},
  last_timestamp_(std::chrono::steady_clock::now()),
  omni_target_priority_{ArmorPriority::fifth}
{
  if (solver_ == nullptr) throw std::invalid_argument("Tracker requires a non-null Solver");
  auto yaml = YAML::LoadFile(config_path);
  enemy_color_str_ = yaml["enemy_color"].as<std::string>();
  enemy_color_ = (enemy_color_str_ == "red") ? Color::red : Color::blue;
  min_detect_count_ = yaml["min_detect_count"].as<int>();
  max_temp_lost_count_ = yaml["max_temp_lost_count"].as<int>();
  outpost_max_temp_lost_count_ = yaml["outpost_max_temp_lost_count"].as<int>();
  normal_temp_lost_count_ = max_temp_lost_count_;
  center_acceleration_estimator_ =
    CenterAccelerationEstimator(read_center_acceleration_config(yaml));

  // 多装甲板融合：默认关闭，保持单装甲板行为；开启后同一帧可融合多块匹配装甲板
  multi_armor_fusion_ = optional_value(yaml, "multi_armor_fusion", false);
  multi_armor_gate_ = optional_value(yaml, "multi_armor_gate", 20.0);
  multi_armor_angle_sigma_scale_ =
    optional_value(yaml, "multi_armor_angle_sigma_scale", 10.0);
  // 观测噪声标准差覆盖（rad / m / rad）；默认沿用 EKF 内部启发式
  meas_azimuth_sigma_ = optional_value(yaml, "meas_azimuth_sigma", -1.0);
  meas_distance_sigma_ = optional_value(yaml, "meas_distance_sigma", -1.0);
  meas_angle_sigma_ = optional_value(yaml, "meas_angle_sigma", -1.0);
  load_uv_config(yaml);

  last_cam_is_short = true;
}

void Tracker::apply_measurement_sigmas()
{
  target_.set_measurement_sigmas(
    meas_azimuth_sigma_, meas_distance_sigma_, meas_angle_sigma_);
}

void Tracker::load_uv_config(const YAML::Node & yaml)
{
  uv_config_.enabled = optional_value(yaml, "uv_observation", false);
  uv_config_.sigma_px = optional_value(yaml, "uv_sigma_px", uv_config_.sigma_px);
  uv_config_.sigma_len_ratio =
    optional_value(yaml, "uv_sigma_len_ratio", uv_config_.sigma_len_ratio);
  uv_config_.sigma_attitude_deg =
    optional_value(yaml, "uv_sigma_attitude_deg", uv_config_.sigma_attitude_deg);
  uv_config_.attitude_common_ratio =
    optional_value(yaml, "uv_attitude_common_ratio", uv_config_.attitude_common_ratio);
  uv_config_.sigma_angle = optional_value(yaml, "uv_sigma_angle", uv_config_.sigma_angle);
  uv_config_.associate_gate_px =
    optional_value(yaml, "uv_associate_gate_px", uv_config_.associate_gate_px);
  uv_config_.facing_cos_min = optional_value(yaml, "uv_facing_cos_min", uv_config_.facing_cos_min);
  uv_config_.radius_prior_sigma =
    optional_value(yaml, "uv_radius_prior_sigma", uv_config_.radius_prior_sigma);
  uv_config_.process_noise_z_scale =
    optional_value(yaml, "uv_process_noise_z_scale", uv_config_.process_noise_z_scale);
  uv_radius_prior_sigma_ = uv_config_.radius_prior_sigma;
}

void Tracker::set_uv_enabled(bool enabled)
{
  if (uv_config_.enabled == enabled) return;
  uv_config_.enabled = enabled;
  uv_fallback_warned_ = false;
  reset();
  tools::logger()->info("[Tracker] observation mode -> {}", enabled ? "UV(pixel)" : "YPD(world)");
}

void Tracker::apply_uv_config()
{
  if (!uv_config_.enabled) return;
  // 每帧从当前求解器取相机几何：长短焦切换时 solver_ 会换成另一台相机的求解器
  const auto geometry = solver_->camera_geometry();
  target_.set_uv_config(uv_config_);
  target_.set_camera_geometry(geometry);
  if (!geometry.valid && !uv_fallback_warned_) {
    // 静默回退会让"以为在跑 UV"的 A/B 结论失真，因此至少警告一次
    uv_fallback_warned_ = true;
    tools::logger()->warn(
      "[Tracker] uv_observation=true 但当前求解器未提供有效相机几何（内参/畸变/外参），"
      "已回退传统 ypd 观测");
  }
}

void Tracker::setSolver(Solver * solver)
{
  setPoseSolver(solver);
}

void Tracker::setPoseSolver(IArmorPoseSolver * solver)
{
  if (solver == nullptr) throw std::invalid_argument("Tracker requires a non-null Solver");
  solver_ = solver;
}

std::string Tracker::state() const { return state_; }

void Tracker::reset()
{
  detect_count_ = 0;
  temp_lost_count_ = 0;
  state_ = "lost";
  pre_state_ = "lost";
  last_timestamp_ = std::chrono::steady_clock::now();
  last_mode_.reset();
  center_acceleration_estimator_.reset();
}

void Tracker::set_fft(tools::FFTExample * fft)
{
  fft_ = fft;
  reset_fft_sample_state();
}

void Tracker::reset_fft_sample_state()
{
  if (fft_) fft_->reset();
}

void Tracker::update_fft_sample(
  const Armor & armor, std::chrono::steady_clock::time_point t)
{
  if (!fft_) return;
  fft_->add_sample(t, target_.last_id, armor.xyz_in_world.z());
}

void Tracker::update_fft_sample_from_filter(std::chrono::steady_clock::time_point t)
{
  if (!fft_) return;
  const int id = target_.last_id;
  if (id < 0) return;
  // UV 观测模式下逐帧不跑 PnP，armor.xyz_in_world 不会被写入（恒为 0），
  // 若照旧传 armor，FFT 会拿到全 0 的 z 序列、周期性 z 前馈被静默关闭
  fft_->add_sample(t, id, target_.h_armor_xyz(target_.ekf_x(), id).z());
}

void Tracker::update_camera_mode(bool cam_is_short)
{
  if (cam_is_short != last_cam_is_short) center_acceleration_estimator_.reset();
  target_.cam_is_short = cam_is_short;
  last_cam_is_short = cam_is_short;
}

bool Tracker::use_center_acceleration() const
{
  return target_.name != ArmorName::base && target_.name != ArmorName::outpost;
}

std::list<Target> Tracker::sb_track(
  std::list<Armor> & armors, std::chrono::steady_clock::time_point t,bool cam_is_short, bool use_enemy_color)
{
  auto dt = tools::delta_time(t, last_timestamp_);
  last_timestamp_ = t;

  // TODO
  if(gimbal_ == nullptr) {
    tools::logger()->error("[Tracker] gimbal_不能为空指针，请先调用set_gimbal()设置云台指针");
    return {};
  }
  io::GimbalState g = gimbal_->state();
  if (enemy_color_str_ == "auto") {
    enemy_color_ = enemy_color_from_gimbal(EnemyColorPolicy::Sentry, g.enemy_color);
  }

  update_camera_mode(cam_is_short);

  // 时间间隔过长，说明可能发生了相机离线
  if (state_ != "lost" && dt > 0.1) {
    tools::logger()->warn("[Tracker] Large dt: {:.3f}s", dt);
    state_ = "lost";
  }
  // 过滤掉非我方装甲板
  filter_enemy_armors(armors, enemy_color_, use_enemy_color);

  // 过滤前哨站顶部装甲板
  // armors.remove_if([this](const auto_aim::Armor & a) {
  //   return a.name == ArmorName::outpost &&
  //          solver_.oupost_reprojection_error(a, 27.5 * CV_PI / 180.0) <
  //            solver_.oupost_reprojection_error(a, -15 * CV_PI / 180.0);
  // });

  // 优先选择靠近图像中心的装甲板
  sort_by_image_center(armors);

  // 按优先级排序，优先级最高在首位(优先级越高数字越小，1的优先级最高)
  armors.sort(
    [](const auto_aim::Armor & a, const auto_aim::Armor & b) { return a.priority < b.priority; });

  bool found;
  if (state_ == "lost") {
    found = set_target(armors, t);
  }

  else {
    found = update_target(armors, t);
  }

  state_machine(found);

  // 发散检测
  if (state_ != "lost" && target_.diverged()) {
    tools::logger()->debug("[Tracker] Target diverged!");
    state_ = "lost";
    return {};
  }

  if (
  std::accumulate(
    target_.ekf().recent_nis_failures.begin(), target_.ekf().recent_nis_failures.end(), 0) >=
  (0.4 * target_.ekf().window_size)) {
      tools::logger()->debug("[Target] Bad Converge Found!");
      state_ = "lost";
      return {};
  }

  if (state_ == "lost") return {};

  std::list<Target> targets = {target_};
  return targets;
}

std::list<Target> Tracker::track(
  std::list<Armor> & armors, std::chrono::steady_clock::time_point t, bool cam_is_short, bool use_enemy_color)
{
  auto dt = tools::delta_time(t, last_timestamp_);
  last_timestamp_ = t;
  if(gimbal_ == nullptr) {
    tools::logger()->error("[Tracker] gimbal_不能为空指针，请先调用set_gimbal()设置云台指针");
    return {};
  }
  io::GimbalState g = gimbal_->state();
  if (enemy_color_str_ == "auto") {
    enemy_color_ = enemy_color_from_gimbal(EnemyColorPolicy::Standard, g.enemy_color);
  }

  update_camera_mode(cam_is_short);

  // 时间间隔过长，说明可能发生了相机离线
  if (state_ != "lost" && dt > 0.1) {
    tools::logger()->warn("[Tracker] Large dt: {:.3f}s", dt);
    state_ = "lost";
  }
  // 过滤掉非我方装甲板
  filter_enemy_armors(armors, enemy_color_, use_enemy_color);

  // 过滤前哨站顶部装甲板
  // armors.remove_if([this](const auto_aim::Armor & a) {
  //   return a.name == ArmorName::outpost &&
  //          solver_.oupost_reprojection_error(a, 27.5 * CV_PI / 180.0) <
  //            solver_.oupost_reprojection_error(a, -15 * CV_PI / 180.0);
  // });

  // 优先选择靠近图像中心的装甲板
  sort_by_image_center(armors);

  // 按优先级排序，优先级最高在首位(优先级越高数字越小，1的优先级最高)
  // armors.sort(
  //   [](const auto_aim::Armor & a, const auto_aim::Armor & b) { return a.priority < b.priority; });

  bool found = 0;

  if (!last_mode_.has_value()) last_mode_ = g.mode;
  bool mode_switch_0to1 = (*last_mode_ == 0 && g.mode == 1);
  //按下右键时，mouse为1则跟随上一次的目标，不按则瞄准最近的装甲板
  if(!mode_switch_0to1)
  {
    if (state_ == "lost") {
        found = set_target(armors, t);
        // tools::logger()->debug("按下右键，只选择正在跟踪的装甲板，跳过其他兵种，直至丢跟踪，初始化跟踪类型为 {}", ARMOR_NAMES[armors.front().name]);
    }
    else {
      found = update_target(armors, t);
    }
  }else {
    if (state_ == "lost") {
        found = set_target(armors, t);
    }
    else if (armors.empty()) {
      found = update_target(armors, t);
    }
    else {
      if(target_.name == armors.front().name 
        && target_.armor_type == armors.front().type)
      {
        found = update_target(armors, t);
      }else{
        found = set_target(armors, t);
        state_ = "detecting";
        detect_count_ = 1;
        // tools::logger()->debug("不按右键，默认选中离图像中心最近的兵种，切换至： {}, 跟踪器重置, 置信度:{:.3f}", ARMOR_NAMES[armors.front().name],armors.front().confidence );
      }

     
    }
  }
  last_mode_ = g.mode;
  // found = set_target(armors, t);

  state_machine(found);

  // 发散检测
  if (state_ != "lost" && target_.diverged()) {
    // tools::logger()->debug("[Tracker] Target diverged!");
    state_ = "lost";
    return {};
  }

  if (
  std::accumulate(
    target_.ekf().recent_nis_failures.begin(), target_.ekf().recent_nis_failures.end(), 0) >=
  (0.4 * target_.ekf().window_size)) {
      tools::logger()->debug("[Target] Bad Converge Found!");
      state_ = "lost";
      return {};
  }

  if (state_ == "lost") return {};

  

  std::list<Target> targets = {target_};
  return targets;
}


std::list<Target> Tracker::test_track(
  std::list<Armor> & armors, std::chrono::steady_clock::time_point t, bool cam_is_short, bool use_enemy_color)
{
  auto dt = tools::delta_time(t, last_timestamp_);
  last_timestamp_ = t;
  
 
  update_camera_mode(cam_is_short);

  // 时间间隔过长，说明可能发生了相机离线
  if (state_ != "lost" && dt > 0.1) {
    tools::logger()->warn("[Tracker] Large dt: {:.3f}s", dt);
    state_ = "lost";
  }
  // 过滤掉非我方装甲板
  filter_enemy_armors(armors, enemy_color_, use_enemy_color);

  // 过滤前哨站顶部装甲板
  // armors.remove_if([this](const auto_aim::Armor & a) {
  //   return a.name == ArmorName::outpost &&
  //          solver_.oupost_reprojection_error(a, 27.5 * CV_PI / 180.0) <
  //            solver_.oupost_reprojection_error(a, -15 * CV_PI / 180.0);
  // });

  // 优先选择靠近图像中心的装甲板
  sort_by_image_center(armors);

  // 按优先级排序，优先级最高在首位(优先级越高数字越小，1的优先级最高)
  // armors.sort(
  //   [](const auto_aim::Armor & a, const auto_aim::Armor & b) { return a.priority < b.priority; });

  bool found = 0;
  
  if (state_ == "lost") {
    found = set_target(armors, t);
    // tools::logger()->debug("按下右键，只选择正在跟踪的装甲板，跳过其他兵种，直至丢跟踪，初始化跟踪类型为 {}", ARMOR_NAMES[armors.front().name]);
  }
  else {
    found = update_target(armors, t);
  }
  
  // found = set_target(armors, t);

  state_machine(found);

  // 发散检测
  if (state_ != "lost" && target_.diverged()) {
    // tools::logger()->debug("[Tracker] Target diverged!");
    state_ = "lost";
    return {};
  }

  if (
  std::accumulate(
    target_.ekf().recent_nis_failures.begin(), target_.ekf().recent_nis_failures.end(), 0) >=
  (0.4 * target_.ekf().window_size)) {
      tools::logger()->debug("[Target] Bad Converge Found!");
      state_ = "lost";
      return {};
  }

  if (state_ == "lost") return {};

  

  std::list<Target> targets = {target_};
  return targets;
}

std::tuple<omniperception::DetectionResult, std::list<Target>> Tracker::track(
  const std::vector<omniperception::DetectionResult> & detection_queue, std::list<Armor> & armors,
  std::chrono::steady_clock::time_point t, bool use_enemy_color)
{
  omniperception::DetectionResult switch_target{std::list<Armor>(), t, 0, 0};
  omniperception::DetectionResult temp_target{std::list<Armor>(), t, 0, 0};
  if (!detection_queue.empty()) {
    temp_target = detection_queue.front();
  }

  auto dt = tools::delta_time(t, last_timestamp_);
  last_timestamp_ = t;

  filter_enemy_armors(armors, enemy_color_, use_enemy_color);

  // 时间间隔过长，说明可能发生了相机离线
  if (state_ != "lost" && dt > 0.1) {
    tools::logger()->warn("[Tracker] Large dt: {:.3f}s", dt);
    state_ = "lost";
  }

  // 优先选择靠近图像中心的装甲板
  sort_by_image_center(armors);

  // 按优先级排序，优先级最高在首位(优先级越高数字越小，1的优先级最高)
  armors.sort([](const Armor & a, const Armor & b) { return a.priority < b.priority; });

  bool found;
  if (state_ == "lost") {
    found = set_target(armors, t);
  }

  // 此时主相机画面中出现了优先级更高的装甲板，切换目标
  else if (state_ == "tracking" && !armors.empty() && armors.front().priority < target_.priority) {
    found = set_target(armors, t);
    tools::logger()->debug("auto_aim switch target to {}", ARMOR_NAMES[armors.front().name]);
  }

  // 此时全向感知相机画面中出现了优先级更高的装甲板，切换目标
  else if (
    state_ == "tracking" && !temp_target.armors.empty() &&
    temp_target.armors.front().priority < target_.priority && target_.convergened()) {
    state_ = "switching";
    switch_target = omniperception::DetectionResult{
      temp_target.armors, t, temp_target.delta_yaw, temp_target.delta_pitch};
    omni_target_priority_ = temp_target.armors.front().priority;
    found = false;
    tools::logger()->debug("omniperception find higher priority target");
  }

  else if (state_ == "switching") {
    found = !armors.empty() && armors.front().priority == omni_target_priority_;
  }

  else if (state_ == "detecting" && pre_state_ == "switching") {
    found = set_target(armors, t);
  }

  else {
    found = update_target(armors, t);
  }

  pre_state_ = state_;
  // 更新状态机
  state_machine(found);

  // 发散检测
  if (state_ != "lost" && target_.diverged()) {
    tools::logger()->debug("[Tracker] Target diverged!");
    state_ = "lost";
    return {switch_target, {}};  // 返回switch_target和空的targets
  }

  if (state_ == "lost") return {switch_target, {}};  // 返回switch_target和空的targets

  std::list<Target> targets = {target_};
  return {switch_target, targets};
}

void Tracker::state_machine(bool found)
{
  if (state_ == "lost") {
    if (!found) return;

    state_ = "detecting";
    detect_count_ = 1;
  }

  else if (state_ == "detecting") {
    if (found) {
      detect_count_++;
      if (detect_count_ >= min_detect_count_) state_ = "tracking";
    } else {
      detect_count_ = 0;
      state_ = "lost";
    }
  }

  else if (state_ == "tracking") {
    if (found) return;

    temp_lost_count_ = 1;
    state_ = "temp_lost";
  }

  else if (state_ == "switching") {
    if (found) {
      state_ = "detecting";
    } else {
      temp_lost_count_++;
      if (temp_lost_count_ > 200) state_ = "lost";
    }
  }

  else if (state_ == "temp_lost") {
    if (found) {
      state_ = "tracking";
    } else {
      temp_lost_count_++;
      if (target_.name == ArmorName::outpost)
        //前哨站的temp_lost_count需要设置的大一些
        max_temp_lost_count_ = outpost_max_temp_lost_count_;
      else
        max_temp_lost_count_ = normal_temp_lost_count_;

      if (temp_lost_count_ > max_temp_lost_count_) state_ = "lost";
    }
  }
}

bool Tracker::set_target(std::list<Armor> & armors, std::chrono::steady_clock::time_point t)
{
  last_update_count_ = 0;
  if (armors.empty()) return false;

  const bool cam_is_short = target_.cam_is_short;
  auto & armor = armors.front();
  if (!solver_->try_solve(armor)) return false;

  // 根据兵种优化初始化参数
  auto is_balance = (armor.type == ArmorType::big) &&
                    (armor.name == ArmorName::three || armor.name == ArmorName::four ||
                     armor.name == ArmorName::five);

  double radius = 0.2;
  int armor_num = 4;
  Eigen::VectorXd P0_dig{{1, 64, 1, 64, 1, 64, 0.4, 100, 1, 1, 1}};

  if (is_balance) {
    P0_dig << 1, 64, 1, 64, 1, 64, 0.4, 100, 1, 1, 1;
    radius = 0.2;
    armor_num = 2;
  }

  else if (armor.name == ArmorName::outpost) {
    P0_dig << 1, 64, 1, 64, 1, 81, 0.4, 100, 1e-4, 0, 0;
    radius = 0.2765;
    armor_num = 3;
  }

  else if (armor.name == ArmorName::base) {
    P0_dig << 1, 64, 1, 64, 1, 64, 0.4, 100, 1e-4, 0, 0;
    radius = 0.3205;
    armor_num = 3;
  }

  // 单装甲退化保护：UV 模式下中心深度与半径强耦合，收紧半径先验
  if (uv_config_.enabled && uv_radius_prior_sigma_ > 0.0) {
    P0_dig[8] = uv_radius_prior_sigma_ * uv_radius_prior_sigma_;
  }

  target_ = Target(armor, t, radius, armor_num, P0_dig);

  target_.cam_is_short = cam_is_short;
  last_cam_is_short = cam_is_short;

  center_acceleration_estimator_.reset();
  if (use_center_acceleration()) {
    const Eigen::VectorXd state = target_.ekf_x();
    center_acceleration_estimator_.add_sample(t, {state[0], state[2]});
  }

  reset_fft_sample_state();
  update_fft_sample(armor, t);
  apply_measurement_sigmas();
  return true;
}
bool Tracker::update_target(std::list<Armor> & armors, std::chrono::steady_clock::time_point t)
{
  if(fft_ != nullptr ){
    auto wave = fft_->get_wave();
    target_.wave_ = wave.valid()
              ? std::make_optional(std::move(wave))
              : std::nullopt;

  }
  Eigen::VectorXd acceleration = Eigen::VectorXd::Zero(3);
  if (use_center_acceleration()) {
    acceleration.head<2>() = center_acceleration_estimator_.acceleration(t);
  }
  // XY acceleration is intentionally limited to this online frame-to-frame prediction.
  target_.predict(t, acceleration);

  bool found = false;
  last_update_count_ = 0;

  // UV 模式：把本帧所有匹配装甲板关联后做一次联合更新（内部为批量 EKF）
  // 若求解器不提供相机几何（例如测试用的桩求解器），自动回退到传统世界系观测
  if (uv_config_.enabled) {
    apply_uv_config();
  }
  if (target_.uv_ready()) {
    // 前哨站/基地：每帧只更新一块装甲板（与老路径一致）
    // 若允许帧内顺序更新多块板，last_id 会在帧内来回跳，下一帧立刻触发"换板"，
    // 前哨站高度锚点的采样窗口被压到 1~2 帧，锚点会学成 0.1×真实高度、
    // 高度阶梯方向翻转，瞄准点随之偏 0.2~0.4m（见 Target::kTowerArmorMinAnchorSamples）
    if (target_.name == ArmorName::outpost || target_.name == ArmorName::base) {
      for (auto & armor : armors) {
        if (armor.name != target_.name || armor.type != target_.armor_type) continue;
        // 第一块（最靠近图像中心）不设门限，保持与原行为一致
        if (!target_.update(armor, -1.0, {})) continue;
        found = true;
        last_update_count_ = 1;
        if (use_center_acceleration()) {
          const Eigen::VectorXd state = target_.ekf_x();
          center_acceleration_estimator_.add_sample(t, {state[0], state[2]});
        }
        update_fft_sample_from_filter(t);
        break;
      }
      return found;
    }

    const int max_armors = multi_armor_fusion_ ? 99 : 1;
    found = target_.update_batch(armors, multi_armor_gate_, max_armors);
    if (found) {
      last_update_count_ = target_.last_batch_observation_count();
      if (use_center_acceleration()) {
        const Eigen::VectorXd state = target_.ekf_x();
        center_acceleration_estimator_.add_sample(t, {state[0], state[2]});
      }
      // FFT 采样用当前主装甲板（UV 模式装甲板未经 PnP，取滤波器估计高度）
      update_fft_sample_from_filter(t);
    }
    return found;
  }

  int matched = 0;
  // 本帧已经被使用的装甲板 ID，防止两块不同的装甲板被匹配到同一个 ID 上
  std::vector<int> used_ids;
  // 前哨站/基地使用各自的几何匹配逻辑，暂不参与多装甲板融合
  const bool multi_ok = multi_armor_fusion_ && target_.name != ArmorName::outpost &&
                        target_.name != ArmorName::base;

  // 由于 armors 在 track/sb_track 中已经按距离图像中心的远近排序
  // 遍历找到的第一个匹配目标的装甲板，即为视野中最居中、畸变最小的装甲板
  // 多装甲板融合开启后，继续把同一帧的其它匹配装甲板也送入滤波器：
  // 第一块装甲板不设门限（保持原有行为），后续装甲板必须通过马氏距离门限且 ID 不重复，
  // 以避免把另一台车的同名装甲板或误检融合进当前目标
  for (auto & armor : armors) {
    if (armor.name != target_.name || armor.type != target_.armor_type) continue;

    // 已经匹配到主装甲板且本目标不参与融合时直接结束：必须在 try_solve 之前判断，
    // 否则会白白多跑一次 PnP + 140 步 yaw 搜索（约 100~200us/帧）
    if (matched > 0 && !multi_ok) break;

    if (!solver_->try_solve(armor)) continue;

    const double gate = (matched == 0) ? -1.0 : multi_armor_gate_;
    // 第二块及之后装甲板的 PnP 朝向常因搜索窗口/镜像解而不可靠，按系数降权
    const double angle_scale = (matched == 0) ? 1.0 : multi_armor_angle_sigma_scale_;
    // update 返回 false 说明观测未匹配到装甲板（或被门限/已用ID拒绝）、EKF 未执行校正，
    // 滤波器状态没有变化，不能算作跟踪成功，继续尝试后续装甲板
    if (!target_.update(armor, gate, used_ids, angle_scale)) continue;

    matched++;
    last_update_count_ = matched;
    found = true;
    used_ids.push_back(target_.last_id);

    // 中心加速度前馈与 FFT 采样每帧只记录一次，用最先匹配（最居中）的装甲板
    if (matched == 1) {
      if (use_center_acceleration()) {
        const Eigen::VectorXd state = target_.ekf_x();
        center_acceleration_estimator_.add_sample(t, {state[0], state[2]});
      }
      update_fft_sample(armor, t);
    }
  }

  return found;
}

}  // namespace auto_aim
