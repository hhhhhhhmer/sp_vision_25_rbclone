#include "planner.hpp"

#include <vector>
#include <stdexcept>

#include "tools/math_tools.hpp"
#include "tools/trajectory.hpp"
#include "tools/yaml.hpp"

using namespace std::chrono_literals;

namespace auto_aim
{
Planner::Planner(const std::string & config_path) : config_path_(config_path)
{
  auto yaml = tools::load(config_path);
  yaw_offset_ = tools::read<double>(yaml, "yaw_offset") / 57.3;
  pitch_offset_ = tools::read<double>(yaml, "pitch_offset") / 57.3;
  far_pitch_offset_ = tools::read<double>(yaml, "far_pitch_offset") / 57.3;
  far_high_pitch_offset_ = tools::read<double>(yaml, "far_high_pitch_offset") / 57.3;
  target_dist_error_ = tools::read<double>(yaml, "target_dist_error");
  target_h_error_ = tools::read<double>(yaml, "target_h_error");
  fire_thresh_ = tools::read<double>(yaml, "fire_thresh");
  decision_speed_ = tools::read<double>(yaml, "decision_speed");
  high_speed_delay_time_ = tools::read<double>(yaml, "high_speed_delay_time");
  low_speed_delay_time_ = tools::read<double>(yaml, "low_speed_delay_time");
  small_armor_tolerance = tools::read<double>(yaml, "small_armor_tolerance");
  big_armor_tolerance = tools::read<double>(yaml, "big_armor_tolerance");
  tower_and_base_armor_tolerance_ = tools::read<double>(yaml, "tower_and_base_armor_tolerance_");
  gimbal_control_delay = tools::read<double>(yaml, "gimbal_control_delay");
  tower_pitch_prediction_time_ = tools::read<double>(yaml, "tower_pitch_prediction_time");
  gimbal_delay_ = tools::read<double>(yaml, "gimbal_delay");
  shoot_offset_ = tools::read<int>(yaml, "shoot_offset");
  if (!valid_shoot_offset(shoot_offset_)) {
    throw std::invalid_argument("shoot_offset must keep the firing index inside the MPC horizon");
  }

  // ===== 方案 C：云台电机响应模型（可选，缺失时使用默认值；标定后填入 yaml）=====
  const auto opt = [&yaml](const char * key, double fallback) {
    return yaml[key] ? yaml[key].as<double>() : fallback;
  };
  // 校验后再 sync()：T_cl<=0 会让 a>1 导致落地检查发散（若容差开着就永久禁射），
  // tau<0 或 NaN 同样无意义；非法值一律回退到默认并告警，不静默使用
  const auto load_axis_model = [&](GimbalAxisModel & model, const char * tau_key,
                                   const char * T_key, double tau_default,
                                   double T_default) {
    model.tau_s = opt(tau_key, tau_default);
    model.T_cl_s = opt(T_key, T_default);
    if (!std::isfinite(model.tau_s) || model.tau_s < 0) {
      tools::logger()->warn(
        "[Planner] {} = {} 非法(需 >= 0)，回退为 {}s", tau_key, model.tau_s, tau_default);
      model.tau_s = tau_default;
    }
    if (!std::isfinite(model.T_cl_s) || model.T_cl_s <= 0) {
      tools::logger()->warn(
        "[Planner] {} = {} 非法(需 > 0)，回退为 {}s", T_key, model.T_cl_s, T_default);
      model.T_cl_s = T_default;
    }
    model.sync();
  };
  load_axis_model(yaw_axis_model_, "gimbal_yaw_tau_s", "gimbal_yaw_T_cl_s", 0.005, 0.012);
  load_axis_model(pitch_axis_model_, "gimbal_pitch_tau_s", "gimbal_pitch_T_cl_s", 0.005, 0.015);
  fire_landing_tolerance_ = opt("fire_landing_tolerance_deg", -1.0) / 57.3;
  if (yaml["gimbal_command_mode"]) {
    gimbal_command_mode_step_ = yaml["gimbal_command_mode"].as<std::string>() == "trajectory_step";
  }
  // 离散模型的实际等效延时 = k_d·DT（纯死区，已被 DT 量化）+ DT·a/(1-a)（一阶收敛）
  const auto eff_delay_ms = [](const GimbalAxisModel & m) {
    const double first_order = (m.a < 1.0) ? DT * m.a / (1.0 - m.a) : 0.0;
    return (m.k_d * DT + first_order) * 1e3;
  };
  tools::logger()->info(
    "[Planner] Gimbal axis model: yaw tau={:.1f}ms T_cl={:.1f}ms k_d={} eff={:.1f}ms | "
    "pitch tau={:.1f}ms T_cl={:.1f}ms k_d={} eff={:.1f}ms | landing tolerance={:.2f}deg | mode={}",
    yaw_axis_model_.tau_s * 1e3, yaw_axis_model_.T_cl_s * 1e3, yaw_axis_model_.k_d,
    eff_delay_ms(yaw_axis_model_), pitch_axis_model_.tau_s * 1e3,
    pitch_axis_model_.T_cl_s * 1e3, pitch_axis_model_.k_d, eff_delay_ms(pitch_axis_model_),
    fire_landing_tolerance_ * 57.3,
    gimbal_command_mode_step_ ? "trajectory_step" : "fire_aim");

  setup_yaw_solver(config_path);
  setup_pitch_solver(config_path);
}

Planner::Planner(const Planner & other) : Planner(other.config_path_)
{
  debug_xyza = other.debug_xyza;
  aim_target_yaw = other.aim_target_yaw;
  is_far = other.is_far;
  is_high = other.is_high;
  last_selected_idx = other.last_selected_idx;
  last_selected_xyz = other.last_selected_xyz;
  outpost_z_stable_start_time_ = other.outpost_z_stable_start_time_;
  outpost_is_make = other.outpost_is_make;
}

Planner & Planner::operator=(const Planner & other)
{
  if (this == &other) return *this;
  Planner replacement(other);
  *this = std::move(replacement);
  return *this;
}

Plan Planner::plan(Target target, double bullet_speed, const GimbalFeedback & fb)
{
  if (target.armor_xyza_list().empty()) return {false};
  // std::cout<<target.getEKFXest()[0]<<std::endl;
  // std::cout<<target.getEKFXest()[0]<<std::endl;

  // std::cout<<"x:"<<target.getEKFXest()[0]<<std::endl;
  // std::cout<<"y:"<<target.getEKFXest()[2]<<std::endl;
  // std::cout<<"z:"<<target.getEKFXest()[4]<<std::endl;


  // 0. Check bullet speed
  if (bullet_speed < 10 || bullet_speed > 25) {
    bullet_speed = 22;
  }

  // 1. Predict fly_time
  Eigen::Vector3d xyz;
  auto min_dist = 1e10;
  for (auto & xyza : target.armor_xyza_list()) {
    auto dist = xyza.head<2>().norm();
    if (dist < min_dist) {
      min_dist = dist;
      xyz = xyza.head<3>();
    }
  }
  min_dist+=target_dist_error_;
  double target_h = xyz.z(); 
  target_h+= target_h_error_;
  auto bullet_traj = tools::Trajectory(bullet_speed, min_dist, target_h);

  // 2. Get trajectory
  double yaw0;
  Trajectory traj;
  try {
    yaw0 = aim(target, bullet_speed)(0);
    traj = get_trajectory(target, yaw0, bullet_speed);
  } catch (const std::exception & e) {
    tools::logger()->warn("Unsolvable target {:.2f}", bullet_speed);
    return {false};
  }

  // 3. Solve yaw (初始状态 = 云台实际状态，方案 C：让 MPC 知道云台现在在哪、能否追上)
  Eigen::VectorXd x0(2);
  x0 << tools::limit_rad(fb.yaw_deg / 57.3 - yaw0), fb.yaw_vel;
  tiny_set_x0(yaw_solver_.get(), x0);

  yaw_solver_->work->Xref = traj.block(0, 0, 2, HORIZON);
  tiny_solve(yaw_solver_.get());

  // 4. Solve pitch
  x0 << fb.pitch_deg / 57.3, fb.pitch_vel;
  tiny_set_x0(pitch_solver_.get(), x0);

  pitch_solver_->work->Xref = traj.block(2, 0, 2, HORIZON);
  tiny_solve(pitch_solver_.get());

  Plan plan{};
  plan.control = true;

  plan.target_yaw = tools::limit_rad(traj(0, HALF_HORIZON) + yaw0);

  plan.target_pitch = traj(2, HALF_HORIZON);

  write_mpc_commands(plan, yaw_solver_->work->x, yaw_solver_->work->u,
                     pitch_solver_->work->x, pitch_solver_->work->u, yaw0);

  // auto shoot_offset_ = 2;
  plan.fire =
    std::hypot(
      traj(0, HALF_HORIZON + shoot_offset_) - yaw_solver_->work->x(0, HALF_HORIZON + shoot_offset_),
      traj(2, HALF_HORIZON + shoot_offset_) -
        pitch_solver_->work->x(0, HALF_HORIZON + shoot_offset_)) < fire_thresh_;

  // 方案 C：云台电机响应落地检查（用标定模型模拟真实云台，验证开火时刻确实到位）
  if (fire_landing_tolerance_ > 0) {
    double yaw_err = 0, pitch_err = 0;
    gimbal_landing_check(
      traj, yaw_solver_->work->x, pitch_solver_->work->x, fb, yaw0, yaw_err, pitch_err);
    const bool landed = yaw_err < fire_landing_tolerance_ && pitch_err < fire_landing_tolerance_;
    if (plan.fire && !landed) {
      tools::logger()->debug(
        "[Planner] landing check blocked: yaw_err={:.3f}deg pitch_err={:.3f}deg (tol={:.2f}deg)",
        yaw_err * 57.3, pitch_err * 57.3, fire_landing_tolerance_ * 57.3);
    }
    plan.fire = plan.fire && landed;
  }
  return plan;
}


Plan Planner::sbplan(Target target, double bullet_speed, const GimbalFeedback & fb)
{
  const double gimbal_yaw = fb.yaw_deg;
  if (target.armor_xyza_list().empty()) return {false};
  // std::cout<<target.getEKFXest()[0]<<std::endl;
  // std::cout<<target.getEKFXest()[0]<<std::endl;

  // std::cout<<"x:"<<target.getEKFXest()[0]<<std::endl;
  // std::cout<<"y:"<<target.getEKFXest()[2]<<std::endl;
  // std::cout<<"z:"<<target.getEKFXest()[4]<<std::endl;


  // 0. Check bullet speed
  if (bullet_speed < 10 || bullet_speed > 25) {
    bullet_speed = 22;
  }

  // 1. Predict fly_time
  Eigen::Vector3d xyz;
  auto min_dist = 1e10;
  for (auto & xyza : target.armor_xyza_list()) {
    auto dist = xyza.head<2>().norm();
    if (dist < min_dist) {
      min_dist = dist;
      xyz = xyza.head<3>();
    }
  }
  min_dist+=target_dist_error_;
  double target_h = xyz.z(); 
  target_h+= target_h_error_;
  auto bullet_traj = tools::Trajectory(bullet_speed, min_dist, target_h);
  is_far = target_h > 1.0;
  
  target.predict(bullet_traj.fly_time);

  // tools::logger()->info("h:{}, xy_d:{}, xyz_d:{}, fly_time:{}, ", target_h, min_dist, xyz.norm(), bullet_traj.fly_time);

  // 2. Get trajectory
  double yaw0;
  Trajectory traj;
  Eigen::Vector2d yaw_pitch;
  try {
    yaw_pitch = rbaim(target, bullet_speed);
    yaw0 = yaw_pitch(0);
    traj = rbget_trajectory(target, yaw0, bullet_speed);
  } catch (const std::exception & e) {
    tools::logger()->warn("Unsolvable target {:.2f}", bullet_speed);
    return {false};
  }

  // 3. Solve yaw (初始状态 = 云台实际状态)
  Eigen::VectorXd x0(2);
  x0 << tools::limit_rad(fb.yaw_deg / 57.3 - yaw0), fb.yaw_vel;
  tiny_set_x0(yaw_solver_.get(), x0);

  yaw_solver_->work->Xref = traj.block(0, 0, 2, HORIZON);
  tiny_solve(yaw_solver_.get());

  // 4. Solve pitch
  x0 << fb.pitch_deg / 57.3, fb.pitch_vel;
  tiny_set_x0(pitch_solver_.get(), x0);

  pitch_solver_->work->Xref = traj.block(2, 0, 2, HORIZON);
  tiny_solve(pitch_solver_.get());

  Plan plan{};
  plan.control = true;
  //mubiaojaiiodu
  plan.target_yaw = tools::limit_rad(traj(0, HALF_HORIZON) + yaw0);
  
  plan.target_pitch = traj(2, HALF_HORIZON);

  write_mpc_commands(plan, yaw_solver_->work->x, yaw_solver_->work->u,
                     pitch_solver_->work->x, pitch_solver_->work->u, yaw0);

  

  // 前哨站迭代限制
  if (target.name == ArmorName::outpost) {
      double vz = target.ekf_x()(5); // 获取当前前哨站中心Z轴坐标
      // double delta_z = std::abs(current_z - outpost_z_baseline_);
      auto now = std::chrono::steady_clock::now();

      // 如果Z轴变化幅度大于指定阈值（例如0.05米），重置基准和计时器，并禁止开火
      if (vz > 0.09) { 
          outpost_z_stable_start_time_ = now;
          // suggest_fire = false; 
          outpost_is_make = false;
      } else {
        // 如果变化幅度在阈值内，判断持续时间是否达到 0.7 秒
        double stable_duration = tools::delta_time(now ,outpost_z_stable_start_time_ );
        if (
          // stable_duration < 1 || 
          target.update_count_ < 500) {
            // suggest_fire = false; // 持续时间不足 0.7s，不开火
            outpost_is_make = false;
        }
        else{
          outpost_is_make = true;
        }
      }

      if(!outpost_is_make){
        Eigen::Vector2d yaw_pitch_nan = heroaim(target, 100000, gimbal_yaw);
        plan.yaw = yaw_pitch_nan(0);
        plan.yaw_vel = 0;
        plan.yaw_acc = 0;

        plan.pitch = yaw_pitch_nan(1);
        plan.pitch_vel = 0;
        plan.pitch_acc = 0;
        plan.fire = 0;
        return  plan;
      }
  }

  // auto shoot_offset_ = 2;
  if(abs(tools::limit_rad((gimbal_yaw )/57.3 - yaw_offset_ - plan.target_yaw)) * 57.3 < 3){
    plan.fire =
      std::hypot(
        traj(0, HALF_HORIZON + shoot_offset_) - yaw_solver_->work->x(0, HALF_HORIZON + shoot_offset_),
        traj(2, HALF_HORIZON + shoot_offset_) -
          pitch_solver_->work->x(0, HALF_HORIZON + shoot_offset_)) < fire_thresh_;
  }else plan.fire = 0;

  // 方案 C：云台电机响应落地检查
  if (fire_landing_tolerance_ > 0) {
    double yaw_err = 0, pitch_err = 0;
    gimbal_landing_check(
      traj, yaw_solver_->work->x, pitch_solver_->work->x, fb, yaw0, yaw_err, pitch_err);
    const bool landed = yaw_err < fire_landing_tolerance_ && pitch_err < fire_landing_tolerance_;
    if (plan.fire && !landed) {
      tools::logger()->debug(
        "[Planner] landing check blocked: yaw_err={:.3f}deg pitch_err={:.3f}deg (tol={:.2f}deg)",
        yaw_err * 57.3, pitch_err * 57.3, fire_landing_tolerance_ * 57.3);
    }
    plan.fire = plan.fire && landed;
  }

  // target.predict(-gimbal_control_delay);
  // plan.fire = rbShoot(target, (gimbal_yaw )/57.3 - yaw_offset_);
  // tools::logger()->warn("fire:{}", plan.fire);
  // plan.target_yaw = (aim_target_yaw + yaw_offset_ )* 57.3;




  return plan;
}




bool Planner::rbShoot(Target target, double gimbal_yaw, bool tower_fixed_pitch){
  if (target.armor_xyza_list().empty()) return false;
  bool suggest_fire = 1;
    // auto x_est = target.getEKFXest();
    // double est_x =  x_est(0);
    // double est_y = x_est(2);
    // double est_yaw = x_est(6);


  Eigen::Vector4d target_armor_xyza;
  auto min_dist = 1e10;
  for (auto & xyza : target.armor_xyza_list()) {
    auto dist = xyza.head<2>().norm();
    if (dist < min_dist) {
      min_dist = dist;
      target_armor_xyza = xyza;
    }
  }

  double target_yaw = target_armor_xyza(3) ;

  aim_target_yaw = atan2(target_armor_xyza(1), target_armor_xyza(0));//+ 0.3/57.3;
  // feedback_yaw = gimbal_yaw;

  double shoot_range = target.armor_type == ArmorType::big ? big_armor_tolerance : small_armor_tolerance;

  if(target.name == ArmorName::base || target.name == ArmorName::outpost) shoot_range = tower_and_base_armor_tolerance_;

    // 打击范围计算

  // 左边缘（沿切线正方向偏移半宽）
  double left_x = target_armor_xyza(0) + 0.5 * shoot_range * (-sin(target_yaw));
  double left_y = target_armor_xyza(1) + 0.5 * shoot_range * cos(target_yaw);

  // 右边缘（沿切线负方向偏移半宽）
  double right_x = target_armor_xyza(0) - 0.5 * shoot_range * (-sin(target_yaw));
  double right_y = target_armor_xyza(1) - 0.5 * shoot_range * cos(target_yaw);


  // 目标中心方向
  double center_angle = atan2(target_armor_xyza(1), target_armor_xyza(0));
  // 当前云台偏差（相对于中心）
  double delta = tools::limit_rad(center_angle - gimbal_yaw);


  double left_angle = atan2(left_y, left_x);
  double right_angle = atan2(right_y, right_x);
  // 计算左右边缘相对于中心的角度偏移
  double d_left = tools::limit_rad(left_angle - center_angle);
  double d_right = tools::limit_rad(right_angle - center_angle);

  // 取较小的和较大的偏移（因为左右边缘距离中心不会超过 90°，所以 d_left 和 d_right 符号相反且绝对值 < π/2）
  double d_min = std::min(d_left, d_right);
  double d_max = std::max(d_left, d_right);



  // double ax = target_armor_xyza(0) - 0.5f * shoot_range * sin(target_yaw);
  // double ay = target_armor_xyza(1) + 0.5f * shoot_range * cos(target_yaw);
  // double bx = target_armor_xyza(0) + 0.5f * shoot_range * sin(target_yaw);
  // double by = target_armor_xyza(1) - 0.5f * shoot_range * cos(target_yaw);
  // double angle_a = atan2(ay, ax);
  // double angle_b = atan2(by, bx);
  // double angle_c = atan2(target_armor_xyza(1), target_armor_xyza(0));
  // // double allow_fire_ang_max = angle_c - angle_b;
  // // double allow_fire_ang_min = angle_c - angle_a;
  // double allow_fire_ang_max = std::max(angle_a, angle_b) - angle_c;
  // double allow_fire_ang_min = std::min(angle_a, angle_b) - angle_c;
  // allow_fire_ang_max = tools::limit_rad(allow_fire_ang_max);
  // allow_fire_ang_min = tools::limit_rad(allow_fire_ang_min);
  

  // pitch
  bool suggest_pitch = true;
  if(tower_fixed_pitch && abs(target.ekf_x()(4) - target_armor_xyza(2)) > 0.001){
    suggest_pitch = false;
  }

  // // yaw_ang_ref
  // double control_delta_angle =
  //     tools::limit_rad(atan2(target_armor_xyza(1), target_armor_xyza(0)) - gimbal_yaw );
  // suggest_fire = (control_delta_angle < allow_fire_ang_max &&
  //                 control_delta_angle > allow_fire_ang_min && suggest_pitch) ;

  // 判断 delta 是否在 [d_min, d_max] 范围内
  suggest_fire = (delta >= d_min && delta <= d_max) && suggest_pitch;



  if(!outpost_is_make && target.name == ArmorName::outpost) suggest_fire = 0;
  if(!suggest_fire){
    // tools::logger()->info("not fire! control_delta_angle: {},  allow_fire_ang_max: {}, allow_fire_ang_min: {}",
    //   control_delta_angle, allow_fire_ang_max, allow_fire_ang_min
    // );
    
  }
    
    return suggest_fire;
}

Plan Planner::rbplan(Target target, double bullet_speed, const GimbalFeedback & fb)
{
  const double gimbal_yaw = fb.yaw_deg;
  if (target.armor_xyza_list().empty()) return {false};
  // std::cout<<target.getEKFXest()[0]<<std::endl;
  // std::cout<<target.getEKFXest()[0]<<std::endl;

  // std::cout<<"x:"<<target.getEKFXest()[0]<<std::endl;
  // std::cout<<"y:"<<target.getEKFXest()[2]<<std::endl;
  // std::cout<<"z:"<<target.getEKFXest()[4]<<std::endl;


  // 0. Check bullet speed
  if (bullet_speed < 10 || bullet_speed > 25) {
    bullet_speed = 22;
  }

  // 1. Predict fly_time
  Eigen::Vector3d xyz;
  auto min_dist = 1e10;
  for (auto & xyza : target.armor_xyza_list()) {
    auto dist = xyza.head<2>().norm();
    if (dist < min_dist) {
      min_dist = dist;
      xyz = xyza.head<3>();
    }
  }
  min_dist+=target_dist_error_;
  double target_h = xyz.z(); 
  target_h+= target_h_error_;
  auto bullet_traj = tools::Trajectory(bullet_speed, min_dist, target_h);
  
  target.predict(bullet_traj.fly_time);
  is_far = min_dist > 5.0;
  is_high = target_h > 1.3;

  tools::logger()->info("h:{}, xy_d:{}, xyz_d:{}, fly_time:{}, ", target_h, min_dist, xyz.norm(), bullet_traj.fly_time);

  // 2. Get trajectory
  double yaw0;
  Trajectory traj;
  Eigen::Vector2d yaw_pitch;
  try {
    yaw_pitch = aim(target, bullet_speed);
    yaw0 = yaw_pitch(0);
    traj = rbget_trajectory(target, yaw0, bullet_speed);
  } catch (const std::exception & e) {
    tools::logger()->warn("Unsolvable target {:.2f}", bullet_speed);
    return {false};
  }

  // 3. Solve yaw (初始状态 = 云台实际状态)
  Eigen::VectorXd x0(2);
  x0 << tools::limit_rad(gimbal_yaw / 57.3 - yaw0), fb.yaw_vel;
  tiny_set_x0(yaw_solver_.get(), x0);

  yaw_solver_->work->Xref = traj.block(0, 0, 2, HORIZON);
  tiny_solve(yaw_solver_.get());

  // 4. Solve pitch
  x0 << fb.pitch_deg / 57.3, fb.pitch_vel;
  tiny_set_x0(pitch_solver_.get(), x0);

  pitch_solver_->work->Xref = traj.block(2, 0, 2, HORIZON);
  tiny_solve(pitch_solver_.get());

  Plan plan{};
  plan.control = true;
  //mubiaojaiiodu
  // plan.target_yaw = tools::limit_rad(traj(0, HALF_HORIZON) + yaw0);
  
  plan.target_pitch = traj(2, HALF_HORIZON);

  write_mpc_commands(plan, yaw_solver_->work->x, yaw_solver_->work->u,
                     pitch_solver_->work->x, pitch_solver_->work->u, yaw0);

  

  // 前哨站迭代限制
  if (target.name == ArmorName::outpost) {
      double vz = target.ekf_x()(5); // 获取当前前哨站中心Z轴坐标
      // double delta_z = std::abs(current_z - outpost_z_baseline_);
      auto now = std::chrono::steady_clock::now();

      // 如果Z轴变化幅度大于指定阈值（例如0.05米），重置基准和计时器，并禁止开火
      // if (vz > 0.09) { 
      //     outpost_z_stable_start_time_ = now;
      //     // suggest_fire = false; 
      //     outpost_is_make = false;
      // } else {
        // 如果变化幅度在阈值内，判断持续时间是否达到 0.7 秒
        double stable_duration = tools::delta_time(now ,outpost_z_stable_start_time_ );
        if (
          // stable_duration < 1 || 
          target.update_count_ < 500) {
            // suggest_fire = false; // 持续时间不足 0.7s，不开火
            outpost_is_make = false;
        }
        else{
          outpost_is_make = true;
        }
      // }

      if(!outpost_is_make){
        Eigen::Vector2d yaw_pitch_nan = heroaim(target, 100000, gimbal_yaw);
        plan.yaw = yaw_pitch_nan(0);
        plan.yaw_vel = 0;
        plan.yaw_acc = 0;

        plan.pitch = yaw_pitch_nan(1);
        plan.pitch_vel = 0;
        plan.pitch_acc = 0;
      }
  }

  // 开火判断依据
  auto is_fire = [this](const double plan_yaw, const Target& target_, bool tower_fixed_pitch){
    bool suggest_fire = 1;

    auto xyzad = target_.get_recent_armor_xyzad();
    Eigen::Vector4d target_armor_xyza = xyzad.head<4>();
    double target_yaw = target_armor_xyza(3) ;
    aim_target_yaw = atan2(target_armor_xyza(1), target_armor_xyza(0));//+ 0.3/57.3;
    double shoot_range = target_.armor_type == ArmorType::big ? big_armor_tolerance : small_armor_tolerance;

    if(target_.name == ArmorName::base || target_.name == ArmorName::outpost) 
      shoot_range = tower_and_base_armor_tolerance_;

      
    // 打击范围计算
    // 左边缘（沿切线正方向偏移半宽）
    double left_x = target_armor_xyza(0) + 0.5 * shoot_range * (-sin(target_yaw));
    double left_y = target_armor_xyza(1) + 0.5 * shoot_range * cos(target_yaw);

    // 右边缘（沿切线负方向偏移半宽）
    double right_x = target_armor_xyza(0) - 0.5 * shoot_range * (-sin(target_yaw));
    double right_y = target_armor_xyza(1) - 0.5 * shoot_range * cos(target_yaw);


    // 目标中心方向
    double center_angle = atan2(target_armor_xyza(1), target_armor_xyza(0));
    // 当前云台偏差（相对于中心）
    double delta = tools::limit_rad(plan_yaw - center_angle);


    double left_angle = atan2(left_y, left_x);
    double right_angle = atan2(right_y, right_x);
    // 计算左右边缘相对于中心的角度偏移
    double d_left = tools::limit_rad(left_angle - center_angle);
    double d_right = tools::limit_rad(right_angle - center_angle);

    // 取较小的和较大的偏移（因为左右边缘距离中心不会超过 90°，所以 d_left 和 d_right 符号相反且绝对值 < π/2）
    double d_min = std::min(d_left, d_right);
    double d_max = std::max(d_left, d_right);


    // pitch
    bool suggest_pitch = true;
    if(tower_fixed_pitch && abs(target_.ekf_x()(4) - target_armor_xyza(2)) > 0.001){
      suggest_pitch = false;
    }
    // 判断 delta 是否在 [d_min, d_max] 范围内
    suggest_fire = (delta >= d_min && delta <= d_max) && suggest_pitch;



    if(!outpost_is_make && target_.name == ArmorName::outpost) suggest_fire = 0;
    if(!suggest_fire){
      // tools::logger()->info("not fire! control_delta_angle: {},  allow_fire_ang_max: {}, allow_fire_ang_min: {}",
      //   control_delta_angle, allow_fire_ang_max, allow_fire_ang_min
      // );
    }
      
    return suggest_fire;
  };
  // gimbal_yaw 由上层以「度」传入（见 io::GimbalState::yaw），yaw_offset_ 为弧度，先统一到弧度
  // 开火门限参考 = 子弹命中点 = 目标在(出膛时刻+飞行时间) 的位置
  //   = θ(t + τ_fire + fly)，τ_fire 即 high/low_speed_delay_time（发弹命令→出膛延迟）。
  // 当前 target 的时间戳 = now + speed_delay + gimbal_delay + fly，
  // 只需回退云台提前量 gimbal_delay（τ+T）即落在 θ(t + speed_delay + fly)。
  // 注意：不能把 speed_delay 也回退（上一版错误）——否则门限参考晚于命中点，
  // 高速目标即使完全命中也被门限拒发。
  Target target_at_bullet = target;
  target_at_bullet.predict(-gimbal_delay_);
  plan.fire = is_fire(gimbal_yaw / 57.3 - yaw_offset_, target_at_bullet, false);
  // tools::logger()->warn("fire:{}", plan.fire);
  plan.target_yaw = (aim_target_yaw + yaw_offset_ )* 57.3;

  // 方案 C：云台电机响应落地检查（用标定模型模拟真实云台，验证开火时刻确实到位）
  if (fire_landing_tolerance_ > 0) {
    double yaw_err = 0, pitch_err = 0;
    gimbal_landing_check(
      traj, yaw_solver_->work->x, pitch_solver_->work->x, fb, yaw0, yaw_err, pitch_err);
    const bool landed = yaw_err < fire_landing_tolerance_ && pitch_err < fire_landing_tolerance_;
    if (plan.fire && !landed) {
      tools::logger()->debug(
        "[Planner] landing check blocked: yaw_err={:.3f}deg pitch_err={:.3f}deg (tol={:.2f}deg)",
        yaw_err * 57.3, pitch_err * 57.3, fire_landing_tolerance_ * 57.3);
    }
    plan.fire = plan.fire && landed;
  }

  return plan;
}


Plan Planner::rbHeroplan(Target target, double bullet_speed, double gimbal_yaw){
  if (target.armor_xyza_list().empty()) return {false};
  // 0. Check bullet speed
  if (bullet_speed < 10 || bullet_speed > 25) {
    bullet_speed = 22;
  }

  // 1. Predict fly_time
  Eigen::Vector3d xyz;
  auto min_dist = 1e10;
  for (auto & xyza : target.armor_xyza_list()) {
    auto dist = xyza.head<2>().norm();
    if (dist < min_dist) {
      min_dist = dist;
      xyz = xyza.head<3>();
    }
  }
  min_dist+=target_dist_error_;
  double target_h = xyz.z(); 
  target_h+= target_h_error_;

    // tools::logger()->info("h:{}, xy_d:{}, xyz_d:{}, fly_time:{}, ", xyz.z(), min_dist, xyz.norm(), bullet_traj.fly_time);

  auto bullet_traj = tools::Trajectory(bullet_speed, min_dist, target_h);
  is_far = min_dist > 5.0;
//  tools::logger()->info("h:{}, xy_d:{}, xyz_d:{}, fly_time:{}, is_far{} ", target_h, min_dist, xyz.norm(), bullet_traj.fly_time, is_far);
  
  target.predict(bullet_traj.fly_time );

  // 2. Get trajectory
  Eigen::Vector2d yaw_pitch;
  try {
    yaw_pitch = heroaim(target, bullet_speed, gimbal_yaw);
    if (!yaw_pitch.allFinite()) throw std::runtime_error("non-finite hero aim result");
  } catch (const std::exception & e) {
    tools::logger()->warn("Unsolvable target {:.2f}", bullet_speed);
    return {false};
  }



  Plan plan{};
  plan.control = true;
  // plan.target_yaw = tools::limit_rad(traj(0, HALF_HORIZON) + yaw0);
  
  plan.target_pitch = yaw_pitch(1);

  plan.yaw = yaw_pitch(0); // tools::limit_rad(yaw_solver_->work->x(0, HALF_HORIZON) + yaw0);
  plan.yaw_vel = 0; //yaw_solver_->work->x(1, HALF_HORIZON);
  plan.yaw_acc = 0; //yaw_solver_->work->u(0, HALF_HORIZON);

  plan.pitch = yaw_pitch(1); //itch_solver_->work->x(0, HALF_HORIZON);
  plan.pitch_vel = 0; //pitch_solver_->work->x(1, HALF_HORIZON);
  plan.pitch_acc = 0; //pitch_solver_->work->u(0, HALF_HORIZON);

  
  // plan.fire =
  //   std::hypot(
  //     traj(0, HALF_HORIZON + shoot_offset_) - yaw_solver_->work->x(0, HALF_HORIZON + shoot_offset_),
  //     traj(2, HALF_HORIZON + shoot_offset_) -
  //       pitch_solver_->work->x(0, HALF_HORIZON + shoot_offset_)) < fire_thresh_;
  target.predict(-gimbal_control_delay);
  plan.fire = rbShoot(target, (gimbal_yaw )/57.3 - yaw_offset_
                                                    ,true
                                                  );
  // tools::logger()->warn("fire:{}", plan.fire);
  plan.target_yaw = (aim_target_yaw + yaw_offset_ )* 57.3;



  // 前哨站迭代限制
  if (target.name == ArmorName::outpost) {
      double vz = target.ekf_x()(5); // 获取当前前哨站中心Z轴坐标
      // double delta_z = std::abs(current_z - outpost_z_baseline_);
      auto now = std::chrono::steady_clock::now();

      // 如果Z轴变化幅度大于指定阈值（例如0.05米），重置基准和计时器，并禁止开火
      if (vz > 0.01) { 
          outpost_z_stable_start_time_ = now;
          // suggest_fire = false; 
          outpost_is_make = false;
      } else {
        if (
          target.update_count_ < 500) {
            // suggest_fire = false; // 持续时间不足 0.7s，不开火
            outpost_is_make = false;
        }
        else{
          outpost_is_make = true;
        }
      }

      if(!outpost_is_make){
        Eigen::Vector2d yaw_pitch_nan = heroaim(target, 100000, gimbal_yaw);
        plan.yaw = yaw_pitch_nan(0);
        plan.yaw_vel = 0;
        plan.yaw_acc = 0;

        plan.pitch = yaw_pitch_nan(1) + 10/57.3;
        plan.pitch_vel = 0;
        plan.pitch_acc = 0;
      }
  }

  return plan;
}

Eigen::Matrix<double, 2, 1> Planner::aim(const Target & target, double bullet_speed)
{
  if (target.armor_xyza_list().empty()) throw std::runtime_error("Target has no armor pose");
  Eigen::Vector3d xyz;
  double yaw;
  auto min_dist = 1e10;

  for (auto & xyza : target.armor_xyza_list()) {
    auto dist = xyza.head<2>().norm();
    if (dist < min_dist) {
      min_dist = dist;
      xyz = xyza.head<3>();
      yaw = xyza[3];
    }
  }
  debug_xyza = Eigen::Vector4d(xyz.x(), xyz.y(), xyz.z(), yaw);

  auto azim = std::atan2(xyz.y(), xyz.x());
  auto bullet_traj = tools::Trajectory(bullet_speed, min_dist, xyz.z());
  if (bullet_traj.unsolvable) throw std::runtime_error("Unsolvable bullet trajectory!");

  return {tools::limit_rad(azim + yaw_offset_), bullet_traj.pitch + pitch_offset_};
}

Eigen::Matrix<double, 2, 1> Planner::rbaim(const Target & target, double bullet_speed)
{
  if (target.armor_xyza_list().empty()) throw std::runtime_error("Target has no armor pose");

  Eigen::Matrix<double, 5, 1> xyzad = target.get_recent_armor_xyzad();
  Eigen::Vector3d xyz = xyzad.head<3>();
  double yaw = xyzad(3);
  auto min_dist = xyz.head<2>().norm();

  debug_xyza = Eigen::Vector4d(xyz.x(), xyz.y(), xyz.z(), yaw);

  auto azim = std::atan2(xyz.y(), xyz.x());
  auto bullet_traj = tools::Trajectory(bullet_speed, min_dist, xyz.z());
  if (bullet_traj.unsolvable) throw std::runtime_error("Unsolvable bullet trajectory!");

  double now_pitch_offset = 0;
  if(is_far & is_high) {now_pitch_offset = far_high_pitch_offset_;
    // tools::logger()->info("far_high_pitch_offset_");
  }
  else if(is_far) {
    now_pitch_offset = far_pitch_offset_;
    // tools::logger()->info("far_pitch_offset_");
  }
  else {now_pitch_offset = pitch_offset_; 
    // tools::logger()->info("pitch_offset_");
  }

  return {tools::limit_rad(azim + yaw_offset_), bullet_traj.pitch + now_pitch_offset};


}

Eigen::Matrix<double, 2, 1> Planner::heroaim(const Target & target, double bullet_speed, double gimbal_yaw)
{
  auto armors = target.armor_xyza_list();
  if (armors.empty()) throw std::runtime_error("Target has no armor pose");

  Eigen::Vector3d xyz;
  double yaw;
  auto min_dist = 1e10;

  Eigen::VectorXd ekf_x = target.ekf_x();
  // 如果delta_angle为0，则该装甲板中心和整车中心的连线在世界坐标系的xy平面过原点
  std::vector<std::pair<int ,double>> armorId_delta_list;
  std::vector<Eigen::Vector4d> armor_xyza_list = target.armor_xyza_list();

  auto armor_num = armor_xyza_list.size();
  // // 如果装甲板未发生过跳变，则只有当前装甲板的位置已知
  // if (!target.jumped) return {true, armor_xyza_list[0]};

  // 整车旋转中心的球坐标yaw
  auto center_yaw = std::atan2(ekf_x[2], ekf_x[0]);

  for (int i = 0; i < armor_num; i++) {
    auto delta_angle = tools::limit_rad(armor_xyza_list[i][3] - center_yaw);
    // auto dist = armor_xyza_list[i].head<2>().norm();
    armorId_delta_list.emplace_back(std::make_pair(i, delta_angle));
  }
  
  for (auto & xyza : target.armor_xyza_list()) {
    auto dist = xyza.head<2>().norm();
    if (dist < min_dist) {
      min_dist = dist;
      xyz = xyza.head<3>();
      yaw = xyza[3];
    }
  }

  double abs_vyaw = abs(ekf_x(7));
  if(target.last_id >= 0 && static_cast<std::size_t>(target.last_id) < armor_xyza_list.size() &&
    abs_vyaw < 90./57.3 &&
    armorId_delta_list[target.last_id].second < 60./57.3){// 判断当前看到的装甲板在预测时间之后是否还在视野内
    min_dist = armor_xyza_list[target.last_id].head<2>().norm();
    xyz = armor_xyza_list[target.last_id].head<3>();
    yaw = armor_xyza_list[target.last_id](3);
  }



  auto r = target.ekf_x()(8);
  auto v_yaw = target.ekf_x()(7);

    // 旋转中心的坐标
  auto center_x = target.ekf_x()(0);
  auto center_y = target.ekf_x()(2);

  auto direction_yaw = atan2(center_y, center_x);
  auto aim_point_x = center_x - r*std::cos(direction_yaw);
  auto aim_point_y = center_y - r*std::sin(direction_yaw);
  auto aim_point_z = xyz.z();

  if(abs(v_yaw) < 1){
    aim_point_x = xyz.x();
    aim_point_y = xyz.y();
  }
  auto min_dist1 = min_dist;
  if(target.name == ArmorName::outpost){
    if (armor_xyza_list.size() < 3) throw std::runtime_error("Outpost target requires three armor poses");
    Target target_pitch = target;
    min_dist1 = 1e10;
    Eigen::Vector3d xyz1;
    double yaw1;
    double max_h_armor = 10e-6, min_h_armor = 10e+6;
    size_t max_armor_id, min_armor_id;
    target_pitch.predict(tower_pitch_prediction_time_);
    // for (auto & xyza : target.armor_xyza_list()) {
    //   auto dist = xyza.head<2>().norm();
    //   if (dist < min_dist1) {
    //     min_dist1 = dist;
    //     xyz1 = xyza.head<3>();
    //     yaw1 = xyza[3];
    //   }
    //   if(max_h_armor < xyza(2)) {
    //     max_h_armor = xyza(2); 
    //   }
    // }
    for(int i = 0; i < 3; i++){
      auto xyza = armor_xyza_list[i];
      auto dist = xyza.head<2>().norm();
      if (dist < min_dist1) {
        min_dist1 = dist;
        xyz1 = xyza.head<3>();
        yaw1 = xyza[3];
      }
      if(max_h_armor < xyza(2)) {
        max_h_armor = xyza(2); 
        max_armor_id = i;
      }
      if(min_h_armor > xyza(2)){
        min_h_armor = xyza(2); 
        min_armor_id = i;
      }
    }
    size_t middle_armor_id = 0;
    for(int i = 0; i < 3; i++){
      if(min_armor_id != i && max_armor_id != i ) middle_armor_id = i;
    }
    // if(abs(max_h_armor - target.ekf_x()(4)) < 0.05) aim_point_z = 
    // else aim_point_z = target.ekf_x()(4);
    aim_point_z = armor_xyza_list[middle_armor_id](2);
  }
  
  //补偿距离和补偿高度
  double comp_dist = 0;
  double comp_h = 0;
  min_dist = sqrt(aim_point_x*aim_point_x + aim_point_y*aim_point_y);

  debug_xyza = Eigen::Vector4d(aim_point_x, aim_point_y, aim_point_z, yaw);

  auto azim = std::atan2(aim_point_y, aim_point_x);
  auto bullet_traj = tools::Trajectory(bullet_speed, min_dist1 - comp_dist, aim_point_z - comp_h);
  if (bullet_traj.unsolvable) throw std::runtime_error("Unsolvable bullet trajectory!");

  auto now_pitch_offset = is_far ? far_pitch_offset_ : pitch_offset_;

  return {tools::limit_rad(azim + yaw_offset_), bullet_traj.pitch + now_pitch_offset};
}

void Planner::write_mpc_commands(
  Plan & plan,
  const Eigen::MatrixXd & yaw_x,
  const Eigen::MatrixXd & yaw_u,
  const Eigen::MatrixXd & pitch_x,
  const Eigen::MatrixXd & pitch_u,
  double yaw0) const
{
  /// 命令索引选择：
  /// - fire_aim（默认，兼容原行为）：发送开火时刻（时域中心）的瞄点；
  /// - trajectory_step：发送"经传输延时后下一拍"的规划轨迹状态，与下位机 1kHz
  ///   窗口插值器配合（命令序列即为云台应走的轨迹），并使落地检查模型自洽。
  const int idx = gimbal_command_mode_step_ ? (yaw_axis_model_.k_d + 1) : HALF_HORIZON;
  const int idx_clamped = std::max(0, std::min(idx, HORIZON - 2));

  plan.yaw = tools::limit_rad(yaw_x(0, idx_clamped) + yaw0);
  plan.yaw_vel = yaw_x(1, idx_clamped);
  plan.yaw_acc = yaw_u(0, idx_clamped);

  plan.pitch = pitch_x(0, idx_clamped);
  plan.pitch_vel = pitch_x(1, idx_clamped);
  plan.pitch_acc = pitch_u(0, idx_clamped);
}

void Planner::gimbal_landing_check(
  const Trajectory & traj,
  const Eigen::MatrixXd & yaw_x,
  const Eigen::MatrixXd & pitch_x,
  const GimbalFeedback & fb,
  double yaw0,
  double & yaw_err,
  double & pitch_err) const
{
  /// 用标定的"纯死区 k_d + 一阶 a"模型模拟真实云台从实际状态出发、
  /// 跟随"发送给下位机的命令序列"的响应，检查开火索引时刻与参考瞄点的残差。
  /// 命令序列：trajectory_step 模式取 MPC 规划轨迹（与发送的下一拍状态一致）；
  /// fire_aim 模式取参考轨迹（发送的瞄点序列在稳态下近似参考轨迹）。
  /// 注意：fire_aim 模式下模型云台稳态滞后命令 (τ+T)·ω，
  /// 因此比较基准取"回退 k_L 拍"的参考值（= 子弹到达时刻参考），
  /// 否则提前量与检查双重计滞后；trajectory_step 模式命令已按 k_d+1 预平移，
  /// 只剩余 (T−DT)·ω 小残差，无需再回退。
  const int k_f = HALF_HORIZON + shoot_offset_;
  yaw_err = 1e9;
  pitch_err = 1e9;
  if (k_f < 0 || k_f >= HORIZON) return;

  const int k_ref_yaw = gimbal_command_mode_step_
                          ? k_f
                          : std::max(0, k_f - static_cast<int>(std::lround(
                              (yaw_axis_model_.tau_s + yaw_axis_model_.T_cl_s) / DT)));
  const int k_ref_pitch = gimbal_command_mode_step_
                            ? k_f
                            : std::max(0, k_f - static_cast<int>(std::lround(
                                (pitch_axis_model_.tau_s + pitch_axis_model_.T_cl_s) / DT)));

  const auto simulate = [k_f](const GimbalAxisModel & model,
                              double x0,
                              const Eigen::MatrixXd & plan_x,
                              const Trajectory & traj,
                              int traj_row,
                              bool use_plan) {
    double p = x0;
    for (int k = 0; k <= k_f; ++k) {
      if (k < model.k_d) continue;  // 命令仍在传输/处理中，云台保持当前位置
      const int t = std::min(k - model.k_d, HORIZON - 1);
      const double cmd = use_plan ? plan_x(0, t) : traj(traj_row, t);
      p = model.a * p + (1.0 - model.a) * cmd;
    }
    return p;
  };

  // yaw：相对 yaw0 框架；pitch：框架同参考轨迹（绝对带偏移）
  const double yaw_sim = simulate(
    yaw_axis_model_, tools::limit_rad(fb.yaw_deg / 57.3 - yaw0),
    yaw_x, traj, 0, gimbal_command_mode_step_);
  const double pitch_sim = simulate(
    pitch_axis_model_, fb.pitch_deg / 57.3,
    pitch_x, traj, 2, gimbal_command_mode_step_);

  yaw_err = std::abs(yaw_sim - traj(0, k_ref_yaw));
  pitch_err = std::abs(pitch_sim - traj(2, k_ref_pitch));
}


}  // namespace auto_aim
