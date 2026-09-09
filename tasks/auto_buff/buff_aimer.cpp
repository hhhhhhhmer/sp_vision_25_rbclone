#include "buff_aimer.hpp"

#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/trajectory.hpp"

namespace auto_buff
{
Aimer::Aimer(const std::string & config_path)
{
  auto yaml = YAML::LoadFile(config_path);
  yaw_offset_ = yaml["yaw_offset"].as<double>() / 57.3;      // degree to rad
  pitch_offset_ = yaml["pitch_offset"].as<double>() / 57.3;  // degree to rad
  fire_gap_time_ = yaml["fire_gap_time"].as<double>();
  predict_time_ = yaml["predict_time"].as<double>();
  if (yaml["buff_rune_radius_m"]) RUNE_RADIUS_M = yaml["buff_rune_radius_m"].as<double>();
  if (yaml["buff_small_direction"]) SMALL_BUFF_DIRECTION = yaml["buff_small_direction"].as<int>();
  if (yaml["buff_fire_full_observation_max_age_s"]) {
    BUFF_FIRE_FULL_OBSERVATION_MAX_AGE_S =
      yaml["buff_fire_full_observation_max_age_s"].as<double>();
  }

  last_fire_t_ = std::chrono::steady_clock::now();
}

io::Command Aimer::aim(
  auto_buff::Target & target, std::chrono::steady_clock::time_point & timestamp,
  double bullet_speed, bool to_now)
{
  io::Command command = {false, false, 0, 0};
  predicted_target_.reset();
  if (!target.can_control()) return command;

  // 如果子弹速度小于10，将其设为24
  if (bullet_speed < 10) bullet_speed = 24;

  auto now = std::chrono::steady_clock::now();

  auto detect_now_gap = tools::delta_time(now, timestamp);
  auto future = to_now ? (detect_now_gap + predict_time_) : 0.1 + predict_time_;
  double yaw, pitch;

  if (get_send_angle(target, future, bullet_speed, to_now, yaw, pitch, true)) {
    command.yaw = yaw;
    command.pitch = -pitch;  //世界坐标系下的pitch向上为负
    command.control = true;
  }

  if (!command.control || !solution_converged_ || !target.can_fire(now)) {
    command.shoot = false;
    last_fire_t_ = now;
  } else if (tools::delta_time(now, last_fire_t_) > fire_gap_time_) {
    command.shoot = true;
    last_fire_t_ = now;
  }

  return command;
}

auto_aim::Plan Aimer::mpc_aim(
  auto_buff::Target & target, std::chrono::steady_clock::time_point & timestamp, io::GimbalState gs,
  bool to_now)
{
  auto_aim::Plan plan = {false, false, 0, 0, 0, 0, 0, 0, 0, 0};
  predicted_target_.reset();
  if (!target.can_control()) return plan;

  double bullet_speed;
  // 如果子弹速度小于10，将其设为24
  if (gs.bullet_speed < 10)
    bullet_speed = 24;
  else
    bullet_speed = gs.bullet_speed;

  auto now = std::chrono::steady_clock::now();

  auto detect_now_gap = tools::delta_time(now, timestamp);
  auto future = to_now ? (detect_now_gap + predict_time_) : 0.1 + predict_time_;
  double yaw, pitch;

  if (get_send_angle(target, future, bullet_speed, to_now, yaw, pitch, true)) {
    const bool future_solution_converged = solution_converged_;
    plan.yaw = yaw;
    plan.pitch = pitch;
    plan.control = true;

    if (plan.control) {
    if (first_in_aimer_) {
      plan.yaw_vel = 0; plan.yaw_acc = 0;
      plan.pitch_vel = 0; plan.pitch_acc = 0;
      first_in_aimer_ = false;
    } else {
      auto dt = predict_time_;
      double last_yaw_mpc, last_pitch_mpc;

      // 使用另一个全新的副本，推算“当前瞬间”的位姿，而不是传入负时间！
      auto target_for_now = target.clone();
      double now_time = to_now ? detect_now_gap : 0.1; 
      if (get_send_angle(*target_for_now, now_time, bullet_speed, to_now, last_yaw_mpc, last_pitch_mpc)) {
        plan.yaw_vel = tools::limit_rad(yaw - last_yaw_mpc) / (2 * dt);
        plan.yaw_acc =
          (tools::limit_rad(yaw - gs.yaw) - tools::limit_rad(gs.yaw - last_yaw_mpc)) /
          std::pow(dt, 2);
        plan.pitch_vel = tools::limit_rad(-pitch + last_pitch_mpc) / (2 * dt);
        plan.pitch_acc =
          (-pitch - gs.pitch - (gs.pitch + last_pitch_mpc)) / std::pow(dt, 2);
      } else {
        plan.yaw_vel = plan.yaw_acc = 0.0;
        plan.pitch_vel = plan.pitch_acc = 0.0;
      }
    }
    solution_converged_ = future_solution_converged;
  }
}

  if (!plan.control || !solution_converged_ || !target.can_fire(now)) {
    plan.fire = false;
    last_fire_t_ = now;
  } else if (tools::delta_time(now, last_fire_t_) > fire_gap_time_) {
    plan.fire = true;
    last_fire_t_ = now;
  }

  return plan;
}

bool Aimer::get_send_angle(
  auto_buff::Target & target, const double predict_time, const double bullet_speed,
  const bool to_now, double & yaw, double & pitch, bool save_prediction)
{
  (void)to_now;
  solution_converged_ = false;
  double flight_time = 0.0;
  bool has_solution = false;
  std::unique_ptr<Target> solved_prediction;

  for (int iteration = 0; iteration < 3; ++iteration) {
    auto predicted = target.clone();
    predicted->predict(std::max(0.0, predict_time + flight_time));
    angle = predicted->ekf_x()[5];

    const auto aim_in_world =
      predicted->point_buff2world(Eigen::Vector3d(0.0, 0.0, RUNE_RADIUS_M));
    const double d = std::sqrt(
      aim_in_world[0] * aim_in_world[0] + aim_in_world[1] * aim_in_world[1]);
    const double h = aim_in_world[2];
    if (!std::isfinite(d) || !std::isfinite(h) || d > 20.0) {
      tools::logger()->debug("[Aimer] Invalid coordinate blocked: d={:.2f}, h={:.2f}", d, h);
      return false;
    }

    tools::Trajectory trajectory(bullet_speed, d, h);
    if (trajectory.unsolvable) {
      tools::logger()->debug(
        "[Aimer] Unsolvable trajectory: {:.2f} {:.2f} {:.2f}", bullet_speed, d, h);
      return false;
    }

    yaw = std::atan2(aim_in_world[1], aim_in_world[0]) + yaw_offset_;
    pitch = trajectory.pitch + pitch_offset_;
    has_solution = true;
    const double time_error = trajectory.fly_time - flight_time;
    flight_time = trajectory.fly_time;
    if (save_prediction) solved_prediction = std::move(predicted);
    if (iteration > 0 && std::abs(time_error) <= 0.005) {
      solution_converged_ = true;
      break;
    }
  }

  if (has_solution && !solution_converged_) {
    tools::logger()->debug("[Aimer] trajectory did not converge, control only");
  }
  if (has_solution && save_prediction) predicted_target_ = std::move(solved_prediction);
  return has_solution;
};

}  // namespace auto_buff
