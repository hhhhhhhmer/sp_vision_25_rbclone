#ifndef AUTO_AIM__PLANNER_HPP
#define AUTO_AIM__PLANNER_HPP

#include <Eigen/Dense>
#include <list>
#include <optional>

#include "tasks/auto_aim/target.hpp"
#include "tinympc/tiny_api.hpp"
#include "tools/logger.hpp"

namespace auto_aim
{
constexpr double DT = 0.01;
constexpr int HALF_HORIZON = 50;
constexpr int HORIZON = HALF_HORIZON * 2;

using Trajectory = Eigen::Matrix<double, 4, HORIZON>;  // yaw, yaw_vel, pitch, pitch_vel

struct Plan
{
  bool control;
  bool fire;
  float target_yaw;
  float target_pitch;
  float yaw;
  float yaw_vel;
  float yaw_acc;
  float pitch;
  float pitch_vel;
  float pitch_acc;
};

class Planner
{
public:
  enum ShootStrategy{//开火策略
    Dynamics,          //动力学
    rbSuppressiveFire, //旧火控,火力压制
    rbHero,             //英雄
    SB                  //哨兵
  };
  Eigen::Vector4d debug_xyza;
  double aim_target_yaw;
  Planner(const std::string & config_path);

  Plan plan(Target target, double bullet_speed);
  inline Plan plan(std::optional<Target> target, 
            double bullet_speed, 
            double gimbal_yaw = 0,
            ShootStrategy strategy = Dynamics){

    if (!target.has_value()) return {false};

    double delay_time =
      (std::abs(target->ekf_x()[7]) > decision_speed_ ? high_speed_delay_time_ : low_speed_delay_time_) + gimbal_delay_;

    if(std::abs(target->ekf_x()[7]) > decision_speed_) tools::logger()->warn("std::abs(target->ekf_x()[7]) > {}", decision_speed_);

    auto future = std::chrono::steady_clock::now() + std::chrono::microseconds(int(delay_time * 1e6));
    is_far = false;
    is_high = false;
    
    target->predict(future);

    switch (strategy)
    {
    case Dynamics:
      return plan(*target, bullet_speed);
    case rbSuppressiveFire:
      return rbplan(*target, bullet_speed, gimbal_yaw);
    case rbHero:
      return rbHeroplan(*target, bullet_speed, gimbal_yaw);
    case SB:
      return sbplan(*target, bullet_speed, gimbal_yaw);
    default:
      tools::logger()->error("Unknown shoot strategy: {}", static_cast<int>(strategy));
      return {false};
    }
  }
  Plan rbplan(Target target, double bullet_speed, double gimbal_yaw);
  Plan sbplan(Target target, double bullet_speed, double gimbal_yaw);
  bool rbShoot(Target target, double gimbal_yaw,  bool tower_fixed_pitch = false);
  Plan rbHeroplan(Target target, double bullet_speed, double gimbal_yaw); 
private:
  bool is_far = false;
  bool is_high = false;
  double yaw_offset_;
  double pitch_offset_;
  double far_pitch_offset_;
  double far_high_pitch_offset_;
  double fire_thresh_;
  double target_dist_error_, target_h_error_;
  double low_speed_delay_time_, high_speed_delay_time_, decision_speed_;
  double small_armor_tolerance, big_armor_tolerance;
  double tower_and_base_armor_tolerance_;
  double gimbal_control_delay;
  double tower_pitch_prediction_time_;

  TinySolver * yaw_solver_;
  TinySolver * pitch_solver_;

  int last_selected_idx = -1;
  Eigen::Vector3d last_selected_xyz = Eigen::Vector3d::Zero();

  void setup_yaw_solver(const std::string & config_path);
  void setup_pitch_solver(const std::string & config_path);

  Eigen::Matrix<double, 2, 1> aim(const Target & target, double bullet_speed);
  Eigen::Matrix<double, 2, 1> rbaim(const Target & target, double bullet_speed);
  Eigen::Matrix<double, 2, 1> heroaim(const Target & target, double bullet_speed, double gimbal_yaw);
  Trajectory get_trajectory(Target  target, double yaw0, double bullet_speed);
  Trajectory rbget_trajectory(Target target, double yaw0, double bullet_speed);

  std::chrono::steady_clock::time_point outpost_z_stable_start_time_;
  bool outpost_is_make = true;

  double gimbal_delay_;

  int shoot_offset_;
};

}  // namespace auto_aim

#endif  // AUTO_AIM__PLANNER_HPP
