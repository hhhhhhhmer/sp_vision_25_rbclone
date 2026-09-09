#ifndef AUTO_BUFF__AIMER_HPP
#define AUTO_BUFF__AIMER_HPP

#include <yaml-cpp/yaml.h>

#include <Eigen/Dense>
#include <chrono>
#include <cmath>
#include <memory>
#include <vector>

#include "../auto_aim/planner/planner.hpp"
#include "buff_target.hpp"
#include "buff_type.hpp"
#include "io/command.hpp"
#include "io/gimbal/gimbal.hpp"

namespace auto_buff
{
class Aimer
{
public:
  Aimer(const std::string & config_path);

  io::Command aim(
    Target & target, std::chrono::steady_clock::time_point & timestamp, double bullet_speed,
    bool to_now = true);

  auto_aim::Plan mpc_aim(
    Target & target, std::chrono::steady_clock::time_point & timestamp, io::GimbalState gs,
    bool to_now = true);

  const Target * predicted_target() const { return predicted_target_.get(); }

  double angle;      ///
  double t_gap = 0;  ///

private:
  SmallTarget target_;
  double yaw_offset_;
  double pitch_offset_;

  double fire_gap_time_;
  double predict_time_;

  bool solution_converged_ = false;
  std::unique_ptr<Target> predicted_target_;

  // for mpc
  bool first_in_aimer_ = true;

  std::chrono::steady_clock::time_point last_fire_t_;

  bool get_send_angle(
    auto_buff::Target & target, const double predict_time, const double bullet_speed,
    const bool to_now, double & yaw, double & pitch, bool save_prediction = false);
};
}  // namespace auto_buff
#endif  // AUTO_AIM__AIMER_HPP
