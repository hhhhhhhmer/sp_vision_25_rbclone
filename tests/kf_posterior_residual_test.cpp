#include <cmath>
#include <iostream>

#include "tasks/auto_aim/kf_example/ca_from_tju.hpp"
#include "tasks/auto_aim/kf_example/rv_from_fyt.hpp"
#include "tasks/auto_aim/target.hpp"
#include "tools/math_tools.hpp"

namespace
{

constexpr double kPi = 3.14159265358979323846;

bool near(const Eigen::Vector4d & actual, const Eigen::Vector4d & expected)
{
  return (actual - expected).cwiseAbs().maxCoeff() < 1e-9;
}

bool near(double actual, double expected, double tolerance = 1e-9)
{
  return std::abs(actual - expected) < tolerance;
}

}  // namespace

int main()
{
  Eigen::VectorXd ca_state = Eigen::VectorXd::Zero(auto_aim::CAFromTJU::kStateDimension);
  ca_state.head<4>() << 1.0, 2.0, 3.0, kPi - 0.1;
  const Eigen::MatrixXd ca_covariance = Eigen::MatrixXd::Identity(
    auto_aim::CAFromTJU::kStateDimension, auto_aim::CAFromTJU::kStateDimension);
  auto_aim::CAFromTJU ca(ca_state, ca_covariance);

  const Eigen::Vector4d ca_observation(2.0, 0.0, 3.5, -kPi + 0.1);
  const Eigen::Vector4d expected_ca(1.0, 4.0, 0.25, 0.04);
  const auto & ca_interface = static_cast<const auto_aim::State2Est &>(ca);
  if (!near(ca_interface.posterior_residual_squared(ca_observation), expected_ca)) {
    std::cerr << "CAFromTJU posterior residual is incorrect\n";
    return 1;
  }

  Eigen::VectorXd rv_state = Eigen::VectorXd::Zero(auto_aim::RVfromFYT::kStateDimension);
  rv_state[0] = 1.0;
  rv_state[2] = 2.0;
  rv_state[4] = 3.0;
  rv_state[6] = kPi - 0.1;
  rv_state[8] = 0.2;
  const Eigen::MatrixXd rv_covariance = Eigen::MatrixXd::Identity(
    auto_aim::RVfromFYT::kStateDimension, auto_aim::RVfromFYT::kStateDimension);
  auto_aim::RVfromFYT rv(rv_state, rv_covariance, 4, auto_aim::ArmorName::sentry);

  const Eigen::Vector3d posterior_xyz = rv.h_armor_xyz(rv.x, 0);
  const Eigen::Vector4d rv_observation(
    posterior_xyz[0] + 1.0, posterior_xyz[1] - 2.0, posterior_xyz[2] + 0.5,
    -kPi + 0.1);
  const Eigen::Vector4d expected_rv(1.0, 4.0, 0.25, 0.04);
  const auto & rv_interface = static_cast<const auto_aim::State2Est &>(rv);
  if (!near(rv_interface.posterior_residual_squared(rv_observation, 0), expected_rv)) {
    std::cerr << "RVfromFYT posterior residual is incorrect\n";
    return 1;
  }

  auto_aim::Target target(1.0, 0.0, 0.2, 0.0);
  const std::vector<cv::Point2f> keypoints{
    {0.0F, 0.0F}, {1.0F, 0.0F}, {1.0F, 1.0F}, {0.0F, 1.0F}};
  auto_aim::Armor armor(0, 1.0F, cv::Rect{}, keypoints);
  armor.xyz_in_world = {0.85, 0.05, 0.02};
  armor.ypr_in_world = {0.1, 0.0, 0.0};
  armor.ypd_in_world = tools::xyz2ypd(armor.xyz_in_world);
  target.update(armor);

  Eigen::Vector4d target_observation;
  target_observation << armor.xyz_in_world, armor.ypr_in_world[0];
  const Eigen::Vector4d expected_target =
    target.ekf().posterior_residual_squared(target_observation, target.last_id);
  if (!near(target.rv_residual, expected_target)) {
    std::cerr << "Target did not store the RVfromFYT posterior residual\n";
    return 1;
  }

  auto_aim::Target timed_target(1.0, 0.0, 0.2, 0.0);
  const auto prediction_start = timed_target.getTimePoint();
  tools::Wave wave;
  wave.is_periodic = true;
  wave.frequency = 1.0;
  wave.amplitude = 0.1;
  wave.time_origin = prediction_start;
  timed_target.wave_ = wave;

  constexpr double dt = 0.1;
  const auto prediction_end = prediction_start +
    std::chrono::duration_cast<std::chrono::steady_clock::duration>(
      std::chrono::duration<double>(dt));
  const auto prediction_midpoint = prediction_start +
    std::chrono::duration_cast<std::chrono::steady_clock::duration>(
      std::chrono::duration<double>(dt / 2.0));
  const double expected_acceleration = wave.get_acceleration(prediction_midpoint);
  timed_target.predict(prediction_end);
  if (!near(timed_target.ekf_x()[4], 0.5 * expected_acceleration * dt * dt, 1e-8) ||
      !near(timed_target.ekf_x()[5], expected_acceleration * dt, 1e-8)) {
    std::cerr << "Target time-point prediction did not apply Wave acceleration\n";
    return 1;
  }

  auto_aim::Target relative_target(1.0, 0.0, 0.2, 0.0);
  const auto relative_start = relative_target.getTimePoint();
  wave.time_origin = relative_start;
  relative_target.wave_ = wave;
  const auto relative_midpoint = relative_start +
    std::chrono::duration_cast<std::chrono::steady_clock::duration>(
      std::chrono::duration<double>(dt / 2.0));
  const double relative_acceleration = wave.get_acceleration(relative_midpoint);
  relative_target.predict(dt);
  if (!near(relative_target.ekf_x()[4], 0.5 * relative_acceleration * dt * dt, 1e-8) ||
      !near(relative_target.ekf_x()[5], relative_acceleration * dt, 1e-8)) {
    std::cerr << "Target relative prediction did not apply Wave acceleration\n";
    return 1;
  }

  const Eigen::VectorXd round_trip_state = relative_target.ekf_x();
  const auto round_trip_time = relative_target.getTimePoint();
  relative_target.predict(dt);
  relative_target.predict(-dt);
  if ((relative_target.ekf_x() - round_trip_state).cwiseAbs().maxCoeff() > 1e-8 ||
      relative_target.getTimePoint() != round_trip_time) {
    std::cerr << "Target relative prediction did not preserve Wave time on rollback\n";
    return 1;
  }

  bool invalid_control_rejected = false;
  try {
    relative_target.predict(0.01, Eigen::VectorXd::Zero(2));
  } catch (const std::invalid_argument &) {
    invalid_control_rejected = true;
  }
  if (!invalid_control_rejected) {
    std::cerr << "Target accepted an undersized acceleration vector\n";
    return 1;
  }

  auto_aim::Target placeholder;
  bool uninitialized_target_rejected = false;
  try {
    placeholder.predict(0.01);
  } catch (const std::logic_error &) {
    uninitialized_target_rejected = true;
  }
  if (!uninitialized_target_rejected) {
    std::cerr << "Default Target allowed prediction before initialization\n";
    return 1;
  }

  return 0;
}
