#include "tasks/auto_aim/tracking/center_acceleration_estimator.hpp"

#include <Eigen/Dense>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <vector>

#define CHECK(condition)            \
  do {                              \
    if (!(condition)) std::abort(); \
  } while (false)

namespace
{

using Estimator = auto_aim::CenterAccelerationEstimator;

Estimator::TimePoint at_seconds(Estimator::TimePoint origin, double seconds)
{
  return origin + std::chrono::duration_cast<Estimator::TimePoint::duration>(
                    std::chrono::duration<double>(seconds));
}

Eigen::Vector2d position(double time, const Eigen::Vector2d & acceleration)
{
  const Eigen::Vector2d initial_position(1.0, -2.0);
  const Eigen::Vector2d initial_velocity(0.8, 0.3);
  return initial_position + initial_velocity * time + 0.5 * acceleration * time * time;
}

}  // namespace

int main()
{
  const auto origin = Estimator::TimePoint{};
  const std::vector<double> steps{0.018, 0.023, 0.021, 0.026, 0.019, 0.024};

  // The shipped default is OFF; enable explicitly for the behaviour tests below.
  CHECK(!auto_aim::CenterAccelerationEstimatorConfig{}.enabled);
  auto enabled_config = auto_aim::CenterAccelerationEstimatorConfig{};
  enabled_config.enabled = true;

  Estimator default_estimator;
  for (int index = 0; index < 10; ++index) {
    default_estimator.add_sample(
      at_seconds(origin, index * 0.02), position(index * 0.02, {2.0, -1.0}));
  }
  CHECK(default_estimator.sample_count() == 0);
  CHECK(!default_estimator.valid(at_seconds(origin, 0.2)));

  Estimator insufficient(enabled_config);
  double time = 0.0;
  for (int index = 0; index < 5; ++index) {
    time += steps[static_cast<std::size_t>(index) % steps.size()];
    insufficient.add_sample(at_seconds(origin, time), position(time, {2.0, -1.0}));
  }
  CHECK(!insufficient.valid(at_seconds(origin, time)));
  CHECK(insufficient.acceleration(at_seconds(origin, time)).isZero(0.0));

  Estimator constant_acceleration(enabled_config);
  time = 0.0;
  for (int index = 0; index < 45; ++index) {
    time += steps[static_cast<std::size_t>(index) % steps.size()];
    Eigen::Vector2d sample = position(time, {2.0, -1.0});
    sample.x() += 0.002 * std::sin(17.0 * time);
    sample.y() += 0.002 * std::cos(13.0 * time);
    constant_acceleration.add_sample(at_seconds(origin, time), sample);
  }
  CHECK(constant_acceleration.valid(at_seconds(origin, time)));
  CHECK(
    (constant_acceleration.acceleration(at_seconds(origin, time)) - Eigen::Vector2d(2.0, -1.0))
      .norm() < 0.35);
  CHECK(constant_acceleration.last_fit_rmse() < 0.01);
  const double constant_acceleration_time = time;

  Estimator constant_velocity(enabled_config);
  time = 0.0;
  for (int index = 0; index < 35; ++index) {
    time += steps[static_cast<std::size_t>(index) % steps.size()];
    constant_velocity.add_sample(at_seconds(origin, time), position(time, {0.0, 0.0}));
  }
  CHECK(constant_velocity.valid(at_seconds(origin, time)));
  CHECK(constant_velocity.acceleration(at_seconds(origin, time)).norm() < 1e-8);

  CHECK(!constant_acceleration.valid(at_seconds(origin, constant_acceleration_time + 0.16)));
  CHECK(constant_acceleration.acceleration(at_seconds(origin, constant_acceleration_time + 0.16))
          .isZero(0.0));

  auto_aim::CenterAccelerationEstimatorConfig limited_config;
  limited_config.enabled = true;
  limited_config.window_seconds = 0.2;
  limited_config.min_samples = 3;
  limited_config.min_span_seconds = 0.015;
  limited_config.ema_alpha = 1.0;
  limited_config.max_acceleration = 2.0;
  limited_config.max_jerk = 10.0;
  limited_config.max_fit_rmse = 0.01;
  Estimator limited(limited_config);
  for (int index = 0; index < 3; ++index) {
    const double t = index * 0.01;
    limited.add_sample(at_seconds(origin, t), position(t, {20.0, 0.0}));
  }
  CHECK(limited.valid(at_seconds(origin, 0.02)));
  CHECK(limited.acceleration(at_seconds(origin, 0.02)).norm() <= 0.100001);

  limited.add_sample(at_seconds(origin, 0.03), Eigen::Vector2d(100.0, -100.0));
  CHECK(!limited.valid(at_seconds(origin, 0.03)));
  CHECK(limited.acceleration(at_seconds(origin, 0.03)).isZero(0.0));

  limited.add_sample(at_seconds(origin, 0.01), Eigen::Vector2d::Zero());
  CHECK(limited.sample_count() == 1);
  CHECK(!limited.valid(at_seconds(origin, 0.01)));

  Estimator gap_reset(enabled_config);
  gap_reset.add_sample(at_seconds(origin, 0.0), Eigen::Vector2d::Zero());
  gap_reset.add_sample(at_seconds(origin, 0.02), Eigen::Vector2d::Zero());
  gap_reset.add_sample(at_seconds(origin, 0.18), Eigen::Vector2d::Zero());
  CHECK(gap_reset.sample_count() == 1);

  auto disabled_config = auto_aim::CenterAccelerationEstimatorConfig{};
  disabled_config.enabled = false;
  Estimator disabled(disabled_config);
  for (int index = 0; index < 10; ++index) {
    disabled.add_sample(at_seconds(origin, index * 0.02), position(index * 0.02, {2.0, -1.0}));
  }
  CHECK(disabled.sample_count() == 0);
  CHECK(!disabled.valid(at_seconds(origin, 0.2)));

  return 0;
}
