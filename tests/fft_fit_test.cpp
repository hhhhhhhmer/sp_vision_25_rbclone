#include <chrono>
#include <cmath>
#include <iostream>

#include "tools/fft.hpp"

namespace
{

using Clock = std::chrono::steady_clock;

Clock::time_point at_seconds(Clock::time_point origin, double seconds)
{
  return origin +
         std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(seconds));
}

}  // namespace

int main()
{
  constexpr double amplitude = 0.14;
  constexpr double omega = 7.0;
  const double expected_frequency = omega / (2.0 * M_PI);
  const double armor_offsets[] = {0.72, 0.81, 0.66};

  tools::FFTExample fft;
  const auto time_origin = Clock::now();
  double t = 0.0;
  int sample = 0;
  while (t < 6.2) {
    const int armor_id = static_cast<int>(t / 0.55) % 3;
    const double outlier = sample % 97 == 0 ? 0.12 : 0.0;
    const double noise =
      0.01 * std::sin(31.0 * t) + 0.006 * std::sin(47.0 * t) + outlier;
    const double z =
      armor_offsets[armor_id] + 0.008 * t + amplitude * std::sin(omega * t) + noise;
    fft.add_sample(at_seconds(time_origin, t), armor_id, z);
    ++sample;
    t += (1.0 / 30.0) * (1.0 + 0.08 * std::sin(sample * 0.37));
  }

  const bool first_detection = fft.analyze();
  const bool second_detection = fft.analyze();
  const double test_time = 6.0;
  const double expected_component = amplitude * std::sin(omega * test_time);
  const double fitted_component = fft.get_value(at_seconds(time_origin, test_time));
  const tools::Wave fitted_wave = fft.get_wave();
  const double wave_component = fitted_wave.get_value(at_seconds(time_origin, test_time));
  const double expected_acceleration = -omega * omega * expected_component;
  const bool periodic_fit_ok =
    !first_detection && second_detection &&
    fitted_wave.valid() &&
    std::abs(fft.get_frequency() - expected_frequency) < 0.03 &&
    std::abs(fft.get_amplitude() - amplitude) < 0.02 &&
    std::abs(fitted_component - expected_component) < 0.03 &&
    std::abs(wave_component - fitted_component) < 1e-12 &&
    std::abs(fitted_wave.get_acceleration(at_seconds(time_origin, test_time)) -
             expected_acceleration) < 1.0;
  if (!periodic_fit_ok) {
    std::cerr << "periodic fit failed: frequency=" << fft.get_frequency()
              << " amplitude=" << fft.get_amplitude()
              << " fitted_component=" << fitted_component
              << " expected_component=" << expected_component << '\n';
    return 1;
  }

  // Keep margin below the four-second online detection target.
  fft.reset();
  if (!fitted_wave.valid() || fft.get_wave().valid() ||
      std::abs(fitted_wave.get_value(at_seconds(time_origin, test_time)) - wave_component) >
        1e-12) {
    std::cerr << "wave snapshot lifetime is incorrect\n";
    return 1;
  }
  constexpr double latency_test_frequency = 0.65;
  constexpr double latency_test_amplitude = 0.12;
  const auto latency_origin = time_origin + std::chrono::seconds(20);
  t = 0.0;
  sample = 0;
  double detected_at = 0.0;
  double next_analysis_at = 2.5;
  while (t < 3.5 && detected_at == 0.0) {
    const int armor_id = static_cast<int>(t / 0.55) % 3;
    const double z = armor_offsets[armor_id] + 0.006 * t +
                     latency_test_amplitude * std::sin(2.0 * M_PI * latency_test_frequency * t) +
                     0.006 * std::sin(37.0 * t);
    fft.add_sample(at_seconds(latency_origin, t), armor_id, z);
    if (t >= next_analysis_at) {
      if (fft.analyze()) detected_at = t;
      next_analysis_at += 0.25;
    }
    ++sample;
    t += (1.0 / 30.0) * (1.0 + 0.08 * std::sin(sample * 0.37));
  }
  if (detected_at == 0.0 || detected_at >= 3.5 ||
      std::abs(fft.get_frequency() - latency_test_frequency) > 0.05) {
    std::cerr << "early periodic detection failed: detected_at=" << detected_at
              << " frequency=" << fft.get_frequency() << '\n';
    return 1;
  }

  fft.reset();
  const auto nonperiodic_origin = time_origin + std::chrono::seconds(10);
  t = 0.0;
  while (t < 5.2) {
    const int armor_id = static_cast<int>(t / 0.7) % 3;
    fft.add_sample(
      at_seconds(nonperiodic_origin, t), armor_id, armor_offsets[armor_id] + 0.02 * t);
    t += 1.0 / 160.0;
  }
  if (fft.analyze() || fft.analyze()) {
    std::cerr << "non-periodic input was classified as periodic\n";
    return 1;
  }

  fft.reset();
  constexpr double below_threshold_amplitude = 0.05;
  const auto low_amplitude_origin = time_origin + std::chrono::seconds(40);
  t = 0.0;
  while (t < 5.2) {
    fft.add_sample(
      at_seconds(low_amplitude_origin, t), 0,
      below_threshold_amplitude * std::sin(2.0 * M_PI * 0.8 * t));
    t += 1.0 / 30.0;
  }
  if (fft.analyze() || fft.analyze()) {
    std::cerr << "periodic input below the amplitude threshold was accepted\n";
    return 1;
  }

  return 0;
}
