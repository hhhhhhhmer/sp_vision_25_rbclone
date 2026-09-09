#pragma once

#include <Eigen/Dense>
#include <deque>
#include <iostream>
#include <limits>
#include <vector>

namespace tools
{

class RansacSineFitter
{
public:
  struct Result
  {
    double A = 0.0;
    double omega = 0.0;
    double phi = 0.0;
    double C = 0.0;
    int inliers = 0;
    double rms = std::numeric_limits<double>::infinity();
  };
  Result best_result_;

  RansacSineFitter(int max_iterations, double threshold, double min_omega, double max_omega);

  void add_data(double t, double v);

  void fit();

  void clear();

  size_t sample_count() const;

  double time_span() const;

  double inlier_ratio() const;

  bool ready(
    size_t min_samples, double min_time_span, double min_inlier_ratio,
    double max_rms = std::numeric_limits<double>::infinity()) const;

  double sine_function(double t, double A, double omega, double phi, double C)
  {
    return A * std::sin(omega * t + phi) + C;
  }

private:
  double threshold_;
  double min_omega_;
  double max_omega_;
  std::deque<std::pair<double, double>> fit_data_;

  bool fit_weighted_model(
    double omega, const std::vector<double> & weights, Eigen::Vector3d & params) const;

  Result evaluate_model(double A, double omega, double phi, double C) const;
};

}  // namespace tools
