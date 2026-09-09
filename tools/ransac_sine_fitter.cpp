#include "ransac_sine_fitter.hpp"

#include <Eigen/Dense>
#include <algorithm>
#include <cmath>
#include <limits>

namespace tools
{

RansacSineFitter::RansacSineFitter(
  int, double threshold, double min_omega, double max_omega)
: threshold_(threshold),
  min_omega_(min_omega),
  max_omega_(max_omega)
{
}

void RansacSineFitter::add_data(double t, double v)
{
  if (!fit_data_.empty() && t - fit_data_.back().first > 0.5) clear();
  fit_data_.emplace_back(std::make_pair(t, v));
  while (fit_data_.size() > 180) fit_data_.pop_front();
}

void RansacSineFitter::fit()
{
  if (fit_data_.size() < 3) return;

  Result best;
  constexpr double omega_step = 0.002;
  for (double omega = min_omega_; omega <= max_omega_ + 1e-9; omega += omega_step) {
    std::vector<double> weights(fit_data_.size(), 1.0);
    Eigen::Vector3d params;
    if (!fit_weighted_model(omega, weights, params)) continue;

    for (int iteration = 0; iteration < 4; ++iteration) {
      for (size_t i = 0; i < fit_data_.size(); ++i) {
        const double local_t = fit_data_[i].first - fit_data_.front().first;
        const double predicted = params[0] * std::sin(omega * local_t) +
                                 params[1] * std::cos(omega * local_t) + params[2];
        const double residual = std::abs(fit_data_[i].second - predicted);
        weights[i] = residual <= threshold_ ? 1.0 : threshold_ / std::max(residual, 1e-9);
      }
      if (!fit_weighted_model(omega, weights, params)) break;
    }

    const double A = std::hypot(params[0], params[1]);
    const double local_phi = std::atan2(params[1], params[0]);
    const double phi = local_phi - omega * fit_data_.front().first;
    const double C = params[2];
    if (A < 0.65 || A > 1.15 || C < 0.85 || C > 1.55) continue;

    Result result = evaluate_model(A, omega, phi, C);
    if (
      result.inliers > best.inliers ||
      (result.inliers == best.inliers && result.rms < best.rms)) {
      best = result;
    }
  }
  best_result_ = best;
}

void RansacSineFitter::clear()
{
  fit_data_.clear();
  best_result_ = Result{};
}

size_t RansacSineFitter::sample_count() const { return fit_data_.size(); }

double RansacSineFitter::time_span() const
{
  if (fit_data_.size() < 2) return 0.0;
  return fit_data_.back().first - fit_data_.front().first;
}

double RansacSineFitter::inlier_ratio() const
{
  if (fit_data_.empty()) return 0.0;
  return static_cast<double>(best_result_.inliers) / static_cast<double>(fit_data_.size());
}

bool RansacSineFitter::ready(
  size_t min_samples, double min_time_span, double min_inlier_ratio, double max_rms) const
{
  return sample_count() >= min_samples && time_span() >= min_time_span &&
         inlier_ratio() >= min_inlier_ratio && best_result_.rms <= max_rms &&
         best_result_.omega > 1e-6;
}

bool RansacSineFitter::fit_weighted_model(
  double omega, const std::vector<double> & weights, Eigen::Vector3d & params) const
{
  if (weights.size() != fit_data_.size()) return false;
  Eigen::MatrixXd X(fit_data_.size(), 3);
  Eigen::VectorXd Y(fit_data_.size());

  for (size_t i = 0; i < fit_data_.size(); ++i) {
    const double weight = std::sqrt(std::max(weights[i], 0.0));
    const double local_t = fit_data_[i].first - fit_data_.front().first;
    X(i, 0) = weight * std::sin(omega * local_t);
    X(i, 1) = weight * std::cos(omega * local_t);
    X(i, 2) = weight;
    Y(i) = weight * fit_data_[i].second;
  }

  try {
    params = X.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(Y);
    return true;
  } catch (...) {
    return false;
  }
}

RansacSineFitter::Result RansacSineFitter::evaluate_model(
  double A, double omega, double phi, double C) const
{
  Result result;
  result.A = A;
  result.omega = omega;
  result.phi = phi;
  result.C = C;
  double squared_error = 0.0;
  for (const auto & p : fit_data_) {
    const double predicted = A * std::sin(omega * p.first + phi) + C;
    const double residual = p.second - predicted;
    if (std::abs(residual) < threshold_) {
      result.inliers++;
      squared_error += residual * residual;
    }
  }
  if (result.inliers > 0) {
    result.rms = std::sqrt(squared_error / static_cast<double>(result.inliers));
  }
  return result;
}

}  // namespace tools
