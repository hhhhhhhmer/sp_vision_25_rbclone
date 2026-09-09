#include "extended_kalman_filter.hpp"

#include <array>
#include <cmath>
#include <numeric>
#include <stdexcept>

namespace
{
double chi_square_95_upper_threshold(Eigen::Index degrees_of_freedom)
{
  // Exact 95th percentiles for every observation dimension currently used by this project.
  constexpr std::array<double, 4> thresholds{3.841459, 5.991465, 7.814728, 9.487729};

  if (degrees_of_freedom <= 0) {
    throw std::invalid_argument("NIS requires a non-empty observation vector");
  }
  if (degrees_of_freedom <= static_cast<Eigen::Index>(thresholds.size())) {
    return thresholds[degrees_of_freedom - 1];
  }

  // Wilson-Hilferty approximation keeps the generic EKF usable for larger observations.
  constexpr double normal_95_quantile = 1.6448536269514722;
  const double dof = static_cast<double>(degrees_of_freedom);
  const double transformed =
    1.0 - 2.0 / (9.0 * dof) + normal_95_quantile * std::sqrt(2.0 / (9.0 * dof));
  return dof * transformed * transformed * transformed;
}
}  // namespace

namespace tools
{
ExtendedKalmanFilter::ExtendedKalmanFilter(
  const Eigen::VectorXd & x0, const Eigen::MatrixXd & P0,
  std::function<Eigen::VectorXd(const Eigen::VectorXd &, const Eigen::VectorXd &)> x_add)
: x(x0), P(P0), I(Eigen::MatrixXd::Identity(x0.rows(), x0.rows())), x_add(x_add)
{
  data["residual_yaw"] = 0.0;
  data["residual_pitch"] = 0.0;
  data["residual_distance"] = 0.0;
  data["residual_angle"] = 0.0;
  data["nis"] = 0.0;
  data["nees"] = 0.0;
  data["nis_fail"] = 0.0;
  data["nees_fail"] = 0.0;
  data["recent_nis_failures"] = 0.0;
}

Eigen::VectorXd ExtendedKalmanFilter::predict(const Eigen::MatrixXd & F, const Eigen::MatrixXd & Q)
{
  return predict(F, Q, [&](const Eigen::VectorXd & x) { return F * x; });
}

Eigen::VectorXd ExtendedKalmanFilter::predict(
  const Eigen::MatrixXd & F, const Eigen::MatrixXd & Q,
  std::function<Eigen::VectorXd(const Eigen::VectorXd &)> f)
{
  P = F * P * F.transpose() + Q;
  x = f(x);
  return x;
}

Eigen::VectorXd ExtendedKalmanFilter::update(
  const Eigen::VectorXd & z, const Eigen::MatrixXd & H, const Eigen::MatrixXd & R,
  std::function<Eigen::VectorXd(const Eigen::VectorXd &, const Eigen::VectorXd &)> z_subtract)
{
  return update(z, H, R, [&](const Eigen::VectorXd & x) { return H * x; }, z_subtract);
}

Eigen::VectorXd ExtendedKalmanFilter::update(
  const Eigen::VectorXd & z, const Eigen::MatrixXd & H, const Eigen::MatrixXd & R,
  std::function<Eigen::VectorXd(const Eigen::VectorXd &)> h,
  std::function<Eigen::VectorXd(const Eigen::VectorXd &, const Eigen::VectorXd &)> z_subtract)
{
  Eigen::VectorXd x_prior = x;

  // NIS must use the innovation and covariance from the predicted (prior) state.
  Eigen::VectorXd residual = z_subtract(z, h(x_prior));
  Eigen::MatrixXd PHt = P * H.transpose();
  Eigen::MatrixXd S = H * PHt + R;
  Eigen::LDLT<Eigen::MatrixXd> s_ldlt(S);
  if (s_ldlt.info() != Eigen::Success || !s_ldlt.isPositive()) {
    throw std::runtime_error("Innovation covariance is not positive definite in EKF update");
  }

  double nis = residual.dot(s_ldlt.solve(residual));
  double nis_threshold = chi_square_95_upper_threshold(residual.size());
  bool nis_failed = nis > nis_threshold;

  Eigen::MatrixXd K = s_ldlt.solve(PHt.transpose()).transpose();

  // Stable Compution of the Posterior Covariance
  // https://github.com/rlabbe/Kalman-and-Bayesian-Filters-in-Python/blob/master/07-Kalman-Filter-Math.ipynb
  P = (I - K * H) * P * (I - K * H).transpose() + K * R * K.transpose();

  x = x_add(x_prior, K * residual);

  /// 卡方检验
  double nees = (x - x_prior).transpose() * P.inverse() * (x - x_prior);

  // This is retained as a legacy diagnostic; x - x_prior is not a standard NEES error.
  constexpr double nees_threshold = 0.711;

  data["nis_fail"] = nis_failed ? 1.0 : 0.0;
  if (nis_failed) nis_count_++;
  if (nees > nees_threshold) nees_count_++, data["nees_fail"] = 1;
  total_count_++;
  last_nis = nis;

  recent_nis_failures.push_back(nis_failed ? 1 : 0);

  if (recent_nis_failures.size() > window_size) {
    recent_nis_failures.pop_front();
  }

  int recent_failures = std::accumulate(recent_nis_failures.begin(), recent_nis_failures.end(), 0);
  double recent_rate = static_cast<double>(recent_failures) / recent_nis_failures.size();

  data["residual_yaw"] = residual.size() > 0 ? residual[0] : 0.0;
  data["residual_pitch"] = residual.size() > 1 ? residual[1] : 0.0;
  data["residual_distance"] = residual.size() > 2 ? residual[2] : 0.0;
  data["residual_angle"] = residual.size() > 3 ? residual[3] : 0.0;
  data["nis"] = nis;
  data["nees"] = nees;
  data["recent_nis_failures"] = recent_rate;

  return x;
}

}  // namespace tools
