#include "timestamp_calib.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace tools
{
namespace
{

using Clock = std::chrono::steady_clock;
using Sample = std::pair<Clock::time_point, Eigen::Vector3d>;

constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();

/** @brief 两个时刻之差（秒），a 为较新时刻 @return a-b */
double duration_seconds(Clock::time_point a, Clock::time_point b)
{
  return std::chrono::duration<double>(a - b).count();
}

/** @brief 把秒数换算成可加减的时间偏移（四舍五入到微秒，避免浮点误差累积） */
Clock::duration to_duration(double seconds)
{
  return std::chrono::microseconds(static_cast<long long>(std::llround(seconds * 1e6)));
}

/** @brief 计算 τ 下每帧的世界系位置；姿态历史覆盖不到的帧填 NaN
 *  @note p_w = R(q(t-τ))·p_g，其中 p_g 与姿态无关，这正是能把 τ 反解出来的前提
 */
std::vector<Eigen::Vector3d> world_positions(
  const std::vector<Sample> & samples, const std::vector<TimestampedQuaternion> & q_history,
  double tau_s)
{
  std::vector<Eigen::Vector3d> pw;
  pw.reserve(samples.size());

  const auto offset = to_duration(tau_s);
  for (const auto & sample : samples) {
    bool covered = false;
    const auto q =
      slerp_lookup(q_history.begin(), q_history.end(), sample.first - offset, &covered);
    pw.push_back(
      covered ? Eigen::Vector3d(q.toRotationMatrix() * sample.second)
              : Eigen::Vector3d::Constant(kNaN));
  }
  return pw;
}

/** @brief 对给定帧集合求质心与平方残差和
 *  @param idx 参与计算的样本下标（调用方保证这些帧的 pw 均为有限值）
 *  @param pw 世界系位置（world_positions 的输出）
 *  @param mean 输出：质心
 *  @param sq 输出：每帧到质心的平方距离（与 idx 一一对齐）
 *  @return 平方残差和 J = Σ|p-mean|²；有效帧不足 3 时返回 NaN
 */
double residual_sum(
  const std::vector<int> & idx, const std::vector<Eigen::Vector3d> & pw, Eigen::Vector3d & mean,
  std::vector<double> & sq)
{
  mean.setZero();
  if (idx.size() < 3) return kNaN;
  for (int i : idx) mean += pw[i];
  mean /= static_cast<double>(idx.size());

  sq.clear();
  sq.reserve(idx.size());
  double J = 0;
  for (int i : idx) {
    const double d2 = (pw[i] - mean).squaredNorm();
    sq.push_back(d2);
    J += d2;
  }
  return J;
}

/** @brief 用中位数/MAD 剔除离群帧（目标被人挪动、误检测等） */
void reject_outliers(
  const std::vector<int> & idx, const std::vector<double> & sq, double outlier_sigma,
  std::vector<char> & inlier)
{
  if (idx.size() != sq.size() || sq.empty()) return;

  std::vector<double> d;
  d.reserve(sq.size());
  for (double v : sq) d.push_back(std::sqrt(v));

  std::vector<double> sorted = d;
  auto mid = sorted.begin() + static_cast<long>(sorted.size() / 2);
  std::nth_element(sorted.begin(), mid, sorted.end());
  const double median = *mid;

  for (double & v : d) v = std::abs(v - median);
  sorted = d;
  mid = sorted.begin() + static_cast<long>(sorted.size() / 2);
  std::nth_element(sorted.begin(), mid, sorted.end());
  const double mad = *mid * 1.4826;
  if (!(mad > 1e-9)) return;  // 残差几乎一致，不做剔除

  const double threshold = median + outlier_sigma * mad;
  for (std::size_t k = 0; k < idx.size(); ++k) {
    if (std::sqrt(sq[k]) > threshold) inlier[static_cast<std::size_t>(idx[k])] = 0;
  }
}

/** @brief 云台的角速度激励（RMS 与峰值），用于判断"有没有摆动" */
void excitation(
  const std::vector<Sample> & samples, const std::vector<TimestampedQuaternion> & q_history,
  double & rms, double & peak)
{
  rms = 0;
  peak = 0;
  const auto h = std::chrono::milliseconds(2);
  const double dt = std::chrono::duration<double>(h).count() * 2.0;

  double sum = 0;
  int n = 0;
  for (const auto & sample : samples) {
    bool covered_a = false, covered_b = false;
    const auto qa = slerp_lookup(q_history.begin(), q_history.end(), sample.first - h, &covered_a);
    const auto qb = slerp_lookup(q_history.begin(), q_history.end(), sample.first + h, &covered_b);
    if (!covered_a || !covered_b) continue;

    const double angle = Eigen::AngleAxisd(qa.conjugate() * qb).angle();
    const double w = angle / dt;
    sum += w * w;
    peak = std::max(peak, w);
    ++n;
  }

  rms = (n > 0) ? std::sqrt(sum / n) : 0.0;
}

/** @brief 单个数据子集上的 τ 一维搜索 */
struct SubFit
{
  bool valid = false;
  double tau = 0.0;
  double J = kNaN;
  double rms = kNaN;
  double sigma = kNaN;
  bool at_boundary = false;
  int used = 0;
  std::vector<char> inlier;                      ///< 与输入子集对齐的内点掩码
  std::vector<std::pair<double, double>> curve;  ///< (τ, RMS)，覆盖整个搜索区间
};

/** @brief 在 [lo, hi] 上均匀取 n 个点搜索 τ，返回最优值与曲线 */
struct SearchResult
{
  bool found = false;
  double tau = 0.0;
  double J = kNaN;
  int n = 0;
  std::vector<int> idx;
  std::vector<std::pair<double, double>> curve;
};

SearchResult search_grid(
  const std::vector<Sample> & samples, const std::vector<TimestampedQuaternion> & q_history,
  const std::vector<char> & inlier, double lo, double hi, int n_points)
{
  SearchResult best;
  if (n_points < 2) n_points = 2;

  const double step = (hi - lo) / static_cast<double>(n_points - 1);
  for (int k = 0; k < n_points; ++k) {
    const double tau = lo + static_cast<double>(k) * step;
    const auto pw = world_positions(samples, q_history, tau);

    std::vector<int> idx;
    idx.reserve(samples.size());
    for (std::size_t i = 0; i < samples.size(); ++i) {
      if (inlier[i] && pw[i].allFinite()) idx.push_back(static_cast<int>(i));
    }
    if (idx.size() < 3) continue;

    Eigen::Vector3d mean;
    std::vector<double> sq;
    const double J = residual_sum(idx, pw, mean, sq);
    if (!std::isfinite(J)) continue;

    best.curve.emplace_back(tau, std::sqrt(J / static_cast<double>(idx.size())));
    if (!best.found || J < best.J) {
      best.found = true;
      best.tau = tau;
      best.J = J;
      best.n = static_cast<int>(idx.size());
      best.idx = std::move(idx);
    }
  }
  return best;
}

SubFit fit_subrange(
  const std::vector<Sample> & samples, const std::vector<TimestampedQuaternion> & q_history,
  const TimestampCalibConfig & cfg, double tau_lo, double tau_hi, std::size_t begin,
  std::size_t end)
{
  SubFit fit;
  if (end <= begin || end - begin < 3) return fit;

  const std::vector<Sample> subset(
    samples.begin() + static_cast<long>(begin), samples.begin() + static_cast<long>(end));

  std::vector<char> inlier(subset.size(), 1);
  double tau = 0.0;
  SearchResult best;
  SearchResult last_coarse;

  // 交替进行「按残差剔除离群帧」和「粗搜 + 细搜」，两三轮即可稳定
  for (int iter = 0; iter < 3; ++iter) {
    // 1) 在当前 τ 处剔除离群帧
    {
      const auto pw = world_positions(subset, q_history, tau);
      std::vector<int> idx;
      idx.reserve(subset.size());
      for (std::size_t i = 0; i < subset.size(); ++i) {
        if (inlier[i] && pw[i].allFinite()) idx.push_back(static_cast<int>(i));
      }
      Eigen::Vector3d mean;
      std::vector<double> sq;
      residual_sum(idx, pw, mean, sq);
      reject_outliers(idx, sq, cfg.outlier_sigma, inlier);
    }

    // 2) 粗搜全区间，再在最优点附近细搜
    const auto coarse = search_grid(subset, q_history, inlier, tau_lo, tau_hi, 81);
    if (!coarse.found) return fit;

    const double half_step = (tau_hi - tau_lo) / 80.0;
    const double fine_lo = std::max(tau_lo, coarse.tau - half_step);
    const double fine_hi = std::min(tau_hi, coarse.tau + half_step);
    const auto fine = search_grid(subset, q_history, inlier, fine_lo, fine_hi, 41);
    if (!fine.found) return fit;

    last_coarse = coarse;
    best = fine;
    tau = fine.tau;
  }

  if (!best.found || best.idx.size() < 3) return fit;

  fit.valid = true;
  fit.used = best.n;
  fit.inlier = inlier;
  // 对外给全区间曲线（便于看形状），抛物线细化用细搜曲线
  fit.curve = last_coarse.curve;
  const auto & curve = best.curve;
  std::size_t kmin = 0;
  for (std::size_t k = 1; k < curve.size(); ++k) {
    if (curve[k].second < curve[kmin].second) kmin = k;
  }
  fit.at_boundary = (kmin == 0 || kmin + 1 >= curve.size());

  double tau_hat = curve[kmin].first;
  double J_hat = best.J;
  double curvature = kNaN;

  if (!fit.at_boundary) {
    const double n_used = static_cast<double>(best.n);
    const double j0 = curve[kmin - 1].second * curve[kmin - 1].second * n_used;
    const double j1 = curve[kmin].second * curve[kmin].second * n_used;
    const double j2 = curve[kmin + 1].second * curve[kmin + 1].second * n_used;
    const double denom = j0 - 2.0 * j1 + j2;
    if (denom > 1e-12) {
      const double step = curve[kmin + 1].first - curve[kmin].first;
      tau_hat += 0.5 * step * (j0 - j2) / denom;
      J_hat = j1 - 0.25 * (j0 - j2) * (j0 - j2) / denom;
      // J(τ) ≈ J_min + S(τ-τ̂)² ⇒ S = J''/2
      curvature = denom / (step * step) / 2.0;
    }
  }

  fit.tau = tau_hat;
  fit.J = J_hat;
  fit.rms = std::sqrt(std::max(0.0, J_hat) / static_cast<double>(best.n));

  if (std::isfinite(curvature) && curvature > 0) {
    // 每帧贡献 3 个坐标残差，自由度为 3N-3
    const double dof = std::max(1.0, 3.0 * static_cast<double>(best.n) - 3.0);
    const double sigma2 = std::max(0.0, J_hat) / dof;
    fit.sigma = std::sqrt(sigma2 / curvature);
  }

  return fit;
}

}  // namespace

void TimestampCalibrator::clear() { samples_.clear(); }

void TimestampCalibrator::add_sample(
  std::chrono::steady_clock::time_point t, const Eigen::Vector3d & xyz_in_gimbal)
{
  if (!xyz_in_gimbal.allFinite()) return;
  if (!samples_.empty() && t <= samples_.back().first) return;  // 时间必须单调递增
  samples_.emplace_back(t, xyz_in_gimbal);
}

TimestampCalibResult TimestampCalibrator::fit(
  const std::vector<TimestampedQuaternion> & q_history, const TimestampCalibConfig & cfg) const
{
  TimestampCalibResult result;
  result.frames = static_cast<int>(samples_.size());

  if (samples_.empty()) {
    result.reason = "没有采集到样本";
    return result;
  }
  if (q_history.size() < 2) {
    result.reason = "姿态历史为空（云台反馈未收到？）";
    return result;
  }
  if (result.frames < cfg.min_samples) {
    result.reason = "样本太少（需至少 " + std::to_string(cfg.min_samples) + " 帧）";
    return result;
  }

  // 姿态历史必须覆盖所有查询时刻 t-τ，否则边界处会被夹取，把 τ 拉向边界
  const double tau_lo =
    std::max(cfg.tau_min_s, -duration_seconds(q_history.back().t, samples_.back().first));
  const double tau_hi =
    std::min(cfg.tau_max_s, duration_seconds(samples_.front().first, q_history.front().t));
  result.q_coverage_lo_s = tau_lo;
  result.q_coverage_hi_s = tau_hi;

  if (tau_hi <= tau_lo) {
    result.reason = "姿态历史覆盖范围不足，无法搜索 τ";
    return result;
  }

  excitation(samples_, q_history, result.excitation_rad_s, result.excitation_peak_rad_s);

  const auto sub = fit_subrange(samples_, q_history, cfg, tau_lo, tau_hi, 0, samples_.size());
  if (!sub.valid) {
    result.reason = "拟合失败（有效帧不足）";
    return result;
  }

  result.tau_s = sub.tau;
  result.sigma_s = sub.sigma;
  result.used = sub.used;
  result.rejected = result.frames - sub.used;
  result.rms_at_tau_m = sub.rms;
  result.curve = sub.curve;

  // τ=0 时的离散度：对照「不改代码」的水平（用同一批内点，避免离群帧夸大改善）
  {
    const auto pw = world_positions(samples_, q_history, 0.0);
    std::vector<int> idx;
    idx.reserve(samples_.size());
    for (std::size_t i = 0; i < samples_.size() && i < sub.inlier.size(); ++i) {
      if (sub.inlier[i] && pw[i].allFinite()) idx.push_back(static_cast<int>(i));
    }
    Eigen::Vector3d mean;
    std::vector<double> sq;
    const double J0 = residual_sum(idx, pw, mean, sq);
    result.rms_at_zero_m =
      std::isfinite(J0) ? std::sqrt(J0 / static_cast<double>(idx.size())) : kNaN;
  }

  // 前后半段一致性：采样期间目标若被挪动/转动，两段会明显不一致
  const std::size_t half = samples_.size() / 2;
  const auto first = fit_subrange(samples_, q_history, cfg, tau_lo, tau_hi, 0, half);
  const auto second = fit_subrange(samples_, q_history, cfg, tau_lo, tau_hi, half, samples_.size());
  result.tau_first_half_s = first.valid ? first.tau : kNaN;
  result.tau_second_half_s = second.valid ? second.tau : kNaN;

  // 可信性判定
  if (sub.at_boundary) {
    result.reason = "τ̂ 落在搜索边界，真实值可能超出 ±20ms 或数据无区分度";
    return result;
  }
  if (result.excitation_rad_s < cfg.min_excitation_rad_s) {
    result.reason = "云台摆动不足（RMS |ω| 过低），请重新采样并左右快速摆云台";
    return result;
  }
  if (!std::isfinite(result.sigma_s) || result.sigma_s > cfg.max_sigma_s) {
    result.reason = "τ 的不确定度过大，数据不足以定出时间偏移";
    return result;
  }
  if (
    std::isfinite(result.tau_first_half_s) && std::isfinite(result.tau_second_half_s) &&
    std::abs(result.tau_first_half_s - result.tau_second_half_s) > 4.0 * cfg.max_sigma_s) {
    result.reason = "前后半段拟合结果不一致，采样期间目标可能被移动/转动了";
    return result;
  }

  result.ok = true;
  return result;
}

double timestamp_rms_at(
  const std::vector<std::pair<std::chrono::steady_clock::time_point, Eigen::Vector3d>> & samples,
  const std::vector<TimestampedQuaternion> & q_history, double tau_s, int * used)
{
  if (used) *used = 0;
  if (samples.empty() || q_history.size() < 2) return kNaN;

  const auto pw = world_positions(samples, q_history, tau_s);
  std::vector<int> idx;
  idx.reserve(samples.size());
  for (std::size_t i = 0; i < samples.size(); ++i) {
    if (pw[i].allFinite()) idx.push_back(static_cast<int>(i));
  }

  Eigen::Vector3d mean;
  std::vector<double> sq;
  const double J = residual_sum(idx, pw, mean, sq);
  if (!std::isfinite(J) || idx.empty()) return kNaN;

  if (used) *used = static_cast<int>(idx.size());
  return std::sqrt(J / static_cast<double>(idx.size()));
}

SweepProfile::Sample SweepProfile::at(double t) const
{
  Sample s;
  if (!(t > 0.0)) t = 0.0;

  if (align_s > 0.0 && t < align_s) {
    // 对准段：以恒定速度从当前角度过渡到目标方位，避免视觉刚接管时的一个阶跃
    const double k = t / align_s;
    s.yaw = yaw_now + (yaw_center - yaw_now) * k;
    s.vel = (yaw_center - yaw_now) / align_s;
    return s;
  }

  const double ts = t - align_s;
  if (ts >= sweep_s) {
    s.yaw = yaw_center;
    s.done = true;
    return s;
  }

  const double w = 2.0 * M_PI * freq;
  s.yaw = yaw_center + amp * std::sin(w * ts);
  s.vel = amp * w * std::cos(w * ts);
  s.acc = -amp * w * w * std::sin(w * ts);
  return s;
}

}  // namespace tools
