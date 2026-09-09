// fft.cpp
#include "fft.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>
#include <unordered_map>
#include <utility>
#include <vector>

namespace tools
{

  bool Wave::valid() const
{
  return is_periodic && std::isfinite(frequency) && frequency > 0.0 &&
         std::isfinite(amplitude) && amplitude >= 0.0 && std::isfinite(phase) &&
         std::isfinite(phase_offset) && std::isfinite(reference_time);
}

double Wave::get_value(double t) const
{
  if (!valid() || !std::isfinite(t)) return 0.0;
  const double omega = 2.0 * M_PI * frequency;
  return amplitude * std::cos(omega * (t - reference_time) + phase + phase_offset);
}

double Wave::get_velocity(double t) const
{
  if (!valid() || !std::isfinite(t)) return 0.0;
  const double omega = 2.0 * M_PI * frequency;
  return -omega * amplitude *
         std::sin(omega * (t - reference_time) + phase + phase_offset);
}

double Wave::get_acceleration(double t) const
{
  if (!valid() || !std::isfinite(t)) return 0.0;
  const double omega = 2.0 * M_PI * frequency;
  return -omega * omega * get_value(t);
}

double Wave::get_value(TimePoint t) const
{
  if (!time_origin) return 0.0;
  return get_value(std::chrono::duration<double>(t - *time_origin).count());
}

double Wave::get_velocity(TimePoint t) const
{
  if (!time_origin) return 0.0;
  return get_velocity(std::chrono::duration<double>(t - *time_origin).count());
}

double Wave::get_acceleration(TimePoint t) const
{
  if (!time_origin) return 0.0;
  return get_acceleration(std::chrono::duration<double>(t - *time_origin).count());
}


namespace
{

/**
 * @brief 拟合结果结构体
 */
struct FrequencyFit
{
  double frequency = 0.0;                             ///< 频率 (Hz)
  double sse = std::numeric_limits<double>::infinity(); ///< 残差平方和
  Eigen::VectorXd coefficients;                      ///< 拟合系数
  Eigen::VectorXd prediction;                        ///< 拟合预测值
};

/**
 * @brief 在给定频率下执行最小二乘拟合
 *
 * 设计矩阵包含：
 *   - 每个armor_id的独立偏置（one-hot编码）
 *   - 时间趋势项（t）
 *   - 正弦项 sin(2πf t)
 *   - 余弦项 cos(2πf t)
 *
 * @param time 时间向量（相对参考点）
 * @param values 观测值向量
 * @param id_columns 每个样本对应的armor_id列索引（已映射）
 * @param id_count 不同armor_id的数量
 * @param frequency 测试频率
 * @param weights 可选加权向量（用于鲁棒拟合）
 * @return 包含拟合结果的结构体
 */
FrequencyFit fit_frequency(
  const Eigen::VectorXd & time, const Eigen::VectorXd & values,
  const std::vector<int> & id_columns, int id_count, double frequency,
  const Eigen::VectorXd * weights = nullptr)
{
  const Eigen::Index sample_count = values.size();
  Eigen::MatrixXd design = Eigen::MatrixXd::Zero(sample_count, id_count + 3);
  const double omega = 2.0 * M_PI * frequency;
  for (Eigen::Index i = 0; i < sample_count; ++i) {
    design(i, id_columns[static_cast<std::size_t>(i)]) = 1.0; // 类别偏置
    design(i, id_count) = time[i];                          // 趋势项
    design(i, id_count + 1) = std::sin(omega * time[i]);    // 正弦
    design(i, id_count + 2) = std::cos(omega * time[i]);    // 余弦
  }

  // 若有权重，对设计矩阵和观测值进行缩放
  Eigen::MatrixXd solve_design = design;
  Eigen::VectorXd solve_values = values;
  if (weights) {
    for (Eigen::Index i = 0; i < sample_count; ++i) {
      const double scale = std::sqrt(std::max(0.0, (*weights)[i]));
      solve_design.row(i) *= scale;
      solve_values[i] *= scale;
    }
  }

  // 求解最小二乘
  FrequencyFit fit;
  fit.frequency = frequency;
  fit.coefficients = solve_design.colPivHouseholderQr().solve(solve_values);
  fit.prediction = design * fit.coefficients;
  fit.sse = (values - fit.prediction).squaredNorm();
  return fit;
}

/**
 * @brief 计算残差绝对值的中位数（用于鲁棒尺度估计）
 */
double median_absolute_residual(const Eigen::VectorXd & residual)
{
  std::vector<double> absolute_values(static_cast<std::size_t>(residual.size()));
  for (Eigen::Index i = 0; i < residual.size(); ++i) {
    absolute_values[static_cast<std::size_t>(i)] = std::abs(residual[i]);
  }
  const auto middle = absolute_values.begin() + absolute_values.size() / 2;
  std::nth_element(absolute_values.begin(), middle, absolute_values.end());
  return *middle;
}

}  // namespace



// ---------- 数据添加接口（对外） ----------
void FFTExample::add_sample(double t, double val) { add_sample(t, 0, val); }

void FFTExample::add_sample(double t, int armor_id, double val)
{
  std::lock_guard<std::mutex> lock(mtx_);
  if (!std::isfinite(t) || !std::isfinite(val)) return;
  if (!t_buf_.empty()) {
    const double gap = t - t_buf_.back();
    if (gap <= 0.0) return; // 时间必须递增
    if (gap > max_sample_gap_seconds_) reset_locked(); // 间隔过大则重置
  }
  add_sample_locked(t, armor_id, val);
}

void FFTExample::add_sample(TimePoint t, double val) { add_sample(t, 0, val); }

void FFTExample::add_sample(TimePoint t, int armor_id, double val)
{
  std::lock_guard<std::mutex> lock(mtx_);
  if (!std::isfinite(val)) return;

  if (last_sample_time_) {
    const double gap = std::chrono::duration<double>(t - *last_sample_time_).count();
    if (gap <= 0.0) return;
    if (gap > max_sample_gap_seconds_) reset_locked();
  }
  if (!time_origin_) time_origin_ = t; // 设置时间原点

  const double elapsed = std::chrono::duration<double>(t - *time_origin_).count();
  add_sample_locked(elapsed, armor_id, val);
  last_sample_time_ = t;
}

void FFTExample::add_sample_locked(double t, int armor_id, double val)
{
  t_buf_.push_back(t);
  val_buf_.push_back(val);
  id_buf_.push_back(armor_id);

  // 移除窗口外的旧数据
  const double oldest_allowed = t - window_seconds_;
  while (!t_buf_.empty() && t_buf_.front() < oldest_allowed) {
    t_buf_.pop_front();
    val_buf_.pop_front();
    id_buf_.pop_front();
  }
}

// ---------- 重置 ----------
void FFTExample::reset()
{
  std::lock_guard<std::mutex> lock(mtx_);
  reset_locked();
}

void FFTExample::reset_locked()
{
  t_buf_.clear();
  val_buf_.clear();
  id_buf_.clear();
  is_periodic_ = false;
  last_freq_ = 0.0;
  last_amp_ = 0.0;
  last_phase_ = 0.0;
  last_period_ = 0.0;
  fit_reference_time_ = 0.0;
  fit_quality_ = 0.0;
  signal_to_noise_ratio_ = 0.0;
  mean_val = 0.0;
  low_pass_initialized_ = false;
  low_pass_value_ = 0.0;
  time_origin_.reset();
  last_sample_time_.reset();
  candidate_frequency_.reset();
  consecutive_periodic_count_ = 0;
  consecutive_nonperiodic_count_ = 0;
  ++sample_generation_; // 递增代数，使旧分析结果失效
}

// ---------- 低通滤波 ----------
double FFTExample::low_pass(double value, double alpha)
{
  std::lock_guard<std::mutex> lock(mtx_);
  alpha = std::clamp(alpha, 0.0, 1.0);
  if (!low_pass_initialized_) {
    low_pass_value_ = value;
    low_pass_initialized_ = true;
  } else {
    low_pass_value_ = alpha * value + (1.0 - alpha) * low_pass_value_;
  }
  return low_pass_value_;
}

void FFTExample::reset_low_pass()
{
  std::lock_guard<std::mutex> lock(mtx_);
  low_pass_initialized_ = false;
  low_pass_value_ = 0.0;
}

// ---------- 预测计算（内部） ----------
double FFTExample::value_at_locked(double elapsed) const
{
  return wave_locked().get_value(elapsed);
}

Wave FFTExample::wave_locked() const
{
  Wave wave;
  wave.is_periodic = is_periodic_;
  wave.frequency = last_freq_;
  wave.amplitude = last_amp_;
  wave.phase = last_phase_;
  wave.phase_offset = phase_offset_;
  wave.reference_time = fit_reference_time_;
  wave.fit_quality = fit_quality_;
  wave.signal_to_noise_ratio = signal_to_noise_ratio_;
  wave.time_origin = time_origin_;
  return wave;
}

double FFTExample::get_acceleration(double t) const
{
  std::lock_guard<std::mutex> lock(mtx_);
  return wave_locked().get_acceleration(t);
}

double FFTExample::get_acceleration(TimePoint t) const
{
  std::lock_guard<std::mutex> lock(mtx_);
  return wave_locked().get_acceleration(t);
}

double FFTExample::get_value(double t) const
{
  std::lock_guard<std::mutex> lock(mtx_);
  return value_at_locked(t);
}

double FFTExample::get_value(TimePoint t) const
{
  std::lock_guard<std::mutex> lock(mtx_);
  return wave_locked().get_value(t);
}

double FFTExample::get_val_buf_front() const
{
  std::lock_guard<std::mutex> lock(mtx_);
  return val_buf_.empty() ? 0.0 : val_buf_.front();
}

double FFTExample::get_latest_value() const
{
  std::lock_guard<std::mutex> lock(mtx_);
  return val_buf_.empty() ? 0.0 : val_buf_.back();
}

bool FFTExample::get_is_periodic() const
{
  std::lock_guard<std::mutex> lock(mtx_);
  return is_periodic_;
}

void FFTExample::set_phase_offset(double offset)
{
  std::lock_guard<std::mutex> lock(mtx_);
  phase_offset_ = offset;
}

double FFTExample::get_mean_val() const
{
  std::lock_guard<std::mutex> lock(mtx_);
  return mean_val;
}

double FFTExample::get_frequency() const
{
  std::lock_guard<std::mutex> lock(mtx_);
  return last_freq_;
}

double FFTExample::get_amplitude() const
{
  std::lock_guard<std::mutex> lock(mtx_);
  return last_amp_;
}

double FFTExample::get_fit_quality() const
{
  std::lock_guard<std::mutex> lock(mtx_);
  return fit_quality_;
}

double FFTExample::get_signal_to_noise_ratio() const
{
  std::lock_guard<std::mutex> lock(mtx_);
  return signal_to_noise_ratio_;
}

Wave FFTExample::get_wave() const
{
  std::lock_guard<std::mutex> lock(mtx_);
  return wave_locked();
}

// ---------- 核心分析函数 ----------
bool FFTExample::analyze(bool force)
{
  // 从缓冲区拷贝数据（加锁保护）
  std::vector<double> t_vec;
  std::vector<double> val_vec;
  std::vector<int> id_vec;
  std::size_t generation = 0;
  {
    std::lock_guard<std::mutex> lock(mtx_);
    // 检查数据是否超时（无新数据）
    if (
      last_sample_time_ &&
      std::chrono::duration<double>(TimePoint::clock::now() - *last_sample_time_).count() >
        stale_timeout_seconds_) {
      reset_locked();
      return false;
    }
    if (t_buf_.size() < min_points_) return is_periodic_;
    const double duration = t_buf_.back() - t_buf_.front();
    if (!force && duration < min_window_seconds_) return is_periodic_;

    t_vec.assign(t_buf_.begin(), t_buf_.end());
    val_vec.assign(val_buf_.begin(), val_buf_.end());
    id_vec.assign(id_buf_.begin(), id_buf_.end());
    generation = sample_generation_;
  }

  const Eigen::Index sample_count = static_cast<Eigen::Index>(val_vec.size());
  const double reference_time = t_vec.back(); // 以最后一个点为参考
  Eigen::VectorXd relative_time(sample_count);
  Eigen::VectorXd values(sample_count);
  std::unordered_map<int, int> id_to_column;
  std::vector<int> id_columns(val_vec.size());
  for (Eigen::Index i = 0; i < sample_count; ++i) {
    relative_time[i] = t_vec[static_cast<std::size_t>(i)] - reference_time;
    values[i] = val_vec[static_cast<std::size_t>(i)];
    const int armor_id = id_vec[static_cast<std::size_t>(i)];
    const auto insertion =
      id_to_column.emplace(armor_id, static_cast<int>(id_to_column.size()));
    id_columns[static_cast<std::size_t>(i)] = insertion.first->second;
  }
  const int id_count = static_cast<int>(id_to_column.size());

  // ---------- 基线模型（无周期信号，仅偏置+趋势） ----------
  Eigen::MatrixXd baseline = Eigen::MatrixXd::Zero(sample_count, id_count + 1);
  for (Eigen::Index i = 0; i < sample_count; ++i) {
    baseline(i, id_columns[static_cast<std::size_t>(i)]) = 1.0;
    baseline(i, id_count) = relative_time[i];
  }
  const Eigen::VectorXd baseline_coefficients = baseline.colPivHouseholderQr().solve(values);
  const double baseline_sse = (values - baseline * baseline_coefficients).squaredNorm();

  // ---------- 频率扫描（粗扫 + 精扫） ----------
  FrequencyFit best_fit;
  std::vector<std::pair<double, double>> coarse_scores;
  if (baseline_sse > 1e-9) {
    // 粗扫，步长0.01Hz
    for (double frequency = min_frequency_; frequency <= max_frequency_ + 1e-9;
         frequency += 0.01) {
      auto fit = fit_frequency(relative_time, values, id_columns, id_count, frequency);
      coarse_scores.emplace_back(frequency, fit.sse);
      if (fit.sse < best_fit.sse) best_fit = std::move(fit);
    }

    // 精扫，在最佳频率附近 ±0.02Hz，步长0.001Hz
    const double refine_min = std::max(min_frequency_, best_fit.frequency - 0.02);
    const double refine_max = std::min(max_frequency_, best_fit.frequency + 0.02);
    for (double frequency = refine_min; frequency <= refine_max + 1e-9; frequency += 0.001) {
      auto fit = fit_frequency(relative_time, values, id_columns, id_count, frequency);
      if (fit.sse < best_fit.sse) best_fit = std::move(fit);
    }
  }

  // ---------- 鲁棒加权（Huber回归，迭代3次） ----------
  if (std::isfinite(best_fit.sse)) {
    Eigen::VectorXd weights = Eigen::VectorXd::Ones(sample_count);
    for (int iteration = 0; iteration < 3; ++iteration) {
      const Eigen::VectorXd residual = values - best_fit.prediction;
      const double robust_sigma =
        std::max(1e-6, 1.4826 * median_absolute_residual(residual));
      const double huber_limit = 1.5 * robust_sigma;
      for (Eigen::Index i = 0; i < sample_count; ++i) {
        const double absolute_residual = std::abs(residual[i]);
        weights[i] = absolute_residual <= huber_limit ? 1.0 : huber_limit / absolute_residual;
      }
      best_fit = fit_frequency(
        relative_time, values, id_columns, id_count, best_fit.frequency, &weights);
    }
  }

  // ---------- 评估拟合质量 ----------
  const double improvement = baseline_sse - best_fit.sse;
  const double fit_quality = baseline_sse > 1e-9 ? improvement / baseline_sse : 0.0;
  const double signal_to_noise_ratio =
    best_fit.sse > 1e-9 ? improvement / best_fit.sse : std::numeric_limits<double>::infinity();
  // 计算次优频率的改进（用于判断峰值是否显著）
  double second_best_improvement = 0.0;
  for (const auto & [frequency, sse] : coarse_scores) {
    if (std::abs(frequency - best_fit.frequency) < 0.2) continue;
    second_best_improvement = std::max(second_best_improvement, baseline_sse - sse);
  }
  const double peak_ratio =
    second_best_improvement > 1e-9 ? improvement / second_best_improvement
                                   : std::numeric_limits<double>::infinity();

  // 提取幅度和相位（从正弦/余弦系数）
  double amplitude = 0.0;
  double phase = 0.0;
  if (best_fit.coefficients.size() >= id_count + 3) {
    const double sin_coefficient = best_fit.coefficients[id_count + 1];
    const double cos_coefficient = best_fit.coefficients[id_count + 2];
    amplitude = std::hypot(sin_coefficient, cos_coefficient);
    phase = std::atan2(-sin_coefficient, cos_coefficient);
  }

  const double duration = t_vec.back() - t_vec.front();
  // 有效性判据
  const bool valid =
    std::isfinite(best_fit.sse) && fit_quality >= 0.55 && signal_to_noise_ratio >= 1.2 &&
    peak_ratio >= 1.05 && amplitude >= min_amplitude_ && amplitude <= 0.5 &&
    duration * best_fit.frequency >= min_observed_cycles_;

  // ---------- 更新内部状态（加锁） ----------
  std::lock_guard<std::mutex> lock(mtx_);
  if (generation != sample_generation_) return is_periodic_; // 数据已重置

  if (!valid) {
    candidate_frequency_.reset();
    consecutive_periodic_count_ = 0;
    ++consecutive_nonperiodic_count_;
    if (consecutive_nonperiodic_count_ >= nonperiodic_confirmations_) {
      is_periodic_ = false;
      fit_quality_ = fit_quality;
      signal_to_noise_ratio_ = signal_to_noise_ratio;
    }
    return is_periodic_;
  }

  // 滞后确认：频率稳定才正式判定为周期性
  if (candidate_frequency_ && std::abs(*candidate_frequency_ - best_fit.frequency) <= 0.08) {
    ++consecutive_periodic_count_;
  } else {
    candidate_frequency_ = best_fit.frequency;
    consecutive_periodic_count_ = 1;
  }
  consecutive_nonperiodic_count_ = 0;

  const bool may_update_existing =
    is_periodic_ && std::abs(last_freq_ - best_fit.frequency) <= 0.15;
  // 若连续两次确认，或已有周期性且频率相近，则更新
  if (consecutive_periodic_count_ >= periodic_confirmations_ || may_update_existing) {
    is_periodic_ = true;
    last_freq_ = best_fit.frequency;
    last_amp_ = amplitude;
    last_phase_ = phase;
    last_period_ = 1.0 / best_fit.frequency;
    fit_reference_time_ = reference_time;
    fit_quality_ = fit_quality;
    signal_to_noise_ratio_ = signal_to_noise_ratio;
    mean_val = std::accumulate(val_vec.begin(), val_vec.end(), 0.0) / val_vec.size();
  }
  return is_periodic_;
}

}  // namespace tools
