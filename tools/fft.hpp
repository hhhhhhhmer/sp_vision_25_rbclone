// periodic_motion_analyzer.hpp
#ifndef PERIODIC_MOTION_ANALYZER_HPP
#define PERIODIC_MOTION_ANALYZER_HPP

#include <chrono>
#include <cmath>
#include <cstddef>
#include <mutex>
#include <optional>

#include <Eigen/Dense>
#include <boost/circular_buffer.hpp>

namespace tools
{

/**
 * @brief 周期分量的不可变拟合快照
 *
 * 时间参数 double 与 FFTExample 保持一致：表示相对分析器时间原点的秒数。
 * 返回的 value 仅包含零均值周期分量，不包含装甲板高度偏置和线性趋势。
 */
struct Wave
{
  using TimePoint = std::chrono::steady_clock::time_point;

  bool is_periodic = false;
  double frequency = 0.0;              ///< Hz
  double amplitude = 0.0;
  double phase = 0.0;                  ///< rad，以 reference_time 为参考
  double phase_offset = 0.0;           ///< rad，外部设置的附加相位
  double reference_time = 0.0;         ///< 相对 time_origin 的秒数
  double fit_quality = 0.0;
  double signal_to_noise_ratio = 0.0;
  std::optional<TimePoint> time_origin;

  /** @brief 快照是否包含可用的周期波形 */
  bool valid() const;

  double get_value(double t) const;
  double get_velocity(double t) const;
  double get_acceleration(double t) const;

  double get_value(TimePoint t) const;
  double get_velocity(TimePoint t) const;
  double get_acceleration(TimePoint t) const;
};

/**
 * @brief 周期性运动分析器（基于FFT/正弦拟合）
 *
 * 该类接收时序采样数据（时间戳和数值），通过拟合正弦波模型
 *   value(t) = offset + trend*t + A*sin(2πf*t + φ)
 * 来判断数据是否呈现周期性，并提取频率、幅度、相位等信息。
 *
 * 支持多个数据源（通过 armor_id 区分），可分别拟合每个源的整体偏移量，
 * 但共享相同的周期信号成分（频率、幅度、相位）。
 *
 * 内部使用滑动窗口缓冲区，自动清理陈旧数据，并采用鲁棒拟合（Huber权重）
 * 以抵抗异常值。
 */
class FFTExample
{
public:
  using Buffer = boost::circular_buffer<double>;
  using TimePoint = std::chrono::steady_clock::time_point;

  /**
   * @brief 构造函数
   * @param max_points 缓冲区最大容量（默认1200）
   */
  FFTExample(std::size_t max_points = 1200)
  : t_buf_(max_points), val_buf_(max_points), id_buf_(max_points)
  {
  }

  // ---------- 数据添加接口 ----------
  /** @brief 添加样本（时间以秒为单位，不含armor_id） */
  void add_sample(double t, double val);
  /** @brief 添加样本（时间以秒为单位，含armor_id） */
  void add_sample(double t, int armor_id, double val);
  /** @brief 添加样本（使用steady时钟时间点，不含armor_id） */
  void add_sample(TimePoint t, double val);
  /** @brief 添加样本（使用steady时钟时间点，含armor_id） */
  void add_sample(TimePoint t, int armor_id, double val);

  /** @brief 清空所有内部状态，重置分析器 */
  void reset();

  // ---------- 低通滤波辅助 ----------
  /**
   * @brief 对输入值进行一阶低通滤波（指数平滑）
   * @param value 当前输入值
   * @param alpha 滤波系数 (0~1)，默认0.05
   * @return 滤波后的值
   */
  double low_pass(double value, double alpha = 0.05);
  /** @brief 重置低通滤波器的初始状态 */
  void reset_low_pass();

  // ---------- 周期性分析 ----------
  /**
   * @brief 执行周期性分析（正弦波拟合）
   * @param force 若为true，则忽略最小窗口时长限制，强制分析
   * @return 当前是否为周期性状态（true表示周期运动已确认）
   *
   * 内部会检查数据量、时间跨度，进行多频率扫描和鲁棒拟合，
   * 并通过连续多次验证（滞后机制）来稳定判定结果。
   */
  bool analyze(bool force = false);

  // ---------- 查询接口 ----------
  /**
   * @brief 在指定时间点计算加速度（二阶导数）
   * @param t 时间（秒，相对于内部原点）
   * @return 加速度值（仅当周期性时有效）
   */
  double get_acceleration(double t) const;
  /** @brief 使用 steady_clock 时间点计算加速度 */
  double get_acceleration(TimePoint t) const;
  /**
   * @brief 在指定时间点获取预测值（根据拟合的正弦模型）
   * @param t 时间（秒）
   * @return 预测值
   */
  double get_value(double t) const;
  /**
   * @brief 使用 steady_clock 时间点获取预测值
   */
  double get_value(TimePoint t) const;
  /** @brief 获取缓冲区中最旧的值 */
  double get_val_buf_front() const;
  /** @brief 获取缓冲区中最新的值 */
  double get_latest_value() const;

  /** @brief 是否判定为周期性运动 */
  bool get_is_periodic() const;
  /** @brief 设置相位偏移（用于调整预测相位） */
  void set_phase_offset(double offset);
  /** @brief 获取拟合均值（offset） */
  double get_mean_val() const;
  /** @brief 获取频率（Hz） */
  double get_frequency() const;
  /** @brief 获取幅度 */
  double get_amplitude() const;
  /** @brief 获取拟合质量（0~1，越接近1越好） */
  double get_fit_quality() const;
  /** @brief 获取信噪比（信号功率/噪声功率） */
  double get_signal_to_noise_ratio() const;
  /** @brief 返回当前拟合波形的线程安全值快照 */
  Wave get_wave() const;

private:
  // ---------- 缓冲区 ----------
  Buffer t_buf_;          ///< 时间缓冲区（秒）
  Buffer val_buf_;        ///< 数值缓冲区
  boost::circular_buffer<int> id_buf_;  ///< 数据源ID缓冲区

  mutable std::mutex mtx_;  ///< 互斥锁，保护所有成员

  // ---------- 周期判定状态 ----------
  bool is_periodic_ = false;       ///< 当前是否判定为周期性
  double last_freq_ = 0.0;         ///< 最近一次确认的频率 (Hz)
  double last_amp_ = 0.0;          ///< 最近一次确认的幅度
  double last_phase_ = 0.0;        ///< 最近一次确认的相位 (rad)
  double last_period_ = 0.0;       ///< 周期 (秒)
  double phase_offset_ = 0.0;      ///< 人为相位偏移
  double fit_reference_time_ = 0.0;///< 拟合模型的时间参考点（用于预测）
  double fit_quality_ = 0.0;       ///< 最近一次拟合质量
  double signal_to_noise_ratio_ = 0.0; ///< 信噪比

  // ---------- 配置参数 ----------
  std::size_t min_points_ = 60;          ///< 最小数据点数（20 FPS时约3秒）
  double window_seconds_ = 5.0;          ///< 滑动窗口时长（秒）
  double min_window_seconds_ = 2.5;      ///< 强制分析所需最小窗口时长
  double max_sample_gap_seconds_ = 0.5;  ///< 最大允许采样间隔（超过则重置）
  double stale_timeout_seconds_ = 0.5;   ///< 数据超时阈值（无新数据则重置）
  double min_frequency_ = 0.3;           ///< 搜索频率下限 (Hz)
  double max_frequency_ = 3.0;           ///< 搜索频率上限 (Hz)
  double min_amplitude_ = 0.055;         ///< 周期运动的最小拟合幅值
  double min_observed_cycles_ = 1.5;     ///< 首次确认至少覆盖的周期数
  std::size_t periodic_confirmations_ = 2;    ///< 首次确认需要连续通过的分析次数
  std::size_t nonperiodic_confirmations_ = 4; ///< 周期消失需要连续失败的分析次数

  // ---------- 辅助状态 ----------
  double mean_val = 0.0;                 ///< 拟合均值（直流偏移）
  bool low_pass_initialized_ = false;    ///< 低通滤波器是否已初始化
  double low_pass_value_ = 0.0;          ///< 低通滤波器当前值
  std::optional<TimePoint> time_origin_; ///< 时间原点（用于内部时间转换）
  std::optional<TimePoint> last_sample_time_; ///< 最后一次采样时间
  std::optional<double> candidate_frequency_; ///< 候选频率（用于滞后确认）
  std::size_t consecutive_periodic_count_ = 0;    ///< 连续周期性计数
  std::size_t consecutive_nonperiodic_count_ = 0; ///< 连续非周期性计数
  std::size_t sample_generation_ = 0;   ///< 样本代数（用于检测重置）

  // ---------- 内部辅助方法 ----------
  /** 带锁的样本添加（内部使用） */
  void add_sample_locked(double t, int armor_id, double val);
  /** 重置（内部，不加锁） */
  void reset_locked();
  /** 在给定时间（相对时间）计算预测值（不加锁） */
  double value_at_locked(double elapsed) const;
  /** 构造当前波形快照（不加锁） */
  Wave wave_locked() const;
};

}  // namespace tools

#endif  // PERIODIC_MOTION_ANALYZER_HPP
