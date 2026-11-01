#ifndef TOOLS__TIMESTAMP_CALIB_HPP
#define TOOLS__TIMESTAMP_CALIB_HPP

#include <Eigen/Core>
#include <chrono>
#include <cstddef>
#include <string>
#include <utility>
#include <vector>

#include "tools/quaternion_history.hpp"

namespace tools
{

/** @brief 时间戳标定的参数 */
struct TimestampCalibConfig
{
  /** @brief τ 搜索下界（s）。τ>0 表示图像的真实曝光时刻早于其时间戳 */
  double tau_min_s = -0.020;
  /** @brief τ 搜索上界（s） */
  double tau_max_s = 0.020;
  /** @brief τ 搜索步长（s） */
  double tau_step_s = 0.0001;
  /** @brief 离群剔除门限（MAD 倍数）：世界系位置偏离中位数超过该倍数的帧不参与拟合 */
  double outlier_sigma = 4.0;
  /** @brief 云台 RMS 角速度低于该值（rad/s）认为摆动不足，结果不可信 */
  double min_excitation_rad_s = 0.5;
  /** @brief 参与拟合的最少帧数 */
  int min_samples = 60;
  /** @brief 1σ 不确定度超过该值（s）认为结果不可信 */
  double max_sigma_s = 0.002;
};

/** @brief 时间戳标定结果 */
struct TimestampCalibResult
{
  bool ok = false;                 ///< 标定是否可信
  std::string reason;              ///< 不可信时的原因（可信时为空）
  double tau_s = 0.0;              ///< 最优 τ：姿态应在 t-τ 处查询
  double sigma_s = 0.0;            ///< τ 的 1σ 不确定度
  double tau_first_half_s = 0.0;   ///< 前半段单独拟合结果（一致性检查）
  double tau_second_half_s = 0.0;  ///< 后半段单独拟合结果（一致性检查）
  double rms_at_zero_m = 0.0;  ///< τ=0 时世界系位置的 RMS 离散度（不改代码时的水平）
  double rms_at_tau_m = 0.0;           ///< τ=τ̂ 时世界系位置的 RMS 离散度
  double excitation_rad_s = 0.0;       ///< 采样期间云台 RMS 角速度
  double excitation_peak_rad_s = 0.0;  ///< 采样期间云台峰值角速度
  int frames = 0;                      ///< 采集到的总帧数
  int used = 0;                        ///< 剔除离群后参与拟合的帧数
  int rejected = 0;                    ///< 被剔除的帧数
  double q_coverage_lo_s = 0.0;  ///< 实际可用的 τ 搜索下界（受姿态历史覆盖范围限制）
  double q_coverage_hi_s = 0.0;  ///< 实际可用的 τ 搜索上界
  /** @brief τ-RMS 曲线（等间隔抽样），用于打印/画图 */
  std::vector<std::pair<double, double>> curve;
};

/** @brief 图像—姿态时间失配 τ 的自标定器
 *
 * 原理：静止目标在云台系下的位置 p_g 与姿态无关（PnP + 手眼外参直接给出），
 * 而它在世界系下的位置 p_w(τ) = R(q(t-τ))·p_g 只有在 τ 取到真实值时才对时间保持不变。
 * 因此把「τ=0 时 p_w 的离散度」当作代价函数做一维搜索，即可反解出包含曝光、读出、
 * 传输、调度在内的全部固定延迟，不需要秒表、不需要知道曝光时间。
 *
 * 灵敏度 ∝ 云台角速度 × 目标距离：采样时必须用遥控让云台左右快速摆动（有角速度变化才有信息），
 * 静止不动时代价函数是平的。
 */
class TimestampCalibrator
{
public:
  /** @brief 清空已采集的样本 */
  void clear();
  /** @brief 追加一帧观测 @param t 图像时间戳 @param xyz_in_gimbal 装甲板在云台系下的位置（m），必须与姿态无关 */
  void add_sample(std::chrono::steady_clock::time_point t, const Eigen::Vector3d & xyz_in_gimbal);
  /** @brief 已采集的帧数 */
  std::size_t size() const { return samples_.size(); }

  /** @brief 用姿态历史拟合 τ
   *  @param q_history 覆盖整个采样时间段的姿态历史（按时间升序）
   *  @param cfg 拟合参数
   *  @return 标定结果；即使 ok=false 也填好可诊断的字段
   */
  TimestampCalibResult fit(
    const std::vector<TimestampedQuaternion> & q_history,
    const TimestampCalibConfig & cfg = TimestampCalibConfig()) const;

private:
  using Sample = std::pair<std::chrono::steady_clock::time_point, Eigen::Vector3d>;
  std::vector<Sample> samples_;
};

/** @brief 计算给定 τ 下世界系位置的 RMS 离散度
 *  @param samples 观测样本
 *  @param q_history 姿态历史
 *  @param tau_s 时间偏移（s）
 *  @param used 输出：有效帧数（姿态历史未覆盖的帧会被跳过）
 *  @return RMS 离散度（m）；有效帧不足 3 时返回 NaN
 */
double timestamp_rms_at(
  const std::vector<std::pair<std::chrono::steady_clock::time_point, Eigen::Vector3d>> & samples,
  const std::vector<TimestampedQuaternion> & q_history, double tau_s, int * used = nullptr);

/** @brief 自动标定用的云台摆动轨迹：先平滑对准到目标方位，再绕该方位做正弦摆动
 *
 * 纯数学、无副作用，现场程序按 200 Hz 采样它下发即可；单测里用它验证
 * 「对准连续、摆幅不越界、速度/加速度与位置自洽」。
 */
struct SweepProfile
{
  double yaw_now = 0.0;     ///< 起始 yaw（rad）
  double yaw_center = 0.0;  ///< 摆动中心（rad，绝对世界角）
  double amp = 0.0;         ///< 摆幅（rad）
  double freq = 1.0;        ///< 摆动频率（Hz）
  double align_s = 0.5;     ///< 对准时长（s）
  double sweep_s = 3.0;     ///< 摆动时长（s）

  /** @brief 某一时刻的指令 */
  struct Sample
  {
    double yaw = 0.0;   ///< 目标 yaw（rad）
    double vel = 0.0;   ///< 前馈角速度（rad/s）
    double acc = 0.0;   ///< 前馈角加速度（rad/s²）
    bool done = false;  ///< 摆动是否已结束（此时 yaw 停在中心角）
  };

  /** @brief 采样轨迹 @param t 自开始以来的时间（s，负数按 0 处理） @return 该时刻的指令 */
  Sample at(double t) const;

  /** @brief 轨迹总时长（对准 + 摆动，s） */
  double total_s() const { return align_s + sweep_s; }
};

}  // namespace tools

#endif  // TOOLS__TIMESTAMP_CALIB_HPP
