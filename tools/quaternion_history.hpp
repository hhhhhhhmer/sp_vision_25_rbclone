#ifndef TOOLS__QUATERNION_HISTORY_HPP
#define TOOLS__QUATERNION_HISTORY_HPP

#include <Eigen/Geometry>
#include <algorithm>
#include <chrono>
#include <iterator>

namespace tools
{

/** @brief 带时间戳的云台姿态样本（IMU 四元数 + 本机收到的时刻） */
struct TimestampedQuaternion
{
  std::chrono::steady_clock::time_point t;
  Eigen::Quaterniond q = Eigen::Quaterniond::Identity();
};

/** @brief 在按时间升序的姿态样本上做 SLERP 插值查询
 *
 * 这是「图像时间戳 → 云台姿态」的唯一入口：图像与反馈都带有本机单调时钟时间戳，
 * 失配补偿就是在这里把查询时刻前移/后移一个 τ。
 *
 * @param begin 样本区间起点（含）
 * @param end 样本区间终点（不含）
 * @param t 查询时刻
 * @param covered 可选输出：查询时刻被样本区间真正覆盖（落在首尾样本之间）时为 true；
 *                越界被夹到首/尾样本时返回 false，调用方可据此判断数据是否够用
 * @return 查询时刻的姿态；样本为空时返回单位四元数
 */
template <typename Iterator>
Eigen::Quaterniond slerp_lookup(
  Iterator begin, Iterator end, std::chrono::steady_clock::time_point t, bool * covered = nullptr)
{
  if (covered) *covered = false;
  if (begin == end) return Eigen::Quaterniond::Identity();

  const auto last = std::prev(end);
  if (t <= begin->t) return begin->q;
  if (t >= last->t) return last->q;

  // 第一个 t_i >= t
  const auto it = std::lower_bound(
    begin, end, t, [](const TimestampedQuaternion & s, std::chrono::steady_clock::time_point v) {
      return s.t < v;
    });
  if (it->t == t) {
    if (covered) *covered = true;
    return it->q;
  }

  const auto prev = std::prev(it);
  const double span_s = std::chrono::duration<double>(it->t - prev->t).count();
  const double frac_s = std::chrono::duration<double>(t - prev->t).count();
  const double k = (span_s > 1e-12) ? (frac_s / span_s) : 0.0;
  if (covered) *covered = true;
  return prev->q.slerp(k, it->q).normalized();
}

}  // namespace tools

#endif  // TOOLS__QUATERNION_HISTORY_HPP
