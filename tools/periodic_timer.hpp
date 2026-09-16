#ifndef TOOLS__PERIODIC_TIMER_HPP
#define TOOLS__PERIODIC_TIMER_HPP

#include <chrono>
#include <cstdint>
#include <thread>

#ifdef __linux__
#include <sys/prctl.h>
#endif

namespace tools
{

/**
 * @brief 绝对时刻周期调度器：让循环节拍由时钟决定，而不是"工作耗时 + sleep"
 *
 * 典型用法：
 * @code
 *   tools::PeriodicTimer tick(std::chrono::microseconds(5000));   // 200 Hz
 *   while (running) {
 *     tick.wait_next();     // 睡到下一个绝对节拍点
 *     do_work();            // 耗时可变，但不会累积进节拍
 *   }
 * @endcode
 *
 * 语义（与 std::this_thread::sleep_for 的关键差别）：
 *  - 节拍点固定为 t0 + k·period（steady_clock，单调时钟），与单轮工作耗时无关；
 *    sleep_for 的周期是 "工作耗时 + period"，工作耗时抖动会 1:1 传到发送间隔上。
 *  - 若某一轮工作超过一个周期，**不补发、不爆发**：直接对齐到下一个未来节拍点，
 *    并把跳过的节拍数计入 missed()。对控制指令流而言，"少发一拍"远好于"连发两拍"。
 *  - 假设 period 恒正；不处理跨进程时间同步。
 *
 * @note 该调度器只保证"被唤醒的时刻"贴近节拍点，实际抖动还取决于内核定时器精度、
 *       CFS 调度延迟和同机负载。配合 tighten_timer_slack() 与实时优先级（见
 *       docs/send_rate_stability.md）可把典型抖动压到几十微秒量级。
 */
class PeriodicTimer
{
public:
  using Clock = std::chrono::steady_clock;

  /** @brief 构造调度器，第一个节拍点为 now + period @param period 节拍周期，必须 > 0 */
  explicit PeriodicTimer(Clock::duration period) : period_(period), next_(Clock::now() + period) {}

  /**
   * @brief 睡到下一个节拍点，并推进节拍
   * @return 本次实际唤醒时刻相对节拍点的偏差：<= 0 表示按时或提前，> 0 表示迟到
   * @note 返回值可直接喂给统计（均值/标准差/最大值）用于在线抖动监控
   */
  Clock::duration wait_next()
  {
    const auto now = Clock::now();
    if (now >= next_) {
      // 上一轮超时：跳到未来第一个整节拍点，绝不连续补发
      const auto behind = now - next_;
      const auto skipped = behind / period_ + 1;
      missed_ += static_cast<std::uint64_t>(skipped);
      next_ += period_ * skipped;
    }

    std::this_thread::sleep_until(next_);

    const auto lateness = Clock::now() - next_;
    next_ += period_;
    return lateness;
  }

  /** @brief 因超时被跳过的节拍总数（> 0 说明这个循环的算力预算不够） @return 节拍数 */
  std::uint64_t missed() const { return missed_; }

  /** @brief 周期 @return 周期 */
  Clock::duration period() const { return period_; }

  /** @brief 把下一个节拍点重置为 now + period（用于模式切换后重新对齐） */
  void reset() { next_ = Clock::now() + period_; }

private:
  Clock::duration period_;
  Clock::time_point next_;
  std::uint64_t missed_ = 0;
};

/**
 * @brief 收紧当前线程的定时器松弛（timer slack），降低 sleep 的系统性迟到
 * @param slack_ns 允许内核合并定时器的窗口，默认 1 ns（即尽量不合并）
 *
 * Linux 默认 slack 是 50 us：内核可能为了省电把你的唤醒推迟最多 50 us。
 * 对 5 ms 周期来说这是 1% 的系统性偏移，且随负载变化，属于"看不见的抖动源"。
 * 非 Linux 平台为空实现。
 */
inline void tighten_timer_slack(long slack_ns = 1)
{
#ifdef __linux__
  ::prctl(PR_SET_TIMERSLACK, static_cast<unsigned long>(slack_ns), 0UL, 0UL, 0UL);
#else
  (void)slack_ns;
#endif
}

}  // namespace tools

#endif  // TOOLS__PERIODIC_TIMER_HPP
