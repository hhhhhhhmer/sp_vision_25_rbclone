// PeriodicTimer 回归测试：
//  1) 节拍对齐不变量 —— 每次唤醒时刻都落在 t0 + k*period 的网格上
//  2) 不连发 —— 单轮工作超过周期时跳整拍，绝不出现"背靠背"的短间隔
//  3) 均值不受工作耗时影响 —— 周期不再等于"工作耗时 + sleep"
//  4) missed() 正确计数被跳过的节拍
//  5) 与 sleep_for 的对照：sleep_for 的均值 = 周期 + 平均工作耗时
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <numeric>
#include <thread>
#include <vector>

#include "tools/periodic_timer.hpp"

namespace
{
int failures = 0;

void check(bool ok, const char * name, double value = 0.0, double tol = 0.0)
{
  if (ok) {
    std::printf("  [ ok ] %s\n", name);
  } else {
    std::printf("  [FAIL] %s (value=%.6g, tol=%.6g)\n", name, value, tol);
    failures++;
  }
}

using Clock = std::chrono::steady_clock;

/** @brief 忙等指定时长，模拟"耗时可变的一轮工作" @param us 微秒 */
void busy_work(double us)
{
  const auto until = Clock::now() + std::chrono::duration_cast<Clock::duration>(
                                      std::chrono::duration<double, std::micro>(us));
  volatile double sink = 0;
  while (Clock::now() < until) {
    for (int k = 0; k < 64; ++k) sink += std::sin(k);
  }
}

struct Stats
{
  double mean = 0, min = 0, p99 = 0, max = 0;
};

Stats stats(std::vector<double> v)
{
  Stats s;
  if (v.empty()) return s;
  std::sort(v.begin(), v.end());
  s.mean = std::accumulate(v.begin(), v.end(), 0.0) / v.size();
  s.min = v.front();
  s.p99 = v[(std::size_t)(v.size() * 0.99)];
  s.max = v.back();
  return s;
}

}  // namespace

int main()
{
  constexpr double kPeriodUs = 2000.0;  // 2 ms
  constexpr int kRounds = 200;

  tools::tighten_timer_slack();

  // ---------------------------------------------------------------- 正常负载
  {
    std::printf("=== 节拍对齐与不连发（工作耗时 100~600us，周期 2000us）===\n");
    tools::PeriodicTimer tick(std::chrono::microseconds(2000));
    const auto t0 = Clock::now();

    std::vector<double> intervals;
    double worst_grid_error = 0;
    double worst_lateness = -1e9;  // lateness 只能 >= 0
    Clock::time_point prev{};

    for (int i = 0; i < kRounds; ++i) {
      busy_work(100.0 + (i * 37) % 500);  // 0.1 ~ 0.6 ms 的可变工作
      const auto lateness = tick.wait_next();
      const auto wake = Clock::now();

      // 不变量：唤醒时刻必须落在 t0 + k*period 的网格上（允许 clock 读数的微小误差）
      const double since_t0 = std::chrono::duration<double, std::micro>(wake - t0).count();
      const double grid_error = std::fabs(std::remainder(since_t0, kPeriodUs));
      worst_grid_error = std::max(worst_grid_error, grid_error);
      worst_lateness =
        std::max(worst_lateness, -std::chrono::duration<double, std::micro>(lateness).count());

      if (i) intervals.push_back(std::chrono::duration<double, std::micro>(wake - prev).count());
      prev = wake;
    }

    const auto s = stats(intervals);
    check(worst_lateness <= 1.0, "lateness 从不为负（内核不会提前唤醒）", worst_lateness, 1.0);
    check(worst_grid_error < 500.0, "所有唤醒点都在节拍网格上", worst_grid_error, 500.0);
    check(s.min > kPeriodUs * 0.5, "没有连发（最小间隔 > 半个周期）", s.min, kPeriodUs * 0.5);
    check(std::fabs(s.mean - kPeriodUs) < kPeriodUs * 0.02, "平均间隔 = 周期 ±2%", s.mean, kPeriodUs * 0.02);
    check(tick.missed() == 0, "工作 < 周期时不跳拍", (double)tick.missed(), 0.0);
    std::printf(
      "         mean %.1f  min %.1f  p99 %.1f  max %.1f us\n", s.mean, s.min, s.p99, s.max);
  }

  // ---------------------------------------------------------------- 超时跳拍
  {
    std::printf("=== 超时策略：工作 14ms > 周期 10ms，只跳整拍、不连发 ===\n");
    tools::PeriodicTimer tick(std::chrono::milliseconds(10));
    std::vector<double> intervals;
    Clock::time_point prev{};
    bool saw_long_gap = false;
    bool saw_burst = false;

    for (int i = 0; i < 8; ++i) {
      busy_work(i == 3 ? 14000.0 : 2000.0);  // 第 4 轮故意超时
      tick.wait_next();
      const auto wake = Clock::now();
      if (i) {
        const double ms = std::chrono::duration<double, std::milli>(wake - prev).count();
        intervals.push_back(ms);
        if (ms > 15.0) saw_long_gap = true;   // 跳到未来整节拍 ⇒ 间隔变成整数倍周期
        if (ms < 5.0) saw_burst = true;       // 连发 ⇒ 出现远小于周期的间隔
      }
      prev = wake;
    }

    check(saw_long_gap, "超时后间隔变成整数倍周期（跳拍）");
    check(!saw_burst, "超时后没有出现连发");
    check(tick.missed() >= 1, "missed() 记录了被跳过的节拍", (double)tick.missed(), 1.0);
    std::printf("         missed=%llu, intervals:", (unsigned long long)tick.missed());
    for (double v : intervals) std::printf(" %.1f", v);
    std::printf(" ms\n");
  }

  // ---------------------------------------------------------------- 与 sleep_for 对照
  {
    std::printf("=== 对照：work + sleep_for(2ms) 的均值 = 周期 + 平均工作耗时 ===\n");
    constexpr int N = 100;
    auto measure = [](bool use_timer) {
      tools::PeriodicTimer tick(std::chrono::microseconds(2000));
      std::vector<double> intervals;
      Clock::time_point prev{};
      for (int i = 0; i < N; ++i) {
        busy_work(500.0);  // 固定 0.5ms 工作
        if (use_timer) {
          tick.wait_next();
        } else {
          std::this_thread::sleep_for(std::chrono::microseconds(2000));
        }
        const auto wake = Clock::now();
        if (i) intervals.push_back(std::chrono::duration<double, std::micro>(wake - prev).count());
        prev = wake;
      }
      return stats(intervals).mean;
    };

    const double with_sleep = measure(false);
    const double with_timer = measure(true);
    std::printf(
      "         sleep_for: %.1f us   PeriodicTimer: %.1f us   (差 %.1f us ≈ 工作耗时)\n",
      with_sleep, with_timer, with_sleep - with_timer);
    check(std::fabs(with_timer - kPeriodUs) < 100.0, "PeriodicTimer 均值 ≈ 周期", with_timer, 100.0);
    check(with_sleep - with_timer > 200.0, "sleep_for 均值明显大于周期（含工作耗时）",
          with_sleep - with_timer, 200.0);
  }

  std::printf(failures == 0 ? "\n全部通过\n" : "\n失败 %d 项\n", failures);
  return failures == 0 ? 0 : 1;
}
