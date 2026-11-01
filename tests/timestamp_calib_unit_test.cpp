// 时间戳标定算法的离线自检：用合成数据验证「能反解出已知的 τ」。
//
// 这个测试不依赖相机/云台/检测器，纯数学：
//   1) 给定一条真实的云台姿态轨迹 q_true(t) 和一个静止目标 p_w；
//   2) 图像的真实曝光时刻比时间戳早 τ_true，因此每帧观测到的云台系位置是
//      p_g = R(q_true(t-τ_true))ᵀ · p_w + 噪声；
//   3) 标定器只拿到 (t, p_g) 和 q_true 的历史，应当反解出 τ ≈ τ_true。
//
// 同时检查"云台不动时无法标定"这一必要前提是否被正确识别（不能给出假结果）。

#include <Eigen/Geometry>
#include <cmath>
#include <cstdio>
#include <random>
#include <string>
#include <vector>

#include "tools/timestamp_calib.hpp"

namespace
{
using Clock = std::chrono::steady_clock;

int failures = 0;

void check(bool cond, const std::string & what)
{
  std::printf("  [%s] %s\n", cond ? " OK " : "FAIL", what.c_str());
  if (!cond) ++failures;
}

/** @brief 合成一条带 yaw/pitch/roll 摆动的云台姿态轨迹 */
Eigen::Quaterniond attitude(double t)
{
  const double yaw = 0.45 * std::sin(2.0 * M_PI * 1.3 * t) + 0.20 * t;
  const double pitch = 0.06 * std::sin(2.0 * M_PI * 0.7 * t + 0.4);
  const double roll = 0.04 * std::sin(2.0 * M_PI * 1.9 * t + 1.1);
  return Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
         Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
         Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX());
}

struct Scene
{
  tools::TimestampCalibrator calib;
  std::vector<tools::TimestampedQuaternion> q_history;
  double tau_true_s = 0;
};

/** @brief 生成一段"云台摆动 + 静止目标"的合成数据
 *  @param moving false 时云台完全不动（用于验证激励不足必须被拒绝）
 */
Scene make_scene(double tau_true_s, bool moving, double noise_std, unsigned seed)
{
  Scene scene;
  scene.tau_true_s = tau_true_s;

  const Eigen::Vector3d p_w(2.6, 1.1, 0.15);  // 静止目标（云台系原点为世界原点）
  const auto t0 = Clock::time_point(std::chrono::seconds(1000));
  const double frame_dt = 1.0 / 120.0;
  const double sample_dt = 1.0 / 800.0;
  const int frames = 360;  // 3 s

  std::mt19937 rng(seed);
  std::normal_distribution<double> noise(0.0, noise_std);

  // 图像 + 观测：内容对应 t-τ_true 时刻的姿态
  for (int i = 0; i < frames; ++i) {
    const double t = static_cast<double>(i) * frame_dt;
    const auto q_content = moving ? attitude(t - tau_true_s) : attitude(0.0);
    Eigen::Vector3d p_g = q_content.toRotationMatrix().transpose() * p_w;
    p_g += Eigen::Vector3d(noise(rng), noise(rng), noise(rng));
    scene.calib.add_sample(t0 + std::chrono::microseconds(static_cast<long long>(t * 1e6)), p_g);
  }

  // 姿态历史：覆盖整个采样段前后各 0.5 s
  const int n_q = static_cast<int>(4.0 / sample_dt);
  for (int k = -n_q / 8; k < n_q; ++k) {
    const double t = static_cast<double>(k) * sample_dt;
    const auto q = moving ? attitude(t) : attitude(0.0);
    scene.q_history.push_back({t0 + std::chrono::microseconds(static_cast<long long>(t * 1e6)), q});
  }
  return scene;
}

}  // namespace

int main()
{
  std::printf("=== timestamp_calib_unit_test ===\n");

  // ---- 1. 已知 τ，检查能否反解 ----
  std::printf("\n[1] 已知 tau = 7.30 ms（3 s 摆动、1.5 cm 噪声）\n");
  {
    const double tau_true = 0.0073;
    const auto scene = make_scene(tau_true, true, 0.015, 42);
    const auto r = scene.calib.fit(scene.q_history);

    std::printf(
      "  帧数 %d，使用 %d，剔除 %d；RMS|w| %.2f rad/s（峰值 %.2f）\n", r.frames, r.used, r.rejected,
      r.excitation_rad_s, r.excitation_peak_rad_s);
    std::printf(
      "  tau = %.2f ms +- %.2f ms；世界系离散度 %.1f cm (tau=0) -> %.1f cm\n", r.tau_s * 1e3,
      r.sigma_s * 1e3, r.rms_at_zero_m * 100, r.rms_at_tau_m * 100);
    std::printf("  前后半段 %.2f / %.2f ms\n", r.tau_first_half_s * 1e3, r.tau_second_half_s * 1e3);

    check(r.ok, std::string("标定应成功") + (r.ok ? "" : "，原因：" + r.reason));
    check(std::abs(r.tau_s - tau_true) < 0.6e-3, "tau 误差应小于 0.6 ms");
    check(r.sigma_s < 0.5e-3, "1 sigma 应小于 0.5 ms");
    check(r.rms_at_tau_m < r.rms_at_zero_m * 0.5, "拟合后离散度应显著下降");
    check(std::abs(r.tau_first_half_s - r.tau_second_half_s) < 1.5e-3, "前后半段应一致");
  }

  // ---- 2. 云台不动：必须拒绝 ----
  std::printf("\n[2] 云台不动（激励不足）应被拒绝\n");
  {
    const auto scene = make_scene(0.0073, false, 0.015, 7);
    const auto r = scene.calib.fit(scene.q_history);
    std::printf(
      "  RMS|w| = %.3f rad/s，ok = %d，原因：%s\n", r.excitation_rad_s, r.ok ? 1 : 0,
      r.reason.c_str());
    check(!r.ok, "静止云台不应给出可信结果");
    check(r.excitation_rad_s < 0.5, "静止时激励应接近 0");
  }

  // ---- 3. 不同 τ 的真值都要能反解出来 ----
  std::printf("\n[3] 多组真值扫描\n");
  {
    const double truths[] = {-3.0e-3, 0.0, 1.5e-3, 4.0e-3, 9.5e-3, 13.0e-3};
    double worst = 0;
    for (double truth : truths) {
      const auto scene = make_scene(truth, true, 0.015, 1234);
      const auto r = scene.calib.fit(scene.q_history);
      const double err_ms = (r.tau_s - truth) * 1e3;
      worst = std::max(worst, std::abs(err_ms));
      std::printf(
        "  真值 %+6.2f ms -> 标定 %+7.2f ms (误差 %+5.2f ms, sigma %.2f ms, ok=%d)\n", truth * 1e3,
        r.tau_s * 1e3, err_ms, r.sigma_s * 1e3, r.ok ? 1 : 0);
      if (!r.ok) {
        std::printf("    失败原因：%s\n", r.reason.c_str());
        ++failures;
      }
    }
    check(worst < 0.6, "各真值下最大误差应小于 0.6 ms");
  }

  // ---- 4. 姿态历史覆盖不足时应报错而不是给假结果 ----
  std::printf("\n[4] 姿态历史覆盖不足\n");
  {
    auto scene = make_scene(0.0073, true, 0.015, 99);
    // 砍掉后半段姿态历史
    scene.q_history.resize(scene.q_history.size() / 2);
    const auto r = scene.calib.fit(scene.q_history);
    std::printf("  ok = %d，原因：%s\n", r.ok ? 1 : 0, r.reason.c_str());
    check(!r.ok, "覆盖不足时不应给出可信结果");
  }

  // ---- 5. 多种随机噪声下应无偏（单个种子差 0.15 ms 可能只是噪声实现）----
  std::printf("\n[5] 20 组随机种子的无偏性\n");
  {
    const double tau_true = 0.0073;
    double sum = 0, sumsq = 0;
    int n = 0, fails = 0;
    for (unsigned seed = 0; seed < 20; ++seed) {
      const auto scene = make_scene(tau_true, true, 0.015, seed);
      const auto r = scene.calib.fit(scene.q_history);
      if (!r.ok) {
        ++fails;
        continue;
      }
      const double err = (r.tau_s - tau_true) * 1e3;
      sum += err;
      sumsq += err * err;
      ++n;
    }
    const double mean = (n > 0) ? sum / n : 0.0;
    const double sd = (n > 0) ? std::sqrt(std::max(0.0, sumsq / n - mean * mean)) : 0.0;
    std::printf("  n=%d 失败 %d：均值误差 %+.3f ms，标准差 %.3f ms\n", n, fails, mean, sd);
    check(fails == 0, "20 组都不应被拒绝");
    check(std::abs(mean) < 0.2, "均值误差应接近 0（无偏）");
    check(sd < 0.3, "标准差应小于 0.3 ms");
  }

  // ---- 6. 自动摆动的轨迹：连续、不越界、速度/加速度与位置自洽 ----
  std::printf("\n[6] 自动摆动轨迹 SweepProfile\n");
  {
    tools::SweepProfile p;
    p.yaw_now = 0.30;  // 起始角比中心角偏 0.2 rad
    p.yaw_center = 0.10;
    p.amp = 20.0 * M_PI / 180.0;
    p.freq = 1.2;
    p.align_s = 0.5;
    p.sweep_s = 3.0;

    const auto s0 = p.at(0.0);
    const auto s_align = p.at(p.align_s);
    const auto s_end = p.at(p.total_s());
    std::printf(
      "  yaw(0)=%.4f yaw(align)=%.4f yaw(total)=%.4f done=%d\n", s0.yaw, s_align.yaw, s_end.yaw,
      s_end.done ? 1 : 0);

    check(std::abs(s0.yaw - p.yaw_now) < 1e-12, "t=0 时从当前角出发");
    check(std::abs(s_align.yaw - p.yaw_center) < 1e-9, "对准结束时正好到目标方位");
    check(std::abs(s0.vel - (p.yaw_center - p.yaw_now) / p.align_s) < 1e-9, "对准段速度为定值");
    check(s_end.done && std::abs(s_end.yaw - p.yaw_center) < 1e-9, "结束时停在中心角");

    // 摆幅不越界 + 位置/速度/加速度互为导数
    // 注意：只在中段做差分——对准段与正弦段的连接处速度本来就是跳变，末尾之外 at() 已判定结束
    double max_dev = 0, max_vel_err = 0, max_acc_err = 0;
    const double h = 1e-5;
    for (double t = p.align_s + 0.01; t < p.total_s() - 0.01; t += 0.001) {
      const auto s = p.at(t);
      max_dev = std::max(max_dev, std::abs(s.yaw - p.yaw_center));
      const double vel_fd = (p.at(t + h).yaw - p.at(t - h).yaw) / (2 * h);
      const double acc_fd = (p.at(t + h).vel - p.at(t - h).vel) / (2 * h);
      max_vel_err = std::max(max_vel_err, std::abs(vel_fd - s.vel));
      max_acc_err = std::max(max_acc_err, std::abs(acc_fd - s.acc));
    }
    std::printf(
      "  最大偏离 %.4f rad（摆幅 %.4f），速度误差 %.2e，加速度误差 %.2e\n", max_dev, p.amp,
      max_vel_err, max_acc_err);
    check(max_dev <= p.amp + 1e-6, "摆动不超出设定幅度");
    check(max_vel_err < 1e-6, "位置与速度自洽");
    check(max_acc_err < 1e-3, "速度与加速度自洽");
  }

  std::printf("\n=== %s ===\n", failures == 0 ? "PASS" : "FAIL");
  return failures == 0 ? 0 : 1;
}
