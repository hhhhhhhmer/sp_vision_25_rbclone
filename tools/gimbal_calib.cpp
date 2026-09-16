/**
 * @file gimbal_calib.cpp
 * @brief 云台电机响应标定工具（方案 C：τ + T_cl 模型参数与等效延迟 D(ω)）
 *
 * 流程：自动对 yaw/pitch 两轴依次执行
 *   1) 阶跃测试：±3°/±8°/±15°，网格搜索拟合纯死区 τ 与一阶收敛时间常数 T_cl；
 *   2) 匀速梯形波：2/4/6 rad/s（含匀速巡航段），测量稳态滞后 → 等效延迟 D(ω) ≈ τ+T；
 * 最后打印 **可直接写入配置的 `gimbal_delay`（= 等效延迟，单位秒）**，
 * 以及各轴的 τ / T_cl 作为响应品质参考。
 *
 * 注意：Planner 已不再读取 τ / T_cl —— 方案 C 的"开火落地检查"因对云台真实状态
 * 无判别力（0.5 s 时域内初始误差衰减到 1e-19，残差只反映拍数编排与真实延时之差）
 * 已整体删除。本工具保留的价值是：产出 gimbal_delay 的实测值，并给出
 * "稳态滞后随角速度是否线性"（D(ω) 一致性）这一电控响应品质评估。
 *
 * 用法：
 *   gimbal_calib <config.yaml> [--out records.jsonl]
 *
 * 前置条件（重要）：
 *   - 车体静止、装弹状态与实战一致、电控 PID 参数已是实战值；
 *   - 反馈为四元数(IMU)推导（上行包 gimbal_yaw/gimbal_pitch 字段已停用），
 *     标定值会包含 IMU 姿态解算滤波滞后（约 5~10ms，pitch 轴更明显）；
 *     若实车出现"补偿过度(单边反向偏)"，把 gimbal_delay 减小对应毫秒数；
 *   - 仪具以当前反馈角度为基准做小幅度运动，云台需在安全范围内。
 */
#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <limits>
#include <mutex>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <nlohmann/json.hpp>

#include "io/gimbal/gimbal.hpp"
#include "tools/math_tools.hpp"
#include "tools/yaml.hpp"

using namespace std::chrono_literals;

namespace
{
constexpr double kDegToRad = M_PI / 180.0;
constexpr double kRadToDeg = 180.0 / M_PI;

/** @brief 匀速段期望测量窗口（s）；速度过高时会因安全幅度限制而被裁短 */
constexpr double kCruiseWindowS = 0.25;
/** @brief yaw 轴单方向最大幅度（度） */
constexpr double kYawMaxAmpDeg = 45.0;
/** @brief pitch 轴单方向最大幅度（度）：机械限位内的小幅度 */
constexpr double kPitchMaxAmpDeg = 10.0;

struct CmdSample
{
  double t = 0;     // 发送时刻（相对起始，s）
  double yaw = 0;   // 发送的 yaw 设定值（度）
  double pitch = 0; // 发送的 pitch 设定值（度）
};

struct FbSample
{
  double t = 0; // 反馈帧接收时刻（同一时钟，s）
  double yaw = 0;
  double pitch = 0;
};

struct StepFit
{
  bool valid = false;
  double tau = 0;        // 纯死区 s
  double T = 0;          // 一阶时间常数 s
  double overshoot = 0;  // 相对幅度
};

/** @brief 一阶 + 死区模型 */
inline double first_order_model(double t, double t0, double base, double delta, double tau, double T)
{
  if (t <= t0 + tau) return base;
  return base + delta * (1.0 - std::exp(-(t - t0 - tau) / T));
}

/**
 * @brief 阶跃段拟合：网格搜索 (τ, T) 最小化一阶模型与反馈残差
 */
StepFit fit_step_segment(
  const std::vector<FbSample> & fb, int axis, double t_jump, double base, double target)
{
  const double delta = target - base;
  if (std::abs(delta) < 0.5) return {};
  if (!std::isfinite(base) || !std::isfinite(target)) return {};

  std::vector<std::pair<double, double>> seg;
  for (const auto & s : fb) {
    const double v = axis == 0 ? s.yaw : s.pitch;
    if (s.t > t_jump + 0.002 && s.t <= t_jump + 0.7) seg.emplace_back(s.t, v);
  }
  if (seg.size() < 20) return {};

  double best_tau = 0, best_T = 0.005, best_sse = std::numeric_limits<double>::infinity();
  for (int tau_ms = 0; tau_ms <= 30; ++tau_ms) {
    for (int T_ms = 1; T_ms <= 50; ++T_ms) {
      const double tau = tau_ms * 1e-3;
      const double T = T_ms * 1e-3;
      double sse = 0;
      for (const auto & [t, v] : seg) {
        const double e = v - first_order_model(t, t_jump, base, delta, tau, T);
        sse += e * e;
      }
      if (sse < best_sse) {
        best_sse = sse;
        best_tau = tau;
        best_T = T;
      }
    }
  }
  if (best_sse / seg.size() > 1.5) return {};  // 残差过大（如丢帧/干扰），丢弃该段

  double peak = base;
  for (const auto & [t, v] : seg) {
    if (delta > 0 ? v > peak : v < peak) peak = v;
  }
  StepFit fit;
  fit.valid = true;
  fit.tau = best_tau;
  fit.T = best_T;
  fit.overshoot = std::abs(peak - target) / std::abs(delta);
  return fit;
}

/**
 * @brief 按记录的真实发送时刻线性插值出某时刻的"理想命令值"
 * @note 命令序列按时间递增；用真实发送时刻而非名义时间表，可消除发送循环
 *       （串口写 + 记录落盘）开销累积造成的相位漂移
 */
double cmd_at(const std::vector<CmdSample> & cmds, int axis, double t)
{
  if (cmds.empty()) return 0;
  const auto value = [axis](const CmdSample & s) { return axis == 0 ? s.yaw : s.pitch; };
  if (t <= cmds.front().t) return value(cmds.front());
  for (std::size_t i = 1; i < cmds.size(); ++i) {
    if (cmds[i].t < t) continue;
    const double t0 = cmds[i - 1].t;
    const double t1 = cmds[i].t;
    const double v0 = value(cmds[i - 1]);
    const double v1 = value(cmds[i]);
    if (t1 <= t0) return v1;
    return v0 + (v1 - v0) * (t - t0) / (t1 - t0);
  }
  return value(cmds.back());
}

/** @brief 匀速巡航段稳态滞后 → 等效延迟 D（s），<0 表示无效 */
double fit_cruise_lag(
  const std::vector<FbSample> & fb, const std::vector<CmdSample> & cmds, int axis,
  double t_win_start, double t_win_end, double w_deg_s)
{
  if (t_win_end - t_win_start < 0.02) return -1;  // 窗口太短，无统计意义
  double sum = 0;
  int n = 0;
  for (const auto & s : fb) {
    if (s.t < t_win_start || s.t > t_win_end) continue;
    const double v = axis == 0 ? s.yaw : s.pitch;
    sum += v - cmd_at(cmds, axis, s.t);  // 滞后 → 负
    ++n;
  }
  if (n < 20) return -1;
  const double lag_deg = sum / n;
  return std::abs(lag_deg) / std::abs(w_deg_s);  // 度 / (度/s) = s
}

std::string axis_name(int axis) { return axis == 0 ? "yaw" : "pitch"; }
}  // namespace

int main(int argc, char ** argv)
{
  // ---- 自检模式：合成已知 τ/T 的一阶+延迟被控对象，验证拟合数学 ---- 
  for (int i = 1; i < argc; ++i) {
    if (std::strcmp(argv[i], "--self-test") == 0) {
      const double tau_gt = 0.007, T_gt = 0.011, base = 10.0, delta = 8.0, t_jump = 1.0;
      std::vector<FbSample> synth;
      for (double t = 0; t < t_jump + 0.7; t += 0.002) {
        double v = base;
        if (t > t_jump + tau_gt) {
          v = base + delta * (1.0 - std::exp(-(t - t_jump - tau_gt) / T_gt));
        }
        synth.push_back({t, v, v});
      }
      const auto fit = fit_step_segment(synth, 0, t_jump, base, base + delta);
      const bool step_ok =
        std::abs(fit.tau - tau_gt) < 0.003 && std::abs(fit.T - T_gt) < 0.004;
      std::fprintf(
        stderr, "[self-test] 真值 τ=%.1fms T=%.1fms → 拟合 τ=%.1fms T=%.1fms (%s)\n",
        tau_gt * 1e3, T_gt * 1e3, fit.tau * 1e3, fit.T * 1e3, step_ok ? "PASS" : "FAIL");

      // 巡航稳态滞后 → 等效延迟（稳态下 y(t) = u(t - τ - T)）
      // 命令按真实发送时刻记录，反馈 = 命令滞后 (τ+T)
      const double w_deg = 4.0 * kRadToDeg;
      std::vector<CmdSample> crank_cmd;
      std::vector<FbSample> crank;
      const double t0 = 2.0;
      for (double t = t0; t < t0 + 1.0; t += 0.002) {
        const double cmd = base + w_deg * (t - t0);
        crank_cmd.push_back({t, cmd, cmd});
        const double v_ramp = base + w_deg * (t - t0 - tau_gt - T_gt);
        crank.push_back({t, v_ramp, v_ramp});
      }
      const double D = fit_cruise_lag(crank, crank_cmd, 0, t0 + 0.1, t0 + 0.9, w_deg);
      const bool cruise_ok = std::abs(D - (tau_gt + T_gt)) < 0.003;
      std::fprintf(
        stderr, "[self-test] 真值 τ+T=%.1fms → 巡航等效 D=%.1fms (%s)\n",
        (tau_gt + T_gt) * 1e3, D * 1e3, cruise_ok ? "PASS" : "FAIL");
      return (step_ok && cruise_ok) ? 0 : 1;
    }
  }

  if (argc < 2) {
    std::fprintf(stderr, "用法: gimbal_calib <config.yaml> [--out records.jsonl] [--self-test]\n");
    return 1;
  }
  const std::string config_path = argv[1];
  std::string out_path = "gimbal_calib_records.jsonl";
  for (int i = 2; i < argc; ++i) {
    if (std::strcmp(argv[i], "--out") == 0 && i + 1 < argc) out_path = argv[++i];
  }

  std::fprintf(stderr, "[标定] 连接云台 (config: %s) ...\n", config_path.c_str());
  io::Gimbal gimbal(config_path);

  const auto [init_state, init_t] = gimbal.state_at();
  if (init_t.time_since_epoch().count() == 0) {
    std::fprintf(stderr, "[标定] 串口未收到云台反馈帧，无法标定\n");
    return 1;
  }
  std::fprintf(
    stderr, "[标定] 当前 yaw=%.2f° pitch=%.2f°；请确认车体静止、无弹，3 秒后开始...\n",
    init_state.yaw, init_state.pitch);
  std::this_thread::sleep_for(3s);

  const auto t_start = std::chrono::steady_clock::now();
  const auto to_s = [&](std::chrono::steady_clock::time_point t) {
    return std::chrono::duration<double>(t - t_start).count();
  };

  // ---------------- 采集线程（1kHz 轮询，记录带接收时间戳的反馈） ----------------
  std::vector<FbSample> fb_samples;
  std::mutex fb_mu;
  std::atomic<bool> stop_rec{false};
  std::thread recorder([&]() {
    while (!stop_rec) {
      const auto [st, t] = gimbal.state_at();
      if (t.time_since_epoch().count() != 0) {
        std::lock_guard<std::mutex> lock(fb_mu);
        fb_samples.push_back({to_s(t), st.yaw, st.pitch});
      }
      std::this_thread::sleep_for(500us);
    }
  });

  // ---------------- 命令序列 / 记录 ----------------
  std::vector<CmdSample> cmd_samples;
  std::ofstream records(out_path);
  const auto send = [&](double yaw_deg, double pitch_deg, const char * phase) {
    const double t = to_s(std::chrono::steady_clock::now());
    gimbal.send(
      true, false, static_cast<float>(yaw_deg * kDegToRad), 0, 0,
      static_cast<float>(pitch_deg * kDegToRad), 0, 0);
    {
      std::lock_guard<std::mutex> lock(fb_mu);
      cmd_samples.push_back({t, yaw_deg, pitch_deg});
    }
    // 落盘放在锁外：文件 I/O 持锁会阻塞采集线程并放大发送周期抖动
    if (records.is_open()) {
      records << nlohmann::json{
        {"time_us", static_cast<int64_t>(t * 1e6)},
        {"data", {{"phase", phase}, {"cmd_yaw_deg", yaw_deg}, {"cmd_pitch_deg", pitch_deg}}}}
                   .dump()
              << '\n';
      records.flush();
    }
  };

  // 阶跃：0→0.7s 命令 = target，0.7→1.4s 命令 = base（每 10ms 一帧）
  const auto run_step = [&](int axis, double base_deg, double target_deg, const char * tag) {
    const double dt_cmd = 0.01;
    for (double t = 0; t < 1.4; t += dt_cmd) {
      const bool up = t < 0.7;
      const double yaw = axis == 0 ? (up ? target_deg : base_deg) : base_deg;
      const double pitch = axis == 1 ? (up ? target_deg : base_deg) : base_deg;
      send(yaw, pitch, tag);
      std::this_thread::sleep_until(
        std::chrono::steady_clock::now() + std::chrono::duration<double>(dt_cmd));
    }
  };

  // 匀速段：幅度受安全上限约束（yaw 可大、pitch 必须小），速度越高匀速窗口越短；
  // 之后等速回位并停顿 0.3s，随后反方向一次。
  // 注意：位置幅度 = w·t_up，绝不能让它随 ω 无上限增长（否则会撞限位/进入限幅，
  // 测出的 D(ω) 也就没有意义了）。
  struct RampSeg
  {
    int axis = 0;
    int dir = 0;         // ±1
    double w_deg_s = 0;  // 匀速速度(度/s)
    double t0 = 0;       // 段绝对起始时刻
    double t_up = 0;     // 单方向匀速时长（= 幅度 / w）
  };
  std::vector<RampSeg> ramp_segs;

  const auto run_ramp = [&](int axis, double base_deg, double omega_rad_s, const char * tag) {
    const double w_deg = omega_rad_s * kRadToDeg;
    const double dt_cmd = 0.01;
    const double a_max = axis == 0 ? kYawMaxAmpDeg : kPitchMaxAmpDeg;  // 单轴安全幅度
    const double t_up = std::min(kCruiseWindowS, a_max / w_deg);       // 匀速时长
    const double t_dir = 2.0 * t_up + 0.3;                             // 上行+回位+停顿
    const double total = t_dir * 2;
    const double t0 = to_s(std::chrono::steady_clock::now());
    for (int dir = 1; dir >= -1; dir -= 2) {
      ramp_segs.push_back({axis, dir, w_deg, t0 + (dir > 0 ? 0.0 : t_dir), t_up});
    }
    for (double t = 0; t < total; t += dt_cmd) {
      const int seg = t < t_dir ? 0 : 1;
      const double local = t - seg * t_dir;
      const double dir = seg == 0 ? 1.0 : -1.0;
      double value = base_deg;
      if (local < t_up) {
        value = base_deg + dir * w_deg * local;                       // 上行匀速
      } else if (local < 2.0 * t_up) {
        value = base_deg + dir * w_deg * (2.0 * t_up - local);        // 等速回位
      } else {
        value = base_deg;                                             // 停顿
      }
      const double yaw = axis == 0 ? value : base_deg;
      const double pitch = axis == 1 ? value : base_deg;
      send(yaw, pitch, tag);
      std::this_thread::sleep_until(
        std::chrono::steady_clock::now() + std::chrono::duration<double>(dt_cmd));
    }
  };

  // ---------------- 执行标定 ----------------
  std::vector<StepFit> step_fits[2];
  std::vector<double> cruise_D[2];
  std::vector<double> cruise_omega[2];
  double axis_base_deg[2] = {init_state.yaw, init_state.pitch};

  for (int axis = 0; axis < 2; ++axis) {
    const double base_deg = axis_base_deg[axis];
    std::fprintf(stderr, "[标定] ==== %s 轴, base=%.2f° ====\n", axis_name(axis).c_str(), base_deg);

    // --- 阶跃 ---
    for (double amp : {3.0, 8.0, 15.0}) {
      for (int dir : {1, -1}) {
        run_step(axis, base_deg, base_deg + dir * amp,
                 (axis == 0 ? "step_yaw" : "step_pitch"));
      }
    }
    // 阶跃拟合：从命令序列找跳变
    {
      std::lock_guard<std::mutex> lock(fb_mu);
      for (std::size_t i = 1; i < cmd_samples.size(); ++i) {
        const double v0 = axis == 0 ? cmd_samples[i - 1].yaw : cmd_samples[i - 1].pitch;
        const double v1 = axis == 0 ? cmd_samples[i].yaw : cmd_samples[i].pitch;
        if (std::isfinite(v0) && std::isfinite(v1) && std::abs(v1 - v0) > 1.5) {
          const auto fit = fit_step_segment(fb_samples, axis, cmd_samples[i].t, v0, v1);
          if (fit.valid) step_fits[axis].push_back(fit);
        }
      }
      std::fprintf(stderr, "[标定] %s 阶跃有效段: %zu\n", axis_name(axis).c_str(), step_fits[axis].size());
    }
    std::this_thread::sleep_for(0.5s);

    // --- 匀速段 ---
    for (double w : {2.0, 4.0, 6.0}) {
      run_ramp(axis, base_deg, w, axis == 0 ? "ramp_yaw" : "ramp_pitch");
    }
    // 测量窗口取匀速段的后 70%（避开起步瞬间的限幅/过渡）
    {
      std::lock_guard<std::mutex> lock(fb_mu);
      for (const auto & seg : ramp_segs) {
        if (seg.axis != axis) continue;
        const double win0 = seg.t0 + 0.3 * seg.t_up;
        const double win1 = seg.t0 + seg.t_up;
        // 理想命令由真实发送时刻插值得到，不受发送循环抖动影响
        const double D = fit_cruise_lag(
          fb_samples, cmd_samples, axis, win0, win1, seg.dir * seg.w_deg_s);
        if (D > 0) {
          cruise_D[axis].push_back(D);
          cruise_omega[axis].push_back(seg.w_deg_s / kRadToDeg);
        } else {
          std::fprintf(
            stderr, "[标定] %s 段 ω=%.0f°/s 窗口仅 %.0fms，样本不足，已跳过\n",
            axis_name(axis).c_str(), seg.w_deg_s, (win1 - win0) * 1e3);
        }
      }
    }
    std::this_thread::sleep_for(0.5s);
  }

  // ---------------- 汇总输出 ----------------
  stop_rec = true;
  recorder.join();

  std::fprintf(stderr, "\n=== 标定完成 ===\n");
  double median_tau[2], median_T[2];
  for (int axis = 0; axis < 2; ++axis) {
    if (step_fits[axis].empty()) {
      std::fprintf(stderr, "[%s] 无有效阶跃数据，使用默认 τ=5ms T=12ms(建议重测)\n", axis_name(axis).c_str());
      median_tau[axis] = 0.005;
      median_T[axis] = 0.012;
      continue;
    }
    std::vector<double> taus, Ts, overs;
    for (const auto & f : step_fits[axis]) {
      taus.push_back(f.tau);
      Ts.push_back(f.T);
      overs.push_back(f.overshoot);
    }
    std::sort(taus.begin(), taus.end());
    std::sort(Ts.begin(), Ts.end());
    std::sort(overs.begin(), overs.end());
    median_tau[axis] = taus[taus.size() / 2];
    median_T[axis] = Ts[Ts.size() / 2];
    const double max_overshoot = overs.back();
    std::fprintf(
      stderr, "[%s] τ=%.1fms  T_cl=%.1fms  τ+T=%.1fms  最大过冲=%.0f%%%s\n", axis_name(axis).c_str(),
      median_tau[axis] * 1e3, median_T[axis] * 1e3, (median_tau[axis] + median_T[axis]) * 1e3,
      max_overshoot * 100, max_overshoot > 0.08 ? "  [警告] 过冲>8%，模型偏二阶，数值仅供参考" : "");
  }

  // 等效延迟 D(ω) 一致性检查
  for (int axis = 0; axis < 2; ++axis) {
    if (cruise_D[axis].empty()) {
      std::fprintf(stderr, "[%s] 无有效巡航数据，无法给出 D(ω)\n", axis_name(axis).c_str());
      continue;
    }
    std::fprintf(stderr, "[%s] D(ω):", axis_name(axis).c_str());
    for (std::size_t i = 0; i < cruise_D[axis].size(); ++i) {
      std::fprintf(stderr, " %.0frad/s→%.0fms", cruise_omega[axis][i], cruise_D[axis][i] * 1e3);
    }
    const auto [mn, mx] = std::minmax_element(cruise_D[axis].begin(), cruise_D[axis].end());
    const double spread = (mx - mn) / std::max(1e-6, *mn);
    std::fprintf(
      stderr, "%s\n", spread > 0.25
        ? "  [警告] D 随速度变化>25%(非线性/限幅)，建议改用速度相关曲线表，常数不适用"
        : "");
  }

  // 单位注意：gimbal_delay 在配置里是"秒"，这里绝不能把 ms 混进来
  // （否则会把 17ms 写成 17s）。
  // 说明：yaw/pitch 的 τ、T_cl 已不再被 Planner 消费（方案 C 的"开火落地检查"因
  // 对云台真实状态无判别力已删除），但保留输出用于手工评估电控响应品质：
  // 等效延迟 D(ω) 就是稳态滞后，可直接作为 gimbal_delay 的参考值。
  std::fprintf(
    stderr,
    "\n###### gimbal_delay 写入配置 (configs/xxx.yaml)，单位为秒 ######\n"
    "gimbal_delay: %.4f\n"
    "###### 以下为各轴响应品质参考（Planner 不再读取）######\n"
    "gimbal_yaw_tau_s: %.4f\n"
    "gimbal_yaw_T_cl_s: %.4f\n"
    "gimbal_pitch_tau_s: %.4f\n"
    "gimbal_pitch_T_cl_s: %.4f\n",
    median_tau[0] + median_T[0], median_tau[0], median_T[0], median_tau[1], median_T[1]);

  std::fprintf(stderr, "[记录] %s\n", out_path.c_str());
  return 0;
}
