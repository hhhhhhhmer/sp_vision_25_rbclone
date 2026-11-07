#include <atomic>
#include <chrono>
#include <cstdint>
#include <nlohmann/json.hpp>
#include <opencv2/opencv.hpp>
#include <optional>
#include <string>
#include <thread>

#include "io/camera/camera.hpp"
#include "io/gimbal/gimbal.hpp"
#include "tasks/auto_aim/aiming/planner/planner.hpp"
#include "tasks/auto_aim/detection/yolo.hpp"
#include "tasks/auto_aim/geometry/solver.hpp"
#include "tasks/auto_aim/tracking/target.hpp"
#include "tasks/auto_aim/tracking/tracker.hpp"
#include "tools/exiter.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/periodic_timer.hpp"
#include "tools/plotter.hpp"
#include "tools/systemd_watchdog.hpp"
#include "tools/thread_safe_queue.hpp"

using namespace std::chrono_literals;

namespace rb_runtime
{

/** @brief 两个自瞄程序的公共运行选项 */
struct Options
{
  std::string config_path;  ///< yaml 配置路径
  bool multithread = true;  ///< true: 异步检测 + 独立下发线程；false: 单线程同步
  bool sb_send = false;     ///< true: 哨兵 sb_send 帧；false: 普通 send 帧
};

/** @brief 命令行参数定义 @return cv::CommandLineParser 使用的 keys */
inline const std::string & cli_keys()
{
  static const std::string keys =
    "{help h usage ? |                           | 输出命令行参数说明}"
    "{@config-path   | ../configs/rb_auto_aim.yaml | 位置参数，yaml配置文件路径 }"
    "{thread t       | 1                         | 1=多线程(异步推理+独立下发线程)，0=单线程同步 }"
    "{sb_send s      | 0                         | 1=哨兵 sb_send 帧(带目标坐标)，0=普通 send 帧 }";
  return keys;
}

/** @brief 解析命令行参数 @param argc 参数个数 @param argv 参数数组 @param opt 输出选项 @return 参数有效且需要继续运行时返回 true */
inline bool parse_options(int argc, char ** argv, Options & opt)
{
  cv::CommandLineParser cli(argc, argv, cli_keys());
  opt.config_path = cli.get<std::string>(0);
  if (cli.has("help") || opt.config_path.empty()) {
    cli.printMessage();
    return false;
  }

  const int thread = cli.get<int>("thread");
  const int sb_send = cli.get<int>("sb_send");
  if ((thread != 0 && thread != 1) || (sb_send != 0 && sb_send != 1)) {
    tools::logger()->error("--thread 和 --sb_send 只能是 0 或 1");
    cli.printMessage();
    return false;
  }

  opt.multithread = (thread == 1);
  opt.sb_send = (sb_send == 1);
  return true;
}

/** @brief 判断云台是否处于打符模式（本程序只管自瞄，打符时不下发自瞄指令） @param mode 云台模式 @return 打符模式返回 true */
inline bool is_buff(io::GimbalMode mode)
{
  return mode == io::GimbalMode::SMALL_BUFF || mode == io::GimbalMode::BIG_BUFF;
}

/** @brief 按选项发送一帧自瞄指令 @param gimbal 云台串口 @param plan 规划结果 @param target 当前目标（sb_send 需要目标坐标） @param sb_send 是否使用哨兵帧 */
inline void send_plan(
  io::Gimbal & gimbal, const auto_aim::Plan & plan, const std::optional<auto_aim::Target> & target,
  bool sb_send)
{
  if (!sb_send) {
    gimbal.send(
      plan.control, plan.fire, plan.yaw, plan.yaw_vel, plan.yaw_acc, plan.pitch, plan.pitch_vel,
      plan.pitch_acc);
    return;
  }

  // 哨兵帧额外下发目标在场地坐标系中的中心位置与编号；沿用仓库里 sb_send 的既有约定：
  // EKF 的 x/z 是场地平面坐标，name + 1 对应下位机的目标编号（0 表示无目标）。
  float target_x = 0.0F;
  float target_y = 0.0F;
  std::uint8_t target_name = 0;
  if (target.has_value()) {
    const auto ekf_x = target->ekf_x();
    target_x = static_cast<float>(ekf_x[0]);
    target_y = static_cast<float>(ekf_x[2]);
    target_name = static_cast<std::uint8_t>(target->name) + 1;
  }

  gimbal.sb_send(
    plan.control, plan.fire, plan.yaw, plan.yaw_vel, plan.yaw_acc, plan.pitch, plan.pitch_vel,
    plan.pitch_acc, target_x, target_y, target_name);
}

/** @brief 释放控制并让云台保持在当前角度 @param gimbal 云台串口 @param sb_send 是否使用哨兵帧 @param yaw 当前 yaw，单位 rad @param pitch 当前 pitch，单位 rad */
inline void send_stop(io::Gimbal & gimbal, bool sb_send, float yaw, float pitch)
{
  if (sb_send) {
    gimbal.sb_send(false, false, yaw, 0.0F, 0.0F, pitch, 0.0F, 0.0F, 0.0F, 0.0F, 0);
  } else {
    gimbal.send(false, false, yaw, 0.0F, 0.0F, pitch, 0.0F, 0.0F);
  }
}

}  // namespace rb_runtime

namespace
{

constexpr auto kSendPeriod = 10ms;
using TargetQueue = tools::ThreadSafeQueue<std::optional<auto_aim::Target>, true>;

/**
 * @brief 正常比赛用的自瞄程序
 *
 * 命令行可切换两条正交的执行路径：
 *  - --thread=1：主线程采集+异步推理+跟踪，独立线程按 10ms 节拍规划并下发；
 *  - --thread=0：采集、推理、跟踪、规划、下发全部在同一个循环里完成。
 *  - --sb_send=1：下发哨兵帧（额外带目标坐标），=0 下发普通自瞄帧。
 *
 * 本程序不处理打符；云台切到打符模式时只释放控制，不打符也不下发自瞄指令。
 */

}  // namespace

int main(int argc, char ** argv)
{
  rb_runtime::Options opt;
  if (!rb_runtime::parse_options(argc, argv, opt)) return 0;

  tools::SystemdWatchdog watchdog;
  tools::Exiter exiter;
  tools::Plotter plotter;

  io::Gimbal gimbal(opt.config_path);
  io::Camera camera(opt.config_path);

  // 比赛程序关闭 YOLO 内部调试窗口，避免弹窗影响帧率
  auto_aim::YOLO yolo(opt.config_path, false);
  auto_aim::Solver solver(opt.config_path);
  auto_aim::Tracker tracker(opt.config_path, &solver);
  tracker.set_gimbal(&gimbal);
  auto_aim::Planner planner(opt.config_path);

  tools::logger()->info(
    "[rb_auto_aim] config={} thread={} sb_send={}", opt.config_path, opt.multithread ? 1 : 0,
    opt.sb_send ? 1 : 0);

  TargetQueue target_queue(1);
  target_queue.push(std::nullopt);
  std::atomic<bool> quit{false};

  auto plot_plan = [&](
                     const auto_aim::Plan & plan, const std::optional<auto_aim::Target> & target,
                     const io::GimbalState & gs, bool fired, double plan_ms,
                     std::chrono::steady_clock::duration lateness, std::uint64_t missed,
                     std::chrono::steady_clock::time_point t0) {
    nlohmann::json data;
    data["t"] = tools::delta_time(std::chrono::steady_clock::now(), t0);
    data["multithread"] = opt.multithread ? 1 : 0;
    data["sb_send"] = opt.sb_send ? 1 : 0;

    data["gimbal_yaw"] = gs.yaw;
    data["gimbal_yaw_vel"] = gs.yaw_vel;
    data["gimbal_pitch"] = gs.pitch;
    data["gimbal_pitch_vel"] = gs.pitch_vel;
    data["q2yaw"] = gs.q2yaw;
    data["q2pitch"] = gs.q2pitch;

    data["target_yaw"] = plan.target_yaw;
    data["target_pitch"] = plan.target_pitch;

    data["plan_mode"] = plan.control ? (plan.fire ? 2 : 1) : 0;
    data["plan_yaw"] = plan.yaw / CV_PI * 180.0;
    data["plan_yaw_vel"] = plan.yaw_vel;
    data["plan_yaw_acc"] = plan.yaw_acc;
    data["plan_pitch"] = plan.pitch * 57.3;
    data["plan_pitch_vel"] = plan.pitch_vel;
    data["plan_pitch_acc"] = plan.pitch_acc;
    data["fire"] = plan.fire ? 1 : 0;
    data["fired"] = fired ? 1 : 0;

    if (target.has_value()) {
      const auto ekf_x = target->ekf_x();
      data["target_z"] = ekf_x[4];
      data["target_vz"] = ekf_x[5];
      data["target_update_count"] = target->update_count_;
      data["tower_h1"] = target->tower_armor_hs[0].second;
      data["tower_h2"] = target->tower_armor_hs[1].second;
      data["tower_h3"] = target->tower_armor_hs[2].second;
      data["tower_armor_h"] = target->tower_armor_h;

      data["ekf_x"] = ekf_x[0];
      data["ekf_vx"] = ekf_x[1];
      data["ekf_y"] = ekf_x[2];
      data["ekf_vy"] = ekf_x[3];
      data["ekf_z"] = ekf_x[4];
      data["ekf_vz"] = ekf_x[5];
      data["ekf_yaw"] = ekf_x[6] * 57.3;
      data["ekf_vyaw"] = ekf_x[7] * 57.3;
      data["ekf_r"] = ekf_x[8];
    }

    data["plan_ms"] = plan_ms;
    data["send_late_us"] = std::chrono::duration_cast<std::chrono::microseconds>(lateness).count();
    data["send_missed"] = static_cast<double>(missed);
    plotter.plot(data);
  };

  if (!watchdog.ready("Vision pipeline is ready")) {
    tools::logger()->warn("无法向 systemd 发送 READY 通知");
  }

  if (opt.multithread) {
    // ---------------- 多线程：异步推理 + 独立下发线程 ----------------
    const auto t0 = std::chrono::steady_clock::now();

    std::thread plan_thread([&] {
      tools::tighten_timer_slack();
      tools::PeriodicTimer tick(kSendPeriod);

      std::optional<auto_aim::Target> target = std::nullopt;
      std::uint16_t last_bullet_count = 0;
      bool was_buff = false;

      while (!quit.load()) {
        const auto gs = gimbal.state();
        const auto mode = gimbal.mode();

        if (rb_runtime::is_buff(mode)) {
          if (!was_buff) {
            rb_runtime::send_stop(gimbal, opt.sb_send, gs.yaw / 57.3F, gs.pitch / 57.3F);
            was_buff = true;
          }
          target.reset();
          tick.wait_next();
          continue;
        }
        was_buff = false;

        // 只保留最新目标；队列里是 nullopt 时表示跟踪丢失，必须覆盖旧目标
        std::optional<auto_aim::Target> incoming;
        while (target_queue.try_pop(incoming)) target = std::move(incoming);

        const auto plan_start = std::chrono::steady_clock::now();
        auto plan = planner.plan(
          target, gs.bullet_speed, gs.yaw, auto_aim::Planner::ShootStrategy::rbSuppressiveFire);
        const auto plan_end = std::chrono::steady_clock::now();
        const double plan_ms =
          std::chrono::duration<double, std::milli>(plan_end - plan_start).count();

        // 睡到绝对节拍点后再发送：发送间隔与解算耗时无关，超时则跳整拍
        const auto lateness = tick.wait_next();
        rb_runtime::send_plan(gimbal, plan, target, opt.sb_send);

        const bool fired = gs.bullet_count > last_bullet_count;
        last_bullet_count = gs.bullet_count;

        plot_plan(plan, target, gs, fired, plan_ms, lateness, tick.missed(), t0);
      }
    });

    int frame_count = 0;
    auto auto_aim_started_at = std::chrono::steady_clock::time_point{};
    auto last_mode = gimbal.mode();
    cv::Mat img;
    std::chrono::steady_clock::time_point t;

    while (!exiter.exit() && !quit.load()) {
      const auto mode = gimbal.mode();
      if (mode != last_mode) {
        tools::logger()->info("Switch to {}", gimbal.str(mode));
        if (rb_runtime::is_buff(mode)) target_queue.push(std::nullopt);
        if (!rb_runtime::is_buff(mode)) {
          // 打符期间异步流水线里可能还有旧帧，切回自瞄后按采集时间过滤
          auto_aim_started_at = std::chrono::steady_clock::now();
          target_queue.push(std::nullopt);
        }
        last_mode = mode;
      }

      camera.read(img, t);
      if (img.empty()) {
        tools::logger()->warn("Camera frame empty! Waiting...");
        std::this_thread::sleep_for(1ms);
        continue;
      }

      watchdog.ping();
      if (rb_runtime::is_buff(mode)) continue;

      const auto q = gimbal.q(t);
      auto yolo_frame = yolo.detect(auto_aim::YOLOFrameData(img, q, t), frame_count++);
      if (yolo_frame.is_empty || yolo_frame.timestamp < auto_aim_started_at) continue;

      solver.set_R_gimbal2world(yolo_frame.gimbal_q);
      auto targets = tracker.track(yolo_frame.armors, yolo_frame.timestamp);

      if (targets.empty()) {
        target_queue.push(std::nullopt);
      } else {
        target_queue.push(std::optional<auto_aim::Target>(targets.front()));
      }
    }

    quit = true;
    target_queue.close();
    if (plan_thread.joinable()) plan_thread.join();
  } else {
    // ---------------- 单线程：同步推理，规划与下发都在主循环 ----------------
    cv::Mat img;
    std::chrono::steady_clock::time_point t;
    const auto t0 = std::chrono::steady_clock::now();
    std::uint16_t last_bullet_count = 0;
    bool was_buff = rb_runtime::is_buff(gimbal.mode());

    while (!exiter.exit()) {
      const auto mode = gimbal.mode();
      camera.read(img, t);
      if (img.empty()) {
        tools::logger()->warn("Camera frame empty! Waiting...");
        std::this_thread::sleep_for(1ms);
        continue;
      }

      watchdog.ping();
      const auto gs = gimbal.state();

      if (rb_runtime::is_buff(mode)) {
        if (!was_buff) {
          rb_runtime::send_stop(gimbal, opt.sb_send, gs.yaw / 57.3F, gs.pitch / 57.3F);
          was_buff = true;
        }
        continue;
      }
      was_buff = false;

      const auto q = gimbal.q(t);
      solver.set_R_gimbal2world(q);
      auto armors = yolo.detect(img);
      auto targets = tracker.track(armors, t);

      std::optional<auto_aim::Target> target = std::nullopt;
      if (!targets.empty()) target = targets.front();

      const auto plan_start = std::chrono::steady_clock::now();
      auto plan = planner.plan(
        target, gs.bullet_speed, gs.yaw, auto_aim::Planner::ShootStrategy::rbSuppressiveFire);
      const auto plan_end = std::chrono::steady_clock::now();
      const double plan_ms =
        std::chrono::duration<double, std::milli>(plan_end - plan_start).count();

      rb_runtime::send_plan(gimbal, plan, target, opt.sb_send);

      const bool fired = gs.bullet_count > last_bullet_count;
      last_bullet_count = gs.bullet_count;

      plot_plan(
        plan, target, gs, fired, plan_ms, std::chrono::steady_clock::duration::zero(), 0, t0);
    }
  }

  // 退出前释放控制：让下位机按当前角度保持，而不是继续执行上一帧自瞄指令
  const auto final_state = gimbal.state();
  rb_runtime::send_stop(gimbal, opt.sb_send, final_state.yaw / 57.3F, final_state.pitch / 57.3F);

  return 0;
}
