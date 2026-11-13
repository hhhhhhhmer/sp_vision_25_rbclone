// 图像—姿态时间失配 τ 的现场自标定程序。
//
// 做法：把装甲板摆在静止位置（靶车/架子都行），让云台左右快速摆几秒。
// 静止目标在云台系下的位置 p_g 与姿态无关，只有把 τ 取对，
// p_w(τ) = R(q(t-τ))·p_g 才对时间保持不变；于是"世界系离散度最小的 τ"就是真实失配。
//
// 摆动有两种方式：
//   手动：按空格后自己用遥控摆（程序不碰云台）
//   自动：加 --auto，程序先对准目标再自己按正弦摆动（遥控需切到自瞄模式）
//
// 得到的 τ 包含曝光、读出、USB/网口传输、串口、调度在内的全部固定延迟，
// 直接写进 YAML 的 timestamp_offset_us，把姿态查询改成 gimbal.q(t - offset) 即可。
//
// 用法：
//   ./build/timestamp_calib_test ../configs/rb_auto_aim.yaml            # 手动摆
//   ./build/timestamp_calib_test ../configs/rb_auto_aim.yaml --auto     # 自动摆
//   空格 开始/重新采样，采样期间任意键中止（自动模式会立刻放开云台），q 退出。
//   --detector traditional 退回传统视觉检测器；默认 yolo，与自瞄主程序同一套检测。

#include "tools/timestamp_calib.hpp"

#include <fmt/core.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <list>
#include <memory>
#include <mutex>
#include <opencv2/opencv.hpp>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include "io/camera/camera.hpp"
#include "io/gimbal/gimbal.hpp"
#include "tasks/auto_aim/detection/detector.hpp"
#include "tasks/auto_aim/detection/yolo.hpp"
#include "tasks/auto_aim/geometry/solver.hpp"
#include "tools/exiter.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"

using namespace std::chrono_literals;
using namespace tools;

const std::string keys =
  "{help h usage ? |                             | 输出命令行参数说明}"
  "{@config-path   | ../configs/sb_short.yaml | 位置参数，yaml 配置文件路径}"
  "{d duration     | 3.0                         | 采样时长(s)，自动模式下是摆动时长}"
  "{auto a         | true                        | 自动控制云台摆动（需把遥控切到自瞄模式）}"
  "{amplitude      | 20.0                        | 自动摆动的幅度(deg)，限制在 5~40}"
  "{freq           | 1.2                         | 自动摆动的频率(Hz)，限制在 0.3~3}"
  "{detector       | yolo                        | 检测器后端：yolo（默认，与自瞄一致）或 traditional}"
  "{sb s           | true                        | 用哨兵帧 sb_send 下发（默认用带 CRC 的 send）}";

namespace
{

/** @brief 自动摆动的对准时间（s）：先平滑转到目标方位，再开始正弦摆动 */
constexpr double kAlignSeconds = 0.5;

/** @brief 云台自动摆动控制器
 *
 * 只做一件事：以 200 Hz 下发 yaw = 中心角 + A·sin(2πft) 的绝对世界角（带前馈速度/加速度），
 * 并在结束/中止/析构时立刻发 mode=0 的释放帧，把云台交回遥控。
 * 板子只有在"自瞄模式"下才会接受视觉指令，所以自动模式需要先把遥控切到自瞄。
 */
class SweepController
{
public:
  /** @brief 构造并启动下发线程 @param gimbal 云台接口 @param use_sb 是否用哨兵帧（带目标位置） */
  SweepController(io::Gimbal & gimbal, bool use_sb) : gimbal_(gimbal), use_sb_(use_sb)
  {
    thread_ = std::thread([this] { run(); });
  }

  ~SweepController()
  {
    quit_ = true;
    if (thread_.joinable()) thread_.join();
    release();
  }

  /** @brief 开始一次摆动：先对准到目标方位，再绕该方位正弦摆动
   *  @param yaw_now 当前云台 yaw（rad）
   *  @param yaw_center 目标方位（rad，绝对世界角）
   *  @param pitch_hold 保持的 pitch（rad）
   *  @param amp_rad 摆幅（rad）
   *  @param freq_hz 摆动频率（Hz）
   *  @param sweep_s 摆动时长（s）
   */
  void start(
    double yaw_now, double yaw_center, double pitch_hold, double amp_rad, double freq_hz,
    double sweep_s)
  {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      profile_.yaw_now = yaw_now;
      profile_.yaw_center = yaw_center;
      profile_.amp = amp_rad;
      profile_.freq = freq_hz;
      profile_.align_s = kAlignSeconds;
      profile_.sweep_s = sweep_s;
      pitch_hold_ = pitch_hold;
      t0_ = std::chrono::steady_clock::now();
    }
    armed_ = true;
    released_ = false;
    finished_ = false;
    active_ = true;
  }

  /** @brief 中止摆动并放开云台 */
  void abort() { active_ = false; }

  /** @brief 摆动是否已自然结束 */
  bool finished() const { return finished_.load(); }
  /** @brief 是否正在控制云台 */
  bool active() const { return active_.load(); }
  /** @brief 摆动总时长（对准 + 摆动，s） */
  double total_seconds() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return profile_.total_s();
  }

private:
  void run()
  {
    const auto period = std::chrono::milliseconds(5);
    auto next = std::chrono::steady_clock::now();

    while (!quit_) {
      next += period;

      double yaw = 0, vel = 0, acc = 0, pitch = 0;
      bool send = false;

      if (active_) {
        std::lock_guard<std::mutex> lock(mutex_);
        const auto cmd = profile_.at(tools::delta_time(std::chrono::steady_clock::now(), t0_));
        pitch = pitch_hold_;
        if (cmd.done) {
          active_ = false;
          finished_ = true;
        } else {
          yaw = cmd.yaw;
          vel = cmd.vel;
          acc = cmd.acc;
          send = true;
        }
      }

      if (send) {
        send_command(yaw, vel, acc, pitch);
      } else if (!active_ && armed_ && !released_) {
        release();
      }

      std::this_thread::sleep_until(next);
      if (std::chrono::steady_clock::now() - next > std::chrono::milliseconds(200)) {
        next = std::chrono::steady_clock::now();  // 线程被长时间挂起时重新对齐节拍，避免补发
      }
    }
  }

  void send_command(double yaw, double vel, double acc, double pitch)
  {
    if (use_sb_) {
      gimbal_.sb_send(
        true, false, static_cast<float>(yaw), static_cast<float>(vel), static_cast<float>(acc),
        static_cast<float>(pitch), 0.0f, 0.0f, 0.0f, 0.0f, 0);
    } else {
      gimbal_.send(
        true, false, static_cast<float>(yaw), static_cast<float>(vel), static_cast<float>(acc),
        static_cast<float>(pitch), 0.0f, 0.0f);
    }
  }

  /** @brief 放开云台：连发三帧 mode=0，避免单帧丢失后板子仍处于视觉接管状态 */
  void release()
  {
    if (!armed_.exchange(false)) return;
    released_ = true;
    for (int i = 0; i < 3; ++i) {
      if (use_sb_) {
        gimbal_.sb_send(false, false, 0, 0, 0, 0, 0, 0, 0, 0, 0);
      } else {
        gimbal_.send(false, false, 0, 0, 0, 0, 0, 0);
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
  }

  io::Gimbal & gimbal_;
  bool use_sb_;

  std::thread thread_;
  std::atomic<bool> quit_{false};
  std::atomic<bool> active_{false};
  std::atomic<bool> finished_{false};
  std::atomic<bool> armed_{false};
  std::atomic<bool> released_{true};

  mutable std::mutex mutex_;
  tools::SweepProfile profile_;
  double pitch_hold_ = 0;
  std::chrono::steady_clock::time_point t0_{};
};

/** @brief 打印标定报告（同时给出可粘贴的 YAML 项） */
void print_report(
  const TimestampCalibResult & r, const std::string & config_path, double window_s, bool auto_mode)
{
  std::printf("\n================ 时间戳标定结果 ================\n");
  std::printf("采样: %d 帧，剔除 %d，参与拟合 %d\n", r.frames, r.rejected, r.used);
  std::printf(
    "云台激励: RMS |w| = %.2f rad/s，峰值 %.2f rad/s（采样 %.1f s）\n", r.excitation_rad_s,
    r.excitation_peak_rad_s, window_s);

  if (!r.ok) {
    std::printf("\n[不可信] %s\n", r.reason.c_str());
    if (r.excitation_rad_s < 0.5) {
      if (auto_mode) {
        std::printf("  -> 自动模式下发指令但云台没怎么动：检查遥控是否已切到自瞄模式、\n");
        std::printf("     电控是否执行视觉角度；必要时先用遥控确认手动能摆起来\n");
      } else {
        std::printf("  -> 请重来一次：采样开始后立刻用遥控让云台左右快速摆（幅度 ±20°~30°）\n");
      }
    }
    if (std::abs(r.tau_s) > 0.019) {
      std::printf("  -> tau 顶到搜索边界，检查是否用错了相机/云台，或姿态历史没有覆盖采样段\n");
    }
    std::printf(
      "  （仍给出参考值 tau = %+.2f ms +- %.2f ms，仅供参考）\n", r.tau_s * 1e3, r.sigma_s * 1e3);
  } else {
    std::printf("\ntau = %+.2f ms  +- %.2f ms\n", r.tau_s * 1e3, r.sigma_s * 1e3);
    std::printf(
      "世界系离散度: %.1f cm (tau=0)  ->  %.1f cm (tau=tau)\n", r.rms_at_zero_m * 100,
      r.rms_at_tau_m * 100);
    if (std::isfinite(r.tau_first_half_s) && std::isfinite(r.tau_second_half_s)) {
      std::printf(
        "前后半段一致性: %+.2f / %+.2f ms（差 %.2f ms）\n", r.tau_first_half_s * 1e3,
        r.tau_second_half_s * 1e3, std::abs(r.tau_first_half_s - r.tau_second_half_s) * 1e3);
    }

    std::printf("\ntau-RMS 曲线（ms / cm）:\n");
    if (!r.curve.empty()) {
      const std::size_t stride = std::max<std::size_t>(1, r.curve.size() / 12);
      for (std::size_t i = 0; i < r.curve.size(); i += stride) {
        std::printf("  %+7.2f  %6.2f\n", r.curve[i].first * 1e3, r.curve[i].second * 100);
      }
      const auto & best = *std::min_element(
        r.curve.begin(), r.curve.end(),
        [](const auto & a, const auto & b) { return a.second < b.second; });
      std::printf("  %+7.2f  %6.2f  <== 最优\n", best.first * 1e3, best.second * 100);
    }

    const long long offset_us = std::llround(r.tau_s * 1e6);
    std::printf("\n落地方法:\n");
    std::printf("  1) %s 里写 timestamp_offset_us: %lld\n", config_path.c_str(), offset_us);
    std::printf("  2) 姿态查询改成 gimbal.q(t - <timestamp_offset>)\n");
    std::printf(
      "     当前 debug 程序硬编码的 t - 3ms 等价于 tau = 3 ms；上场程序 rb_auto_standard 用的是 "
      "t，等价于 tau = 0\n");
  }
  std::printf("说明: tau > 0 表示图像真实曝光时刻早于它的时间戳，姿态应在 t-tau 处查询\n");
  std::printf("================================================\n\n");
}

/** @brief 在画面上写一行文字（英文字体，中文在终端输出） */
void put(cv::Mat & img, const std::string & text, int y, const cv::Scalar & color)
{
  cv::putText(img, text, cv::Point(20, y), cv::FONT_HERSHEY_SIMPLEX, 0.7, color, 2);
}

}  // namespace

int main(int argc, char * argv[])
{
  cv::CommandLineParser cli(argc, argv, keys);
  auto config_path = cli.get<std::string>(0);
  if (cli.has("help") || config_path.empty()) {
    cli.printMessage();
    return 0;
  }
  const double window_s = std::clamp(cli.get<double>("duration"), 1.0, 10.0);
  const bool auto_mode = cli.get<bool>("auto");
  const bool use_sb = cli.get<bool>("sb");
  const double amp_deg = std::clamp(cli.get<double>("amplitude"), 5.0, 40.0);
  const double freq_hz = std::clamp(cli.get<double>("freq"), 0.3, 3.0);
  const double amp_rad = amp_deg * M_PI / 180.0;

  tools::Exiter exiter;
  io::Camera camera(config_path);
  io::Gimbal gimbal(config_path);

  // 检测器：自瞄主程序用的是 YOLO，这里默认跟它保持一致，避免传统视觉在现场光照下
  // 检不到装甲板；--detector traditional 可以退回旧路径做对照。
  const std::string detector_name = cli.get<std::string>("detector");
  const bool use_yolo = detector_name != "traditional";
  if (detector_name != "yolo" && detector_name != "traditional") {
    tools::logger()->warn(
      "[calib] 未知检测器 \"{}\"，按 yolo 处理（可选：yolo / traditional）", detector_name);
  }
  std::unique_ptr<auto_aim::YOLO> yolo;
  std::unique_ptr<auto_aim::Detector> traditional;
  if (use_yolo) {
    yolo = std::make_unique<auto_aim::YOLO>(config_path, false);
  } else {
    traditional = std::make_unique<auto_aim::Detector>(config_path, false);
  }
  // 标定必须用同步检测：异步流水线返回的是流水线里较早的那一帧，会和当前帧时间戳错配
  auto detect_armors = [&](const cv::Mat & image, int frame_index) -> std::list<auto_aim::Armor> {
    if (yolo) return yolo->detect(image, frame_index);
    return traditional->detect(image, frame_index);
  };

  auto_aim::Solver solver(config_path);

  // 声明顺序很重要：sweep 必须比 gimbal 先析构，析构时才能把云台放开
  SweepController sweep(gimbal, use_sb);

  tools::logger()->info("[calib] 检测器: {}", use_yolo ? "yolo" : "traditional");

  if (auto_mode) {
    tools::logger()->warn(
      "[calib] 自动模式：程序会接管云台并做 ±{:.0f}° 的正弦摆动（{:.2f} Hz，{:.1f} s）", amp_deg,
      freq_hz, window_s);
    tools::logger()->warn(
      "[calib] 请确认车已固定/周围无人，并把遥控切到自瞄模式（否则下位机不执行视觉角度）");
  } else {
    tools::logger()->info("[calib] 手动模式：程序不会发送任何控制帧，用遥控摆云台");
  }

  tools::TimestampCalibrator calib;
  tools::TimestampCalibConfig calib_cfg;

  std::optional<double> ref_dist;  // 选中装甲板的参考距离，用于跨帧锁定同一块板
  bool capturing = false;
  std::chrono::steady_clock::time_point capture_begin{};
  std::chrono::steady_clock::time_point last_valid_t{};
  int frame_count = 0;
  int selected_frames = 0;
  std::string last_message = auto_mode
                               ? "SPACE: auto sweep (gimbal will move!), q: quit"
                               : "SPACE: start capture (sweep gimbal L/R by remote), q: quit";

  cv::Mat img;
  std::chrono::steady_clock::time_point t;

  while (!exiter.exit()) {
    camera.read(img, t);
    const auto armors = detect_armors(img, frame_count++);

    // 选一块"始终是同一块"的装甲板：解算成功 + 距离合理 + 与参考距离最接近
    std::optional<auto_aim::Armor> chosen;
    double chosen_cost = 1e18;
    for (const auto & armor : armors) {
      auto solved = armor;
      if (!solver.try_solve(solved)) continue;
      const double dist = solved.xyz_in_gimbal.norm();
      if (dist < 1.0 || dist > 12.0) continue;
      const double cost = ref_dist ? std::abs(dist - *ref_dist) : 0.0;
      if (!chosen || cost < chosen_cost) {
        chosen = solved;
        chosen_cost = cost;
      }
    }

    bool valid = false;
    if (chosen) {
      const double dist = chosen->xyz_in_gimbal.norm();
      if (!ref_dist) {
        ref_dist = dist;
        valid = true;
      } else if (std::abs(dist - *ref_dist) <= 0.35) {
        *ref_dist = 0.9 * (*ref_dist) + 0.1 * dist;  // 缓慢跟随
        valid = true;
      }
    }

    if (capturing) {
      if (valid) {
        calib.add_sample(t, chosen->xyz_in_gimbal);
        ++selected_frames;
        last_valid_t = t;
      }

      const double elapsed = tools::delta_time(t, capture_begin);
      const double total_s = auto_mode ? sweep.total_seconds() : window_s;
      const bool target_lost =
        auto_mode && tools::delta_time(t, last_valid_t) > 0.6 && selected_frames > 0;
      const bool done = auto_mode ? sweep.finished() : (elapsed >= total_s);
      // 安全超时：自动模式下若控制器因故没有报告结束，也不能无限把云台攥在手里
      const bool timeout = auto_mode && elapsed > total_s + 3.0;

      if (target_lost) {
        sweep.abort();
        capturing = false;
        last_message = "ABORTED: target lost (see terminal)";
        tools::logger()->warn("[calib] 采样中目标丢失超过 0.6 s，已中止并放开云台");
      } else if (done || timeout) {
        if (timeout) tools::logger()->warn("[calib] 摆动超时，强制结束并放开云台");
        capturing = false;
        if (auto_mode) sweep.abort();  // 正常结束时 run 线程已经释放，这里只是兜底
        const auto result = calib.fit(gimbal.q_history(), calib_cfg);
        print_report(result, config_path, total_s, auto_mode);
        last_message =
          result.ok
            ? fmt::format("tau = {:+.2f} ms +- {:.2f} ms", result.tau_s * 1e3, result.sigma_s * 1e3)
            : "calibration FAILED (see terminal)";
      }
    }

    // 画面提示
    if (chosen) {
      for (std::size_t i = 0; i < chosen->points.size(); ++i) {
        cv::circle(
          img, chosen->points[i], 4, valid ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 0, 255), 2);
      }
    }
    put(img, last_message, 30, cv::Scalar(0, 255, 255));
    if (capturing) {
      const double elapsed = tools::delta_time(t, capture_begin);
      const double total_s = auto_mode ? sweep.total_seconds() : window_s;
      put(
        img,
        fmt::format(
          "{} {:.1f}/{:.1f}s  frames={}  valid={}", auto_mode ? "AUTO SWEEP" : "CAPTURING", elapsed,
          total_s, calib.size(), selected_frames),
        62, cv::Scalar(0, 0, 255));
      put(
        img,
        auto_mode ? "gimbal under auto control - any key to abort"
                  : "sweep the gimbal left/right, keep the target still",
        94, cv::Scalar(0, 0, 255));
    } else {
      put(
        img, fmt::format("q history: {} samples", gimbal.q_history_size()), 62,
        cv::Scalar(200, 200, 200));
    }
    if (chosen) {
      put(
        img, fmt::format("target: {:.2f} m", chosen->xyz_in_gimbal.norm()), 94,
        cv::Scalar(0, 255, 0));
    } else {
      put(img, "target lost", 94, cv::Scalar(0, 0, 255));
    }

    cv::resize(img, img, {}, 0.5, 0.5);
    cv::imshow("timestamp_calib", img);
    const int key = cv::waitKey(1);
    if (key == 'q' || key == 27) break;

    if (capturing && auto_mode && key >= 0) {  // 自动模式下任意键中止
      sweep.abort();
      capturing = false;
      last_message = "ABORTED (see terminal)";
      tools::logger()->warn("[calib] 用户中止，已放开云台");
      continue;
    }

    if (key == ' ') {
      // 自动模式先算目标方位，再让控制器接管
      double yaw_now = 0, pitch_now = 0;
      {
        const auto state = gimbal.state();
        yaw_now = state.yaw / 57.3;
        pitch_now = state.pitch / 57.3;
      }

      double yaw_center = yaw_now;
      if (chosen) {
        // p_g 在云台系下：x 轴即枪管指向，因此目标方位 = 当前 yaw + 云台系内偏角
        double bearing = std::atan2(chosen->xyz_in_gimbal.y(), chosen->xyz_in_gimbal.x());
        // 安全钳位：即使目标在视野边缘，也不让摆动中心偏离当前角度超过 45°
        const double max_bearing = 45.0 * M_PI / 180.0;
        bearing = std::clamp(bearing, -max_bearing, max_bearing);
        yaw_center = yaw_now + bearing;
      } else if (auto_mode) {
        tools::logger()->warn("[calib] 没看到目标就开始了：摆动中心取当前云台角度");
      }

      calib.clear();
      selected_frames = 0;
      ref_dist.reset();
      capturing = true;
      capture_begin = t;
      last_valid_t = t;

      if (auto_mode) {
        sweep.start(yaw_now, yaw_center, pitch_now, amp_rad, freq_hz, window_s);
        last_message = "AUTO SWEEP ...";
        tools::logger()->info(
          "[calib] 自动摆动开始：中心 {:.1f}°，摆幅 ±{:.0f}°，{:.2f} Hz，{:.1f} s",
          yaw_center * 180.0 / M_PI, amp_deg, freq_hz, window_s);
      } else {
        last_message = "CAPTURING ...";
        tools::logger()->info("[calib] 开始采样 {:.1f}s：请用遥控左右快速摆云台", window_s);
      }
    }
  }

  sweep.abort();
  return 0;
}
