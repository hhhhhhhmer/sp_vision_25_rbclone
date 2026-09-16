#include <fmt/core.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <deque>
#include <mutex>
#include <nlohmann/json.hpp>
#include <opencv2/opencv.hpp>
#include <optional>
#include <thread>

#include "io/camera/camera.hpp"
#include "io/gimbal/gimbal.hpp"
#include "tasks/auto_aim/aiming/planner/planner.hpp"
#include "tasks/auto_aim/geometry/solver.hpp"
#include "tasks/auto_aim/tracking/tracker.hpp"
#include "tasks/auto_aim/detection/yolo.hpp"
#include "tools/exiter.hpp"
#include "tools/img_tools.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/plotter.hpp"
#include "tools/recorder.hpp"
#include "tools/systemd_watchdog.hpp"
#include "tools/thread_safe_queue.hpp"
#include "method_set/binocular_aim.hpp"

using namespace std::chrono_literals;

const std::string keys =
  "{help h usage ? |                           | 输出命令行参数说明}"
  "{short_camera   | ../configs/sb_short.yaml | 短焦相机配置文件路径}"
  "{long_camera    | ../configs/sb_long.yaml  | 长焦相机配置文件路径}"
  "{long_no_target_timeout | 1000 | 长焦连续无目标后切回短焦的超时时间(ms)}";

int main(int argc, char * argv[])
{
  tools::SystemdWatchdog systemd_watchdog;
  tools::Exiter exiter;
  tools::Plotter plotter;
  tools::Recorder recorder;

  cv::CommandLineParser cli(argc, argv, keys);
  if (cli.has("help")) {
    cli.printMessage();
    return 0;
  }

  const auto short_camera_config_path = cli.get<std::string>("short_camera");
  const auto long_camera_config_path = cli.get<std::string>("long_camera");
  const auto long_no_target_timeout_ms = cli.get<int>("long_no_target_timeout");
  if (
    short_camera_config_path.empty() || long_camera_config_path.empty() ||
    long_no_target_timeout_ms <= 0) {
    cli.printMessage();
    return 1;
  }
  const auto long_no_target_timeout = std::chrono::milliseconds(long_no_target_timeout_ms);

  io::Camera::initSDK();
  io::Camera short_camera(short_camera_config_path);
  io::Camera long_camera(long_camera_config_path);
  io::Gimbal gimbal(short_camera_config_path);

  auto_aim::YOLO yolo(short_camera_config_path,false);
  auto_aim::Solver short_camera_solver(short_camera_config_path);
  auto_aim::Solver long_camera_solver(long_camera_config_path);
  auto_aim::Tracker short_camera_tracker(short_camera_config_path, &short_camera_solver);
  auto_aim::Tracker long_camera_tracker(long_camera_config_path, &long_camera_solver);
  short_camera_tracker.set_gimbal(&gimbal);
  long_camera_tracker.set_gimbal(&gimbal);
  auto_aim::Planner short_camera_planner(short_camera_config_path);
  auto_aim::Planner long_camera_planner(long_camera_config_path);

  tools::FFTExample long_fft;
  long_camera_tracker.set_fft(&long_fft);
  tools::FFTExample short_fft;
  short_camera_tracker.set_fft(&short_fft);

  BinocularAim binocular_aim(
    short_camera, long_camera, short_camera_solver, long_camera_solver, short_camera_planner,
    long_camera_planner);

  tools::ThreadSafeQueue<std::optional<auto_aim::Target>, true> target_queue(1);
  target_queue.push(std::nullopt);

  std::mutex short_planner_mutex;
  std::mutex long_planner_mutex;
  std::atomic<bool> quit = false;
  auto plan_thread = std::thread([&]() {
    auto t0 = std::chrono::steady_clock::now();
    uint16_t last_bullet_count = 0;

    while (!quit) {
      auto target = target_queue.front();
      auto gs = gimbal.state();

      const bool use_short_planner = !target.has_value() || target->cam_is_short;
      auto & planner = use_short_planner ? short_camera_planner : long_camera_planner;
      auto & planner_mutex = use_short_planner ? short_planner_mutex : long_planner_mutex;

      uint8_t name = 0;
      float tx = 0.0f;
      float ty = 0.0f;

      if (target.has_value()) {
          name = static_cast<uint8_t>(target->name) + 1;
          tx = target->ekf_x()[0]; 
          ty = target->ekf_x()[2]; 

          // tools::logger()->info("{},{},{}", name,tx,ty);

        }

      auto_aim::Plan plan{false};
      {
        std::lock_guard<std::mutex> lock(planner_mutex);
        plan = planner.plan(
          target, gs.bullet_speed, gs.yaw,
          auto_aim::Planner::ShootStrategy::rbSuppressiveFire);
      }

    //    if (binocular_aim.force_control_frames > 0) {
    //     plan.control = true;
    //     binocular_aim.force_control_frames--;
    // }

     gimbal.sb_send(
        plan.control, plan.fire,
        plan.yaw, plan.yaw_vel, plan.yaw_acc,
        plan.pitch, plan.pitch_vel, plan.pitch_acc,
        tx,ty,name
      );    
      const auto fired = gs.bullet_count > last_bullet_count;
      last_bullet_count = gs.bullet_count;

      nlohmann::json data;
      data["t"] = tools::delta_time(std::chrono::steady_clock::now(), t0);
      data["gimbal_yaw"] = gs.yaw;
      data["gimbal_yaw_vel"] = gs.yaw_vel;
      data["gimbal_pitch"] = gs.pitch;
      data["gimbal_pitch_vel"] = gs.pitch_vel;
      data["q2yaw"] = gs.q2yaw;
      data["q2pitch"] = gs.q2pitch;
      data["target_yaw"] = plan.target_yaw;
      data["target_pitch"] = plan.target_pitch;
      data["plan_mode"] = plan.control ? (plan.fire ? 2 : 1) : 0;
      data["plan_yaw"] = plan.yaw / CV_PI * 180.;
      data["plan_yaw_vel"] = plan.yaw_vel;
      data["plan_yaw_acc"] = plan.yaw_acc;
      data["plan_pitch"] = plan.pitch * 57.3;
      data["plan_pitch_vel"] = plan.pitch_vel;
      data["plan_pitch_acc"] = plan.pitch_acc;
      data["fire"] = plan.fire ? 1 : 0;
      data["fired"] = fired ? 1 : 0;
      data["camera_is_short"] = use_short_planner ? 1 : 0;

      if (target.has_value()) {
        const auto ekf = target->ekf_x();
        data["target_z"] = ekf[4];
        data["target_vz"] = ekf[5];
        data["tower_h1"] = target->tower_armor_hs[0];
        data["tower_h2"] = target->tower_armor_hs[1];
        data["tower_h3"] = target->tower_armor_hs[2];
        data["tower_armor_h"] = target->tower_armor_h;
        data["ekf_x"] = ekf(0);
        data["ekf_vx"] = ekf(1);
        data["ekf_y"] = ekf(2);
        data["ekf_vy"] = ekf(3);
        data["ekf_z"] = ekf(4);
        data["ekf_vz"] = ekf(5);
        data["ekf_yaw"] = ekf(6) * 57.3;
        data["ekf_vyaw"] = ekf(7) * 57.3;
        data["ekf_r"] = ekf(8);
      }

      plotter.plot(data);
      std::this_thread::sleep_for(10ms);
    }
  });

  auto fft_thread = std::thread([&] {
    bool was_periodic = false;
    while (!quit) {
      const auto analysis_start = std::chrono::steady_clock::now();
      bool is_periodic = short_fft.analyze();
      if (is_periodic != was_periodic) {
        const double elapsed_ms = std::chrono::duration<double, std::milli>(
                                    std::chrono::steady_clock::now() - analysis_start)
                                    .count();
        if (is_periodic) {
          tools::logger()->info("[shortFFT] 检测到周期运动，分析耗时 {:.2f} ms", elapsed_ms);
        } else { 
          tools::logger()->info("[shortFFT] 周期运动已消失");
        }
        was_periodic = is_periodic;
      }
      is_periodic = long_fft.analyze();
      if (is_periodic != was_periodic) {
        const double elapsed_ms = std::chrono::duration<double, std::milli>(
                                    std::chrono::steady_clock::now() - analysis_start)
                                    .count();
        if (is_periodic) {
          tools::logger()->info("[long_FFT] 检测到周期运动，分析耗时 {:.2f} ms", elapsed_ms);
        } else {
          tools::logger()->info("[long_FFT] 周期运动已消失");
        }
        was_periodic = is_periodic;
      }
      for (int i = 0; i < 5 && !quit; ++i) std::this_thread::sleep_for(50ms);
    }
    
  });

  struct PendingFrame
  {
    std::chrono::steady_clock::time_point timestamp;
    bool is_short;
    std::uint64_t generation;
  };

  struct LongCameraWatchdog
  {
    bool active = false;
    std::uint64_t generation = 0;
    std::chrono::steady_clock::time_point last_target_at;
  };

  std::deque<PendingFrame> pending_frames;
  LongCameraWatchdog long_camera_watchdog;
  std::optional<std::chrono::steady_clock::time_point> last_tracker_timestamp;
  cv::Mat img;
  std::chrono::steady_clock::time_point t;
  std::chrono::steady_clock::time_point last_t;
  int frame_count = 0;

  if (!systemd_watchdog.ready("Vision pipeline is ready")) {
    tools::logger()->warn("无法向 systemd 发送 READY 通知");
  }

  auto force_long_camera_if_requested = [&]() {
    const bool force_long_camera =
      gimbal.state().mode == static_cast<uint8_t>(io::GimbalMode::LONG_FOCAL_LENGTH);
    if (!force_long_camera) return false;

    if (!binocular_aim.is_short) {
      if (
        !long_camera_watchdog.active ||
        long_camera_watchdog.generation != binocular_aim.generation()) {
        long_camera_watchdog.active = true;
        long_camera_watchdog.generation = binocular_aim.generation();
        long_camera_watchdog.last_target_at = std::chrono::steady_clock::now();
      }
      return false;
    }

    if (!binocular_aim.Switch(short_camera_tracker, true, false)) return false;

    long_camera_tracker.reset();
    long_camera_watchdog.active = true;
    long_camera_watchdog.generation = binocular_aim.generation();
    long_camera_watchdog.last_target_at = std::chrono::steady_clock::now();
    target_queue.push(std::nullopt);
    tools::logger()->info("[BinocularAim] 下位机 mode=4，强制切换并锁定长焦相机");
    return true;
  };

  while (!exiter.exit()) {
    if (force_long_camera_if_requested()) continue;

    binocular_aim.cameras.aim_ptr->read(img, t);
    if (img.empty()) continue;
    if (!systemd_watchdog.ping()) {
      tools::logger()->warn("无法向 systemd 发送 Watchdog 心跳");
    }

    const bool input_is_short = binocular_aim.is_short;
    const auto input_generation = binocular_aim.generation();
    const auto timestamp_offset =
      input_is_short ? short_camera.timestamp_offset : long_camera.timestamp_offset;
    auto q = gimbal.q(t - 3ms);
    // recorder.record(img, q, t, input_is_short ? "short" : "long");
    if (last_t != std::chrono::steady_clock::time_point{}) {
      const auto elapsed_us =
        std::chrono::duration_cast<std::chrono::microseconds>(t - last_t).count();
      if (elapsed_us > 0) {
        tools::logger()->info("capture fps: {:.2f}", 1000000. / elapsed_us);
      }
    }
    last_t = t;

    pending_frames.push_back({t, input_is_short, input_generation});
    auto yolo_frame = yolo.detect(auto_aim::YOLOFrameData(img, q, t), frame_count++);
    if (yolo_frame.is_empty) {
      continue;
    }

    const auto pending_frame = std::find_if(
      pending_frames.begin(), pending_frames.end(), [&](const PendingFrame & frame) {
        return frame.timestamp == yolo_frame.timestamp;
      });
    if (pending_frame == pending_frames.end()) {
      tools::logger()->warn("[BinocularAim] 无法确定异步检测结果的相机来源");
      continue;
    }

    const bool frame_is_short = pending_frame->is_short;
    const auto frame_generation = pending_frame->generation;
    pending_frames.erase(pending_frame);

    // YOLO 异步推理期间 mode 可能变为 4，在处理该帧前再检查一次。
    if (force_long_camera_if_requested()) continue;

    img = yolo_frame.frame;
    q = yolo_frame.gimbal_q;
    t = yolo_frame.timestamp;

    if (
      frame_generation != binocular_aim.generation() ||
      frame_is_short != binocular_aim.is_short) {
      tools::logger()->debug(
        "[BinocularAim] 丢弃切换前的异步结果: frame_generation={}, current_generation={}",
        frame_generation, binocular_aim.generation());
      continue;
    }

    if (last_tracker_timestamp.has_value() && t <= *last_tracker_timestamp) {
      tools::logger()->warn("[BinocularAim] 丢弃时间戳未递增的检测结果");
      continue;
    }
    last_tracker_timestamp = t;

    auto & frame_solver = frame_is_short ? short_camera_solver : long_camera_solver;
    auto & frame_tracker =
      frame_is_short ? short_camera_tracker : long_camera_tracker;
    auto & frame_planner = frame_is_short ? short_camera_planner : long_camera_planner;
    auto & frame_planner_mutex = frame_is_short ? short_planner_mutex : long_planner_mutex;

    frame_solver.set_R_gimbal2world(q);

    auto armors = std::move(yolo_frame.armors);
    auto targets = frame_tracker.track(armors, t, frame_is_short);

    if (long_camera_watchdog.active) {
      if (
        frame_generation != long_camera_watchdog.generation || frame_is_short ||
        binocular_aim.is_short) {
        long_camera_watchdog.active = false;
      } else {
        const auto now = std::chrono::steady_clock::now();
        const bool force_long_camera =
          gimbal.state().mode == static_cast<uint8_t>(io::GimbalMode::LONG_FOCAL_LENGTH);
        if (force_long_camera || !armors.empty()) {
          // mode=4 期间持续刷新，避免无目标超时将长焦切回短焦。
          long_camera_watchdog.last_target_at = now;
        } else {
          const auto no_target_elapsed = now - long_camera_watchdog.last_target_at;
          if (
            no_target_elapsed >= long_no_target_timeout &&
            binocular_aim.Switch(frame_tracker, true, false)) {
            const auto no_target_ms =
              std::chrono::duration_cast<std::chrono::milliseconds>(no_target_elapsed).count();
            tools::logger()->warn(
              "[BinocularAim] 长焦连续 {}ms 未检测到目标，回退短焦", no_target_ms);
            long_camera_watchdog.active = false;
            short_camera_tracker.reset();
            target_queue.push(std::nullopt);
            continue;
          }
        }
      }
    }

    
    const auto ypr = tools::eulers(q, 2, 1, 0);
    const float yaw_deg = ypr[0] * 180.0 / M_PI;
    const float pitch_deg = ypr[1] * 180.0 / M_PI;
    tools::draw_text(img, fmt::format("rb_Yaw {:.2f}", yaw_deg), {40, 40}, {0, 128, 255});
    tools::draw_text(
      img, fmt::format("rb_Pitch {:.2f}", pitch_deg), {40, 80}, {0, 255, 255});
    tools::draw_text(
      img, frame_is_short ? "Camera: short" : "Camera: long", {40, 120}, {255, 255, 0});

    if (targets.empty()) {
      target_queue.push(std::nullopt);
    } else {
      target_queue.push(targets.front());
      auto & target = targets.front();
      const auto ekf_x = target.getEKFXest();

      const Eigen::Vector3d center_world(ekf_x[0], ekf_x[2], ekf_x[4]);
      const Eigen::Vector3d velocity(ekf_x[1], ekf_x[3], ekf_x[5]);
      const Eigen::Vector3d pred_center = center_world + velocity * 0.5;
      Eigen::Vector3d v_yaw_axis_tvec = center_world;
      v_yaw_axis_tvec[2] += ekf_x[7] * 0.1;

      const auto center_img =
        frame_solver.reproject_armor(center_world, 0.0, target.armor_type, target.name);
      const auto pred_point_img =
        frame_solver.reproject_armor(pred_center, 0.0, target.armor_type, target.name);
      const auto v_yaw_axis_point_img =
        frame_solver.reproject_armor(v_yaw_axis_tvec, 0.0, target.armor_type, target.name);

      if (!center_img.empty() && !pred_point_img.empty()) {
        cv::circle(img, center_img[0], 5, cv::Scalar(51, 153, 237), -1);
        cv::circle(img, pred_point_img[0], 8, cv::Scalar(0, 0, 255), -1);
        cv::line(img, center_img[0], pred_point_img[0], cv::Scalar(0, 255, 255), 2);
        if (!v_yaw_axis_point_img.empty()) {
          cv::line(img, center_img[0], v_yaw_axis_point_img[0], cv::Scalar(0, 255, 0), 2);
        }
      }

      for (const auto & xyza : target.armor_xyza_list()) {
        const auto image_points =
          frame_solver.reproject_armor(xyza.head(3), xyza[3], target.armor_type, target.name);
        tools::draw_points(img, image_points, {235, 206, 135});
      }

      Eigen::Vector4d aim_xyza;
      {
        std::lock_guard<std::mutex> lock(frame_planner_mutex);
        aim_xyza = frame_planner.debug_xyza;
      }
      const auto aim_points = frame_solver.reproject_armor(
        aim_xyza.head(3), aim_xyza[3], target.armor_type, target.name);
      tools::draw_points(img, aim_points, {0, 0, 255});

      const bool force_long_camera =
        gimbal.state().mode == static_cast<uint8_t>(io::GimbalMode::LONG_FOCAL_LENGTH);
      if (
        !force_long_camera &&
        binocular_aim.ChangeTheScope(target, frame_tracker, false)) {
        auto & activated_tracker =
          binocular_aim.is_short ? short_camera_tracker : long_camera_tracker;
        activated_tracker.reset();
        target_queue.push(std::nullopt);

        if (binocular_aim.is_short) {
          long_camera_watchdog.active = false;
        } else {
          long_camera_watchdog.active = true;
          long_camera_watchdog.generation = binocular_aim.generation();
          long_camera_watchdog.last_target_at = std::chrono::steady_clock::now();
          tools::logger()->info(
            "[BinocularAim] 已切换长焦，{}ms 内无目标将回退短焦",
            long_no_target_timeout_ms);
        }
      }
    }
    

    cv::resize(img, img, {}, 0.5, 0.5);
    cv::imshow("reprojection", img);
    const auto key = cv::waitKey(1);
    if (key == 'q') break;
    if (key == 'c') binocular_aim.Switch(frame_tracker, true, false);
  }

  if (fft_thread.joinable()) fft_thread.join();


  quit = true;
  if (plan_thread.joinable()) plan_thread.join();

  const auto current_state = gimbal.state();
  gimbal.send(
    false, false, current_state.yaw / 57.3f, 0.0f, 0.0f, current_state.pitch / 57.3f, 0.0f,
    0.0f);

  return 0;
}
