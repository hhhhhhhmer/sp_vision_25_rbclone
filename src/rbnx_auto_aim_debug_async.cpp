#include <fmt/core.h>

#include <atomic>
#include <chrono>
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
#include "tools/systemd_watchdog.hpp"
#include "tools/img_tools.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/plotter.hpp"
#include "tools/thread_safe_queue.hpp"
#include "tools/reprojection.hpp"

using namespace std::chrono_literals;

const std::string keys =
  "{help h usage ? |                         | 输出命令行参数说明}"
  "{@config-path   | ../configs/drone.yaml | 位置参数，yaml配置文件路径 }";

int main(int argc, char * argv[])
{
  tools::Exiter exiter;
  tools::Plotter plotter;

  cv::CommandLineParser cli(argc, argv, keys);
  auto config_path = cli.get<std::string>(0);
  if (cli.has("help") || config_path.empty()) {
    cli.printMessage();
    return 0;
  }

  io::Gimbal gimbal(config_path);
  io::Camera camera(config_path);

  auto_aim::YOLO yolo(config_path, true);
  auto_aim::Solver solver(config_path);
  auto_aim::Tracker tracker(config_path, &solver);
  tracker.set_gimbal(&gimbal);
  auto_aim::Planner planner(config_path);

  tools::ThreadSafeQueue<std::optional<auto_aim::Target>, true> target_queue(1);
  target_queue.push(std::nullopt);

  std::atomic<bool> quit = false;
  auto plan_thread = std::thread([&]() {
    auto t0 = std::chrono::steady_clock::now();
    uint16_t last_bullet_count = 0;

    while (!quit) {
      auto target = target_queue.front();
      auto gs = gimbal.state();

      auto plan = planner.plan(
        target, gs.bullet_speed, gs.yaw, auto_aim::Planner::ShootStrategy::rbSuppressiveFire);
      gimbal.send(
        plan.control, plan.fire, plan.yaw, plan.yaw_vel, plan.yaw_acc, plan.pitch,
        plan.pitch_vel, plan.pitch_acc);

      auto fired = gs.bullet_count > last_bullet_count;
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

  cv::Mat img;
  std::chrono::steady_clock::time_point t;
  std::chrono::steady_clock::time_point last_t;
  int frame_count = 0;
  tools::SystemdWatchdog systemd_watchdog;

  if (!systemd_watchdog.ready("Vision pipeline is ready")) {
    tools::logger()->warn("无法向 systemd 发送 READY 通知");
  }

  while (!exiter.exit()) {
    camera.read(img, t);
    if (img.empty()) continue;
    systemd_watchdog.ping();

    auto q = gimbal.q(t);
    if (last_t != std::chrono::steady_clock::time_point{}) {
      double fps =
        1. / std::chrono::duration_cast<std::chrono::microseconds>(t - last_t).count() * 1000000;
      tools::logger()->info("capture fps: {:.2f}", fps);
    }
    last_t = t;

    auto yolo_frame = yolo.detect(auto_aim::YOLOFrameData(img, q, t), frame_count++);
    if (yolo_frame.is_empty) {
      continue;
    }

    img = yolo_frame.frame;
    q = yolo_frame.gimbal_q;
    t = yolo_frame.timestamp;

    auto ypr = tools::eulers(q, 2, 1, 0);
    float yaw_deg = ypr[0] * 180.0 / M_PI;
    float pitch_deg = ypr[1] * 180.0 / M_PI;

    solver.set_R_gimbal2world(q);
    auto armors = yolo_frame.armors;
    auto targets = tracker.track(armors, t);

    tools::draw_text(img, fmt::format("rb_Yaw {:.2f}", yaw_deg), {40, 40}, {0, 128, 255});
    tools::draw_text(img, fmt::format("rb_Pitch {:.2f}", pitch_deg), {40, 80}, {0, 255, 255});

    if (targets.empty()) {
      target_queue.push(std::nullopt);
    } else {
      target_queue.push(targets.front());
      tools::draw_reprojection(
        img, solver, targets.front(), planner.debug_xyza, cv::Scalar(235, 206, 135));
    }

    cv::resize(img, img, {}, 0.5, 0.5);
    cv::imshow("reprojection", img);
    auto key = cv::waitKey(1);
    if (key == 'q') break;
  }

  quit = true;
  if (plan_thread.joinable()) plan_thread.join();

  auto current_state = gimbal.state();
  gimbal.send(
    false, false, current_state.yaw / 57.3f, 0.0f, 0.0f, current_state.pitch / 57.3f, 0.0f,
    0.0f);

  return 0;
}
