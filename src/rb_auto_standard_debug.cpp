#include <fmt/core.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <nlohmann/json.hpp>
#include <opencv2/opencv.hpp>
#include <thread>

#include "io/camera/camera.hpp"
#include "io/gimbal/gimbal.hpp"
#include "tasks/auto_aim/aiming/planner/planner.hpp"
#include "tasks/auto_aim/geometry/solver.hpp"
#include "tasks/auto_aim/tracking/tracker.hpp"
#include "tasks/auto_aim/aiming/aimer.hpp"
#include "tasks/auto_aim/aiming/shooter.hpp"
#include "tasks/auto_aim/detection/yolo.hpp"
#include "tools/exiter.hpp"
#include "tools/systemd_watchdog.hpp"
#include "tools/img_tools.hpp"
#include "tools/reprojection.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/plotter.hpp"
#include "tools/thread_safe_queue.hpp"
#include "tools/recorder.hpp"

#include "tasks/auto_buff/rune_debug_draw.hpp"
#include "tasks/auto_buff/rune_system.hpp"

using namespace std::chrono_literals;
using namespace tools;

const std::string keys =
  "{help h usage ? |                        | 输出命令行参数说明}"
  "{@config-path   | ../configs/xiaohei.yaml | 位置参数，yaml配置文件路径 }";

int main(int argc, char * argv[])
{
  tools::SystemdWatchdog systemd_watchdog;
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

  // 自瞄相关对象初始化
  auto_aim::YOLO yolo(config_path, true);
  auto_aim::Solver solver(config_path);
  auto_aim::Tracker tracker(config_path, &solver);
  tracker.set_gimbal(&gimbal);
  auto_aim::Aimer aimer(config_path);
  auto_aim::Shooter shooter(config_path);
  auto_aim::Planner planner(config_path);
  // tools::Recorder recor(90);
  bool stopkey = false;

  auto_buff::RuneSystem rune_system(config_path);

  tools::ThreadSafeQueue<std::optional<auto_aim::Target>, true> target_queue(1);
  target_queue.push(std::nullopt);

  std::atomic<bool> quit = false;
  std::atomic<io::GimbalMode> mode{io::GimbalMode::IDLE}; // 全局云台模式

  auto plan_thread = std::thread([&]() {
    auto t0 = std::chrono::steady_clock::now();
    uint16_t last_bullet_count = 0;

    while (!quit) {
      // 【修改】非打符模式，全认为是自瞄
      if (mode.load() != io::GimbalMode::SMALL_BUFF && mode.load() != io::GimbalMode::BIG_BUFF) {
        auto target = target_queue.front(); 
        auto gs = gimbal.state();

        // MPC预测以及+自家火控
        auto plan = planner.plan(target, gs.bullet_speed, gs.yaw,  auto_aim::Planner::ShootStrategy::rbSuppressiveFire);

        gimbal.send(
          plan.control, plan.fire, plan.yaw, plan.yaw_vel, plan.yaw_acc, plan.pitch, plan.pitch_vel,
          plan.pitch_acc);      
       
        auto fired = gs.bullet_count > last_bullet_count;
        last_bullet_count = gs.bullet_count;

        nlohmann::json data;
        data["t"] = tools::delta_time(std::chrono::steady_clock::now(), t0);

        data["gimbal_yaw"] = gs.yaw;
        data["gimbal_yaw_vel"] = gs.yaw_vel;
        data["gimbal_pitch"] = gs.pitch;
        data["gimbal_pitch_vel"] = gs.pitch_vel;

        data["target_yaw"] = plan.target_yaw;
        data["target_pitch"] = plan.target_pitch;

        data["plan_mode"] = plan.control ? (plan.fire ? 2 : 1) : 0;
        data["plan_yaw"] = plan.yaw / CV_PI * 180. ;
        data["plan_yaw_vel"] = plan.yaw_vel;
        data["plan_yaw_acc"] = plan.yaw_acc;

        data["plan_pitch"] = plan.pitch * 57.3;
        data["plan_pitch_vel"] = plan.pitch_vel;
        data["plan_pitch_acc"] = plan.pitch_acc;

        data["fire"] = plan.fire ? 1 : 0;
        data["fired"] = fired ? 1 : 0;

        if (target.has_value()) {
          data["target_z"] = target->ekf_x()[4];   //z
          data["target_vz"] = target->ekf_x()[5];  //vz
          data["tower_h1"] = target->tower_armor_hs[0];
          data["tower_h2"] = target->tower_armor_hs[1];
          data["tower_h3"] = target->tower_armor_hs[2];
          data["tower_armor_h"] = target->tower_armor_h;

          const auto ekf_satic = target->ekf_x();
          data["ekf_x"] = ekf_satic(0);
          data["ekf_vx"] = ekf_satic(1);
          data["ekf_y"] = ekf_satic(2);
          data["ekf_vy"] = ekf_satic(3);
          data["ekf_z"] = ekf_satic(4);
          data["ekf_vz"] = ekf_satic(5);
          data["ekf_yaw"] = ekf_satic(6) * 57.3;
          data["ekf_vyaw"] = ekf_satic(7) * 57.3;
          data["ekf_r"] = ekf_satic(8);        
        }

        plotter.plot(data);

        std::this_thread::sleep_for(10ms);
      } else {
        // 若是打符模式，由主循环发送指令，发送线程仅休眠
        std::this_thread::sleep_for(10ms);
      }
    }
  });

  auto t0_main = std::chrono::steady_clock::now();
  uint16_t last_bullet_count_main = 0;

  cv::Mat img;
  std::chrono::steady_clock::time_point t;
  std::chrono::steady_clock::time_point last_t;
  auto last_mode{io::GimbalMode::IDLE}; // 记录上次模式

  if (!systemd_watchdog.ready("Vision pipeline is ready")) {
    tools::logger()->warn("无法向 systemd 发送 READY 通知");
  }

  while (!exiter.exit()) {
    mode = gimbal.mode(); // 每帧获取最新云台模式
    
    if (last_mode != mode.load()) {
      tools::logger()->info("Switch to {}", gimbal.str(mode.load()));
      const bool was_buff = last_mode == io::GimbalMode::SMALL_BUFF ||
                            last_mode == io::GimbalMode::BIG_BUFF;
      const bool is_buff = mode.load() == io::GimbalMode::SMALL_BUFF ||
                           mode.load() == io::GimbalMode::BIG_BUFF;
      // 打符期间主循环不再更新 target_queue，先清空避免切回自瞄时 plan_thread 用到过期目标
      if (is_buff) target_queue.push(std::nullopt);
      if (was_buff && !is_buff) {
        rune_system.reset();
        gimbal.send(false, false, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
      }
      last_mode = mode.load();
    }

    camera.read(img, t);

    if (img.empty()) {
      tools::logger()->warn("Camera frame empty! Waiting...");
      std::this_thread::sleep_for(1ms); // 稍微等一下相机曝光
      continue; // 跳过这一帧，不往下执行
    }

    systemd_watchdog.ping();
    auto q = gimbal.q(t);

    double fps = 1./std::chrono::duration_cast<std::chrono::microseconds>(t - last_t).count()*1000000;
    // tools::draw_text(img, "fps: "+std::to_string(fps), cv::Point(40, 130));
    last_t = t;
    // tools::logger()->info("fps:: {:.2f}", fps);

    auto ypr = tools::eulers(q, 2, 1, 0);

    float yaw_deg = ypr[0] * 180.0 / M_PI;
    float pitch_deg = ypr[1] * 180.0 / M_PI;
    float roll_deg = ypr[2] * 180.0 / M_PI;
        
    // std::cout << "DK_Yaw: " << yaw_deg << std::endl;
    // std::cout << "DK_Pitch: " << pitch_deg << std::endl;
    // if(yaw_deg == 0 || pitch_deg ==0)std::cout<<"shit"<<std::endl;
    // tools::draw_text(img, fmt::format("rb_Yaw {:.2f}", yaw_deg), {40, 40}, {0, 128, 255});
    // tools::draw_text(img, fmt::format("rb_Pitch {:.2f}", pitch_deg), {40, 80}, {0, 255, 255});
    // std::cout << "Roll: " << roll_deg << std::endl;

    if (mode.load() == io::GimbalMode::SMALL_BUFF || mode.load() == io::GimbalMode::BIG_BUFF) {
      auto gs = gimbal.state();
      const auto buff_mode = mode.load() == io::GimbalMode::BIG_BUFF
                               ? auto_buff::BuffMode::BIG
                               : auto_buff::BuffMode::SMALL;
      const auto command = rune_system.process(
        img, t, q, buff_mode, static_cast<auto_buff::EnemyColor>(gs.enemy_color),
        gs.bullet_speed);
      gimbal.send(
        command.found, command.found && command.fire, command.yaw, 0.0f, 0.0f,
        command.pitch, 0.0f, 0.0f);
      
      auto fired = gs.bullet_count > last_bullet_count_main;
      last_bullet_count_main = gs.bullet_count;

      const auto & debug = rune_system.debug_snapshot();
      auto_buff::draw_rune_debug(img, debug);
      if (command.found) {
        nlohmann::json data;
        data["t"] = tools::delta_time(std::chrono::steady_clock::now(), t0_main);
        data["gimbal_yaw"] = gs.yaw;
        data["gimbal_pitch"] = gs.pitch;
        data["plan_mode"] = command.fire ? 2 : 1;
        data["plan_yaw"] = command.yaw / CV_PI * 180.0;
        data["plan_pitch"] = command.pitch * 180.0 / CV_PI;
        data["fire"] = command.fire ? 1 : 0;
        data["fired"] = fired ? 1 : 0;
        data["rune_detection_ms"] = debug.detection_ms;
        data["rune_core_ms"] = debug.core_ms;
        if (debug.phase) data["rune_phase"] = *debug.phase;
        if (debug.angular_velocity) data["rune_angular_velocity"] = *debug.angular_velocity;
        if (debug.big_rune_model_ready) {
          data["rune_fit_A"] = debug.big_rune_parameters[0];
          data["rune_fit_B"] = debug.big_rune_parameters[1];
          data["rune_fit_b"] = debug.big_rune_parameters[2];
          data["rune_fit_C"] = debug.big_rune_parameters[3];
          data["rune_fit_omega"] = debug.big_rune_parameters[4];
        }
        plotter.plot(data);
      }
    }

    // 【修改】除打符模式外，全认为是自瞄
    else {
      // 原版自瞄逻辑
      solver.set_R_gimbal2world(q);
      auto armors = yolo.detect(img);
      auto targets = tracker.track(armors, t);
      // recor.record(img, q, t);

      if (!targets.empty()) {
        target_queue.push(targets.front());
        tools::draw_reprojection(
          img, solver, targets.front(), planner.debug_xyza, cv::Scalar(235, 206, 135));
      } else {
        target_queue.push(std::nullopt);
      }

    } 
    cv::resize(img, img, {}, 0.5, 0.5);  // 显示时缩小图片尺寸
    cv::imshow("reprojection", img);
    auto key = cv::waitKey(1);
    if (key == 'q') break;
    if(key == 'r') {//TUDO :右键手动更改
      io::GimbalState* g_demo = gimbal.set_state_();
      g_demo->mode = !g_demo->mode;
    }
    // if(key == 's') {
    //   stopkey = !stopkey;
    // }
  }
  quit = true;
  if (plan_thread.joinable()) plan_thread.join();
  
  // 获取当前下位机发来的云台状态数据
  auto current_state = gimbal.state();
  
  // 发送当前数据（注意：由于 gimbal.cpp 中接收时乘了 57.3 转成了角度，发回下位机时需要除以 57.3 转回弧度）
  // 因为下位机没有发来速度和加速度数据，所以 vel 和 acc 继续填 0 即可
  gimbal.send(
      false, 
      false, 
      current_state.yaw / 57.3f, 
      0.0f, 
      0.0f, 
      current_state.pitch / 57.3f, 
      0.0f, 
      0.0f
  );

  return 0;
}
