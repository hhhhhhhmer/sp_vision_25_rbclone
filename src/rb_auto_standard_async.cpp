#include <fmt/core.h>

#include <atomic>
#include <chrono>
#include <nlohmann/json.hpp>
#include <opencv2/opencv.hpp>
#include <thread>

#include "io/camera.hpp"
#include "io/gimbal/gimbal.hpp"
#include "tasks/auto_aim/planner/planner.hpp"
#include "tasks/auto_aim/solver.hpp"
#include "tasks/auto_aim/tracker.hpp"
#include "tasks/auto_aim/aimer.hpp"
#include "tasks/auto_aim/shooter.hpp"
#include "tasks/auto_aim/yolo.hpp"
#include "tools/exiter.hpp"
#include "tools/systemd_watchdog.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/plotter.hpp"
#include "tools/thread_safe_queue.hpp"

// 打符相关头文件
#include "tasks/auto_buff/buff_aimer.hpp"
#include "tasks/auto_buff/buff_detector.hpp"
#include "tasks/auto_buff/buff_solver.hpp"
#include "tasks/auto_buff/buff_target.hpp"
#include "tasks/auto_buff/buff_type.hpp"

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

  // 自瞄相关对象初始化 (关闭YOLO debug模式以防弹窗)
  auto_aim::YOLO yolo(config_path, false);
  auto_aim::Solver solver(config_path);
  auto_aim::Tracker tracker(config_path, &solver);
  tracker.set_gimbal(&gimbal);
  auto_aim::Aimer aimer(config_path);
  auto_aim::Shooter shooter(config_path);
  auto_aim::Planner planner(config_path);

  // 打符相关对象初始化
  auto_buff::Buff_Detector buff_detector(config_path);
  auto_buff::Solver buff_solver(config_path);
  auto_buff::SmallTarget buff_small_target;
  auto_buff::BigTarget buff_big_target;
  auto_buff::Aimer buff_aimer(config_path);

  tools::ThreadSafeQueue<std::optional<auto_aim::Target>, true> target_queue(1);
  target_queue.push(std::nullopt);

  std::atomic<bool> quit = false;
  std::atomic<io::GimbalMode> mode{io::GimbalMode::IDLE}; // 全局云台模式

  auto plan_thread = std::thread([&]() {
    auto t0 = std::chrono::steady_clock::now();
    uint16_t last_bullet_count = 0;

    while (!quit) {
      // 非打符模式，全认为是自瞄
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
          data["tower_h1"] = target->tower_armor_hs[0].second;
          data["tower_h2"] = target->tower_armor_hs[1].second;
          data["tower_h3"] = target->tower_armor_hs[2].second;
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
  auto last_mode{io::GimbalMode::IDLE}; // 记录上次模式
  int auto_aim_frame_count = 0;
  auto auto_aim_started_at = std::chrono::steady_clock::time_point{};

  if (!systemd_watchdog.ready("Vision pipeline is ready")) {
    tools::logger()->warn("无法向 systemd 发送 READY 通知");
  }

  while (!exiter.exit()) {
    mode = gimbal.mode(); // 每帧获取最新云台模式
    
    if (last_mode != mode.load()) {
      tools::logger()->info("Switch to {}", gimbal.str(mode.load()));
      const auto was_buff =
        last_mode == io::GimbalMode::SMALL_BUFF || last_mode == io::GimbalMode::BIG_BUFF;
      const auto is_buff = mode.load() == io::GimbalMode::SMALL_BUFF ||
                           mode.load() == io::GimbalMode::BIG_BUFF;
      if (is_buff) target_queue.push(std::nullopt);
      if (was_buff && !is_buff) auto_aim_started_at = std::chrono::steady_clock::now();
      last_mode = mode.load();
    }

    camera.read(img, t);

    if (img.empty()) {
      tools::logger()->warn("Camera frame empty! Waiting...");
      std::this_thread::sleep_for(1ms); // 稍微等一下相机曝光
      continue; 
    }

    systemd_watchdog.ping();
    auto q = gimbal.q(t);

    // 如果是打符模式
   if (mode.load() == io::GimbalMode::SMALL_BUFF || mode.load() == io::GimbalMode::BIG_BUFF) {
      auto gs = gimbal.state();
      buff_solver.set_R_gimbal2world(q);

      const auto buff_mode = mode.load() == io::GimbalMode::BIG_BUFF
                               ? auto_buff::BuffMode::BIG
                               : auto_buff::BuffMode::SMALL;
      auto buff_observations = buff_detector.detect_tracks(img, buff_mode, t);
      auto power_runes = buff_solver.solve_all(buff_observations);

      auto_aim::Plan buff_plan;
      auto_buff::Target* active_target = nullptr;
      std::unique_ptr<auto_buff::Target> target_copy_ptr = nullptr;

      if (mode.load() == io::GimbalMode::SMALL_BUFF) {
        buff_small_target.get_target(power_runes, t);
        active_target = &buff_small_target;
        auto target_copy = buff_small_target;
        buff_plan = buff_aimer.mpc_aim(target_copy, t, gs, true);
        target_copy_ptr = std::make_unique<auto_buff::SmallTarget>(target_copy);
      } else if (mode.load() == io::GimbalMode::BIG_BUFF) {
        buff_big_target.get_target(power_runes, t);
        active_target = &buff_big_target;
        auto target_copy = buff_big_target;
        buff_plan = buff_aimer.mpc_aim(target_copy, t, gs, true);
        target_copy_ptr = std::make_unique<auto_buff::BigTarget>(target_copy);
      }
      
      // 直接发送打符相关的控制指令
      gimbal.send(
        buff_plan.control, buff_plan.fire, buff_plan.yaw, buff_plan.yaw_vel, buff_plan.yaw_acc,
        buff_plan.pitch, buff_plan.pitch_vel, buff_plan.pitch_acc);
      
      auto fired = gs.bullet_count > last_bullet_count_main;
      last_bullet_count_main = gs.bullet_count;

      if(buff_plan.control != 0)
      {
        nlohmann::json data;

        data["t"] = tools::delta_time(std::chrono::steady_clock::now(), t0_main);

        data["gimbal_yaw"] = gs.yaw;
        data["gimbal_yaw_vel"] = gs.yaw_vel;
        data["gimbal_pitch"] = gs.pitch;
        data["gimbal_pitch_vel"] = gs.pitch_vel;

        data["target_yaw"] = buff_plan.target_yaw;
        data["target_pitch"] = buff_plan.target_pitch;

        data["plan_mode"] = buff_plan.control ? (buff_plan.fire ? 2 : 1) : 0;
        data["plan_yaw"] = buff_plan.yaw / CV_PI * 180.;
        data["plan_yaw_vel"] = buff_plan.yaw_vel;
        data["plan_yaw_acc"] = buff_plan.yaw_acc;

        data["plan_pitch"] = buff_plan.pitch * 57.3;
        data["plan_pitch_vel"] = buff_plan.pitch_vel;
        data["plan_pitch_acc"] = buff_plan.pitch_acc;

        data["fire"] = buff_plan.fire ? 1 : 0;
        data["fired"] = fired ? 1 : 0;

        if (active_target && !active_target->is_unsolve()) {
          Eigen::VectorXd x = active_target->ekf_x();
          data["R_yaw"] = x[0];
          data["R_V_yaw"] = x[1];
          data["R_pitch"] = x[2];
          data["R_dis"] = x[3];
          data["yaw"] = x[4] * 57.3;

          data["angle"] = x[5] * 57.3;
          data["spd"] = x[6] * 57.3;
          if (x.size() >= 10) { // 大符状态量更多
            data["spd"] = x[6];
            data["a"] = x[7];
            data["w"] = x[8];
            data["fi"] = x[9];
            data["spd0"] = active_target->spd;
          }
        }

        plotter.plot(data);
      }
    }

    // 除打符模式外，全认为是自瞄
    else {
      auto yolo_frame =
        yolo.detect(auto_aim::YOLOFrameData(img, q, t), auto_aim_frame_count++);
      if (yolo_frame.is_empty || yolo_frame.timestamp < auto_aim_started_at) continue;

      img = yolo_frame.frame;
      q = yolo_frame.gimbal_q;
      t = yolo_frame.timestamp;

      solver.set_R_gimbal2world(q);
      auto targets = tracker.track(yolo_frame.armors, t);

      if (!targets.empty()){
        target_queue.push(targets.front());
      }
      else {
        target_queue.push(std::nullopt);
      }
    } 
  }
  
  quit = true;
  if (plan_thread.joinable()) plan_thread.join();
  
  // 获取当前下位机发来的云台状态数据
  auto current_state = gimbal.state();
  
  // 发送当前数据回到下位机归位（因为下位机没有发来速度和加速度数据，所以 vel 和 acc 填 0 即可）
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
