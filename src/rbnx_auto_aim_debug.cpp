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
#include "tools/img_tools.hpp"
#include "tools/reprojection.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/plotter.hpp"
#include "tools/thread_safe_queue.hpp"
#include "tools/recorder.hpp"

using namespace std::chrono_literals;
using namespace tools;


const std::string keys =
  "{help h usage ? |                        | 输出命令行参数说明}"
  "{@config-path   | ../configs/drone.yaml | 位置参数，yaml配置文件路径 }";

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

  auto_aim::YOLO yolo(config_path, true);
  auto_aim::Solver solver(config_path);
  auto_aim::Tracker tracker(config_path, &solver);
  tracker.set_gimbal(&gimbal);
  auto_aim::Aimer aimer(config_path);
  auto_aim::Shooter shooter(config_path);
  auto_aim::Planner planner(config_path);
  // tools::Recorder recor(90);
  bool stopkey = false;

  tools::ThreadSafeQueue<std::optional<auto_aim::Target>, true> target_queue(1);
  target_queue.push(std::nullopt);

  std::atomic<bool> quit = false;
  auto plan_thread = std::thread([&]() {
    auto t0 = std::chrono::steady_clock::now();
    uint16_t last_bullet_count = 0;

    while (!quit) {
      auto target = target_queue.front(); 
      auto gs = gimbal.state();



      //MPC预测以及+自家火控
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
      data["q2yaw"] = gs.q2yaw;
      data["q2pitch"] = gs.q2pitch;

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
    }
  });

  cv::Mat img;
  std::chrono::steady_clock::time_point t;
  std::chrono::steady_clock::time_point last_t;

  if (!systemd_watchdog.ready("Vision pipeline is ready")) {
    tools::logger()->warn("无法向 systemd 发送 READY 通知");
  }

  while (!exiter.exit()) {
    camera.read(img, t);
    if (!img.empty()) systemd_watchdog.ping();
    auto q = gimbal.q(t -3ms);


    double fps = 1./std::chrono::duration_cast<std::chrono::microseconds>(t - last_t).count()*1000000;
    // tools::draw_text(img, "fps: "+std::to_string(fps), cv::Point(40, 130));
    last_t = t;
    tools::logger()->info("fps:: {:.2f}", fps);

    auto ypr = tools::eulers(q, 2, 1, 0);

    float yaw_deg = ypr[0] * 180.0 / M_PI;
    float pitch_deg = ypr[1] * 180.0 / M_PI;
    float roll_deg = ypr[2] * 180.0 / M_PI;
        
    // std::cout << "DK_Yaw: " << yaw_deg << std::endl;
    // std::cout << "DK_Pitch: " << pitch_deg << std::endl;
  
    // std::cout << "Roll: " << roll_deg << std::endl;

    solver.set_R_gimbal2world(q);
    auto armors = yolo.detect(img);
    auto targets = tracker.track(armors, t);
    // recor.record(img, q, t);
  if(yaw_deg == 0 || pitch_deg ==0)std::cout<<"shit"<<std::endl;
     tools::draw_text(img, fmt::format("rb_Yaw {:.2f}", yaw_deg), {40, 40}, {0, 128, 255});
      tools::draw_text(img, fmt::format("rb_Pitch {:.2f}", pitch_deg), {40, 80}, {0, 255, 255});

    if (!targets.empty()) {
      target_queue.push(targets.front());
      tools::draw_reprojection(
        img, solver, targets.front(), planner.debug_xyza, cv::Scalar(235, 206, 135));
    } else {
      target_queue.push(std::nullopt);
    }

    cv::resize(img, img, {}, 0.5, 0.5);  // 显示时缩小图片尺寸
    cv::imshow("reprojection", img);
    auto key = cv::waitKey(1);
    if (key == 'q') break;
    if(key == 'r') {//TUDO :右键手动更改
      io::GimbalState* g_demo = gimbal.set_state_();
      g_demo->mode = !g_demo->mode;
    }
    if(key == 's') {
      stopkey = !stopkey;
    }


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
