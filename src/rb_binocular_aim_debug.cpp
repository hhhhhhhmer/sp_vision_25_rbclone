#include <chrono>
#include <opencv2/opencv.hpp>
#include <thread>


#include "io/gimbal/gimbal.hpp"  
// #include "io/usbcamera/usbcamera.hpp"
#include "tasks/auto_aim/yolo.hpp"
#include "tasks/omniperception/decider.hpp"
#include "tools/exiter.hpp"
#include "tools/systemd_watchdog.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/plotter.hpp"
#include "tools/recorder.hpp"
#include "tools/yaml.hpp"
#include "tools/img_tools.hpp"
#include "tools/reprojection.hpp"

#include "method_set/binocular_aim.hpp"

const std::string keys =
  "{help h usage ? |                        | 输出命令行参数说明}"
  "{short_camera   | ../configs/sb_short.yaml | 短焦相机配置文件路径 }"
  "{long_camera    | ../configs/sb_long.yaml  | 长焦相机配置文件路径 }";

using namespace std::chrono_literals;




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
  auto short_camera_config_path = cli.get<std::string>("short_camera");
  auto long_camera_config_path = cli.get<std::string>("long_camera");


  // 主相机（工业相机）
  io::Camera short_camera(short_camera_config_path);
  io::Camera long_camera(long_camera_config_path);  
  
  // 串口通信
  io::Gimbal gimbal(short_camera_config_path);
  
  // 视觉模块
  auto_aim::YOLO yolo(short_camera_config_path, false);  // 主相机YOLO

  auto_aim::Solver short_camera_solver(short_camera_config_path);
  auto_aim::Solver long_camera_solver(long_camera_config_path);

  auto_aim::Tracker tracker(short_camera_config_path, &short_camera_solver);//默认短焦
  tracker.set_gimbal(&gimbal);
  
  // MPC 规划器
  auto_aim::Planner short_camera_planner(short_camera_config_path);
  auto_aim::Planner long_camera_planner(long_camera_config_path);

  //双目切换
  BinocularAim bincameras(short_camera, long_camera, 
                          short_camera_solver, long_camera_solver, 
                          short_camera_planner, long_camera_planner );
  
  
  // 线程安全队列（用于MPC规划线程）
  tools::ThreadSafeQueue<std::optional<auto_aim::Target>, true> target_queue(1);
  target_queue.push(std::nullopt);
  
  cv::Mat img;
  std::chrono::steady_clock::time_point timestamp;
  
  // 云台状态
  Eigen::Vector3d gimbal_euler;
  
  // 获取云台模式
  auto last_mode = io::GimbalMode::IDLE;
  
  // MPC 规划线程（独立线程运行MPC控制器）
  std::atomic<bool> quit = false;
  auto mpc_thread = std::thread([&]() {
    auto_aim::Plan plan{false};
    auto t0 = std::chrono::steady_clock::now();
    uint16_t last_bullet_count = 0;

    while (!quit) {
        auto target = target_queue.front();
        auto gs = gimbal.state();
        
        // // 使用MPC规划器计算控制指令
        // plan = bincameras.planners.aim_ptr->plan(*target, 22);
          
        //MPC预测以及+自家火控
        auto_aim::Planner * plan_short_or_long = target->cam_is_short ? &bincameras.planners.short_aim : &bincameras.planners.long_aim;
        // auto_aim::Planner * plan_short_or_long = &short_camera_planner;
        auto plan =  plan_short_or_long->plan(target, gs.bullet_speed, gs.yaw,  auto_aim::Planner::ShootStrategy::rbSuppressiveFire);
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
        data["plan_yaw"] = plan.yaw * 57.3;
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
          data["ekf_vyaw"] = ekf_satic(7);
          data["ekf_r"] = ekf_satic(8);        
        }



        plotter.plot(data);

      
        std::this_thread::sleep_for(10ms);
    }
  });

  // 主循环
  if (!systemd_watchdog.ready("Vision pipeline is ready")) {
    tools::logger()->warn("无法向 systemd 发送 READY 通知");
  }

  while (!exiter.exit()) {

    // 读取主相机图像
    bincameras.cameras.aim_ptr->read(img, timestamp);
    if (!img.empty()) systemd_watchdog.ping();
    // short_camera.read(img, timestamp);

    // auto q = gimbal.q(timestamp - bincameras.cameras.aim_ptr->timestamp_offset);
    auto q = gimbal.q(timestamp - 3ms);


    // tools::logger()->info("当前使用 {} 焦镜头", bincameras.is_short ? "短" : "长");
    
    
    // 获取云台欧拉角
    gimbal_euler = tools::eulers(q, 2, 1, 0);

    float yaw_deg = gimbal_euler[0] * 180.0 / M_PI;
    float pitch_deg = gimbal_euler[1] * 180.0 / M_PI;
    float roll_deg = gimbal_euler[2] * 180.0 / M_PI;


    bincameras.solvers.aim_ptr->set_R_gimbal2world(q);
    // short_camera_solver.set_R_gimbal2world(q);
    // 主相机检测
    auto armors = yolo.detect(img);


    // std::cout << "DK_Yaw: " << yaw_deg << std::endl;
    // std::cout << "DK_Pitch: " << pitch_deg << std::endl;
    if(yaw_deg == 0 || pitch_deg ==0)std::cout<<"shit"<<std::endl;
     tools::draw_text(img, fmt::format("rb_Yaw {:.2f}", yaw_deg), {40, 40}, {0, 128, 255});
      tools::draw_text(img, fmt::format("rb_Pitch {:.2f}", pitch_deg), {40, 80}, {0, 255, 255});
    // std::cout << "Roll: " << roll_deg << std::endl;
    
    // 跟踪目标
    
    auto targets = tracker.track(armors, timestamp);
     
    // 自瞄模式 - 使用MPC
    if (!targets.empty()) {
        // 将目标放入队列供MPC线程处理
        target_queue.push(targets.front());
        tools::draw_reprojection(
          img, *bincameras.solvers.aim_ptr, targets.front(), short_camera_planner.debug_xyza);
        // 自动长短焦切换
        bincameras.ChangeTheScope(targets.front(), tracker);
    } else {
        target_queue.push(std::nullopt);
    }
    
    cv::resize(img, img, {}, 0.5, 0.5);  // 显示时缩小图片尺寸
    cv::imshow("reprojection", img);
    auto key = cv::waitKey(1);
    if (key == 'q') break;
    if (key == 'c'){// 强制切换长短焦
        bincameras.Switch(tracker);
    }
  }
  
  // 清理
  quit = true;
  if (mpc_thread.joinable()) {
    mpc_thread.join();
  }
  
  
  return 0;
}
