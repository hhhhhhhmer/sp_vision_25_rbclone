#include <chrono>
#include <opencv2/opencv.hpp>
#include <thread>


#include "io/ros2/publish2nav.hpp"
#include "io/ros2/ros2.hpp"
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
#include "tools/yaml.hpp"
// #include "method_set/binocular_aim.hpp" // 双目自瞄

const std::string keys =
  "{help h usage ? |                                             | 输出命令行参数说明}"
  "{aim_camera   | ../configs/sb_short.yaml                          | 自瞄相机配置文件路径 }"
  "{l_cam          | ../configs/omn_camera_left.yaml | 左感知相机 }"
  "{r_cam          | ../configs/omn_camera_right.yaml  | 右感知相机 }";

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
  auto short_camera_config_path = cli.get<std::string>("aim_camera");

  // ROS2 通信
  io::ROS2 ros2;
  
  // 主相机（工业相机）
  io::Camera short_camera(short_camera_config_path);
  // io::Camera long_camera(long_camera_config_path);
  
  
  // 全向感知相机（工业相机）
  std::string omnl_yaml_name = cli.get<std::string>("l_cam");
  std::string omnr_yaml_name = cli.get<std::string>("r_cam");
  io::Camera omn_cam1(omnl_yaml_name);
  io::Camera omn_cam2(omnr_yaml_name);
  auto omn_l_yaml = tools::load(omnl_yaml_name);
  auto omn_r_yaml = tools::load(omnr_yaml_name);
  omn_cam1.main_and_secondary = tools::read<std::string>(omn_l_yaml, "main_and_secondary");
  omn_cam2.main_and_secondary = tools::read<std::string>(omn_r_yaml, "main_and_secondary");
  // io::Camera back_camera("configs/camera.yaml");
  
  // 改为使用Gimbal串口通信（替代CBoard）
  io::Gimbal gimbal(short_camera_config_path);
  
  // 视觉模块
  auto_aim::YOLO yolo(short_camera_config_path, false);  // 主相机YOLO
  auto_aim::Solver short_camera_solver(short_camera_config_path);
  auto_aim::Tracker tracker(short_camera_config_path, &short_camera_solver);//默认短焦
  tracker.set_gimbal(&gimbal);
  
  // MPC 规划器
  auto_aim::Planner short_camera_planner(short_camera_config_path);

  
  // 全向感知决策器
  auto_aim::Solver left_solver(omnl_yaml_name);
  auto_aim::Solver  right_solver(omnr_yaml_name);
  omniperception::Decider decider(omnl_yaml_name);
  decider.set_gimbal(&gimbal);
  
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
    auto_aim::Plan current_plan{false};
    auto t0 = std::chrono::steady_clock::now();
    uint16_t last_bullet_count = 0;
    
    while (!quit) {
        auto target = target_queue.front();
        auto gs = gimbal.state();
        
        // // 使用MPC规划器计算控制指令
        // plan = bincameras.planners.aim_ptr->plan(*target, 22);
          
        //MPC预测以及+自家火控
        // auto_aim::Planner * plan_short_or_long = target->cam_is_short ? &bincameras.planners.short_aim : &bincameras.planners.long_aim;
        auto_aim::Planner * plan_short_or_long = &short_camera_planner;
        auto plan =  plan_short_or_long->plan(target, gs.bullet_speed, gs.yaw,  auto_aim::Planner::ShootStrategy::rbSuppressiveFire);
        // 1. 设置默认值
        uint8_t name = 0;
        float tx = 0.0f;
        float ty = 0.0f;

        // 2. 只有在 target 有值时才去提取数据
        if (target.has_value()) {
          name = static_cast<uint8_t>(target->name) + 1;
          tx = target->ekf_x()[0]; 
          ty = target->ekf_x()[2]; 

          // tools::logger()->info("{},{},{}", name,tx,ty);

        }

        gimbal.sb_send(
        plan.control, plan.fire,
        plan.yaw, plan.yaw_vel, plan.yaw_acc,
        plan.pitch, plan.pitch_vel, plan.pitch_acc,
        tx,ty,name);

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
  std::chrono::steady_clock::time_point last, last_lost_point = std::chrono::steady_clock::now();
  // 主循环



  if (!systemd_watchdog.ready("Vision pipeline is ready")) {
    tools::logger()->warn("无法向 systemd 发送 READY 通知");
  }

  while (!exiter.exit()) {
    // 读取云台模式
    auto mode = gimbal.mode();
    auto now = std::chrono::steady_clock::now();
    auto dt = tools::delta_time(now, last);
    tools::logger()->info("{:.2f} fps", 1 / dt);
    last = now;

    // 读取主相机图像
    // bincameras.cameras.aim_ptr->read(img, timestamp);
    short_camera.read(img, timestamp);
    if (!img.empty()) systemd_watchdog.ping();
    
    // 获取云台姿态（四元数）
    Eigen::Quaterniond q = gimbal.q(timestamp - 3ms);
    


    // 更新解算器姿态
    short_camera_solver.set_R_gimbal2world(q);
    
    // 获取云台欧拉角
    gimbal_euler = tools::eulers(short_camera_solver.R_gimbal2world(), 2, 1, 0);

    float yaw_deg = gimbal_euler[0] * 180.0 / M_PI;
    float pitch_deg = gimbal_euler[1] * 180.0 / M_PI;
    float roll_deg = gimbal_euler[2] * 180.0 / M_PI;

    // 主相机检测
    auto armors = yolo.detect(img);

    // std::cout << "DK_Yaw: " << yaw_deg << std::endl;
    // std::cout << "DK_Pitch: " << pitch_deg << std::endl;
    if(yaw_deg == 0 || pitch_deg ==0)std::cout<<"shit"<<std::endl;
     tools::draw_text(img, fmt::format("rb_Yaw {:.2f}", yaw_deg), {40, 40}, {0, 128, 255});
      tools::draw_text(img, fmt::format("rb_Pitch {:.2f}", pitch_deg), {40, 80}, {0, 255, 255});
    // std::cout << "Roll: " << roll_deg << std::endl;
    
    // // 更新无敌状态装甲板
    // decider.get_invincible_armor(ros2.subscribe_enemy_status());
    
    // // 过滤装甲板
    // decider.armor_filter(armors);
    
    // 设置优先级
    decider.set_priority(armors);
    
    // 跟踪目标
    auto targets = tracker.track(armors, timestamp);

    if (tracker.state() == "lost") {
        // 只有需要时才执行重负载的 YOLO 和 决策
        io::VisionToGimbal vision_cmd = decider.decide_g(
            yolo, gimbal_euler, omn_cam1, omn_cam2, left_solver, right_solver);
          
        static size_t omn_detect_num = 0;
        if(vision_cmd.mode == 3) omn_detect_num++;
        // 只有在 lost 状态下才发送全向指令，避免干扰主线程控制
        if (tracker.state() == "lost" 
          // && omn_detect_num == 3
        ) {
          omn_detect_num = 0;
          io::sb_VisionToGimbal sb_cmd;
          sb_cmd.mode = vision_cmd.mode;         // 这里包含 mode = 3 的全向感知指令
          sb_cmd.yaw = vision_cmd.yaw;
          sb_cmd.yaw_vel = vision_cmd.yaw_vel;
          sb_cmd.yaw_acc = vision_cmd.yaw_acc;
          sb_cmd.pitch = vision_cmd.pitch;
          sb_cmd.pitch_vel = vision_cmd.pitch_vel;
          sb_cmd.pitch_acc = vision_cmd.pitch_acc;

          // 全向感知暂时不输出特定目标的坐标和名称，赋 0 即可
          sb_cmd.target_x = 0.0f;
          sb_cmd.target_y = 0.0f;
          sb_cmd.target_name = 0;

          gimbal.sb_send(sb_cmd);
        }
    } 

    
    // // 模式判断：如果跟踪器丢失目标，切换到全向感知模式
    // if (tracker.state() == "lost") {
    //   // 发现目标丢失，通知线程开启相机
    //   need_omni_perception = true;

    //   // 自动切换至短焦（用于全向感知后的接力）
    //   if(!bincameras.is_short){
    //       tools::logger()->info("进入全向感知模式，切换至短焦镜头");
    //       bincameras.Switch(tracker);
    //   }

    //   // 全向感知决策（非阻塞判断）
    //   if(!omn_cam1.is_paused() && !omn_cam2.is_paused()){
    //       io::VisionToGimbal vision_cmd = decider.decide_g(
    //           yolo, gimbal_euler, omn_cam1, omn_cam2, left_solver, right_solver);
    //       gimbal.send(vision_cmd);
    //   }
      
    // } else {
    //     // 挂起全向相机
    //     need_omni_perception = false;
    //     // omn_cam1.pause();
    //     // omn_cam2.pause();      
    // }

    {
      // 自瞄模式 - 使用MPC
      if (!targets.empty()) {
        tools::draw_reprojection(
          img, short_camera_solver, targets.front(), short_camera_planner.debug_xyza);
        // 将目标放入队列供MPC线程处理
        target_queue.push(targets.front());
        //自动切换相机
        // bincameras.ChangeTheScope(targets.front(), tracker);
      } else {
        target_queue.push(std::nullopt);
      }
    }


    cv::resize(img, img, {}, 0.5, 0.5);  // 显示时缩小图片尺寸
    cv::imshow("reprojection", img);
    auto key = cv::waitKey(1);
    if (key == 'q') break;




    
    // ROS2通信 - 发布目标信息
    Eigen::Vector4d target_info = decider.get_target_info(armors, targets);
    ros2.publish(target_info);
  }
  
  // 清理
  quit = true;
  if (mpc_thread.joinable()) {
    mpc_thread.join();
  }
  // if (resume_omncamera_thread.joinable()) {
  //   resume_omncamera_thread.join();
  // }
  // 发送停止指令
  gimbal.send(false, false, 0, 0, 0, 0, 0, 0);
  
  return 0;
}
