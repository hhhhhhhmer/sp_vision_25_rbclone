#include <fmt/core.h>

#include <algorithm>
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
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/plotter.hpp"
#include "tools/thread_safe_queue.hpp"
#include "tools/recorder.hpp"

// 打符相关头文件
#include "tasks/auto_buff/buff_aimer.hpp"
#include "tasks/auto_buff/buff_detector.hpp"
#include "tasks/auto_buff/buff_solver.hpp"
#include "tasks/auto_buff/buff_target.hpp"
#include "tasks/auto_buff/buff_type.hpp"

using namespace std::chrono_literals;
using namespace tools;

namespace
{
void draw_buff_reprojection(
  cv::Mat & img, const std::vector<cv::Point2f> & points, const cv::Scalar & color)
{
  if (points.size() >= 4) {
    tools::draw_points(
      img, std::vector<cv::Point2f>(points.begin(), points.begin() + 4), color);
  }
  if (points.size() >= 8) {
    tools::draw_points(
      img, std::vector<cv::Point2f>(points.begin() + 4, points.begin() + 8), color);
  }
  if (points.size() >= 9) tools::draw_point(img, points[8], color, 3);
}
}  // namespace


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

    // 【修改】如果是打符模式
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

      if (mode.load() == io::GimbalMode::SMALL_BUFF) {
        buff_small_target.get_target(power_runes, t);
        active_target = &buff_small_target;
        auto target_copy = buff_small_target;
        buff_plan = buff_aimer.mpc_aim(target_copy, t, gs, true);
      } else if (mode.load() == io::GimbalMode::BIG_BUFF) {
        buff_big_target.get_target(power_runes, t);
        active_target = &buff_big_target;
        auto target_copy = buff_big_target;
        buff_plan = buff_aimer.mpc_aim(target_copy, t, gs, true);
      }
      
      // 直接发送打符相关的控制指令
      gimbal.send(
        buff_plan.control, buff_plan.fire, buff_plan.yaw, buff_plan.yaw_vel, buff_plan.yaw_acc,
        buff_plan.pitch, buff_plan.pitch_vel, buff_plan.pitch_acc);
      
      auto fired = gs.bullet_count > last_bullet_count_main;
      last_bullet_count_main = gs.bullet_count;

      // ========================== 新增：打符图像调试重投影 ==========================
      if (active_target && !active_target->is_unsolve()) {
        const auto primary = std::find_if(
          power_runes.begin(), power_runes.end(),
          [](const auto_buff::PowerRune & rune) { return rune.primary; });

        // 1. 显示当前主轨的识别特征点和中心
        if (primary != power_runes.end()) {
          for (const auto & point : primary->target().points) tools::draw_point(img, point);
          for (const auto & point : primary->target().fan_points) {
            tools::draw_point(img, point, {0, 128, 255});
          }
          if (!primary->target().points.empty()) {
            tools::draw_point(img, primary->target().center, {0, 0, 255}, 3);
          }
          if (!primary->target().fan_points.empty()) {
            tools::draw_point(img, primary->target().fan_center, {0, 128, 255}, 3);
          }
          tools::draw_point(img, primary->r_center, {0, 0, 255}, 3);
        }

        // 2. 当前帧target更新后buff位置 (绿色)
        auto Rxyz_in_world_now = active_target->point_buff2world(Eigen::Vector3d(0.0, 0.0, 0.0));
        auto image_points_now =
          buff_solver.reproject_buff(Rxyz_in_world_now, active_target->rotation_buff2world());
        draw_buff_reprojection(img, image_points_now, {0, 255, 0});

        // 3. Aimer弹道迭代最终使用的buff瞄准预测位置 (蓝色)
        if (const auto * predicted_target = buff_aimer.predicted_target()) {
          auto Rxyz_in_world_pre =
            predicted_target->point_buff2world(Eigen::Vector3d(0.0, 0.0, 0.0));
          auto image_points_pre =
            buff_solver.reproject_buff(Rxyz_in_world_pre, predicted_target->rotation_buff2world());
          draw_buff_reprojection(img, image_points_pre, {255, 0, 0});
        }
      }
      // =========================================================================

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

        // ========================== 新增：打符内部数据上传PlotJuggler ==========================
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
        // ==================================================================================

        plotter.plot(data);
      }
    }

    // 【修改】除打符模式外，全认为是自瞄
    else {
      // 原版自瞄逻辑
      auto yolo_frame =
        yolo.detect(auto_aim::YOLOFrameData(img, q, t), auto_aim_frame_count++);
      if (yolo_frame.is_empty || yolo_frame.timestamp < auto_aim_started_at) continue;

      img = yolo_frame.frame;
      q = yolo_frame.gimbal_q;
      t = yolo_frame.timestamp;

      solver.set_R_gimbal2world(q);
      auto targets = tracker.track(yolo_frame.armors, t);
      // recor.record(img, q, t);

      if (!targets.empty()){
        target_queue.push(targets.front());

        auto& target = targets.front();
      
        // 获取EKF状态向量
        Eigen::VectorXd ekf_x = target.getEKFXest();
        
        // 1. 计算旋转中心的世界坐标
        Eigen::Vector3d center_world(ekf_x[0], ekf_x[2], ekf_x[4]);
        
        // 2. 计算速度终点（预测0.5秒后的位置）
        double dt = 0.5; // 预测时间
        double scale_factor = 1.0; // 放大2倍
        Eigen::Vector3d velocity(ekf_x[1], ekf_x[3], ekf_x[5]);
        Eigen::Vector3d pred_center = center_world + velocity * dt * scale_factor ;
        
        // 3. 计算角速度方向终点
        double w = ekf_x[7]; // 角速度
        Eigen::Vector3d v_yaw_axis_tvec = center_world;
        v_yaw_axis_tvec[2] += w * 0.1; // 在y方向加上角速度的影响

        double speed_magnitude = std::sqrt(ekf_x[1]*ekf_x[1] + ekf_x[3]*ekf_x[3] + ekf_x[5]*ekf_x[5]);

        auto center_img = solver.reproject_armor(center_world, 0.0, target.armor_type, target.name);
        auto pred_point_img = solver.reproject_armor(pred_center, 0.0, target.armor_type, target.name);
        auto v_yaw_axis_point_img = solver.reproject_armor(v_yaw_axis_tvec, 0.0, target.armor_type, target.name);
        
        // 5. 绘制速度和角速度方向
        if (!center_img.empty() && !pred_point_img.empty()) {
          // 绘制旋转中心
          cv::circle(img, center_img[0], 5, cv::Scalar(51, 153, 237), -1);
          
          // 绘制预测点（速度方向）
          cv::circle(img, pred_point_img[0], 8, cv::Scalar(0, 0, 255), -1);
          
          // 绘制速度方向线
          cv::line(img, center_img[0], pred_point_img[0], cv::Scalar(0, 255, 255), 2);
          
          // 绘制角速度方向线
          if (!v_yaw_axis_point_img.empty()) {
            cv::line(img, center_img[0], v_yaw_axis_point_img[0], cv::Scalar(0, 255, 0), 2);
          }
        }
      }
      else {
        target_queue.push(std::nullopt);
      }

      if (!targets.empty()) {
        auto target = targets.front();

        // 当前帧target更新后
        std::vector<Eigen::Vector4d> armor_xyza_list = target.armor_xyza_list();
        for (const Eigen::Vector4d & xyza : armor_xyza_list) {
          auto image_points =
            solver.reproject_armor(xyza.head(3), xyza[3], target.armor_type, target.name);
          tools::draw_points(img, image_points, {235, 206, 135});
        }

        Eigen::Vector4d aim_xyza = planner.debug_xyza;
        auto image_points =
          solver.reproject_armor(aim_xyza.head(3), aim_xyza[3], target.armor_type, target.name);
        tools::draw_points(img, image_points, {0, 0, 255});
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
