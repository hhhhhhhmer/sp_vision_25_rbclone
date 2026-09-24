#include <fmt/core.h>

#include <chrono>
#include <filesystem>
#include <fstream>
#include <list>
#include <nlohmann/json.hpp>
#include <opencv2/opencv.hpp>

#include "tasks/auto_aim/aiming/aimer.hpp"
#include "tasks/auto_aim/geometry/solver.hpp"
#include "tasks/auto_aim/tracking/tracker.hpp"
#include "tasks/auto_aim/tracking/uv_model.hpp"
#include "tasks/auto_aim/detection/yolo.hpp"
// #include "tasks/auto_aim/detection/detector.hpp"
#include "tools/exiter.hpp"
#include "tools/img_tools.hpp"
#include "tools/reprojection.hpp"
#include "tools/uv_overlay.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/plotter.hpp"

namespace
{
}  // namespace

const std::string keys =
  "{help h usage ? |                   | 输出命令行参数说明 }"
  "{config-path c  | ../configs/rb_auto_aim.yaml | yaml配置文件的路径}"
  "{start-index s  | 0                 | 视频起始帧下标    }"
  "{end-index e    | 0                 | 视频结束帧下标    }"
  "{@input-path    | ../快/2026-04-04_19-59-45  | avi和txt文件的路径}"
  // 注意：短选项名不能与 end-index 的 e 重复。OpenCV 4.5.4 会把重名的两个选项绑到同一个键上，
  // 传 -e=60 会同时改掉 end-index 和 save-every（静默开始写图）。
  "{save-every n    | 0                           | 每 N 帧保存一张带标注的图片，0 表示不保存}"
  "{save-dir d      | /tmp/auto_aim_frames        | 保存目录}";

int main(int argc, char * argv[])
{
  // 读取命令行参数
  cv::CommandLineParser cli(argc, argv, keys);
  if (cli.has("help")) {
    cli.printMessage();
    return 0;
  }
  auto input_path = cli.get<std::string>(0);
  auto config_path = cli.get<std::string>("config-path");
  auto start_index = cli.get<int>("start-index");
  auto end_index = cli.get<int>("end-index");
  const int save_every = cli.get<int>("save-every");
  const auto save_dir = cli.get<std::string>("save-dir");
  if (save_every > 0) {
    std::filesystem::create_directories(save_dir);
    tools::logger()->info("[test] saving annotated frames to {} every {} frames", save_dir, save_every);
  }

  tools::Plotter plotter;
  tools::Exiter exiter;

  auto video_path = fmt::format("{}.avi", input_path);
  auto text_path = fmt::format("{}.txt", input_path);
  cv::VideoCapture video(video_path);
  std::ifstream text(text_path);

  auto_aim::YOLO yolo(config_path);
  // auto_aim::Detector traditional(config_path, true);
  auto_aim::Solver solver(config_path);
  auto_aim::Tracker tracker(config_path, &solver);
  auto_aim::Aimer aimer(config_path);

  cv::Mat img, drawing;
  auto t0 = std::chrono::steady_clock::now();

  auto_aim::Target last_target;
  io::Command last_command;
  double last_t = -1;

  video.set(cv::CAP_PROP_POS_FRAMES, start_index);
  for (int i = 0; i < start_index; i++) {
    double t, w, x, y, z;
    text >> t >> w >> x >> y >> z;
  }

  for (int frame_count = start_index; !exiter.exit(); frame_count++) {
    if (end_index > 0 && frame_count > end_index) break;
    // auto inshow_start = std::chrono::steady_clock::now();
    video.read(img);
    if (img.empty()) break;

    double t, w, x, y, z;
    text >> t >> w >> x >> y >> z;
    auto timestamp = t0 + std::chrono::microseconds(int(t * 1e6));

    /// 自瞄核心逻辑

    solver.set_R_gimbal2world({w, x, y, z});

    auto yolo_start = std::chrono::steady_clock::now();
    auto armors = yolo.detect(img, frame_count);
    // auto traditional_start = std::chrono::steady_clock::now();
    // auto armors = traditional.detect(img, frame_count);

    auto tracker_start = std::chrono::steady_clock::now();
    auto targets = tracker.test_track(armors, timestamp);

    auto aimer_start = std::chrono::steady_clock::now();
    auto command = aimer.aim(targets, timestamp, 27, false);

    if (
      !targets.empty() && aimer.debug_aim_point.valid &&
      std::abs(command.yaw - last_command.yaw) * 57.3 < 2)
      command.shoot = true;

    if (command.control) last_command = command;
    /// 调试输出

    auto finish = std::chrono::steady_clock::now();
    tools::logger()->info(
      "[{}] yolo: {:.1f}ms, tracker: {:.1f}ms, aimer: {:.1f}ms", frame_count,
      tools::delta_time(tracker_start, yolo_start) * 1e3,
      tools::delta_time(aimer_start, tracker_start) * 1e3,
      tools::delta_time(finish, aimer_start) * 1e3);

    tools::draw_text(
      img,
      fmt::format(
        "command is {},{:.2f},{:.2f},shoot:{}", command.control, command.yaw * 57.3,
        command.pitch * 57.3, command.shoot),
      {10, 60}, {154, 50, 205});

    Eigen::Quaternion gimbal_q = {w, x, y, z};
    tools::draw_text(
      img,
      fmt::format(
        "gimbal yaw{:.2f}", (tools::eulers(gimbal_q.toRotationMatrix(), 2, 1, 0) * 57.3)[0]),
      {10, 90}, {255, 255, 255});

    nlohmann::json data;

    const bool uv_mode = tracker.uv_enabled();

    // 装甲板原始观测数据（UV 模式下不做逐帧 PnP，位姿字段为空，改用滤波器预测值）
    data["armor_num"] = armors.size();
    data["uv_mode"] = uv_mode ? 1 : 0;
    if (!armors.empty()) {
      const auto & armor = armors.front();
      data["armor_x"] = armor.xyz_in_world[0];
      data["armor_y"] = armor.xyz_in_world[1];
      data["armor_yaw"] = armor.ypr_in_world[0] * 57.3;
      data["armor_yaw_raw"] = armor.yaw_raw * 57.3;
      data["armor_center_x"] = armor.center_norm.x;
      data["armor_center_y"] = armor.center_norm.y;
    }

    Eigen::Quaternion q{w, x, y, z};
    auto yaw = tools::eulers(q, 2, 1, 0)[0];
    data["gimbal_yaw"] = yaw * 57.3;
    data["cmd_yaw"] = command.yaw * 57.3;
    data["shoot"] = command.shoot;

    if (!targets.empty()) {
      auto target = targets.front();

      if (last_t == -1) {
        last_target = target;
        last_t = t;
        continue;
      }

      auto aim_point = aimer.debug_aim_point;
      std::optional<Eigen::Vector4d> aim_xyza;
      if (aim_point.valid) aim_xyza = aim_point.xyza;
      // 传统世界系重投影（黄绿轮廓）；UV 模式下再叠加"观测端点 vs 预测端点"的可视化
      tools::draw_reprojection(img, solver, target, aim_xyza);
      auto uv_focus = tools::draw_uv_overlay(
        img, target, armors, uv_mode, tracker.last_update_count(), target.ekf().last_nis);
      if (uv_mode) tools::draw_zoom_inset(img, uv_focus);

      // 观测器内部数据
      Eigen::VectorXd x = target.ekf_x();
      data["x"] = x[0];
      data["vx"] = x[1];
      data["y"] = x[2];
      data["vy"] = x[3];
      data["z"] = x[4];
      data["vz"] = x[5];
      data["a"] = x[6] * 57.3;
      data["w"] = x[7];
      data["r"] = x[8];
      data["l"] = x[9];
      data["h"] = x[10];
      data["last_id"] = target.last_id;

      // 卡方检验数据
      data["residual_yaw"] = target.ekf().data.at("residual_yaw");
      data["residual_pitch"] = target.ekf().data.at("residual_pitch");
      data["residual_distance"] = target.ekf().data.at("residual_distance");
      data["residual_angle"] = target.ekf().data.at("residual_angle");
      data["nis"] = target.ekf().data.at("nis");
      data["nees"] = target.ekf().data.at("nees");
      data["nis_fail"] = target.ekf().data.at("nis_fail");
      data["nees_fail"] = target.ekf().data.at("nees_fail");
      data["recent_nis_failures"] = target.ekf().data.at("recent_nis_failures");
    }
    
    plotter.plot(data);

    // 顶部提示当前观测模式与按键
    tools::draw_text(
      img, fmt::format("[u] switch obs mode   current: {}", uv_mode ? "UV(pixel)" : "YPD(world)"),
      {10, img.rows - 20}, {255, 255, 255});

    if (save_every > 0 && frame_count % save_every == 0) {
      cv::imwrite(fmt::format("{}/{:06d}.png", save_dir, frame_count), img);
    }

    cv::resize(img, img, {}, 0.5, 0.5);  // 显示时缩小图片尺寸
    cv::imshow("reprojection", img);
    auto key = cv::waitKey(10);
    if (key == 'q') break;
    if (key == 'u') {
      // 运行时切换 UV / 传统观测：切换后会 reset，下一帧用 PnP 重新初始化
      tracker.set_uv_enabled(!tracker.uv_enabled());
      last_t = -1;  // 让下一帧重新走"首帧"逻辑
      tools::logger()->info("[test] obs mode -> {}", tracker.uv_enabled() ? "UV" : "YPD");
    }

    //  tools::logger()->info(
    //     "imshow : {:.1f}ms",  tools::delta_time(std::chrono::steady_clock::now(), inshow_start) * 1e3);
  }

  return 0;
}
