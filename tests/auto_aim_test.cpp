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
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/plotter.hpp"

namespace
{
/**
 * @brief 绘制 UV 观测可视化
 *
 * 画三样东西：
 *  1) 模型预测的全部装甲板轮廓（灰色=非当前装甲板，绿色=当前 last_id），可以直观看到
 *     滤波器对整车 4 块装甲板几何的估计；
 *  2) 每块检测到的装甲板：观测到的灯条端点（青色实心点）与模型预测端点（红色十字），
 *     两者之间用橙色连线表示残差——连线越短说明 UV 观测拟合越好；
 *  3) 每条灯条的 UVL 残差数值（角度/中心/长度）。
 *
 * 只有在 UV 模式下（target.ekf().uv_model().ready()）才会绘制。
 */
cv::Rect draw_uv_overlay(
  cv::Mat & img, const auto_aim::Target & target, const std::list<auto_aim::Armor> & armors,
  bool uv_mode, int fused, double nis)
{
  const auto & model = target.ekf().uv_model();
  const Eigen::VectorXd x = target.ekf_x();

  if (!uv_mode || !model.ready()) {
    tools::draw_text(
      img, fmt::format("obs: YPD(world)  fused:{}  nis:{:.1f}", fused, nis), {10, 30},
      {0, 128, 255});
    return {};
  }

  const int armor_num = static_cast<int>(target.armor_xyza_list().size());

  // 1) 整车预测轮廓
  for (int id = 0; id < armor_num; id++) {
    const auto corners = model.project_corners(x, id, target.armor_type);
    if (corners.size() != 4) continue;
    const bool current = (id == target.last_id);
    const cv::Scalar color = current ? cv::Scalar(0, 255, 0) : cv::Scalar(140, 140, 140);
    for (int i = 0; i < 4; i++) cv::line(img, corners[i], corners[(i + 1) % 4], color, current ? 2 : 1);
    cv::putText(img, fmt::format("id{}", id), corners[0], cv::FONT_HERSHEY_SIMPLEX, 0.6, color, 2);
  }

  // 2) 观测 vs 预测
  int shown = 0;
  cv::Rect focus{};
  for (const auto & armor : armors) {
    if (armor.name != target.name || armor.type != target.armor_type) continue;
    if (armor.points.size() != 4) continue;

    int best_id = -1;
    double best_err = 1e18;
    std::vector<cv::Point2f> best_corners;
    for (int id = 0; id < armor_num; id++) {
      const auto c = model.project_corners(x, id, armor.type);
      if (c.size() != 4) continue;
      double err = 0;
      for (int i = 0; i < 4; i++) err += cv::norm(c[i] - armor.points[i]);
      if (err < best_err) {
        best_err = err;
        best_id = id;
        best_corners = c;
      }
    }
    if (best_id < 0) continue;

    if (shown == 0) {
      float min_x = armor.points[0].x, max_x = armor.points[0].x;
      float min_y = armor.points[0].y, max_y = armor.points[0].y;
      for (const auto & p : armor.points) {
        min_x = std::min(min_x, p.x);
        max_x = std::max(max_x, p.x);
        min_y = std::min(min_y, p.y);
        max_y = std::max(max_y, p.y);
      }
      focus = cv::Rect(
        cv::Point(static_cast<int>(min_x), static_cast<int>(min_y)),
        cv::Point(static_cast<int>(max_x), static_cast<int>(max_y)));
    }

    for (int i = 0; i < 4; i++) {
      cv::line(img, armor.points[i], best_corners[i], cv::Scalar(0, 165, 255), 1);
      cv::circle(img, armor.points[i], 4, cv::Scalar(255, 255, 0), -1);
      cv::drawMarker(
        img, best_corners[i], cv::Scalar(0, 0, 255), cv::MARKER_TILTED_CROSS, 14, 2);
    }

    // UVL 残差（每条灯条 4 维）
    const auto z_obs = auto_aim::UvModel::observation(armor.points);
    const auto z_pred = model.predict(x, best_id, armor.type);
    const double ang_l = tools::limit_rad(z_obs[0] - z_pred[0]) * 57.3;
    const double ang_r = tools::limit_rad(z_obs[4] - z_pred[4]) * 57.3;

    int base_y = 150 + shown * 60;
    tools::draw_text(
      img,
      fmt::format(
        "match id{}  corner_err {:.1f}px", best_id, best_err / 4.0),
      {10, base_y}, {0, 255, 255});
    tools::draw_text(
      img,
      fmt::format(
        "L: ang{:+.2f}deg  c({:+.1f},{:+.1f})  len{:+.2f}px", ang_l, z_obs[1] - z_pred[1],
        z_obs[2] - z_pred[2], z_obs[3] - z_pred[3]),
      {10, base_y + 26}, {255, 255, 0});
    tools::draw_text(
      img,
      fmt::format(
        "R: ang{:+.2f}deg  c({:+.1f},{:+.1f})  len{:+.2f}px", ang_r, z_obs[5] - z_pred[5],
        z_obs[6] - z_pred[6], z_obs[7] - z_pred[7]),
      {10, base_y + 52}, {255, 255, 0});
    shown++;
    if (shown >= 2) break;
  }

  tools::draw_text(
    img, fmt::format("obs: UV(pixel)  fused:{}  nis:{:.1f}", fused, nis), {10, 30},
    {0, 255, 0});
  return focus;
}

/**
 * @brief 把装甲板附近区域放大贴到右上角，方便肉眼看灯条端点级别的拟合
 * @param img 已绘制完标注的图像 @param focus 需要放大的区域 @param size 放大窗口边长（原图像素）
 */
void draw_zoom_inset(cv::Mat & img, const cv::Rect & focus, int size = 520)
{
  if (focus.width <= 0 || focus.height <= 0) return;

  const int margin = std::max(focus.width, focus.height);
  cv::Rect roi(
    focus.x - margin / 2, focus.y - margin / 2, focus.width + margin, focus.height + margin);
  roi &= cv::Rect(0, 0, img.cols, img.rows);
  if (roi.width < 8 || roi.height < 8) return;

  cv::Mat crop = img(roi).clone();
  cv::resize(crop, crop, {size, size}, 0, 0, cv::INTER_NEAREST);

  const int margin_px = 12;
  if (img.cols < size + 2 * margin_px || img.rows < size + 2 * margin_px) return;
  const cv::Rect dst(img.cols - size - margin_px, margin_px, size, size);
  crop.copyTo(img(dst));
  cv::rectangle(img, dst, {255, 255, 255}, 3);
  cv::putText(
    img, "zoom x" + std::to_string(size / std::max(roi.width, roi.height)), {dst.x + 8, dst.y + 30},
    cv::FONT_HERSHEY_SIMPLEX, 0.9, {255, 255, 255}, 2);
}
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
      auto uv_focus =
        draw_uv_overlay(img, target, armors, uv_mode, tracker.last_update_count(), target.ekf().last_nis);
      if (uv_mode) draw_zoom_inset(img, uv_focus);

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
