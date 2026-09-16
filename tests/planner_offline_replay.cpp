/**
 * @file planner_offline_replay.cpp
 * @brief 离线回放：真实检测 -> Tracker -> **Planner**，把"上位机会下发的命令"逐帧导出。
 *
 * 用途：用录制的 avi + 同名逐帧四元数 txt，纯离线重现上位机的命令流，
 *       用于判断"大甩头"的命令跳变是否由上位机产生（不依赖上车）。
 *
 * 用法：planner_offline_replay <视频(不含扩展名)> [配置路径] [输出jsonl]
 */
#include <cstdio>
#include <fstream>
#include <list>
#include <optional>
#include <string>

#include <nlohmann/json.hpp>
#include <opencv2/opencv.hpp>

#include "tasks/auto_aim/aiming/planner/planner.hpp"
#include "tasks/auto_aim/detection/yolo.hpp"
#include "tasks/auto_aim/geometry/solver.hpp"
#include "tasks/auto_aim/tracking/tracker.hpp"
#include "tools/math_tools.hpp"
#include "tools/logger.hpp"

int main(int argc, char ** argv)
{
  const std::string input = (argc > 1) ? argv[1] : "";
  const std::string config = (argc > 2) ? argv[2] : "../configs/rb_auto_aim.yaml";
  const std::string out_path = (argc > 3) ? argv[3] : "";
  if (input.empty()) {
    std::printf("usage: planner_offline_replay <video(no ext)> [config] [out.jsonl]\n");
    return 2;
  }

  cv::VideoCapture video(input + ".avi");
  if (!video.isOpened()) {
    std::printf("无法打开 %s.avi\n", input.c_str());
    return 1;
  }
  std::ifstream text(input + ".txt");
  if (!text) {
    std::printf("无法打开 %s.txt\n", input.c_str());
    return 1;
  }
  std::ofstream out;
  if (!out_path.empty()) out.open(out_path);
  std::printf("回放: %s  (配置 %s)\n", input.c_str(), config.c_str());

  auto_aim::YOLO detector(config);
  auto_aim::Solver solver(config);
  auto_aim::Tracker tracker(config, &solver);
  auto_aim::Planner planner(config);

  const auto t0 = std::chrono::steady_clock::now();
  auto_aim::Plan last{};
  bool has_last = false;
  double max_jump = 0, sum_jump = 0;
  int n = 0, big = 0;
  double worst_t = 0;
  std::list<std::pair<double, double>> events;

  cv::Mat img;
  for (int i = 0; !video.read(img), !img.empty(); ++i) {
    double t = 0, w = 0, x = 0, y = 0, z = 0;
    if (!(text >> t >> w >> x >> y >> z)) break;
    auto stamp = t0 + std::chrono::microseconds(static_cast<int64_t>(t * 1e6));

    Eigen::Quaterniond q(w, x, y, z);
    if (q.norm() < 1e-9) continue;
    solver.set_R_gimbal2world(q.normalized());
    const double gimbal_yaw_deg = tools::eulers(q.normalized(), 2, 1, 0)[0] * 57.2957795;

    auto armors = detector.detect(img, i);
    auto targets = tracker.test_track(armors, stamp);
    if (targets.empty()) continue;

    auto plan = planner.plan(
      std::optional<auto_aim::Target>(targets.front()), 22.0, gimbal_yaw_deg,
      auto_aim::Planner::ShootStrategy::rbSuppressiveFire);

    const double cmd_deg = plan.yaw * 57.2957795;
    double jump = 0;
    if (has_last) {
      jump = std::fabs(std::remainder(plan.yaw - last.yaw, 2 * M_PI)) * 57.2957795;
      sum_jump += jump; ++n;
      if (jump > max_jump) { max_jump = jump; worst_t = t; }
      if (jump > 30.0) { ++big; if (events.size() < 40) events.emplace_back(t, jump); }
    }
    last = plan; has_last = true;

    if (out.is_open()) {
      out << nlohmann::json{
               {"t", t},
               {"gimbal_yaw", gimbal_yaw_deg},
               {"plan_yaw", cmd_deg},
               {"plan_pitch", plan.pitch * 57.2957795},
               {"jump", jump},
               {"fire", plan.fire ? 1 : 0},
               {"armors", static_cast<int>(armors.size())}}
                 .dump()
          << '\n';
    }
  }

  std::printf("\n===== 汇总 (%d 帧有目标) =====\n", n);
  std::printf("命令单帧跳变: 中位 %.2f°, 最大 %.2f° @ t=%.2fs, >30° 共 %d 次\n",
              n ? sum_jump / n : 0.0, max_jump, worst_t, big);
  if (!events.empty()) {
    std::printf(">30° 的时刻(前 20 个): ");
    int c = 0;
    for (auto & e : events) { std::printf("t=%.2f(%.0f°) ", e.first, e.second); if (++c >= 20) break; }
    std::printf("\n");
  }
  if (out.is_open()) std::printf("逐帧记录已写入 %s\n", out_path.c_str());
  return 0;
}
