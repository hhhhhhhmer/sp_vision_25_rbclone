#include <algorithm>
#include <chrono>
#include <fstream>
#include <string>

#include <Eigen/Geometry>
#include <opencv2/opencv.hpp>

#include "tasks/auto_buff/rune_debug_draw.hpp"
#include "tasks/auto_buff/rune_system.hpp"
#include "tools/logger.hpp"

namespace
{
const std::string keys =
  "{help h usage ? | | show help }"
  "{config-path c | ../configs/xiaohei.yaml | robot yaml }"
  "{mode m | small | small or big }"
  "{color | blue | red or blue target }"
  "{headless | false | disable GUI }"
  "{start-frame | 0 | skip the first N frames }"
  "{max-frames | 0 | stop after N frames; zero means all }"
  "{dump-commands | | optional CSV path for per-frame yaw/pitch }"
  "{@video | /home/cyn/Desktop/sp_vision_25_rbclone/yolo_buff/123/蓝.mp4 | input video }";
}

int main(int argc, char ** argv)
{
  cv::CommandLineParser cli(argc, argv, keys);
  if (cli.has("help")) {
    cli.printMessage();
    return 0;
  }
  const std::string video_path = cli.get<std::string>(0);
  const std::string config_path = cli.get<std::string>("config-path");
  const auto mode = cli.get<std::string>("mode") == "big"
                      ? auto_buff::BuffMode::BIG
                      : auto_buff::BuffMode::SMALL;
  const auto color = cli.get<std::string>("color") == "blue"
                       ? auto_buff::EnemyColor::BLUE
                       : auto_buff::EnemyColor::RED;
  const bool headless = cli.get<bool>("headless");
  const int start_frame = std::max(0, cli.get<int>("start-frame"));
  const int max_frames = cli.get<int>("max-frames");
  const std::string dump_commands = cli.get<std::string>("dump-commands");

  cv::VideoCapture video(video_path);
  if (!video.isOpened()) {
    tools::logger()->error("[auto_buff_test] cannot open {}", video_path);
    return 1;
  }

  auto_buff::RuneSystem rune(config_path);
  const double fps = std::max(1.0, video.get(cv::CAP_PROP_FPS));
  const auto start = std::chrono::steady_clock::now();
  cv::Mat image;
  for (int i = 0; i < start_frame; ++i) {
    if (!video.read(image)) break;
  }
  int frame_index = start_frame;
  int processed_frames = 0;
  int detection_frames = 0;
  int found_frames = 0;
  int uv_frames = 0;
  int uv_reprojection_samples = 0;
  double uv_reprojection_sum = 0.0;
  int uv_rejected_blades = 0;
  int pnp_frames = 0;
  int keypoint_samples = 0;
  double keypoint_error_sum = 0.0;
  double core_ms_sum = 0.0;
  std::ofstream command_csv;
  if (!dump_commands.empty()) {
    command_csv.open(dump_commands);
    command_csv << "frame,found,fire,yaw,pitch,uv_active,uv_rmse\n";
  }
  while (video.read(image)) {
    if (max_frames > 0 && processed_frames >= max_frames) break;
    const auto timestamp = start + std::chrono::duration_cast<std::chrono::steady_clock::duration>(
      std::chrono::duration<double>(frame_index / fps));
    const auto command = rune.process(
      image, timestamp, Eigen::Quaterniond::Identity(), mode, color, 24.5f);
    const auto & debug = rune.debug_snapshot();
    detection_frames += debug.detections.empty() ? 0 : 1;
    found_frames += command.found ? 1 : 0;
    core_ms_sum += debug.core_ms;
    if (debug.pnp_used) ++pnp_frames;
    if (debug.keypoint_reprojection_error) {
      keypoint_error_sum += *debug.keypoint_reprojection_error;
      ++keypoint_samples;
    }
    if (command_csv.is_open()) {
      command_csv << frame_index << ',' << (command.found ? 1 : 0) << ','
                  << (command.fire ? 1 : 0) << ',' << command.yaw << ',' << command.pitch << ','
                  << (debug.uv_pose_active ? 1 : 0) << ','
                  << (debug.uv_reprojection_error ? *debug.uv_reprojection_error : 0.0)
                  << '\n';
    }
    if (debug.uv_pose_active) {
      ++uv_frames;
      uv_rejected_blades += debug.uv_rejected_blades;
      if (debug.uv_reprojection_error) {
        uv_reprojection_sum += *debug.uv_reprojection_error;
        ++uv_reprojection_samples;
      }
    }
    if (!headless) {
      auto_buff::draw_rune_debug(image, debug);
      cv::imshow("RB-26Rune replay", image);
      if (cv::waitKey(20) == 'q') break;
    }
    ++frame_index;
    ++processed_frames;
  }

  tools::logger()->info(
    "[auto_buff_test] frames={} detection_frames={} found_frames={} uv_frames={} "
    "uv_rmse_avg={:.3f}px uv_rejected_blades={} kp_rmse_avg={:.3f}px "
    "pnp_frames={} core_ms_avg={:.3f}",
    processed_frames, detection_frames, found_frames, uv_frames,
    uv_reprojection_samples > 0 ? uv_reprojection_sum / uv_reprojection_samples : 0.0,
    uv_rejected_blades,
    keypoint_samples > 0 ? keypoint_error_sum / keypoint_samples : 0.0,
    pnp_frames,
    processed_frames > 0 ? core_ms_sum / processed_frames : 0.0);
  return processed_frames > 0 && detection_frames > 0 ? 0 : 2;
}
