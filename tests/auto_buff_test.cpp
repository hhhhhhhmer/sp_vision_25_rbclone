#include <fmt/core.h>

#include <array>
#include <algorithm>
#include <chrono>
#include <fstream>
#include <memory>
#include <vector>
#include <nlohmann/json.hpp>
#include <opencv2/opencv.hpp>
#include <yaml-cpp/yaml.h>

#include "tasks/auto_buff/buff_aimer.hpp"
#include "tasks/auto_buff/buff_detector.hpp"
#include "tasks/auto_buff/buff_solver.hpp"
#include "tasks/auto_buff/buff_target.hpp"
#include "tasks/auto_buff/buff_type.hpp"
#include "tools/exiter.hpp"
#include "tools/img_tools.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/plotter.hpp"

namespace
{
using Clock = std::chrono::steady_clock;

struct FrameTiming
{
  double total_ms = 0.0;
  double read_ms = 0.0;
  double imu_ms = 0.0;
  double set_imu_ms = 0.0;
  double detect_ms = 0.0;
  double solve_ms = 0.0;
  double target_ms = 0.0;
  double aim_ms = 0.0;
  double debug_ms = 0.0;
  double plot_ms = 0.0;
  double show_wait_ms = 0.0;
};

double elapsed_ms(const Clock::time_point & start, const Clock::time_point & end)
{
  return std::chrono::duration<double, std::milli>(end - start).count();
}

double percentile_ms(std::vector<double> values, double percentile)
{
  if (values.empty()) return 0.0;
  std::sort(values.begin(), values.end());
  const size_t index = static_cast<size_t>(
    std::clamp(percentile, 0.0, 1.0) * static_cast<double>(values.size() - 1));
  return values[index];
}

size_t observation_index(auto_buff::BuffObservationType type)
{
  if (type == auto_buff::BuffObservationType::TARGET_ONLY) return 1;
  if (type == auto_buff::BuffObservationType::FAN_ONLY) return 2;
  return 0;
}

size_t pose_index(auto_buff::BuffPoseQuality quality)
{
  if (quality == auto_buff::BuffPoseQuality::PARTIAL_5_POINT) return 1;
  if (quality == auto_buff::BuffPoseQuality::PARTIAL_4_POINT) return 2;
  return 0;
}

std::string timing_terminal_text(const FrameTiming & timing)
{
  return fmt::format(
    "time total={:.2f}ms read={:.2f} imu={:.2f} setR={:.2f} detect={:.2f} solve={:.2f} "
    "target={:.2f} aim={:.2f} debug={:.2f} plot={:.2f} show_wait={:.2f}",
    timing.total_ms, timing.read_ms, timing.imu_ms, timing.set_imu_ms, timing.detect_ms,
    timing.solve_ms, timing.target_ms, timing.aim_ms, timing.debug_ms, timing.plot_ms,
    timing.show_wait_ms);
}

void add_timing_data(nlohmann::json & data, const FrameTiming & timing)
{
  data["time_read_ms"] = timing.read_ms;
  data["time_imu_ms"] = timing.imu_ms;
  data["time_setR_ms"] = timing.set_imu_ms;
  data["time_detect_ms"] = timing.detect_ms;
  data["time_solve_ms"] = timing.solve_ms;
  data["time_target_ms"] = timing.target_ms;
  data["time_aim_ms"] = timing.aim_ms;
  data["time_debug_ms"] = timing.debug_ms;
}

cv::Point2f mean_point(const std::vector<cv::Point2f> & points, size_t begin, size_t end)
{
  cv::Point2f sum(0.0f, 0.0f);
  for (size_t i = begin; i < end; ++i) sum += points[i];
  return sum * (1.0f / static_cast<float>(end - begin));
}

double mean_error(
  const std::vector<cv::Point2f> & reference, const std::vector<cv::Point2f> & candidate,
  size_t begin, size_t end)
{
  double sum = 0.0;
  for (size_t i = begin; i < end; ++i) sum += cv::norm(candidate[i] - reference[i]);
  return sum / static_cast<double>(end - begin);
}

double vector_angle_error_deg(
  const cv::Point2f & reference_from, const cv::Point2f & reference_to,
  const cv::Point2f & candidate_from, const cv::Point2f & candidate_to)
{
  const auto reference = reference_to - reference_from;
  const auto candidate = candidate_to - candidate_from;
  const double reference_angle = std::atan2(reference.y, reference.x);
  const double candidate_angle = std::atan2(candidate.y, candidate.x);
  return tools::limit_rad(candidate_angle - reference_angle) * 57.3;
}

struct PixelCompare
{
  cv::Point2f raw_target_center;
  cv::Point2f raw_fan_center;
  cv::Point2f target_center;
  cv::Point2f fan_center;
  double target_error = 0.0;
  double fan_error = 0.0;
  double mean_error_8 = 0.0;
  double angle_error_deg = 0.0;
};

std::optional<PixelCompare> compare_points(
  const std::vector<cv::Point2f> & raw, const std::vector<cv::Point2f> & candidate)
{
  if (raw.size() < 8 || candidate.size() < 8) return std::nullopt;

  PixelCompare result;
  result.raw_target_center = mean_point(raw, 0, 4);
  result.raw_fan_center = mean_point(raw, 4, 8);
  result.target_center = mean_point(candidate, 0, 4);
  result.fan_center = mean_point(candidate, 4, 8);
  result.target_error = cv::norm(result.target_center - result.raw_target_center);
  result.fan_error = cv::norm(result.fan_center - result.raw_fan_center);
  result.mean_error_8 = mean_error(raw, candidate, 0, 8);
  result.angle_error_deg = vector_angle_error_deg(
    result.raw_target_center, result.raw_fan_center, result.target_center, result.fan_center);
  return result;
}

void add_compare_data(
  nlohmann::json & data, const std::string & prefix, const PixelCompare & compare)
{
  data[prefix + "_target_dx"] = compare.target_center.x - compare.raw_target_center.x;
  data[prefix + "_target_dy"] = compare.target_center.y - compare.raw_target_center.y;
  data[prefix + "_fan_dx"] = compare.fan_center.x - compare.raw_fan_center.x;
  data[prefix + "_fan_dy"] = compare.fan_center.y - compare.raw_fan_center.y;
  data[prefix + "_target_err_px"] = compare.target_error;
  data[prefix + "_fan_err_px"] = compare.fan_error;
  data[prefix + "_mean8_err_px"] = compare.mean_error_8;
  data[prefix + "_angle_err_deg"] = compare.angle_error_deg;
}

std::string compare_text(const std::string & name, const PixelCompare & compare)
{
  return fmt::format(
    "{} T:{:.1f}px F:{:.1f}px mean:{:.1f}px ang:{:.1f}deg", name, compare.target_error,
    compare.fan_error, compare.mean_error_8, compare.angle_error_deg);
}

std::string compare_terminal_text(const std::string & name, const PixelCompare & compare)
{
  return fmt::format(
    "{}:T={:.1f},F={:.1f},M={:.1f},A={:.1f},Tdx={:.1f},Tdy={:.1f},Fdx={:.1f},Fdy={:.1f}",
    name, compare.target_error, compare.fan_error, compare.mean_error_8, compare.angle_error_deg,
    compare.target_center.x - compare.raw_target_center.x,
    compare.target_center.y - compare.raw_target_center.y,
    compare.fan_center.x - compare.raw_fan_center.x, compare.fan_center.y - compare.raw_fan_center.y);
}
}  // namespace

const std::string keys =
  "{help h usage ? |                        | 输出命令行参数说明 }"
  "{config-path c  | ../configs/xiaohei.yaml    | yaml配置文件的路径}"
  "{start-index s  | 0                      | 视频起始帧下标    }"
  "{end-index e    | 0                      | 视频结束帧下标    }"
  "{buff-mode m   | small                  | small 或 big      }"
  "{print-debug p | flase                   | 是否在终端输出误差 }"
  "{print-time    | true                  | 是否在终端输出各步骤耗时 }"
  "{headless      | false                  | 不显示窗口且不等待按键 }"
  "{print-step    | 1                      | 每隔多少帧输出一次 }"
  "{debug-predict-time | -1                | 蓝色调试框预测时间, 负数使用配置 }"
  "{@input-path    |    /home/rm/Desktop/sp_vision_25_rbclone/assets/buff/buff guanfang   | avi和txt文件的路径}";

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
  auto buff_mode = cli.get<std::string>("buff-mode");
  const auto detector_mode = buff_mode == "big" ? auto_buff::BuffMode::BIG
                                                : auto_buff::BuffMode::SMALL;
  auto print_debug = cli.get<bool>("print-debug");
  auto print_time = cli.get<bool>("print-time");
  auto headless = cli.get<bool>("headless");
  auto print_step = std::max(1, cli.get<int>("print-step"));
  auto debug_predict_time_cli = cli.get<double>("debug-predict-time");

  tools::Plotter plotter;
  tools::Exiter exiter;

  auto video_path = fmt::format("{}.avi", input_path);
  auto text_path = fmt::format("{}.txt", input_path);
  cv::VideoCapture video(video_path);
  std::ifstream text(text_path);
  bool use_imu_text = text.is_open();
  if (!video.isOpened()) {
    tools::logger()->error("[auto_buff_test] failed to open video: {}", video_path);
    return 1;
  }
  if (!use_imu_text) {
    tools::logger()->warn(
      "[auto_buff_test] imu text not found: {}, using identity gimbal quaternion", text_path);
  }
  const double fps = video.get(cv::CAP_PROP_FPS);
  const double frame_dt = fps > 1e-3 ? 1.0 / fps : 1.0 / 60.0;
  auto yaml = YAML::LoadFile(config_path);
  const double debug_predict_time = debug_predict_time_cli >= 0.0
                                      ? debug_predict_time_cli
                                      : 0.1 + (yaml["predict_time"]
                                                 ? yaml["predict_time"].as<double>()
                                                 : 0.0);

  auto_buff::Buff_Detector detector(config_path);
  auto_buff::Solver solver(config_path);
  std::unique_ptr<auto_buff::Target> target;
  if (buff_mode == "big") {
    target = std::make_unique<auto_buff::BigTarget>();
  } else {
    if (buff_mode != "small") {
      tools::logger()->warn("[auto_buff_test] unknown buff-mode: {}, using small", buff_mode);
    }
    target = std::make_unique<auto_buff::SmallTarget>();
  }
  auto_buff::Aimer aimer(config_path);

  cv::Mat img, drawing;
  auto t0 = std::chrono::steady_clock::now();

  io::Command last_command;
  double last_t = -1;
  std::array<int, 3> observation_counts{0, 0, 0};
  std::array<int, 3> pose_counts{0, 0, 0};
  int pnp_failure_count = 0;
  int track_switch_count = 0;
  int last_track_id = -1;
  int solved_frames = 0;
  int blind_frames = 0;
  int current_solved_streak = 0;
  int longest_solved_streak = 0;
  int processed_frames = 0;
  int multi_track_frames = 0;
  int first_control_frame = -1;
  int first_predict_frame = -1;
  std::vector<double> detect_timings_ms;
  std::vector<double> postprocess_timings_ms;

  video.set(cv::CAP_PROP_POS_FRAMES, start_index);
  for (int i = 0; use_imu_text && i < start_index; i++) {
    double t, w, x, y, z;
    if (!(text >> t >> w >> x >> y >> z)) {
      use_imu_text = false;
      tools::logger()->warn(
        "[auto_buff_test] imu text ended before start-index, using identity gimbal quaternion");
    }
  }

  for (int frame_count = start_index; !exiter.exit(); frame_count++) {
    if (end_index > 0 && frame_count > end_index) break;

    FrameTiming timing;
    const auto frame_start = Clock::now();

    auto step_start = Clock::now();
    video.read(img);
    timing.read_ms = elapsed_ms(step_start, Clock::now());
    if (img.empty()) break;

    step_start = Clock::now();
    double t = frame_count * frame_dt;
    double w = 1.0, x = 0.0, y = 0.0, z = 0.0;
    if (use_imu_text && !(text >> t >> w >> x >> y >> z)) {
      use_imu_text = false;
      t = frame_count * frame_dt;
      w = 1.0;
      x = y = z = 0.0;
      tools::logger()->warn(
        "[auto_buff_test] failed to read imu text, using identity gimbal quaternion");
    }
    auto timestamp = t0 + std::chrono::microseconds(int(t * 1e6));
    timing.imu_ms = elapsed_ms(step_start, Clock::now());

    /// 自瞄核心逻辑

    step_start = Clock::now();
    solver.set_R_gimbal2world({w, x, y, z});
    timing.set_imu_ms = elapsed_ms(step_start, Clock::now());

    step_start = Clock::now();
    auto buff_observations = detector.detect_tracks(img, detector_mode, timestamp);
    processed_frames++;
    if (buff_observations.size() > 1) multi_track_frames++;
    for (const auto & observation : buff_observations) {
      observation_counts[observation_index(observation.type)]++;
    }
    if (!buff_observations.empty()) {
      const auto primary = std::find_if(
        buff_observations.begin(), buff_observations.end(),
        [](const auto_buff::BuffObservation & observation) { return observation.primary; });
      if (primary != buff_observations.end()) {
        if (last_track_id >= 0 && primary->track_id != last_track_id) track_switch_count++;
        last_track_id = primary->track_id;
      }
    }
    timing.detect_ms = elapsed_ms(step_start, Clock::now());

    step_start = Clock::now();
    auto power_runes = solver.solve_all(buff_observations);
    pnp_failure_count += static_cast<int>(buff_observations.size() - power_runes.size());
    for (const auto & rune : power_runes) pose_counts[pose_index(rune.pose_quality)]++;
    timing.solve_ms = elapsed_ms(step_start, Clock::now());

    step_start = Clock::now();
    target->get_target(power_runes, timestamp);
    if (target->can_control() && first_control_frame < 0) first_control_frame = frame_count;
    if (target->prediction_ready() && first_predict_frame < 0) first_predict_frame = frame_count;
    if (!target->is_unsolve()) {
      solved_frames++;
      current_solved_streak++;
      longest_solved_streak = std::max(longest_solved_streak, current_solved_streak);
      if (target->is_blind()) blind_frames++;
    } else {
      current_solved_streak = 0;
    }
    timing.target_ms = elapsed_ms(step_start, Clock::now());

    step_start = Clock::now();
    auto aim_target_copy = target->clone();

    auto command = aimer.aim(*aim_target_copy, timestamp, 22, false);
    timing.aim_ms = elapsed_ms(step_start, Clock::now());
    detect_timings_ms.push_back(timing.detect_ms);
    postprocess_timings_ms.push_back(timing.solve_ms + timing.target_ms + timing.aim_ms);

    // cboard.send(command);

    // -------------- 调试输出 --------------

    const auto debug_start = Clock::now();
    nlohmann::json data;

    // data["bullet_speed"] = cboard.bullet_speed;

    // buff原始观测数据
    if (!power_runes.empty()) {
      const auto & p = power_runes.front();
      data["buff_R_yaw"] = p.ypd_in_world[0];
      data["buff_R_pitch"] = p.ypd_in_world[1];
      data["buff_R_dis"] = p.ypd_in_world[2];
    }

    if (!target->is_unsolve()) {
      auto * p = power_runes.empty() ? nullptr : &power_runes.front();
      std::optional<std::vector<cv::Point2f>> pnp_points;
      std::optional<PixelCompare> green_compare;
      std::optional<PixelCompare> blue_compare;
      double green_r_error = -1.0;

      if (p != nullptr) {
        // 显示
        for (const auto & point : p->target().points) tools::draw_point(img, point);
        for (const auto & point : p->target().fan_points) {
          tools::draw_point(img, point, {0, 128, 255});
        }
        if (!p->target().points.empty()) tools::draw_point(img, p->target().center, {0, 0, 255}, 3);
        tools::draw_point(img, p->r_center, {0, 0, 255}, 3);

        // raw PNP重投影: 只检查8个关键点PNP本身，不经过EKF和预测
        pnp_points = solver.reproject_pnp_points();
        if (pnp_points.has_value()) {
          tools::draw_points(
            img, std::vector<cv::Point2f>(pnp_points->begin(), pnp_points->begin() + 4),
            {0, 255, 255});
          tools::draw_points(
            img, std::vector<cv::Point2f>(pnp_points->begin() + 4, pnp_points->end()),
            {0, 255, 255});
        }
      }

      // 当前帧target更新后buff
      auto Rxyz_in_world_now = target->point_buff2world(Eigen::Vector3d(0.0, 0.0, 0.0));
      auto green_points =
        solver.reproject_buff(Rxyz_in_world_now, target->rotation_buff2world());
      if (p != nullptr && !green_points.empty()) {
        green_r_error = cv::norm(green_points.back() - p->r_center);
      }
      tools::draw_points(
        img, std::vector<cv::Point2f>(green_points.begin(), green_points.begin() + 4),
        {0, 255, 0});
      tools::draw_points(
        img, std::vector<cv::Point2f>(green_points.begin() + 4, green_points.end()),
        {0, 255, 0});
      if (pnp_points.has_value()) {
        green_compare = compare_points(*pnp_points, green_points);
        if (green_compare.has_value()) add_compare_data(data, "green_vs_raw", green_compare.value());
      }

      // buff瞄准位置(预测)
      auto debug_target_copy = target->clone();
      debug_target_copy->predict(debug_predict_time);
      double dangle = target->ekf_x()[5] - debug_target_copy->ekf_x()[5];
      auto Rxyz_in_world_pre =
        debug_target_copy->point_buff2world(Eigen::Vector3d(0.0, 0.0, 0.0));
      auto blue_points =
        solver.reproject_buff(Rxyz_in_world_pre, debug_target_copy->rotation_buff2world());
      tools::draw_points(
        img, std::vector<cv::Point2f>(blue_points.begin(), blue_points.begin() + 4), {255, 0, 0});
      tools::draw_points(
        img, std::vector<cv::Point2f>(blue_points.begin() + 4, blue_points.end()), {255, 0, 0});
      if (pnp_points.has_value()) {
        blue_compare = compare_points(*pnp_points, blue_points);
        if (blue_compare.has_value()) add_compare_data(data, "blue_vs_raw", blue_compare.value());
      }

      int debug_y = 130;
      if (green_compare.has_value()) {
        tools::draw_text(img, compare_text("green ekf", green_compare.value()), {20, debug_y},
          {0, 255, 0}, 0.55, 1);
        debug_y += 22;
      }
      if (blue_compare.has_value()) {
        tools::draw_text(img, compare_text("blue pred", blue_compare.value()), {20, debug_y},
          {255, 0, 0}, 0.55, 1);
      }

      if (print_debug && frame_count % print_step == 0) {
        std::string green_text = green_compare.has_value()
                                   ? compare_terminal_text("green", green_compare.value())
                                   : "green:NA";
        std::string blue_text =
          blue_compare.has_value() ? compare_terminal_text("blue", blue_compare.value())
                                   : (p == nullptr ? "blue:blind" : "blue:NA");
        if (!green_compare.has_value() && p == nullptr) green_text = "green:blind";
        const double debug_spd_deg = target->ekf_x().size() > 6 ? target->ekf_x()[6] * 57.3 : 0.0;
        int debug_pred_image_direction = 0;
        auto direction_probe = target->clone();
        direction_probe->predict(0.01);
        const auto direction_center =
          direction_probe->point_buff2world(Eigen::Vector3d(0.0, 0.0, 0.0));
        const auto direction_points =
          solver.reproject_buff(direction_center, direction_probe->rotation_buff2world());
        if (green_points.size() >= 9 && direction_points.size() >= 9) {
          const double current_image_phase =
            std::atan2(
              mean_point(green_points, 0, 4).y - green_points.back().y,
              mean_point(green_points, 0, 4).x - green_points.back().x);
          const double predicted_image_phase =
            std::atan2(
              mean_point(direction_points, 0, 4).y - direction_points.back().y,
              mean_point(direction_points, 0, 4).x - direction_points.back().x);
          const double image_delta =
            tools::limit_rad(predicted_image_phase - current_image_phase);
          debug_pred_image_direction = image_delta > 1e-5 ? 1 : (image_delta < -1e-5 ? -1 : 0);
        }
        fmt::print(
          "frame={} t={:.3f} mode={} imu={} spd={:.1f}deg/s phase={:.1f}deg plane={:.1f}deg "
          "Rdis={:.2f}/{:.2f}m green_R={:.1f}px pred_img_dir={} {} | {}\n",
          frame_count, t, buff_mode, use_imu_text ? "txt" : "identity", debug_spd_deg,
          target->ekf_x()[5] * 57.3, target->ekf_x()[4] * 57.3,
          p != nullptr ? p->ypd_in_world[2] : 0.0, target->ekf_x()[3], green_r_error,
          debug_pred_image_direction, green_text, blue_text);
      }

      // 观测器内部数据
      Eigen::VectorXd x = target->ekf_x();
      data["R_yaw"] = x[0];
      data["R_V_yaw"] = x[1];
      data["R_pitch"] = x[2];
      data["R_dis"] = x[3];
      data["yaw"] = x[4] * 57.3;

      data["angle"] = x[5] * 57.3;
      data["spd"] = x[6] * 57.3;
      if (x.size() >= 10) {
        data["spd"] = x[6];
        data["a"] = x[7];
        data["w"] = x[8];
        data["fi"] = x[9];
        data["spd0"] = target->spd;
      }
    }

    tools::draw_text(
      img, "yellow: raw PNP  green: EKF now  blue: prediction", {20, 30}, {0, 255, 255},
      0.6, 1);
    tools::draw_text(
      img, use_imu_text ? "imu: txt" : "imu: identity fallback", {20, 55}, {0, 255, 255}, 0.6,
      1);
    tools::draw_text(img, fmt::format("mode: {}", buff_mode), {20, 80}, {0, 255, 255}, 0.6, 1);
    tools::draw_text(
      img, fmt::format("debug predict: {:.3f}s", debug_predict_time), {20, 105}, {0, 255, 255},
      0.6, 1);

    // 云台响应情况
    Eigen::Vector3d ypr = tools::eulers(solver.R_gimbal2world(), 2, 1, 0);
    data["gimbal_yaw"] = ypr[0] * 57.3;
    data["gimbal_pitch"] = -ypr[1] * 57.3;

    if (command.control) {
      data["cmd_yaw"] = command.yaw * 57.3;
      data["cmd_pitch"] = command.pitch * 57.3;
    }

    timing.debug_ms = elapsed_ms(debug_start, Clock::now());
    if (print_time) add_timing_data(data, timing);

    step_start = Clock::now();
    plotter.plot(data);
    timing.plot_ms = elapsed_ms(step_start, Clock::now());

    step_start = Clock::now();
    if (!headless) {
      cv::imshow("result", img);

      int key = cv::waitKey(50);
      if (key == 'q') break;
      while (key == ' ') {
        int y = cv::waitKey(30);
        if (y == 'q') break;
      }
    }
    timing.show_wait_ms = elapsed_ms(step_start, Clock::now());
    timing.total_ms = elapsed_ms(frame_start, Clock::now());

    if (print_time && frame_count % print_step == 0) {
      fmt::print("frame={} {}\n", frame_count, timing_terminal_text(timing));
    }
  }
  fmt::print(
    "summary frames={} obs(full/target/fan)={}/{}/{} pose(8/5/4)={}/{}/{} "
    "pnp_fail={} temporal_reject={} gate_episode={} track_switch={}/{} ekf_reset={} solved={} blind={} "
    "longest_solved={} multi_track_frames={} first_control={} first_predict={} "
    "p95_detect={:.2f}ms p95_post={:.2f}ms imu_valid={}\n",
    processed_frames, observation_counts[0], observation_counts[1], observation_counts[2],
    pose_counts[0], pose_counts[1], pose_counts[2], pnp_failure_count,
    detector.temporal_reject_count(),
    detector.gate_failure_count(), track_switch_count, detector.confirmed_switch_count(),
    target->reset_count(), solved_frames, blind_frames, longest_solved_streak, multi_track_frames,
    first_control_frame, first_predict_frame, percentile_ms(detect_timings_ms, 0.95),
    percentile_ms(postprocess_timings_ms, 0.95), use_imu_text);
  cv::destroyAllWindows();
  text.close();  // 关闭文件

  return 0;
}
