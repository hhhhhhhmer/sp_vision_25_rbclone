// rm_aim_sim 能量机关（打符）共享内存接入
//
// 数据流：
//   request.json -> 仿真渲染 -> frame.json + frame-*.raw
//   -> RuneSystem.process(image, capture_time, q_gimbal2world, mode, color, bullet_speed)
//   -> command.json(yaw/pitch/fire) -> 仿真二阶云台 -> 出膛/命中/环数
//
// 与 rm_aim_sim_client（装甲）的区别只在“算法入口”：本程序用 auto_buff::RuneSystem，
// 不经过 auto_aim 的 Solver/Tracker/Planner。协议、云台符号、开火对齐门限与前者一致。
//
// 运行目录约定为 build/（模型路径相对仓库根目录解析）。

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <memory>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include <fcntl.h>
#include <sys/file.h>
#include <sys/stat.h>
#include <unistd.h>

#include <Eigen/Geometry>
#include <fmt/format.h>
#include <nlohmann/json.hpp>
#include <opencv2/opencv.hpp>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <yaml-cpp/yaml.h>

#include "tasks/auto_buff/rune_debug_draw.hpp"
#include "tasks/auto_buff/rune_system.hpp"
#include "tools/logger.hpp"
#include "tools/periodic_timer.hpp"
#include "tools/thread_safe_queue.hpp"

namespace
{

using json = nlohmann::json;
using Clock = std::chrono::steady_clock;

constexpr const char * kProtocol = "rm_aim_sim/2";
constexpr std::size_t kMaxJsonBytes = 65536;

std::atomic<bool> g_running{true};

void on_signal(int) { g_running.store(false); }

double now_unix_seconds()
{
  return std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count();
}

double seconds_since(Clock::time_point t)
{
  if (t.time_since_epoch().count() == 0) return 1e9;
  return std::chrono::duration<double>(Clock::now() - t).count();
}

bool valid_channel(const std::string & channel)
{
  if (channel.empty() || channel.size() > 80) return false;
  return std::all_of(channel.begin(), channel.end(), [](unsigned char c) {
    return std::isalnum(c) || c == '_' || c == '-';
  });
}

bool read_json_file(const std::string & path, json & out)
{
  std::ifstream in(path, std::ios::binary);
  if (!in) return false;
  std::string text((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
  if (text.empty() || text.size() > kMaxJsonBytes) return false;
  try {
    out = json::parse(text);
  } catch (const std::exception &) {
    return false;
  }
  return true;
}

// 协议要求：同目录临时文件写完再原子 rename，读者只会看到完整旧值或完整新值。
bool write_json_file(const std::string & directory, const std::string & name, const json & value)
{
  const std::string final_path = directory + "/" + name;
  const std::string temporary = final_path + ".client.tmp";
  {
    std::ofstream out(temporary, std::ios::binary | std::ios::trunc);
    if (!out) return false;
    out << value.dump();
    out.flush();
    if (!out) return false;
  }
  return std::rename(temporary.c_str(), final_path.c_str()) == 0;
}

std::vector<double> number_array(const json & node, std::size_t expected, const char * name)
{
  if (!node.is_array() || node.size() != expected) {
    throw std::runtime_error(fmt::format("{} 必须是 {} 个数字的数组", name, expected));
  }
  auto values = node.get<std::vector<double>>();
  for (double value : values) {
    if (!std::isfinite(value)) throw std::runtime_error(fmt::format("{} 含非有限数", name));
  }
  return values;
}

struct FrameRequest
{
  std::string request_id;
  std::string publisher_id;
};

struct SimFrame
{
  json metadata;
  cv::Mat bgr;
  Eigen::Quaterniond q_gimbal2world = Eigen::Quaterniond::Identity();
  Eigen::Matrix3d camera_matrix = Eigen::Matrix3d::Identity();
  std::vector<double> distort_coeffs;
  Eigen::Matrix3d R_camera2gimbal = Eigen::Matrix3d::Identity();
  Eigen::Vector3d t_camera2gimbal = Eigen::Vector3d::Zero();
  int width = 0;
  int height = 0;
  double sim_time = 0.0;
  double camera_timestamp = 0.0;
  Clock::time_point capture_time{};
  double gimbal_yaw = 0.0;
  double gimbal_pitch = 0.0;
  std::string session_id;
  std::string publisher_id;
  Clock::time_point received_at{};
};

// ---------------------------------------------------------------------------
// 本机共享内存客户端：锁、状态、指令、图像握手；不阻塞在算法里
// ---------------------------------------------------------------------------
class RmSimClient
{
public:
  RmSimClient(std::string channel, std::chrono::milliseconds connect_timeout)
  : channel_(std::move(channel))
  {
    if (!valid_channel(channel_)) {
      throw std::invalid_argument("IPC channel 只能包含字母、数字、下划线、连字符（1-80 位）");
    }
    directory_ = "/dev/shm/rm_aim_sim_" + channel_;
    wait_for_directory(connect_timeout);
    open_lock();
    token_ = fmt::format(
      "rune-{}-{:x}", static_cast<long>(::getpid()),
      static_cast<unsigned long long>(Clock::now().time_since_epoch().count()));
    tools::logger()->info("本机 IPC 已连接: {} (token={})", directory_, token_);
  }

  RmSimClient(const RmSimClient &) = delete;
  RmSimClient & operator=(const RmSimClient &) = delete;

  ~RmSimClient() { stop(); }

  bool wait_for_state(std::chrono::milliseconds timeout)
  {
    const auto deadline = Clock::now() + timeout;
    bool reported = false;
    while (g_running.load() && Clock::now() < deadline) {
      if (poll_state(std::chrono::milliseconds(0))) return true;
      if (!reported) {
        tools::logger()->info("等待仿真发布 state.json（确认仿真已启动且 channel 一致）...");
        reported = true;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    return has_state();
  }

  // 返回 true 表示读到了新的 publication。
  bool poll_state(std::chrono::milliseconds timeout)
  {
    const auto deadline = Clock::now() + timeout;
    bool first = true;
    while (g_running.load() && (first || Clock::now() < deadline)) {
      first = false;
      json value;
      if (
        read_json_file(directory_ + "/state.json", value) &&
        value.value("protocol", "") == kProtocol && value.value("type", "") == "state") {
        const std::string publisher = value.value("publisher_id", "");
        const int64_t publication = value.value("publication", static_cast<int64_t>(-1));
        const std::string session = value.value("session_id", "");
        const double wall = value.value("wall_unix_sec", 0.0);
        const double age = now_unix_seconds() - wall;
        // 拒绝崩溃遗留或来自另一个时钟域的旧状态：只接受 -100ms ~ +250ms。
        if (
          !publisher.empty() && publication >= 0 && !session.empty() && age >= -0.1 &&
          age <= 0.25) {
          std::lock_guard<std::mutex> lock(mutex_);
          if (has_state_ && publisher == publisher_id_ && publication == publication_) {
            return false;
          }
          state_ = std::move(value);
          has_state_ = true;
          publisher_id_ = publisher;
          session_id_ = session;
          publication_ = publication;
          return true;
        }
      }
      if (timeout.count() == 0) break;
      std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    return false;
  }

  bool has_state() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return has_state_;
  }

  json state() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return state_;
  }

  std::string publisher_id() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return publisher_id_;
  }

  bool send_command(
    bool control, bool fire, double yaw, double pitch, double yaw_vel = 0.0, double pitch_vel = 0.0,
    double yaw_acc = 0.0, double pitch_acc = 0.0)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!has_state_ || session_id_.empty()) return false;

    if (sequence_session_ != session_id_) {
      sequence_session_ = session_id_;
      sequence_ = 0;
    }
    int64_t last_seq = -1;
    if (state_.contains("ipc") && state_["ipc"].is_object()) {
      last_seq = state_["ipc"].value("last_seq", static_cast<int64_t>(-1));
    }
    sequence_ = std::max<int64_t>(sequence_, last_seq + 1);

    const json packet = {
      {"protocol", kProtocol},
      {"type", "command"},
      {"session_id", session_id_},
      {"publisher_id", publisher_id_},
      {"seq", sequence_},
      {"control", control},
      {"fire", fire},
      {"yaw", yaw},
      {"pitch", pitch},
      {"yaw_vel", yaw_vel},
      {"pitch_vel", pitch_vel},
      {"yaw_acc", yaw_acc},
      {"pitch_acc", pitch_acc}};
    if (!write_json_file(directory_, "command.json", packet)) return false;
    ++sequence_;
    return true;
  }

  FrameRequest request_frame()
  {
    FrameRequest request;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (!has_state_) return request;
      request.publisher_id = publisher_id_;
      request.request_id = fmt::format("{}-{}", token_, ++request_counter_);
    }
    const json packet = {
      {"publisher_id", request.publisher_id}, {"request_id", request.request_id}};
    if (!write_json_file(directory_, "request.json", packet)) return FrameRequest{};
    return request;
  }

  std::optional<SimFrame> receive_frame(
    const FrameRequest & request, std::chrono::milliseconds timeout)
  {
    const auto deadline = Clock::now() + timeout;
    while (g_running.load() && Clock::now() < deadline) {
      json metadata;
      if (
        read_json_file(directory_ + "/frame.json", metadata) &&
        metadata.value("protocol", "") == kProtocol && metadata.value("type", "") == "frame" &&
        metadata.value("request_id", "") == request.request_id &&
        metadata.value("publisher_id", "") == request.publisher_id) {
        return decode_frame(metadata, request);
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    return std::nullopt;
  }

  void stop()
  {
    bool expected = false;
    if (!stopped_.compare_exchange_strong(expected, true)) return;
    try {
      send_command(false, false, 0.0, 0.0);
    } catch (...) {
      // 退出路径不抛异常；仿真端看门狗会兜底。
    }
    if (lock_fd_ >= 0) {
      ::close(lock_fd_);
      lock_fd_ = -1;
    }
  }

private:
  void wait_for_directory(std::chrono::milliseconds timeout)
  {
    const auto deadline = Clock::now() + timeout;
    while (g_running.load()) {
      struct stat info
      {
      };
      if (::stat(directory_.c_str(), &info) == 0) {
        if (!S_ISDIR(info.st_mode)) {
          throw std::runtime_error(directory_ + " 不是目录");
        }
        if (info.st_uid != ::getuid()) {
          throw std::runtime_error(directory_ + " 不属于当前用户");
        }
        if ((info.st_mode & 077) != 0) {
          throw std::runtime_error(directory_ + " 权限必须为 0700");
        }
        return;
      }
      if (Clock::now() >= deadline) {
        throw std::runtime_error(fmt::format(
          "等待 {} 超时。请确认 rm_aim_sim 正在运行、channel 一致，且仿真不是 --no-ipc。",
          directory_));
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    throw std::runtime_error("启动被中断");
  }

  void open_lock()
  {
    const std::string lock_path = directory_ + "/client.lock";
    lock_fd_ = ::open(lock_path.c_str(), O_RDWR | O_CREAT | O_CLOEXEC, 0600);
    if (lock_fd_ < 0) {
      throw std::runtime_error(fmt::format("无法打开 {}: {}", lock_path, std::strerror(errno)));
    }
    if (::flock(lock_fd_, LOCK_EX | LOCK_NB) != 0) {
      ::close(lock_fd_);
      lock_fd_ = -1;
      throw std::runtime_error(
        "同一通道已有其他客户端占用 client.lock；请先退出 control_demo.py / camera_demo.py "
        "或另一个自瞄。");
    }
  }

  std::optional<SimFrame> decode_frame(const json & metadata, const FrameRequest & request)
  {
    const std::string expected_file = "frame-" + request.request_id + ".raw";
    const std::string frame_file = metadata.value("frame_file", "");
    if (frame_file != expected_file) {
      throw std::runtime_error(
        fmt::format("frame_file 不匹配: {} (期望 {})", frame_file, expected_file));
    }

    const json & camera = metadata.at("camera");
    const int width = camera.at("width").get<int>();
    const int height = camera.at("height").get<int>();
    const std::string encoding = metadata.value("encoding", "");
    int channels = 0;
    if (encoding == "rgb8") {
      channels = 3;
    } else if (encoding == "rgba8") {
      channels = 4;
    } else {
      throw std::runtime_error("不支持的 encoding: " + encoding);
    }
    if (width <= 0 || height <= 0) throw std::runtime_error("非法图像尺寸");
    const std::size_t expected_bytes =
      static_cast<std::size_t>(width) * static_cast<std::size_t>(height) * channels;
    const int64_t image_bytes = metadata.value("image_bytes", static_cast<int64_t>(-1));
    if (image_bytes != static_cast<int64_t>(expected_bytes)) {
      throw std::runtime_error(
        fmt::format("image_bytes={} 与 {}x{}x{} 不一致", image_bytes, width, height, channels));
    }

    const std::string path = directory_ + "/" + frame_file;
    std::ifstream in(path, std::ios::binary | std::ios::ate);
    if (!in) throw std::runtime_error("无法打开图像文件 " + path);
    const std::streamoff file_size = in.tellg();
    if (file_size != static_cast<std::streamoff>(expected_bytes)) {
      throw std::runtime_error(fmt::format("图像文件大小 {} 与 image_bytes 不一致", file_size));
    }
    in.seekg(0, std::ios::beg);
    std::vector<unsigned char> pixels(expected_bytes);
    if (!in.read(
          reinterpret_cast<char *>(pixels.data()), static_cast<std::streamsize>(expected_bytes))) {
      throw std::runtime_error("读取图像文件失败");
    }
    in.close();

    const cv::Mat rgb(height, width, CV_MAKETYPE(CV_8U, channels), pixels.data());
    SimFrame frame;
    cv::cvtColor(rgb, frame.bgr, channels == 4 ? cv::COLOR_RGBA2BGR : cv::COLOR_RGB2BGR);
    frame.metadata = metadata;
    frame.width = width;
    frame.height = height;
    frame.sim_time = metadata.value("sim_time", 0.0);
    frame.session_id = metadata.value("session_id", "");
    frame.publisher_id = metadata.value("publisher_id", "");
    frame.received_at = Clock::now();
    frame.camera_timestamp = camera.value("timestamp", frame.sim_time);
    // 重要：RuneSystem / RuneDecisionModule 用 steady_clock::now() 与 capture_time 比较
    // 判断目标是否过期（temp.data_life、critical_timeout 等），所以这里必须传本机
    // steady_clock 时间，不能传仿真的 sim_time。用“收到帧的时刻 - 发布/读回开销”
    // 近似真实曝光时刻。
    double transport_ms = 0.0;
    if (metadata.contains("capture_timing") && metadata["capture_timing"].is_object()) {
      transport_ms = metadata["capture_timing"].value("publish_ms", 0.0) +
                     metadata["capture_timing"].value("readback_ms", 0.0);
    }
    frame.capture_time =
      frame.received_at - std::chrono::duration_cast<Clock::duration>(
                            std::chrono::duration<double, std::milli>(transport_ms));

    const json & gimbal = metadata.at("gimbal");
    const auto q = number_array(gimbal.at("quaternion_wxyz"), 4, "gimbal.quaternion_wxyz");
    frame.q_gimbal2world = Eigen::Quaterniond(q[0], q[1], q[2], q[3]);
    if (!frame.q_gimbal2world.coeffs().allFinite() || frame.q_gimbal2world.norm() < 1e-9) {
      throw std::runtime_error("非法 gimbal.quaternion_wxyz");
    }
    frame.q_gimbal2world.normalize();
    frame.gimbal_yaw = gimbal.value("yaw", 0.0);
    frame.gimbal_pitch = gimbal.value("pitch", 0.0);

    const auto K = number_array(camera.at("K"), 9, "camera.K");
    frame.camera_matrix << K[0], K[1], K[2], K[3], K[4], K[5], K[6], K[7], K[8];
    frame.distort_coeffs = camera.value("distortion", std::vector<double>{});
    const auto R = number_array(camera.at("R_camera2gimbal"), 9, "camera.R_camera2gimbal");
    frame.R_camera2gimbal << R[0], R[1], R[2], R[3], R[4], R[5], R[6], R[7], R[8];
    const auto t = number_array(camera.at("t_camera2gimbal"), 3, "camera.t_camera2gimbal");
    frame.t_camera2gimbal << t[0], t[1], t[2];
    return frame;
  }

  std::string channel_;
  std::string directory_;
  std::string token_;
  int lock_fd_ = -1;
  std::atomic<bool> stopped_{false};

  mutable std::mutex mutex_;
  json state_;
  bool has_state_ = false;
  std::string publisher_id_;
  std::string session_id_;
  int64_t publication_ = -1;
  int64_t sequence_ = 0;
  std::string sequence_session_;
  uint64_t request_counter_ = 0;
};

// ---------------------------------------------------------------------------
// 跨线程快照
// ---------------------------------------------------------------------------
struct RuneSnapshot
{
  auto_buff::RuneCommand command{};
  auto_buff::RuneDebugSnapshot debug{};
  double sim_time = 0.0;
  Clock::time_point received_at{};
  std::string session_id;
  cv::Mat image;
};

struct Options
{
  std::string config_path = "../configs/rm_aim_sim_rune.yaml";
  std::string channel = "default";
  std::string rune = "auto";  // auto / small / big
  std::string color = "auto";  // auto / red / blue
  bool fire = false;
  bool display = true;
  double duration = 0.0;
  double fire_tolerance = 1.0;
  int hold_ms = 150;
  int target_timeout_ms = 300;
  int wait_seconds = 8;
  std::string probe_image;
};

void draw_status(cv::Mat & image, const RuneSnapshot & snapshot, const json & state)
{
  const auto & command = snapshot.command;
  const auto & debug = snapshot.debug;
  const std::string line1 = fmt::format(
    "found {} fire {}  yaw {:+.2f}deg  pitch {:+.2f}deg (up+)",
    command.found ? "YES" : "no", command.fire ? "ON" : "off", command.yaw * 180.0 / CV_PI,
    command.pitch * 180.0 / CV_PI);
  const std::string line2 = fmt::format(
    "dets {}  core {:.1f}ms  fail {}  uv {}  pnp {}", debug.detections.size(), debug.core_ms,
    static_cast<int>(debug.failure_reason), debug.uv_pose_active ? "yes" : "no",
    debug.pnp_used ? "yes" : "no");
  cv::putText(
    image, line1, {20, 90}, cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 0), 1, cv::LINE_AA);
  cv::putText(
    image, line2, {20, 115}, cv::FONT_HERSHEY_SIMPLEX, 0.55, cv::Scalar(0, 255, 255), 1,
    cv::LINE_AA);
  if (state.contains("rune") && state["rune"].is_object()) {
    const auto & rune = state["rune"];
    const std::string line3 = fmt::format(
      "sim rune {} {}  rings {}/{}  speed {:.2f}rad/s", rune.value("kind", "?"),
      rune.value("state", "?"), rune.value("rings", 0), rune.value("max_rings", 0),
      rune.value("speed", 0.0));
    cv::putText(
      image, line3, {20, 140}, cv::FONT_HERSHEY_SIMPLEX, 0.55, cv::Scalar(255, 255, 255), 1,
      cv::LINE_AA);
  }
}

std::string team_to_color(const json & state, const std::string & override_color)
{
  if (override_color == "red" || override_color == "blue") return override_color;
  const auto team = state.contains("appearance") ? state["appearance"].value("team", "blue") : "blue";
  return team == "red" ? "red" : "blue";
}

auto_buff::BuffMode parse_mode(const std::string & name, const json & state)
{
  if (name == "big") return auto_buff::BuffMode::BIG;
  if (name == "small") return auto_buff::BuffMode::SMALL;
  const auto kind = state.contains("rune") ? state["rune"].value("kind", "small") : "small";
  return kind == "large" ? auto_buff::BuffMode::BIG : auto_buff::BuffMode::SMALL;
}

void check_camera_config(
  const std::string & config_path, const SimFrame & frame, std::atomic<bool> & warned)
{
  if (warned.exchange(true)) return;
  try {
    const auto yaml = YAML::LoadFile(config_path);
    const auto K = yaml["camera_matrix"].as<std::vector<double>>();
    const auto R = yaml["R_camera2gimbal"].as<std::vector<double>>();
    const auto t = yaml["t_camera2gimbal"].as<std::vector<double>>();
    if (K.size() != 9 || R.size() != 9 || t.size() != 3) {
      tools::logger()->warn("配置里的相机参数长度不对；RuneSystem 只能用 YAML 内参，请检查");
      return;
    }
    const double fx_diff = std::abs(K[0] - frame.camera_matrix(0, 0)) / frame.camera_matrix(0, 0);
    const double cx_diff = std::abs(K[2] - frame.camera_matrix(0, 2)) / frame.width;
    double r_diff = 0.0;
    Eigen::Matrix3d R_config;
    R_config << R[0], R[1], R[2], R[3], R[4], R[5], R[6], R[7], R[8];
    r_diff = (R_config - frame.R_camera2gimbal).cwiseAbs().maxCoeff();
    double t_diff = 0.0;
    for (int i = 0; i < 3; ++i) {
      t_diff = std::max(t_diff, std::abs(t[i] - frame.t_camera2gimbal(i)));
    }
    if (fx_diff > 0.01 || cx_diff > 0.01 || r_diff > 0.01 || t_diff > 0.01) {
      tools::logger()->warn(
        "配置内参与仿真 frame.json 不一致（fx {:.1f} vs {:.1f}, cx {:.1f} vs {:.1f}, "
        "R 最大差 {:.3f}, t 最大差 {:.3f} m）。RuneSystem 在构造时读取 YAML 内参，"
        "不会随帧更新；请在 {} 里改成仿真当前相机参数。",
        K[0], frame.camera_matrix(0, 0), K[2], frame.camera_matrix(0, 2), r_diff, t_diff,
        config_path);
    }
  } catch (const std::exception & error) {
    tools::logger()->warn("读取相机配置失败: {}", error.what());
  }
}

int run(const Options & options)
{
  const auto color_override = options.color;

  // 离线单帧探针：不连仿真，只检查“模型 + 后处理”在仿真画面上能否出检测。
  if (!options.probe_image.empty()) {
    cv::Mat image = cv::imread(options.probe_image, cv::IMREAD_COLOR);
    if (image.empty()) {
      tools::logger()->error("无法读取图片 {}", options.probe_image);
      return 1;
    }
    auto_buff::RuneSystem rune(options.config_path);
    const auto mode = options.rune == "big" ? auto_buff::BuffMode::BIG
                                            : auto_buff::BuffMode::SMALL;
    const auto color = color_override == "red" ? auto_buff::EnemyColor::RED
                                               : auto_buff::EnemyColor::BLUE;
    const auto command = rune.process(
      image, Clock::now(), Eigen::Quaterniond::Identity(), mode, color, 23.0f);
    const auto & debug = rune.debug_snapshot();
    tools::logger()->info(
      "{} 检测结果: {} 个, found={} fire={} fail={} yaw={:+.2f}° pitch={:+.2f}°",
      options.probe_image, debug.detections.size(), command.found, command.fire,
      static_cast<int>(debug.failure_reason), command.yaw * 180.0 / CV_PI,
      command.pitch * 180.0 / CV_PI);
    for (const auto & detection : debug.detections) {
      tools::logger()->info(
        "  cls={} conf={:.2f} kps=({:.0f},{:.0f}) ({:.0f},{:.0f}) ({:.0f},{:.0f}) "
        "({:.0f},{:.0f}) ({:.0f},{:.0f})",
        detection.class_id, detection.confidence, detection.keypoints[0].x,
        detection.keypoints[0].y, detection.keypoints[1].x, detection.keypoints[1].y,
        detection.keypoints[2].x, detection.keypoints[2].y, detection.keypoints[3].x,
        detection.keypoints[3].y, detection.keypoints[4].x, detection.keypoints[4].y);
    }
    return 0;
  }

  RmSimClient client(options.channel, std::chrono::seconds(options.wait_seconds));
  if (!client.wait_for_state(std::chrono::seconds(options.wait_seconds))) {
    tools::logger()->error(
      "在 {} 秒内没有读到新鲜的 state.json。请先启动仿真：cd rm_aim_sim && "
      "./run.sh -- --remote --config=res://configs/rune.cfg",
      options.wait_seconds);
    return 1;
  }

  std::string color = "blue";
  {
    const auto state = client.state();
    const auto & appearance = state["appearance"];
    const auto & camera = state["camera"];
    const auto & weapon = state["weapon"];
    const auto & ipc = state["ipc"];
    tools::logger()->info(
      "收到仿真状态: session={} sim_time={:.2f} target_mode={} robot={} team={} camera={}x{} "
      "bullet_speed={:.2f} remote={}",
      state.value("session_id", ""), state.value("sim_time", 0.0),
      state.value("target_mode", "?"), appearance.value("robot", "?"),
      appearance.value("team", "?"), camera.value("width", 0), camera.value("height", 0),
      weapon.value("speed", 0.0), ipc.value("enabled", false));
    if (state.value("target_mode", "") != "rune") {
      tools::logger()->warn(
        "仿真当前不是能量机关模式（target_mode={}）。请在左侧面板切换到“能量机关模式”，"
        "或用 --config=res://configs/rune.cfg 启动仿真。",
        state.value("target_mode", "?"));
    }
    if (!ipc.value("enabled", false)) {
      tools::logger()->warn(
        "仿真当前没有开启“本机自瞄控制”。本程序仍可取图/检测/显示，但 command.json 会被拒绝；"
        "请用 ./run.sh -- --remote 启动或在界面勾选。");
    }
    color = team_to_color(state, color_override);
    tools::logger()->info(
      "打符参数: mode={} enemy_color={} fire={} config={}", options.rune, color,
      options.fire ? "enabled" : "dry-run", options.config_path);
  }

  tools::ThreadSafeQueue<SimFrame> frame_queue(2, tools::OverflowPolicy::Block);
  tools::ThreadSafeQueue<RuneSnapshot> display_queue(1, tools::OverflowPolicy::DropOldest);

  std::mutex snapshot_mutex;
  RuneSnapshot latest;
  std::atomic<double> bullet_speed{23.0};
  std::atomic<bool> warned_camera{false};

  std::mutex error_mutex;
  std::exception_ptr worker_error;
  auto report_error = [&](std::exception_ptr error) {
    {
      std::lock_guard<std::mutex> lock(error_mutex);
      if (!worker_error) worker_error = error;
    }
    g_running.store(false);
    frame_queue.close();
    display_queue.close();
  };

  // 采图线程：上一帧复制完才发下一次请求，不会在仿真侧积压图像。
  std::thread capture_thread([&] {
    try {
      uint64_t timeouts = 0;
      while (g_running.load()) {
        if (!client.has_state()) {
          std::this_thread::sleep_for(std::chrono::milliseconds(5));
          continue;
        }
        const auto request = client.request_frame();
        if (request.request_id.empty()) {
          std::this_thread::sleep_for(std::chrono::milliseconds(5));
          continue;
        }
        auto frame = client.receive_frame(request, std::chrono::milliseconds(600));
        if (!frame) {
          ++timeouts;
          if (timeouts % 20 == 1) {
            tools::logger()->warn(
              "等待图像帧超时。仿真必须处于图形模式（--headless 没有像素输出），"
              "并确认窗口未最小化到无渲染。");
          }
          continue;
        }
        timeouts = 0;
        if (frame_queue.push(std::move(*frame)) == tools::QueuePushResult::Closed) break;
      }
    } catch (...) {
      report_error(std::current_exception());
    }
    frame_queue.close();
  });

  // 感知线程：RuneSystem 独占在本线程内（OpenVINO 上下文不跨线程共享）。
  std::thread perception_thread([&] {
    try {
      auto_buff::RuneSystem rune(options.config_path);
      std::string last_session;
      std::string last_publisher;
      std::string camera_signature;
      uint64_t frame_count = 0;
      while (g_running.load()) {
        auto captured = frame_queue.wait_pop_for(std::chrono::milliseconds(100));
        if (!captured) {
          if (frame_queue.closed()) break;
          continue;
        }
        auto & frame = *captured;

        if (frame.session_id != last_session || frame.publisher_id != last_publisher) {
          if (!last_session.empty()) {
            tools::logger()->info("仿真 session/publisher 变化，清空打符状态");
          }
          rune.reset();
          last_session = frame.session_id;
          last_publisher = frame.publisher_id;
          std::lock_guard<std::mutex> lock(snapshot_mutex);
          latest = RuneSnapshot{};
        }

        const std::string signature =
          fmt::format("{}x{}-{:.4f}", frame.width, frame.height, frame.camera_matrix(0, 0));
        if (!camera_signature.empty() && signature != camera_signature) {
          tools::logger()->warn("仿真相机内参/分辨率变化，清空打符状态；请同步修改配置内参");
          rune.reset();
        }
        camera_signature = signature;
        check_camera_config(options.config_path, frame, warned_camera);

        const auto state = client.state();
        const auto mode = parse_mode(options.rune, state);
        const auto enemy_color = color == "red" ? auto_buff::EnemyColor::RED
                                                : auto_buff::EnemyColor::BLUE;
        const auto command = rune.process(
          frame.bgr, frame.capture_time, frame.q_gimbal2world, mode, enemy_color,
          static_cast<float>(bullet_speed.load()));
        const auto & debug = rune.debug_snapshot();

        RuneSnapshot snapshot;
        snapshot.command = command;
        snapshot.debug = debug;
        snapshot.sim_time = frame.sim_time;
        snapshot.received_at = frame.received_at;
        snapshot.session_id = frame.session_id;
        {
          std::lock_guard<std::mutex> lock(snapshot_mutex);
          latest = snapshot;
        }

        if (options.display && frame_count % 2 == 0) {
          auto_buff::draw_rune_debug(frame.bgr, debug);
          snapshot.image = frame.bgr.clone();
          display_queue.push(std::move(snapshot));
        }

        if (frame_count % 30 == 0) {
          tools::logger()->info(
            "[rune] frames={} dets={} found={} fire={} fail={} core={:.1f}ms uv={} pnp={}",
            frame_count, debug.detections.size(), command.found, command.fire,
            static_cast<int>(debug.failure_reason), debug.core_ms,
            debug.uv_pose_active ? 1 : 0, debug.pnp_used ? 1 : 0);
        }
        ++frame_count;
      }
    } catch (...) {
      report_error(std::current_exception());
    }
    frame_queue.close();
  });

  // 控制线程：固定 100 Hz 读状态 + 下发，绝不等待图像或显示。
  std::thread control_thread([&] {
    try {
      tools::PeriodicTimer tick(std::chrono::milliseconds(10));
      std::string control_session;
      Clock::time_point last_state_time{};
      Clock::time_point last_control_time{};
      Clock::time_point last_log{};
      bool sent_control = false;
      auto_buff::RuneCommand cached{};
      while (g_running.load()) {
        tick.wait_next();
        if (client.poll_state(std::chrono::milliseconds(0))) {
          last_state_time = Clock::now();
        }
        if (!client.has_state()) continue;
        const auto state = client.state();
        const std::string session = state.value("session_id", "");
        if (control_session != session) {
          control_session = session;
          sent_control = false;
          cached = auto_buff::RuneCommand{};
          tools::logger()->info("新控制会话 {}：清空缓存指令", session);
        }

        if (state.contains("weapon")) {
          bullet_speed.store(state["weapon"].value("speed", 23.0));
        }

        const bool ipc_enabled = state.contains("ipc") && state["ipc"].value("enabled", false);
        const bool paused = state.value("paused", false);
        if (!ipc_enabled || paused) {
          if (sent_control) {
            client.send_command(false, false, 0.0, 0.0);
            sent_control = false;
          }
          continue;
        }
        if (seconds_since(last_state_time) > 0.20) {
          if (sent_control) {
            client.send_command(false, false, 0.0, 0.0);
            sent_control = false;
          }
          continue;
        }

        RuneSnapshot snapshot;
        {
          std::lock_guard<std::mutex> lock(snapshot_mutex);
          snapshot = latest;
        }
        const bool fresh =
          snapshot.command.found && snapshot.session_id == session &&
          seconds_since(snapshot.received_at) <= options.target_timeout_ms / 1000.0;

        bool control = false;
        bool fire = false;
        double yaw = 0.0;
        double pitch_sim = 0.0;
        if (fresh) {
          cached = snapshot.command;
          last_control_time = Clock::now();
          control = true;
          yaw = cached.yaw;
          // RuneSystem 的 pitch 抬头为正；仿真协议 pitch 向下为正，必须取反。
          pitch_sim = -cached.pitch;
        } else if (
          cached.found && seconds_since(last_control_time) <= options.hold_ms / 1000.0) {
          // 短时丢帧：保持最后瞄准角，但不开火。
          control = true;
          yaw = cached.yaw;
          pitch_sim = -cached.pitch;
        }

        if (control) {
          const double actual_yaw =
            state.contains("gimbal") ? state["gimbal"].value("yaw", 0.0) : 0.0;
          const double actual_pitch =
            state.contains("gimbal") ? state["gimbal"].value("pitch", 0.0) : 0.0;
          const double yaw_err_deg =
            std::abs(std::remainder(actual_yaw - yaw, 2.0 * CV_PI)) * 180.0 / CV_PI;
          const double pitch_err_deg = std::abs(actual_pitch - pitch_sim) * 180.0 / CV_PI;
          const bool aligned =
            yaw_err_deg <= options.fire_tolerance && pitch_err_deg <= options.fire_tolerance;
          fire = options.fire && fresh && cached.fire && aligned;

          if (client.send_command(true, fire, yaw, pitch_sim)) {
            sent_control = true;
            if (seconds_since(last_log) >= 1.0) {
              last_log = Clock::now();
              const auto & stats = state["stats"];
              const auto & rune = state["rune"];
              tools::logger()->info(
                "[control] yaw={:+.2f}° pitch={:+.2f}°(下发,下正) fire={} align=({:.2f},{:.2f})° "
                "fresh={} core_fire={} | sim_t={:.1f}s rune={}/{} rings={}/{} speed={:.2f} "
                "fired={} hit={} rate={:.1f}% seq={}",
                yaw * 180.0 / CV_PI, pitch_sim * 180.0 / CV_PI, fire, yaw_err_deg, pitch_err_deg,
                fresh, cached.fire, state.value("sim_time", 0.0), rune.value("state", "?"),
                rune.value("kind", "?"), stats.value("rune_rings", 0),
                stats.value("rune_max_rings", 0), rune.value("speed", 0.0),
                stats.value("fired", 0), stats.value("hit", 0),
                stats.value("hit_rate", 0.0) * 100.0,
                state.contains("ipc") ? state["ipc"].value("last_seq", -1) : -1);
            }
          }
        } else {
          fire = false;
          if (sent_control) {
            client.send_command(false, false, 0.0, 0.0);
            sent_control = false;
          }
          if (seconds_since(last_log) >= 1.0) {
            last_log = Clock::now();
            const bool has_frame = snapshot.received_at.time_since_epoch().count() != 0;
            tools::logger()->info(
              "[control] 无有效目标（found={} age={:.0f}ms fail={} dets={}）；不控制云台",
              snapshot.command.found,
              has_frame ? seconds_since(snapshot.received_at) * 1000.0 : -1.0,
              static_cast<int>(snapshot.debug.failure_reason),
              snapshot.debug.detections.size());
          }
        }
      }
    } catch (...) {
      report_error(std::current_exception());
    }
  });

  const auto start = Clock::now();
  const bool display = options.display && std::getenv("DISPLAY") != nullptr;
  if (options.display && !display) {
    tools::logger()->warn("DISPLAY 未设置，关闭调试窗口");
  }

  while (g_running.load()) {
    if (
      options.duration > 0.0 &&
      std::chrono::duration<double>(Clock::now() - start).count() >= options.duration) {
      g_running.store(false);
      break;
    }
    auto snapshot = display_queue.wait_pop_for(std::chrono::milliseconds(100));
    if (!snapshot) continue;
    if (!display || snapshot->image.empty()) continue;
    draw_status(snapshot->image, *snapshot, client.state());
    cv::Mat show;
    cv::resize(snapshot->image, show, {}, 0.5, 0.5);
    cv::imshow("rm_aim_sim_rune_client", show);
    const int key = cv::waitKey(1);
    if (key == 'q' || key == 27) {
      g_running.store(false);
      break;
    }
  }

  g_running.store(false);
  frame_queue.close();
  display_queue.close();
  if (capture_thread.joinable()) capture_thread.join();
  if (perception_thread.joinable()) perception_thread.join();
  if (control_thread.joinable()) control_thread.join();

  client.stop();
  cv::destroyAllWindows();

  if (worker_error) {
    try {
      std::rethrow_exception(worker_error);
    } catch (const std::exception & e) {
      tools::logger()->error("工作线程退出: {}", e.what());
      return 1;
    }
  }

  try {
    const auto state = client.state();
    if (!state.empty() && state.contains("stats")) {
      const auto & stats = state["stats"];
      tools::logger()->info(
        "结束汇总: sim_t={:.1f}s 尝试={} 成功={} 失败={} 环数={} | fired={} hit={} rate={:.1f}%",
        state.value("sim_time", 0.0), stats.value("rune_attempts", 0),
        stats.value("rune_activations", 0), stats.value("rune_failures", 0),
        stats.value("rune_rings_total", 0), stats.value("fired", 0), stats.value("hit", 0),
        stats.value("hit_rate", 0.0) * 100.0);
    }
  } catch (...) {
  }
  return 0;
}

void configure_console_log_level()
{
  auto logger = tools::logger();
  for (auto & sink : logger->sinks()) {
    if (std::dynamic_pointer_cast<spdlog::sinks::stdout_color_sink_mt>(sink)) {
      sink->set_level(spdlog::level::info);
    }
  }
}

void print_usage()
{
  std::printf(
    "用法: rm_aim_sim_rune_client [config.yaml] [选项]\n"
    "\n"
    "  --channel=default        共享内存通道，必须与仿真 [ipc].channel 一致\n"
    "  --rune=auto|small|big    机关类型；auto 读仿真状态里的 rune.kind\n"
    "  --color=auto|red|blue    敌方颜色；auto 用仿真 target team\n"
    "  --fire                   允许开火；默认只瞄准\n"
    "  --duration=0             运行秒数，0 表示一直运行\n"
    "  --no-display             不打开调试窗口\n"
    "  --fire-tolerance=1.0     实际云台与下发角的开火对齐容差（度）\n"
    "  --hold-ms=150            丢目标后保持最后瞄准角的时间\n"
    "  --target-timeout-ms=300  目标过期时间\n"
    "  --probe-image=PATH       只对单张图离线跑检测后退出\n"
    "  --wait=8                 等待仿真启动的秒数\n");
}
}  // namespace

int main(int argc, char ** argv)
{
  configure_console_log_level();
  std::signal(SIGINT, on_signal);
  std::signal(SIGTERM, on_signal);

  Options options;
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    if (arg == "--help" || arg == "-h") {
      print_usage();
      return 0;
    }
    auto value = [&](const std::string & prefix) -> std::optional<std::string> {
      if (arg.rfind(prefix, 0) != 0) return std::nullopt;
      return arg.substr(prefix.size());
    };
    if (arg == "--fire") {
      options.fire = true;
    } else if (arg == "--no-display") {
      options.display = false;
    } else if (auto v = value("--channel=")) {
      options.channel = *v;
    } else if (auto v = value("--rune=")) {
      options.rune = *v;
    } else if (auto v = value("--color=")) {
      options.color = *v;
    } else if (auto v = value("--duration=")) {
      options.duration = std::stod(*v);
    } else if (auto v = value("--fire-tolerance=")) {
      options.fire_tolerance = std::stod(*v);
    } else if (auto v = value("--hold-ms=")) {
      options.hold_ms = std::stoi(*v);
    } else if (auto v = value("--target-timeout-ms=")) {
      options.target_timeout_ms = std::stoi(*v);
    } else if (auto v = value("--wait=")) {
      options.wait_seconds = std::stoi(*v);
    } else if (auto v = value("--probe-image=")) {
      options.probe_image = *v;
    } else if (!arg.empty() && arg[0] != '-') {
      options.config_path = arg;
    } else {
      std::printf("未知参数: %s\n", arg.c_str());
      print_usage();
      return 2;
    }
  }

  if (options.rune != "auto" && options.rune != "small" && options.rune != "big") {
    tools::logger()->error("--rune 只能是 auto / small / big");
    return 2;
  }
  if (options.color != "auto" && options.color != "red" && options.color != "blue") {
    tools::logger()->error("--color 只能是 auto / red / blue");
    return 2;
  }

  try {
    return run(options);
  } catch (const std::exception & error) {
    tools::logger()->error("启动失败: {}", error.what());
    return 1;
  }
}
