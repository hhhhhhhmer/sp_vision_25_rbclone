// rm_aim_sim (RoboMaster Aim Lab) 本机共享内存自瞄接入程序
//
// 数据流：共享内存原始 RGB 帧 -> YOLO 检测 -> PnP/跟踪 -> MPC 规划 -> command.json
// 不依赖 ROS，直接复用本仓库的 tasks/auto_aim 与 tools。
//
// 协议说明见 rm_aim_sim 仓库 docs/protocol.md：
//   /dev/shm/rm_aim_sim_<channel>/
//     state.json   仿真 -> 本程序（约 60 Hz）
//     command.json 本程序 -> 仿真（本程序 100 Hz）
//     request.json / frame.json / frame-<id>.raw（图像握手）
//
// 坐标系：仿真 RM 世界系 x 前 / y 左 / z 上，yaw 左正，pitch 向下为正。
// 本仓库 Planner 的 plan.pitch 来自弹道解算，抬头为正，因此下发时取反。

#include <fcntl.h>
#include <fmt/core.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <sys/file.h>
#include <sys/stat.h>
#include <unistd.h>
#include <yaml-cpp/yaml.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <exception>
#include <fstream>
#include <functional>
#include <list>
#include <memory>
#include <mutex>
#include <nlohmann/json.hpp>
#include <opencv2/opencv.hpp>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "tasks/auto_aim/aiming/planner/planner.hpp"
#ifdef OPENVINO_MAKE
// 传统检测器依赖 Classifier 的 OpenVINO 头文件；TensorRT 构建里不编译它。
#include "tasks/auto_aim/detection/detector.hpp"
#endif
#include "tasks/auto_aim/detection/yolo.hpp"
#include "tasks/auto_aim/geometry/solver.hpp"
#include "tasks/auto_aim/tracking/tracker.hpp"
#include "tools/fps_solve.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
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
  std::vector<std::array<cv::Point2f, 4>> ideal_corners;
  int width = 0;
  int height = 0;
  double sim_time = 0.0;
  double gimbal_yaw = 0.0;
  double gimbal_pitch = 0.0;
  std::string session_id;
  std::string publisher_id;
  Clock::time_point received_at{};
};

// ---------------------------------------------------------------------------
// 本机共享内存客户端：负责锁、状态、指令、图像握手，不阻塞在算法里
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
      "cpp-{}-{:x}", static_cast<long>(::getpid()),
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

  // 返回 true 表示读到了新的 publication；false 表示没有新状态或状态暂时不新鲜。
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
          if (!has_state_ || publisher != publisher_id_ || session != session_id_) {
            ++epoch_;
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

  uint64_t epoch() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return epoch_;
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

    // 诊断用理想角点：只在显式 --show-ideal 时绘制，不能当作检测结果使用。
    if (camera.contains("armors") && camera["armors"].is_array()) {
      for (const auto & armor : camera["armors"]) {
        const auto corners = armor.value("corners", json::array());
        if (!corners.is_array() || corners.size() != 4) continue;
        std::array<cv::Point2f, 4> points{};
        bool valid = true;
        for (std::size_t i = 0; i < 4; ++i) {
          if (!corners[i].is_array() || corners[i].size() != 2) {
            valid = false;
            break;
          }
          points[i] = cv::Point2f(corners[i][0].get<float>(), corners[i][1].get<float>());
        }
        if (valid) frame.ideal_corners.push_back(points);
      }
    }
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
  uint64_t epoch_ = 0;
  int64_t sequence_ = 0;
  std::string sequence_session_;
  uint64_t request_counter_ = 0;
};

// ---------------------------------------------------------------------------
// 跨线程快照
// ---------------------------------------------------------------------------
struct TargetSnapshot
{
  std::optional<auto_aim::Target> target;
  double sim_time = 0.0;
  Clock::time_point received_at{};
  std::string session_id;
};

struct PlanSnapshot
{
  auto_aim::Plan plan{};
  bool valid = false;
  bool fire = false;
  double yaw = 0.0;
  double pitch_sim = 0.0;
  Clock::time_point time{};
};

struct Perception
{
  cv::Mat image;
  std::optional<auto_aim::Target> target;
  double sim_time = 0.0;
  double gimbal_yaw = 0.0;
  Clock::time_point received_at{};
  std::string session_id;
  std::size_t armor_count = 0;
  double detect_ms = 0.0;
  double track_ms = 0.0;
  std::string tracker_state;
};

struct Options
{
  std::string config_path = "../configs/rm_aim_sim.yaml";
  std::string channel = "default";
  bool fire = false;
  bool display = true;
  bool show_ideal = false;
  double duration = 0.0;
  std::string strategy = "rb";
  std::string detector = "traditional";
  bool color_filter = false;
  std::string probe_image;
  int blur = 3;
  double fire_tolerance = 1.0;
  int hold_ms = 80;
  int target_timeout_ms = 150;
  int wait_seconds = 8;
};

auto_aim::Planner::ShootStrategy parse_strategy(const std::string & name)
{
  if (name == "rb") return auto_aim::Planner::ShootStrategy::rbSuppressiveFire;
  if (name == "sb") return auto_aim::Planner::ShootStrategy::SB;
  if (name == "hero") return auto_aim::Planner::ShootStrategy::rbHero;
  if (name == "dynamics") return auto_aim::Planner::ShootStrategy::Dynamics;
  throw std::invalid_argument("strategy 只能是 rb / sb / hero / dynamics");
}

// 默认 1440x1080/FOV45 下 fx≈1304，4 m 处 5–6 mm 宽的灯条只有约 1.6–2 px。
// 这是角分辨率问题：改仿真相机到 2560x1920（fx≈2318）可以根治（--blur=0）；
// 保留 1440x1080 时，3x3 高斯模糊是给检测器的低成本低通，能把灯条摊成更平滑的亮带。
// 模糊只作用于检测输入，显示和 PnP 仍使用原图（模糊对称，角点中心不移位）。
cv::Mat preprocess_for_detection(const cv::Mat & image, int blur)
{
  if (blur < 3) return image;
  cv::Mat blurred;
  cv::GaussianBlur(image, blurred, cv::Size(blur, blur), 0);
  return blurred;
}

void draw_armors(cv::Mat & image, const std::list<auto_aim::Armor> & armors)
{
  for (const auto & armor : armors) {
    if (armor.points.size() == 4) {
      std::vector<cv::Point> polygon;
      polygon.reserve(4);
      for (const auto & point : armor.points) {
        polygon.emplace_back(cv::Point(cvRound(point.x), cvRound(point.y)));
      }
      cv::polylines(image, std::vector<std::vector<cv::Point>>{polygon}, true, {0, 255, 0}, 2);
    }
    if (armor.box.width > 0 && armor.box.height > 0) {
      cv::rectangle(image, armor.box, {255, 128, 0}, 1);
    }
  }
}

void draw_ideal(cv::Mat & image, const std::vector<std::array<cv::Point2f, 4>> & corners)
{
  for (const auto & quad : corners) {
    std::vector<cv::Point> polygon;
    polygon.reserve(4);
    for (const auto & point : quad) {
      polygon.emplace_back(cv::Point(cvRound(point.x), cvRound(point.y)));
    }
    cv::polylines(image, std::vector<std::vector<cv::Point>>{polygon}, true, {160, 160, 160}, 1);
  }
}

void draw_text(
  cv::Mat & image, const std::string & text, const cv::Point & origin, const cv::Scalar & color,
  double scale = 0.7)
{
  cv::putText(image, text, origin, cv::FONT_HERSHEY_SIMPLEX, scale, {0, 0, 0}, 3, cv::LINE_AA);
  cv::putText(image, text, origin, cv::FONT_HERSHEY_SIMPLEX, scale, color, 1, cv::LINE_AA);
}

int run(const Options & options)
{
  const auto strategy = parse_strategy(options.strategy);

  // 离线图片探针：不连接仿真，只验证当前配置下检测器对单帧的输出。
  if (!options.probe_image.empty()) {
    cv::Mat image = cv::imread(options.probe_image);
    if (image.empty()) {
      tools::logger()->error("无法读取图片 {}", options.probe_image);
      return 1;
    }
    const cv::Mat detect_image = preprocess_for_detection(image, options.blur);
    std::list<auto_aim::Armor> armors;
    if (options.detector == "traditional") {
#ifdef OPENVINO_MAKE
      auto_aim::Detector detector(options.config_path, false);
      armors = detector.detect(detect_image, 0);
#else
      tools::logger()->error("当前构建未编译 traditional 检测器，请使用 --detector=yolo");
      return 1;
#endif
    } else {
      auto_aim::YOLO detector(options.config_path, false);
      armors = detector.detect(detect_image, 0);
    }
    tools::logger()->info(
      "{} 检测结果: {} 块 (blur={})", options.detector, armors.size(), options.blur);
    for (const auto & armor : armors) {
      tools::logger()->info(
        "  color={} name={} type={} conf={:.2f}", auto_aim::COLORS[static_cast<int>(armor.color)],
        auto_aim::ARMOR_NAMES[static_cast<int>(armor.name)],
        auto_aim::ARMOR_TYPES[static_cast<int>(armor.type)], armor.confidence);
    }
    return 0;
  }

  // 读一下配置里的敌方颜色，用来和仿真当前 team 做一致性检查。
  std::string configured_enemy_color;
  try {
    auto yaml = YAML::LoadFile(options.config_path);
    if (yaml["enemy_color"]) configured_enemy_color = yaml["enemy_color"].as<std::string>();
  } catch (const std::exception & e) {
    tools::logger()->warn("读取配置失败（稍后构造算法时还会报错）: {}", e.what());
  }

  RmSimClient client(options.channel, std::chrono::seconds(options.wait_seconds));
  if (!client.wait_for_state(std::chrono::seconds(options.wait_seconds))) {
    tools::logger()->error(
      "在 {} 秒内没有读到新鲜的 state.json。请先启动仿真：cd rm_aim_sim && ./run.sh -- --remote",
      options.wait_seconds);
    return 1;
  }

  {
    const auto state = client.state();
    const auto & appearance = state["appearance"];
    const auto & camera = state["camera"];
    const auto & weapon = state["weapon"];
    const auto & ipc = state["ipc"];
    tools::logger()->info(
      "收到仿真状态: session={} publisher={} sim_time={:.2f} robot={} team={} camera={}x{} "
      "bullet_speed={:.2f} remote={}",
      state.value("session_id", ""), state.value("publisher_id", ""), state.value("sim_time", 0.0),
      appearance.value("robot", "?"), appearance.value("team", "?"), camera.value("width", 0),
      camera.value("height", 0), weapon.value("speed", 0.0), ipc.value("enabled", false));
    if (!ipc.value("enabled", false)) {
      tools::logger()->warn(
        "仿真当前没有开启“本机自瞄控制”。本程序仍可取图/检测/显示，但 command.json 会被拒绝；"
        "请用 ./run.sh -- --remote 启动或在界面勾选。");
    }
    const auto team = appearance.value("team", "");
    if (!configured_enemy_color.empty() && team != configured_enemy_color) {
      if (options.color_filter) {
        tools::logger()->warn(
          "配置 enemy_color={} 与仿真靶车 team={} 不一致，且已开启 --color-filter，"
          "Tracker 会把目标当友军过滤；请修改 {} 或关闭颜色过滤。",
          configured_enemy_color, team, options.config_path);
      } else {
        tools::logger()->info(
          "配置 enemy_color={} 与仿真靶车 team={} 不一致；当前未开启 --color-filter，不影响跟踪。",
          configured_enemy_color, team);
      }
    }
  }

  tools::logger()->info("加载检测/跟踪/规划配置: {}", options.config_path);

  tools::ThreadSafeQueue<SimFrame> frame_queue(2, tools::OverflowPolicy::Block);
  tools::ThreadSafeQueue<Perception> perception_queue(1, tools::OverflowPolicy::DropOldest);

  std::mutex target_mutex;
  TargetSnapshot target_snapshot;

  std::mutex plan_mutex;
  PlanSnapshot plan_snapshot;

  std::mutex error_mutex;
  std::exception_ptr worker_error;

  auto report_error = [&](std::exception_ptr error) {
    {
      std::lock_guard<std::mutex> lock(error_mutex);
      if (!worker_error) worker_error = error;
    }
    g_running.store(false);
    frame_queue.close();
    perception_queue.close();
  };

  // 采图线程：每帧都是“上一帧已复制”后才发下一次请求，因此不会在仿真侧积压图像。
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
              "等待图像帧超时。仿真必须处于图形模式（--headless "
              "没有像素输出），并确认窗口未最小化到无渲染。");
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

  // 感知线程：检测 + PnP/跟踪，只更新目标快照，不阻塞控制线程。
  std::thread perception_thread([&] {
    try {
      // 算法对象在各自线程内构造和使用：OpenVINO/OpenCV DNN 的上下文不跨线程共享。
      auto_aim::Solver solver(options.config_path);
      auto_aim::Tracker tracker(options.config_path, &solver);
      std::function<std::list<auto_aim::Armor>(const cv::Mat &, int)> detect_armors;
      std::unique_ptr<auto_aim::YOLO> yolo;
#ifdef OPENVINO_MAKE
      std::unique_ptr<auto_aim::Detector> traditional;
      if (options.detector == "traditional") {
        traditional = std::make_unique<auto_aim::Detector>(options.config_path, false);
        detect_armors = [&traditional](const cv::Mat & image, int frame_count) {
          return traditional->detect(image, frame_count);
        };
      }
#endif
      if (!detect_armors) {
        yolo = std::make_unique<auto_aim::YOLO>(options.config_path, false);
        detect_armors = [&yolo](const cv::Mat & image, int frame_count) {
          return yolo->detect(image, frame_count);
        };
      }

      std::string last_session;
      std::string last_publisher;
      std::string camera_signature;
      int frame_count = 0;
      tools::fpsSolve fps;
      bool logged_first_target = false;
      bool logged_first_armor = false;

      while (g_running.load()) {
        auto captured = frame_queue.wait_pop_for(std::chrono::milliseconds(100));
        if (!captured) {
          if (frame_queue.closed()) break;
          continue;
        }
        auto & frame = *captured;
        const auto detect_begin = Clock::now();

        if (frame.session_id != last_session || frame.publisher_id != last_publisher) {
          if (!last_session.empty()) {
            tools::logger()->info(
              "仿真 session/publisher 变化，清空跟踪状态（旧目标不会跨会话复用）");
          }
          tracker.reset();
          last_session = frame.session_id;
          last_publisher = frame.publisher_id;
          std::lock_guard<std::mutex> lock(target_mutex);
          target_snapshot = TargetSnapshot{};
        }

        const std::string camera_key =
          fmt::format("{}x{}-{:.4f}", frame.width, frame.height, frame.camera_matrix(0, 0));
        if (!camera_signature.empty() && camera_key != camera_signature) {
          tools::logger()->warn("仿真相机内参/分辨率变化，清空跟踪状态");
          tracker.reset();
        }
        camera_signature = camera_key;

        solver.set_R_gimbal2world_from_tf(frame.q_gimbal2world);
        solver.set_camera_calibration(frame.camera_matrix, frame.distort_coeffs);
        solver.set_camera2gimbal(frame.R_camera2gimbal, frame.t_camera2gimbal);

        const cv::Mat detect_image = preprocess_for_detection(frame.bgr, options.blur);
        auto armors = detect_armors(detect_image, frame_count++);
        const auto detect_end = Clock::now();
        const std::size_t armor_count = armors.size();
        if (!logged_first_armor && !armors.empty()) {
          const auto & armor = armors.front();
          tools::logger()->info(
            "检测样例: color={} name={} type={} confidence={:.2f} 角点数={}",
            auto_aim::COLORS[static_cast<int>(armor.color)],
            auto_aim::ARMOR_NAMES[static_cast<int>(armor.name)],
            auto_aim::ARMOR_TYPES[static_cast<int>(armor.type)], armor.confidence,
            armor.points.size());
          logged_first_armor = true;
        }
        draw_armors(frame.bgr, armors);

        auto targets = tracker.test_track(armors, frame.received_at, true, options.color_filter);
        const auto track_end = Clock::now();

        std::optional<auto_aim::Target> target;
        if (!targets.empty()) target = targets.front();

        {
          std::lock_guard<std::mutex> lock(target_mutex);
          target_snapshot.target = target;
          target_snapshot.sim_time = frame.sim_time;
          target_snapshot.received_at = frame.received_at;
          target_snapshot.session_id = frame.session_id;
        }

        if (target && !logged_first_target) {
          const auto x = target->ekf_x();
          tools::logger()->info(
            "首次锁定目标: x={:.2f} y={:.2f} z={:.2f} m, name={} type={}", x[0], x[2], x[4],
            auto_aim::ARMOR_NAMES[static_cast<int>(target->name)],
            auto_aim::ARMOR_TYPES[static_cast<int>(target->armor_type)]);
          logged_first_target = true;
        }

        const double detect_ms =
          std::chrono::duration<double, std::milli>(detect_end - detect_begin).count();
        const double track_ms =
          std::chrono::duration<double, std::milli>(track_end - detect_end).count();
        const double fps_value = fps.update(track_end);
        if (frame_count % 30 == 0) {
          tools::logger()->info(
            "[perception] frames={} fps={:.1f} yolo={:.1f}ms track={:.1f}ms armors={} "
            "target={} tracker={}",
            frame_count, fps_value, detect_ms, track_ms, armor_count,
            target.has_value() ? "yes" : "no", tracker.state());
        }

        Perception perception;
        perception.image = std::move(frame.bgr);
        perception.target = target;
        perception.sim_time = frame.sim_time;
        perception.gimbal_yaw = frame.gimbal_yaw;
        perception.received_at = frame.received_at;
        perception.session_id = frame.session_id;
        perception.armor_count = armor_count;
        perception.detect_ms = detect_ms;
        perception.track_ms = track_ms;
        perception.tracker_state = tracker.state();
        if (options.show_ideal) draw_ideal(perception.image, frame.ideal_corners);
        perception_queue.push(std::move(perception));
      }
    } catch (...) {
      report_error(std::current_exception());
    }
    frame_queue.close();
    perception_queue.close();
  });

  // 控制线程：固定 100 Hz 读状态 + 规划 + 发指令，绝不等待图像或显示。
  std::thread control_thread([&] {
    try {
      auto_aim::Planner planner(options.config_path);
      tools::PeriodicTimer tick(std::chrono::milliseconds(10));
      std::string control_session;
      Clock::time_point last_state_time{};
      Clock::time_point last_plan_time{};
      Clock::time_point last_log{};
      auto_aim::Plan cached_plan{};
      bool has_cached_plan = false;
      bool sent_control = false;
      int fire_streak = 0;
      bool warned_plan = false;

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
          planner = auto_aim::Planner(options.config_path);
          has_cached_plan = false;
          fire_streak = 0;
          sent_control = false;
          tools::logger()->info("新控制会话 {}：重建 Planner，清空旧计划", session);
        }

        const bool ipc_enabled = state.contains("ipc") && state["ipc"].value("enabled", false);
        const bool paused = state.value("paused", false);
        if (!ipc_enabled || paused) {
          fire_streak = 0;
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

        TargetSnapshot snapshot;
        {
          std::lock_guard<std::mutex> lock(target_mutex);
          snapshot = target_snapshot;
        }
        const bool target_fresh =
          snapshot.target.has_value() && snapshot.session_id == session &&
          seconds_since(snapshot.received_at) <= options.target_timeout_ms / 1000.0;

        auto_aim::Plan plan{};
        if (target_fresh) {
          const double gimbal_yaw = state.contains("gimbal") ? state["gimbal"].value("yaw", 0.0)
                                                             : snapshot.target->ekf_x()[6];
          const double bullet_speed =
            state.contains("weapon") ? state["weapon"].value("speed", 22.0) : 22.0;
          try {
            plan =
              planner.plan(*snapshot.target, bullet_speed, gimbal_yaw * 180.0 / CV_PI, strategy);
          } catch (const std::exception & e) {
            if (!warned_plan) {
              tools::logger()->warn("Planner 异常（后续同类异常不再刷屏）: {}", e.what());
              warned_plan = true;
            }
            plan = auto_aim::Plan{};
          }
        }

        if (plan.control && std::isfinite(plan.yaw) && std::isfinite(plan.pitch)) {
          cached_plan = plan;
          last_plan_time = Clock::now();
          has_cached_plan = true;
        } else if (has_cached_plan && seconds_since(last_plan_time) <= options.hold_ms / 1000.0) {
          plan = cached_plan;
          plan.fire = false;
          plan.control = true;
        } else {
          has_cached_plan = false;
          plan = auto_aim::Plan{};
        }

        if (!plan.control) {
          fire_streak = 0;
          if (sent_control) {
            client.send_command(false, false, 0.0, 0.0);
            sent_control = false;
          }
        } else {
          if (plan.fire && options.fire) {
            ++fire_streak;
          } else {
            fire_streak = 0;
          }
          // 仿真的实际云台带伺服滞后；只按规划角开火会在云台还没跟上时把弹打偏。
          // 用实际 yaw/pitch 与下发目标做对齐门限，和实车火控的容差思路一致。
          const double actual_yaw =
            state.contains("gimbal") ? state["gimbal"].value("yaw", 0.0) : 0.0;
          const double actual_pitch =
            state.contains("gimbal") ? state["gimbal"].value("pitch", 0.0) : 0.0;
          const double yaw_err_deg =
            std::abs(tools::limit_rad(actual_yaw - plan.yaw)) * 180.0 / CV_PI;
          const double pitch_err_deg = std::abs(actual_pitch + plan.pitch) * 180.0 / CV_PI;
          const bool aligned =
            yaw_err_deg <= options.fire_tolerance && pitch_err_deg <= options.fire_tolerance;
          const bool fire =
            options.fire && plan.control && plan.fire && fire_streak >= 2 && aligned;

          // 仿真协议 pitch 向下为正；Planner 的弹道 pitch 抬头为正，必须取反。
          const double yaw = plan.yaw;
          const double pitch_sim = -plan.pitch;
          if (client.send_command(
                true, fire, yaw, pitch_sim, plan.yaw_vel, -plan.pitch_vel, plan.yaw_acc,
                -plan.pitch_acc)) {
            sent_control = true;
            if (seconds_since(last_log) >= 1.0) {
              last_log = Clock::now();
              const auto & stats = state["stats"];
              std::string target_desc = "none";
              if (target_fresh) {
                const auto x = snapshot.target->ekf_x();
                target_desc = fmt::format(
                  "({:.2f},{:.2f},{:.2f}) yaw={:.1f}° vyaw={:.1f}°/s r={:.3f} alt_r={:+.3f} "
                  "alt_h={:+.3f} ref={:+.2f}° aim_id={} hz_aim=({:.2f},{:.2f},{:.2f})",
                  x[0], x[2], x[4], x[6] * 180.0 / CV_PI, x[7] * 180.0 / CV_PI,
                  x.size() > 8 ? x[8] : 0.0, x.size() > 9 ? x[9] : 0.0,
                  x.size() > 10 ? x[10] : 0.0, -plan.target_pitch * 180.0 / CV_PI,
                  planner.aim_armor_id(), planner.debug_xyza[0], planner.debug_xyza[1],
                  planner.debug_xyza[2]);
              }
              tools::logger()->info(
                "[control] yaw={:+.2f}° pitch={:+.2f}°(下发,下正) fire={} align=({:.2f},{:.2f})° "
                "target_age={:.0f}ms target={} truth_angle={:+.1f}° | sim_t={:.1f}s fired={} "
                "hit={} rate={:.1f}% seq={}",
                yaw * 180.0 / CV_PI, pitch_sim * 180.0 / CV_PI, fire, yaw_err_deg, pitch_err_deg,
                seconds_since(snapshot.received_at) * 1000.0, target_desc,
                state.contains("truth") ? state["truth"].value("angle", 0.0) * 180.0 / CV_PI : 0.0,
                state.value("sim_time", 0.0), stats.value("fired", 0), stats.value("hit", 0),
                stats.value("hit_rate", 0.0) * 100.0,
                state.contains("ipc") ? state["ipc"].value("last_seq", -1) : -1);
            }
          }
        }

        {
          std::lock_guard<std::mutex> lock(plan_mutex);
          plan_snapshot.plan = plan;
          plan_snapshot.valid = plan.control;
          plan_snapshot.fire = plan.control && plan.fire && options.fire;
          plan_snapshot.yaw = plan.yaw;
          plan_snapshot.pitch_sim = -plan.pitch;
          plan_snapshot.time = Clock::now();
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
    auto perception = perception_queue.wait_pop_for(std::chrono::milliseconds(100));
    if (!perception) continue;
    if (!display || perception->image.empty()) continue;

    PlanSnapshot plan;
    {
      std::lock_guard<std::mutex> lock(plan_mutex);
      plan = plan_snapshot;
    }
    draw_text(
      perception->image,
      fmt::format(
        "det {:+.2f}°  plan yaw {:+.2f}° pitch {:+.2f}° fire {}",
        perception->gimbal_yaw * 180.0 / CV_PI, plan.yaw * 180.0 / CV_PI,
        plan.pitch_sim * 180.0 / CV_PI, plan.fire ? "ON" : "off"),
      {20, 40}, {0, 255, 255});
    draw_text(
      perception->image,
      fmt::format(
        "armors {}  target {}  tracker {}  yolo {:.1f}ms  track {:.1f}ms", perception->armor_count,
        perception->target ? "YES" : "no", perception->tracker_state, perception->detect_ms,
        perception->track_ms),
      {20, 80}, {0, 255, 0});

    cv::Mat show;
    cv::resize(perception->image, show, {}, 0.5, 0.5);
    cv::imshow("rm_aim_sim_client", show);
    const int key = cv::waitKey(1);
    if (key == 'q' || key == 27) {
      g_running.store(false);
      break;
    }
  }

  g_running.store(false);
  frame_queue.close();
  perception_queue.close();
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
        "结束汇总: sim_t={:.1f}s fired={} resolved={} hit={} ground={} expired={} "
        "hit_rate={:.1f}% damage={} kills={}",
        state.value("sim_time", 0.0), stats.value("fired", 0), stats.value("resolved", 0),
        stats.value("hit", 0), stats.value("ground", 0), stats.value("expired", 0),
        stats.value("hit_rate", 0.0) * 100.0, stats.value("damage", 0), stats.value("kills", 0));
    }
  } catch (...) {
  }
  return 0;
}

}  // namespace

// 控制台只看 info 及以上；文件日志仍保留 debug。Planner/Tracker 的逐帧 debug 不刷屏。
void configure_console_log_level()
{
  auto logger = tools::logger();
  for (auto & sink : logger->sinks()) {
    if (std::dynamic_pointer_cast<spdlog::sinks::stdout_color_sink_mt>(sink)) {
      sink->set_level(spdlog::level::info);
    }
  }
}

int main(int argc, char * argv[])
{
  const std::string keys =
    "{help h usage ? |                              | 输出命令行参数说明}"
    "{@config-path   | ../configs/rm_aim_sim.yaml    | YAML 配置路径}"
    "{channel        | default                      | 本机 IPC 通道名（与仿真 [ipc].channel 一致）}"
    "{fire f         | false                        | 允许开火；默认只瞄准不开火}"
    "{no-display     | false                        | 不打开调试窗口}"
    "{show-ideal     | false                        | 叠加仿真理想角点（仅调试）}"
    "{duration       | 0                            | 运行时长（秒），0 表示一直运行}"
    "{strategy       | rb                           | 云台策略 rb / sb / hero / dynamics}"
    "{detector       | traditional                  | 检测器 traditional（仿真推荐）/ yolo}"
    "{color-filter   | false                        | 是否按 enemy_color 过滤装甲（仿真默认关闭）}"
    "{probe-image    |                              | 只对单张图片跑检测后退出（离线调试）}"
    "{blur           | 3                            | 检测前高斯模糊核，奇数；0=关闭（高分辨率建议 0）}"
    "{fire-tolerance | 1.0                          | 实际云台与规划角的开火对齐容差（度）}"
    "{hold-ms        | 80                           | 丢失目标后保持最后瞄准角的时间（ms）}"
    "{target-timeout-ms | 150                       | 目标过期时间（ms）}"
    "{wait           | 8                            | 等待仿真启动的超时（秒）}";

  cv::CommandLineParser cli(argc, argv, keys);
  if (cli.has("help")) {
    cli.printMessage();
    return 0;
  }
  if (!cli.check()) {
    cli.printErrors();
    return 2;
  }

  Options options;
  options.config_path = cli.get<std::string>(0);
  options.channel = cli.get<std::string>("channel");
  options.fire = cli.get<bool>("fire");
  options.display = !cli.get<bool>("no-display");
  options.show_ideal = cli.get<bool>("show-ideal");
  options.duration = cli.get<double>("duration");
  options.strategy = cli.get<std::string>("strategy");
  options.detector = cli.get<std::string>("detector");
  options.color_filter = cli.get<bool>("color-filter");
  options.probe_image = cli.get<std::string>("probe-image");
  options.blur = cli.get<int>("blur");
  options.fire_tolerance = cli.get<double>("fire-tolerance");
  if (options.fire_tolerance < 0.0) {
    tools::logger()->error("fire-tolerance 不能为负");
    return 2;
  }
  if (options.blur != 0 && (options.blur < 3 || options.blur % 2 == 0)) {
    tools::logger()->error("blur 必须是 0 或 >=3 的奇数");
    return 2;
  }
  options.hold_ms = cli.get<int>("hold-ms");
  options.target_timeout_ms = cli.get<int>("target-timeout-ms");
  options.wait_seconds = cli.get<int>("wait");
  if (options.detector != "yolo" && options.detector != "traditional") {
    tools::logger()->error("detector 只能是 yolo 或 traditional");
    return 2;
  }
#ifndef OPENVINO_MAKE
  if (options.detector == "traditional") {
    tools::logger()->error(
      "当前构建未包含 traditional 检测器（需要 OpenVINO 后端），请加 --detector=yolo");
    return 2;
  }
#endif

  std::signal(SIGINT, on_signal);
  std::signal(SIGTERM, on_signal);
  configure_console_log_level();

  try {
    tools::logger()->info(
      "rm_aim_sim_client: config={} channel={} fire={} display={} strategy={} detector={} blur={}",
      options.config_path, options.channel, options.fire, options.display, options.strategy,
      options.detector, options.blur);
    return run(options);
  } catch (const std::exception & e) {
    tools::logger()->error("rm_aim_sim_client 退出: {}", e.what());
    return 1;
  }
}
