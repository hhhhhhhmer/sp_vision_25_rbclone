#include <fmt/core.h>

#include <Eigen/Geometry>
#include <algorithm>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <cstdlib>
#include <deque>
#include <exception>
#include <limits>
#include <memory>
#include <mutex>
#include <nlohmann/json.hpp>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include <opencv2/opencv.hpp>
#include <geometry_msgs/msg/transform.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <tf2_msgs/msg/tf_message.hpp>

#include "tasks/auto_aim/planner/planner.hpp"
#include "tasks/auto_aim/solver.hpp"
#include "tasks/auto_aim/tracker.hpp"
#include "tasks/auto_aim/yolo.hpp"
#include "tools/fft.hpp"
#include "tools/fps_solve.hpp"
#include "tools/img_tools.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/plotter.hpp"
#include "tools/reprojection.hpp"

using namespace std::chrono_literals;

namespace
{

constexpr uint32_t kImageWidth = 1440;
constexpr uint32_t kImageHeight = 1080;
constexpr uint32_t kImageStep = kImageWidth * 3;
constexpr std::size_t kImageDataSize =
  static_cast<std::size_t>(kImageStep) * kImageHeight;

const std::string kKeys =
  "{help h usage ?    |                                       | 输出命令行参数说明}"
  "{@config-path      | ../configs/rb_auto_aim_simulator.yaml | yaml配置文件路径}"
  "{image-topic       | /image_raw                            | sensor_msgs/Image话题}"
  "{camera-info-topic | /camera_info                          | sensor_msgs/CameraInfo话题}"
  "{tf-topic          | /tf                                   | tf2_msgs/TFMessage话题}"
  "{bullet-speed      | 22.0                                  | 模拟弹速(m/s)}";

int64_t stamp_ns(const builtin_interfaces::msg::Time & stamp)
{
  return static_cast<int64_t>(stamp.sec) * 1000000000LL + stamp.nanosec;
}

void log_ros_environment()
{
  const char * rmw = std::getenv("RMW_IMPLEMENTATION");
  const char * domain = std::getenv("ROS_DOMAIN_ID");
  const char * transports = std::getenv("FASTDDS_BUILTIN_TRANSPORTS");
  tools::logger()->info(
    "ROS 2环境: RMW_IMPLEMENTATION={}, ROS_DOMAIN_ID={}, FASTDDS_BUILTIN_TRANSPORTS={}",
    rmw ? rmw : "<unset>", domain ? domain : "<unset>", transports ? transports : "<unset>");

  if (rmw && std::string(rmw) != "rmw_fastrtps_cpp") {
    tools::logger()->warn(
      "RMW_IMPLEMENTATION={}，与发送端要求的rmw_fastrtps_cpp不一致", rmw);
  }
  if (domain && std::string(domain) != "0") {
    tools::logger()->warn("ROS_DOMAIN_ID={}，与发送端要求的0不一致", domain);
  }
  if (transports && std::string(transports) != "SHM") {
    tools::logger()->warn(
      "FASTDDS_BUILTIN_TRANSPORTS={}，同机共享内存模式应设为SHM", transports);
  }
}

struct SimulatorFrame
{
  cv::Mat image;
  sensor_msgs::msg::Image::ConstSharedPtr image_owner;
  Eigen::Quaterniond q_gimbal2world;
  Eigen::Matrix3d camera_matrix;
  std::vector<double> distort_coeffs;
  Eigen::Isometry3d T_camera2gimbal;
  std::chrono::steady_clock::time_point received_at;
  double source_fps = 0.0;
  int64_t stamp = 0;
};

struct CapturedFrame
{
  SimulatorFrame frame;
  double image_wait_ms = 0.0;
};

struct PerceptionFrame
{
  SimulatorFrame frame;
  std::optional<auto_aim::Target> target;
  Eigen::Vector3d ypr = Eigen::Vector3d::Zero();
  double image_wait_ms = 0.0;
  double perception_ms = 0.0;
};

template <typename T>
class BoundedLatestQueue
{
public:
  explicit BoundedLatestQueue(std::size_t capacity)
  : capacity_(std::max<std::size_t>(1, capacity))
  {
  }

  bool push(T value)
  {
    {
      std::lock_guard lock(mutex_);
      if (closed_) return false;
      if (queue_.size() >= capacity_) {
        queue_.pop_front();
        ++dropped_;
      }
      queue_.push_back(std::move(value));
    }
    not_empty_.notify_one();
    return true;
  }

  std::optional<T> pop()
  {
    std::unique_lock lock(mutex_);
    not_empty_.wait(lock, [this] { return closed_ || !queue_.empty(); });
    if (queue_.empty()) return std::nullopt;

    T value = std::move(queue_.front());
    queue_.pop_front();
    return value;
  }

  void close()
  {
    {
      std::lock_guard lock(mutex_);
      closed_ = true;
    }
    not_empty_.notify_all();
  }

  std::size_t dropped() const
  {
    std::lock_guard lock(mutex_);
    return dropped_;
  }

private:
  const std::size_t capacity_;
  mutable std::mutex mutex_;
  std::condition_variable not_empty_;
  std::deque<T> queue_;
  bool closed_ = false;
  std::size_t dropped_ = 0;
};

struct StampedTransformState
{
  int64_t stamp;
  Eigen::Quaterniond q_gimbal2world;
  Eigen::Isometry3d T_camera2gimbal;
};

struct StampedCameraInfo
{
  int64_t stamp;
  Eigen::Matrix3d camera_matrix;
  std::vector<double> distort_coeffs;
};

class SimulatorInput : public rclcpp::Node
{
public:
  SimulatorInput(
    const std::string & image_topic, const std::string & camera_info_topic,
    const std::string & tf_topic)
  : Node("rb_auto_aim_simulator"), image_topic_(image_topic)
  {
    image_callback_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    metadata_callback_group_ =
      create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    rclcpp::SubscriptionOptions image_options;
    image_options.callback_group = image_callback_group_;
    const auto image_qos =
      rclcpp::SensorDataQoS().keep_last(1).best_effort().durability_volatile();
    image_sub_ = create_subscription<sensor_msgs::msg::Image>(
      image_topic, image_qos,
      [this](sensor_msgs::msg::Image::ConstSharedPtr msg) { on_image(std::move(msg)); },
      image_options);

    rclcpp::SubscriptionOptions metadata_options;
    metadata_options.callback_group = metadata_callback_group_;
    auto camera_info_qos = rclcpp::SensorDataQoS();
    camera_info_qos.keep_last(1);
    camera_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
      camera_info_topic, camera_info_qos,
      [this](sensor_msgs::msg::CameraInfo::ConstSharedPtr msg) { on_camera_info(*msg); },
      metadata_options);
    tf_sub_ = create_subscription<tf2_msgs::msg::TFMessage>(
      tf_topic, rclcpp::QoS(30),
      [this](tf2_msgs::msg::TFMessage::ConstSharedPtr msg) { on_tf(*msg); }, metadata_options);
  }

  std::optional<SimulatorFrame> wait_for_frame(std::chrono::milliseconds timeout)
  {
    std::unique_lock lock(mutex_);
    frame_ready_.wait_for(lock, timeout, [this] {
      return pending_image_.has_value() && !transform_states_.empty() && !camera_infos_.empty();
    });
    if (!pending_image_ || transform_states_.empty() || camera_infos_.empty()) {
      const auto now = std::chrono::steady_clock::now();
      if (now - last_wait_warning_ >= 2s) {
        last_wait_warning_ = now;
        const auto received_count = image_message_count_;
        const bool has_pending_image = pending_image_.has_value();
        const bool has_camera_info = !camera_infos_.empty();
        const bool has_tf = !transform_states_.empty();
        lock.unlock();
        RCLCPP_WARN(
          get_logger(),
          "输入尚未就绪: image_topic=%s publishers=%zu received=%zu latest_image=%s "
          "camera_info=%s tf_chain=%s",
          image_topic_.c_str(), image_sub_->get_publisher_count(), received_count,
          has_pending_image ? "yes" : "no", has_camera_info ? "yes" : "no",
          has_tf ? "yes" : "no");
      }
      return std::nullopt;
    }

    auto pending_image = std::move(*pending_image_);
    pending_image_.reset();

    auto nearest_transform = transform_states_.begin();
    auto transform_delta = std::numeric_limits<int64_t>::max();
    for (auto it = transform_states_.begin(); it != transform_states_.end(); ++it) {
      const auto delta = std::llabs(it->stamp - pending_image.stamp);
      if (delta < transform_delta) {
        nearest_transform = it;
        transform_delta = delta;
      }
    }

    auto nearest_camera_info = camera_infos_.begin();
    auto camera_info_delta = std::numeric_limits<int64_t>::max();
    for (auto it = camera_infos_.begin(); it != camera_infos_.end(); ++it) {
      const auto delta = std::llabs(it->stamp - pending_image.stamp);
      if (delta < camera_info_delta) {
        nearest_camera_info = it;
        camera_info_delta = delta;
      }
    }

    if (transform_delta > 100000000LL && !warned_tf_delay_) {
      tools::logger()->warn(
        "图像与TF标定时间差为 {:.1f} ms，将使用最近数据", transform_delta / 1e6);
      warned_tf_delay_ = true;
    }

    const auto transform = *nearest_transform;
    const auto camera_info = *nearest_camera_info;
    lock.unlock();

    // to_bgr reads through a shared RGB view; the returned BGR matrix owns its converted buffer.
    auto image = to_bgr(*pending_image.message);
    return SimulatorFrame{
      std::move(image),
      std::move(pending_image.message),
      transform.q_gimbal2world,
      camera_info.camera_matrix,
      camera_info.distort_coeffs,
      transform.T_camera2gimbal,
      pending_image.received_at,
      pending_image.source_fps,
      pending_image.stamp};
  }

private:
  struct PendingImage
  {
    sensor_msgs::msg::Image::ConstSharedPtr message;
    std::chrono::steady_clock::time_point received_at;
    double source_fps;
    int64_t stamp;
  };

  static cv::Mat to_bgr(const sensor_msgs::msg::Image & msg)
  {
    const cv::Mat rgb(
      static_cast<int>(msg.height), static_cast<int>(msg.width), CV_8UC3,
      const_cast<unsigned char *>(msg.data.data()), msg.step);
    cv::Mat bgr;
    cv::cvtColor(rgb, bgr, cv::COLOR_RGB2BGR);
    return bgr;
  }

  static std::optional<Eigen::Isometry3d> to_isometry(
    const geometry_msgs::msg::Transform & transform)
  {
    const auto & rotation = transform.rotation;
    Eigen::Quaterniond q(rotation.w, rotation.x, rotation.y, rotation.z);
    if (!q.coeffs().allFinite() || q.norm() < 1e-9) return std::nullopt;

    Eigen::Vector3d translation(
      transform.translation.x, transform.translation.y, transform.translation.z);
    if (!translation.allFinite()) return std::nullopt;

    Eigen::Isometry3d result = Eigen::Isometry3d::Identity();
    result.linear() = q.normalized().toRotationMatrix();
    result.translation() = translation;
    return result;
  }

  void on_image(sensor_msgs::msg::Image::ConstSharedPtr msg)
  {
    try {
      if (
        msg->width != kImageWidth || msg->height != kImageHeight || msg->encoding != "rgb8" ||
        msg->step != kImageStep || msg->data.size() < kImageDataSize) {
        throw std::runtime_error(fmt::format(
          "无效图像消息: {}x{}, encoding={}, step={}, data={} (期望 {}x{}, rgb8, {}, {})",
          msg->width, msg->height, msg->encoding, msg->step, msg->data.size(), kImageWidth,
          kImageHeight, kImageStep, kImageDataSize));
      }

      const auto received_at = std::chrono::steady_clock::now();
      const auto image_stamp = stamp_ns(msg->header.stamp);
      bool first_image = false;
      {
        std::lock_guard lock(mutex_);
        if (last_image_received_at_) {
          const auto interval =
            std::chrono::duration<double>(received_at - *last_image_received_at_).count();
          if (interval > 0.0) {
            source_period_seconds_ = source_period_seconds_ == 0.0
                                       ? interval
                                       : 0.9 * source_period_seconds_ + 0.1 * interval;
            source_fps_ = 1.0 / source_period_seconds_;
          }
        }
        last_image_received_at_ = received_at;
        first_image = image_message_count_ == 0;
        ++image_message_count_;
        pending_image_ = PendingImage{
          std::move(msg), received_at, source_fps_, image_stamp};
      }
      frame_ready_.notify_one();
      if (first_image) {
        tools::logger()->info(
          "已收到首帧原始图像: topic={}, size={}x{}, encoding=rgb8, step={}", image_topic_,
          kImageWidth, kImageHeight, kImageStep);
      }
    } catch (const std::exception & e) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000, "%s", e.what());
    }
  }

  void on_camera_info(const sensor_msgs::msg::CameraInfo & msg)
  {
    Eigen::Matrix3d camera_matrix;
    camera_matrix << msg.k[0], msg.k[1], msg.k[2], msg.k[3], msg.k[4], msg.k[5], msg.k[6],
      msg.k[7], msg.k[8];
    if (!camera_matrix.allFinite() || camera_matrix(0, 0) <= 0 || camera_matrix(1, 1) <= 0) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000, "收到无效CameraInfo");
      return;
    }

    {
      std::lock_guard lock(mutex_);
      camera_infos_.push_back({stamp_ns(msg.header.stamp), camera_matrix, msg.d});
      while (camera_infos_.size() > 60) camera_infos_.pop_front();
    }
    frame_ready_.notify_one();

    if (!camera_info_received_.exchange(true)) {
      tools::logger()->info(
        "动态相机内参: {}x{}, fx={:.3f}, fy={:.3f}, cx={:.3f}, cy={:.3f}", msg.width,
        msg.height, msg.k[0], msg.k[4], msg.k[2], msg.k[5]);
    }
  }

  void on_tf(const tf2_msgs::msg::TFMessage & msg)
  {
    std::lock_guard lock(mutex_);
    std::optional<Eigen::Isometry3d> T_gimbal2world;
    std::optional<Eigen::Isometry3d> T_camera_link2gimbal;
    std::optional<Eigen::Isometry3d> T_camera_optical2camera_link;
    int64_t tf_stamp = 0;

    for (const auto & transform : msg.transforms) {
      const auto value = to_isometry(transform.transform);
      if (!value) continue;

      if (transform.header.frame_id == "odom" && transform.child_frame_id == "gimbal_link") {
        T_gimbal2world = *value;
        tf_stamp = stamp_ns(transform.header.stamp);
      } else if (
        transform.header.frame_id == "gimbal_link" &&
        transform.child_frame_id == "camera_link") {
        T_camera_link2gimbal = *value;
      } else if (
        transform.header.frame_id == "camera_link" &&
        transform.child_frame_id == "camera_optical_frame") {
        T_camera_optical2camera_link = *value;
      }
    }

    if (T_gimbal2world && T_camera_link2gimbal && T_camera_optical2camera_link) {
      const Eigen::Isometry3d T_camera2gimbal =
        *T_camera_link2gimbal * *T_camera_optical2camera_link;
      transform_states_.push_back(
        {tf_stamp, Eigen::Quaterniond(T_gimbal2world->linear()), T_camera2gimbal});
      while (transform_states_.size() > 300) transform_states_.pop_front();

      if (!handeye_received_.exchange(true)) {
        const auto & t = T_camera2gimbal.translation();
        tools::logger()->info(
          "动态手眼外参已接入: t_camera2gimbal=[{:.4f}, {:.4f}, {:.4f}] m", t.x(), t.y(),
          t.z());
      }
    }
    frame_ready_.notify_one();
  }

  std::mutex mutex_;
  std::condition_variable frame_ready_;
  std::optional<PendingImage> pending_image_;
  std::deque<StampedTransformState> transform_states_;
  std::deque<StampedCameraInfo> camera_infos_;
  std::atomic<bool> camera_info_received_ = false;
  std::atomic<bool> handeye_received_ = false;
  bool warned_tf_delay_ = false;
  std::chrono::steady_clock::time_point last_wait_warning_{};
  std::optional<std::chrono::steady_clock::time_point> last_image_received_at_;
  std::size_t image_message_count_ = 0;
  double source_period_seconds_ = 0.0;
  double source_fps_ = 0.0;
  std::string image_topic_;

  rclcpp::CallbackGroup::SharedPtr image_callback_group_;
  rclcpp::CallbackGroup::SharedPtr metadata_callback_group_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr tf_sub_;
};

void plot_target_debug(
  tools::Plotter & plotter, tools::FFTExample & fft, auto_aim::Target & target,
  const auto_aim::Plan & plan, const Eigen::Vector3d & ypr,
  std::chrono::steady_clock::time_point plot_start, double fps, double mean_fps,
  double latency_ms, double perception_ms, double planner_ms)
{
  nlohmann::json data;
  data["t"] =
    std::chrono::duration<double>(std::chrono::steady_clock::now() - plot_start).count();
  data["gimbal_yaw"] = ypr[0];
  data["gimbal_pitch"] = ypr[1];
  data["plan_mode"] = plan.control ? (plan.fire ? 2 : 1) : 0;
  data["plan_yaw"] = plan.yaw / CV_PI * 180.0;
  data["plan_yaw_vel"] = plan.yaw_vel;
  data["plan_yaw_acc"] = plan.yaw_acc;
  data["plan_pitch"] = plan.pitch / CV_PI * 180.0;
  data["plan_pitch_vel"] = plan.pitch_vel;
  data["plan_pitch_acc"] = plan.pitch_acc;
  data["fire"] = plan.fire ? 1 : 0;
  data["target_yaw"] = plan.target_yaw;
  data["target_pitch"] = plan.target_pitch;
  data["target_z"] = target.ekf_x()[4];
  data["target_vz"] = target.ekf_x()[5];
  data["tower_h1"] = target.tower_armor_hs[0].second;
  data["tower_h2"] = target.tower_armor_hs[1].second;
  data["tower_h3"] = target.tower_armor_hs[2].second;
  data["tower_armor_h"] = target.tower_armor_h;

  const auto ekf = target.ekf_x();
  data["ekf_x"] = ekf[0];
  data["ekf_vx"] = ekf[1];
  data["ekf_y"] = ekf[2];
  data["ekf_vy"] = ekf[3];
  data["ekf_z"] = ekf[4];
  data["ekf_vz"] = ekf[5];
  data["ekf_yaw"] = ekf[6] / CV_PI * 180.0;
  data["ekf_vyaw"] = ekf[7] / CV_PI * 180.0;
  data["ekf_r"] = ekf[8];

  if(target.rv_residual.has_value()) {
    data["rv_residual_x"] = target.rv_residual->x();
    data["rv_residual_y"] = target.rv_residual->y();
    data["rv_residual_z"] = target.rv_residual->z();
    data["rv_residual_yaw"] = target.rv_residual->w();
  }

  const bool is_periodic = fft.get_is_periodic();
  data["fft_periodic"] = is_periodic ? 1 : 0;
  data["fft_input_z"] = fft.get_latest_value();
  // data["fft_frequency"] = fft.get_frequency();
  // data["fft_amplitude"] = fft.get_amplitude();
  // data["fft_fit_quality"] = fft.get_fit_quality();
  // data["fft_snr"] = fft.get_signal_to_noise_ratio();
  if (is_periodic) {
    data["fft_value"] = fft.get_value(target.getTimePoint());
    data["target_xyz_in_world_z"] = target.xyz_in_world.z();
    data["fft_original_value"] = fft.get_latest_value();
  }
  data["fps"] = fps;
  data["mean_fps"] = mean_fps;
  data["latency_ms"] = latency_ms;
  data["perception_ms"] = perception_ms;
  data["planner_ms"] = planner_ms;
  data["plan_thread_dt_s"] = planner_ms;
  plotter.plot(data);
}

}  // namespace

int main(int argc, char * argv[])
{
  cv::CommandLineParser cli(argc, argv, kKeys);
  if (cli.has("help")) {
    cli.printMessage();
    return 0;
  }

  const auto config_path = cli.get<std::string>(0);
  const auto image_topic = cli.get<std::string>("image-topic");
  const auto camera_info_topic = cli.get<std::string>("camera-info-topic");
  const auto tf_topic = cli.get<std::string>("tf-topic");
  const auto bullet_speed = cli.get<double>("bullet-speed");
  if (!cli.check()) {
    cli.printErrors();
    return 2;
  }

  log_ros_environment();
  rclcpp::init(0, nullptr);
  auto input = std::make_shared<SimulatorInput>(image_topic, camera_info_topic, tf_topic);
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(input);
  std::thread spin_thread([&executor] { executor.spin(); });

  int result = 0;
  try {
    BoundedLatestQueue<CapturedFrame> capture_queue(2);
    BoundedLatestQueue<PerceptionFrame> perception_queue(2);
    std::atomic<bool> stop_pipeline = false;
    std::mutex worker_error_mutex;
    std::exception_ptr worker_error;

    auto stop_workers = [&] {
      stop_pipeline = true;
      capture_queue.close();
      perception_queue.close();
      executor.cancel();
    };
    auto report_worker_error = [&](std::exception_ptr error) {
      {
        std::lock_guard lock(worker_error_mutex);
        if (!worker_error) worker_error = error;
      }
      stop_workers();
    };

    auto_aim::Planner planner(config_path);
    auto_aim::Solver display_solver(config_path);
    tools::FFTExample fft;
    tools::Plotter plotter;
    tools::fpsSolve fps_solver;
    const auto plot_start = std::chrono::steady_clock::now();

    std::thread capture_thread;
    std::thread perception_thread;
    std::thread fft_thread;
    try {
      capture_thread = std::thread([&] {
        try {
          while (!stop_pipeline && rclcpp::ok()) {
            const auto wait_begin = std::chrono::steady_clock::now();
            auto frame = input->wait_for_frame(200ms);
            const auto wait_end = std::chrono::steady_clock::now();
            if (!frame) continue;

            const double image_wait_ms =
              std::chrono::duration<double, std::milli>(wait_end - wait_begin).count();
            if (!capture_queue.push({std::move(*frame), image_wait_ms})) break;
          }
          capture_queue.close();
        } catch (...) {
          report_worker_error(std::current_exception());
        }
      });

      perception_thread = std::thread([&] {
        try {
          // HighGUI remains in the planner/main thread, so detector debug must stay disabled here.
          auto_aim::YOLO detector(config_path, false);
          auto_aim::Solver solver(config_path);
          auto_aim::Tracker tracker(config_path, &solver);
          tracker.set_fft(&fft);
          int frame_count = 0;

          while (!stop_pipeline) {
            auto captured = capture_queue.pop();
            if (!captured) break;

            const auto perception_begin = std::chrono::steady_clock::now();
            auto & frame = captured->frame;
            solver.set_R_gimbal2world_from_tf(frame.q_gimbal2world);
            solver.set_camera_calibration(frame.camera_matrix, frame.distort_coeffs);
            solver.set_camera2gimbal(
              frame.T_camera2gimbal.linear(), frame.T_camera2gimbal.translation());
            auto armors = detector.detect(frame.image, frame_count++);
            auto targets = tracker.test_track(armors, frame.received_at);

            auto ypr = tools::eulers(frame.q_gimbal2world, 2, 1, 0);
            // ROS uses a right-handed +Y pitch; vision treats muzzle-up as positive.
            ypr[1] = -ypr[1];
            std::optional<auto_aim::Target> target;
            if (!targets.empty()) target = targets.front();

            const auto perception_end = std::chrono::steady_clock::now();
            const double perception_ms =
              std::chrono::duration<double, std::milli>(
                perception_end - perception_begin).count();
            if (!perception_queue.push(
                  {std::move(frame), std::move(target), ypr, captured->image_wait_ms,
                   perception_ms})) {
              break;
            }
          }
          perception_queue.close();
        } catch (...) {
          report_worker_error(std::current_exception());
        }
      });

      fft_thread = std::thread([&] {
        try {
          bool was_periodic = false;
          while (!stop_pipeline) {
            const auto analysis_start = std::chrono::steady_clock::now();
            const bool is_periodic = fft.analyze();
            if (is_periodic != was_periodic) {
              const double elapsed_ms = std::chrono::duration<double, std::milli>(
                                          std::chrono::steady_clock::now() - analysis_start)
                                          .count();
              if (is_periodic) {
                tools::logger()->info("[FFT] 检测到周期运动，分析耗时 {:.2f} ms", elapsed_ms);
              } else {
                tools::logger()->info("[FFT] 周期运动已消失");
              }
              was_periodic = is_periodic;
            }
            for (int i = 0; i < 5 && !stop_pipeline; ++i) std::this_thread::sleep_for(50ms);
          }
        } catch (...) {
          report_worker_error(std::current_exception());
        }
      });
    } catch (...) {
      stop_workers();
      if (capture_thread.joinable()) capture_thread.join();
      if (perception_thread.joinable()) perception_thread.join();
      if (fft_thread.joinable()) fft_thread.join();
      throw;
    }

    tools::logger()->info(
      "ROS流水线已启动: image_wait -> perception/FFT -> planner/display/plotter; image={}, tf={}",
      image_topic, tf_topic);

    std::exception_ptr main_error;
    try {
      while (!stop_pipeline && rclcpp::ok()) {
        auto perception = perception_queue.pop();
        if (!perception) break;

        auto & frame = perception->frame;
        cv::Mat display_image = frame.image.clone();
        display_solver.set_R_gimbal2world_from_tf(frame.q_gimbal2world);
        display_solver.set_camera_calibration(frame.camera_matrix, frame.distort_coeffs);
        display_solver.set_camera2gimbal(
          frame.T_camera2gimbal.linear(), frame.T_camera2gimbal.translation());

        const auto planner_begin = std::chrono::steady_clock::now();
        const auto plan = planner.plan(
          perception->target, bullet_speed, perception->ypr[0],
          auto_aim::Planner::ShootStrategy::rbSuppressiveFire);
        const auto planner_end = std::chrono::steady_clock::now();
        const double planner_ms =
          std::chrono::duration<double, std::milli>(planner_end - planner_begin).count();

        if (perception->target) {
          const std::optional<Eigen::Vector4d> aim_xyza =
            plan.control ? std::make_optional(planner.debug_xyza) : std::nullopt;
          tools::draw_reprojection(
            display_image, display_solver, *perception->target, aim_xyza,
            cv::Scalar(235, 206, 135));
        }

        const auto now = std::chrono::steady_clock::now();
        const double fps = fps_solver.update(now);
        const double mean_fps = fps_solver.get_mean_fps();
        const double latency_ms =
          std::chrono::duration<double, std::milli>(now - frame.received_at).count();

        if (perception->target) {
          plot_target_debug(
            plotter, fft, *perception->target, plan, perception->ypr, plot_start, fps, mean_fps,
            latency_ms, perception->perception_ms, planner_ms);
        }

        tools::draw_text(
          display_image,
          fmt::format("FPS {:.1f} mean {:.1f} latency {:.1f} ms", fps, mean_fps, latency_ms),
          {40, 40}, {0, 255, 0});
        tools::draw_text(
          display_image,
          fmt::format(
            "Input {:.1f} FPS Wait {:.1f} Detect/Track {:.1f} Plan {:.1f} ms Drop {}/{}",
            frame.source_fps, perception->image_wait_ms, perception->perception_ms, planner_ms,
            capture_queue.dropped(), perception_queue.dropped()),
          {40, 80}, {255, 255, 255});
        tools::draw_text(
          display_image, fmt::format("Yaw {:.2f}", perception->ypr[0] * 180.0 / CV_PI),
          {40, 120}, {0, 128, 255});
        tools::draw_text(
          display_image, fmt::format("Pitch {:.2f}", perception->ypr[1] * 180.0 / CV_PI),
          {40, 160}, {0, 255, 255});
        tools::draw_text(
          display_image, fmt::format("Roll {:.2f}", perception->ypr[2] * 180.0 / CV_PI),
          {40, 200}, {0, 255, 255});
        tools::draw_text(
          display_image,
          fmt::format(
            "FFT {} f {:.3f} Hz A {:.3f} R2 {:.2f}",
            fft.get_is_periodic() ? "periodic" : "waiting", fft.get_frequency(),
            fft.get_amplitude(), fft.get_fit_quality()),
          {40, 240}, {255, 128, 0});
        if (plan.control) {
          tools::draw_text(
            display_image,
            fmt::format(
              "Plan yaw {:.2f} pitch {:.2f} fire {}", plan.yaw * 180.0 / CV_PI,
              plan.pitch * 180.0 / CV_PI, plan.fire),
            {40, 280}, {255, 255, 0});
        }

        cv::Mat display;
        cv::resize(display_image, display, {}, 0.5, 0.5);
        cv::imshow("rb_auto_aim_simulator", display);
        const auto key = cv::waitKey(1);
        if (key == 'q' || key == 27) break;
      }
    } catch (...) {
      main_error = std::current_exception();
    }

    stop_workers();
    if (capture_thread.joinable()) capture_thread.join();
    if (perception_thread.joinable()) perception_thread.join();
    if (fft_thread.joinable()) fft_thread.join();

    if (main_error) std::rethrow_exception(main_error);
    {
      std::lock_guard lock(worker_error_mutex);
      if (worker_error) std::rethrow_exception(worker_error);
    }
  } catch (const std::exception & e) {
    tools::logger()->error("rb_auto_aim_simulator退出: {}", e.what());
    result = 1;
  }

  executor.cancel();
  if (spin_thread.joinable()) spin_thread.join();
  rclcpp::shutdown();
  cv::destroyAllWindows();
  return result;
}
