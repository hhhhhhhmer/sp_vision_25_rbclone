#include "gimbal.hpp"

#include "tools/crc.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/yaml.hpp"
#include <opencv2/opencv.hpp>

namespace io
{
Gimbal::Gimbal(const std::string & config_path)
{
  auto yaml = tools::load(config_path);
  const auto com_port = yaml["com_port"];
  if (!com_port) {
    tools::logger()->error("[YAML] com_port not found!");
    exit(1);
  }

  try {
    if (com_port.IsScalar()) {
      com_ports_.push_back(com_port.as<std::string>());
    } else if (com_port.IsSequence()) {
      com_ports_ = com_port.as<std::vector<std::string>>();
    }
  } catch (const YAML::Exception & e) {
    tools::logger()->error("[Gimbal] Invalid com_port config: {}", e.what());
    exit(1);
  }

  if (com_ports_.empty()) {
    tools::logger()->error("[Gimbal] com_port must be a serial port or a non-empty list.");
    exit(1);
  }

  this->gimbal_yaw2vision = tools::read<int>(yaml, "gimbal_y1");
  this->gimbal_pitch2vision = tools::read<int>(yaml, "gimbal_p2");
  this->gimbal_roll2vision = tools::read<int>(yaml, "gimbal_r3");

  serial_.setBaudrate(460800);
  auto timeout = serial::Timeout::simpleTimeout(2);
  serial_.setTimeout(timeout);
  if (!open_serial()) {
    tools::logger()->error("[Gimbal] Failed to open all configured serial ports.");
    exit(1);
  }

  thread_ = std::thread(&Gimbal::read_thread, this);

  queue_.pop();
  tools::logger()->info("[Gimbal] First q received.");
}

Gimbal::~Gimbal()
{
  quit_ = true;
  if (thread_.joinable()) thread_.join();
  serial_.close();
}

GimbalMode Gimbal::mode() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return mode_;
}

GimbalState Gimbal::state() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return state_;
}

std::string Gimbal::str(GimbalMode mode) const
{
  switch (mode) {
    case GimbalMode::IDLE:
      return "IDLE";
    case GimbalMode::AUTO_AIM:
      return "AUTO_AIM";
    case GimbalMode::SMALL_BUFF:
      return "SMALL_BUFF";
    case GimbalMode::BIG_BUFF:
      return "BIG_BUFF";
    default:
      return "INVALID";
  }
}

Eigen::Quaterniond Gimbal::q(std::chrono::steady_clock::time_point t)
{
  while (true) {
    auto [q_a, t_a] = queue_.pop();
    auto [q_b, t_b] = queue_.front();
    auto t_ab = tools::delta_time(t_a, t_b);
    auto t_ac = tools::delta_time(t_a, t);
    auto k = t_ac / t_ab;
    Eigen::Quaterniond q_c = q_a.slerp(k, q_b).normalized();
    if (t < t_a) return q_c;
    if (!(t_a < t && t <= t_b)) continue;

    return q_c;
  }
}

void Gimbal::sb_send(io::sb_VisionToGimbal VisionToGimbal)
{
  sb_tx_data_.mode = VisionToGimbal.mode;
  sb_tx_data_.yaw = VisionToGimbal.yaw;
  sb_tx_data_.yaw_vel = VisionToGimbal.yaw_vel;
  sb_tx_data_.yaw_acc = VisionToGimbal.yaw_acc;
  sb_tx_data_.pitch = VisionToGimbal.pitch;
  sb_tx_data_.pitch_vel = VisionToGimbal.pitch_vel;
  sb_tx_data_.pitch_acc = VisionToGimbal.pitch_acc;
  sb_tx_data_.target_x = VisionToGimbal.target_x;
  sb_tx_data_.target_y = VisionToGimbal.target_y;
  sb_tx_data_.target_name = VisionToGimbal.target_name;

  try {
    // 2. 这里的底层缓冲必须是 sb_tx_data_，不能是 tx_data_ ！
    serial_.write(reinterpret_cast<uint8_t *>(&sb_tx_data_), sizeof(sb_tx_data_));
  } catch (const std::exception & e) {
    tools::logger()->warn("[Gimbal] Failed to write serial: {}", e.what());
  }
}

void Gimbal::omni_send(const io::OmniVisionToGimbal & VisionToGimbal)
{
  omni_send(
    VisionToGimbal.mode, VisionToGimbal.yaw, VisionToGimbal.pitch, VisionToGimbal.distance);
}

void Gimbal::omni_send(uint8_t mode, float yaw, float pitch, float distance)
{
  omni_tx_data_.mode = mode;
  omni_tx_data_.yaw = yaw;
  omni_tx_data_.pitch = pitch;
  omni_tx_data_.distance = distance;

  try {
    serial_.write(reinterpret_cast<const uint8_t *>(&omni_tx_data_), sizeof(omni_tx_data_));
  } catch (const std::exception & e) {
    tools::logger()->warn("[Gimbal] Failed to write serial: {}", e.what());
  }
}

void Gimbal::send(io::VisionToGimbal VisionToGimbal)
{
  tx_data_.mode = VisionToGimbal.mode;
  tx_data_.yaw = VisionToGimbal.yaw;
  tx_data_.yaw_vel = VisionToGimbal.yaw_vel;
  tx_data_.yaw_acc = VisionToGimbal.yaw_acc;
  tx_data_.pitch = VisionToGimbal.pitch;
  tx_data_.pitch_vel = VisionToGimbal.pitch_vel;
  tx_data_.pitch_acc = VisionToGimbal.pitch_acc;
      reinterpret_cast<uint8_t *>(&tx_data_), sizeof(tx_data_) ;

  try {
    serial_.write(reinterpret_cast<uint8_t *>(&tx_data_), sizeof(tx_data_));
  } catch (const std::exception & e) {
    tools::logger()->warn("[Gimbal] Failed to write serial: {}", e.what());
  }
}

void Gimbal::send(
  bool control, bool fire, float yaw, float yaw_vel, float yaw_acc, float pitch, float pitch_vel,
  float pitch_acc)
{
  tx_data_.mode = control ? (fire ? 2 : 1) : 0;
  tx_data_.yaw = yaw;
  tx_data_.yaw_vel = yaw_vel;
  tx_data_.yaw_acc = yaw_acc;
  tx_data_.pitch = pitch;
  tx_data_.pitch_vel = pitch_vel;
  tx_data_.pitch_acc = pitch_acc;
      reinterpret_cast<uint8_t *>(&tx_data_), sizeof(tx_data_) ;

  try {
    serial_.write(reinterpret_cast<uint8_t *>(&tx_data_), sizeof(tx_data_));
  } catch (const std::exception & e) {
    tools::logger()->warn("[Gimbal] Failed to write serial: {}", e.what());
  }
}

void Gimbal::sb_send(
  bool control, bool fire, float yaw, float yaw_vel, float yaw_acc,
  float pitch, float pitch_vel, float pitch_acc, float target_x, float target_y, uint8_t target_name)
{
  sb_tx_data_.mode = control ? (fire ? 2 : 1) : 0;
  sb_tx_data_.yaw = yaw;
  sb_tx_data_.yaw_vel = yaw_vel;
  sb_tx_data_.yaw_acc = yaw_acc;
  sb_tx_data_.pitch = pitch;
  sb_tx_data_.pitch_vel = pitch_vel;
  sb_tx_data_.pitch_acc = pitch_acc;
  sb_tx_data_.target_x = target_x;
  sb_tx_data_.target_y = target_y;
  sb_tx_data_.target_name = target_name;

  try {
    serial_.write(reinterpret_cast<const uint8_t *>(&sb_tx_data_), sizeof(sb_tx_data_)); 
  } catch (const std::exception & e) {
    tools::logger()->warn("[Gimbal] Failed to write serial: {}", e.what());
  }
}

// void Gimbal::sb_send(
//   bool control, WorkMode work_mode, bool fire, float yaw, float yaw_vel, float yaw_acc, float pitch, float pitch_vel,
//   float pitch_acc)
// {
//   tx_data_.mode = control ? (fire ? 2 : 1) : 0;
//   tx_data_.work_mode = static_cast<uint8_t>(work_mode);
//   tx_data_.yaw = yaw;
//   tx_data_.yaw_vel = yaw_vel;
//   tx_data_.yaw_acc = yaw_acc;
//   tx_data_.pitch = pitch;
//   tx_data_.pitch_vel = pitch_vel;
//   tx_data_.pitch_acc = pitch_acc;
//       reinterpret_cast<uint8_t *>(&tx_data_), sizeof(tx_data_) ;
//   tx_data_.crc16 = tools::get_crc16(
//     reinterpret_cast<uint8_t *>(&tx_data_), sizeof(tx_data_) - sizeof(tx_data_.crc16));

//   try {
//     serial_.write(reinterpret_cast<uint8_t *>(&tx_data_), sizeof(tx_data_));
//   } catch (const std::exception & e) {
//     tools::logger()->warn("[Gimbal] Failed to write serial: {}", e.what());
//   }
// }

bool Gimbal::read(uint8_t * buffer, size_t size)
{
  try {
    return serial_.read(buffer, size) == size;
  } catch (const std::exception & e) {
    tools::logger()->warn("[Gimbal] Failed to read serial: {}", e.what());
    try {
      serial_.close();
    } catch (...) {
    }
    return false;
  }
}

bool Gimbal::open_serial()
{
  for (const auto & port : com_ports_) {
    try {
      if (serial_.isOpen()) serial_.close();
      serial_.setPort(port);
      serial_.open();
      tools::logger()->info("[Gimbal] Opened serial port: {}", port);
      return true;
    } catch (const std::exception & e) {
      tools::logger()->warn("[Gimbal] Failed to open serial port {}: {}", port, e.what());
    }
  }
  return false;
}

void Gimbal::read_thread()
{
  tools::logger()->info("[Gimbal] read_thread started.");
  int error_count = 0;

  while (!quit_) {
    if (!serial_.isOpen() || error_count > 50000) {
      error_count = 0;
      tools::logger()->warn("[Gimbal] Serial unavailable, attempting to reconnect...");
      reconnect();
      continue;
    }

    // 1. 一次性读取完整的一帧数据（基于 GimbalToVision 结构体的大小）
    if (!read(reinterpret_cast<uint8_t *>(&rx_data_), sizeof(rx_data_))) {
      error_count++;
      continue;
    }

    // 2. 检查帧头是否正确
    if (rx_data_.head[0] != 0x5a || rx_data_.head[1] != 0x53) {
      // 如果帧头不对，说明数据由于丢包等原因发生了错位（失步）
      // 此时必须立刻清空底层的接收缓冲区，把残留的错位数据全部丢弃，以便下一次能读到全新的完整帧
      serial_.flushInput(); 
      error_count++;
      // 可选：添加一条 debug 日志观察失步频率
      // tools::logger()->debug("[Gimbal] 帧头错位，已清空缓冲区");
      continue;
    }

    // 3. 记录成功接收到有效帧的时间戳
    auto t = std::chrono::steady_clock::now();

    // 4. 检查 CRC 校验和
    if (!tools::check_crc16(reinterpret_cast<uint8_t *>(&rx_data_), sizeof(rx_data_))) {
      // tools::logger()->debug("[Gimbal] CRC16 check failed.");
      error_count++;
      continue;
    }

    // --- 以下为原本的数据处理逻辑，保持不变 ---
    error_count = 0;
    Eigen::Quaterniond q_(rx_data_.q[0], rx_data_.q[1], rx_data_.q[2], rx_data_.q[3]);
    auto ypr = tools::eulers(q_, 2, 1, 0);
    
    float yaw = ypr[abs(gimbal_yaw2vision) -  1];
    float pitch = ypr[abs(gimbal_pitch2vision) - 1];
    float roll = ypr[abs(gimbal_roll2vision) - 1];

    yaw = gimbal_yaw2vision > 0 ? yaw : -yaw;
    pitch = gimbal_pitch2vision > 0 ? pitch : -pitch;
    roll = gimbal_roll2vision > 0 ? roll : -roll;

    Eigen::Quaterniond q = 
        Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) * // 绕Z轴旋转yaw
        Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) * // 绕Y轴旋转pitch
        Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX());   // 绕X轴旋转roll

    queue_.push({q, t});

    std::lock_guard<std::mutex> lock(mutex_);
    auto ypr_now = tools::eulers(q, 2, 1, 0);
    state_.yaw = ypr_now[0] * 57.3;
    state_.pitch = ypr_now[1] * 57.3;
    
    // state_.mode = 1;
    state_.mode = rx_data_.mode;
    state_.enemy_color = !rx_data_.color;
    state_.bullet_speed = rx_data_.bullet_speed;
    state_.bullet_count = rx_data_.bullet_count;
    // rx_data_.mode = 2;
    // 

    switch (rx_data_.mode) {
      case 0:
        mode_ = GimbalMode::IDLE;
        break;
      case 1:
        mode_ = GimbalMode::AUTO_AIM;
        break;
      case 2:
        mode_ = GimbalMode::SMALL_BUFF;
        break;
      case 3:
        mode_ = GimbalMode::BIG_BUFF;
        break;
      case 4:
        mode_ = GimbalMode::LONG_FOCAL_LENGTH;
        break;
      default:
        mode_ = GimbalMode::IDLE;
        tools::logger()->warn("[Gimbal] Invalid mode: {}", rx_data_.mode);
        break;
    }
  }

  tools::logger()->info("[Gimbal] read_thread stopped.");
}

void Gimbal::reconnect()
{
  int max_retry_count = 10;
  for (int i = 0; i < max_retry_count && !quit_; ++i) {
    tools::logger()->warn("[Gimbal] Reconnecting serial, attempt {}/{}...", i + 1, max_retry_count);
    try {
      serial_.close();
      std::this_thread::sleep_for(std::chrono::seconds(1));
    } catch (...) {
    }

    if (open_serial()) {
      queue_.clear();
      tools::logger()->info("[Gimbal] Reconnected serial successfully on {}.", serial_.getPort());
      break;
    } else {
      tools::logger()->warn("[Gimbal] Reconnect attempt failed on all configured ports.");
      std::this_thread::sleep_for(std::chrono::seconds(1));
    }
  }
}

}  // namespace io
