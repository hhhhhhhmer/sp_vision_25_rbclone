#include "gimbal.hpp"

#include "tools/crc.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/yaml.hpp"
#include <opencv2/opencv.hpp>

namespace io
{
/** @brief yaw/pitch 角速度估计的一阶平滑时间常数（s）；按 dt 折算，不随反馈频率变化 */
constexpr double kVelFilterTauS = 0.02;

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

std::pair<GimbalState, std::chrono::steady_clock::time_point> Gimbal::state_at() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return {state_, state_time_};
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
  // 入参已是完整帧（head/end 由 NSDMI 提供），直接发送局部副本即可
  write_frame(VisionToGimbal);
}

void Gimbal::omni_send(const io::OmniVisionToGimbal & VisionToGimbal)
{
  omni_send(
    VisionToGimbal.mode, VisionToGimbal.yaw, VisionToGimbal.pitch, VisionToGimbal.distance);
}

void Gimbal::omni_send(uint8_t mode, float yaw, float pitch, float distance)
{
  OmniVisionToGimbal frame;
  frame.mode = mode;
  frame.yaw = yaw;
  frame.pitch = pitch;
  frame.distance = distance;

  write_frame(frame);
}

void Gimbal::send(io::VisionToGimbal VisionToGimbal)
{
  // 下行 CRC16 —— 必须对齐下位机 communication.c 的 check_crc16(buf, 29)：
  //   uint16_t crc = (buf[27] << 8) | buf[26];        // 即"小端存放"
  //   return get_crc16(buf, 29 - 3) == crc;           // 注意：只算了前 26 字节
  // 所以这里要把 CRC 放在字段 26..27（VisionToGimbal.crc16 的位置，小端）
  // 并且**只覆盖前 26 字节**。表、初值(0xffff)、字节序两边已逐位核对一致。
  //
  // 注意下位机自身的不一致：它发送上行帧时是 get_crc16(buf,27) 且 CRC 在 27..28，
  // 与 check_crc16 的读法错开一个字节。上行两个方向自洽（视觉 tools::check_crc16
  // 用 len-2=27 覆盖 0..26、读 buf[26..27]），下行按上面的方式对齐才能通过校验。
  // 任何一侧改了帧布局都必须同步，否则表现为"所有帧被静默丢弃"。
  VisionToGimbal.crc16 = tools::get_crc16(
    reinterpret_cast<uint8_t *>(&VisionToGimbal),
    sizeof(VisionToGimbal) - sizeof(VisionToGimbal.crc16) - 1);
  write_frame(VisionToGimbal);
}

void Gimbal::send(
  bool control, bool fire, float yaw, float yaw_vel, float yaw_acc, float pitch, float pitch_vel,
  float pitch_acc)
{
  VisionToGimbal frame;
  frame.mode = control ? (fire ? 2 : 1) : 0;
  frame.yaw = yaw;
  frame.yaw_vel = yaw_vel;
  frame.yaw_acc = yaw_acc;
  frame.pitch = pitch;
  frame.pitch_vel = pitch_vel;
  frame.pitch_acc = pitch_acc;

  send(frame);
}

void Gimbal::sb_send(
  bool control, bool fire, float yaw, float yaw_vel, float yaw_acc,
  float pitch, float pitch_vel, float pitch_acc, float target_x, float target_y, uint8_t target_name)
{
  sb_VisionToGimbal frame;
  frame.mode = control ? (fire ? 2 : 1) : 0;
  frame.yaw = yaw;
  frame.yaw_vel = yaw_vel;
  frame.yaw_acc = yaw_acc;
  frame.pitch = pitch;
  frame.pitch_vel = pitch_vel;
  frame.pitch_acc = pitch_acc;
  frame.target_x = target_x;
  frame.target_y = target_y;
  frame.target_name = target_name;

  write_frame(frame);
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

    // 先更新线程安全状态、再入队：构造函数里的 queue_.pop() 返回时，state_/state_time_
    // 必须已经就绪，否则首帧会出现 state_at() 时间戳仍为 0（gimbal_calib 会误判"未收到反馈"）
    {
      std::lock_guard<std::mutex> lock(mutex_);
      auto ypr_now = tools::eulers(q, 2, 1, 0);
      // 云台状态反馈：一律由四元数(IMU)推导（上行包 gimbal_yaw/gimbal_pitch 字段已停用）
      const double yaw_deg = ypr_now[0] * 57.3;
      const double pitch_deg = ypr_now[1] * 57.3;
      state_.yaw = static_cast<float>(yaw_deg);
      state_.pitch = static_cast<float>(pitch_deg);

      // 角速度估计（rad/s）：相邻两帧反馈差分 + 一阶平滑
      // 注意 delta_time(a, b) 返回 a-b 且约定 a 为较新时刻，写成 (last, now) 会得到负 dt，
      // 导致下面所有分支都不成立、速度永远停在初值（首帧前是未定义值）。
      const double dt = has_last_fb_ ? tools::delta_time(t, last_fb_time_) : 0.0;
      if (has_last_fb_ && dt > 1e-4 && dt < 0.5) {
        const double raw_yaw_vel =
          tools::limit_rad((yaw_deg - last_fb_yaw_deg_) / 57.3) / dt;
        const double raw_pitch_vel =
          tools::limit_rad((pitch_deg - last_fb_pitch_deg_) / 57.3) / dt;
        if (!has_vel_estimate_) {
          // 首个有效差分直接作为初值
          state_.yaw_vel = static_cast<float>(raw_yaw_vel);
          state_.pitch_vel = static_cast<float>(raw_pitch_vel);
          has_vel_estimate_ = true;
        } else {
          // 平滑系数按 dt 计算，反馈频率变化时时间常数保持不变
          const double alpha = 1.0 - std::exp(-dt / kVelFilterTauS);
          state_.yaw_vel = static_cast<float>((1.0 - alpha) * state_.yaw_vel + alpha * raw_yaw_vel);
          state_.pitch_vel =
            static_cast<float>((1.0 - alpha) * state_.pitch_vel + alpha * raw_pitch_vel);
        }
      } else if (has_last_fb_ && dt >= 0.5) {
        // 反馈中断过久：清速度并等待下一次重新起步，避免使用陈旧角速度
        state_.yaw_vel = 0;
        state_.pitch_vel = 0;
        has_vel_estimate_ = false;
      }
      last_fb_time_ = t;
      last_fb_yaw_deg_ = static_cast<float>(yaw_deg);
      last_fb_pitch_deg_ = static_cast<float>(pitch_deg);
      has_last_fb_ = true;
      state_time_ = t;

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

    queue_.push({q, t});
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
