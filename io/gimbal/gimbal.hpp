#ifndef IO__GIMBAL_HPP
#define IO__GIMBAL_HPP

#include <Eigen/Geometry>
#include <atomic>
#include <chrono>
#include <mutex>
#include <string>
#include <thread>
#include <tuple>
#include <vector>

#include "serial/serial.h"
#include "tools/thread_safe_queue.hpp"

namespace io
{

enum class WorkMode : uint8_t
{
  IDLE = 0,              // 空闲
  AUTO_AIM = 1,          // 自瞄模式
  OMNI_PERCEPTION = 2,   // 全向感知模式
};

struct __attribute__((packed)) GimbalToVision
{
  uint8_t head[2] = {0x5a,0x53};
  uint8_t mode;  // 0: 空闲, 1: 自瞄, 2: 小符, 3: 大符, 4:开长焦  电控控制右键0，1
  uint16_t color; // 0: 红色, 1: 蓝色
  float q[4];    // wxyz顺序
  float bullet_speed;
  uint16_t bullet_count;  // 子弹累计发送次数
  float gimbal_yaw;
  float gimbal_pitch;
  uint16_t crc16;
};

static_assert(sizeof(GimbalToVision) <= 64);

struct __attribute__((packed)) VisionToGimbal
{
  uint8_t head = {0x66};
  uint8_t mode = 0;  // 0: 不控制, 1: 控制云台但不开火，2:控制云台开火                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                    
  float yaw = 0;
  float yaw_vel = 0;
  float yaw_acc = 0;
  float pitch = 0;
  float pitch_vel = 0;
  float pitch_acc = 0;
  uint16_t crc16;
  uint8_t end = {0x11};
};

struct __attribute__((packed)) sb_VisionToGimbal
{
  uint8_t head = {0x66};
  uint8_t mode = 0;  // 0: 不控制, 1: 控制云台但不开火，2:控制云台开火                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                    
  float yaw = 0;
  float yaw_vel = 0;
  float yaw_acc = 0;
  float pitch = 0;
  float pitch_vel = 0;
  float pitch_acc = 0;
  float target_x = 0;
  float target_y = 0;
  uint8_t target_name = 0;
  uint8_t end = {0x11};
};

struct __attribute__((packed)) OmniVisionToGimbal
{
  uint8_t head = {0x66};
  uint8_t mode = 0;
  float yaw = 0;       // rad
  float pitch = 0;     // rad
  float distance = 0;  // m, target-to-gimbal 3D distance
  uint8_t end = {0x11};
};

static_assert(sizeof(VisionToGimbal) <= 64);
static_assert(sizeof(OmniVisionToGimbal) == 15);

enum class GimbalMode
{
  IDLE,        // 空闲
  AUTO_AIM,    // 自瞄
  SMALL_BUFF,  // 小符
  BIG_BUFF,     // 大符
  LONG_FOCAL_LENGTH //长焦
};

struct GimbalState
{
  float yaw;
  float yaw_vel;
  float pitch;
  float pitch_vel;
  float q2yaw;
  float q2pitch;
  uint8_t mode;
  uint8_t enemy_color; // 0: 蓝色, 1: 红色
  float bullet_speed;
  uint16_t bullet_count;
};

class Gimbal
{
public:
  Gimbal(const std::string & config_path);

  ~Gimbal();

  GimbalMode mode() const;
  GimbalState state() const;
  std::string str(GimbalMode mode) const;
  Eigen::Quaterniond q(std::chrono::steady_clock::time_point t);

  void send(
    bool control,bool fire, float yaw, float yaw_vel, float yaw_acc, float pitch, float pitch_vel,
    float pitch_acc);
  
  void sb_send(
  bool control, bool fire, float yaw, float yaw_vel, float yaw_acc,
  float pitch, float pitch_vel, float pitch_acc, float target_x, float target_y, uint8_t target_name);

  void send(io::VisionToGimbal VisionToGimbal);

  void sb_send(io::sb_VisionToGimbal VisionToGimbal);

  void omni_send(uint8_t mode, float yaw, float pitch, float distance);

  void omni_send(const io::OmniVisionToGimbal & VisionToGimbal);

  GimbalState* set_state_(){
    return &state_;
  }

private:
  serial::Serial serial_;
  std::vector<std::string> com_ports_;

  std::thread thread_;
  std::atomic<bool> quit_ = false;
  mutable std::mutex mutex_;

  GimbalToVision rx_data_;
  VisionToGimbal tx_data_;
  sb_VisionToGimbal sb_tx_data_;
  OmniVisionToGimbal omni_tx_data_;

  GimbalMode mode_ = GimbalMode::IDLE;
  GimbalState state_;
  tools::ThreadSafeQueue<std::tuple<Eigen::Quaterniond, std::chrono::steady_clock::time_point>>
    queue_{1000};

  int gimbal_yaw2vision, gimbal_pitch2vision, gimbal_roll2vision;

  bool read(uint8_t * buffer, size_t size);
  bool open_serial();
  void read_thread();
  void reconnect();
};

}  // namespace io

#endif  // IO__GIMBAL_HPP
