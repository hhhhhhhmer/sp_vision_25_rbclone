#include "commandgener.hpp"

#include "tools/math_tools.hpp"
#include "tools/periodic_timer.hpp"

namespace auto_aim
{
namespace multithread
{

CommandGener::CommandGener(
  auto_aim::Shooter & shooter, auto_aim::Aimer & aimer, io::CBoard & cboard,
  tools::Plotter & plotter, bool debug)
: shooter_(shooter), aimer_(aimer), cboard_(cboard), plotter_(plotter), stop_(false), debug_(debug)
{
  thread_ = std::thread(&CommandGener::generate_command, this);
}

CommandGener::~CommandGener()
{
  {
    std::lock_guard<std::mutex> lock(mtx_);
    stop_ = true;
  }
  cv_.notify_all();
  if (thread_.joinable()) thread_.join();
}

void CommandGener::push(
  const std::list<auto_aim::Target> & targets, const std::chrono::steady_clock::time_point & t,
  double bullet_speed, const Eigen::Vector3d & gimbal_pos)
{
  std::lock_guard<std::mutex> lock(mtx_);
  latest_ = {targets, t, bullet_speed, gimbal_pos};
  cv_.notify_one();
}

void CommandGener::generate_command()
{
  auto t0 = std::chrono::steady_clock::now();

  // 下发节拍：固定为绝对时刻 t0 + k*2ms，周期不再等于"解算耗时 + sleep"
  tools::tighten_timer_slack();
  tools::PeriodicTimer tick(std::chrono::milliseconds(2));  // approximately 500Hz

  while (!stop_) {
    std::optional<Input> input;
    {
      std::lock_guard<std::mutex> lock(mtx_);
      if (latest_ && tools::delta_time(std::chrono::steady_clock::now(), latest_->t) < 0.2) {
        input = latest_;
      } else
        input = std::nullopt;
    }

    // 先把命令算好，节拍点一到就发（发送时刻与 aimer/shooter 耗时无关）
    std::optional<io::Command> command;
    if (input) {
      command = aimer_.aim(input->targets_, input->t, input->bullet_speed);
      command->shoot = shooter_.shoot(*command, aimer_, input->targets_, input->gimbal_pos);
      command->horizon_distance = input->targets_.empty()
                                    ? 0
                                    : std::sqrt(
                                        tools::square(input->targets_.front().ekf_x()[0]) +
                                        tools::square(input->targets_.front().ekf_x()[2]));
    }

    // 睡到绝对节拍点后再发送：从唤醒到写出只隔一次函数调用，
    // 因此发送间隔与解算耗时无关（超时则跳整拍，绝不连发）
    const auto lateness = tick.wait_next();

    if (command) {
      cboard_.send(*command);
      if (debug_) {
        nlohmann::json data;
        data["t"] = tools::delta_time(std::chrono::steady_clock::now(), t0);
        data["cmd_yaw"] = command->yaw * 57.3;
        data["cmd_pitch"] = command->pitch * 57.3;
        data["shoot"] = command->shoot;
        data["horizon_distance"] = command->horizon_distance;
        data["send_late_us"] =
          std::chrono::duration_cast<std::chrono::microseconds>(lateness).count();
        data["send_missed"] = static_cast<double>(tick.missed());
        plotter_.plot(data);
      }
    }
  }
}

}  // namespace multithread

}  // namespace auto_aim