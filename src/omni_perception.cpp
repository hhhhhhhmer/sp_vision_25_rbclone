#include <chrono>
#include <thread>

#include <Eigen/Dense>
#include <opencv2/opencv.hpp>

#include "io/camera.hpp"
#include "io/gimbal/gimbal.hpp"
#include "tasks/auto_aim/solver.hpp"
#include "tasks/auto_aim/yolo.hpp"
#include "tasks/omniperception/decider.hpp"
#include "tools/exiter.hpp"
#include "tools/systemd_watchdog.hpp"
#include "tools/logger.hpp"
#include "tools/yaml.hpp"

const std::string keys =
  "{help h usage ? |                                  | print this message}"
  "{l_cam          | ../configs/omn_camera_left.yaml  | left omni camera config}"
  "{r_cam          | ../configs/omn_camera_right.yaml | right omni camera config}"
  "{gimbal_config  | ../configs/omn_camera_left.yaml  | gimbal serial config}";

using namespace std::chrono_literals;

int main(int argc, char * argv[])
{
  tools::SystemdWatchdog systemd_watchdog;
  tools::Exiter exiter;

  cv::CommandLineParser cli(argc, argv, keys);
  if (cli.has("help")) {
    cli.printMessage();
    return 0;
  }

  const auto left_config_path = cli.get<std::string>("l_cam");
  const auto right_config_path = cli.get<std::string>("r_cam");
  const auto gimbal_config_path = cli.get<std::string>("gimbal_config");

  io::Camera::initSDK();

  io::Camera left_camera(left_config_path);
  io::Camera right_camera(right_config_path);

  auto left_yaml = tools::load(left_config_path);
  auto right_yaml = tools::load(right_config_path);
  left_camera.main_and_secondary = tools::read<std::string>(left_yaml, "main_and_secondary");
  right_camera.main_and_secondary = tools::read<std::string>(right_yaml, "main_and_secondary");

  io::Gimbal gimbal(gimbal_config_path);

  auto_aim::YOLO yolo(left_config_path, false);
  auto_aim::Solver left_solver(left_config_path);
  auto_aim::Solver right_solver(right_config_path);

  omniperception::Decider decider(left_config_path);
  decider.set_gimbal(&gimbal);

  tools::logger()->info("[OmniPerception] started.");

  if (!systemd_watchdog.ready("Vision pipeline is ready")) {
    tools::logger()->warn("无法向 systemd 发送 READY 通知");
  }

  while (!exiter.exit()) {
    const auto state = gimbal.state();
    const Eigen::Vector3d gimbal_euler(state.yaw / 57.3, state.pitch / 57.3, 0.0);

    float target_distance = 0.0f;
    const auto vision_cmd = decider.decide_g(
      yolo, gimbal_euler, left_camera, right_camera, left_solver, right_solver, &target_distance);

    systemd_watchdog.ping();
    gimbal.omni_send(vision_cmd.mode, vision_cmd.yaw, vision_cmd.pitch, target_distance);
    std::this_thread::sleep_for(1ms);
  }

  gimbal.omni_send(0, 0.0f, 0.0f, 0.0f);
  tools::logger()->info("[OmniPerception] stopped.");

  return 0;
}
