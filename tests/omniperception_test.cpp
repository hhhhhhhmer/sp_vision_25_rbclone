#include <chrono>
#include <opencv2/opencv.hpp>
#include <thread>

#include "io/camera.hpp"
#include "io/gimbal/gimbal.hpp"  // 改为使用Gimbal串口通信
#include "io/usbcamera/usbcamera.hpp"
#include "tasks/auto_aim/aimer.hpp"
#include "tasks/auto_aim/shooter.hpp"
#include "tasks/auto_aim/solver.hpp"
#include "tasks/auto_aim/tracker.hpp"
#include "tasks/auto_aim/planner/planner.hpp"  // MPC 规划器
#include "tasks/auto_aim/yolo.hpp"
#include "tasks/omniperception/decider.hpp"
#include "tools/exiter.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/plotter.hpp"
#include "tools/recorder.hpp"
#include "tools/yaml.hpp"

const std::string keys =
  "{help h usage ? |                                             | 输出命令行参数说明}"
  "{l_cam          | ../configs/omn_camera_left.yaml | 左感知相机 }"
  "{r_cam          | ../configs/omn_camera_right.yaml  | 右感知相机 }";

using namespace std::chrono_literals;


int main(int argc, char * argv[])
{
  tools::Exiter exiter;
  tools::Plotter plotter;
  // tools::Recorder recorder(90);
  int i =0;
  
  cv::CommandLineParser cli(argc, argv, keys);
  if (cli.has("help")) {
    cli.printMessage();
    return 0;
  }


  // 全向感知相机（工业相机）
  std::string omnl_yaml_name = cli.get<std::string>("l_cam");
  std::string omnr_yaml_name = cli.get<std::string>("r_cam");
  io::Camera omn_caml(omnl_yaml_name);
  io::Camera omn_camr(omnr_yaml_name);
  // 全向感知决策器
  auto_aim::Solver left_solver(omnl_yaml_name);
  auto_aim::Solver  right_solver(omnr_yaml_name);
  auto omn_l_yaml = tools::load(omnl_yaml_name);
  auto omn_r_yaml = tools::load(omnr_yaml_name);
  omn_caml.main_and_secondary = tools::read<std::string>(omn_l_yaml, "main_and_secondary");
  omn_camr.main_and_secondary = tools::read<std::string>(omn_r_yaml, "main_and_secondary");
  // io::Camera back_camera("configs/camera.yaml");
  tools::logger()->info("初始化");
  // 改为使用Gimbal串口通信（替代CBoard）
  io::Gimbal gimbal(omnl_yaml_name);

  auto_aim::Solver solver(omnl_yaml_name);
  
  // 视觉模块
  auto_aim::YOLO yolo(omnl_yaml_name, false);  // 主相机YOLO
  
  cv::Mat img1, img2;
  std::chrono::steady_clock::time_point t1, t2;
  std::chrono::steady_clock::time_point last_t;

  // 全向感知决策器
  omniperception::Decider decider(omnl_yaml_name);
  


  // 新增一个变量用于记录全向相机是否处于暂停状态
  bool is_omn_paused = false; 

  // 主循环5
  while (!exiter.exit()) {

    // omn_caml.read(img1, t1);
    // omn_camr.read(img2, t2);

    // 获取云台欧拉角
    auto gimbal_euler = tools::eulers(solver.R_gimbal2world(), 2, 1, 0);
    auto q = gimbal.q(t1);
    static io::VisionToGimbal last_vision_cmd;
    // 全向感知模式
    io::VisionToGimbal vision_cmd = decider.decide_g(
      yolo, gimbal_euler, omn_caml, omn_camr, left_solver, right_solver);
    
    nlohmann::json data;

    // data["mode"] = vision_cmd.mode;
    // data["yaw"] =(float)vision_cmd.yaw;

    plotter.plot(data);
    // recorder.record(img1, q, t1);
    
    // gimbal.send(vision_cmd);

    // cv::imshow("l_cam", img1);
    // cv::imshow("r_cam", img2);
    // auto key = cv::waitKey(1);
    // if (key == 'q') break;
    

}
}