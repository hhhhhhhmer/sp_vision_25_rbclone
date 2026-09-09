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
#include "tools/img_tools.hpp"
#include "tools/reprojection.hpp"

#include "method_set/binocular_aim.hpp"

const std::string keys =
  "{help h usage ? |                                             | 输出命令行参数说明}"
  "{short_camera   | ../configs/sb_short.yaml                          | 短焦相机配置文件路径 }"
  "{long_camera    | ../configs/sb_long.yaml                     | 长焦相机配置文件路径 }";


using namespace std::chrono_literals;



int main(int argc, char * argv[])
{
  tools::Exiter exiter;
  tools::Plotter plotter;
  tools::Recorder recorder;
  
  cv::CommandLineParser cli(argc, argv, keys);
  if (cli.has("help")) {
    cli.printMessage();
    return 0;
  }
  auto short_camera_config_path = cli.get<std::string>("short_camera");
  auto long_camera_config_path = cli.get<std::string>("long_camera");


  
  // 主相机（工业相机）
  io::Camera short_camera(short_camera_config_path);
  io::Camera long_camera(long_camera_config_path);
  
//   串口通信
//   io::Gimbal gimbal(short_camera_config_path);
  
  // 视觉模块
  auto_aim::YOLO yolo(short_camera_config_path, false);  // 主相机YOLO

  auto_aim::Solver short_camera_solver(short_camera_config_path);
  auto_aim::Solver long_camera_solver(long_camera_config_path);

  auto_aim::Tracker tracker(short_camera_config_path, &short_camera_solver);//默认短焦
  
  // MPC 规划器
  auto_aim::Planner short_camera_planner(short_camera_config_path);
  auto_aim::Planner long_camera_planner(long_camera_config_path);

  //双目切换
  BinocularAim bincameras(short_camera, long_camera, 
                          short_camera_solver, long_camera_solver, 
                          short_camera_planner, long_camera_planner );
  
  
  // 线程安全队列（用于MPC规划线程）
  tools::ThreadSafeQueue<std::optional<auto_aim::Target>, true> target_queue(1);
  target_queue.push(std::nullopt);
  
  cv::Mat img;
  std::chrono::steady_clock::time_point timestamp;
  std::chrono::steady_clock::time_point last_t;
  
  // 云台状态
  Eigen::Vector3d gimbal_euler;
  
  
  // MPC 规划线程（独立线程运行MPC控制器）
  std::atomic<bool> quit = false;

  // 主循环
  while (!exiter.exit()) {
    static int count = 0;
    // tools::logger()->info("当前使用 {} 焦镜头", bincameras.is_short ? "短" : "长");
    // 读取主相机图像
    bincameras.read(img, timestamp, tracker);
    // short_camera.read(img, timestamp);
    
    // // 获取云台欧拉角
    gimbal_euler = tools::eulers(bincameras.solvers.aim_ptr->R_gimbal2world(), 2, 1, 0);
    
    // 主相机检测
    auto armors = yolo.detect(img);
    
    // 跟踪目标
    auto targets = tracker.sb_track(armors, timestamp);
    double fps = 1./tools::delta_time(std::chrono::steady_clock::now(), last_t);
    tools::draw_text(img, "fps: "+std::to_string(fps), cv::Point(40, 130));
    last_t = std::chrono::steady_clock::now();
    tools::logger()->info("fps:: {:.2f}", fps);
     
    // tools::draw_reprojection(
    //   img, *bincameras.solvers.aim_ptr, targets.front(),
    //   bincameras.planners.aim_ptr->debug_xyza);

    cv::resize(img, img, {}, 0.5, 0.5);  // 显示时缩小图片尺寸
    cv::imshow("reprojection", img);
    auto key = cv::waitKey(1);
    if (key == 'q') break;
    if (key == 'c'){// 强制切换
        bincameras.Switch(tracker);
    }
  }
  
  // 清理
  quit = true;
  
  
  return 0;
}
