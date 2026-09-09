#include <fmt/core.h>

#include <chrono>
#include <opencv2/opencv.hpp>

#include "io/camera.hpp"
#include "tasks/auto_aim/rv_detector.hpp"
#include "tasks/auto_aim/armor.hpp"
#include "tasks/auto_aim/tracker.hpp"
#include "tools/exiter.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/img_tools.hpp"
#include "tools/reprojection.hpp"

const std::string keys =
  "{help h usage ? |                        | 输出命令行参数说明 }"
  "{@config-path   | ../configs/drone.yaml    | yaml配置文件的路径}"
  "{tradition t    |  true                 | 是否使用传统方法识别}";

int main(int argc, char * argv[])
{
  // 读取命令行参数
  cv::CommandLineParser cli(argc, argv, keys);
  if (cli.has("help")) {
    cli.printMessage();
    return 0;
  }
  auto config_path = cli.get<std::string>(0);
  auto use_tradition = cli.get<bool>("tradition");

  tools::Exiter exiter;

  io::Camera camera(config_path);
  rv_aim::Detector detector(config_path, false);
  auto_aim::Solver solver(config_path);
  auto_aim::Tracker tracker(config_path, &solver);

  std::chrono::steady_clock::time_point timestamp;

  while (!exiter.exit()) {
    cv::Mat img;
    std::list<auto_aim::Armor> armors;

    camera.read(img, timestamp);

    if (img.empty()) break;

    auto last = std::chrono::steady_clock::now();

    
    armors = detector.detect(img);
    

    auto now = std::chrono::steady_clock::now();
    auto targets = tracker.test_track(armors, now);
    if (!targets.empty()) {
      tools::draw_reprojection(img, solver, targets.front());
    }
    auto dt = tools::delta_time(now, last);
    tools::logger()->info("{:.2f} fps", 1 / dt);

    cv::imshow("img", img);
    auto key = cv::waitKey(33);
    if (key == 'q') break;
  }

  return 0;
}
