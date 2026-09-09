#include "io/camera.hpp"

#include <opencv2/opencv.hpp>

#include "tools/exiter.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"                                         
#include "tools/img_tools.hpp"

const std::string keys =
  "{help h usage ? |                     | 输出命令行参数说明}"
  "{config-path c  | ../configs/sb_short.yaml | yaml配置文件路径 }"
  "{d display      |        1             | 显示视频流       }";

int main(int argc, char * argv[])
{
  cv::CommandLineParser cli(argc, argv, keys);
  if (cli.has("help")) {
    cli.printMessage();
    return 0;
  }

  tools::Exiter exiter;

  auto config_path = cli.get<std::string>("config-path");
  auto display = cli.has("display");
  io::Camera camera(config_path);

  cv::Mat img;
  std::chrono::steady_clock::time_point timestamp;
  std::chrono::steady_clock::time_point last_t;

  auto last_stamp = std::chrono::steady_clock::now();
  while (!exiter.exit()) {
    camera.read(img, timestamp);

    double fps = 1./std::chrono::duration_cast<std::chrono::microseconds>(timestamp - last_t).count()*1000000;
    tools::draw_text(img, "fps: "+std::to_string(fps), cv::Point(40, 130));
    last_t = timestamp;

    auto dt = tools::delta_time(timestamp, last_stamp);
    last_stamp = timestamp;

    tools::logger()->info("{:.2f} fps", 1 / dt);

    if (!display) continue;
    cv::imshow("img", img);
    if (cv::waitKey(1) == 'q') break;
  }
}