#include "camera.hpp"

#include <stdexcept>

#include "hikrobot/hikrobot.hpp"
#include "mindvision/mindvision.hpp"
#include "daheng/daheng.hpp"
#include "tools/yaml.hpp"

namespace io
{
void Camera::initSDK()
{
  DahengCamera::initSDK();
}

Camera::Camera(const std::string & config_path)
{
  auto yaml = tools::load(config_path);
  auto camera_name = tools::read<std::string>(yaml, "camera_name");
  auto exposure_us = tools::read<double>(yaml, "exposure_us");
  auto timestamp_offset_us = tools::read<int>(yaml, "timestamp_offset_us");
  timestamp_offset = std::chrono::microseconds(timestamp_offset_us);

  bool flip = tools::read<bool>(yaml, "flip");
  bool mirror = tools::read<bool>(yaml, "mirror");

  img_gamma = tools::read<double>(yaml, "img_gamma");

  int lut_size = 1 << 8;
  this->img_gamma_lut = cv::Mat(lut_size, 1, CV_8U);
  for(int i = 0; i < lut_size; i++){
    img_gamma_lut.data[i] = cv::saturate_cast<uchar>(pow(i/255.0, img_gamma) * 255.0);
  }

  if (camera_name == "mindvision") {
    auto gamma = tools::read<double>(yaml, "gamma");
    auto vid_pid = tools::read<std::string>(yaml, "vid_pid");
    camera_ = std::make_unique<MindVision>(exposure_us, gamma, vid_pid);
  }

  else if (camera_name == "hikrobot") {
    auto gain = tools::read<double>(yaml, "gain");
    auto vid_pid = "2bdf:0001";
    auto sn = tools::read<std::string>(yaml, "camera_sn");
    camera_ = std::make_unique<HikRobot>(sn, exposure_us, gain, vid_pid, flip, mirror);
  }

  else if(camera_name == "daheng"){
    auto gain = tools::read<double>(yaml, "gain");
    auto gamma = tools::read<double>(yaml, "gamma");
    auto vid_pid = "2ba2:4d55";
    auto camera_sn = tools::read<std::string>(yaml, "camera_sn");
    camera_ = std::make_unique<DahengCamera>(camera_sn, exposure_us, gain, gamma, flip, mirror);
  }

  else {
    throw std::runtime_error("Unknow camera_name: " + camera_name + "!");
  }
}

void Camera::read(cv::Mat & img, std::chrono::steady_clock::time_point & timestamp)
{
  camera_->read(img, timestamp);
  // if(img_gamma == 1.0){
  //   cv::LUT(img, img_gamma_lut, img);
  // }

  if (std::abs(img_gamma - 1.0) > 1e-6)
  {
    cv::LUT(img, img_gamma_lut, img);
  }
}
bool Camera::try_read(cv::Mat & img, std::chrono::steady_clock::time_point & timestamp)
{
  return camera_->try_read(img, timestamp);
}
}  // namespace io
