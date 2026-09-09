#ifndef TOOLS__REPROJECTION_HPP
#define TOOLS__REPROJECTION_HPP

#include <Eigen/Dense>
#include <functional>
#include <optional>
#include <opencv2/core.hpp>
#include <vector>

namespace tools
{
using ArmorReprojector =
  std::function<std::vector<cv::Point2f>(const Eigen::Vector3d &, double)>;

void draw_reprojection(
  cv::Mat & image, const Eigen::VectorXd & ekf_x,
  const std::vector<Eigen::Vector4d> & armor_xyza_list,
  const std::optional<Eigen::Vector4d> & aim_xyza, const ArmorReprojector & reproject_armor,
  const cv::Scalar & armor_color = {0, 255, 0},
  const cv::Scalar & aim_color = {0, 0, 255});

template<typename Solver, typename Target>
void draw_reprojection(
  cv::Mat & image, const Solver & solver, const Target & target,
  const std::optional<Eigen::Vector4d> & aim_xyza = std::nullopt,
  const cv::Scalar & armor_color = {0, 255, 0},
  const cv::Scalar & aim_color = {0, 0, 255})
{
  auto reproject_armor = [&](const Eigen::Vector3d & xyz, double yaw) {
    return solver.reproject_armor(xyz, yaw, target.armor_type, target.name);
  };
  draw_reprojection(
    image, target.ekf_x(), target.armor_xyza_list(), aim_xyza, reproject_armor, armor_color,
    aim_color);
}

}  // namespace tools

#endif  // TOOLS__REPROJECTION_HPP
