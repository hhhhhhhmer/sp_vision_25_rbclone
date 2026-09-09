#include "reprojection.hpp"

#include <opencv2/imgproc.hpp>

#include "img_tools.hpp"

namespace tools
{
void draw_reprojection(
  cv::Mat & image, const Eigen::VectorXd & ekf_x,
  const std::vector<Eigen::Vector4d> & armor_xyza_list,
  const std::optional<Eigen::Vector4d> & aim_xyza, const ArmorReprojector & reproject_armor,
  const cv::Scalar & armor_color, const cv::Scalar & aim_color)
{
  if (ekf_x.size() >= 8) {
    const Eigen::Vector3d center_world(ekf_x[0], ekf_x[2], ekf_x[4]);
    const Eigen::Vector3d velocity(ekf_x[1], ekf_x[3], ekf_x[5]);
    const Eigen::Vector3d predicted_center = center_world + velocity * 0.5;

    Eigen::Vector3d angular_velocity_endpoint = center_world;
    angular_velocity_endpoint[2] += ekf_x[7] * 0.1;

    const auto center_image = reproject_armor(center_world, 0.0);
    const auto predicted_center_image = reproject_armor(predicted_center, 0.0);
    const auto angular_velocity_image = reproject_armor(angular_velocity_endpoint, 0.0);

    if (!center_image.empty() && !predicted_center_image.empty()) {
      cv::circle(image, center_image[0], 5, cv::Scalar(51, 153, 237), -1);
      cv::circle(image, predicted_center_image[0], 8, cv::Scalar(0, 0, 255), -1);
      cv::line(image, center_image[0], predicted_center_image[0], cv::Scalar(0, 255, 255), 2);

      if (!angular_velocity_image.empty()) {
        cv::line(image, center_image[0], angular_velocity_image[0], cv::Scalar(0, 255, 0), 2);
      }
    }
  }

  for (const Eigen::Vector4d & xyza : armor_xyza_list) {
    draw_points(image, reproject_armor(xyza.head<3>(), xyza[3]), armor_color);
  }

  if (aim_xyza.has_value()) {
    draw_points(
      image, reproject_armor(aim_xyza->head<3>(), (*aim_xyza)[3]), aim_color);
  }
}

}  // namespace tools
