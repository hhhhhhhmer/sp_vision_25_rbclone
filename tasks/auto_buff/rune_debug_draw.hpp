#ifndef AUTO_BUFF__RUNE_DEBUG_DRAW_HPP
#define AUTO_BUFF__RUNE_DEBUG_DRAW_HPP

#include <array>
#include <string>

#include <opencv2/imgproc.hpp>

#include "rune_system.hpp"

namespace auto_buff
{
inline void draw_rune_debug(cv::Mat & image, const RuneDebugSnapshot & debug)
{
  static const std::array<cv::Scalar, 5> point_colors{
    cv::Scalar(0, 255, 0), cv::Scalar(0, 255, 255), cv::Scalar(255, 0, 0),
    cv::Scalar(255, 0, 255), cv::Scalar(0, 0, 255)};
  for (const auto & detection : debug.detections) {
    for (std::size_t i = 0; i < detection.keypoints.size(); ++i) {
      cv::circle(image, detection.keypoints[i], i == 2 ? 5 : 3, point_colors[i], -1, cv::LINE_AA);
    }
  }
  cv::drawContours(image, debug.armor_contours, -1, cv::Scalar(0, 255, 0), 2, cv::LINE_AA);
  cv::drawContours(image, debug.light_arm_contours, -1, cv::Scalar(0, 200, 255), 2, cv::LINE_AA);
  cv::drawContours(image, debug.center_contours, -1, cv::Scalar(255, 255, 0), 2, cv::LINE_AA);
  if (!debug.current_reprojection.empty()) {
    for (const cv::Point2f & point : debug.current_reprojection) {
      cv::circle(image, point, 1, cv::Scalar(255, 0, 0), -1, cv::LINE_AA);
    }
  }
  if (!debug.predicted_reprojection.empty()) {
    for (const cv::Point2f & point : debug.predicted_reprojection) {
      cv::circle(image, point, 5, cv::Scalar(255, 0, 255), 2, cv::LINE_AA);
    }
  }
  const std::string status = cv::format(
    "rune det %.2f ms core %.2f ms found %d fire %d fail %d",
    debug.detection_ms, debug.core_ms, debug.found ? 1 : 0, debug.fire ? 1 : 0,
    static_cast<int>(debug.failure_reason));
  cv::putText(
    image, status, cv::Point(20, 32), cv::FONT_HERSHEY_SIMPLEX, 0.55,
    cv::Scalar(255, 255, 255), 1, cv::LINE_AA);

  if (debug.uv_pose_active) {
    const std::string uv_status = debug.uv_reprojection_error
      ? cv::format(
          "UV pose rmse %.2f px nis %.1f reject %d", *debug.uv_reprojection_error,
          debug.uv_nis, debug.uv_rejected_blades)
      : cv::format("UV pose nis %.1f reject %d", debug.uv_nis, debug.uv_rejected_blades);
    cv::putText(
      image, uv_status, cv::Point(20, 56), cv::FONT_HERSHEY_SIMPLEX, 0.5,
      cv::Scalar(0, 255, 0), 1, cv::LINE_AA);
  }
}
}  // namespace auto_buff

#endif
