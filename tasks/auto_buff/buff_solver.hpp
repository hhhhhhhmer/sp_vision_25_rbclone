#ifndef AUTO_BUFF__SOLVER_HPP
#define AUTO_BUFF__SOLVER_HPP

#include <yaml-cpp/yaml.h>

#include <Eigen/Dense>  // 必须在opencv2/core/eigen.hpp上面
#include <opencv2/core/eigen.hpp>
#include <optional>
#include <unordered_map>
#include <vector>

#include "buff_type.hpp"
#include "tools/math_tools.hpp"
namespace auto_buff
{
// 旋转角度
const double THETA = 2.0 * CV_PI / 5.0;  // 2/5π

class Solver
{
public:
  explicit Solver(const std::string & config_path);

  Eigen::Matrix3d R_gimbal2world() const;

  void set_R_gimbal2world(const Eigen::Quaterniond & q);

  std::optional<PowerRune> solve(const BuffObservation & observation) const;

  std::optional<PowerRune> solve(
    const std::optional<BuffObservation> & observation) const;

  std::vector<PowerRune> solve_all(const std::vector<BuffObservation> & observations) const;

  // 调试用
  cv::Point2f point_buff2pixel(cv::Point3f x);

  std::optional<std::vector<cv::Point2f>> reproject_pnp_points() const;

  std::vector<cv::Point2f> reproject_buff(
    const Eigen::Vector3d & xyz_in_world, const Eigen::Matrix3d & R_buff2world) const;

private:
  cv::Mat camera_matrix_;
  cv::Mat distort_coeffs_;
  Eigen::Matrix3d R_gimbal2imubody_;
  Eigen::Matrix3d R_camera2gimbal_;
  Eigen::Vector3d t_camera2gimbal_;
  Eigen::Matrix3d R_gimbal2world_;

  mutable cv::Vec3d rvec_, tvec_;
  mutable bool has_pnp_solution_ = false;
  struct PoseCache
  {
    cv::Vec3d rvec;
    cv::Vec3d tvec;
  };
  mutable std::unordered_map<int, PoseCache> pose_cache_;

  double full_reprojection_gate_px_ = 6.0;
  double target_center_reprojection_gate_px_ = 6.0;
  double fan_center_reprojection_gate_px_ = 8.0;
  double partial_four_center_gate_px_ = 8.0;
  double partial_four_angle_gate_rad_ = 15.0 / 57.3;

  const std::vector<cv::Point3f> PNP_OBJECT_POINTS = {
    cv::Point3f(0.0f, -0.095f, 0.0f), cv::Point3f(0.095f, 0.0f, 0.0f),
    cv::Point3f(0.0f, 0.095f, 0.0f), cv::Point3f(-0.095f, 0.0f, 0.0f),
    cv::Point3f(-0.030f, 0.191f, 0.0f), cv::Point3f(0.030f, 0.191f, 0.0f),
    cv::Point3f(0.030f, 0.521f, 0.0f), cv::Point3f(-0.030f, 0.521f, 0.0f)};

  std::vector<cv::Point3f> reproject_object_points() const;

  // 函数：生成绕x轴旋转的旋转矩阵
  cv::Matx33f rotation_matrix(double angle) const;

  // 函数：旋转点并填充到 OBJECT_POINTS 中
  void compute_rotated_points(std::vector<std::vector<cv::Point3f>> & object_points);
};
}  // namespace auto_buff
#endif  // AUTO_AIM__SOLVER_HPP
