#include "uv_model.hpp"

#include <algorithm>
#include <cmath>

namespace auto_aim
{
namespace
{
constexpr double kPi = 3.14159265358979323846;

/** @brief 把角度归一化到 [-pi, pi] @param angle 输入角度 @return 归一化角度 */
double limit_rad(double angle)
{
  while (angle > kPi) angle -= 2 * kPi;
  while (angle <= -kPi) angle += 2 * kPi;
  return angle;
}

/** @brief 由灯条上下端点构造 UVL 特征对四个像素分量的雅可比（4x4） @param top 上端点 @param bottom 下端点 @return 雅可比，行 = [角度, 中心x, 中心y, 长度]，列 = [A.x, A.y, B.x, B.y] */
Eigen::Matrix<double, 4, 4> uvl_jacobian_from_corners(
  const Eigen::Vector2d & top, const Eigen::Vector2d & bottom)
{
  const double dx = top.x() - bottom.x();
  const double dy = top.y() - bottom.y();
  const double d2 = std::max(dx * dx + dy * dy, 1e-9);
  const double length = std::sqrt(d2);

  Eigen::Matrix<double, 4, 4> J;
  J << -dy / d2, dx / d2, dy / d2, -dx / d2,  // angle
    0.5, 0.0, 0.5, 0.0,                       // center x
    0.0, 0.5, 0.0, 0.5,                       // center y
    dx / length, dy / length, -dx / length, -dy / length;  // length
  return J;
}
}  // namespace

void UvModel::configure(const UvConfig & config, int armor_num, ArmorName armor_name)
{
  config_ = config;
  armor_num_ = armor_num > 0 ? armor_num : 1;
  armor_name_ = armor_name;
}

double UvModel::armor_width(ArmorType type)
{
  return type == ArmorType::big ? kBigArmorWidth : kSmallArmorWidth;
}

double UvModel::armor_pitch() const
{
  return armor_name_ == ArmorName::outpost ? -15.0 * kPi / 180.0 : 15.0 * kPi / 180.0;
}

double UvModel::tower_height_multiplier(int armor_id) const
{
  // 与 RVfromFYT::h_armor_xyz 共用同一份实现：观测几何与瞄准几何必须一致
  return tower_armor_height_multiplier(tower_heights_, armor_id);
}

std::array<Eigen::Vector3d, 4> UvModel::armor_corner_local(ArmorType type)
{
  const double w = armor_width(type) / 2.0;
  const double l = lightbar_length() / 2.0;
  // 顺序与 Armor::points / SMALL_ARMOR_POINTS 一致：[左上, 右上, 右下, 左下]
  return {Eigen::Vector3d(0, +w, +l), Eigen::Vector3d(0, -w, +l), Eigen::Vector3d(0, -w, -l),
          Eigen::Vector3d(0, +w, -l)};
}

double UvModel::armor_yaw(const Eigen::VectorXd & state, int armor_id) const
{
  return limit_rad(state[6] + armor_id * 2.0 * kPi / armor_num_);
}

Eigen::Matrix3d UvModel::armor_rotation(const Eigen::VectorXd & state, int armor_id) const
{
  const double yaw = armor_yaw(state, armor_id);
  const double pitch = armor_pitch();
  const double cy = std::cos(yaw), sy = std::sin(yaw);
  const double cp = std::cos(pitch), sp = std::sin(pitch);

  Eigen::Matrix3d R;
  R << cy * cp, -sy, cy * sp,  //
    sy * cp, cy, sy * sp,      //
    -sp, 0, cp;
  return R;
}

Eigen::Vector3d UvModel::armor_position(const Eigen::VectorXd & state, int armor_id) const
{
  const double angle = armor_yaw(state, armor_id);
  const bool use_alternate_radius = armor_num_ == 4 && (armor_id == 1 || armor_id == 3);
  const double radius = use_alternate_radius ? state[8] + state[9] : state[8];

  const double height_offset =
    armor_name_ == ArmorName::outpost
      ? state[10] * tower_height_multiplier(armor_id)
      : (use_alternate_radius ? state[10] : 0.0);

  return Eigen::Vector3d(
    state[0] - radius * std::cos(angle), state[2] - radius * std::sin(angle),
    state[4] + height_offset);
}

std::array<Eigen::Vector3d, 4> UvModel::armor_corners_world(
  const Eigen::VectorXd & state, int armor_id, ArmorType type) const
{
  const auto local = armor_corner_local(type);
  const Eigen::Matrix3d R = armor_rotation(state, armor_id);
  const Eigen::Vector3d center = armor_position(state, armor_id);

  std::array<Eigen::Vector3d, 4> out;
  for (int i = 0; i < 4; i++) out[i] = center + R * local[i];
  return out;
}

Eigen::Vector3d UvModel::world_to_camera(const Eigen::Vector3d & point_world) const
{
  return geometry_.R_camera2gimbal.transpose() *
         (geometry_.R_gimbal2world.transpose() * point_world - geometry_.t_camera2gimbal);
}

double UvModel::facing_cos(const Eigen::VectorXd & state, int armor_id) const
{
  const Eigen::Vector3d armor = armor_position(state, armor_id);
  const Eigen::Vector3d camera = geometry_.R_gimbal2world * geometry_.t_camera2gimbal;

  Eigen::Vector2d to_camera(camera.x() - armor.x(), camera.y() - armor.y());
  if (to_camera.norm() < 1e-6) return 0.0;
  to_camera.normalize();

  // 工程约定下装甲板坐标系 x 轴（R 第一列）指向背离相机方向，朝相机的法线取其反向
  const Eigen::Matrix3d R = armor_rotation(state, armor_id);
  Eigen::Vector2d normal(-R(0, 0), -R(1, 0));
  if (normal.norm() < 1e-6) return 0.0;
  normal.normalize();

  return normal.dot(to_camera);
}

bool UvModel::project_point(
  const Eigen::Vector3d & point_camera, Eigen::Vector2d & uv,
  Eigen::Matrix<double, 2, 3> & jacobian) const
{
  const double Z = point_camera.z();
  if (!(Z > kMinDepth) || !std::isfinite(Z)) return false;

  const double fx = geometry_.camera_matrix(0, 0);
  const double fy = geometry_.camera_matrix(1, 1);
  const double cx = geometry_.camera_matrix(0, 2);
  const double cy = geometry_.camera_matrix(1, 2);

  const double x = point_camera.x() / Z;
  const double y = point_camera.y() / Z;
  if (!std::isfinite(x) || !std::isfinite(y)) return false;

  const double k1 = geometry_.distort_coeffs.size() > 0 ? geometry_.distort_coeffs[0] : 0.0;
  const double k2 = geometry_.distort_coeffs.size() > 1 ? geometry_.distort_coeffs[1] : 0.0;
  const double p1 = geometry_.distort_coeffs.size() > 2 ? geometry_.distort_coeffs[2] : 0.0;
  const double p2 = geometry_.distort_coeffs.size() > 3 ? geometry_.distort_coeffs[3] : 0.0;
  const double k3 = geometry_.distort_coeffs.size() > 4 ? geometry_.distort_coeffs[4] : 0.0;

  const double r2 = x * x + y * y;
  const double radial = 1.0 + k1 * r2 + k2 * r2 * r2 + k3 * r2 * r2 * r2;
  const double dradial_dr2 = k1 + 2.0 * k2 * r2 + 3.0 * k3 * r2 * r2;

  const double xd = x * radial + 2.0 * p1 * x * y + p2 * (r2 + 2.0 * x * x);
  const double yd = y * radial + p1 * (r2 + 2.0 * y * y) + 2.0 * p2 * x * y;
  uv = Eigen::Vector2d(fx * xd + cx, fy * yd + cy);
  if (!std::isfinite(uv.x()) || !std::isfinite(uv.y())) return false;

  Eigen::Matrix2d distortion;
  distortion << radial + x * (2.0 * x * dradial_dr2) + 2.0 * p1 * y + p2 * (6.0 * x),
    x * (2.0 * y * dradial_dr2) + 2.0 * p1 * x + p2 * (2.0 * y),
    y * (2.0 * x * dradial_dr2) + p1 * (2.0 * x) + 2.0 * p2 * y,
    radial + y * (2.0 * y * dradial_dr2) + p1 * (6.0 * y) + 2.0 * p2 * x;

  Eigen::Matrix<double, 2, 3> perspective;
  perspective << 1.0 / Z, 0.0, -x / Z,  //
    0.0, 1.0 / Z, -y / Z;

  Eigen::Matrix2d focal = Eigen::Matrix2d::Zero();
  focal(0, 0) = fx;
  focal(1, 1) = fy;

  jacobian = focal * distortion * perspective;
  return true;
}

Eigen::Matrix<double, 3, 11> UvModel::corner_world_jacobian(
  const Eigen::VectorXd & state, int armor_id, const Eigen::Vector3d & local, ArmorType) const
{
  Eigen::Matrix<double, 3, 11> J = Eigen::Matrix<double, 3, 11>::Zero();

  const double angle = armor_yaw(state, armor_id);
  const bool use_alternate_radius = armor_num_ == 4 && (armor_id == 1 || armor_id == 3);
  const double radius = use_alternate_radius ? state[8] + state[9] : state[8];
  const double height_multiplier = armor_name_ == ArmorName::outpost
                                     ? tower_height_multiplier(armor_id)
                                     : (use_alternate_radius ? 1.0 : 0.0);

  // 中心位置
  J(0, 0) = 1.0;
  J(1, 2) = 1.0;
  J(2, 4) = 1.0;

  // 半径方向
  const Eigen::Vector3d d_radius(-std::cos(angle), -std::sin(angle), 0.0);
  J.col(8) += d_radius;
  if (use_alternate_radius) J.col(9) += d_radius;

  // 高度偏移
  J(2, 10) = height_multiplier;

  // 偏航角：圆心绕转的导数 + 姿态旋转的导数
  const Eigen::Vector3d d_center(radius * std::sin(angle), -radius * std::cos(angle), 0.0);
  const double pitch = armor_pitch();
  const double cy = std::cos(angle), sy = std::sin(angle);
  const double cp = std::cos(pitch), sp = std::sin(pitch);
  Eigen::Matrix3d dR;
  dR << -sy * cp, -cy, -sy * sp,  //
    cy * cp, -sy, cy * sp,        //
    0.0, 0.0, 0.0;
  J.col(6) += d_center + dR * local;

  return J;
}

bool UvModel::observation_valid(const std::vector<cv::Point2f> & points)
{
  if (points.size() != 4) return false;
  for (const auto & p : points) {
    if (!std::isfinite(p.x) || !std::isfinite(p.y)) return false;
  }
  return true;
}

void UvModel::lightbar_to_uvl(
  const Eigen::Vector2d & top, const Eigen::Vector2d & bottom, Eigen::Matrix<double, 4, 1> & out)
{
  const double dx = top.x() - bottom.x();
  const double dy = top.y() - bottom.y();
  const double length = std::sqrt(dx * dx + dy * dy);

  out[0] = std::atan2(dx, -dy);
  out[1] = 0.5 * (top.x() + bottom.x());
  out[2] = 0.5 * (top.y() + bottom.y());
  out[3] = length;
}

UvModel::Observation UvModel::observation(const std::vector<cv::Point2f> & points)
{
  Observation z = Observation::Zero();
  if (!observation_valid(points)) return z;

  // 左灯条 = 左上/左下，右灯条 = 右上/右下
  const Eigen::Vector2d lt(points[0].x, points[0].y), lb(points[3].x, points[3].y);
  const Eigen::Vector2d rt(points[1].x, points[1].y), rb(points[2].x, points[2].y);
  Eigen::Matrix<double, 4, 1> left, right;
  lightbar_to_uvl(lt, lb, left);
  lightbar_to_uvl(rt, rb, right);
  z << left, right;
  return z;
}

UvModel::Observation UvModel::predict(
  const Eigen::VectorXd & state, int armor_id, ArmorType type) const
{
  return predict_with_rotation(state, armor_id, type, geometry_.R_gimbal2world);
}

UvModel::Observation UvModel::predict_with_rotation(
  const Eigen::VectorXd & state, int armor_id, ArmorType type,
  const Eigen::Matrix3d & R_gimbal2world) const
{
  Observation z = Observation::Zero();
  if (!geometry_.valid) return z;

  CameraGeometry geometry = geometry_;
  geometry.R_gimbal2world = R_gimbal2world;

  const auto local = armor_corner_local(type);
  const Eigen::Matrix3d R = armor_rotation(state, armor_id);
  const Eigen::Vector3d center = armor_position(state, armor_id);

  std::array<Eigen::Vector2d, 4> uv;
  for (int i = 0; i < 4; i++) {
    const Eigen::Vector3d point_world = center + R * local[i];
    const Eigen::Vector3d point_camera =
      geometry.R_camera2gimbal.transpose() *
      (geometry.R_gimbal2world.transpose() * point_world - geometry.t_camera2gimbal);
    Eigen::Matrix<double, 2, 3> unused;
    if (!project_point(point_camera, uv[i], unused)) return Observation::Zero();
  }

  Eigen::Matrix<double, 4, 1> left, right;
  lightbar_to_uvl(uv[0], uv[3], left);
  lightbar_to_uvl(uv[1], uv[2], right);
  z << left, right;
  return z;
}

UvModel::Jacobian UvModel::jacobian(
  const Eigen::VectorXd & state, int armor_id, ArmorType type) const
{
  Jacobian H = Jacobian::Zero();
  if (!geometry_.valid) return H;

  const auto local = armor_corner_local(type);
  const Eigen::Matrix3d R = armor_rotation(state, armor_id);
  const Eigen::Vector3d center = armor_position(state, armor_id);

  // 每个角点的 2x11 像素雅可比
  Eigen::Matrix<double, 2, 11> J_corner[4];
  Eigen::Vector2d pixel[4];
  for (int i = 0; i < 4; i++) {
    const Eigen::Vector3d point_world = center + R * local[i];
    Eigen::Matrix<double, 2, 3> J_proj;
    if (!project_point(world_to_camera(point_world), pixel[i], J_proj)) return Jacobian::Zero();

    const Eigen::Matrix<double, 3, 11> J_world =
      corner_world_jacobian(state, armor_id, local[i], type);
    // 世界 -> 相机：P_cam = R_cg^T (R_gw^T P - t_cg)
    const Eigen::Matrix3d R_world2camera = geometry_.R_camera2gimbal.transpose() *
                                           geometry_.R_gimbal2world.transpose();
    J_corner[i] = J_proj * R_world2camera * J_world;
  }

  const Eigen::Matrix<double, 4, 4> J_left = uvl_jacobian_from_corners(pixel[0], pixel[3]);
  const Eigen::Matrix<double, 4, 4> J_right = uvl_jacobian_from_corners(pixel[1], pixel[2]);

  Eigen::Matrix<double, 4, 11> corners_left;
  corners_left << J_corner[0], J_corner[3];  // (A.x,A.y,B.x,B.y)
  Eigen::Matrix<double, 4, 11> corners_right;
  corners_right << J_corner[1], J_corner[2];

  H.topRows<4>() = J_left * corners_left;
  H.bottomRows<4>() = J_right * corners_right;
  return H;
}

double UvModel::attitude_sigma_px() const
{
  const double fx = geometry_.camera_matrix(0, 0);
  const double fy = geometry_.camera_matrix(1, 1);
  return 0.5 * (fx + fy) * config_.sigma_attitude_deg * kPi / 180.0;
}

UvModel::Noise UvModel::diagonal_noise(const Observation & z) const
{
  Noise R = Noise::Zero();

  for (int light = 0; light < 2; light++) {
    const int base = light * 4;
    const double length = std::max(z[base + 3], 1.0);

    // 端点噪声：检测噪声 + 与灯条像素长度成比例的尺度项
    const double sigma_endpoint =
      std::sqrt(config_.sigma_px * config_.sigma_px + std::pow(config_.sigma_len_ratio * length, 2.0));

    // 中心：检测噪声 + 姿态误差中"独立"的那一部分
    const double sigma_independent =
      std::sqrt(std::max(0.0, 1.0 - config_.attitude_common_ratio * config_.attitude_common_ratio)) *
      attitude_sigma_px();
    const double sigma_center =
      std::sqrt(sigma_endpoint * sigma_endpoint / 2.0 + sigma_independent * sigma_independent);

    // 长度：差分量，不受共模平移影响
    const double sigma_length = sigma_endpoint * std::sqrt(2.0);

    // 角度：端点噪声在灯条法向上的投影
    const double angle_from_pixel = sigma_endpoint * std::sqrt(2.0) / length;
    const double sigma_angle =
      std::sqrt(config_.sigma_angle * config_.sigma_angle + angle_from_pixel * angle_from_pixel);

    R(base + 0, base + 0) = sigma_angle * sigma_angle;
    R(base + 1, base + 1) = sigma_center * sigma_center;
    R(base + 2, base + 2) = sigma_center * sigma_center;
    R(base + 3, base + 3) = sigma_length * sigma_length;
  }
  return R;
}

Eigen::Matrix<double, UvModel::kDim, 3> UvModel::attitude_jacobian(
  const Eigen::VectorXd & state, int armor_id, ArmorType type) const
{
  Eigen::Matrix<double, kDim, 3> J = Eigen::Matrix<double, kDim, 3>::Zero();
  if (!geometry_.valid) return J;

  constexpr double kDelta = 1e-4;  // rad
  for (int axis = 0; axis < 3; axis++) {
    Eigen::Vector3d axis_vector = Eigen::Vector3d::Zero();
    axis_vector[axis] = 1.0;

    const Eigen::Matrix3d perturbation =
      Eigen::AngleAxisd(kDelta, axis_vector).toRotationMatrix();
    const Observation plus = predict_with_rotation(
      state, armor_id, type, geometry_.R_gimbal2world * perturbation);
    const Observation minus = predict_with_rotation(
      state, armor_id, type, geometry_.R_gimbal2world * perturbation.transpose());

    Eigen::VectorXd column = (plus - minus) / (2.0 * kDelta);
    for (int k = 0; k < kDim; k += 4) column[k] = std::remainder(column[k], 2 * kPi);
    J.col(axis) = column;
  }
  return J;
}

UvModel::Noise UvModel::noise(
  const Eigen::VectorXd & state, int armor_id, ArmorType type, const Observation & z) const
{
  Noise R = diagonal_noise(z);
  const auto J_att = attitude_jacobian(state, armor_id, type);
  // J_att 的量纲是 px/rad，因此必须乘姿态误差方差 (rad^2) 而不是像素方差
  const double sigma = config_.attitude_common_ratio * attitude_sigma_rad();
  R.noalias() += sigma * sigma * (J_att * J_att.transpose());
  return R;
}

std::vector<cv::Point2f> UvModel::project_corners(
  const Eigen::VectorXd & state, int armor_id, ArmorType type) const
{
  std::vector<cv::Point2f> out;
  if (!geometry_.valid) return out;

  const auto local = armor_corner_local(type);
  const Eigen::Matrix3d R = armor_rotation(state, armor_id);
  const Eigen::Vector3d center = armor_position(state, armor_id);

  out.reserve(4);
  for (int i = 0; i < 4; i++) {
    const Eigen::Vector3d point_world = center + R * local[i];
    Eigen::Vector2d pixel;
    Eigen::Matrix<double, 2, 3> unused;
    if (!project_point(world_to_camera(point_world), pixel, unused)) return {};
    out.emplace_back(static_cast<float>(pixel.x()), static_cast<float>(pixel.y()));
  }
  return out;
}

}  // namespace auto_aim
