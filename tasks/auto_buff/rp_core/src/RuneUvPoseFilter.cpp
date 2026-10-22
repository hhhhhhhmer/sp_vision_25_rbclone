#include "RuneUvPoseFilter.hpp"

#include <algorithm>
#include <cmath>

namespace
{
constexpr double kPi = 3.14159265358979323846;

/** @brief 角度转弧度 @param degree 角度 @return 弧度 */
double deg2rad(double degree) { return degree * kPi / 180.0; }

/** @brief 由旋转向量构造旋转矩阵 @param rvec 旋转向量 @return 旋转矩阵 */
Eigen::Matrix3d rotation_from_rvec(const Eigen::Vector3d & rvec)
{
  const double angle = rvec.norm();
  if (!std::isfinite(angle) || angle < 1e-12) return Eigen::Matrix3d::Identity();
  return Eigen::AngleAxisd(angle, rvec / angle).toRotationMatrix();
}

/** @brief 由旋转矩阵构造旋转向量 @param rotation 旋转矩阵 @return 旋转向量 */
Eigen::Vector3d rvec_from_rotation(const Eigen::Matrix3d & rotation)
{
  Eigen::AngleAxisd angle_axis(rotation);
  if (!std::isfinite(angle_axis.angle()) || angle_axis.angle() < 1e-12) {
    return Eigen::Vector3d::Zero();
  }
  return angle_axis.angle() * angle_axis.axis();
}

/** @brief 把矩阵投影回 SO(3)，避免数值累计误差 @param rotation 输入旋转 @return 正交化后的旋转 */
Eigen::Matrix3d orthonormalize(const Eigen::Matrix3d & rotation)
{
  Eigen::JacobiSVD<Eigen::Matrix3d> svd(
    rotation, Eigen::ComputeFullU | Eigen::ComputeFullV);
  Eigen::Matrix3d result = svd.matrixU() * svd.matrixV().transpose();
  if (result.determinant() < 0.0) {
    Eigen::Matrix3d u = svd.matrixU();
    u.col(2) *= -1.0;
    result = u * svd.matrixV().transpose();
  }
  return result;
}
/** @brief 把一条 UV 观测拼成向量：[五点关键点, 四个锚框点] @param observation 观测 @param include_center 是否包含中心 R 点 @return 观测向量，非法时为空 */
Eigen::VectorXd observation_vector(
  const RuneUvPoseFilter::Observation & observation, bool include_center)
{
  if (!observation.has_anchors) return {};
  const int anchor_base = include_center ? 10 : 8;
  Eigen::VectorXd z(anchor_base + 8);

  z.segment<2>(0) = Eigen::Vector2d(observation.keypoints[0].x, observation.keypoints[0].y);
  z.segment<2>(2) = Eigen::Vector2d(observation.keypoints[1].x, observation.keypoints[1].y);
  if (include_center) {
    z.segment<2>(4) = Eigen::Vector2d(observation.keypoints[2].x, observation.keypoints[2].y);
    z.segment<2>(6) = Eigen::Vector2d(observation.keypoints[3].x, observation.keypoints[3].y);
    z.segment<2>(8) = Eigen::Vector2d(observation.keypoints[4].x, observation.keypoints[4].y);
  } else {
    z.segment<2>(4) = Eigen::Vector2d(observation.keypoints[3].x, observation.keypoints[3].y);
    z.segment<2>(6) = Eigen::Vector2d(observation.keypoints[4].x, observation.keypoints[4].y);
  }
  for (int i = 0; i < 4; ++i) {
    z.segment<2>(anchor_base + 2 * i) =
      Eigen::Vector2d(observation.anchors[i].x, observation.anchors[i].y);
  }
  if (!z.allFinite()) return {};
  return z;
}

/** @brief 某一行观测对应的像素噪声标准差 @param row 行号 @param include_center 是否包含中心 R 点 @param config 配置 @return 标准差 */
double row_sigma(int row, bool include_center, const RuneUvPoseConfig & config)
{
  const int anchor_base = include_center ? 10 : 8;
  if (row >= anchor_base) return config.anchor_sigma_px;
  if (include_center && row >= 4 && row < 6) return config.sigma_center_px;
  return config.sigma_px;
}

}  // namespace

void RuneUvPoseFilter::configure(
  const RuneUvPoseConfig & config, const cv::Matx33d & camera_matrix,
  const cv::Matx<double, 1, 5> & distortion)
{
  config_ = config;
  camera_matrix_ = camera_matrix;
  distortion_ = distortion;
  config_.circle_radius = std::max(1e-3, config_.circle_radius);
  config_.circle_center_radius = std::max(1e-3, config_.circle_center_radius);
  reset();
}

void RuneUvPoseFilter::reset()
{
  R_car_from_plane_ = Eigen::Matrix3d::Identity();
  t_car_from_plane_ = Eigen::Vector3d::Zero();
  angular_velocity_ = 0.0;
  P_ = Eigen::Matrix<double, 7, 7>::Identity();
  initialized_ = false;
  diagnostics_ = Diagnostics{};
}

bool RuneUvPoseFilter::initialize(
  const CameraPose & camera_pose, const Eigen::Matrix<double, 6, 1> & pose_rvec_tvec,
  bool preserve_angular_velocity)
{
  if (!pose_rvec_tvec.allFinite() || !camera_pose.R_car_from_camera.allFinite() ||
      !camera_pose.t_car_from_camera.allFinite()) {
    return false;
  }
  if (!set_pose_from_camera(camera_pose, pose_rvec_tvec)) return false;

  // PnP 兜底重建时保留上一帧的角速度；首次建立目标/模式切换时仍从 0 开始。
  const double preserved_angular_velocity =
    (preserve_angular_velocity && initialized_ && std::isfinite(angular_velocity_))
      ? angular_velocity_
      : 0.0;

  const double sigma_rotation = deg2rad(config_.initial_rotation_sigma_deg);
  const double sigma_translation = config_.initial_translation_sigma_m;
  P_.setZero();
  P_.diagonal().head<3>().array() = sigma_rotation * sigma_rotation;
  P_.diagonal().segment<3>(3).array() = sigma_translation * sigma_translation;
  P_(6, 6) = config_.initial_angular_velocity_sigma * config_.initial_angular_velocity_sigma;
  angular_velocity_ = preserved_angular_velocity;

  const int reinitialize_count = diagnostics_.reinitialize_count + 1;
  diagnostics_ = Diagnostics{};
  diagnostics_.initialized = true;
  diagnostics_.reinitialize_count = reinitialize_count;
  diagnostics_.angular_velocity = angular_velocity_;

  initialized_ = true;
  return true;
}

bool RuneUvPoseFilter::set_pose_from_camera(
  const CameraPose & camera_pose, const Eigen::Matrix<double, 6, 1> & pose)
{
  const Eigen::Matrix3d R_camera_from_plane = rotation_from_rvec(pose.head<3>());
  const Eigen::Vector3d t_camera_from_plane = pose.tail<3>();
  if (!R_camera_from_plane.allFinite() || !t_camera_from_plane.allFinite()) return false;

  R_car_from_plane_ =
    orthonormalize(camera_pose.R_car_from_camera * R_camera_from_plane);
  t_car_from_plane_ =
    camera_pose.R_car_from_camera * t_camera_from_plane + camera_pose.t_car_from_camera;
  return R_car_from_plane_.allFinite() && t_car_from_plane_.allFinite();
}

Eigen::Matrix<double, 6, 1> RuneUvPoseFilter::camera_pose_from_state(
  const CameraPose & camera_pose) const
{
  const Eigen::Matrix3d R_camera_from_plane =
    orthonormalize(camera_pose.R_car_from_camera.transpose() * R_car_from_plane_);
  const Eigen::Vector3d t_camera_from_plane =
    camera_pose.R_car_from_camera.transpose() *
    (t_car_from_plane_ - camera_pose.t_car_from_camera);

  Eigen::Matrix<double, 6, 1> pose;
  pose.head<3>() = rvec_from_rotation(R_camera_from_plane);
  pose.tail<3>() = t_camera_from_plane;
  return pose;
}

bool RuneUvPoseFilter::predict(const CameraPose & camera_pose, double dt)
{
  if (!initialized_) return false;
  if (!camera_pose.R_car_from_camera.allFinite() ||
      !camera_pose.t_car_from_camera.allFinite()) {
    return false;
  }
  if (!(dt >= 0.0) || !std::isfinite(dt) || dt > config_.max_prediction_gap_s) {
    // 间隔过大时不信任上一帧位姿，让调用方走 PnP 重建。
    return false;
  }

  // 状态已经在固定 RP 系中，相机/云台转动只影响观测投影，不参与状态传播。
  // 这里只让符叶绕自身 z 轴按角速度旋转。
  const double rotation_step = angular_velocity_ * dt;
  const Eigen::Matrix3d R_blade_step =
    Eigen::AngleAxisd(rotation_step, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  R_car_from_plane_ = orthonormalize(R_car_from_plane_ * R_blade_step);

  // 误差状态 [dtheta(3), dc(3), dw(1)]。
  // 右乘误差在预测后要逆旋转回新本体坐标系；角速度通过 dtheta 的雅可比耦合进来。
  // 平移误差 dc 定义在固定 RP 系里，符叶自转不会改变平移，所以平移块保持单位阵。
  Eigen::Matrix<double, 7, 7> F = Eigen::Matrix<double, 7, 7>::Identity();
  F.topLeftCorner<3, 3>() = R_blade_step.transpose();
  F.block<3, 1>(0, 6) = Eigen::Vector3d::UnitZ() * dt;

  P_ = F * P_ * F.transpose();

  // 过程噪声按 sqrt(dt) 缩放；30Hz 时就是配置里的每帧噪声。
  const double scale = std::sqrt(std::max(dt, 1e-3) / (1.0 / 30.0));
  const double sigma_rotation = deg2rad(config_.process_rotation_sigma_deg) * scale;
  const double sigma_translation = config_.process_translation_sigma_m * scale;
  const double sigma_angular_velocity = config_.process_angular_velocity_sigma * scale;
  P_.diagonal().head<3>().array() += sigma_rotation * sigma_rotation;
  P_.diagonal().segment<3>(3).array() += sigma_translation * sigma_translation;
  P_(6, 6) += sigma_angular_velocity * sigma_angular_velocity;

  sanitize_covariance();
  return true;
}

RuneUvPoseFilter::CircleModel RuneUvPoseFilter::circle_model(int location_num) const
{
  CircleModel model;
  model.radius = config_.circle_radius;
  const int location = std::clamp(location_num, 1, 5);
  const double angle = (location - 1) * 2.0 * kPi / 5.0;
  model.rotation = Eigen::AngleAxisd(angle, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  model.center_local =
    model.rotation * Eigen::Vector3d(0.0, -config_.circle_center_radius, 0.0);
  return model;
}

bool RuneUvPoseFilter::project(
  const CameraPose & camera_pose, const Eigen::Vector3d & point_car,
  Eigen::Vector2d & uv) const
{
  const Eigen::Vector3d point_camera =
    camera_pose.R_car_from_camera.transpose() * (point_car - camera_pose.t_car_from_camera);
  const double Z = point_camera.z();
  if (!(Z > 1e-6) || !std::isfinite(Z)) return false;

  const double x = point_camera.x() / Z;
  const double y = point_camera.y() / Z;
  if (!std::isfinite(x) || !std::isfinite(y)) return false;

  const double fx = camera_matrix_(0, 0);
  const double fy = camera_matrix_(1, 1);
  const double cx = camera_matrix_(0, 2);
  const double cy = camera_matrix_(1, 2);

  const double k1 = distortion_(0);
  const double k2 = distortion_(1);
  const double p1 = distortion_(2);
  const double p2 = distortion_(3);
  const double k3 = distortion_(4);

  const double r2 = x * x + y * y;
  const double radial = 1.0 + k1 * r2 + k2 * r2 * r2 + k3 * r2 * r2 * r2;
  const double xd = x * radial + 2.0 * p1 * x * y + p2 * (r2 + 2.0 * x * x);
  const double yd = y * radial + p1 * (r2 + 2.0 * y * y) + 2.0 * p2 * x * y;
  uv = Eigen::Vector2d(fx * xd + cx, fy * yd + cy);
  return std::isfinite(uv.x()) && std::isfinite(uv.y());
}

bool RuneUvPoseFilter::predict_plate_points_with_pose(
  const CameraPose & camera_pose, const CircleModel & model,
  const Eigen::Matrix3d & R_car_from_plane, const Eigen::Vector3d & t_car_from_plane,
  std::array<Eigen::Vector2d, 4> & out) const
{
  // 网络四点的顺序固定在符叶局部系：外侧(0,-r)、左侧(-r,0)、右侧(+r,0)、内侧(0,+r)。
  // 注意：位置号带来的旋转必须同时作用在靶心中心与这四个偏移上，否则不同位置号的
  // 符叶会被投到同一朝向（大符第二片会整体差 90°）。
  const std::array<Eigen::Vector3d, 4> offsets{{
    Eigen::Vector3d(0.0, -model.radius, 0.0),
    Eigen::Vector3d(-model.radius, 0.0, 0.0),
    Eigen::Vector3d(model.radius, 0.0, 0.0),
    Eigen::Vector3d(0.0, model.radius, 0.0),
  }};
  for (int i = 0; i < 4; ++i) {
    const Eigen::Vector3d local =
      model.center_local + model.rotation * offsets[static_cast<std::size_t>(i)];
    const Eigen::Vector3d point_car = t_car_from_plane + R_car_from_plane * local;
    if (!project(camera_pose, point_car, out[static_cast<std::size_t>(i)])) return false;
  }
  return true;
}

bool RuneUvPoseFilter::predict_plate_points(
  const CameraPose & camera_pose, const CircleModel & model,
  std::array<Eigen::Vector2d, 4> & out) const
{
  return predict_plate_points_with_pose(
    camera_pose, model, R_car_from_plane_, t_car_from_plane_, out);
}

bool RuneUvPoseFilter::predict_blade_with_pose(
  const CameraPose & camera_pose, const CircleModel & model, bool include_center,
  const Eigen::Matrix3d & R_car_from_plane, const Eigen::Vector3d & t_car_from_plane,
  Eigen::VectorXd & out) const
{
  std::array<Eigen::Vector2d, 4> extrema;
  if (!predict_plate_points_with_pose(
        camera_pose, model, R_car_from_plane, t_car_from_plane, extrema)) {
    return false;
  }

  // 中心 R 点就是符平面原点，不是靶心环中心。
  Eigen::Vector2d center;
  if (!project(camera_pose, t_car_from_plane, center)) {
    return false;
  }

  const int anchor_base = include_center ? 10 : 8;
  out.resize(anchor_base + 8);
  if (include_center) {
    out.segment<2>(0) = extrema[0];
    out.segment<2>(2) = extrema[1];
    out.segment<2>(4) = center;
    out.segment<2>(6) = extrema[2];
    out.segment<2>(8) = extrema[3];
  } else {
    out.segment<2>(0) = extrema[0];
    out.segment<2>(2) = extrema[1];
    out.segment<2>(4) = extrema[2];
    out.segment<2>(6) = extrema[3];
  }

  // 整片符叶锚框：与 rp_core 的 anchor_inactive_points_world 一致。
  // 它提供比四个圆点更长的基线，用来约束单圆点观测中"深度-法向"的弱方向。
  static const std::array<Eigen::Vector3d, 4> kAnchorPoints{{
    Eigen::Vector3d(-0.15, -0.85, 0.0),
    Eigen::Vector3d(-0.15, -0.174, 0.0),
    Eigen::Vector3d(0.15, -0.174, 0.0),
    Eigen::Vector3d(0.15, -0.85, 0.0),
  }};
  for (int i = 0; i < 4; ++i) {
    const Eigen::Vector3d local = model.rotation * kAnchorPoints[static_cast<std::size_t>(i)];
    const Eigen::Vector3d point_car = t_car_from_plane + R_car_from_plane * local;
    Eigen::Vector2d uv;
    if (!project(camera_pose, point_car, uv)) return false;
    out.segment<2>(anchor_base + 2 * i) = uv;
  }
  return out.allFinite();
}

bool RuneUvPoseFilter::predict_blade(
  const CameraPose & camera_pose, const CircleModel & model, bool include_center,
  Eigen::VectorXd & out) const
{
  return predict_blade_with_pose(
    camera_pose, model, include_center, R_car_from_plane_, t_car_from_plane_, out);
}

bool RuneUvPoseFilter::predict_blade_perturbed_with_pose(
  const CameraPose & camera_pose, const CircleModel & model, bool include_center,
  const Eigen::Matrix3d & R_base, const Eigen::Vector3d & t_base,
  const Eigen::Vector3d & dtheta, const Eigen::Vector3d & dt, Eigen::VectorXd & out) const
{
  const Eigen::Matrix3d R_perturbed = orthonormalize(R_base * rotation_from_rvec(dtheta));
  const Eigen::Vector3d t_perturbed = t_base + dt;
  return predict_blade_with_pose(
    camera_pose, model, include_center, R_perturbed, t_perturbed, out);
}

bool RuneUvPoseFilter::predict_blade_perturbed(
  const CameraPose & camera_pose, const CircleModel & model, bool include_center,
  const Eigen::Vector3d & dtheta, const Eigen::Vector3d & dt, Eigen::VectorXd & out) const
{
  return predict_blade_perturbed_with_pose(
    camera_pose, model, include_center, R_car_from_plane_, t_car_from_plane_, dtheta, dt, out);
}

bool RuneUvPoseFilter::update(
  const CameraPose & camera_pose, const std::vector<Observation> & observations)
{
  if (!initialized_ || observations.empty()) {
    diagnostics_.updated = false;
    return false;
  }

  bool center_used = false;
  int accepted = 0;
  diagnostics_.accepted_blades = 0;
  diagnostics_.rejected_blades = 0;
  diagnostics_.reprojection_rmse_px = 0.0;
  double residual_squared_sum = 0.0;
  int residual_count = 0;
  double max_nis = 0.0;

  for (const Observation & observation : observations) {
    const bool include_center = !center_used;
    const Eigen::VectorXd z = observation_vector(observation, include_center);
    if (z.size() == 0) {
      diagnostics_.rejected_blades++;
      continue;
    }

    const CircleModel model = circle_model(observation.location_num);
    Eigen::VectorXd h0;
    if (!predict_blade(camera_pose, model, include_center, h0) || h0.size() != z.size()) {
      diagnostics_.rejected_blades++;
      continue;
    }

    // 目标切换：只有 1 号位是当前跟踪的符叶，才允许重定义状态帧。
    // 符有 5 次对称性：R*Rz(k*72°) 配 model L 等价于 model L+k。对非 1 号位做
    // 这个切换，会把它的位置号误差吸收成一次状态旋转，而 1 号位已经被前面的
    // 更新固定，最终整帧位姿会静默错 72°。其他位置只做门限/拒绝，不动状态。
    const double base_rmse = std::sqrt((z - h0).squaredNorm() / static_cast<double>(z.size()));
    bool switch_applied = false;
    Eigen::Matrix3d pre_switch_R = Eigen::Matrix3d::Identity();
    Eigen::Matrix<double, 7, 7> pre_switch_P = Eigen::Matrix<double, 7, 7>::Identity();
    const auto restore_switch = [&]() {
      if (!switch_applied) return;
      R_car_from_plane_ = pre_switch_R;
      P_ = pre_switch_P;
      switch_applied = false;
    };

    if (observation.location_num == 1 && accepted == 0 &&
        base_rmse > config_.switch_rmse_px) {
      double best_rmse = base_rmse;
      Eigen::VectorXd best_h = h0;
      int best_switch = 0;
      for (int k = 1; k <= 4; ++k) {
        const double angle = k * 2.0 * kPi / 5.0;
        const Eigen::Matrix3d R_switch =
          R_car_from_plane_ * Eigen::AngleAxisd(angle, Eigen::Vector3d::UnitZ()).toRotationMatrix();
        Eigen::VectorXd h_switch;
        if (!predict_blade_with_pose(
              camera_pose, model, include_center, R_switch, t_car_from_plane_, h_switch) ||
            h_switch.size() != z.size()) {
          continue;
        }
        const double rmse =
          std::sqrt((z - h_switch).squaredNorm() / static_cast<double>(z.size()));
        if (rmse < best_rmse) {
          best_rmse = rmse;
          best_h = h_switch;
          best_switch = k;
        }
      }
      // 留 2px 余量，避免正常运动预测误差被误判成切换。
      if (best_switch != 0 && best_rmse + 2.0 < base_rmse) {
        pre_switch_R = R_car_from_plane_;
        pre_switch_P = P_;
        const Eigen::Matrix3d R_switch_delta =
          Eigen::AngleAxisd(
            best_switch * 2.0 * kPi / 5.0, Eigen::Vector3d::UnitZ()).toRotationMatrix();
        R_car_from_plane_ = orthonormalize(R_car_from_plane_ * R_switch_delta);
        h0 = best_h;
        // 状态帧绕 z 重定义后，误差状态 dtheta 也要跟着转；平移和角速度不变。
        Eigen::Matrix<double, 7, 7> J = Eigen::Matrix<double, 7, 7>::Identity();
        J.topLeftCorner<3, 3>() = R_switch_delta.transpose();
        P_ = J * P_ * J.transpose();
        // 切换后滚转方向的不确定度重新放大，避免协方差过度自信。
        const double switch_sigma = deg2rad(2.0);
        P_.diagonal().head<3>().array() += switch_sigma * switch_sigma;
        switch_applied = true;
      }
    }

    // 数值雅可比：状态维度小，且投影含畸变，数值差分比手写解析雅可比更不容易出错。
    Eigen::MatrixXd H = Eigen::MatrixXd::Zero(z.size(), 7);
    constexpr double kRotationEpsilon = 1e-3;   // rad
    constexpr double kTranslationEpsilon = 1e-3;  // m
    bool jacobian_valid = true;
    // 第 6 列是角速度：当前帧的像素观测不直接依赖 w，只通过预测协方差间接更新。
    for (int axis = 0; axis < 6; ++axis) {
      Eigen::Vector3d dtheta = Eigen::Vector3d::Zero();
      Eigen::Vector3d dt = Eigen::Vector3d::Zero();
      if (axis < 3) dtheta[axis] = kRotationEpsilon;
      else dt[axis - 3] = kTranslationEpsilon;

      Eigen::VectorXd plus;
      Eigen::VectorXd minus;
      if (!predict_blade_perturbed(camera_pose, model, include_center, dtheta, dt, plus) ||
          !predict_blade_perturbed(camera_pose, model, include_center, -dtheta, -dt, minus) ||
          plus.size() != z.size() || minus.size() != z.size()) {
        jacobian_valid = false;
        break;
      }
      const double step = axis < 3 ? 2.0 * kRotationEpsilon : 2.0 * kTranslationEpsilon;
      H.col(axis) = (plus - minus) / step;
    }
    if (!jacobian_valid || !H.allFinite()) {
      restore_switch();
      diagnostics_.rejected_blades++;
      continue;
    }

    Eigen::VectorXd residual = z - h0;
    Eigen::MatrixXd R_measurement = Eigen::MatrixXd::Zero(z.size(), z.size());
    for (int i = 0; i < z.size(); i += 2) {
      const double sigma = std::max(1e-6, row_sigma(i, include_center, config_));
      R_measurement(i, i) = sigma * sigma;
      R_measurement(i + 1, i + 1) = sigma * sigma;
    }

    Eigen::MatrixXd S = H * P_ * H.transpose() + R_measurement;
    const Eigen::LDLT<Eigen::MatrixXd> ldlt(S);
    if (ldlt.info() != Eigen::Success) {
      restore_switch();
      diagnostics_.rejected_blades++;
      continue;
    }
    const Eigen::VectorXd S_inv_residual = ldlt.solve(residual);
    if (!S_inv_residual.allFinite()) {
      restore_switch();
      diagnostics_.rejected_blades++;
      continue;
    }
    const double nis = residual.dot(S_inv_residual);
    max_nis = std::max(max_nis, nis);
    if (!std::isfinite(nis) || nis > config_.gate_chi2) {
      restore_switch();
      diagnostics_.rejected_blades++;
      continue;
    }

    const Eigen::MatrixXd K = P_ * H.transpose() * ldlt.solve(Eigen::MatrixXd::Identity(S.rows(), S.cols()));
    if (!K.allFinite()) {
      restore_switch();
      diagnostics_.rejected_blades++;
      continue;
    }
    const Eigen::VectorXd dx = K * residual;
    if (!dx.allFinite()) {
      restore_switch();
      diagnostics_.rejected_blades++;
      continue;
    }

    const Eigen::Vector3d dtheta = dx.head<3>();
    const Eigen::Vector3d dt = dx.segment<3>(3);
    R_car_from_plane_ = orthonormalize(R_car_from_plane_ * rotation_from_rvec(dtheta));
    t_car_from_plane_ += dt;
    angular_velocity_ += dx[6];

    // Joseph 形式更新协方差，数值上比 (I-KH)P 更稳。
    const Eigen::Matrix<double, 7, 7> I = Eigen::Matrix<double, 7, 7>::Identity();
    const Eigen::Matrix<double, 7, 7> A = I - K * H;
    P_ = A * P_ * A.transpose() + K * R_measurement * K.transpose();
    sanitize_covariance();
    // 切换假设已经通过门限并和本次更新一起提交，不再回滚。
    switch_applied = false;

    center_used = center_used || include_center;
    accepted++;

    Eigen::VectorXd h_after;
    if (predict_blade(camera_pose, model, include_center, h_after) && h_after.size() == z.size()) {
      residual_squared_sum += (z - h_after).squaredNorm();
      residual_count += static_cast<int>(z.size());
    }
  }

  diagnostics_.updated = accepted > 0;
  diagnostics_.accepted_blades = accepted;
  diagnostics_.nis = max_nis;
  if (residual_count > 0) {
    diagnostics_.reprojection_rmse_px =
      std::sqrt(residual_squared_sum / static_cast<double>(residual_count));
  } else {
    diagnostics_.reprojection_rmse_px = 0.0;
  }
  diagnostics_.initialized = initialized_;
  diagnostics_.angular_velocity = angular_velocity_;
  return accepted > 0;
}

Eigen::Matrix<double, 6, 1> RuneUvPoseFilter::pose_in_camera(
  const CameraPose & camera_pose) const
{
  return camera_pose_from_state(camera_pose);
}

std::vector<cv::Point2f> RuneUvPoseFilter::predict_keypoints(
  const CameraPose & camera_pose, int location_num) const
{
  std::vector<cv::Point2f> out;
  if (!initialized_) return out;

  Eigen::VectorXd z;
  if (!predict_blade(camera_pose, circle_model(location_num), true, z) || z.size() < 10) {
    return out;
  }
  out.reserve(5);
  out.emplace_back(static_cast<float>(z[0]), static_cast<float>(z[1]));  // top
  out.emplace_back(static_cast<float>(z[2]), static_cast<float>(z[3]));  // left
  out.emplace_back(static_cast<float>(z[4]), static_cast<float>(z[5]));  // center R
  out.emplace_back(static_cast<float>(z[6]), static_cast<float>(z[7]));  // right
  out.emplace_back(static_cast<float>(z[8]), static_cast<float>(z[9]));  // bottom
  return out;
}

void RuneUvPoseFilter::sanitize_covariance()
{
  if (!P_.allFinite()) {
    P_ = Eigen::Matrix<double, 7, 7>::Identity() * 1e-6;
    return;
  }
  P_ = 0.5 * (P_ + P_.transpose());
  for (int i = 0; i < P_.rows(); ++i) {
    if (!std::isfinite(P_(i, i)) || P_(i, i) < 1e-12) {
      // 数值异常时退化成单位阵，下一帧仍可由 UV 观测重新收敛。
      P_ = Eigen::Matrix<double, 7, 7>::Identity() * 1e-6;
      return;
    }
  }
}
