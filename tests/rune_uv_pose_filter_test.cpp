#include <array>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <vector>

#include <Eigen/Geometry>

#include "RuneUvPoseFilter.hpp"

#define CHECK(condition) \
  do { \
    if (!(condition)) { \
      std::cerr << "CHECK failed at line " << __LINE__ << ": " #condition << std::endl; \
      std::abort(); \
    } \
  } while (false)

namespace
{
constexpr double kPi = 3.14159265358979323846;
constexpr double kFx = 1200.0;
constexpr double kFy = 1200.0;
constexpr double kCx = 720.0;
constexpr double kCy = 540.0;
constexpr double kRadius = 0.15;
constexpr double kCenterRadius = 0.7;

Eigen::Vector3d rvec(const Eigen::Matrix3d & rotation)
{
  const Eigen::AngleAxisd angle_axis(rotation);
  return angle_axis.angle() * angle_axis.axis();
}

Eigen::Matrix3d rotation(double x, double y, double z)
{
  return (Eigen::AngleAxisd(x, Eigen::Vector3d::UnitX()) *
          Eigen::AngleAxisd(y, Eigen::Vector3d::UnitY()) *
          Eigen::AngleAxisd(z, Eigen::Vector3d::UnitZ())).toRotationMatrix();
}

Eigen::Vector2d project(const Eigen::Vector3d & point_camera)
{
  return {kFx * point_camera.x() / point_camera.z() + kCx,
          kFy * point_camera.y() / point_camera.z() + kCy};
}

RuneUvPoseFilter::Observation make_observation(
  const Eigen::Matrix3d & R_car_from_plane, const Eigen::Vector3d & t_car_from_plane,
  int location)
{
  const double angle = (location - 1) * 2.0 * kPi / 5.0;
  const Eigen::Matrix3d R_location =
    Eigen::AngleAxisd(angle, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  const Eigen::Vector3d center = R_location * Eigen::Vector3d(0.0, -kCenterRadius, 0.0);
  const std::array<Eigen::Vector3d, 4> offsets{{
    Eigen::Vector3d(0.0, -kRadius, 0.0),
    Eigen::Vector3d(-kRadius, 0.0, 0.0),
    Eigen::Vector3d(kRadius, 0.0, 0.0),
    Eigen::Vector3d(0.0, kRadius, 0.0),
  }};
  const std::array<Eigen::Vector3d, 5> local{{
    center + R_location * offsets[0], center + R_location * offsets[1],
    Eigen::Vector3d::Zero(), center + R_location * offsets[2],
    center + R_location * offsets[3]}};

  RuneUvPoseFilter::Observation observation;
  observation.location_num = location;
  for (int i = 0; i < 5; ++i) {
    const Eigen::Vector2d uv = project(R_car_from_plane * local[i] + t_car_from_plane);
    observation.keypoints[i] = cv::Point2f(
      static_cast<float>(uv.x()), static_cast<float>(uv.y()));
  }

  const std::array<Eigen::Vector3d, 4> anchor_local{{
    Eigen::Vector3d(-0.15, -0.85, 0.0),
    Eigen::Vector3d(-0.15, -0.174, 0.0),
    Eigen::Vector3d(0.15, -0.174, 0.0),
    Eigen::Vector3d(0.15, -0.85, 0.0),
  }};
  for (int i = 0; i < 4; ++i) {
    const Eigen::Vector2d uv = project(
      R_car_from_plane * (R_location * anchor_local[static_cast<std::size_t>(i)]) +
      t_car_from_plane);
    observation.anchors[i] = cv::Point2f(
      static_cast<float>(uv.x()), static_cast<float>(uv.y()));
  }
  observation.has_anchors = true;
  return observation;
}

Eigen::Matrix<double, 6, 1> pose_vector(
  const Eigen::Matrix3d & R_camera_from_plane, const Eigen::Vector3d & t_camera_from_plane)
{
  Eigen::Matrix<double, 6, 1> pose;
  pose.head<3>() = rvec(R_camera_from_plane);
  pose.tail<3>() = t_camera_from_plane;
  return pose;
}

Eigen::Matrix3d rotation_from_pose(const Eigen::Matrix<double, 6, 1> & pose)
{
  const Eigen::Vector3d rotation_vector = pose.head<3>();
  const double angle = rotation_vector.norm();
  if (angle < 1e-12) return Eigen::Matrix3d::Identity();
  return Eigen::AngleAxisd(angle, rotation_vector / angle).toRotationMatrix();
}

double rotation_error(const Eigen::Matrix3d & lhs, const Eigen::Matrix3d & rhs)
{
  return Eigen::AngleAxisd(lhs.transpose() * rhs).angle();
}

void configure_filter(RuneUvPoseFilter & filter, const RuneUvPoseConfig & config)
{
  filter.configure(
    config, cv::Matx33d(kFx, 0.0, kCx, 0.0, kFy, kCy, 0.0, 0.0, 1.0),
    cv::Matx<double, 1, 5>(0.0, 0.0, 0.0, 0.0, 0.0));
}
}  // namespace

int main()
{
  RuneUvPoseConfig config;
  config.sigma_px = 0.5;
  config.sigma_center_px = 0.5;
  config.gate_chi2 = 200.0;
  config.initial_rotation_sigma_deg = 5.0;
  config.initial_translation_sigma_m = 0.15;
  config.circle_radius = kRadius;
  config.circle_center_radius = kCenterRadius;

  RuneUvPoseFilter filter;
  configure_filter(filter, config);

  CameraPose camera_pose;
  camera_pose.R_car_from_camera = Eigen::Matrix3d::Identity();
  camera_pose.t_car_from_camera = Eigen::Vector3d::Zero();

  const Eigen::Matrix3d R_truth = rotation(0.08, -0.05, 0.30);
  const Eigen::Vector3d t_truth(0.25, -0.10, 8.0);

  // 1) 初始化后相机系位姿应与输入一致。
  CHECK(filter.initialize(camera_pose, pose_vector(R_truth, t_truth)));
  const auto initialized = filter.pose_in_camera(camera_pose);
  const Eigen::Matrix3d R_initialized =
    Eigen::AngleAxisd(initialized.head<3>().norm(),
                      initialized.head<3>().norm() > 1e-12
                        ? initialized.head<3>().normalized() : Eigen::Vector3d::UnitX())
      .toRotationMatrix();
  CHECK((rvec(R_truth.transpose() * R_initialized)).norm() < 1e-9);
  CHECK((initialized.tail<3>() - t_truth).norm() < 1e-9);

  // 2) 连续旋转跟踪：角速度应逐渐收敛到真实值。
  const double omega = 1.0;
  const double dt = 1.0 / 30.0;
  Eigen::Matrix3d R_ground = R_truth;
  for (int i = 0; i < 40; ++i) {
    R_ground = R_ground * Eigen::AngleAxisd(omega * dt, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    const auto step_observation = make_observation(R_ground, t_truth, 1);
    CHECK(filter.predict(camera_pose, dt));
    CHECK(filter.update(camera_pose, {step_observation}));
    CHECK(filter.diagnostics().reprojection_rmse_px < 1.0);
  }
  CHECK(std::abs(filter.diagnostics().angular_velocity - omega) < 0.25);

  // 3) 目标切换：跟踪片换成另一片后，新片会重新成为 1 号位；
  //    72° 假设应能直接接上而不是丢帧。
  R_ground = R_ground * Eigen::AngleAxisd(2.0 * kPi / 5.0, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  const auto switched_observation = make_observation(R_ground, t_truth, 1);
  CHECK(filter.predict(camera_pose, dt));
  CHECK(filter.update(camera_pose, {switched_observation}));
  CHECK(filter.diagnostics().accepted_blades == 1);
  CHECK(filter.diagnostics().reprojection_rmse_px < 1.0);

  // 4) 相机/云台转动不应该改变固定 RP 系中的目标状态。
  //    修复前 predict 会再乘一次相机相对旋转，这里会差 10°。
  {
    RuneUvPoseFilter camera_filter;
    configure_filter(camera_filter, config);
    CHECK(camera_filter.initialize(camera_pose, pose_vector(R_truth, t_truth)));

    CameraPose rotated_camera;
    rotated_camera.R_car_from_camera =
      Eigen::AngleAxisd(10.0 * kPi / 180.0, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    rotated_camera.t_car_from_camera = Eigen::Vector3d::Zero();
    CHECK(camera_filter.predict(rotated_camera, dt));

    const auto predicted = camera_filter.pose_in_camera(rotated_camera);
    const Eigen::Matrix3d expected_rotation =
      rotated_camera.R_car_from_camera.transpose() * R_truth;
    const Eigen::Vector3d expected_translation =
      rotated_camera.R_car_from_camera.transpose() *
      (t_truth - rotated_camera.t_car_from_camera);
    CHECK(rotation_error(expected_rotation, rotation_from_pose(predicted)) < 1e-9);
    CHECK((predicted.tail<3>() - expected_translation).norm() < 1e-9);
  }

  // 5) 多符叶时，第二片位置号错一档不能把 1 号位带偏。
  //    修复前会触发 72° 切换，accepted=2、rmse=0，但整帧姿态错 72°。
  {
    RuneUvPoseFilter multi_filter;
    configure_filter(multi_filter, config);
    CHECK(multi_filter.initialize(camera_pose, pose_vector(R_truth, t_truth)));

    const auto good_observation = make_observation(R_truth, t_truth, 1);
    auto misassigned_observation = make_observation(R_truth, t_truth, 2);
    misassigned_observation.location_num = 3;
    CHECK(multi_filter.predict(camera_pose, dt));
    CHECK(multi_filter.update(camera_pose, {good_observation, misassigned_observation}));
    CHECK(multi_filter.diagnostics().accepted_blades == 1);
    CHECK(multi_filter.diagnostics().rejected_blades == 1);

    const auto pose = multi_filter.pose_in_camera(camera_pose);
    CHECK(rotation_error(R_truth, rotation_from_pose(pose)) < 0.02);
    CHECK((pose.tail<3>() - t_truth).norm() < 0.01);
  }

  // 6) 切换假设被门限拒绝时，状态必须回滚到切换前。
  {
    RuneUvPoseConfig reject_config = config;
    reject_config.gate_chi2 = -1.0;  // 任何非负 NIS 都拒绝，专门测回滚
    RuneUvPoseFilter reject_filter;
    configure_filter(reject_filter, reject_config);
    CHECK(reject_filter.initialize(camera_pose, pose_vector(R_truth, t_truth)));
    CHECK(reject_filter.predict(camera_pose, dt));
    const auto before = reject_filter.pose_in_camera(camera_pose);

    const Eigen::Matrix3d shifted_rotation =
      R_truth *
      Eigen::AngleAxisd(2.0 * kPi / 5.0, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    const auto shifted_observation = make_observation(shifted_rotation, t_truth, 1);
    CHECK(!reject_filter.update(camera_pose, {shifted_observation}));

    const auto after = reject_filter.pose_in_camera(camera_pose);
    CHECK(rotation_error(rotation_from_pose(before), rotation_from_pose(after)) < 1e-12);
    CHECK((before.tail<3>() - after.tail<3>()).norm() < 1e-12);
    CHECK(reject_filter.diagnostics().rejected_blades == 1);
  }

  // 7) PnP 兜底重建应保留已经收敛的角速度，而不是清零。
  {
    RuneUvPoseFilter reinit_filter;
    configure_filter(reinit_filter, config);
    CHECK(reinit_filter.initialize(camera_pose, pose_vector(R_truth, t_truth)));
    Eigen::Matrix3d R_reinit = R_truth;
    for (int i = 0; i < 40; ++i) {
      R_reinit =
        R_reinit * Eigen::AngleAxisd(omega * dt, Eigen::Vector3d::UnitZ()).toRotationMatrix();
      const auto observation = make_observation(R_reinit, t_truth, 1);
      CHECK(reinit_filter.predict(camera_pose, dt));
      CHECK(reinit_filter.update(camera_pose, {observation}));
    }

    const double angular_velocity_before = reinit_filter.diagnostics().angular_velocity;
    CHECK(std::abs(angular_velocity_before - omega) < 0.25);
    CHECK(reinit_filter.initialize(camera_pose, pose_vector(R_reinit, t_truth), true));
    CHECK(std::abs(reinit_filter.diagnostics().angular_velocity - angular_velocity_before) < 1e-9);
    CHECK(reinit_filter.predict(camera_pose, dt));

    const Eigen::Matrix3d expected_rotation =
      R_reinit *
      Eigen::AngleAxisd(angular_velocity_before * dt, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    const auto pose = reinit_filter.pose_in_camera(camera_pose);
    CHECK(rotation_error(expected_rotation, rotation_from_pose(pose)) < 0.02);
  }

  return 0;
}
