#include "buff_solver.hpp"

#include <algorithm>
#include <limits>

#include "tools/logger.hpp"

namespace auto_buff
{
namespace
{
double reprojection_error(
  const std::vector<cv::Point3f> & object_points, const std::vector<cv::Point2f> & image_points,
  const cv::Vec3d & rvec, const cv::Vec3d & tvec, const cv::Mat & camera_matrix,
  const cv::Mat & distort_coeffs)
{
  std::vector<cv::Point2f> projected;
  cv::projectPoints(object_points, rvec, tvec, camera_matrix, distort_coeffs, projected);
  double squared_error = 0.0;
  for (size_t i = 0; i < projected.size(); ++i) {
    const double error = cv::norm(projected[i] - image_points[i]);
    squared_error += error * error;
  }
  return std::sqrt(squared_error / static_cast<double>(projected.size()));
}

double image_angle_around_center(const cv::Point2f & point, const cv::Point2f & center)
{
  return std::atan2(point.y - center.y, point.x - center.x);
}

const char * observation_name(BuffObservationType type)
{
  switch (type) {
    case BuffObservationType::FULL:
      return "full";
    case BuffObservationType::TARGET_ONLY:
      return "target_only";
    case BuffObservationType::FAN_ONLY:
      return "fan_only";
  }
  return "unknown";
}
}  // namespace

cv::Matx33f Solver::rotation_matrix(double angle) const
{
  return cv::Matx33f(
    1, 0, 0, 0, std::cos(angle), -std::sin(angle), 0, std::sin(angle), std::cos(angle));
}

void Solver::compute_rotated_points(std::vector<std::vector<cv::Point3f>> & object_points)
{
  const std::vector<cv::Point3f> & base_points = object_points[0];
  for (int i = 1; i < 5; ++i) {
    double angle = i * THETA;
    cv::Matx33f R = rotation_matrix(angle);
    std::vector<cv::Point3f> rotated_points;
    for (const auto & point : base_points) {
      cv::Vec3f vec(point.x, point.y, point.z);
      cv::Vec3f rotated_vec = R * vec;
      rotated_points.emplace_back(rotated_vec[0], rotated_vec[1], rotated_vec[2]);
    }
    object_points[i] = rotated_points;
  }
}

Solver::Solver(const std::string & config_path) : R_gimbal2world_(Eigen::Matrix3d::Identity())
{
  auto yaml = YAML::LoadFile(config_path);
  if (yaml["buff_rune_radius_m"]) RUNE_RADIUS_M = yaml["buff_rune_radius_m"].as<double>();
  if (yaml["buff_small_direction"]) SMALL_BUFF_DIRECTION = yaml["buff_small_direction"].as<int>();
  if (yaml["buff_pnp_full_reprojection_gate_px"]) {
    full_reprojection_gate_px_ = yaml["buff_pnp_full_reprojection_gate_px"].as<double>();
  }
  if (yaml["buff_pnp_target_center_gate_px"]) {
    target_center_reprojection_gate_px_ = yaml["buff_pnp_target_center_gate_px"].as<double>();
  }
  if (yaml["buff_pnp_fan_center_gate_px"]) {
    fan_center_reprojection_gate_px_ = yaml["buff_pnp_fan_center_gate_px"].as<double>();
  }
  if (yaml["buff_pnp_partial_center_gate_px"]) {
    partial_four_center_gate_px_ = yaml["buff_pnp_partial_center_gate_px"].as<double>();
  }
  if (yaml["buff_pnp_partial_angle_gate_deg"]) {
    partial_four_angle_gate_rad_ = yaml["buff_pnp_partial_angle_gate_deg"].as<double>() / 57.3;
  }

  auto R_gimbal2imubody_data = yaml["R_gimbal2imubody"].as<std::vector<double>>();
  auto R_camera2gimbal_data = yaml["R_camera2gimbal"].as<std::vector<double>>();
  auto t_camera2gimbal_data = yaml["t_camera2gimbal"].as<std::vector<double>>();
  R_gimbal2imubody_ = Eigen::Matrix<double, 3, 3, Eigen::RowMajor>(R_gimbal2imubody_data.data());
  R_camera2gimbal_ = Eigen::Matrix<double, 3, 3, Eigen::RowMajor>(R_camera2gimbal_data.data());
  t_camera2gimbal_ = Eigen::Matrix<double, 3, 1>(t_camera2gimbal_data.data());

  auto camera_matrix_data = yaml["camera_matrix"].as<std::vector<double>>();
  auto distort_coeffs_data = yaml["distort_coeffs"].as<std::vector<double>>();
  Eigen::Matrix<double, 3, 3, Eigen::RowMajor> camera_matrix(camera_matrix_data.data());
  Eigen::Matrix<double, 1, 5> distort_coeffs(distort_coeffs_data.data());
  cv::eigen2cv(camera_matrix, camera_matrix_);
  cv::eigen2cv(distort_coeffs, distort_coeffs_);
}

std::vector<cv::Point3f> Solver::reproject_object_points() const
{
  const float r = static_cast<float>(RUNE_RADIUS_M);
  return {
    cv::Point3f(0.0f, 0.0f, r + 0.095f), cv::Point3f(0.0f, -0.095f, r),
    cv::Point3f(0.0f, 0.0f, r - 0.095f), cv::Point3f(0.0f, 0.095f, r),
    cv::Point3f(0.0f, 0.030f, r - 0.191f), cv::Point3f(0.0f, -0.030f, r - 0.191f),
    cv::Point3f(0.0f, -0.030f, r - 0.521f), cv::Point3f(0.0f, 0.030f, r - 0.521f),
    cv::Point3f(0.0f, 0.0f, 0.0f)};
}

Eigen::Matrix3d Solver::R_gimbal2world() const { return R_gimbal2world_; }

void Solver::set_R_gimbal2world(const Eigen::Quaterniond & q)
{
  Eigen::Matrix3d R_imubody2imuabs = q.toRotationMatrix();
  R_gimbal2world_ = R_gimbal2imubody_.transpose() * R_imubody2imuabs * R_gimbal2imubody_;
}

std::optional<PowerRune> Solver::solve(
  const std::optional<BuffObservation> & observation) const
{
  if (!observation.has_value()) {
    has_pnp_solution_ = false;
    return std::nullopt;
  }
  return solve(*observation);
}

std::vector<PowerRune> Solver::solve_all(
  const std::vector<BuffObservation> & observations) const
{
  has_pnp_solution_ = false;
  std::vector<PowerRune> solved;
  solved.reserve(observations.size());
  for (const auto & observation : observations) {
    auto rune = solve(observation);
    if (rune.has_value()) solved.push_back(std::move(*rune));
  }
  if (!solved.empty()) {
    const auto cache = pose_cache_.find(solved.front().track_id);
    if (cache != pose_cache_.end()) {
      rvec_ = cache->second.rvec;
      tvec_ = cache->second.tvec;
      has_pnp_solution_ = true;
    }
  }
  return solved;
}

std::optional<PowerRune> Solver::solve(const BuffObservation & observation) const
{
  has_pnp_solution_ = false;
  if (!observation.has_target() && !observation.has_fan()) {
    tools::logger()->debug("[Buff_Solver] observation has no real keypoints");
    return std::nullopt;
  }

  std::vector<cv::Point3f> object_points;
  std::vector<cv::Point2f> image_points;
  if (observation.has_target()) {
    object_points.insert(object_points.end(), PNP_OBJECT_POINTS.begin(), PNP_OBJECT_POINTS.begin() + 4);
    image_points.insert(
      image_points.end(), observation.target_points.begin(), observation.target_points.end());
  }
  if (observation.has_fan()) {
    object_points.insert(object_points.end(), PNP_OBJECT_POINTS.begin() + 4, PNP_OBJECT_POINTS.end());
    image_points.insert(image_points.end(), observation.fan_points.begin(), observation.fan_points.end());
  }

  const bool partial = observation.type != BuffObservationType::FULL;
  const bool use_detected_center =
    partial && observation.center_source == RuneCenterSource::DETECTED;
  if (use_detected_center) {
    object_points.emplace_back(0.0f, static_cast<float>(RUNE_RADIUS_M), 0.0f);
    image_points.push_back(observation.r_center);
  }

  BuffPoseQuality pose_quality = BuffPoseQuality::FULL_8_POINT;
  double noise_scale = 1.0;
  double reprojection_gate = full_reprojection_gate_px_;
  if (partial && use_detected_center) {
    pose_quality = BuffPoseQuality::PARTIAL_5_POINT;
    if (observation.type == BuffObservationType::TARGET_ONLY) {
      noise_scale = 2.0;
      reprojection_gate = target_center_reprojection_gate_px_;
    } else {
      noise_scale = 3.0;
      reprojection_gate = fan_center_reprojection_gate_px_;
    }
  } else if (partial) {
    pose_quality = BuffPoseQuality::PARTIAL_4_POINT;
    noise_scale = 6.0;
    reprojection_gate = full_reprojection_gate_px_;
  }

  std::vector<cv::Vec3d> rvecs;
  std::vector<cv::Vec3d> tvecs;
  cv::Mat reprojection_errors;
  const int solution_count = cv::solvePnPGeneric(
    object_points, image_points, camera_matrix_, distort_coeffs_, rvecs, tvecs, false,
    cv::SOLVEPNP_IPPE, cv::noArray(), cv::noArray(), reprojection_errors);
  if (solution_count <= 0 || rvecs.empty()) {
    tools::logger()->debug("[Buff_Solver] solvePnPGeneric failed");
    return std::nullopt;
  }

  int best_index = -1;
  double best_score = std::numeric_limits<double>::max();
  double best_error = std::numeric_limits<double>::max();
  double diagnostic_reprojection = std::numeric_limits<double>::max();
  double diagnostic_center_error = std::numeric_limits<double>::max();
  double diagnostic_angle_error = std::numeric_limits<double>::max();
  for (int i = 0; i < solution_count; ++i) {
    if (tvecs[i][2] <= 0.0) continue;
    const double error = reprojection_error(
      object_points, image_points, rvecs[i], tvecs[i], camera_matrix_, distort_coeffs_);
    diagnostic_reprojection = std::min(diagnostic_reprojection, error);
    if (!std::isfinite(error)) continue;

    std::vector<cv::Point3f> probes{
      cv::Point3f(0.0f, 0.0f, 0.0f),
      cv::Point3f(0.0f, static_cast<float>(RUNE_RADIUS_M), 0.0f)};
    std::vector<cv::Point2f> projected_probes;
    cv::projectPoints(
      probes, rvecs[i], tvecs[i], camera_matrix_, distort_coeffs_, projected_probes);
    const double center_error = cv::norm(projected_probes[1] - observation.r_center);
    const double projected_angle =
      image_angle_around_center(projected_probes[0], observation.r_center);
    const double angle_error =
      std::abs(tools::limit_rad(projected_angle - observation.angle));
    diagnostic_center_error = std::min(diagnostic_center_error, center_error);
    diagnostic_angle_error = std::min(diagnostic_angle_error, angle_error);
    if (
      pose_quality == BuffPoseQuality::PARTIAL_4_POINT &&
      (center_error > partial_four_center_gate_px_ ||
       angle_error > partial_four_angle_gate_rad_)) {
      continue;
    }

    double continuity_penalty = 0.0;
    const auto cached_pose = pose_cache_.find(observation.track_id);
    if (cached_pose != pose_cache_.end()) {
      continuity_penalty = 0.05 * cv::norm(rvecs[i] - cached_pose->second.rvec) +
                           0.05 * cv::norm(tvecs[i] - cached_pose->second.tvec);
    }
    const double score = error + 0.15 * center_error + 2.0 * angle_error + continuity_penalty;
    if (score < best_score) {
      best_score = score;
      best_error = error;
      best_index = i;
    }
  }
  if (best_index < 0) {
    tools::logger()->debug(
      "[Buff_Solver] {} rejected: rms {:.2f}px center {:.1f}px angle {:.1f}deg",
      observation_name(observation.type), diagnostic_reprojection, diagnostic_center_error,
      diagnostic_angle_error * 57.3);
    return std::nullopt;
  }

  rvec_ = rvecs[best_index];
  tvec_ = tvecs[best_index];
  cv::solvePnPRefineLM(object_points, image_points, camera_matrix_, distort_coeffs_, rvec_, tvec_);
  best_error =
    reprojection_error(object_points, image_points, rvec_, tvec_, camera_matrix_, distort_coeffs_);
  if (!std::isfinite(best_error) || best_error > reprojection_gate || tvec_[2] <= 0.0) {
    tools::logger()->debug("[Buff_Solver] refined PnP rejected, rms {:.2f}px", best_error);
    return std::nullopt;
  }
  has_pnp_solution_ = true;
  pose_cache_[observation.track_id] = PoseCache{rvec_, tvec_};
  if (pose_cache_.size() > 64) {
    const auto oldest = std::min_element(
      pose_cache_.begin(), pose_cache_.end(), [](const auto & lhs, const auto & rhs) {
        return lhs.first < rhs.first;
      });
    if (oldest != pose_cache_.end() && oldest->first != observation.track_id) {
      pose_cache_.erase(oldest);
    }
  }

  PowerRune p(observation);
  p.pose_quality = pose_quality;
  const double geometry_noise_scale =
    std::clamp(1.0 + 2.0 * (1.0 - observation.quad_quality), 1.0, 3.0);
  p.measurement_noise_scale =
    noise_scale * observation.keypoint_noise_scale * geometry_noise_scale;
  p.reprojection_error = best_error;

  Eigen::Vector3d t_target2camera;
  cv::cv2eigen(tvec_, t_target2camera);
  cv::Mat rmat;
  cv::Rodrigues(rvec_, rmat);
  Eigen::Matrix3d R_target2camera;
  cv::cv2eigen(rmat, R_target2camera);

  const Eigen::Vector3d center_in_target(0.0, RUNE_RADIUS_M, 0.0);
  const Eigen::Vector3d target_in_camera = t_target2camera;
  Eigen::Vector3d center_in_camera = R_target2camera * center_in_target + t_target2camera;
  if (!partial && observation.center_source == RuneCenterSource::DETECTED) {
    std::vector<cv::Point2f> distorted_center{observation.r_center};
    std::vector<cv::Point2f> normalized_center;
    cv::undistortPoints(
      distorted_center, normalized_center, camera_matrix_, distort_coeffs_);
    if (!normalized_center.empty()) {
      Eigen::Vector3d center_ray(
        normalized_center[0].x, normalized_center[0].y, 1.0);
      const Eigen::Vector3d plane_normal_camera = R_target2camera * Eigen::Vector3d::UnitZ();
      const double denominator = plane_normal_camera.dot(center_ray);
      if (std::abs(denominator) > 1e-6) {
        const double ray_scale = plane_normal_camera.dot(center_in_camera) / denominator;
        if (ray_scale > 0.0 && std::isfinite(ray_scale)) center_in_camera = ray_scale * center_ray;
      }
    }
  }

  const Eigen::Matrix3d R_target2gimbal = R_camera2gimbal_ * R_target2camera;
  const Eigen::Vector3d target_in_gimbal = R_camera2gimbal_ * target_in_camera + t_camera2gimbal_;
  const Eigen::Vector3d center_in_gimbal = R_camera2gimbal_ * center_in_camera + t_camera2gimbal_;

  const Eigen::Vector3d target_in_world = R_gimbal2world_ * target_in_gimbal;
  const Eigen::Vector3d center_in_world = R_gimbal2world_ * center_in_gimbal;
  const Eigen::Matrix3d R_target2world = R_gimbal2world_ * R_target2gimbal;

  p.xyz_in_world = center_in_world;
  p.ypd_in_world = tools::xyz2ypd(p.xyz_in_world);
  p.blade_xyz_in_world = target_in_world;
  p.blade_ypd_in_world = tools::xyz2ypd(p.blade_xyz_in_world);
  p.plane_normal_in_world = R_target2world * Eigen::Vector3d::UnitZ();
  return p;
}

cv::Point2f Solver::point_buff2pixel(cv::Point3f x)
{
  std::vector<cv::Point3d> world_points;
  std::vector<cv::Point2d> image_points;
  world_points.push_back(x);
  cv::projectPoints(world_points, rvec_, tvec_, camera_matrix_, distort_coeffs_, image_points);
  return image_points.back();
}

std::optional<std::vector<cv::Point2f>> Solver::reproject_pnp_points() const
{
  if (!has_pnp_solution_) return std::nullopt;

  std::vector<cv::Point2f> image_points;
  cv::projectPoints(PNP_OBJECT_POINTS, rvec_, tvec_, camera_matrix_, distort_coeffs_, image_points);
  return image_points;
}

std::vector<cv::Point2f> Solver::reproject_buff(
  const Eigen::Vector3d & xyz_in_world, const Eigen::Matrix3d & R_buff2world) const
{
  const Eigen::Vector3d & t_buff2world = xyz_in_world;
  Eigen::Matrix3d R_buff2camera =
    R_camera2gimbal_.transpose() * R_gimbal2world_.transpose() * R_buff2world;
  Eigen::Vector3d t_buff2camera =
    R_camera2gimbal_.transpose() * (R_gimbal2world_.transpose() * t_buff2world - t_camera2gimbal_);

  cv::Vec3d rvec;
  cv::Mat R_buff2camera_cv;
  cv::eigen2cv(R_buff2camera, R_buff2camera_cv);
  cv::Rodrigues(R_buff2camera_cv, rvec);
  cv::Vec3d tvec(t_buff2camera[0], t_buff2camera[1], t_buff2camera[2]);

  std::vector<cv::Point2f> image_points;
  cv::projectPoints(
    reproject_object_points(), rvec, tvec, camera_matrix_, distort_coeffs_, image_points);
  return image_points;
}
}  // namespace auto_buff
