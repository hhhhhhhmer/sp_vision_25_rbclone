#include "buff_detector.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <functional>
#include <limits>
#include <numeric>

#include "tools/logger.hpp"

namespace auto_buff
{
namespace
{
cv::Point2f mean_points(const std::vector<cv::Point2f> & points)
{
  cv::Point2f sum{0.0f, 0.0f};
  for (const auto & point : points) sum += point;
  if (!points.empty()) sum *= 1.0f / static_cast<float>(points.size());
  return sum;
}

float dot2d(const cv::Point2f & a, const cv::Point2f & b) { return a.x * b.x + a.y * b.y; }

float cross2d(const cv::Point2f & a, const cv::Point2f & b) { return a.x * b.y - a.y * b.x; }

cv::Point2f normalize_or(const cv::Point2f & v, const cv::Point2f & fallback)
{
  const float n = cv::norm(v);
  if (n < 1e-3f) return fallback;
  return v * (1.0f / n);
}

bool valid_keypoints(const YOLO11_BUFF::Object & object, float threshold)
{
  if (object.kpt.size() != 4) return false;
  if (object.kpt_conf.empty()) return true;
  return std::all_of(object.kpt_conf.begin(), object.kpt_conf.end(), [&](float conf) {
    return conf >= threshold;
  });
}

float minimum_keypoint_confidence(const YOLO11_BUFF::Object & object)
{
  if (object.kpt_conf.empty()) return 1.0f;
  return *std::min_element(object.kpt_conf.begin(), object.kpt_conf.end());
}

std::optional<std::vector<cv::Point2f>> order_target_points(
  const std::vector<cv::Point2f> & points, const cv::Point2f & center,
  const cv::Point2f & axis_y)
{
  if (points.size() != 4) return std::nullopt;
  const cv::Point2f axis_x{axis_y.y, -axis_y.x};
  const std::array<cv::Point2f, 4> expected{-axis_y, axis_x, axis_y, -axis_x};
  std::array<int, 4> permutation{0, 1, 2, 3};
  std::array<int, 4> index = permutation;
  double best_cost = std::numeric_limits<double>::max();
  do {
    double cost = 0.0;
    for (int role = 0; role < 4; ++role) {
      const cv::Point2f direction =
        normalize_or(points[permutation[role]] - center, expected[role]);
      cost += 1.0 - dot2d(direction, expected[role]);
    }
    if (cost < best_cost) {
      best_cost = cost;
      index = permutation;
    }
  } while (std::next_permutation(permutation.begin(), permutation.end()));

  return std::vector<cv::Point2f>{
    points[index[0]], points[index[1]], points[index[2]], points[index[3]]};
}

std::vector<cv::Point2f> order_rect_points(
  const std::vector<cv::Point2f> & points, const cv::Point2f & center,
  const cv::Point2f & axis_y)
{
  std::vector<int> indexes(points.size());
  std::iota(indexes.begin(), indexes.end(), 0);
  std::sort(indexes.begin(), indexes.end(), [&](int lhs, int rhs) {
    return dot2d(points[lhs] - center, axis_y) < dot2d(points[rhs] - center, axis_y);
  });

  std::array<int, 2> top{indexes[0], indexes[1]};
  std::array<int, 2> bottom{indexes[2], indexes[3]};
  auto left_first = [&](int lhs, int rhs) {
    return cross2d(axis_y, points[lhs] - center) > cross2d(axis_y, points[rhs] - center);
  };
  std::sort(top.begin(), top.end(), left_first);
  std::sort(bottom.begin(), bottom.end(), left_first);

  return {points[top[0]], points[top[1]], points[bottom[1]], points[bottom[0]]};
}

double quad_quality_score(const std::vector<cv::Point2f> & points)
{
  if (points.size() != 4) return 0.0;
  for (size_t i = 0; i < points.size(); ++i) {
    for (size_t j = i + 1; j < points.size(); ++j) {
      if (cv::norm(points[i] - points[j]) < 3.0) return 0.0;
    }
  }
  std::vector<cv::Point2f> hull;
  cv::convexHull(points, hull);
  const double area = std::abs(cv::contourArea(hull));
  if (hull.size() != 4 || area < 20.0) return 0.0;

  std::array<double, 4> sides{};
  for (size_t i = 0; i < points.size(); ++i) {
    sides[i] = cv::norm(points[(i + 1) % points.size()] - points[i]);
  }
  const double opposite_a = std::min(sides[0], sides[2]) / std::max(sides[0], sides[2]);
  const double opposite_b = std::min(sides[1], sides[3]) / std::max(sides[1], sides[3]);
  const double compactness = std::clamp(area / 100.0, 0.0, 1.0);
  return std::min({opposite_a, opposite_b, 0.5 + 0.5 * compactness});
}

bool valid_ordered_quad(const std::vector<cv::Point2f> & points)
{
  return quad_quality_score(points) >= 0.10;
}

std::vector<cv::Point2f> rotate_points(
  const std::vector<cv::Point2f> & points, const cv::Point2f & old_center,
  const cv::Point2f & new_center, double angle)
{
  const float c = static_cast<float>(std::cos(angle));
  const float s = static_cast<float>(std::sin(angle));
  std::vector<cv::Point2f> rotated;
  rotated.reserve(points.size());
  for (const auto & point : points) {
    const cv::Point2f relative = point - old_center;
    rotated.emplace_back(
      new_center + cv::Point2f(c * relative.x - s * relative.y, s * relative.x + c * relative.y));
  }
  return rotated;
}

double quad_scale(const std::vector<cv::Point2f> & points)
{
  if (points.size() != 4) return 0.0;
  const cv::Point2f center = mean_points(points);
  double radius = 0.0;
  for (const auto & point : points) radius += cv::norm(point - center);
  return radius / 4.0;
}

double angle_around(const cv::Point2f & point, const cv::Point2f & center)
{
  return std::atan2(point.y - center.y, point.x - center.x);
}

double seconds_between(
  std::chrono::steady_clock::time_point now, std::chrono::steady_clock::time_point before)
{
  return std::chrono::duration<double>(now - before).count();
}

int observation_rank(BuffObservationType type)
{
  switch (type) {
    case BuffObservationType::FULL:
      return 0;
    case BuffObservationType::TARGET_ONLY:
      return 1;
    case BuffObservationType::FAN_ONLY:
      return 2;
  }
  return 3;
}

}  // namespace

Buff_Detector::Buff_Detector(const std::string & config) : status_(LOSE), lose_(0), MODE_(config)
{
  auto yaml = YAML::LoadFile(config);
  if (yaml["buff_keypoint_threshold"]) keypoint_threshold_ = yaml["buff_keypoint_threshold"].as<float>();
  if (yaml["buff_keypoint_hard_threshold"]) {
    hard_keypoint_threshold_ = yaml["buff_keypoint_hard_threshold"].as<float>();
  }
  if (yaml["buff_keypoint_temporal_gate_px"]) {
    temporal_residual_gate_px_ = yaml["buff_keypoint_temporal_gate_px"].as<double>();
  }
  if (yaml["buff_center_innovation_gate_px"]) {
    center_innovation_gate_px_ = yaml["buff_center_innovation_gate_px"].as<double>();
  }
  if (yaml["buff_center_recovery_hits"]) {
    center_recovery_hits_ = yaml["buff_center_recovery_hits"].as<int>();
  }
  if (yaml["buff_center_lost_max"]) center_lost_max_ = yaml["buff_center_lost_max"].as<int>();
  if (yaml["buff_rune_radius_m"]) RUNE_RADIUS_M = yaml["buff_rune_radius_m"].as<double>();
  if (yaml["buff_small_direction"]) SMALL_BUFF_DIRECTION = yaml["buff_small_direction"].as<int>();
  if (yaml["buff_pair_angle_gate_deg"]) {
    pair_angle_gate_rad_ = yaml["buff_pair_angle_gate_deg"].as<double>() / 57.3;
  }
  if (yaml["buff_pair_ratio_min"]) pair_ratio_min_ = yaml["buff_pair_ratio_min"].as<double>();
  if (yaml["buff_pair_ratio_max"]) pair_ratio_max_ = yaml["buff_pair_ratio_max"].as<double>();
  if (yaml["buff_pair_ratio_center"]) {
    pair_ratio_center_ = yaml["buff_pair_ratio_center"].as<double>();
  }
  if (yaml["buff_track_gate_min_deg"]) {
    track_gate_min_rad_ = yaml["buff_track_gate_min_deg"].as<double>() / 57.3;
  }
  if (yaml["buff_track_gate_max_deg"]) {
    track_gate_max_rad_ = yaml["buff_track_gate_max_deg"].as<double>() / 57.3;
  }
  if (yaml["buff_blind_timeout_s"]) blind_timeout_s_ = yaml["buff_blind_timeout_s"].as<double>();
  if (yaml["buff_track_reset_timeout_s"]) {
    track_reset_timeout_s_ = yaml["buff_track_reset_timeout_s"].as<double>();
  }
  if (yaml["buff_switch_confirm_frames"]) {
    switch_confirm_frames_ = yaml["buff_switch_confirm_frames"].as<int>();
    adjacent_switch_confirm_frames_ = switch_confirm_frames_;
  }
  if (yaml["buff_same_slot_confirm_frames"]) {
    same_slot_confirm_frames_ = yaml["buff_same_slot_confirm_frames"].as<int>();
  }
  if (yaml["buff_adjacent_switch_confirm_frames"]) {
    adjacent_switch_confirm_frames_ = yaml["buff_adjacent_switch_confirm_frames"].as<int>();
  }
  if (yaml["buff_adjacent_switch_delay_s"]) {
    adjacent_switch_delay_s_ = yaml["buff_adjacent_switch_delay_s"].as<double>();
  }
  if (yaml["buff_slot_tolerance_deg"]) {
    slot_tolerance_rad_ = yaml["buff_slot_tolerance_deg"].as<double>() / 57.3;
  }
  if (yaml["buff_switch_pair_angle_gate_deg"]) {
    switch_pair_angle_gate_rad_ =
      yaml["buff_switch_pair_angle_gate_deg"].as<double>() / 57.3;
  }
  if (yaml["buff_switch_pair_ratio_min"]) {
    switch_pair_ratio_min_ = yaml["buff_switch_pair_ratio_min"].as<double>();
  }
  if (yaml["buff_switch_pair_ratio_max"]) {
    switch_pair_ratio_max_ = yaml["buff_switch_pair_ratio_max"].as<double>();
  }
  if (yaml["buff_big_speed_phase_window"]) {
    BUFF_BIG_SPEED_PHASE_WINDOW = yaml["buff_big_speed_phase_window"].as<int>();
  }
  if (yaml["buff_big_speed_min_span_s"]) {
    BUFF_BIG_SPEED_MIN_SPAN_S = yaml["buff_big_speed_min_span_s"].as<double>();
  }
  if (yaml["buff_big_fit_min_span_s"]) {
    BUFF_BIG_FIT_MIN_SPAN_S = yaml["buff_big_fit_min_span_s"].as<double>();
  }
  if (yaml["buff_big_fit_min_inlier_ratio"]) {
    BUFF_BIG_FIT_MIN_INLIER_RATIO = yaml["buff_big_fit_min_inlier_ratio"].as<double>();
  }
  if (yaml["buff_big_fit_max_rms"]) {
    BUFF_BIG_FIT_MAX_RMS = yaml["buff_big_fit_max_rms"].as<double>();
  }
  if (yaml["buff_big_fit_blend_s"]) {
    BUFF_BIG_FIT_BLEND_S = yaml["buff_big_fit_blend_s"].as<double>();
  }
  BUFF_BLIND_TIMEOUT_S = blind_timeout_s_;

  BuffTrackBank::Config track_config;
  track_config.confirm_hits =
    yaml["buff_track_confirm_hits"] ? yaml["buff_track_confirm_hits"].as<int>() : 2;
  track_config.recovery_hits =
    yaml["buff_track_recovery_hits"] ? yaml["buff_track_recovery_hits"].as<int>() : 2;
  track_config.association_gate_rad = track_gate_max_rad_;
  track_config.point_residual_gate_px = temporal_residual_gate_px_;
  track_config.control_blind_timeout_s =
    yaml["buff_control_blind_timeout_s"]
      ? yaml["buff_control_blind_timeout_s"].as<double>()
      : 0.100;
  track_config.retention_timeout_s =
    yaml["buff_track_retention_s"] ? yaml["buff_track_retention_s"].as<double>() : 0.400;
  track_config.spawn_keypoint_threshold = keypoint_threshold_;
  center_retention_s_ = track_config.retention_timeout_s;
  blind_timeout_s_ = track_config.control_blind_timeout_s;
  BUFF_BLIND_TIMEOUT_S = blind_timeout_s_;
  BUFF_TRACK_RETENTION_S = track_config.retention_timeout_s;
  BUFF_DIRECTION_CONFIRM_INTERVALS =
    yaml["buff_direction_confirm_intervals"]
      ? yaml["buff_direction_confirm_intervals"].as<int>()
      : 3;
  track_bank_.configure(track_config);
}

void Buff_Detector::handle_img(const cv::Mat & bgr_img, cv::Mat & dilated_img)
{
  cv::Mat gray_img;
  cv::cvtColor(bgr_img, gray_img, cv::COLOR_BGR2GRAY);

  cv::Mat binary_img;
  cv::threshold(gray_img, binary_img, 100, 255, cv::THRESH_BINARY);

  cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));
  cv::dilate(binary_img, dilated_img, kernel, cv::Point(-1, -1), 1);
}

cv::Point2f Buff_Detector::get_r_center(std::vector<FanBlade> & fanblades, cv::Mat & bgr_img)
{
  if (fanblades.empty()) {
    tools::logger()->debug("[Buff_Detector] 无法计算r_center!");
    return {0, 0};
  }

  cv::Point2f r_center_t = {0, 0};
  int valid_count = 0;
  for (auto & fanblade : fanblades) {
    if (fanblade.fan_center != cv::Point2f(0, 0)) {
      const auto axis = normalize_or(fanblade.fan_center - fanblade.center, {0.0f, 1.0f});
      r_center_t += fanblade.center +
                    axis * static_cast<float>(
                             cv::norm(fanblade.fan_center - fanblade.center) * 2.0);
      valid_count++;
    }
  }
  if (valid_count == 0) return fanblades.front().center;
  r_center_t *= 1.0f / static_cast<float>(valid_count);

  cv::Mat dilated_img;
  handle_img(bgr_img, dilated_img);
  double radius = cv::norm(fanblades[0].center - r_center_t) * 0.25;
  cv::Mat mask = cv::Mat::zeros(dilated_img.size(), CV_8U);
  circle(mask, r_center_t, radius, cv::Scalar(255), -1);
  bitwise_and(dilated_img, mask, dilated_img);

  std::vector<std::vector<cv::Point>> contours;
  auto r_center = r_center_t;
  cv::findContours(dilated_img, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);
  double ratio_1 = INF;
  for (auto & it : contours) {
    auto rotated_rect = cv::minAreaRect(it);
    double ratio = rotated_rect.size.height > rotated_rect.size.width
                     ? rotated_rect.size.height / rotated_rect.size.width
                     : rotated_rect.size.width / rotated_rect.size.height;
    ratio += cv::norm(rotated_rect.center - r_center_t) / std::max(radius / 3.0, 1.0);
    if (ratio < ratio_1) {
      ratio_1 = ratio;
      r_center = rotated_rect.center;
    }
  }
  return r_center;
}

void Buff_Detector::handle_lose()
{
  lose_++;
  if (lose_ >= LOSE_MAX) {
    status_ = LOSE;
  } else {
    status_ = TEM_LOSE;
  }
}

std::optional<Buff_Detector::CenterEstimate> Buff_Detector::select_r_center(
  const std::vector<YOLO11_BUFF::Object> & results,
  std::chrono::steady_clock::time_point timestamp)
{
  const YOLO11_BUFF::Object * best_center = nullptr;
  for (const auto & result : results) {
    if (result.label != RUNE_CENTER || !valid_keypoints(result, keypoint_threshold_)) continue;
    if (best_center == nullptr || result.prob > best_center->prob) best_center = &result;
  }

  if (!has_last_r_center_) {
    if (best_center == nullptr) return std::nullopt;
    last_r_center_ = mean_points(best_center->kpt);
    has_last_r_center_ = true;
    center_lost_count_ = 0;
    last_r_center_time_ = timestamp;
    last_r_center_seen_time_ = timestamp;
    return CenterEstimate{last_r_center_, RuneCenterSource::DETECTED};
  }

  const double dt = std::max(0.0, seconds_between(timestamp, last_r_center_time_));
  const cv::Point2f predicted = last_r_center_ + r_center_velocity_ * static_cast<float>(dt);
  if (best_center != nullptr) {
    const cv::Point2f observed = mean_points(best_center->kpt);
    const double residual = cv::norm(observed - predicted);
    if (residual <= center_innovation_gate_px_) {
      if (dt > 1e-4) {
        cv::Point2f measured_velocity = (observed - last_r_center_) * static_cast<float>(1.0 / dt);
        const double speed = cv::norm(measured_velocity);
        if (speed > 1500.0) measured_velocity *= static_cast<float>(1500.0 / speed);
        r_center_velocity_ = 0.65f * r_center_velocity_ + 0.35f * measured_velocity;
      }
      // A valid center observation is the current-frame geometric anchor. Keep
      // velocity prediction for gating and dropouts, but do not add image-space
      // lag while the center is visible.
      last_r_center_ = observed;
      last_r_center_time_ = timestamp;
      last_r_center_seen_time_ = timestamp;
      center_lost_count_ = 0;
      has_pending_r_center_ = false;
      pending_r_center_hits_ = 0;
      return CenterEstimate{last_r_center_, RuneCenterSource::DETECTED};
    }

    if (
      has_pending_r_center_ &&
      cv::norm(observed - pending_r_center_) <= center_innovation_gate_px_) {
      pending_r_center_hits_++;
      pending_r_center_ = observed;
    } else {
      has_pending_r_center_ = true;
      pending_r_center_ = observed;
      pending_r_center_hits_ = 1;
    }
    if (pending_r_center_hits_ >= center_recovery_hits_) {
      last_r_center_ = pending_r_center_;
      r_center_velocity_ = {0.0f, 0.0f};
      last_r_center_time_ = timestamp;
      last_r_center_seen_time_ = timestamp;
      has_pending_r_center_ = false;
      pending_r_center_hits_ = 0;
      return CenterEstimate{last_r_center_, RuneCenterSource::DETECTED};
    }
  }

  if (seconds_between(timestamp, last_r_center_seen_time_) <= center_retention_s_) {
    last_r_center_ = predicted;
    last_r_center_time_ = timestamp;
    center_lost_count_++;
    return CenterEstimate{last_r_center_, RuneCenterSource::PREDICTED};
  }

  return std::nullopt;
}

std::vector<BuffObservation> Buff_Detector::build_candidates(
  const std::vector<YOLO11_BUFF::Object> & results, const cv::Point2f & r_center) const
{
  struct Detection
  {
    const YOLO11_BUFF::Object * object;
    cv::Point2f center;
  };
  struct PairEdge
  {
    int target;
    int fan;
    double angle_error;
    double ratio;
    double score;
  };

  std::vector<Detection> targets;
  std::vector<Detection> fans;
  for (const auto & result : results) {
    if (!valid_keypoints(result, hard_keypoint_threshold_)) continue;
    if (result.label == INACTIVE_TARGET) targets.push_back({&result, mean_points(result.kpt)});
    if (result.label == INACTIVE_FAN) fans.push_back({&result, mean_points(result.kpt)});
  }

  std::vector<PairEdge> edges;
  for (int ti = 0; ti < static_cast<int>(targets.size()); ++ti) {
    const auto & target = targets[ti];
    const double target_radius = cv::norm(target.center - r_center);
    if (target_radius < 3.0) continue;
    for (int fi = 0; fi < static_cast<int>(fans.size()); ++fi) {
      const auto & fan = fans[fi];
      const double fan_radius = cv::norm(fan.center - r_center);
      const cv::Point2f r_to_fan = fan.center - r_center;
      const cv::Point2f fan_to_target = target.center - fan.center;
      const double fan_target_distance = cv::norm(fan_to_target);
      if (fan_radius < 3.0 || fan_target_distance < 3.0 || target_radius <= fan_radius) continue;

      const cv::Point2f u = normalize_or(r_to_fan, {0.0f, 1.0f});
      const cv::Point2f v = normalize_or(fan_to_target, {0.0f, 1.0f});
      const double direction_dot = std::clamp(static_cast<double>(dot2d(u, v)), -1.0, 1.0);
      if (direction_dot <= 0.0) continue;
      const double angle_error = std::acos(direction_dot);
      const double ratio = fan_target_distance / target_radius;
      if (
        angle_error > pair_angle_gate_rad_ || ratio < pair_ratio_min_ ||
        ratio > pair_ratio_max_) {
        continue;
      }

      const double confidence = 0.5 * (target.object->prob + fan.object->prob);
      const double angle_term = angle_error / std::max(pair_angle_gate_rad_, 1e-6);
      const double ratio_term = (ratio - pair_ratio_center_) / 0.20;
      const double score = angle_term * angle_term + ratio_term * ratio_term + (1.0 - confidence);
      edges.push_back({ti, fi, angle_error, ratio, score});
    }
  }

  std::vector<int> current_match(targets.size(), -1);
  std::vector<int> best_match(targets.size(), -1);
  std::vector<bool> used_fans(fans.size(), false);
  int best_count = -1;
  double best_score = std::numeric_limits<double>::max();
  std::function<void(int, int, double)> assign = [&](int ti, int count, double score) {
    if (ti == static_cast<int>(targets.size())) {
      if (count > best_count || (count == best_count && score < best_score)) {
        best_count = count;
        best_score = score;
        best_match = current_match;
      }
      return;
    }

    current_match[ti] = -1;
    assign(ti + 1, count, score);
    for (int ei = 0; ei < static_cast<int>(edges.size()); ++ei) {
      const auto & edge = edges[ei];
      if (edge.target != ti || used_fans[edge.fan]) continue;
      current_match[ti] = ei;
      used_fans[edge.fan] = true;
      assign(ti + 1, count + 1, score + edge.score);
      used_fans[edge.fan] = false;
      current_match[ti] = -1;
    }
  };
  assign(0, 0, 0.0);

  std::vector<BuffObservation> candidates;
  std::vector<bool> matched_targets(targets.size(), false);
  std::vector<bool> matched_fans(fans.size(), false);
  for (int ti = 0; ti < static_cast<int>(best_match.size()); ++ti) {
    if (best_match[ti] < 0) continue;
    const auto & edge = edges[best_match[ti]];
    const auto & target = targets[ti];
    const auto & fan = fans[edge.fan];
    const cv::Point2f axis_to_center = normalize_or(r_center - target.center, {0.0f, 1.0f});
    const auto ordered_target = order_target_points(target.object->kpt, target.center, axis_to_center);
    const auto ordered_fan = order_rect_points(fan.object->kpt, fan.center, axis_to_center);
    if (!ordered_target.has_value() || !valid_ordered_quad(*ordered_target) ||
        !valid_ordered_quad(ordered_fan)) {
      continue;
    }

    BuffObservation observation;
    observation.type = BuffObservationType::FULL;
    observation.target_points = *ordered_target;
    observation.fan_points = ordered_fan;
    observation.raw_target_points = observation.target_points;
    observation.raw_fan_points = observation.fan_points;
    observation.target_center = target.center;
    observation.fan_center = fan.center;
    observation.target_center_observed = true;
    observation.fan_center_observed = true;
    observation.angle = angle_around(target.center, r_center);
    observation.pair_angle_error = edge.angle_error;
    observation.pair_distance_ratio = edge.ratio;
    observation.confidence = 0.5f * (target.object->prob + fan.object->prob);
    observation.min_keypoint_confidence = std::min(
      minimum_keypoint_confidence(*target.object), minimum_keypoint_confidence(*fan.object));
    observation.quad_quality =
      std::min(quad_quality_score(*ordered_target), quad_quality_score(ordered_fan));
    observation.association_cost = edge.score;
    if (observation.min_keypoint_confidence < keypoint_threshold_) {
      const double confidence_span = std::max(
        static_cast<double>(keypoint_threshold_ - hard_keypoint_threshold_), 1e-3);
      observation.keypoint_noise_scale =
        1.0 + 2.0 * (keypoint_threshold_ - observation.min_keypoint_confidence) / confidence_span;
    }
    candidates.emplace_back(std::move(observation));
    matched_targets[ti] = true;
    matched_fans[edge.fan] = true;
  }

  for (int ti = 0; ti < static_cast<int>(targets.size()); ++ti) {
    if (matched_targets[ti]) continue;
    const auto & target = targets[ti];
    const cv::Point2f axis_to_center = normalize_or(r_center - target.center, {0.0f, 1.0f});
    const auto ordered_target = order_target_points(target.object->kpt, target.center, axis_to_center);
    if (!ordered_target.has_value() || !valid_ordered_quad(*ordered_target)) continue;

    BuffObservation observation;
    observation.type = BuffObservationType::TARGET_ONLY;
    observation.target_points = *ordered_target;
    observation.raw_target_points = observation.target_points;
    observation.target_center = target.center;
    observation.target_center_observed = true;
    observation.fan_center = r_center + 0.49f * (target.center - r_center);
    observation.angle = angle_around(target.center, r_center);
    observation.confidence = target.object->prob;
    observation.min_keypoint_confidence = minimum_keypoint_confidence(*target.object);
    observation.quad_quality = quad_quality_score(*ordered_target);
    observation.association_cost = 1.0 - observation.confidence;
    if (observation.min_keypoint_confidence < keypoint_threshold_) {
      const double confidence_span = std::max(
        static_cast<double>(keypoint_threshold_ - hard_keypoint_threshold_), 1e-3);
      observation.keypoint_noise_scale =
        1.0 + 2.0 * (keypoint_threshold_ - observation.min_keypoint_confidence) / confidence_span;
    }
    candidates.emplace_back(std::move(observation));
  }

  for (int fi = 0; fi < static_cast<int>(fans.size()); ++fi) {
    if (matched_fans[fi]) continue;
    const auto & fan = fans[fi];
    const cv::Point2f axis_to_center = normalize_or(r_center - fan.center, {0.0f, 1.0f});
    const auto ordered_fan = order_rect_points(fan.object->kpt, fan.center, axis_to_center);
    if (!valid_ordered_quad(ordered_fan)) continue;

    BuffObservation observation;
    observation.type = BuffObservationType::FAN_ONLY;
    observation.fan_points = ordered_fan;
    observation.raw_fan_points = observation.fan_points;
    observation.fan_center = fan.center;
    observation.fan_center_observed = true;
    observation.target_center = r_center + (1.0f / 0.49f) * (fan.center - r_center);
    observation.angle = angle_around(fan.center, r_center);
    observation.confidence = fan.object->prob;
    observation.min_keypoint_confidence = minimum_keypoint_confidence(*fan.object);
    observation.quad_quality = quad_quality_score(ordered_fan);
    observation.association_cost = 1.0 - observation.confidence;
    if (observation.min_keypoint_confidence < keypoint_threshold_) {
      const double confidence_span = std::max(
        static_cast<double>(keypoint_threshold_ - hard_keypoint_threshold_), 1e-3);
      observation.keypoint_noise_scale =
        1.0 + 2.0 * (keypoint_threshold_ - observation.min_keypoint_confidence) / confidence_span;
    }
    candidates.emplace_back(std::move(observation));
  }

  std::sort(candidates.begin(), candidates.end(), [](const BuffObservation & a, const BuffObservation & b) {
    if (observation_rank(a.type) != observation_rank(b.type)) {
      return observation_rank(a.type) < observation_rank(b.type);
    }
    return a.confidence > b.confidence;
  });
  return candidates;
}

std::optional<BuffObservation> Buff_Detector::select_locked_candidate(
  const std::vector<BuffObservation> & candidates,
  std::chrono::steady_clock::time_point timestamp)
{
  if (
    has_locked_target_ && seconds_between(timestamp, last_seen_time_) > track_reset_timeout_s_) {
    has_locked_target_ = false;
    has_temporal_candidate_ = false;
    reset_pending_switch();
  }

  if (candidates.empty()) {
    if (has_locked_target_) lost_locked_count_++;
    reset_pending_switch();
    return std::nullopt;
  }

  auto best_initial = std::min_element(
    candidates.begin(), candidates.end(), [](const BuffObservation & a, const BuffObservation & b) {
      if (observation_rank(a.type) != observation_rank(b.type)) {
        return observation_rank(a.type) < observation_rank(b.type);
      }
      const double a_score = a.association_cost + (1.0 - a.confidence);
      const double b_score = b.association_cost + (1.0 - b.confidence);
      return a_score < b_score;
    });

  if (!has_locked_target_) {
    has_locked_target_ = true;
    locked_track_id_ = next_track_id_++;
    lost_locked_count_ = 0;
    reset_pending_switch();
    angular_velocity_ = 0.0;
    last_locked_angle_ = best_initial->angle;
    last_locked_time_ = timestamp;
    last_seen_time_ = timestamp;
    gate_episode_active_ = false;
    auto selected = *best_initial;
    selected.track_id = locked_track_id_;
    selected.prediction_error = 0.0;
    return selected;
  }

  const double prediction_dt = std::max(0.0, seconds_between(timestamp, last_locked_time_));
  const double predicted_angle = last_locked_angle_ + angular_velocity_ * prediction_dt;
  auto best_by_prediction = std::min_element(
    candidates.begin(), candidates.end(), [&](const BuffObservation & a, const BuffObservation & b) {
      const double a_error = std::abs(tools::limit_rad(a.angle - predicted_angle));
      const double b_error = std::abs(tools::limit_rad(b.angle - predicted_angle));
      const double a_score = a_error + observation_rank(a.type) * 0.02 +
                             (1.0 - a.confidence) * 0.01 + a.association_cost * 0.01;
      const double b_score = b_error + observation_rank(b.type) * 0.02 +
                             (1.0 - b.confidence) * 0.01 + b.association_cost * 0.01;
      return a_score < b_score;
    });
  const double best_error =
    std::abs(tools::limit_rad(best_by_prediction->angle - predicted_angle));
  const double lost_time = std::max(0.0, seconds_between(timestamp, last_seen_time_));
  const double gate_ratio = std::clamp(lost_time / std::max(blind_timeout_s_, 1e-3), 0.0, 1.0);
  const double track_gate =
    track_gate_min_rad_ + gate_ratio * (track_gate_max_rad_ - track_gate_min_rad_);
  if (best_error <= track_gate) {
    const double observation_dt = std::max(0.0, seconds_between(timestamp, last_locked_time_));
    if (observation_dt > 1e-4) {
      const double measured_velocity =
        tools::limit_rad(best_by_prediction->angle - last_locked_angle_) / observation_dt;
      if (std::abs(measured_velocity) <= 3.0) {
        angular_velocity_ = 0.8 * angular_velocity_ + 0.2 * measured_velocity;
      }
    }
    lost_locked_count_ = 0;
    reset_pending_switch();
    last_locked_angle_ = best_by_prediction->angle;
    last_locked_time_ = timestamp;
    last_seen_time_ = timestamp;
    gate_episode_active_ = false;
    auto selected = *best_by_prediction;
    selected.track_id = locked_track_id_;
    selected.prediction_error = best_error;
    selected.slot_offset = 0;
    selected.slot_residual = best_error;
    return selected;
  }

  lost_locked_count_++;
  if (!gate_episode_active_) {
    gate_episode_active_ = true;
    gate_failure_count_++;
    tools::logger()->debug(
      "[Buff_Detector] candidate gated, err {:.1f} deg, track {}, blind {:.0f}ms",
      best_error * 57.3, locked_track_id_, lost_time * 1000.0);
  }
  if (lost_time <= blind_timeout_s_) {
    reset_pending_switch();
    return std::nullopt;
  }

  const BuffObservation * switch_candidate = nullptr;
  int switch_slot_offset = 0;
  double switch_slot_residual = std::numeric_limits<double>::max();
  double switch_score = std::numeric_limits<double>::max();
  for (const auto & candidate : candidates) {
    if (candidate.type != BuffObservationType::FULL) continue;

    const double delta = tools::limit_rad(candidate.angle - predicted_angle);
    int slot_offset = static_cast<int>(std::round(delta / RUNE_SLOT_ANGLE));
    if (slot_offset > 2) slot_offset -= 5;
    if (slot_offset < -2) slot_offset += 5;
    const double slot_residual =
      std::abs(tools::limit_rad(delta - slot_offset * RUNE_SLOT_ANGLE));
    if (slot_residual > slot_tolerance_rad_) continue;

    if (slot_offset != 0) {
      if (lost_time < adjacent_switch_delay_s_) continue;
      if (
        candidate.pair_angle_error > switch_pair_angle_gate_rad_ ||
        candidate.pair_distance_ratio < switch_pair_ratio_min_ ||
        candidate.pair_distance_ratio > switch_pair_ratio_max_) {
        continue;
      }
    }

    const double candidate_score =
      slot_residual + candidate.association_cost * 0.02 + (1.0 - candidate.confidence) * 0.01;
    if (candidate_score < switch_score) {
      switch_score = candidate_score;
      switch_candidate = &candidate;
      switch_slot_offset = slot_offset;
      switch_slot_residual = slot_residual;
    }
  }

  if (switch_candidate == nullptr) {
    reset_pending_switch();
    return std::nullopt;
  }

  if (!has_pending_switch_ || pending_slot_offset_ != switch_slot_offset) {
    has_pending_switch_ = true;
    pending_slot_offset_ = switch_slot_offset;
    pending_switch_angle_ = switch_candidate->angle;
    pending_switch_time_ = timestamp;
    switch_confirm_count_ = 1;
  } else {
    const double pending_dt = std::max(0.0, seconds_between(timestamp, pending_switch_time_));
    const double pending_prediction = pending_switch_angle_ + angular_velocity_ * pending_dt;
    const double pending_error =
      std::abs(tools::limit_rad(switch_candidate->angle - pending_prediction));
    if (pending_error <= track_gate_max_rad_) {
      pending_switch_angle_ = switch_candidate->angle;
      pending_switch_time_ = timestamp;
      switch_confirm_count_++;
    } else {
      pending_switch_angle_ = switch_candidate->angle;
      pending_switch_time_ = timestamp;
      switch_confirm_count_ = 1;
    }
  }

  const int required_confirm_frames =
    switch_slot_offset == 0 ? same_slot_confirm_frames_ : adjacent_switch_confirm_frames_;
  if (switch_confirm_count_ < required_confirm_frames) return std::nullopt;

  const int old_track_id = locked_track_id_;
  if (switch_slot_offset != 0) locked_track_id_ = next_track_id_++;
  lost_locked_count_ = 0;
  reset_pending_switch();
  last_locked_angle_ = switch_candidate->angle;
  last_locked_time_ = timestamp;
  last_seen_time_ = timestamp;
  gate_episode_active_ = false;
  if (switch_slot_offset != 0) {
    confirmed_switch_count_++;
    tools::logger()->debug(
      "[Buff_Detector] confirmed slot switch {} -> {}, offset {}, blind {:.0f}ms "
      "slot {:.1f}deg pair {:.1f}deg ratio {:.2f}",
      old_track_id, locked_track_id_, switch_slot_offset, lost_time * 1000.0,
      switch_slot_residual * 57.3, switch_candidate->pair_angle_error * 57.3,
      switch_candidate->pair_distance_ratio);
  } else {
    tools::logger()->debug("[Buff_Detector] recovered track {} in same slot", locked_track_id_);
  }
  auto selected = *switch_candidate;
  selected.track_id = locked_track_id_;
  selected.prediction_error = switch_slot_residual;
  selected.slot_offset = switch_slot_offset;
  selected.slot_residual = switch_slot_residual;
  return selected;
}

void Buff_Detector::reset_pending_switch()
{
  has_pending_switch_ = false;
  pending_slot_offset_ = 0;
  switch_confirm_count_ = 0;
}

std::optional<BuffObservation> Buff_Detector::stabilize_candidate(BuffObservation candidate)
{
  const bool low_confidence = candidate.min_keypoint_confidence < keypoint_threshold_;
  if (!has_temporal_candidate_) {
    if (low_confidence) {
      temporal_reject_count_++;
      return std::nullopt;
    }
    return candidate;
  }

  const double angle_delta = tools::limit_rad(candidate.angle - temporal_angle_);
  if (std::abs(angle_delta) > track_gate_max_rad_) {
    if (low_confidence) {
      temporal_reject_count_++;
      return std::nullopt;
    }
    return candidate;
  }

  double max_residual = 0.0;
  auto filter_points = [&](std::vector<cv::Point2f> & observed,
                           const std::vector<cv::Point2f> & previous) -> bool {
    if (observed.empty()) return true;
    if (previous.size() != 4) return !low_confidence;

    const auto predicted =
      rotate_points(previous, temporal_r_center_, candidate.r_center, angle_delta);
    const double previous_scale = quad_scale(predicted);
    const double observed_scale = quad_scale(observed);
    if (previous_scale < 1e-3 || observed_scale < 1e-3) return false;
    const double scale_ratio = observed_scale / previous_scale;
    if (scale_ratio < 0.65 || scale_ratio > 1.55) return false;

    for (size_t i = 0; i < observed.size(); ++i) {
      max_residual = std::max(max_residual, cv::norm(observed[i] - predicted[i]));
    }
    if (max_residual > temporal_residual_gate_px_) return false;

    const double confidence_range = std::max(1.0 - hard_keypoint_threshold_, 1e-3);
    const double confidence_ratio = std::clamp(
      (candidate.min_keypoint_confidence - hard_keypoint_threshold_) / confidence_range,
      0.0, 1.0);
    const double speed_boost =
      0.15 * std::clamp(std::abs(angle_delta) / (15.0 / 57.3), 0.0, 1.0);
    const double observation_weight =
      std::clamp(0.65 + 0.25 * confidence_ratio + speed_boost, 0.65, 0.90);
    for (size_t i = 0; i < observed.size(); ++i) {
      observed[i] = static_cast<float>(observation_weight) * observed[i] +
                    static_cast<float>(1.0 - observation_weight) * predicted[i];
    }
    return valid_ordered_quad(observed);
  };

  if (!filter_points(candidate.target_points, temporal_target_points_)) {
    temporal_reject_count_++;
    return std::nullopt;
  }
  if (!filter_points(candidate.fan_points, temporal_fan_points_)) {
    temporal_reject_count_++;
    return std::nullopt;
  }

  if (candidate.has_target()) candidate.target_center = mean_points(candidate.target_points);
  if (candidate.has_fan()) candidate.fan_center = mean_points(candidate.fan_points);
  candidate.angle = angle_around(
    candidate.has_target() ? candidate.target_center : candidate.fan_center, candidate.r_center);
  candidate.keypoint_temporal_residual = max_residual;
  const double target_quality =
    candidate.has_target() ? quad_quality_score(candidate.target_points) : 1.0;
  const double fan_quality = candidate.has_fan() ? quad_quality_score(candidate.fan_points) : 1.0;
  candidate.quad_quality = std::min(target_quality, fan_quality);
  return candidate;
}

void Buff_Detector::remember_temporal_candidate(const BuffObservation & candidate)
{
  const bool same_track = has_temporal_candidate_ && temporal_track_id_ == candidate.track_id;
  if (same_track) {
    const double angle_delta = tools::limit_rad(candidate.angle - temporal_angle_);
    if (!candidate.has_target() && temporal_target_points_.size() == 4) {
      temporal_target_points_ = rotate_points(
        temporal_target_points_, temporal_r_center_, candidate.r_center, angle_delta);
    }
    if (!candidate.has_fan() && temporal_fan_points_.size() == 4) {
      temporal_fan_points_ =
        rotate_points(temporal_fan_points_, temporal_r_center_, candidate.r_center, angle_delta);
    }
  } else {
    temporal_target_points_.clear();
    temporal_fan_points_.clear();
  }

  if (candidate.has_target()) {
    temporal_target_points_ = candidate.raw_target_points.empty() ? candidate.target_points
                                                                   : candidate.raw_target_points;
  }
  if (candidate.has_fan()) {
    temporal_fan_points_ =
      candidate.raw_fan_points.empty() ? candidate.fan_points : candidate.raw_fan_points;
  }
  temporal_track_id_ = candidate.track_id;
  temporal_angle_ = candidate.angle;
  temporal_r_center_ = candidate.r_center;
  has_temporal_candidate_ = true;
}

void Buff_Detector::reset_for_mode(BuffMode mode)
{
  if (has_current_mode_ && current_mode_ == mode) return;
  current_mode_ = mode;
  has_current_mode_ = true;
  track_bank_.reset();
  has_last_r_center_ = false;
  r_center_velocity_ = {0.0f, 0.0f};
  has_pending_r_center_ = false;
  pending_r_center_hits_ = 0;
  status_ = LOSE;
  lose_ = 0;
}

std::vector<BuffObservation> Buff_Detector::detect_tracks_impl(
  cv::Mat & bgr_img, bool single_candidate, BuffMode mode,
  std::chrono::steady_clock::time_point timestamp)
{
  reset_for_mode(mode);
  std::vector<YOLO11_BUFF::Object> results = single_candidate ? MODE_.get_onecandidatebox(bgr_img)
                                                              : MODE_.get_multicandidateboxes(bgr_img);
  if (results.empty()) {
    track_bank_.update({}, timestamp, mode);
    handle_lose();
    return {};
  }

  auto r_center = select_r_center(results, timestamp);
  if (!r_center.has_value()) {
    track_bank_.update({}, timestamp, mode);
    handle_lose();
    return {};
  }

  auto candidates = build_candidates(results, r_center->point);
  for (auto & candidate : candidates) {
    candidate.r_center = r_center->point;
    candidate.center_source = r_center->source;
    candidate.timestamp = timestamp;
  }
  auto tracked = track_bank_.update(candidates, timestamp, mode);
  if (tracked.empty()) {
    handle_lose();
    return {};
  }

  tools::draw_point(bgr_img, r_center->point, {0, 0, 255}, 3);
  for (const auto & observation : tracked) {
    if (observation.target_center_observed) {
      tools::draw_point(
        bgr_img, observation.target_center,
        observation.primary ? cv::Scalar(255, 0, 255) : cv::Scalar(255, 255, 0), 3);
    }
    if (observation.fan_center_observed) {
      tools::draw_point(
        bgr_img, observation.fan_center,
        observation.primary ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 180, 255), 3);
    }
  }

  status_ = TRACK;
  lose_ = 0;
  return tracked;
}

std::optional<BuffObservation> Buff_Detector::detect_impl(
  cv::Mat & bgr_img, bool single_candidate, BuffMode mode,
  std::chrono::steady_clock::time_point timestamp)
{
  auto tracks = detect_tracks_impl(bgr_img, single_candidate, mode, timestamp);
  if (tracks.empty()) return std::nullopt;
  return tracks.front();
}

std::vector<BuffObservation> Buff_Detector::detect_tracks(
  cv::Mat & bgr_img, BuffMode mode, std::chrono::steady_clock::time_point timestamp)
{
  return detect_tracks_impl(bgr_img, false, mode, timestamp);
}

std::vector<BuffObservation> Buff_Detector::detect_tracks(cv::Mat & bgr_img, BuffMode mode)
{
  return detect_tracks(bgr_img, mode, std::chrono::steady_clock::now());
}

std::optional<BuffObservation> Buff_Detector::detect_24(
  cv::Mat & bgr_img, BuffMode mode, std::chrono::steady_clock::time_point timestamp)
{
  return detect_impl(bgr_img, false, mode, timestamp);
}

std::optional<BuffObservation> Buff_Detector::detect_24(
  cv::Mat & bgr_img, std::chrono::steady_clock::time_point timestamp)
{
  return detect_impl(bgr_img, false, BuffMode::SMALL, timestamp);
}

std::optional<BuffObservation> Buff_Detector::detect_24(cv::Mat & bgr_img)
{
  return detect_24(bgr_img, std::chrono::steady_clock::now());
}

std::optional<BuffObservation> Buff_Detector::detect(
  cv::Mat & bgr_img, BuffMode mode, std::chrono::steady_clock::time_point timestamp)
{
  return detect_impl(bgr_img, false, mode, timestamp);
}

std::optional<BuffObservation> Buff_Detector::detect(
  cv::Mat & bgr_img, std::chrono::steady_clock::time_point timestamp)
{
  return detect_impl(bgr_img, false, BuffMode::SMALL, timestamp);
}

std::optional<BuffObservation> Buff_Detector::detect(cv::Mat & bgr_img)
{
  return detect(bgr_img, std::chrono::steady_clock::now());
}

std::optional<BuffObservation> Buff_Detector::detect_debug(cv::Mat & bgr_img, cv::Point2f)
{
  return detect(bgr_img, std::chrono::steady_clock::now());
}

}  // namespace auto_buff
