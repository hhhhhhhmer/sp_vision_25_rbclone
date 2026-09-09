#include "buff_track_bank.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>

#include "tools/math_tools.hpp"

namespace auto_buff
{
namespace
{
double seconds_between(
  std::chrono::steady_clock::time_point now, std::chrono::steady_clock::time_point before)
{
  return std::chrono::duration<double>(now - before).count();
}

cv::Point2f mean_points(const std::vector<cv::Point2f> & points)
{
  cv::Point2f sum{0.0f, 0.0f};
  for (const auto & point : points) sum += point;
  if (!points.empty()) sum *= 1.0f / static_cast<float>(points.size());
  return sum;
}

double quad_scale(const std::vector<cv::Point2f> & points)
{
  if (points.size() != 4) return 0.0;
  const auto center = mean_points(points);
  double scale = 0.0;
  for (const auto & point : points) scale += cv::norm(point - center);
  return scale / 4.0;
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
    const auto relative = point - old_center;
    rotated.emplace_back(
      new_center + cv::Point2f(c * relative.x - s * relative.y, s * relative.x + c * relative.y));
  }
  return rotated;
}

double max_point_residual(
  const std::vector<cv::Point2f> & observed, const std::vector<cv::Point2f> & predicted)
{
  if (observed.size() != predicted.size()) return std::numeric_limits<double>::infinity();
  double residual = 0.0;
  for (size_t i = 0; i < observed.size(); ++i) {
    residual = std::max(residual, cv::norm(observed[i] - predicted[i]));
  }
  return residual;
}
}  // namespace

BuffTrackBank::BuffTrackBank() = default;

BuffTrackBank::BuffTrackBank(const Config & config) : config_(config) {}

void BuffTrackBank::configure(const Config & config) { config_ = config; }

void BuffTrackBank::reset()
{
  tracks_.clear();
  primary_track_id_ = -1;
  primary_missing_frames_ = 0;
}

int BuffTrackBank::capacity(BuffMode mode) const { return mode == BuffMode::BIG ? 2 : 1; }

double BuffTrackBank::predicted_angle(
  const Track & track, std::chrono::steady_clock::time_point timestamp) const
{
  const double dt = std::max(0.0, seconds_between(timestamp, track.last_update));
  return track.angle + track.angular_velocity * dt;
}

bool BuffTrackBank::stabilize_for_track(BuffObservation & candidate, const Track & track) const
{
  const double angle_delta = tools::limit_rad(candidate.angle - track.angle);
  double max_residual = 0.0;

  auto stabilize_points = [&](std::vector<cv::Point2f> & points,
                              const std::vector<cv::Point2f> & previous) {
    if (points.empty() || previous.empty()) return true;
    if (points.size() != 4 || previous.size() != 4) return false;
    const auto predicted =
      rotate_points(previous, track.observation.r_center, candidate.r_center, angle_delta);
    const double previous_scale = quad_scale(predicted);
    const double current_scale = quad_scale(points);
    if (previous_scale < 1e-3 || current_scale < 1e-3) return false;
    const double scale_ratio = current_scale / previous_scale;
    if (scale_ratio < 0.65 || scale_ratio > 1.55) return false;
    max_residual = std::max(max_residual, max_point_residual(points, predicted));
    const double hard_residual_gate = std::max(30.0, 3.0 * config_.point_residual_gate_px);
    if (max_residual > hard_residual_gate) return false;

    const double residual_ratio =
      max_residual / std::max(config_.point_residual_gate_px, 1.0);
    if (residual_ratio > 1.0) {
      candidate.keypoint_noise_scale *= std::min(6.0, residual_ratio * residual_ratio);
    }
    const double observation_weight = std::clamp(
      0.75 + 0.15 * static_cast<double>(candidate.min_keypoint_confidence) -
        0.20 * std::max(0.0, residual_ratio - 1.0),
      0.50, 0.90);
    for (size_t i = 0; i < points.size(); ++i) {
      points[i] = static_cast<float>(observation_weight) * points[i] +
                  static_cast<float>(1.0 - observation_weight) * predicted[i];
    }
    return true;
  };

  if (!stabilize_points(candidate.target_points, track.observation.target_points)) return false;
  if (!stabilize_points(candidate.fan_points, track.observation.fan_points)) return false;
  if (candidate.has_target()) candidate.target_center = mean_points(candidate.target_points);
  if (candidate.has_fan()) candidate.fan_center = mean_points(candidate.fan_points);
  const auto point = candidate.has_target() ? candidate.target_center : candidate.fan_center;
  candidate.angle = std::atan2(point.y - candidate.r_center.y, point.x - candidate.r_center.x);
  candidate.keypoint_temporal_residual = max_residual;
  return true;
}

bool BuffTrackBank::recovery_consistent(
  const BuffObservation & candidate, const BuffObservation & pending) const
{
  if (candidate.type != BuffObservationType::FULL || pending.type != BuffObservationType::FULL) {
    return false;
  }
  const double angle_error = std::abs(tools::limit_rad(candidate.angle - pending.angle));
  if (angle_error > config_.association_gate_rad) return false;
  if (candidate.has_target() && pending.has_target()) {
    const double scale = std::max(quad_scale(pending.target_points), 1.0);
    if (cv::norm(candidate.target_center - pending.target_center) > std::max(20.0, 0.75 * scale)) {
      return false;
    }
  }
  return true;
}

void BuffTrackBank::update_track(
  Track & track, BuffObservation candidate, std::chrono::steady_clock::time_point timestamp)
{
  const double dt = std::max(0.0, seconds_between(timestamp, track.last_update));
  if (dt > 1e-4) {
    const double measured_velocity = tools::limit_rad(candidate.angle - track.angle) / dt;
    if (std::abs(measured_velocity) <= 3.0) {
      const double alpha = track.hits < 3 ? 0.65 : 0.30;
      track.angular_velocity =
        (1.0 - alpha) * track.angular_velocity + alpha * measured_velocity;
    }
  }
  track.angle = candidate.angle;
  track.last_update = timestamp;
  track.last_seen = timestamp;
  track.hits++;
  if (track.hits >= config_.confirm_hits) track.confirmed = true;
  candidate.track_id = track.id;
  candidate.track_status = track.confirmed ? BuffTrackStatus::CONFIRMED
                                           : BuffTrackStatus::TENTATIVE;
  track.observation = std::move(candidate);
  track.pending_recovery.reset();
  track.recovery_hits = 0;
}

bool BuffTrackBank::can_spawn(const BuffObservation & candidate) const
{
  return candidate.type == BuffObservationType::FULL &&
         candidate.min_keypoint_confidence >= config_.spawn_keypoint_threshold;
}

void BuffTrackBank::spawn_track(
  BuffObservation candidate, std::chrono::steady_clock::time_point timestamp)
{
  Track track;
  track.id = next_track_id_++;
  track.hits = 1;
  track.confirmed = config_.confirm_hits <= 1;
  track.angle = candidate.angle;
  track.last_update = timestamp;
  track.last_seen = timestamp;
  candidate.track_id = track.id;
  candidate.track_status = track.confirmed ? BuffTrackStatus::CONFIRMED
                                           : BuffTrackStatus::TENTATIVE;
  track.observation = std::move(candidate);
  tracks_.push_back(std::move(track));
}

void BuffTrackBank::purge(std::chrono::steady_clock::time_point timestamp)
{
  tracks_.erase(
    std::remove_if(tracks_.begin(), tracks_.end(), [&](const Track & track) {
      if (!track.confirmed) return seconds_between(timestamp, track.last_seen) > 0.100;
      return seconds_between(timestamp, track.last_seen) > config_.retention_timeout_s;
    }),
    tracks_.end());
  const bool primary_alive = std::any_of(tracks_.begin(), tracks_.end(), [&](const Track & track) {
    return track.id == primary_track_id_;
  });
  if (!primary_alive) primary_track_id_ = -1;
}

std::vector<BuffObservation> BuffTrackBank::update(
  const std::vector<BuffObservation> & candidates, std::chrono::steady_clock::time_point timestamp,
  BuffMode mode)
{
  if (!has_mode_ || mode != mode_) {
    reset();
    mode_ = mode;
    has_mode_ = true;
  }
  purge(timestamp);

  std::vector<Edge> edges;
  for (size_t ti = 0; ti < tracks_.size(); ++ti) {
    const double prediction = predicted_angle(tracks_[ti], timestamp);
    for (size_t ci = 0; ci < candidates.size(); ++ci) {
      const double angle_error = std::abs(tools::limit_rad(candidates[ci].angle - prediction));
      if (angle_error > config_.association_gate_rad) continue;
      const double type_penalty = candidates[ci].type == BuffObservationType::FULL ? 0.0 : 0.02;
      const double quality_penalty = (1.0 - candidates[ci].confidence) * 0.02 +
                                     candidates[ci].association_cost * 0.01;
      edges.push_back({ti, ci, angle_error + type_penalty + quality_penalty});
    }
  }
  std::sort(edges.begin(), edges.end(), [](const Edge & lhs, const Edge & rhs) {
    return lhs.cost < rhs.cost;
  });

  std::vector<bool> track_used(tracks_.size(), false);
  std::vector<bool> candidate_used(candidates.size(), false);
  std::vector<bool> visible(tracks_.size(), false);
  for (const auto & edge : edges) {
    if (track_used[edge.track_index] || candidate_used[edge.candidate_index]) continue;
    auto candidate = candidates[edge.candidate_index];
    auto & track = tracks_[edge.track_index];
    if (!stabilize_for_track(candidate, track)) {
      temporal_reject_count_++;
      continue;
    }
    update_track(track, std::move(candidate), timestamp);
    track_used[edge.track_index] = true;
    candidate_used[edge.candidate_index] = true;
    visible[edge.track_index] = true;
  }

  // A coherent two-frame displacement can recover a track without accepting the first jump.
  for (size_t ci = 0; ci < candidates.size(); ++ci) {
    if (candidate_used[ci] || !can_spawn(candidates[ci])) continue;
    Track * recovery_track = nullptr;
    for (size_t ti = 0; ti < tracks_.size(); ++ti) {
      if (track_used[ti] || !tracks_[ti].confirmed) continue;
      if (tracks_[ti].pending_recovery.has_value() &&
          recovery_consistent(candidates[ci], *tracks_[ti].pending_recovery)) {
        recovery_track = &tracks_[ti];
        break;
      }
    }
    if (recovery_track != nullptr) {
      recovery_track->recovery_hits++;
      if (recovery_track->recovery_hits >= config_.recovery_hits) {
        auto candidate = candidates[ci];
        recovery_track->angle = candidate.angle;
        recovery_track->angular_velocity = 0.0;
        update_track(*recovery_track, std::move(candidate), timestamp);
        const size_t ti = static_cast<size_t>(recovery_track - tracks_.data());
        track_used[ti] = true;
        visible[ti] = true;
        candidate_used[ci] = true;
      } else {
        recovery_track->pending_recovery = candidates[ci];
      }
      continue;
    }

    if (static_cast<int>(tracks_.size()) >= capacity(mode)) {
      auto replace = std::max_element(tracks_.begin(), tracks_.end(), [&](const Track & lhs, const Track & rhs) {
        return seconds_between(timestamp, lhs.last_seen) < seconds_between(timestamp, rhs.last_seen);
      });
      if (
        replace != tracks_.end() && replace->id != primary_track_id_ &&
        seconds_between(timestamp, replace->last_seen) > config_.control_blind_timeout_s) {
        const size_t replace_index = static_cast<size_t>(replace - tracks_.begin());
        if (!track_used[replace_index]) {
          tracks_.erase(replace);
          track_used.erase(track_used.begin() + replace_index);
          visible.erase(visible.begin() + replace_index);
        }
      }
    }
    if (static_cast<int>(tracks_.size()) < capacity(mode)) {
      spawn_track(candidates[ci], timestamp);
      candidate_used[ci] = true;
      track_used.push_back(true);
      visible.push_back(config_.confirm_hits <= 1);
    } else {
      // Keep one uncommitted recovery reference per unmatched established track.
      auto pending = std::find_if(tracks_.begin(), tracks_.end(), [&](const Track & track) {
        return track.confirmed && track.id == primary_track_id_;
      });
      if (pending != tracks_.end() && !pending->pending_recovery.has_value()) {
        pending->pending_recovery = candidates[ci];
        pending->recovery_hits = 1;
      }
    }
  }

  std::vector<size_t> visible_confirmed;
  for (size_t ti = 0; ti < tracks_.size(); ++ti) {
    if (ti < visible.size() && visible[ti] && tracks_[ti].confirmed) visible_confirmed.push_back(ti);
  }
  if (visible_confirmed.empty()) return {};

  const int old_primary_id = primary_track_id_;
  auto primary_it = std::find_if(visible_confirmed.begin(), visible_confirmed.end(), [&](size_t ti) {
    return tracks_[ti].id == primary_track_id_;
  });
  const bool old_primary_retained = std::any_of(tracks_.begin(), tracks_.end(), [&](const Track & track) {
    return track.id == old_primary_id;
  });
  if (
    old_primary_id >= 0 && primary_it == visible_confirmed.end() && old_primary_retained &&
    primary_missing_frames_ == 0) {
    primary_missing_frames_ = 1;
    std::vector<BuffObservation> backup_observations;
    backup_observations.reserve(visible_confirmed.size());
    for (const size_t ti : visible_confirmed) {
      auto observation = tracks_[ti].observation;
      observation.track_id = tracks_[ti].id;
      observation.track_status = BuffTrackStatus::CONFIRMED;
      observation.primary = false;
      observation.slot_offset = 0;
      backup_observations.push_back(std::move(observation));
    }
    return backup_observations;
  }
  if (primary_it == visible_confirmed.end()) {
    primary_it = std::min_element(visible_confirmed.begin(), visible_confirmed.end(), [&](size_t lhs, size_t rhs) {
      const auto & a = tracks_[lhs].observation;
      const auto & b = tracks_[rhs].observation;
      if (a.type != b.type) return a.type == BuffObservationType::FULL;
      return a.confidence > b.confidence;
    });
    primary_track_id_ = tracks_[*primary_it].id;
  }
  primary_missing_frames_ = 0;

  int switch_slot_offset = 0;
  double switch_slot_residual = 0.0;
  if (old_primary_id >= 0 && old_primary_id != primary_track_id_) {
    const auto old = std::find_if(tracks_.begin(), tracks_.end(), [&](const Track & track) {
      return track.id == old_primary_id;
    });
    if (old != tracks_.end()) {
      const double delta = tools::limit_rad(tracks_[*primary_it].angle - predicted_angle(*old, timestamp));
      switch_slot_offset = static_cast<int>(std::round(delta / RUNE_SLOT_ANGLE));
      switch_slot_offset = std::clamp(switch_slot_offset, -2, 2);
      switch_slot_residual =
        std::abs(tools::limit_rad(delta - switch_slot_offset * RUNE_SLOT_ANGLE));
    }
    confirmed_switch_count_++;
  }

  std::sort(visible_confirmed.begin(), visible_confirmed.end(), [&](size_t lhs, size_t rhs) {
    if (tracks_[lhs].id == primary_track_id_) return true;
    if (tracks_[rhs].id == primary_track_id_) return false;
    return tracks_[lhs].observation.confidence > tracks_[rhs].observation.confidence;
  });

  std::vector<BuffObservation> output;
  output.reserve(visible_confirmed.size());
  for (const size_t ti : visible_confirmed) {
    auto observation = tracks_[ti].observation;
    observation.track_id = tracks_[ti].id;
    observation.track_status = BuffTrackStatus::CONFIRMED;
    observation.primary = tracks_[ti].id == primary_track_id_;
    if (observation.primary && old_primary_id >= 0 && old_primary_id != primary_track_id_) {
      observation.slot_offset = switch_slot_offset;
      observation.slot_residual = switch_slot_residual;
    } else {
      observation.slot_offset = 0;
    }
    output.push_back(std::move(observation));
  }
  return output;
}
}  // namespace auto_buff
