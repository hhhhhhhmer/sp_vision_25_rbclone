#ifndef AUTO_BUFF__TRACK_BANK_HPP
#define AUTO_BUFF__TRACK_BANK_HPP

#include <chrono>
#include <optional>
#include <vector>

#include "buff_type.hpp"

namespace auto_buff
{
class BuffTrackBank
{
public:
  struct Config
  {
    int confirm_hits = 2;
    int recovery_hits = 2;
    double association_gate_rad = 25.0 / 57.3;
    double point_residual_gate_px = 10.0;
    double control_blind_timeout_s = 0.100;
    double retention_timeout_s = 0.400;
    float spawn_keypoint_threshold = 0.30f;
  };

  BuffTrackBank();
  explicit BuffTrackBank(const Config & config);

  void configure(const Config & config);
  void reset();

  std::vector<BuffObservation> update(
    const std::vector<BuffObservation> & candidates, std::chrono::steady_clock::time_point timestamp,
    BuffMode mode);

  int primary_track_id() const { return primary_track_id_; }
  int confirmed_switch_count() const { return confirmed_switch_count_; }
  int temporal_reject_count() const { return temporal_reject_count_; }
  size_t track_count() const { return tracks_.size(); }

private:
  struct Track
  {
    int id = -1;
    int hits = 0;
    bool confirmed = false;
    double angle = 0.0;
    double angular_velocity = 0.0;
    std::chrono::steady_clock::time_point last_update{};
    std::chrono::steady_clock::time_point last_seen{};
    BuffObservation observation;
    std::optional<BuffObservation> pending_recovery;
    int recovery_hits = 0;
  };

  struct Edge
  {
    size_t track_index = 0;
    size_t candidate_index = 0;
    double cost = 0.0;
  };

  int capacity(BuffMode mode) const;
  double predicted_angle(const Track & track, std::chrono::steady_clock::time_point timestamp) const;
  bool stabilize_for_track(BuffObservation & candidate, const Track & track) const;
  bool recovery_consistent(
    const BuffObservation & candidate, const BuffObservation & pending) const;
  void update_track(
    Track & track, BuffObservation candidate, std::chrono::steady_clock::time_point timestamp);
  bool can_spawn(const BuffObservation & candidate) const;
  void spawn_track(
    BuffObservation candidate, std::chrono::steady_clock::time_point timestamp);
  void purge(std::chrono::steady_clock::time_point timestamp);

  Config config_;
  std::vector<Track> tracks_;
  BuffMode mode_ = BuffMode::SMALL;
  bool has_mode_ = false;
  int next_track_id_ = 1;
  int primary_track_id_ = -1;
  int primary_missing_frames_ = 0;
  int confirmed_switch_count_ = 0;
  int temporal_reject_count_ = 0;
};
}  // namespace auto_buff

#endif  // AUTO_BUFF__TRACK_BANK_HPP
