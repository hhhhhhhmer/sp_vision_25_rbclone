#ifndef AUTO_BUFF__TRACK_HPP
#define AUTO_BUFF__TRACK_HPP

#include <yaml-cpp/yaml.h>

#include <deque>
#include <optional>
#include <vector>

#include "buff_type.hpp"
#include "buff_track_bank.hpp"
#include "tools/img_tools.hpp"
#include "yolo11_buff.hpp"
const int LOSE_MAX = 5;  // 丢失的阙值
namespace auto_buff
{
class Buff_Detector
{
public:
  Buff_Detector(const std::string & config);

  std::vector<BuffObservation> detect_tracks(
    cv::Mat & bgr_img, BuffMode mode, std::chrono::steady_clock::time_point timestamp);

  std::vector<BuffObservation> detect_tracks(cv::Mat & bgr_img, BuffMode mode);

  std::optional<BuffObservation> detect_24(
    cv::Mat & bgr_img, BuffMode mode, std::chrono::steady_clock::time_point timestamp);

  std::optional<BuffObservation> detect_24(
    cv::Mat & bgr_img, std::chrono::steady_clock::time_point timestamp);

  std::optional<BuffObservation> detect_24(cv::Mat & bgr_img);

  std::optional<BuffObservation> detect(
    cv::Mat & bgr_img, BuffMode mode, std::chrono::steady_clock::time_point timestamp);

  std::optional<BuffObservation> detect(
    cv::Mat & bgr_img, std::chrono::steady_clock::time_point timestamp);

  std::optional<BuffObservation> detect(cv::Mat & bgr_img);

  std::optional<BuffObservation> detect_debug(cv::Mat & bgr_img, cv::Point2f v);

  int gate_failure_count() const { return gate_failure_count_; }

  int confirmed_switch_count() const { return track_bank_.confirmed_switch_count(); }
  int temporal_reject_count() const { return track_bank_.temporal_reject_count(); }

private:
  void handle_img(const cv::Mat & bgr_img, cv::Mat & dilated_img);

  cv::Point2f get_r_center(std::vector<FanBlade> & fanblades, cv::Mat & bgr_img);

  void handle_lose();

  struct CenterEstimate
  {
    cv::Point2f point;
    RuneCenterSource source;
  };

  std::optional<BuffObservation> detect_impl(
    cv::Mat & bgr_img, bool single_candidate, BuffMode mode,
    std::chrono::steady_clock::time_point timestamp);

  std::vector<BuffObservation> detect_tracks_impl(
    cv::Mat & bgr_img, bool single_candidate, BuffMode mode,
    std::chrono::steady_clock::time_point timestamp);

  void reset_for_mode(BuffMode mode);

  std::vector<BuffObservation> build_candidates(
    const std::vector<YOLO11_BUFF::Object> & results, const cv::Point2f & r_center) const;

  std::optional<CenterEstimate> select_r_center(
    const std::vector<YOLO11_BUFF::Object> & results,
    std::chrono::steady_clock::time_point timestamp);

  std::optional<BuffObservation> select_locked_candidate(
    const std::vector<BuffObservation> & candidates,
    std::chrono::steady_clock::time_point timestamp);

  std::optional<BuffObservation> stabilize_candidate(BuffObservation candidate);

  void remember_temporal_candidate(const BuffObservation & candidate);

  void reset_pending_switch();

  YOLO11_BUFF MODE_;
  Track_status status_;
  int lose_;  // 丢失的次数
  double lastlen_;

  float keypoint_threshold_ = 0.3f;
  float hard_keypoint_threshold_ = 0.15f;
  double temporal_residual_gate_px_ = 10.0;
  int center_lost_max_ = 6;
  double center_innovation_gate_px_ = 45.0;
  int center_recovery_hits_ = 2;
  double center_retention_s_ = 0.400;

  double pair_angle_gate_rad_ = 15.0 / 57.3;
  double pair_ratio_min_ = 0.30;
  double pair_ratio_max_ = 0.70;
  double pair_ratio_center_ = 0.51;
  double track_gate_min_rad_ = 12.0 / 57.3;
  double track_gate_max_rad_ = 25.0 / 57.3;
  double blind_timeout_s_ = 0.080;
  double track_reset_timeout_s_ = 0.500;
  int switch_confirm_frames_ = 5;
  int same_slot_confirm_frames_ = 3;
  int adjacent_switch_confirm_frames_ = 8;
  double adjacent_switch_delay_s_ = 0.180;
  double slot_tolerance_rad_ = 12.0 / 57.3;
  double switch_pair_angle_gate_rad_ = 10.0 / 57.3;
  double switch_pair_ratio_min_ = 0.38;
  double switch_pair_ratio_max_ = 0.64;

  bool has_last_r_center_ = false;
  cv::Point2f last_r_center_{0.0f, 0.0f};
  int center_lost_count_ = 0;
  std::chrono::steady_clock::time_point last_r_center_time_{};
  std::chrono::steady_clock::time_point last_r_center_seen_time_{};
  cv::Point2f r_center_velocity_{0.0f, 0.0f};
  bool has_pending_r_center_ = false;
  cv::Point2f pending_r_center_{0.0f, 0.0f};
  int pending_r_center_hits_ = 0;

  BuffTrackBank track_bank_;
  BuffMode current_mode_ = BuffMode::SMALL;
  bool has_current_mode_ = false;

  bool has_locked_target_ = false;
  double last_locked_angle_ = 0.0;
  double angular_velocity_ = 0.0;
  int next_track_id_ = 1;
  int locked_track_id_ = -1;
  int lost_locked_count_ = 0;
  bool has_pending_switch_ = false;
  double pending_switch_angle_ = 0.0;
  std::chrono::steady_clock::time_point pending_switch_time_{};
  int switch_confirm_count_ = 0;
  int pending_slot_offset_ = 0;
  std::chrono::steady_clock::time_point last_locked_time_{};
  std::chrono::steady_clock::time_point last_seen_time_{};
  bool gate_episode_active_ = false;
  int gate_failure_count_ = 0;
  int confirmed_switch_count_ = 0;
  int temporal_reject_count_ = 0;

  bool has_temporal_candidate_ = false;
  int temporal_track_id_ = -1;
  double temporal_angle_ = 0.0;
  cv::Point2f temporal_r_center_{0.0f, 0.0f};
  std::vector<cv::Point2f> temporal_target_points_;
  std::vector<cv::Point2f> temporal_fan_points_;
};
}  // namespace auto_buff
#endif  // DETECTOR_HPP
