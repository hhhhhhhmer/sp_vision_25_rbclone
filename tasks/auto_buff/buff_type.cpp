#include "buff_type.hpp"

#include <algorithm>

#include "tools/logger.hpp"

namespace auto_buff
{

FanBlade::FanBlade(const std::vector<cv::Point2f> & kpt, cv::Point2f keypoints_center, FanBlade_type t)
: center(keypoints_center), fan_center(keypoints_center), type(t)
{
  points.insert(points.end(), kpt.begin(), kpt.end());
}

FanBlade::FanBlade(
  const std::vector<cv::Point2f> & target_kpt, const std::vector<cv::Point2f> & fan_kpt,
  cv::Point2f target_center, cv::Point2f fan_center_, FanBlade_type t, float confidence_,
  int slot_id_)
: center(target_center),
  fan_center(fan_center_),
  points(target_kpt),
  fan_points(fan_kpt),
  slot_id(slot_id_),
  confidence(confidence_),
  type(t)
{
}

FanBlade::FanBlade(FanBlade_type t) : type(t)
{
  if (t != _unlight) exit(-1);
}

PowerRune::PowerRune(
  std::vector<FanBlade> & ts, const cv::Point2f center, std::optional<PowerRune> last_powerrune)
: r_center(center), light_num(ts.size())
{
  if (ts.empty()) {
    unsolvable_ = true;
    return;
  }

  auto target_it = ts.begin();
  if (ts.size() > 1 && last_powerrune.has_value() && !last_powerrune->is_unsolve()) {
    const double last_angle = last_powerrune->target_angle;
    target_it = std::min_element(ts.begin(), ts.end(), [&](const FanBlade & a, const FanBlade & b) {
      return std::abs(tools::limit_rad(atan_angle(a.center) - last_angle)) <
             std::abs(tools::limit_rad(atan_angle(b.center) - last_angle));
    });
  }

  target_it->type = _target;
  std::iter_swap(ts.begin(), target_it);

  target_slot_id = ts[0].slot_id;
  target_angle = atan_angle(ts[0].center);

  const double base_angle = target_angle;
  for (auto & t : ts) {
    t.angle = atan_angle(t.center) - base_angle;
    if (t.angle < -1e-3) t.angle += CV_2PI;
  }
  std::sort(ts.begin(), ts.end(), [](const FanBlade & a, const FanBlade & b) {
    return a.angle < b.angle;
  });

  const std::vector<double> standard_angles = {
    0, RUNE_SLOT_ANGLE, 2.0 * RUNE_SLOT_ANGLE, 3.0 * RUNE_SLOT_ANGLE,
    4.0 * RUNE_SLOT_ANGLE};
  for (int i = 0, j = 0; i < 5 && j < static_cast<int>(ts.size()); ++i) {
    if (std::fabs(ts[j].angle - standard_angles[i]) < CV_PI / 5.0)
      fanblades.emplace_back(ts[j++]);
    else
      fanblades.emplace_back(FanBlade(_unlight));
  }
}

PowerRune::PowerRune(const BuffObservation & observation)
: r_center(observation.r_center),
  light_num(1),
  track_id(observation.track_id),
  slot_offset(observation.slot_offset),
  primary(observation.primary),
  track_status(observation.track_status),
  target_angle(observation.angle),
  slot_residual(observation.slot_residual),
  center_source(observation.center_source),
  observation_type(observation.type),
  prediction_error(observation.prediction_error)
{
  FanBlade blade(
    observation.target_points, observation.fan_points, observation.target_center,
    observation.fan_center, _target, observation.confidence, -1);
  blade.angle = observation.angle;
  fanblades.emplace_back(std::move(blade));
}

double PowerRune::atan_angle(cv::Point2f point) const
{
  auto v = point - r_center;
  auto angle = std::atan2(v.y, v.x);
  return angle >= 0 ? angle : angle + CV_2PI;
}

}  // namespace auto_buff
