#include "uv_overlay.hpp"

#include <fmt/core.h>

#include <algorithm>
#include <cmath>
#include <vector>

#include "img_tools.hpp"
#include "math_tools.hpp"

namespace tools
{
namespace
{
constexpr double kRadToDeg = 57.29577951308232;
}  // namespace

cv::Rect draw_uv_overlay(
  cv::Mat & img, const auto_aim::Target & target, const std::list<auto_aim::Armor> & armors,
  bool uv_mode, int fused, double nis, const UvOverlayConfig & config)
{
  if (!uv_mode || !target.ekf().uv_model().ready()) {
    tools::draw_text(
      img, fmt::format("obs: YPD(world)  fused:{}  nis:{:.1f}", fused, nis), config.status_pos,
      {0, 128, 255}, config.font_scale);
    return {};
  }

  const auto & model = target.ekf().uv_model();
  const Eigen::VectorXd x = target.ekf_x();
  const int armor_num = static_cast<int>(target.armor_xyza_list().size());

  // 1) 整车预测轮廓：灰色=非当前装甲板，绿色=当前 last_id
  if (config.draw_all_armors) {
    for (int id = 0; id < armor_num; id++) {
      const auto corners = model.project_corners(x, id, target.armor_type);
      if (corners.size() != 4) continue;
      const bool current = (id == target.last_id);
      const cv::Scalar color = current ? cv::Scalar(0, 255, 0) : cv::Scalar(140, 140, 140);
      for (int i = 0; i < 4; i++) {
        cv::line(
          img, corners[i], corners[(i + 1) % 4], color, current ? 2 : 1);
      }
      cv::putText(
        img, fmt::format("id{}", id), corners[0], cv::FONT_HERSHEY_SIMPLEX, 0.6 * config.scale,
        color, 2);
    }
  }

  // 2) 观测端点（青点）vs 预测端点（红十字）+ 橙色残差连线
  int shown = 0;
  cv::Rect focus{};
  for (const auto & armor : armors) {
    if (armor.name != target.name || armor.type != target.armor_type) continue;
    if (armor.points.size() != 4) continue;

    int best_id = -1;
    double best_err = 1e18;
    std::vector<cv::Point2f> best_corners;
    for (int id = 0; id < armor_num; id++) {
      const auto c = model.project_corners(x, id, armor.type);
      if (c.size() != 4) continue;
      double err = 0;
      for (int i = 0; i < 4; i++) err += cv::norm(c[i] - armor.points[i]);
      if (err < best_err) {
        best_err = err;
        best_id = id;
        best_corners = c;
      }
    }
    if (best_id < 0) continue;

    if (shown == 0) {
      float min_x = armor.points[0].x, max_x = armor.points[0].x;
      float min_y = armor.points[0].y, max_y = armor.points[0].y;
      for (const auto & p : armor.points) {
        min_x = std::min(min_x, p.x);
        max_x = std::max(max_x, p.x);
        min_y = std::min(min_y, p.y);
        max_y = std::max(max_y, p.y);
      }
      focus = cv::Rect(
        cv::Point(static_cast<int>(min_x), static_cast<int>(min_y)),
        cv::Point(static_cast<int>(max_x), static_cast<int>(max_y)));
    }

    const int radius = std::max(1, static_cast<int>(std::lround(4 * config.scale)));
    const int marker_size = std::max(3, static_cast<int>(std::lround(14 * config.scale)));
    for (int i = 0; i < 4; i++) {
      cv::line(img, armor.points[i], best_corners[i], cv::Scalar(0, 165, 255), 1);
      cv::circle(img, armor.points[i], radius, cv::Scalar(255, 255, 0), -1);
      cv::drawMarker(
        img, best_corners[i], cv::Scalar(0, 0, 255), cv::MARKER_TILTED_CROSS, marker_size, 2);
    }

    // 3) UVL 残差（每条灯条 4 维）
    if (config.draw_residual_text) {
      const auto z_obs = auto_aim::UvModel::observation(armor.points);
      const auto z_pred = model.predict(x, best_id, armor.type);
      const double ang_l = tools::limit_rad(z_obs[0] - z_pred[0]) * kRadToDeg;
      const double ang_r = tools::limit_rad(z_obs[4] - z_pred[4]) * kRadToDeg;

      const int base_y = config.text_base_y + shown * config.text_line_step;
      const int line_step = std::max(8, static_cast<int>(std::lround(26 * config.font_scale)));
      tools::draw_text(
        img, fmt::format("match id{}  corner_err {:.1f}px", best_id, best_err / 4.0), {10, base_y},
        {0, 255, 255}, config.font_scale);
      tools::draw_text(
        img,
        fmt::format(
          "L: ang{:+.2f}deg  c({:+.1f},{:+.1f})  len{:+.2f}px", ang_l, z_obs[1] - z_pred[1],
          z_obs[2] - z_pred[2], z_obs[3] - z_pred[3]),
        {10, base_y + line_step}, {255, 255, 0}, config.font_scale);
      tools::draw_text(
        img,
        fmt::format(
          "R: ang{:+.2f}deg  c({:+.1f},{:+.1f})  len{:+.2f}px", ang_r, z_obs[5] - z_pred[5],
          z_obs[6] - z_pred[6], z_obs[7] - z_pred[7]),
        {10, base_y + 2 * line_step}, {255, 255, 0}, config.font_scale);
    }

    shown++;
    if (shown >= config.max_shown) break;
  }

  tools::draw_text(
    img, fmt::format("obs: UV(pixel)  fused:{}  nis:{:.1f}", fused, nis), config.status_pos,
    {0, 255, 0}, config.font_scale);
  return focus;
}

void draw_zoom_inset(cv::Mat & img, const cv::Rect & focus, int size, double scale, int margin_px)
{
  if (focus.width <= 0 || focus.height <= 0) return;

  const double s = (scale > 0) ? scale : 1.0;
  const int size_px = std::max(16, static_cast<int>(std::lround(size * s)));
  const int margin = std::max(0, static_cast<int>(std::lround(margin_px * s)));

  const int margin_roi = std::max(focus.width, focus.height);
  cv::Rect roi(
    focus.x - margin_roi / 2, focus.y - margin_roi / 2, focus.width + margin_roi,
    focus.height + margin_roi);
  roi &= cv::Rect(0, 0, img.cols, img.rows);
  if (roi.width < 8 || roi.height < 8) return;

  cv::Mat crop = img(roi).clone();
  cv::resize(crop, crop, {size_px, size_px}, 0, 0, cv::INTER_NEAREST);

  if (img.cols < size_px + 2 * margin || img.rows < size_px + 2 * margin) return;
  const cv::Rect dst(img.cols - size_px - margin, margin, size_px, size_px);
  crop.copyTo(img(dst));
  cv::rectangle(img, dst, {255, 255, 255}, std::max(1, static_cast<int>(std::lround(3 * s))));
  cv::putText(
    img, "zoom x" + std::to_string(size_px / std::max(roi.width, roi.height)),
    {dst.x + static_cast<int>(std::lround(8 * s)), dst.y + static_cast<int>(std::lround(30 * s))},
    cv::FONT_HERSHEY_SIMPLEX, 0.9 * s, {255, 255, 255}, std::max(1, static_cast<int>(std::lround(2 * s))));
}

}  // namespace tools
