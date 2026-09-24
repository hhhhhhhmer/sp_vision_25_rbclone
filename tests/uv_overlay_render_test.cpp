// UV 观测可视化渲染测试：
// 用 uv_filter_test 同款相机几何 + 一块合成的观测装甲板，直接调用 tools::draw_uv_overlay，
// 统计被绘制像素的数量与位置，确认：
//   1) UV 模式下 4 个观测角点（青点）、4 个预测角点（红十字）与残差连线真的画上去了；
//   2) 残差文本区有内容；
//   3) 非 UV 模式只画一行模式提示，不画角点；
//   4) tools::draw_zoom_inset 确实把局部区域放大贴到右上角（含 0.5 缩放路径）。
#include <Eigen/Dense>
#include <cmath>
#include <cstdio>
#include <opencv2/opencv.hpp>
#include <random>
#include <vector>

#include "tasks/auto_aim/tracking/target.hpp"
#include "tasks/auto_aim/tracking/uv_model.hpp"
#include "tools/uv_overlay.hpp"

using namespace auto_aim;

namespace
{
int failures = 0;

void check(bool ok, const char * name, double value, double tol)
{
  if (ok) {
    std::printf("  [ ok ] %s (value=%.6g)\n", name, value);
  } else {
    std::printf("  [FAIL] %s (value=%.6g, tol=%.6g)\n", name, value, tol);
    failures++;
  }
}

CameraGeometry make_geometry()
{
  CameraGeometry g;
  g.valid = true;
  g.camera_matrix << 1813.554114428731, 0.0, 712.274797427587,  //
    0.0, 1813.369040609312, 526.9183416131992,                   //
    0.0, 0.0, 1.0;
  g.distort_coeffs = {-0.06533754966400937, 0.11288365366404525, -0.0008807155121340363,
                      -0.0006436478698530189, 0.035117257854987};
  g.R_camera2gimbal << -0.0006300680323985472, -0.03084684563629979, 0.9995239242402179,  //
    -0.9998094041241314, -0.019484318694324685, -0.0012315640329051238,                   //
    0.019513032548112753, -0.999334195071553, -0.030828689908395462;
  g.t_camera2gimbal << 0.09879211822869594, -0.05661876507830866, 0.14718317632615113;
  g.R_gimbal2world = Eigen::Matrix3d::Identity();
  return g;
}

/** @brief 真值状态（与 uv_filter_test 同参数）：x=3.2 y=1.1 z=0.15 yaw=0.42 r=0.2 */
Eigen::VectorXd make_truth()
{
  Eigen::VectorXd x = Eigen::VectorXd::Zero(11);
  x << 3.2, 0.4, 1.1, -0.3, 0.15, 0.0, 0.42, 1.8, 0.20, 0.06, 0.02;
  return x;
}

/** @brief 统计两图差异像素数、平均灰度差，以及差异像素的包围盒 */
struct Diff
{
  int changed = 0;
  double mean = 0.0;
  cv::Rect bbox{};
};

Diff diff_of(const cv::Mat & a, const cv::Mat & b)
{
  cv::Mat gray_a, gray_b, d;
  cv::cvtColor(a, gray_a, cv::COLOR_BGR2GRAY);
  cv::cvtColor(b, gray_b, cv::COLOR_BGR2GRAY);
  cv::absdiff(gray_a, gray_b, d);
  Diff out;
  double sum = 0.0;
  int min_x = d.cols, min_y = d.rows, max_x = -1, max_y = -1;
  for (int y = 0; y < d.rows; y++) {
    const uchar * row = d.ptr<uchar>(y);
    for (int x = 0; x < d.cols; x++) {
      if (row[x] > 8) {
        out.changed++;
        sum += row[x];
        min_x = std::min(min_x, x);
        min_y = std::min(min_y, y);
        max_x = std::max(max_x, x);
        max_y = std::max(max_y, y);
      }
    }
  }
  out.mean = out.changed > 0 ? sum / out.changed : 0.0;
  if (max_x >= 0) out.bbox = cv::Rect(cv::Point(min_x, min_y), cv::Point(max_x, max_y));
  return out;
}

/** @brief 统计差异图中落在每个点邻域内的像素数 */
std::vector<int> count_near(
  const cv::Mat & a, const cv::Mat & b, const std::vector<cv::Point2f> & pts, int r)
{
  cv::Mat gray_a, gray_b, d;
  cv::cvtColor(a, gray_a, cv::COLOR_BGR2GRAY);
  cv::cvtColor(b, gray_b, cv::COLOR_BGR2GRAY);
  cv::absdiff(gray_a, gray_b, d);
  std::vector<int> out(pts.size(), 0);
  for (std::size_t i = 0; i < pts.size(); i++) {
    const int x0 = std::max(0, static_cast<int>(pts[i].x) - r);
    const int x1 = std::min(d.cols - 1, static_cast<int>(pts[i].x) + r);
    const int y0 = std::max(0, static_cast<int>(pts[i].y) - r);
    const int y1 = std::min(d.rows - 1, static_cast<int>(pts[i].y) + r);
    for (int y = y0; y <= y1; y++) {
      for (int x = x0; x <= x1; x++) {
        if (d.at<uchar>(y, x) > 8) out[i]++;
      }
    }
  }
  return out;
}
}  // namespace

int main()
{
  const CameraGeometry geometry = make_geometry();

  UvConfig config;
  config.enabled = true;
  config.sigma_px = 1.0;
  config.sigma_len_ratio = 0.02;
  config.sigma_attitude_deg = 0.5;
  config.sigma_angle = 0.1;
  config.associate_gate_px = 60.0;
  config.facing_cos_min = 0.15;
  config.radius_prior_sigma = 0.02;

  UvModel model;
  model.configure(config, 4, ArmorName::one);
  model.set_camera_geometry(geometry);

  const Eigen::VectorXd truth = make_truth();
  const int armor_id = 0;
  const auto corners = model.project_corners(truth, armor_id, ArmorType::small);
  std::printf("=== 合成观测 ===\n");
  std::printf(
    "预测角点: (%.1f,%.1f) (%.1f,%.1f) (%.1f,%.1f) (%.1f,%.1f)\n", corners[0].x, corners[0].y,
    corners[1].x, corners[1].y, corners[2].x, corners[2].y, corners[3].x, corners[3].y);
  check(corners.size() == 4, "project_corners 返回 4 点", static_cast<double>(corners.size()), 4);

  // 观测装甲板：单角点加 1.5px 偏移，制造可见的残差连线
  Armor armor;
  armor.type = ArmorType::small;
  armor.name = ArmorName::one;
  armor.points = corners;
  armor.points[2].x += 1.5F;

  // 目标：以"已解算"的观测初始化（世界系 x=3.2 y=1.1 z=0.15，r=0.2，4 块板）
  Eigen::VectorXd x0 = Eigen::VectorXd::Zero(11);
  x0 << 3.2, 0.4, 1.1, -0.3, 0.15, 0.0, 0.42, 1.8, 0.20, 0.06, 0.02;

  Armor seed = armor;
  seed.xyz_in_world = model.armor_position(x0, armor_id);
  seed.ypr_in_world = Eigen::Vector3d(model.armor_yaw(x0, armor_id), 0.0, 0.0);
  seed.ypd_in_world = Eigen::Vector3d(3.2, 1.1, 1.8);

  Eigen::VectorXd P0_dig = Eigen::VectorXd::Ones(11) * 0.1;
  Target target(seed, std::chrono::steady_clock::now(), x0[8], 4, P0_dig);
  target.set_uv_config(config);
  target.set_camera_geometry(geometry);
  check(target.uv_ready(), "Target 进入 UV 就绪状态", target.uv_ready() ? 1 : 0, 1);

  const auto pred_corners = model.project_corners(target.ekf_x(), target.last_id, armor.type);
  check(
    pred_corners.size() == 4, "目标状态可投影出 4 点", static_cast<double>(pred_corners.size()), 4);

  std::list<Armor> armors = {armor};
  const cv::Mat background(1080, 1440, CV_8UC3, cv::Scalar(40, 40, 40));

  std::printf("\n=== UV 模式绘制 ===\n");
  cv::Mat uv_img = background.clone();
  const cv::Rect focus = tools::draw_uv_overlay(uv_img, target, armors, true, 1, 0.4);
  const Diff uv_diff = diff_of(background, uv_img);
  std::printf(
    "差异像素 %d，平均灰度差 %.1f，包围盒 (%d,%d %dx%d)\n", uv_diff.changed, uv_diff.mean,
    uv_diff.bbox.x, uv_diff.bbox.y, uv_diff.bbox.width, uv_diff.bbox.height);
  check(uv_diff.changed > 200, "UV 模式绘制了足够多的像素", uv_diff.changed, 200);
  check(!focus.empty(), "返回了非空放大区域", focus.area() > 0 ? 1 : 0, 1);

  const auto per_corner = count_near(background, uv_img, armor.points, 6);
  int corners_drawn = 0;
  for (int i = 0; i < 4; i++) {
    std::printf("  观测角点 %d 邻域差异像素: %d\n", i, per_corner[i]);
    if (per_corner[i] > 0) corners_drawn++;
  }
  check(corners_drawn == 4, "4 个观测角点都被画上（青点）", corners_drawn, 4);

  const auto per_pred = count_near(background, uv_img, pred_corners, 10);
  int pred_drawn = 0;
  for (int i = 0; i < 4; i++) {
    std::printf("  预测角点 %d 邻域差异像素: %d\n", i, per_pred[i]);
    if (per_pred[i] > 0) pred_drawn++;
  }
  check(pred_drawn == 4, "4 个预测角点都被画上（红十字）", pred_drawn, 4);

  const cv::Rect text_roi(0, 140, 700, 100);
  const Diff text_diff = diff_of(background(text_roi).clone(), uv_img(text_roi).clone());
  std::printf("残差文本区差异像素: %d\n", text_diff.changed);
  check(text_diff.changed > 100, "残差文本已绘制", text_diff.changed, 100);

  const cv::Rect status_roi(0, 0, 700, 45);
  const Diff status_diff = diff_of(background(status_roi).clone(), uv_img(status_roi).clone());
  std::printf("状态行差异像素: %d\n", status_diff.changed);
  check(status_diff.changed > 100, "状态行已绘制", status_diff.changed, 100);

  std::printf("\n=== 非 UV 模式 ===\n");
  cv::Mat ypd_img = background.clone();
  tools::draw_uv_overlay(ypd_img, target, armors, false, 0, 0.0);
  const auto ypd_per_corner = count_near(background, ypd_img, armor.points, 6);
  int ypd_corner_pixels = 0;
  for (int i = 0; i < 4; i++) ypd_corner_pixels += ypd_per_corner[i];
  std::printf("角点邻域差异像素合计: %d\n", ypd_corner_pixels);
  check(ypd_corner_pixels == 0, "非 UV 模式不画角点", ypd_corner_pixels, 0);

  std::printf("\n=== 0.5 缩放路径（实时程序用）===\n");
  cv::Mat small_bg(540, 720, CV_8UC3, cv::Scalar(40, 40, 40));
  cv::Mat small = small_bg.clone();
  tools::UvOverlayConfig small_cfg;
  small_cfg.scale = 0.5;
  small_cfg.font_scale = 0.5;
  small_cfg.status_pos = {10, 15};
  small_cfg.text_base_y = 75;
  small_cfg.text_line_step = 30;
  std::list<Armor> small_armors = {armor};
  for (auto & a : small_armors) {
    for (auto & p : a.points) p *= 0.5F;
  }
  const cv::Rect small_focus =
    tools::draw_uv_overlay(small, target, small_armors, true, 1, 0.4, small_cfg);
  const Diff small_diff = diff_of(small_bg, small);
  std::printf(
    "半尺寸画面差异像素: %d，包围盒 (%d,%d %dx%d)\n", small_diff.changed, small_diff.bbox.x,
    small_diff.bbox.y, small_diff.bbox.width, small_diff.bbox.height);
  check(small_diff.changed > 50, "半尺寸画面同样绘制", small_diff.changed, 50);
  check(!small_focus.empty(), "半尺寸返回非空放大区域", small_focus.area() > 0 ? 1 : 0, 1);

  std::printf("\n=== 放大贴图 ===\n");
  cv::Mat zoom = small.clone();
  tools::draw_zoom_inset(zoom, small_focus, 520, 0.5, 12);
  const cv::Rect inset_roi(720 - 260 - 6, 6, 260, 260);
  const Diff inset_diff = diff_of(small, zoom);
  std::printf(
    "放大贴图后新增差异像素: %d，diff bbox (%d,%d %dx%d)\n", inset_diff.changed,
    inset_diff.bbox.x, inset_diff.bbox.y, inset_diff.bbox.width, inset_diff.bbox.height);
  check(inset_diff.changed > 100, "右上角出现放大贴图", inset_diff.changed, 100);
  check(
    inset_diff.bbox.x >= inset_roi.x - 40 && inset_diff.bbox.y <= inset_roi.y + inset_roi.height,
    "贴图落在右上角", static_cast<double>(inset_diff.bbox.x), inset_roi.x - 40);

  std::printf(
    "\n%s (%d failures)\n",
    failures == 0 ? "UV OVERLAY RENDER TESTS PASSED" : "UV OVERLAY RENDER TESTS FAILED", failures);
  return failures == 0 ? 0 : 1;
}
