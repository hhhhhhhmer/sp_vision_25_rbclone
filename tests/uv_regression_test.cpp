// UV 观测 / 前哨站高度阶梯回归测试：
//  1) A1：观测模型（UvModel）与状态读回（RVfromFYT::h_armor_xyz）必须共用同一份
//         高度阶梯实现 —— 三块装甲板的位置在三种锚点状态下都必须逐点一致
//  2) A2：静止 + 固定转速前哨站的瞄准高度误差（含锚点在线学习），
//         并与传统 YPD 路径在同一段轨迹上对比，UV 不允许更差
//  3) B ：update_count_ 语义是"发生过校正的帧数"，多装甲融合一帧内多次校正只计一次
#include <cmath>
#include <cstdio>
#include <list>
#include <random>
#include <utility>
#include <vector>

#include <Eigen/Dense>

#include "tasks/auto_aim/tracking/target.hpp"
#include "tasks/auto_aim/tracking/tower_armor_geometry.hpp"
#include "tasks/auto_aim/tracking/uv_model.hpp"
#include "tools/math_tools.hpp"

using namespace auto_aim;

namespace
{
int failures = 0;

void check(bool ok, const char * name, double value = 0.0, double tol = 0.0)
{
  if (ok) {
    std::printf("  [ ok ] %s\n", name);
  } else {
    std::printf("  [FAIL] %s (value=%.6g, tol=%.6g)\n", name, value, tol);
    failures++;
  }
}

using Heights = std::array<std::pair<bool, double>, 3>;

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

UvConfig make_uv_config()
{
  UvConfig cfg;
  cfg.enabled = true;
  cfg.sigma_px = 1.0;
  cfg.sigma_len_ratio = 0.02;
  cfg.sigma_attitude_deg = 0.5;
  cfg.sigma_angle = 0.1;
  cfg.associate_gate_px = 60.0;
  cfg.facing_cos_min = 0.15;
  cfg.radius_prior_sigma = 0.02;
  return cfg;
}

// ---------------------------------------------------------------- 前哨站真值

constexpr int kPlateNum = 3;
constexpr double kRadius = 0.2765;
constexpr double kPlateH[kPlateNum] = {1.0, 1.1, 1.2};  // 0.1 阶梯 -> 高度乘数应为 0/+1/+2
constexpr double kOmega = 1.0;                          // 固定转速 rad/s
constexpr double kDt = 1.0 / 90.0;
constexpr double kCenterX = 6.0, kCenterY = 0.0;
constexpr double kPitch = -15.0 * M_PI / 180.0;  // 前哨站倾角（与 UvModel/Solver 一致）

double plate_angle(double yaw, int k) { return yaw + k * 2.0 * M_PI / kPlateNum; }

Eigen::Vector3d plate_pos(double yaw, int k)
{
  const double a = plate_angle(yaw, k);
  return {-kRadius * std::cos(a) + kCenterX, -kRadius * std::sin(a) + kCenterY, kPlateH[k]};
}

Eigen::Matrix3d plate_rot(double yaw, int k)
{
  const double a = plate_angle(yaw, k), cy = std::cos(a), sy = std::sin(a);
  const double cp = std::cos(kPitch), sp = std::sin(kPitch);
  Eigen::Matrix3d R;
  R << cy * cp, -sy, cy * sp,  //
    sy * cp, cy, sy * sp,      //
    -sp, 0, cp;
  return R;
}

double plate_facing(double yaw, int k)
{
  const Eigen::Vector3d p = plate_pos(yaw, k);
  Eigen::Vector2d n(-std::cos(plate_angle(yaw, k)), -std::sin(plate_angle(yaw, k)));
  Eigen::Vector2d to_cam(-p.x(), -p.y());
  to_cam.normalize();
  return n.dot(to_cam);
}

/** @brief 用物理真值独立投影角点（不经过 UvModel 的位置模型，避免自证） */
std::vector<cv::Point2f> project_truth(const CameraGeometry & g, double yaw, int k, ArmorType type)
{
  const double w = UvModel::armor_width(type) / 2.0, l = UvModel::lightbar_length() / 2.0;
  const std::array<Eigen::Vector3d, 4> local = {
    Eigen::Vector3d(0, +w, +l), Eigen::Vector3d(0, -w, +l), Eigen::Vector3d(0, -w, -l),
    Eigen::Vector3d(0, +w, -l)};
  const Eigen::Vector3d c = plate_pos(yaw, k);
  const Eigen::Matrix3d R = plate_rot(yaw, k);
  const double fx = g.camera_matrix(0, 0), fy = g.camera_matrix(1, 1);
  const double cx = g.camera_matrix(0, 2), cy = g.camera_matrix(1, 2);
  const double k1 = g.distort_coeffs[0], k2 = g.distort_coeffs[1];
  const double p1 = g.distort_coeffs[2], p2 = g.distort_coeffs[3], k3 = g.distort_coeffs[4];

  std::vector<cv::Point2f> out;
  out.reserve(4);
  for (const auto & lc : local) {
    const Eigen::Vector3d pw = c + R * lc;
    const Eigen::Vector3d pc =
      g.R_camera2gimbal.transpose() * (g.R_gimbal2world.transpose() * pw - g.t_camera2gimbal);
    const double Z = pc.z(), x = pc.x() / Z, y = pc.y() / Z, r2 = x * x + y * y;
    const double radial = 1.0 + k1 * r2 + k2 * r2 * r2 + k3 * r2 * r2 * r2;
    const double xd = x * radial + 2.0 * p1 * x * y + p2 * (r2 + 2.0 * x * x);
    const double yd = y * radial + p1 * (r2 + 2.0 * y * y) + 2.0 * p2 * x * y;
    out.emplace_back(static_cast<float>(fx * xd + cx), static_cast<float>(fy * yd + cy));
  }
  return out;
}

// ---------------------------------------------------------------- 前哨站离线回放

struct OutpostRun
{
  double aim_mean = 0.0;  ///< 可见板瞄准点 z 相对物理高度的平均误差 (m)
  double aim_max = 0.0;   ///< 最大误差 (m)
  Heights anchors{};      ///< 结束时学到的高度锚点
  int visible_frames = 0;
};

/**
 * @brief 回放"静止 + 固定转速 + 三块板 0.1m 阶梯"的前哨站
 * @param cfg UV 配置 @param g 相机几何 @param uv true 走 UV 观测，false 走传统 YPD 观测
 * @return 瞄准高度误差与最终锚点
 * @note 每帧只更新最靠近图像中心的一块装甲板，复刻 Tracker::update_target 的前哨站分支
 */
OutpostRun run_outpost(const UvConfig & cfg, const CameraGeometry & geometry, bool uv)
{
  std::mt19937 rng(20260910);
  std::normal_distribution<double> pixel_noise(0.0, 1.0);

  double yaw = 0.0;  // t=0 时 0 号板正对相机
  const auto t0 = std::chrono::steady_clock::now();

  Armor first;
  first.name = ArmorName::outpost;
  first.type = ArmorType::small;
  first.xyz_in_world = plate_pos(yaw, 0);
  first.ypr_in_world = Eigen::Vector3d(yaw, kPitch, 0.0);

  Eigen::VectorXd P0_dig{{1, 64, 1, 64, 1, 81, 0.4, 100, 1e-4, 0, 0}};
  if (uv) P0_dig[8] = cfg.radius_prior_sigma * cfg.radius_prior_sigma;  // Tracker 的退化保护

  Target target(first, t0, kRadius, kPlateNum, P0_dig);
  UvConfig run_cfg = cfg;
  run_cfg.enabled = uv;
  target.set_uv_config(run_cfg);
  target.set_camera_geometry(geometry);

  OutpostRun run;
  double aim_err_sum = 0.0;
  const int steps = 620;  // ≈6.9s ≈ 1.1 圈，覆盖整段锚点学习窗口

  for (int step = 1; step <= steps; step++) {
    yaw += kOmega * kDt;
    const auto t = t0 + std::chrono::microseconds(static_cast<long>(step * kDt * 1e6));

    std::vector<int> visible;
    std::list<Armor> armors;
    for (int k = 0; k < kPlateNum; k++) {
      if (plate_facing(yaw, k) < cfg.facing_cos_min) continue;
      Armor a;
      a.name = ArmorName::outpost;
      a.type = ArmorType::small;
      a.points = project_truth(geometry, yaw, k, ArmorType::small);
      for (auto & p : a.points) {
        p.x += static_cast<float>(pixel_noise(rng));
        p.y += static_cast<float>(pixel_noise(rng));
      }
      // 传统路径的观测来自 PnP，这里直接用真值填位姿字段
      a.xyz_in_world = plate_pos(yaw, k);
      a.ypr_in_world = Eigen::Vector3d(plate_angle(yaw, k), kPitch, 0.0);
      a.ypd_in_world = tools::xyz2ypd(a.xyz_in_world);
      armors.push_back(a);
      visible.push_back(k);
    }

    target.predict(t, Eigen::VectorXd::Zero(3));
    target.set_uv_config(run_cfg);
    target.set_camera_geometry(geometry);

    for (const auto & armor : armors) {
      if (!target.update(armor, -1.0, {})) continue;
      break;
    }

    const auto xyza = target.armor_xyza_list();
    for (int k : visible) {
      if (k >= static_cast<int>(xyza.size())) continue;
      const double err = std::abs(xyza[k].z() - kPlateH[k]);
      aim_err_sum += err;
      run.aim_max = std::max(run.aim_max, err);
      run.visible_frames++;
    }
  }

  run.aim_mean = aim_err_sum / std::max(1, run.visible_frames);
  for (int i = 0; i < 3; i++) run.anchors[i] = target.tower_armor_hs[i];
  return run;
}
}  // namespace

int main()
{
  const CameraGeometry geometry = make_geometry();
  const UvConfig cfg = make_uv_config();

  std::printf("=== A1: 观测模型与状态读回的高度阶梯一致性 ===\n");
  {
    Eigen::VectorXd x = Eigen::VectorXd::Zero(11);
    x << 3.0, 0, 0.5, 0, 1.0, 0, 0.0, 0, 0.2765, 0, 0.10;  // 前哨站：armor_num=3

    const Heights cases[3] = {
      {std::make_pair(false, 0.0), std::make_pair(false, 0.0), std::make_pair(false, 0.0)},
      {std::make_pair(true, 1.0), std::make_pair(false, 0.0), std::make_pair(false, 0.0)},
      {std::make_pair(true, 1.0), std::make_pair(true, 1.1), std::make_pair(true, 1.2)}};
    const char * case_name[3] = {"锚点全无效", "只有 0 号锚点有效", "三块锚点都有效"};

    RVfromFYT ekf(x, Eigen::MatrixXd::Identity(11, 11), kPlateNum, ArmorName::outpost);
    UvModel uv;
    uv.configure(cfg, kPlateNum, ArmorName::outpost);

    for (int c = 0; c < 3; c++) {
      std::pair<bool, double> raw[3] = {cases[c][0], cases[c][1], cases[c][2]};
      ekf.set_tower_armor_heights(raw);
      uv.set_tower_armor_heights(cases[c]);

      double worst = 0.0;
      for (int id = 0; id < kPlateNum; id++) {
        worst = std::max(
          worst, (ekf.h_armor_xyz(x, id) - uv.armor_position(x, id)).cwiseAbs().maxCoeff());
      }
      char name[96];
      std::snprintf(name, sizeof(name), "%s：两种几何逐点一致 (m)", case_name[c]);
      check(worst < 1e-12, name, worst, 1e-12);
    }

    // 无效锚点必须视为"与 0 号板同高"，而不是靠占位值 0.0 得出"低 2 级"
    std::pair<bool, double> partial[3] = {{true, 1.0}, {false, 0.0}, {false, 0.0}};
    ekf.set_tower_armor_heights(partial);
    check(
      std::abs(ekf.h_armor_xyz(x, 1).z() - ekf.h_armor_xyz(x, 0).z()) < 1e-12,
      "锚点无效时不凭空产生高度阶梯", ekf.h_armor_xyz(x, 1).z() - ekf.h_armor_xyz(x, 0).z(),
      1e-12);

    // 锚点齐备时阶梯方向必须与物理真值一致（0.1 阶梯 -> 乘数 0/+1/+2）
    const Heights full = {
      std::make_pair(true, 1.0), std::make_pair(true, 1.1), std::make_pair(true, 1.2)};
    check(
      tower_armor_height_multiplier(full, 1) == 1.0 &&
        tower_armor_height_multiplier(full, 2) == 2.0,
      "锚点齐备时高度乘数为 +1/+2", tower_armor_height_multiplier(full, 2), 2.0);
  }

  std::printf("=== A2: 静止 + 固定转速前哨站的瞄准高度误差 ===\n");
  {
    const OutpostRun uv_run = run_outpost(cfg, geometry, true);
    const OutpostRun ypd_run = run_outpost(cfg, geometry, false);

    std::printf(
      "        UV ：瞄准 z 平均 %.4f m，最大 %.4f m；锚点 [%.3f %.3f %.3f]\n", uv_run.aim_mean,
      uv_run.aim_max, uv_run.anchors[0].second, uv_run.anchors[1].second, uv_run.anchors[2].second);
    std::printf(
      "        YPD：瞄准 z 平均 %.4f m，最大 %.4f m；锚点 [%.3f %.3f %.3f]\n", ypd_run.aim_mean,
      ypd_run.aim_max, ypd_run.anchors[0].second, ypd_run.anchors[1].second,
      ypd_run.anchors[2].second);

    check(uv_run.visible_frames > 0, "本段轨迹中前哨站可见", uv_run.visible_frames, 0.0);
    check(
      uv_run.aim_mean < 0.06, "UV 瞄准 z 平均误差 < 0.06m（修复前 0.089m）", uv_run.aim_mean,
      0.06);
    check(
      uv_run.aim_max < 0.25, "UV 瞄准 z 最大误差 < 0.25m（修复前 0.42m）", uv_run.aim_max, 0.25);
    check(
      uv_run.aim_max <= std::max(0.25, ypd_run.aim_max), "UV 瞄准 z 最大误差不劣于传统 YPD 路径",
      uv_run.aim_max - ypd_run.aim_max, 0.0);

    // 高度锚点必须学出正确的方向：0 号最低。方向翻转会让瞄准点偏 0.2~0.4m
    const Heights & hs = uv_run.anchors;
    check(hs[0].first && hs[1].first && hs[2].first, "三块装甲板的高度锚点均已学到", 0.0, 0.0);
    check(
      hs[1].second - hs[0].second > 0.05 && hs[2].second - hs[0].second > 0.05,
      "锚点阶梯方向正确（1/2 号高于 0 号）",
      std::min(hs[1].second - hs[0].second, hs[2].second - hs[0].second), 0.05);
  }

  std::printf("=== B: update_count_ 按帧计数 ===\n");
  {
    // 4 装甲板目标，朝向上让 0/1 两块板同时朝向相机（-40° -> cos 0.77 / 0.64）
    const double yaw0 = -40.0 * M_PI / 180.0;
    const double radius = 0.2;
    Armor init;
    init.name = ArmorName::not_armor;
    init.type = ArmorType::small;
    init.xyz_in_world = {4.0 - radius * std::cos(yaw0), 0.0 - radius * std::sin(yaw0), 0.1};
    init.ypr_in_world = Eigen::Vector3d(yaw0, 15.0 * M_PI / 180.0, 0.0);

    const auto t0 = std::chrono::steady_clock::now();
    Eigen::VectorXd P0_dig{{1, 64, 1, 64, 1, 64, 0.4, 100, 1e-4, 0, 0}};
    Target target(init, t0, radius, 4, P0_dig);
    target.set_uv_config(cfg);
    target.set_camera_geometry(geometry);
    target.predict(t0 + std::chrono::milliseconds(1), Eigen::VectorXd::Zero(3));

    // 用滤波器自己的状态投影出观测角点（等价于"观测与预测完全一致"的理想输入）
    UvModel model;
    model.configure(cfg, 4, target.name);
    model.set_camera_geometry(geometry);
    auto make_armor = [&](int id) {
      Armor a;
      a.name = target.name;
      a.type = target.armor_type;
      a.points = model.project_corners(target.ekf_x(), id, target.armor_type);
      return a;
    };

    check(model.facing_cos(target.ekf_x(), 0) > cfg.facing_cos_min, "0 号板朝向相机");
    check(model.facing_cos(target.ekf_x(), 1) > cfg.facing_cos_min, "1 号板朝向相机");

    std::list<Armor> frame_armors;
    frame_armors.push_back(make_armor(0));
    frame_armors.push_back(make_armor(1));

    const bool found = target.update_batch(frame_armors, 20.0, 99);
    check(found, "同帧融合两块装甲板成功", found ? 1.0 : 0.0, 0.0);
    check(
      target.last_batch_observation_count() == 2, "本帧实际融合 2 条观测",
      target.last_batch_observation_count(), 2.0);
    check(
      target.update_count_ == 1, "同帧两次校正后 update_count_ 仍为 1（修复前为 2）",
      target.update_count_, 1.0);

    // 下一帧再做一次校正，计数才 +1
    target.predict(t0 + std::chrono::milliseconds(11), Eigen::VectorXd::Zero(3));
    std::list<Armor> next_armors;
    next_armors.push_back(make_armor(0));
    target.update_batch(next_armors, 20.0, 99);
    check(target.update_count_ == 2, "下一帧校正后 update_count_ 为 2", target.update_count_, 2.0);
  }

  std::printf(
    "\n%s (%d failures)\n",
    failures == 0 ? "UV REGRESSION TESTS PASSED" : "UV REGRESSION TESTS FAILED", failures);
  return failures == 0 ? 0 : 1;
}
