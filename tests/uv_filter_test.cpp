// UV 观测端到端测试：合成一段旋转目标轨迹，用带噪声的灯条像素观测驱动 RVfromFYT，
// 检查状态能否从较大的初始误差收敛（覆盖 UV 模型 + 解析雅可比 + 关联 + 批量更新）。
#include <Eigen/Dense>
#include <cmath>
#include <cstdio>
#include <random>
#include <vector>

#include "tasks/auto_aim/tracking/rv_from_fyt.hpp"
#include "tasks/auto_aim/tracking/uv_model.hpp"

using namespace auto_aim;

namespace
{
int failures = 0;
void check(bool ok, const char * name, double value, double tol)
{
  if (ok) {
    std::printf("  [ ok ] %s\n", name);
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

/** @brief 造一块观测装甲板：4 个角点由真值状态投影得到，再加高斯像素噪声 */
Armor make_armor(
  const UvModel & model, const Eigen::VectorXd & truth, int armor_id, ArmorType type,
  std::normal_distribution<double> & noise, std::mt19937 & rng)
{
  Armor armor;
  armor.type = type;
  armor.name = ArmorName::one;
  armor.points = model.project_corners(truth, armor_id, type);
  for (auto & p : armor.points) {
    p.x += static_cast<float>(noise(rng));
    p.y += static_cast<float>(noise(rng));
  }
  return armor;
}
}  // namespace

int main()
{
  UvConfig config;
  config.enabled = true;
  config.sigma_px = 1.0;
  config.sigma_len_ratio = 0.02;
  config.sigma_attitude_deg = 0.5;
  config.sigma_angle = 0.1;
  config.associate_gate_px = 60.0;
  config.facing_cos_min = 0.15;
  config.radius_prior_sigma = 0.02;

  const CameraGeometry geometry = make_geometry();
  UvModel model;
  model.configure(config, 4, ArmorName::one);
  model.set_camera_geometry(geometry);

  const std::array<int, 3> scenarios = {1, 2, 4};  // 每帧可见装甲板数
  std::mt19937 rng(20260910);
  std::normal_distribution<double> pixel_noise(0.0, 1.0);

  std::printf("=== UV 端到端收敛测试（4 装甲板旋转目标，100 步 @10ms）===\n");
  for (int visible : scenarios) {
    // 真值：[x, vx, y, vy, z, vz, yaw, vyaw, r, r_off, h]
    Eigen::VectorXd truth = Eigen::VectorXd::Zero(11);
    truth << 4.5, -0.8, 1.2, 0.5, 0.1, 0.0, 0.0, 1.5, 0.20, 0.05, 0.0;

    // 初始状态带较大误差
    Eigen::VectorXd x0 = truth;
    x0[0] += 0.6;
    x0[2] -= 0.5;
    x0[4] += 0.4;
    x0[6] += 0.5;
    x0[8] += 0.06;
    x0[7] = 0.0;

    Eigen::VectorXd P0_dig = Eigen::VectorXd::Constant(11, 1.0);
    P0_dig[1] = P0_dig[3] = P0_dig[5] = 64.0;
    P0_dig[7] = 100.0;
    // 单装甲退化保护：生产路径由 Tracker 把半径先验收紧到这个量级
    P0_dig[8] = config.radius_prior_sigma * config.radius_prior_sigma;
    P0_dig[9] = 0;
    P0_dig[10] = 0;
    Eigen::MatrixXd P0 = P0_dig.asDiagonal();

    RVfromFYT filter(x0, P0, 4, ArmorName::one);
    filter.enable_uv(config);
    filter.set_camera_geometry(geometry);

    const double dt = 0.01;
    for (int step = 0; step < 100; step++) {
      // 真值推进
      truth[0] += truth[1] * dt;
      truth[2] += truth[3] * dt;
      truth[4] += truth[5] * dt;
      truth[6] += truth[7] * dt;

      Eigen::VectorXd u = Eigen::VectorXd::Zero(3);
      Eigen::VectorXd noises(2);
      noises << 100.0, 400.0;
      filter.kf_predict(dt, u, noises);

      // 选出朝向相机的若干装甲板作为本帧观测
      std::vector<int> ids;
      for (int id = 0; id < 4 && static_cast<int>(ids.size()) < visible; id++) {
        if (model.facing_cos(filter.x, id) < config.facing_cos_min) continue;
        ids.push_back(id);
      }
      if (ids.empty()) continue;

      filter.clear_uv_observations();
      std::vector<int> used;
      for (int id : ids) {
        Armor armor = make_armor(model, truth, id, ArmorType::small, pixel_noise, rng);
        const int matched = filter.associate_uv_armor(armor, used, -1.0);
        if (matched < 0) continue;
        if (!filter.add_uv_observation(matched, armor)) continue;
        used.push_back(matched);
      }
      if (filter.uv_observation_count() == 0) continue;
      filter.correct_uv_batch();
    }

    const double pos_err = std::hypot(filter.x[0] - truth[0], filter.x[2] - truth[2]);
    const double z_err = std::abs(filter.x[4] - truth[4]);
    const double yaw_err = std::abs(std::remainder(filter.x[6] - truth[6], 2 * M_PI));
    const double r_err = std::abs(filter.x[8] - truth[8]);

    char name[96];
    std::snprintf(name, sizeof(name), "可见 %d 块装甲：位置误差 < 0.15m", visible);
    check(pos_err < 0.15, name, pos_err, 0.15);
    std::snprintf(name, sizeof(name), "可见 %d 块装甲：深度误差 < 0.15m", visible);
    check(z_err < 0.15, name, z_err, 0.15);
    std::snprintf(name, sizeof(name), "可见 %d 块装甲：偏航误差 < 0.15rad", visible);
    check(yaw_err < 0.15, name, yaw_err, 0.15);
    std::snprintf(name, sizeof(name), "可见 %d 块装甲：半径误差 < 0.05m", visible);
    check(r_err < 0.05, name, r_err, 0.05);

    std::printf("        pos=%.4f z=%.4f yaw=%.4f r=%.4f (r_est=%.3f r_true=%.3f)\n", pos_err,
                z_err, yaw_err, r_err, filter.x[8], truth[8]);
  }

  std::printf("\n%s (%d failures)\n",
              failures == 0 ? "UV FILTER TESTS PASSED" : "UV FILTER TESTS FAILED", failures);
  return failures == 0 ? 0 : 1;
}
