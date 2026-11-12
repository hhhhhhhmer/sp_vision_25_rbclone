// UV + 窄窗 PnP yaw 测试：
//  1) Solver::try_solve_yaw() 能从合成角点恢复装甲朝向，并置 pnp_yaw_valid
//  2) 挂上该朝向观测后，RVfromFYT 的 θ 会被拉向测量；>3σ 的野值（正对时的镜像解）被拒
//  3) 自转目标跑 100 步：开/关朝向观测都不劣化位置/深度/半径收敛
#include <chrono>
#include <cmath>
#include <cstdio>
#include <random>
#include <string>
#include <vector>

#include <Eigen/Dense>

#include "tasks/auto_aim/geometry/solver.hpp"
#include "tasks/auto_aim/tracking/rv_from_fyt.hpp"
#include "tasks/auto_aim/tracking/uv_model.hpp"

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

/** @brief 目标整车真值：4 装甲板、静止在 4.2m 附近 @param vyaw 自转角速度 (rad/s) @return 17 维状态 */
Eigen::VectorXd make_truth(double vyaw)
{
  Eigen::VectorXd truth = Eigen::VectorXd::Zero(RVfromFYT::kStateDimension);
  truth[0] = 4.2;
  truth[2] = 0.35;
  truth[4] = 0.25;
  truth[6] = 0.0;
  truth[7] = vyaw;
  truth[8] = 0.20;
  return truth;
}

/** @brief 4 装甲板上某块板的绝对朝向 @param truth 真值状态 @param armor_id 编号 @return 朝向 (rad) */
double plate_yaw(const Eigen::VectorXd & truth, int armor_id)
{
  return truth[6] + armor_id * 2.0 * M_PI / 4.0;
}

/** @brief 用真值 + 像素噪声造一块观测装甲板 @return 装甲板（角点已加噪） */
Armor make_armor(
  const UvModel & model, const Eigen::VectorXd & truth, int armor_id,
  std::normal_distribution<double> & noise, std::mt19937 & rng)
{
  Armor armor;
  armor.type = ArmorType::small;
  armor.name = ArmorName::one;
  armor.points = model.project_corners(truth, armor_id, ArmorType::small);
  for (auto & p : armor.points) {
    p.x += static_cast<float>(noise(rng));
    p.y += static_cast<float>(noise(rng));
  }
  return armor;
}

UvConfig make_config(bool pnp_yaw_enabled)
{
  UvConfig config;
  config.enabled = true;
  config.sigma_px = 1.0;
  config.sigma_len_ratio = 0.045;
  config.sigma_attitude_deg = 0.5;
  config.sigma_angle = 0.1;
  config.associate_gate_px = 60.0;
  config.facing_cos_min = 0.15;
  config.radius_prior_sigma = 0.02;
  config.process_noise_z_scale = 0.05;
  config.pnp_yaw_enabled = pnp_yaw_enabled;
  config.pnp_yaw_sigma_deg = 5.0;
  config.pnp_yaw_window_deg = 20.0;
  config.pnp_yaw_steps = 20;
  return config;
}

Eigen::VectorXd make_p0()
{
  Eigen::VectorXd P0_dig = Eigen::VectorXd::Constant(RVfromFYT::kStateDimension, 1.0);
  P0_dig[1] = P0_dig[3] = P0_dig[5] = 64.0;
  P0_dig[7] = 100.0;
  P0_dig[8] = 0.02 * 0.02;
  for (int i = 0; i < 3; ++i) {
    P0_dig[RVfromFYT::kLateralOffsetIndex + i] = 0.0025;
    P0_dig[RVfromFYT::kHeightOffsetIndex + i] = 0.0025;
  }
  return P0_dig;
}
}  // namespace

int main(int argc, char ** argv)
{
  const std::string config_path = argc > 1 ? argv[1] : "../configs/rb_auto_aim.yaml";

  Solver solver(config_path);
  const CameraGeometry geometry = solver.camera_geometry();
  check(geometry.valid, "求解器提供了有效相机几何");
  if (!geometry.valid) {
    std::printf("\nUV PNP YAW TESTS FAILED (%d failures)\n", failures);
    return 1;
  }

  // ---------------------------------------------------------------- 1) 窄窗 yaw 解算
  std::printf("=== 1) Solver::try_solve_yaw ===\n");
  {
    // 让装甲板正对相机：θ 取"装甲板指向原点"的方向
    const Eigen::Vector3d xyz(4.2, 0.35, 0.25);
    const double truth_yaw = std::atan2(xyz.y(), xyz.x());

    Armor armor;
    armor.type = ArmorType::small;
    armor.name = ArmorName::one;
    armor.points = solver.reproject_armor(xyz, truth_yaw, armor.type, armor.name);

    const bool solved = solver.try_solve_yaw(armor, 20.0, 20);
    const double yaw_err = std::abs(std::remainder(armor.ypr_in_world[0] - truth_yaw, 2 * M_PI));
    const double pos_err = (armor.xyz_in_world - xyz).norm();

    check(solved, "try_solve_yaw 成功");
    check(armor.pnp_yaw_valid, "置位 pnp_yaw_valid");
    std::printf("        yaw_err=%.4f rad (%.2f deg), pos_err=%.4f m\n", yaw_err,
                yaw_err * 180.0 / M_PI, pos_err);
    check(yaw_err < 3.0 * M_PI / 180.0, "朝向误差 < 3deg", yaw_err, 3.0 * M_PI / 180.0);
    check(pos_err < 0.20, "位置误差 < 0.2m", pos_err, 0.20);

    // 平衡装甲（英雄大装甲）不做 yaw 优化，必须返回 false
    Armor balance = armor;
    balance.type = ArmorType::big;
    balance.name = ArmorName::three;
    balance.pnp_yaw_valid = false;
    check(!solver.try_solve_yaw(balance, 20.0, 20), "平衡装甲拒绝 yaw 解算");

    // 可选：量一下窄窗 yaw 的耗时（不影响测试结果）
    if (argc > 2 && std::string(argv[2]) == "--bench") {
      constexpr int kIter = 2000;
      Armor bench = armor;
      const auto t0 = std::chrono::steady_clock::now();
      for (int i = 0; i < kIter; i++) {
        bench.pnp_yaw_valid = false;
        solver.try_solve_yaw(bench, 20.0, 20);
      }
      const double us =
        std::chrono::duration<double, std::micro>(std::chrono::steady_clock::now() - t0).count() /
        kIter;
      std::printf("        try_solve_yaw: %.1f us/次（含一次 solvePnP + 20 步窄窗搜索）\n", us);
    }
  }

  // ---------------------------------------------------------------- 2) θ 的 1 维观测
  std::printf("=== 2) 装甲朝向的 1 维观测 ===\n");
  UvModel model;
  model.configure(make_config(true), 4, ArmorName::one);
  model.set_camera_geometry(geometry);

  const Eigen::VectorXd truth = make_truth(0.0);
  const int id = 0;

  // 同一份观测数据（无像噪）供三个子测试复用
  Armor armor;
  armor.type = ArmorType::small;
  armor.name = ArmorName::one;
  armor.points = model.project_corners(truth, id, ArmorType::small);
  armor.ypr_in_world[0] = plate_yaw(truth, id);
  armor.pnp_yaw_valid = true;

  {
    // a) 只做 yaw 更新：θ 必须被拉向测量，且 H 只有第 6 列，其它状态一位都不许变
    Eigen::VectorXd x0 = truth;
    x0[6] += 0.10;
    RVfromFYT filter(x0, make_p0().asDiagonal(), 4, ArmorName::one);
    filter.enable_uv(make_config(true));
    filter.set_camera_geometry(geometry);

    const Eigen::VectorXd x_before = filter.x;
    const double before = std::abs(std::remainder(filter.x[6] - truth[6], 2 * M_PI));
    const bool applied = filter.correct_uv_yaw(id, armor.ypr_in_world[0], 5.0 * M_PI / 180.0);
    const double after = std::abs(std::remainder(filter.x[6] - truth[6], 2 * M_PI));
    check(applied, "correct_uv_yaw 执行成功");
    std::printf("        theta 误差 %.4f -> %.4f rad\n", before, after);
    check(after < before * 0.1, "朝向观测把 θ 误差压到 10% 以下", after, before * 0.1);

    Eigen::VectorXd delta = filter.x - x_before;
    delta[6] = 0.0;
    check(delta.norm() < 1e-12, "只有 θ 状态被改动", delta.norm(), 1e-12);

    // b) 8σ 野值（正对时 PnP 的镜像解）必须被 3σ 门限拒绝、状态不变
    const Eigen::VectorXd x_snapshot = filter.x;
    const bool outlier_applied =
      filter.correct_uv_yaw(id, plate_yaw(truth, id) + 40.0 * M_PI / 180.0, 5.0 * M_PI / 180.0);
    check(!outlier_applied, "8σ 的朝向野值被拒绝");
    check(
      (filter.x - x_snapshot).norm() < 1e-12, "被拒绝后状态不变", (filter.x - x_snapshot).norm(),
      1e-12);
  }

  {
    // c) 接线：pnp_yaw_valid 的装甲板经 add_uv_observation 后，θ 应当比纯 UV 更准
    double yaw_err[2] = {0.0, 0.0};
    for (int variant = 0; variant < 2; variant++) {
      const bool pnp_yaw = variant == 1;
      Eigen::VectorXd x0 = truth;
      x0[6] += 0.10;
      RVfromFYT filter(x0, make_p0().asDiagonal(), 4, ArmorName::one);
      filter.enable_uv(make_config(pnp_yaw));
      filter.set_camera_geometry(geometry);

      Armor observation = armor;
      observation.pnp_yaw_valid = pnp_yaw;  // 纯 UV 变体不带窄窗 yaw
      filter.clear_uv_observations();
      check(filter.add_uv_observation(id, observation), "add_uv_observation 成功");
      filter.correct_uv_batch();
      yaw_err[variant] = std::abs(std::remainder(filter.x[6] - truth[6], 2 * M_PI));
    }
    std::printf("        theta 误差：纯 UV %.4f rad，UV+PnP yaw %.4f rad\n", yaw_err[0], yaw_err[1]);
    check(yaw_err[1] < yaw_err[0], "UV+PnP yaw 的 θ 比纯 UV 更准", yaw_err[1], yaw_err[0]);
  }

  // ---------------------------------------------------------------- 3) 自转目标 100 步
  std::printf("=== 3) 自转目标 100 步（θ 观测开 / 关）===\n");
  for (bool pnp_yaw : {false, true}) {
    Eigen::VectorXd state = make_truth(1.2);
    Eigen::VectorXd truth_rot = make_truth(1.2);
    state[6] = truth_rot[6] - 0.4;  // 初始相位偏 0.4 rad

    const UvConfig config = make_config(pnp_yaw);
    RVfromFYT filter(state, make_p0().asDiagonal(), 4, ArmorName::one);
    filter.enable_uv(config);
    filter.set_camera_geometry(geometry);
    UvModel local_model;
    local_model.configure(config, 4, ArmorName::one);
    local_model.set_camera_geometry(geometry);

    std::mt19937 local_rng(20261113);
    std::normal_distribution<double> local_noise(0.0, 1.0);
    const double dt = 0.01;
    for (int step = 0; step < 100; step++) {
      truth_rot[6] += truth_rot[7] * dt;
      Eigen::VectorXd u = Eigen::VectorXd::Zero(3);
      Eigen::VectorXd noises(2);
      noises << 100.0, 400.0;
      filter.kf_predict(dt, u, noises);

      filter.clear_uv_observations();
      std::vector<int> used;
      for (int id = 0; id < 4; id++) {
        if (local_model.facing_cos(filter.x, id) < config.facing_cos_min) continue;
        Armor armor = make_armor(local_model, truth_rot, id, local_noise, local_rng);
        if (pnp_yaw) {
          // 直接给真值朝向 + 1σ 噪声，避免测试依赖 PnP 在正对时的镜像解行为
          std::normal_distribution<double> yaw_noise(0.0, config.pnp_yaw_sigma_deg * M_PI / 180.0);
          armor.ypr_in_world[0] = plate_yaw(truth_rot, id) + yaw_noise(local_rng);
          armor.pnp_yaw_valid = true;
        }
        const int matched = filter.associate_uv_armor(armor, used, -1.0);
        if (matched < 0) continue;
        if (!filter.add_uv_observation(matched, armor)) continue;
        used.push_back(matched);
      }
      if (filter.uv_observation_count() == 0) continue;
      filter.correct_uv_batch();
    }

    const double pos_err = std::hypot(filter.x[0] - truth_rot[0], filter.x[2] - truth_rot[2]);
    const double z_err = std::abs(filter.x[4] - truth_rot[4]);
    const double yaw_err = std::abs(std::remainder(filter.x[6] - truth_rot[6], 2 * M_PI));
    const double r_err = std::abs(filter.x[8] - truth_rot[8]);
    std::printf("        %s: pos=%.4f z=%.4f yaw=%.4f r=%.4f\n", pnp_yaw ? "UV+PnP yaw" : "纯 UV    ",
                pos_err, z_err, yaw_err, r_err);

    const char * tag = pnp_yaw ? "（开 θ 观测）" : "（纯 UV）";
    char name[128];
    std::snprintf(name, sizeof(name), "位置误差 < 0.15m %s", tag);
    check(pos_err < 0.15, name, pos_err, 0.15);
    std::snprintf(name, sizeof(name), "深度误差 < 0.15m %s", tag);
    check(z_err < 0.15, name, z_err, 0.15);
    std::snprintf(name, sizeof(name), "朝向误差 < 0.15rad %s", tag);
    check(yaw_err < 0.15, name, yaw_err, 0.15);
    std::snprintf(name, sizeof(name), "半径误差 < 0.05m %s", tag);
    check(r_err < 0.05, name, r_err, 0.05);
  }

  std::printf("\n%s (%d failures)\n",
              failures == 0 ? "UV PNP YAW TESTS PASSED" : "UV PNP YAW TESTS FAILED", failures);
  return failures == 0 ? 0 : 1;
}
