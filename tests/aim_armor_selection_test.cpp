/**
 * @file aim_armor_selection_test.cpp
 * @brief 瞄板选择：滞环规则 + "状态跨帧存活"回归测试
 *
 * 背景（历史 bug）：滞环状态原本存在 Target::aim_armor_id_ / aim_select_frame_ 里，
 * 但 Target 在下发链路上是逐层按值拷贝的（tracker → queue → plan(optional<Target>) → rbplan），
 * 状态写在拷贝上、下一帧就丢，于是"保持当前板"这一侧从未生效，实际行为等价于
 * 纯 argmin 最近板（在 45° 临界点来回切）。
 *
 * 本测试两部分：
 *  1) 纯规则：直接校验 Target::select_aim_armor(held_id, &nearest) 的保持/换板/兜底分支，
 *     并记录"held=-1 时恒等于最近板"（即历史 bug 的行为）；
 *  2) 端到端：跑真实 Planner（含 MPC）跟踪匀速小陀螺目标，校验
 *     a. 切板频率 ≈ 4 次/圈；
 *     b. 选板滞环真的生效（Planner::aim_selection_debug().hold_engaged 出现次数 > 0）。
 *     验收标准就是 b —— 旧代码下它恒为 0。
 *
 * 用法：aim_armor_selection_test <config.yaml>
 */
#include <cmath>
#include <cstdio>
#include <optional>
#include <string>

#include "tasks/auto_aim/aiming/planner/planner.hpp"
#include "tasks/auto_aim/tracking/target.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"

namespace
{
constexpr double kDeg = 180.0 / M_PI;

int failures = 0;

void check(bool ok, const std::string & what)
{
  if (ok) {
    std::printf("  [ok]   %s\n", what.c_str());
  } else {
    std::printf("  [FAIL] %s\n", what.c_str());
    failures++;
  }
}

/** @brief 造一个已转到指定 yaw 的四板目标（虚警：yaw=0 时 0 号板正对相机） */
auto_aim::Target make_target(double yaw_deg, double dist = 3.0, double radius = 0.2)
{
  auto_aim::Target t(dist, 1.0, radius, 0.1);  // 1 rad/s，方便用 predict 控制 yaw
  t.predict(yaw_deg / kDeg);                   // yaw += 1.0 * dt
  return t;
}

/** @brief 第一部分：纯规则分支 */
void test_rule()
{
  std::printf("== 1. 纯规则 Target::select_aim_armor(held_id) ==\n");

  {
    auto t = make_target(0.0);
    int nearest = -1;
    const int id = t.select_aim_armor(-1, &nearest);
    check(id == 0 && nearest == 0, "yaw=0°：最近板 = 0 号板，未持板时返回 0");
  }

  {
    // yaw=50°：0 号板已转过临界点（|50°|），最近板变成 3 号板（|-40°|）
    auto t = make_target(50.0);
    int nearest = -1;
    const int pure = t.select_aim_armor(-1, &nearest);
    check(pure == 3 && nearest == 3, "yaw=50°：纯 argmin 最近板 = 3 号板（对照组）");

    int nearest2 = -1;
    const int held = t.select_aim_armor(0, &nearest2);
    check(
      held == 0 && nearest2 == 3,
      "yaw=50°：持有 0 号板时滞环生效（保持 0，而最近板是 3）—— 历史 bug 下这里会返回 3");
  }

  {
    // yaw=55°：最近板比持有的板近 > kAimHysteresisDist(3cm)，滞环允许换板
    auto t = make_target(55.0);
    int nearest = -1;
    const int id = t.select_aim_armor(0, &nearest);
    check(id == 3 && nearest == 3, "yaw=55°：差距超过滞环门限，允许换到 3 号板");
  }

  {
    // yaw=120°：持有的 0 号板已转过头（>kAimHoldAngle=100°），必须放手
    auto t = make_target(120.0);
    int nearest = -1;
    const int id = t.select_aim_armor(0, &nearest);
    check(id == nearest && id >= 0, "yaw=120°：持板不可用（>100°）时回退到最近板");
  }

  {
    auto t = make_target(50.0);
    int nearest = -1;
    const int id = t.select_aim_armor(99, &nearest);  // 非法编号
    check(id == nearest && id >= 0, "持板编号非法时回退到最近板");
  }

  {
    // 历史 bug 的行为：held 恒为 -1（状态丢失）时，选择永远等于最近板
    int same = 0;
    for (int yaw = 0; yaw < 360; yaw += 5) {
      auto t = make_target(static_cast<double>(yaw));
      int nearest = -1;
      if (t.select_aim_armor(-1, &nearest) == nearest) same++;
    }
    check(same == 72, "held=-1（状态丢失）时 72/72 个角度都等于最近板 = 旧行为");
  }
}

struct E2eStats
{
  int frames = 0;
  int switches = 0;
  int hold_engaged = 0;
  double max_step_deg = 0;
  bool finite = true;
};

/** @brief 第二部分：真实 Planner 端到端（小陀螺 4 rad/s、3 m） */
E2eStats test_planner(const std::string & config, double vyaw, double seconds)
{
  auto_aim::Planner planner(config);
  auto_aim::Target target(3.0, vyaw, 0.2, 0.1);  // 半径 0.2m 的标准机器人

  E2eStats st;
  int last_id = -1;
  double last_yaw = 0;
  const int frames = static_cast<int>(seconds / 0.01);
  double gimbal_yaw = 0;

  for (int i = 0; i < frames; ++i) {
    target.predict(0.01);
    target.update_count_++;  // 真实由 Target::bookkeep_after_update 每帧自增一次

    const auto plan = planner.plan(
      std::optional<auto_aim::Target>(target), 22.0, gimbal_yaw,
      auto_aim::Planner::ShootStrategy::rbSuppressiveFire);

    if (!plan.control || !std::isfinite(plan.yaw) || !std::isfinite(plan.yaw_vel) ||
        !std::isfinite(plan.pitch)) {
      st.finite = false;
      return st;
    }
    st.frames++;

    const int id = planner.aim_armor_id();
    if (last_id >= 0 && id != last_id) {
      st.switches++;
      const double step = std::abs(tools::limit_rad(plan.yaw - last_yaw)) * kDeg;
      if (step > st.max_step_deg) st.max_step_deg = step;
    }
    last_id = id;
    last_yaw = plan.yaw;
    gimbal_yaw = plan.yaw;
    if (planner.aim_selection_debug().hold_engaged) st.hold_engaged++;
  }
  return st;
}

}  // namespace

int main(int argc, char ** argv)
{
  if (argc != 2) {
    std::printf("usage: aim_armor_selection_test <config.yaml>\n");
    return 2;
  }
  // 规划器每拍都会往控制台打 fly_time，测试里关掉以免刷屏
  tools::logger()->set_level(spdlog::level::off);

  test_rule();

  std::printf("\n== 2. 端到端：真实 Planner 跟踪小陀螺（3m，4rad/s，10s） ==\n");
  const auto st = test_planner(argv[1], 4.0, 10.0);

  check(st.finite, "全程输出有限值且有控制量");
  if (st.finite && st.frames > 0) {
    const double expected = 4.0 * 4.0 * (st.frames * 0.01) / (2 * M_PI);  // 4 次/圈
    std::printf(
      "  实测：切板 %d 次 / %.1fs（理论 %.1f 次），最大单帧台阶 %.2f°，"
      "滞环保持侧生效 %d 帧\n",
      st.switches, st.frames * 0.01, expected, st.max_step_deg, st.hold_engaged);

    check(
      std::abs(st.switches - expected) <= 0.25 * expected,
      "切板频率 ≈ 4 次/圈（小陀螺换板频率由几何决定）");
    check(st.max_step_deg < 10.0, "单帧命令台阶 < 10°（未出现异常甩头）");
    // 验收标准：滞环状态真的跨帧存活，否则恒为 0（旧代码就是这样）
    check(st.hold_engaged > 0, "选板滞环生效：hold_engaged 帧数 > 0");
  }

  if (failures == 0) {
    std::printf("\naim_armor_selection_test OK\n");
    return 0;
  }
  std::printf("\naim_armor_selection_test FAILED: %d 项\n", failures);
  return 1;
}
