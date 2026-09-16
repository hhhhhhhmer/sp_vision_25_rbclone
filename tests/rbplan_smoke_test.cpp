/**
 * @file rbplan_smoke_test.cpp
 * @brief rbplan 策略冒烟测试（方案 C：真实云台状态 x0 + 开火门限参考角）
 *
 * 校验内容：
 *  1) 加入 gimbal_delay 提前量与陀螺运动目标下，rbplan 全程输出有限值；
 *  2) "匀速小陀螺目标"必须能触发开火——开火门限的参考角若写错
 *     （例如把 speed_delay 也回退），会出现"完全命中也被拒发"；
 *  3) 下发的角命令幅值在合理范围内（命令索引/单位出错会表现为 ±90° 量级跳变）。
 *
 * 用法：rbplan_smoke_test <config.yaml>
 */
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <optional>
#include <string>

#include "tasks/auto_aim/aiming/planner/planner.hpp"

namespace
{
constexpr double kDegToRad = M_PI / 180.0;

struct Stats
{
  bool finite = true;
  bool ever_fire = false;
  double yaw_span_deg = 0;
  double pitch_span_deg = 0;
};

/** @brief 跑一段匀速小陀螺目标，统计开火与命令幅值 @param config 规划器配置路径 */
Stats run(const std::string & config)
{
  auto_aim::Planner planner(config);

  // 匀速小陀螺目标：3 m、2 rad/s
  auto_aim::Target target(3.0, 2.0, 0.2, 0.1);
  auto_aim::Plan last{};
  Stats st;
  double yaw_min = 1e9, yaw_max = -1e9, pitch_min = 1e9, pitch_max = -1e9;

  for (int i = 0; i < 300; ++i) {
    target.predict(0.01);
    // 模拟真实云台：稳态下云台已跟随上一拍瞄点（用瞄点角作为"云台实际角"近似）
    const auto plan = planner.plan(
      std::optional<auto_aim::Target>(target), 22.0, last.target_yaw,
      auto_aim::Planner::ShootStrategy::rbSuppressiveFire);
    last = plan;

    if (!std::isfinite(plan.yaw) || !std::isfinite(plan.pitch) ||
        !std::isfinite(plan.yaw_vel) || !std::isfinite(plan.pitch_vel)) {
      st.finite = false;
      return st;
    }
    if (plan.control) {
      yaw_min = std::min(yaw_min, static_cast<double>(plan.yaw));
      yaw_max = std::max(yaw_max, static_cast<double>(plan.yaw));
      pitch_min = std::min(pitch_min, static_cast<double>(plan.pitch));
      pitch_max = std::max(pitch_max, static_cast<double>(plan.pitch));
    }
    if (plan.fire) st.ever_fire = true;
  }

  st.yaw_span_deg = (yaw_max - yaw_min) / kDegToRad;
  st.pitch_span_deg = (pitch_max - pitch_min) / kDegToRad;
  return st;
}
}  // namespace

int main(int argc, char ** argv)
{
  if (argc != 2) {
    std::printf("usage: rbplan_smoke_test <config.yaml>\n");
    return 2;
  }
  const auto st = run(argv[1]);

  if (!st.finite) {
    std::printf("FAIL: 规划输出出现非有限值\n");
    return 1;
  }
  // 2 rad/s 小陀螺、3 m：瞄点必在云台可达范围内，命令幅值不应出现异常跳变
  if (!(st.yaw_span_deg < 360.0)) {
    std::printf("FAIL: yaw 命令幅值异常 = %.1f deg\n", st.yaw_span_deg);
    return 1;
  }
  if (!(st.pitch_span_deg < 90.0)) {
    std::printf("FAIL: pitch 命令幅值异常 = %.1f deg\n", st.pitch_span_deg);
    return 1;
  }
  // 门限参考角写错时，稳态匀速目标会永远不开火
  if (!st.ever_fire) {
    std::printf("FAIL: 稳态匀速小陀螺目标未能触发开火（开火门限参考角可能不对）\n");
    return 1;
  }

  std::printf(
    "rbplan smoke OK | fire=%s span(yaw=%.1fdeg pitch=%.1fdeg)\n", st.ever_fire ? "yes" : "no",
    st.yaw_span_deg, st.pitch_span_deg);
  return 0;
}
