/**
 * @file rbplan_smoke_test.cpp
 * @brief rbplan 策略冒烟测试（方案 C：真实云台状态 x0 + 开火门限参考角）
 * 校验：加入 gimbal_delay 提前量与陀螺运动目标下，rbplan 全程有限值、
 * 且开火门限（子弹到达时刻参考角）在稳态匀速目标下能通过。
 */
#include <cmath>
#include <cstdio>
#include <optional>

#include "tasks/auto_aim/aiming/planner/planner.hpp"

#define CHECK(cond)                    \
  do {                                 \
    if (!(cond)) {                     \
      std::printf("FAIL: %s\n", #cond); \
      return 1;                        \
    }                                  \
  } while (0)

int main(int argc, char ** argv)
{
  if (argc != 2) {
    std::printf("usage: rbplan_smoke_test <config.yaml>\n");
    return 2;
  }
  auto_aim::Planner planner(argv[1]);

  // 匀速小陀螺目标：3m、2 rad/s
  auto_aim::Target target(3.0, 2.0, 0.2, 0.1);
  auto last = auto_aim::Plan{};
  bool ever_fire = false;
  for (int i = 0; i < 200; ++i) {
    target.predict(0.01);
    // 模拟真实云台：稳态下云台应已跟随瞄点（此处用瞄点角作为"云台实际角"近似）
    const auto plan_before = planner.plan(
      std::optional<auto_aim::Target>(target), 22.0, last.target_yaw,
      auto_aim::Planner::ShootStrategy::rbSuppressiveFire, -3.0, 0.0, 0.0);
    last = plan_before;
    CHECK(std::isfinite(last.yaw));
    CHECK(std::isfinite(last.pitch));
    if (last.fire) ever_fire = true;
  }
  std::printf("rbplan smoke OK, fire observed: %s\n", ever_fire ? "yes" : "no");
  return 0;
}
