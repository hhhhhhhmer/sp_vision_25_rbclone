#include <cmath>
#include <cstdlib>
#include <optional>
#include <utility>

#include "tasks/auto_aim/aiming/planner/planner.hpp"

#define CHECK(condition)             \
  do {                               \
    if (!(condition)) std::abort();  \
  } while (false)

namespace
{
bool finite(const auto_aim::Plan & plan)
{
  return std::isfinite(plan.target_yaw) && std::isfinite(plan.target_pitch) &&
         std::isfinite(plan.yaw) && std::isfinite(plan.yaw_vel) &&
         std::isfinite(plan.yaw_acc) && std::isfinite(plan.pitch) &&
         std::isfinite(plan.pitch_vel) && std::isfinite(plan.pitch_acc);
}

bool near(double lhs, double rhs) { return std::abs(lhs - rhs) < 1e-8; }
}  // namespace

int main(int argc, char ** argv)
{
  CHECK(argc == 2);

  auto_aim::Planner original(argv[1]);
  auto_aim::Planner copied(original);
  auto_aim::Planner assigned(argv[1]);
  assigned = original;
  auto_aim::Planner moved(std::move(assigned));

  const auto_aim::Target target(3.0, 0.5, 0.2, 0.1);
  const auto original_plan = original.plan(target, 22.0);
  const auto copied_plan = copied.plan(target, 22.0);
  const auto moved_plan = moved.plan(target, 22.0);

  CHECK(original_plan.control && copied_plan.control && moved_plan.control);
  CHECK(finite(original_plan) && finite(copied_plan) && finite(moved_plan));
  CHECK(near(original_plan.yaw, copied_plan.yaw));
  CHECK(near(original_plan.pitch, copied_plan.pitch));
  CHECK(near(original_plan.yaw, moved_plan.yaw));
  CHECK(near(original_plan.pitch, moved_plan.pitch));

  const auto hero_plan = original.rbHeroplan(target, 22.0, 0.0);
  CHECK(hero_plan.control);
  CHECK(finite(hero_plan));

  const auto idle_plan = original.plan(std::optional<auto_aim::Target>{}, 22.0);
  CHECK(!idle_plan.control);
  return 0;
}
