#ifndef AUTO_AIM__TOWER_ARMOR_GEOMETRY_HPP
#define AUTO_AIM__TOWER_ARMOR_GEOMETRY_HPP

#include <array>
#include <cmath>
#include <utility>

namespace auto_aim
{
/** @brief 前哨站装甲板高度锚点差大于该值时认为相差 2 级 */
constexpr double kTowerArmorLargeHeightJump = 0.16;
/** @brief 前哨站装甲板高度锚点差大于该值时认为相差 1 级 */
constexpr double kTowerArmorSmallHeightJump = 0.05;

/**
 * @brief 由前哨站装甲板高度锚点计算高度乘数
 *
 * 观测模型（UvModel::armor_position）与状态读回（RVfromFYT::h_armor_xyz）必须共用这一份实现：
 * 前者决定滤波器把观测拟合到什么几何上，后者决定瞄准点取在哪，两者一旦分叉，
 * 滤波器估计出的状态与瞄准用的几何就会不一致（曾导致前哨站 1/2 号板 0.2~0.4m 的高度偏差）。
 *
 * 锚点无效时返回 0（视为与该前哨站 0 号装甲板同高）：无效锚点的占位值是 0.0，
 * 直接参与差值计算会得出"该板比 0 号板低 2 级"这类凭空结论，凭空结论会被瞄准点直接吃进去。
 *
 * @param heights 3 块装甲板的高度锚点（是否有效 + 高度，单位 m）
 * @param armor_id 装甲板编号
 * @return 高度乘数，取值范围 {-2, -1, 0, 1, 2}
 */
inline double tower_armor_height_multiplier(
  const std::array<std::pair<bool, double>, 3> & heights, int armor_id)
{
  if (armor_id < 0 || armor_id >= static_cast<int>(heights.size())) return 0.0;
  if (!heights[0].first || !heights[armor_id].first) return 0.0;

  const double height_delta = heights[armor_id].second - heights[0].second;
  const double direction = height_delta > 0.0 ? 1.0 : -1.0;

  int step_count = 0;
  if (std::abs(height_delta) > kTowerArmorLargeHeightJump) {
    step_count = 2;
  } else if (std::abs(height_delta) > kTowerArmorSmallHeightJump) {
    step_count = 1;
  }
  return direction * step_count;
}
}  // namespace auto_aim

#endif  // AUTO_AIM__TOWER_ARMOR_GEOMETRY_HPP
