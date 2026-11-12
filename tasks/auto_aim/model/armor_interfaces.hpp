#ifndef AUTO_AIM__ARMOR_INTERFACES_HPP
#define AUTO_AIM__ARMOR_INTERFACES_HPP

#include <list>

#include <opencv2/core.hpp>

#include "armor.hpp"
#include "camera_geometry.hpp"

namespace auto_aim
{
class IArmorDetector
{
public:
  /** @brief 销毁装甲板检测器接口 */
  virtual ~IArmorDetector() = default;
  /** @brief 检测图像中的装甲板 @param image 输入图像 @param frame_count 调试帧编号，负数表示不指定 @return 检测到的装甲板列表 */
  virtual std::list<Armor> detect(const cv::Mat & image, int frame_count = -1) = 0;
};

class IArmorPoseSolver
{
public:
  /** @brief 销毁装甲板位姿求解器接口 */
  virtual ~IArmorPoseSolver() = default;
  /** @brief 尝试求解并写回装甲板位姿 @param armor 待求解装甲板 @return 求解成功时返回 true */
  virtual bool try_solve(Armor & armor) const = 0;

  /**
   * @brief 只解算装甲板朝向（窄窗 yaw 搜索），供 UV 模式的 θ 弱观测使用
   * @param armor 待求解装甲板；成功时就地写入 ypr_in_world[0]（以及位置），并置 pnp_yaw_valid
   * @param half_window_deg 搜索半宽（deg），以云台朝向为中心
   * @param steps 搜索步数（步长 = 2*half_window/steps）
   * @return 成功时返回 true
   * @note 相比 try_solve() 只省掉"±70° 全窗口"那部分搜索，用于 UV 模式每帧给装甲朝向
   *       补一个直接观测；默认实现返回 false，测试桩求解器会自动退化为纯 UV 观测
   */
  virtual bool try_solve_yaw(Armor & armor, double half_window_deg, int steps) const
  {
    (void)armor;
    (void)half_window_deg;
    (void)steps;
    return false;
  }

  /**
   * @brief 获取当前相机的内外参几何（UV 观测使用）
   * @return 相机几何；求解器不支持时 valid=false
   * @note 长短焦切换时 Tracker 会换成另一台相机的求解器，因此本接口必须能返回当前相机
   */
  virtual CameraGeometry camera_geometry() const { return CameraGeometry{}; }
};
}  // namespace auto_aim

#endif  // AUTO_AIM__ARMOR_INTERFACES_HPP
