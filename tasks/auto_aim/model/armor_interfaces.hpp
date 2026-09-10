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
   * @brief 获取当前相机的内外参几何（UV 观测使用）
   * @return 相机几何；求解器不支持时 valid=false
   * @note 长短焦切换时 Tracker 会换成另一台相机的求解器，因此本接口必须能返回当前相机
   */
  virtual CameraGeometry camera_geometry() const { return CameraGeometry{}; }
};
}  // namespace auto_aim

#endif  // AUTO_AIM__ARMOR_INTERFACES_HPP
