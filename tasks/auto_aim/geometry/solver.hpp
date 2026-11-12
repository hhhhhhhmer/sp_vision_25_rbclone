#ifndef AUTO_AIM__SOLVER_HPP
#define AUTO_AIM__SOLVER_HPP

#include <Eigen/Dense>  // 必须在opencv2/core/eigen.hpp上面
#include <Eigen/Geometry>
#include <opencv2/core/eigen.hpp>
#include <vector>

#include "../model/armor.hpp"
#include "../model/armor_interfaces.hpp"

namespace auto_aim
{
class Solver : public IArmorPoseSolver
{
public:
  /** @brief 根据配置加载相机标定与坐标变换 @param config_path YAML 配置文件路径 */
  explicit Solver(const std::string & config_path);

  /** @brief 获取云台坐标系到世界坐标系的旋转 @return 旋转矩阵 */
  Eigen::Matrix3d R_gimbal2world() const;

  /** @brief 由 IMU 四元数更新云台到世界旋转 @param q IMU 姿态四元数 */
  void set_R_gimbal2world(const Eigen::Quaterniond & q);

  /** @brief 由 TF 四元数直接更新云台到世界旋转 @param q TF 姿态四元数 */
  void set_R_gimbal2world_from_tf(const Eigen::Quaterniond & q);

  /** @brief 更新相机内参与畸变参数 @param camera_matrix 3x3 相机内参矩阵 @param distort_coeffs 畸变系数 */
  void set_camera_calibration(
    const Eigen::Matrix3d & camera_matrix, const std::vector<double> & distort_coeffs);

  /** @brief 更新相机到云台的外参 @param R_camera2gimbal 旋转矩阵 @param t_camera2gimbal 平移向量 */
  void set_camera2gimbal(
    const Eigen::Matrix3d & R_camera2gimbal, const Eigen::Vector3d & t_camera2gimbal);

  /** @brief 求解装甲板位姿并写回结果；失败时记录警告 @param armor 待求解装甲板 */
  void solve(Armor & armor) const;

  /** @brief 尝试求解装甲板位姿 @param armor 待求解装甲板 @return 成功时返回 true，求解异常时返回 false */
  bool try_solve(Armor & armor) const override;

  /**
   * @brief 只解算装甲板朝向：PnP 定位置 + 以云台朝向为中心的窄窗 yaw 搜索
   * @param armor 待求解装甲板（就地写回 ypr_in_world/xyz_in_world 并置 pnp_yaw_valid）
   * @param half_window_deg 搜索半宽（deg）
   * @param steps 搜索步数
   * @return 成功时返回 true
   * @note 给 UV 观测补 θ 的直接观测用。窗口窄（默认 ±20°）因此不会像 ±70° 全窗口那样
   *       跳到镜像解/相邻板，代价约为全窗口搜索的 1/7（约 30us + 一次 solvePnP）
   */
  bool try_solve_yaw(Armor & armor, double half_window_deg, int steps) const override;

  /** @brief 获取当前相机内外参与云台到世界的旋转（UV 观测使用） @return 相机几何 */
  CameraGeometry camera_geometry() const override;

  /** @brief 获取相机内参矩阵 @return 3x3 内参 */
  const cv::Mat & camera_matrix() const { return camera_matrix_; }
  /** @brief 获取畸变系数 @return 畸变系数向量 */
  const cv::Mat & distort_coeffs() const { return distort_coeffs_; }
  /** @brief 获取相机到云台的旋转 @return 旋转矩阵 */
  const Eigen::Matrix3d & R_camera2gimbal() const { return R_camera2gimbal_; }
  /** @brief 获取相机到云台的平移 @return 平移向量 */
  const Eigen::Vector3d & t_camera2gimbal() const { return t_camera2gimbal_; }

  /** @brief 求解全向相机观测在大云台坐标系中的偏航 @param armor 待求解装甲板 @param R_camera2biggimbal_ypr 相机到大云台的欧拉角 @param t_camera2biggimbal 相机到大云台的平移 */
  void omn_dig_yaw_solve(Armor & armor,  Eigen::Vector3d R_camera2biggimbal_ypr, Eigen::Vector3d t_camera2biggimbal ) const;

  /** @brief 将世界坐标中的装甲板重投影到图像 @param xyz_in_world 装甲板中心世界坐标 @param yaw 装甲板偏航角 @param type 装甲板尺寸类型 @param name 装甲板名称 @return 四个像素角点 */
  std::vector<cv::Point2f> reproject_armor(
    const Eigen::Vector3d & xyz_in_world, double yaw, ArmorType type, ArmorName name) const;

  /** @brief 计算前哨站装甲板重投影误差 @param armor 装甲板 @param picth 倾斜角 @return 重投影误差 */
  double oupost_reprojection_error(Armor armor, const double & picth);

  /** @brief 将一组世界坐标点投影到图像 @param worldPoints 世界坐标点 @return 像素坐标点 */
  std::vector<cv::Point2f> world2pixel(const std::vector<cv::Point3f> & worldPoints);

private:
  cv::Mat camera_matrix_;
  cv::Mat distort_coeffs_;
  Eigen::Matrix3d R_gimbal2imubody_;
  Eigen::Matrix3d R_camera2gimbal_;
  Eigen::Vector3d t_camera2gimbal_;
  Eigen::Matrix3d R_gimbal2world_;

  /** @brief 通过重投影误差优化装甲板偏航角 @param armor 待优化装甲板 */
  void optimize_yaw(Armor & armor) const;

  /** @brief 在指定中心与半宽内搜索最优偏航角 @param armor 待优化装甲板 @param center_yaw 搜索中心 (rad) @param half_range_deg 搜索半宽 (deg) @param steps 步数 */
  void optimize_yaw_in_range(
    Armor & armor, double center_yaw, double half_range_deg, int steps) const;

  /** @brief 执行 PnP 并把位姿写回装甲板（不含 yaw 优化） @param armor 就地写入 @return 成功时返回 true @note 失败时 armor 可能被部分改写，调用方应传入副本 */
  bool solve_pnp_stage(Armor & armor) const;

  /** @brief 计算指定偏航和倾角下的装甲板重投影误差 @param armor 装甲板 @param yaw 偏航角 @param inclined 倾斜角 @return 重投影误差 */
  double armor_reprojection_error(const Armor & armor, double yaw, const double & inclined) const;
  /** @brief 计算参考点与投影点之间的加权代价 @param cv_refs 参考像素点 @param cv_pts 投影像素点 @param inclined 倾斜角 @return 代价值 */
  double SJTU_cost(
    const std::vector<cv::Point2f> & cv_refs, const std::vector<cv::Point2f> & cv_pts,
    const double & inclined) const;
};

}  // namespace auto_aim

#endif  // AUTO_AIM__SOLVER_HPP
