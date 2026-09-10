#ifndef AUTO_AIM__CAMERA_GEOMETRY_HPP
#define AUTO_AIM__CAMERA_GEOMETRY_HPP

#include <Eigen/Dense>
#include <vector>

namespace auto_aim
{
/**
 * @brief UV 观测所需的相机/世界几何
 *
 * 由位姿求解器提供（Solver 从 yaml 读取），供 UV 观测模型把状态预测的装甲板投影到像素。
 * 长短焦切换时 Tracker 会通过 setSolver/setPoseSolver 换成另一台相机的求解器，
 * 因此本结构每帧从当前求解器获取即可自动跟随相机切换。
 */
struct CameraGeometry
{
  bool valid = false;                              ///< 求解器是否支持提供相机几何
  Eigen::Matrix3d camera_matrix = Eigen::Matrix3d::Identity();  ///< 3x3 内参
  std::vector<double> distort_coeffs;              ///< 畸变系数 k1,k2,p1,p2,k3
  Eigen::Matrix3d R_camera2gimbal = Eigen::Matrix3d::Identity();  ///< 相机到云台旋转
  Eigen::Vector3d t_camera2gimbal = Eigen::Vector3d::Zero();      ///< 相机到云台平移
  Eigen::Matrix3d R_gimbal2world = Eigen::Matrix3d::Identity();   ///< 云台到世界旋转
};
}  // namespace auto_aim

#endif  // AUTO_AIM__CAMERA_GEOMETRY_HPP
