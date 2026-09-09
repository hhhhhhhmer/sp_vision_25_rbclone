// rv_from_fyt.hpp
#ifndef STATE2EST_HPP
#define STATE2EST_HPP

#include <Eigen/Dense>
#include <array>
#include <chrono>
#include <utility>
#include <vector>

#include "tasks/auto_aim/armor.hpp"
#include "tools/extended_kalman_filter.hpp"

namespace auto_aim
{
/// @brief 状态估计接口
class State2Est : public tools::ExtendedKalmanFilter
{
public:
  using tools::ExtendedKalmanFilter::ExtendedKalmanFilter;

  virtual ~State2Est() = default;
    
  /// @brief         卡尔曼更新
  /// @param dt      采集间隔
  /// @param u       控制量
  /// @param noises  噪声
  virtual void kf_predict(double dt, const Eigen::VectorXd & u, const Eigen::VectorXd noises) = 0;

  /// @brief         mpc预测
  /// @param dt      采集间隔
  /// @param u       控制量
  /// @param noises  噪声
  virtual void mpc_predict(double dt, const Eigen::VectorXd & u, const Eigen::VectorXd noises) = 0;

  /// @brief 计算 [x, y, z, yaw] 观测量与后验估计量的逐分量平方残差
  /// @param observation_xyzyaw 观测量 [x, y, z, yaw]
  /// @param armor_id 装甲板 ID；对不区分装甲板的模型忽略此参数
  /// @return [dx^2, dy^2, dz^2, dyaw^2]，其中偏航残差已归一化到 [-pi, pi]
  virtual Eigen::Vector4d posterior_residual_squared(
    const Eigen::Vector4d & observation_xyzyaw, int armor_id = 0) const = 0;
};
}  // namespace auto_aim


#endif  // STATE2EST_HPP
