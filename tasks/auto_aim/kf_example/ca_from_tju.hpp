// ca_from_tju.hpp
#ifndef AUTO_AIM__KF_EXAMPLE__CA_FROM_TJU_HPP
#define AUTO_AIM__KF_EXAMPLE__CA_FROM_TJU_HPP

#include "state2est.hpp"

namespace auto_aim
{

/**
 * @brief 基于TJU TrackQueueV3的常加速度状态估计器。
 * 
 * 状态向量 (11维)： [x, y, z, yaw, vx, vy, vz, vyaw, ax, ay, ayaw]
 * 测量向量：         [x, y, z, yaw]
 */
class CAFromTJU : public State2Est
{
public:
  static constexpr Eigen::Index kStateDimension = 11;  ///< 状态维度
  static constexpr Eigen::Index kMeasurementDimension = 4;  ///< 测量维度
//   static constexpr double kMeasurementVariance = 0.1;  ///< TJU模型默认测量方差

  /** 默认构造函数（未初始化） */
  CAFromTJU() = default;

  /**
   * @brief 带参构造函数
   * @param x0 初始状态向量 (11维)
   * @param P0 初始协方差矩阵 (11x11)
   * @throws std::invalid_argument 如果维度不匹配
   */
  CAFromTJU(const Eigen::VectorXd & x0, const Eigen::MatrixXd & P0);

  /**
   * @brief 执行一次EKF预测步骤（卡尔曼滤波预测）
   * 
   * 此模型为自主模型（无外部输入），因此 @p u 被忽略。
   * @param dt 预测时间步长 (秒)
   * @param u 外部输入（此处未使用，保留接口）
   * @param noises 过程噪声协方差矩阵的对角线元素 (11个非负值)
   */
  void kf_predict(
    double dt, const Eigen::VectorXd & u, const Eigen::VectorXd noises) override;

  /**
   * @brief MPC预测接口（与 kf_predict 相同，保持兼容）
   * @param dt 预测时间步长
   * @param u 外部输入（未使用）
   * @param noises 过程噪声对角线
   */
  void mpc_predict(
    double dt, const Eigen::VectorXd & u, const Eigen::VectorXd noises) override;

  /**
   * @brief 应用常加速度转移矩阵和协方差预测
   * @param dt 预测间隔 (秒)
   * @param process_noise_diagonal 过程噪声协方差矩阵 Q 的 11 个对角线元素（非负）
   * @throws std::invalid_argument 如果 dt 无效或噪声维度/值非法
   */
  void predict_model(double dt, const Eigen::VectorXd & process_noise_diagonal);

  /**
   * @brief 准备单个装甲板的测量数据
   *
   * 从装甲板提取世界坐标和世界偏航角，组成 [x, y, z, yaw]，并使用
   * TrackQueueV3 的默认测量协方差 R = 0.1I。
   * @param armor 当前检测到的单个装甲板
   * @throws std::invalid_argument 如果位置或偏航角包含非有限值
   */
  void prepare_measurement(const Armor & armor);

  /**
   * @brief 计算 [x, y, z, yaw] 观测量与当前后验状态的逐分量平方残差
   * @return [dx^2, dy^2, dz^2, dyaw^2]
   */
  Eigen::Vector4d posterior_residual_squared(
    const Eigen::Vector4d & observation_xyzyaw, int armor_id = 0) const override;

private:
  /**
   * @brief 状态加法，并对偏航角进行归一化
   */
  static Eigen::VectorXd state_add(
    const Eigen::VectorXd & state, const Eigen::VectorXd & delta);

  Eigen::Vector4d z_ = Eigen::Vector4d::Zero();
  Eigen::Matrix4d R_ = Eigen::Matrix4d::Identity() ;
  bool measurement_ready_ = false;
};

}  // namespace auto_aim

#endif  // AUTO_AIM__KF_EXAMPLE__CA_FROM_TJU_HPP
