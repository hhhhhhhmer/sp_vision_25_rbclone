// rv_from_fyt.hpp
#ifndef AUTO_AIM__KF_EXAMPLE__RV_FROM_FYT_HPP
#define AUTO_AIM__KF_EXAMPLE__RV_FROM_FYT_HPP

#include "./state2est.hpp"

namespace auto_aim
{
/**
 * @brief 基于"半径-速度"运动模型的扩展卡尔曼滤波器，用于装甲板状态估计。
 * 
 * 状态向量： [x, vx, y, vy, z, vz, yaw, vyaw, radius, radius_offset, height_offset]
 * 测量向量： [yaw, pitch, distance, armor_yaw]
 */
class RVfromFYT : public State2Est
{
public:
  static constexpr Eigen::Index kStateDimension = 11;   ///< 状态维度
  static constexpr double kTowerArmorHeightStep = 0.10; ///< 前哨站装甲板高低差

  /** 默认构造函数（未初始化） */
  RVfromFYT() = default;

  /**
   * @brief 带参构造函数
   * @param x0 初始状态向量 (11维)
   * @param P0 初始协方差矩阵 (11x11)
   * @param armor_num 装甲板数量 (例如 3 或 4)
   * @param armor_name 装甲类型 (如 outpost, sentry, etc.)
   * @throws std::invalid_argument 如果维度不匹配或 armor_num <= 0
   */
  RVfromFYT(
    const Eigen::VectorXd & x0, const Eigen::MatrixXd & P0, int armor_num,
    ArmorName armor_name);


  void kf_predict(double dt, const Eigen::VectorXd & u, const Eigen::VectorXd noises);

  void mpc_predict(double dt, 
      const Eigen::VectorXd & u, 
      const Eigen::VectorXd noises) {this->kf_predict(dt, u, noises); };

  /**
   * @brief 执行预测步骤（运动模型）
   * @param dt 时间步长 (秒)
   * @param acceleration 加速度向量 [ax, ay, az] (世界坐标系)
   * @param position_process_noise 位置过程噪声方差 (用于位置和速度)
   * @param yaw_process_noise 偏航过程噪声方差 (用于偏航和角速度)
   */
  void predict_model(
    double dt, const Eigen::Vector3d & acceleration, double position_process_noise,
    double yaw_process_noise);

  /**
   * @brief 准备测量数据，计算测量协方差并存储观测值
   * @param armor 当前检测到的装甲板数据 (包含世界坐标和ypr)
   * @param cam_is_short 是否为短距离相机模式
   * @param update_count 当前更新计数，用于相机切换后的协方差膨胀
   */
  void prepare_measurement(const Armor & armor, bool cam_is_short, int update_count);

  /**
   * @brief 基于马氏距离选择与当前观测最匹配的装甲板ID
   * @param last_id 上一帧选中的ID (-1 表示无上一帧)
   * @return 选中的装甲板ID (0 ~ armor_num_-1)
   * @throws std::logic_error 如果未调用 prepare_measurement
   */
  int select_armor_id(int last_id) const;

  /**
   * @brief 执行校正步骤（更新状态）
   * @param armor_id 选中的装甲板ID
   * @return 更新后的状态向量 (11维)
   * @throws std::logic_error 如果未调用 prepare_measurement
   */
  Eigen::VectorXd correct(int armor_id);

  /**
   * @brief 设置塔装甲的高度偏移 (用于outpost)
   * @param heights 长度为3的数组，每个元素为 pair<bool,double>，表示是否启用及高度值
   */
  void set_tower_armor_heights(const std::pair<bool, double> (&heights)[3]);

  /**
   * @brief 计算给定装甲板在世界坐标系中的3D位置
   * @param state 当前状态向量
   * @param armor_id 装甲板ID
   * @return 世界坐标 [x, y, z]
   */
  Eigen::Vector3d h_armor_xyz(const Eigen::VectorXd & state, int armor_id) const;

  /**
   * @brief 获取所有装甲板的世界坐标和偏航角列表
   * @return 向量，每个元素为 [x, y, z, yaw] (yaw已归一化到 [-pi, pi])
   */
  std::vector<Eigen::Vector4d> armor_xyza_list() const;

  /**
   * @brief 计算 [x, y, z, yaw] 观测量与指定装甲板后验估计量的逐分量平方残差
   * @param observation_xyzyaw 装甲板观测量 [x, y, z, yaw]
   * @param armor_id 装甲板 ID
   * @return [dx^2, dy^2, dz^2, dyaw^2]
   */
  Eigen::Vector4d posterior_residual_squared(
    const Eigen::Vector4d & observation_xyzyaw, int armor_id = 0) const override;

private:
  int armor_num_ = 0;                         ///< 装甲板数量
  ArmorName armor_name_ = ArmorName::not_armor; ///< 装甲类型
  std::array<double, 3> tower_armor_heights_{}; ///< 塔装甲高度偏移 (仅用于outpost)

  Eigen::Vector4d z_ = Eigen::Vector4d::Zero(); ///< 当前测量向量 [yaw, pitch, distance, armor_yaw]
  Eigen::Matrix4d R_ = Eigen::Matrix4d::Identity(); ///< 测量协方差矩阵
  bool measurement_ready_ = false; ///< 测量是否已准备

  bool last_cam_is_short_ = true; ///< 上一帧相机模式 (短/长)
  std::chrono::steady_clock::time_point camera_switch_time_{}; ///< 相机切换时间戳

  /**
   * @brief 观测模型函数 h(state, armor_id)
   * @param state 状态向量
   * @param armor_id 装甲ID
   * @return 观测向量 [yaw, pitch, distance, armor_yaw]
   */
  Eigen::Vector4d h(const Eigen::VectorXd & state, int armor_id) const;

  /**
   * @brief 观测模型的雅可比矩阵 ∂h/∂state
   * @param state 状态向量
   * @param armor_id 装甲ID
   * @return 4x11 雅可比矩阵
   */
  Eigen::Matrix<double, 4, kStateDimension> h_jacobian(
    const Eigen::VectorXd & state, int armor_id) const;

  /**
   * @brief 计算塔装甲高度乘数 (仅用于outpost)
   * @param armor_id 装甲ID
   * @return 高度乘数 (通常为 -2, -1, 0, 1, 2)
   */
  double tower_height_multiplier(int armor_id) const;

  /**
   * @brief 状态加法（重载）: state + delta，并对偏航角归一化
   */
  static Eigen::VectorXd state_add(
    const Eigen::VectorXd & state, const Eigen::VectorXd & delta);

  /**
   * @brief 观测残差计算: observation - prediction，并对角度残差归一化
   */
  static Eigen::VectorXd observation_subtract(
    const Eigen::VectorXd & observation, const Eigen::VectorXd & prediction);
};

}  // namespace auto_aim

#endif  // AUTO_AIM__KF_EXAMPLE__RV_FROM_FYT_HPP
