// rv_from_fyt.hpp
#ifndef AUTO_AIM__KF_EXAMPLE__RV_FROM_FYT_HPP
#define AUTO_AIM__KF_EXAMPLE__RV_FROM_FYT_HPP

#include <array>
#include <utility>
#include <vector>

#include "../model/camera_geometry.hpp"
#include "state2est.hpp"
#include "uv_model.hpp"

namespace auto_aim
{
/**
 * @brief 基于"半径-速度"运动模型的扩展卡尔曼滤波器，用于装甲板状态估计。
 *
 * 状态向量： [x, vx, y, vy, z, vz, yaw, vyaw, radius, radius_offset, height_offset]
 * 测量向量：
 *  - 传统模式：[yaw, pitch, distance, armor_yaw]（世界系，需要逐帧 PnP）
 *  - UV 模式：每条可见装甲板 8 维 UVL（像素系，直接由检测角点构造，无需 PnP）
 */
class RVfromFYT : public State2Est
{
public:
  static constexpr Eigen::Index kStateDimension = 11;   ///< 状态维度
  static constexpr double kTowerArmorHeightStep = 0.10; ///< 前哨站装甲板高低差

  /** @brief 默认构造未初始化的估计器 */
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

  /** @brief 按半径-速度模型执行 EKF 预测 @param dt 时间步长，单位 s @param u 三轴加速度输入 @param noises 过程噪声参数 */
  void kf_predict(double dt, const Eigen::VectorXd & u, const Eigen::VectorXd noises);

  // ---------------------------------------------------------------- UV 观测模式

  /** @brief 启用/配置 UV 观测模式 @param config UV 配置 */
  void enable_uv(const UvConfig & config);

  /** @brief UV 观测模式是否启用且相机几何可用 @return 可用时返回 true */
  bool uv_ready() const { return uv_config_.enabled && uv_model_.ready(); }

  /** @brief 更新相机几何 @param geometry 相机内外参与云台到世界旋转 */
  void set_camera_geometry(const CameraGeometry & geometry);

  /**
   * @brief 关联一个观测装甲板：在候选编号中找 UVL 残差最小且通过门限的编号
   * @param armor 观测装甲板（使用其 points 角点）
   * @param exclude_ids 本帧已被占用的编号
   * @param max_mahalanobis 最大允许马氏距离；<=0 表示不做门限
   * @return 匹配的装甲板编号；无可用编号时返回 -1
   * @note 先用配置里的灯条中心像素距离做粗门限，再用 UVL 马氏距离做精门限
   */
  int associate_uv_armor(
    const Armor & armor, const std::vector<int> & exclude_ids,
    double max_mahalanobis) const;

  /** @brief 清空本帧待更新的 UV 观测 */
  void clear_uv_observations() { uv_pending_.clear(); }

  /**
   * @brief 追加一条已关联编号的 UV 观测
   * @param armor_id 装甲板编号 @param armor 观测装甲板
   * @return 观测有效时返回 true
   */
  bool add_uv_observation(int armor_id, const Armor & armor);

  /** @brief 本帧待更新的 UV 观测量 @return 观测条数 */
  std::size_t uv_observation_count() const { return uv_pending_.size(); }

  /**
   * @brief 用本帧所有待更新 UV 观测做一次联合更新（等价于批量观测的 EKF）
   * @return 实际执行的观测条数；失败返回 0
   */
  std::size_t correct_uv_batch();

  /** @brief 预测某条候选编号的 UVL 观测（调试/关联用） @param armor_id 编号 @param type 尺寸类型 @return 8 维 UVL */
  UvModel::Observation uv_predict(int armor_id, ArmorType type) const
  {
    return uv_model_.predict(x, armor_id, type);
  }

  /** @brief 预测某条候选编号的可见性余弦 @param armor_id 编号 @return 余弦值 */
  double uv_facing_cos(int armor_id) const { return uv_model_.facing_cos(x, armor_id); }

  /** @brief 获取 UV 模型只读引用 @return UV 模型 */
  const UvModel & uv_model() const { return uv_model_; }

  /** @brief 执行 MPC 兼容预测 @param dt 时间步长，单位 s @param u 三轴加速度输入 @param noises 过程噪声参数 */
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
   * @param angle_sigma_scale 装甲板朝向观测噪声放大系数，用于给不可靠的朝向观测降权
   */
  void prepare_measurement(
    const Armor & armor, bool cam_is_short, int update_count, double angle_sigma_scale = 1.0);

  /**
   * @brief 基于马氏距离选择与当前观测最匹配的装甲板ID
   * @param last_id 上一帧选中的ID (-1 表示无上一帧)
   * @return 选中的装甲板ID (0 ~ armor_num_-1)
   * @throws std::logic_error 如果未调用 prepare_measurement
   */
  int select_armor_id(int last_id) const;

  /**
   * @brief 基于马氏距离选择最匹配装甲板ID，并可按门限拒绝异常观测
   * @param last_id 上一帧选中的ID (-1 表示无上一帧)
   * @param max_mahalanobis 最大允许马氏距离；<= 0 表示不做门限
   * @param exclude_ids 本帧已被其它装甲板占用的ID，这些ID不参与匹配；非空时同时关闭滞后机制
   * @return 选中的装甲板ID (0 ~ armor_num_-1)；超过门限或无可选ID时返回 -1
   * @throws std::logic_error 如果未调用 prepare_measurement
   * @note 供多装甲板融合使用：第二块及之后的装甲板必须通过门限且不能与已用ID重复，
   *       避免把另一台车的同名装甲板、或同一块装甲板重复融合进当前目标
   */
  int select_armor_id(
    int last_id, double max_mahalanobis, const std::vector<int> & exclude_ids) const;

  /**
   * @brief 获取最近一次 select_armor_id 计算出的最小马氏距离
   * @return 最小马氏距离；未计算时返回 -1
   */
  double last_mahalanobis() const { return last_mahalanobis_; }

  /**
   * @brief 执行校正步骤（更新状态）
   * @param armor_id 选中的装甲板ID
   * @return 更新后的状态向量 (11维)
   * @throws std::logic_error 如果未调用 prepare_measurement
   */
  Eigen::VectorXd correct(int armor_id);

  /**
   * @brief 设置观测噪声标准差，覆盖内部启发式
   * @param azimuth_sigma 方位角噪声标准差 (rad)；<= 0 表示沿用原启发式
   * @param distance_sigma 距离噪声标准差 (m)；<= 0 表示沿用原启发式
   * @param angle_sigma 装甲板朝向噪声标准差 (rad)；<= 0 表示沿用原启发式
   */
  void set_measurement_sigmas(double azimuth_sigma, double distance_sigma, double angle_sigma)
  {
    azimuth_sigma_ = azimuth_sigma;
    distance_sigma_ = distance_sigma;
    angle_sigma_ = angle_sigma;
  }

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
  std::array<std::pair<bool, double>, 3> tower_armor_heights_{}; ///< 塔装甲高度锚点（是否有效 + 高度，仅用于 outpost）
  mutable double last_mahalanobis_ = -1.0;      ///< 最近一次匹配的最小马氏距离
  UvConfig uv_config_{};                        ///< UV 观测配置
  UvModel uv_model_{};                          ///< UV 观测模型

  /** @brief 本帧待更新的 UV 观测 */
  struct UvPending
  {
    int armor_id = -1;
    ArmorType type = ArmorType::small;
    UvModel::Observation z = UvModel::Observation::Zero();
    UvModel::Noise R = UvModel::Noise::Zero();
    Eigen::Matrix<double, UvModel::kDim, 3> J_att =
      Eigen::Matrix<double, UvModel::kDim, 3>::Zero();
  };
  std::vector<UvPending> uv_pending_;           ///< 本帧待更新的 UV 观测列表

  /** @brief 构造批量观测的残差函数 @return 残差（角度分量做 ±pi 归一化） */
  static Eigen::VectorXd uv_observation_subtract(
    const Eigen::VectorXd & observation, const Eigen::VectorXd & prediction);
  double azimuth_sigma_ = -1.0;                 ///< 方位角噪声标准差 (rad)，<=0 用启发式
  double distance_sigma_ = -1.0;                ///< 距离噪声标准差 (m)，<=0 用启发式
  double angle_sigma_ = -1.0;                   ///< 朝向噪声标准差 (rad)，<=0 用启发式

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
   * @param state 原状态向量
   * @param delta 状态增量
   * @return 相加并归一化后的状态
   */
  static Eigen::VectorXd state_add(
    const Eigen::VectorXd & state, const Eigen::VectorXd & delta);

  /**
   * @brief 观测残差计算: observation - prediction，并对角度残差归一化
   * @param observation 实际观测
   * @param prediction 预测观测
   * @return 归一化后的观测残差
   */
  static Eigen::VectorXd observation_subtract(
    const Eigen::VectorXd & observation, const Eigen::VectorXd & prediction);
};

}  // namespace auto_aim

#endif  // AUTO_AIM__KF_EXAMPLE__RV_FROM_FYT_HPP
