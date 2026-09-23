#ifndef AUTO_AIM__TARGET_HPP
#define AUTO_AIM__TARGET_HPP

#include <Eigen/Dense>
#include <chrono>
#include <list>
#include <optional>
#include <queue>
#include <string>
#include <vector>

#include "../model/armor.hpp"
#include "rv_from_fyt.hpp"
#include "tools/fft.hpp"

namespace auto_aim
{

class Target
{
public:
  ArmorName name;
  ArmorType armor_type;
  ArmorPriority priority;
  bool jumped;
  int last_id;  // debug only
  Eigen::Vector3d xyz_in_world;
  // rvFromFYT 残差平方和，x y z yaw
  std::optional<Eigen::Vector4d> rv_residual = std::nullopt;

  /** @brief 构造空目标 */
  Target();
  /** @brief 由首次装甲板观测初始化目标 @param armor 首次观测装甲板 @param t 观测时间 @param radius 初始旋转半径 @param armor_num 装甲板数量 @param P0_dig 初始协方差对角元素 */
  Target(
    const Armor & armor, std::chrono::steady_clock::time_point t, double radius, int armor_num,
    Eigen::VectorXd P0_dig);
  /** @brief 构造用于测试的简化目标 @param x 初始 X 坐标 @param vyaw 初始偏航角速度 @param radius 旋转半径 @param h 目标高度 */
  Target(double x, double vyaw, double radius, double h);

  /** @brief 将目标预测到指定时间 @param t 目标时间 */
  void predict(std::chrono::steady_clock::time_point t);
  /** @brief 按时间步长预测目标 @param dt 时间步长，单位 s @param u_xyz 三轴控制输入 */
  void predict(double dt, Eigen::VectorXd u_xyz = Eigen::VectorXd::Zero(3));
  /** @brief 使用控制输入将目标预测到指定时间 @param t 目标时间 @param u_xyz 三轴控制输入 */
  void predict(std::chrono::steady_clock::time_point t, Eigen::VectorXd u_xyz);
  /** @brief 使用装甲板观测更新目标滤波器 @param armor 新观测装甲板 @return 完成 EKF 校正时返回 true；观测无法匹配到装甲板时返回 false，此时滤波器状态保持不变 */
  bool update(const Armor & armor);

  /** @brief 使用装甲板观测更新目标滤波器，并可按马氏距离门限拒绝异常观测 @param armor 新观测装甲板 @param max_mahalanobis 最大允许马氏距离；<= 0 表示不做门限 @return 完成 EKF 校正时返回 true；被门限拒绝或无法匹配时返回 false，滤波器状态保持不变 */
  bool update(const Armor & armor, double max_mahalanobis);

  /** @brief 使用装甲板观测更新目标滤波器，可按马氏距离门限与已用ID拒绝异常观测 @param armor 新观测装甲板 @param max_mahalanobis 最大允许马氏距离；<= 0 表示不做门限 @param exclude_ids 本帧已被其它装甲板占用的ID @param angle_sigma_scale 装甲板朝向观测噪声放大系数，用于给不可靠的朝向观测降权 @return 完成 EKF 校正时返回 true；被拒绝或无法匹配时返回 false，滤波器状态保持不变 */
  bool update(
    const Armor & armor, double max_mahalanobis, const std::vector<int> & exclude_ids,
    double angle_sigma_scale = 1.0);

  /**
   * @brief 用一帧内的多块装甲板做联合更新
   * @param armors 本帧候选装甲板列表（按离图像中心由近到远排序）
   * @param max_mahalanobis 第二块及之后装甲板的马氏距离门限；<= 0 表示不限制
   * @param max_armors 本帧最多融合的装甲板数量
   * @return 至少完成一次校正时返回 true
   * @note UV 模式下把所有关联成功的装甲板拼成一次批量观测（联合更新）；
   *       非 UV 模式下退化为逐块顺序更新，行为与旧版本一致
   */
  bool update_batch(const std::list<Armor> & armors, double max_mahalanobis, int max_armors = 99);

  /** @brief 最近一次批量更新实际使用的观测条数 @return 观测条数 */
  int last_batch_observation_count() const { return last_batch_count_; }

  /** @brief 配置 UV 观测模式 @param config UV 配置 */
  void set_uv_config(const UvConfig & config);
  /** @brief 更新相机几何（UV 观测使用） @param geometry 相机内外参与云台到世界旋转 */
  void set_camera_geometry(const CameraGeometry & geometry);
  /** @brief UV 观测模式是否可用 @return 可用时返回 true */
  bool uv_ready() const;

  /** @brief 获取滤波状态副本 @return EKF 状态向量 */
  Eigen::VectorXd ekf_x() const;
  /** @brief 获取滤波器只读引用 @return RV 扩展卡尔曼滤波器 */
  const RVfromFYT & ekf() const;
  /** @brief 计算目标全部装甲板的坐标与偏航角 @return xyza 列表 */
  std::vector<Eigen::Vector4d> armor_xyza_list() const;
  /** @brief 获取最近装甲板的位置、偏航和距离 @return xyzad 向量
   *  @note 无状态版本：每帧按"最近板 + last_id 门限"重新决断，且不做跨帧记忆。
   *        会逐帧抖动，新代码请用 get_aim_armor_xyzad()。 */
  Eigen::Matrix<double, 5, 1> get_recent_armor_xyzad() const;

  /**
   * @brief 选择当前该瞄的装甲板编号（带滞环、帧间记忆）
   *
   * 解决"布尔门限逐帧重选"导致的瞄点抖动：门限从"硬阈值开关"改为
   * "保持当前板，除非它明显更差"。
   *  1. 上一帧选中的板若仍可用（朝向角在 kAimHoldAngle 内），优先保持；
   *     —— 不再用 60° 这种硬阈值来回切，只有在板确实转过头时才放手；
   *  2. 否则优先跟踪器最近观测到的板（last_id），但它也必须"可用"，避免瞄到背面板；
   *  3. 都不满足才回退到"离相机最近"的板；
   *  4. 跨帧滞环：新板必须比当前板近 kAimHysteresisDist 以上才允许换板；
   *  5. 同一帧内（update_count_ 未变）锁死已选编号——预测时域里 100 多次采样
   *     因此始终瞄同一块板，参考轨迹不会再在时域中途换板。
   *
   * @param out_nearest 可选输出：本帧"离相机最近"的板编号（诊断/对照组用）
   * @return 选中的装甲板编号；无装甲板时返回 -1
   */
  int select_aim_armor(int * out_nearest = nullptr) const;

  /**
   * @brief 获取"当前该瞄的装甲板"的位置、偏航与距离（带滞环、帧内锁定）
   * @return xyzad 向量（x, y, z, armor_yaw, 距离）
   * @note 与 get_recent_armor_xyzad() 的唯一区别就是选板规则见 select_aim_armor()。
   *       规划器、开火判据必须用同一个本函数，否则"发的"和"判的"不是同一块板。
   */
  Eigen::Matrix<double, 5, 1> get_aim_armor_xyzad();

  /** @brief 设置观测噪声标准差，覆盖 EKF 内部启发式 @param azimuth_sigma 方位角噪声标准差 (rad) @param distance_sigma 距离噪声标准差 (m) @param angle_sigma 朝向噪声标准差 (rad) */
  void set_measurement_sigmas(double azimuth_sigma, double distance_sigma, double angle_sigma)
  {
    ekf_.set_measurement_sigmas(azimuth_sigma, distance_sigma, angle_sigma);
  }

  /** @brief 根据状态计算指定编号装甲板的位置 @param x 目标状态 @param id 装甲板编号 @return 装甲板世界坐标 */
  Eigen::Vector3d h_armor_xyz(const Eigen::VectorXd & x, int id) const;

  /** @brief 判断滤波器是否发散 @return 发散时返回 true */
  bool diverged() const;

  /** @brief 更新并查询滤波器收敛状态 @return 收敛时返回 true */
  bool convergened();

  bool isinit = false;

  /** @brief 检查目标是否完成初始化 @return 已初始化时返回 true */
  bool checkinit();

  /** @brief 获取 EKF 状态估计 @return 状态向量副本 */
  inline Eigen::VectorXd getEKFXest() {
    return ekf_.x;
  }

  /** @brief 获取目标当前滤波时间 @return 时间戳 */
  inline std::chrono::steady_clock::time_point getTimePoint() {
    return t_;
  }

  //前哨站
  std::pair<bool, double> tower_armor_hs[3] = {std::pair<bool, double>(false, 0), std::pair<bool, double>(false, 0), std::pair<bool, double>(false, 0)};
  // double tower_armor_hs[3] = {0,0,0};  
  double tower_armor_h = 0.0;
  double tower_armor_hs_datas[3] = {0,0,0}; 
  double last_tower_armor_h[3] = {0,0,0};
  int tower_armor_hs_datas_ptr[3] = {0, 0, 0};

  //长短焦
  bool cam_is_short = true;
  
  /**
   * @brief 前哨站高度锚点的最短采样帧数
   *
   * 锚点由 h_armor_xyz 的互补滤波结果累加平均得到，而互补滤波从 0 起步（系数 0.1），
   * 只有 1~2 帧样本时平均值 ≈ 0.1×真实高度。这样的锚点会让高度阶梯方向翻转
   * （本来高 0.1m 的板被判成低 0.1m），瞄准点随之偏 0.2~0.4m。
   */
  static constexpr int kTowerArmorMinAnchorSamples = 5;

  /**
   * @brief 发生过 EKF 校正的帧数
   *
   * 语义是"帧数"而非"校正次数"：多装甲融合会在同一帧内多次校正，这里只按帧自增一次。
   * 下游（convergened()、planner 开火门槛、binocular 等待帧数）的阈值都是按每帧一次调的。
   */
  int update_count_;
  std::optional<tools::Wave> wave_;

private:
  int armor_num_;
  int switch_count_;
  

  bool is_switch_, is_converged_;

  RVfromFYT ekf_;
  State2Est* est = &ekf_;

  std::chrono::steady_clock::time_point t_;

  /** @brief 将前哨站装甲板高度观测同步到目标模型 */
  void sync_tower_armor_heights();

  /**
   * @brief 校正后的统一记账（换板统计、前哨站高度累加、update_count、last_id 等）
   * @param id 本次校正使用的装甲板编号
   * @param primary 是否为主观测（决定 last_id 与 xyz_in_world）
   * @param armor_xyz 该装甲板的世界坐标（UV 模式下用校正后的预测值，传统模式用 PnP 观测值）
   */
  void bookkeep_after_update(int id, bool primary, const Eigen::Vector3d & armor_xyz);

  /** @brief 最近一次批量更新的观测条数 */
  int last_batch_count_ = 0;

  /** @brief 已计入 update_count_ 的帧时间戳（同一帧内多次校正只计一次） */
  std::chrono::steady_clock::time_point last_counted_frame_{};

  // ---------------------------------------------------------------- 瞄板选择（带滞环）
  /** @brief 保持当前板的朝向角上限：超过它才允许换板（比旧的 60° 硬门限宽，避免来回切） */
  static constexpr double kAimHoldAngle = 100.0 / 57.3;
  /** @brief 换板滞环：新板必须比当前板近这么多米才换（避免两块板等距时逐帧翻） */
  static constexpr double kAimHysteresisDist = 0.03;
  /** @brief 当前锁定的瞄板编号，-1 表示尚未选择 */
  int aim_armor_id_ = -1;
  /** @brief 上一次执行瞄板选择的帧号（= 当时的 update_count_）；同帧内不重选 */
  int aim_select_frame_ = -1;
};

}  // namespace auto_aim

#endif  // AUTO_AIM__TARGET_HPP
