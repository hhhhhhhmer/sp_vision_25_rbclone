#ifndef AUTO_AIM__UV_MODEL_HPP
#define AUTO_AIM__UV_MODEL_HPP

#include <Eigen/Dense>
#include <array>
#include <opencv2/core.hpp>
#include <vector>

#include "../model/armor.hpp"
#include "../model/camera_geometry.hpp"
#include "tower_armor_geometry.hpp"

namespace auto_aim
{
/** @brief UV（像素/UVL）观测模型配置 */
struct UvConfig
{
  /** @brief 是否启用 UV 观测（关闭时沿用世界系 ypd 观测） */
  bool enabled = false;
  /** @brief 灯条端点检测噪声标准差 (px) */
  double sigma_px = 0.3;
  /** @brief 与灯条像素长度成比例的附加端点噪声（尺度不变项） */
  double sigma_len_ratio = 0.01;
  /** @brief 云台/外参姿态误差等效标准差 (deg)，换算成像素后加到灯条中心噪声上 */
  double sigma_attitude_deg = 0.5;
  /**
   * @brief 姿态误差中"整帧共模"部分的比例（0~1）
   *
   * 姿态误差本质上是一帧内所有特征共同平移，物理上应以 rank-3 相关项表达；
   * 但完全共模会让滤波器无法利用"共同平移"里的目标方位信息（方位只能靠时间连续性估计），
   * 而完全独立又会过度自信。这里把它拆成两部分：
   *   σ_com = ratio · σ_att      -> rank-3 相关项 J J^T
   *   σ_ind = sqrt(1-ratio²) · σ_att -> 各灯条独立对角项
   * ratio=0 等价于把姿态误差当作独立噪声（旧行为），ratio=1 为完全共模。
   */
  double attitude_common_ratio = 0.0;
  /** @brief 灯条角度噪声下限 (rad) */
  double sigma_angle = 0.03;
  /** @brief 灯条中心像素距离关联门限 (px) */
  double associate_gate_px = 40.0;
  /** @brief 可见性判据：装甲板法线与视线夹角的余弦下限 */
  double facing_cos_min = 0.15;
  /** @brief 单装甲退化保护：半径初始协方差标准差 (m)，<=0 表示沿用原值 */
  double radius_prior_sigma = 0.03;
  /**
   * @brief 深度方向过程噪声缩放系数（仅 UV 模式生效）
   *
   * 原实现三个平移轴共用同一位移过程噪声（1e4 量级），对"深度"这一最不可观测的方向过于宽松：
   * 传统观测里距离噪声本就很大（σ≈1m）所以看不出来，UV 观测把尺度信息真正用起来后，
   * 过大的深度过程噪声会让深度跟着长度测量抖动。这里给 z 轴单独缩放。
   */
  double process_noise_z_scale = 1.0;
};

/**
 * @brief UVL 观测模型：把 EKF 状态预测的装甲板投影成两条灯条的 UVL 特征
 *
 * 观测向量（8 维）= [左灯条(角度, 中心x, 中心y, 长度), 右灯条(角度, 中心x, 中心y, 长度)]
 * - 角度 = atan2(dx, -dy)，竖直灯条为 0，绕过了 atan2(dx,dy) 在 ±π 处的回绕
 * - 中心 = (上端点 + 下端点) / 2，单位 px
 * - 长度 = |上端点 - 下端点|，单位 px
 *
 * 该参数化的好处：
 * 1) 长度是差分量，对云台姿态误差（共模平移）不敏感，是深度信息的主要来源；
 * 2) 中心像素噪声可以按姿态误差给大值，长度噪声按检测噪声给小值，各向异性天然表达；
 * 3) 装甲板 4 角点是对称矩形，法线翻转 180° 时投影点集合完全不变，
 *    因此 UV 观测天然没有 PnP 的镜像解歧义。
 */
class UvModel
{
public:
  /** @brief UV 观测维度（每条灯条 4 维 × 2 条灯条） */
  static constexpr int kDim = 8;
  using Observation = Eigen::Matrix<double, kDim, 1>;
  using Jacobian = Eigen::Matrix<double, kDim, 11>;
  using Noise = Eigen::Matrix<double, kDim, kDim>;

  /** @brief 配置观测模型 @param config UV 配置 @param armor_num 装甲板数量 @param armor_name 装甲板名称（决定倾角） */
  void configure(const UvConfig & config, int armor_num, ArmorName armor_name);

  /** @brief 更新相机几何 @param geometry 相机内外参与云台到世界旋转 */
  void set_camera_geometry(const CameraGeometry & geometry) { geometry_ = geometry; }

  /** @brief 更新前哨站各装甲板高度锚点 @param heights 3 个装甲板的高度（是否有效 + 高度） */
  void set_tower_armor_heights(const std::array<std::pair<bool, double>, 3> & heights)
  {
    tower_heights_ = heights;
  }

  /** @brief 相机几何是否可用 @return 可用时返回 true */
  bool ready() const { return geometry_.valid; }

  /** @brief 获取配置 @return UV 配置 */
  const UvConfig & config() const { return config_; }

  // ---------------------------------------------------------------- 观测（像素 -> UVL）

  /**
   * @brief 由 4 个角点构造 UVL 观测
   * @param points 角点，顺序与 Armor::points 一致：[左上, 右上, 右下, 左下]
   * @return 8 维 UVL 观测；输入不合法时返回全零
   */
  static Observation observation(const std::vector<cv::Point2f> & points);

  /** @brief 判断角点是否可用于构造观测 @param points 角点 @return 可用时返回 true */
  static bool observation_valid(const std::vector<cv::Point2f> & points);

  /**
   * @brief 由灯条上下端点构造 UVL 特征
   * @param top 上端点 @param bottom 下端点 @param out 输出的 4 维特征
   * @note 全程使用 double，避免 float 量化破坏雅可比的数值验证
   */
  static void lightbar_to_uvl(
    const Eigen::Vector2d & top, const Eigen::Vector2d & bottom, Eigen::Matrix<double, 4, 1> & out);

  // ---------------------------------------------------------------- 预测（状态 -> UVL）

  /**
   * @brief 预测指定装甲板的 UVL 观测
   * @param state 11 维 EKF 状态 @param armor_id 装甲板编号 @param type 装甲板尺寸类型
   * @return 8 维 UVL；投影失败（深度为负等）时返回全零
   */
  Observation predict(const Eigen::VectorXd & state, int armor_id, ArmorType type) const;

  /**
   * @brief 预测观测对状态的解析雅可比
   * @param state 11 维 EKF 状态 @param armor_id 装甲板编号 @param type 装甲板尺寸类型
   * @return 8x11 雅可比；投影失败时返回全零
   */
  Jacobian jacobian(const Eigen::VectorXd & state, int armor_id, ArmorType type) const;

  /**
   * @brief 仅检测噪声的对角协方差（不含云台姿态误差）
   * @param z 预测观测（用其像素长度确定尺度相关噪声）
   * @return 8x8 对角噪声矩阵
   */
  Noise diagonal_noise(const Observation & z) const;

  /**
   * @brief 云台/外参姿态误差对观测的雅可比（8x3）
   * @param state 状态 @param armor_id 装甲板编号 @param type 尺寸类型
   * @return 每条灯条对 3 个姿态误差分量的偏导
   * @note 姿态误差是整帧共模的，必须用 rank-3 相关项而不是对角项表达，
   *       否则多灯条/多装甲时会错误地"平均掉"姿态误差，导致滤波器过度自信
   */
  Eigen::Matrix<double, kDim, 3> attitude_jacobian(
    const Eigen::VectorXd & state, int armor_id, ArmorType type) const;

  /**
   * @brief 单条观测的测量噪声（对角检测噪声 + 自身姿态误差相关项）
   * @param state 状态 @param armor_id 装甲板编号 @param type 尺寸类型 @param z 预测观测
   * @return 8x8 噪声矩阵
   */
  Noise noise(
    const Eigen::VectorXd & state, int armor_id, ArmorType type, const Observation & z) const;

  /**
   * @brief 姿态误差的等效像素标准差（= 焦距 × 姿态标准差），用于把姿态误差折算到对角噪声
   * @return 像素标准差
   */
  double attitude_sigma_px() const;

  /** @brief 姿态误差标准差（弧度） @return 弧度 */
  double attitude_sigma_rad() const
  {
    return config_.sigma_attitude_deg * 3.14159265358979323846 / 180.0;
  }

  /** @brief 计算某个装甲板朝向观测者的余弦（可见性判据） @param state 状态 @param armor_id 装甲板编号 @return 余弦值，>0 表示朝向相机 */
  double facing_cos(const Eigen::VectorXd & state, int armor_id) const;

  // ---------------------------------------------------------------- 几何辅助

  /** @brief 状态中的装甲板偏航角 @param state 状态 @param armor_id 装甲板编号 @return 偏航角 (rad) */
  double armor_yaw(const Eigen::VectorXd & state, int armor_id) const;

  /** @brief 状态中的装甲板中心世界坐标 @param state 状态 @param armor_id 装甲板编号 @return 世界坐标 */
  Eigen::Vector3d armor_position(const Eigen::VectorXd & state, int armor_id) const;

  /** @brief 状态中的装甲板到世界旋转 @param state 状态 @param armor_id 装甲板编号 @return 旋转矩阵 */
  Eigen::Matrix3d armor_rotation(const Eigen::VectorXd & state, int armor_id) const;

  /** @brief 装甲板 4 角点在世界系中的坐标 @param state 状态 @param armor_id 装甲板编号 @param type 尺寸类型 @return 4 个世界点 */
  std::array<Eigen::Vector3d, 4> armor_corners_world(
    const Eigen::VectorXd & state, int armor_id, ArmorType type) const;

  /** @brief 把状态预测的装甲板角点投影到像素 @param state 状态 @param armor_id 装甲板编号 @param type 尺寸类型 @return 4 个像素点（顺序同 Armor::points）；失败时为空 */
  std::vector<cv::Point2f> project_corners(
    const Eigen::VectorXd & state, int armor_id, ArmorType type) const;

  /** @brief 装甲板尺寸（宽度） @param type 尺寸类型 @return 宽度 (m) */
  static double armor_width(ArmorType type);
  /** @brief 灯条长度 @return 长度 (m) */
  static double lightbar_length() { return kLightbarLength; }

  /** @brief 装甲板倾角 @return 弧度（前哨站为 -15°，其余 +15°） */
  double armor_pitch() const;

  static constexpr double kSmallArmorWidth = 0.135;
  static constexpr double kBigArmorWidth = 0.230;
  static constexpr double kLightbarLength = 0.056;
  /** @brief 最小有效深度 (m)，低于该值认为投影无效 */
  static constexpr double kMinDepth = 0.2;

private:
  /** @brief 相机系点投影到像素，并给出对相机系点的 2x3 雅可比 @return 投影有效时返回 true */
  bool project_point(
    const Eigen::Vector3d & point_camera, Eigen::Vector2d & uv,
    Eigen::Matrix<double, 2, 3> & jacobian) const;

  /** @brief 世界点转相机系 @param point_world 世界点 @return 相机系点 */
  Eigen::Vector3d world_to_camera(const Eigen::Vector3d & point_world) const;

  /** @brief 在给定云台到世界旋转下预测 UVL 观测 @param state 状态 @param armor_id 编号 @param type 尺寸类型 @param R_gimbal2world 云台到世界旋转 @return 8 维 UVL */
  Observation predict_with_rotation(
    const Eigen::VectorXd & state, int armor_id, ArmorType type,
    const Eigen::Matrix3d & R_gimbal2world) const;

  /** @brief 装甲板角点在装甲板坐标系中的坐标 @param type 尺寸类型 @return 4 个点（顺序同 Armor::points） */
  static std::array<Eigen::Vector3d, 4> armor_corner_local(ArmorType type);

  /** @brief 世界点对状态的雅可比 @param state 状态 @param armor_id 装甲板编号 @param local 装甲板系点 @param type 尺寸类型 @return 3x11 雅可比 */
  Eigen::Matrix<double, 3, 11> corner_world_jacobian(
    const Eigen::VectorXd & state, int armor_id, const Eigen::Vector3d & local,
    ArmorType type) const;

  /** @brief 前哨站高度乘数（与 RVfromFYT::h_armor_xyz 共用 tower_armor_height_multiplier） @param armor_id 装甲板编号 @return 高度乘数 */
  double tower_height_multiplier(int armor_id) const;

  UvConfig config_{};
  CameraGeometry geometry_{};
  int armor_num_ = 4;
  ArmorName armor_name_ = ArmorName::not_armor;
  std::array<std::pair<bool, double>, 3> tower_heights_{};
};
}  // namespace auto_aim

#endif  // AUTO_AIM__UV_MODEL_HPP
