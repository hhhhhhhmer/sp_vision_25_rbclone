#pragma once

#include <array>
#include <vector>

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <opencv2/core.hpp>

#include "runtime_types.hpp"

/**
 * @brief 符平面的 UV 观测配置
 *
 * 与装甲板的 UV 观测思路一致：正常情况下逐帧不再做 PnP，而是把检测到的五点
 * 关键点和整片符叶锚框点直接作为像素观测，在 EKF 里维护符平面位姿。
 * PnP 只在建立/重建目标、以及 UV 更新失败的兜底帧使用。
 */
struct RuneUvPoseConfig
{
  bool enabled = true;
  /** @brief 关键点检测噪声标准差 (px) */
  double sigma_px = 3.0;
  /** @brief 中心 R 点的检测噪声标准差 (px) */
  double sigma_center_px = 1.5;
  /** @brief 整片符叶锚框（轮廓 PCA 框）的检测噪声标准差 (px) */
  double anchor_sigma_px = 4.0;
  /** @brief 单帧单符叶观测的卡方门限（18 维，按实际噪声协方差计算） */
  double gate_chi2 = 200.0;
  /** @brief 目标切换搜索：当前预测 RMSE 超过该值时尝试 72° 整数倍旋转假设 (px) */
  double switch_rmse_px = 10.0;
  /** @brief 初始化旋转标准差 (deg)，PnP 先验通常比这更准 */
  double initial_rotation_sigma_deg = 5.0;
  /** @brief 初始化平移标准差 (m) */
  double initial_translation_sigma_m = 0.15;
  /** @brief 每帧旋转过程噪声标准差 (deg)，用于补偿标定/模型误差 */
  double process_rotation_sigma_deg = 0.2;
  /** @brief 每帧平移过程噪声标准差 (m)，用于补偿机器人缓慢移动 */
  double process_translation_sigma_m = 0.015;
  /** @brief 角速度初始标准差 (rad/s)；方向未知时给足够大的先验 */
  double initial_angular_velocity_sigma = 2.0;
  /** @brief 角速度过程噪声标准差 (rad/s)，大符加减速时靠它跟踪 */
  double process_angular_velocity_sigma = 1.0;
  /** @brief 两帧间隔超过该值时不信任预测，直接重建 (s) */
  double max_prediction_gap_s = 0.25;
  /** @brief 靶心环半径 (m) */
  double circle_radius = 0.15;
  /** @brief 靶心环中心到符心的距离 (m) */
  double circle_center_radius = 0.7;
};

/**
 * @brief 能量机关符平面的 UV 位姿滤波器
 *
 * 状态为"当前 1 号位符叶"在 RP 固定系下的位姿（旋转 + 平移）、以及绕符平面
 * 法向的角速度，共 7 维。误差定义：
 *   R_true = R_est * Exp(dtheta), c_true = c_est + dc, w_true = w_est + dw
 * R 的 z 轴是符平面法向，R 的局部 x/y 轴就是符叶坐标系，因此 R 会随符旋转。
 * 这里的"车系"沿用 RP 代码的命名，实际是 world_to_rp 后的固定坐标系；相机/云台
 * 姿态只用于把观测投到像素，不参与状态传播（否则会把云台旋转重复算一次）。
 * 观测为每个未激活符叶的 5 个网络关键点和 4 个轮廓锚框点：
 *   [外侧点, 左侧点, 中心R点, 右侧点, 内侧点, 外左, 内左, 内右, 外右]
 * 四个关键点固定在靶心圆上、随符叶位置旋转；锚框点是整片符叶的 PCA 外接框。
 * 中心 R 点对应符平面原点。没有 PnP 参与逐帧更新。
 */
class RuneUvPoseFilter
{
public:
  /** @brief 单个未激活符叶的观测（顺序与网络输出一致） */
  struct Observation
  {
    int location_num = 1;
    std::array<cv::Point2f, 5> keypoints{};  // [top, left, center, right, bottom]
    /** @brief 整片符叶锚框四点，顺序同 anchor_inactive_points_world:[外左,内左,内右,外右] */
    std::array<cv::Point2f, 4> anchors{};
    bool has_anchors = false;
  };

  /** @brief 调试/测试用的诊断信息 */
  struct Diagnostics
  {
    bool initialized = false;
    bool updated = false;
    int accepted_blades = 0;
    int rejected_blades = 0;
    int reinitialize_count = 0;
    double reprojection_rmse_px = 0.0;
    double nis = 0.0;
    double angular_velocity = 0.0;
  };

  /** @brief 设置配置与相机模型 @param config UV 配置 @param camera_matrix 3x3 内参 @param distortion 5 维畸变系数 */
  void configure(
    const RuneUvPoseConfig & config, const cv::Matx33d & camera_matrix,
    const cv::Matx<double, 1, 5> & distortion);

  /** @brief 清空滤波器状态 */
  void reset();

  /** @brief 是否已经初始化 @return 已初始化返回 true */
  bool initialized() const { return initialized_; }

  /**
   * @brief 用 PnP/Chamfer 给出的相机系位姿初始化滤波器
   * @param camera_pose 当前帧的 RP 相机位姿
   * @param pose_rvec_tvec 相机系位姿 [rvec(3), tvec(3)]
   * @param preserve_angular_velocity 上一次已经跟踪过时是否保留角速度估计；
   *        首次建立目标或模式切换时应传 false。PnP 兜底重建时保留 w 可以避免
   *        每掉一次帧就重新收敛角速度。
   * @return 输入有效时返回 true
   */
  bool initialize(
    const CameraPose & camera_pose, const Eigen::Matrix<double, 6, 1> & pose_rvec_tvec,
    bool preserve_angular_velocity = false);

  /**
   * @brief 按角速度把符叶绕自身 z 轴旋转到当前帧
   *
   * 状态在固定 RP 系中，相机/云台转动不需要、也不应该在这里补偿；相机姿态只用于
   * 输入有效性检查和后续观测投影。时间间隔 dt 同时用于角速度积分和过程噪声缩放。
   *
   * @param camera_pose 当前帧的 RP 相机位姿（仅做有效性检查）
   * @param dt 两帧时间间隔 (s)
   * @return 滤波器已初始化且输入有效时返回 true
   */
  bool predict(const CameraPose & camera_pose, double dt);

  /**
   * @brief 用本帧所有未激活符叶的关键点 + 锚框点做 EKF 更新
   * @param camera_pose 当前帧的 RP 相机位姿
   * @param observations 未激活符叶观测
   * @return 至少接受一个符叶的观测时返回 true
   */
  bool update(
    const CameraPose & camera_pose, const std::vector<Observation> & observations);

  /**
   * @brief 输出当前滤波位姿对应的相机系位姿
   * @param camera_pose 当前帧的 RP 相机位姿
   * @return [rvec(3), tvec(3)]，供 rp_core 原有投影/平面拟合链路使用
   */
  Eigen::Matrix<double, 6, 1> pose_in_camera(const CameraPose & camera_pose) const;

  /** @brief 预测某个位置号符叶的五点像素坐标（调试用） @param camera_pose 相机位姿 @param location_num 位置号 1~5 @return [top,left,center,right,bottom]，失败时为空 */
  std::vector<cv::Point2f> predict_keypoints(
    const CameraPose & camera_pose, int location_num) const;

  /** @brief 最近一次更新的诊断信息 @return 诊断信息 */
  const Diagnostics & diagnostics() const { return diagnostics_; }

  /** @brief 配置 @return 配置 */
  const RuneUvPoseConfig & config() const { return config_; }

private:
  /** @brief 本地圆模型参数 */
  struct CircleModel
  {
    Eigen::Vector3d center_local = Eigen::Vector3d(0.0, -0.7, 0.0);
    Eigen::Matrix3d rotation = Eigen::Matrix3d::Identity();
    double radius = 0.15;
  };

  /** @brief 构造某个位置号的靶心圆模型 @param location_num 位置号 1~5 @return 模型 */
  CircleModel circle_model(int location_num) const;

  /** @brief 把相机系位姿转换为滤波器内部车系位姿 @param camera_pose 相机位姿 @param pose 相机系位姿 @return 有效时返回 true */
  bool set_pose_from_camera(
    const CameraPose & camera_pose, const Eigen::Matrix<double, 6, 1> & pose);

  /** @brief 把滤波器内部位姿转换为相机系位姿 @param camera_pose 相机位姿 @return 相机系位姿 */
  Eigen::Matrix<double, 6, 1> camera_pose_from_state(const CameraPose & camera_pose) const;

  /** @brief 车系点投影到像素 @param camera_pose 相机位姿 @param point_car 车系点 @param uv 输出像素 @return 有效时返回 true */
  bool project(
    const CameraPose & camera_pose, const Eigen::Vector3d & point_car,
    Eigen::Vector2d & uv) const;

  /** @brief 用指定位姿投影靶心圆上的四个固定关键点 @param camera_pose 相机位姿 @param model 圆模型 @param R_car_from_plane 平面到车系旋转 @param t_car_from_plane 平面原点到车系平移 @param out [top,left,right,bottom] @return 有效时返回 true */
  bool predict_plate_points_with_pose(
    const CameraPose & camera_pose, const CircleModel & model,
    const Eigen::Matrix3d & R_car_from_plane, const Eigen::Vector3d & t_car_from_plane,
    std::array<Eigen::Vector2d, 4> & out) const;

  /** @brief 用指定位姿预测单个符叶的观测 @param camera_pose 相机位姿 @param model 圆模型 @param include_center 是否包含中心 R 点 @param R_car_from_plane 平面到车系旋转 @param t_car_from_plane 平面原点到车系平移 @param out 输出观测向量（五点关键点 + 四锚框点） @return 有效时返回 true */
  bool predict_blade_with_pose(
    const CameraPose & camera_pose, const CircleModel & model, bool include_center,
    const Eigen::Matrix3d & R_car_from_plane, const Eigen::Vector3d & t_car_from_plane,
    Eigen::VectorXd & out) const;

  /** @brief 预测单个符叶的观测（当前状态） @param camera_pose 相机位姿 @param model 圆模型 @param include_center 是否包含中心 R 点 @param out 输出观测向量（五点关键点 + 四锚框点） @return 有效时返回 true */
  bool predict_blade(
    const CameraPose & camera_pose, const CircleModel & model, bool include_center,
    Eigen::VectorXd & out) const;

  /** @brief 在给定基准位姿和扰动下预测单个符叶的观测 @param camera_pose 相机位姿 @param model 圆模型 @param include_center 是否包含中心 R 点 @param R_base 基准旋转 @param t_base 基准平移 @param dtheta 旋转扰动（右乘） @param dt 平移扰动 @param out 输出观测向量 @return 有效时返回 true */
  bool predict_blade_perturbed_with_pose(
    const CameraPose & camera_pose, const CircleModel & model, bool include_center,
    const Eigen::Matrix3d & R_base, const Eigen::Vector3d & t_base,
    const Eigen::Vector3d & dtheta, const Eigen::Vector3d & dt, Eigen::VectorXd & out) const;

  /** @brief 在给定状态扰动下预测单个符叶的观测 @param camera_pose 相机位姿 @param model 圆模型 @param include_center 是否包含中心 R 点 @param dtheta 旋转扰动（右乘） @param dt 平移扰动 @param out 输出观测向量 @return 有效时返回 true */
  bool predict_blade_perturbed(
    const CameraPose & camera_pose, const CircleModel & model, bool include_center,
    const Eigen::Vector3d & dtheta, const Eigen::Vector3d & dt, Eigen::VectorXd & out) const;

  /** @brief 预测靶心圆上的四个固定关键点（当前状态） @param camera_pose 相机位姿 @param model 圆模型 @param out [外侧,左侧,右侧,内侧] @return 有效时返回 true */
  bool predict_plate_points(
    const CameraPose & camera_pose, const CircleModel & model,
    std::array<Eigen::Vector2d, 4> & out) const;

  /** @brief 对协方差做对称化和数值保护 */
  void sanitize_covariance();

  RuneUvPoseConfig config_{};
  cv::Matx33d camera_matrix_ = cv::Matx33d::eye();
  cv::Matx<double, 1, 5> distortion_{};

  Eigen::Matrix3d R_car_from_plane_ = Eigen::Matrix3d::Identity();
  Eigen::Vector3d t_car_from_plane_ = Eigen::Vector3d::Zero();
  double angular_velocity_ = 0.0;
  Eigen::Matrix<double, 7, 7> P_ = Eigen::Matrix<double, 7, 7>::Identity();

  bool initialized_ = false;
  Diagnostics diagnostics_{};
};
