#ifndef AUTO_AIM__PLANNER_HPP
#define AUTO_AIM__PLANNER_HPP

#include <Eigen/Dense>
#include <cmath>
#include <list>
#include <memory>
#include <optional>
#include <string>

#include "../../tracking/target.hpp"
#include "tinympc/tiny_api.hpp"
#include "tools/logger.hpp"

namespace auto_aim
{
constexpr double DT = 0.01;
constexpr int HALF_HORIZON = 50;
constexpr int HORIZON = HALF_HORIZON * 2;

/** @brief 检查开火采样偏移是否在规划时域内 @param offset 相对时域中心的采样偏移 @return 偏移有效时返回 true */
inline bool valid_shoot_offset(int offset)
{
  return HALF_HORIZON + offset >= 0 && HALF_HORIZON + offset < HORIZON;
}

using Trajectory = Eigen::Matrix<double, 4, HORIZON>;  // yaw, yaw_vel, pitch, pitch_vel

struct Plan
{
  bool control;
  bool fire;
  float target_yaw;
  float target_pitch;
  float yaw;
  float yaw_vel;
  float yaw_acc;
  float pitch;
  float pitch_vel;
  float pitch_acc;
};

struct TinySolverDeleter
{
  /** @brief 释放 TinyMPC 求解器 @param solver 求解器指针 */
  void operator()(TinySolver * solver) const noexcept { tiny_cleanup(solver); }
};

using TinySolverHandle = std::unique_ptr<TinySolver, TinySolverDeleter>;

/**
 * @brief 云台实际状态反馈（方案 C 的 MPC 初始状态来源）
 * @note yaw/pitch 单位为度（与 io::GimbalState 约定一致），角速度单位为 rad/s
 */
struct GimbalFeedback
{
  double yaw_deg = 0;
  double pitch_deg = 0;
  double yaw_vel = 0;
  double pitch_vel = 0;
};

/**
 * @brief 单轴云台电机响应模型：纯死区 τ + 一阶收敛 T_cl
 * 离散化（DT=10ms）：y[k+1] = a·y[k] + (1-a)·cmd[k-k_d]
 * 该模型由标定得到（阶跃/斜坡测试），用于方案 C 的开火落地检查
 */
struct GimbalAxisModel
{
  double tau_s = 0.005;   // 纯死区（串口+电控接收+计算）
  double T_cl_s = 0.012;  // 闭环收敛时间常数
  int k_d = 1;            // 输入延时拍数 = round(tau_s / DT)
  double a = 0.4346;      // 一阶系数 = exp(-DT / T_cl_s)

  void sync() noexcept
  {
    // 参数自检：非法值退化为"无滞后"模型（a=0），避免 exp 溢出/NaN 污染仿真；
    // 真正的告警在 Planner 构造里给出（那里才有 logger）
    if (!std::isfinite(tau_s) || tau_s < 0) tau_s = 0;
    if (!std::isfinite(T_cl_s) || T_cl_s <= 0) {
      T_cl_s = 0;
      a = 0.0;  // 无滞后：模型输出恒等于命令
    } else {
      a = std::exp(-DT / T_cl_s);
    }
    // k_d 只能是 DT 的整数倍：默认 tau=5ms 在 DT=10ms 下量化为 1 拍(10ms)，
    // 属偏保守的取整（模型延时略大于真实纯死区）
    k_d = static_cast<int>(std::lround(tau_s / DT));
    if (k_d < 0) k_d = 0;
    if (k_d > HORIZON / 2) k_d = HORIZON / 2;
  }
};

class Planner
{
public:
  enum ShootStrategy{//开火策略
    Dynamics,          //动力学
    rbSuppressiveFire, //旧火控,火力压制
    rbHero,             //英雄
    SB                  //哨兵
  };
  Eigen::Vector4d debug_xyza;
  double aim_target_yaw;
  /** @brief 根据配置初始化轨迹规划器与 MPC 求解器 @param config_path YAML 配置路径 */
  Planner(const std::string & config_path);
  /** @brief 深拷贝规划器并重新创建求解器 @param other 源规划器 */
  Planner(const Planner & other);
  /** @brief 深拷贝赋值规划器 @param other 源规划器 @return 当前规划器 */
  Planner & operator=(const Planner & other);
  /** @brief 移动构造规划器 @param other 源规划器 */
  Planner(Planner && other) noexcept = default;
  /** @brief 移动赋值规划器 @param other 源规划器 @return 当前规划器 */
  Planner & operator=(Planner && other) noexcept = default;

  /** @brief 使用动力学策略规划云台轨迹 @param target 跟踪目标 @param bullet_speed 弹速，单位 m/s @param fb 云台实际状态反馈 @return 云台控制计划 */
  Plan plan(Target target, double bullet_speed, const GimbalFeedback & fb = {});
  /** @brief 预测目标并按指定策略规划 @param target 可选跟踪目标 @param bullet_speed 弹速，单位 m/s @param gimbal_yaw 当前云台偏航角(度) @param strategy 开火策略 @param gimbal_pitch 当前云台俯仰角(度) @param gimbal_yaw_vel 云台偏航角速度(rad/s) @param gimbal_pitch_vel 云台俯仰角速度(rad/s) @return 云台控制计划；无目标时 control 为 false */
  inline Plan plan(
    std::optional<Target> target,
    double bullet_speed,
    double gimbal_yaw = 0,
    ShootStrategy strategy = Dynamics,
    double gimbal_pitch = 0,
    double gimbal_yaw_vel = 0,
    double gimbal_pitch_vel = 0){

    if (!target.has_value()) return {false};

    double delay_time =
      (std::abs(target->ekf_x()[7]) > decision_speed_ ? high_speed_delay_time_ : low_speed_delay_time_) + gimbal_delay_;

    if(std::abs(target->ekf_x()[7]) > decision_speed_) tools::logger()->warn("std::abs(target->ekf_x()[7]) > {}", decision_speed_);

    auto future = std::chrono::steady_clock::now() + std::chrono::microseconds(int(delay_time * 1e6));
    is_far = false;
    is_high = false;
    
    target->predict(future);

    GimbalFeedback fb{gimbal_yaw, gimbal_pitch, gimbal_yaw_vel, gimbal_pitch_vel};

    switch (strategy)
    {
    case Dynamics:
      return plan(*target, bullet_speed, fb);
    case rbSuppressiveFire:
      return rbplan(*target, bullet_speed, fb);
    case rbHero:
      return rbHeroplan(*target, bullet_speed, gimbal_yaw);
    case SB:
      return sbplan(*target, bullet_speed, fb);
    default:
      tools::logger()->error("Unknown shoot strategy: {}", static_cast<int>(strategy));
      return {false};
    }
  }
  /** @brief 使用步兵压制射击策略规划 @param target 跟踪目标 @param bullet_speed 弹速 @param gimbal_yaw 当前云台偏航角 @return 云台控制计划 */
  Plan rbplan(Target target, double bullet_speed, const GimbalFeedback & fb);
  /** @brief 使用哨兵策略规划 @param target 跟踪目标 @param bullet_speed 弹速 @param gimbal_yaw 当前云台偏航角 @return 云台控制计划 */
  Plan sbplan(Target target, double bullet_speed, const GimbalFeedback & fb);
  /** @brief 判断步兵策略当前是否允许开火 @param target 跟踪目标 @param gimbal_yaw 当前云台偏航角 @param tower_fixed_pitch 是否使用前哨站固定俯仰约束 @return 允许开火时返回 true */
  bool rbShoot(Target target, double gimbal_yaw,  bool tower_fixed_pitch = false);
  /** @brief 使用英雄策略规划 @param target 跟踪目标 @param bullet_speed 弹速 @param gimbal_yaw 当前云台偏航角 @return 云台控制计划 */
  Plan rbHeroplan(Target target, double bullet_speed, double gimbal_yaw); 
private:
  bool is_far = false;
  bool is_high = false;
  double yaw_offset_;
  double pitch_offset_;
  double far_pitch_offset_;
  double far_high_pitch_offset_;
  double fire_thresh_;
  double target_dist_error_, target_h_error_;
  double low_speed_delay_time_, high_speed_delay_time_, decision_speed_;
  double small_armor_tolerance, big_armor_tolerance;
  double tower_and_base_armor_tolerance_;
  double gimbal_control_delay;
  double tower_pitch_prediction_time_;

  TinySolverHandle yaw_solver_;
  TinySolverHandle pitch_solver_;
  std::string config_path_;

  int last_selected_idx = -1;
  Eigen::Vector3d last_selected_xyz = Eigen::Vector3d::Zero();

  /** @brief 初始化偏航轴 MPC 求解器 @param config_path YAML 配置路径 */
  void setup_yaw_solver(const std::string & config_path);
  /** @brief 初始化俯仰轴 MPC 求解器 @param config_path YAML 配置路径 */
  void setup_pitch_solver(const std::string & config_path);

  /** @brief 计算动力学策略瞄准角 @param target 跟踪目标 @param bullet_speed 弹速 @return yaw、pitch */
  Eigen::Matrix<double, 2, 1> aim(const Target & target, double bullet_speed);
  /** @brief 计算步兵策略瞄准角 @param target 跟踪目标 @param bullet_speed 弹速 @return yaw、pitch */
  Eigen::Matrix<double, 2, 1> rbaim(const Target & target, double bullet_speed);
  /** @brief 计算英雄策略瞄准角 @param target 跟踪目标 @param bullet_speed 弹速 @param gimbal_yaw 当前云台偏航角 @return yaw、pitch */
  Eigen::Matrix<double, 2, 1> heroaim(const Target & target, double bullet_speed, double gimbal_yaw);
  /** @brief 生成动力学策略预测轨迹 @param target 跟踪目标 @param yaw0 初始偏航角 @param bullet_speed 弹速 @return 规划时域轨迹 */
  Trajectory get_trajectory(Target  target, double yaw0, double bullet_speed);
  /** @brief 生成步兵策略预测轨迹 @param target 跟踪目标 @param yaw0 初始偏航角 @param bullet_speed 弹速 @return 规划时域轨迹 */
  Trajectory rbget_trajectory(Target target, double yaw0, double bullet_speed);

  /**
   * @brief 云台电机响应落地检查（方案 C）：用标定的一阶+延时模型模拟真实云台,
   *        检查开火索引时刻模拟位置与参考轨迹的残差 @param traj 参考轨迹 @param yaw_x yaw MPC 状态轨迹 @param pitch_x pitch MPC 状态轨迹 @param fb 云台实际状态 @param yaw0 偏航参考零点 @param yaw_err 输出：yaw 轴模型残差(rad) @param pitch_err 输出：pitch 轴模型残差(rad)
   */
  void gimbal_landing_check(
    const Trajectory & traj,
    const Eigen::MatrixXd & yaw_x,
    const Eigen::MatrixXd & pitch_x,
    const GimbalFeedback & fb,
    double yaw0,
    double & yaw_err,
    double & pitch_err) const;

  /** @brief 将 MPC 状态写入计划（含命令模式：fire_aim / trajectory_step） @param plan 输出计划 @param yaw_x yaw MPC 状态 @param yaw_u yaw MPC 控制 @param pitch_x pitch MPC 状态 @param pitch_u pitch MPC 控制 @param yaw0 偏航参考零点 */
  void write_mpc_commands(
    Plan & plan,
    const Eigen::MatrixXd & yaw_x,
    const Eigen::MatrixXd & yaw_u,
    const Eigen::MatrixXd & pitch_x,
    const Eigen::MatrixXd & pitch_u,
    double yaw0) const;

  std::chrono::steady_clock::time_point outpost_z_stable_start_time_;
  bool outpost_is_make = true;

  double gimbal_delay_;

  int shoot_offset_;

  // ===== 方案 C：云台电机响应模型与开火落地检查 =====
  GimbalAxisModel yaw_axis_model_;
  GimbalAxisModel pitch_axis_model_;
  /** @brief 开火落地检查容差(rad)；<0 表示关闭检查 */
  double fire_landing_tolerance_ = -1.0;
  /** @brief 命令模式：true=发送 MPC 轨迹步进(下一拍状态)，false=发送开火时刻瞄点(原行为) */
  bool gimbal_command_mode_step_ = false;
};

}  // namespace auto_aim

#endif  // AUTO_AIM__PLANNER_HPP
