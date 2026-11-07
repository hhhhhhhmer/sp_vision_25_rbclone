#include <fmt/core.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <nlohmann/json.hpp>
#include <opencv2/opencv.hpp>
#include <optional>
#include <string>
#include <thread>

#include "io/camera/camera.hpp"
#include "io/gimbal/gimbal.hpp"
#include "tasks/auto_aim/aiming/planner/planner.hpp"
#include "tasks/auto_aim/detection/yolo.hpp"
#include "tasks/auto_aim/geometry/solver.hpp"
#include "tasks/auto_aim/tracking/target.hpp"
#include "tasks/auto_aim/tracking/tracker.hpp"
#include "tools/exiter.hpp"
#include "tools/img_tools.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/periodic_timer.hpp"
#include "tools/plotter.hpp"
#include "tools/reprojection.hpp"
#include "tools/systemd_watchdog.hpp"
#include "tools/thread_safe_queue.hpp"
#include "tools/trajectory.hpp"
#include "tools/yaml.hpp"

using namespace std::chrono_literals;

namespace rb_runtime
{

/** @brief 两个自瞄程序的公共运行选项 */
struct Options
{
  std::string config_path;  ///< yaml 配置路径
  bool multithread = true;  ///< true: 异步检测 + 独立下发线程；false: 单线程同步
  bool sb_send = false;     ///< true: 哨兵 sb_send 帧；false: 普通 send 帧
};

/** @brief 命令行参数定义 @return cv::CommandLineParser 使用的 keys */
inline const std::string & cli_keys()
{
  static const std::string keys =
    "{help h usage ? |                           | 输出命令行参数说明}"
    "{@config-path   | ../configs/rb_auto_aim.yaml | 位置参数，yaml配置文件路径 }"
    "{thread t       | 1                         | 1=多线程(异步推理+独立下发线程)，0=单线程同步 }"
    "{sb_send s      | 0                         | 1=哨兵 sb_send 帧(带目标坐标)，0=普通 send 帧 }";
  return keys;
}

/** @brief 解析命令行参数 @param argc 参数个数 @param argv 参数数组 @param opt 输出选项 @return 参数有效且需要继续运行时返回 true */
inline bool parse_options(int argc, char ** argv, Options & opt)
{
  cv::CommandLineParser cli(argc, argv, cli_keys());
  opt.config_path = cli.get<std::string>(0);
  if (cli.has("help") || opt.config_path.empty()) {
    cli.printMessage();
    return false;
  }

  const int thread = cli.get<int>("thread");
  const int sb_send = cli.get<int>("sb_send");
  if ((thread != 0 && thread != 1) || (sb_send != 0 && sb_send != 1)) {
    tools::logger()->error("--thread 和 --sb_send 只能是 0 或 1");
    cli.printMessage();
    return false;
  }

  opt.multithread = (thread == 1);
  opt.sb_send = (sb_send == 1);
  return true;
}

/** @brief 判断云台是否处于打符模式（本程序只管自瞄，打符时不下发自瞄指令） @param mode 云台模式 @return 打符模式返回 true */
inline bool is_buff(io::GimbalMode mode)
{
  return mode == io::GimbalMode::SMALL_BUFF || mode == io::GimbalMode::BIG_BUFF;
}

/** @brief 按选项发送一帧自瞄指令 @param gimbal 云台串口 @param plan 规划结果 @param target 当前目标（sb_send 需要目标坐标） @param sb_send 是否使用哨兵帧 */
inline void send_plan(
  io::Gimbal & gimbal, const auto_aim::Plan & plan, const std::optional<auto_aim::Target> & target,
  bool sb_send)
{
  if (!sb_send) {
    gimbal.send(
      plan.control, plan.fire, plan.yaw, plan.yaw_vel, plan.yaw_acc, plan.pitch, plan.pitch_vel,
      plan.pitch_acc);
    return;
  }

  // 哨兵帧额外下发目标在场地坐标系中的中心位置与编号；沿用仓库里 sb_send 的既有约定：
  // EKF 的 x/z 是场地平面坐标，name + 1 对应下位机的目标编号（0 表示无目标）。
  float target_x = 0.0F;
  float target_y = 0.0F;
  std::uint8_t target_name = 0;
  if (target.has_value()) {
    const auto ekf_x = target->ekf_x();
    target_x = static_cast<float>(ekf_x[0]);
    target_y = static_cast<float>(ekf_x[2]);
    target_name = static_cast<std::uint8_t>(target->name) + 1;
  }

  gimbal.sb_send(
    plan.control, plan.fire, plan.yaw, plan.yaw_vel, plan.yaw_acc, plan.pitch, plan.pitch_vel,
    plan.pitch_acc, target_x, target_y, target_name);
}

/** @brief 释放控制并让云台保持在当前角度 @param gimbal 云台串口 @param sb_send 是否使用哨兵帧 @param yaw 当前 yaw，单位 rad @param pitch 当前 pitch，单位 rad */
inline void send_stop(io::Gimbal & gimbal, bool sb_send, float yaw, float pitch)
{
  if (sb_send) {
    gimbal.sb_send(false, false, yaw, 0.0F, 0.0F, pitch, 0.0F, 0.0F, 0.0F, 0.0F, 0);
  } else {
    gimbal.send(false, false, yaw, 0.0F, 0.0F, pitch, 0.0F, 0.0F);
  }
}

}  // namespace rb_runtime

namespace
{

constexpr auto kSendPeriod = 10ms;
using TargetQueue = tools::ThreadSafeQueue<std::optional<auto_aim::Target>, true>;

/**
 * ============================ debug plot 字段说明 ============================
 * 所有数据都通过 plotter.plot(data) 以 JSON 发送并落盘，PlotJuggler 里直接按下面的
 * 键名查看。这里按“数据来源/用途”分组，方便上车时快速定位。
 *
 * 【时间与运行模式】
 *   t                    程序启动以来的时间，单位 s
 *   multithread          1=多线程(异步推理+独立下发线程)，0=单线程
 *   sb_send              1=哨兵 sb_send 帧，0=普通 send 帧
 *
 * 【云台反馈】(io::Gimbal 从下位机收到)
 *   gimbal_yaw           云台当前 yaw，单位 deg
 *   gimbal_pitch         云台当前 pitch，单位 deg
 *   gimbal_yaw_vel       云台 yaw 角速度，单位 rad/s
 *   gimbal_pitch_vel     云台 pitch 角速度，单位 rad/s
 *   q2yaw/q2pitch        GimbalState 保留字段；当前 gimbal 反馈没有赋值，通常为 0
 *
 * 【Planner 实际下发】
 *   plan_mode            0=不控制，1=控制但不开火，2=控制并开火
 *   plan_yaw             下发的 yaw 指令，单位 deg
 *   plan_pitch           下发的 pitch 指令，单位 deg
 *   plan_yaw_vel         yaw 角速度指令，单位 rad/s
 *   plan_pitch_vel       pitch 角速度指令，单位 rad/s
 *   plan_yaw_acc         yaw 角加速度指令，单位 rad/s^2
 *   plan_pitch_acc       pitch 角加速度指令，单位 rad/s^2
 *   target_yaw           Planner 内部目标参考 yaw；rbSuppressiveFire 下单位是 deg
 *   target_pitch         Planner 内部目标参考 pitch；rbSuppressiveFire 下单位是 rad
 *   fire                 本帧 Planner 是否允许开火
 *   fired                本帧下位机弹丸计数是否增加（实际是否打出弹）
 *   plan_ms              planner.plan() 单帧耗时，单位 ms
 *
 * 【发送与主循环耗时】
 *   send_late_us         下发线程相对 10ms 节拍点的迟到时间，单位 us；<=0 为准时
 *   send_missed          下发线程累计因超时跳过的节拍数
 *   detect_ms            YOLO 检测耗时，单位 ms
 *   track_ms             tracker.track() 耗时，单位 ms
 *   capture_fps          相机采集帧率
 *
 * 【目标与延迟】
 *   has_target           1=本帧有目标，0=无目标
 *   target_age_ms        目标时间戳到当前时刻的年龄，单位 ms
 *   delay_speed          发弹延迟（high/low_speed_delay_time），单位 s
 *   delay_gimbal         云台延迟（gimbal_delay），单位 s
 *   delay_total_before_fly 目标年龄 + 发弹延迟 + 云台延迟，单位 s
 *
 * 【目标中心 EKF 状态】(世界系)
 *   ekf_x/ekf_vx         目标中心 x 位置/速度，单位 m、m/s
 *   ekf_y/ekf_vy         目标中心 y 位置/速度，单位 m、m/s
 *   ekf_z/ekf_vz         目标中心 z 位置/速度，单位 m、m/s
 *   ekf_yaw/ekf_vyaw     目标中心 yaw/角速度，单位 deg、deg/s
 *   ekf_r                目标旋转半径，单位 m
 *   target_name          ArmorName 枚举：one=0,two=1,three=2,four=3,five=4,
 *                        sentry=5,outpost=6,base=7,not_armor=8
 *   armor_type           0=big，1=small
 *   armor_num            目标装甲板数量（3 或 4）
 *   target_uuid          目标实例唯一编号；换目标时变化
 *   target_update_count  EKF 已执行校正的帧数
 *   target_jumped        跟踪器是否发生过换板跳变
 *   target_last_id       跟踪器最近一次观测到的装甲板编号
 *
 * 【每块装甲板】(i = 0 .. armor_num-1)
 *   armorI_x/y/z         该装甲板世界坐标，单位 m
 *   armorI_yaw_deg       该装甲板朝向 yaw，单位 deg
 *   armorI_dist          该装甲板到云台的水平距离，单位 m
 *   armorI_height_offset 该装甲板相对目标中心的高度偏置，单位 m
 *   armorI_lateral_offset 该装甲板切向左右偏置，单位 m（仅四装甲目标，其余为 0）
 *   ekf_lateral_offset0..2 / ekf_height_offset0..2
 *                        EKF 内部零均值相对偏置原始状态（只存前 3 块）
 *   ekf_tower_height_offset 前哨站高度阶梯状态；非前哨站通常为 0
 *   tower_h1/h2/h3       前哨站三块板的高度观测，单位 m；.second=0 表示尚未采样
 *   tower_armor_h        前哨站装甲板高度，单位 m
 *
 * 【Planner 实际选板/瞄点】
 *   aim_armor_id         本帧实际选中的装甲板编号
 *   aim_held_id          跨帧滞环保持的装甲板编号
 *   aim_nearest_id       纯按距离最近的装甲板编号
 *   aim_hold_engaged     1=滞环压住了最近板（held != nearest）
 *   aim_point_x/y/z      实际瞄准点世界坐标，单位 m
 *   aim_point_yaw_deg    实际瞄准点处装甲板朝向，单位 deg
 *
 * 【三份预测对比】prefix = pure / fly / full
 *   pure_*  只按弹丸飞行时间外推：目标时间戳 + fly_time，不含图像年龄和其他延迟
 *   fly_*   目标时间戳 + 图像年龄 + fly_time（从当前时刻起只算飞行时间）
 *   full_*  目标时间戳 + 图像年龄 + 发弹延迟 + 云台延迟 + fly_time
 *           （与 Planner::plan() 的预测口径一致）
 *
 *   每个 prefix 都有：
 *   <prefix>_valid       1=该口径解算成功
 *   <prefix>_armor_id    该口径下选中的装甲板编号
 *   <prefix>_nearest_id  该口径下纯按距离最近的装甲板编号
 *   <prefix>_fly_time    弹丸飞行时间，单位 s
 *   <prefix>_dist        选中装甲板水平距离，单位 m
 *   <prefix>_x/y/z       选中装甲板在“命中时刻”的世界坐标，单位 m
 *   <prefix>_raw_yaw_deg 到选中装甲板的方位角（未加 yaw_offset），单位 deg
 *   <prefix>_armor_yaw_deg 选中装甲板自身朝向 yaw，单位 deg
 *   <prefix>_raw_pitch_deg 弹道俯仰角（未加 pitch 补偿），单位 deg
 *   <prefix>_cmd_yaw_deg   应下发 yaw = raw_yaw + yaw_offset，单位 deg
 *   <prefix>_cmd_pitch_deg 应下发 pitch = raw_pitch + pitch 补偿，单位 deg
 *   <prefix>_delta_yaw_deg / <prefix>_delta_pitch_deg
 *                        与真实 plan_yaw/plan_pitch 的差；仅 plan.control=1 时写入
 *   <prefix>_ekf_x/...   该口径预测后的目标 EKF 状态（字段同上面的 ekf_*）
 *
 * 【三份之间的差】
 *   pure_vs_fly_delta_x/y/z
 *                        pure 与 fly 的位置差（图像年龄造成的影响），单位 m
 *   fly_vs_full_delta_yaw_deg / _pitch_deg
 *                        fly 与 full 的瞄准角差（发弹+云台延迟造成的影响），单位 deg
 *   fly_vs_full_delta_x/y/z
 *                        fly 与 full 的位置差，单位 m
 *   fly_target_yaw / fly_target_pitch
 *                        注意：这两个是 pure 与 fly 的“瞄准角差”，不是目标 yaw；
 *                        目标 yaw 请看 pure_ekf_yaw_deg / fly_ekf_yaw_deg。
 * ============================================================================
 */

/** @brief 调试程序额外需要的 yaml 参数（用于复现/对比 Planner 的延迟与补偿） */
struct DebugParams
{
  double target_dist_error = 0.0;        ///< 距离补偿，用于估算飞行时间，单位 m
  double target_h_error = 0.0;           ///< 高度补偿，用于估算飞行时间，单位 m
  double decision_speed = 8.0;           ///< 高低速目标判断阈值，单位 rad/s
  double high_speed_delay_time = 0.030;  ///< 高速目标发弹延迟，单位 s
  double low_speed_delay_time = 0.015;   ///< 低速目标发弹延迟，单位 s
  double gimbal_delay = 0.002;           ///< 云台/链路延迟，单位 s
  double yaw_offset = 0.0;               ///< yaw 补偿，单位 rad
  double pitch_offset = 0.0;             ///< 近距离 pitch 补偿，单位 rad
  double far_pitch_offset = 0.0;         ///< 远距离 pitch 补偿，单位 rad
  double far_high_pitch_offset = 0.0;    ///< 远距离且高目标 pitch 补偿，单位 rad
};

/** @brief 读取可选 double 配置，缺失时使用默认值 @param yaml 配置根节点 @param key 键名 @param fallback 默认值 @return 配置值 */
double read_double(const YAML::Node & yaml, const char * key, double fallback)
{
  return yaml[key] ? yaml[key].as<double>() : fallback;
}

/** @brief 加载调试参数 @param config_path yaml 路径 @return 调试参数 */
DebugParams load_debug_params(const std::string & config_path)
{
  const auto yaml = tools::load(config_path);
  DebugParams p;
  p.target_dist_error = read_double(yaml, "target_dist_error", 0.0);
  p.target_h_error = read_double(yaml, "target_h_error", 0.0);
  p.decision_speed = read_double(yaml, "decision_speed", 8.0);
  p.high_speed_delay_time = read_double(yaml, "high_speed_delay_time", 0.030);
  p.low_speed_delay_time = read_double(yaml, "low_speed_delay_time", 0.015);
  p.gimbal_delay = read_double(yaml, "gimbal_delay", 0.002);
  p.yaw_offset = read_double(yaml, "yaw_offset", 0.0) / 57.3;
  p.pitch_offset = read_double(yaml, "pitch_offset", 0.0) / 57.3;
  p.far_pitch_offset = read_double(yaml, "far_pitch_offset", 0.0) / 57.3;
  p.far_high_pitch_offset = read_double(yaml, "far_high_pitch_offset", 0.0) / 57.3;
  return p;
}

/** @brief 一次瞄准解算结果（用于 debug 对比） */
struct AimSolution
{
  bool valid = false;                             ///< 解算是否成功
  int armor_id = -1;                              ///< 选中的装甲板编号
  int nearest_id = -1;                            ///< 纯按距离最近的装甲板编号
  double fly_time = 0.0;                          ///< 弹丸飞行时间，单位 s
  double horizontal_distance = 0.0;               ///< 选中装甲板水平距离，单位 m
  Eigen::Vector3d xyz = Eigen::Vector3d::Zero();  ///< 命中时刻装甲板世界坐标，单位 m
  double raw_yaw = 0.0;                           ///< 未加 yaw_offset 的方位角，单位 rad
  double armor_yaw = 0.0;                         ///< 选中装甲板的朝向 yaw，单位 rad
  double raw_pitch = 0.0;  ///< 未加 pitch 补偿的弹道俯仰角，单位 rad
  double cmd_yaw = 0.0;    ///< 与 Planner::rbaim 同口径的下发 yaw，单位 rad
  double cmd_pitch = 0.0;  ///< 与 Planner::rbaim 同口径的下发 pitch，单位 rad
  Eigen::VectorXd ekf_x;   ///< 预测到命中时刻后的目标 EKF 状态
};

/**
 * @brief 只做“指定延迟 + 弹丸飞行时间”的瞄准解算
 *
 * 三种口径由 delay_before_fly 区分：
 *  - 0：pure，严格只考虑弹丸飞行时间，从目标时间戳直接外推 fly_time；
 *  - 目标数据年龄：fly，从当前时刻起只算飞行时间（含图像到现在的年龄）；
 *  - 目标年龄 + speed_delay + gimbal_delay：full，与 Planner::plan() 的预测口径一致。
 * 这里刻意按 Planner::rbplan() 的顺序复现：先预测到出膛时刻，再用那时的几何算飞行时间，
 * 最后预测飞行时间并选板/算弹道。
 *
 * @param target 跟踪目标（不会被修改） @param bullet_speed 弹速 @param delay_before_fly 出膛前延迟
 * @param p 调试参数 @return 解算结果；无法解算时 valid=false
 */
AimSolution solve_aim(
  const auto_aim::Target & target, double bullet_speed, double delay_before_fly,
  const DebugParams & p)
{
  AimSolution s;
  if (target.armor_xyza_list().empty()) return s;

  if (bullet_speed < 10.0 || bullet_speed > 25.0) bullet_speed = 22.0;

  auto staged = target;
  try {
    if (delay_before_fly > 0.0) staged.predict(delay_before_fly);
  } catch (const std::exception &) {
    return s;
  }

  // 与 Planner::rbplan 相同：先用“最近板 + 距离/高度补偿”估算飞行时间
  double min_dist = 1e10;
  Eigen::Vector3d nearest_xyz = Eigen::Vector3d::Zero();
  for (const auto & xyza : staged.armor_xyza_list()) {
    const double dist = xyza.head<2>().norm();
    if (dist < min_dist) {
      min_dist = dist;
      nearest_xyz = xyza.head<3>();
    }
  }
  const double plan_dist = min_dist + p.target_dist_error;
  const double plan_h = nearest_xyz.z() + p.target_h_error;
  tools::Trajectory bullet_traj(bullet_speed, plan_dist, plan_h);
  if (bullet_traj.unsolvable) return s;
  s.fly_time = bullet_traj.fly_time;

  try {
    staged.predict(s.fly_time);
  } catch (const std::exception &) {
    return s;
  }

  s.armor_id = staged.select_aim_armor(-1, &s.nearest_id);
  if (s.armor_id < 0) return s;

  const auto xyzad = staged.aim_armor_xyzad(s.armor_id);
  if (!std::isfinite(xyzad(4))) return s;

  s.xyz = xyzad.head<3>();
  s.armor_yaw = xyzad(3);
  s.horizontal_distance = xyzad.head<2>().norm();
  tools::Trajectory aim_traj(bullet_speed, s.horizontal_distance, s.xyz.z());
  if (aim_traj.unsolvable) return s;

  s.raw_yaw = std::atan2(s.xyz.y(), s.xyz.x());
  s.raw_pitch = aim_traj.pitch;

  // Planner::rbplan 的 is_far/is_high 用的是预测到出膛时刻后的最近板几何
  const bool far = plan_dist > 5.0;
  const bool high = plan_h > 1.3;
  const double pitch_offset =
    far ? (high ? p.far_high_pitch_offset : p.far_pitch_offset) : p.pitch_offset;
  s.cmd_yaw = tools::limit_rad(s.raw_yaw + p.yaw_offset);
  s.cmd_pitch = s.raw_pitch + pitch_offset;

  s.ekf_x = staged.ekf_x();
  s.valid = true;
  return s;
}

/**
 * @brief 写目标中心 EKF、每块装甲板位置/偏置、前哨站高度
 * @param data 输出 json @param target 当前目标
 * @note 字段单位/含义见文件顶部的“debug plot 字段说明”。
 */
void append_target_json(nlohmann::json & data, const auto_aim::Target & target)
{
  const auto ekf_x = target.ekf_x();
  const auto armors = target.armor_xyza_list();
  const int armor_num = static_cast<int>(armors.size());

  // ---- 目标基本信息 ----
  data["target_name"] = static_cast<int>(target.name);       // ArmorName 枚举值
  data["armor_type"] = static_cast<int>(target.armor_type);  // 0=big, 1=small
  data["armor_num"] = armor_num;                             // 3 或 4
  data["target_uuid"] = target.uuid();                       // 目标实例唯一编号
  data["target_update_count"] = target.update_count_;        // EKF 已校正帧数
  data["target_jumped"] = target.jumped ? 1 : 0;             // 是否发生过换板跳变
  data["target_last_id"] = target.last_id;                   // 最近观测到的板号

  // ---- 目标中心 EKF 状态（世界系，x/y 为场地平面，z 为高度）----
  data["ekf_x"] = ekf_x[0];            // 中心 x，m
  data["ekf_vx"] = ekf_x[1];           // 中心 vx，m/s
  data["ekf_y"] = ekf_x[2];            // 中心 y，m
  data["ekf_vy"] = ekf_x[3];           // 中心 vy，m/s
  data["ekf_z"] = ekf_x[4];            // 中心 z，m
  data["ekf_vz"] = ekf_x[5];           // 中心 vz，m/s
  data["ekf_yaw"] = ekf_x[6] * 57.3;   // 中心 yaw，deg
  data["ekf_vyaw"] = ekf_x[7] * 57.3;  // 中心角速度，deg/s
  data["ekf_r"] = ekf_x[8];            // 旋转半径，m

  // ---- 每块装甲板的位置/朝向/偏置 ----
  for (int i = 0; i < armor_num; ++i) {
    const auto & xyza = armors[static_cast<std::size_t>(i)];
    const std::string prefix = "armor" + std::to_string(i);
    data[prefix + "_x"] = xyza[0];                         // 世界系 x，m
    data[prefix + "_y"] = xyza[1];                         // 世界系 y，m
    data[prefix + "_z"] = xyza[2];                         // 世界系 z，m
    data[prefix + "_yaw_deg"] = xyza[3] * 57.3;            // 装甲板朝向，deg
    data[prefix + "_dist"] = xyza.head<2>().norm();        // 水平距离，m
    data[prefix + "_height_offset"] = xyza[2] - ekf_x[4];  // 相对中心高度偏置，m

    // 四装甲目标才有逐板左右偏置；第 4 块由前 3 块负和导出
    double lateral_offset = 0.0;  // 切向左右偏置，m
    if (armor_num == 4) {
      if (i < 3) {
        lateral_offset = ekf_x[9 + i];
      } else {
        lateral_offset = -(ekf_x[9] + ekf_x[10] + ekf_x[11]);
      }
    }
    data[prefix + "_lateral_offset"] = lateral_offset;
  }

  // ---- EKF 内部的零均值相对偏置原始状态（只存前 3 块）----
  for (int i = 0; i < 3; ++i) {
    data["ekf_lateral_offset" + std::to_string(i)] = ekf_x[9 + i];  // 左右偏置，m
    data["ekf_height_offset" + std::to_string(i)] = ekf_x[13 + i];  // 高度偏置，m
  }
  data["ekf_tower_height_offset"] = ekf_x.size() > 16 ? ekf_x[16] : 0.0;  // 前哨站高度阶梯状态

  // ---- 前哨站高度阶梯（非前哨站通常为 0）----
  data["tower_h1"] = target.tower_armor_hs[0].second;  // 板 1 高度观测，m
  data["tower_h2"] = target.tower_armor_hs[1].second;  // 板 2 高度观测，m
  data["tower_h3"] = target.tower_armor_hs[2].second;  // 板 3 高度观测，m
  data["tower_armor_h"] = target.tower_armor_h;        // 前哨站装甲板高度，m
}

/**
 * @brief 把一份瞄准解算结果写入 json，并与实际下发 plan 做差
 * @param data 输出 json @param prefix 字段前缀（pure / fly / full）
 * @param s 解算结果 @param plan 实际规划结果
 *
 * 字段含义（prefix 换成 pure/fly/full）：
 *   prefix_valid        1=该口径解算成功；失败时后面字段不写
 *   prefix_armor_id     选中装甲板编号
 *   prefix_nearest_id   纯按距离最近的装甲板编号
 *   prefix_fly_time     弹丸飞行时间，s
 *   prefix_dist         选中装甲板水平距离，m
 *   prefix_x/y/z        命中时刻选中装甲板世界坐标，m
 *   prefix_raw_yaw_deg  到选中装甲板的方位角（未加 yaw_offset），deg
 *   prefix_armor_yaw_deg 选中装甲板自身朝向 yaw，deg
 *   prefix_raw_pitch_deg 弹道俯仰角（未加 pitch 补偿），deg
 *   prefix_cmd_yaw_deg  应下发 yaw（raw_yaw + yaw_offset），deg
 *   prefix_cmd_pitch_deg 应下发 pitch（raw_pitch + pitch 补偿），deg
 *   prefix_delta_*      与真实 plan_yaw/plan_pitch 的差，deg
 *   prefix_ekf_*        该口径预测后的目标 EKF 状态
 */
void append_solution(
  nlohmann::json & data, const std::string & prefix, const AimSolution & s,
  const auto_aim::Plan & plan)
{
  // 先写 valid，保证解算失败时 plot 里也有这条曲线（其余字段留空）
  data[prefix + "_valid"] = s.valid ? 1 : 0;
  if (!s.valid) return;

  // ---- 选中的装甲板与命中点 ----
  data[prefix + "_armor_id"] = s.armor_id;         // 选中的板号
  data[prefix + "_nearest_id"] = s.nearest_id;     // 最近板号
  data[prefix + "_fly_time"] = s.fly_time;         // 飞行时间，s
  data[prefix + "_dist"] = s.horizontal_distance;  // 水平距离，m
  data[prefix + "_x"] = s.xyz.x();                 // 命中点 x，m
  data[prefix + "_y"] = s.xyz.y();                 // 命中点 y，m
  data[prefix + "_z"] = s.xyz.z();                 // 命中点 z，m

  // ---- 瞄准角 ----
  data[prefix + "_raw_yaw_deg"] = s.raw_yaw * 57.3;      // 方位角，deg
  data[prefix + "_armor_yaw_deg"] = s.armor_yaw * 57.3;  // 装甲板朝向，deg
  data[prefix + "_raw_pitch_deg"] = s.raw_pitch * 57.3;  // 弹道俯仰角，deg
  data[prefix + "_cmd_yaw_deg"] = s.cmd_yaw * 57.3;      // 加补偿后的 yaw，deg
  data[prefix + "_cmd_pitch_deg"] = s.cmd_pitch * 57.3;  // 加补偿后的 pitch，deg

  // ---- 与实际下发 plan 的差（仅实际控制时才有意义）----
  if (plan.control) {
    data[prefix + "_delta_yaw_deg"] = tools::limit_rad(s.cmd_yaw - plan.yaw) * 57.3;
    data[prefix + "_delta_pitch_deg"] = (s.cmd_pitch - plan.pitch) * 57.3;
  }

  // ---- 该口径预测到命中时刻后的目标 EKF 状态 ----
  if (s.ekf_x.size() > 8) {
    data[prefix + "_ekf_x"] = s.ekf_x[0];                // x，m
    data[prefix + "_ekf_vx"] = s.ekf_x[1];               // vx，m/s
    data[prefix + "_ekf_y"] = s.ekf_x[2];                // y，m
    data[prefix + "_ekf_vy"] = s.ekf_x[3];               // vy，m/s
    data[prefix + "_ekf_z"] = s.ekf_x[4];                // z，m
    data[prefix + "_ekf_vz"] = s.ekf_x[5];               // vz，m/s
    data[prefix + "_ekf_yaw_deg"] = s.ekf_x[6] * 57.3;   // 目标 yaw，deg
    data[prefix + "_ekf_vyaw_deg"] = s.ekf_x[7] * 57.3;  // 目标角速度，deg/s
    data[prefix + "_ekf_r"] = s.ekf_x[8];                // 半径，m
  }
}

/**
 * @brief 把调试专用的偏置/延迟/三份预测对比数据写入 json
 * @param data 输出 json @param target 当前目标（可能为空）
 * @param plan Planner 实际下发结果 @param planner 规划器（读取选板诊断）
 * @param p 调试参数 @param target_age 目标时间戳年龄，s @param speed_delay 发弹延迟，s
 * @param pure 只按飞行时间的解算 @param fly 目标年龄+飞行时间的解算
 * @param full 目标年龄+发弹/云台延迟+飞行时间的解算
 * @param detect_ms 检测耗时 @param track_ms 跟踪耗时 @param capture_fps 采集帧率
 */
void append_debug_data(
  nlohmann::json & data, const std::optional<auto_aim::Target> & target,
  const auto_aim::Plan & plan, const auto_aim::Planner & planner, const DebugParams & p,
  double target_age, double speed_delay, const AimSolution & pure, const AimSolution & fly,
  const AimSolution & full, double detect_ms, double track_ms, double capture_fps)
{
  // ---- 默认值：无目标/解算失败时字段仍然存在，避免 plot 曲线整段消失 ----
  data["has_target"] = target.has_value() ? 1 : 0;  // 1=本帧有目标
  data["pure_valid"] = 0;                           // 只按飞行时间是否解算成功
  data["fly_valid"] = 0;                            // 年龄+飞行时间是否解算成功
  data["full_valid"] = 0;                           // 完整延迟是否解算成功
  data["aim_armor_id"] = -1;                        // 实际选中的板号
  data["aim_held_id"] = -1;                         // 滞环保持的板号
  data["aim_nearest_id"] = -1;                      // 纯最近板号
  data["aim_hold_engaged"] = 0;                     // 滞环是否压住最近板

  // ---- 主循环耗时 ----
  data["detect_ms"] = detect_ms;      // YOLO 检测耗时，ms
  data["track_ms"] = track_ms;        // tracker.track() 耗时，ms
  data["capture_fps"] = capture_fps;  // 相机采集帧率

  // ---- 目标年龄与 Planner 用到的延迟 ----
  data["target_age_ms"] = target_age * 1000.0;  // 目标时间戳年龄，ms
  data["delay_speed"] = speed_delay;            // 发弹延迟，s
  data["delay_gimbal"] = p.gimbal_delay;        // 云台延迟，s
  data["delay_total_before_fly"] = target_age + speed_delay + p.gimbal_delay;  // 出膛前总延迟，s

  // ---- 目标中心 EKF、每块装甲板位置/偏置、前哨站高度 ----
  if (target.has_value()) {
    append_target_json(data, *target);
  }

  // ---- Planner 实际选中的瞄板/瞄点（与真正下发的命令同源）----
  if (plan.control) {
    const auto selection = planner.aim_selection_debug();
    data["aim_armor_id"] = planner.aim_armor_id();              // 实际选中的板号
    data["aim_held_id"] = selection.held;                       // 滞环保持的板号
    data["aim_nearest_id"] = selection.nearest;                 // 纯最近板号
    data["aim_hold_engaged"] = selection.hold_engaged ? 1 : 0;  // 滞环是否生效
    data["aim_point_x"] = planner.debug_xyza[0];                // 瞄准点 x，m
    data["aim_point_y"] = planner.debug_xyza[1];                // 瞄准点 y，m
    data["aim_point_z"] = planner.debug_xyza[2];                // 瞄准点 z，m
    data["aim_point_yaw_deg"] = planner.debug_xyza[3] * 57.3;   // 该板朝向，deg
  }

  // ---- 三份预测：pure / fly / full（字段说明见 append_solution）----
  append_solution(data, "pure", pure, plan);  // 只按弹丸飞行时间
  append_solution(data, "fly", fly, plan);    // 目标年龄 + 飞行时间
  append_solution(data, "full", full, plan);  // 目标年龄 + 发弹/云台延迟 + 飞行时间

  // ---- 三份预测之间的差 ----
  if (pure.valid && fly.valid) {
    // 注意：fly_target_yaw / fly_target_pitch 是 pure 与 fly 的“瞄准角差”，
    // 不是目标 yaw；目标 yaw 看 pure_ekf_yaw_deg / fly_ekf_yaw_deg。
    data["fly_target_yaw"] = tools::limit_rad(pure.cmd_yaw - fly.cmd_yaw) * 57.3;
    data["fly_target_pitch"] = (pure.cmd_pitch - fly.cmd_pitch) * 57.3;
    data["pure_vs_fly_delta_x"] = pure.xyz.x() - fly.xyz.x();  // 图像年龄造成的 x 差，m
    data["pure_vs_fly_delta_y"] = pure.xyz.y() - fly.xyz.y();  // 图像年龄造成的 y 差，m
    data["pure_vs_fly_delta_z"] = pure.xyz.z() - fly.xyz.z();  // 图像年龄造成的 z 差，m
  }
  if (fly.valid && full.valid) {
    // 发弹延迟 + 云台延迟造成的影响
    data["fly_vs_full_delta_yaw_deg"] = tools::limit_rad(fly.cmd_yaw - full.cmd_yaw) * 57.3;
    data["fly_vs_full_delta_pitch_deg"] = (fly.cmd_pitch - full.cmd_pitch) * 57.3;
    data["fly_vs_full_delta_x"] = fly.xyz.x() - full.xyz.x();
    data["fly_vs_full_delta_y"] = fly.xyz.y() - full.xyz.y();
    data["fly_vs_full_delta_z"] = fly.xyz.z() - full.xyz.z();
  }
}

/** @brief 在图上画目标装甲板轮廓和瞄准点（数值调试量统一走 Plotter，不在图上堆文字） */
void draw_target_debug(
  cv::Mat & img, const auto_aim::Solver & solver, const auto_aim::Target & target,
  const std::optional<Eigen::Vector4d> & aim_xyza)
{
  tools::draw_reprojection(img, solver, target, aim_xyza);
}

}  // namespace

int main(int argc, char ** argv)
{
  rb_runtime::Options opt;
  if (!rb_runtime::parse_options(argc, argv, opt)) return 0;

  tools::SystemdWatchdog watchdog;
  tools::Exiter exiter;
  tools::Plotter plotter;

  const auto debug_params = load_debug_params(opt.config_path);

  io::Gimbal gimbal(opt.config_path);
  io::Camera camera(opt.config_path);

  // 调试程序打开 YOLO 内部调试输出；不需要时把它改成 false 即可
  auto_aim::YOLO yolo(opt.config_path, true);
  auto_aim::Solver solver(opt.config_path);
  auto_aim::Tracker tracker(opt.config_path, &solver);
  tracker.set_gimbal(&gimbal);
  auto_aim::Planner planner(opt.config_path);

  tools::logger()->info(
    "[rb_auto_aim_debug] config={} thread={} sb_send={}", opt.config_path, opt.multithread ? 1 : 0,
    opt.sb_send ? 1 : 0);

  TargetQueue target_queue(1);
  target_queue.push(std::nullopt);
  std::atomic<bool> quit{false};

  // 主线程只负责采集/推理/跟踪，这些耗时用原子量传给下发线程写进 json
  std::atomic<double> detect_ms{0.0};
  std::atomic<double> track_ms{0.0};
  std::atomic<double> capture_fps{0.0};

  // 每帧组装一份 plot JSON；字段含义见文件顶部的“debug plot 字段说明”。
  // 多线程模式下这个 lambda 在下发线程里执行，单线程模式下在主循环里执行。
  auto plot_plan = [&](
                     const auto_aim::Plan & plan, const std::optional<auto_aim::Target> & target,
                     const io::GimbalState & gs, bool fired, double plan_ms, double target_age,
                     double speed_delay, const AimSolution & pure, const AimSolution & fly,
                     const AimSolution & full, std::chrono::steady_clock::duration lateness,
                     std::uint64_t missed, std::chrono::steady_clock::time_point t0) {
    nlohmann::json data;

    // ---- 时间与运行模式 ----
    data["t"] = tools::delta_time(std::chrono::steady_clock::now(), t0);  // 程序启动后时间，s
    data["multithread"] = opt.multithread ? 1 : 0;  // 1=多线程，0=单线程
    data["sb_send"] = opt.sb_send ? 1 : 0;          // 1=哨兵帧，0=普通帧

    // ---- 云台反馈（yaw/pitch 为 deg，角速度为 rad/s）----
    data["gimbal_yaw"] = gs.yaw;              // 云台 yaw，deg
    data["gimbal_yaw_vel"] = gs.yaw_vel;      // 云台 yaw 角速度，rad/s
    data["gimbal_pitch"] = gs.pitch;          // 云台 pitch，deg
    data["gimbal_pitch_vel"] = gs.pitch_vel;  // 云台 pitch 角速度，rad/s
    data["q2yaw"] = gs.q2yaw;                 // 保留字段，当前通常为 0
    data["q2pitch"] = gs.q2pitch;             // 保留字段，当前通常为 0

    // ---- Planner 内部目标参考角 ----
    data["target_yaw"] = plan.target_yaw;      // rbSuppressiveFire 下单位 deg
    data["target_pitch"] = plan.target_pitch;  // rbSuppressiveFire 下单位 rad

    // ---- Planner 实际下发 ----
    data["plan_mode"] = plan.control ? (plan.fire ? 2 : 1) : 0;  // 0/1/2
    data["plan_yaw"] = plan.yaw / CV_PI * 180.0;                 // 下发 yaw，deg
    data["plan_yaw_vel"] = plan.yaw_vel;                         // yaw 角速度，rad/s
    data["plan_yaw_acc"] = plan.yaw_acc;                         // yaw 角加速度，rad/s^2
    data["plan_pitch"] = plan.pitch * 57.3;                      // 下发 pitch，deg
    data["plan_pitch_vel"] = plan.pitch_vel;                     // pitch 角速度，rad/s
    data["plan_pitch_acc"] = plan.pitch_acc;                     // pitch 角加速度，rad/s^2
    data["fire"] = plan.fire ? 1 : 0;                            // 本帧是否允许开火
    data["fired"] = fired ? 1 : 0;                               // 本帧是否实际发弹
    data["plan_ms"] = plan_ms;                                   // 规划耗时，ms

    // ---- 目标/延迟/三份预测/选板诊断 ----
    append_debug_data(
      data, target, plan, planner, debug_params, target_age, speed_delay, pure, fly, full,
      detect_ms.load(), track_ms.load(), capture_fps.load());

    // ---- 下发节拍统计 ----
    data["send_late_us"] =
      std::chrono::duration_cast<std::chrono::microseconds>(lateness).count();  // 迟到，us
    data["send_missed"] = static_cast<double>(missed);  // 累计跳过的节拍数
    plotter.plot(data);
  };

  if (!watchdog.ready("Vision pipeline is ready")) {
    tools::logger()->warn("无法向 systemd 发送 READY 通知");
  }

  if (opt.multithread) {
    // ---------------- 多线程：异步推理 + 独立下发线程 ----------------
    const auto t0 = std::chrono::steady_clock::now();

    std::thread plan_thread([&] {
      tools::tighten_timer_slack();
      tools::PeriodicTimer tick(kSendPeriod);

      std::optional<auto_aim::Target> target = std::nullopt;
      std::uint16_t last_bullet_count = 0;
      bool was_buff = false;

      while (!quit.load()) {
        const auto gs = gimbal.state();
        const auto mode = gimbal.mode();

        if (rb_runtime::is_buff(mode)) {
          if (!was_buff) {
            rb_runtime::send_stop(gimbal, opt.sb_send, gs.yaw / 57.3F, gs.pitch / 57.3F);
            was_buff = true;
          }
          target.reset();
          tick.wait_next();
          continue;
        }
        was_buff = false;

        std::optional<auto_aim::Target> incoming;
        while (target_queue.try_pop(incoming)) target = std::move(incoming);

        const auto plan_start = std::chrono::steady_clock::now();
        const auto plan_now = std::chrono::steady_clock::now();

        double target_age = 0.0;  // 目标时间戳到当前时刻的年龄，s
        double speed_delay = debug_params.low_speed_delay_time;  // 发弹延迟，s
        if (target.has_value()) {
          target_age = std::max(0.0, tools::delta_time(plan_now, target->getTimePoint()));
          const auto ekf_x = target->ekf_x();
          // 按目标角速度选择高速/低速发弹延迟，与 Planner::plan() 一致
          speed_delay = std::abs(ekf_x[7]) > debug_params.decision_speed
                          ? debug_params.high_speed_delay_time
                          : debug_params.low_speed_delay_time;
        }

        auto plan = planner.plan(
          target, gs.bullet_speed, gs.yaw, auto_aim::Planner::ShootStrategy::rbSuppressiveFire);
        const auto plan_end = std::chrono::steady_clock::now();
        const double plan_ms =
          std::chrono::duration<double, std::milli>(plan_end - plan_start).count();

        const auto lateness = tick.wait_next();
        rb_runtime::send_plan(gimbal, plan, target, opt.sb_send);

        // 三份预测：pure=只按飞行时间；fly=年龄+飞行；full=年龄+发弹/云台延迟+飞行
        AimSolution pure;
        AimSolution fly;
        AimSolution full;
        if (target.has_value()) {
          pure = solve_aim(*target, gs.bullet_speed, 0.0, debug_params);
          fly = solve_aim(*target, gs.bullet_speed, target_age, debug_params);
          full = solve_aim(
            *target, gs.bullet_speed, target_age + speed_delay + debug_params.gimbal_delay,
            debug_params);
        }

        const bool fired = gs.bullet_count > last_bullet_count;
        last_bullet_count = gs.bullet_count;

        plot_plan(
          plan, target, gs, fired, plan_ms, target_age, speed_delay, pure, fly, full, lateness,
          tick.missed(), t0);
      }
    });

    int frame_count = 0;
    auto auto_aim_started_at = std::chrono::steady_clock::time_point{};
    auto last_mode = gimbal.mode();
    auto last_frame_time = std::chrono::steady_clock::time_point{};
    cv::Mat img;
    std::chrono::steady_clock::time_point t;

    while (!exiter.exit() && !quit.load()) {
      const auto mode = gimbal.mode();
      if (mode != last_mode) {
        tools::logger()->info("Switch to {}", gimbal.str(mode));
        if (rb_runtime::is_buff(mode)) target_queue.push(std::nullopt);
        if (!rb_runtime::is_buff(mode)) {
          auto_aim_started_at = std::chrono::steady_clock::now();
          target_queue.push(std::nullopt);
        }
        last_mode = mode;
      }

      camera.read(img, t);
      if (img.empty()) {
        tools::logger()->warn("Camera frame empty! Waiting...");
        std::this_thread::sleep_for(1ms);
        continue;
      }

      watchdog.ping();

      const auto frame_now = std::chrono::steady_clock::now();
      if (last_frame_time != std::chrono::steady_clock::time_point{}) {
        const double dt = tools::delta_time(frame_now, last_frame_time);
        if (dt > 1e-9) capture_fps.store(1.0 / dt);
      }
      last_frame_time = frame_now;

      if (rb_runtime::is_buff(mode)) {
        tools::draw_text(img, "BUFF mode - auto aim released", {20, 40}, {0, 0, 255}, 0.8);
        cv::resize(img, img, {}, 0.5, 0.5);
        cv::imshow("rb_auto_aim_debug", img);
        if (cv::waitKey(1) == 'q') break;
        continue;
      }

      const auto q = gimbal.q(t);

      const auto detect_start = std::chrono::steady_clock::now();
      auto yolo_frame = yolo.detect(auto_aim::YOLOFrameData(img, q, t), frame_count++);
      const auto detect_end = std::chrono::steady_clock::now();
      detect_ms.store(std::chrono::duration<double, std::milli>(detect_end - detect_start).count());

      if (yolo_frame.is_empty || yolo_frame.timestamp < auto_aim_started_at) continue;

      solver.set_R_gimbal2world(yolo_frame.gimbal_q);

      const auto track_start = std::chrono::steady_clock::now();
      auto targets = tracker.track(yolo_frame.armors, yolo_frame.timestamp);
      const auto track_end = std::chrono::steady_clock::now();
      track_ms.store(std::chrono::duration<double, std::milli>(track_end - track_start).count());

      img = yolo_frame.frame;
      if (!targets.empty()) {
        draw_target_debug(img, solver, targets.front(), std::nullopt);
        target_queue.push(std::optional<auto_aim::Target>(targets.front()));
      } else {
        target_queue.push(std::nullopt);
      }

      tools::draw_text(
        img, fmt::format("thread=1 sb_send={} fps={:.1f}", opt.sb_send ? 1 : 0, capture_fps.load()),
        {20, 40}, {0, 255, 255}, 0.8);
      cv::resize(img, img, {}, 0.5, 0.5);
      cv::imshow("rb_auto_aim_debug", img);
      const auto key = cv::waitKey(1);
      if (key == 'q' || key == 27) break;
    }

    quit = true;
    target_queue.close();
    if (plan_thread.joinable()) plan_thread.join();
  } else {
    // ---------------- 单线程：同步推理，规划与下发都在主循环 ----------------
    cv::Mat img;
    std::chrono::steady_clock::time_point t;
    const auto t0 = std::chrono::steady_clock::now();
    auto last_frame_time = t0;
    std::uint16_t last_bullet_count = 0;
    bool was_buff = rb_runtime::is_buff(gimbal.mode());

    while (!exiter.exit()) {
      const auto mode = gimbal.mode();
      camera.read(img, t);
      if (img.empty()) {
        tools::logger()->warn("Camera frame empty! Waiting...");
        std::this_thread::sleep_for(1ms);
        continue;
      }

      watchdog.ping();

      const auto frame_now = std::chrono::steady_clock::now();
      const double dt = tools::delta_time(frame_now, last_frame_time);
      if (dt > 1e-9) capture_fps.store(1.0 / dt);
      last_frame_time = frame_now;

      const auto gs = gimbal.state();
      if (rb_runtime::is_buff(mode)) {
        if (!was_buff) {
          rb_runtime::send_stop(gimbal, opt.sb_send, gs.yaw / 57.3F, gs.pitch / 57.3F);
          was_buff = true;
        }
        tools::draw_text(img, "BUFF mode - auto aim released", {20, 40}, {0, 0, 255}, 0.8);
        cv::resize(img, img, {}, 0.5, 0.5);
        cv::imshow("rb_auto_aim_debug", img);
        const auto key = cv::waitKey(1);
        if (key == 'q' || key == 27) break;
        continue;
      }
      was_buff = false;

      const auto q = gimbal.q(t);

      const auto detect_start = std::chrono::steady_clock::now();
      auto armors = yolo.detect(img);
      const auto detect_end = std::chrono::steady_clock::now();
      detect_ms.store(std::chrono::duration<double, std::milli>(detect_end - detect_start).count());

      const auto track_start = std::chrono::steady_clock::now();
      auto targets = tracker.track(armors, t);
      const auto track_end = std::chrono::steady_clock::now();
      track_ms.store(std::chrono::duration<double, std::milli>(track_end - track_start).count());

      std::optional<auto_aim::Target> target = std::nullopt;
      if (!targets.empty()) target = targets.front();

      const auto plan_start = std::chrono::steady_clock::now();
      double target_age = 0.0;  // 目标时间戳到当前时刻的年龄，s
      double speed_delay = debug_params.low_speed_delay_time;  // 发弹延迟，s
      if (target.has_value()) {
        target_age = std::max(0.0, tools::delta_time(plan_start, target->getTimePoint()));
        const auto ekf_x = target->ekf_x();
        // 按目标角速度选择高速/低速发弹延迟，与 Planner::plan() 一致
        speed_delay = std::abs(ekf_x[7]) > debug_params.decision_speed
                        ? debug_params.high_speed_delay_time
                        : debug_params.low_speed_delay_time;
      }

      auto plan = planner.plan(
        target, gs.bullet_speed, gs.yaw, auto_aim::Planner::ShootStrategy::rbSuppressiveFire);
      const auto plan_end = std::chrono::steady_clock::now();
      const double plan_ms =
        std::chrono::duration<double, std::milli>(plan_end - plan_start).count();

      rb_runtime::send_plan(gimbal, plan, target, opt.sb_send);

      // 三份预测：pure=只按飞行时间；fly=年龄+飞行；full=年龄+发弹/云台延迟+飞行
      AimSolution pure;
      AimSolution fly;
      AimSolution full;
      if (target.has_value()) {
        pure = solve_aim(*target, gs.bullet_speed, 0.0, debug_params);
        fly = solve_aim(*target, gs.bullet_speed, target_age, debug_params);
        full = solve_aim(
          *target, gs.bullet_speed, target_age + speed_delay + debug_params.gimbal_delay,
          debug_params);
      }

      const bool fired = gs.bullet_count > last_bullet_count;
      last_bullet_count = gs.bullet_count;

      plot_plan(
        plan, target, gs, fired, plan_ms, target_age, speed_delay, pure, fly, full,
        std::chrono::steady_clock::duration::zero(), 0, t0);

      if (target.has_value() && plan.control) {
        draw_target_debug(img, solver, *target, planner.debug_xyza);
      }
      tools::draw_text(
        img,
        fmt::format(
          "thread=0 sb_send={} fps={:.1f} detect={:.2f}ms track={:.2f}ms", opt.sb_send ? 1 : 0,
          capture_fps.load(), detect_ms.load(), track_ms.load()),
        {20, 40}, {0, 255, 255}, 0.8);
      cv::resize(img, img, {}, 0.5, 0.5);
      cv::imshow("rb_auto_aim_debug", img);

      const auto key = cv::waitKey(1);
      if (key == 'q' || key == 27) break;
    }
  }

  // 退出前释放控制：让下位机按当前角度保持
  const auto final_state = gimbal.state();
  rb_runtime::send_stop(gimbal, opt.sb_send, final_state.yaw / 57.3F, final_state.pitch / 57.3F);

  return 0;
}
