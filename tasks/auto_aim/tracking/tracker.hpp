#ifndef AUTO_AIM__TRACKER_HPP
#define AUTO_AIM__TRACKER_HPP

#include <Eigen/Dense>
#include <yaml-cpp/yaml.h>

#include <chrono>
#include <list>
#include <optional>
#include <string>

#include "io/gimbal/gimbal.hpp"
#include "../model/armor.hpp"
#include "../model/armor_interfaces.hpp"
#include "center_acceleration_estimator.hpp"
#include "target.hpp"
#include "tasks/omniperception/detection.hpp"
#include "tools/thread_safe_queue.hpp"

namespace tools
{
class FFTExample;
}

namespace auto_aim
{
class Solver;

class Tracker
{
public:
  /** @brief 使用具体求解器构造目标跟踪器 @param config_path YAML 配置文件路径 @param solver 非拥有求解器指针 */
  Tracker(const std::string & config_path, Solver * solver);
  /** @brief 使用位姿求解接口构造目标跟踪器 @param config_path YAML 配置文件路径 @param solver 非拥有求解器接口指针 */
  Tracker(const std::string & config_path, IArmorPoseSolver * solver);

  /** @brief 获取跟踪状态名称 @return 状态字符串 */
  std::string state() const;

  /** @brief 重置跟踪状态和当前目标 */
  void reset();

  /** @brief 执行哨兵目标跟踪 @param armors 当前帧装甲板列表，函数会原地过滤和求解 @param t 帧时间戳 @param cam_is_short 是否来自短焦相机 @param use_enemy_color 是否过滤敌方颜色 @return 当前有效目标列表 */
  std::list<Target> sb_track(
    std::list<Armor> & armors, std::chrono::steady_clock::time_point t,
    bool cam_is_short = true,
    bool use_enemy_color = true);

  /** @brief 执行普通目标跟踪 @param armors 当前帧装甲板列表，函数会原地过滤和求解 @param t 帧时间戳 @param cam_is_short 是否来自短焦相机 @param use_enemy_color 是否过滤敌方颜色 @return 当前有效目标列表 */
  std::list<Target> track(
    std::list<Armor> & armors, std::chrono::steady_clock::time_point t, 
    bool cam_is_short = true,
    bool use_enemy_color = true);

  /** @brief 执行测试用目标跟踪流程 @param armors 当前帧装甲板列表 @param t 帧时间戳 @param cam_is_short 是否来自短焦相机 @param use_enemy_color 是否过滤敌方颜色 @return 当前有效目标列表 */
  std::list<Target> test_track(
    std::list<Armor> & armors, std::chrono::steady_clock::time_point t, 
    bool cam_is_short = true,
    bool use_enemy_color = true);


  /** @brief 融合全向感知与自瞄检测结果进行跟踪 @param detection_queue 全向感知结果 @param armors 自瞄装甲板列表 @param t 帧时间戳 @param use_enemy_color 是否过滤敌方颜色 @return 选中的全向检测结果和目标列表 */
  std::tuple<omniperception::DetectionResult, std::list<Target>> track(
    const std::vector<omniperception::DetectionResult> & detection_queue, std::list<Armor> & armors,
    std::chrono::steady_clock::time_point t, bool use_enemy_color = true);

  /** @brief 切换到具体求解器 @param solver 非拥有求解器指针 */
  void setSolver(Solver * solver);
  /** @brief 切换到位姿求解接口 @param solver 非拥有求解器接口指针 */
  void setPoseSolver(IArmorPoseSolver * solver);
  /** @brief 绑定云台状态源 @param gimbal 非拥有云台指针 */
  void set_gimbal(io::Gimbal* gimbal) { gimbal_ = gimbal; }
  /** @brief 绑定周期运动分析器 @param fft 非拥有分析器指针 */
  void set_fft(tools::FFTExample * fft);
  /** @brief 获取当前目标累计更新次数 @return 更新次数 */
  inline size_t get_update_count(){return this->target_.update_count_;}
  /** @brief 获取最近一帧对目标滤波器执行的校正次数 @return 校正次数（0 表示本帧未校正） */
  inline int last_update_count() const { return last_update_count_; }
  /** @brief 查询 UV 观测是否开启 @return 开启返回 true */
  inline bool uv_enabled() const { return uv_config_.enabled; }
  /**
   * @brief 运行时切换 UV / 传统观测（调试与实车 A/B 用）
   * @param enabled 是否使用 UV 观测
   * @note 两种观测模式的滤波状态不通用，切换后会 reset()，下一帧用 PnP 重新初始化
   */
  void set_uv_enabled(bool enabled);
private:
  IArmorPoseSolver * solver_;
  io::Gimbal* gimbal_ = nullptr; // 新增一个云台指针，默认为空
  tools::FFTExample * fft_ = nullptr;  // non-owning
  Color enemy_color_;
  std::string enemy_color_str_;
  int min_detect_count_;
  int max_temp_lost_count_;
  int detect_count_;
  int temp_lost_count_;
  int outpost_max_temp_lost_count_;
  int normal_temp_lost_count_;
  std::string state_, pre_state_;
  Target target_;
  CenterAccelerationEstimator center_acceleration_estimator_;
  std::chrono::steady_clock::time_point last_timestamp_;
  ArmorPriority omni_target_priority_;
  std::optional<uint8_t> last_mode_;
  bool cam_is_switch = false, last_cam_is_short = true;
  /** 多装甲板融合开关：同一帧内把多块匹配装甲板都送入滤波器 */
  bool multi_armor_fusion_ = false;
  /** 多装甲板融合时第二块及之后装甲板的马氏距离门限；<= 0 表示不限制 */
  double multi_armor_gate_ = 20.0;
  /** 多装甲板融合时第二块及之后装甲板朝向观测的噪声放大系数 */
  double multi_armor_angle_sigma_scale_ = 10.0;
  /** 观测噪声标准差覆盖值；<= 0 表示沿用 EKF 内部启发式 */
  double meas_azimuth_sigma_ = -1.0;
  double meas_distance_sigma_ = -1.0;
  double meas_angle_sigma_ = -1.0;
  /** UV 观测配置（enabled=false 时沿用传统世界系 ypd 观测） */
  UvConfig uv_config_{};
  /** 单装甲退化保护：UV 模式下半径先验标准差覆盖值 (m)；<= 0 不覆盖 */
  double uv_radius_prior_sigma_ = -1.0;
  /** 本帧成功执行的 EKF 校正次数 */
  int last_update_count_ = 0;
  /** UV 观测因缺少相机几何而回退传统路径的告警是否已打印（每轮跟踪只提示一次） */
  bool uv_fallback_warned_ = false;

  /** @brief 根据本帧是否匹配目标推进跟踪状态机 @param found 是否找到匹配装甲板 */
  void state_machine(bool found);

  /** @brief 从装甲板列表初始化新目标 @param armors 候选装甲板列表 @param t 帧时间戳 @return 成功建立目标时返回 true */
  bool set_target(std::list<Armor> & armors, std::chrono::steady_clock::time_point t);

  /** @brief 使用候选装甲板更新当前目标 @param armors 候选装甲板列表 @param t 帧时间戳 @return 成功匹配时返回 true */
  bool update_target(std::list<Armor> & armors, std::chrono::steady_clock::time_point t);

  /** @brief 把配置中的观测噪声标准差应用到当前目标 */
  void apply_measurement_sigmas();

  /** @brief 把 UV 配置与当前相机几何应用到当前目标 */
  void apply_uv_config();

  /** @brief 解析 yaml 中的 UV 配置 */
  void load_uv_config(const YAML::Node & yaml);

  /** @brief 重置周期运动采样状态 */
  void reset_fft_sample_state();
  /** @brief 向周期运动分析器添加装甲板样本 @param armor 匹配装甲板（其 xyz_in_world 须已由 PnP 写入） @param t 帧时间戳 */
  void update_fft_sample(const Armor & armor, std::chrono::steady_clock::time_point t);

  /**
   * @brief UV 观测模式下向周期运动分析器添加样本
   * @param t 帧时间戳
   * @note UV 模式逐帧不跑 PnP，armor.xyz_in_world 不会被写入（恒为 0），
   *       因此改用滤波器估计的当前装甲板高度；否则 FFT 拿到全 0 的 z 序列，
   *       周期性 z 加速度前馈会被静默关闭
   */
  void update_fft_sample_from_filter(std::chrono::steady_clock::time_point t);

  /** @brief 更新相机模式；相机切换时清空中心加速度历史 */
  void update_camera_mode(bool cam_is_short);

  /** @brief 当前目标是否允许使用整车中心平移加速度前馈 */
  bool use_center_acceleration() const;

  
};

}  // namespace auto_aim

#endif  // AUTO_AIM__TRACKER_HPP
