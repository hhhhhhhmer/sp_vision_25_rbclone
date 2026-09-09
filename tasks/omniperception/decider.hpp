#ifndef OMNIPERCEPTION__DECIDER_HPP
#define OMNIPERCEPTION__DECIDER_HPP

#include <Eigen/Dense>  // 必须在opencv2/core/eigen.hpp上面
#include <iostream>
#include <list>
#include <unordered_map>

// #include "armor.hpp"
#include "detection.hpp"
#include "io/camera.hpp"
#include "io/command.hpp"
#include "io/gimbal/gimbal.hpp"
#include "io/usbcamera/usbcamera.hpp"
#include "tasks/auto_aim/armor.hpp"
#include "tasks/auto_aim/target.hpp"
#include "tasks/auto_aim/yolo.hpp"
#include "tasks/auto_aim/solver.hpp"

namespace omniperception
{
class Decider
{
public:
  Decider(const std::string & config_path);

  io::VisionToGimbal decide_g(
    auto_aim::YOLO & yolo, const Eigen::Vector3d & gimbal_pos, io::Camera & omn_cam1_l,
    io::Camera & omn_cam2_r, const auto_aim::Solver & left_solver,
    const auto_aim::Solver & right_solver, float * target_distance = nullptr);
  
  io::Command decide(
  auto_aim::YOLO & yolo, const Eigen::Vector3d & gimbal_pos, io::USBCamera & usbcam1,
  io::USBCamera & usbcam2, io::Camera & back_camera);

  io::Command decide(
    auto_aim::YOLO & yolo, const Eigen::Vector3d & gimbal_pos, io::Camera & back_cammera);

  io::Command decide(const std::vector<DetectionResult> & detection_queue);

  Eigen::Vector2d delta_angle(
    const std::list<auto_aim::Armor> & armors, const std::string & camera);

  Eigen::Vector2d delta_angle_3d(
     std::list<auto_aim::Armor> & armors, const std::string & camera, 
            const auto_aim::Solver & left_solver, const auto_aim::Solver & right_solver  );

  bool armor_filter(std::list<auto_aim::Armor> & armors);

  bool not_base_armor_filter(std::list<auto_aim::Armor> & armors);

  void set_priority(std::list<auto_aim::Armor> & armors);
  //对队列中的每一个DetectionResult进行过滤，同时将DetectionResult排序
  void sort(std::vector<DetectionResult> & detection_queue);

  Eigen::Vector4d get_target_info(
    const std::list<auto_aim::Armor> & armors, const std::list<auto_aim::Target> & targets);

  void get_invincible_armor(const std::vector<int8_t> & invincible_enemy_ids);

  void get_auto_aim_target(
    std::list<auto_aim::Armor> & armors, const std::vector<int8_t> & auto_aim_target);

  io::Gimbal* gimbal_ = nullptr; // 新增一个云台指针，默认为空

  void set_gimbal(io::Gimbal* gimbal) { gimbal_ = gimbal; }

  void set_mode(int mode){
    this->mode_ = mode;
  }

private:
  int img_width_;
  int img_height_;
  double fov_h_, new_fov_h_;
  double fov_v_, new_fov_v_;
  int mode_;
  int count_;
  int last_count_ = -1;
  io::VisionToGimbal last_vision_cmd; 

  auto_aim::Color enemy_color_;
  std::string enemy_color_str_;
  auto_aim::YOLO detector_;
  std::vector<auto_aim::ArmorName> invincible_armor_;  //无敌状态机器人编号,英雄为1，哨兵为6

  // 定义ArmorName到ArmorPriority的映射类型
  using PriorityMap = std::unordered_map<auto_aim::ArmorName, auto_aim::ArmorPriority>;

  const PriorityMap mode1 = {
    {auto_aim::ArmorName::one, auto_aim::ArmorPriority::second},
    {auto_aim::ArmorName::two, auto_aim::ArmorPriority::forth},
    {auto_aim::ArmorName::three, auto_aim::ArmorPriority::first},
    {auto_aim::ArmorName::four, auto_aim::ArmorPriority::first},
    {auto_aim::ArmorName::five, auto_aim::ArmorPriority::third},
    {auto_aim::ArmorName::sentry, auto_aim::ArmorPriority::third},
    {auto_aim::ArmorName::outpost, auto_aim::ArmorPriority::fifth},
    {auto_aim::ArmorName::base, auto_aim::ArmorPriority::fifth},
    {auto_aim::ArmorName::not_armor, auto_aim::ArmorPriority::fifth}};

  const PriorityMap mode2 = {
    {auto_aim::ArmorName::two, auto_aim::ArmorPriority::first},
    {auto_aim::ArmorName::one, auto_aim::ArmorPriority::second},
    {auto_aim::ArmorName::three, auto_aim::ArmorPriority::second},
    {auto_aim::ArmorName::four, auto_aim::ArmorPriority::second},
    {auto_aim::ArmorName::five, auto_aim::ArmorPriority::second},
    {auto_aim::ArmorName::sentry, auto_aim::ArmorPriority::third},
    {auto_aim::ArmorName::outpost, auto_aim::ArmorPriority::third},
    {auto_aim::ArmorName::base, auto_aim::ArmorPriority::third},
    {auto_aim::ArmorName::not_armor, auto_aim::ArmorPriority::third}};

  const PriorityMap mode3 = {
    {auto_aim::ArmorName::outpost, auto_aim::ArmorPriority::first},
    {auto_aim::ArmorName::base, auto_aim::ArmorPriority::second},
    {auto_aim::ArmorName::one, auto_aim::ArmorPriority::second},
    {auto_aim::ArmorName::two, auto_aim::ArmorPriority::second},
    {auto_aim::ArmorName::three, auto_aim::ArmorPriority::second},
    {auto_aim::ArmorName::four, auto_aim::ArmorPriority::third},
    {auto_aim::ArmorName::five, auto_aim::ArmorPriority::third},
    {auto_aim::ArmorName::sentry, auto_aim::ArmorPriority::third},
    {auto_aim::ArmorName::not_armor, auto_aim::ArmorPriority::third}};
};

enum PriorityMode
{
  MODE_ONE = 1,
  MODE_TWO,
  MODE_THREE
};

}  // namespace omniperception

#endif
