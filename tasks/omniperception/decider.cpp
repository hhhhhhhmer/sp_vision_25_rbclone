#include "decider.hpp"

#include <yaml-cpp/yaml.h>

#include <filesystem>
#include <opencv2/opencv.hpp>

#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/img_tools.hpp"
#include "tasks/auto_aim/armor.hpp"

namespace omniperception
{
Decider::Decider(const std::string & config_path) : detector_(config_path), count_(0)
{
  auto yaml = YAML::LoadFile(config_path);
  img_width_ = yaml["image_width"].as<double>();
  img_height_ = yaml["image_height"].as<double>();
  fov_h_ = yaml["fov_h"].as<double>();
  fov_v_ = yaml["fov_v"].as<double>();
  new_fov_h_ = yaml["new_fov_h"].as<double>();
  new_fov_v_ = yaml["new_fov_v"].as<double>();
  enemy_color_ =
    (yaml["enemy_color"].as<std::string>() == "red") ? auto_aim::Color::red : auto_aim::Color::blue;
  enemy_color_str_ = yaml["enemy_color"].as<std::string>();
  mode_ = yaml["mode"].as<double>();
}

io::VisionToGimbal Decider::decide_g(
  auto_aim::YOLO & yolo, const Eigen::Vector3d & gimbal_pos,
   io::Camera & omn_cam1_l, io::Camera & omn_cam2_r,
  const auto_aim::Solver & left_solver, const auto_aim::Solver & right_solver,
  float * target_distance
  )
{
  if (target_distance != nullptr) *target_distance = 0.0f;

  Eigen::Vector2d delta_angle;
  io::Camera * cams[] = {&omn_cam1_l, &omn_cam2_r};

  cv::Mat omn_img;
  std::chrono::steady_clock::time_point timestamp;
  if (count_ < 0 || count_ > 2) {
    throw std::runtime_error("count_ out of valid range [0,2]");
  }
  
  io::VisionToGimbal vision_cmd;
  vision_cmd.mode = 0;  // 不控制
  vision_cmd.yaw = 0.0f;
  vision_cmd.yaw_vel = 0.0f;
  vision_cmd.yaw_acc = 0.0f;
  vision_cmd.pitch = 0.0f;
  vision_cmd.pitch_vel = 0.0f;
  vision_cmd.pitch_acc = 0.0f;    
  last_vision_cmd = vision_cmd;
  
  int camera_num = 2;
  bool read_full = cams[count_]->try_read(omn_img, timestamp);
  if(!read_full){
    count_ = (count_ + 1) % camera_num;
    // to
  }
  if(!read_full && !cams[count_]->try_read(omn_img, timestamp)){
    count_ = (count_ + 1) % camera_num;
    tools::logger()->info("[omniperception::Decider] 感知相机均无img");
    return vision_cmd;
  }


  auto armors = yolo.detect(omn_img);
  auto empty = armor_filter(armors);

  for(auto armor : armors){
    auto image_points = armor.points;
    tools::draw_points(omn_img, image_points, {235, 206, 135});    
  }

  // tools::logger()->info("[omniperception::Decider] 1111");

  if(!empty){
    delta_angle = this->delta_angle_3d(armors, cams[count_]->main_and_secondary, left_solver, right_solver);
    const auto distance = static_cast<float>(armors.front().xyz_in_gimbal.norm());
    if (target_distance != nullptr) *target_distance = distance;
    

    tools::logger()->debug(
      "[{} camera] delta yaw:{:.2f},target pitch:{:.2f},distance:{:.2f}m,armor number:{},armor name:{}",
      ( cams[count_]->main_and_secondary), delta_angle[0]*57.3, delta_angle[1]*57.3,
      distance, armors.size(), auto_aim::ARMOR_NAMES[armors.front().name]);


      
    if(abs(delta_angle[0]) < 95/57.3){
      delta_angle[0] = delta_angle[0] > 0 ? 95/57.3 : -95/57.3;
    }
    
    vision_cmd.mode = 3;  // 全向感知模式识别到目标，控制大云台
    vision_cmd.yaw = -delta_angle[0];
    vision_cmd.yaw_vel = 0.0f;  // 角速度设为0，可根据需要计算
    vision_cmd.yaw_acc = 0.0f;  // 角加速度设为0
    vision_cmd.pitch = tools::limit_rad(delta_angle[1]);
    vision_cmd.pitch_vel = 0.0f;
    vision_cmd.pitch_acc = 0.0f;

    last_count_ = count_;
    last_vision_cmd = vision_cmd;
    
  }

  count_ = (count_ + 1) % 2;

  return vision_cmd;
  // if (!empty && last_vision_cmd.mode == 0) {
    
  //   delta_angle = this->delta_angle(armors, cams[count_]->main_and_secondary);
    

  //   tools::logger()->debug(
  //     "[{} camera] delta yaw:{:.2f},target pitch:{:.2f},armor number:{},armor name:{}",
  //     ( cams[count_]->main_and_secondary), delta_angle[0], delta_angle[1],
  //     armors.size(), auto_aim::ARMOR_NAMES[armors.front().name]);

    
  //   vision_cmd.mode = 3;  // 全向感知模式识别到目标，控制大云台
  //   vision_cmd.yaw = static_cast<float>(delta_angle[0] / 57.3);
  //   vision_cmd.yaw_vel = 0.0f;  // 角速度设为0，可根据需要计算
  //   vision_cmd.yaw_acc = 0.0f;  // 角加速度设为0
  //   vision_cmd.pitch = tools::limit_rad((delta_angle[1] + 15 )/ 57.3);
  //   vision_cmd.pitch_vel = 0.0f;
  //   vision_cmd.pitch_acc = 0.0f;

  //   last_count_ = count_;
  //   last_vision_cmd = vision_cmd;
    
  //   return vision_cmd;
  // }else if((empty && last_vision_cmd.mode == 3) 
  //  || (last_vision_cmd.mode == 3 && !empty) )
  // {
  //   return last_vision_cmd;
  // }
  // else if(last_vision_cmd.mode == 0 && empty){
    
  //   // 如果没有找到目标，返回不控制的指令
  //   vision_cmd.mode = 0;  // 不控制
  //   vision_cmd.yaw = 0.0f;
  //   vision_cmd.yaw_vel = 0.0f;
  //   vision_cmd.yaw_acc = 0.0f;
  //   vision_cmd.pitch = 0.0f;
  //   vision_cmd.pitch_vel = 0.0f;
  //   vision_cmd.pitch_acc = 0.0f;    
  //   last_vision_cmd = vision_cmd;

  //   tools::logger()->debug("全向感知未识别到目标");
  //   return vision_cmd;
  // }else {
  //   tools::logger()->debug("全向感知，发生其他情况，需排除");
  //   return last_vision_cmd;
  // }
  





 
  
}

io::Command Decider::decide(
  auto_aim::YOLO & yolo, const Eigen::Vector3d & gimbal_pos, io::USBCamera & usbcam1,
  io::USBCamera & usbcam2, io::Camera & back_camera)
{
  Eigen::Vector2d delta_angle;
  io::USBCamera * cams[] = {&usbcam1, &usbcam2};

  cv::Mat usb_img;
  std::chrono::steady_clock::time_point timestamp;
  if (count_ < 0 || count_ > 2) {
    throw std::runtime_error("count_ out of valid range [0,2]");
  }
  if (count_ == 2) {
    back_camera.read(usb_img, timestamp);
  } else {
    cams[count_]->read(usb_img, timestamp);
  }
  auto armors = yolo.detect(usb_img);
  auto empty = armor_filter(armors);

  if (!empty) {
    if (count_ == 2) {
      delta_angle = this->delta_angle(armors, "back");
    } else {
      delta_angle = this->delta_angle(armors, cams[count_]->device_name);
    }

    tools::logger()->debug(
      "[{} camera] delta yaw:{:.2f},target pitch:{:.2f},armor number:{},armor name:{}",
      (count_ == 2 ? "back" : cams[count_]->device_name), delta_angle[0], delta_angle[1],
      armors.size(), auto_aim::ARMOR_NAMES[armors.front().name]);

    count_ = (count_ + 1) % 3;

    return io::Command{
      true, false, tools::limit_rad(gimbal_pos[0] + delta_angle[0] / 57.3),
      tools::limit_rad(delta_angle[1] / 57.3)};
  }

  count_ = (count_ + 1) % 3;
  // 如果没有找到目标，返回默认命令
  return io::Command{false, false, 0, 0};
}

io::Command Decider::decide(
  auto_aim::YOLO & yolo, const Eigen::Vector3d & gimbal_pos, io::Camera & back_cammera)
{
  cv::Mat img;
  std::chrono::steady_clock::time_point timestamp;
  back_cammera.read(img, timestamp);
  auto armors = yolo.detect(img);
  auto empty = armor_filter(armors);

  if (!empty) {
    auto delta_angle = this->delta_angle(armors, "back");
    tools::logger()->debug(
      "[back camera] delta yaw:{:.2f},target pitch:{:.2f},armor number:{},armor name:{}",
      delta_angle[0], delta_angle[1], armors.size(), auto_aim::ARMOR_NAMES[armors.front().name]);

    return io::Command{
      true, false, tools::limit_rad(gimbal_pos[0] + delta_angle[0] / 57.3),
      tools::limit_rad(delta_angle[1] / 57.3)};
  }

  return io::Command{false, false, 0, 0};
}

io::Command Decider::decide(const std::vector<DetectionResult> & detection_queue)
{
  if (detection_queue.empty()) {
    return io::Command{false, false, 0, 0};
  }

  DetectionResult dr = detection_queue.front();
  if (dr.armors.empty()) return io::Command{false, false, 0, 0};
  tools::logger()->info(
    "omniperceptron find {},delta yaw is {:.4f}", auto_aim::ARMOR_NAMES[dr.armors.front().name],
    dr.delta_yaw * 57.3);

  return io::Command{true, false, dr.delta_yaw, dr.delta_pitch};
};

Eigen::Vector2d Decider::delta_angle(
  const std::list<auto_aim::Armor> & armors, const std::string & camera)
{
  Eigen::Vector2d delta_angle;

  //TUDO:计算大yaw旋转角度
  if (camera == "left") {
    delta_angle[0] = 120 + (new_fov_h_ / 2) - armors.front().center_norm.x * new_fov_h_;
    tools::logger()->info("left");
    // delta_angle[0] = 120;
    delta_angle[1] = armors.front().center_norm.y * new_fov_v_ - new_fov_v_ / 2;
    return delta_angle;        
  }

  else if (camera == "right") {
    delta_angle[0] = -120 + (new_fov_h_ / 2) - armors.front().center_norm.x * new_fov_h_;
      tools::logger()->info("right");
    // delta_angle[0] = 120;
    delta_angle[1] = armors.front().center_norm.y * new_fov_v_ - new_fov_v_ / 2;
    return delta_angle;
  }

  else {
    delta_angle[0] = 170 + (54.2 / 2) - armors.front().center_norm.x * 54.2;
    delta_angle[1] = armors.front().center_norm.y * 44.5 - 44.5 / 2;
    return delta_angle;
  }

}


Eigen::Vector2d Decider::delta_angle_3d(
    std::list<auto_aim::Armor> & armors, const std::string & camera, const auto_aim::Solver & left_solver, const auto_aim::Solver & right_solver ){
     
  Eigen::Vector2d delta_angle;
  if(armors.empty()){
    tools::logger()->debug("[Decider] armors 为空，有bug！！！");
  }
  


  //TUDO:计算大yaw旋转角度
  if (camera == "left") {
    left_solver.omn_dig_yaw_solve(armors.front(), Eigen::Vector3d(0,0,-(105. * CV_PI / 180.0)), Eigen::Vector3d(-0.127611, -0.136932, 0.16) );
    auto xyz = armors.front().xyz_in_gimbal;
    tools::logger()->info("omn_xyz :x{}, y{} ,z{}", xyz(0), xyz(1), xyz(2));
    auto ypd_angle = 145 /57.3 - std::atan2(xyz(0), xyz(1));
    delta_angle[0] = ypd_angle;
    delta_angle[1] =std::atan2(xyz(2), std::sqrt(xyz(0) * xyz(0) + xyz(1) * xyz(1))); 
    return delta_angle;        
  }

  else if (camera == "right") {
    right_solver.omn_dig_yaw_solve(armors.front(), Eigen::Vector3d(0,0, -(105. * CV_PI / 180.0)), Eigen::Vector3d(-0.127611, 0.136932, 0.16) );
    auto xyz = armors.front().xyz_in_gimbal;
    tools::logger()->info("omn_xyz :x{}, y{} ,z{}", xyz(0), xyz(1), xyz(2));
    auto ypd_angle = -120/57.3 - std::atan2(xyz(0), xyz(1));
    delta_angle[0] = ypd_angle;
    delta_angle[1] = std::atan2(xyz(2), std::sqrt(xyz(0) * xyz(0) + xyz(1) * xyz(1))); 
    return delta_angle; 
  }

  else {
    tools::logger()->debug("[Decider] left 和 right打错字了");
    delta_angle[0] = 170 + (54.2 / 2) - armors.front().center_norm.x * 54.2;
    delta_angle[1] = armors.front().center_norm.y * 44.5 - 44.5 / 2;
    return delta_angle;
  }

}


bool Decider::armor_filter(std::list<auto_aim::Armor> & armors)
{
  if (armors.empty()) return true;


  if(gimbal_ == nullptr) {
    tools::logger()->error("[omniperception::Decider] gimbal_不能为空指针，请先调用set_gimbal()设置云台指针");
    return {};
  }
  io::GimbalState g = gimbal_->state();
  if(enemy_color_str_ == "auto") enemy_color_ = (g.enemy_color == 0) ?   auto_aim::Color::red : auto_aim::Color::blue;

  // 过滤非敌方装甲板
  armors.remove_if([&](const auto_aim::Armor & a) { return a.color != enemy_color_; });



  // 25赛季没有5号装甲板
  armors.remove_if([&](const auto_aim::Armor & a) { return a.name == auto_aim::ArmorName::five; });
  // 不打工程
  // armors.remove_if([&](const auto_aim::Armor & a) { return a.name == auto_aim::ArmorName::two; });
  // // 不打前哨站
  // armors.remove_if(
  //   [&](const auto_aim::Armor & a) { return a.name == auto_aim::ArmorName::outpost; });

  // 过滤掉刚复活无敌的装甲板
  armors.remove_if([&](const auto_aim::Armor & a) {
    return std::find(invincible_armor_.begin(), invincible_armor_.end(), a.name) !=
           invincible_armor_.end();
  });

  return armors.empty();
}

bool Decider::not_base_armor_filter(std::list<auto_aim::Armor> & armors)
{
  if (armors.empty()) return true;


  if(gimbal_ == nullptr) {
    tools::logger()->error("[omniperception::Decider] gimbal_不能为空指针，请先调用set_gimbal()设置云台指针");
    return {};
  }
  io::GimbalState g = gimbal_->state();
  if(enemy_color_str_ == "auto") enemy_color_ = (g.enemy_color == 0) ?   auto_aim::Color::red : auto_aim::Color::blue;

  // 过滤非基地装甲的装甲板
  armors.remove_if([&](const auto_aim::Armor & a) { return a.name != auto_aim::ArmorName::base; });

  return armors.empty();
}


void Decider::set_priority(std::list<auto_aim::Armor> & armors)
{
  if (armors.empty()) return;

  // const= (mode_ == MODE_ONE) ? mode1 : mode2;
  PriorityMap  priority_map ;
  switch(mode_){
    case MODE_ONE: priority_map = mode1;
    break;
    case MODE_TWO: priority_map = mode2;
    break;
    case MODE_THREE: priority_map = mode3;
  }

  if (!armors.empty()) {
    for (auto & armor : armors) {
      armor.priority = priority_map.at(armor.name);
    }
  }
}

void Decider::sort(std::vector<DetectionResult> & detection_queue)
{
  if (detection_queue.empty()) return;

  // 对每个 DetectionResult 调用 armor_filter 和 set_priority
  for (auto & dr : detection_queue) {
    armor_filter(dr.armors);
    set_priority(dr.armors);

    // 对每个 DetectionResult 中的 armors 进行排序
    dr.armors.sort(
      [](const auto_aim::Armor & a, const auto_aim::Armor & b) { return a.priority < b.priority; });
  }

  // 根据优先级对 DetectionResult 进行排序
  std::sort(
    detection_queue.begin(), detection_queue.end(),
    [](const DetectionResult & a, const DetectionResult & b) {
      return a.armors.front().priority < b.armors.front().priority;
    });
}

Eigen::Vector4d Decider::get_target_info(
  const std::list<auto_aim::Armor> & armors, const std::list<auto_aim::Target> & targets)
{
  if (armors.empty() || targets.empty()) return Eigen::Vector4d::Zero();

  auto target = targets.front();

  for (const auto & armor : armors) {
    if (armor.name == target.name) {
      return Eigen::Vector4d{
        armor.xyz_in_gimbal[0], armor.xyz_in_gimbal[1], 1,
        static_cast<double>(armor.name) + 1};  //避免歧义+1(详见通信协议)
    }
  }

  return Eigen::Vector4d::Zero();
}

void Decider::get_invincible_armor(const std::vector<int8_t> & invincible_enemy_ids)
{
  invincible_armor_.clear();

  if (invincible_enemy_ids.empty()) return;

  for (const auto & id : invincible_enemy_ids) {
    tools::logger()->info("invincible armor id: {}", id);
    invincible_armor_.push_back(auto_aim::ArmorName(id - 1));
  }
}

void Decider::get_auto_aim_target(
  std::list<auto_aim::Armor> & armors, const std::vector<int8_t> & auto_aim_target)
{
  if (auto_aim_target.empty()) return;

  std::vector<auto_aim::ArmorName> auto_aim_targets;

  for (const auto & target : auto_aim_target) {
    if (target <= 0 || static_cast<size_t>(target) > auto_aim::ARMOR_NAMES.size()) {
      tools::logger()->warn("Received invalid auto_aim target value: {}", int(target));
      continue;
    }
    auto_aim_targets.push_back(static_cast<auto_aim::ArmorName>(target - 1));
    tools::logger()->info("nav send auto_aim target is {}", auto_aim::ARMOR_NAMES[target - 1]);
  }

  if (auto_aim_targets.empty()) return;

  armors.remove_if([&](const auto_aim::Armor & a) {
    return std::find(auto_aim_targets.begin(), auto_aim_targets.end(), a.name) ==
           auto_aim_targets.end();
  });
}

}  // namespace omniperceptionADFSafsdL
