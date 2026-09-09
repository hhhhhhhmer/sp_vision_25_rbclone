#include "target.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>

#include "tools/math_tools.hpp"

namespace auto_aim
{

Target::Target()
: name(ArmorName::not_armor),
  armor_type(ArmorType::small),
  priority(ArmorPriority::fifth),
  jumped(false),
  last_id(0),
  xyz_in_world(Eigen::Vector3d::Zero()),
  update_count_(0),
  armor_num_(0),
  switch_count_(0),
  is_switch_(false),
  is_converged_(false),
  t_(std::chrono::steady_clock::now())
{
}

Target::Target(
  const Armor & armor, std::chrono::steady_clock::time_point t, double radius, int armor_num,
  Eigen::VectorXd P0_dig)
: name(armor.name),
  armor_type(armor.type),
  priority(armor.priority),
  jumped(false),
  last_id(0),
  update_count_(0),
  armor_num_(armor_num),
  switch_count_(0),
  is_switch_(false),
  is_converged_(false),
  t_(t)
{
  auto r = radius;
  const Eigen::VectorXd & xyz = armor.xyz_in_world;
  const Eigen::VectorXd & ypr = armor.ypr_in_world;

  // 根据当前装甲板位置和半径，反推旋转中心的坐标
  auto center_x = xyz[0] + r * std::cos(ypr[0]);
  auto center_y = xyz[1] + r * std::sin(ypr[0]);
  auto center_z = xyz[2];

 if(name == ArmorName::outpost){
  tower_armor_hs[0].first = true;       // 标记 0 号位已成功初始化
  tower_armor_hs[0].second = center_z;  // 记录真实高度
  }

  // ==========================================
  // EKF 11维状态向量定义:
  // [0]x, [1]vx, [2]y, [3]vy, [4]z, [5]vz, 
  // [6]yaw(偏航角), [7]vyaw(自转角速度), 
  // [8]r(基础半径), [9]r_(半径补偿量), [10]z_(高度补偿量)
  // ==========================================
  Eigen::VectorXd x0 = Eigen::VectorXd::Zero(11);

  // 如果是前哨站，将 z_ (x[10]) 的初始值设为物理理论值
  double initial_dz =
    name == ArmorName::outpost ? RVfromFYT::kTowerArmorHeightStep : 0.0;

  x0 << center_x, 0, center_y, 0, center_z, 0, ypr[0], 0, r, 0, initial_dz;
  
  Eigen::MatrixXd P0 = Eigen::MatrixXd::Identity(11, 11) * 10.0;
  P0.block(0, 0, 11, 11) = P0_dig.asDiagonal();

  ekf_ = RVfromFYT(x0, P0, armor_num_, name);
  sync_tower_armor_heights();
}

// 供手动初始化使用的构造函数
Target::Target(double x, double vyaw, double radius, double h) 
: name(ArmorName::not_armor),
  armor_type(ArmorType::small),
  priority(ArmorPriority::fifth),
  jumped(false),
  last_id(0),
  update_count_(0),
  armor_num_(4),
  switch_count_(0),
  is_switch_(false),
  is_converged_(false),
  t_(std::chrono::steady_clock::now())
{
  Eigen::VectorXd x0 = Eigen::VectorXd::Zero(11);
  x0 << x, 0, 0, 0, 0, 0, 0, vyaw, radius, 0, h;

  Eigen::VectorXd P0_dig{{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}};
  Eigen::MatrixXd P0 = Eigen::MatrixXd::Identity(11, 11) * 10.0;
  P0.block(0, 0, 11, 11) = P0_dig.asDiagonal();

  ekf_ = RVfromFYT(x0, P0, armor_num_, name);
  sync_tower_armor_heights();
}

void Target::predict(std::chrono::steady_clock::time_point t)
{
  predict(t, Eigen::VectorXd::Zero(3));
}

void Target::predict(std::chrono::steady_clock::time_point t, Eigen::VectorXd u_xyz)
{
  if (t <= t_) return;
  const double dt = tools::delta_time(t, t_);
  predict(dt, std::move(u_xyz));
  // Avoid duration conversion rounding accumulating in the online timestamp path.
  t_ = t;
}

void Target::predict(double dt, Eigen::VectorXd u_xyz)
{
  if (ekf_.x.size() != RVfromFYT::kStateDimension) {
    throw std::logic_error("Target must be initialized before prediction");
  }
  if (!std::isfinite(dt)) {
    throw std::invalid_argument("Target::predict requires a finite dt");
  }
  if (u_xyz.size() < 3 || !u_xyz.head<3>().allFinite()) {
    throw std::invalid_argument("Target::predict requires a finite 3D acceleration");
  }
  if (dt == 0.0) return;

  const auto duration =
    std::chrono::duration_cast<std::chrono::steady_clock::duration>(
      std::chrono::duration<double>(dt));
  const auto prediction_time = t_ + duration;
  if (wave_ && wave_->valid() && this->name != ArmorName::outpost) {
    const auto midpoint = t_ + duration / 2;
    u_xyz[2] += wave_->get_acceleration(midpoint);
  }

  // rv predict
  auto rvFromFytpredict = [&]() -> void {
    double v1, v2;
    if (name == ArmorName::outpost) {
      ekf_.x[10] = RVfromFYT::kTowerArmorHeightStep;
      // 前哨站位置固定，收敛后极大限制平移噪声
      if (this->convergened()) {
          v1 = 0.1;  // 锁死 X, Y, Z 中心
      } else {
          v1 = 20;   // 允许前期寻找中心
      }
      v2 = 0.1;      // 允许自转速度存在微小波动
    }
    else {
      v1 = 100;
      v2 = 400;
    }

    // 前哨站收敛后限制最大转速防飞
    if (this->convergened() && this->name == ArmorName::outpost) {
      if (std::abs(this->ekf_.x[7]) > 2) this->ekf_.x[7] = this->ekf_.x[7] > 0 ? 2.51 : -2.51;
    }
    Eigen::VectorXd noises(2);
    noises << v1, v2;
    ekf_.kf_predict(dt, u_xyz.head<3>(), noises);
  };rvFromFytpredict();
  

  t_ = prediction_time;

  rv_residual = std::nullopt;
}

void Target::update(const Armor & armor)
{
  int id = 0;
  ekf_.prepare_measurement(armor, cam_is_short, update_count_);

  if (this->name == ArmorName::outpost) {
    // 【策略 A：前哨站专用】
    // 纯几何匹配(距离+复合角度)，绕开因高度阶梯跳变导致 EKF 协方差波动的干扰
    auto min_angle_error = 1e10;
    const std::vector<Eigen::Vector4d> & xyza_list = armor_xyza_list();

    ekf_.x[10] = RVfromFYT::kTowerArmorHeightStep;

    std::vector<std::pair<Eigen::Vector4d, int>> xyza_i_list;
    for (int i = 0; i < armor_num_; i++) {
      xyza_i_list.push_back({xyza_list[i], i});
    }

    // 按距离(ypd[2])由近及远排序
    std::sort(
      xyza_i_list.begin(), xyza_i_list.end(),
      [](const std::pair<Eigen::Vector4d, int> & a, const std::pair<Eigen::Vector4d, int> & b) {
        Eigen::Vector3d ypd1 = tools::xyz2ypd(a.first.head(3));
        Eigen::Vector3d ypd2 = tools::xyz2ypd(b.first.head(3));
        return ypd1[2] < ypd2[2];
      });

    // 只取最近的3个装甲板验证角度匹配度
    for (int i = 0; i < 3; i++) {
      const auto & xyza = xyza_i_list[i].first;
      Eigen::Vector3d ypd = tools::xyz2ypd(xyza.head(3));
      
      auto angle_error = std::abs(tools::limit_rad(armor.ypr_in_world[0] - xyza[3])) +
                         std::abs(tools::limit_rad(armor.ypd_in_world[0] - ypd[0]));

      if (std::abs(angle_error) < std::abs(min_angle_error)) {
        id = xyza_i_list[i].second;
        min_angle_error = angle_error;
      }
    }

  } else {
    id = ekf_.select_armor_id(last_id);
  }


  if (id != 0) jumped = true;

  // 检测换板事件
  if (id != last_id) {
    is_switch_ = true;
    switch_count_++;
    
    // 换板时，将上一块装甲板的历史累加数据计算为平均高度锚点
    if (name == ArmorName::outpost) {
      if (tower_armor_hs_datas_ptr[last_id] > 0) {
        tower_armor_hs[last_id].first = true; // 标记该装甲板已有有效的历史数据
        tower_armor_hs[last_id].second = tower_armor_hs_datas[last_id] / tower_armor_hs_datas_ptr[last_id];
      }
    }
  } else {
    is_switch_ = false;
  }

  // 累加当前块装甲板的高度特征
  if(name == ArmorName::outpost){
    double a = 0.1; // 互补滤波系数
    tower_armor_h = a * armor.xyz_in_world[2] + (1 - a) * last_tower_armor_h[id];
    
    tower_armor_hs_datas[id] += tower_armor_h;
    last_tower_armor_h[id] = tower_armor_h;
    tower_armor_hs_datas_ptr[id]++;     

    // 历史高度数据保护机制，防止长时间追踪导致累加溢出
    if(tower_armor_hs_datas[id] > 10000){
      tower_armor_hs_datas[id] = (tower_armor_hs_datas[id] / tower_armor_hs_datas_ptr[id]) * 600;
      tower_armor_hs_datas_ptr[id] = 600;
    }
    sync_tower_armor_heights();
  }

  last_id = id;
  update_count_++;    
  xyz_in_world = armor.xyz_in_world;

  ekf_.correct(id);

  Eigen::Vector4d observation_xyzyaw;
  observation_xyzyaw << armor.xyz_in_world, armor.ypr_in_world[0];
  rv_residual =
    ekf_.posterior_residual_squared(observation_xyzyaw, id);
}

// 获取 EKF 状态向量
Eigen::VectorXd Target::ekf_x() const { return ekf_.x; }

// 获取滤波器常引用
const RVfromFYT & Target::ekf() const { return ekf_; }

// 返回所有装甲板的预测四维状态 (X, Y, Z, Angle) 列表
std::vector<Eigen::Vector4d> Target::armor_xyza_list() const
{
  return ekf_.armor_xyza_list();
}

Eigen::Matrix<double, 5, 1> Target::get_recent_armor_xyzad() const
{
  Eigen::Vector3d xyz;
  double yaw;
  auto min_dist = 1e10;

  Eigen::VectorXd ekf_x = this->ekf_x();
  // 如果delta_angle为0，则该装甲板中心和整车中心的连线在世界坐标系的xy平面过原点
  static std::vector<std::pair<int ,double>> armorId_delta_list;  
  if(!armorId_delta_list.empty()) armorId_delta_list.clear();
  std::vector<Eigen::Vector4d> armor_xyza_list = this->armor_xyza_list();

  auto armor_num = armor_xyza_list.size();
  // // 如果装甲板未发生过跳变，则只有当前装甲板的位置已知
  // if (!target.jumped) return {true, armor_xyza_list[0]};

  // 整车旋转中心的球坐标yaw
  auto center_yaw = std::atan2(ekf_x[2], ekf_x[0]);

  for (int i = 0; i < armor_num; i++) {
    auto delta_angle = tools::limit_rad(armor_xyza_list[i][3] - center_yaw);
    // auto dist = armor_xyza_list[i].head<2>().norm();
    armorId_delta_list.emplace_back(std::make_pair(i, delta_angle));
  }
  
  for (auto & xyza : this->armor_xyza_list()) {
    auto dist = xyza.head<2>().norm();
    if (dist < min_dist) {
      min_dist = dist;
      xyz = xyza.head<3>();
      yaw = xyza[3];
    }
  }

  double abs_vyaw = abs(ekf_x(7));
  if(abs_vyaw < 90./57.3 
    && abs(armorId_delta_list[this->last_id].second) < 60./57.3){// 判断当前看到的装甲板在预测时间之后是否还在视野内
    min_dist = armor_xyza_list[this->last_id].head<2>().norm();
    xyz = armor_xyza_list[this->last_id].head<3>();
    yaw = armor_xyza_list[this->last_id](3);
  }

  Eigen::Matrix<double, 5, 1> result;
  result << xyz[0], xyz[1], xyz[2], yaw, min_dist;
  return result;
}

// 检查滤波器半径是否发散
bool Target::diverged() const
{
  auto r_ok = ekf_.x[8] > 0.05 && ekf_.x[8] < 0.5;
  auto l_ok = ekf_.x[8] + ekf_.x[9] > 0.05 && ekf_.x[8] + ekf_.x[9] < 0.5;
  if (r_ok && l_ok) return false;
  return true;
}

// 判断当前目标是否收敛
bool Target::convergened()
{
  if (this->name != ArmorName::outpost && update_count_ > 3 && !this->diverged()) {
    is_converged_ = true;
  }
  if (this->name == ArmorName::outpost && update_count_ > 10 && !this->diverged()) {
    is_converged_ = true;
  }
  return is_converged_;
}

Eigen::Vector3d Target::h_armor_xyz(const Eigen::VectorXd & x, int id) const
{
  return ekf_.h_armor_xyz(x, id);
}

void Target::sync_tower_armor_heights()
{
  ekf_.set_tower_armor_heights(tower_armor_hs);
}

// 检查是否完成初始化
bool Target::checkinit() { return isinit; }

}  // namespace auto_aim
