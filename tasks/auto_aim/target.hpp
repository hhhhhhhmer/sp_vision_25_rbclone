#ifndef AUTO_AIM__TARGET_HPP
#define AUTO_AIM__TARGET_HPP

#include <Eigen/Dense>
#include <chrono>
#include <optional>
#include <queue>
#include <string>
#include <vector>

#include "armor.hpp"
#include "kf_example/rv_from_fyt.hpp"
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

  Target();
  Target(
    const Armor & armor, std::chrono::steady_clock::time_point t, double radius, int armor_num,
    Eigen::VectorXd P0_dig);
  Target(double x, double vyaw, double radius, double h);

  void predict(std::chrono::steady_clock::time_point t);
  void predict(double dt, Eigen::VectorXd u_xyz = Eigen::VectorXd::Zero(3));
  void predict(std::chrono::steady_clock::time_point t, Eigen::VectorXd u_xyz);
  void update(const Armor & armor);

  Eigen::VectorXd ekf_x() const;
  const RVfromFYT & ekf() const;
  std::vector<Eigen::Vector4d> armor_xyza_list() const;
  Eigen::Matrix<double, 5, 1> get_recent_armor_xyzad() const;

  Eigen::Vector3d h_armor_xyz(const Eigen::VectorXd & x, int id) const;

  bool diverged() const;

  bool convergened();

  bool isinit = false;

  bool checkinit();

  inline Eigen::VectorXd getEKFXest() {
    return ekf_.x;
  }

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
  
  int update_count_;

  std::optional<tools::Wave> wave_;

private:
  int armor_num_;
  int switch_count_;
  

  bool is_switch_, is_converged_;

  RVfromFYT ekf_;
  State2Est* est = &ekf_;

  std::chrono::steady_clock::time_point t_;

  void sync_tower_armor_heights();
};

}  // namespace auto_aim

#endif  // AUTO_AIM__TARGET_HPP
