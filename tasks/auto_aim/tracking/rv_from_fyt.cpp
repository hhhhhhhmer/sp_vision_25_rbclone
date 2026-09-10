// rv_from_fyt.cpp
#include "rv_from_fyt.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

#include "tower_armor_geometry.hpp"
#include "tools/math_tools.hpp"

namespace auto_aim
{
namespace
{
constexpr double kChiSquareThreshold = 9.488;
constexpr double kHysteresisMargin = 5.0;
}  // namespace

/**
 * @brief 构造函数：初始化状态、协方差、装甲数量和类型
 */
RVfromFYT::RVfromFYT(
  const Eigen::VectorXd & x0, const Eigen::MatrixXd & P0, int armor_num,
  ArmorName armor_name)
: State2Est(x0, P0, state_add),
  armor_num_(armor_num),
  armor_name_(armor_name)
{
  if (x0.size() != kStateDimension || P0.rows() != kStateDimension ||
      P0.cols() != kStateDimension) {
    throw std::invalid_argument("RVfromFYT requires an 11-dimensional state");
  }
  if (armor_num_ <= 0) {
    throw std::invalid_argument("RVfromFYT requires at least one armor");
  }
}


void RVfromFYT::kf_predict(double dt, 
     const Eigen::VectorXd & u, 
     const Eigen::VectorXd noises){
  auto v1 = noises(0);
  auto v2 = noises(1);
  this->predict_model(dt, u.head<3>(), v1, v2);
}



/**
 * @brief 预测步骤：使用常速度模型，输入加速度作为控制量，并加入过程噪声
 */
void RVfromFYT::predict_model(
  double dt, const Eigen::Vector3d & acceleration, double position_process_noise,
  double yaw_process_noise)
{
  // 常速度转移矩阵块
  Eigen::Matrix2d constant_velocity_transition;
  constant_velocity_transition << 1.0, dt,
                                  0.0, 1.0;

  // 完整状态转移矩阵 F
  Eigen::MatrixXd F = Eigen::MatrixXd::Identity(kStateDimension, kStateDimension);
  F.block<2, 2>(0, 0) = constant_velocity_transition;  // x, vx
  F.block<2, 2>(2, 2) = constant_velocity_transition;  // y, vy
  F.block<2, 2>(4, 4) = constant_velocity_transition;  // z, vz
  F.block<2, 2>(6, 6) = constant_velocity_transition;  // yaw, vyaw

  const double dt2 = dt * dt;
  const double dt3 = dt2 * dt;
  const double dt4 = dt2 * dt2;

  // 过程噪声协方差块（基于加速度白噪声）
  auto process_noise_block = [&](double variance) {
    Eigen::Matrix2d block;
    block << dt4 / 4.0 * variance, dt3 / 2.0 * variance,
             dt3 / 2.0 * variance, dt2 * variance;
    return block;
  };

  // UV 模式下深度（最不可观测方向）使用独立的过程噪声缩放
  const double z_process_noise =
    uv_ready() ? position_process_noise * std::max(uv_config_.process_noise_z_scale, 0.0)
               : position_process_noise;

  Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(kStateDimension, kStateDimension);
  Q.block<2, 2>(0, 0) = process_noise_block(position_process_noise);  // x, vx
  Q.block<2, 2>(2, 2) = process_noise_block(position_process_noise);  // y, vy
  Q.block<2, 2>(4, 4) = process_noise_block(z_process_noise);         // z, vz
  Q.block<2, 2>(6, 6) = process_noise_block(yaw_process_noise);       // yaw, vyaw

  // 控制输入矩阵 B（加速度对位置和速度的影响）
  const double half_dt2 = 0.5 * dt2;
  Eigen::Matrix<double, kStateDimension, 3> B;
  B << half_dt2, 0.0,      0.0,       // x
       dt,       0.0,      0.0,       // vx
       0.0,      half_dt2, 0.0,       // y
       0.0,      dt,       0.0,       // vy
       0.0,      0.0,      half_dt2,  // z
       0.0,      0.0,      dt,        // vz
       0.0,      0.0,      0.0,       // yaw
       0.0,      0.0,      0.0,       // vyaw
       0.0,      0.0,      0.0,       // radius
       0.0,      0.0,      0.0,       // radius offset
       0.0,      0.0,      0.0;       // height offset

  // 定义非线性预测函数 f(state) = F*state + B*acceleration，并对偏航角归一化
  auto f = [&](const Eigen::VectorXd & state) {
    Eigen::VectorXd prediction = F * state + B * acceleration;
    prediction[6] = tools::limit_rad(prediction[6]);
    return prediction;
  };

  tools::ExtendedKalmanFilter::predict(F, Q, f);
}

/**
 * @brief 准备测量：计算观测向量和测量协方差，处理相机切换后的协方差膨胀
 * @param angle_sigma_scale 装甲板朝向观测的噪声放大系数；多装甲板融合时第二块及之后
 *        装甲板的 PnP 朝向常因 ±70° 搜索窗口/镜像解而不可靠，可用该系数把它降权
 */
void RVfromFYT::prepare_measurement(
  const Armor & armor, bool cam_is_short, int update_count, double angle_sigma_scale)
{
  if (last_cam_is_short_ != cam_is_short) {
    camera_switch_time_ = std::chrono::steady_clock::now();
    last_cam_is_short_ = cam_is_short;
  }

  const double center_yaw = std::atan2(armor.xyz_in_world[1], armor.xyz_in_world[0]);
  const double delta_angle = tools::limit_rad(armor.ypr_in_world[0] - center_yaw);

  // 基础测量方差：优先使用配置值，否则沿用原启发式
  double azimuth_variance =
    azimuth_sigma_ > 0.0 ? azimuth_sigma_ * azimuth_sigma_ : 4e-3;
  const double pitch_variance = azimuth_variance;
  double distance_variance = distance_sigma_ > 0.0
                               ? distance_sigma_ * distance_sigma_
                               : std::log(std::abs(delta_angle) + 1.0) + 1.0;
  double angle_variance = angle_sigma_ > 0.0
                            ? angle_sigma_ * angle_sigma_
                            : std::log(std::abs(armor.ypd_in_world[2]) + 1.0) / 200.0 + 9e-2;
  if (angle_sigma_scale > 0.0) angle_variance *= angle_sigma_scale * angle_sigma_scale;


  // 构造测量协方差矩阵 R
  R_ << azimuth_variance, 0.0,            0.0,               0.0,             // yaw
        0.0,              pitch_variance, 0.0,               0.0,             // pitch
        0.0,              0.0,            distance_variance, 0.0,             // distance
        0.0,              0.0,            0.0,               angle_variance;  // armor yaw

  z_ << armor.ypd_in_world[0],
        armor.ypd_in_world[1],
        armor.ypd_in_world[2],
        armor.ypr_in_world[0];
  measurement_ready_ = true;
}

/**
 * @brief 基于马氏距离选择最佳装甲ID，并应用滞后机制防止频繁跳变
 */
int RVfromFYT::select_armor_id(int last_id) const
{
  return select_armor_id(last_id, -1.0, {});
}

/**
 * @brief 基于马氏距离选择最佳装甲ID，并可按门限/已用ID拒绝异常观测
 */
int RVfromFYT::select_armor_id(
  int last_id, double max_mahalanobis, const std::vector<int> & exclude_ids) const
{
  if (!measurement_ready_) {
    throw std::logic_error("RVfromFYT measurement must be prepared before armor selection");
  }

  int best_id = -1;
  double min_distance = std::numeric_limits<double>::infinity();
  std::vector<double> distances(armor_num_, std::numeric_limits<double>::infinity());

  for (int armor_id = 0; armor_id < armor_num_; ++armor_id) {
    // 本帧已被其它装甲板占用的ID不参与匹配
    if (
      std::find(exclude_ids.begin(), exclude_ids.end(), armor_id) != exclude_ids.end()) {
      continue;
    }
    const Eigen::VectorXd residual = observation_subtract(z_, h(x, armor_id));
    const Eigen::Matrix<double, 4, kStateDimension> H = h_jacobian(x, armor_id);
    const Eigen::Matrix4d S = H * P * H.transpose() + R_;
    const Eigen::LDLT<Eigen::Matrix4d> decomposition(S);
    if (decomposition.info() != Eigen::Success || !decomposition.isPositive()) continue;

    const double distance = residual.dot(decomposition.solve(residual));
    distances[armor_id] = distance;
    if (distance < min_distance) {
      min_distance = distance;
      best_id = armor_id;
    }
  }

  last_mahalanobis_ = std::isfinite(min_distance) ? min_distance : -1.0;

  if (best_id < 0) return -1;

  // 门限：多装甲板融合时用于拒绝另一台车/误检装甲板
  if (max_mahalanobis > 0.0 && !(min_distance <= max_mahalanobis)) return -1;

  // 滞后机制：如果上一帧ID的马氏距离小于阈值且与最小距离差距在容差内，则保持上一帧ID。
  // 同帧融合第二块及之后装甲板时（exclude_ids 非空）关闭滞后，否则会把两块不同的
  // 装甲板都匹配到同一个ID上，导致状态被互相矛盾的观测来回拉扯
  if (
    exclude_ids.empty() && last_id >= 0 && last_id < armor_num_ &&
    distances[last_id] < kChiSquareThreshold &&
    min_distance > distances[last_id] - kHysteresisMargin) {
    return last_id;
  }
  return best_id;
}

/**
 * @brief 执行校正：使用指定的装甲ID更新状态
 */
Eigen::VectorXd RVfromFYT::correct(int armor_id)
{
  if (!measurement_ready_) {
    throw std::logic_error("RVfromFYT measurement must be prepared before correction");
  }

  const Eigen::Matrix<double, 4, kStateDimension> H = h_jacobian(x, armor_id);
  auto observation_model = [&](const Eigen::VectorXd & state) { return h(state, armor_id); };
  measurement_ready_ = false;
  return tools::ExtendedKalmanFilter::update(
    z_, H, R_, observation_model, observation_subtract);
}

/**
 * @brief 设置塔装甲高度偏移
 */
void RVfromFYT::set_tower_armor_heights(const std::pair<bool, double> (&heights)[3])
{
  for (std::size_t i = 0; i < tower_armor_heights_.size(); ++i) {
    tower_armor_heights_[i] = heights[i];
  }
  // UV 观测模型与 h_armor_xyz 必须拿到同一份锚点，否则两者的几何会分叉
  uv_model_.set_tower_armor_heights(tower_armor_heights_);
}

// ---------------------------------------------------------------- UV 观测模式

void RVfromFYT::enable_uv(const UvConfig & config)
{
  uv_config_ = config;
  uv_model_.configure(config, armor_num_, armor_name_);
}

void RVfromFYT::set_camera_geometry(const CameraGeometry & geometry)
{
  uv_model_.set_camera_geometry(geometry);
}

Eigen::VectorXd RVfromFYT::uv_observation_subtract(
  const Eigen::VectorXd & observation, const Eigen::VectorXd & prediction)
{
  Eigen::VectorXd residual = observation - prediction;
  // 每条灯条的第一个分量是角度，需要做 ±pi 归一化
  for (Eigen::Index i = 0; i < residual.size(); i += 4) {
    residual[i] = tools::limit_rad(residual[i]);
  }
  return residual;
}

int RVfromFYT::associate_uv_armor(
  const Armor & armor, const std::vector<int> & exclude_ids, double max_mahalanobis) const
{
  if (!uv_ready()) return -1;
  if (!UvModel::observation_valid(armor.points)) return -1;

  const UvModel::Observation z_obs = UvModel::observation(armor.points);

  int best_id = -1;
  double min_distance = std::numeric_limits<double>::infinity();

  for (int armor_id = 0; armor_id < armor_num_; ++armor_id) {
    if (std::find(exclude_ids.begin(), exclude_ids.end(), armor_id) != exclude_ids.end()) continue;

    // 可见性粗筛：背对相机的装甲板不参与关联
    if (uv_model_.facing_cos(x, armor_id) < uv_config_.facing_cos_min) continue;

    const UvModel::Observation z_pred = uv_model_.predict(x, armor_id, armor.type);
    if (z_pred.squaredNorm() <= 0.0) continue;

    const UvModel::Jacobian H = uv_model_.jacobian(x, armor_id, armor.type);
    const UvModel::Noise R = uv_model_.noise(x, armor_id, armor.type, z_pred);
    const Eigen::Matrix<double, UvModel::kDim, UvModel::kDim> S = H * P * H.transpose() + R;

    // 像素粗门限：取"配置下限"与"状态不确定度 3σ"的较大者，
    // 这样初始/丢失后状态较散时门限会自动放宽，不会因为门限过紧而永远无法重新关联
    const double predicted_sigma_px = 3.0 * std::sqrt(std::max(
                                             std::max(S(1, 1), S(2, 2)),
                                             std::max(S(5, 5), S(6, 6))));
    const double pixel_gate = std::max(uv_config_.associate_gate_px, predicted_sigma_px);
    const double center_dx = z_obs[1] - z_pred[1], center_dy = z_obs[2] - z_pred[2];
    const double center_dx_r = z_obs[5] - z_pred[5], center_dy_r = z_obs[6] - z_pred[6];
    const double d_left = std::sqrt(center_dx * center_dx + center_dy * center_dy);
    const double d_right = std::sqrt(center_dx_r * center_dx_r + center_dy_r * center_dy_r);
    if (std::min(d_left, d_right) > pixel_gate) continue;

    const Eigen::LDLT<Eigen::Matrix<double, UvModel::kDim, UvModel::kDim>> decomposition(S);
    if (decomposition.info() != Eigen::Success || !decomposition.isPositive()) continue;

    const Eigen::VectorXd residual = uv_observation_subtract(z_obs, z_pred);
    const double distance = residual.dot(decomposition.solve(residual));
    if (distance < min_distance) {
      min_distance = distance;
      best_id = armor_id;
    }
  }

  last_mahalanobis_ = std::isfinite(min_distance) ? min_distance : -1.0;
  if (best_id < 0) return -1;
  if (max_mahalanobis > 0.0 && !(min_distance <= max_mahalanobis)) return -1;
  return best_id;
}

bool RVfromFYT::add_uv_observation(int armor_id, const Armor & armor)
{
  if (!uv_ready() || armor_id < 0 || armor_id >= armor_num_) return false;
  if (!UvModel::observation_valid(armor.points)) return false;

  UvPending pending;
  pending.armor_id = armor_id;
  pending.type = armor.type;
  pending.z = UvModel::observation(armor.points);
  // 对角部分（检测噪声）；云台姿态误差是整帧共模，在 correct_uv_batch 里统一加 rank-3 项
  pending.R = uv_model_.diagonal_noise(pending.z);
  pending.J_att = uv_model_.attitude_jacobian(x, armor_id, armor.type);
  uv_pending_.push_back(pending);
  return true;
}

std::size_t RVfromFYT::correct_uv_batch()
{
  if (!uv_ready() || uv_pending_.empty()) return 0;

  const std::size_t count = uv_pending_.size();
  const Eigen::Index dim = static_cast<Eigen::Index>(count) * UvModel::kDim;

  Eigen::VectorXd z(dim);
  Eigen::MatrixXd R = Eigen::MatrixXd::Zero(dim, dim);
  Eigen::MatrixXd J_att(dim, 3);
  for (std::size_t k = 0; k < count; k++) {
    const Eigen::Index offset = static_cast<Eigen::Index>(k) * UvModel::kDim;
    z.segment<UvModel::kDim>(offset) = uv_pending_[k].z;
    R.block<UvModel::kDim, UvModel::kDim>(offset, offset) = uv_pending_[k].R;
    J_att.block<UvModel::kDim, 3>(offset, 0) = uv_pending_[k].J_att;
  }
  // 云台/外参姿态误差：整帧共模，用 rank-3 相关项表达（不能各灯条独立）
  const double sigma_att =
    uv_model_.config().attitude_common_ratio * uv_model_.attitude_sigma_rad();
  R.noalias() += sigma_att * sigma_att * (J_att * J_att.transpose());

  const auto pending = uv_pending_;
  auto h_batch = [this, pending](const Eigen::VectorXd & state) -> Eigen::VectorXd {
    Eigen::VectorXd prediction(static_cast<Eigen::Index>(pending.size()) * UvModel::kDim);
    for (std::size_t k = 0; k < pending.size(); k++) {
      prediction.segment<UvModel::kDim>(static_cast<Eigen::Index>(k) * UvModel::kDim) =
        uv_model_.predict(state, pending[k].armor_id, pending[k].type);
    }
    return prediction;
  };

  Eigen::MatrixXd H(dim, kStateDimension);
  for (std::size_t k = 0; k < count; k++) {
    H.block<UvModel::kDim, kStateDimension>(
      static_cast<Eigen::Index>(k) * UvModel::kDim, 0) =
      uv_model_.jacobian(x, uv_pending_[k].armor_id, uv_pending_[k].type);
  }

  uv_pending_.clear();

  try {
    tools::ExtendedKalmanFilter::update(z, H, R, h_batch, uv_observation_subtract);
  } catch (const std::exception &) {
    return 0;
  }
  return count;
}

/**
 * @brief 计算指定装甲在世界坐标系中的位置
 */
Eigen::Vector3d RVfromFYT::h_armor_xyz(
  const Eigen::VectorXd & state, int armor_id) const
{
  const double angle =
    tools::limit_rad(state[6] + armor_id * 2.0 * CV_PI / armor_num_);
  const bool use_alternate_radius = armor_num_ == 4 && (armor_id == 1 || armor_id == 3);
  const double radius = use_alternate_radius ? state[8] + state[9] : state[8];

  const double armor_x = state[0] - radius * std::cos(angle);
  const double armor_y = state[2] - radius * std::sin(angle);
  const double armor_z = armor_name_ == ArmorName::outpost
                           ? state[4] + state[10] * tower_height_multiplier(armor_id)
                           : state[4] + (use_alternate_radius ? state[10] : 0.0);
  return {armor_x, armor_y, armor_z};
}

/**
 * @brief 获取所有装甲的世界坐标和偏航角
 */
std::vector<Eigen::Vector4d> RVfromFYT::armor_xyza_list() const
{
  std::vector<Eigen::Vector4d> armors;
  armors.reserve(armor_num_);
  for (int armor_id = 0; armor_id < armor_num_; ++armor_id) {
    const Eigen::Vector3d xyz = h_armor_xyz(x, armor_id);
    const double angle = tools::limit_rad(x[6] + armor_id * 2.0 * CV_PI / armor_num_);
    armors.push_back({xyz[0], xyz[1], xyz[2], angle});
  }
  return armors;
}

Eigen::Vector4d RVfromFYT::posterior_residual_squared(
  const Eigen::Vector4d & observation_xyzyaw, int armor_id) const
{
  if (x.size() != kStateDimension) {
    throw std::logic_error("RVfromFYT must be initialized before residual calculation");
  }
  if (armor_id < 0 || armor_id >= armor_num_) {
    throw std::out_of_range("RVfromFYT armor_id is out of range");
  }
  if (!observation_xyzyaw.allFinite()) {
    throw std::invalid_argument("RVfromFYT observation must contain only finite values");
  }

  const Eigen::Vector3d posterior_xyz = h_armor_xyz(x, armor_id);
  const double posterior_yaw =
    tools::limit_rad(x[6] + armor_id * 2.0 * CV_PI / armor_num_);
  Eigen::Vector4d residual;
  residual.head<3>() = observation_xyzyaw.head<3>() - posterior_xyz;
  residual[3] = tools::limit_rad(observation_xyzyaw[3] - posterior_yaw);
  return residual.array().square().matrix();
}

/**
 * @brief 观测模型：从状态和装甲ID计算观测向量
 */
Eigen::Vector4d RVfromFYT::h(const Eigen::VectorXd & state, int armor_id) const
{
  const Eigen::Vector3d ypd = tools::xyz2ypd(h_armor_xyz(state, armor_id));
  const double angle =
    tools::limit_rad(state[6] + armor_id * 2.0 * CV_PI / armor_num_);

  Eigen::Vector4d observation;
  observation << ypd[0], ypd[1], ypd[2], angle;
  return observation;
}

/**
 * @brief 观测模型的雅可比矩阵 (4x11)
 */
Eigen::Matrix<double, 4, RVfromFYT::kStateDimension> RVfromFYT::h_jacobian(
  const Eigen::VectorXd & state, int armor_id) const
{
  const double angle =
    tools::limit_rad(state[6] + armor_id * 2.0 * CV_PI / armor_num_);
  const bool use_alternate_radius = armor_num_ == 4 && (armor_id == 1 || armor_id == 3);
  const double radius = use_alternate_radius ? state[8] + state[9] : state[8];

  const double dx_dyaw = radius * std::sin(angle);
  const double dy_dyaw = -radius * std::cos(angle);
  const double dx_dr = -std::cos(angle);
  const double dy_dr = -std::sin(angle);
  const double dx_dr_offset = use_alternate_radius ? dx_dr : 0.0;
  const double dy_dr_offset = use_alternate_radius ? dy_dr : 0.0;
  const double dz_dh = armor_name_ == ArmorName::outpost
                         ? tower_height_multiplier(armor_id)
                         : (use_alternate_radius ? 1.0 : 0.0);

  // 状态到装甲坐标 [x,y,z,yaw] 的雅可比
  Eigen::Matrix<double, 4, kStateDimension> H_xyza;
  H_xyza << 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, dx_dyaw, 0.0, dx_dr, dx_dr_offset, 0.0,   // x
            0.0, 0.0, 1.0, 0.0, 0.0, 0.0, dy_dyaw, 0.0, dy_dr, dy_dr_offset, 0.0,   // y
            0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0,     0.0, 0.0,   0.0,          dz_dh, // z
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0,     0.0, 0.0,   0.0,          0.0;   // yaw

  // 装甲坐标到观测 [yaw,pitch,distance,armor_yaw] 的雅可比
  const Eigen::Matrix3d H_ypd = tools::xyz2ypd_jacobian(h_armor_xyz(state, armor_id));
  Eigen::Matrix4d H_ypda;
  H_ypda << H_ypd(0, 0), H_ypd(0, 1), H_ypd(0, 2), 0.0,  // yaw
            H_ypd(1, 0), H_ypd(1, 1), H_ypd(1, 2), 0.0,  // pitch
            H_ypd(2, 0), H_ypd(2, 1), H_ypd(2, 2), 0.0,  // distance
            0.0,         0.0,         0.0,         1.0;  // armor yaw
  return H_ypda * H_xyza;
}

/**
 * @brief 计算塔装甲的高度乘数，基于预定义的高度差阈值决定步数
 * @note 与 UvModel 共用 tower_armor_height_multiplier，禁止在此另写一份
 */
double RVfromFYT::tower_height_multiplier(int armor_id) const
{
  return tower_armor_height_multiplier(tower_armor_heights_, armor_id);
}

/**
 * @brief 状态加法，并确保偏航角在 [-pi, pi] 范围内
 */
Eigen::VectorXd RVfromFYT::state_add(
  const Eigen::VectorXd & state, const Eigen::VectorXd & delta)
{
  Eigen::VectorXd result = state + delta;
  result[6] = tools::limit_rad(result[6]);
  return result;
}

/**
 * @brief 观测残差计算，对角度残差进行归一化
 */
Eigen::VectorXd RVfromFYT::observation_subtract(
  const Eigen::VectorXd & observation, const Eigen::VectorXd & prediction)
{
  Eigen::VectorXd residual = observation - prediction;
  residual[0] = tools::limit_rad(residual[0]);
  residual[1] = tools::limit_rad(residual[1]);
  residual[3] = tools::limit_rad(residual[3]);
  return residual;
}

}  // namespace auto_aim
