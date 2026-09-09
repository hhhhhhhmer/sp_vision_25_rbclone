// ca_from_tju.cpp
#include "ca_from_tju.hpp"

#include <cmath>
#include <stdexcept>

#include "tools/math_tools.hpp"

namespace auto_aim
{

/**
 * @brief 构造函数：验证维度并初始化基类
 */
CAFromTJU::CAFromTJU(const Eigen::VectorXd & x0, const Eigen::MatrixXd & P0)
: State2Est(x0, P0, state_add)
{
  if (
    x0.size() != kStateDimension || P0.rows() != kStateDimension ||
    P0.cols() != kStateDimension) {
    throw std::invalid_argument("CAFromTJU requires an 11-dimensional state");
  }
}

/**
 * @brief 卡尔曼滤波预测接口（调用 predict_model）
 */
void CAFromTJU::kf_predict(
  double dt, const Eigen::VectorXd & u, const Eigen::VectorXd noises)
{
  // TrackQueueV3 为自主模型，加速度已包含在状态中，故忽略输入 u
  (void)u;
  predict_model(dt, noises);
}

/**
 * @brief MPC预测接口（直接复用 kf_predict）
 */
void CAFromTJU::mpc_predict(
  double dt, const Eigen::VectorXd & u, const Eigen::VectorXd noises)
{
  kf_predict(dt, u, noises);
}

/**
 * @brief 执行常加速度模型的预测步骤
 * 
 * 构建状态转移矩阵 F 和过程噪声协方差 Q，然后调用基类的 predict 方法。
 * 状态转移方程（连续时间常加速度离散化）：
 *   x_new = x + v*dt + 0.5*a*dt^2
 *   v_new = v + a*dt
 *   a_new = a
 * 偏航角及其速度和加速度同理。
 */
void CAFromTJU::predict_model(
  double dt, const Eigen::VectorXd & process_noise_diagonal)
{
  // 参数有效性检查
  if (!std::isfinite(dt) || dt < 0.0) {
    throw std::invalid_argument("CAFromTJU requires a finite, non-negative dt");
  }
  if (process_noise_diagonal.size() != kStateDimension) {
    throw std::invalid_argument("CAFromTJU requires 11 process-noise values");
  }
  if (
    !process_noise_diagonal.allFinite() ||
    (process_noise_diagonal.array() < 0.0).any()) {
    throw std::invalid_argument(
      "CAFromTJU process-noise values must be finite and non-negative");
  }

  const double half_dt_squared = 0.5 * dt * dt;
  Eigen::MatrixXd F = Eigen::MatrixXd::Identity(kStateDimension, kStateDimension);

  // 位置状态索引：x=0, y=1, z=2, yaw=3
  // 速度状态索引：vx=4, vy=5, vz=6, vyaw=7
  // 加速度状态索引：ax=8, ay=9, ayaw=10
  // 设定 F 中各元素：x = x + vx*dt + ax*0.5*dt^2
  F(0, 4) = dt;
  F(0, 8) = half_dt_squared;
  F(1, 5) = dt;
  F(1, 9) = half_dt_squared;
  F(2, 6) = dt;
  F(3, 7) = dt;
  F(3, 10) = half_dt_squared;  // yaw = yaw + vyaw*dt + ayaw*0.5*dt^2
  // 速度更新：vx = vx + ax*dt
  F(4, 8) = dt;
  F(5, 9) = dt;
  F(7, 10) = dt;  // vyaw = vyaw + ayaw*dt


  // 加速度自身不变（恒加速度模型）
    const double dt2 = dt * dt;
    const double dt3 = dt2 * dt;
    const double dt4 = dt2 * dt2;
    const double dt5 = dt4 * dt;

    auto getQ = [&]{
        using Matrix11d = Eigen::Matrix<double, 11, 11>;
        Matrix11d Q = Matrix11d::Zero();

        auto ca_noise_block = [&](double jerk_noise) {
        Eigen::Matrix3d block;
        block << dt5 / 20.0, dt4 / 8.0, dt3 / 6.0,
                dt4 / 8.0,  dt3 / 3.0, dt2 / 2.0,
                dt3 / 6.0,  dt2 / 2.0, dt;
        return jerk_noise * block;
        };

        auto set_ca_block =
        [&](int position_index, int velocity_index, int acceleration_index,
            const Eigen::Matrix3d & block) {
            const int indices[3]{
            position_index, velocity_index, acceleration_index};

            for (int row = 0; row < 3; ++row) {
            for (int col = 0; col < 3; ++col) {
                Q(indices[row], indices[col]) = block(row, col);
            }
            }
        };

        // 建议 process_noise 包含：qx_jerk, qy_jerk, qz_acceleration, qyaw_jerk
        const double qx = process_noise_diagonal(0);
        const double qy = process_noise_diagonal(0);
        const double qz = process_noise_diagonal(1);
        const double qyaw = process_noise_diagonal(2);

        set_ca_block(0, 4, 8, ca_noise_block(qx));
        set_ca_block(1, 5, 9, ca_noise_block(qy));
        set_ca_block(3, 7, 10, ca_noise_block(qyaw));

        // z 没有 az 状态，使用 [z, vz] 恒速度模型，由白加速度驱动
        Eigen::Matrix2d z_noise;
        z_noise << dt3 / 3.0, dt2 / 2.0,
                dt2 / 2.0, dt;
        z_noise *= qz;

        Q(2, 2) = z_noise(0, 0);
        Q(2, 6) = z_noise(0, 1);
        Q(6, 2) = z_noise(1, 0);
        Q(6, 6) = z_noise(1, 1);
        return Q;
    };
    Eigen::MatrixXd Q = getQ();


    // Eigen::MatrixXd Q(11,11);
    // double q1 = 0.1;
    // Q <<  q1,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
    //         0,   q1, 0,   0,   0,   0,   0,   0,   0,   0,   0,
    //         0,   0,   q1, 0,   0,   0,   0,   0,   0,   0,   0,
    //         0,   0,   0,   q1, 0,   0,   0,   0,   0,   0,   0,
    //         0,   0,   0,   0,   q1, 0,   0,   0,   0,   0,   0,
    //         0,   0,   0,   0,   0,   q1, 0,   0,   0,   0,   0,
    //         0,   0,   0,   0,   0,   0,   q1, 0,   0,   0,   0,
    //         0,   0,   0,   0,   0,   0,   0,   q1, 0,   0,   0,
    //         0,   0,   0,   0,   0,   0,   0,   0,   q1, 0,   0,
    //         0,   0,   0,   0,   0,   0,   0,   0,   0,   q1, 0,
    //         0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   q1;

  // 定义非线性预测函数（线性，但偏航角需归一化）
  auto f = [&F](const Eigen::VectorXd & state) {
    Eigen::VectorXd prediction = F * state;
    prediction[3] = tools::limit_rad(prediction[3]);  // 偏航角限制在 [-pi, pi]
    return prediction;
  };

  // 调用基类的预测方法（EKF预测）
  tools::ExtendedKalmanFilter::predict(F, Q, f);
}

/**
 * @brief 从单个装甲板构造 [x, y, z, yaw] 测量
 */
void CAFromTJU::prepare_measurement(const Armor & armor)
{
  const double yaw_in_gimbal = armor.ypr_in_gimbal[0];
//   if (!armor.xyz_in_world.allFinite() || !std::isfinite(armor_yaw)) {
//     throw std::invalid_argument("CAFromTJU measurement must contain only finite values");
//   }

//   double r1 = 0.1;
//   z_ << armor.xyz_in_world[0], armor.xyz_in_world[1], armor.xyz_in_world[2], armor_yaw;
//   R_.setIdentity();
//   R_ *= r1;

    const double sigma_x = 0.02;                    // 2 cm
    const double sigma_y = 0.02;
    const double sigma_z = 0.06;                    // 3 cm
    const double sigma_yaw = pow(2.0 * CV_PI / 180.0 * abs(yaw_in_gimbal), 2);  // 2 degree

    R_ = Eigen::Matrix4d::Zero();
    R_.diagonal() << sigma_x * sigma_x,
                     sigma_y * sigma_y,
                     sigma_z * sigma_z,
                     sigma_yaw * sigma_yaw;
     measurement_ready_ = true;
}

Eigen::Vector4d CAFromTJU::posterior_residual_squared(
  const Eigen::Vector4d & observation_xyzyaw, int armor_id) const
{
  (void)armor_id;
  if (x.size() != kStateDimension) {
    throw std::logic_error("CAFromTJU must be initialized before residual calculation");
  }
  if (!observation_xyzyaw.allFinite()) {
    throw std::invalid_argument("CAFromTJU observation must contain only finite values");
  }

  Eigen::Vector4d residual = observation_xyzyaw - x.head<4>();
  residual[3] = tools::limit_rad(residual[3]);
  return residual.array().square().matrix();
}

/**
 * @brief 状态加法，并对偏航角归一化
 */
Eigen::VectorXd CAFromTJU::state_add(
  const Eigen::VectorXd & state, const Eigen::VectorXd & delta)
{
  Eigen::VectorXd result = state + delta;
  result[3] = tools::limit_rad(result[3]);
  return result;
}

}  // namespace auto_aim
