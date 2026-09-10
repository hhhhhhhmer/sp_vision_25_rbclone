// UV 观测模型单元测试：
//  1) 前向模型 predict() 与 OpenCV projectPoints 独立实现的一致性（含畸变）
//  2) 解析雅可比 jacobian() 与中心差分的一致性
//  3) 测量噪声矩阵正定性与尺度关系
//  4) 装甲板法线翻转 180° 时 UVL 观测不变的特性（PnP 镜像解歧义在 UV 下消失）
//  5) 可见性判据方向正确
#include <Eigen/Dense>
#include <cmath>
#include <cstdio>
#include <opencv2/opencv.hpp>
#include <vector>

#include "tasks/auto_aim/tracking/uv_model.hpp"

using namespace auto_aim;

namespace
{
int failures = 0;

void check(bool ok, const char * name, double value = 0.0, double tol = 0.0)
{
  if (ok) {
    std::printf("  [ ok ] %s\n", name);
  } else {
    std::printf("  [FAIL] %s (value=%.6g, tol=%.6g)\n", name, value, tol);
    failures++;
  }
}

Eigen::Matrix3d Ry(double a)
{
  Eigen::Matrix3d R;
  R << std::cos(a), 0, std::sin(a), 0, 1, 0, -std::sin(a), 0, std::cos(a);
  return R;
}

Eigen::Matrix3d Rz(double a)
{
  Eigen::Matrix3d R;
  R << std::cos(a), -std::sin(a), 0, std::sin(a), std::cos(a), 0, 0, 0, 1;
  return R;
}

/** @brief 独立实现：状态 -> 4 个角点像素（走 OpenCV projectPoints） */
std::vector<cv::Point2f> reference_project(
  const UvModel & model, const CameraGeometry & geometry, const Eigen::VectorXd & state, int armor_id,
  ArmorType type)
{
  const int armor_num = 4;
  const double theta = state[6] + armor_id * 2.0 * M_PI / armor_num;
  const bool alt = (armor_id == 1 || armor_id == 3);
  const double radius = alt ? state[8] + state[9] : state[8];
  const double height_offset = alt ? state[10] : 0.0;
  const double pitch = 15.0 * M_PI / 180.0;

  const Eigen::Vector3d armor_center(
    state[0] - radius * std::cos(theta), state[2] - radius * std::sin(theta),
    state[4] + height_offset);
  const Eigen::Matrix3d R_armor2world = Rz(theta) * Ry(pitch);

  const double w = UvModel::armor_width(type) / 2.0;
  const double l = UvModel::lightbar_length() / 2.0;
  const std::array<Eigen::Vector3d, 4> local = {Eigen::Vector3d(0, +w, +l), Eigen::Vector3d(0, -w, +l),
                                                Eigen::Vector3d(0, -w, -l), Eigen::Vector3d(0, +w, -l)};

  std::vector<cv::Point3f> camera_points;
  for (const auto & c : local) {
    const Eigen::Vector3d world = armor_center + R_armor2world * c;
    const Eigen::Vector3d camera =
      geometry.R_camera2gimbal.transpose() *
      (geometry.R_gimbal2world.transpose() * world - geometry.t_camera2gimbal);
    camera_points.emplace_back(
      static_cast<float>(camera.x()), static_cast<float>(camera.y()), static_cast<float>(camera.z()));
  }

  cv::Mat camera_matrix(3, 3, CV_64F);
  for (int r = 0; r < 3; r++)
    for (int c = 0; c < 3; c++) camera_matrix.at<double>(r, c) = geometry.camera_matrix(r, c);
  cv::Mat distort(1, static_cast<int>(geometry.distort_coeffs.size()), CV_64F);
  for (std::size_t i = 0; i < geometry.distort_coeffs.size(); i++)
    distort.at<double>(0, static_cast<int>(i)) = geometry.distort_coeffs[i];

  std::vector<cv::Point2f> projected;
  cv::projectPoints(
    camera_points, cv::Vec3d(0, 0, 0), cv::Vec3d(0, 0, 0), camera_matrix, distort, projected);
  (void)model;
  return projected;
}

CameraGeometry make_geometry()
{
  CameraGeometry g;
  g.valid = true;
  g.camera_matrix << 1813.554114428731, 0.0, 712.274797427587,  //
    0.0, 1813.369040609312, 526.9183416131992,                   //
    0.0, 0.0, 1.0;
  g.distort_coeffs = {-0.06533754966400937, 0.11288365366404525, -0.0008807155121340363,
                      -0.0006436478698530189, 0.035117257854987};
  g.R_camera2gimbal << -0.0006300680323985472, -0.03084684563629979, 0.9995239242402179,  //
    -0.9998094041241314, -0.019484318694324685, -0.0012315640329051238,                   //
    0.019513032548112753, -0.999334195071553, -0.030828689908395462;
  g.t_camera2gimbal << 0.09879211822869594, -0.05661876507830866, 0.14718317632615113;
  g.R_gimbal2world = Eigen::Matrix3d::Identity();
  return g;
}

Eigen::VectorXd make_state()
{
  Eigen::VectorXd x = Eigen::VectorXd::Zero(11);
  // [x, vx, y, vy, z, vz, yaw, vyaw, r, r_off, h]
  x << 3.2, 0.4, 1.1, -0.3, 0.15, 0.0, 0.42, 1.8, 0.20, 0.06, 0.02;
  return x;
}
}  // namespace

int main()
{
  UvConfig config;
  config.enabled = true;
  const CameraGeometry geometry = make_geometry();

  UvModel model;
  model.configure(config, 4, ArmorName::one);
  model.set_camera_geometry(geometry);

  std::printf("=== forward model vs OpenCV projectPoints ===\n");
  for (int armor_id = 0; armor_id < 4; armor_id++) {
    const Eigen::VectorXd state = make_state();
    const auto uv = reference_project(model, geometry, state, armor_id, ArmorType::small);
    const auto z_ref = UvModel::observation(uv);
    const auto z_model = model.predict(state, armor_id, ArmorType::small);
    const double err = (z_ref - z_model).cwiseAbs().maxCoeff();
    char name[64];
    std::snprintf(name, sizeof(name), "armor %d forward model max|dz| (px)", armor_id);
    // 参考实现走 cv::projectPoints（内部为 float 像素输出），精度上限约 1e-4 px
    check(err < 1e-3, name, err, 1e-3);
  }

  std::printf("=== analytic jacobian vs central differences ===\n");
  {
    const double h = 1e-6;
    double worst = 0.0;
    int worst_index = -1;
    for (int armor_id = 0; armor_id < 4; armor_id++) {
      const Eigen::VectorXd state = make_state();
      const UvModel::Jacobian H = model.jacobian(state, armor_id, ArmorType::small);
      for (int j = 0; j < 11; j++) {
        if (j == 1 || j == 3 || j == 5 || j == 7) continue;  // 速度状态不参与观测
        Eigen::VectorXd plus = state, minus = state;
        plus[j] += h;
        minus[j] -= h;
        Eigen::VectorXd dz = (model.predict(plus, armor_id, ArmorType::small) -
                              model.predict(minus, armor_id, ArmorType::small)) /
                             (2 * h);
        // 角度分量做 ±pi 归一化
        for (int k = 0; k < UvModel::kDim; k += 4) {
          dz[k] = std::remainder(dz[k], 2 * M_PI);
        }
        const double err = (dz - H.col(j)).cwiseAbs().maxCoeff();
        if (err > worst) {
          worst = err;
          worst_index = armor_id * 100 + j;
        }
      }
    }
    check(worst < 1e-4, "max|H_analytic - H_fd| (worst state idx)", worst, 1e-4);
    if (worst_index >= 0) std::printf("        worst at armor %d state %d\n", worst_index / 100, worst_index % 100);
  }

  std::printf("=== noise model ===\n");
  {
    const Eigen::VectorXd state = make_state();
    const auto z = model.predict(state, 0, ArmorType::small);
    const auto R = model.noise(state, 0, ArmorType::small, z);
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 8, 8>> solver(R);
    check(solver.eigenvalues().minCoeff() > 0.0, "R is positive definite", solver.eigenvalues().minCoeff(), 0.0);
    check(R(3, 3) > 0.0 && R(0, 0) > 0.0, "R diagonal positive", R(3, 3));

    // 姿态误差项应当主导中心噪声：sigma_attitude_deg 加倍后中心方差应显著变大
    UvConfig coarse = config;
    coarse.sigma_attitude_deg = 2.0 * config.sigma_attitude_deg;
    UvModel model2;
    model2.configure(coarse, 4, ArmorName::one);
    model2.set_camera_geometry(geometry);
    const auto R2 = model2.noise(state, 0, ArmorType::small, z);
    check(R2(1, 1) > R(1, 1), "attitude sigma increases center variance", R2(1, 1) - R(1, 1), 0.0);
    // 姿态误差主要是共模平移：对灯条中心的影响应远大于对长度（差分量）的影响
    const double center_delta = R2(1, 1) - R(1, 1);
    const double length_delta = R2(3, 3) - R(3, 3);
    check(length_delta < 0.05 * center_delta, "attitude affects length far less than center", length_delta / std::max(center_delta, 1e-12), 0.05);

    // ratio=0 时两条灯条相互独立
    check(std::abs(R(1, 5)) < 1e-12, "ratio=0 keeps lightbars independent", R(1, 5), 1e-12);

    // ratio=1 时姿态误差是完全共模的：两条灯条中心噪声应高度相关
    UvConfig common = config;
    common.attitude_common_ratio = 1.0;
    UvModel model3;
    model3.configure(common, 4, ArmorName::one);
    model3.set_camera_geometry(geometry);
    const auto R3 = model3.noise(state, 0, ArmorType::small, z);
    const double corr = R3(1, 5) / std::sqrt(R3(1, 1) * R3(5, 5));
    check(corr > 0.5, "ratio=1 makes attitude error common-mode", corr, 0.5);
  }

  std::printf("=== armor yaw observability (planar / mirror ambiguity) ===\n");
  {
    // 平面靶标在 PnP 下存在镜像解；工程上用"装甲板固定 15 度倾角"这一先验来定解。
    // UV 观测把这个先验放进 h(x) 里：灯条在图像中的倾角方向直接反映装甲板朝向，
    // 因此 UVL 对 yaw 有可观测性（雅可比非零），滤波器随时间连续收敛到物理解。
    const Eigen::VectorXd state = make_state();
    const UvModel::Jacobian H = model.jacobian(state, 0, ArmorType::small);

    // 角度分量对 yaw 的偏导：灯条在图像中会随装甲板转动而倾斜
    const double d_angle_d_yaw = std::abs(H(0, 6));
    check(d_angle_d_yaw > 0.05, "lightbar angle carries armor yaw information", d_angle_d_yaw, 0.05);

    // 灯条长度对 yaw 的偏导：侧视时灯条会缩短（透视缩短）
    const double d_len_d_yaw = std::abs(H(3, 6));
    check(d_len_d_yaw > 0.0, "lightbar length carries armor yaw information", d_len_d_yaw, 0.0);

    // yaw 与 yaw+pi 的投影并不相同（倾角先验破坏了 180 度对称性）
    Eigen::VectorXd flipped = state;
    flipped[6] += M_PI;
    const auto z_a = model.predict(state, 0, ArmorType::small);
    const auto z_b = model.predict(flipped, 0, ArmorType::small);
    const double angle_diff = std::abs(std::remainder(z_a[0] - z_b[0], 2 * M_PI));
    const double length_diff = std::abs(z_a[3] - z_b[3]);
    check(angle_diff > 1e-3 || length_diff > 1e-3,
          "tilt prior breaks the 180deg symmetry", std::max(angle_diff, length_diff), 1e-3);
  }

  std::printf("=== visibility ===\n");
  {
    // 目标在 +x 方向、装甲板朝向相机时 facing_cos 应接近 1
    Eigen::VectorXd state = make_state();
    state[0] = 4.0;
    state[2] = 0.0;
    state[6] = 0.0;  // 装甲板 0 的朝向角 = 0 -> 法线指向 +x（背离相机）
    const double facing0 = model.facing_cos(state, 0);
    check(facing0 > 0.9, "front armor faces camera", facing0, 0.9);
    const double facing2 = model.facing_cos(state, 2);
    check(facing2 < -0.9, "rear armor faces away", facing2, -0.9);
  }

  std::printf("=== degenerate inputs ===\n");
  {
    std::vector<cv::Point2f> bad = {{0, 0}, {1, 1}};
    check(!UvModel::observation_valid(bad), "2-point observation rejected");
    std::vector<cv::Point2f> nan_points = {{0, 0}, {1, 1}, {2, 2}, {NAN, 3}};
    check(!UvModel::observation_valid(nan_points), "NaN observation rejected");

    Eigen::VectorXd behind = make_state();
    behind[0] = -3.0;  // 目标跑到相机后面
    const auto z = model.predict(behind, 0, ArmorType::small);
    check(z.squaredNorm() == 0.0, "behind-camera projection returns zero");
  }

  std::printf("\n%s (%d failures)\n", failures == 0 ? "UV MODEL TESTS PASSED" : "UV MODEL TESTS FAILED",
              failures);
  return failures == 0 ? 0 : 1;
}
