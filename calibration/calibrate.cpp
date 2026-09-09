#include <fmt/core.h>
#include <yaml-cpp/yaml.h>

#include <Eigen/Dense>  // Must be included before opencv2/core/eigen.hpp.
#include <algorithm>
#include <cctype>
#include <cfloat>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <opencv2/core/eigen.hpp>
#include <opencv2/opencv.hpp>
#include <stdexcept>
#include <string>
#include <vector>

#include "io/camera.hpp"
#include "io/gimbal/gimbal.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/yaml.hpp"

namespace fs = std::filesystem;

const std::string keys =
  "{help h usage ? |                          | 输出命令行参数说明}"
  "{config-path c  | ../configs/sb_short.yaml | yaml配置文件路径 }"
  "{@input-folder  | ../assets/img_with_q        | 输入文件夹路径   }";

namespace
{

constexpr char kWindowName[] = "Camera calibration";

struct CalibrationConfig
{
  int pattern_cols;
  int pattern_rows;
  double square_size_mm;
  std::vector<double> R_gimbal2imubody_data;
  Eigen::Matrix3d R_gimbal2imubody;

  cv::Size pattern_size() const { return {pattern_cols, pattern_rows}; }
};

struct Sample
{
  int id;
  fs::path image_path;
  Eigen::Quaterniond q;
  std::vector<cv::Point2f> corners;
};

struct CameraCalibration
{
  cv::Mat camera_matrix;
  cv::Mat distort_coeffs;
  double reprojection_error;
};

struct HandEyeCalibration
{
  cv::Mat R_camera2gimbal;
  cv::Mat t_camera2gimbal;
  Eigen::Vector3d camera_ypr;
  double board_distance;
  Eigen::Vector3d board_ypr;
};

CalibrationConfig load_config(const std::string & config_path)
{
  auto yaml = tools::load(config_path);
  CalibrationConfig config;
  config.pattern_cols = tools::read<int>(yaml, "pattern_cols");
  config.pattern_rows = tools::read<int>(yaml, "pattern_rows");
  config.square_size_mm = tools::read<double>(yaml, "square_size_mm");
  config.R_gimbal2imubody_data = tools::read<std::vector<double>>(yaml, "R_gimbal2imubody");

  if (config.pattern_cols <= 0 || config.pattern_rows <= 0 || config.square_size_mm <= 0) {
    throw std::runtime_error("棋盘格参数必须为正数");
  }
  if (config.R_gimbal2imubody_data.size() != 9) {
    throw std::runtime_error("R_gimbal2imubody 必须包含 9 个元素");
  }

  using RowMajorMatrix3d = Eigen::Matrix<double, 3, 3, Eigen::RowMajor>;
  config.R_gimbal2imubody = RowMajorMatrix3d(config.R_gimbal2imubody_data.data());
  return config;
}

std::vector<cv::Point3f> camera_board_points(const CalibrationConfig & config)
{
  std::vector<cv::Point3f> points;
  points.reserve(config.pattern_cols * config.pattern_rows);
  for (int row = 0; row < config.pattern_rows; ++row) {
    for (int col = 0; col < config.pattern_cols; ++col) {
      points.emplace_back(col * config.square_size_mm, row * config.square_size_mm, 0.0);
    }
  }
  return points;
}

std::vector<cv::Point3f> handeye_board_points(const CalibrationConfig & config)
{
  std::vector<cv::Point3f> points;
  points.reserve(config.pattern_cols * config.pattern_rows);
  for (int row = 0; row < config.pattern_rows; ++row) {
    for (int col = 0; col < config.pattern_cols; ++col) {
      // Keep the original robot-world-hand-eye board convention: X=0, Y-Z plane.
      points.emplace_back(0.0, col * config.square_size_mm, row * config.square_size_mm);
    }
  }
  return points;
}

bool find_corners(
  const cv::Mat & image, const cv::Size & pattern_size, std::vector<cv::Point2f> & corners,
  bool fast_check)
{
  cv::Mat gray;
  cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
  int flags = 0;
  if (fast_check) {
    flags |= cv::CALIB_CB_FAST_CHECK;
  } else {
    // 仅在非快速模式下使用完整预处理（用于离线批处理等）
    flags |= cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE;
  }

  if (!cv::findChessboardCorners(gray, pattern_size, corners, flags)) return false;

  cv::cornerSubPix(
    gray, corners, cv::Size(11, 11), cv::Size(-1, -1),
    cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER, 30, 0.001));
  return true;
}

bool write_q(const fs::path & path, const Eigen::Quaterniond & q)
{
  std::ofstream file(path);
  if (!file) return false;
  file << fmt::format("{} {} {} {}", q.w(), q.x(), q.y(), q.z());
  return file.good();
}

bool read_q(const fs::path & path, Eigen::Quaterniond & q)
{
  std::ifstream file(path);
  double w, x, y, z;
  if (!(file >> w >> x >> y >> z)) return false;
  if (!std::isfinite(w) || !std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
    return false;
  }

  q = Eigen::Quaterniond(w, x, y, z);
  if (q.norm() < 1e-9) return false;
  q.normalize();
  return true;
}

std::vector<int> image_ids(const fs::path & folder)
{
  std::vector<int> ids;
  if (!fs::exists(folder)) return ids;

  for (const auto & entry : fs::directory_iterator(folder)) {
    if (!entry.is_regular_file() || entry.path().extension() != ".jpg") continue;
    const auto stem = entry.path().stem().string();
    if (stem.empty() || !std::all_of(stem.begin(), stem.end(), [](unsigned char ch) {
          return std::isdigit(ch);
        })) {
      continue;
    }
    ids.push_back(std::stoi(stem));
  }

  std::sort(ids.begin(), ids.end());
  ids.erase(std::unique(ids.begin(), ids.end()), ids.end());
  return ids;
}

int next_image_id(const fs::path & folder)
{
  const auto ids = image_ids(folder);
  return ids.empty() ? 1 : ids.back() + 1;
}

cv::Mat fit_for_display(const cv::Mat & image, int max_width = 1280, int max_height = 800)
{
  const double scale = std::min(
    {1.0, static_cast<double>(max_width) / image.cols,
     static_cast<double>(max_height) / image.rows});
  if (scale >= 1.0) return image.clone();

  cv::Mat resized;
  cv::resize(image, resized, {}, scale, scale, cv::INTER_AREA);
  return resized;
}

void draw_translucent_rect(
  cv::Mat & image, const cv::Rect & rect, const cv::Scalar & color, double opacity)
{
  cv::Mat overlay = image.clone();
  cv::rectangle(overlay, rect, color, cv::FILLED);
  cv::addWeighted(overlay, opacity, image, 1.0 - opacity, 0.0, image);
}

void draw_label(
  cv::Mat & image, const std::string & text, cv::Point origin, double scale,
  const cv::Scalar & color, int thickness = 1)
{
  cv::putText(
    image, text, origin, cv::FONT_HERSHEY_SIMPLEX, scale, cv::Scalar(0, 0, 0), thickness + 2,
    cv::LINE_AA);
  cv::putText(image, text, origin, cv::FONT_HERSHEY_SIMPLEX, scale, color, thickness, cv::LINE_AA);
}

cv::Mat make_capture_view(
  const cv::Mat & image, const CalibrationConfig & config, const std::vector<cv::Point2f> & corners,
  bool board_found, const Eigen::Vector3d & ypr, int saved_count)
{
  cv::Mat view = image.clone();
  if (board_found) {
    cv::drawChessboardCorners(view, config.pattern_size(), corners, true);
  }
  view = fit_for_display(view);

  const int top_height = std::clamp(view.rows / 11, 54, 76);
  const int bottom_height = std::clamp(view.rows / 12, 48, 68);
  draw_translucent_rect(view, cv::Rect(0, 0, view.cols, top_height), cv::Scalar(20, 24, 28), 0.88);
  draw_translucent_rect(
    view, cv::Rect(0, view.rows - bottom_height, view.cols, bottom_height), cv::Scalar(20, 24, 28),
    0.88);

  const cv::Scalar green(94, 220, 134);
  const cv::Scalar red(92, 104, 244);
  const cv::Scalar white(238, 242, 245);
  const cv::Scalar muted(174, 184, 193);
  const int title_y = top_height / 2 + 8;
  draw_label(view, "CALIBRATION", {22, title_y}, 0.72, white, 2);

  const auto pose_text =
    fmt::format("YAW {:+.1f}   PITCH {:+.1f}   ROLL {:+.1f}", ypr[0], ypr[1], ypr[2]);
  int pose_baseline = 0;
  cv::Size pose_size =
    cv::getTextSize(pose_text, cv::FONT_HERSHEY_SIMPLEX, 0.52, 1, &pose_baseline);
  const std::string board_text = view.cols >= 620
                                   ? (board_found ? "BOARD DETECTED" : "BOARD NOT FOUND")
                                   : (board_found ? "BOARD OK" : "NO BOARD");
  draw_label(view, board_text, {205, title_y}, 0.58, board_found ? green : red, 2);
  if (view.cols >= 900) {
    draw_label(view, pose_text, {view.cols - pose_size.width - 22, title_y}, 0.52, muted);
  }

  const int footer_y = view.rows - bottom_height / 2 + 7;
  if (view.cols >= 620) {
    draw_label(view, "S  CAPTURE", {22, footer_y}, 0.58, white, 2);
    draw_label(view, "T  CALIBRATE", {172, footer_y}, 0.58, green, 2);
    draw_label(view, "Q  QUIT", {362, footer_y}, 0.58, muted, 2);

    const auto saved_text = fmt::format("SAVED  {:02d}", saved_count);
    int saved_baseline = 0;
    const auto saved_size =
      cv::getTextSize(saved_text, cv::FONT_HERSHEY_SIMPLEX, 0.58, 2, &saved_baseline);
    draw_label(view, saved_text, {view.cols - saved_size.width - 22, footer_y}, 0.58, white, 2);
  } else {
    draw_label(view, "S SAVE", {14, footer_y}, 0.46, white, 2);
    draw_label(view, "T RUN", {112, footer_y}, 0.46, green, 2);
    draw_label(view, "Q EXIT", {196, footer_y}, 0.46, muted, 2);
  }
  return view;
}

void show_review(
  const cv::Mat & image, const CalibrationConfig & config, const std::vector<cv::Point2f> & corners,
  bool found, int current, int total)
{
  cv::Mat view = image.clone();
  if (found) cv::drawChessboardCorners(view, config.pattern_size(), corners, true);
  view = fit_for_display(view);

  const int bar_height = std::clamp(view.rows / 10, 54, 72);
  draw_translucent_rect(view, cv::Rect(0, 0, view.cols, bar_height), cv::Scalar(20, 24, 28), 0.9);
  const cv::Scalar color = found ? cv::Scalar(94, 220, 134) : cv::Scalar(92, 104, 244);
  draw_label(
    view, found ? "CHECKING BOARD   PASS" : "CHECKING BOARD   SKIP", {22, bar_height / 2 + 8}, 0.64,
    color, 2);

  const auto progress = fmt::format("{:02d} / {:02d}", current, total);
  int baseline = 0;
  const auto size = cv::getTextSize(progress, cv::FONT_HERSHEY_SIMPLEX, 0.6, 2, &baseline);
  draw_label(
    view, progress, {view.cols - size.width - 22, bar_height / 2 + 8}, 0.6,
    cv::Scalar(238, 242, 245), 2);
  cv::imshow(kWindowName, view);
  cv::waitKey(100);
}

std::vector<Sample> load_samples(
  const fs::path & folder, const CalibrationConfig & config, cv::Size & image_size)
{
  const auto ids = image_ids(folder);
  std::vector<Sample> samples;

  for (size_t index = 0; index < ids.size(); ++index) {
    const int id = ids[index];
    const fs::path image_path = folder / fmt::format("{}.jpg", id);
    const fs::path q_path = folder / fmt::format("{}.txt", id);
    const cv::Mat image = cv::imread(image_path.string());
    if (image.empty()) {
      tools::logger()->warn("[{}] 无法读取 {}", id, image_path.string());
      continue;
    }

    if (image_size.empty()) image_size = image.size();
    if (image.size() != image_size) {
      tools::logger()->warn("[{}] 图像尺寸不一致，已跳过", id);
      show_review(image, config, {}, false, index + 1, ids.size());
      continue;
    }

    std::vector<cv::Point2f> corners;
    const bool found = find_corners(image, config.pattern_size(), corners, true);
    show_review(image, config, corners, found, index + 1, ids.size());
    fmt::print("[{}] {}\n", found ? "success" : "failure", image_path.string());
    if (!found) continue;

    Eigen::Quaterniond q;
    if (!read_q(q_path, q)) {
      tools::logger()->warn("[{}] 四元数文件缺失或无效: {}", id, q_path.string());
      continue;
    }
    samples.push_back({id, image_path, q, std::move(corners)});
  }
  return samples;
}

CameraCalibration calibrate_camera(
  const std::vector<Sample> & samples, const CalibrationConfig & config,
  const cv::Size & image_size)
{
  const auto board_points = camera_board_points(config);
  std::vector<std::vector<cv::Point3f>> object_points(samples.size(), board_points);
  std::vector<std::vector<cv::Point2f>> image_points;
  image_points.reserve(samples.size());
  for (const auto & sample : samples) image_points.push_back(sample.corners);

  CameraCalibration result;
  std::vector<cv::Mat> rvecs, tvecs;
  const auto criteria =
    cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 100, DBL_EPSILON);
  cv::calibrateCamera(
    object_points, image_points, image_size, result.camera_matrix, result.distort_coeffs, rvecs,
    tvecs, 0, criteria);

  double error_sum = 0;
  size_t total_points = 0;
  for (size_t i = 0; i < object_points.size(); ++i) {
    std::vector<cv::Point2f> reprojected_points;
    cv::projectPoints(
      object_points[i], rvecs[i], tvecs[i], result.camera_matrix, result.distort_coeffs,
      reprojected_points);
    total_points += reprojected_points.size();
    for (size_t j = 0; j < reprojected_points.size(); ++j) {
      error_sum += cv::norm(image_points[i][j] - reprojected_points[j]);
    }
  }
  result.reprojection_error = error_sum / total_points;
  return result;
}

HandEyeCalibration calibrate_handeye(
  const std::vector<Sample> & samples, const CalibrationConfig & config,
  const CameraCalibration & camera)
{
  const auto board_points = handeye_board_points(config);
  std::vector<cv::Mat> rvecs, tvecs;
  std::vector<cv::Mat> R_world2gimbal_list, t_world2gimbal_list;

  for (const auto & sample : samples) {
    cv::Mat rvec, tvec;
    if (!cv::solvePnP(
          board_points, sample.corners, camera.camera_matrix, camera.distort_coeffs, rvec, tvec,
          false, cv::SOLVEPNP_ITERATIVE)) {
      tools::logger()->warn("[{}] solvePnP 失败，已跳过", sample.id);
      continue;
    }

    const Eigen::Matrix3d R_imubody2imuabs = sample.q.toRotationMatrix();
    // q is expressed in IMU axes. Conjugation changes both sides into gimbal axes.
    const Eigen::Matrix3d R_gimbal2world =
      config.R_gimbal2imubody.transpose() * R_imubody2imuabs * config.R_gimbal2imubody;
    const Eigen::Matrix3d R_world2gimbal = R_gimbal2world.transpose();

    cv::Mat R_world2gimbal_cv;
    cv::eigen2cv(R_world2gimbal, R_world2gimbal_cv);
    rvecs.emplace_back(rvec);
    tvecs.emplace_back(tvec);
    R_world2gimbal_list.emplace_back(R_world2gimbal_cv);
    t_world2gimbal_list.emplace_back(cv::Mat::zeros(3, 1, CV_64F));
  }

  if (rvecs.size() < 3) {
    throw std::runtime_error(
      fmt::format("手眼标定至少需要 3 组有效样本，当前只有 {} 组", rvecs.size()));
  }

  cv::Mat R_gimbal2camera, t_gimbal2camera;
  cv::Mat R_world2board, t_world2board;
  cv::calibrateRobotWorldHandEye(
    rvecs, tvecs, R_world2gimbal_list, t_world2gimbal_list, R_world2board, t_world2board,
    R_gimbal2camera, t_gimbal2camera);
  t_gimbal2camera /= 1e3;
  t_world2board /= 1e3;

  HandEyeCalibration result;
  cv::transpose(R_gimbal2camera, result.R_camera2gimbal);
  result.t_camera2gimbal = -result.R_camera2gimbal * t_gimbal2camera;

  cv::Mat R_board2world;
  cv::transpose(R_world2board, R_board2world);
  const cv::Mat t_board2world = -R_board2world * t_world2board;

  Eigen::Matrix3d R_camera2gimbal_eigen;
  cv::cv2eigen(result.R_camera2gimbal, R_camera2gimbal_eigen);
  const Eigen::Matrix3d R_gimbal2ideal{{0, -1, 0}, {0, 0, -1}, {1, 0, 0}};
  const Eigen::Matrix3d R_camera2ideal = R_gimbal2ideal * R_camera2gimbal_eigen;
  result.camera_ypr = tools::eulers(R_camera2ideal, 1, 0, 2) * 57.3;

  const double x = t_board2world.at<double>(0);
  const double y = t_board2world.at<double>(1);
  result.board_distance = std::sqrt(x * x + y * y);

  Eigen::Matrix3d R_board2world_eigen;
  cv::cv2eigen(R_board2world, R_board2world_eigen);
  result.board_ypr = tools::eulers(R_board2world_eigen, 2, 1, 0) * 57.3;
  return result;
}

void print_camera_yaml(const CameraCalibration & camera)
{
  YAML::Emitter result;
  const std::vector<double> camera_matrix_data(
    camera.camera_matrix.begin<double>(), camera.camera_matrix.end<double>());
  const std::vector<double> distort_coeffs_data(
    camera.distort_coeffs.begin<double>(), camera.distort_coeffs.end<double>());

  result << YAML::BeginMap;
  result << YAML::Comment(fmt::format("重投影误差: {:.4f}px", camera.reprojection_error));
  result << YAML::Key << "camera_matrix";
  result << YAML::Value << YAML::Flow << camera_matrix_data;
  result << YAML::Key << "distort_coeffs";
  result << YAML::Value << YAML::Flow << distort_coeffs_data;
  result << YAML::Newline;
  result << YAML::EndMap;
  fmt::print("\n{}\n", result.c_str());
}

void print_handeye_yaml(const CalibrationConfig & config, const HandEyeCalibration & handeye)
{
  YAML::Emitter result;
  const std::vector<double> R_camera2gimbal_data(
    handeye.R_camera2gimbal.begin<double>(), handeye.R_camera2gimbal.end<double>());
  const std::vector<double> t_camera2gimbal_data(
    handeye.t_camera2gimbal.begin<double>(), handeye.t_camera2gimbal.end<double>());

  result << YAML::BeginMap;
  result << YAML::Key << "R_gimbal2imubody";
  result << YAML::Value << YAML::Flow << config.R_gimbal2imubody_data;
  result << YAML::Newline;
  result << YAML::Newline;
  result << YAML::Comment(fmt::format(
    "相机同理想情况的偏角: yaw{:.2f} pitch{:.2f} roll{:.2f} degree", handeye.camera_ypr[0],
    handeye.camera_ypr[1], handeye.camera_ypr[2]));
  result << YAML::Newline;
  result << YAML::Comment(
    fmt::format("标定板到世界坐标系原点的水平距离: {:.2f} m", handeye.board_distance));
  result << YAML::Newline;
  result << YAML::Comment(fmt::format(
    "标定板同竖直摆放时的偏角: yaw{:.2f} pitch{:.2f} roll{:.2f} degree", handeye.board_ypr[0],
    handeye.board_ypr[1], handeye.board_ypr[2]));
  result << YAML::Key << "R_camera2gimbal";
  result << YAML::Value << YAML::Flow << R_camera2gimbal_data;
  result << YAML::Key << "t_camera2gimbal";
  result << YAML::Value << YAML::Flow << t_camera2gimbal_data;
  result << YAML::Newline;
  result << YAML::EndMap;
  fmt::print("\n{}\n", result.c_str());
}

bool run_calibration(const fs::path & folder, const CalibrationConfig & config)
{
  tools::logger()->info("开始读取标定样本");
  cv::Size image_size;
  const auto samples = load_samples(folder, config, image_size);
  if (samples.size() < 3) {
    tools::logger()->error("至少需要 3 组有效图片和四元数，当前只有 {} 组", samples.size());
    return false;
  }

  tools::logger()->info("使用 {} 组样本进行相机标定", samples.size());
  const auto camera = calibrate_camera(samples, config, image_size);
  print_camera_yaml(camera);

  tools::logger()->info("使用本次相机内参进行手眼标定");
  const auto handeye = calibrate_handeye(samples, config, camera);
  print_handeye_yaml(config, handeye);
  tools::logger()->info("相机标定和手眼标定完成");
  return true;
}

void capture_loop(
  const std::string & config_path, const fs::path & input_folder, const CalibrationConfig & config)
{
  io::Gimbal gimbal(config_path);
  io::Camera::initSDK();
  io::Camera camera(config_path);

  int next_id = 1;          // 每次启动从 1 开始编号
  int saved_count = 0;      // 显示已保存数量，初始为 0
  cv::namedWindow(kWindowName, cv::WINDOW_AUTOSIZE);
  tools::logger()->info("按 's' 保存图片和IMU数据，按 't' 开始标定，按 'q' 退出");

  while (true) {
    cv::Mat image;
    std::chrono::steady_clock::time_point timestamp;
    camera.read(image, timestamp);
    if (image.empty()) {
      tools::logger()->warn("相机返回空图像");
      continue;
    }

    const Eigen::Quaterniond q = gimbal.q(timestamp).normalized();
    const Eigen::Matrix3d R_gimbal2world =
      config.R_gimbal2imubody.transpose() * q.toRotationMatrix() * config.R_gimbal2imubody;
    const Eigen::Vector3d ypr = tools::eulers(R_gimbal2world, 2, 1, 0) * 57.3;

    std::vector<cv::Point2f> corners;
    const bool found = find_corners(image, config.pattern_size(), corners, true);
    cv::imshow(kWindowName, make_capture_view(image, config, corners, found, ypr, saved_count));

    const int key = cv::waitKey(1) & 0xff;
    if (key == 'q') break;

    if (key == 's') {
      const fs::path image_path = input_folder / fmt::format("{}.jpg", next_id);
      const fs::path q_path = input_folder / fmt::format("{}.txt", next_id);
      if (!cv::imwrite(image_path.string(), image) || !write_q(q_path, q)) {
        tools::logger()->error("[{}] 保存失败: {}", next_id, input_folder.string());
        std::error_code error;
        fs::remove(image_path, error);
        fs::remove(q_path, error);
        continue;
      }

      ++saved_count;
      tools::logger()->info("[{}] Saved in {}", next_id, input_folder.string());
      ++next_id;
      continue;
    }

    if (key == 't') {
      try {
        run_calibration(input_folder, config);
      } catch (const cv::Exception & e) {
        tools::logger()->error("OpenCV 标定失败: {}", e.what());
      } catch (const std::exception & e) {
        tools::logger()->error("标定失败: {}", e.what());
      }
      camera.clear_camera_frame_buffer();
    }
  }
  cv::destroyWindow(kWindowName);
}

}  // namespace

int main(int argc, char * argv[])
{
  cv::CommandLineParser cli(argc, argv, keys);
  if (cli.has("help")) {
    cli.printMessage();
    return 0;
  }
  if (!cli.check()) {
    cli.printErrors();
    return 1;
  }

  const auto config_path = cli.get<std::string>("config-path");
  const fs::path input_folder = cli.get<std::string>(0);

  try {
    fs::create_directories(input_folder);
    const auto config = load_config(config_path);
    capture_loop(config_path, input_folder, config);
  } catch (const cv::Exception & e) {
    tools::logger()->error("OpenCV 错误: {}", e.what());
    return 1;
  } catch (const std::exception & e) {
    tools::logger()->error("标定程序启动失败: {}", e.what());
    return 1;
  }

  tools::logger()->warn("注意四元数输出顺序为wxyz");
  return 0;
}
