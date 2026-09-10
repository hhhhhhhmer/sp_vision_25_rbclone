#include <fmt/core.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <fstream>
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

#include "tasks/auto_aim/aiming/aimer.hpp"
#include "tasks/auto_aim/detection/yolo.hpp"
#include "tasks/auto_aim/geometry/solver.hpp"
#include "tasks/auto_aim/tracking/tracker.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"

namespace
{
struct SnapshotArmor
{
  auto_aim::ArmorName name;
  auto_aim::ArmorType type;
  std::vector<cv::Point2f> points;
  cv::Point2f center;
};

struct RunningStats
{
  std::vector<double> values;
  void add(double v)
  {
    if (std::isfinite(v)) values.push_back(v);
  }
  double mean() const
  {
    if (values.empty()) return 0.0;
    double s = 0;
    for (double v : values) s += v;
    return s / values.size();
  }
  double rms() const
  {
    if (values.empty()) return 0.0;
    double s = 0;
    for (double v : values) s += v * v;
    return std::sqrt(s / values.size());
  }
  double median() const
  {
    if (values.empty()) return 0.0;
    auto c = values;
    std::sort(c.begin(), c.end());
    return c[c.size() / 2];
  }
};

const std::string keys =
  "{help h usage ? |                             | 输出命令行参数说明}"
  "{config-path c  | ../configs/demo.yaml        | yaml配置文件路径}"
  "{bullet-speed b | 27.0                        | 弹速 m/s}"
  "{max-frames m   | 0                           | 最多处理帧数，0 表示全部}"
  "{fps            | 0.0                         | 无位姿txt时的帧率，0 表示读取视频}"
  "{@input-path    | ../快/2026-04-04_19-59-07    | avi和txt文件的路径（不含扩展名）}"
  "{@output-csv    | /tmp/tracker_bench.csv      | 逐帧结果 CSV 输出路径}";
}  // namespace

int main(int argc, char * argv[])
{
  cv::CommandLineParser cli(argc, argv, keys);
  if (cli.has("help")) {
    cli.printMessage();
    return 0;
  }
  const auto input_path = cli.get<std::string>(0);
  const auto output_csv = cli.get<std::string>(1);
  const auto config_path = cli.get<std::string>("config-path");
  const double bullet_speed = cli.get<double>("bullet-speed");
  const int max_frames = cli.get<int>("max-frames");
  const double fps_override = cli.get<double>("fps");

  const auto video_path = fmt::format("{}.avi", input_path);
  const auto text_path = fmt::format("{}.txt", input_path);
  cv::VideoCapture video(video_path);
  std::ifstream text(text_path);
  if (!video.isOpened()) {
    tools::logger()->error("[bench] cannot open video {}", video_path);
    return 1;
  }
  // 没有位姿 txt 时退化为"静止相机"模式：四元数取单位值，时间戳由视频帧率合成。
  // 适用于三脚架固定相机的录像（背景不变），此时世界系=相机系，A/B 指标仍然有效。
  const bool static_camera = !text.is_open();
  double fps = fps_override;
  if (fps <= 0.0) fps = video.get(cv::CAP_PROP_FPS);
  if (static_camera) {
    // 没有位姿 txt 就既没有真实时间戳、也没有云台姿态。
    // 容器头里的帧率在本项目的录像里很不可靠（实测 1440 录像标称 90fps、真实中位间隔 20ms），
    // 而 dt 偏小会让 EKF 预测严重不足——目标转得越快误差越大。
    // 所以这种素材只适合做定性对比，定量结论请用带 txt 的录像，或用 --fps 扫出最优帧率。
    tools::logger()->warn(
      "[bench] no pose txt ({}); static-camera mode with SYNTHETIC timestamps at fps={:.1f} "
      "(container header, often wrong for this project's recordings -> sweep --fps)",
      text_path, fps);
  }
  if (fps <= 0.0) fps = 30.0;

  auto_aim::YOLO yolo(config_path, false);
  auto_aim::Solver solver(config_path);
  auto_aim::Tracker tracker(config_path, &solver);
  auto_aim::Aimer aimer(config_path);

  std::ofstream csv(output_csv);
  csv << "frame,t,n_det,n_cand,n_upd,state,has_target,last_id,nis,"
         "x,vx,y,vy,z,vz,yaw,vyaw,r,r_off,h,"
         "reproj_best_px,reproj_mean_px,cmd_control,cmd_shoot,cmd_yaw_deg,cmd_pitch_deg\n";
  csv << std::fixed;

  const auto t0 = std::chrono::steady_clock::now();
  cv::Mat img;

  RunningStats reproj_best, reproj_mean, reproj_self, pred_err, nis_stats, z_accel, yaw_accel,
    cmd_yaw_accel;
  std::vector<Eigen::Vector4d> prev_predicted;
  auto_aim::ArmorName prev_name = auto_aim::ArmorName::not_armor;
  auto_aim::ArmorType prev_type = auto_aim::ArmorType::small;
  bool prev_valid = false;
  int pred_err_n = 0;
  int frames = 0, frames_with_target = 0, interruptions = 0, update_frames_multi = 0;
  int cand_total = 0;
  bool prev_has_target = false;
  double prev_z = 0, prev_prev_z = 0, prev_dt = 0;
  double prev_cmd_yaw = 0, prev_prev_cmd_yaw = 0;
  int consec = 0;

  for (int frame_count = 0;; frame_count++) {
    if (max_frames > 0 && frame_count >= max_frames) break;
    video.read(img);
    if (img.empty()) break;
    double t = 0, w = 1, x = 0, y = 0, z = 0;
    if (static_camera) {
      t = frame_count / fps;
    } else if (!(text >> t >> w >> x >> y >> z)) {
      break;
    }

    const auto timestamp = t0 + std::chrono::microseconds(static_cast<long>(t * 1e6));
    solver.set_R_gimbal2world(Eigen::Quaterniond(w, x, y, z));

    auto armors = yolo.detect(img, frame_count);

    // 快照本帧原始检测，track() 会原地过滤/写入位姿
    std::vector<SnapshotArmor> snapshot;
    snapshot.reserve(armors.size());
    for (const auto & armor : armors) {
      snapshot.push_back({armor.name, armor.type, armor.points, armor.center});
    }
    const int n_det = static_cast<int>(armors.size());

    // 离线回放路径：与 tests/auto_aim_test.cpp 一致，不依赖云台硬件
    auto targets = tracker.test_track(armors, timestamp);
    auto command = aimer.aim(targets, timestamp, bullet_speed, false);

    frames++;
    const bool has_target = !targets.empty();
    if (has_target) frames_with_target++;
    if (prev_has_target && !has_target) interruptions++;
    prev_has_target = has_target;

    int n_cand = 0, n_upd = 0;
    double reproj_best_px = -1, reproj_mean_px = -1, reproj_self_px = -1;
    int last_id = -1, armor_num = 0;
    double nis = -1;
    Eigen::VectorXd ekf_x = Eigen::VectorXd::Zero(11);

    if (has_target) {
      const auto & target = targets.front();
      ekf_x = target.ekf_x();
      last_id = target.last_id;
      nis = target.ekf().last_nis;
      n_upd = tracker.last_update_count();
      const auto predicted = target.armor_xyza_list();
      armor_num = static_cast<int>(predicted.size());

      double sum_best = 0;
      for (const auto & armor : snapshot) {
        if (armor.name != target.name || armor.type != target.armor_type) continue;
        n_cand++;
        double best = 1e18;
        for (const auto & xyza : predicted) {
          const auto reproj =
            solver.reproject_armor(xyza.head<3>(), xyza[3], target.armor_type, target.name);
          if (reproj.size() != armor.points.size()) continue;
          double err = 0;
          for (std::size_t i = 0; i < reproj.size(); i++)
            err += cv::norm(reproj[i] - armor.points[i]);
          best = std::min(best, err / static_cast<double>(reproj.size()));
        }
        if (best < 1e17) sum_best += best;

        // 无偏度量：用上一帧状态预测的装甲板位置，和本帧观测比较（一步预测误差）
        if (prev_valid && armor.name == prev_name && armor.type == prev_type) {
          double best_prev = 1e18;
          for (const auto & xyza : prev_predicted) {
            const auto reproj = solver.reproject_armor(xyza.head<3>(), xyza[3], armor.type, armor.name);
            if (reproj.size() != armor.points.size()) continue;
            double err = 0;
            for (std::size_t i = 0; i < reproj.size(); i++)
              err += cv::norm(reproj[i] - armor.points[i]);
            best_prev = std::min(best_prev, err / static_cast<double>(reproj.size()));
          }
          if (best_prev < 1e17) {
            pred_err.add(best_prev);
            pred_err_n++;
          }
        }

        // 诊断：用该装甲板自身的 PnP 位姿重投影，检查度量本身是否可信（应 ~1px）
        if (reproj_self_px < 0) {
          auto_aim::Armor self;
          self.points = armor.points;
          self.type = armor.type;
          self.name = armor.name;
          if (solver.try_solve(self)) {
            const auto reproj_self =
              solver.reproject_armor(self.xyz_in_world, self.ypr_in_world[0], self.type, self.name);
            if (reproj_self.size() == armor.points.size()) {
              double err = 0;
              for (std::size_t i = 0; i < reproj_self.size(); i++)
                err += cv::norm(reproj_self[i] - armor.points[i]);
              reproj_self_px = err / static_cast<double>(reproj_self.size());
            }
          }
        }
      }
      prev_predicted = predicted;
      prev_name = target.name;
      prev_type = target.armor_type;
      prev_valid = true;
      if (n_cand > 0) {
        reproj_mean_px = sum_best / n_cand;
        reproj_best_px = reproj_mean_px;
        reproj_best.add(reproj_best_px);
        reproj_mean.add(reproj_mean_px);
      }
      if (reproj_self_px >= 0) reproj_self.add(reproj_self_px);
      if (std::isfinite(nis)) nis_stats.add(nis);
      cand_total += n_cand;
      if (n_upd > 1) update_frames_multi++;

      // 平滑度：状态 z 与指令 yaw 的二阶差分（按 dt 归一化为加速度）
      if (consec >= 2) {
        const double dt = t - prev_dt;
        if (dt > 1e-4) {
          z_accel.add((ekf_x[4] - 2 * prev_z + prev_prev_z) / (dt * dt));
          cmd_yaw_accel.add((command.yaw - 2 * prev_cmd_yaw + prev_prev_cmd_yaw) / (dt * dt));
        }
      }
      prev_prev_z = prev_z;
      prev_z = ekf_x[4];
      prev_prev_cmd_yaw = prev_cmd_yaw;
      prev_cmd_yaw = command.yaw;
      prev_dt = t;
      consec++;
    } else {
      consec = 0;
    }

    csv << frame_count << ',' << t << ',' << n_det << ',' << n_cand << ',' << n_upd << ','
        << tracker.state() << ',' << (has_target ? 1 : 0) << ',' << last_id << ',' << nis;
    for (int i = 0; i < 11; i++) csv << ',' << ekf_x[i];
    csv << ',' << reproj_best_px << ',' << reproj_mean_px << ',' << (command.control ? 1 : 0)
        << ',' << (command.shoot ? 1 : 0) << ',' << command.yaw * 57.29577951308232 << ','
        << command.pitch * 57.29577951308232 << '\n';
  }

  csv.flush();
  csv.close();

  fmt::print("\n===== tracker offline bench =====\n");
  fmt::print("input          : {}\n", input_path);
  fmt::print("config         : {}\n", config_path);
  fmt::print("csv            : {}\n", output_csv);
  fmt::print("frames         : {}\n", frames);
  fmt::print("with target    : {} ({:.1f}%)\n", frames_with_target,
             frames > 0 ? 100.0 * frames_with_target / frames : 0.0);
  fmt::print("interruptions  : {}\n", interruptions);
  fmt::print("candidates/frm : {:.2f}\n", frames > 0 ? 1.0 * cand_total / frames : 0.0);
  fmt::print("frames n_upd>1 : {} ({:.1f}% of tracked)\n", update_frames_multi,
             frames_with_target > 0 ? 100.0 * update_frames_multi / frames_with_target : 0.0);
  fmt::print("reproj err px  : mean {:.3f}  rms {:.3f}  median {:.3f}  (n={})\n",
             reproj_mean.mean(), reproj_mean.rms(), reproj_mean.median(),
             reproj_mean.values.size());
  fmt::print("reproj self px : mean {:.3f}  median {:.3f}  (n={})  [PnP 自洽性诊断]\n",
             reproj_self.mean(), reproj_self.median(), reproj_self.values.size());
  fmt::print(
    "pred err px    : mean {:.3f}  rms {:.3f}  median {:.3f}  (n={})  [上帧状态预测本帧观测]\n",
    pred_err.mean(), pred_err.rms(), pred_err.median(), pred_err_n);
  fmt::print("NIS            : mean {:.3f}  median {:.3f}  (n={})\n", nis_stats.mean(),
             nis_stats.median(), nis_stats.values.size());
  fmt::print("z  accel rms   : {:.2f} m/s^2   median |a| {:.2f}\n", z_accel.rms(),
             z_accel.median());
  fmt::print("cmd yaw acc rms: {:.2f} rad/s^2 median |a| {:.2f}\n", cmd_yaw_accel.rms(),
             cmd_yaw_accel.median());
  return 0;
}
