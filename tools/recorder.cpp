#include "recorder.hpp"

#include <fmt/chrono.h>

#include <filesystem>
#include <cctype>
#include <string>

#include "math_tools.hpp"
#include "tools/logger.hpp"

namespace tools
{
Recorder::Recorder(double fps) : init_(false), fps_(fps), queue_(8), stop_thread_(false)
{
  start_time_ = std::chrono::steady_clock::now();
  last_time_ = start_time_;

  auto folder_path = "records";
  auto file_name = fmt::format("{:%Y-%m-%d_%H-%M-%S}", std::chrono::system_clock::now());
  text_path_ = fmt::format("{}/{}.txt", folder_path, file_name);
  video_base_path_ = fmt::format("{}/{}", folder_path, file_name);

  std::filesystem::create_directory(folder_path);
}

Recorder::~Recorder()
{
  stop_thread_ = true;
  // 退出时给队列中额外推入一个空帧，避免pop一直等待
  queue_.push(
    {cv::Mat::zeros(0, 0, 0), {0, 0, 0, 0}, std::chrono::steady_clock::now(), ""});
  if (saving_thread_.joinable()) saving_thread_.join();  // 等待视频保存线程结束

  if (!init_) return;
  text_writer_.close();
  video_writer_.release();
}

void Recorder::save_to_file()
{
  while (!stop_thread_) {
    FrameData frame;
    queue_.pop(frame);  // 从队列中取出帧数据
    if (frame.img.empty()) {
      tools::logger()->debug("Recorder received empty img. Skip this frame.");
      continue;
    }

    if (
      !video_writer_.isOpened() || frame.stream_id != active_stream_id_ ||
      frame.img.size() != video_size_ || frame.img.type() != video_type_) {
      if (!open_video_segment(frame.img, frame.stream_id)) continue;
    }

    // 写入视频文件
    try {
      video_writer_.write(frame.img);
    } catch (const cv::Exception & e) {
      tools::logger()->error("Recorder failed to write {}: {}", video_path_, e.what());
      video_writer_.release();
      continue;
    }

    // 写入文本文件（输出顺序为wxyz）
    Eigen::Vector4d xyzw = frame.q.coeffs();
    auto since_begin = tools::delta_time(frame.timestamp, start_time_);
    text_writer_ << fmt::format(
      "{} {} {} {} {}\n", since_begin, xyzw[3], xyzw[0], xyzw[1], xyzw[2]);
  }
}

void Recorder::record(
  const cv::Mat & img, const Eigen::Quaterniond & q,
  const std::chrono::steady_clock::time_point & timestamp, const std::string & stream_id)
{
  if (img.empty()) return;
  if (!init_) init(img, stream_id);

  if (stream_id != last_stream_id_ || timestamp <= last_time_) {
    last_stream_id_ = stream_id;
    last_time_ = timestamp - std::chrono::microseconds(static_cast<int>(1e6 / fps_));
  }

  auto since_last = tools::delta_time(timestamp, last_time_);
  if (since_last < 1.0 / fps_) return;

  last_time_ = timestamp;
  queue_.push({img.clone(), q, timestamp, stream_id});
}

void Recorder::init(const cv::Mat & img, const std::string & stream_id)
{
  text_writer_.open(text_path_);
  if (!text_writer_.is_open()) {
    tools::logger()->error("Recorder failed to open {}", text_path_);
  }
  open_video_segment(img, stream_id);
  saving_thread_ = std::thread(&Recorder::save_to_file, this);  // 启动保存线程
  init_ = true;
}

bool Recorder::open_video_segment(const cv::Mat & img, const std::string & stream_id)
{
  if (video_writer_.isOpened()) video_writer_.release();

  auto safe_stream_id = stream_id.empty() ? std::string("default") : stream_id;
  for (auto & c : safe_stream_id) {
    if (!std::isalnum(static_cast<unsigned char>(c)) && c != '-' && c != '_') c = '_';
  }

  video_path_ =
    fmt::format("{}_{}_{:03d}.avi", video_base_path_, safe_stream_id, segment_index_++);
  auto fourcc = cv::VideoWriter::fourcc('M', 'J', 'P', 'G');
  video_writer_.open(video_path_, fourcc, fps_, img.size());
  if (!video_writer_.isOpened()) {
    tools::logger()->error(
      "Recorder failed to open video segment {} ({}x{}, type={})", video_path_, img.cols,
      img.rows, img.type());
    return false;
  }

  active_stream_id_ = stream_id;
  video_size_ = img.size();
  video_type_ = img.type();
  tools::logger()->info(
    "Recorder opened {} (stream={}, {}x{})", video_path_, safe_stream_id, img.cols, img.rows);
  return true;
}

}  // namespace tools
