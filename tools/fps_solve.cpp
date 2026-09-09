#include "fps_solve.hpp"

#include <algorithm>

namespace tools
{

fpsSolve::fpsSolve(std::size_t window_size) : window_size_(std::max<std::size_t>(1, window_size)) {}

double fpsSolve::update(std::chrono::steady_clock::time_point timestamp)
{
  if (!initialized_) {
    last_timestamp_ = timestamp;
    initialized_ = true;
    fps_ = 0.0;
    return fps_;
  }

  const double interval = std::chrono::duration<double>(timestamp - last_timestamp_).count();
  last_timestamp_ = timestamp;
  if (interval <= 0.0) return fps_;

  fps_ = 1.0 / interval;
  fps_window_.push_back(fps_);
  fps_sum_ += fps_;
  if (fps_window_.size() > window_size_) {
    fps_sum_ -= fps_window_.front();
    fps_window_.pop_front();
  }
  return fps_;
}

double fpsSolve::get_fps() const { return fps_; }

double fpsSolve::get_mean_fps() const
{
  return fps_window_.empty() ? 0.0 : fps_sum_ / fps_window_.size();
}

void fpsSolve::reset()
{
  fps_window_.clear();
  last_timestamp_ = {};
  fps_ = 0.0;
  fps_sum_ = 0.0;
  initialized_ = false;
}

}  // namespace tools
