#ifndef TOOLS__FPS_SOLVE_HPP
#define TOOLS__FPS_SOLVE_HPP

#include <chrono>
#include <cstddef>
#include <deque>

namespace tools
{

class fpsSolve
{
public:
  explicit fpsSolve(std::size_t window_size = 200);

  double update(
    std::chrono::steady_clock::time_point timestamp = std::chrono::steady_clock::now());
  double get_fps() const;
  double get_mean_fps() const;
  void reset();

private:
  std::size_t window_size_;
  std::deque<double> fps_window_;
  std::chrono::steady_clock::time_point last_timestamp_{};
  double fps_ = 0.0;
  double fps_sum_ = 0.0;
  bool initialized_ = false;
};

}  // namespace tools

#endif  // TOOLS__FPS_SOLVE_HPP
