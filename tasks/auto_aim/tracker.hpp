#ifndef AUTO_AIM__TRACKER_HPP
#define AUTO_AIM__TRACKER_HPP

#include <Eigen/Dense>
#include <chrono>
#include <list>
#include <optional>
#include <string>

#include "io/gimbal/gimbal.hpp"
#include "armor.hpp"
#include "solver.hpp"
#include "target.hpp"
#include "tasks/omniperception/perceptron.hpp"
#include "tools/thread_safe_queue.hpp"

namespace tools
{
class FFTExample;
}

namespace auto_aim
{
class Tracker
{
public:
  Tracker(const std::string & config_path, Solver * solver);

  std::string state() const;

  void reset();

  std::list<Target> sb_track(
    std::list<Armor> & armors, std::chrono::steady_clock::time_point t,
    bool cam_is_short = true,
    bool use_enemy_color = true);

  std::list<Target> track(
    std::list<Armor> & armors, std::chrono::steady_clock::time_point t, 
    bool cam_is_short = true,
    bool use_enemy_color = true);

  std::list<Target> test_track(
    std::list<Armor> & armors, std::chrono::steady_clock::time_point t, 
    bool cam_is_short = true,
    bool use_enemy_color = true);


  std::tuple<omniperception::DetectionResult, std::list<Target>> track(
    const std::vector<omniperception::DetectionResult> & detection_queue, std::list<Armor> & armors,
    std::chrono::steady_clock::time_point t, bool use_enemy_color = true);

  inline void setSolver(Solver * solver__){this->solver_ = solver__; }
  void set_gimbal(io::Gimbal* gimbal) { gimbal_ = gimbal; }
  void set_fft(tools::FFTExample * fft);
  inline size_t get_update_count(){return this->target_.update_count_;}
private:
  Solver * solver_;
  io::Gimbal* gimbal_ = nullptr; // 新增一个云台指针，默认为空
  tools::FFTExample * fft_ = nullptr;  // non-owning
  Color enemy_color_;
  std::string enemy_color_str_;
  int min_detect_count_;
  int max_temp_lost_count_;
  int detect_count_;
  int temp_lost_count_;
  int outpost_max_temp_lost_count_;
  int normal_temp_lost_count_;
  std::string state_, pre_state_;
  Target target_;
  std::chrono::steady_clock::time_point last_timestamp_;
  ArmorPriority omni_target_priority_;
  std::optional<uint8_t> last_mode_;
  bool cam_is_switch = false, last_cam_is_short = true;

  void state_machine(bool found);

  bool set_target(std::list<Armor> & armors, std::chrono::steady_clock::time_point t);

  bool update_target(std::list<Armor> & armors, std::chrono::steady_clock::time_point t);

  void reset_fft_sample_state();
  void update_fft_sample(const Armor & armor, std::chrono::steady_clock::time_point t);

  
};

}  // namespace auto_aim

#endif  // AUTO_AIM__TRACKER_HPP
