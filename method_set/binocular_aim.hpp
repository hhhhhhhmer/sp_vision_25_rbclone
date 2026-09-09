#ifndef BINOCUAR_AIM_HPP
#define BINOCUAR_AIM_HPP

#include <atomic>
#include <cstdint>

#include "../io/camera.hpp"
#include "../tasks/auto_aim/planner/planner.hpp"
#include "../tasks/auto_aim/solver.hpp"
#include "../tasks/auto_aim/tracker.hpp"
#include "../tools/logger.hpp"

using namespace std::chrono_literals;


template<typename T> 
class BinocularType{
public:
  using Ptr = T*;
  using Address = T&;

  Address short_aim, long_aim;
  Ptr aim_ptr = nullptr;
  BinocularType(T& short_aim_, T&long_aim_): short_aim(short_aim_), long_aim(long_aim_), aim_ptr(&short_aim_){}

  void Switch(){
    aim_ptr = aim_ptr == &short_aim ? &long_aim: &short_aim;
  }
};


enum CameraState{
  whack,                    //正常
  long_camera_is_off_line,  // 长焦相机离线
  short_camera_is_off_line, // 短焦相机离线
  off_line                  // 双相机离线
};

struct BinocularAim{
  BinocularAim(
        io::Camera& cam_short, io::Camera& cam_long,
        auto_aim::Solver& solver_short, auto_aim::Solver& solver_long,
        auto_aim::Planner& planner_short, auto_aim::Planner& planner_long
    ) : cameras(cam_short, cam_long),
        solvers(solver_short, solver_long),
        planners(planner_short, planner_long) 
    {}
  BinocularType<io::Camera> cameras;
  BinocularType<auto_aim::Solver> solvers;
  BinocularType<auto_aim::Planner> planners;
  bool is_short = true;
  CameraState camera_state = CameraState::whack;


  std::chrono::steady_clock::time_point switch_time_point;

  // //长短焦各射程范围 min_near到max_far
  // double short_min_near = 0, short_max_far = 3.3;
  // double long_min_near = 1.5, long_max_far = 5.5;

  // 缓冲区 far2near and near2far
  double short2long_point =  5.0;//(short_max_far + long_min_near)/2.;
  double long2short_point = 3.50;

  std::atomic<int> force_control_frames{0};

  std::uint64_t generation() const
  {
    return switch_generation_.load(std::memory_order_acquire);
  }

  /// @brief 长短焦强制切换
  bool Switch(
    auto_aim::Tracker& tracker, bool forced_switch = false,
    bool update_tracker_solver = true){

    if(!forced_switch){
      if(tools::delta_time(std::chrono::steady_clock::now(), switch_time_point) < 3.0) return false;
      if(is_short && tracker.get_update_count() < 70 ) return false;
    }
    bool switched = false;
    auto is_switch = [&](){
      auto & target_camera = is_short ? this->cameras.long_aim : this->cameras.short_aim;
      target_camera.clear_camera_frame_buffer();
      // this->cameras.aim_ptr->pause();
      this->cameras.Switch();
      this->solvers.Switch();
      this->planners.Switch();
      // this->cameras.aim_ptr->resume();
      is_short = !is_short;
      switch_time_point = std::chrono::steady_clock::now();
      if (update_tracker_solver) tracker.setSolver(this->solvers.aim_ptr);
      switch_generation_.fetch_add(1, std::memory_order_release);
      switched = true;

      force_control_frames = 5;
      
      // if(is_short)
      // {
      //   if(!this->cameras.long_aim.is_paused()) this->cameras.long_aim.pause();
      //   if(!this->cameras.short_aim.is_paused()) this->cameras.short_aim.resume();
      // }
      // else
      // {
      //   if(!this->cameras.long_aim.is_paused()) this->cameras.long_aim.resume();
      //   if(!this->cameras.short_aim.is_paused()) this->cameras.short_aim.pause();
      // }
    };
    // if(cameras.short_aim)
    if(camera_state == CameraState::whack || camera_state == CameraState::off_line){
      is_switch();
    }else {
      if(camera_state == CameraState::long_camera_is_off_line && !is_short) is_switch();
      if(camera_state == CameraState::short_camera_is_off_line && is_short) is_switch();
    }

    return switched;
  }

  bool read(cv::Mat & img, std::chrono::steady_clock::time_point & timestamp, auto_aim::Tracker& tracker){
    bool is_full = this->cameras.aim_ptr->try_read(img, timestamp);
    // std::chrono::steady_clock::time_point t = std::chrono::steady_clock::now();

    bool long_camera_is_running = this->cameras.long_aim.get_capturing();
    bool short_camera_is_running = this->cameras.short_aim.get_capturing();

    // tools::logger()->info("当前相机状态：{}, long_camera:{}, short_camera:{}", (int)camera_state, long_camera_is_running, short_camera_is_running);

    while (!is_full)
    {
      std::this_thread::sleep_for(2ms);
      is_full = this->cameras.aim_ptr->try_read(img, timestamp);
      if(tools::delta_time(std::chrono::steady_clock::now(), 
            this->cameras.aim_ptr->get_last_read_t()) > 1) {
        Switch(tracker);//切换至另一个相机
        tools::logger()->debug("切换相机");
      }
      
      tools::logger()->debug("死循环");
      // if(tools::delta_time(std::chrono::steady_clock::now(), 
      //       this->cameras.aim_ptr->get_last_read_t()) > 1) {
      //   if(camera_state == CameraState::whack){
      //     camera_state = is_short ? CameraState::short_camera_is_off_line : CameraState::long_camera_is_off_line;
      //   }else {
      //     camera_state = CameraState::off_line;
      //   }
      // }
    }



    if(long_camera_is_running && short_camera_is_running){
      camera_state = CameraState::whack;
      this->is_short = this->cameras.aim_ptr == &this->cameras.short_aim ? true : false;
    }
    if(!long_camera_is_running && short_camera_is_running){
      camera_state =  CameraState::long_camera_is_off_line;
      this->is_short = true;
    }
    if(long_camera_is_running && !short_camera_is_running){
      camera_state =  CameraState::short_camera_is_off_line;
      this->is_short = false;
    }
    if(!long_camera_is_running && !short_camera_is_running){
      camera_state = CameraState::off_line;
    }

    return is_full;
    
  }

  /// @brief 长短焦自动切换逻辑
  bool ChangeTheScope(
    auto_aim::Target target, auto_aim::Tracker& tracker,
    bool update_tracker_solver = true){

    auto now = std::chrono::steady_clock::now();
    auto elapsed_time = std::chrono::duration_cast<std::chrono::milliseconds>(now - switch_time_point).count();
    
    if (elapsed_time < 1000) {
        return false; // 在冷却时间内，直接退出，不执行切换判断
    }
    const auto x_est = target.getEKFXest();
    const double x = x_est(0), y = x_est(2), z = x_est(4);
    double dis = sqrt( x*x + y*y + z*z);

    // if(is_short ) dis -= 1.16;

    // tools::logger()->info("dis = {}", dis);
    
    if(is_short && dis > short2long_point ){
      // tools::logger()->info("切换至长焦镜头, dis = {}", dis);
      return Switch(tracker, false, update_tracker_solver);
    }else if(!is_short && dis < long2short_point){
      // tools::logger()->info("切换至短焦镜头 dis = {}", dis);
      return Switch(tracker, false, update_tracker_solver);
    }
    return false;
  }

private:
  std::atomic<std::uint64_t> switch_generation_{0};
};

#endif //BINOCUAR_AIM_HPP
