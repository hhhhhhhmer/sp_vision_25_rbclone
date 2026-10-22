#pragma once
#include <array>
#include <optional>
#include "RuneObservationRefiner.hpp"
#include "PowerRunePlane.hpp"
#include "PhaseMotionEstimator.hpp"
#include "RuneDecisionModule.hpp"
#include "power_rune_interface.hpp"

struct PowerRuneCoreDebugState
{
    std::vector<std::vector<cv::Point>> armor_contours;
    std::vector<std::vector<cv::Point>> light_arm_contours;
    std::vector<std::vector<cv::Point>> center_contours;
    std::vector<cv::Point2f> current_reprojection;
    std::optional<double> reprojection_error;
    bool uv_pose_active = false;
    std::optional<double> uv_reprojection_error;
    double uv_nis = 0.0;
    int uv_rejected_blades = 0;
    bool pnp_used = false;
    std::optional<double> keypoint_reprojection_error;
    std::optional<double> phase;
    std::optional<double> continuous_phase;
    std::optional<double> angular_velocity;
    std::array<double, 5> big_rune_parameters{};
    bool big_rune_model_ready = false;
    bool produced_target = false;
};

class PowerRuneProcessor
{
public:
    PowerRuneProcessor(RuneDecisionModule &rune_decision_module);
    void process_power_rune(const power_rune::RuneInput &detect_input);
    const PowerRuneCoreDebugState & debug_state() const { return m_debug_state; }

private:
    RuneObservation convert2rune_observation(const power_rune::RuneInput &detect_input);//构建符链路的结构体
    RuneObservationRefiner m_rune_observation_refiner;
    PowerRunePlane m_power_rune_plane;
    PhaseMotionEstimator m_phase_motion_estimator;
    RuneDecisionModule &m_rune_decision_module;
    std::vector<RuneTarget> m_debug_rune_targets;
    PowerRuneCoreDebugState m_debug_state;
};
