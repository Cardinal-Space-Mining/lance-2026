#pragma once

#include "robot/core/motor_interface.hpp"


namespace lance
{

struct StallAnalyzerConfig
{
    double max_stall_velocity_rps{1.0};
    double min_stall_current_amps{10.0};
    double stall_debounce_seconds{0.25};
    double recovery_debounce_seconds{0.10};
    double current_filter_alpha{0.25};
};

struct MotorStallInfo
{
    bool is_stalled{false};
    bool current_limit_warning{false};
    bool stator_current_limit_warning{false};
    bool supply_current_limit_warning{false};

    double velocity_rps{0.0};
    double output_current_amps{0.0};
    double supply_current_amps{0.0};
    double filtered_output_current_amps{0.0};
    double filtered_supply_current_amps{0.0};

    double stall_candidate_seconds{0.0};
    double recovery_candidate_seconds{0.0};
};

struct RobotStallStatus
{
    MotorStallInfo track_right;
    MotorStallInfo track_left;
    MotorStallInfo trencher;
    MotorStallInfo hopper_belt;
    MotorStallInfo hopper_actuator;
};

class StallAnalyzer
{
public:
    explicit StallAnalyzer(StallAnalyzerConfig config = {});

    const StallAnalyzerConfig& getConfig() const;
    void setConfig(const StallAnalyzerConfig& config);
    void reset();

    RobotStallStatus analyze(
        const RobotMotorStatus& motor_status,
        const RobotMotorFaults& motor_faults,
        double dt_seconds);

private:
    struct MotorState
    {
        bool initialized{false};
        bool is_stalled{false};
        double filtered_output_current_amps{0.0};
        double filtered_supply_current_amps{0.0};
        double stall_candidate_seconds{0.0};
        double recovery_candidate_seconds{0.0};
    };

    MotorStallInfo analyzeMotor(
        MotorState& state,
        const TalonInfoMsg& status,
        const TalonFaultsMsg& faults,
        double dt_seconds) const;

    StallAnalyzerConfig config;
    MotorState track_right_state;
    MotorState track_left_state;
    MotorState trencher_state;
    MotorState hopper_belt_state;
    MotorState hopper_actuator_state;
};

};  // namespace lance
