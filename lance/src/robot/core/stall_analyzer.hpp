#pragma once

#include "robot/core/motor_interface.hpp"


namespace lance
{

struct StallAnalyzerConfig
{
    struct MotorConfig
    {
        double acceleration_jump_rps_per_second{100.0};
        double debounce_time_seconds{0.25};
        double min_output_current_amps{10.0};
        double velocity_error_rps{20.0};
        double min_command_value{0.01};
        double min_output_percent{0.05};
        double min_output_voltage{1.0};
    };

    MotorConfig tracks{
        .acceleration_jump_rps_per_second = 100.0,
        .debounce_time_seconds = 0.25,
        .min_output_current_amps = 50.0,
        .velocity_error_rps = 20.0,
        .min_command_value = 0.01,
        .min_output_percent = 0.05,
        .min_output_voltage = 1.0};
    MotorConfig trencher{
        .acceleration_jump_rps_per_second = 100.0,
        .debounce_time_seconds = 0.25,
        .min_output_current_amps = 15.0,
        .velocity_error_rps = 20.0,
        .min_command_value = 1.0,
        .min_output_percent = 0.05,
        .min_output_voltage = 1.0};

    double recovery_debounce_seconds{0.10};
};

struct MotorStallInfo
{
    bool is_stalled{false};
    double time_stalled_seconds{0.0};
};

enum class TrackSide
{
    LEFT,
    RIGHT
};

class StallAnalyzer
{
public:
    explicit StallAnalyzer(StallAnalyzerConfig config = {});

    const StallAnalyzerConfig& getConfig() const;
    void setConfig(const StallAnalyzerConfig& config);
    void reset();

    MotorStallInfo analyzeTrack(
        TrackSide side,
        const TalonInfoMsg& status,
        const TalonFaultsMsg& faults,
        const TalonCtrlMsg& command,
        double dt_seconds);

    MotorStallInfo analyzeTrencher(
        const TalonInfoMsg& status,
        const TalonFaultsMsg& faults,
        const TalonCtrlMsg& command,
        double dt_seconds);

private:
    struct MotorState
    {
        bool initialized{false};
        bool is_stalled{false};
        double stall_candidate_seconds{0.0};
        double recovery_candidate_seconds{0.0};
        double time_stalled_seconds{0.0};
    };

    MotorStallInfo analyzeMotor(
        MotorState& state,
        const TalonInfoMsg& status,
        const TalonCtrlMsg& command,
        const StallAnalyzerConfig::MotorConfig& motor_config,
        double dt_seconds) const;

    StallAnalyzerConfig config;
    MotorState track_right_state;
    MotorState track_left_state;
    MotorState trencher_state;
};

};  // namespace lance
