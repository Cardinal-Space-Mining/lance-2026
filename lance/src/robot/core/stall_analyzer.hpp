#pragma once

#include "robot/core/motor_interface.hpp"


namespace lance
{

struct StallAnalyzerConfig
{
    static constexpr double DEFAULT_MAX_STALL_VELOCITY_RPS = 1.0;
    static constexpr double DEFAULT_MIN_STALL_CURRENT_AMPS = 10.0;

    struct MotorConfig
    {
        double max_stall_velocity_rps{DEFAULT_MAX_STALL_VELOCITY_RPS};
        double max_stall_position_delta{0.001};
        double min_stall_output_current_amps{DEFAULT_MIN_STALL_CURRENT_AMPS};
        double min_stall_supply_current_amps{DEFAULT_MIN_STALL_CURRENT_AMPS};
        double min_command_value{0.01};
        double min_output_percent{0.05};
        double min_output_voltage{1.0};
        bool use_position_delta{false};
    };

    double max_stall_velocity_rps{DEFAULT_MAX_STALL_VELOCITY_RPS};
    double min_stall_current_amps{DEFAULT_MIN_STALL_CURRENT_AMPS};

    MotorConfig track_right{};
    MotorConfig track_left{};
    MotorConfig trencher{
        .max_stall_velocity_rps = 1.0,
        .max_stall_position_delta = 0.001,
        .min_stall_output_current_amps = 15.0,
        .min_stall_supply_current_amps = 10.0,
        .min_command_value = 1.0,
        .min_output_percent = 0.05,
        .min_output_voltage = 1.0,
        .use_position_delta = false};
    MotorConfig hopper_belt{
        .max_stall_velocity_rps = 1.0,
        .max_stall_position_delta = 0.001,
        .min_stall_output_current_amps = 10.0,
        .min_stall_supply_current_amps = 10.0,
        .min_command_value = 1.0,
        .min_output_percent = 0.05,
        .min_output_voltage = 1.0,
        .use_position_delta = false};
    MotorConfig hopper_actuator{
        .max_stall_velocity_rps = 0.01,
        .max_stall_position_delta = 0.0005,
        .min_stall_output_current_amps = 2.0,
        .min_stall_supply_current_amps = 2.0,
        .min_command_value = 0.01,
        .min_output_percent = 0.05,
        .min_output_voltage = 1.0,
        .use_position_delta = true};

    double stall_debounce_seconds{0.25};
    double recovery_debounce_seconds{0.10};
    double current_filter_alpha{0.25};
};

struct MotorStallInfo
{
    bool is_stalled{false};
    bool is_commanded{false};
    bool low_motion{false};
    bool high_current{false};
    bool current_limit_warning{false};
    bool stator_current_limit_warning{false};
    bool supply_current_limit_warning{false};
    bool controller_enabled{false};
    bool controller_connected{false};

    int command_mode{TalonCtrlMsg::DISABLED};
    double command_value{0.0};
    double position{0.0};
    double position_delta{0.0};
    double velocity_rps{0.0};
    double output_percent{0.0};
    double output_voltage{0.0};
    double bus_voltage{0.0};
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
        const RobotMotorCommands& motor_commands,
        double dt_seconds);

    RobotStallStatus analyze(
        const RobotMotorStatus& motor_status,
        const RobotMotorFaults& motor_faults,
        double dt_seconds);

private:
    struct MotorState
    {
        bool initialized{false};
        bool is_stalled{false};
        double previous_position{0.0};
        double position_delta{0.0};
        double filtered_output_current_amps{0.0};
        double filtered_supply_current_amps{0.0};
        double stall_candidate_seconds{0.0};
        double recovery_candidate_seconds{0.0};
    };

    MotorStallInfo analyzeMotor(
        MotorState& state,
        const TalonInfoMsg& status,
        const TalonFaultsMsg& faults,
        const TalonCtrlMsg& command,
        const StallAnalyzerConfig::MotorConfig& motor_config,
        bool require_command,
        double dt_seconds) const;

    StallAnalyzerConfig config;
    MotorState track_right_state;
    MotorState track_left_state;
    MotorState trencher_state;
    MotorState hopper_belt_state;
    MotorState hopper_actuator_state;
};

};  // namespace lance
