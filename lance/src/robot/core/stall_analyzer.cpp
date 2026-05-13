#include "stall_analyzer.hpp"

#include <algorithm>
#include <cmath>


namespace lance
{
namespace
{

bool isClosedLoopOrOutputCommand(int mode)
{
    return mode == TalonCtrlMsg::PERCENT_OUTPUT ||
           mode == TalonCtrlMsg::POSITION || mode == TalonCtrlMsg::VELOCITY ||
           mode == TalonCtrlMsg::CURRENT || mode == TalonCtrlMsg::VOLTAGE ||
           mode == TalonCtrlMsg::MOTION_PROFILE ||
           mode == TalonCtrlMsg::MOTION_MAGIC ||
           mode == TalonCtrlMsg::MOTION_PROFILE_ARC;
}

bool isCommanded(
    const TalonCtrlMsg& command,
    const TalonInfoMsg& status,
    const StallAnalyzerConfig::MotorConfig& motor_config)
{
    if (!isClosedLoopOrOutputCommand(command.mode))
    {
        return false;
    }

    return std::abs(command.value) >= motor_config.min_command_value ||
           std::abs(status.output_percent) >= motor_config.min_output_percent ||
           std::abs(status.output_voltage) >= motor_config.min_output_voltage;
}

StallAnalyzerConfig::MotorConfig applyLegacyConfig(
    const StallAnalyzerConfig& config,
    StallAnalyzerConfig::MotorConfig motor_config)
{
    if (config.max_stall_velocity_rps !=
        StallAnalyzerConfig::DEFAULT_MAX_STALL_VELOCITY_RPS)
    {
        motor_config.max_stall_velocity_rps = config.max_stall_velocity_rps;
    }
    if (config.min_stall_current_amps !=
        StallAnalyzerConfig::DEFAULT_MIN_STALL_CURRENT_AMPS)
    {
        motor_config.min_stall_output_current_amps =
            config.min_stall_current_amps;
        motor_config.min_stall_supply_current_amps =
            config.min_stall_current_amps;
    }

    return motor_config;
}

}  // namespace

StallAnalyzer::StallAnalyzer(StallAnalyzerConfig config) : config{config} {}

const StallAnalyzerConfig& StallAnalyzer::getConfig() const
{
    return this->config;
}

void StallAnalyzer::setConfig(const StallAnalyzerConfig& config)
{
    this->config = config;
}

void StallAnalyzer::reset()
{
    this->track_right_state = {};
    this->track_left_state = {};
    this->trencher_state = {};
    this->hopper_belt_state = {};
    this->hopper_actuator_state = {};
}

RobotStallStatus StallAnalyzer::analyze(
    const RobotMotorStatus& motor_status,
    const RobotMotorFaults& motor_faults,
    const RobotMotorCommands& motor_commands,
    double dt_seconds)
{
    RobotStallStatus stall_status;
    stall_status.track_right = this->analyzeMotor(
        this->track_right_state,
        motor_status.track_right,
        motor_faults.track_right,
        motor_commands.track_right,
        applyLegacyConfig(this->config, this->config.track_right),
        true,
        dt_seconds);
    stall_status.track_left = this->analyzeMotor(
        this->track_left_state,
        motor_status.track_left,
        motor_faults.track_left,
        motor_commands.track_left,
        applyLegacyConfig(this->config, this->config.track_left),
        true,
        dt_seconds);
    stall_status.trencher = this->analyzeMotor(
        this->trencher_state,
        motor_status.trencher,
        motor_faults.trencher,
        motor_commands.trencher,
        applyLegacyConfig(this->config, this->config.trencher),
        true,
        dt_seconds);
    stall_status.hopper_belt = this->analyzeMotor(
        this->hopper_belt_state,
        motor_status.hopper_belt,
        motor_faults.hopper_belt,
        motor_commands.hopper_belt,
        applyLegacyConfig(this->config, this->config.hopper_belt),
        true,
        dt_seconds);
    stall_status.hopper_actuator = this->analyzeMotor(
        this->hopper_actuator_state,
        motor_status.hopper_actuator,
        motor_faults.hopper_actuator,
        motor_commands.hopper_actuator,
        applyLegacyConfig(this->config, this->config.hopper_actuator),
        true,
        dt_seconds);

    return stall_status;
}

RobotStallStatus StallAnalyzer::analyze(
    const RobotMotorStatus& motor_status,
    const RobotMotorFaults& motor_faults,
    double dt_seconds)
{
    RobotMotorCommands motor_commands;
    motor_commands.track_right.set__mode(TalonCtrlMsg::VELOCITY)
        .set__value(motor_status.track_right.velocity);
    motor_commands.track_left.set__mode(TalonCtrlMsg::VELOCITY)
        .set__value(motor_status.track_left.velocity);
    motor_commands.trencher.set__mode(TalonCtrlMsg::VELOCITY)
        .set__value(motor_status.trencher.velocity);
    motor_commands.hopper_belt.set__mode(TalonCtrlMsg::VELOCITY)
        .set__value(motor_status.hopper_belt.velocity);
    motor_commands.hopper_actuator.set__mode(TalonCtrlMsg::VOLTAGE)
        .set__value(motor_status.hopper_actuator.output_voltage);

    RobotStallStatus stall_status;
    stall_status.track_right = this->analyzeMotor(
        this->track_right_state,
        motor_status.track_right,
        motor_faults.track_right,
        motor_commands.track_right,
        applyLegacyConfig(this->config, this->config.track_right),
        false,
        dt_seconds);
    stall_status.track_left = this->analyzeMotor(
        this->track_left_state,
        motor_status.track_left,
        motor_faults.track_left,
        motor_commands.track_left,
        applyLegacyConfig(this->config, this->config.track_left),
        false,
        dt_seconds);
    stall_status.trencher = this->analyzeMotor(
        this->trencher_state,
        motor_status.trencher,
        motor_faults.trencher,
        motor_commands.trencher,
        applyLegacyConfig(this->config, this->config.trencher),
        false,
        dt_seconds);
    stall_status.hopper_belt = this->analyzeMotor(
        this->hopper_belt_state,
        motor_status.hopper_belt,
        motor_faults.hopper_belt,
        motor_commands.hopper_belt,
        applyLegacyConfig(this->config, this->config.hopper_belt),
        false,
        dt_seconds);
    stall_status.hopper_actuator = this->analyzeMotor(
        this->hopper_actuator_state,
        motor_status.hopper_actuator,
        motor_faults.hopper_actuator,
        motor_commands.hopper_actuator,
        applyLegacyConfig(this->config, this->config.hopper_actuator),
        false,
        dt_seconds);

    return stall_status;
}

MotorStallInfo StallAnalyzer::analyzeMotor(
    MotorState& state,
    const TalonInfoMsg& status,
    const TalonFaultsMsg& faults,
    const TalonCtrlMsg& command,
    const StallAnalyzerConfig::MotorConfig& motor_config,
    bool require_command,
    double dt_seconds) const
{
    const double safe_dt_seconds = std::max(0.0, dt_seconds);
    const double alpha =
        std::clamp(this->config.current_filter_alpha, 0.0, 1.0);
    const double output_current_amps = std::abs(status.output_current);
    const double supply_current_amps = std::abs(status.supply_current);

    if (!state.initialized)
    {
        state.filtered_output_current_amps = output_current_amps;
        state.filtered_supply_current_amps = supply_current_amps;
        state.previous_position = status.position;
        state.position_delta = 0.0;
        state.initialized = true;
    }
    else
    {
        state.position_delta = status.position - state.previous_position;
        state.previous_position = status.position;
        state.filtered_output_current_amps =
            (alpha * output_current_amps) +
            ((1.0 - alpha) * state.filtered_output_current_amps);
        state.filtered_supply_current_amps =
            (alpha * supply_current_amps) +
            ((1.0 - alpha) * state.filtered_supply_current_amps);
    }

    const bool low_velocity =
        std::abs(status.velocity) <= motor_config.max_stall_velocity_rps;
    const bool low_position_delta =
        std::abs(state.position_delta) <= motor_config.max_stall_position_delta;
    const bool low_motion =
        motor_config.use_position_delta ? low_position_delta : low_velocity;
    const bool high_current =
        state.filtered_output_current_amps >=
            motor_config.min_stall_output_current_amps ||
        state.filtered_supply_current_amps >=
            motor_config.min_stall_supply_current_amps ||
        faults.stator_current_limit_fault || faults.supply_current_limit_fault;
    const bool commanded = isCommanded(command, status, motor_config);
    const bool controller_enabled =
        (status.status & TalonInfoMsg::STATUS_PHX_ENABLED) != 0;
    const bool controller_connected =
        (status.status & TalonInfoMsg::STATUS_CONNECTED) != 0;
    const bool allowed_by_command = !require_command || commanded;
    const bool stall_candidate =
        allowed_by_command && controller_enabled && controller_connected &&
        low_motion && high_current;

    if (stall_candidate)
    {
        state.stall_candidate_seconds += safe_dt_seconds;
        state.recovery_candidate_seconds = 0.0;
    }
    else
    {
        state.recovery_candidate_seconds += safe_dt_seconds;
        state.stall_candidate_seconds = 0.0;
    }

    if (!state.is_stalled &&
        state.stall_candidate_seconds >= this->config.stall_debounce_seconds)
    {
        state.is_stalled = true;
    }
    else if (
        state.is_stalled &&
        state.recovery_candidate_seconds >=
            this->config.recovery_debounce_seconds)
    {
        state.is_stalled = false;
    }

    MotorStallInfo info;
    info.is_stalled = state.is_stalled;
    info.is_commanded = commanded;
    info.low_motion = low_motion;
    info.high_current = high_current;
    info.stator_current_limit_warning = faults.stator_current_limit_fault;
    info.supply_current_limit_warning = faults.supply_current_limit_fault;
    info.current_limit_warning =
        info.stator_current_limit_warning || info.supply_current_limit_warning;
    info.controller_enabled = controller_enabled;
    info.controller_connected = controller_connected;
    info.command_mode = command.mode;
    info.command_value = command.value;
    info.position = status.position;
    info.position_delta = state.position_delta;
    info.velocity_rps = status.velocity;
    info.output_percent = status.output_percent;
    info.output_voltage = status.output_voltage;
    info.bus_voltage = status.bus_voltage;
    info.output_current_amps = status.output_current;
    info.supply_current_amps = status.supply_current;
    info.filtered_output_current_amps = state.filtered_output_current_amps;
    info.filtered_supply_current_amps = state.filtered_supply_current_amps;
    info.stall_candidate_seconds = state.stall_candidate_seconds;
    info.recovery_candidate_seconds = state.recovery_candidate_seconds;

    return info;
}

};  // namespace lance
