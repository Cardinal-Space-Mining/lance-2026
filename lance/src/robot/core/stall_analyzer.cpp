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

int sign(double value)
{
    if (value > 0.0)
    {
        return 1;
    }
    if (value < 0.0)
    {
        return -1;
    }
    return 0;
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
}

MotorStallInfo StallAnalyzer::analyzeTrack(
    TrackSide side,
    const TalonInfoMsg& status,
    const TalonFaultsMsg&,
    const TalonCtrlMsg& command,
    double dt_seconds)
{
    MotorState& state =
        side == TrackSide::LEFT ? this->track_left_state
                                : this->track_right_state;
    return this->analyzeMotor(
        state,
        status,
        command,
        this->config.tracks,
        dt_seconds);
}

MotorStallInfo StallAnalyzer::analyzeTrencher(
    const TalonInfoMsg& status,
    const TalonFaultsMsg&,
    const TalonCtrlMsg& command,
    double dt_seconds)
{
    return this->analyzeMotor(
        this->trencher_state,
        status,
        command,
        this->config.trencher,
        dt_seconds);
}

MotorStallInfo StallAnalyzer::analyzeMotor(
    MotorState& state,
    const TalonInfoMsg& status,
    const TalonCtrlMsg& command,
    const StallAnalyzerConfig::MotorConfig& motor_config,
    double dt_seconds) const
{
    const double safe_dt_seconds = std::max(0.0, dt_seconds);
    const bool commanded = isCommanded(command, status, motor_config);
    const bool controller_enabled =
        (status.status & TalonInfoMsg::STATUS_PHX_ENABLED) != 0;
    const bool controller_connected =
        (status.status & TalonInfoMsg::STATUS_CONNECTED) != 0;
    const bool ready =
        commanded && controller_enabled && controller_connected;

    const int output_direction = sign(status.output_voltage);
    const int acceleration_direction = sign(status.acceleration);
    const bool acceleration_opposes_output =
        output_direction != 0 &&
        acceleration_direction == -output_direction;
    const bool acceleration_jump =
        acceleration_opposes_output &&
        std::abs(status.acceleration) >=
            motor_config.acceleration_jump_rps_per_second;

    const bool high_current =
        std::abs(status.output_current) >= motor_config.min_output_current_amps;
    const bool large_velocity_error =
        std::abs(command.value - status.velocity) >=
        motor_config.velocity_error_rps;
    const bool confirmation = high_current && large_velocity_error;

    if (!state.initialized)
    {
        state.initialized = true;
    }

    if (ready && state.stall_candidate_seconds <= 0.0 && acceleration_jump)
    {
        state.stall_candidate_seconds = safe_dt_seconds;
        state.recovery_candidate_seconds = 0.0;
    }
    else if (ready && state.stall_candidate_seconds > 0.0 && confirmation)
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
        state.stall_candidate_seconds >= motor_config.debounce_time_seconds)
    {
        state.is_stalled = true;
        state.time_stalled_seconds = state.stall_candidate_seconds;
    }
    else if (
        state.is_stalled &&
        state.recovery_candidate_seconds >=
            this->config.recovery_debounce_seconds)
    {
        state.is_stalled = false;
        state.time_stalled_seconds = 0.0;
    }
    else if (state.is_stalled)
    {
        state.time_stalled_seconds += safe_dt_seconds;
    }

    MotorStallInfo info;
    info.is_stalled = state.is_stalled;
    info.time_stalled_seconds = state.time_stalled_seconds;

    return info;
}

};  // namespace lance
