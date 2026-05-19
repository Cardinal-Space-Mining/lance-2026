#include "stall_analyzer.hpp"

#include <cmath>
#include <algorithm>


namespace lance
{

StallAnalyzerConfig StallAnalyzerConfig::fromParams(const RobotParams& params)
{
    StallAnalyzerConfig config;

    config.tracks.debounce_time_seconds =
        params.stall_analyzer_tracks_debounce_time_s;
    config.tracks.minimum_velocity_proportion =
        params.stall_analyzer_tracks_min_vel_proportion;
    config.tracks.command_deadzone_rps =
        params.stall_analyzer_tracks_command_deadzone_rps;

    config.trencher.debounce_time_seconds =
        params.stall_analyzer_trencher_debounce_time_s;
    config.trencher.minimum_velocity_proportion =
        params.stall_analyzer_trencher_min_vel_proportion;
    config.trencher.command_deadzone_rps =
        params.stall_analyzer_trencher_command_deadzone_rps;

    return config;
}


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
    const TalonFaultsMsg& faults,
    const TalonCtrlMsg& command,
    double dt_seconds)
{
    MotorState& state = side == TrackSide::LEFT ? this->track_left_state
                                                : this->track_right_state;
    return this->analyzeMotor(
        state,
        status,
        faults,
        command,
        this->config.tracks,
        dt_seconds);
}

MotorStallInfo StallAnalyzer::analyzeTrencher(
    const TalonInfoMsg& status,
    const TalonFaultsMsg& faults,
    const TalonCtrlMsg& command,
    double dt_seconds)
{
    return this->analyzeMotor(
        this->trencher_state,
        status,
        faults,
        command,
        this->config.trencher,
        dt_seconds);
}

MotorStallInfo StallAnalyzer::analyzeMotor(
    MotorState& state,
    const TalonInfoMsg& status,
    const TalonFaultsMsg& faults,
    const TalonCtrlMsg& command,
    const StallAnalyzerConfig::MotorConfig& motor_config,
    double dt_seconds) const
{
    const double safe_dt_seconds = std::max(0.0, dt_seconds);
    const bool command_outside_deadzone =
        std::abs(command.value) > motor_config.command_deadzone_rps;
    const double velocity_proportion =
        command_outside_deadzone ? status.velocity / command.value : 0.0;
    const bool stall_condition =
        command.mode == TalonCtrlMsg::VELOCITY && command_outside_deadzone &&
        faults.stator_current_limit_fault &&
        velocity_proportion < motor_config.minimum_velocity_proportion;

    if (!state.initialized)
    {
        state.initialized = true;
    }

    if (stall_condition)
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
        state.recovery_candidate_seconds >= motor_config.debounce_time_seconds)
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

StallState::StallState(StallAnalyzerConfig config) : analyzer{config} {}

void StallState::setConfig(const StallAnalyzerConfig& config)
{
    this->analyzer.setConfig(config);
}

void StallState::reset()
{
    this->analyzer.reset();
    this->track_left = {};
    this->track_right = {};
    this->trencher_info = {};
}

void StallState::update(
    const RobotMotorStatus& status,
    const RobotMotorFaults& faults,
    const RobotMotorCommands& commands,
    double dt_seconds)
{
    this->track_left = this->analyzer.analyzeTrack(
        TrackSide::LEFT,
        status.track_left,
        faults.track_left,
        commands.track_left,
        dt_seconds);
    this->track_right = this->analyzer.analyzeTrack(
        TrackSide::RIGHT,
        status.track_right,
        faults.track_right,
        commands.track_right,
        dt_seconds);
    this->trencher_info = this->analyzer.analyzeTrencher(
        status.trencher,
        faults.trencher,
        commands.trencher,
        dt_seconds);
}

};  // namespace lance
