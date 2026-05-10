#include "stall_analyzer.hpp"

#include <algorithm>
#include <cmath>


namespace lance
{

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
    double dt_seconds)
{
    RobotStallStatus stall_status;
    stall_status.track_right = this->analyzeMotor(
        this->track_right_state,
        motor_status.track_right,
        motor_faults.track_right,
        dt_seconds);
    stall_status.track_left = this->analyzeMotor(
        this->track_left_state,
        motor_status.track_left,
        motor_faults.track_left,
        dt_seconds);
    stall_status.trencher = this->analyzeMotor(
        this->trencher_state,
        motor_status.trencher,
        motor_faults.trencher,
        dt_seconds);
    stall_status.hopper_belt = this->analyzeMotor(
        this->hopper_belt_state,
        motor_status.hopper_belt,
        motor_faults.hopper_belt,
        dt_seconds);
    stall_status.hopper_actuator = this->analyzeMotor(
        this->hopper_actuator_state,
        motor_status.hopper_actuator,
        motor_faults.hopper_actuator,
        dt_seconds);

    return stall_status;
}

MotorStallInfo StallAnalyzer::analyzeMotor(
    MotorState& state,
    const TalonInfoMsg& status,
    const TalonFaultsMsg& faults,
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
        state.initialized = true;
    }
    else
    {
        state.filtered_output_current_amps =
            (alpha * output_current_amps) +
            ((1.0 - alpha) * state.filtered_output_current_amps);
        state.filtered_supply_current_amps =
            (alpha * supply_current_amps) +
            ((1.0 - alpha) * state.filtered_supply_current_amps);
    }

    const bool low_velocity =
        std::abs(status.velocity) <= this->config.max_stall_velocity_rps;
    const bool high_current =
        std::max(
            state.filtered_output_current_amps,
            state.filtered_supply_current_amps) >=
        this->config.min_stall_current_amps;
    const bool stall_candidate = low_velocity && high_current;

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
    info.stator_current_limit_warning = faults.stator_current_limit_fault;
    info.supply_current_limit_warning = faults.supply_current_limit_fault;
    info.current_limit_warning =
        info.stator_current_limit_warning || info.supply_current_limit_warning;
    info.velocity_rps = status.velocity;
    info.output_current_amps = status.output_current;
    info.supply_current_amps = status.supply_current;
    info.filtered_output_current_amps = state.filtered_output_current_amps;
    info.filtered_supply_current_amps = state.filtered_supply_current_amps;
    info.stall_candidate_seconds = state.stall_candidate_seconds;
    info.recovery_candidate_seconds = state.recovery_candidate_seconds;

    return info;
}

};  // namespace lance
