#pragma once

/**
 * @file stall_analyzer.hpp
 * @brief Debounced stall and jam detection for track drive and trencher motors.
 *
 * Continuously evaluates CTRE Talon FX motor feedback (current limit faults and velocity deficits)
 * to detect mechanical jams or track stalls in lunar regolith. Uses dual debounce timers for both
 * stall entry and stall recovery to prevent spurious triggers from short current spikes.
 */

#include "robot_params.hpp"
#include "motor_interface.hpp"


namespace lance
{

/**
 * @struct StallAnalyzerConfig
 * @brief Tuning thresholds for motor stall classification.
 */
struct StallAnalyzerConfig
{
    /**
     * @struct MotorConfig
     * @brief Per-motor stall evaluation parameters.
     */
    struct MotorConfig
    {
        double debounce_time_seconds{0.25};       ///< Duration stall/recovery condition must persist before toggling flag.
        double minimum_velocity_proportion{0.20}; ///< Actual / commanded velocity ratio below which motor is considered stalled.
        double command_deadzone_rps{0.01};        ///< Velocity command magnitude below which stall evaluation is skipped.
    };

    MotorConfig tracks{
        .debounce_time_seconds = 0.25,
        .minimum_velocity_proportion = 0.20,
        .command_deadzone_rps = 0.01}; ///< Track drive motor parameters.

    MotorConfig trencher{
        .debounce_time_seconds = 0.25,
        .minimum_velocity_proportion = 0.20,
        .command_deadzone_rps = 1.0};  ///< Trencher cutter head parameters.

    /**
     * @brief Populate configuration values from loaded RobotParams.
     */
    static StallAnalyzerConfig fromParams(const RobotParams&);
};

/**
 * @struct MotorStallInfo
 * @brief Current stall diagnosis and cumulative stalled duration for a motor.
 */
struct MotorStallInfo
{
    bool is_stalled{false};             ///< True if motor is actively diagnosed as stalled/jammed.
    double time_stalled_seconds{0.0};   ///< Total continuous time spent in stalled state (seconds).
};

/**
 * @enum TrackSide
 * @brief Identifies left vs right track channel for parameter lookup and state tracking.
 */
enum class TrackSide
{
    LEFT,
    RIGHT
};

/**
 * @class StallAnalyzer
 * @brief Low-level stateful analyzer tracking debounce timers and evaluating stall predicates.
 */
class StallAnalyzer
{
public:
    explicit StallAnalyzer(StallAnalyzerConfig config = {});

    const StallAnalyzerConfig& getConfig() const;
    void setConfig(const StallAnalyzerConfig& config);
    void reset();

    /**
     * @brief Evaluate stall condition for a track motor.
     * @param side LEFT or RIGHT track.
     * @param status Motor telemetry from TalonInfo.
     * @param faults Motor controller faults from TalonFaults.
     * @param command Commanded output setpoint.
     * @param dt_seconds Time step duration since last evaluation tick.
     * @return Updated stall diagnosis.
     */
    MotorStallInfo analyzeTrack(
        TrackSide side,
        const TalonInfoMsg& status,
        const TalonFaultsMsg& faults,
        const TalonCtrlMsg& command,
        double dt_seconds);

    /**
     * @brief Evaluate stall condition for the trencher motor.
     */
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
        double stall_candidate_seconds{0.0};    ///< Accumulated time meeting stall criteria.
        double recovery_candidate_seconds{0.0}; ///< Accumulated time meeting normal criteria.
        double time_stalled_seconds{0.0};       ///< Total continuous time in confirmed stalled state.
    };

    /**
     * @brief Common evaluation logic across all motor channels.
     */
    MotorStallInfo analyzeMotor(
        MotorState& state,
        const TalonInfoMsg& status,
        const TalonFaultsMsg& faults,
        const TalonCtrlMsg& command,
        const StallAnalyzerConfig::MotorConfig& motor_config,
        double dt_seconds) const;

    StallAnalyzerConfig config;
    MotorState track_right_state;
    MotorState track_left_state;
    MotorState trencher_state;
};

/**
 * @class StallState
 * @brief High-level container maintaining stall diagnosis across left track, right track, and trencher.
 */
class StallState
{
public:
    explicit StallState(StallAnalyzerConfig config = {});

    void setConfig(const StallAnalyzerConfig&);
    void reset();

    /**
     * @brief Step the analyzer forward across all active robot motor channels.
     */
    void update(
        const RobotMotorStatus&,
        const RobotMotorFaults&,
        const RobotMotorCommands&,
        double dt_seconds);

    inline const MotorStallInfo& trackLeft() const
    {
        return this->track_left;
    }
    inline const MotorStallInfo& trackRight() const
    {
        return this->track_right;
    }
    inline const MotorStallInfo& trencher() const { return this->trencher_info; }

    /// @brief True if any track or the trencher is currently stalled.
    inline bool anyStalled() const
    {
        return this->track_left.is_stalled || this->track_right.is_stalled ||
               this->trencher_info.is_stalled;
    }

private:
    StallAnalyzer analyzer;
    MotorStallInfo track_left;
    MotorStallInfo track_right;
    MotorStallInfo trencher_info;
};

};  // namespace lance
