#pragma once

#include "robot_params.hpp"
#include "motor_interface.hpp"


namespace lance
{

struct StallAnalyzerConfig
{
    struct MotorConfig
    {
        double debounce_time_seconds{0.25};
        double minimum_velocity_proportion{0.20};
        double command_deadzone_rps{0.01};
    };

    MotorConfig tracks{
        .debounce_time_seconds = 0.25,
        .minimum_velocity_proportion = 0.20,
        .command_deadzone_rps = 0.01};
    MotorConfig trencher{
        .debounce_time_seconds = 0.25,
        .minimum_velocity_proportion = 0.20,
        .command_deadzone_rps = 1.0};

    static StallAnalyzerConfig fromParams(const RobotParams&);
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
        const TalonFaultsMsg& faults,
        const TalonCtrlMsg& command,
        const StallAnalyzerConfig::MotorConfig& motor_config,
        double dt_seconds) const;

    StallAnalyzerConfig config;
    MotorState track_right_state;
    MotorState track_left_state;
    MotorState trencher_state;
};

class StallState
{
public:
    explicit StallState(StallAnalyzerConfig config = {});

    void setConfig(const StallAnalyzerConfig&);
    void reset();
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
