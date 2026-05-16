#include <gtest/gtest.h>

#include "robot/core/stall_analyzer.hpp"


namespace lance
{
namespace
{

TalonCtrlMsg makeVelocityCommand(double value)
{
    TalonCtrlMsg command;
    command.mode = TalonCtrlMsg::VELOCITY;
    command.value = value;
    return command;
}

TalonInfoMsg makeStatus(double velocity)
{
    TalonInfoMsg status;
    status.velocity = velocity;
    return status;
}

TalonFaultsMsg makeFaults(bool stator_current_limit_fault)
{
    TalonFaultsMsg faults;
    faults.stator_current_limit_fault = stator_current_limit_fault;
    return faults;
}

StallAnalyzerConfig makeConfig()
{
    StallAnalyzerConfig config;
    config.tracks = {
        .debounce_time_seconds = 0.2,
        .minimum_velocity_proportion = 0.5,
        .command_deadzone_rps = 0.1};
    config.trencher = {
        .debounce_time_seconds = 0.3,
        .minimum_velocity_proportion = 0.25,
        .command_deadzone_rps = 1.0};
    return config;
}

TEST(StallAnalyzerTest, RequiresFaultAndLowVelocityProportion)
{
    StallAnalyzer analyzer{makeConfig()};
    const auto command = makeVelocityCommand(10.0);
    const auto low_velocity = makeStatus(1.0);

    EXPECT_FALSE(analyzer
                     .analyzeTrack(
                         TrackSide::LEFT,
                         low_velocity,
                         makeFaults(false),
                         command,
                         0.2)
                     .is_stalled);

    analyzer.reset();
    EXPECT_FALSE(analyzer
                     .analyzeTrack(
                         TrackSide::LEFT,
                         makeStatus(8.0),
                         makeFaults(true),
                         command,
                         0.2)
                     .is_stalled);
}

TEST(StallAnalyzerTest, StallsAfterDebounceWhenBothConditionsPersist)
{
    StallAnalyzer analyzer{makeConfig()};
    const auto command = makeVelocityCommand(10.0);
    const auto status = makeStatus(1.0);
    const auto faults = makeFaults(true);

    EXPECT_FALSE(
        analyzer.analyzeTrack(TrackSide::LEFT, status, faults, command, 0.1)
            .is_stalled);
    EXPECT_TRUE(
        analyzer.analyzeTrack(TrackSide::LEFT, status, faults, command, 0.1)
            .is_stalled);
}

TEST(StallAnalyzerTest, OppositeDirectionCountsAsLowSignedProportion)
{
    StallAnalyzer analyzer{makeConfig()};

    EXPECT_TRUE(analyzer
                    .analyzeTrack(
                        TrackSide::LEFT,
                        makeStatus(-10.0),
                        makeFaults(true),
                        makeVelocityCommand(10.0),
                        0.2)
                    .is_stalled);
}

TEST(StallAnalyzerTest, IgnoresCommandsInsideDeadzone)
{
    StallAnalyzer analyzer{makeConfig()};

    EXPECT_FALSE(analyzer
                     .analyzeTrack(
                         TrackSide::LEFT,
                         makeStatus(0.0),
                         makeFaults(true),
                         makeVelocityCommand(0.05),
                         1.0)
                     .is_stalled);
}

TEST(StallAnalyzerTest, UsesSeparateTrackAndTrencherThresholds)
{
    StallAnalyzer analyzer{makeConfig()};
    const auto command = makeVelocityCommand(10.0);
    const auto status = makeStatus(3.0);
    const auto faults = makeFaults(true);

    EXPECT_TRUE(
        analyzer.analyzeTrack(TrackSide::LEFT, status, faults, command, 0.2)
            .is_stalled);
    EXPECT_FALSE(
        analyzer.analyzeTrencher(status, faults, command, 0.3).is_stalled);
}

TEST(StallAnalyzerTest, ClearsAfterSameDebounceInterval)
{
    StallAnalyzer analyzer{makeConfig()};
    const auto command = makeVelocityCommand(10.0);
    const auto faults = makeFaults(true);

    EXPECT_TRUE(analyzer
                    .analyzeTrack(
                        TrackSide::LEFT,
                        makeStatus(1.0),
                        faults,
                        command,
                        0.2)
                    .is_stalled);
    EXPECT_TRUE(analyzer
                    .analyzeTrack(
                        TrackSide::LEFT,
                        makeStatus(8.0),
                        faults,
                        command,
                        0.1)
                    .is_stalled);
    EXPECT_FALSE(analyzer
                     .analyzeTrack(
                         TrackSide::LEFT,
                         makeStatus(8.0),
                         faults,
                         command,
                         0.1)
                     .is_stalled);
}

}  // namespace
}  // namespace lance
