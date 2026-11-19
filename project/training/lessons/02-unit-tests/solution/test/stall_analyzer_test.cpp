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

// --- New tests (Lesson 2) ----------------------------------------------------

TEST(StallAnalyzerTest, NonVelocityModesNeverStall)
{
    StallAnalyzer analyzer{makeConfig()};
    TalonCtrlMsg command;
    command.mode = TalonCtrlMsg::PERCENT_OUTPUT;
    command.value = 10.0;

    // Stall-like conditions for a full second, but not in velocity mode
    for (int i = 0; i < 10; i++)
    {
        EXPECT_FALSE(analyzer
                         .analyzeTrack(
                             TrackSide::LEFT,
                             makeStatus(0.0),
                             makeFaults(true),
                             command,
                             0.1)
                         .is_stalled);
    }
}

TEST(StallAnalyzerTest, LeftAndRightTracksAreIndependent)
{
    StallAnalyzer analyzer{makeConfig()};
    const auto command = makeVelocityCommand(10.0);

    EXPECT_TRUE(analyzer
                    .analyzeTrack(
                        TrackSide::LEFT,
                        makeStatus(1.0),
                        makeFaults(true),
                        command,
                        0.2)
                    .is_stalled);
    EXPECT_FALSE(analyzer
                     .analyzeTrack(
                         TrackSide::RIGHT,
                         makeStatus(10.0),
                         makeFaults(false),
                         command,
                         0.2)
                     .is_stalled);
}

TEST(StallAnalyzerTest, TimeStalledAccumulates)
{
    StallAnalyzer analyzer{makeConfig()};
    const auto command = makeVelocityCommand(10.0);
    const auto status = makeStatus(1.0);
    const auto faults = makeFaults(true);

    // Stall begins once the 0.2s debounce has passed...
    auto info =
        analyzer.analyzeTrack(TrackSide::LEFT, status, faults, command, 0.2);
    ASSERT_TRUE(info.is_stalled);
    EXPECT_NEAR(info.time_stalled_seconds, 0.2, 1e-9);

    // ...and keeps counting while it continues
    info = analyzer.analyzeTrack(TrackSide::LEFT, status, faults, command, 0.1);
    EXPECT_NEAR(info.time_stalled_seconds, 0.3, 1e-9);
}

TEST(StallAnalyzerTest, NegativeTimeStepsAreIgnored)
{
    StallAnalyzer analyzer{makeConfig()};

    EXPECT_FALSE(analyzer
                     .analyzeTrack(
                         TrackSide::LEFT,
                         makeStatus(1.0),
                         makeFaults(true),
                         makeVelocityCommand(10.0),
                         -5.0)
                     .is_stalled);
}

TEST(StallAnalyzerTest, DetectsStallWhileReversing)
{
    StallAnalyzer analyzer{makeConfig()};
    const auto command = makeVelocityCommand(-10.0);

    // Moving backwards at full speed: fine
    EXPECT_FALSE(analyzer
                     .analyzeTrack(
                         TrackSide::LEFT,
                         makeStatus(-9.0),
                         makeFaults(true),
                         command,
                         0.2)
                     .is_stalled);
    // Barely moving backwards: stalled
    EXPECT_TRUE(analyzer
                    .analyzeTrack(
                        TrackSide::LEFT,
                        makeStatus(-1.0),
                        makeFaults(true),
                        command,
                        0.2)
                    .is_stalled);
}

TEST(StallAnalyzerTest, ResetClearsStall)
{
    StallAnalyzer analyzer{makeConfig()};
    ASSERT_TRUE(analyzer
                    .analyzeTrencher(
                        makeStatus(0.0),
                        makeFaults(true),
                        makeVelocityCommand(10.0),
                        0.3)
                    .is_stalled);

    analyzer.reset();

    // Healthy input right after reset: not stalled (no recovery delay needed)
    EXPECT_FALSE(analyzer
                     .analyzeTrencher(
                         makeStatus(10.0),
                         makeFaults(false),
                         makeVelocityCommand(10.0),
                         0.01)
                     .is_stalled);
}

TEST(StallStateTest, AnyStalledReportsTrencherStall)
{
    StallState state{makeConfig()};
    RobotMotorStatus status;
    RobotMotorFaults faults;
    RobotMotorCommands commands;

    commands.setTracksVelocity(10.0, 10.0);
    commands.setTrencherVelocity(10.0);
    status.track_left.velocity = 10.0;
    status.track_right.velocity = 10.0;
    status.trencher.velocity = 0.0;
    faults.trencher.stator_current_limit_fault = true;

    state.update(status, faults, commands, 0.3);

    EXPECT_FALSE(state.trackLeft().is_stalled);
    EXPECT_FALSE(state.trackRight().is_stalled);
    EXPECT_TRUE(state.trencher().is_stalled);
    EXPECT_TRUE(state.anyStalled());
}

}  // namespace
}  // namespace lance
