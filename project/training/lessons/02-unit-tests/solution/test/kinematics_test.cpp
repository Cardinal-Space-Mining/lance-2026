// Lesson 2 reference solution: tests for lance/src/robot/model/kinematics.hpp

#include <cmath>

#include <gtest/gtest.h>

#include "robot/model/kinematics.hpp"


namespace
{

using util::kmx::applyTrackLimits;
using util::kmx::decellDist;
using util::kmx::maxStartVel;


// --- maxStartVel / decellDist ------------------------------------------------

TEST(KinematicsTest, MaxStartVelFromRest)
{
    // Vi = sqrt(0^2 + 2 * 1 * 2) = 2
    EXPECT_DOUBLE_EQ(maxStartVel(0.0, 2.0, 1.0), 2.0);
}

TEST(KinematicsTest, MaxStartVelWithEndVelocity)
{
    // Vi = sqrt(3^2 + 2 * 4 * 2) = sqrt(25) = 5
    EXPECT_DOUBLE_EQ(maxStartVel(3.0, 2.0, 4.0), 5.0);
}

TEST(KinematicsTest, DecelerationSignDoesNotMatter)
{
    EXPECT_DOUBLE_EQ(maxStartVel(0.0, 2.0, -1.0), maxStartVel(0.0, 2.0, 1.0));
    EXPECT_DOUBLE_EQ(decellDist(4.0, -2.0), decellDist(4.0, 2.0));
}

TEST(KinematicsTest, DecellDist)
{
    // x = 4^2 / (2 * 2) = 4
    EXPECT_DOUBLE_EQ(decellDist(4.0, 2.0), 4.0);
}

TEST(KinematicsTest, MaxStartVelAndDecellDistAreInverses)
{
    // Works with float too, but floats are less precise, so EXPECT_NEAR
    for (float dist : {0.1f, 1.f, 3.7f, 10.f})
    {
        const float v = maxStartVel(0.f, dist, 1.5f);
        EXPECT_NEAR(decellDist(v, 1.5f), dist, 1e-5f) << "dist = " << dist;
    }
}


// --- applyTrackLimits (5 argument version) -----------------------------------

TEST(TrackLimitsTest, WithinLimitsIsUnchanged)
{
    double l, r;
    applyTrackLimits(0.5, -0.25, 1.0, l, r);
    EXPECT_DOUBLE_EQ(l, 0.5);
    EXPECT_DOUBLE_EQ(r, -0.25);
}

TEST(TrackLimitsTest, ScalesDownAndKeepsRatio)
{
    double l, r;
    applyTrackLimits(2.0, 1.0, 1.0, l, r);
    EXPECT_DOUBLE_EQ(l, 1.0);
    EXPECT_DOUBLE_EQ(r, 0.5);
}

TEST(TrackLimitsTest, ScalesUsingLargestMagnitudeEvenIfNegative)
{
    double l, r;
    applyTrackLimits(-3.0, 1.5, 1.0, l, r);
    EXPECT_DOUBLE_EQ(l, -1.0);
    EXPECT_DOUBLE_EQ(r, 0.5);
}

TEST(TrackLimitsTest, ZeroTargetsDoNotProduceNaN)
{
    double l, r;
    applyTrackLimits(0.0, 0.0, 1.0, l, r);
    EXPECT_DOUBLE_EQ(l, 0.0);
    EXPECT_DOUBLE_EQ(r, 0.0);

    applyTrackLimits(0.0, 3.0, 1.0, l, r);
    EXPECT_DOUBLE_EQ(l, 0.0);
    EXPECT_DOUBLE_EQ(r, 1.0);
}


// --- applyTrackLimits (8 argument version, marked "BROKEN") ------------------
//
// Stretch goal: this test demonstrates one reason the function is marked as
// broken. It is DISABLED_ so the suite stays green; run it on purpose with:
//   ./build/lance/kinematics_test --gtest_also_run_disabled_tests
//
// The target is "spin right in place" (left forward, right backward). The
// previous velocities and max change (Vd_max) make that unreachable this step,
// but the function returns velocities that spin LEFT, the opposite direction.
// Step 3 accepts a negative scale factor `t`, which keeps the curvature but
// reverses the motion. Clamping each side separately would give (0.2, 0.1),
// which at least turns right.
TEST(TrackLimitsTest, DISABLED_DeltaLimitedVersionShouldNotReverseTurn)
{
    double l, r;
    applyTrackLimits(
        1.0,    // Vl_target
        -1.0,   // Vr_target
        0.0,    // Vl_prev
        0.3,    // Vr_prev
        1.0,    // V_max
        0.2,    // Vd_max
        l,
        r);

    // Target turns right (l > r), so the output should too
    EXPECT_GT(l, r) << "output was l = " << l << ", r = " << r;
}

}  // namespace
