/*******************************************************************************
*   Copyright (C) 2025-2026 Cardinal Space Mining Club                         *
*                                                                              *
*                                 ;xxxxxxx:                                    *
*                                ;$$$$$$$$$       ...::..                      *
*                                $$$$$$$$$$x   .:::::::::::..                  *
*                             x$$$$$$$$$$$$$$::::::::::::::::.                 *
*                         :$$$$$&X;      .xX:::::::::::::.::...                *
*                 .$$Xx++$$$$+  :::.     :;:   .::::::.  ....  :               *
*                :$$$$$$$$$  ;:      ;xXXXXXXXx  .::.  .::::. .:.              *
*               :$$$$$$$$: ;      ;xXXXXXXXXXXXXx: ..::::::  .::.              *
*              ;$$$$$$$$ ::   :;XXXXXXXXXXXXXXXXXX+ .::::.  .:::               *
*               X$$$$$X : +XXXXXXXXXXXXXXXXXXXXXXXX; .::  .::::.               *
*                .$$$$ :xXXXXXXXXXXXXXXXXXXXXXXXXXXX.   .:::::.                *
*                 X$$X XXXXXXXXXXXXXXXXXXXXXXXXXXXXx:  .::::.                  *
*                 $$$:.XXXXXXXXXXXXXXXXXXXXXXXXXXX  ;; ..:.                    *
*                 $$& :XXXXXXXXXXXXXXXXXXXXXXXX;  +XX; X$$;                    *
*                 $$$: XXXXXXXXXXXXXXXXXXXXXX; :XXXXX; X$$;                    *
*                 X$$X XXXXXXXXXXXXXXXXXXX; .+XXXXXXX; $$$                     *
*                 $$$$ ;XXXXXXXXXXXXXXX+  +XXXXXXXXx+ X$$$+                    *
*               x$$$$$X ;XXXXXXXXXXX+ :xXXXXXXXX+   .;$$$$$$                   *
*              +$$$$$$$$ ;XXXXXXx;;+XXXXXXXXX+    : +$$$$$$$$                  *
*               +$$$$$$$$: xXXXXXXXXXXXXXX+      ; X$$$$$$$$                   *
*                :$$$$$$$$$. +XXXXXXXXX;      ;: x$$$$$$$$$                    *
*                ;x$$$$XX$$$$+ .;+X+      :;: :$$$$$xX$$$X                     *
*               ;;;;;;;;;;X$$$$$$$+      :X$$$$$$&.                            *
*               ;;;;;;;:;;;;;x$$$$$$$$$$$$$$$$x.                               *
*               :;;;;;;;;;;;;.  :$$$$$$$$$$X                                   *
*                .;;;;;;;;:;;    +$$$$$$$$$                                    *
*                  .;;;;;;.       X$$$$$$$:                                    *
*                                                                              *
*   Unless required by applicable law or agreed to in writing, software        *
*   distributed under the License is distributed on an "AS IS" BASIS,          *
*   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.   *
*   See the License for the specific language governing permissions and        *
*   limitations under the License.                                             *
*                                                                              *
*******************************************************************************/

#pragma once

/**
 * @file dynamics.hpp
 * @brief Mechanical constants, kinematics, and volumetric dynamics models for LANCE.
 *
 * This header defines:
 *   - Hardware specifications for LANCE-1 and LANCE-2 robot platforms:
 *       * Differential track drive gearing, sprocket pitch radius, and track gauge.
 *       * Trencher cutter head dimensions, gearing, bucket spacing, and volumetric capacity.
 *       * Hopper conveyor belt geometry, gearing, and safe offload parameters.
 *       * Linear actuator calibration models mapping normalized stroke [0.0, 1.0] to cutting depth.
 *   - Kinematic conversion routines:
 *       * Actuator RPS <-> linear track/belt surface velocity (m/s).
 *       * Differential drive forward and inverse kinematics.
 *       * Linear actuator position <-> digging depth & joint tilt angle.
 *   - Volumetric excavation models:
 *       * Trencher circular segment excavation volume during plunge cuts.
 *       * Dynamic coupling between ground advance rate, trencher speed, and collection throughput.
 *       * Sweep distance estimation for target payload mass/volume.
 */

#include <cmath>
#include <limits>
#include <numbers>
#include <algorithm>


#ifndef LANCE
    #define LANCE 2
#endif

#define TWO_PI             (std::numbers::pi * 2)
#define RADIANS_PER_DEGREE (std::numbers::pi / 180)
#define LITERS_PER_M_CUBED (1000)

/**
 * @brief Helper macro defining both a templated constant (e.g. name_<float>) and a double alias (name).
 *
 * This pattern allows generic template functions to operate without static_cast overhead on floating types,
 * while still exposing standard double constants for non-templated code.
 */
#define CONSTEXPR_VAL_TEMPLATE(name, val)             \
    template<typename T>                              \
    inline constexpr T name##_ = static_cast<T>(val); \
    inline constexpr double name = name##_<double>;

namespace lance
{

#if LANCE <= 1
// =============================================================================
// LANCE-1 Platform Parameters
// =============================================================================

/// Gear reduction ratio between track drive motors (Talon FX) and track sprockets.
CONSTEXPR_VAL_TEMPLATE(TRACK_GEARING, 64)
/// Effective rolling pitch radius of the drive track sprockets (meters).
CONSTEXPR_VAL_TEMPLATE(TRACK_EFFECTIVE_OUTPUT_RADIUS_M, 0.07032851)
/// Center-to-center track gauge separation (meters).
CONSTEXPR_VAL_TEMPLATE(TRACK_SEPARATION_M, 0.636)

/// Effective cutting width of the trencher chain/drum (meters).
CONSTEXPR_VAL_TEMPLATE(TRENCHER_WIDTH_M, 0.254)
/// Gear reduction ratio for the trencher drive gearbox.
CONSTEXPR_VAL_TEMPLATE(TRENCHER_GEARING, 32)
// bucket separation (CAD): 0.05107 m, actuation radius (CAD): 0.04890 m,
// strict bucket volume (CAD): 0.04309 L
/// Regolith excavation capacity per output sprocket rotation (Liters/rev).
CONSTEXPR_VAL_TEMPLATE(
    TRENCHER_LITERS_PER_OUTPUT_ROTATION,
    (0.04309 * ((0.04826 * TWO_PI) / 0.05107)))
/// Effective radial distance from cutter pivot to outer bucket tips (meters).
CONSTEXPR_VAL_TEMPLATE(TRENCHER_IMPACT_EFFECTIVE_RADIUS_M, 0.09270911)

/// Trencher tilt angle at lowest actuator stroke (degrees).
CONSTEXPR_VAL_TEMPLATE(ACTUATOR_LOWEST_ANGLE_DEG, 15)
/// Trencher tilt angle at highest actuator stroke (degrees).
CONSTEXPR_VAL_TEMPLATE(ACTUATOR_HIGHEST_ANGLE_DEG, -15)

/// Gear reduction ratio between hopper conveyor motor and drive roller.
CONSTEXPR_VAL_TEMPLATE(HOPPER_BELT_GEARING, 100)
/// Effective radius of the hopper conveyor drive roller (meters).
CONSTEXPR_VAL_TEMPLATE(HOPPER_BELT_EFFECTIVE_OUTPUT_RADIUS_M, 0.0508)
/// Usable length of the regolith collection hopper container bed (meters).
CONSTEXPR_VAL_TEMPLATE(HOPPER_BELT_CONTAINER_LENGTH_M, 0.6)
/// Safe linear travel distance of the conveyor belt to complete offload (meters).
CONSTEXPR_VAL_TEMPLATE(HOPPER_BELT_SAFE_OFFLOAD_DIST_M, 0.7)
/// Conservative rated storage capacity of the regolith hopper (Liters).
CONSTEXPR_VAL_TEMPLATE(CONSERVATIVE_HOPPER_CAPACITY_L, 30)

// Linear calibration fit for digging depth: depth = slope * actuator_pos + offset
// (mining depth) ~ -0.3159 * (normalized actuator pos) + 0.1129
CONSTEXPR_VAL_TEMPLATE(MINING_DEPTH_FX_OFFSET, 0.1129)
CONSTEXPR_VAL_TEMPLATE(MINING_DEPTH_FX_SLOPE, -0.3159)
/// Maximum trenching depth below ground surface (meters) [~4 inches].
CONSTEXPR_VAL_TEMPLATE(MINING_MAX_DEPTH_M, 0.1016)

#elif LANCE >= 2
// =============================================================================
// LANCE-2 Platform Parameters
// =============================================================================

/// Gear reduction ratio between track drive motors (Talon FX) and track sprockets.
CONSTEXPR_VAL_TEMPLATE(TRACK_GEARING, 64)
/// Effective rolling pitch radius of the drive track sprockets (meters).
CONSTEXPR_VAL_TEMPLATE(TRACK_EFFECTIVE_OUTPUT_RADIUS_M, 0.045)
/// Center-to-center track gauge separation (meters).
CONSTEXPR_VAL_TEMPLATE(TRACK_SEPARATION_M, 0.632)

/// Effective cutting width of the trencher chain/drum (meters).
CONSTEXPR_VAL_TEMPLATE(TRENCHER_WIDTH_M, 0.260)
/// Gear reduction ratio for the trencher drive gearbox.
CONSTEXPR_VAL_TEMPLATE(TRENCHER_GEARING, 64)
// VOLUME / OUTPUT ROTATION = VOLUME / DISTANCE * DISTANCE / OUTPUT ROTATON =
// (VOLUME / 1 bucket * NUM buckets) / BELT LENGTH * OUTPUT RADIUS / 2pi -->
// belt length : ~62 in --> 1.5748 m
// num buckets : ~44
// vol per bucket (CAD) : 0.106 L
// actuation radius (CAD): 0.04890 m,
/// Regolith excavation capacity per output sprocket rotation (Liters/rev).
CONSTEXPR_VAL_TEMPLATE(
    TRENCHER_LITERS_PER_OUTPUT_ROTATION,
    ((0.106 * 44) / 1.5748) * (0.04826 * TWO_PI))
/// Effective radial distance from cutter pivot to outer bucket tips (meters).
CONSTEXPR_VAL_TEMPLATE(TRENCHER_IMPACT_EFFECTIVE_RADIUS_M, 0.0953)

/// Trencher tilt angle at lowest actuator stroke (degrees).
CONSTEXPR_VAL_TEMPLATE(ACTUATOR_LOWEST_ANGLE_DEG, 10)
/// Trencher tilt angle at highest actuator stroke (degrees).
CONSTEXPR_VAL_TEMPLATE(ACTUATOR_HIGHEST_ANGLE_DEG, -10)

/// Gear reduction ratio between hopper conveyor motor and drive roller.
CONSTEXPR_VAL_TEMPLATE(HOPPER_BELT_GEARING, 64)
/// Effective radius of the hopper conveyor drive roller (meters).
CONSTEXPR_VAL_TEMPLATE(HOPPER_BELT_EFFECTIVE_OUTPUT_RADIUS_M, 0.028)
/// Usable length of the regolith collection hopper container bed (meters).
CONSTEXPR_VAL_TEMPLATE(HOPPER_BELT_CONTAINER_LENGTH_M, 0.75)
/// Safe linear travel distance of the conveyor belt to complete offload (meters).
CONSTEXPR_VAL_TEMPLATE(HOPPER_BELT_SAFE_OFFLOAD_DIST_M, 0.8)
/// Conservative rated storage capacity of the regolith hopper (Liters).
CONSTEXPR_VAL_TEMPLATE(CONSERVATIVE_HOPPER_CAPACITY_L, 45)

// Linear calibration fit for digging depth: depth = slope * actuator_pos + offset
// (mining depth) ~ -0.3425 * (normalized actuator pos) + 0.1112
CONSTEXPR_VAL_TEMPLATE(MINING_DEPTH_FX_OFFSET, 0.1108)
CONSTEXPR_VAL_TEMPLATE(MINING_DEPTH_FX_SLOPE, -0.3397)
/// Maximum trenching depth below ground surface (meters).
CONSTEXPR_VAL_TEMPLATE(MINING_MAX_DEPTH_M, 0.1108)
#endif


// =============================================================================
// Track Kinematics Conversions
// =============================================================================

/**
 * @brief Convert track motor shaft rotational speed (RPS) to linear ground speed (m/s).
 *
 * Formula: v = rps * (1 / G_track) * (r_sprocket * 2 * pi)
 *
 * @tparam T Floating-point type.
 * @param rps Motor shaft revolutions per second.
 * @return Ground speed in meters per second.
 */
template<typename T>
constexpr inline T trackMotorRpsToGroundMps(const T& rps)
{
    return static_cast<T>(
        rps * ((1 / TRACK_GEARING) * TRACK_EFFECTIVE_OUTPUT_RADIUS_M * TWO_PI));
}

/**
 * @brief Convert linear ground speed (m/s) to track motor shaft rotational speed (RPS).
 *
 * Formula: rps = v * (1 / (r_sprocket * 2 * pi)) * G_track
 *
 * @tparam T Floating-point type.
 * @param mps Ground speed in meters per second.
 * @return Motor shaft revolutions per second.
 */
template<typename T>
constexpr inline T groundMpsToTrackMotorRps(const T& mps)
{
    return static_cast<T>(
        mps * (1 / (TRACK_EFFECTIVE_OUTPUT_RADIUS_M * TWO_PI) * TRACK_GEARING));
}

/**
 * @brief Compute robot forward body linear velocity from individual left and right track speeds.
 *
 * Differential drive forward kinematics: v = (v_left + v_right) / 2
 *
 * @tparam T Floating-point type.
 * @param left_mps Left track linear speed (m/s).
 * @param right_mps Right track linear speed (m/s).
 * @return Forward linear velocity (m/s).
 */
template<typename T>
constexpr inline T trackVelocitiesToForwardVelocity(
    const T& left_mps,
    const T& right_mps)
{
    return static_cast<T>((left_mps + right_mps) / 2.);
}

/**
 * @brief Compute robot yaw angular velocity from individual left and right track speeds.
 *
 * Differential drive forward kinematics: omega = (v_left - v_right) / W_track
 * Positive angular velocity corresponds to counter-clockwise (CCW / left turn) rotation.
 *
 * @tparam T Floating-point type.
 * @param left_mps Left track linear speed (m/s).
 * @param right_mps Right track linear speed (m/s).
 * @return Angular velocity in radians per second.
 */
template<typename T>
constexpr inline T trackVelocitiesToAngularVelocity(
    const T& left_mps,
    const T& right_mps)
{
    return static_cast<T>((left_mps - right_mps) / TRACK_SEPARATION_M);
}

/**
 * @brief Compute left track linear speed from body linear and angular velocities.
 *
 * Differential drive inverse kinematics: v_left = v - omega * (W_track / 2)
 *
 * @tparam T Floating-point type.
 * @param v_mps Body linear velocity along forward axis (m/s).
 * @param omega_radps Body angular yaw velocity (rad/s).
 * @return Left track linear speed (m/s).
 */
template<typename T>
constexpr inline T bodyDynamicsToLeftTrackVelocityMps(
    const T& v_mps,
    const T& omega_radps)
{
    return static_cast<T>(v_mps - omega_radps * (TRACK_SEPARATION_M / 2));
}

/**
 * @brief Compute right track linear speed from body linear and angular velocities.
 *
 * Differential drive inverse kinematics: v_right = v + omega * (W_track / 2)
 *
 * @tparam T Floating-point type.
 * @param v_mps Body linear velocity along forward axis (m/s).
 * @param omega_radps Body angular yaw velocity (rad/s).
 * @return Right track linear speed (m/s).
 */
template<typename T>
constexpr inline T bodyDynamicsToRightTrackVelocityMps(
    const T& v_mps,
    const T& omega_radps)
{
    return static_cast<T>(v_mps + omega_radps * (TRACK_SEPARATION_M / 2));
}


// =============================================================================
// Linear Actuator & Digging Depth Calibration
// =============================================================================

/**
 * @brief Convert normalized linear actuator position [0.0, 1.0] to raw digging depth without clamping.
 *
 * Uses the linear fit: depth = OFFSET + SLOPE * pos
 *
 * @tparam T Floating-point type.
 * @param actuator_normalized_pos Normalized stroke (0.0 = fully retracted, 1.0 = fully extended).
 * @return Digging depth in meters.
 */
template<typename T>
constexpr inline T linearActuatorToMiningDepthUnclamped(
    const T& actuator_normalized_pos)
{
    return static_cast<T>(
        MINING_DEPTH_FX_OFFSET +
        MINING_DEPTH_FX_SLOPE * actuator_normalized_pos);
}

/**
 * @brief Convert digging depth to raw normalized actuator stroke without clamping.
 *
 * Inverse linear fit: pos = (depth - OFFSET) / SLOPE
 *
 * @tparam T Floating-point type.
 * @param depth_m Digging depth in meters.
 * @return Normalized actuator position [0.0, 1.0].
 */
template<typename T>
constexpr inline T miningDepthToLinearActuatorUnclamped(const T& depth_m)
{
    return static_cast<T>(
        (depth_m - MINING_DEPTH_FX_OFFSET) * (1 / MINING_DEPTH_FX_SLOPE));
}

/**
 * @brief Convert normalized actuator position [0.0, 1.0] to digging depth clamped to [0, MINING_MAX_DEPTH_M].
 *
 * Clamps result between zero depth (ground surface) and maximum mechanically permissible trenching depth.
 *
 * @tparam T Floating-point type.
 * @param actuator_normalized_pos Normalized stroke.
 * @return Clamped digging depth in meters.
 */
template<typename T>
constexpr inline T linearActuatorToMiningDepthClamped(
    const T& actuator_normalized_pos)
{
    return static_cast<T>(std::clamp<double>(
        linearActuatorToMiningDepthUnclamped<double>(
            static_cast<double>(actuator_normalized_pos)),
        0.,
        MINING_MAX_DEPTH_M));
}

/**
 * @brief Convert desired digging depth to clamped normalized linear actuator position.
 *
 * Clamps input depth to [0, MINING_MAX_DEPTH_M] before computing actuator stroke.
 *
 * @tparam T Floating-point type.
 * @param depth_m Desired digging depth in meters.
 * @return Normalized actuator position [0.0, 1.0].
 */
template<typename T>
constexpr inline T miningDepthToLinearActuatorClamped(const T& depth_m)
{
    return static_cast<T>(miningDepthToLinearActuatorUnclamped(
        std::clamp<double>(
            static_cast<double>(depth_m),
            0.,
            MINING_MAX_DEPTH_M)));
}

/**
 * @brief Convert normalized linear actuator position to trencher joint pitch angle in radians.
 *
 * Linearly maps [0.0, 1.0] to [ACTUATOR_LOWEST_ANGLE_DEG, ACTUATOR_HIGHEST_ANGLE_DEG] in radians.
 * Used for publishing joint states to TF / URDF visualization.
 *
 * @tparam T Floating-point type.
 * @param actuator_normalized_pos Normalized actuator stroke [0.0, 1.0].
 * @return Joint angle in radians.
 */
template<typename T>
constexpr inline T linearActuatorToJointAngle(const T& actuator_normalized_pos)
{
    constexpr double ACUTATION_RANGE_DEGREES =
        (ACTUATOR_HIGHEST_ANGLE_DEG - ACTUATOR_LOWEST_ANGLE_DEG);

    return static_cast<T>(
        RADIANS_PER_DEGREE *
        (ACTUATOR_LOWEST_ANGLE_DEG +
         actuator_normalized_pos * ACUTATION_RANGE_DEGREES));
}


// =============================================================================
// Volumetric Excavation & Trencher Dynamics
// =============================================================================

/**
 * @brief Calculate volume of regolith excavated during a stationary vertical plunge cut.
 *
 * When plunging the trencher into regolith to depth d, the cut cross-section is modeled as:
 *   - For d < R: A circular segment of radius R:
 *       Area = R^2 * acos((R - d) / R) - (R - d) * sqrt(2 * R * d - d^2)
 *   - For d >= R: A semi-circle plus a rectangular column extending below:
 *       Area = (pi / 2) * R^2 + (d - R) * R
 * The total volume is Area * Width * 1000 Liters/m^3.
 *
 * @tparam T Floating-point type.
 * @param depth_m Excavation depth in meters.
 * @return Impact volume in Liters.
 */
template<typename T>
constexpr inline T miningDepthToTrencherImpactVolume(const T& depth_m)
{
    constexpr double R = TRENCHER_IMPACT_EFFECTIVE_RADIUS_M;
    constexpr double R2 = (R * R);
    const double d = static_cast<double>(depth_m);

    if (d <= 0)
    {
        return static_cast<T>(0);
    }

    double cross_section_area = 0;
    if (d < R)
    {
        // Circular segment area formula given radius R and cut depth d:
        cross_section_area = (R2 * std::acos((R - d) / R)) -
                             ((R - d) * std::sqrt(2 * R * d - d * d));
    }
    else
    {
        // Half-circle cross-section + rectangular extension for depths exceeding radius
        cross_section_area =
            (0.5 * std::numbers::pi * R2) + ((depth_m - R) * R);
    }
    // cross-section * width * 1000 liters/m^3
    return static_cast<T>(
        cross_section_area * TRENCHER_WIDTH_M * LITERS_PER_M_CUBED);
}

/**
 * @brief Calculate regolith volumetric intake rate (L/s) given track motor RPS and cutting depth.
 *
 * Volumetric rate = ground_speed * depth * width * 1000 L/m^3
 *
 * @tparam T Floating-point type.
 * @param motor_rps Track drive motor shaft rotational speed (RPS).
 * @param depth_m Excavation trench depth (meters).
 * @return Volume rate in Liters per second.
 */
template<typename T>
constexpr inline T trackMotorRpsToVolumeRate(
    const T& motor_rps,
    const T& depth_m)
{
    // dist * depth * width * 1000 liters/m^3
    return static_cast<T>(
        trackMotorRpsToGroundMps<double>(static_cast<double>(motor_rps)) *
        depth_m * (TRENCHER_WIDTH_M * LITERS_PER_M_CUBED));
}

/**
 * @brief Calculate required track drive motor RPS to achieve a desired volumetric excavation rate.
 *
 * Inverse of trackMotorRpsToVolumeRate().
 *
 * @tparam T Floating-point type.
 * @param vol_rate_lps Desired volume intake rate in Liters per second.
 * @param depth_m Trench depth in meters.
 * @return Track motor shaft speed in RPS.
 */
template<typename T>
constexpr inline T volumeRateToTrackMotorRps(
    const T& vol_rate_lps,
    const T& depth_m)
{
    // (vol rate * 0.001 m^3/liter) / (depth * width)
    return static_cast<T>(groundMpsToTrackMotorRps<double>(
        (static_cast<double>(vol_rate_lps) / static_cast<double>(depth_m)) *
        ((1 / TRENCHER_WIDTH_M) / LITERS_PER_M_CUBED)));
}

/**
 * @brief Calculate the maximum regolith volume rate that the trencher can convey at a given motor speed.
 *
 * Rate = rps * (1 / G_trencher) * Liters_per_output_rotation
 *
 * @tparam T Floating-point type.
 * @param rps Trencher motor shaft rotational speed (RPS).
 * @return Maximum conveyable volume rate in Liters per second.
 */
template<typename T>
constexpr inline T trencherMotorRpsToMaxVolumeRate(const T& rps)
{
    return static_cast<T>(
        rps * ((1 / TRENCHER_GEARING) * TRENCHER_LITERS_PER_OUTPUT_ROTATION));
}

/**
 * @brief Calculate required trencher motor speed (RPS) to handle a target regolith volumetric rate.
 *
 * Inverse of trencherMotorRpsToMaxVolumeRate().
 *
 * @tparam T Floating-point type.
 * @param vol_rate_lps Desired volume rate in Liters per second.
 * @return Required trencher motor RPS.
 */
template<typename T>
constexpr inline T targetVolRateToTrencherMotorRps(const T& vol_rate_lps)
{
    return static_cast<T>(
        vol_rate_lps *
        ((1 / TRENCHER_LITERS_PER_OUTPUT_ROTATION) * TRENCHER_GEARING));
}

/**
 * @brief Compute the maximum permissible track motor RPS such that the excavated volume
 * does not exceed the trencher's conveying capacity at the given trencher RPS.
 *
 * Prevents trencher stall/clogging by limiting rover advance speed to match cutting capacity.
 *
 * @tparam T Floating-point type.
 * @param trencher_rps Trencher motor RPS.
 * @param depth_m Current trench depth (meters).
 * @return Maximum allowable track motor RPS.
 */
template<typename T>
constexpr inline T trencherMotorRpsToMaxTrackMotorRps(
    const T& trencher_rps,
    const T& depth_m)
{
    return static_cast<T>(volumeRateToTrackMotorRps<double>(
        trencherMotorRpsToMaxVolumeRate<double>(
            static_cast<double>(trencher_rps)),
        static_cast<double>(depth_m)));
}

/**
 * @brief Compute synchronized trencher motor RPS matching the material excavation rate of the tracks.
 *
 * @tparam T Floating-point type.
 * @param track_rps Track drive motor RPS.
 * @param depth_m Current trench depth (meters).
 * @return Synchronized trencher motor RPS.
 */
template<typename T>
constexpr inline T trackMotorRpsToTrencherMotorRps(
    const T& track_rps,
    const T& depth_m)
{
    return static_cast<T>(targetVolRateToTrencherMotorRps(
        trackMotorRpsToVolumeRate<double>(
            static_cast<double>(track_rps),
            static_cast<double>(depth_m))));
}

/**
 * @brief Estimate remaining time until hopper reaches conservative volumetric capacity.
 *
 * Time (s) = CONSERVATIVE_HOPPER_CAPACITY_L / vol_rate_lps
 *
 * @tparam T Floating-point type.
 * @param vol_rate_lps Current filling rate in Liters per second.
 * @return Estimated time to full in seconds.
 */
template<typename T>
constexpr inline T volumeRateToHopperFullTime(const T& vol_rate_lps)
{
    return static_cast<T>(CONSERVATIVE_HOPPER_CAPACITY_L / vol_rate_lps);
}

/**
 * @brief Compute required forward sweep distance (meters) to excavate a target volume of regolith.
 *
 * Distance = (target_vol / (depth * efficiency)) / (width * 1000 L/m^3)
 *
 * @tparam T Floating-point type.
 * @param target_vol_l Desired volume of material to collect (Liters).
 * @param depth_m Cut depth in meters.
 * @param transfer_efficiency Ratio of cut material successfully captured into hopper (0.0 to 1.0).
 * @return Required forward driving distance in meters.
 */
template<typename T>
constexpr inline T targetVolumeToSweepDistance(
    const T& target_vol_l,
    const T& depth_m,
    const T& transfer_efficiency)
{
    return static_cast<T>(
        (static_cast<double>(target_vol_l) / static_cast<double>(depth_m) /
         static_cast<double>(transfer_efficiency)) /
        (TRENCHER_WIDTH_M * LITERS_PER_M_CUBED));
}


// =============================================================================
// Hopper Conveyor Belt Dynamics
// =============================================================================

/**
 * @brief Convert hopper conveyor motor shaft rotational speed (RPS) to linear belt speed (m/s).
 *
 * Formula: v_belt = rps * (1 / G_hopper) * (r_roller * 2 * pi)
 *
 * @tparam T Floating-point type.
 * @param motor_rps Hopper belt motor shaft speed in RPS.
 * @return Linear conveyor belt surface speed in meters per second.
 */
template<typename T>
constexpr inline T hopperBeltMotorRpsToBeltMps(const T& motor_rps)
{
    return static_cast<T>(
        motor_rps * ((1 / HOPPER_BELT_GEARING) *
                     (HOPPER_BELT_EFFECTIVE_OUTPUT_RADIUS_M * TWO_PI)));
}

/**
 * @brief Convert desired linear conveyor belt speed (m/s) to hopper motor shaft speed (RPS).
 *
 * Formula: rps = v_belt * (1 / (r_roller * 2 * pi)) * G_hopper
 *
 * @tparam T Floating-point type.
 * @param belt_mps Linear conveyor belt speed in meters per second.
 * @return Required hopper motor shaft speed in RPS.
 */
template<typename T>
constexpr inline T hopperBeltMpsToMotorRps(const T& belt_mps)
{
    return static_cast<T>(
        belt_mps * ((1 / (HOPPER_BELT_EFFECTIVE_OUTPUT_RADIUS_M * TWO_PI)) *
                    HOPPER_BELT_GEARING));
}

};  // namespace lance

#undef TWO_PI
#undef RADIANS_PER_DEGREE
#undef LITERS_PER_M_CUBED
#undef CONSTEXPR_VAL_TEMPLATE
