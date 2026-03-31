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

#include <cmath>
#include <limits>
#include <numbers>
#include <algorithm>


namespace lance
{
#define TWO_PI             (std::numbers::pi * 2)
#define RADIANS_PER_DEGREE (std::numbers::pi / 180)
#define LITERS_PER_M_CUBED (1000)

#define CONSTEXPR_VAL_TEMPLATE(name, val)             \
    template<typename T>                              \
    inline constexpr T name##_ = static_cast<T>(val); \
    inline constexpr double name = name##_<double>;

#ifndef LANCE
    #define LANCE 2
#endif

#if LANCE <= 1
CONSTEXPR_VAL_TEMPLATE(TRACK_GEARING, 64)
CONSTEXPR_VAL_TEMPLATE(TRACK_EFFECTIVE_OUTPUT_RADIUS_M, 0.07032851)
CONSTEXPR_VAL_TEMPLATE(TRACK_SEPARATION_M, 0.636)

CONSTEXPR_VAL_TEMPLATE(TRENCHER_WIDTH_M, 0.254)
CONSTEXPR_VAL_TEMPLATE(TRENCHER_GEARING, 32)
// bucket separation (CAD): 0.05107 m, actuation radius (CAD): 0.04890 m, strict bucket volume (CAD): 0.04309 L
CONSTEXPR_VAL_TEMPLATE(
    TRENCHER_LITERS_PER_OUTPUT_ROTATION,
    (0.04309 * ((0.04826 * TWO_PI) / 0.05107)))
CONSTEXPR_VAL_TEMPLATE(TRENCHER_IMPACT_EFFECTIVE_RADIUS_M, 0.09270911)

CONSTEXPR_VAL_TEMPLATE(ACTUATOR_LOWEST_ANGLE_DEG, 15)
CONSTEXPR_VAL_TEMPLATE(ACTUATOR_HIGHEST_ANGLE_DEG, -15)

CONSTEXPR_VAL_TEMPLATE(HOPPER_BELT_GEARING, 100)
CONSTEXPR_VAL_TEMPLATE(HOPPER_BELT_EFFECTIVE_OUTPUT_RADIUS_M, 0.0508)
CONSTEXPR_VAL_TEMPLATE(HOPPER_BELT_CONTAINER_LENGTH_M, 0.6)
CONSTEXPR_VAL_TEMPLATE(HOPPER_BELT_SAFE_OFFLOAD_DIST_M, 0.7)
CONSTEXPR_VAL_TEMPLATE(CONSERVATIVE_HOPPER_CAPACITY_L, 30)

// (mining depth) ~ -0.3159 * (normalized actuator pos) + 0.1129
CONSTEXPR_VAL_TEMPLATE(MINING_DEPTH_FX_OFFSET, 0.1129)
CONSTEXPR_VAL_TEMPLATE(MINING_DEPTH_FX_SLOPE, -0.3159)
CONSTEXPR_VAL_TEMPLATE(MINING_MAX_DEPTH_M, 0.1016)
#elif LANCE >= 2
CONSTEXPR_VAL_TEMPLATE(TRACK_GEARING, 64)
CONSTEXPR_VAL_TEMPLATE(TRACK_EFFECTIVE_OUTPUT_RADIUS_M, 0.045)
CONSTEXPR_VAL_TEMPLATE(TRACK_SEPARATION_M, 0.648)

CONSTEXPR_VAL_TEMPLATE(TRENCHER_WIDTH_M, 0.260)
CONSTEXPR_VAL_TEMPLATE(TRENCHER_GEARING, 32)
// bucket separation (CAD): 0.03228 m, actuation radius (CAD): 0.04826 m, strict bucket volume (CAD): 0.04309 L
CONSTEXPR_VAL_TEMPLATE(
    TRENCHER_LITERS_PER_OUTPUT_ROTATION,
    (0.04309 * ((0.04826 * TWO_PI) / 0.03228)))
CONSTEXPR_VAL_TEMPLATE(TRENCHER_IMPACT_EFFECTIVE_RADIUS_M, 0.092)

CONSTEXPR_VAL_TEMPLATE(ACTUATOR_LOWEST_ANGLE_DEG, 10)
CONSTEXPR_VAL_TEMPLATE(ACTUATOR_HIGHEST_ANGLE_DEG, -10)

CONSTEXPR_VAL_TEMPLATE(HOPPER_BELT_GEARING, 100)
CONSTEXPR_VAL_TEMPLATE(HOPPER_BELT_EFFECTIVE_OUTPUT_RADIUS_M, 0.028)
CONSTEXPR_VAL_TEMPLATE(HOPPER_BELT_CONTAINER_LENGTH_M, 0.7)
CONSTEXPR_VAL_TEMPLATE(HOPPER_BELT_SAFE_OFFLOAD_DIST_M, 0.8)
CONSTEXPR_VAL_TEMPLATE(CONSERVATIVE_HOPPER_CAPACITY_L, 50)

// (mining depth) ~ -0.3376 * (normalized actuator pos) + 0.1539
CONSTEXPR_VAL_TEMPLATE(MINING_DEPTH_FX_OFFSET, 0.1539)
CONSTEXPR_VAL_TEMPLATE(MINING_DEPTH_FX_SLOPE, -0.3376)
CONSTEXPR_VAL_TEMPLATE(MINING_MAX_DEPTH_M, 0.1524)
#endif


template<typename T>
constexpr inline T trackMotorRpsToGroundMps(const T& rps)
{
    return static_cast<T>(
        rps * ((1 / TRACK_GEARING) * TRACK_EFFECTIVE_OUTPUT_RADIUS_M * TWO_PI));
}
template<typename T>
constexpr inline T groundMpsToTrackMotorRps(const T& mps)
{
    return static_cast<T>(
        mps * (1 / (TRACK_EFFECTIVE_OUTPUT_RADIUS_M * TWO_PI) * TRACK_GEARING));
}
template<typename T>
constexpr inline T trackVelocitiesToForwardVelocity(
    const T& left_mps,
    const T& right_mps)
{
    return static_cast<T>((left_mps + right_mps) / 2.);
}
template<typename T>
constexpr inline T trackVelocitiesToAngularVelocity(
    const T& left_mps,
    const T& right_mps)
{
    return static_cast<T>((left_mps - right_mps) / TRACK_SEPARATION_M);
}
template<typename T>
constexpr inline T bodyDynamicsToLeftTrackVelocityMps(
    const T& v_mps,
    const T& omega_radps)
{
    return static_cast<T>(v_mps - omega_radps * (TRACK_SEPARATION_M / 2));
}
template<typename T>
constexpr inline T bodyDynamicsToRightTrackVelocityMps(
    const T& v_mps,
    const T& omega_radps)
{
    return static_cast<T>(v_mps + omega_radps * (TRACK_SEPARATION_M / 2));
}

template<typename T>
constexpr inline T linearActuatorToMiningDepthUnclamped(
    const T& actuator_normalized_pos)
{
    return static_cast<T>(
        MINING_DEPTH_FX_OFFSET +
        MINING_DEPTH_FX_SLOPE * actuator_normalized_pos);
}
template<typename T>
constexpr inline T miningDepthToLinearActuatorUnclamped(const T& depth_m)
{
    return static_cast<T>(
        (depth_m - MINING_DEPTH_FX_OFFSET) * (1 / MINING_DEPTH_FX_SLOPE));
}
// Domain/range: [0.03, ~0.35] -> [0m, 0.1016m (4in)] (lance-1)
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
// Domain/range: [0m, 0.1016m (4in)] -> [0.03, ~0.35] (lance-1)
template<typename T>
constexpr inline T miningDepthToLinearActuatorClamped(const T& depth_m)
{
    return static_cast<T>(miningDepthToLinearActuatorUnclamped(
        std::clamp<double>(
            static_cast<double>(depth_m),
            0.,
            MINING_MAX_DEPTH_M)));
}

// Domain/range: [0, 1] -> [-0.262, 0.262] (+/-15 deg - lance-1)
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
    else if (d < R)
    {
        double x = (R - d);
        double cross_section_area =
            (R2 * acos(x / R) - x * std::sqrt(R2 - x * x));
        // cross-section * width * 1000 liters/m^3
        return static_cast<T>(
            cross_section_area * TRENCHER_WIDTH_M * LITERS_PER_M_CUBED);
    }
    else
    {
        // (full semi-circle cross-section + additional depth rect) * width * 1000 liters/m^3
        return static_cast<T>(
            ((std::numbers::pi * R2) + ((depth_m - R) * TRENCHER_WIDTH_M * R)) *
            LITERS_PER_M_CUBED);
    }
}

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

template<typename T>
constexpr inline T trencherMotorRpsToMaxVolumeRate(const T& rps)
{
    return static_cast<T>(
        rps * ((1 / TRENCHER_GEARING) * TRENCHER_LITERS_PER_OUTPUT_ROTATION));
}
template<typename T>
constexpr inline T targetVolRateToTrencherMotorRps(const T& vol_rate_lps)
{
    return static_cast<T>(
        vol_rate_lps *
        ((1 / TRENCHER_LITERS_PER_OUTPUT_ROTATION) * TRENCHER_GEARING));
}

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

template<typename T>
constexpr inline T volumeRateToHopperFullTime(const T& vol_rate_lps)
{
    return static_cast<T>(CONSERVATIVE_HOPPER_CAPACITY_L / vol_rate_lps);
}

template<typename T>
constexpr inline T hopperBeltMotorRpsToBeltMps(const T& motor_rps)
{
    return static_cast<T>(
        motor_rps * ((1 / HOPPER_BELT_GEARING) *
                     (HOPPER_BELT_EFFECTIVE_OUTPUT_RADIUS_M * TWO_PI)));
}
template<typename T>
constexpr inline T hopperBeltMpsToMotorRps(const T& belt_mps)
{
    return static_cast<T>(
        belt_mps * ((1 / (HOPPER_BELT_EFFECTIVE_OUTPUT_RADIUS_M * TWO_PI)) *
                    HOPPER_BELT_GEARING));
}



namespace geom
{

#if LANCE <= 1
CONSTEXPR_VAL_TEMPLATE(FOOTPRINT_X_MAX, 0.735)
CONSTEXPR_VAL_TEMPLATE(FOOTPRINT_X_MIN, -0.765)
CONSTEXPR_VAL_TEMPLATE(FOOTPRINT_Y_MAX, 0.369)
CONSTEXPR_VAL_TEMPLATE(FOOTPRINT_Y_MIN, -0.369)
CONSTEXPR_VAL_TEMPLATE(COLLISION_Z_MAX, 0.675)
CONSTEXPR_VAL_TEMPLATE(COLLISION_Z_MIN, -0.102)
#elif LANCE >= 2
CONSTEXPR_VAL_TEMPLATE(FOOTPRINT_X_MAX, 0.591)
CONSTEXPR_VAL_TEMPLATE(FOOTPRINT_X_MIN, -0.640)
CONSTEXPR_VAL_TEMPLATE(FOOTPRINT_Y_MAX, 0.362)
CONSTEXPR_VAL_TEMPLATE(FOOTPRINT_Y_MIN, -0.362)
CONSTEXPR_VAL_TEMPLATE(COLLISION_Z_MAX, 0.810)
CONSTEXPR_VAL_TEMPLATE(COLLISION_Z_MIN, -0.102)
#endif

CONSTEXPR_VAL_TEMPLATE(PRIMARY_COLLISION_ZONE_X, FOOTPRINT_X_MIN)
CONSTEXPR_VAL_TEMPLATE(
    PRIMARY_COLLISION_ZONE_Y,
    ((FOOTPRINT_Y_MAX + FOOTPRINT_Y_MIN) / 2))
CONSTEXPR_VAL_TEMPLATE(
    PRIMARY_COLLISION_ZONE_Z,
    ((COLLISION_Z_MAX + COLLISION_Z_MIN) / 2))

CONSTEXPR_VAL_TEMPLATE(
    PRIMARY_COLLISION_ZONE_LENGTH_OFFSET,
    (FOOTPRINT_X_MAX - FOOTPRINT_X_MIN))
CONSTEXPR_VAL_TEMPLATE(
    PRIMARY_COLLISION_ZONE_WIDTH,
    (FOOTPRINT_Y_MAX - FOOTPRINT_Y_MIN))
CONSTEXPR_VAL_TEMPLATE(
    PRIMARY_COLLISION_ZONE_HEIGHT,
    (COLLISION_Z_MAX - COLLISION_Z_MIN))

};  // namespace geom

#undef TWO_PI
#undef RADIANS_PER_DEGREE
#undef LITERS_PER_M_CUBED
#undef CONSTEXPR_VAL_TEMPLATE

};  // namespace lance


/* Kinematics Utilities */
namespace kmx
{

/* Solve the kinematics equation `Vf^2 = Vi^2 + 2ax` for `Vi` such that the target
 * `Vf` is attained within `x` while decelerating at `a` (gives the maximum `Vi`). */
template<typename F>
inline F maxStartVel(F end_vel, F dist, F max_decell)
{
    return std::sqrt((end_vel * end_vel) + (2 * dist * std::abs(max_decell)));
}

/* Compute the distance it will take to decellerate to `Vf = 0` */
template<typename F>
inline F decellDist(F vel, F max_decell)
{
    // x = v^2 / 2a
    return (vel * vel) / (2 * std::abs(max_decell));
}

/* Limit track velocities while maintaining trajectory curvature. */
template<typename F>
inline void
    applyTrackLimits(F Vl_target, F Vr_target, F V_max, F& Vl_out, F& Vr_out)
{
    const F Vl_target_abs = std::abs(Vl_target);
    const F Vr_target_abs = std::abs(Vr_target);
    const F s = std::min(
        {static_cast<F>(1), (V_max / Vl_target_abs), (V_max / Vr_target_abs)});
    Vl_out = Vl_target * s;
    Vr_out = Vr_target * s;
}

/* WARNING: BROKEN.
 * Constrains the target left/right track velocities such that Vmax and Vdelta
 * are within the specified limits, while minimizing the error in trajectory
 * curvature. Previous velocities MUST be within the given limits, otherwise
 * this may create a positive feedback loop. */
template<typename F>
inline void applyTrackLimits(
    F Vl_target,
    F Vr_target,
    F Vl_prev,
    F Vr_prev,
    F V_max,
    F Vd_max,
    F& Vl_out,
    F& Vr_out)
{
    const F Vl_min = std::clamp(Vl_prev - Vd_max, -V_max, V_max);
    const F Vl_max = std::clamp(Vl_prev + Vd_max, -V_max, V_max);
    const F Vr_min = std::clamp(Vr_prev - Vd_max, -V_max, V_max);
    const F Vr_max = std::clamp(Vr_prev + Vd_max, -V_max, V_max);

    // 1. If target is already in attainable range, short-circuit.
    if ((Vl_target <= Vl_max && Vl_target >= Vl_min) &&
        (Vr_target <= Vr_max && Vr_target >= Vr_min))
    {
        Vl_out = Vl_target;
        Vr_out = Vr_target;
        return;
    }

    const F Vl_target_abs = std::abs(Vl_target);
    const F Vr_target_abs = std::abs(Vr_target);

    // 2. If targetting no movement, pick edge point that's closest
    if (Vl_target_abs <= 1e-7 && Vr_target_abs <= 1e-7)
    {
        Vl_out = std::clamp(Vl_target, Vl_min, Vl_max);
        Vr_out = std::clamp(Vr_target, Vr_min, Vr_max);
        return;
    }

    // 3. If constant-curvature line intersects range, pick intersection point
    //    that is closest
    {
        F t_min = -std::numeric_limits<F>::max();
        F t_max = std::numeric_limits<F>::max();
        if (Vl_target_abs > 1e-7)
        {
            const F a = (Vl_min / Vl_target);
            const F b = (Vl_max / Vl_target);
            t_min = std::max(t_min, std::min(a, b));
            t_max = std::min(t_max, std::max(a, b));
        }
        else
        {
            // L-component is zero, thus equality line is the R-axis. If range
            // doesn't intersect L=0, then there cannot be an intersection.
            if (Vl_min > 0 || Vl_max < 0)
            {
                goto SKIP_INTERSECT_L;
            }
        }
        if (Vr_target_abs > 1e-7)
        {
            const F a = (Vr_min / Vr_target);
            const F b = (Vr_max / Vr_target);
            t_min = std::max(t_min, std::min(a, b));
            t_max = std::min(t_max, std::max(a, b));
        }
        else
        {
            // R-componenet is zero, thus equality line is the L-axis. If range
            // doesn't intersect R=0, then there cannot be an intersection.
            if (Vr_min > 0 || Vr_max < 0)
            {
                goto SKIP_INTERSECT_L;
            }
        }

        // Pick intersection point that's closest to the target
        if (t_min <= t_max)
        {
            const F t =
                (std::abs(t_min - 1) < std::abs(t_max - 1)) ? t_min : t_max;
            Vl_out = Vl_target * t;
            Vr_out = Vr_target * t;
            return;
        }
    }

SKIP_INTERSECT_L:

    // 4. Use the corner with minimum curvature error
    //    Pick C or R form based on which is most stable across all corners.
    {
        // Clamp to nearest as a default if anything fails...
        Vl_out = std::clamp(Vl_target, Vl_min, Vl_max);
        Vr_out = std::clamp(Vr_target, Vr_min, Vr_max);

        const F Vlr_sum = Vl_target + Vr_target;
        const F Vlr_diff = Vl_target - Vr_target;
        const bool R_ok = std::abs(Vlr_diff) > 1e-7;
        const bool C_ok = std::abs(Vlr_sum) > 1e-7;
        if (!R_ok && !C_ok)
        {
            // No valid target metric at all - use clamped values
            return;
        }

        // Find worst-case (minimum) denominator for each form across all corners
        F C_denorm_min = std::numeric_limits<F>::max();
        F R_denom_min = std::numeric_limits<F>::max();
        for (size_t i = 0; i < 4; i++)
        {
            const F l = (i & 0b01) ? Vl_min : Vl_max;
            const F r = (i & 0b10) ? Vr_min : Vr_max;
            C_denorm_min = std::min(C_denorm_min, std::abs(l + r));
            R_denom_min = std::min(R_denom_min, std::abs(l - r));
        }

        // Pick the single most stable form, respecting validity
        // Then compute target reference value with that form
        const bool use_R = R_ok && (!C_ok || R_denom_min >= C_denorm_min);
        const F target_ref = use_R ? (Vlr_sum / Vlr_diff)   // R_target
                                   : (Vlr_diff / Vlr_sum);  // C_target

        // Evaluate all 4 corners with the chosen form
        F err_min = std::numeric_limits<F>::max();
        for (size_t i = 0; i < 4; i++)
        {
            const F l = (i & 0b01) ? Vl_min : Vl_max;
            const F r = (i & 0b10) ? Vr_min : Vr_max;
            const F denom = use_R ? (l - r) : (l + r);
            const F numer = use_R ? (l + r) : (l - r);

            if (std::abs(denom) <= 1e-7)
            {
                // singular corner for chosen form, skip
                continue;
            }

            const F err = std::abs(numer / denom - target_ref);
            if (err < err_min)
            {
                err_min = err;
                Vl_out = l;
                Vr_out = r;
            }
        }
    }
}

};  // namespace kmx
