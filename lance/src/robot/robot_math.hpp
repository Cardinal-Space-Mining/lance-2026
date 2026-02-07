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
#include <numbers>


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
// bucket separation (CAD): 0.05107 m, actuation radius (CAD): 0.04890 m, struct bucket volume (CAD): 0.04309 L
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

#undef CONSTEXPR_VAL_TEMPLATE


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

#undef TWO_PI
#undef RADIANS_PER_DEGREE
#undef LITERS_PER_M_CUBED

};  // namespace lance
