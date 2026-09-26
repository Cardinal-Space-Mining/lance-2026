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
 * @file collection_state.hpp
 * @brief Volumetric estimation and conveyor belt regolith distribution tracking.
 *
 * Provides real-time state estimation for regolith excavation and storage:
 *   - HopperState: Models the physical pile/berm of collected material on the hopper conveyor bed,
 *     tracking linear footprint [low_pos_m, high_pos_m], volume in liters, and indexing setpoints.
 *   - CollectionState: Integrates encoder feedback from track travel, plunge depth change,
 *     and trencher bucket rotations to infer instantaneous material excavated and transfer rates.
 */

#include <limits>
#include <optional>

#include "motor_interface.hpp"


namespace lance
{

/**
 * @class HopperState
 * @brief Geometric model of the regolith pile residing on the hopper conveyor belt.
 *
 * Tracks the spatial extent of regolith as a continuous 1D berm along the conveyor belt.
 * As new material drops in from the trencher, it expands the berm near `high_pos_m`.
 * When the belt is indexed forward, material shifts towards the discharge edge (`low_pos_m`).
 * Once material crosses the discharge roller (`cutoff_pos_m`), it falls off the belt into
 * the collection bin, proportionally reducing `total_vol_l`.
 */
class HopperState
{
public:
    /**
     * @brief Construct HopperState with geometric and volumetric parameters.
     *
     * @param initial_volume_l Minimum volume threshold before first belt indexing move (Liters).
     * @param capacity_volume_l Maximum total rated volume capacity of the hopper (Liters).
     * @param initial_footprint_m Initial longitudinal spread of material when entering empty belt (m).
     * @param capacity_len_m Total usable length of the conveyor bed (m).
     * @param offload_len_m Total belt advance distance required to fully purge hopper (m).
     * @param transfer_efficiency Ratio of cut material successfully entering hopper [0.0 - 1.0].
     */
    HopperState(
        double initial_volume_l,
        double capacity_volume_l,
        double initial_footprint_m,
        double capacity_len_m,
        double offload_len_m,
        double transfer_efficiency);

    /**
     * @brief Update the hopper berm model with newly excavated regolith and updated belt position.
     *
     * @param delta_volume_l Increment of newly excavated volume since last tick (Liters).
     * @param belt_rotations Accumulated hopper belt motor rotations.
     */
    void update(double delta_volume_l, double belt_rotations);

public:
    /// @brief Estimated total volume of regolith currently retained in the hopper (Liters).
    double volume() const;

    /// @brief Remaining volumetric capacity before reaching capacity_volume_l (Liters).
    double remainingVolume() const;

    /// @brief Absolute cumulative belt displacement in meters.
    double beltPosMeters() const;

    /// @brief Position of the head of the regolith pile (closest to trencher intake) in meters.
    double startPosMeters() const;

    /// @brief Position of the tail of the regolith pile (closest to offload roller) in meters.
    double endPosMeters() const;

    /// @brief Total longitudinal length of conveyor belt currently occupied by regolith (meters).
    double beltUsageMeters() const;

    /// @brief Relative fraction of conveyor length occupied [0.0 - 1.0].
    double beltUsagePercent() const;

public:
    /// @brief True if total stored volume meets or exceeds capacity_vol_l.
    bool isVolCapacity() const;

    /// @brief True if regolith pile has reached the full longitudinal length of the belt.
    bool isBeltCapacity() const;

public:
    /// @brief Target motor position (rotations) to index belt forward during mining to distribute regolith.
    double miningTargetMotorPosition() const;

    /// @brief Target motor position (rotations) to completely discharge current pile off the belt.
    double offloadTargetMotorPosition() const;

public:
    /**
     * @brief Calculate offload target motor position starting from a specified motor baseline position.
     * @param beg_motor_pos Starting motor position in rotations.
     * @return Target motor position after adding offload_len_m worth of rotation.
     */
    double calcOffloadTargetMotorPosition(double beg_motor_pos) const;

private:
    /// @brief Distance between high_pos_m and low_pos_m (meters).
    double occupied_delta_m() const;

    /// @brief Position along belt travel at which material drops off the discharge roller (meters).
    double cutoff_pos_m() const;

private:  // Model parameters
    const double initial_vol_l;         ///< Minimum volume before initial indexing step.
    const double cap_vol_l;             ///< Volumetric capacity in Liters.
    const double initial_footprint_m;   ///< Initial pile spread length in meters.
    const double cap_len_m;             ///< Conveyor bed usable length in meters.
    const double offload_len_m;         ///< Travel distance to purge payload in meters.
    const double transfer_efficiency;   ///< Transfer factor accounting for spillage.

private:  // Tracked State
    double total_vol_l{0.};  ///< Integrated volume currently on belt (Liters).
    double belt_pos_m{0.};   ///< Current linear belt travel (meters).
    double high_pos_m{0.};   ///< Front edge position of regolith pile (meters).
    double low_pos_m{0.};    ///< Rear edge position of regolith pile (meters).
};

/**
 * @class CollectionState
 * @brief High-level observer calculating volume intake from robot actuators and feeding HopperState.
 *
 * Integrates:
 *   1. Plunge cut impact volume: Volumetric change from lower trencher depth.
 *   2. Forward sweep volume: Volumetric change from track driving forward into regolith.
 *   3. Trencher throughput limit: Bucket speed cap preventing unrealistic intake rates.
 */
class CollectionState
{
public:
    CollectionState(
        double initial_volume_l,
        double capacity_volume_l,
        double initial_footprint_m,
        double capacity_len_m,
        double offload_len_m,
        double transfer_efficiency);

    /**
     * @brief Process latest motor telemetry and advance the volumetric model.
     * @param motors_status Current motor positions, velocities, and linear actuator state.
     */
    void update(const RobotMotorStatus& motors_status);

public:
    /// @brief Read-only accessor for the underlying HopperState.
    const HopperState& getHopperState() const;

private:
    HopperState hopper_state; ///< Embedded conveyor pile state model.

    // Previous tick values for numerical differentiation / integration
    std::optional<double> prev_trencher_rotations;
    std::optional<double> prev_ltrack_rotations;
    std::optional<double> prev_rtrack_rotations;
    std::optional<double> prev_mining_depth;
    std::optional<double> prev_impact_volume;
};

};  // namespace lance
