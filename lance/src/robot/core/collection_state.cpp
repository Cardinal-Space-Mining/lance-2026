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

/**
 * @file collection_state.cpp
 * @brief Implementation of regolith volume accumulation and hopper conveyor belt model.
 */

#include "collection_state.hpp"

#include "robot/model/dynamics.hpp"

#include <iostream>


namespace lance
{

HopperState::HopperState(
    double initial_volume_l,
    double capacity_volume_l,
    double initial_footprint_m,
    double capacity_len_m,
    double offload_len_m,
    double transfer_efficiency) :
    initial_vol_l(initial_volume_l),
    cap_vol_l(capacity_volume_l),
    initial_footprint_m(initial_footprint_m),
    cap_len_m(capacity_len_m),
    offload_len_m(offload_len_m),
    transfer_efficiency(transfer_efficiency),
    total_vol_l(0),
    belt_pos_m(0),
    high_pos_m(0),
    low_pos_m(0)
{
}

double HopperState::occupied_delta_m() const
{
    return this->high_pos_m - this->low_pos_m;
}

double HopperState::cutoff_pos_m() const
{
    return this->belt_pos_m - this->offload_len_m;
}

double HopperState::volume() const { return this->total_vol_l; }

double HopperState::remainingVolume() const
{
    return this->cap_vol_l - this->total_vol_l;
}

double HopperState::beltPosMeters() const { return this->belt_pos_m; }

double HopperState::startPosMeters() const { return this->high_pos_m; }

double HopperState::endPosMeters() const { return this->low_pos_m; }

double HopperState::beltUsageMeters() const { return this->occupied_delta_m(); }

double HopperState::beltUsagePercent() const
{
    return this->occupied_delta_m() / this->cap_len_m;
}

bool HopperState::isVolCapacity() const
{
    return this->total_vol_l >= this->cap_vol_l;
}

bool HopperState::isBeltCapacity() const
{
    return this->occupied_delta_m() >= this->cap_len_m;
}


void HopperState::update(double delta_volume_l, double belt_rotations)
{
    // Convert current motor rotations to linear belt displacement
    this->belt_pos_m = lance::hopperBeltMotorRpsToBeltMps(belt_rotations);

    // 1. Process incoming regolith from trencher
    if (delta_volume_l > 0.)
    {
        // If starting with an empty hopper, seed the initial longitudinal footprint
        if (this->total_vol_l <= 0.)
        {
            this->low_pos_m = (this->belt_pos_m - this->initial_footprint_m);
            this->high_pos_m = this->belt_pos_m;
        }
        // Expand the head of the pile as belt indexes forward
        else if (this->belt_pos_m > this->high_pos_m)
        {
            this->high_pos_m = this->belt_pos_m;
            // Constrain pile length so it does not exceed maximum physical conveyor bed length
            if (occupied_delta_m() > this->cap_len_m)
            {
                this->low_pos_m = this->high_pos_m - this->cap_len_m;
            }
        }
        // Scale added material by transfer efficiency (accounting for dust/spillage losses)
        this->total_vol_l += delta_volume_l * transfer_efficiency;
    }

    // 2. Process offload discharge and belt movement
    if (this->total_vol_l > 0.)
    {
        // Mitigate edge-case if belt moved backwards: shift tracked markers
        if (this->high_pos_m > this->belt_pos_m)
        {
            double occ_delta_m = this->occupied_delta_m();
            this->high_pos_m = this->belt_pos_m;
            this->low_pos_m = this->belt_pos_m - occ_delta_m;
        }

        // Determine discharge cutoff threshold along belt coordinate
        double cutoff_pos_m = this->cutoff_pos_m();

        // If entire pile has passed beyond the discharge cutoff point, hopper is empty
        if (this->high_pos_m < cutoff_pos_m)
        {
            this->total_vol_l = 0.;
            this->high_pos_m = this->low_pos_m = this->belt_pos_m;
        }
        // Partial offload: tail of the pile has crossed cutoff point
        else if (this->low_pos_m < cutoff_pos_m)
        {
            double cutoff_delta_m = (cutoff_pos_m - this->low_pos_m);
            double remainder_proportion =
                1. - (cutoff_delta_m / this->occupied_delta_m());

            this->total_vol_l *= remainder_proportion;
            this->low_pos_m = cutoff_pos_m;
        }
    }
    else
    {
        // Empty hopper: reset pile markers to current belt position
        this->high_pos_m = this->low_pos_m = this->belt_pos_m;
    }
}

double HopperState::miningTargetMotorPosition() const
{
    // Don't index belt until initial deposit volume and footprint thresholds are satisfied
    if (this->total_vol_l < this->initial_vol_l &&
        this->occupied_delta_m() <= this->initial_footprint_m)
    {
        return lance::hopperBeltMpsToMotorRps(this->belt_pos_m);
    }
    else
    {
        // Index belt proportionally to fill ratio along the conveyor bed length
        return lance::hopperBeltMpsToMotorRps(
            std::max(
                (this->low_pos_m +
                 (std::min(this->total_vol_l / this->cap_vol_l, 1.) *
                  this->cap_len_m)),
                this->belt_pos_m));
    }
}

double HopperState::offloadTargetMotorPosition() const
{
    if (this->occupied_delta_m() > 0.)
    {
        // Drive belt forward until the entire pile (up through high_pos_m) clears offload_len_m
        return lance::hopperBeltMpsToMotorRps(
            std::max(this->high_pos_m + this->offload_len_m, this->belt_pos_m));
    }
    else
    {
        return lance::hopperBeltMpsToMotorRps(this->belt_pos_m);
    }
}

double HopperState::calcOffloadTargetMotorPosition(double beg_motor_pos) const
{
    return lance::hopperBeltMpsToMotorRps(
        lance::hopperBeltMotorRpsToBeltMps(beg_motor_pos) +
        this->offload_len_m);
}



CollectionState::CollectionState(
    double initial_volume_l,
    double capacity_volume_l,
    double initial_footprint_m,
    double capacity_len_m,
    double offload_len_m,
    double transfer_efficiency) :
    hopper_state(
        initial_volume_l,
        capacity_volume_l,
        initial_footprint_m,
        capacity_len_m,
        offload_len_m,
        transfer_efficiency)
{
}

void CollectionState::update(const RobotMotorStatus& motors_status)
{
    const double trencher_rotations = motors_status.trencher.position;
    const double belt_rotations = motors_status.hopper_belt.position;
    const double ltrack_rotations = motors_status.track_left.position;
    const double rtrack_rotations = motors_status.track_right.position;

    // Determine current trencher cutting depth from linear actuator stroke
    double curr_mining_depth_m = lance::linearActuatorToMiningDepthClamped(
        motors_status.getHopperActNormalizedValue());
    double curr_impact_volume =
        lance::miningDepthToTrencherImpactVolume(curr_mining_depth_m);

    // First cycle initialization check: store baselines and return
    if (!prev_trencher_rotations.has_value() ||
        !prev_ltrack_rotations.has_value() ||
        !prev_rtrack_rotations.has_value() || !prev_mining_depth.has_value() ||
        !prev_impact_volume.has_value())
    {
        prev_trencher_rotations = trencher_rotations;
        prev_ltrack_rotations = ltrack_rotations;
        prev_rtrack_rotations = rtrack_rotations;
        prev_mining_depth = curr_mining_depth_m;
        prev_impact_volume = curr_impact_volume;
        this->hopper_state.update(0, belt_rotations);
        return;
    }

    // 1. Trencher capacity ceiling: max volume trencher bucket chain could convey
    double delta_trencher_rotations =
        trencher_rotations - this->prev_trencher_rotations.value();
    double trencher_max_delta_volume =
        lance::trencherMotorRpsToMaxVolumeRate(delta_trencher_rotations);

    // 2. Track sweep volume: material carved as rover drives forward
    double avg_mining_depth_m =
        (curr_mining_depth_m + this->prev_mining_depth.value()) * 0.5;
    double avg_track_delta_rotations =
        ((ltrack_rotations - this->prev_ltrack_rotations.value()) +
         (rtrack_rotations - this->prev_rtrack_rotations.value())) *
        0.5;
    double delta_sweep_volume = lance::trackMotorRpsToVolumeRate(
        avg_track_delta_rotations,
        avg_mining_depth_m);

    // 3. Impact volume: material carved strictly by plunging trencher deeper into regolith
    double delta_impact_volume =
        std::max(curr_impact_volume - this->prev_impact_volume.value(), 0.);

    // Total transmitted volume is bounded by physical cutting and trencher conveying capacity
    double transmitted_volume = std::min(
        (delta_impact_volume + delta_sweep_volume),
        trencher_max_delta_volume);

    // Update hopper berm model
    this->hopper_state.update(transmitted_volume, belt_rotations);

    // Advance history registers
    this->prev_trencher_rotations = trencher_rotations;
    this->prev_ltrack_rotations = ltrack_rotations;
    this->prev_rtrack_rotations = rtrack_rotations;
    this->prev_mining_depth = curr_mining_depth_m;
    this->prev_impact_volume = curr_impact_volume;
}

const HopperState& CollectionState::getHopperState() const
{
    return this->hopper_state;
}

};  // namespace lance
