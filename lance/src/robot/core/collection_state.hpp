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

#include <limits>
#include <optional>

#include "motor_interface.hpp"


namespace lance
{

class HopperState
{
public:
    /// @brief Sets the hopper parameters
    /// @param initial_volume_l How much regolith to collect before the hopper is first rotated
    /// @param capacity_volume_l Total capacity of the hopper in liters
    /// @param initial_footprint_m How far back the initial pile in the belt is
    /// @param capacity_len_m How long the hopper belt is
    /// @param offload_len_m How far to move the belt to offload
    /// @param transfer_efficiency Efficiency of regolith transfer between trencher and hopper
    HopperState(
        double initial_volume_l,
        double capacity_volume_l,
        double initial_footprint_m,
        double capacity_len_m,
        double offload_len_m,
        double transfer_efficiency);

    /// @brief Updates internal hopper model
    /// @param delta_volume_l How much additional regolith has been added
    /// @param belt_rotations Current position of belt in Motor Angle Units
    void update(double delta_volume_l, double belt_rotations);

public:
    // estimated volume in liters
    double volume() const;
    // volume left until full in liters
    double remainingVolume() const;
    // tracked belt position in meters
    double beltPosMeters() const;
    // belt position of "head" of regolith pile (closest to trencher)
    double startPosMeters() const;
    // belt position of "end" of regloith pile (closest to opening)
    double endPosMeters() const;
    // region of belt occupiled by regolith pile, in meters
    double beltUsageMeters() const;
    // the relative proportion of the belt which is used
    double beltUsagePercent() const;  //TODO: This is returning values > one

public:
    // have we reached the max configured volume
    bool isVolCapacity() const;
    // has the belt reached the end
    bool isBeltCapacity() const;

public:
    // output is in motor rotations
    double miningTargetMotorPosition() const;
    // outut is in motor rotations
    double offloadTargetMotorPosition() const;

public:
    double calcOffloadTargetMotorPosition(double beg_motor_pos) const;

private:
    double occupied_delta_m() const;
    double cutoff_pos_m() const;

private:  // parameters
    const double initial_vol_l;
    const double cap_vol_l;
    const double initial_footprint_m;
    const double cap_len_m;
    const double offload_len_m;
    const double transfer_efficiency;

private:  // State
    /// @brief Current volume of regolith stored
    double total_vol_l;

    /// @brief Current belt position
    double belt_pos_m;

    /// @brief Absolute start of pile
    double high_pos_m;

    /// @brief Absolute end of pile
    double low_pos_m;
};

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


    void update(const RobotMotorStatus& motors_status);

public:
    const HopperState& getHopperState() const;


private:
    HopperState hopper_state;

    std::optional<double> prev_trencher_rotations;
    std::optional<double> prev_ltrack_rotations;
    std::optional<double> prev_rtrack_rotations;
    std::optional<double> prev_mining_depth;
    std::optional<double> prev_impact_volume;
};

};  // namespace lance
