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
    // set initial params
    void setParams(
        double initial_volume_l,
        double capacity_volume_l,
        double initial_footprint_m,
        double capacity_len_m,
        double offload_len_m,
        double transfer_efficiency);

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
    double beltUsagePercent() const;

    // have we reached the max configured volume
    bool isVolCapacity() const;
    // has the belt reached the end
    bool isBeltCapacity() const;

    // output is in motor rotations
    double miningTargetMotorPosition() const;
    // outut is in motor rotations
    double offloadTargetMotorPosition() const;

    double calcOffloadTargetMotorPosition(double beg_motor_pos) const;

private:
    double occupied_delta_m() const;
    double cutoff_pos_m() const;

private:
    double initial_vol_l = 12.;
    double cap_vol_l = 30.;
    double initial_footprint_m = 0.2;
    double cap_len_m = 0.6;
    double offload_len_m = 0.7;
    double transfer_efficiency = 0.5;

    double total_vol_l = 0.;
    double belt_pos_m = 0.;
    double high_pos_m = 0.;
    double low_pos_m = 0.;
};

class CollectionState
{
public:
    void setParams(
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
