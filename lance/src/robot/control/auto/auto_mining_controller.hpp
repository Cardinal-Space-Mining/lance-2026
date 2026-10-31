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

#include <chrono>
#include <optional>

#include <csm_utils/ros_utils.hpp>

#include "robot/core/robot_params.hpp"
#include "robot/core/motor_interface.hpp"
#include "robot/core/collection_state.hpp"
#include "robot/control/shared/shared_controllers.hpp"
#include "robot/sensing/sensing_interfaces.hpp"
#include "robot/planning/mining_planner.hpp"
#include "robot/planning/polar_mining_planner.hpp"


namespace lance
{

class AutoMiningController
{
    friend class TelemetrySerializer;
    friend class TelemetryDeserializer;

public:
    AutoMiningController(
        const RobotParams&,
        SensingInterfaces&,
        SharedControllerCollection&);
    ~AutoMiningController() = default;

public:
    void initialize(bool quick = false);
    bool isFinished();
    void setCancelled();

    void iterate(
        const RobotMotorStatus& motor_status,
        RobotMotorCommands& commands);

protected:

    enum class Stage
    {
        INITIALIZATION,
        POLAR_PLANNING,
        GRID_PLANNING,
        TRAVERSING,
        MINING,
        PIVOT_TURNING,
        PIVOT_PLANNING,  // re-sweeping at the pivot to choose leg 2
        FINISHED
    };

protected:
    // After a mining run ends: replan if worthwhile, otherwise finish
    void replanOrFinish();

protected:
    const RobotParams& params;
    SensingInterfaces& sensing_interfaces;

    TraversalController& traversal_controller;
    MiningController& mining_controller;

    // Only set when the grid planner chose the path
    std::optional<DirectedMiningPath> current_mining_path;
    PolarMiningPlanner polar_planner;
    MiningPlanner mining_planner;

    // Two-leg polar routes: mine leg1_dist along leg1_dir, then re-sweep at
    // the pivot to choose leg 2 (pending_leg2_dir is the planned leg 2; it
    // marks a two-leg route but the re-sweep's heading is what gets mined)
    float leg1_dist{std::numeric_limits<float>::infinity()};
    PolarMiningPlanner::Vec2f leg1_dir{PolarMiningPlanner::Vec2f::Zero()};
    std::optional<PolarMiningPlanner::Vec2f> pending_leg2_dir;
    std::chrono::steady_clock::time_point next_path_validity_recheck_time{};

    Stage stage{Stage::FINISHED};

    bool is_quick_run = false;
    int iteration_count = 0;

};

};  // namespace lance
