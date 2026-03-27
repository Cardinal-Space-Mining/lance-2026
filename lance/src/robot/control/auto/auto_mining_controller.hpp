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

#include <cardinal_perception/msg/mining_eval_results.hpp>
#include <cardinal_perception/srv/update_mining_eval_mode.hpp>

#include "util/ros_utils.hpp"
#include "robot/core/robot_params.hpp"
#include "robot/core/mining_planner.hpp"
#include "robot/core/motor_interface.hpp"
#include "robot/core/collection_state.hpp"

#include "robot/control/shared/shared_controllers.hpp"


namespace lance
{

class AutoMiningController
{
    friend class TelemetrySerializer;
    friend class TelemetryDeserializer;

    using GenericPubMap = util::GenericPubMap;
    using UpdateMiningEvalSrv = cardinal_perception::srv::UpdateMiningEvalMode;
    using MiningEvalResultsMsg = cardinal_perception::msg::MiningEvalResults;

public:
    AutoMiningController(
        GenericPubMap&,
        const RobotParams&,
        SharedControllerCollection&);
    ~AutoMiningController() = default;

public:
    void initialize();
    bool isFinished();
    void setCancelled();

    void iterate(
        const RobotMotorStatus& motor_status,
        RobotMotorCommands& commands);

protected:
    enum class Stage
    {
        INITIALIZATION,
        PLANNING,
        TRAVERSING,
        MINING,
        FINISHED
    };

protected:
    float evalMiningDistance(float x, float y, float heading_deg) const;

protected:
    GenericPubMap& pub_map;
    const RobotParams& params;
    MiningPlanner mining_planner;

    ros_aliases::SharedPub<MiningEvalResultsMsg> mining_eval_sub;
    ros_aliases::SharedClient<UpdateMiningEvalSrv> mining_eval_client;

    Stage stage{Stage::FINISHED};

    MiningEvalResults::ConstSharedPtr eval_results{ nullptr };

    TraversalController& traversal_controller;
    MiningController& mining_controller;
};

};  // namespace lance
