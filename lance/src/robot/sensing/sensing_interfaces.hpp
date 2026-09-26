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
 * @file sensing_interfaces.hpp
 * @brief Aggregator bundle of perception and coordinate transformation interfaces.
 *
 * Combines:
 *   - TfCache: TF2 transform buffer and cached transforms for map, odom, and base_link.
 *   - PathPlanInterface: Client interface requesting waypoint trajectories from perception.
 *   - MiningEvalInterface: Service and topic client analyzing mining cut depth from LiDAR elevation maps.
 *   - ReflectorHintInterface: LiDAR retroreflector detector subscriber and enable service client.
 */

#include "tf_cache.hpp"
#include "path_plan.hpp"
#include "mining_eval.hpp"
#include "reflector_hint.hpp"


namespace lance
{

/**
 * @class SensingInterfaces
 * @brief Aggregates sensing subsystems passed down to controllers and planning routines.
 */
class SensingInterfaces : public util::UsingRosAliases
{
public:
    TfCache tf_cache;                                 ///< TF2 spatial transform listener and query helper.
    PathPlanInterface path_plan_interface;             ///< Autonomous path planning request/response client.
    MiningEvalInterface mining_eval_interface;         ///< Mining area traversability and depth evaluator.
    ReflectorHintInterface reflector_hint_interface;   ///< LiDAR retroreflector beacon detector interface.

public:
    SensingInterfaces(RclNode& node, const RobotParams& params) :
        tf_cache{node, params},
        path_plan_interface{node},
        mining_eval_interface{node, params},
        reflector_hint_interface{node}
    {
    }
};

};  // namespace lance
