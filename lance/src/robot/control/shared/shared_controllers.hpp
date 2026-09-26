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
 * @file shared_controllers.hpp
 * @brief Aggregator container for modular sub-controllers used across both teleoperation and autonomy.
 *
 * Encapsulates the four core behavior controllers:
 *   - MiningController: Closed-loop plunge and linear cut excavation state machine.
 *   - OffloadController: High-tilt conveyor belt discharge sequence.
 *   - TraversalController: Path following via Stanley steering and curvature smoothing.
 *   - LocalizationController: 360-degree LiDAR beacon scan and standoff docking.
 */

#include "mining_controller.hpp"
#include "offload_controller.hpp"
#include "traversal_controller.hpp"
#include "localization_controller.hpp"


namespace lance
{

/**
 * @class SharedControllerCollection
 * @brief Bundle of reusable sub-controllers instantiated by RobotController.
 *
 * Allows both TeleopController (driver assistance / semi-auto triggers) and AutoController
 * (fully autonomous mission cycles) to invoke identical validated control logic.
 */
class SharedControllerCollection : public util::UsingRosAliases
{
public:
    MiningController mining_controller;             ///< Regolith excavation controller.
    OffloadController offload_controller;           ///< Hopper purging and deposit controller.
    TraversalController traversal_controller;       ///< Waypoint and trajectory tracking controller.
    LocalizationController localization_controller; ///< Retroreflector beacon alignment controller.

public:
    inline SharedControllerCollection(
        const RobotParams& params,
        const HopperState& hopper_state,
        const StallState& stall_state,
        SensingInterfaces& sensing_interfaces) :
        mining_controller{params, hopper_state, stall_state, sensing_interfaces},
        offload_controller{params, hopper_state},
        traversal_controller{params, sensing_interfaces},
        localization_controller{params, sensing_interfaces}
    {
    }
};

};  // namespace lance
