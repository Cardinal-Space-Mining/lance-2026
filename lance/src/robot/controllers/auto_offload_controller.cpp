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

#include "auto_offload_controller.hpp"

#include <Eigen/Core>


AutoOffloadController::AutoOffloadController(
    RclNode& node,
    GenericPubMap& pub_map,
    const RobotParams& params,
    const HopperState& hopper_state,
    TraversalController& trav_controller) :
    pub_map{pub_map},
    params{params},
    hopper_state{hopper_state},
    traversal_controller{trav_controller},
    offload_controller{node, pub_map, params, hopper_state}
{
}

void AutoOffloadController::initialize()
{
    this->stage = Stage::INITIALIZATION;
}

bool AutoOffloadController::isFinished()
{
    return this->stage == Stage::FINISHED;
}

void AutoOffloadController::setCancelled() { this->stage = Stage::FINISHED; }

void AutoOffloadController::iterate(
    const RobotMotorStatus& motor_status,
    RobotMotorCommands& commands)
{
    switch (this->stage)
    {
        case Stage::INITIALIZATION:
        {
            this->stage = Stage::PLANNING;
            [[fallthrough]];
        }
        case Stage::PLANNING:
        {
            //this is where I divide up the offload zone into points to cycle through

            /** in-progress code
             * 
             * float spotWidth, spotHeight; // area of a single dump 
             * // ^ASK HOPPER TEAM FOR VALUES, THEY WILL BE CONSTANTS
             * float areaWidth, areaHeight; // area of whole offload zone (from mining_zome_bounds)
             * int numXSpots = round(areaWidth / spotWidth)
             * int numYSpots = round(areaHeight / spotHeight)
             * int numsSpots = numXSpots* numYSpots;
             * float startPointX = spotWidth / 2;
             * float startPointY = spotHeight / 2;
             * float xSpotDifference = areaWidth / numXSpots;
             * float ySpotDifference = areaHeight / numYSpots;
             * // then, depending on how many x spots there are and how many
             * // sorties mechanical expects to be able to do, decide whether
             * // to do just one row, or to actually cycle through multiple rows
             * // to avoid crushing previous drop-offs
             * 
             */

            // init with planned destination
            this->traversal_controller.initializePoint(Eigen::Vector2f::Zero());
            this->stage = Stage::TRAVERSING;
            [[fallthrough]];
        }
        case Stage::TRAVERSING:
        {
            this->traversal_controller.iterate(motor_status, commands);
            if (!this->traversal_controller.isFinished())
            {
                break;
            }

            // initialize with backup if necessary
            this->offload_controller.initialize(0.f);
            this->stage = Stage::OFFLOADING;
            [[fallthrough]];
        }
        case Stage::OFFLOADING:
        {
            this->offload_controller.iterate(motor_status, commands);
            if (!this->offload_controller.isFinished())
            {
                break;
            }

            this->stage = Stage::FINISHED;
            [[fallthrough]];
        }
        case Stage::FINISHED:
        {
        }
    }
}
