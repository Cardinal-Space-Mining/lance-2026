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
#include "robot/model/geometry.hpp"

#include <Eigen/Core>


namespace lance
{

AutoOffloadController::AutoOffloadController(
    const RobotParams& params,
    SharedControllerCollection& controllers) :
    params{params},
    traversal_controller{controllers.traversal_controller},
    offload_controller{controllers.offload_controller}
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

void AutoOffloadController::setCancelled()
{
    switch (this->stage)
    {
        case Stage::TRAVERSING:
        {
            this->traversal_controller.setCancelled();
            break;
        }
        case Stage::OFFLOADING:
        {
            this->traversal_controller.setCancelled();
            break;
        }
        default:
        {
        }
    }
    this->stage = Stage::FINISHED;
}

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
            // placeholder for testing
            // Eigen::Vector2f target_dir{0.f, 1.f};
            Eigen::Vector2f target_dir = lance::geom::innerZoneNormalDir(
                this->params.bounds.arena_zone,
                this->params.bounds.offload_zone);
            Eigen::Vector2f target_pos =
                ((this->params.bounds.offload_zone.max() +
                  this->params.bounds.offload_zone.min()) *
                 0.5f) +
                (target_dir * std::abs(lance::geom::OFFLOAD_FOOTPRINT_OFFSET));

            // init with planned destination
            this->traversal_controller.initializePoint(target_pos, target_dir);
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

};  // namespace lance
