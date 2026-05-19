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

#include "auto_controller.hpp"

#include <memory>

#include "robot/core/robot_status.hpp"
#include "robot/model/geometry.hpp"


namespace lance
{

AutoController::AutoController(
    const RobotParams& params,
    SensingInterfaces& sensing_interfaces,
    SharedControllerCollection& controllers) :
    params{params},
    sensing_interfaces{sensing_interfaces},
    localization_controller{controllers.localization_controller},
    traversal_controller{controllers.traversal_controller},
    mining_controller{params, sensing_interfaces, controllers},
    offload_controller{params, controllers}
{
}

void AutoController::initialize()
{
    // if we previously transitioned from LOCALIZATION, and we are initializing
    // again, then we don't know what the robot state is!
    if (this->stage != Stage::LOCALIZATION)
    {
        this->stage = Stage::UNKNOWN;
    }
    else
    {
        this->localization_controller.initialize();
    }
}

void AutoController::setCancelled()
{
    switch (this->stage)
    {
        case Stage::LOCALIZATION:
        {
            this->localization_controller.setCancelled();
            break;
        }
        case Stage::TRAVERSE_TO_MINING:
        case Stage::TRAVERSE_TO_OFFLOAD:
        {
            this->traversal_controller.setCancelled();
            break;
        }
        case Stage::MINING:
        {
            this->mining_controller.setCancelled();
            break;
        }
        case Stage::OFFLOAD:
        {
            this->offload_controller.setCancelled();
            break;
        }
        default:
        {
        }
    }
}

void AutoController::iterate(
    uint8_t opts,
    const RobotMotorStatus& motor_status,
    RobotMotorCommands& commands)
{
    switch (this->stage)
    {
        case Stage::UNKNOWN:
        {
            // for now, just rerun localization as a check since it should exit
            // immediately as long as a global alignment transform was
            // previously published
            this->localization_controller.initialize();
            this->stage = Stage::LOCALIZATION;
            [[fallthrough]];
        }
        case Stage::LOCALIZATION:
        {
            this->localization_controller.iterate(motor_status, commands);
            if (!this->localization_controller.isFinished())
            {
                break;
            }

            this->targetInitialTraversalToMining();
            this->stage = Stage::TRAVERSE_TO_MINING;
            [[fallthrough]];
        }
        TRAVERSE_TO_MINING_L:
        case Stage::TRAVERSE_TO_MINING:
        {
            this->traversal_controller.iterate(motor_status, commands);
            if (!this->traversal_controller.isFinished())
            {
                break;
            }

            this->mining_controller.initialize(
                opts & static_cast<uint8_t>(ControlOpts::QUICK_AUTO));
            this->stage = Stage::MINING;
            [[fallthrough]];
        }
        case Stage::MINING:
        {
            this->mining_controller.iterate(motor_status, commands);
            if (!this->mining_controller.isFinished())
            {
                break;
            }

            this->traversal_controller.initializeZone(
                this->params.bounds.construction_zone.min(),
                this->params.bounds.construction_zone.max());
            this->stage = Stage::TRAVERSE_TO_OFFLOAD;
            [[fallthrough]];
        }
        case Stage::TRAVERSE_TO_OFFLOAD:
        {
            this->traversal_controller.iterate(motor_status, commands);
            if (!this->traversal_controller.isFinished())
            {
                break;
            }

            this->offload_controller.initialize();
            this->stage = Stage::OFFLOAD;
            [[fallthrough]];
        }
        case Stage::OFFLOAD:
        {
            this->offload_controller.iterate(motor_status, commands);
            if (!this->offload_controller.isFinished())
            {
                break;
            }

            this->traversal_controller.initializeZone(
                this->params.bounds.mining_zone.min(),
                this->params.bounds.mining_zone.max());
            // chatgpt says I should change this to a while loop that wraps the entire switch-case
            this->stage = Stage::TRAVERSE_TO_MINING;
            goto TRAVERSE_TO_MINING_L;
        }
        default:
        {
        }
    }
}

void AutoController::targetInitialTraversalToMining()
{
    if (this->sensing_interfaces.tf_cache.hasTf(ROBOT_TO_ARENA_TF))
    {
        using namespace lance::geom;

        const Vec3f& pos =
            this->sensing_interfaces.tf_cache.getTf(ROBOT_TO_ARENA_TF)
                ->pose.vec;
        const Vec2f p2 = pos.template head<2>();

#define MINING_ZONE this->params.bounds.mining_zone
        // attempt to target the opposite half if already inside the zone
        if (MINING_ZONE.contains(p2))
        {
            const Vec2f diff_max = (p2 - MINING_ZONE.max()).cwiseAbs();
            const Vec2f diff_min = (p2 - MINING_ZONE.min()).cwiseAbs();
            const float farthest_x = (diff_max.x() > diff_min.x())
                                         ? MINING_ZONE.max().x()
                                         : MINING_ZONE.min().x();
            const float farthest_y = (diff_max.y() > diff_min.y())
                                         ? MINING_ZONE.max().y()
                                         : MINING_ZONE.min().y();

            Box2f z = MINING_ZONE;
            if (farthest_x > farthest_y)
            {
                z.min().x() = std::min(farthest_x, MINING_ZONE.center().x());
                z.max().x() = std::max(farthest_x, MINING_ZONE.center().x());
            }
            else
            {
                z.min().y() = std::min(farthest_y, MINING_ZONE.center().y());
                z.max().y() = std::max(farthest_y, MINING_ZONE.center().y());
            }

            this->traversal_controller.initializeZone(z.min(), z.max());
            return;
        }
#undef MINING_ZONE
    }

    // default to targetting full mining zone bounds
    this->traversal_controller.initializeZone(
        this->params.bounds.mining_zone.min(),
        this->params.bounds.mining_zone.max());
}

};  // namespace lance
