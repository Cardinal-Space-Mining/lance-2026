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

#include "auto_mining_controller.hpp"


namespace lance
{

AutoMiningController::AutoMiningController(
    const RobotParams& params,
    SensingInterfaces& sensing_interfaces,
    SharedControllerCollection& controllers) :
    params{params},
    sensing_interfaces{sensing_interfaces},
    traversal_controller{controllers.traversal_controller},
    mining_controller{controllers.mining_controller},
    mining_planner{sensing_interfaces.mining_eval_interface, params}
{
}

void AutoMiningController::initialize(bool quick)
{
    this->is_quick_run = quick;
    this->stage = Stage::INITIALIZATION;
}

bool AutoMiningController::isFinished()
{
    return this->stage == Stage::FINISHED;
}

void AutoMiningController::setCancelled()
{
    switch (this->stage)
    {
        case Stage::TRAVERSING:
        {
            this->traversal_controller.setCancelled();
            break;
        }
        case Stage::MINING:
        {
            this->mining_controller.setCancelled();
            break;
        }
        default:
        {
        }
    }
    this->stage = Stage::FINISHED;
}

void AutoMiningController::iterate(
    const RobotMotorStatus& motor_status,
    RobotMotorCommands& commands)
{
    int iteration_count = 0;

    switch (this->stage)
    {
        case Stage::INITIALIZATION:
        {
            this->stage = Stage::PLANNING;
            [[fallthrough]];
        }
        case Stage::PLANNING:
        {
            // this->mining_planner.iterate();
            if (!this->mining_planner.updateMappedMatrices())
            {
                break;
            }


            // std::cout
            //     << "Mining evaluation results received. Generating mining paths...\n";

            const MiningPlanner::DirectedMiningPaths& paths =
                mining_planner.finalOutput();


            if (paths.empty())
            {
                std::cout
                    << "Uh oh, no mining paths found. Finishing auto mining controller.\n";
                this->stage = Stage::FINISHED;
                break;
            }
            DirectedMiningPath::MiningSwath swath;

            if (is_quick_run)
            {
                swath =
                    paths.back()
                        .getPathStartInWorldFrame();  // Lowest quality shortest path for first run (full cycle)
                this->current_mining_path = paths.back();
            }
            else
            {
                swath = paths.front().getPathStartInWorldFrame();
                this->current_mining_path = paths.front();
                paths.front().print();
            }


            this->traversal_controller.initializePoint(
                swath.first,
                swath.second);

            this->stage = Stage::TRAVERSING;
            [[fallthrough]];
        }
        case Stage::TRAVERSING:
        {
            this->traversal_controller.iterate(motor_status, commands);

            // adding in a check to see if the path is still valid
            std::cout << "Checking path validity during traversal...\n";
            if (!this->mining_planner.recheckPathValidity(this->current_mining_path.value()))
            {
                std::cout << "Current path is no longer valid. Replanning...\n";
                this->stage = Stage::PLANNING;
                break;
            }

            if (!this->traversal_controller.isFinished())
            {
                break;
            }

            this->mining_controller.initialize(
                this->is_quick_run ? -this->params.preset_mining_vol_l : 0.f);
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

            if (this->current_mining_path.has_value())
            {
                this->mining_planner.markMiningOnMatrix(
                    this->current_mining_path.value());
                this->current_mining_path.reset();
            }

            if (!is_quick_run &&
                (iteration_count < this->params.auto_mining_max_iterations) &&
                (this->mining_controller.hopper_state.remainingVolume() >
                 this->params.auto_mining_min_replan_vol_liters))
            {
                // Replan if we have enough remaining volume to make it worth it
                this->stage = Stage::PLANNING;
                iteration_count += 1;
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
