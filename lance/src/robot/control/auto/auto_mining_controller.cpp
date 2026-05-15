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

void AutoMiningController::initialize() { this->stage = Stage::INITIALIZATION; }

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


            std::cout << "Mining evaluation results received. Generating mining paths...\n";

            const MiningPlanner::DirectedMiningPaths& paths =
                mining_planner.finalOutput();

            std::cout << "Passed final output: " << paths.size() << " mining paths.\n";

            const auto miningDirectionToString = [](lance::MiningDirection dir)
            {
                switch (dir)
                {
                    case lance::MiningDirection::YPLUS:
                        return "YPLUS";
                    case lance::MiningDirection::YMINUS:
                        return "YMINUS";
                    case lance::MiningDirection::XMINUS:
                        return "XMINUS";
                    case lance::MiningDirection::XPLUS:
                        return "XPLUS";
                    default:
                        return "UNKNOWN";
                }
            };
            std::cout << "Evaluated " << paths.size() << " mining paths:\n";
            for (const auto& path : paths)
            {
                std::cout << "\n=== Evaluating Path ===\n";
                lance::DirectedMiningPath::MiningSwath p =
                    path.getPathStartInWorldFrame();
                std::cout << "Base Frame - Start: (" << p.first.x() << ", "
                          << p.first.y() << ")  Direction (In Coords): ("
                          << p.second.x() << ", " << p.second.y()
                          << ") | Direction: "
                          << miningDirectionToString(path.getDirection())
                          << " | Distance: " << path.getDistance()
                          << " | Quality: " << path.getQuality(mining_planner.getPreviouslyMinedCellsByDirection()[path.getDirection()]) << "\n";

                path.print();
                          
            }


            if (paths.empty())
            {
                std::cout
                    << "Uh oh, no mining paths found. Finishing auto mining controller.\n";
                this->stage = Stage::FINISHED;
                break;
            }
            DirectedMiningPath::MiningSwath swath;

            if (is_first_run){
                swath = paths.back().getPathStartInWorldFrame(); // Lowest quality shortest path for first run (full cycle)
                this->current_mining_path = paths.back();
            }
            else{
                swath = paths.front().getPathStartInWorldFrame();
                this->current_mining_path = paths.front();
            }
            
            
            
            std::cout << "USING PATTH - Start: (" << swath.first.x() << ", "
                      << swath.first.y() << ")  Direction (In Coords): ("
                      << swath.second.x() << ", " << swath.second.y()
                      << ") | Direction: "
                      << miningDirectionToString(this->current_mining_path->getDirection())
                      << " | Distance: " << this->current_mining_path->getDistance() << "\n";

            this->traversal_controller.initializePoint(
                swath.first,
                swath.second);

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

            this->mining_controller.initialize();
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
                this->mining_planner.markMiningOnMatrix(this->current_mining_path.value());
                this->current_mining_path.reset();
            }

            if (this->mining_controller.hopper_state.remainingVolume() > this->params.auto_mining_min_replan_vol_liters && 
                iteration_count < this->params.auto_mining_max_iterations &&
                !is_first_run    //On first run you want to mine as little as possible so skip this check
            )
            {
                // Replan if we have enough remaining volume to make it worth it
                this->stage = Stage::PLANNING;
                iteration_count += 1;
                break;
            }
            is_first_run = false;// First run is always false beyond this point
            this->stage = Stage::FINISHED;
            [[fallthrough]];
        }
        case Stage::FINISHED:
        {
        }
    }
}

};  // namespace lance
