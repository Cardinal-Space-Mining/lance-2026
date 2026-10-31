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
    polar_planner{
        sensing_interfaces.mining_eval_interface,
        sensing_interfaces.tf_cache,
        params},
    mining_planner{sensing_interfaces.mining_eval_interface, params}
{
}

void AutoMiningController::initialize(bool quick)
{
    this->is_quick_run = quick;
    this->stage = Stage::INITIALIZATION;
    this->iteration_count = 0;
    this->polar_planner.reset();
    this->leg1_dist = std::numeric_limits<float>::infinity();
    this->pending_leg2_dir.reset();
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
        case Stage::PIVOT_TURNING:
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
    this->polar_planner.reset();
    this->pending_leg2_dir.reset();
    this->stage = Stage::FINISHED;
}

void AutoMiningController::iterate(
    const RobotMotorStatus& motor_status,
    RobotMotorCommands& commands)
{
    //int iteration_count = 0;

    switch (this->stage)
    {
        // Leg 1 of a two-leg route reached its pivot: re-sweep from here so
        // leg 2 is chosen against the current terrain
        case Stage::PIVOT_PLANNING:
        {
            std::optional<PolarMiningPlanner::Result> res;
            if (!this->polar_planner.iteratePivot(this->leg1_dir, res))
            {
                break;
            }

            if (res.has_value())
            {
                std::cout << "[AutoMiningController] Turning for second leg\n";
                this->traversal_controller.initializePoint(
                    res->origin,
                    res->dir);
                this->stage = Stage::PIVOT_TURNING;
                break;
            }

            std::cout << "[AutoMiningController] No second leg from pivot\n";
            this->replanOrFinish();
            break;
        }
        // Turning in place at the pivot of a two-leg route (leg 1 is done and
        // the trencher is raised); then mine leg 2 with no distance limit
        case Stage::PIVOT_TURNING:
        {
            this->traversal_controller.iterate(motor_status, commands);
            if (!this->traversal_controller.isFinished())
            {
                break;
            }

            this->mining_controller.initialize(0.f);
            this->stage = Stage::MINING;
            break;
        }
        case Stage::INITIALIZATION:
        {
            this->stage = Stage::POLAR_PLANNING;
            [[fallthrough]];
        }
        case Stage::POLAR_PLANNING:
        {
            // Quick runs keep the grid planner's shortest-path behavior
            if (!this->is_quick_run)
            {
                std::optional<PolarMiningPlanner::Result> res;
                if (!this->polar_planner.iterate(res))
                {
                    break;
                }

                if (res.has_value())
                {
                    this->current_mining_path.reset();
                    this->leg1_dist = res->leg1_dist;
                    this->leg1_dir = res->dir;
                    this->pending_leg2_dir = res->leg2_dir;
                    this->traversal_controller.initializePoint(
                        res->origin,
                        res->dir);
                    this->stage = Stage::TRAVERSING;
                    break;
                }

                std::cout << "Polar mining planner found no heading. Falling "
                             "back to grid planner.\n";
            }

            this->stage = Stage::GRID_PLANNING;
            [[fallthrough]];
        }
        case Stage::GRID_PLANNING:
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


            // Grid path chosen, so stop visualizing the stale polar sweep
            this->polar_planner.clearLastSweep();
            this->leg1_dist = std::numeric_limits<float>::infinity();
            this->pending_leg2_dir.reset();

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
            // std::cout << "Checking path validity during traversal...\n";
            // Only grid paths need revalidation; polar headings start in place
            if (this->current_mining_path.has_value() &&
                !this->mining_planner.recheckPathValidity(this->current_mining_path.value()))
            {
                // std::cout << "Current path is no longer valid. Replanning...\n";Load
                this->stage = Stage::GRID_PLANNING;
                break;
            }

            if (!this->traversal_controller.isFinished())
            {
                break;
            }

            // leg1_dist is finite only for the first leg of a two-leg route
            this->mining_controller.initialize(
                this->is_quick_run ? -this->params.preset_mining_vol_l : 0.f,
                this->leg1_dist);
            this->leg1_dist = std::numeric_limits<float>::infinity();
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

            // Leg 1 of a two-leg route reached its pivot: re-sweep to choose
            // leg 2. If mining stopped for any other reason (hopper full,
            // obstacle, zone edge, stall), the second leg is dropped.
            if (this->pending_leg2_dir.has_value())
            {
                this->pending_leg2_dir.reset();

                if (this->mining_controller.constraints.currentConstraint() ==
                    MiningConstraints::CONSTRAINT_DISTANCE_LIMIT)
                {
                    std::cout << "[AutoMiningController] Reached pivot, "
                                 "re-sweeping for second leg\n";
                    this->polar_planner.reset();
                    this->stage = Stage::PIVOT_PLANNING;
                    break;
                }
                std::cout << "[AutoMiningController] First leg ended before "
                             "pivot, skipping second leg\n";
            }

            this->replanOrFinish();
            break;
        }
        case Stage::FINISHED:
        {
        }
    }
}

void AutoMiningController::replanOrFinish()
{
    if (!is_quick_run &&
        (iteration_count < this->params.auto_mining_max_iterations) &&
        (this->mining_controller.hopper_state.remainingVolume() >
         this->params.auto_mining_min_replan_vol_liters))
    {
        // Replan if we have enough remaining volume to make it worth it
        this->stage = Stage::POLAR_PLANNING;
        iteration_count += 1;
        return;
    }

    this->stage = Stage::FINISHED;
}

};  // namespace lance
