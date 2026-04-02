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
    mining_controller{controllers.mining_controller}
// mining_planner{params}
{
}

void AutoMiningController::initialize() { this->stage = Stage::INITIALIZATION; }

bool AutoMiningController::isFinished()
{
    return this->stage == Stage::FINISHED;
}

void AutoMiningController::setCancelled()
{
    // TODO: need to shutdown sub-controllers if they are running!
    // (ex. traveral needs to cancel pathing service)
    this->stage = Stage::FINISHED;
}

void AutoMiningController::iterate(
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
            // -----------------------------------------------------------------
            // 1. generate target evals
            // 2. geometry converter to build eval list
            // 3. >>
            // this->mining_eval_client->async_send_request(
            //     req,
            //     [](rclcpp::Client<UpdateMiningSrv>::SharedFuture f){ /* Use the response here! */ } );
            // 4. update planner accordingly >>

            // This will update the mining planner's internal matrices based on the current state of the world as perceived by the robot. It should be called periodically to ensure the planner has up-to-date information, but for now we will call it once at the beginning of the routine.
            // mining_planner.updateMappedMatrices();

            // I don't know how to do the query service, but the updateMappedMatrices() would call that a bunch of times
            // I don't think it would have to be called here because it really only needs to be called once (or very periodically)
            // it saves the results in a matrix that is used later on

            // Wouldn't be a bad idea to check the best path it gives you one more time though
            // The final ouput is sorted so the top has the highest quality

            // const MiningPlanner::DirectedMiningPaths& paths =
            //     mining_planner.finalOutput();
            // if (paths.empty())
            // {
            //     std::cout
            //         << "Uh oh, no mining paths found. Finishing auto mining controller.\n";
            //     this->stage = Stage::FINISHED;
            //     break;
            // }
            // else
            // {
            //     // first has coords of where to start, second has (1,0), (-1, 0), (0, -1),
            //     // or (0, 1) depend on the direction it is going
            //     const DirectedMiningPath::MiningSwath& swath =
            //         paths.front().getPathCoordinatesInWorldFrame();

            //     // init with planned destination
            //     this->traversal_controller.initializePoint(
            //         swath.first,
            //         swath.second);
            // }
            // -----------------------------------------------------------------

            // init with planned destination
            this->traversal_controller.initializePoint(
                this->params.mining_zone_bounds.max() -
                    Eigen::Vector2f::Constant(0.8f),
                Eigen::Vector2f{0.f, -1.f});

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

            // initialize with query result
            this->mining_controller.initialize(0.f);
            this->sensing_interfaces.mining_eval_interface.queryRobotFrame();
            this->stage = Stage::MINING;
            [[fallthrough]];
        }
        case Stage::MINING:
        {
            if (this->sensing_interfaces.mining_eval_interface.hasResult())
            {
                this->mining_controller.setRemaining(
                    this->sensing_interfaces.mining_eval_interface.getDists()
                        ->front());
            }
            this->mining_controller.iterate(motor_status, commands);
            if (!this->mining_controller.isFinished())
            {
                break;
            }

            this->sensing_interfaces.mining_eval_interface.cancelQuery();
            this->stage = Stage::FINISHED;
            [[fallthrough]];
        }
        case Stage::FINISHED:
        {
        }
    }
}

};  // namespace lance
