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
#include "mining_planner.hpp"
#include <Eigen/Core>

std::function<float(float, float, float)> perception_eval_fn = [](float x, float y, float heading_deg) {
    // Temporary until perception service/client exists.
    // Replace with service call to perception when implemented.
    (void)x;
    (void)heading_deg;
    return std::max(0.0f, 10.0f - y);
};

namespace lance
{

AutoMiningController::AutoMiningController(
    GenericPubMap& pub_map,
    const RobotParams& params,
    SharedControllerCollection& controllers) :
    pub_map{pub_map},
    params{params},
    traversal_controller{controllers.traversal_controller},
    mining_controller{controllers.mining_controller},

    // Creates my implementation of the mining planner
    mining_planner{
        perception_eval_fn,
        params,
        this->params.mining_zone_bounds
    }
{
    (void)node;
}

void AutoMiningController::initialize() { this->stage = Stage::INITIALIZATION; }

bool AutoMiningController::isFinished()
{
    return this->stage == Stage::FINISHED;
}

void AutoMiningController::setCancelled() { this->stage = Stage::FINISHED; }

void AutoMiningController::iterate(
    const RobotMotorStatus& motor_status,
    RobotMotorCommands& commands)
{
    switch (this->stage)
    {
        case Stage::INITIALIZATION:
        {
            this->stage = Stage::PLANNING;

            // This will update the mining planner's internal matrices based on the current state of the world as perceived by the robot. It should be called periodically to ensure the planner has up-to-date information, but for now we will call it once at the beginning of the routine.
            mining_planner.update_mapped_matrices();
            [[fallthrough]];
        }
        case Stage::PLANNING:
        {
            if (false)  // *if not finished planning*
            {
                // call query service, wait for response, determine best option
                
                break;
                


        
            }
            Eigen::Vector2f target_pos;
            Eigen::Vector2f target_dir;

            // I don't know how to do the query service, but the update_mapped_matrices() would call that a bunch of times
            // I don't think it would have to be called here because it really only needs to be called once (or very periodically)
                // it saves the results in a matrix that is used later on
            
            // Wouldn't be a bad idea to check the best path it gives you one more time though
                // The final ouput is sorted so the top has the highest quality

            DirectedMiningPaths paths = mining_planner.final_output();
            if (paths.empty()) {
                std::cout << "Uh oh, no mining paths found. Finishing auto mining controller.\n";
                this->stage = Stage::FINISHED;
                break;
            }
            else {
                std::pair<Eigen::Vector2f,Eigen::Vector2f> base_output = paths.at(0).get_path_coordinates_in_world_frame(params.track_Width);
                target_pos = base_output.first; // has the coords of where to start
                target_dir = base_output.second; // has 1,0 -1,0 0,-1 or 0,1 depending on the path it is going
            }

            // placeholder for testing
            // Eigen::Vector2f target_pos = this->params.mining_zone_bounds.min() +
            //                              Eigen::Vector2f::Constant(0.8f);
            // Eigen::Vector2f target_pos = Eigen::Vector2f{0.f, 0.f};  // this should be set based on query result

            // Eigen::Vector2f target_pos = Eigen::Vector2f{
            //     this->params.mining_zone_bounds.max().x() - 0.8f,
            //     this->params.mining_zone_bounds.min().y() + 0.8f};  // this should be set based on query result
            // Eigen::Vector2f target_pos = Eigen::Vector2f{
            //     this->params.mining_zone_bounds.max().x()-this->params.mining_zone_bounds.sizes().x()/2.f,
            //     this->params.mining_zone_bounds.min().y() + 0.8f};  // this should be set based on query result

            // Eigen::Vector2f target_dir{-1.f, 0.f};

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

            // initialize with query result
            this->mining_controller.initialize(0.f);
            this->stage = Stage::MINING;
            [[fallthrough]];
        }
        case Stage::MINING:
        {
            // this->mining_controller.setRemaining(); // <-- update remaining distance
            this->mining_controller.iterate(motor_status, commands);
            if (!this->mining_controller.isFinished())
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




// pkill -f "gz sim|gzserver|gzclient|foxglove|parameter_bridge|lance1_controller|lance2_controller|perception_node" || true

// cd /home/brandon/lance-ws && pkill -f "gz sim|lance2_controller|parameter_bridge|foxglove_bridge|perception_node" || true && unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH && source /opt/ros/jazzy/setup.bash && colcon build --symlink-install --packages-select lance --cmake-clean-first && source /home/brandon/lance-ws/install/setup.bash && pkill -f "gz sim|gzserver|gzclient|foxglove|parameter_bridge|lance1_controller|lance2_controller|perception_node" || true