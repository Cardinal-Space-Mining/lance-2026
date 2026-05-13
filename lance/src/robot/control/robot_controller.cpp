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

#include "robot_controller.hpp"


inline constexpr int encodeTransition(
    lance::ControlMode from,
    lance::ControlMode to)
{
    return (static_cast<int>(from) << 2) | static_cast<int>(to);
}

template<lance::ControlMode FromV, lance::ControlMode ToV>
inline constexpr int transition_v = encodeTransition(FromV, ToV);


namespace lance
{

RobotController::RobotController(RclNode& node) :
    params{node},
    sensing_interfaces{node, params},
    shared_controllers{
        params,
        this->collection_state.getHopperState(),
        this->sensing_interfaces},
    auto_controller{params, sensing_interfaces, shared_controllers},
    teleop_controller{node, params, sensing_interfaces, shared_controllers}
{
    this->collection_state.setParams(
        this->params.collection_model_initial_volume_liters,
        this->params.collection_model_capacity_volume_liters,
        this->params.collection_model_initial_belt_footprint_meters,
        this->params.collection_model_belt_capacity_meters,
        this->params.collection_model_belt_offload_length_meters,
        this->params.collection_model_transfer_efficiency);
}

const HopperState& RobotController::hopperState() const
{
    return this->collection_state.getHopperState();
}

const RobotParams& RobotController::getParams() const { return this->params; }

const TfCache::Tf2Buffer& RobotController::getTfBuffer() const
{
    return this->sensing_interfaces.tf_cache.getBuffer();
}

void RobotController::iterate(
    int32_t ctrl_status,
    const JoyState& joy,
    const RobotMotorStatus& motor_status,
    RobotMotorCommands& commands)
{
    const RobotMotorStatus& filtered_status =
        this->handleTestModeStateInjection(motor_status, ctrl_status);
    this->collection_state.update(filtered_status);
    this->sensing_interfaces.tf_cache.refresh();

    const ControlMode prev_mode = this->control_mode;
    this->control_mode = ControlStatus::getMode(ctrl_status);

    // process transition actions
    switch (encodeTransition(prev_mode, this->control_mode))
    {
        case transition_v<ControlMode::DISABLED, ControlMode::TELEOPERATED>:
        {
            this->teleop_controller.initialize();
            break;
        }
        case transition_v<ControlMode::DISABLED, ControlMode::AUTONOMOUS>:
        {
            this->auto_controller.initialize();
            break;
        }
        case transition_v<ControlMode::TELEOPERATED, ControlMode::DISABLED>:
        {
            this->teleop_controller.setCancelled();
            break;
        }
        case transition_v<ControlMode::TELEOPERATED, ControlMode::AUTONOMOUS>:
        {
            this->teleop_controller.setCancelled();
            this->auto_controller.initialize();
            break;
        }
        case transition_v<ControlMode::AUTONOMOUS, ControlMode::DISABLED>:
        {
            this->auto_controller.setCancelled();
            break;
        }
        case transition_v<ControlMode::AUTONOMOUS, ControlMode::TELEOPERATED>:
        {
            this->auto_controller.setCancelled();
            this->teleop_controller.initialize();
            break;
        }
        default:
        {
        }
    }

    // process current state actions
    switch (this->control_mode)
    {
        case ControlMode::TELEOPERATED:
        {
            this->teleop_controller.iterate(joy, filtered_status, commands);
            break;
        }
        case ControlMode::AUTONOMOUS:
        {
            this->auto_controller.iterate(filtered_status, commands);
            break;
        }
        default:
        {
        }
    }
}

const RobotMotorStatus& RobotController::handleTestModeStateInjection(
    const RobotMotorStatus& ref,
    int32_t ctrl_status)
{
    if (ControlStatus::hasOpt<ControlOpts::TEST_MODE>(ctrl_status))
    {
        if (ref.getHopperActNormalizedValue() <
            this->params.hopper_actuator_traversal_target_val)
        {
            this->filtered_status = ref;
            this->filtered_status.hopper_actuator.position = -1.;
            return this->filtered_status;
        }
    }
    return ref;
}

};  // namespace lance
