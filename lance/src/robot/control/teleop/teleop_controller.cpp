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

#include "teleop_controller.hpp"

#include "robot/core/hid_bindings.hpp"
#include "robot/core/ros_interface.hpp"
#include "robot/model/geometry.hpp"

#include "remote_commands.hpp"


namespace lance
{

TeleopController::TeleopController(
    RclNode& node,
    const RobotParams& params,
    SensingInterfaces& sensing_interfaces,
    SharedControllerCollection& controllers) :
    params{params},
    sensing_interfaces{sensing_interfaces},
    mining_controller{controllers.mining_controller},
    offload_controller{controllers.offload_controller},
    traversal_controller{controllers.traversal_controller},
    remote_commands_sub{node.create_subscription<BytesMsg>(
        lance::REMOTE_COMMANDS_TOPIC,
        rclcpp::SensorDataQoS{},
        [this](const BytesMsg::ConstSharedPtr& msg)
        { this->remote_command = msg; })},
    driving_rps_scalar{
        params.driving_medium_scalar * params.tracks_max_velocity_rps}
{
}

void TeleopController::initialize() { this->op_mode = Operation::MANUAL; }

void TeleopController::setCancelled()
{
    switch (this->op_mode)
    {
        case Operation::ASSISTED_MINING:
        {
            this->mining_controller.setCancelled();
            break;
        }
        case Operation::ASSISTED_OFFLOAD:
        case Operation::PRESET_OFFLOAD:
        {
            this->offload_controller.setCancelled();
            break;
        }
        case Operation::PLANNED_TRAVERSAL:
        {
            this->traversal_controller.setCancelled();
            break;
        }
        default:
        {
        }
    }
}

void TeleopController::iterate(
    const JoyState& joy,
    const RobotMotorStatus& motor_status,
    RobotMotorCommands& commands)
{
    // handle "config" setters and "disable all" button
    if (!this->handleGlobalInputs(joy))
    {
        commands.disableAll();
        return;
    }

    this->mining_controller.updateConstraints(joy);

    // iterate controllers... if inputs result in finish state, continue
    // to iterate manual mode below (motor commands meaningless anyway)
    bool command_finished = false;
    switch (this->op_mode)
    {
        case Operation::ASSISTED_MINING:
        {
            this->mining_controller.iterate(joy, motor_status, commands);
            command_finished = this->mining_controller.isFinished();
            break;
        }
        case Operation::ASSISTED_OFFLOAD:
        {
            this->offload_controller.iterate(joy, motor_status, commands);
            command_finished = this->offload_controller.isFinished();
            break;
        }
        case Operation::PRESET_OFFLOAD:
        {
            this->offload_controller.iterate(motor_status, commands);
            command_finished = this->offload_controller.isFinished();
            break;
        }
        case Operation::PLANNED_TRAVERSAL:
        {
            this->handleRemoteCommand(true);
            this->traversal_controller.iterate(motor_status, commands);
            command_finished = this->traversal_controller.isFinished();
            break;
        }
        default:
        {
        }
    }
    if (command_finished)
    {
        this->op_mode = Operation::MANUAL;
        // commands.disableAll(); <-- can add back to be extra safe
    }

    // controllers either iterated and didn't finish (op_mode isn't MANUAL),
    // or a transition to MANUAL occurred, in which case we can override
    // any motor commands since they are worthless
    if (this->op_mode == Operation::MANUAL)
    {
        // handle manual control
        this->handleTeleopInputs(joy, motor_status, commands);

        // iterate controllers ONLY IF an op_mode transition occurred
        // (otherwise op_mode will still be MANUAL)
        switch (this->op_mode)
        {
            case Operation::ASSISTED_MINING:
            {
                this->mining_controller.iterate(joy, motor_status, commands);
                break;
            }
            case Operation::ASSISTED_OFFLOAD:
            {
                this->offload_controller.iterate(joy, motor_status, commands);
                break;
            }
            case Operation::PRESET_OFFLOAD:
            {
                this->offload_controller.iterate(motor_status, commands);
                break;
            }
            case Operation::PLANNED_TRAVERSAL:
            {
                this->traversal_controller.iterate(motor_status, commands);
                break;
            }
            default:
            {
            }
        }
    }

    this->handleRemoteCommand(false);
}

bool TeleopController::handleGlobalInputs(const JoyState& joy)
{
    if (TeleopLowSpeedButton::wasPressed(joy))
    {
        this->driving_rps_scalar = this->params.driving_low_scalar *
                                   this->params.tracks_max_velocity_rps;
    }
    if (TeleopMediumSpeedButton::wasPressed(joy))
    {
        this->driving_rps_scalar = this->params.driving_medium_scalar *
                                   this->params.tracks_max_velocity_rps;
    }
    if (TeleopHighSpeedButton::wasPressed(joy))
    {
        this->driving_rps_scalar = this->params.driving_high_scalar *
                                   this->params.tracks_max_velocity_rps;
    }

    if (DisableAllActionsButton::rawValue(joy))
    {
        this->mining_controller.setCancelled();
        this->offload_controller.setCancelled();
        this->traversal_controller.setCancelled();
        this->op_mode = Operation::MANUAL;
        return false;
    }

    return true;
}

bool TeleopController::handleRemoteCommand(bool can_apply)
{
    if (this->remote_command && can_apply)
    {
        geom::Pose3f dest;
        KeyFrame f;
        if (RemoteCommands::deserializeTraversalCmd(
                *this->remote_command,
                dest,
                f))
        {
            this->traversal_controller.initializePose(dest, f);
            this->op_mode = Operation::PLANNED_TRAVERSAL;
        }
        this->remote_command = nullptr;
        return true;
    }
    this->remote_command = nullptr;
    return false;
}

void TeleopController::handleTeleopInputs(
    const JoyState& joy,
    const RobotMotorStatus& motor_status,
    RobotMotorCommands& commands)
{
    if (AssistedMiningToggleButton::wasPressed(joy))
    {
        this->mining_controller.initialize();
        this->op_mode = Operation::ASSISTED_MINING;
        return;
    }
    if (AssistedOffloadToggleButton::wasPressed(joy))
    {
        this->offload_controller.initialize();
        this->op_mode = Operation::ASSISTED_OFFLOAD;
        return;
    }

    if (PresetOffloadInitButton::wasPressed(joy))
    {
        this->offload_controller.initialize(
            this->params.preset_offload_backup_dist_meters);
        this->op_mode = Operation::PRESET_OFFLOAD;
        return;
    }
    if (this->handleRemoteCommand(true))
    {
        return;
    }

    // tracks
    {
        const float f = TeleopDriveForwardAxis::rawValue(joy);
        const float o = TeleopDriveRotationAxis::rawValue(joy);
        if ((f * f + o * o) < (this->params.driving_magnitude_deadzone *
                               this->params.driving_magnitude_deadzone))
        {
            commands.setTracksVelocity(0., 0.);
        }
        else
        {
            const float l = f - o;
            const float r = f + o;
            const float s =
                (this->driving_rps_scalar /
                 std::max({1.f, std::abs(l), std::abs(r)}));
            commands.setTracksVelocity((l * s), (r * s));
        }
    }
    // trencher
    {
        float trencher_percent = TeleopTrencherSpeedAxis::triggerValue(joy);
        if (TeleopTrencherInvertButton::rawValue(joy))
        {
            trencher_percent *= -1.f;
        }

        commands.setTrencherVelocity(
            trencher_percent * this->params.trencher_max_velocity_rps);
    }
    // hopper
    {
        float hopper_belt_percent = TeleopHopperSpeedAxis::triggerValue(joy);
        if (TeleopHopperInvertButton::rawValue(joy))
        {
            hopper_belt_percent *= -1.;
        }
        commands.setHopperBeltVelocity(
            hopper_belt_percent * this->params.hopper_belt_max_velocity_rps);

        float hopper_act_scalar = TeleopHopperActuateAxis::rawValue(joy);
        if ((std::abs(hopper_act_scalar) <
             this->params.default_stick_deadzone) ||
            (motor_status.getHopperActNormalizedValue() < 0. &&
             hopper_act_scalar < 0.f))
        {
            hopper_act_scalar = 0.f;
        }
        commands.setHopperActPercent(hopper_act_scalar);
    }
}

};  // namespace lance
