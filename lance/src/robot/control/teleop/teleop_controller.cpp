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

void TeleopController::setCancelled() { this->cancelCurrentCommand(); }

void TeleopController::iterate(
    const JoyState& joy,
    const RobotMotorStatus& motor_status,
    RobotMotorCommands& commands)
{
    if (!this->handleGlobalControls(joy))
    {
        this->clearRemoteCommand();
        commands.disableAll();
        return;
    }

    this->handleRemoteCommand();

    // iterate controllers... if inputs result in finish state, continue
    // to iterate manual mode below (motor commands meaningless anyway)
    this->iterateCurrentCommand(joy, motor_status, commands);

    // controllers either iterated and didn't finish (op_mode isn't MANUAL),
    // or a transition to MANUAL occurred, in which case we can override
    // any motor commands since the original operation is completed
    if (this->op_mode == Operation::MANUAL)
    {
        this->handleManualControl(joy, motor_status, commands);
    }
}



bool TeleopController::handleGlobalControls(const JoyState& joy)
{
    this->mining_controller.updateConstraints(joy);

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
        this->cancelCurrentCommand();
        this->op_mode = Operation::MANUAL;
        return false;
    }

    return true;
}

void TeleopController::cancelCurrentCommand()
{
    switch (this->op_mode)
    {
        case Operation::ASSISTED_MINING:
        case Operation::PLANNED_MINING_E:
        {
            this->mining_controller.setCancelled();
            break;
        }
        case Operation::ASSISTED_OFFLOAD:
        case Operation::PLANNED_OFFLOAD_E:
        {
            this->offload_controller.setCancelled();
            break;
        }
        case Operation::PLANNED_TRAVERSAL:
        case Operation::PLANNED_MINING_T:
        case Operation::PLANNED_OFFLOAD_T:
        {
            this->traversal_controller.setCancelled();
            break;
        }
        default:
        {
        }
    }
}

void TeleopController::clearRemoteCommand() { this->remote_command.reset(); }

void TeleopController::handleRemoteCommand()
{
    if (this->remote_command)
    {
        geom::Pose3f dest;
        switch (RemoteCommands::getCmdType(*this->remote_command))
        {
            case RemoteCommands::COMMAND_TRAVERSAL:
            {
                KeyFrame f;
                if (RemoteCommands::deserializeTraversalCmd(
                        *this->remote_command,
                        dest,
                        f))
                {
                    this->traversal_controller.initializePose(dest, f);
                    this->op_mode = Operation::PLANNED_TRAVERSAL;
                }
                break;
            }
            case RemoteCommands::COMMAND_MINING:
            {
                if (RemoteCommands::deserializeMiningCmd(
                        *this->remote_command,
                        dest))
                {
                    this->traversal_controller.initializePose(
                        dest,
                        KeyFrame::ARENA_FRAME);
                    this->op_mode = Operation::PLANNED_MINING_T;
                }
                break;
            }
            case RemoteCommands::COMMAND_OFFLOAD:
            {
                float dist;
                if (RemoteCommands::deserializeOffloadCmd(
                        *this->remote_command,
                        dest,
                        dist))
                {
                    this->traversal_controller.initializePose(
                        dest,
                        KeyFrame::ARENA_FRAME);
                    this->offload_controller.initialize(dist);
                    this->op_mode = Operation::PLANNED_OFFLOAD_T;
                }
            }
            default:
            {
            }
        }
        this->remote_command.reset();
    }
}

void TeleopController::iterateCurrentCommand(
    const JoyState& joy,
    const RobotMotorStatus& motor_status,
    RobotMotorCommands& commands)
{
    switch (this->op_mode)
    {
        case Operation::ASSISTED_MINING:
        {
            this->iterateAssistedMining(joy, motor_status, commands);
            break;
        }
        case Operation::ASSISTED_OFFLOAD:
        {
            this->iterateAssistedOffload(joy, motor_status, commands);
            break;
        }
        case Operation::PLANNED_TRAVERSAL:
        {
            this->iteratePlannedTraversal(motor_status, commands);
            break;
        }
        case Operation::PLANNED_MINING_T:
        case Operation::PLANNED_MINING_E:
        {
            this->iteratePlannedMining(motor_status, commands);
            break;
        }
        case Operation::PLANNED_OFFLOAD_T:
        case Operation::PLANNED_OFFLOAD_E:
        {
            this->iteratePlannedOffload(motor_status, commands);
            break;
        }
        default:
        {
        }
    }
}

void TeleopController::handleManualControl(
    const JoyState& joy,
    const RobotMotorStatus& motor_status,
    RobotMotorCommands& commands)
{
    if (AssistedMiningToggleButton::wasPressed(joy))
    {
        this->mining_controller.initialize();
        this->mining_controller.iterate(joy, motor_status, commands);
        this->op_mode = Operation::ASSISTED_MINING;
        return;
    }
    if (AssistedOffloadToggleButton::wasPressed(joy))
    {
        this->offload_controller.initialize();
        this->offload_controller.iterate(joy, motor_status, commands);
        this->op_mode = Operation::ASSISTED_OFFLOAD;
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
        // if ((std::abs(hopper_act_scalar) <
        //      this->params.default_stick_deadzone) ||
        //     (motor_status.getHopperActNormalizedValue() < 0. &&
        //      hopper_act_scalar < 0.f))
        // {
        //     hopper_act_scalar = 0.f;
        // }
        commands.setHopperActVoltage(hopper_act_scalar * 20.0);
    }
}


void TeleopController::iterateAssistedMining(
    const JoyState& joy,
    const RobotMotorStatus& motor_status,
    RobotMotorCommands& commands)
{
    this->mining_controller.iterate(joy, motor_status, commands);
    if (this->mining_controller.isFinished())
    {
        this->op_mode = Operation::MANUAL;
    }
}
void TeleopController::iterateAssistedOffload(
    const JoyState& joy,
    const RobotMotorStatus& motor_status,
    RobotMotorCommands& commands)
{
    this->offload_controller.iterate(joy, motor_status, commands);
    if (this->offload_controller.isFinished())
    {
        this->op_mode = Operation::MANUAL;
    }
}
void TeleopController::iteratePlannedTraversal(
    const RobotMotorStatus& motor_status,
    RobotMotorCommands& commands)
{
    this->traversal_controller.iterate(motor_status, commands);
    if (this->traversal_controller.isFinished())
    {
        this->op_mode = Operation::MANUAL;
    }
}
void TeleopController::iteratePlannedMining(
    const RobotMotorStatus& motor_status,
    RobotMotorCommands& commands)
{
    switch (this->op_mode)
    {
        case Operation::PLANNED_MINING_T:
        {
            this->traversal_controller.iterate(motor_status, commands);
            if (this->traversal_controller.isFinished())
            {
                this->mining_controller.initialize();
                this->op_mode = Operation::PLANNED_MINING_E;
                [[fallthrough]];
            }
            else
            {
                return;
            }
        }
        case Operation::PLANNED_MINING_E:
        {
            this->mining_controller.iterate(motor_status, commands);
            if (!this->mining_controller.isFinished())
            {
                return;
            }
            [[fallthrough]];
        }
        default:
        {
        }
    }

    this->op_mode = Operation::MANUAL;
}
void TeleopController::iteratePlannedOffload(
    const RobotMotorStatus& motor_status,
    RobotMotorCommands& commands)
{
    switch (this->op_mode)
    {
        case Operation::PLANNED_OFFLOAD_T:
        {
            this->traversal_controller.iterate(motor_status, commands);
            if (!this->traversal_controller.isFinished())
            {
                return;
            }
            // offload controller was already initialized with target distance
            // when remote command was deserialized
            this->op_mode = Operation::PLANNED_OFFLOAD_E;
            [[fallthrough]];
        }
        case Operation::PLANNED_OFFLOAD_E:
        {
            this->offload_controller.iterate(motor_status, commands);
            if (!this->offload_controller.isFinished())
            {
                return;
            }
            [[fallthrough]];
        }
        default:
        {
        }
    }

    this->op_mode = Operation::MANUAL;
}

};  // namespace lance
