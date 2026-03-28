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


#define FG_CLICKED_POINT_TOPIC "/clicked_point"


namespace lance
{

TeleopController::TeleopController(
    RclNode& node,
    GenericPubMap& pub_map,
    const RobotParams& params,
    SharedControllerCollection& controllers) :
    pub_map{pub_map},
    params{params},
    clicked_point_sub{node.create_subscription<PointStampedMsg>(
        FG_CLICKED_POINT_TOPIC,
        rclcpp::SensorDataQoS{},
        [this](const PointStampedMsg::ConstSharedPtr& msg)
        { this->clicked_point = msg; })},
    driving_rps_scalar{
        params.driving_medium_scalar * params.tracks_max_velocity_rps},
    mining_controller{controllers.mining_controller},
    offload_controller{controllers.offload_controller},
    traversal_controller{controllers.traversal_controller}
{
}

void TeleopController::initialize() { this->op_mode = Operation::MANUAL; }

void TeleopController::setCancelled()
{
    switch (this->op_mode)
    {
        case Operation::ASSISTED_MINING:
        case Operation::PRESET_MINING:
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
        case Operation::AUTO_TRAVERSAL:
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
        case Operation::PRESET_MINING:
        {
            this->mining_controller.iterate(motor_status, commands);
            command_finished = this->mining_controller.isFinished();
            break;
        }
        case Operation::PRESET_OFFLOAD:
        {
            this->offload_controller.iterate(motor_status, commands);
            command_finished = this->offload_controller.isFinished();
            break;
        }
        case Operation::AUTO_TRAVERSAL:
        {
            this->handleClickedPoint(true);
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
            case Operation::PRESET_MINING:
            {
                this->mining_controller.iterate(motor_status, commands);
                break;
            }
            case Operation::PRESET_OFFLOAD:
            {
                this->offload_controller.iterate(motor_status, commands);
                break;
            }
            case Operation::AUTO_TRAVERSAL:
            {
                this->traversal_controller.iterate(motor_status, commands);
                break;
            }
            default:
            {
            }
        }
    }

    this->handleClickedPoint(false);
}

bool TeleopController::handleGlobalInputs(const JoyState& joy)
{
    using namespace Bindings;

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

bool TeleopController::handleClickedPoint(bool can_apply)
{
    if (this->clicked_point && can_apply)
    {
        this->traversal_controller.initializePoint(*this->clicked_point);
        this->op_mode = Operation::AUTO_TRAVERSAL;
        this->clicked_point = nullptr;
        return true;
    }
    this->clicked_point = nullptr;
    return false;
}

void TeleopController::handleTeleopInputs(
    const JoyState& joy,
    const RobotMotorStatus& motor_status,
    RobotMotorCommands& commands)
{
    using namespace Bindings;

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

    if (PresetMiningInitButton::wasPressed(joy))
    {
        this->mining_controller.initialize(
            this->params.preset_mining_traversal_dist_meters);
        this->op_mode = Operation::PRESET_MINING;
        return;
    }
    if (PresetOffloadInitButton::wasPressed(joy))
    {
        this->offload_controller.initialize(
            this->params.preset_offload_backup_dist_meters);
        this->op_mode = Operation::PRESET_OFFLOAD;
        return;
    }
    if (this->handleClickedPoint(true))
    {
        return;
    }

    // tracks
    {
        const float x = TeleopDriveXAxis::rawValue(joy);
        const float y = TeleopDriveYAxis::rawValue(joy);
        if ((x * x + y * y) < (this->params.driving_magnitude_deadzone *
                               this->params.driving_magnitude_deadzone))
        {
            commands.setTracksVelocity(0., 0.);
        }
        else
        {
            const float l = y - x;  // y + (-x)
            const float r = y + x;  // y - (-x)
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
