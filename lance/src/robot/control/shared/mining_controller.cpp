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

#include "mining_controller.hpp"

#include <limits>

#include "robot/core/hid_bindings.hpp"
#include "robot/model/dynamics.hpp"


namespace lance
{

MiningConstraints::MiningConstraints(
    const RobotParams& params,
    const StallState& stall_state) :
    params{params},
    stall_state{stall_state}
{
}

void MiningConstraints::updateSettings(const JoyState& joy)
{
    if (ToggleMiningStallConstraintButton::wasPressed(joy))
    {
        this->enabled_constraints ^= CONSTRAINT_MOTOR_STALL;
    }
    if (ToggleMiningObstacleConstraintButton::wasPressed(joy))
    {
        this->enabled_constraints ^= CONSTRAINT_OBSTACLE;
    }
    if (ToggleMiningHopperConstraintButton::wasPressed(joy))
    {
        this->enabled_constraints ^= CONSTRAINT_HOPPER_FULL;
    }
    if (ToggleMiningZoneConstraintButton::wasPressed(joy))
    {
        this->enabled_constraints ^= CONSTRAINT_ZONE_BOUNDARY;
    }
}
void MiningConstraints::resetState()
{
    this->remaining_dist = std::numeric_limits<float>::infinity();
    this->prev_odom = std::numeric_limits<float>::infinity();
    this->current_constraint = CONSTRAINT_NONE;
}
void MiningConstraints::updateState(
    const RobotMotorStatus& motor_status,
    const HopperState& hopper_state,
    const TfCache& tf_cache,
    const MiningEvalInterface& mining_eval)
{
    this->updateOdom(motor_status);

    if (!(this->current_constraint & this->enabled_constraints))
    {
        // tf cache or eval invalidation might also need to trigger this?
        this->remaining_dist = std::numeric_limits<float>::infinity();
    }

    if ((this->enabled_constraints & CONSTRAINT_MOTOR_STALL) &&
        this->stall_state.anyStalled())
    {
        this->remaining_dist = 0.f;
        this->current_constraint = CONSTRAINT_MOTOR_STALL;
    }

    if ((this->enabled_constraints & CONSTRAINT_OBSTACLE) &&
        mining_eval.hasResult() && !mining_eval.getDists()->empty())
    {
        const float d = mining_eval.getDists()->front();
        if (d < this->remaining_dist ||
            this->current_constraint == CONSTRAINT_OBSTACLE)
        {
            this->remaining_dist = d;
            this->current_constraint = CONSTRAINT_OBSTACLE;
        }
    }
    if (this->enabled_constraints & CONSTRAINT_HOPPER_FULL)
    {
        if (hopper_state.isBeltCapacity())
        {
            this->remaining_dist = 0.f;
            this->current_constraint = CONSTRAINT_HOPPER_FULL;
        }
        else
        {
            const float d =
                static_cast<float>(lance::targetVolumeToSweepDistance(
                    hopper_state.remainingVolume(),
                    lance::linearActuatorToMiningDepthClamped(
                        motor_status.getHopperActNormalizedValue())));
            if (d < this->remaining_dist ||
                this->current_constraint == CONSTRAINT_HOPPER_FULL)
            {
                this->remaining_dist = d;
                this->current_constraint = CONSTRAINT_HOPPER_FULL;
            }
        }
    }
    if ((this->enabled_constraints & CONSTRAINT_ZONE_BOUNDARY) &&
        tf_cache.hasTf(ROBOT_TO_ARENA_TF))
    {
        const float d = lance::geom::distToBounds(
                            *tf_cache.getTf(ROBOT_TO_ARENA_TF),
                            this->params.bounds.mining_zone) -
                        lance::geom::TRENCHER_X_MAX_<float>;
        if (d < this->remaining_dist ||
            this->current_constraint == CONSTRAINT_ZONE_BOUNDARY)
        {
            this->remaining_dist = d;
            this->current_constraint = CONSTRAINT_ZONE_BOUNDARY;
        }
    }
}

float MiningConstraints::remainingDist() const { return this->remaining_dist; }
bool MiningConstraints::hasRemaining() const
{
    return this->remaining_dist > 0.f;
}

uint8_t MiningConstraints::enabledConstraints() const
{
    return this->enabled_constraints;
}
uint8_t MiningConstraints::currentConstraint() const
{
    return this->current_constraint;
}
bool MiningConstraints::isConstraintEnabled(uint8_t c) const
{
    return static_cast<bool>(this->enabled_constraints & c);
}

void MiningConstraints::updateOdom(const RobotMotorStatus& motor_status)
{
    const float odom = static_cast<float>(lance::trackMotorRpsToGroundMps(
        0.5 * (motor_status.track_left.position +
               motor_status.track_right.position)));

    if (!std::isinf(this->remaining_dist) && !std::isinf(this->prev_odom))
    {
        this->remaining_dist -= (odom - this->prev_odom);
    }

    this->prev_odom = odom;
}



// --- Mining Controller -------------------------------------------------------

MiningController::MiningController(
    const RobotParams& params,
    const HopperState& hopper_state,
    const StallState& stall_state,
    SensingInterfaces& sensing_interfaces) :
    params{params},
    hopper_state{hopper_state},
    tf_cache{sensing_interfaces.tf_cache},
    mining_eval_interface{sensing_interfaces.mining_eval_interface},
    constraints{params, stall_state}
{
}

void MiningController::initialize()
{
    this->stage = Stage::INITIALIZATION;

    this->mining_eval_interface.queryRobotFrame();
    this->constraints.resetState();
}

bool MiningController::isFinished() { return this->stage == Stage::FINISHED; }

void MiningController::setCancelled()
{
    this->stage = Stage::FINISHED;

    this->mining_eval_interface.cancelQuery();
}

void MiningController::iterate(
    const RobotMotorStatus& motor_status,
    RobotMotorCommands& commands)
{
    this->iterate(nullptr, motor_status, commands);
}

void MiningController::iterate(
    const JoyState& joy,
    const RobotMotorStatus& motor_status,
    RobotMotorCommands& commands)
{
    this->iterate(&joy, motor_status, commands);
}

void MiningController::updateConstraints(const JoyState& joy)
{
    this->constraints.updateSettings(joy);
}


void MiningController::BeltDutyCycleState::setMoved()
{
    this->belt_moving = true;
}
void MiningController::BeltDutyCycleState::setStopped()
{
    if (this->belt_moving)
    {
        this->prev_belt_stop_time = std::chrono::system_clock::now();
        this->belt_moving = false;
    }
}
bool MiningController::BeltDutyCycleState::canMove(float thresh_s)
{
    return std::chrono::duration<float>(
               std::chrono::system_clock::now() - this->prev_belt_stop_time)
               .count() >= thresh_s;
}


void MiningController::iterate(
    const JoyState* joy,
    const RobotMotorStatus& motor_status,
    RobotMotorCommands& commands)
{
    commands.disableAll();

    this->constraints.updateState(
        motor_status,
        this->hopper_state,
        this->tf_cache,
        this->mining_eval_interface);

    if ((this->stage < Stage::RAISING) &&
        (!this->constraints.hasRemaining() ||
         (this->stage != Stage::INITIALIZATION && joy &&
          AssistedMiningToggleButton::wasPressed(*joy))))
    {
        this->stage = Stage::RAISING;
    }

    switch (this->stage)
    {
        case Stage::INITIALIZATION:
        {
            this->stage = Stage::LOWERING;
            [[fallthrough]];
        }
        case Stage::LOWERING:
        {
            const double hopper_act_val =
                motor_status.getHopperActNormalizedValue();
            if (hopper_act_val > this->params.hopper_actuator_mining_target_val)
            {
                commands.setTrencherVelocity(
                    this->params.trencher_mining_velocity_rps);
                if (hopper_act_val >
                    this->params.hopper_actuator_traversal_target_val)
                {
                    commands.setHopperActVelocity(
                        -this->params.hopper_actuator_max_speed);
                }
                else
                {
                    commands.setHopperActVelocity(
                        -this->params.hopper_actuator_plunge_speed);
                }
                break;
            }

            this->stage = Stage::TRAVERSING;
            [[fallthrough]];
        }
        case Stage::TRAVERSING:
        {
            // default setpoints
            float trencher_target = this->params.trencher_mining_velocity_rps;
            float hopper_act_target =
                this->params.hopper_actuator_mining_target_val;
            float tracks_target = this->params.tracks_mining_velocity_rps;
            float hopper_belt_target = 0.f;

            // 1. Set belt via hopper model target
            if (!joy || this->constraints.isConstraintEnabled(
                            MiningConstraints::CONSTRAINT_HOPPER_FULL))
            {
                if (motor_status.hopper_belt.position <
                        this->hopper_state.miningTargetMotorPosition() &&
                    this->belt_duty_cycle.canMove(
                        this->params
                            .hopper_belt_mining_duty_cycle_base_seconds))
                {
                    hopper_belt_target =
                        this->params.hopper_belt_mining_velocity_rps;
                    this->belt_duty_cycle.setMoved();
                }
                else
                {
                    this->belt_duty_cycle.setStopped();
                }
            }

            // 2. Commands/target updates from user input
            if (joy)
            {
                // adjust trencher
                trencher_target =
                    this->params.trencher_mining_velocity_rps *
                    (1.f - TeleopTrencherSpeedAxis::triggerValue(*joy));

                // adjust trencher depth
                {
                    const float raw = TeleopHopperActuateAxis::rawValue(*joy);
                    if (std::abs(raw) >= this->params.default_stick_deadzone)
                    {
                        if (raw > 0.f)
                        {
                            hopper_act_target +=
                                raw *
                                (this->params
                                     .hopper_actuator_transport_target_val -
                                 this->params
                                     .hopper_actuator_mining_target_val);
                        }
                        else if (raw < 0.f)
                        {
                            hopper_act_target +=
                                raw *
                                (this->params
                                     .hopper_actuator_mining_target_val -
                                 this->params.hopper_actuator_mining_min_val);
                        }
                    }
                }
                // adjust tracks
                {
                    const float raw = TeleopDriveForwardAxis::rawValue(*joy);
                    if (std::abs(raw) >=
                        this->params.driving_magnitude_deadzone)
                    {
                        if (raw > 0.f)
                        {
                            tracks_target +=
                                raw *
                                this->params.tracks_mining_adjustment_range_rps;
                        }
                        else if (raw < 0.f)
                        {
                            tracks_target +=
                                raw * this->params.tracks_mining_velocity_rps;
                        }
                    }
                }
                // manual hopper belt - don't override automatic setpts
                if (!this->constraints.isConstraintEnabled(
                        MiningConstraints::CONSTRAINT_HOPPER_FULL))
                {
                    hopper_belt_target =
                        TeleopHopperSpeedAxis::triggerValue(*joy) *
                        this->params.hopper_belt_mining_velocity_rps;
                }
            }

            // 3. Apply targets
            {
                commands.setTrencherVelocity(trencher_target);
                commands.setTracksVelocity(tracks_target, tracks_target);
                commands.setHopperBeltVelocity(hopper_belt_target);

                const double hopper_val =
                    motor_status.getHopperActNormalizedValue();
                if (hopper_val < 0.f ||
                    std::abs(hopper_act_target - hopper_val) <
                        this->params.hopper_actuator_targetting_thresh)
                {
                    commands.disableHopperAct();
                }
                else if (hopper_val < hopper_act_target)
                {
                    commands.setHopperActVelocity(
                        this->params.hopper_actuator_plunge_speed);
                }
                else if (hopper_val > hopper_act_target)
                {
                    commands.setHopperActVelocity(
                        -this->params.hopper_actuator_plunge_speed);
                }
            }

            break;
        }
        case Stage::RAISING:
        {
            if (motor_status.getHopperActNormalizedValue() <
                this->params.hopper_actuator_transport_target_val)
            {
                commands.setTrencherVelocity(
                    this->params.trencher_mining_velocity_rps);
                commands.setHopperActVelocity(
                    this->params.hopper_actuator_extract_speed);
                break;
            }

            this->stage = Stage::FINISHED;
            [[fallthrough]];
        }
        case Stage::FINISHED:
        {
            this->mining_eval_interface.cancelQuery();
        }
    }
}

};  // namespace lance
