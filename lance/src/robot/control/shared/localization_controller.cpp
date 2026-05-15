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

#include "localization_controller.hpp"

#include <cmath>

#include <Eigen/Core>

#include "robot/model/dynamics.hpp"
#include "robot/model/kinematics.hpp"


using Vec2f = Eigen::Vector2f;


namespace lance
{

LocalizationController::LocalizationController(
    const RobotParams& params,
    SensingInterfaces& sensing_interfaces) :
    params{params},
    tf_cache{sensing_interfaces.tf_cache},
    refl_hint_interface{sensing_interfaces.reflector_hint_interface}
{
}

void LocalizationController::initialize()
{
    this->stage = Stage::INITIALIZATION;
    this->refl_hint_interface.clearHint();
}

bool LocalizationController::isFinished()
{
    return this->stage == Stage::FINISHED;
}

void LocalizationController::setCancelled()
{
    this->stage = Stage::FINISHED;
    this->refl_hint_interface.setEnableSrv(false);
}

void LocalizationController::iterate(
    const RobotMotorStatus& motor_status,
    RobotMotorCommands& commands)
{
#define Dt    (this->params.iteration_period_seconds)
#define V_max (this->params.auto_traversal_max_track_velocity_mps)
#define A_max (this->params.auto_traversal_max_track_acceleration_mpss)

    // if at any point the full localization transform is established,
    // the command is finished
    if (this->tf_cache.hasTf(ROBOT_TO_ARENA_TF))
    {
        this->stage = Stage::FINISHED;
    }

    commands.disableAll();

    switch (this->stage)
    {
        case Stage::INITIALIZATION:
        {
            // go up if below
            if (this->params.hopper_actuator_traversal_target_val -
                    motor_status.getHopperActNormalizedValue() >
                this->params.hopper_actuator_targetting_thresh)
            {
                commands.setHopperActVelocity(
                    this->params.hopper_actuator_max_speed);
                break;
            }
            // go down if above
            if (motor_status.getHopperActNormalizedValue() -
                    this->params.hopper_actuator_traversal_target_val >
                this->params.hopper_actuator_targetting_thresh)
            {
                commands.setHopperActVelocity(
                    -this->params.hopper_actuator_max_speed);
                break;
            }

            this->refl_hint_interface.setEnableSrv(true);
            this->stage = Stage::SEARCHING;
            [[fallthrough]];
        }
        case Stage::SEARCHING:
        {
            if (!this->refl_hint_interface.hasHint() ||
                this->refl_hint_interface.getLatestHint()->samples <
                    static_cast<uint32_t>(
                        this->params.auto_localization_min_num_search_samples))
            {
                commands.setTracksVelocity(
                    lance::groundMpsToTrackMotorRps(
                        lance::bodyDynamicsToLeftTrackVelocityMps(
                            0.f,
                            this->params
                                .auto_localization_search_angular_velocity_rps)),
                    lance::groundMpsToTrackMotorRps(
                        lance::bodyDynamicsToRightTrackVelocityMps(
                            0.f,
                            this->params
                                .auto_localization_search_angular_velocity_rps)));
                break;
            }

            this->stage = Stage::ALIGN_HEADING;
            [[fallthrough]];
        }
        case Stage::ALIGN_HEADING:
        {
            // centroid in robot reference frame
            const auto& pt =
                this->refl_hint_interface.getLatestHint()->centroid;
            const Vec2f rel{
                static_cast<float>(pt.point.x),
                static_cast<float>(pt.point.y)};

            const float sin_heading = rel.normalized().y();
            if (std::abs(sin_heading) >
                std::sin(
                    this->params.auto_localization_align_angular_thresh_deg *
                    (std::numbers::pi_v<float> / 180.f)))
            {
                const float s = sin_heading > 0.f ? 1.f : -1.f;
                const float W =
                    this->params.auto_localization_align_angular_velocity_rps *
                    s;

                commands.setTracksVelocity(
                    lance::groundMpsToTrackMotorRps(
                        lance::bodyDynamicsToLeftTrackVelocityMps(0.f, W)),
                    lance::groundMpsToTrackMotorRps(
                        lance::bodyDynamicsToRightTrackVelocityMps(0.f, W)));

                break;
            }

            this->stage = Stage::ADJUST_RANGE;
            [[fallthrough]];
        }
        case Stage::ADJUST_RANGE:
        {
            // centroid in robot reference frame
            const auto& pt =
                this->refl_hint_interface.getLatestHint()->centroid;

            const float range =
                static_cast<float>(std::hypot(pt.point.x, pt.point.y));
            const float range_error =
                (range - this->params.auto_localization_range_target_m);
            const float abs_range_error = std::abs(range_error);
            if (abs_range_error > this->params.auto_localization_range_thresh_m)
            {
                const float Vl_prev =
                    static_cast<float>(lance::trackMotorRpsToGroundMps(
                        motor_status.track_left.velocity));
                const float Vr_prev =
                    static_cast<float>(lance::trackMotorRpsToGroundMps(
                        motor_status.track_right.velocity));
                const float V_prev =
                    lance::trackVelocitiesToForwardVelocity(Vl_prev, Vr_prev);
                const float Vd_max = (A_max * Dt);
                const float V_target =
                    util::kmx::maxStartVel(0.f, abs_range_error, A_max) *
                    (std::signbit(range_error) ? -1.f : 1.f);
                const float V = std::clamp(
                    V_target,
                    std::max((V_prev - Vd_max), -V_max),
                    std::min((V_prev + Vd_max), V_max));

                commands.setTracksVelocity(
                    lance::groundMpsToTrackMotorRps(V),
                    lance::groundMpsToTrackMotorRps(V));
            }

            break;
        }
        case Stage::FINISHED:
        {
            this->refl_hint_interface.setEnableSrv(false);
        }
    }

#undef V_max
#undef A_max
}

};  // namespace lance
