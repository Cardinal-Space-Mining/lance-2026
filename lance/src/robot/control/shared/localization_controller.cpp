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
#include <iostream>

#include <Eigen/Core>

#include "robot/model/dynamics.hpp"
#include "robot/model/geometry.hpp"
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

    this->spin_odom_rad = 0.f;
    this->spin_tracks_rad = 0.f;
    this->spin_accum_rad = 0.f;
    this->have_last_yaw = false;
    this->spin_logged = false;
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

    // all rotation after the hopper is positioned counts toward the 360
    if (this->stage != Stage::INITIALIZATION && this->stage != Stage::FINISHED)
    {
        this->updateSpin(motor_status);
    }

    // once the full localization transform is established, finish whatever
    // is left of the 360 deg spin (if anything) and then the command is done
    if (this->tf_cache.hasTf(ROBOT_TO_ARENA_TF) &&
        this->stage != Stage::FINISHED)
    {
        this->stage =
            this->spinComplete() ? Stage::FINISHED : Stage::COMPLETE_SPIN;
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
                commands.setHopperActSpeed(
                    this->params.hopper_actuator_max_speed);
                break;
            }
            // go down if above
            if (motor_status.getHopperActNormalizedValue() -
                    this->params.hopper_actuator_traversal_target_val >
                this->params.hopper_actuator_targetting_thresh)
            {
                commands.setHopperActSpeed(
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
        case Stage::COMPLETE_SPIN:
        {
            if (!this->spinComplete())
            {
                // continue in the same direction as the search spin
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

            this->stage = Stage::FINISHED;
            [[fallthrough]];
        }
        case Stage::FINISHED:
        {
            this->refl_hint_interface.setEnableSrv(false);
        }
    }

#undef V_max
#undef A_max
}

void LocalizationController::updateSpin(const RobotMotorStatus& motor_status)
{
    // Odom heading (available before arena localization)
    if (this->tf_cache.hasTf(ROBOT_TO_ODOM_TF))
    {
        const float yaw = lance::geom::quatToYaw(
            this->tf_cache.getTf(ROBOT_TO_ODOM_TF)->pose.quat);
        if (this->have_last_yaw)
        {
            const float d = yaw - this->last_yaw;
            this->spin_odom_rad +=
                std::abs(std::atan2(std::sin(d), std::cos(d)));
        }
        this->last_yaw = yaw;
        this->have_last_yaw = true;
    }
    else
    {
        this->have_last_yaw = false;
    }

    // Integrated measured track angular velocity
    const float Vl = static_cast<float>(
        lance::trackMotorRpsToGroundMps(motor_status.track_left.velocity));
    const float Vr = static_cast<float>(
        lance::trackMotorRpsToGroundMps(motor_status.track_right.velocity));
    this->spin_tracks_rad +=
        std::abs(lance::trackVelocitiesToAngularVelocity(Vl, Vr)) *
        this->params.iteration_period_seconds;

    this->spin_accum_rad = std::max(this->spin_odom_rad, this->spin_tracks_rad);

    if (this->spinComplete() && !this->spin_logged &&
        this->params.auto_localization_require_full_rotation)
    {
        constexpr float RAD_TO_DEG = 180.f / std::numbers::pi_v<float>;
        const bool localized = this->tf_cache.hasTf(ROBOT_TO_ARENA_TF);
        std::cout << "[LocalizationController] Full 360 spin complete (odom "
                  << (this->spin_odom_rad * RAD_TO_DEG) << " deg, tracks "
                  << (this->spin_tracks_rad * RAD_TO_DEG) << " deg) - "
                  << (localized ? "localized, finishing"
                                : "not localized yet, continuing to the "
                                  "reflector without extra spinning")
                  << "\n";
        this->spin_logged = true;
    }
}

bool LocalizationController::spinComplete() const
{
    return !this->params.auto_localization_require_full_rotation ||
           this->spin_accum_rad >= 2.f * std::numbers::pi_v<float>;
}

};  // namespace lance
