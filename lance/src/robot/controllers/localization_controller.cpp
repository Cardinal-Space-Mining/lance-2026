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
// #include <iostream>

#include <Eigen/Core>

#include "../robot_math.hpp"


#define PERCEPTION_REFLECTOR_HINT_TOPIC "/cardinal_perception/reflector_hint"
#define PERCEPTION_LFD_CONTROL_TOPIC    "/cardinal_perception/set_global_alignment"

using Vec2f = Eigen::Vector2f;


namespace lance
{

LocalizationController::LocalizationController(
    RclNode& node,
    GenericPubMap& pub_map,
    const RobotParams& params,
    const Tf2Buffer& tf_buffer) :
    pub_map{pub_map},
    params{params},
    tf_buffer{tf_buffer},
    hint_sub{node.create_subscription<ReflectorHintMsg>(
        PERCEPTION_REFLECTOR_HINT_TOPIC,
        rclcpp::SensorDataQoS{},
        [this](const ReflectorHintMsg::ConstSharedPtr& msg)
        { this->last_hint = msg; })},
    lfd_control_client{
        node.create_client<SetBoolSrv>(PERCEPTION_LFD_CONTROL_TOPIC)}
{
}

void LocalizationController::initialize()
{
    this->stage = Stage::INITIALIZATION;
    this->last_hint = nullptr;
}

bool LocalizationController::isFinished()
{
    return this->stage == Stage::FINISHED;
}

void LocalizationController::setCancelled()
{
    this->stage = Stage::FINISHED;
    this->setLfdControl(false);
}

void LocalizationController::iterate(
    const RobotMotorStatus& motor_status,
    RobotMotorCommands& commands)
{
#define Dt    (this->params.iteration_period_seconds)
#define V_max (this->params.auto_traversal_max_track_velocity_mps)
#define A_max (this->params.auto_traversal_max_track_acceleration_mpss)
    // #define W_max (this->params.auto_traversal_max_angular_velocity_rps)

    // TODO: parameters
    constexpr uint32_t MIN_SEARCH_SAMPLES = 100U;
    constexpr double SEARCHING_ANGULAR_VEL = 0.5;
    constexpr double ALIGNING_ANGULAR_VEL = 0.25;
    constexpr float ALIGNMENT_ANGULAR_THRESH =
        2.f * (std::numbers::pi_v<float> / 180.f);
    constexpr float RANGE_TARGET = 1.05f;
    constexpr float RANGE_THRESH = 0.05f;

    // if at any point the full localization transform is established,
    // the command is finished
    // TODO: TF lookup timestamps are wildly inconsistent when using gazebo -
    //      the workaround for now is to just query the alignment tf and not the full tf,
    //      although we should really be checking to make sure we have full localization
    if (this->tf_buffer.canTransform(
            this->params.odom_frame_id,
            this->params.arena_frame_id,
            tf2::TimePointZero))
    {
        this->stage = Stage::FINISHED;
    }

    commands.disableAll();

    switch (this->stage)
    {
        case Stage::INITIALIZATION:
        {
            if (motor_status.getHopperActNormalizedValue() <
                this->params.hopper_actuator_traversal_target)
            {
                commands.setHopperActPercent(
                    this->params.hopper_actuator_max_speed);
                break;
            }

            this->setLfdControl(true);
            this->stage = Stage::SEARCHING;
            [[fallthrough]];
        }
        case Stage::SEARCHING:
        {
            if (!this->last_hint ||
                this->last_hint->samples < MIN_SEARCH_SAMPLES)
            {
                commands.setTracksVelocity(
                    lance::groundMpsToTrackMotorRps(
                        lance::bodyDynamicsToLeftTrackVelocityMps(
                            0.,
                            SEARCHING_ANGULAR_VEL)),
                    lance::groundMpsToTrackMotorRps(
                        lance::bodyDynamicsToRightTrackVelocityMps(
                            0.,
                            SEARCHING_ANGULAR_VEL)));
                break;
            }

            this->stage = Stage::ALIGN_HEADING;
            [[fallthrough]];
        }
        case Stage::ALIGN_HEADING:
        {
            // centroid in robot reference frame
            const auto& pt = this->last_hint->centroid;
            const Vec2f rel{
                static_cast<float>(pt.point.x),
                static_cast<float>(pt.point.y)};

            const float sin_heading = rel.normalized().y();
            if (std::abs(sin_heading) > std::sin(ALIGNMENT_ANGULAR_THRESH))
            {
                const float s = sin_heading > 0.f ? 1.f : -1.f;

                commands.setTracksVelocity(
                    lance::groundMpsToTrackMotorRps(
                        lance::bodyDynamicsToLeftTrackVelocityMps(
                            0.,
                            ALIGNING_ANGULAR_VEL * s)),
                    lance::groundMpsToTrackMotorRps(
                        lance::bodyDynamicsToRightTrackVelocityMps(
                            0.,
                            ALIGNING_ANGULAR_VEL * s)));

                break;
            }

            this->stage = Stage::ADJUST_RANGE;
            [[fallthrough]];
        }
        case Stage::ADJUST_RANGE:
        {
            // centroid in robot reference frame
            const auto& pt = this->last_hint->centroid;

            const float range =
                static_cast<float>(std::hypot(pt.point.x, pt.point.y));
            const float range_error = (range - RANGE_TARGET);
            const float abs_range_error = std::abs(range_error);
            if (abs_range_error > RANGE_THRESH)
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
                    kmx::maxStartVel(0.f, abs_range_error, A_max) *
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
            this->setLfdControl(false);
        }
    }

#undef V_max
#undef A_max
}

void LocalizationController::setLfdControl(bool enabled)
{
    auto req = std::make_shared<SetBoolSrv::Request>();
    req->data = enabled;
    this->lfd_control_client->async_send_request(
        req,
        [](rclcpp::Client<SetBoolSrv>::SharedFuture) {});
}

};  // namespace lance
