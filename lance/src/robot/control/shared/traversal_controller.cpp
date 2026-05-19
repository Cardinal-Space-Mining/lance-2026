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

#include "traversal_controller.hpp"

#include <chrono>
#include <memory>
#include <sstream>
#include <iostream>

#include "util/geometry.hpp"
#include "robot/model/dynamics.hpp"
#include "robot/model/geometry.hpp"
#include "robot/model/kinematics.hpp"


using namespace util::geom::cvt::ops;


inline mpc::MPCParams getMpcParams(const lance::RobotParams& rp)
{
    mpc::MPCParams p;
    p.dt = rp.iteration_period_seconds;
    p.feedback_delay_s = rp.iteration_period_seconds * 2;
    p.v_max = rp.auto_traversal_max_track_velocity_mps;
    p.v_min = 0.0;
    p.a_max = rp.auto_traversal_max_track_acceleration_mpss;
    p.omega_max = rp.auto_traversal_max_angular_velocity_rps;
    p.alpha_max = rp.auto_traversal_max_angular_accel_rpss;
    p.d_hard = rp.auto_traversal_max_path_deviation_m + 0.02;
    p.goal_threshold = rp.auto_traversal_destination_thresh_m;
    p.stanley_k = rp.auto_traversal_stanley_k_coeff;
    p.N = std::max(p.minBrakingSteps(), 30);
    return p;
}


namespace lance
{

TraversalController::TraversalController(
    const RobotParams& params,
    SensingInterfaces& sensing_interfaces) :
    params{params},
    tf_cache{sensing_interfaces.tf_cache},
    pplan_interface{sensing_interfaces.path_plan_interface},
    mpc_params{getMpcParams(params)},
    mpc_logger{"mpc_data.jsonl"},
    mpc_controller{mpc_params}
{
    this->mpc_logger.logParams(mpc_params, 0);
}


void TraversalController::initializePoint(const Vec2f& dest, const Vec2f& dir)
{
    this->pplan_interface.clearPath();
    this->arena_dest_direction = dir.normalized();
    this->destination_type = dir.squaredNorm() > 0.f ? DestinationType::POSE
                                                     : DestinationType::POINT;

    if (this->updateNeedsTraversal(dest, KeyFrame::ARENA_FRAME))
    {
        this->pplan_interface.init(
            Vec3f{dest.x(), dest.y(), 0.f},
            this->tf_cache.arena_frame_id);
    }

    this->state = State::INITIALIZATION;
}

void TraversalController::initializePoint(
    const PointStampedMsg& dest,
    const Vec2f& dir)
{
    this->pplan_interface.clearPath();
    this->arena_dest_direction = dir.normalized();
    this->destination_type = dir.squaredNorm() > 0.f ? DestinationType::POSE
                                                     : DestinationType::POINT;

    if (this->updateNeedsTraversal(
            Vec2f{dest.point.x, dest.point.y},
            this->tf_cache.resolveKeyFrame(dest.header.frame_id)))
    {
        this->pplan_interface.init(dest);
    }

    this->state = State::INITIALIZATION;
}

void TraversalController::initializePose(const PoseStampedMsg& dest)
{
    this->pplan_interface.clearPath();

    lance::geom::Quatf q;
    q << dest.pose.orientation;
    const KeyFrame k = this->tf_cache.resolveKeyFrame(dest.header.frame_id);
    if (q.squaredNorm() > 0.5f && k == KeyFrame::ARENA_FRAME)
    {
        // TODO: compute target orientation in whatever frame is passed!
        const float theta = lance::geom::quatToYaw(q);

        this->destination_type = DestinationType::POSE;
        this->arena_dest_direction = Vec2f{std::cos(theta), std::sin(theta)};
    }
    else
    {
        this->destination_type = DestinationType::POINT;
        this->arena_dest_direction = Vec2f::Zero();
    }

    if (this->updateNeedsTraversal(
            Vec2f{dest.pose.position.x, dest.pose.position.y},
            k))
    {
        this->pplan_interface.init(dest);
    }

    this->state = State::INITIALIZATION;
}

void TraversalController::initializePose(const Pose3f& dest, KeyFrame frame_id)
{
    this->pplan_interface.clearPath();

    if (dest.quat.squaredNorm() > 0.5f && frame_id == KeyFrame::ARENA_FRAME)
    {
        // TODO: compute target orientation in whatever frame is passed!
        const float theta = lance::geom::quatToYaw(dest.quat);

        this->destination_type = DestinationType::POSE;
        this->arena_dest_direction = Vec2f{std::cos(theta), std::sin(theta)};
    }
    else
    {
        this->destination_type = DestinationType::POINT;
        this->arena_dest_direction = Vec2f::Zero();
    }

    if (this->updateNeedsTraversal(dest.vec.template head<2>(), frame_id))
    {
        this->pplan_interface.init(
            dest.vec,
            this->tf_cache.getFrameId(frame_id));
    }

    this->state = State::INITIALIZATION;
}

void TraversalController::initializeZone(
    const Vec2f& dest_min,
    const Vec2f& dest_max)
{
    this->pplan_interface.clearPath();
    this->arena_dest_zone.min() = dest_min;
    this->arena_dest_zone.max() = dest_max;
    this->arena_dest_direction = Vec2f::Zero();
    this->destination_type = DestinationType::ZONE;

    if (this->tf_cache.hasTf(ROBOT_TO_ARENA_TF) &&
        this->arena_dest_zone.contains(this->tf_cache.getTf(ROBOT_TO_ARENA_TF)
                                           ->pose.vec.template head<2>()))
    {
        this->state = State::FINISHED;
        return;
    }

    this->pplan_interface.init(
        Vec3f{
            (dest_min.x() + dest_max.x()) * 0.5f,
            (dest_min.y() + dest_max.y()) * 0.5f,
            0.f},
        this->tf_cache.arena_frame_id);

    this->state = State::INITIALIZATION;
}


bool TraversalController::isFinished()
{
    return this->state == State::FINISHED;
}

void TraversalController::setCancelled()
{
    this->pplan_interface.cancel();

    this->state = State::FINISHED;
}

void TraversalController::iterate(
    const RobotMotorStatus& motor_status,
    RobotMotorCommands& commands)
{
    commands.disableAll();

    switch (this->state)
    {
        case State::INITIALIZATION:
        {
            if (motor_status.getHopperActNormalizedValue() <
                this->params.hopper_actuator_traversal_target_val)
            {
                commands.setHopperActSpeed(
                    this->params.hopper_actuator_max_speed);
                break;
            }
            if (this->need_traverse && !this->pplan_interface.hasPath())
            {
                break;
            }

            this->prev_left_velocity =
                static_cast<float>(lance::trackMotorRpsToGroundMps(
                    motor_status.track_left.velocity));
            this->prev_right_velocity =
                static_cast<float>(lance::trackMotorRpsToGroundMps(
                    motor_status.track_right.velocity));

            this->mpc_controller.reset();

            this->state =
                this->need_traverse ? State::FOLLOW_PATH : State::REORIENT;
            [[fallthrough]];
        }
        case State::FOLLOW_PATH:
        {
            if (!this->iterateTraversal(motor_status, commands))
            {
                break;
            }
            this->state = State::REORIENT;
            [[fallthrough]];
        }
        case State::REORIENT:
        {
            if (!this->iterateReorient(motor_status, commands))
            {
                break;
            }
            this->state = State::FINISHED;
            [[fallthrough]];
        }
        case State::FINISHED:
        {
            this->pplan_interface.cancel();
        }
    }
}



bool TraversalController::updateNeedsTraversal(
    const Vec2f& dest,
    KeyFrame frame)
{
    if (this->tf_cache.hasTf(KeyFrame::ROBOT_FRAME, frame) &&
        (this->tf_cache.getTf(KeyFrame::ROBOT_FRAME, frame)
             ->pose.vec.template head<2>() -
         dest)
                .norm() <= this->params.auto_traversal_destination_thresh_m)
    {
        this->need_traverse = false;
    }
    else
    {
        this->need_traverse = true;
    }

    return this->need_traverse;
}


#define K1          (this->params.auto_traversal_stanley_k_coeff)
#define K2          (this->params.auto_traversal_max_path_deviation_m)
#define W_Kp        (this->params.auto_traversal_angular_kp)
#define Dt          (this->params.iteration_period_seconds)
#define V_max       (this->params.auto_traversal_max_track_velocity_mps)
#define A_max       (this->params.auto_traversal_max_track_acceleration_mpss)
#define W_max       (this->params.auto_traversal_max_angular_velocity_rps)
#define Al_max      (this->params.auto_traversal_max_angular_accel_rpss)
#define V_delta_max (A_max * Dt)

bool TraversalController::iterateTraversal(
    const RobotMotorStatus& motor_status,
    RobotMotorCommands& commands)
{
    (void)motor_status;

    const PathMsg* path = this->pplan_interface.getPath();
    if (!path)
    {
        return false;
    }

    // 1. OBTAIN KEYPOINTS RELATIVE TO ODOM LINK -------------------------------
    mpc::Path keypoints;
    keypoints.pts.resize(path->poses.size());

    if (path->header.frame_id != this->params.odom_frame_id)
    {
        const auto* tf =
            this->tf_cache.getTf(path->header.frame_id, ODOM_FRAME);
        if (tf)
        {
            Vec3f tmp;
            for (size_t i = 0; i < keypoints.size(); i++)
            {
                const auto& pt = path->poses[i].pose.position;
                keypoints.pts[i].pos = (tf->tf * (tmp << pt))
                                           .template head<2>()
                                           .template cast<double>();
            }
        }
        else
        {
            return false;
        }
    }
    else
    {
        for (size_t i = 0; i < keypoints.size(); i++)
        {
            const auto& pt = path->poses[i].pose.position;
            keypoints.pts[i].pos.x() = pt.x;
            keypoints.pts[i].pos.y() = pt.y;
        }
    }

    // 2. Get robot state (odom frame) -----------------------------------------
    const auto* tf = this->tf_cache.getTf(ROBOT_TO_ODOM_TF);
    if (!tf)
    {
        return false;
    }

    mpc::State x;
    x.x = tf->pose.vec.x();
    x.y = tf->pose.vec.y();
    x.theta = lance::geom::quatToYaw(tf->pose.quat);

    // 3. Exit if final keypoint has been reached ------------------------------
    switch (this->destination_type)
    {
        case DestinationType::POINT:
        case DestinationType::POSE:
        {
            if ((Eigen::Vector2d{x.x, x.y} - keypoints.pts.back().pos).norm() <=
                this->params.auto_traversal_destination_thresh_m)
            {
                commands.disableTracks();
                return true;
            }
            break;
        }
        case DestinationType::ZONE:
        {
            const auto* tf = this->tf_cache.getTf(ROBOT_TO_ARENA_TF);
            if (tf &&
                this->arena_dest_zone.contains(tf->pose.vec.template head<2>()))
            {
                commands.disableTracks();
                return true;
            }
            break;
        }
    }

    // 4. Run MPC controller and apply output
    const mpc::Control u = this->mpc_controller.update(x, keypoints);

    double Vl_out, Vr_out;
    util::kmx::applyTrackLimits<double>(
        lance::bodyDynamicsToLeftTrackVelocityMps(u.v, u.omega),
        lance::bodyDynamicsToRightTrackVelocityMps(u.v, u.omega),
        V_max,
        Vl_out,
        Vr_out);

    commands.setTracksVelocity(
        lance::groundMpsToTrackMotorRps(Vl_out),
        lance::groundMpsToTrackMotorRps(Vr_out));

    this->prev_left_velocity = static_cast<float>(Vl_out);
    this->prev_right_velocity = static_cast<float>(Vr_out);

    const mpc::DebugInfo& dbg = this->mpc_controller.debugInfo();
    this->mpc_logger.logFrame(
        std::chrono::duration<double>(
            std::chrono::system_clock::now().time_since_epoch())
            .count(),
        x,
        u,
        keypoints,
        dbg);

    return false;
}


bool TraversalController::iterateReorient(
    const RobotMotorStatus& motor_status,
    RobotMotorCommands& commands)
{
    (void)motor_status;

    switch (this->destination_type)
    {
        case DestinationType::POINT:
        case DestinationType::ZONE:
        {
            return true;
        }
        case DestinationType::POSE:
        default:
        {
            break;
        }
    }

    Vec2f target = this->arena_dest_direction;
    const auto* tf = this->tf_cache.getTf(ARENA_TO_ROBOT_TF);
    if (tf)
    {
        target = (tf->pose.quat.toRotationMatrix() *
                  Vec3f{target.x(), target.y(), 0.f})
                     .template head<2>();
    }
    else
    {
        return false;
    }

    // angular error
    const float theta = std::atan2(target.y(), target.x());
    const float theta_abs = std::abs(theta);

    if (theta_abs < this->params.auto_traversal_align_angular_thresh_deg *
                        (std::numbers::pi_v<float> / 180.f))
    {
        return true;
    }

    // angular velocity proportional to error, clamped to parmertarized max
    // const float W = std::clamp((theta * W_Kp), -W_max, W_max);
    const float W =
        std::min(util::kmx::maxStartVel(0.f, theta_abs, Al_max), W_max) *
        (std::signbit(theta) ? -1.f : 1.f);

    const float Vl_target = lance::bodyDynamicsToLeftTrackVelocityMps(0.f, W);
    const float Vr_target = lance::bodyDynamicsToRightTrackVelocityMps(0.f, W);

    float Vl_out, Vr_out;
    util::kmx::applyTrackLimits(Vl_target, Vr_target, V_max, Vl_out, Vr_out);

    commands.setTracksVelocity(
        lance::groundMpsToTrackMotorRps(Vl_out),
        lance::groundMpsToTrackMotorRps(Vr_out));

    return false;
}


void TraversalController::getFilteredPrevVelocities(
    const RobotMotorStatus& motor_status,
    float& Vl_prev,
    float& Vr_prev)
{
    const float Vd_max = V_delta_max;
    const float Vl_prev_fb = static_cast<float>(
        lance::trackMotorRpsToGroundMps(motor_status.track_left.velocity));
    const float Vr_prev_fb = static_cast<float>(
        lance::trackMotorRpsToGroundMps(motor_status.track_right.velocity));

    // Use previous targets if the feedback data has a large deviation or is OOB
    const float Vl_prev_err = std::abs((Vl_prev_fb - this->prev_left_velocity));
    const float Vr_prev_err =
        std::abs((Vr_prev_fb - this->prev_right_velocity));

    Vl_prev = (Vl_prev_err > Vd_max || std::abs(Vl_prev_fb) > V_max)
                  ? this->prev_left_velocity
                  : Vl_prev_fb;
    Vr_prev = (Vr_prev_err > Vd_max || std::abs(Vr_prev_fb) > V_max)
                  ? this->prev_right_velocity
                  : Vr_prev_fb;
}

#undef K1
#undef K2
#undef W_Kp
#undef Dt
#undef V_max
#undef A_max
#undef W_max

};  // namespace lance
