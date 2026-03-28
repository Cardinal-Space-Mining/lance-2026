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

#include <geometry_msgs/msg/pose_stamped.hpp>

#include "util/geometry.hpp"
#include "util/time_cvt.hpp"

#include "robot/core/robot_math.hpp"
#include "robot/core/hid_bindings.hpp"


#define PERCEPTION_PATH_TOPIC "/cardinal_perception/planned_path"
#define PERCEPTION_PPLAN_CONTROL_TOPIC          \
    "/cardinal_perception/update_path_planning"

using system_clock = std::chrono::system_clock;
using namespace util::geom::cvt::ops;

using Iso3f = Eigen::Isometry3f;
using Quatf = Eigen::Quaternionf;


/* Compute the radius-per-max-corner-deviation coefficient for a junction
 * with a bend of theta degrees. The radius in this case is that of the arc
 * that is tangent to both path segments. */
template<typename F>
inline F computeJunctionRadiusCoeff(F cos_theta)
{
    const F cos_half_theta =
        std::sqrt(static_cast<F>(0.5) + cos_theta * static_cast<F>(0.5));
    return cos_half_theta / (static_cast<F>(1) - cos_half_theta);
}


namespace lance
{

TraversalController::TraversalController(
    RclNode& node,
    GenericPubMap& pub_map,
    const RobotParams& params,
    const TfCache& tf_cache) :
    pub_map{pub_map},
    params{params},
    tf_cache{tf_cache},
    path_sub{node.create_subscription<PathMsg>(
        PERCEPTION_PATH_TOPIC,
        rclcpp::SensorDataQoS{},
        [this](const PathMsg::ConstSharedPtr& msg) { this->last_path = msg; })},
    pplan_control_client{
        node.create_client<UpdatePathPlanSrv>(PERCEPTION_PPLAN_CONTROL_TOPIC)}
{
}

void TraversalController::initializePoint(const Vec2f& dest, const Vec2f& dir)
{
    this->last_path = nullptr;
    this->arena_dest_direction = dir.normalized();
    this->destination_type = dir.squaredNorm() > 0.f ? DestinationType::POSE
                                                     : DestinationType::POINT;

    this->initPlanningService(Vec3f{dest.x(), dest.y(), 0.f});

    this->state = State::INITIALIZATION;
}
void TraversalController::initializePoint(
    const PointStampedMsg& dest,
    const Vec2f& dir)
{
    this->last_path = nullptr;
    this->arena_dest_direction = dir.normalized();
    this->destination_type = dir.squaredNorm() > 0.f ? DestinationType::POSE
                                                     : DestinationType::POINT;
    this->initPlanningService(dest);

    this->state = State::INITIALIZATION;
}
void TraversalController::initializeZone(
    const Vec2f& dest_min,
    const Vec2f& dest_max)
{
    this->last_path = nullptr;
    this->arena_dest_zone.min() = dest_min;
    this->arena_dest_zone.max() = dest_max;
    this->arena_dest_direction = Vec2f::Zero();
    this->destination_type = DestinationType::ZONE;

    this->initPlanningService(
        Vec3f{
            (dest_min.x() + dest_max.x()) * 0.5f,
            (dest_min.y() + dest_max.y()) * 0.5f,
            0.f});

    this->state = State::INITIALIZATION;
}

bool TraversalController::isFinished()
{
    return this->state == State::FINISHED;
}

void TraversalController::setCancelled()
{
    this->stopPlanningService();

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
                this->params.hopper_actuator_traversal_target)
            {
                commands.setHopperActPercent(
                    this->params.hopper_actuator_max_speed);
                break;
            }
            if (!this->last_path)
            {
                break;
            }

            this->prev_left_velocity =
                static_cast<float>(lance::trackMotorRpsToGroundMps(
                    motor_status.track_left.velocity));
            this->prev_right_velocity =
                static_cast<float>(lance::trackMotorRpsToGroundMps(
                    motor_status.track_right.velocity));
            this->state = State::FOLLOW_PATH;
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
            this->stopPlanningService();
        }
    }
}

void TraversalController::initPlanningService(const Vec3f& arena_dest)
{
    auto req = std::make_shared<UpdatePathPlanSrv::Request>();
    req->target.header.frame_id = this->params.arena_frame_id;
    req->target.header.stamp = util::toTimeMsg(system_clock::now());
    req->target.pose.position.x = arena_dest.x();
    req->target.pose.position.y = arena_dest.y();
    req->target.pose.position.z = arena_dest.z();
    req->completed = false;

    this->pplan_control_client->async_send_request(
        req,
        [](rclcpp::Client<UpdatePathPlanSrv>::SharedFuture) {});
}
void TraversalController::initPlanningService(const PointStampedMsg& dest)
{
    auto req = std::make_shared<UpdatePathPlanSrv::Request>();
    req->target.header = dest.header;
    req->target.pose.position = dest.point;
    req->completed = false;

    this->pplan_control_client->async_send_request(
        req,
        [](rclcpp::Client<UpdatePathPlanSrv>::SharedFuture) {});
}
void TraversalController::stopPlanningService()
{
    auto req = std::make_shared<UpdatePathPlanSrv::Request>();
    req->completed = true;

    this->pplan_control_client->async_send_request(
        req,
        [](rclcpp::Client<UpdatePathPlanSrv>::SharedFuture) {});
}



#define K1          (this->params.auto_traversal_stanley_k_coeff)
#define K2          (this->params.auto_traversal_max_path_deviation_m)
#define W_Kp        (this->params.auto_traversal_angular_kp)
#define Dt          (this->params.iteration_period_seconds)
#define V_max       (this->params.auto_traversal_max_track_velocity_mps)
#define A_max       (this->params.auto_traversal_max_track_acceleration_mpss)
#define W_max       (this->params.auto_traversal_max_angular_velocity_rps)
#define V_delta_max (A_max * Dt)

bool TraversalController::iterateTraversal(
    const RobotMotorStatus& motor_status,
    RobotMotorCommands& commands)
{
    // 1. OBTAIN KEYPOINTS RELATIVE TO BASE LINK -------------------------------
    std::vector<Vec2f> keypoints;
    keypoints.resize(this->last_path->poses.size());

    if (this->last_path->header.frame_id != this->params.robot_frame_id)
    {
        const auto* tf =
            this->tf_cache.getTf(this->last_path->header.frame_id, ROBOT_FRAME);
        if (tf)
        {
            Vec3f tmp;
            for (size_t i = 0; i < keypoints.size(); i++)
            {
                const auto& pt = this->last_path->poses[i].pose.position;
                keypoints[i] = (tf->tf * (tmp << pt)).template head<2>();
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
            const auto& pt = this->last_path->poses[i].pose.position;
            keypoints[i].x() = static_cast<float>(pt.x);
            keypoints[i].y() = static_cast<float>(pt.y);
        }
    }

    // 2. Exit if final keypoint has been reached ------------------------------
    switch (this->destination_type)
    {
        case DestinationType::POINT:
        case DestinationType::POSE:
        {
            if (keypoints.back().norm() <=
                this->params.auto_traversal_keypoint_thresh_m)
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

    // 3. FIND TARGET SEGMENT OR KEYPOINT --------------------------------------
    size_t seg_beg_idx = 0;
    size_t seg_end_idx = 0;
    float seg_proj_t = 0.f;
    for (size_t i = 1; i < keypoints.size(); i++)
    {
        const auto& prev = keypoints[i - 1];
        const auto& curr = keypoints[i];

        // project the robot base onto the segment formed by the current
        // two keypoints
        Vec2f diff = curr - prev;
        seg_proj_t = (diff.dot(-prev)) / diff.squaredNorm();

        // proj_t > 1.f --> "after" second keypoint
        // proj_t = 1.f --> at second keypoint
        // proj_t = 0.f --> at first keypoint
        // proj_t < 0.f --> "before" first keypoint
        if (seg_proj_t < 1.f)  // "before" second keypoint
        {
            seg_end_idx = i;
            seg_beg_idx = i - 1;
            break;
        }
        else
        {
            seg_beg_idx = seg_end_idx = i;
        }
    }

    // 4. ALGO -----------------------------------------------------------------
    this->runStanley(
        motor_status,
        keypoints,
        seg_beg_idx,
        seg_end_idx,
        seg_proj_t,
        commands);

    return false;
}


bool TraversalController::iterateReorient(
    const RobotMotorStatus& motor_status,
    RobotMotorCommands& commands)
{
    (void)motor_status;

    constexpr float TARGETTING_THETA_EPSILON =
        0.5f * (std::numbers::pi_v<float> / 180.f);
    constexpr float MAX_ANGULAR_DECELL = 0.5f;

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

    if (theta_abs < TARGETTING_THETA_EPSILON)
    {
        return true;
    }

    // angular velocity proportional to error, clamped to parmertarized max
    // const float W = std::clamp((theta * W_Kp), -W_max, W_max);
    const float W = std::min(kmx::maxStartVel(0.f, theta_abs, MAX_ANGULAR_DECELL), W_max) *
                    (std::signbit(theta) ? -1.f : 1.f);

    const float Vl_target = lance::bodyDynamicsToLeftTrackVelocityMps(0.f, W);
    const float Vr_target = lance::bodyDynamicsToRightTrackVelocityMps(0.f, W);

    float Vl_out, Vr_out;
    kmx::applyTrackLimits(Vl_target, Vr_target, V_max, Vl_out, Vr_out);

    commands.setTracksVelocity(
        lance::groundMpsToTrackMotorRps(Vl_out),
        lance::groundMpsToTrackMotorRps(Vr_out));

    return false;
}



void TraversalController::runStanley(
    const RobotMotorStatus& motor_status,
    const std::vector<Vec2f>& keypoints,
    size_t seg_beg_idx,
    size_t seg_end_idx,
    float seg_proj_t,
    RobotMotorCommands& commands)
{
    // 1. Aliases --------------------------------------------------------------
    const float theta_V = this->params.auto_traversal_min_theta_window *
                          (std::numbers::pi_v<float> / 180.f);
    const float KR_inv = -std::log(0.5f) / K2;
    const float Vd_max = V_delta_max;

    // 2. Current state from sensors -------------------------------------------
    float Vl_prev, Vr_prev;
    this->getFilteredPrevVelocities(motor_status, Vl_prev, Vr_prev);

    const float V_prev =
        lance::trackVelocitiesToForwardVelocity(Vl_prev, Vr_prev);
    // const float W_prev =
    //     lance::trackVelocitiesToAngularVelocity(Vl_prev, Vr_prev);

    // 3. Get track targets ----------------------------------------------------
    float Vl_target = 0.f, Vr_target = 0.f;
    // a. Target last keypoint ---
    if (seg_beg_idx == seg_end_idx)
    {
        const Vec2f& S = keypoints[seg_end_idx];
        // angular error
        const float theta = std::atan2(S.y(), S.x());
        // angular velocity proportional to error, clamped to parmertarized max
        const float W = std::clamp(W_Kp * theta, -W_max, W_max);
        // accelerate to full velocity if roughly pointed straight at target
        const float Va = (std::fabs(theta) < theta_V) ? V_max : 0.f;
        // TODO: backpropegation max velocity
        const float Vb = kmx::maxStartVel(0.f, S.norm(), A_max);
        // apply minimum of two velocities, and clamp initially to min/max
        // acceleration diffs
        const float Vc =
            std::clamp(std::min(Va, Vb), V_prev - Vd_max, V_prev + Vd_max);
        // differential drive kinematics to get left and right velocities
        Vl_target = lance::bodyDynamicsToLeftTrackVelocityMps(Vc, W);
        Vr_target = lance::bodyDynamicsToRightTrackVelocityMps(Vc, W);
    }
    // b. Target current segment ---
    else
    {
        // calculate junction backpropegated velocity limit
        float Vb = std::numeric_limits<float>::infinity();
        {
            const float max_target_vel = std::min(V_max, V_prev + Vd_max);
            const float max_decell_dist =
                kmx::decellDist(max_target_vel, A_max);
            Vec2f pt_a = Vec2f::Zero();
            float sum_dist = 0.f;
            for (size_t i = (seg_proj_t < 0.f ? seg_beg_idx : seg_end_idx);
                 i < keypoints.size() && sum_dist < max_decell_dist;
                 i++)
            {
                const Vec2f& pt_b = keypoints[i];
                const Vec2f seg_a = (pt_b - pt_a);
                sum_dist += seg_a.norm();

                if (sum_dist < max_decell_dist)
                {
                    if (i + 1 < keypoints.size())
                    {
                        const Vec2f& pt_c = keypoints[i + 1];
                        const Vec2f seg_b = (pt_c - pt_b);

                        const float cos_theta =
                            seg_a.normalized().dot(seg_b.normalized());
                        // s is the ratio of radius to juction deviation
                        const float s = ::computeJunctionRadiusCoeff(cos_theta);
                        // multiply s by max deviation to get max radius
                        const float r = (s * K2);
                        // radius times max angular velocity equals max
                        // tangential velocity - use this to backprop
                        // what our current velocity limit should be such
                        // that our acceleration limit is respected
                        Vb = std::min(
                            Vb,
                            kmx::maxStartVel((r * W_max), sum_dist, A_max));
                    }
                    else
                    {
                        Vb = std::min(
                            Vb,
                            kmx::maxStartVel(0.f, sum_dist, A_max));
                    }
                }
            }
        }

        // stanley >>>
        // REMEMBER: EVERYTHING RELATIVE TO ROBOT FRAME (P is <0, 0>, X+ is <1, 0>)!!

        // segment start point
        const Vec2f& S1 = keypoints[seg_beg_idx];
        // segment end point
        const Vec2f& S2 = keypoints[seg_end_idx];
        // segment raw difference
        const Vec2f Sd = S2 - S1;
        // time along segment (+extrapolation) of closest point to origin
        const float t = seg_proj_t;
        // closest time clamped to segment range
        const float u = std::clamp(t, 0.f, 1.f);
        // closest point
        const Vec2f M = S1 + (Sd * t);
        // closest point actually on segment
        const Vec2f L = S1 + (Sd * u);
        // heading error from direction to closest point on segment
        const float theta_L = std::atan2(L.y(), L.x());
        // stanley validity angular range as a function of distance form segment
        const float theta_R =
            std::numbers::pi_v<float> * std::exp(-L.norm() * KR_inv);
        // full forward velocity if within angular range, otherwise zero
        const float Va = std::min(
            V_max,
            V_max * std::max(theta_R, theta_V) / std::fabs(theta_L));
        // (std::fabs(theta_L) < std::max(theta_R, theta_V)) ? V_max : 0.f;
        // clamp to backpropegation max vel
        const float Vc = std::min(Va, Vb);
        // clamp forward velocity by max acceleration
        const float Vd = std::clamp(Vc, V_prev - Vd_max, V_prev + Vd_max);
        // angular error from the segment forward direction
        const float theta_S = std::atan2(Sd.y(), Sd.x());
        // stanley crosstrack error angular coeff (angular velocity)
        const float theta_E =
            std::atan(K1 * M.norm() / Vd) * (std::signbit(M.y()) ? -1.f : 1.f);
        // stanley controller output angular velocity
        const float Wa = (theta_S + theta_E);
        // proportional controller output angular velocity
        // const float Wb = (W_Kp * theta_L);
        // use stanley if eq pt is within window, otherwise proportional control
        // const float Wc = (std::fabs(theta_E) < theta_R) ? Wa : Wb;
        // ensure target angular velocity is not larger than max
        const float Wd =
            std::clamp(Wa, -W_max, W_max);  // IGNORE PROPORTIONAL CONTROLLER
        // apply kinematics
        Vl_target = lance::bodyDynamicsToLeftTrackVelocityMps(Vd, Wd);
        Vr_target = lance::bodyDynamicsToRightTrackVelocityMps(Vd, Wd);
    }

    // 4. Apply per-track V, A limits ------------------------------------------
    float Vl_out, Vr_out;
    kmx::applyTrackLimits(Vl_target, Vr_target, V_max, Vl_out, Vr_out);

    // 5. Apply ----------------------------------------------------------------
    commands.setTracksVelocity(
        lance::groundMpsToTrackMotorRps(Vl_out),
        lance::groundMpsToTrackMotorRps(Vr_out));

    this->prev_left_velocity = Vl_out;
    this->prev_right_velocity = Vr_out;
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
