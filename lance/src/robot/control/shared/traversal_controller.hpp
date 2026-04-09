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

#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "util/ros_utils.hpp"
#include "robot/core/robot_params.hpp"
#include "robot/core/motor_interface.hpp"
#include "robot/sensing/sensing_interfaces.hpp"


namespace lance
{

class TraversalController
{
    friend class TelemetrySerializer;
    friend class TelemetryDeserializer;

    using PathMsg = PathPlanInterface::PathMsg;
    using PoseStampedMsg = PathPlanInterface::PoseStampedMsg;
    using PointStampedMsg = PathPlanInterface::PointStampedMsg;

    using Vec2f = Eigen::Vector2f;
    using Vec3f = Eigen::Vector3f;
    using Box2f = Eigen::AlignedBox2f;

public:
    TraversalController(const RobotParams&, SensingInterfaces&);
    ~TraversalController() = default;

public:
    void initializePoint(
        const Vec2f& dest,
        const Vec2f& dest_direction = Vec2f::Zero());
    void initializePoint(
        const PointStampedMsg& dest,
        const Vec2f& dest_direction = Vec2f::Zero());
    void initializePose(const PoseStampedMsg& dest);
    void initializeZone(const Vec2f& dest_min, const Vec2f& dest_max);

    bool isFinished();
    void setCancelled();

    void iterate(
        const RobotMotorStatus& motor_status,
        RobotMotorCommands& commands);

protected:
    enum class State
    {
        INITIALIZATION,
        FOLLOW_PATH,
        REORIENT,
        FINISHED
    };
    enum class DestinationType
    {
        POINT,
        POSE,
        ZONE
    };

protected:
    bool iterateTraversal(
        const RobotMotorStatus& motor_status,
        RobotMotorCommands& commands);
    bool iterateReorient(
        const RobotMotorStatus& motor_status,
        RobotMotorCommands& commands);

    void runStanley(
        const RobotMotorStatus& motor_status,
        const std::vector<Vec2f>& keypoints,
        size_t seg_beg_idx,
        size_t seg_end_idx,
        float seg_proj_t,
        RobotMotorCommands& commands);

    void getFilteredPrevVelocities(
        const RobotMotorStatus& motor_status,
        float& Vl_prev,
        float& Vr_prev);

protected:
    const RobotParams& params;
    const TfCache& tf_cache;
    PathPlanInterface& pplan_interface;

    State state{State::FINISHED};

    Box2f arena_dest_zone{};
    Vec2f arena_dest_direction{};
    DestinationType destination_type{DestinationType::POINT};

    float prev_left_velocity{0.f};
    float prev_right_velocity{0.f};
};

};  // namespace lance
