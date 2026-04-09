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

#include "inputs.hpp"

#include <chrono>

#include "util/pub_map.hpp"
#include "util/geometry.hpp"

#include "robot/core/hid_bindings.hpp"
#include "robot/core/robot_status.hpp"
#include "robot/core/ros_interface.hpp"


using namespace std::chrono_literals;

using namespace util;
using namespace util::geom::cvt::ops;
using namespace lance::geom;


namespace lance
{

InputInterface::InputInterface(
    RclNode& node,
    const TfCache& tf_cache,
    TelemetryDeserializer& telemetry,
    WatchDog& watchdog,
    const ZoneBounds& bounds) :
    tf_cache{tf_cache},
    telemetry{telemetry},
    watchdog{watchdog},
    bounds{bounds},

    joy_pub{node.create_publisher<JoyMsg>(
        lance::JOY_CTRL_TOPIC,
        rclcpp::SensorDataQoS{})},
    joy_sub{node.create_subscription<JoyMsg>(
        lance::JOY_INPUT_TOPIC,
        rclcpp::SensorDataQoS{},
        [this](const JoyMsg::ConstSharedPtr& msg) { this->handleJoy(*msg); })},

    traversal_target_pub{node.create_publisher<PoseStampedMsg>(
        lance::TRAVERSAL_TARGET_TOPIC,
        rclcpp::SensorDataQoS{})},
    clicked_point_sub{node.create_subscription<PointStampedMsg>(
        lance::CLICKED_POINT_TOPIC,
        rclcpp::SensorDataQoS{},
        [this](const PointStampedMsg::ConstSharedPtr& msg)
        { this->handleClickedPoint(*msg); })},

    interface_pub_timer{
        node.create_wall_timer(50ms, [this]() { this->handleInterfacePubs(); })}
{
}

void InputInterface::handleJoy(const JoyMsg& msg)
{
    this->joy_state.update(msg);

    switch (this->state)
    {
        case State::PASSTHROUGH:
        {
            this->handlePassthroughState();
            break;
        }
        case State::OVERRIDE:
        {
            this->handleOverrideState();
            break;
        }
        case State::TRAV_CURSOR:
        {
            this->handleTravCursorState();
            break;
        }
        case State::MINING_CURSOR:
        {
            this->handleMiningCursorState();
            break;
        }
        case State::OFFLOAD_CURSOR:
        {
            this->handleOffloadCursorState();
            break;
        }
    }

    if (this->state == State::PASSTHROUGH &&
        this->watchdog.isCtrl(ControlMode::TELEOPERATED))
    {
        this->joy_pub->publish(msg);
    }
}

void InputInterface::handleClickedPoint(const PointStampedMsg& msg)
{
    if (this->watchdog.isCtrl(ControlMode::TELEOPERATED))
    {
        return;
    }
    if (this->isCursorState())
    {
        this->state = State::OVERRIDE;
    }

    this->traversal_cursor.header = msg.header;
    this->traversal_cursor.pose.position = msg.point;
    this->traversal_cursor.pose.orientation.w = 0.0;
    this->traversal_cursor.pose.orientation.x = 0.0;
    this->traversal_cursor.pose.orientation.y = 0.0;
    this->traversal_cursor.pose.orientation.z = 0.0;

    this->traversal_target_pub->publish(this->traversal_cursor);
}


void InputInterface::handleInterfacePubs()
{
    GenericPubMap& pub_map = this->telemetry.getPubMap();

    pub_map.publish<Int32Msg>(
        ROBOT_TOPIC("mc_state"),
        static_cast<uint32_t>(this->state));

    if (this->isCursorState())
    {
        pub_map.publish(
            ROBOT_TOPIC("traversal_cursor"),
            this->traversal_cursor);
    }
    if (this->state == State::MINING_CURSOR)
    {
        // TODO
    }
    if (this->state == State::OFFLOAD_CURSOR)
    {
        // TODO
    }
}


void InputInterface::handlePassthroughState()
{
    if (MissionControlOverrideButton::wasPressed(this->joy_state))
    {
        this->state = State::OVERRIDE;
        return;
    }

    if (this->watchdog.isCtrl(ControlMode::AUTONOMOUS) &&
        DisableAllActionsButton::wasPressed(this->joy_state))
    {
        this->watchdog.setCtrl(ControlMode::DISABLED);
    }
}

void InputInterface::handleOverrideState()
{
    if (this->handleCommonOverrides())
    {
        return;
    }

    if (this->watchdog.isCtrl(ControlMode::TELEOPERATED))
    {
        if (SetTravCursorButton::wasPressed(this->joy_state))
        {
            this->initTravCursorState();
            return;
        }
        if (SetMiningCursorButton::wasPressed(this->joy_state))
        {
            this->initMiningCursorState();
            return;
        }
        if (SetOffloadCursorButton::wasPressed(this->joy_state))
        {
            this->initOffloadCursorState();
            return;
        }
    }
}


void InputInterface::initTravCursorState()
{
    this->state = State::TRAV_CURSOR;
    this->homeTravCursor();
}

void InputInterface::initMiningCursorState()
{
    this->state = State::MINING_CURSOR;
    // init cursor to offload zone
}

void InputInterface::initOffloadCursorState()
{
    this->state = State::OFFLOAD_CURSOR;
    this->homeOffloadCursor();
}


void InputInterface::handleTravCursorState()
{
    if (this->handleCommonOverrides() ||
        !this->watchdog.isCtrl(ControlMode::TELEOPERATED))
    {
        this->state = State::OVERRIDE;
        return;
    }

    if (SetTravCursorButton::wasPressed(this->joy_state))
    {
        this->state = State::OVERRIDE;
        return;
    }

    if (ConfirmCursorTargetButton::wasPressed(this->joy_state))
    {
        this->publishTravTarget();
        this->state = State::OVERRIDE;
        return;
    }

    if (SetMiningCursorButton::wasPressed(this->joy_state))
    {
        this->initMiningCursorState();
        return;
    }

    if (SetOffloadCursorButton::wasPressed(this->joy_state))
    {
        this->initOffloadCursorState();
        return;
    }

    this->iterateTravCursor();
}

void InputInterface::handleMiningCursorState()
{
    if (this->handleCommonOverrides() ||
        !this->watchdog.isCtrl(ControlMode::TELEOPERATED))
    {
        this->state = State::OVERRIDE;
        return;
    }

    if (SetMiningCursorButton::wasPressed(this->joy_state))
    {
        this->state = State::OVERRIDE;
        return;
    }

    if (ConfirmCursorTargetButton::wasPressed(this->joy_state))
    {
        this->publishMiningTarget();
        this->state = State::OVERRIDE;
        return;
    }

    if (SetTravCursorButton::wasPressed(this->joy_state))
    {
        this->initTravCursorState();
        return;
    }

    if (SetOffloadCursorButton::wasPressed(this->joy_state))
    {
        this->initOffloadCursorState();
        return;
    }

    this->iterateMiningCursor();
}

void InputInterface::handleOffloadCursorState()
{
    if (this->handleCommonOverrides() ||
        !this->watchdog.isCtrl(ControlMode::TELEOPERATED))
    {
        this->state = State::OVERRIDE;
        return;
    }

    if (SetOffloadCursorButton::wasPressed(this->joy_state))
    {
        this->state = State::OVERRIDE;
        return;
    }

    if (ConfirmCursorTargetButton::wasPressed(this->joy_state))
    {
        this->publishOffloadTarget();
        this->state = State::OVERRIDE;
        return;
    }

    if (SetTravCursorButton::wasPressed(this->joy_state))
    {
        this->initTravCursorState();
        return;
    }

    if (SetMiningCursorButton::wasPressed(this->joy_state))
    {
        this->initMiningCursorState();
        return;
    }

    this->iterateOffloadCursor();
}


bool InputInterface::handleCommonOverrides()
{
    if (MissionControlOverrideButton::wasPressed(this->joy_state))
    {
        this->state = State::PASSTHROUGH;
        return true;
    }

    if (DisableAllActionsButton::wasPressed(this->joy_state))
    {
        this->watchdog.setCtrl(ControlMode::DISABLED);
    }
    if (SetTeleopModeButton::wasPressed(this->joy_state))
    {
        this->watchdog.setCtrl(ControlMode::TELEOPERATED);
    }
    if (SetAutoModeButton::wasPressed(this->joy_state))
    {
        this->watchdog.setCtrl(ControlMode::AUTONOMOUS);
    }
    if (ToggleTestModeButton::wasPressed(this->joy_state))
    {
        this->watchdog.toggleOpt(static_cast<uint8_t>(ControlOpts::TEST_MODE));
    }

    return false;
}

void InputInterface::homeTravCursor()
{
    if (this->tf_cache.hasTf(ROBOT_TO_ARENA_TF))
    {
        const auto* tf = this->tf_cache.getTf(ROBOT_TO_ARENA_TF);
        this->traversal_cursor.pose.position << tf->pose.vec;
        this->traversal_cursor.pose.orientation
            << lance::geom::flattenToYaw(tf->pose.quat);
        this->traversal_cursor.header.frame_id = this->tf_cache.arena_frame_id;
    }
    else if (this->tf_cache.hasTf(ROBOT_TO_ODOM_TF))
    {
        const auto* tf = this->tf_cache.getTf(ROBOT_TO_ODOM_TF);
        this->traversal_cursor.pose.position << tf->pose.vec;
        this->traversal_cursor.pose.orientation
            << lance::geom::flattenToYaw(tf->pose.quat);
        this->traversal_cursor.header.frame_id = this->tf_cache.odom_frame_id;
    }
    else
    {
        this->traversal_cursor.pose.position.x = 0.;
        this->traversal_cursor.pose.position.y = 0.;
        this->traversal_cursor.pose.position.z = 0.;
        this->traversal_cursor.pose.orientation.w = 1.;
        this->traversal_cursor.pose.orientation.x = 0.;
        this->traversal_cursor.pose.orientation.y = 0.;
        this->traversal_cursor.pose.orientation.z = 0.;
        this->traversal_cursor.header.frame_id = this->tf_cache.robot_frame_id;
    }
}

void InputInterface::homeOffloadCursor()
{
    this->offload_zone_norm =
        innerZoneNormalDir(this->bounds.arena_zone, this->bounds.offload_zone);

    this->offload_footprint.x() = std::abs(
        (this->offload_zone_norm.x() * OFFLOAD_FOOTPRINT_LENGTH_<float>)+  //
        (this->offload_zone_norm.y() * OFFLOAD_FOOTPRINT_WIDTH_<float>));
    this->offload_footprint.y() = std::abs(
        (this->offload_zone_norm.y() * OFFLOAD_FOOTPRINT_LENGTH_<float>)+  //
        (this->offload_zone_norm.x() * OFFLOAD_FOOTPRINT_WIDTH_<float>));

    this->offload_target_range = this->bounds.offload_zone;
    this->offload_target_range.min().x() += this->offload_footprint.x() / 2.f;
    this->offload_target_range.min().y() += this->offload_footprint.y() / 2.f;
    this->offload_target_range.max().x() -= this->offload_footprint.x() / 2.f;
    this->offload_target_range.max().y() -= this->offload_footprint.y() / 2.f;

    this->offload_target.template head<2>() =
        this->offload_target_range.center();
    this->offload_target.z() =
        std::atan2(this->offload_zone_norm.y(), this->offload_zone_norm.x());

    this->offload_vis_range = std::max(
        -OFFLOAD_FOOTPRINT_OFFSET_<float>,
        FOOTPRINT_R_MAX_<float> +
            distToBounds(this->offload_target, this->bounds.offload_zone));

    this->traversal_cursor.header.frame_id = this->tf_cache.arena_frame_id;
    this->recalcOffloadTarget();
}

void InputInterface::recalcOffloadTarget()
{
    this->traversal_cursor.pose.position.x =
        this->offload_target.x() +
        (this->offload_vis_range * std::cos(this->offload_target.z()));
    this->traversal_cursor.pose.position.y =
        this->offload_target.y() +
        (this->offload_vis_range * std::sin(this->offload_target.z()));
    this->traversal_cursor.pose.position.z = 0.;
    Quatf q = yawToQuat(this->offload_target.z());
    this->traversal_cursor.pose.orientation << q;
}

void InputInterface::iterateTravCursor()
{
    constexpr float CURSOR_MAX_SPEED_MPS = 2.f;
    constexpr float CURSOR_MAX_SPEED_RPS = 4.f;

    const float d_r =
        CURSOR_MAX_SPEED_RPS *
        std::min(1.f, TraversalCursorRotAxis::trapezoidSum(this->joy_state));

    lance::geom::Quatf q;
    q << this->traversal_cursor.pose.orientation;
    const float theta = lance::geom::quatToYaw(q) + d_r;
    this->traversal_cursor.pose.orientation << lance::geom::yawToQuat(theta);

    // "X" and "Y" axes are those of the controller - swap and invert Y
    // to obtain standard orientation
    const float d_x =
        CURSOR_MAX_SPEED_MPS *
        std::min(1.f, TraversalCursorPosAxes::X::trapezoidSum(this->joy_state));
    const float d_y =
        CURSOR_MAX_SPEED_MPS *
        std::min(1.f, TraversalCursorPosAxes::Y::trapezoidSum(this->joy_state));

    const float cos_theta = std::cos(theta);
    const float sin_theta = std::sin(theta);

    this->traversal_cursor.pose.position.x +=
        (d_x * cos_theta) - (d_y * sin_theta);
    this->traversal_cursor.pose.position.y +=
        (d_x * sin_theta) + (d_y * cos_theta);
}

void InputInterface::iterateMiningCursor() { this->iterateTravCursor(); }

void InputInterface::iterateOffloadCursor()
{
    constexpr float CURSOR_MAX_SPEED_MPS = 1.f;
    constexpr float CURSOR_MAX_SPEED_RPS = 2.f;

    const float d_x =
        CURSOR_MAX_SPEED_MPS *
        std::min(1.f, TraversalCursorPosAxes::X::trapezoidSum(this->joy_state));
    const float d_y =
        CURSOR_MAX_SPEED_MPS *
        std::min(1.f, TraversalCursorPosAxes::Y::trapezoidSum(this->joy_state));

    const float cos_theta = std::cos(this->offload_target.z());
    const float sin_theta = std::sin(this->offload_target.z());

    this->offload_target.x() += (d_x * cos_theta) - (d_y * sin_theta);
    this->offload_target.y() += (d_x * sin_theta) + (d_y * cos_theta);
    this->offload_target.template head<2>().array() =
        this->offload_target.template head<2>().array().cwiseMax(
            this->offload_target_range.min().array());
    this->offload_target.template head<2>().array() =
        this->offload_target.template head<2>().array().cwiseMin(
            this->offload_target_range.max().array());

    this->recalcOffloadTarget();
}


void InputInterface::publishTravTarget()
{
    this->traversal_target_pub->publish(this->traversal_cursor);
}

void InputInterface::publishMiningTarget()
{
    this->traversal_target_pub->publish(this->traversal_cursor);
    // TODO
}

void InputInterface::publishOffloadTarget()
{
    this->traversal_target_pub->publish(this->traversal_cursor);
    // TODO
}


bool InputInterface::isCursorState() const
{
    // return (
    //     this->state == State::TRAV_CURSOR ||
    //     this->state == State::MINING_CURSOR ||
    //     this->state == State::OFFLOAD_CURSOR);
    return this->state > State::OVERRIDE;
}

};  // namespace lance
