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

#include "advanced_controls.hpp"

#include <chrono>

#include <geometry_msgs/msg/pose_stamped.hpp>

#include <csm_utils/pub_map.hpp>
#include <csm_utils/geometry.hpp>

#include "robot/core/hid_bindings.hpp"
#include "robot/core/robot_status.hpp"
#include "robot/core/ros_interface.hpp"
#include "robot/control/teleop/remote_commands.hpp"


using namespace std::chrono_literals;

using namespace util;
using namespace util::geom::cvt::ops;
using namespace lance::geom;

using PoseStampedMsg = geometry_msgs::msg::PoseStamped;


constexpr float TRAV_CURSOR_SPEED_MPS = 2.f;
constexpr float TRAV_CURSOR_SPEED_RPS = 4.f;

constexpr float OFFLOAD_CURSOR_SPEED_MPS = 1.f;
constexpr float OFFLOAD_CURSOR_SPEED_RPS = 2.f;
constexpr float OFFLOAD_ADJUSTMENT_SPEED_MPS = 1.f;

constexpr double OFFLOAD_VISUAL_HEIGHT = 0.5;


namespace lance
{

AdvancedControls::AdvancedControls(
    RclNode& node,
    const TfCache& tf_cache,
    MarkerManager& markers,
    TelemetryDeserializer& telemetry,
    WatchDog& watchdog,
    const ZoneBounds& bounds) :
    tf_cache{tf_cache},
    markers{markers},
    telemetry{telemetry},
    watchdog{watchdog},
    bounds{bounds},
    rcl_clock{node.get_clock()},

    joy_pub{node.create_publisher<JoyMsg>(
        lance::JOY_CTRL_TOPIC,
        rclcpp::SensorDataQoS{})},
    joy_sub{node.create_subscription<JoyMsg>(
        lance::JOY_INPUT_TOPIC,
        rclcpp::SensorDataQoS{},
        [this](const JoyMsg::ConstSharedPtr& msg) { this->handleJoy(*msg); })},

    commands_pub{node.create_publisher<BytesMsg>(
        lance::REMOTE_COMMANDS_TOPIC,
        rclcpp::SensorDataQoS{})},
    clicked_point_sub{node.create_subscription<PointStampedMsg>(
        lance::CLICKED_POINT_TOPIC,
        rclcpp::SensorDataQoS{},
        [this](const PointStampedMsg::ConstSharedPtr& msg)
        { this->handleClickedPoint(*msg); })},

    interface_pub_timer{node.create_wall_timer(
        50ms,
        [this]() { this->handleInterfacePubs(); })},

    footprint_markers_id{this->markers.reserveGroup(1, "robot")},
    mining_markers_id{this->markers.reserveGroup(1, "robot")},
    offload_markers_id{this->markers.reserveGroup(1, "robot")}
{
    this->initMarkers();
}


void AdvancedControls::initMarkers()
{
    {
        auto g = this->markers.getGroup(this->footprint_markers_id)
                     .setType(MarkerManager::MarkerMsg::CUBE)
                     .setDuration(RclDur{0, 100000000})
                     .setColor(0.5f, 0.5f, 0.5f, 0.5f);

        g[0].scale.x = PRIMARY_COLLISION_ZONE_LENGTH_OFFSET;
        g[0].scale.y = PRIMARY_COLLISION_ZONE_WIDTH;
        g[0].scale.z = PRIMARY_COLLISION_ZONE_HEIGHT;
        g[0].pose.position.z = PRIMARY_COLLISION_ZONE_Z;
    }
    {
        auto g = this->markers.getGroup(this->mining_markers_id)
                     .setFrameId(this->tf_cache.arena_frame_id)
                     .setType(MarkerManager::MarkerMsg::CUBE)
                     .setDuration(RclDur{0, 100000000})
                     .setColor(0.5f, 0.5f, 0.5f, 0.5f);

        g[0].scale.y = PRIMARY_COLLISION_ZONE_WIDTH;
        g[0].scale.z = PRIMARY_COLLISION_ZONE_HEIGHT;
        g[0].pose.position.z = PRIMARY_COLLISION_ZONE_Z;
    }
    {
        auto g = this->markers.getGroup(this->offload_markers_id)
                     .setFrameId(this->tf_cache.arena_frame_id)
                     .setType(MarkerManager::MarkerMsg::CUBE)
                     .setDuration(RclDur{0, 100000000})
                     .setColor(0.1f, 0.4f, 0.7f, 0.5f);

        g[0].scale.x = OFFLOAD_FOOTPRINT_LENGTH;
        g[0].scale.y = OFFLOAD_FOOTPRINT_WIDTH;
        g[0].scale.z = OFFLOAD_VISUAL_HEIGHT;
        g[0].pose.position.z = OFFLOAD_VISUAL_HEIGHT / 2;
    }
}


// --- Handlers ----------------------------------------------------------------

void AdvancedControls::handleJoy(const JoyMsg& msg)
{
    this->joy_state.update(msg);

    switch (this->state)
    {
        case State::PASSTHROUGH:
        {
            this->iteratePassthroughMode();
            break;
        }
        case State::OVERRIDE:
        {
            this->iterateOverrideMode();
            break;
        }
        case State::TRAV_CURSOR:
        {
            this->iterateTravCursorMode();
            break;
        }
        case State::MINING_CURSOR:
        {
            this->iterateMiningCursorMode();
            break;
        }
        case State::OFFLOAD_CURSOR:
        {
            this->iterateOffloadCursorMode();
            break;
        }
    }

    if (this->state == State::PASSTHROUGH &&
        this->watchdog.isCtrl(ControlMode::TELEOPERATED))
    {
        this->joy_pub->publish(msg);
    }
}

void AdvancedControls::handleClickedPoint(const PointStampedMsg& msg)
{
    if (!this->watchdog.isCtrl(ControlMode::TELEOPERATED))
    {
        return;
    }
    if (this->isCursorState())
    {
        this->state = State::OVERRIDE;
    }

    this->cursor_pose.vec << msg.point;
    this->cursor_pose.quat = Quatf{0.f, 0.f, 0.f, 0.f};

    BytesMsg cmd_msg;
    RemoteCommands::serializeTraversalCmd(
        cmd_msg,
        this->cursor_pose,
        this->tf_cache.resolveKeyFrame(msg.header.frame_id));

    this->commands_pub->publish(cmd_msg);
}


void AdvancedControls::handleInterfacePubs()
{
    // GenericPubMap& pub_map = this->telemetry.getPubMap();

    this->telemetry.getPubMap().publish<Int32Msg>(
        lance::MC_STATE_TOPIC,
        static_cast<uint32_t>(this->state));

    switch (this->state)
    {
        case State::TRAV_CURSOR:
        {
            this->publishTravVisuals();
            break;
        }
        case State::MINING_CURSOR:
        {
            this->publishMiningVisuals();
            break;
        }
        case State::OFFLOAD_CURSOR:
        {
            this->publishOffloadVisuals();
            break;
        }
        default:
        {
        }
    }
}



// --- Base States & Common ----------------------------------------------------

void AdvancedControls::iteratePassthroughMode()
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

void AdvancedControls::iterateOverrideMode()
{
    if (this->handleCommonOverrides())
    {
        return;
    }

    if (this->watchdog.isCtrl(ControlMode::TELEOPERATED))
    {
        if (SetTravCursorButton::wasPressed(this->joy_state))
        {
            this->initTravCursorMode();
            return;
        }
        if (SetMiningCursorButton::wasPressed(this->joy_state))
        {
            this->initMiningCursorMode();
            return;
        }
        if (SetOffloadCursorButton::wasPressed(this->joy_state))
        {
            this->initOffloadCursorMode();
            return;
        }
    }
}

bool AdvancedControls::handleCommonOverrides()
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
    if (ToggleQuickAutoButton::wasPressed(this->joy_state))
    {
        this->watchdog.toggleOpt(static_cast<uint8_t>(ControlOpts::QUICK_AUTO));
    }
    if (ToggleAssistAsAutoButton::wasPressed(this->joy_state))
    {
        this->watchdog.toggleOpt(
            static_cast<uint8_t>(ControlOpts::ASSIST_AS_AUTO));
    }

    return false;
}

bool AdvancedControls::isCursorState() const
{
    // return (
    //     this->state == State::TRAV_CURSOR ||
    //     this->state == State::MINING_CURSOR ||
    //     this->state == State::OFFLOAD_CURSOR);
    return this->state > State::OVERRIDE;
}




// --- Traversal Cursor Mode ---------------------------------------------------

void AdvancedControls::initTravCursorMode()
{
    if (this->tf_cache.hasTf(ROBOT_TO_ARENA_TF))
    {
        const auto* tf = this->tf_cache.getTf(ROBOT_TO_ARENA_TF);
        this->cursor_pose.vec = tf->pose.vec;
        this->cursor_pose.quat = lance::geom::flattenToYaw(tf->pose.quat);
        this->cursor_frame_id = KeyFrame::ARENA_FRAME;
    }
    else if (this->tf_cache.hasTf(ROBOT_TO_ODOM_TF))
    {
        const auto* tf = this->tf_cache.getTf(ROBOT_TO_ODOM_TF);
        this->cursor_pose.vec = tf->pose.vec;
        this->cursor_pose.quat = lance::geom::flattenToYaw(tf->pose.quat);
        this->cursor_frame_id = KeyFrame::ODOM_FRAME;
    }
    else
    {
        return;
    }

    this->state = State::TRAV_CURSOR;
}

void AdvancedControls::iterateTravCursorMode()
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
        this->initMiningCursorMode();
        return;
    }

    if (SetOffloadCursorButton::wasPressed(this->joy_state))
    {
        this->initOffloadCursorMode();
        return;
    }

    this->iterateTravCursorCtrl();
}

void AdvancedControls::iterateTravCursorCtrl()
{
    const float d_r =
        TRAV_CURSOR_SPEED_RPS *
        std::min(1.f, TraversalCursorRotAxis::trapezoidSum(this->joy_state));

    const float theta = lance::geom::quatToYaw(this->cursor_pose.quat) + d_r;
    this->cursor_pose.quat = lance::geom::yawToQuat(theta);

    // "X" and "Y" axes are those of the controller - swap and invert Y
    // to obtain standard orientation
    const float d_x =
        TRAV_CURSOR_SPEED_MPS *
        std::min(1.f, TraversalCursorPosAxes::X::trapezoidSum(this->joy_state));
    const float d_y =
        TRAV_CURSOR_SPEED_MPS *
        std::min(1.f, TraversalCursorPosAxes::Y::trapezoidSum(this->joy_state));

    const float cos_theta = std::cos(theta);
    const float sin_theta = std::sin(theta);

    this->cursor_pose.vec.x() += (d_x * cos_theta) - (d_y * sin_theta);
    this->cursor_pose.vec.y() += (d_x * sin_theta) + (d_y * cos_theta);
}

void AdvancedControls::publishTravTarget()
{
    BytesMsg msg;
    RemoteCommands::serializeTraversalCmd(
        msg,
        this->cursor_pose,
        this->cursor_frame_id);
    this->commands_pub->publish(msg);
}

void AdvancedControls::publishTravVisuals()
{
    GenericPubMap& pub_map = this->telemetry.getPubMap();

    PoseStampedMsg cursor_msg;
    cursor_msg.header.frame_id =
        this->tf_cache.getFrameId(this->cursor_frame_id);
    cursor_msg.pose << this->cursor_pose;
    pub_map.publish(lance::MC_CURSOR_TOPIC, cursor_msg);

    this->updateFootprintMarkers();

    this->markers.clearOutput();
    this->markers.addGroupToOutput(this->footprint_markers_id);
    this->markers.pubOutputMarkers(
        pub_map.getPub<MarkerManager::MarkerArrayMsg>(
            lance::ROBOT_MARKERS_TOPIC),
        this->rcl_clock->now(),
        true);
}

void AdvancedControls::updateFootprintMarkers()
{
    auto& m = this->markers.getGroup(this->footprint_markers_id)[0];

    m.pose.position.x = this->cursor_pose.vec.x();
    m.pose.position.y = this->cursor_pose.vec.y();
    m.pose.orientation << this->cursor_pose.quat;
    m.header.frame_id = this->tf_cache.getFrameId(this->cursor_frame_id);
}




// --- Mining Cursor Mode ------------------------------------------------------

void AdvancedControls::initMiningCursorMode()
{
    if (!this->tf_cache.hasTf(ROBOT_TO_ARENA_TF))
    {
        return;
    }

    this->cursor_pose.vec.x() = this->bounds.mining_zone.center().x();
    this->cursor_pose.vec.y() = this->bounds.mining_zone.center().y();
    this->cursor_pose.vec.z() = 0.f;
    this->cursor_pose.quat = Quatf::Identity();
    this->cursor_frame_id = KeyFrame::ARENA_FRAME;

    this->state = State::MINING_CURSOR;
}

void AdvancedControls::iterateMiningCursorMode()
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
        this->initTravCursorMode();
        return;
    }

    if (SetOffloadCursorButton::wasPressed(this->joy_state))
    {
        this->initOffloadCursorMode();
        return;
    }

    this->iterateMiningCursorCtrl();
}

void AdvancedControls::iterateMiningCursorCtrl()
{
    this->iterateTravCursorCtrl();
}

void AdvancedControls::publishMiningTarget()
{
    BytesMsg msg;
    RemoteCommands::serializeMiningCmd(msg, this->cursor_pose);
    this->commands_pub->publish(msg);
}

void AdvancedControls::publishMiningVisuals()
{
    GenericPubMap& pub_map = this->telemetry.getPubMap();

    PoseStampedMsg cursor_msg;
    cursor_msg.header.frame_id =
        this->tf_cache.getFrameId(this->cursor_frame_id);
    cursor_msg.pose << this->cursor_pose;
    pub_map.publish(lance::MC_CURSOR_TOPIC, cursor_msg);

    this->markers.clearOutput();

    this->updateFootprintMarkers();
    this->markers.addGroupToOutput(this->footprint_markers_id);
    if (this->updateMiningMarkers())
    {
        this->markers.addGroupToOutput(this->mining_markers_id);
    }

    this->markers.pubOutputMarkers(
        pub_map.getPub<MarkerManager::MarkerArrayMsg>(
            lance::ROBOT_MARKERS_TOPIC),
        this->rcl_clock->now(),
        true);
}

bool AdvancedControls::updateMiningMarkers()
{
    Pose2f mining_start;
    float cos_theta, sin_theta;
    {
        mining_start.z() = quatToYaw(this->cursor_pose.quat);
        cos_theta = std::cos(mining_start.z());
        sin_theta = std::sin(mining_start.z());

        mining_start.x() =
            this->cursor_pose.vec.x() + (cos_theta * FOOTPRINT_X_MAX_<float>);
        mining_start.y() =
            this->cursor_pose.vec.y() + (sin_theta * FOOTPRINT_X_MAX_<float>);
    };

    float dist = 0.f;
    if (this->bounds.mining_zone.contains(mining_start.template head<2>()))
    {
        dist = distToBounds(mining_start, this->bounds.mining_zone);
    }
    else if (this->bounds.arena_zone.contains(mining_start.template head<2>()))
    {
        dist = distToBounds(mining_start, this->bounds.arena_zone);
    }
    else
    {
        return false;
    }

    {
        auto& m = this->markers.getGroup(this->mining_markers_id)[0];

        m.scale.x = dist;
        m.pose.position.x = mining_start.x() + (cos_theta * dist / 2.f);
        m.pose.position.y = mining_start.y() + (sin_theta * dist / 2.f);
        m.pose.orientation << this->cursor_pose.quat;
    }

    return true;
}




// --- Offload Cursor Mode -----------------------------------------------------

void AdvancedControls::initOffloadCursorMode()
{
    if (!this->tf_cache.hasTf(ROBOT_TO_ARENA_TF))
    {
        return;
    }

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

    this->offload_manual_off = 0.f;
    this->recalcOffloadRange();

    this->cursor_frame_id = KeyFrame::ARENA_FRAME;
    this->recalcOffloadTarget();

    this->state = State::OFFLOAD_CURSOR;
}

void AdvancedControls::recalcOffloadRange()
{
    this->offload_vis_range =
        std::max(
            -OFFLOAD_FOOTPRINT_OFFSET_<float>,
            FOOTPRINT_R_MAX_<float> +
                distToBounds(this->offload_target, this->bounds.offload_zone)) +
        this->offload_manual_off;
}

void AdvancedControls::recalcOffloadTarget()
{
    this->cursor_pose.vec.x() =
        this->offload_target.x() +
        (this->offload_vis_range * std::cos(this->offload_target.z()));
    this->cursor_pose.vec.y() =
        this->offload_target.y() +
        (this->offload_vis_range * std::sin(this->offload_target.z()));
    this->cursor_pose.vec.z() = 0.f;
    this->cursor_pose.quat = yawToQuat(this->offload_target.z());
}

void AdvancedControls::iterateOffloadCursorMode()
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
        this->initTravCursorMode();
        return;
    }

    if (SetMiningCursorButton::wasPressed(this->joy_state))
    {
        this->initMiningCursorMode();
        return;
    }

    this->iterateOffloadCursorCtrl();
}

void AdvancedControls::iterateOffloadCursorCtrl()
{
    if (OffloadCursorAlignButton::wasPressed(this->joy_state))
    {
        this->offload_target.z() = std::atan2(
            this->offload_zone_norm.y(),
            this->offload_zone_norm.x());
        this->offload_manual_off = 0.f;
    }

    this->offload_target.z() +=
        OFFLOAD_CURSOR_SPEED_RPS *
        std::min(1.f, TraversalCursorRotAxis::trapezoidSum(this->joy_state));

    const float d_x =
        OFFLOAD_CURSOR_SPEED_MPS *
        std::min(1.f, TraversalCursorPosAxes::X::trapezoidSum(this->joy_state));
    const float d_y =
        OFFLOAD_CURSOR_SPEED_MPS *
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

    this->offload_manual_off +=
        OFFLOAD_ADJUSTMENT_SPEED_MPS *
        (std::min(
             1.f,
             OffloadCursorScaleUpAxis::triggerTrapezoidSum(this->joy_state)) -
         std::min(
             1.f,
             OffloadCursorScaleDownAxis::triggerTrapezoidSum(this->joy_state)));

    this->recalcOffloadRange();
    this->recalcOffloadTarget();
}

void AdvancedControls::publishOffloadTarget()
{
    BytesMsg msg;
    RemoteCommands::serializeOffloadCmd(
        msg,
        this->cursor_pose,
        this->offload_vis_range + OFFLOAD_FOOTPRINT_OFFSET_<float>);
    this->commands_pub->publish(msg);
}

void AdvancedControls::publishOffloadVisuals()
{
    GenericPubMap& pub_map = this->telemetry.getPubMap();

    PoseStampedMsg cursor_msg;
    cursor_msg.header.frame_id =
        this->tf_cache.getFrameId(this->cursor_frame_id);
    cursor_msg.pose << this->cursor_pose;
    pub_map.publish(lance::MC_CURSOR_TOPIC, cursor_msg);

    this->updateFootprintMarkers();
    this->updateOffloadMarkers();

    this->markers.clearOutput();
    this->markers.addGroupToOutput(this->footprint_markers_id);
    this->markers.addGroupToOutput(this->offload_markers_id);
    this->markers.pubOutputMarkers(
        pub_map.getPub<MarkerManager::MarkerArrayMsg>(
            lance::ROBOT_MARKERS_TOPIC),
        this->rcl_clock->now(),
        true);
}

void AdvancedControls::updateOffloadMarkers()
{
    auto& m = this->markers.getGroup(this->offload_markers_id)[0];

    m.pose.position.x = this->offload_target.x();
    m.pose.position.y = this->offload_target.y();
    m.pose.orientation << yawToQuat(this->offload_target.z());
}

};  // namespace lance
