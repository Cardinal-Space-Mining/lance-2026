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

#include <chrono>
#include <memory>
#include <vector>
#include <cassert>
#include <iostream>

#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_srvs/srv/set_bool.hpp>

#include <sensor_msgs/msg/joy.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>

#include <visualization_msgs/msg/marker_array.hpp>

#include "util/geometry.hpp"
#include "util/joy_utils.hpp"
#include "util/ros_utils.hpp"

#include "robot/core/hid_bindings.hpp"
#include "robot/core/robot_status.hpp"
#include "robot/core/ros_interface.hpp"
#include "robot/core/motor_interface.hpp"
#include "robot/model/dynamics.hpp"
#include "robot/sensing/tf_cache.hpp"
#include "robot/telemetry/deserializer.hpp"


using namespace std::chrono;
using namespace std::chrono_literals;

using namespace util;
using namespace lance;
using namespace util::geom::cvt::ops;

#define WATCHDOG_PUB_DT           100ms
#define WATCHDOG_TELEOP_FEED_TIME 250ms
#define WATCHDOG_AUTO_FEED_TIME   10000ms


class WatchDog : public UsingRosAliases
{
    friend class ClientNode;

    using Int32Msg = std_msgs::msg::Int32;
    using SetBoolSrv = std_srvs::srv::SetBool;

    using SetBoolReqPtr = SetBoolSrv::Request::SharedPtr;
    using SetBoolRespPtr = SetBoolSrv::Response::SharedPtr;

public:
    WatchDog(RclNode&);

private:
    int32_t getFeedTime() const;

private:
    RclPubPtr<Int32Msg> watchdog_status_pub;
    RclSrvPtr<SetBoolSrv> set_teleop_srv;
    RclSrvPtr<SetBoolSrv> set_auto_srv;
    RclSrvPtr<SetBoolSrv> test_mode_srv;
    RclTimer::SharedPtr watchdog_timer;

    ControlMode ctrl_mode{ControlMode::DISABLED};
    uint8_t ctrl_opts{0};
};


class ClientNode : public rclcpp::Node, public UsingRosAliases
{
    using BoolMsg = std_msgs::msg::Bool;
    using Int32Msg = std_msgs::msg::Int32;

    using JoyMsg = sensor_msgs::msg::Joy;
    using JointStateMsg = sensor_msgs::msg::JointState;

    using PoseStampedMsg = geometry_msgs::msg::PoseStamped;
    using PointStampedMsg = geometry_msgs::msg::PointStamped;

    using MarkerMsg = visualization_msgs::msg::Marker;
    using MarkerArrayMsg = visualization_msgs::msg::MarkerArray;

public:
    ClientNode();

protected:
    enum : uint32_t
    {
        STATE_NONE = 0,
        STATE_MC_INPUT_OVERRIDE = (1 << 0),
        STATE_CURSOR_ENABLED = (1 << 1)
    };

protected:
    void initMarkers();

    void handleJoy(const JoyMsg&);
    void handleClickedPoint(const PointStampedMsg&);
    void handleInterfacePubs();

    bool hasState(uint32_t) const;
    void setState(uint32_t);
    void clearState(uint32_t);
    bool toggleState(uint32_t);

private:
    TfCache tf_cache;
    TelemetryDeserializer telemetry;

    WatchDog watchdog;

    RclPubPtr<JoyMsg> joy_pub;
    RclSubPtr<JoyMsg> joy_sub;
    RclPubPtr<PoseStampedMsg> traversal_target_pub;
    RclSubPtr<PointStampedMsg> clicked_point_sub;
    RclTimer::SharedPtr interface_pub_timer;

    RclSubPtr<TalonInfoMsg> hopper_info_sub;
    RclPubPtr<MarkerArrayMsg> markers_pub;
    RclTimer::SharedPtr markers_pub_timer;

    MarkerArrayMsg markers;
    JoyState joy_state;
    PoseStampedMsg traversal_cursor;

    uint32_t state{STATE_NONE};
};



// --- Watchdog ----------------------------------------------------------------

WatchDog::WatchDog(RclNode& node) :
    watchdog_status_pub{node.create_publisher<Int32Msg>(
        lance::WATCHDOG_TOPIC,
        rclcpp::SensorDataQoS{})},

    set_teleop_srv{node.create_service<SetBoolSrv>(
        lance::SET_TELEOP_TOPIC,
        [this](SetBoolReqPtr req, SetBoolRespPtr resp)
        {
            this->ctrl_mode =
                req->data ? ControlMode::TELEOPERATED : ControlMode::DISABLED;
            resp->success = true;
        })},

    set_auto_srv{node.create_service<SetBoolSrv>(
        lance::SET_AUTO_TOPIC,
        [this](SetBoolReqPtr req, SetBoolRespPtr resp)
        {
            this->ctrl_mode =
                req->data ? ControlMode::AUTONOMOUS : ControlMode::DISABLED;
            resp->success = true;
        })},

    test_mode_srv{node.create_service<SetBoolSrv>(
        lance::SET_TEST_TOPIC,
        [this](SetBoolReqPtr req, SetBoolRespPtr resp)
        {
            this->ctrl_opts = static_cast<uint8_t>(
                req->data ? ControlOpts::TEST_MODE : ControlOpts::NONE);
            resp->success = true;
        })},

    watchdog_timer{node.create_wall_timer(
        WATCHDOG_PUB_DT,
        [this]()
        {
            this->watchdog_status_pub->publish(
                Int32Msg{}.set__data(this->getFeedTime()));
        })}
{
}

int32_t WatchDog::getFeedTime() const
{
    return ControlStatus::format(
        this->ctrl_mode,
        this->ctrl_opts,
        WATCHDOG_TELEOP_FEED_TIME,
        WATCHDOG_AUTO_FEED_TIME);
}


// --- Implementation ----------------------------------------------------------

ClientNode::ClientNode() :
    Node("mission_control"),
    tf_cache{
        *this,
        declare_and_get_param<std::string>(*this, "arena_frame_id", "map"),
        declare_and_get_param<std::string>(*this, "odom_frame_id", "odom"),
        declare_and_get_param<std::string>(*this, "robot_frame_id", "robot")},
    telemetry{*this, this->tf_cache},

    watchdog{*this},

    joy_pub{this->create_publisher<JoyMsg>(
        lance::JOY_CTRL_TOPIC,
        rclcpp::SensorDataQoS{})},
    joy_sub{this->create_subscription<JoyMsg>(
        lance::JOY_INPUT_TOPIC,
        rclcpp::SensorDataQoS{},
        [this](const JoyMsg::ConstSharedPtr& msg) { this->handleJoy(*msg); })},

    traversal_target_pub{this->create_publisher<PoseStampedMsg>(
        lance::TRAVERSAL_TARGET_TOPIC,
        rclcpp::SensorDataQoS{})},
    clicked_point_sub{this->create_subscription<PointStampedMsg>(
        lance::CLICKED_POINT_TOPIC,
        rclcpp::SensorDataQoS{},
        [this](const PointStampedMsg::ConstSharedPtr& msg)
        { this->handleClickedPoint(*msg); })},

    interface_pub_timer{this->create_wall_timer(
        50ms,
        [this]() { this->handleInterfacePubs(); })},

    hopper_info_sub{this->create_subscription<TalonInfoMsg>(
        TALON_INFO_TOPIC("hopper_act"),
        rclcpp::SensorDataQoS{},
        [this](const TalonInfoMsg& info)
        {
            JointStateMsg msg;
            msg.header = info.header;
            msg.name.push_back(lance::HOPPER_JOINT_NAME);
            msg.position.push_back(
                lance::linearActuatorToJointAngle(info.position / 1000.));
            this->telemetry.getPubMap().publish("joint_states", msg);
        })},

    markers_pub{this->create_publisher<MarkerArrayMsg>(
        lance::ARENA_ZONES_TOPIC,
        rclcpp::SensorDataQoS{})},
    markers_pub_timer{this->create_wall_timer(
        1s,
        [this]() { this->markers_pub->publish(this->markers); })}
{
    this->initMarkers();

    std::cout << "LANCE-" << LANCE << " mission control initialized!"
              << std::endl;
}

void ClientNode::initMarkers()
{
    std::vector<double> min, max;

#define ADD_MARKER(param, NS, R, G, B, A)                            \
    {                                                                \
        util::declare_param(*this, param ".min", min, {0., 0., 0.}); \
        util::declare_param(*this, param ".max", max, {0., 0., 0.}); \
        assert(min.size() >= 3 && max.size() >= 3);                  \
        MarkerMsg& marker = this->markers.markers.emplace_back();    \
        marker.header.frame_id = this->tf_cache.arena_frame_id;      \
        marker.header.stamp = this->now();                           \
        marker.ns = NS;                                              \
        marker.id = this->markers.markers.size();                    \
        marker.type = MarkerMsg::CUBE;                               \
        marker.action = MarkerMsg::ADD;                              \
        marker.lifetime = rclcpp::Duration(0, 0);                    \
        marker.pose.position.x = (min[0] + max[0]) / 2.;             \
        marker.pose.position.y = (min[1] + max[1]) / 2.;             \
        marker.pose.position.z = (min[2] + max[2]) / 2.;             \
        marker.scale.x = (max[0] - min[0]);                          \
        marker.scale.y = (max[1] - min[1]);                          \
        marker.scale.z = (max[2] - min[2]);                          \
        marker.color.r = R;                                          \
        marker.color.g = G;                                          \
        marker.color.b = B;                                          \
        marker.color.a = A;                                          \
    }
    ADD_MARKER("arena_bounds", "arena", 1.f, 1.f, 1.f, 0.f);
    ADD_MARKER("mining_zone_bounds", "zones", 0.8f, 0.4f, 0.f, 0.2f);
    ADD_MARKER("offload_zone_bounds", "zones", 0.f, 0.2f, 0.8f, 0.2f);
    ADD_MARKER("construction_zone_bounds", "zones", 0.1f, 0.9f, 0.2f, 0.1f);

#undef ADD_MARKER
}

void ClientNode::handleJoy(const JoyMsg& msg)
{
    this->joy_state.update(msg);

    if (MissionControlOverrideButton::wasPressed(this->joy_state))
    {
        if (!this->toggleState(STATE_MC_INPUT_OVERRIDE))
        {
            this->clearState(STATE_CURSOR_ENABLED);
        }
    }

    if (!this->hasState(STATE_MC_INPUT_OVERRIDE))
    {
        this->joy_pub->publish(msg);
    }

    if (SetDisabledModeButton::wasPressed(this->joy_state))
    {
        if (this->hasState(STATE_MC_INPUT_OVERRIDE) ||
            this->watchdog.ctrl_mode == ControlMode::AUTONOMOUS)
        {
            this->watchdog.ctrl_mode = ControlMode::DISABLED;
            this->clearState(STATE_CURSOR_ENABLED);
        }
    }

    if (this->hasState(STATE_MC_INPUT_OVERRIDE))
    {
        if (SetTeleopModeButton::wasPressed(this->joy_state))
        {
            this->watchdog.ctrl_mode = ControlMode::TELEOPERATED;
        }
        if (SetAutoModeButton::wasPressed(this->joy_state))
        {
            this->watchdog.ctrl_mode = ControlMode::AUTONOMOUS;
            this->clearState(STATE_CURSOR_ENABLED);
        }
        if (ToggleTestModeButton::wasPressed(this->joy_state))
        {
            this->watchdog.ctrl_opts ^=
                static_cast<uint8_t>(ControlOpts::TEST_MODE);
        }

        if (this->watchdog.ctrl_mode == ControlMode::TELEOPERATED)
        {
            if (ToggleTraversalCursorButton::wasPressed(this->joy_state))
            {
                if (this->toggleState(STATE_CURSOR_ENABLED))
                {
                    if (this->tf_cache.hasTf(ROBOT_TO_ARENA_TF))
                    {
                        const auto* tf =
                            this->tf_cache.getTf(ROBOT_TO_ARENA_TF);
                        this->traversal_cursor.pose.position << tf->pose.vec;
                        this->traversal_cursor.pose.orientation
                            << lance::geom::flattenToYaw(tf->pose.quat);
                        this->traversal_cursor.header.frame_id =
                            this->tf_cache.arena_frame_id;
                    }
                    else if (this->tf_cache.hasTf(ROBOT_TO_ODOM_TF))
                    {
                        const auto* tf = this->tf_cache.getTf(ROBOT_TO_ODOM_TF);
                        this->traversal_cursor.pose.position << tf->pose.vec;
                        this->traversal_cursor.pose.orientation
                            << lance::geom::flattenToYaw(tf->pose.quat);
                        this->traversal_cursor.header.frame_id =
                            this->tf_cache.odom_frame_id;
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
                        this->traversal_cursor.header.frame_id =
                            this->tf_cache.robot_frame_id;
                    }
                }
            }
            if (ConfirmTraversalTargetButton::wasPressed(this->joy_state))
            {
                this->traversal_target_pub->publish(this->traversal_cursor);
                this->clearState(STATE_CURSOR_ENABLED);
            }

            if (this->hasState(STATE_CURSOR_ENABLED))
            {
                constexpr float CURSOR_MAX_SPEED_MPS = 2.f;
                constexpr float CURSOR_MAX_SPEED_RPS = 4.f;

                const float d_r =
                    CURSOR_MAX_SPEED_RPS *
                    std::min(
                        1.f,
                        TraversalCursorDirXAxis::trapezoidSum(this->joy_state));

                lance::geom::Quatf q;
                q << this->traversal_cursor.pose.orientation;
                const float theta = lance::geom::quatToYaw(q) + d_r;
                this->traversal_cursor.pose.orientation
                    << lance::geom::yawToQuat(theta);

                // "X" and "Y" axes are those of the controller - swap and invert Y
                // to obtain standard orientation
                const float d_x =
                    CURSOR_MAX_SPEED_MPS *
                    std::min(
                        1.f,
                        TraversalCursorPosYAxis::trapezoidSum(this->joy_state));
                const float d_y =
                    CURSOR_MAX_SPEED_MPS *
                    std::min(
                        1.f,
                        TraversalCursorPosXAxis::trapezoidSum(this->joy_state));

                const float cos_theta = std::cos(theta);
                const float sin_theta = std::sin(theta);

                this->traversal_cursor.pose.position.x +=
                    (d_x * cos_theta) - (d_y * sin_theta);
                this->traversal_cursor.pose.position.y +=
                    (d_x * sin_theta) + (d_y * cos_theta);
            }
        }
    }
}

void ClientNode::handleClickedPoint(const PointStampedMsg& msg)
{
    this->traversal_cursor.header = msg.header;
    this->traversal_cursor.pose.position = msg.point;
    this->traversal_cursor.pose.orientation.w = 0.0;
    this->traversal_cursor.pose.orientation.x = 0.0;
    this->traversal_cursor.pose.orientation.y = 0.0;
    this->traversal_cursor.pose.orientation.z = 0.0;

    this->traversal_target_pub->publish(this->traversal_cursor);

    this->clearState(STATE_CURSOR_ENABLED);
}

void ClientNode::handleInterfacePubs()
{
    GenericPubMap& pub_map = this->telemetry.getPubMap();

    pub_map.publish<Int32Msg>(ROBOT_TOPIC("mc_state"), this->state);

    if (this->hasState(STATE_CURSOR_ENABLED))
    {
        pub_map.publish(
            ROBOT_TOPIC("traversal_cursor"),
            this->traversal_cursor);
    }
}

bool ClientNode::hasState(uint32_t s) const { return this->state & s; }
void ClientNode::setState(uint32_t s) { this->state |= s; }
void ClientNode::clearState(uint32_t s) { this->state &= ~s; }
bool ClientNode::toggleState(uint32_t s) { return (this->state ^= s) & s; }



// --- Main --------------------------------------------------------------------

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ClientNode>());
    rclcpp::shutdown();
    return 0;
}
