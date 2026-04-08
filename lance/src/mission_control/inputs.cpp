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


namespace lance
{

InputInterface::InputInterface(
    RclNode& node,
    const TfCache& tf_cache,
    TelemetryDeserializer& telemetry,
    WatchDog& watchdog) :
    tf_cache{tf_cache},
    telemetry{telemetry},
    watchdog{watchdog},

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
            this->watchdog.getCtrl() == ControlMode::AUTONOMOUS)
        {
            this->watchdog.setCtrl(ControlMode::DISABLED);
            this->clearState(STATE_CURSOR_ENABLED);
        }
    }

    if (this->hasState(STATE_MC_INPUT_OVERRIDE))
    {
        if (SetTeleopModeButton::wasPressed(this->joy_state))
        {
            this->watchdog.setCtrl(ControlMode::TELEOPERATED);
        }
        if (SetAutoModeButton::wasPressed(this->joy_state))
        {
            this->watchdog.setCtrl(ControlMode::AUTONOMOUS);
            this->clearState(STATE_CURSOR_ENABLED);
        }
        if (ToggleTestModeButton::wasPressed(this->joy_state))
        {
            this->watchdog.toggleOpt(
                static_cast<uint8_t>(ControlOpts::TEST_MODE));
        }

        if (this->watchdog.getCtrl() == ControlMode::TELEOPERATED)
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
                        TraversalCursorRotAxis::trapezoidSum(this->joy_state));

                lance::geom::Quatf q;
                q << this->traversal_cursor.pose.orientation;
                const float theta = lance::geom::quatToYaw(q) + d_r;
                this->traversal_cursor.pose.orientation
                    << lance::geom::yawToQuat(theta);

                // "X" and "Y" axes are those of the controller - swap and invert Y
                // to obtain standard orientation
                const float d_x = CURSOR_MAX_SPEED_MPS *
                                  std::min(
                                      1.f,
                                      TraversalCursorPosAxes::X::trapezoidSum(
                                          this->joy_state));
                const float d_y = CURSOR_MAX_SPEED_MPS *
                                  std::min(
                                      1.f,
                                      TraversalCursorPosAxes::Y::trapezoidSum(
                                          this->joy_state));

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

void InputInterface::handleClickedPoint(const PointStampedMsg& msg)
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

void InputInterface::handleInterfacePubs()
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

bool InputInterface::hasState(uint32_t s) const { return this->state & s; }
void InputInterface::setState(uint32_t s) { this->state |= s; }
void InputInterface::clearState(uint32_t s) { this->state &= ~s; }
bool InputInterface::toggleState(uint32_t s) { return (this->state ^= s) & s; }

};  // namespace lance
