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
#include <iostream>

#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/int32.hpp>
#include <sensor_msgs/msg/joy.hpp>

#include <csm_metrics/profiling.hpp>

#include "util/pub_map.hpp"
#include "util/joy_utils.hpp"
#include "util/ros_utils.hpp"

#include "robot/core/motor_interface.hpp"
#include "robot/control/robot_controller.hpp"
#include "robot/telemetry/telemetry.hpp"


#define ROBOT_TOPIC(subtopic) "lance/" subtopic
#define TALON_CTRL_PUB_QOS                                               \
    rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile()

using namespace std::chrono_literals;
using namespace util::ros_aliases;
using namespace lance;


class RobotControlNode : public rclcpp::Node
{
    using JoyState = util::JoyState;
    using GenericPubMap = util::GenericPubMap;

    using JoyMsg = sensor_msgs::msg::Joy;
    using Int32Msg = std_msgs::msg::Int32;

public:
    RobotControlNode();

protected:
    struct TalonPubSub
    {
        SharedPub<TalonCtrlMsg> ctrl_pub;
        SharedSub<TalonInfoMsg> info_sub;
    };

private:
    GenericPubMap pub_map;
    RobotController robot_controller;
    TelemetrySerializer telemetry;

    TalonPubSub track_right_pub_sub;
    TalonPubSub track_left_pub_sub;
    TalonPubSub trencher_pub_sub;
    TalonPubSub hopper_belt_pub_sub;
    TalonPubSub hopper_actuator_pub_sub;

    SharedSub<JoyMsg> joy_sub;
    SharedSub<Int32Msg> watchdog_sub;
    RclTimer control_iteration_timer;

    RobotMotorStatus robot_motor_status;
    JoyMsg::ConstSharedPtr last_joy_msg{nullptr};
    JoyState joy_state;
    int32_t control_status{0};
};



// ---

#define INIT_TALON_PUB_SUB(device_topic, device_var)            \
    device_var##_pub_sub                                        \
    {                                                           \
        this->create_publisher<TalonCtrlMsg>(                   \
            ROBOT_TOPIC(#device_topic "/ctrl"),                 \
            TALON_CTRL_PUB_QOS),                                \
            this->create_subscription<TalonInfoMsg>(            \
                ROBOT_TOPIC(#device_topic "/info"),             \
                rclcpp::SensorDataQoS{},                        \
                [this](const TalonInfoMsg& msg)                 \
                { this->robot_motor_status.device_var = msg; }) \
    }

RobotControlNode::RobotControlNode() :
    Node{"robot_control"},
    pub_map{*this, "", rclcpp::SensorDataQoS{}},
    robot_controller{*this, this->pub_map},
    telemetry{*this, 1.f},

    INIT_TALON_PUB_SUB(track_right, track_right),
    INIT_TALON_PUB_SUB(track_left, track_left),
    INIT_TALON_PUB_SUB(trencher, trencher),
    INIT_TALON_PUB_SUB(hopper_belt, hopper_belt),
    INIT_TALON_PUB_SUB(hopper_act, hopper_actuator),

    joy_sub{this->create_subscription<JoyMsg>(
        "/joy",
        rclcpp::SensorDataQoS{},
        [this](const JoyMsg::ConstSharedPtr& msg)
        { this->last_joy_msg = msg; })},
    watchdog_sub{this->create_subscription<Int32Msg>(
        ROBOT_TOPIC("watchdog_status"),
        rclcpp::SensorDataQoS{},
        [this](const Int32Msg& status)
        { this->control_status = status.data; })},

    control_iteration_timer{this->create_wall_timer(
        std::chrono::duration<float>(
            this->robot_controller.getParams().iteration_period_seconds),
        [this]()
        {
            PROFILING_SYNC();
            PROFILING_NOTIFY_ALWAYS(iterate_control);

            if (this->last_joy_msg)
            {
                this->joy_state.update(*this->last_joy_msg);
                this->last_joy_msg = nullptr;
            }

            RobotMotorCommands commands;
            this->robot_controller.iterate(
                this->control_status,
                this->joy_state,
                this->robot_motor_status,
                commands);

            this->track_right_pub_sub.ctrl_pub->publish(commands.track_right);
            this->track_left_pub_sub.ctrl_pub->publish(commands.track_left);
            this->trencher_pub_sub.ctrl_pub->publish(commands.trencher);
            this->hopper_belt_pub_sub.ctrl_pub->publish(commands.hopper_belt);
            this->hopper_actuator_pub_sub.ctrl_pub->publish(
                commands.hopper_actuator);

            this->telemetry.update(commands, this->robot_controller);

            PROFILING_NOTIFY_ALWAYS(iterate_control);
            PROFILING_FLUSH();
        })}
{
    std::cout << "LANCE-" << LANCE << " controller initialized!" << std::endl;
}



int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RobotControlNode>();
    PROFILING_INIT(*node, PROFILING_DEFAULT_TOPIC, PROFILING_DEFAULT_QOS);
    rclcpp::spin(node);
    PROFILING_DEINIT();
    rclcpp::shutdown();

    return 0;
}
