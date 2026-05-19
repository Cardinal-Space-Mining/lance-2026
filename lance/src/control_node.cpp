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

#include "util/joy_utils.hpp"
#include "util/ros_utils.hpp"

#include "robot/core/robot_status.hpp"
#include "robot/core/ros_interface.hpp"
#include "robot/core/motor_interface.hpp"
#include "robot/control/robot_controller.hpp"
#include "robot/telemetry/serializer.hpp"


using namespace std::chrono_literals;
using namespace lance;


class RobotControlNode : public rclcpp::Node, public util::UsingRosAliases
{
    using JoyState = util::JoyState;

    using JoyMsg = sensor_msgs::msg::Joy;
    using Int32Msg = std_msgs::msg::Int32;

public:
    RobotControlNode();

protected:
    struct TalonPubSub
    {
        RclPubPtr<TalonCtrlMsg> ctrl_pub;
        RclSubPtr<TalonInfoMsg> info_sub;
        RclSubPtr<TalonFaultsMsg> faults_sub;
    };

private:
    RobotController robot_controller;
    TelemetrySerializer telemetry;

    TalonPubSub track_right_pub_sub;
    TalonPubSub track_left_pub_sub;
    TalonPubSub trencher_pub_sub;
    TalonPubSub hopper_belt_pub_sub;
    TalonPubSub hopper_actuator_pub_sub;

    RclSubPtr<JoyMsg> joy_sub;
    RclSubPtr<Int32Msg> watchdog_sub;
    RclTimer::SharedPtr control_iteration_timer;

    RobotMotorStatus robot_motor_status;
    RobotMotorFaults robot_motor_faults;
    JoyMsg::ConstSharedPtr last_joy_msg{nullptr};
    RclTime last_joy_time;
    JoyState joy_state;
    int32_t control_status{0};
};



// ---

// clang-format off
#define INIT_TALON_PUB_SUB(device_topic, device_var)          \
    device_var##_pub_sub                                      \
    {                                                         \
        this->create_publisher<TalonCtrlMsg>(                 \
            TALON_CTRL_TOPIC(#device_topic),                  \
            TALON_CTRL_PUBSUB_QOS),                           \
        this->create_subscription<TalonInfoMsg>(              \
            TALON_INFO_TOPIC(#device_topic),                  \
            rclcpp::SensorDataQoS{},                          \
            [this](const TalonInfoMsg& msg)                   \
            { this->robot_motor_status.device_var = msg; }),  \
        this->create_subscription<TalonFaultsMsg>(            \
            ROBOT_TOPIC(#device_topic "/faults"),            \
            rclcpp::SensorDataQoS{},                          \
            [this](const TalonFaultsMsg& msg)                 \
            { this->robot_motor_faults.device_var = msg; })   \
    }
// clang-format on

RobotControlNode::RobotControlNode() :
    Node{"robot_control"},
    robot_controller{*this},
    telemetry{*this, 1.f},

    INIT_TALON_PUB_SUB(track_right, track_right),
    INIT_TALON_PUB_SUB(track_left, track_left),
    INIT_TALON_PUB_SUB(trencher, trencher),
    INIT_TALON_PUB_SUB(hopper_belt, hopper_belt),
    INIT_TALON_PUB_SUB(hopper_act, hopper_actuator),

    joy_sub{this->create_subscription<JoyMsg>(
        lance::JOY_CTRL_TOPIC,
        rclcpp::SensorDataQoS{},
        [this](const JoyMsg::ConstSharedPtr& msg)
        {
            this->last_joy_msg = msg;
            this->last_joy_time = this->get_clock()->now();
        })},
    watchdog_sub{this->create_subscription<Int32Msg>(
        lance::WATCHDOG_TOPIC,
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
            else if (
                (this->get_clock()->now() - this->last_joy_time).nanoseconds() >
                static_cast<int64_t>(
                    ControlStatus::getTimeoutMs(this->control_status)) *
                    1000000U)
            {
                this->joy_state.updateDisconnected();
            }

            RobotMotorCommands commands;
            commands.disableAll();
            this->robot_controller.iterate(
                this->control_status,
                this->joy_state,
                this->robot_motor_status,
                this->robot_motor_faults,
                commands);

            this->track_right_pub_sub.ctrl_pub->publish(commands.track_right);
            this->track_left_pub_sub.ctrl_pub->publish(commands.track_left);
            this->trencher_pub_sub.ctrl_pub->publish(commands.trencher);
            this->hopper_belt_pub_sub.ctrl_pub->publish(commands.hopper_belt);
            this->hopper_actuator_pub_sub.ctrl_pub->publish(
                commands.hopper_actuator);

            this->telemetry.update(this->robot_controller);

            PROFILING_NOTIFY_ALWAYS(iterate_control);
            PROFILING_FLUSH();
        })},

    last_joy_time{0, 0, this->get_clock()->get_clock_type()}
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
