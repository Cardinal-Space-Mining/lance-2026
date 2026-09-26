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

/**
 * @file control_node.cpp
 * @brief Robot-side master control process executing the periodic control loop and CTRE hardware IO.
 *
 * ## Architecture Overview
 * This node runs onboard the LANCE rover:
 *   - Subscribes to `/joy_ctrl` for filtered operator joystick input.
 *   - Subscribes to `/lance/watchdog_status` (INT32 heartbeat) encoding the active ControlMode,
 *     timeout interval, and configuration bitflags.
 *   - Manages real-time bidirectional communication with CTRE Talon FX motor controllers:
 *       * Subscribes to `info` and `faults` telemetry topics for all 5 motor channels.
 *       * Publishes low-latency `ctrl` commands (velocity, position, voltage) at ~20 Hz.
 *   - Executes the master `RobotController` state machine every `iteration_period_seconds`.
 *   - Serializes comprehensive hierarchical telemetry into binary packets over `/lance/telemetry`.
 */

#include <chrono>
#include <iostream>

#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/int32.hpp>
#include <sensor_msgs/msg/joy.hpp>

#include <csm_metrics/profiling.hpp>
#include <csm_utils/joy_utils.hpp>
#include <csm_utils/ros_utils.hpp>

#include "robot/core/robot_status.hpp"
#include "robot/core/ros_interface.hpp"
#include "robot/core/motor_interface.hpp"
#include "robot/control/robot_controller.hpp"
#include "robot/telemetry/serializer.hpp"


using namespace std::chrono_literals;
using namespace lance;


/**
 * @class RobotControlNode
 * @brief Main ROS 2 node orchestrating the robot-side control pipeline and Talon FX driver interface.
 */
class RobotControlNode : public rclcpp::Node, public util::UsingRosAliases
{
    using JoyState = util::JoyState;

    using JoyMsg = sensor_msgs::msg::Joy;
    using Int32Msg = std_msgs::msg::Int32;

public:
    RobotControlNode();

protected:
    /**
     * @struct TalonPubSub
     * @brief Bundles command publisher and telemetry subscribers for a single CTRE Talon FX controller.
     */
    struct TalonPubSub
    {
        RclPubPtr<TalonCtrlMsg> ctrl_pub;     ///< Real-time velocity/position command publisher.
        RclSubPtr<TalonInfoMsg> info_sub;     ///< Rotational speed, position, and sensor feedback subscriber.
        RclSubPtr<TalonFaultsMsg> faults_sub; ///< Hardware limits and current limit faults subscriber.
    };

private:
    RobotController robot_controller; ///< Master state machine (disabled, teleop, autonomous).
    TelemetrySerializer telemetry;    ///< Binary telemetry packer streaming to operator station.

    // Five actuator channels
    TalonPubSub track_right_pub_sub;     ///< Right track drive motor IO.
    TalonPubSub track_left_pub_sub;      ///< Left track drive motor IO.
    TalonPubSub trencher_pub_sub;        ///< Excavation trencher motor IO.
    TalonPubSub hopper_belt_pub_sub;     ///< Hopper conveyor belt motor IO.
    TalonPubSub hopper_actuator_pub_sub; ///< Trencher tilt linear actuator IO.

    RclSubPtr<JoyMsg> joy_sub;                 ///< Subscriber for operator gamepad inputs.
    RclSubPtr<Int32Msg> watchdog_sub;          ///< Subscriber for mission control heartbeat.
    RclTimer::SharedPtr control_iteration_timer; ///< High-priority periodic loop timer (nominal 20 Hz).

    RobotMotorStatus robot_motor_status; ///< Latest aggregated actuator feedback.
    RobotMotorFaults robot_motor_faults; ///< Latest aggregated hardware faults.
    JoyMsg::ConstSharedPtr last_joy_msg{nullptr}; ///< Cached joystick packet from latest callback.
    RclTime last_joy_time;               ///< Timestamp when latest joystick packet arrived.
    JoyState joy_state;                  ///< Stateful button debouncer and edge detector.
    int32_t control_status{0};           ///< Current packed INT32 control status word.
};



// --- Implementation ---

/**
 * @def INIT_TALON_PUB_SUB
 * @brief Macro instantiating TalonPubSub with low-latency QoS for commands and sensor data QoS for feedback.
 */
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

    // Initialize all 5 motor publisher/subscriber channels
    INIT_TALON_PUB_SUB(track_right, track_right),
    INIT_TALON_PUB_SUB(track_left, track_left),
    INIT_TALON_PUB_SUB(trencher, trencher),
    INIT_TALON_PUB_SUB(hopper_belt, hopper_belt),
    INIT_TALON_PUB_SUB(hopper_act, hopper_actuator),

    // Gamepad input subscription
    joy_sub{this->create_subscription<JoyMsg>(
        lance::JOY_CTRL_TOPIC,
        rclcpp::SensorDataQoS{},
        [this](const JoyMsg::ConstSharedPtr& msg)
        {
            this->last_joy_msg = msg;
            this->last_joy_time = this->get_clock()->now();
        })},

    // Heartbeat and mode watchdog subscription
    watchdog_sub{this->create_subscription<Int32Msg>(
        lance::WATCHDOG_TOPIC,
        rclcpp::SensorDataQoS{},
        [this](const Int32Msg& status)
        { this->control_status = status.data; })},

    // Main periodic control iteration timer (default 20 Hz / 50ms)
    control_iteration_timer{this->create_wall_timer(
        std::chrono::duration<float>(
            this->robot_controller.getParams().iteration_period_seconds),
        [this]()
        {
            PROFILING_SYNC();
            PROFILING_NOTIFY_ALWAYS(iterate_control);

            // 1. Process joystick updates and monitor connection loss
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
                // Watchdog expired: zero joystick inputs
                this->joy_state.updateDisconnected();
            }

            // 2. Initialize command buffer (failsafe: disable all by default)
            RobotMotorCommands commands;
            commands.disableAll();

            // 3. Step high-level robot controller (state machine dispatch)
            this->robot_controller.iterate(
                this->control_status,
                this->joy_state,
                this->robot_motor_status,
                this->robot_motor_faults,
                commands);

            // 4. Dispatch computed motor setpoints to CTRE hardware drivers
            this->track_right_pub_sub.ctrl_pub->publish(commands.track_right);
            this->track_left_pub_sub.ctrl_pub->publish(commands.track_left);
            this->trencher_pub_sub.ctrl_pub->publish(commands.trencher);
            this->hopper_belt_pub_sub.ctrl_pub->publish(commands.hopper_belt);
            this->hopper_actuator_pub_sub.ctrl_pub->publish(
                commands.hopper_actuator);

            // 5. Serialize full robot state into telemetry chunk for operator UI
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
