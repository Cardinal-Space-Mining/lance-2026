// Lesson 1: motor_monitor_node
//
// Goal: subscribe to the telemetry (TalonInfo) of each LANCE motor, track
// current draw, warn when a motor goes over a limit, and publish the total
// current so it can be plotted in Foxglove.
//
// Work through the TODOs in order, following the lesson README. Build and run
// after every step; each one should do something you can see.

#include <chrono>
#include <cstddef>
#include <cstdio>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

// TODO (Step 2): include the TalonInfo message header.
//   Message `phoenix_ros_driver/msg/TalonInfo` lives in the header
//   <phoenix_ros_driver/msg/talon_info.hpp> (CamelCase -> snake_case).

using namespace std::chrono_literals;


// TODO (Step 3): define a `MotorStats` struct holding what we want to remember
// about one motor: its name, latest supply current, peak supply current, bus
// voltage, velocity, whether it is enabled, and how many messages we've seen.


class MotorMonitorNode : public rclcpp::Node
{
public:
    MotorMonitorNode() : Node("motor_monitor")
    {
        RCLCPP_INFO(this->get_logger(), "Motor monitor started!");

        // TODO (Step 2): subscribe to "lance/track_left/info" and print the
        //   velocity of every message you receive.

        // TODO (Step 3): replace the single subscription with one per motor,
        //   using a "motors" parameter for the list of motor names.

        // TODO (Step 4): declare a "current_warn_amps" parameter.

        // TODO (Step 5): create the total current publisher and the timers.
    }

private:
    // TODO (Step 3): write onMotorInfo(index, msg) to update MotorStats.
    // TODO (Step 4): warn (throttled!) when current exceeds the limit.
    // TODO (Step 5): write publishTotal() and printReport().

private:
    // TODO: member variables (subscriptions, publisher, timers, stats...)
};


int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MotorMonitorNode>());
    rclcpp::shutdown();
    return 0;
}
