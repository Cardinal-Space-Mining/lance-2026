// Lesson 4: hopper_sensor_sim_node
//
// A C++ port of lance/scripts/hopper_fullness.py that publishes hopper
// fullness on "lance/hopper_fullness" as a fraction: 0.0 = empty, 1.0 = full.
// Instead of a serial port, it gets its data from one of two sources, chosen with the "source" parameter:
//   source:=file  replays a log of serial output, one line per tick (Part 3)
//   source:=sim   simulates the hopper filling and emptying (Part 4)

#include <algorithm>
#include <chrono>
#include <cmath>
#include <fstream>
#include <memory>
#include <optional>
#include <random>
#include <stdexcept>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/float32.hpp>
#include <phoenix_ros_driver/msg/talon_info.hpp>

#include "hopper_sensor_sim/fullness_parser.hpp"

using namespace std::chrono_literals;

using Float32Msg = std_msgs::msg::Float32;
using TalonInfoMsg = phoenix_ros_driver::msg::TalonInfo;


class HopperSensorSimNode : public rclcpp::Node
{
public:
    HopperSensorSimNode() : Node("hopper_sensor_sim")
    {
        // TODO (Part 3): create the publisher. Use the same topic and QoS as
        //   the Python node.

        // TODO (Part 3): declare the "source" parameter (default "sim") and
        //   call setupFileSource() or setupSimSource(). For any other value,
        //   throw std::invalid_argument with a helpful message.

        RCLCPP_INFO(this->get_logger(), "Hopper sensor started!");
    }

private:
    // --- Source 1: replay a log file (Part 3) --------------------------------

    // TODO (Part 3): setupFileSource(): open the file named by the
    //   "replay_file" parameter, and create a timer (period from the
    //   "replay_period_s" parameter, default 1.0) that calls replayNextLine().

    // TODO (Part 3): replayNextLine(): read ONE line, parse it, and publish it,
    //   or warn and skip it. At the end of the file, rewind and loop.

    // --- Source 2: simulated hopper (Part 4) ---------------------------------

    // TODO (Part 4): setupSimSource(): parameters, three subscriptions, and a
    //   100ms timer that calls stepSimulation().

    // TODO (Part 4): stepSimulation(): integrate the hopper volume over time
    //   and publish fullness (volume / capacity), with noise.

private:
    // TODO: member variables
};


int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    try
    {
        rclcpp::spin(std::make_shared<HopperSensorSimNode>());
    }
    catch (const std::exception& e)
    {
        // Anything thrown while building or running the node ends up here, so
        // we can print a clear message instead of crashing
        RCLCPP_FATAL(rclcpp::get_logger("hopper_sensor_sim"), "%s", e.what());
        rclcpp::shutdown();
        return 1;
    }
    rclcpp::shutdown();
    return 0;
}
