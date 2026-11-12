// Lesson 4 reference solution: hopper_sensor_sim_node
//
// A C++ port of lance/scripts/hopper_fullness.py that publishes hopper
// fullness on "lance/hopper_fullness" as a fraction: 0.0 = empty, 1.0 = full
// (the Arduino sends its percentage divided by 100). Instead of a serial port,
// it gets its
// data from one of two sources:
//   source:=file  replays a log of serial output, one line per tick (like the
//                 Python node reading one line per second)
//   source:=sim   simulates the hopper filling and emptying, based on the
//                 trencher, belt, and hopper actuator telemetry from motor_sim

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
        // Same topic and QoS as the Python node, so everything downstream
        // (Foxglove, net-adapter) works without changes
        this->fullness_pub = this->create_publisher<Float32Msg>(
            "lance/hopper_fullness",
            rclcpp::SensorDataQoS());

        const std::string source =
            this->declare_parameter<std::string>("source", "sim");
        if (source == "file")
        {
            this->setupFileSource();
        }
        else if (source == "sim")
        {
            this->setupSimSource();
        }
        else
        {
            // Can't continue without a valid source. main() catches this.
            throw std::invalid_argument(
                "Parameter 'source' must be \"file\" or \"sim\", got \"" +
                source + "\"");
        }
    }

private:
    // --- Source 1: replay a log file -----------------------------------------

    void setupFileSource()
    {
        const std::string path =
            this->declare_parameter<std::string>("replay_file", "");
        const double period_s =
            this->declare_parameter<double>("replay_period_s", 1.0);

        this->replay_file.open(path);
        if (!this->replay_file.is_open())
        {
            throw std::runtime_error("Could not open replay_file \"" + path + "\"");
        }

        this->timer = this->create_wall_timer(
            std::chrono::duration<double>(period_s),
            [this]() { this->replayNextLine(); });

        RCLCPP_INFO(this->get_logger(), "Replaying %s", path.c_str());
    }

    void replayNextLine()
    {
        std::string line;
        if (!std::getline(this->replay_file, line))
        {
            // End of file: start again from the beginning
            RCLCPP_INFO(this->get_logger(), "End of replay file, looping");
            this->replay_file.clear();  // clear the end-of-file flag...
            this->replay_file.seekg(0); // ...and rewind
            return;
        }

        // Serial devices end lines with "\r\n", but getline only removes the
        // "\n". A stray "\r" in a log message moves the terminal cursor back to
        // the start of the line and garbles the output, so remove it.
        if (!line.empty() && line.back() == '\r')
        {
            line.pop_back();
        }

        const std::optional<float> value =
            hopper_sensor_sim::parseFullnessLine(line);
        if (!value)
        {
            RCLCPP_WARN(this->get_logger(), "Skipping invalid line: \"%s\"", line.c_str());
            return;
        }

        RCLCPP_INFO(this->get_logger(), "%s --> %.2f", line.c_str(), *value);
        this->publish(*value);
    }

    // --- Source 2: simulated hopper ------------------------------------------

    void setupSimSource()
    {
        this->capacity_l = this->declare_parameter<double>("capacity_l", 50.0);
        this->fill_l_per_trencher_rev =
            this->declare_parameter<double>("fill_l_per_trencher_rev", 0.005);
        this->drain_l_per_belt_rev =
            this->declare_parameter<double>("drain_l_per_belt_rev", 0.03);
        this->dig_below_pos = this->declare_parameter<double>("dig_below_pos", 0.35);
        this->dump_above_pos = this->declare_parameter<double>("dump_above_pos", 0.8);
        this->noise_stddev = this->declare_parameter<double>("noise_stddev", 0.01);

        // Only the latest message from each motor matters, so we just store it
        this->trencher_sub = this->create_subscription<TalonInfoMsg>(
            "lance/trencher/info",
            rclcpp::SensorDataQoS(),
            [this](const TalonInfoMsg& msg) { this->trencher = msg; });
        this->belt_sub = this->create_subscription<TalonInfoMsg>(
            "lance/hopper_belt/info",
            rclcpp::SensorDataQoS(),
            [this](const TalonInfoMsg& msg) { this->belt = msg; });
        this->actuator_sub = this->create_subscription<TalonInfoMsg>(
            "lance/hopper_act/info",
            rclcpp::SensorDataQoS(),
            [this](const TalonInfoMsg& msg) { this->actuator = msg; });

        this->timer =
            this->create_wall_timer(100ms, [this]() { this->stepSimulation(); });

        RCLCPP_INFO(this->get_logger(), "Simulating a %.1f L hopper", this->capacity_l);
    }

    void stepSimulation()
    {
        // Wait until every motor has reported at least once
        if (!this->trencher || !this->belt || !this->actuator)
        {
            RCLCPP_INFO_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                5000,
                "Waiting for motor telemetry...");
            return;
        }

        // How much (simulated) time has passed since the last step?
        const rclcpp::Time now = this->now();
        if (!this->last_step_time)
        {
            this->last_step_time = now;
            return;
        }
        const double dt = (now - *this->last_step_time).seconds();
        this->last_step_time = now;
        if (dt <= 0.0)
        {
            return;  // sim time paused, or jumped backwards
        }

        const double act_pos = this->actuator->position;  // 0 = down, 1 = up
        const double trencher_rps = std::abs(this->trencher->velocity);
        const double belt_rps = std::abs(this->belt->velocity);

        // Euler integration: rate * dt = change
        double fill_rate = 0.0;   // liters per second
        double drain_rate = 0.0;  // liters per second
        if (act_pos < this->dig_below_pos)
        {
            fill_rate = trencher_rps * this->fill_l_per_trencher_rev;
        }
        if (act_pos > this->dump_above_pos)
        {
            drain_rate = belt_rps * this->drain_l_per_belt_rev;
        }
        this->volume_l += (fill_rate - drain_rate) * dt;
        this->volume_l = std::clamp(this->volume_l, 0.0, this->capacity_l);

        // Publish fullness as a fraction of capacity, like the real sensor.
        // A real sensor is noisy, so add some noise to what we publish (but
        // not to the true volume).
        const double fullness = this->volume_l / this->capacity_l;
        std::normal_distribution<double> noise{0.0, this->noise_stddev};
        this->publish(static_cast<float>(fullness + noise(this->rng)));
    }

    // --- Shared ----------------------------------------------------------

    void publish(float value)
    {
        Float32Msg msg;
        msg.data = value;
        this->fullness_pub->publish(msg);
    }

private:
    rclcpp::Publisher<Float32Msg>::SharedPtr fullness_pub;
    rclcpp::TimerBase::SharedPtr timer;

    // File source
    std::ifstream replay_file;

    // Sim source: parameters
    double capacity_l{50.0};
    double fill_l_per_trencher_rev{0.005};
    double drain_l_per_belt_rev{0.03};
    double dig_below_pos{0.35};
    double dump_above_pos{0.8};
    double noise_stddev{0.01};

    // Sim source: inputs
    rclcpp::Subscription<TalonInfoMsg>::SharedPtr trencher_sub;
    rclcpp::Subscription<TalonInfoMsg>::SharedPtr belt_sub;
    rclcpp::Subscription<TalonInfoMsg>::SharedPtr actuator_sub;
    std::optional<TalonInfoMsg> trencher;
    std::optional<TalonInfoMsg> belt;
    std::optional<TalonInfoMsg> actuator;

    // Sim source: state
    double volume_l{0.0};
    std::optional<rclcpp::Time> last_step_time;
    std::mt19937 rng{std::random_device{}()};
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
        RCLCPP_FATAL(rclcpp::get_logger("hopper_sensor_sim"), "%s", e.what());
        rclcpp::shutdown();
        return 1;
    }
    rclcpp::shutdown();
    return 0;
}
