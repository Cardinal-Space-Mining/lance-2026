// Lesson 1 reference solution: motor_monitor_node
//
// Subscribes to the TalonInfo telemetry of each LANCE motor, tracks current
// draw, warns when a motor goes over a configurable limit, and publishes the
// total supply current so it can be plotted in Foxglove.

#include <chrono>
#include <cstddef>
#include <cstdio>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/float32.hpp>
#include <phoenix_ros_driver/msg/talon_info.hpp>

using namespace std::chrono_literals;

using Float32Msg = std_msgs::msg::Float32;
using TalonInfoMsg = phoenix_ros_driver::msg::TalonInfo;


// Everything we remember about one motor.
struct MotorStats
{
    std::string name;
    float supply_current{0.f};       // amps, most recent message
    float peak_supply_current{0.f};  // amps, highest seen since startup
    float bus_voltage{0.f};          // volts, most recent message
    double velocity{0.};             // rotations per second
    bool enabled{false};
    std::size_t msg_count{0};
};


class MotorMonitorNode : public rclcpp::Node
{
public:
    MotorMonitorNode() : Node("motor_monitor")
    {
        // --- Parameters -----------------------------------------------------
        const auto motor_names = this->declare_parameter<std::vector<std::string>>(
            "motors",
            {"track_left", "track_right", "trencher", "hopper_belt", "hopper_act"});
        this->current_warn_amps =
            this->declare_parameter<double>("current_warn_amps", 80.0);

        // --- Subscriptions --------------------------------------------------
        // Size the vector up front. The callbacks below remember an *index*
        // into this vector, which stays valid; a pointer or reference would
        // be left dangling if the vector ever reallocated.
        this->motors.resize(motor_names.size());

        for (std::size_t i = 0; i < motor_names.size(); i++)
        {
            this->motors[i].name = motor_names[i];

            // e.g. "lance/track_left/info" (see TALON_INFO_TOPIC in
            // lance/src/robot/core/ros_interface.hpp)
            const std::string topic = "lance/" + motor_names[i] + "/info";

            // SensorDataQoS is "best effort". The motor sim and the real
            // phoenix driver publish best effort, and a default (reliable)
            // subscription would never receive anything from them.
            this->info_subs.push_back(this->create_subscription<TalonInfoMsg>(
                topic,
                rclcpp::SensorDataQoS(),
                [this, i](const TalonInfoMsg& msg) { this->onMotorInfo(i, msg); }));

            RCLCPP_INFO(this->get_logger(), "Monitoring %s", topic.c_str());
        }

        // --- Publishers -----------------------------------------------------
        this->total_current_pub = this->create_publisher<Float32Msg>(
            "motor_monitor/total_current",
            rclcpp::SensorDataQoS());

        // --- Timers ---------------------------------------------------------
        this->publish_timer =
            this->create_wall_timer(100ms, [this]() { this->publishTotal(); });
        this->report_timer =
            this->create_wall_timer(2s, [this]() { this->printReport(); });
    }

private:
    void onMotorInfo(std::size_t i, const TalonInfoMsg& msg)
    {
        MotorStats& m = this->motors[i];

        m.supply_current = msg.supply_current;
        m.bus_voltage = msg.bus_voltage;
        m.velocity = msg.velocity;
        m.enabled = (msg.status & TalonInfoMsg::STATUS_PHX_ENABLED);
        m.msg_count++;

        if (m.supply_current > m.peak_supply_current)
        {
            m.peak_supply_current = m.supply_current;
        }

        if (m.supply_current > this->current_warn_amps)
        {
            // Telemetry arrives at 50Hz, so throttle to at most once per second
            RCLCPP_WARN_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                1000,
                "%s is drawing %.1f A (limit %.1f A)",
                m.name.c_str(),
                m.supply_current,
                this->current_warn_amps);
        }
    }

    void publishTotal()
    {
        float total = 0.f;
        for (const MotorStats& m : this->motors)
        {
            total += m.supply_current;
        }

        Float32Msg msg;
        msg.data = total;
        this->total_current_pub->publish(msg);
    }

    void printReport()
    {
        std::string report = "\n  motor          msgs   enabled   vel(rps)   I(A)   peak I(A)   bus(V)";
        for (const MotorStats& m : this->motors)
        {
            char line[128];
            std::snprintf(
                line,
                sizeof(line),
                "\n  %-12s %6zu   %-7s %9.2f %7.2f %10.2f %8.2f",
                m.name.c_str(),
                m.msg_count,
                m.enabled ? "yes" : "no",
                m.velocity,
                m.supply_current,
                m.peak_supply_current,
                m.bus_voltage);
            report += line;
        }
        RCLCPP_INFO(this->get_logger(), "%s", report.c_str());
    }

private:
    std::vector<MotorStats> motors;
    double current_warn_amps{0.};

    std::vector<rclcpp::Subscription<TalonInfoMsg>::SharedPtr> info_subs;
    rclcpp::Publisher<Float32Msg>::SharedPtr total_current_pub;
    rclcpp::TimerBase::SharedPtr publish_timer;
    rclcpp::TimerBase::SharedPtr report_timer;
};


int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MotorMonitorNode>());
    rclcpp::shutdown();
    return 0;
}
