// Lesson 6: mining_stats_node
//
// Tracks statistics for a mining session and publishes them as a custom
// MiningStats message on ~/stats. A custom ResetStats service on ~/reset
// returns the final stats and starts a new session.
//
// The input handling and time-step code is done for you. It's the same pattern
// as Lesson 4. Your work is in the TODOs.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <optional>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/float32.hpp>
#include <phoenix_ros_driver/msg/talon_info.hpp>

// TODO (Part 2): include your generated message and service headers

using namespace std::chrono_literals;

using Float32Msg = std_msgs::msg::Float32;
using TalonInfoMsg = phoenix_ros_driver::msg::TalonInfo;
// TODO (Part 2): add aliases MiningStatsMsg and ResetStatsSrv


class MiningStatsNode : public rclcpp::Node
{
public:
    MiningStatsNode() : Node("mining_stats")
    {
        this->dig_below_pos = this->declare_parameter<double>("dig_below_pos", 0.35);
        this->dump_above_pos = this->declare_parameter<double>("dump_above_pos", 0.8);
        this->trencher_min_rps =
            this->declare_parameter<double>("trencher_min_rps", 1.0);

        // --- Inputs: just remember the latest message ---
        this->trencher_sub = this->create_subscription<TalonInfoMsg>(
            "lance/trencher/info",
            rclcpp::SensorDataQoS(),
            [this](const TalonInfoMsg& msg) { this->trencher = msg; });
        this->actuator_sub = this->create_subscription<TalonInfoMsg>(
            "lance/hopper_act/info",
            rclcpp::SensorDataQoS(),
            [this](const TalonInfoMsg& msg) { this->actuator = msg; });
        this->fullness_sub = this->create_subscription<Float32Msg>(
            "lance/hopper_fullness",
            rclcpp::SensorDataQoS(),
            [this](const Float32Msg& msg) { this->hopper_fullness = msg.data; });

        // TODO (Part 3): create the publisher on "~/stats"
        // TODO (Part 4): create the service on "~/reset"

        this->timer = this->create_wall_timer(100ms, [this]() { this->step(); });

        RCLCPP_INFO(this->get_logger(), "Mining stats started");
    }

private:
    // TODO (Part 3): uint8_t classifyState(double actuator_pos, double trencher_rps) const

    void step()
    {
        if (!this->trencher || !this->actuator)
        {
            RCLCPP_INFO_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                5000,
                "Waiting for motor telemetry...");
            return;
        }

        // Elapsed (sim) time since the last step, same as Lesson 4
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
            return;
        }

        // TODO (Part 3): update the state, count dumps, accumulate the
        //   statistics, and publish a MiningStats message
        (void)dt;  // remove once you use dt
    }

    // TODO (Part 3): MiningStatsMsg buildStatsMsg() const
    // TODO (Part 4): void onReset(const ResetStatsSrv::Request&, ResetStatsSrv::Response&)

private:
    // Parameters
    double dig_below_pos{0.35};
    double dump_above_pos{0.8};
    double trencher_min_rps{1.0};

    // Inputs
    rclcpp::Subscription<TalonInfoMsg>::SharedPtr trencher_sub;
    rclcpp::Subscription<TalonInfoMsg>::SharedPtr actuator_sub;
    rclcpp::Subscription<Float32Msg>::SharedPtr fullness_sub;
    std::optional<TalonInfoMsg> trencher;
    std::optional<TalonInfoMsg> actuator;
    std::optional<float> hopper_fullness;

    rclcpp::TimerBase::SharedPtr timer;
    std::optional<rclcpp::Time> last_step_time;

    // TODO (Part 3): publisher and session statistics
    // TODO (Part 4): service
};


int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MiningStatsNode>());
    rclcpp::shutdown();
    return 0;
}
