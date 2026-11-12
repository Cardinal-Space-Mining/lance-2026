// Lesson 6 reference solution: mining_stats_node
//
// Tracks statistics for a mining session (time digging, trencher energy,
// number of dumps...) and publishes them as a custom MiningStats message on
// ~/stats. A custom ResetStats service on ~/reset returns the final stats and
// starts a new session.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <optional>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/float32.hpp>
#include <phoenix_ros_driver/msg/talon_info.hpp>

// Generated from msg/MiningStats.msg and srv/ResetStats.srv
// (CamelCase file names become snake_case headers)
#include <mining_stats_interfaces/msg/mining_stats.hpp>
#include <mining_stats_interfaces/srv/reset_stats.hpp>

using namespace std::chrono_literals;

using Float32Msg = std_msgs::msg::Float32;
using TalonInfoMsg = phoenix_ros_driver::msg::TalonInfo;
using MiningStatsMsg = mining_stats_interfaces::msg::MiningStats;
using ResetStatsSrv = mining_stats_interfaces::srv::ResetStats;


namespace
{

const char* stateName(uint8_t state)
{
    switch (state)
    {
        case MiningStatsMsg::STATE_IDLE:
            return "IDLE";
        case MiningStatsMsg::STATE_DIGGING:
            return "DIGGING";
        case MiningStatsMsg::STATE_DUMPING:
            return "DUMPING";
        default:
            return "UNKNOWN";
    }
}

}  // namespace


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

        // --- Outputs ---
        // "~/" puts the name under this node's name: /mining_stats/stats
        this->stats_pub =
            this->create_publisher<MiningStatsMsg>("~/stats", rclcpp::QoS(10));

        this->reset_srv = this->create_service<ResetStatsSrv>(
            "~/reset",
            [this](
                const std::shared_ptr<ResetStatsSrv::Request> request,
                std::shared_ptr<ResetStatsSrv::Response> response)
            { this->onReset(*request, *response); });

        this->timer = this->create_wall_timer(100ms, [this]() { this->step(); });

        RCLCPP_INFO(this->get_logger(), "Mining stats started");
    }

private:
    uint8_t classifyState(double actuator_pos, double trencher_rps) const
    {
        if (actuator_pos > this->dump_above_pos)
        {
            return MiningStatsMsg::STATE_DUMPING;
        }
        if (actuator_pos < this->dig_below_pos &&
            std::abs(trencher_rps) > this->trencher_min_rps)
        {
            return MiningStatsMsg::STATE_DIGGING;
        }
        return MiningStatsMsg::STATE_IDLE;
    }

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

        // --- State, and dump counting on the "rising edge" ---
        const uint8_t new_state =
            this->classifyState(this->actuator->position, this->trencher->velocity);
        if (new_state != this->state)
        {
            RCLCPP_INFO(
                this->get_logger(),
                "State: %s -> %s",
                stateName(this->state),
                stateName(new_state));

            // Count a dump when we ENTER the dumping state, not every step
            // that we're in it
            if (new_state == MiningStatsMsg::STATE_DUMPING)
            {
                this->dump_count++;
            }
            this->state = new_state;
        }

        // --- Accumulate ---
        this->session_seconds += dt;
        if (this->state == MiningStatsMsg::STATE_DIGGING)
        {
            this->digging_seconds += dt;
        }

        // Power (W) = voltage (V) x current (A). Energy (Wh) = W x hours.
        const double power_w =
            this->trencher->bus_voltage * std::abs(this->trencher->supply_current);
        this->trencher_energy_wh += power_w * (dt / 3600.0);
        this->trencher_peak_current_a = std::max(
            this->trencher_peak_current_a,
            static_cast<double>(std::abs(this->trencher->supply_current)));

        this->stats_pub->publish(this->buildStatsMsg());
    }

    MiningStatsMsg buildStatsMsg() const
    {
        MiningStatsMsg msg;
        msg.header.stamp = this->now();
        msg.state = this->state;
        msg.session_seconds = static_cast<float>(this->session_seconds);
        msg.digging_seconds = static_cast<float>(this->digging_seconds);
        msg.dump_count = this->dump_count;
        msg.trencher_energy_wh = static_cast<float>(this->trencher_energy_wh);
        msg.trencher_peak_current_a = static_cast<float>(this->trencher_peak_current_a);
        msg.hopper_fullness = this->hopper_fullness.value_or(NAN);
        return msg;
    }

    void onReset(const ResetStatsSrv::Request& request, ResetStatsSrv::Response& response)
    {
        // Answer with the stats as they were, THEN clear them
        response.final_stats = this->buildStatsMsg();

        RCLCPP_INFO(
            this->get_logger(),
            "Resetting stats (%s). Final: %.1f s digging, %u dumps, %.2f Wh",
            request.reason.empty() ? "no reason given" : request.reason.c_str(),
            this->digging_seconds,
            this->dump_count,
            this->trencher_energy_wh);

        this->session_seconds = 0.0;
        this->digging_seconds = 0.0;
        this->dump_count = 0;
        this->trencher_energy_wh = 0.0;
        this->trencher_peak_current_a = 0.0;
        // Keep `state`: the robot is still doing whatever it was doing
    }

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

    // Outputs
    rclcpp::Publisher<MiningStatsMsg>::SharedPtr stats_pub;
    rclcpp::Service<ResetStatsSrv>::SharedPtr reset_srv;
    rclcpp::TimerBase::SharedPtr timer;

    // Session statistics
    uint8_t state{MiningStatsMsg::STATE_IDLE};
    double session_seconds{0.0};
    double digging_seconds{0.0};
    uint32_t dump_count{0};
    double trencher_energy_wh{0.0};
    double trencher_peak_current_a{0.0};
    std::optional<rclcpp::Time> last_step_time;
};


int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MiningStatsNode>());
    rclcpp::shutdown();
    return 0;
}
