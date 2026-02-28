#pragma once
#include <string>
#include <zenoh.hxx>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int8.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <rosgraph_msgs/msg/clock.hpp>
#include "ros_utils.hpp"
#include "zenoh_utils.hpp"
#include "adapters/joy_adapter.hpp"
#include "adapters/talon_adapter.hpp"
#include "adapters/generic_adapter.hpp"
#include "adapters/ms136_imu_adapter.hpp"
#include "adapters/ms136_scan_adapter.hpp"
#include "adapters/path_adapter.hpp"

enum class NodeRole
{
    Robot,
    Client
};
enum Flow
{
    INCOMING,
    OUTGOING
};

template<NodeRole Role, typename Adapter, Flow dir>
struct NetTraits
{
    using Type = std::conditional_t<
        (Role == NodeRole::Robot) == (dir == INCOMING),
        typename Adapter::Subscriber,
        typename Adapter::Publisher>;
};

template<NodeRole Role, typename Adapter, Flow dir>
auto make_member(rclcpp::Node& node, zenoh::Session& zsh, const char* topic)
{
    constexpr bool isSub = (Role == NodeRole::Robot) == (dir == INCOMING);
    if constexpr (isSub)
    {
        return Adapter::createSubscriber(node, zsh, topic);
    }
    else
    {
        return Adapter::createPublisher(node, zsh, topic);
    }
}

template<NodeRole Role, typename Adapter, Flow dir>
struct Member
{
    typename NetTraits<Role, Adapter, dir>::Type inner;

    Member(rclcpp::Node& node, zenoh::Session& zsh, const char* topic) :
        inner{make_member<Role, Adapter, dir>(node, zsh, topic)}
    {
    }
};

template<NodeRole Role>
class EndpointNode : public rclcpp::Node
{
    using StdInt8Adapter = GenericAdapter<std_msgs::msg::Int8>;
    using StdInt32Adapter = GenericAdapter<std_msgs::msg::Int32>;
    using StdStringAdapter = GenericAdapter<std_msgs::msg::String>;
    using PointStampedAdapter =
        GenericAdapter<geometry_msgs::msg::PointStamped>;
    using ClockAdapter = GenericAdapter<rosgraph_msgs::msg::Clock>;

    template<typename Adapter, Flow dir>
    using M = Member<Role, Adapter, dir>;

    static constexpr auto node_name = Role == NodeRole::Robot
                                          ? "robot_redux_endpoint"
                                          : "client_redux_endpoint";
    static constexpr auto ip_param =
        Role == NodeRole::Robot ? "client_hostname" : "robot_hostname";
    static constexpr auto default_ip =
        Role == NodeRole::Robot ? "10.11.11.8" : "10.11.11.10";

    struct TalonMotor
    {
        M<TalonInfoAdapter, OUTGOING> info;
        M<TalonFaultsAdapter, OUTGOING> faults;

        TalonMotor(rclcpp::Node& node, zenoh::Session& zsh, const char* base) :
            info{node, zsh, (std::string(base) + "/info").c_str()},
            faults{node, zsh, (std::string(base) + "/faults").c_str()}
        {
        }
    };

    // Creates the correct scan adapter at runtime based on is_sim,
    // stored as shared_ptr<void> since the two types are unrelated
    static std::shared_ptr<void>
        make_scan(rclcpp::Node& node, zenoh::Session& zsh, bool is_sim)
    {
        if (is_sim)
        {
            if constexpr (Role == NodeRole::Robot)
            {
                return MS136SimScanAdapter::createSharedSubscriber(
                    node,
                    zsh,
                    "multiscan/lidar_scan");
            }
            else
            {
                return MS136SimScanAdapter::createSharedPublisher(
                    node,
                    zsh,
                    "multiscan/lidar_scan");
            }
        }
        else
        {
            if constexpr (Role == NodeRole::Robot)
            {
                return MS136ScanAdapter::createSharedSubscriber(
                    node,
                    zsh,
                    "multiscan/lidar_scan");
            }
            else
            {
                return MS136ScanAdapter::createSharedPublisher(
                    node,
                    zsh,
                    "multiscan/lidar_scan");
            }
        }
    }

    // Creates clock adapter only when is_sim, nullptr otherwise
    static std::shared_ptr<void>
        make_clock(rclcpp::Node& node, zenoh::Session& zsh, bool is_sim)
    {
        if (!is_sim)
        {
            return nullptr;
        }
        if constexpr (Role == NodeRole::Robot)
        {
            return ClockAdapter::createSharedSubscriber(node, zsh, "/clock");
        }
        else
        {
            return ClockAdapter::createSharedPublisher(node, zsh, "/clock");
        }
    }

public:
    EndpointNode() :
        Node{node_name},
        zsh{zenoh::Session::open(
            util::configDirectConnectTo(
                util::declare_and_get_param<std::string>(
                    *this,
                    ip_param,
                    default_ip)))},
        is_sim{util::declare_and_get_param(*this, "is_sim", false)},

        // Control (Client -> Robot)
        joy{*this, zsh, "/joy"},
        watchdog_status{*this, zsh, "lance/watchdog_status"},
        clicked_point{*this, zsh, "/clicked_point"},

        // Data (Robot -> Client)
        imu{*this, zsh, "multiscan/imu"},
        lidar_scan{make_scan(*this, zsh, is_sim)},
        path{*this, zsh, "cardinal_perception/planned_path"},
        relay_status{*this, zsh, "lance/relay_status"},
        op_status{*this, zsh, "lance/op_status"},

        // Talon Motors
        track_left{*this, zsh, "lance/track_left"},
        track_right{*this, zsh, "lance/track_right"},
        trencher{*this, zsh, "lance/trencher"},
        hopper_belt{*this, zsh, "lance/hopper_belt"},
        hopper_act{*this, zsh, "lance/hopper_act"},

        // Sim-only
        sim_clock{make_clock(*this, zsh, is_sim)}
    {
        RCLCPP_INFO(this->get_logger(), "Bridge Started");
    }

private:
    zenoh::Session zsh;
    const bool is_sim;
    // Control (Client -> Robot)
    M<JoyAdapter, INCOMING> joy;
    M<StdInt32Adapter, INCOMING> watchdog_status;
    M<PointStampedAdapter, INCOMING> clicked_point;
    // Data (Robot -> Client)
    M<MS136ImuAdapter, OUTGOING> imu;
    std::shared_ptr<void> lidar_scan;  // OUTGOING
    M<PathAdapter, OUTGOING> path;
    M<StdInt8Adapter, OUTGOING> relay_status;
    M<StdStringAdapter, OUTGOING> op_status;

    // Talon Motors
    TalonMotor track_left;
    TalonMotor track_right;
    TalonMotor trencher;
    TalonMotor hopper_belt;
    TalonMotor hopper_act;

    // Sim-only
    std::shared_ptr<void> sim_clock;  // OUTGOING
};
