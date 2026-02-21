#pragma once

#include <string>
#include <zenoh.hxx>
#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/int8.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>

#include "manifest.hpp"
#include "ros_utils.hpp"
#include "zenoh_utils.hpp"

// All Adapters
#include "adapters/joy_adapter.hpp"
#include "adapters/talon_adapter.hpp"
#include "adapters/generic_adapter.hpp"
#include "adapters/ms136_imu_adapter.hpp"
#include "adapters/ms136_scan_adapter.hpp"
#include "adapters/path_adapter.hpp"

enum Flow
{
    INCOMING,
    OUTGOING
};

#ifdef IS_ROBOT
    #define NODE_NAME  "robot_redux_endpoint"
    #define IP_PARAM   "client_hostname"
    #define DEFAULT_IP "10.11.11.8"
#else
    #define NODE_NAME  "client_redux_endpoint"
    #define IP_PARAM   "robot_hostname"
    #define DEFAULT_IP "10.11.11.10"
#endif

// resolves the correct type (Pub or Sub) at compile time
template<typename Adapter, Flow dir>
struct NetTraits
{
#ifdef IS_ROBOT
    using Role = std::conditional_t<
        dir == INCOMING,
        typename Adapter::Subscriber,
        typename Adapter::Publisher>;
#else
    using Role = std::conditional_t<
        dir == INCOMING,
        typename Adapter::Publisher,
        typename Adapter::Subscriber>;
#endif
};

template<typename Adapter, Flow dir>
static auto make_member(
    rclcpp::Node& node,
    zenoh::Session& zsh,
    const std::string& topic)
{
#ifdef IS_ROBOT
    if constexpr (dir == INCOMING)
    {
        return Adapter::createSubscriber(node, zsh, topic);
    }
    else
    {
        return Adapter::createPublisher(node, zsh, topic);
    }
#else
    if constexpr (dir == INCOMING)
    {
        return Adapter::createPublisher(node, zsh, topic);
    }
    else
    {
        return Adapter::createSubscriber(node, zsh, topic);
    }
#endif
}

class EndpointNode : public rclcpp::Node
{
    using StdInt8Adapter = GenericAdapter<std_msgs::msg::Int8>;
    using StdInt32Adapter = GenericAdapter<std_msgs::msg::Int32>;
    using StdStringAdapter = GenericAdapter<std_msgs::msg::String>;
    using PointStampedAdapter =
        GenericAdapter<geometry_msgs::msg::PointStamped>;

public:
#define INIT_MEMBER(topic, adapter, slug, dir)               \
    , m_##slug{make_member<adapter, dir>(*this, zsh, topic)}

    EndpointNode() :
        Node{NODE_NAME},
        zsh{zenoh::Session::open(
            util::configDirectConnectTo(
                util::declare_and_get_param<std::string>(
                    *this,
                    IP_PARAM,
                    DEFAULT_IP)))}  //
        NETWORK_MANIFEST(INIT_MEMBER)
    {
        RCLCPP_INFO(this->get_logger(), "Bridge Started");
    }
#undef INIT_MEMBER

private:
    zenoh::Session zsh;

#define DECLARE(topic, adapter, slug, dir)           \
    typename NetTraits<adapter, dir>::Role m_##slug;

    NETWORK_MANIFEST(DECLARE)
#undef DECLARE
};
