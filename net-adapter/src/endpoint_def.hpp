/*******************************************************************************
*   Copyright (C) 2024-2026 Cardinal Space Mining Club                         *
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

#pragma once

#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include <zenoh.hxx>
#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/int8.hpp>
#include <std_msgs/msg/int32.hpp>
#include <rosgraph_msgs/msg/clock.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>

#include <net_adapter/msg/bytes.hpp>

#include "util/ros_utils.hpp"
#include "util/zenoh_utils.hpp"
#include "util/delay_queue.hpp"

#include "adapters/joy_adapter.hpp"
#include "adapters/talon_adapter.hpp"
#include "adapters/generic_adapter.hpp"
#include "adapters/ms136_imu_adapter.hpp"
#include "adapters/ms136_scan_adapter.hpp"

#include "endpoint_traits.hpp"


#define DEFAULT_ROBOT_HOSTNAME  "10.11.11.15"
#define DEFAULT_CLIENT_HOSTNAME "10.11.11.1"

#ifndef ENABLE_NET_DELAY
    #define ENABLE_NET_DELAY 1
#endif


template<EndPoint E>
class EndPointNode : public rclcpp::Node
{
public:
    EndPointNode();

private:
    using StdInt8Adapter = GenericAdapter<std_msgs::msg::Int8>;
    using StdInt32Adapter = GenericAdapter<std_msgs::msg::Int32>;
    using ClockAdapter = GenericAdapter<rosgraph_msgs::msg::Clock>;
    using PointStampedAdapter =
        GenericAdapter<geometry_msgs::msg::PointStamped>;
    using BytesAdapter = GenericAdapter<net_adapter::msg::Bytes>;

private:
    template<typename AdapterT, DataFlow D>
    using ApTraits = AdapterTraits<AdapterT, D, E>;
    template<typename AdapterT, DataFlow D>
    using Channel = typename ApTraits<AdapterT, D>::ChannelT;

    constexpr static bool Is_Robot = (E == ROBOT_ENDPOINT);
    constexpr static bool Is_Client = (E == CLIENT_ENDPOINT);

private:
    constexpr static char const* Node_Name =
        (Is_Robot ? "robot_redux_endpoint" : "client_redux_endpoint");
    constexpr static char const* Connection_Param_Name =
        (Is_Robot ? "client_hostname" : "robot_hostname");
    constexpr static char const* Default_Connection_Hostname =
        (Is_Robot ? DEFAULT_CLIENT_HOSTNAME : DEFAULT_ROBOT_HOSTNAME);

private:
    template<DataFlow D>
    class MS136ScanChannel
    {
    public:
        MS136ScanChannel(
            rclcpp::Node&,
            zenoh::Session&,
            const std::string&,
            const rclcpp::QoS&,
            bool is_sim,
            DelayQueue* = nullptr);

    private:
        std::shared_ptr<void> data;
    };

    template<DataFlow D>
    class SimClockChannel
    {
    public:
        SimClockChannel(
            rclcpp::Node&,
            zenoh::Session&,
            const std::string&,
            const rclcpp::QoS&,
            bool is_sim,
            DelayQueue* = nullptr);

    private:
        std::shared_ptr<void> data{nullptr};
    };

    template<DataFlow D>
    class TalonFBChannelsGroup
    {
        using InfoChannelT = Channel<TalonInfoAdapter, D>;
        using FaultsChannelT = Channel<TalonFaultsAdapter, D>;

    public:
        TalonFBChannelsGroup(
            rclcpp::Node& node,
            zenoh::Session& zsh,
            const std::vector<std::string>& topics,
            const rclcpp::QoS& qos,
            DelayQueue* dq);

    private:
        std::vector<InfoChannelT> info_channels;
        std::vector<FaultsChannelT> faults_channels;
    };

private:
    /* Returns a pointer to the delay queue when a delay is configured,
     * nullptr otherwise. Used during member initialisation so that adapters
     * self-wire onto the buffer (non-null) or go direct to zenoh (null). */
    DelayQueue* getQueue()
    {
#if ENABLE_NET_DELAY
        return this->delay_queue.getDelay().count() > 0 ? &this->delay_queue
                                                        : nullptr;
#else
        return nullptr;
#endif
    }

private:
    zenoh::Session zsh;
    DelayQueue delay_queue;

    const bool is_sim;

    Channel<JoyAdapter, CLIENT_TO_ROBOT> joy;
    Channel<StdInt32Adapter, CLIENT_TO_ROBOT> watchdog_status;
    Channel<PointStampedAdapter, CLIENT_TO_ROBOT> clicked_point;

    MS136ScanChannel<ROBOT_TO_CLIENT> lidar_scan;
    Channel<MS136ImuAdapter, ROBOT_TO_CLIENT> imu;

    TalonFBChannelsGroup<ROBOT_TO_CLIENT> talon_feedback;

    Channel<BytesAdapter, ROBOT_TO_CLIENT> telemetry;
    Channel<StdInt8Adapter, ROBOT_TO_CLIENT> relay_status;

    SimClockChannel<ROBOT_TO_CLIENT> sim_clock;
};



// ---

#define PARAMS_FROM_TOPIC(topic)                                 \
    *this, zsh, topic, rclcpp::SensorDataQoS{}, this->getQueue()
#define PARAMS_FROM_TOPIC_SIM(topic)                                           \
    *this, zsh, topic, rclcpp::SensorDataQoS{}, this->is_sim, this->getQueue()
#define PARAMS_FROM_TOPICS(...)                                        \
    *this, zsh, __VA_ARGS__, rclcpp::SensorDataQoS{}, this->getQueue()

template<EndPoint E>
EndPointNode<E>::EndPointNode() :
    Node{Node_Name},
    zsh{zenoh::Session::open(
        util::configDirectConnectTo(
            util::declare_and_get_param<std::string>(
                *this,
                Connection_Param_Name,
                Default_Connection_Hostname)))},
    delay_queue{std::chrono::duration<double>{
        util::declare_and_get_param(*this, "net_delay_s", 0.0)}},

    is_sim{util::declare_and_get_param(*this, "is_sim", false)},

    joy{PARAMS_FROM_TOPIC("/joy")},
    watchdog_status{PARAMS_FROM_TOPIC("lance/watchdog_status")},
    clicked_point{PARAMS_FROM_TOPIC("/clicked_point")},

    lidar_scan{PARAMS_FROM_TOPIC_SIM("multiscan/lidar_scan")},
    imu{PARAMS_FROM_TOPIC("multiscan/imu")},

    talon_feedback{PARAMS_FROM_TOPICS(
        {"lance/track_left",
         "lance/track_right",
         "lance/trencher",
         "lance/hopper_belt",
         "lance/hopper_act"})},

    telemetry{PARAMS_FROM_TOPIC("lance/telemetry")},
    relay_status{PARAMS_FROM_TOPIC("lance/relay_status")},

    sim_clock{PARAMS_FROM_TOPIC_SIM("/clock")}
{
#if ENABLE_NET_DELAY
    this->delay_queue.startThread();
#endif
}

#undef PARAMS_FROM_TOPIC
#undef PARAMS_FROM_TOPIC_SIM
#undef PARAMS_FROM_TOPICS



template<EndPoint E>
template<DataFlow D>
EndPointNode<E>::MS136ScanChannel<D>::MS136ScanChannel(
    rclcpp::Node& node,
    zenoh::Session& zsh,
    const std::string& topic,
    const rclcpp::QoS& qos,
    bool is_sim,
    DelayQueue* dq)
{
    using SimChennelT = Channel<MS136SimScanAdapter, D>;
    using LiveChannelT = Channel<MS136ScanAdapter, D>;

    if (is_sim)
    {
        this->data = std::make_shared<SimChennelT>(node, zsh, topic, qos, dq);
    }
    else
    {
        this->data = std::make_shared<LiveChannelT>(node, zsh, topic, qos, dq);
    }
}

template<EndPoint E>
template<DataFlow D>
EndPointNode<E>::SimClockChannel<D>::SimClockChannel(
    rclcpp::Node& node,
    zenoh::Session& zsh,
    const std::string& topic,
    const rclcpp::QoS& qos,
    bool is_sim,
    DelayQueue* dq)
{
    using ChannelT = Channel<ClockAdapter, D>;

    if (is_sim)
    {
        this->data = std::make_shared<ChannelT>(node, zsh, topic, qos, dq);
    }
}

template<EndPoint E>
template<DataFlow D>
EndPointNode<E>::TalonFBChannelsGroup<D>::TalonFBChannelsGroup(
    rclcpp::Node& node,
    zenoh::Session& zsh,
    const std::vector<std::string>& topics,
    const rclcpp::QoS& qos,
    DelayQueue* dq)
{
    this->info_channels.reserve(topics.size());
    this->faults_channels.reserve(topics.size());
    for (const std::string& base_topic : topics)
    {
        this->info_channels
            .emplace_back(node, zsh, (base_topic + "/info"), qos, dq);
        this->faults_channels
            .emplace_back(node, zsh, (base_topic + "/faults"), qos, dq);
    }
}
