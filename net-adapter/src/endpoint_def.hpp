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
#include <string>

#include <zenoh.hxx>

#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/int8.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/string.hpp>
#include <rosgraph_msgs/msg/clock.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>

#include "ros_utils.hpp"
#include "zenoh_utils.hpp"
#include "delay_buffer.hpp"

#include "adapters/joy_adapter.hpp"
#include "adapters/path_adapter.hpp"
#include "adapters/talon_adapter.hpp"
#include "adapters/generic_adapter.hpp"
#include "adapters/ms136_imu_adapter.hpp"
#include "adapters/ms136_scan_adapter.hpp"


#define DEFAULT_ROBOT_HOSTNAME  "10.11.11.15"
#define DEFAULT_CLIENT_HOSTNAME "10.11.11.1"


enum EndPoint
{
    ROBOT_ENDPOINT,
    CLIENT_ENDPOINT
};
enum DataFlow
{
    ROBOT_TO_CLIENT,
    CLIENT_TO_ROBOT
};

template<DataFlow D, EndPoint E>
struct PubSubTraits
{
    constexpr static DataFlow Data_Flow_V = D;
    constexpr static EndPoint End_Point_V = E;

    constexpr static bool Is_Subscriber =
        ((Data_Flow_V == ROBOT_TO_CLIENT) == (End_Point_V == ROBOT_ENDPOINT));
    constexpr static bool Is_Publisher =
        ((Data_Flow_V == CLIENT_TO_ROBOT) == (End_Point_V == ROBOT_ENDPOINT));
};

template<typename Adapter_T, DataFlow D, EndPoint E>
struct AdapterTraits : public PubSubTraits<D, E>
{
    // **traits check that adapter extends BaseAdapter**

    using AdapterT = Adapter_T;
    using RawSubscriberT = typename AdapterT::Subscriber;
    using RawPublisherT = typename AdapterT::Publisher;

    class ISubscriber : public RawSubscriberT
    {
        using RawT = RawSubscriberT;

    public:
        ISubscriber(
            rclcpp::Node&,
            zenoh::Session&,
            const std::string&,
            const rclcpp::QoS& = rclcpp::SensorDataQoS{},
            DelayBuffer* = nullptr);
    };
    class IPublisher : public RawPublisherT
    {
        using RawT = RawPublisherT;

    public:
        IPublisher(
            rclcpp::Node&,
            zenoh::Session&,
            const std::string&,
            const rclcpp::QoS& = rclcpp::SensorDataQoS{},
            DelayBuffer* = nullptr);
    };

    using EffectorT =
        std::conditional_t<Is_Subscriber, ISubscriber, IPublisher>;

    // class Effector :
    //     public std::conditional_t<Is_Subscriber, ISubscriber, IPublisher>
    // {
    //     using IterfaceT =
    //         std::conditional_t<Is_Subscriber, ISubscriber, IPublisher>;

    // public:
    //     Effector(
    //         rclcpp::Node&,
    //         zenoh::Session&,
    //         const std::string&,
    //         const rclcpp::QoS& = rclcpp::SensorDataQoS{},
    //         DelayBuffer* = nullptr);
    // };
};


template<typename A, DataFlow D, EndPoint E>
AdapterTraits<A, D, E>::ISubscriber::ISubscriber(
    rclcpp::Node& node,
    zenoh::Session& zsh,
    const std::string& topic,
    const rclcpp::QoS& qos,
    DelayBuffer* db) :
    RawSubscriberT{node, zsh, topic, qos, db}
{
}

template<typename A, DataFlow D, EndPoint E>
AdapterTraits<A, D, E>::IPublisher::IPublisher(
    rclcpp::Node& node,
    zenoh::Session& zsh,
    const std::string& topic,
    const rclcpp::QoS& qos,
    DelayBuffer* db) :
    RawPublisherT{node, zsh, topic, qos}
{
    (void)db;
}

// template<typename A, DataFlow D, EndPoint E>
// AdapterTraits<A, D, E>::Effector::Effector(
//     rclcpp::Node& node,
//     zenoh::Session& zsh,
//     const std::string& topic,
//     const rclcpp::QoS& qos,
//     DelayBuffer* db) :
//     IterfaceT{node, zsh, topic, qos, db}
// {
// }



template<EndPoint E, DataFlow D>
class MS136ScanEffector
{
    using LiveEffectorT =
        typename AdapterTraits<MS136ScanAdapter, E, D>::EffectorT;
    using SimEffectorT =
        typename AdapterTraits<MS136SimScanAdapter, E, D>::EffectorT;

public:
    MS136ScanEffector(
        rclcpp::Node&,
        zenoh::Session&,
        const std::string&,
        const rclcpp::QoS&,
        bool is_sim,
        DelayBuffer* = nullptr);

private:
    std::shared_ptr<void> data;
};

template<EndPoint E, DataFlow D>
MS136ScanEffector<E, D>::MS136ScanEffector(
    rclcpp::Node& node,
    zenoh::Session& zsh,
    const std::string& topic,
    const rclcpp::QoS& qos,
    bool is_sim,
    DelayBuffer* db)
{
    if (is_sim)
    {
        this->data = std::make_shared<SimEffectorT>(node, zsh, topic, qos, db);
    }
    else
    {
        this->data = std::make_shared<LiveEffectorT>(node, zsh, topic, qos, db);
    }
}



template<EndPoint E, DataFlow D>
class SimClockEffector
{
    using ClockAdapter = GenericAdapter<rosgraph_msgs::msg::Clock>;
    using EffectorT = typename AdapterTraits<ClockAdapter, E, D>::EffectorT;

public:
    SimClockEffector(
        rclcpp::Node&,
        zenoh::Session&,
        const std::string&,
        const rclcpp::QoS&,
        bool is_sim,
        DelayBuffer* = nullptr);

private:
    std::shared_ptr<void> data{nullptr};
};

template<EndPoint E, DataFlow D>
SimClockEffector<E, D>::SimClockEffector(
    rclcpp::Node& node,
    zenoh::Session& zsh,
    const std::string& topic,
    const rclcpp::QoS& qos,
    bool is_sim,
    DelayBuffer* db)
{
    if (is_sim)
    {
        this->data = std::make_shared<EffectorT>(node, zsh, topic, qos, db);
    }
}



template<EndPoint E, DataFlow D>
class TalonEffectorsGroup
{
    using InfoEffectorT =
        typename AdapterTraits<TalonInfoAdapter, E, D>::EffectorT;
    using FaultsEffectorT =
        typename AdapterTraits<TalonFaultsAdapter, E, D>::EffectorT;

public:
    TalonEffectorsGroup(
        rclcpp::Node&,
        zenoh::Session&,
        const std::vector<std::string>&,
        const rclcpp::QoS&,
        DelayBuffer*);

private:
    std::vector<InfoEffectorT> info_effectors;
    std::vector<FaultsEffectorT> faults_effectors;
};

template<EndPoint E, DataFlow D>
TalonEffectorsGroup<E, D>::TalonEffectorsGroup(
    rclcpp::Node& node,
    zenoh::Session& zsh,
    const std::vector<std::string>& topics,
    const rclcpp::QoS& qos,
    DelayBuffer* db)
{
    this->info_effectors.reserve(topics.size());
    this->faults_effectors.reserve(topics.size());
    for (const std::string& base_topic : topics)
    {
        this->info_effectors
            .emplace_back(node, zsh, (base_topic + "/info"), qos, db);
        this->faults_effectors
            .emplace_back(node, zsh, (base_topic + "/faults"), qos, db);
    }
}



template<EndPoint E>
class EndpointNode : public rclcpp::Node
{
public:
    EndpointNode();

private:
    using StdInt8Adapter = GenericAdapter<std_msgs::msg::Int8>;
    using StdInt32Adapter = GenericAdapter<std_msgs::msg::Int32>;
    using StdStringAdapter = GenericAdapter<std_msgs::msg::String>;
    using PointStampedAdapter =
        GenericAdapter<geometry_msgs::msg::PointStamped>;
    // using ClockAdapter = GenericAdapter<rosgraph_msgs::msg::Clock>;

private:
    template<DataFlow D>
    using PSTraits = PubSubTraits<D, E>;
    template<typename AdapterT, DataFlow D>
    using ApTraits = AdapterTraits<AdapterT, D, E>;

    template<typename AdapterT, DataFlow D>
    using M = typename ApTraits<AdapterT, D>::EffectorT;

    constexpr static bool Is_Robot = (E == ROBOT_ENDPOINT);
    constexpr static bool Is_Client = (E == CLIENT_ENDPOINT);

private:
    constexpr static char const* Node_Name =
        (Is_Robot ? "robot_redux_endpoint" : "client_redux_endpoint");
    constexpr static char const* Connection_Param_Name =
        (Is_Robot ? "client_hostname" : "robot_hostname");
    constexpr static char const* Default_Connection_Hostname =
        (Is_Robot ? DEFAULT_CLIENT_HOSTNAME : DEFAULT_ROBOT_HOSTNAME);

    /* How often the drain timer fires — ~1/10th the minimum useful delay. */
    constexpr static uint32_t Drain_Period_Ms = 10;

private:
    /* Returns a pointer to the delay buffer when a delay is configured,
     * nullptr otherwise.  Used during member initialisation so that adapters
     * self-wire onto the buffer (non-null) or go direct to zenoh (null). */
    DelayBuffer* resolveDelayBuff()
    {
        return this->delay_buffer.getDelay().count() > 0 ? &this->delay_buffer
                                                         : nullptr;
    }

private:
    zenoh::Session zsh;
    const bool is_sim;

    // Single shared delay buffer for the entire endpoint
    DelayBuffer delay_buffer;
    rclcpp::TimerBase::SharedPtr drain_timer_;  // null when delay == 0


    M<JoyAdapter, CLIENT_TO_ROBOT> joy;
    M<StdInt32Adapter, CLIENT_TO_ROBOT> watchdog_status;
    M<PointStampedAdapter, CLIENT_TO_ROBOT> clicked_point;

    M<MS136ImuAdapter, ROBOT_TO_CLIENT> imu;
    MS136ScanEffector<E, ROBOT_TO_CLIENT> lidar_scan;
    M<PathAdapter, ROBOT_TO_CLIENT> path;
    M<StdInt8Adapter, ROBOT_TO_CLIENT> relay_status;
    M<StdStringAdapter, ROBOT_TO_CLIENT> op_status;

    TalonEffectorsGroup<E, ROBOT_TO_CLIENT> talon_feedback;

    SimClockEffector<E, ROBOT_TO_CLIENT> sim_clock;
};

template<EndPoint E>
EndpointNode<E>::EndpointNode() :
    Node{
        Node_Name
},
    zsh{zenoh::Session::open(
        util::configDirectConnectTo(
            util::declare_and_get_param<std::string>(
                *this,
                Connection_Param_Name,
                Default_Connection_Hostname)))},
    is_sim{util::declare_and_get_param(*this, "is_sim", false)},

    // Delay buffer ROS param (default 0 = disabled)
    delay_buffer{std::chrono::duration<double>{
        util::declare_and_get_param(*this, "net_delay_s", 0.0)}},

    joy{*this, zsh, "/joy", rclcpp::SensorDataQoS{}, this->resolveDelayBuff()},
    watchdog_status{
        *this,
        zsh,
        "lance/watchdog_status",
        rclcpp::SensorDataQoS{},
        this->resolveDelayBuff()},
    clicked_point{
        *this,
        zsh,
        "/clicked_point",
        rclcpp::SensorDataQoS{},
        this->resolveDelayBuff()},

    imu{*this,
        zsh,
        "multiscan/imu",
        rclcpp::SensorDataQoS{},
        this->resolveDelayBuff()},
    lidar_scan{
        *this,
        zsh,
        "multiscan/lidar_scan",
        rclcpp::SensorDataQoS{},
        is_sim,
        this->resolveDelayBuff()},
    path{
        *this,
        zsh,
        "cardinal_perception/planned_path",
        rclcpp::SensorDataQoS{},
        this->resolveDelayBuff()},
    relay_status{
        *this,
        zsh,
        "lance/relay_status",
        rclcpp::SensorDataQoS{},
        this->resolveDelayBuff()},
    op_status{
        *this,
        zsh,
        "lance/op_status",
        rclcpp::SensorDataQoS{},
        this->resolveDelayBuff()},

    talon_feedback{
        *this,
        zsh,
        {"lance/track_left",
         "lance/track_right",
         "lance/trencher",
         "lance/hopper_belt",
         "lance/hopper_act"},
        rclcpp::SensorDataQoS{},
        this->resolveDelayBuff()},

    sim_clock{
        *this,
        zsh,
        "/clock",
        rclcpp::SensorDataQoS{},
        is_sim,
        this->resolveDelayBuff()}
{
    // Start the drain timer only when a delay is actually configured.
    if (delay_buffer.getDelay().count() > 0)
    {
        drain_timer_ = this->create_wall_timer(
            std::chrono::milliseconds{drain_period_ms},
            [this]() { delay_buffer.drain(); });
    }

    RCLCPP_INFO(
        this->get_logger(),
        "Bridge Started (delay: %ld ms)",
        delay_buffer.getDelay().count());
}
