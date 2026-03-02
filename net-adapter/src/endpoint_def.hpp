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

    class Effector :
        public std::conditional_t<Is_Subscriber, ISubscriber, IPublisher>
    {
        using IterfaceT =
            std::conditional_t<Is_Subscriber, ISubscriber, IPublisher>;

    public:
        Effector(
            rclcpp::Node&,
            zenoh::Session&,
            const std::string&,
            const rclcpp::QoS& = rclcpp::SensorDataQoS{},
            DelayBuffer* = nullptr);
    };

    // static PipelineT makePipelineMember(
    //     rclcpp::Node&,
    //     zenoh::Session&,
    //     std::string&,
    //     rclcpp::QoS& = rclcpp::SensorDataQoS{},
    //     DelayBuffer* = nullptr);
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

template<typename A, DataFlow D, EndPoint E>
AdapterTraits<A, D, E>::Effector::Effector(
    rclcpp::Node& node,
    zenoh::Session& zsh,
    const std::string& topic,
    const rclcpp::QoS& qos,
    DelayBuffer* db) :
    IterfaceT{node, zsh, topic, qos, db}
{
}

// template<typename A, DataFlow D, EndPoint E>
// typename AdapterTraits<A, D, E>::PipelineT
//     AdapterTraits<A, D, E>::makePipelineMember(
//         rclcpp::Node& node,
//         zenoh::Session& zsh,
//         std::string& topic,
//         rclcpp::QoS& qos,
//         DelayBuffer* db)
// {
//     if constexpr (Is_Subscriber)
//     {
//         return AdapterT::createSubscriber(node, zsh, topic, qos, db);
//     }
//     else
//     {
//         return AdapterT::createPublisher(node, zsh, topic, qos);
//     }
// }




// template<NodeRole Role, typename Adapter, DataDir dir>
// auto make_member(
//     rclcpp::Node& node,
//     zenoh::Session& zsh,
//     const char* topic,
//     DelayBuffer* delay_buf)
// {
//     constexpr bool isSub = (Role == NodeRole::ROBOT) == (dir == INCOMING);
//     if constexpr (isSub)
//     {
//         return Adapter::createSubscriber(
//             node,
//             zsh,
//             topic,
//             rclcpp::SensorDataQoS{},
//             delay_buf);
//     }
//     else
//     {
//         return Adapter::createPublisher(node, zsh, topic);
//     }
// }

// template<NodeRole Role, typename Adapter, DataDir dir>
// struct Member
// {
//     typename NetTraits<Role, Adapter, dir>::Type inner;

//     Member(
//         rclcpp::Node& node,
//         zenoh::Session& zsh,
//         const char* topic,
//         DelayBuffer* delay_buf) :
//         inner{make_member<Role, Adapter, dir>(node, zsh, topic, delay_buf)}
//     {
//     }
// };

template<EndPoint E>
class EndpointNode : public rclcpp::Node
{
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

        // Delay buffer ROS param (default 0 = disabled)
        delay_buf_{std::chrono::milliseconds{
            util::declare_and_get_param(*this, "delay_ms", 0)}},

        // Control (CLIENT -> ROBOT)
        joy{*this, zsh, "/joy", delay_ptr()},
        watchdog_status{*this, zsh, "lance/watchdog_status", delay_ptr()},
        clicked_point{*this, zsh, "/clicked_point", delay_ptr()},

        // Data (ROBOT -> CLIENT)
        imu{*this, zsh, "multiscan/imu", delay_ptr()},
        lidar_scan{make_scan(*this, zsh, is_sim, delay_ptr())},
        path{*this, zsh, "cardinal_perception/planned_path", delay_ptr()},
        relay_status{*this, zsh, "lance/relay_status", delay_ptr()},
        op_status{*this, zsh, "lance/op_status", delay_ptr()},

        // Talon Motors
        track_left{*this, zsh, "lance/track_left", delay_ptr()},
        track_right{*this, zsh, "lance/track_right", delay_ptr()},
        trencher{*this, zsh, "lance/trencher", delay_ptr()},
        hopper_belt{*this, zsh, "lance/hopper_belt", delay_ptr()},
        hopper_act{*this, zsh, "lance/hopper_act", delay_ptr()},

        // Sim-only
        sim_clock{make_clock(*this, zsh, is_sim, delay_ptr())}
    {
        // Start the drain timer only when a delay is actually configured.
        if (delay_buf_.getDelay().count() > 0)
        {
            drain_timer_ = this->create_wall_timer(
                std::chrono::milliseconds{drain_period_ms},
                [this]() { delay_buf_.drain(); });
        }

        RCLCPP_INFO(
            this->get_logger(),
            "Bridge Started (delay: %ld ms)",
            delay_buf_.getDelay().count());
    }

private:
    using StdInt8Adapter = GenericAdapter<std_msgs::msg::Int8>;
    using StdInt32Adapter = GenericAdapter<std_msgs::msg::Int32>;
    using StdStringAdapter = GenericAdapter<std_msgs::msg::String>;
    using PointStampedAdapter =
        GenericAdapter<geometry_msgs::msg::PointStamped>;
    using ClockAdapter = GenericAdapter<rosgraph_msgs::msg::Clock>;

private:
    template<DataFlow D>
    using PSTraits = PubSubTraits<D, E>;
    template<typename AdapterT, DataFlow D>
    using ApTraits = AdapterTraits<AdapterT, D, E>;

    template<typename AdapterT, DataFlow D>
    using M = typename ApTraits<AdapterT, D>::Effector;

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
    template<DataFlow D>
    struct TalonMotor
    {
        M<TalonInfoAdapter, D> info;
        M<TalonFaultsAdapter, D> faults;

        TalonMotor(
            rclcpp::Node& node,
            zenoh::Session& zsh,
            const std::string& base,
            const rclcpp::QoS& qos = rclcpp::SensorDataQoS{}
            DelayBuffer* delay_buf = nullptr) :
            info{node, zsh, (base + "/info"), qos, delay_buf},
            faults{node, zsh, (base + "/faults"), qos, delay_buf}
        {
        }
    };

private:
    // Creates the correct scan adapter at runtime based on is_sim.
    // Subscriber-side variants receive delay_buf; Publisher-side ignore it.
    static std::shared_ptr<void> make_scan(
        rclcpp::Node& node,
        zenoh::Session& zsh,
        bool is_sim,
        DelayBuffer* delay_buf)
    {
        if (is_sim)
        {
            if constexpr (Role == NodeRole::ROBOT)
            {
                return MS136SimScanAdapter::createSharedSubscriber(
                    node,
                    zsh,
                    "multiscan/lidar_scan",
                    rclcpp::SensorDataQoS{},
                    delay_buf);
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
            if constexpr (Role == NodeRole::ROBOT)
            {
                return MS136ScanAdapter::createSharedSubscriber(
                    node,
                    zsh,
                    "multiscan/lidar_scan",
                    rclcpp::SensorDataQoS{},
                    delay_buf);
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

    // Creates clock adapter only when is_sim, nullptr otherwise.
    static std::shared_ptr<void> make_clock(
        rclcpp::Node& node,
        zenoh::Session& zsh,
        bool is_sim,
        DelayBuffer* delay_buf)
    {
        if (!is_sim)
        {
            return nullptr;
        }
        if constexpr (Role == NodeRole::ROBOT)
        {
            return ClockAdapter::createSharedSubscriber(
                node,
                zsh,
                "/clock",
                rclcpp::SensorDataQoS{},
                delay_buf);
        }
        else
        {
            return ClockAdapter::createSharedPublisher(node, zsh, "/clock");
        }
    }

    /* Returns a pointer to the delay buffer when a delay is configured,
     * nullptr otherwise.  Used during member initialisation so that adapters
     * self-wire onto the buffer (non-null) or go direct to zenoh (null). */
    DelayBuffer* delay_ptr()
    {
        return delay_buf_.getDelay().count() > 0 ? &delay_buf_ : nullptr;
    }

private:
    zenoh::Session zsh;
    const bool is_sim;

    // Single shared delay buffer for the entire endpoint
    DelayBuffer delay_buf_;
    rclcpp::TimerBase::SharedPtr drain_timer_;  // null when delay == 0


    M<JoyAdapter, CLIENT_TO_ROBOT> joy;
    M<StdInt32Adapter, CLIENT_TO_ROBOT> watchdog_status;
    M<PointStampedAdapter, CLIENT_TO_ROBOT> clicked_point;

    M<MS136ImuAdapter, ROBOT_TO_CLIENT> imu;
    std::shared_ptr<void> lidar_scan;  // OUTGOING
    M<PathAdapter, ROBOT_TO_CLIENT> path;
    M<StdInt8Adapter, ROBOT_TO_CLIENT> relay_status;
    M<StdStringAdapter, ROBOT_TO_CLIENT> op_status;

    TalonMotor<ROBOT_TO_CLIENT> track_left;
    TalonMotor<ROBOT_TO_CLIENT> track_right;
    TalonMotor<ROBOT_TO_CLIENT> trencher;
    TalonMotor<ROBOT_TO_CLIENT> hopper_belt;
    TalonMotor<ROBOT_TO_CLIENT> hopper_act;

    std::shared_ptr<void> sim_clock;  // OUTGOING
};
