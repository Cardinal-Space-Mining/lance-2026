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
#include <geometry_msgs/msg/point_stamped.hpp>
#include <rosgraph_msgs/msg/clock.hpp>

#include "ros_utils.hpp"
#include "zenoh_utils.hpp"
#include "delay_buffer.hpp"
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
auto make_member(
    rclcpp::Node& node,
    zenoh::Session& zsh,
    const char* topic,
    DelayBuffer* delay_buf)
{
    constexpr bool isSub = (Role == NodeRole::Robot) == (dir == INCOMING);
    if constexpr (isSub)
    {
        return Adapter::createSubscriber(
            node,
            zsh,
            topic,
            rclcpp::SensorDataQoS{},
            delay_buf);
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

    Member(
        rclcpp::Node& node,
        zenoh::Session& zsh,
        const char* topic,
        DelayBuffer* delay_buf) :
        inner{make_member<Role, Adapter, dir>(node, zsh, topic, delay_buf)}
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

    /* How often the drain timer fires — ~1/10th the minimum useful delay. */
    static constexpr int drain_period_ms = 10;

    struct TalonMotor
    {
        M<TalonInfoAdapter, OUTGOING> info;
        M<TalonFaultsAdapter, OUTGOING> faults;

        TalonMotor(
            rclcpp::Node& node,
            zenoh::Session& zsh,
            const char* base,
            DelayBuffer* delay_buf) :
            info{node, zsh, (std::string(base) + "/info").c_str(), delay_buf},
            faults{
                node,
                zsh,
                (std::string(base) + "/faults").c_str(),
                delay_buf}
        {
        }
    };

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
            if constexpr (Role == NodeRole::Robot)
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
            if constexpr (Role == NodeRole::Robot)
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
        if constexpr (Role == NodeRole::Robot)
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

        // Control (Client -> Robot)
        joy{*this, zsh, "/joy", delay_ptr()},
        watchdog_status{*this, zsh, "lance/watchdog_status", delay_ptr()},
        clicked_point{*this, zsh, "/clicked_point", delay_ptr()},

        // Data (Robot -> Client)
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
        if (delay_buf_.get_delay().count() > 0)
        {
            drain_timer_ = this->create_wall_timer(
                std::chrono::milliseconds{drain_period_ms},
                [this]() { delay_buf_.drain(); });
        }

        RCLCPP_INFO(
            this->get_logger(),
            "Bridge Started (delay: %ld ms)",
            delay_buf_.get_delay().count());
    }

private:
    /* Returns a pointer to the delay buffer when a delay is configured,
     * nullptr otherwise.  Used during member initialisation so that adapters
     * self-wire onto the buffer (non-null) or go direct to zenoh (null). */
    DelayBuffer* delay_ptr()
    {
        return delay_buf_.get_delay().count() > 0 ? &delay_buf_ : nullptr;
    }

    zenoh::Session zsh;
    const bool is_sim;

    // Single shared delay buffer for the entire endpoint
    DelayBuffer delay_buf_;
    rclcpp::TimerBase::SharedPtr drain_timer_;  // null when delay == 0

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
