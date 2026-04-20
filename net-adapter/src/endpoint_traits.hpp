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

#include "util/ros_utils.hpp"
#include "util/zenoh_utils.hpp"
#include "core/delay_queue.hpp"


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
struct ChannelTraits
{
    constexpr static int Data_Flow_V = D;
    constexpr static int End_Point_V = E;

    constexpr static bool Is_Subscriber =
        ((Data_Flow_V == ROBOT_TO_CLIENT) == (End_Point_V == ROBOT_ENDPOINT));
    constexpr static bool Is_Publisher =
        ((Data_Flow_V == CLIENT_TO_ROBOT) == (End_Point_V == ROBOT_ENDPOINT));
};

template<typename Adapter_T, DataFlow D, EndPoint E>
struct AdapterTraits : public ChannelTraits<D, E>
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
            DelayQueue* = nullptr);
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
            DelayQueue* = nullptr);
    };

    using ChannelTraits<D, E>::Is_Subscriber;
    using ChannelT = std::conditional_t<Is_Subscriber, ISubscriber, IPublisher>;
};


// ---

template<typename A, DataFlow D, EndPoint E>
AdapterTraits<A, D, E>::ISubscriber::ISubscriber(
    rclcpp::Node& node,
    zenoh::Session& zsh,
    const std::string& topic,
    const rclcpp::QoS& qos,
    DelayQueue* dq) :
    RawSubscriberT{node, zsh, topic, qos, dq}
{
}

template<typename A, DataFlow D, EndPoint E>
AdapterTraits<A, D, E>::IPublisher::IPublisher(
    rclcpp::Node& node,
    zenoh::Session& zsh,
    const std::string& topic,
    const rclcpp::QoS& qos,
    DelayQueue* dq) :
    RawPublisherT{node, zsh, topic, qos}
{
    (void)dq;
}
