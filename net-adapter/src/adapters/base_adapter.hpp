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

#include <vector>
#include <cstdint>
#include <type_traits>

#include <zenoh.hxx>
#include <rclcpp/rclcpp.hpp>


/* Base class for adapter impelementations (crtp static polymorphism)
 * Msg_T : ROS message type which the adapter interfaces with
 * Derived_T : CRTP derivee class
 * PubState_T : Optional additional storage state for publishers
 *  (default uses derivee class - must have a constructor that accepts rclcpp::Node&)
 * SubState_T : Optional additional storage state for subscribers
 *  (default uses derivee class - must have a constructor that accepts rclcpp::Node&) */
template<
    typename Msg_T,
    typename Derived_T = void,
    typename PubState_T = Derived_T,
    typename SubState_T = Derived_T>
class BaseAdapter
{
public:
    using MsgT = Msg_T;
    using DerivedT = Derived_T;
    using BaseT = BaseAdapter<MsgT, DerivedT, PubState_T, SubState_T>;
    using PubStateT =
        std::conditional_t<std::is_void_v<PubState_T>, BaseT, PubState_T>;
    using SubStateT =
        std::conditional_t<std::is_void_v<SubState_T>, BaseT, SubState_T>;

    using ZenohPub = zenoh::Publisher;
    using ZenohSub = zenoh::Subscriber<void>;
    using RosPub = typename rclcpp::Publisher<MsgT>::SharedPtr;
    using RosSub = typename rclcpp::Subscription<MsgT>::SharedPtr;

    using byte_t = uint8_t;
    using ByteBuffer = std::vector<byte_t>;

public:
    /* Subscriber to the local ROS network, publisher to the zenoh network. */
    class Subscriber
    {
        friend BaseT;
        friend DerivedT;

        Subscriber(
            rclcpp::Node&,
            zenoh::Session&,
            const std::string&,
            const rclcpp::QoS&);

        SubStateT state;
        ZenohPub zpub;
        RosSub rsub;
    };
    /* Publisher to the remote ROS network, subscriber to the zenoh network. */
    class Publisher
    {
        friend BaseT;
        friend DerivedT;

        Publisher(
            rclcpp::Node&,
            zenoh::Session&,
            const std::string&,
            const rclcpp::QoS&);

        PubStateT state;
        RosPub rpub;
        ZenohSub zsub;
    };

public:
    static Subscriber createSubscriber(
        rclcpp::Node&,
        zenoh::Session&,
        const std::string&,
        const rclcpp::QoS& = rclcpp::SensorDataQoS{});
    static Publisher createPublisher(
        rclcpp::Node&,
        zenoh::Session&,
        const std::string&,
        const rclcpp::QoS& = rclcpp::SensorDataQoS{});

protected:
    /* Override in derivee class to implement serialization to bytes! */
    static bool serializeMsg(ByteBuffer&, const MsgT&, SubStateT&)
    {
        return false;
    }
    /* Override in derivee class to implement deserialization from bytes! */
    static bool deserializeMsg(MsgT&, const ByteBuffer&, PubStateT&)
    {
        return false;
    }

protected:
    BaseAdapter(rclcpp::Node&) {}
};



// --- Implementation ----------------------------------------------------------

template<typename M, typename D, typename P, typename S>
BaseAdapter<M, D, P, S>::Subscriber::Subscriber(
    rclcpp::Node& node,
    zenoh::Session& zsh,
    const std::string& topic,
    const rclcpp::QoS& qos) :
    state{node},
    zpub{zsh.declare_publisher(topic.front() == '/' ? topic.substr(1) : topic)},
    rsub{node.create_subscription<MsgT>(
        topic,
        qos,
        [this](const MsgT& msg)
        {
            ByteBuffer bytes;
            if (DerivedT::serializeMsg(bytes, msg, this->state))
            {
                this->zpub.put(zenoh::Bytes(std::move(bytes)));
            }
        })}
{
}

template<typename M, typename D, typename P, typename S>
BaseAdapter<M, D, P, S>::Publisher::Publisher(
    rclcpp::Node& node,
    zenoh::Session& zsh,
    const std::string& topic,
    const rclcpp::QoS& qos) :
    state{node},
    rpub{node.create_publisher<MsgT>(topic, qos)},
    zsub{zsh.declare_subscriber(
        topic.front() == '/' ? topic.substr(1) : topic,
        [this](const zenoh::Sample& sample)
        {
            ByteBuffer bytes = sample.get_payload().as_vector();
            MsgT msg;
            if (DerivedT::deserializeMsg(msg, bytes, this->state))
            {
                this->rpub->publish(msg);
            }
        },
        []() {})}
{
}

template<typename M, typename D, typename P, typename S>
typename BaseAdapter<M, D, P, S>::Subscriber
    BaseAdapter<M, D, P, S>::createSubscriber(
        rclcpp::Node& node,
        zenoh::Session& zsh,
        const std::string& topic,
        const rclcpp::QoS& qos)
{
    return Subscriber(node, zsh, topic, qos);
}

template<typename M, typename D, typename P, typename S>
typename BaseAdapter<M, D, P, S>::Publisher
    BaseAdapter<M, D, P, S>::createPublisher(
        rclcpp::Node& node,
        zenoh::Session& zsh,
        const std::string& topic,
        const rclcpp::QoS& qos)
{
    return Publisher(node, zsh, topic, qos);
}
