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

#include <memory>
#include <vector>
#include <cstdint>
#include <type_traits>

#include <zenoh.hxx>
#include <rclcpp/rclcpp.hpp>

#include "../util/delay_queue.hpp"
#include "../util/zstd_utils.hpp"


/* Base class for adapter implementations (CRTP static polymorphism).
 *
 * Msg_T      : ROS message type the adapter interfaces with
 * Derived_T  : CRTP derivee class
 * PubState_T : Optional extra storage for publishers (default = Derived_T)
 * SubState_T : Optional extra storage for subscribers (default = Derived_T)
 *
 * Both PubState_T / SubState_T must have a constructor that accepts
 * rclcpp::Node& when they equal Derived_T. */
template<
    typename Msg_T,
    typename Derived_T = void,
    typename PubState_T = Derived_T,
    typename SubState_T = Derived_T,
    bool Compress = false>
class BaseAdapter
{
public:
    using MsgT = Msg_T;
    using DerivedT = Derived_T;
    using BaseT = BaseAdapter<MsgT, DerivedT, PubState_T, SubState_T, Compress>;
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

    public:
        Subscriber(
            rclcpp::Node&,
            zenoh::Session&,
            const std::string&,
            const rclcpp::QoS&,
            DelayQueue* delay_q = nullptr);

    private:
        SubStateT state;
        [[no_unique_address]]
        std::conditional_t<Compress, util::ZstdCCtx, util::ZstdNoCtx>
            compress_ctx;  // compression context if Compress, else zero size
        ZenohPub zpub;
        RosSub rsub;
    };

    /* Subscribes to the zenoh network and publishes onto the local ROS network. */
    class Publisher
    {
        friend BaseT;
        friend DerivedT;

    public:
        Publisher(
            rclcpp::Node&,
            zenoh::Session&,
            const std::string&,
            const rclcpp::QoS&);

    private:
        PubStateT state;
        [[no_unique_address]]
        std::conditional_t<Compress, util::ZstdDCtx, util::ZstdNoCtx>
            decompress_ctx;  // decompression context if Compress, else zero size
        RosPub rpub;
        ZenohSub zsub;
    };

public:
    static Subscriber createSubscriber(
        rclcpp::Node&,
        zenoh::Session&,
        const std::string&,
        const rclcpp::QoS& = rclcpp::SensorDataQoS{},
        DelayQueue* delay_q = nullptr);

    static Publisher createPublisher(
        rclcpp::Node&,
        zenoh::Session&,
        const std::string&,
        const rclcpp::QoS& = rclcpp::SensorDataQoS{});

    static std::shared_ptr<Subscriber> createSharedSubscriber(
        rclcpp::Node&,
        zenoh::Session&,
        const std::string&,
        const rclcpp::QoS& = rclcpp::SensorDataQoS{},
        DelayQueue* delay_q = nullptr);

    static std::shared_ptr<Publisher> createSharedPublisher(
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

template<typename M, typename D, typename P, typename S, bool C>
BaseAdapter<M, D, P, S, C>::Subscriber::Subscriber(
    rclcpp::Node& node,
    zenoh::Session& zsh,
    const std::string& topic,
    const rclcpp::QoS& qos,
    DelayQueue* delay_q) :
    state{node},
    zpub{zsh.declare_publisher(topic.front() == '/' ? topic.substr(1) : topic)},
    rsub{node.create_subscription<MsgT>(
        topic,
        qos,
        [this,
         delay_q,
         logger = node.get_logger(),
         clock = node.get_clock(),
         topic](const MsgT& msg)
        {
            ByteBuffer bytes;
            if (!DerivedT::serializeMsg(bytes, msg, this->state))
            {
                return;
            }

            if constexpr (C)
            {
                if (!this->compress_ctx.compress(bytes))
                {
                    RCLCPP_ERROR_THROTTLE(
                        logger,
                        *clock,
                        1000,
                        "zstd compression failed on topic '%s'; dropping message",
                        topic.c_str());
                    return;
                }
            }

            if (delay_q)
            {
                // Capture a raw pointer to zpub, probably safe because both this
                // Subscriber and the DelayQueue are owned by EndPointNode,
                // so zpub always outlives every queued entry.
                ZenohPub* pub = &this->zpub;
                delay_q->push(
                    [pub](ByteBuffer&& b)
                    { pub->put(zenoh::Bytes(std::move(b))); },
                    std::move(bytes));
            }
            else
            {
                this->zpub.put(zenoh::Bytes(std::move(bytes)));
            }
        })}
{
}

template<typename M, typename D, typename P, typename S, bool C>
BaseAdapter<M, D, P, S, C>::Publisher::Publisher(
    rclcpp::Node& node,
    zenoh::Session& zsh,
    const std::string& topic,
    const rclcpp::QoS& qos) :
    state{node},
    rpub{node.create_publisher<MsgT>(topic, qos)},
    zsub{zsh.declare_subscriber(
        topic.front() == '/' ? topic.substr(1) : topic,
        [this, logger = node.get_logger(), clock = node.get_clock(), topic](
            const zenoh::Sample& sample)
        {
            ByteBuffer bytes = sample.get_payload().as_vector();
            if constexpr (C)
            {
                if (!this->decompress_ctx.decompress(bytes))
                {
                    RCLCPP_ERROR_THROTTLE(
                        logger,
                        *clock,
                        1000,
                        "zstd decompression failed on topic '%s'; dropping message",
                        topic.c_str());
                    return;
                }
            }
            MsgT msg;
            if (DerivedT::deserializeMsg(msg, bytes, this->state))
            {
                this->rpub->publish(msg);
            }
        },
        []() {})}
{
}

template<typename M, typename D, typename P, typename S, bool C>
typename BaseAdapter<M, D, P, S, C>::Subscriber
    BaseAdapter<M, D, P, S, C>::createSubscriber(
        rclcpp::Node& node,
        zenoh::Session& zsh,
        const std::string& topic,
        const rclcpp::QoS& qos,
        DelayQueue* delay_q)
{
    return Subscriber(node, zsh, topic, qos, delay_q);
}

template<typename M, typename D, typename P, typename S, bool C>
typename BaseAdapter<M, D, P, S, C>::Publisher
    BaseAdapter<M, D, P, S, C>::createPublisher(
        rclcpp::Node& node,
        zenoh::Session& zsh,
        const std::string& topic,
        const rclcpp::QoS& qos)
{
    return Publisher(node, zsh, topic, qos);
}

template<typename M, typename D, typename P, typename S, bool C>
std::shared_ptr<typename BaseAdapter<M, D, P, S, C>::Subscriber>
    BaseAdapter<M, D, P, S, C>::createSharedSubscriber(
        rclcpp::Node& node,
        zenoh::Session& zsh,
        const std::string& topic,
        const rclcpp::QoS& qos,
        DelayQueue* delay_q)
{
    return std::make_shared<Subscriber>(
        std::ref(node),
        std::ref(zsh),
        std::cref(topic),
        std::cref(qos),
        delay_q);
}

template<typename M, typename D, typename P, typename S, bool C>
std::shared_ptr<typename BaseAdapter<M, D, P, S, C>::Publisher>
    BaseAdapter<M, D, P, S, C>::createSharedPublisher(
        rclcpp::Node& node,
        zenoh::Session& zsh,
        const std::string& topic,
        const rclcpp::QoS& qos)
{
    return std::make_shared<Publisher>(
        std::ref(node),
        std::ref(zsh),
        std::cref(topic),
        std::cref(qos));
}
