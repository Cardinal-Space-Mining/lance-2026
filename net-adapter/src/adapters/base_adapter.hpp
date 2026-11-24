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

#include "../core/zstd_ctx.hpp"
#include "../core/delay_queue.hpp"
#include "../core/bandwidth_monitor.hpp"


/* Base class for adapter implementations (CRTP static polymorphism).
 *
 * Msg_T            : ROS message type the adapter interfaces with
 * Derived_T        : CRTP derivee class
 * Compression_Lvl: : Zstd compression level to use if > 0
 * PubState_T       : Optional extra storage for publishers (default = Derived_T)
 * SubState_T       : Optional extra storage for subscribers (default = Derived_T)
 *
 * Both PubState_T / SubState_T must have a constructor that accepts
 * rclcpp::Node& when they equal Derived_T. */
template<
    typename Msg_T,
    typename Derived_T = void,
    int Compression_Lvl = 0,
    typename PubState_T = Derived_T,
    typename SubState_T = Derived_T>
class BaseAdapter
{
public:
    using MsgT = Msg_T;
    using DerivedT = Derived_T;
    using BaseT =
        BaseAdapter<MsgT, DerivedT, Compression_Lvl, PubState_T, SubState_T>;
    using PubStateT =
        std::conditional_t<std::is_void_v<PubState_T>, BaseT, PubState_T>;
    using SubStateT =
        std::conditional_t<std::is_void_v<SubState_T>, BaseT, SubState_T>;

    static constexpr int COMPRESSION_LEVEL = Compression_Lvl;
    static constexpr bool USING_COMPRESSION = (COMPRESSION_LEVEL > 0);

    using CompressorT = std::conditional_t<
        USING_COMPRESSION,
        util::ZstdCompressor,
        util::ZstdNoCompression>;
    using DecompressorT = std::conditional_t<
        USING_COMPRESSION,
        util::ZstdDecompressor,
        util::ZstdNoCompression>;

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
            DelayQueue* delay_q = nullptr,
            BandwidthMonitor* bw = nullptr);

    private:
        SubStateT state;
        ZenohPub zpub;
        BandwidthMonitor::Counter* tx_counter;  // null: not counted
        RosSub rsub;

        [[no_unique_address]]
        CompressorT compress_ctx;
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
            const rclcpp::QoS&,
            BandwidthMonitor* bw = nullptr);

    private:
        PubStateT state;
        RosPub rpub;
        BandwidthMonitor::Counter* rx_counter;  // null: not counted
        ZenohSub zsub;

        [[no_unique_address]]
        DecompressorT decompress_ctx;
    };

public:
    static Subscriber createSubscriber(
        rclcpp::Node&,
        zenoh::Session&,
        const std::string&,
        const rclcpp::QoS& = rclcpp::SensorDataQoS{},
        DelayQueue* delay_q = nullptr,
        BandwidthMonitor* bw = nullptr);

    static Publisher createPublisher(
        rclcpp::Node&,
        zenoh::Session&,
        const std::string&,
        const rclcpp::QoS& = rclcpp::SensorDataQoS{},
        BandwidthMonitor* bw = nullptr);

    static std::shared_ptr<Subscriber> createSharedSubscriber(
        rclcpp::Node&,
        zenoh::Session&,
        const std::string&,
        const rclcpp::QoS& = rclcpp::SensorDataQoS{},
        DelayQueue* delay_q = nullptr,
        BandwidthMonitor* bw = nullptr);

    static std::shared_ptr<Publisher> createSharedPublisher(
        rclcpp::Node&,
        zenoh::Session&,
        const std::string&,
        const rclcpp::QoS& = rclcpp::SensorDataQoS{},
        BandwidthMonitor* bw = nullptr);

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

template<typename M, typename D, int C, typename P, typename S>
BaseAdapter<M, D, C, P, S>::Subscriber::Subscriber(
    rclcpp::Node& node,
    zenoh::Session& zsh,
    const std::string& topic,
    const rclcpp::QoS& qos,
    DelayQueue* delay_q,
    BandwidthMonitor* bw) :
    state{node},
    zpub{zsh.declare_publisher(topic.front() == '/' ? topic.substr(1) : topic)},
    tx_counter{
        bw ? bw->addChannel(topic, BandwidthMonitor::Direction::TX) : nullptr},
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

            if constexpr (USING_COMPRESSION)
            {
                if (!this->compress_ctx.compress(bytes, COMPRESSION_LEVEL))
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
                // Same reasoning for tx_counter (owned by EndPointNode too).
                ZenohPub* pub = &this->zpub;
                BandwidthMonitor::Counter* tx = this->tx_counter;
                delay_q->push(
                    [pub, tx](ByteBuffer&& b)
                    {
                        if (tx)
                        {
                            tx->add(b.size());
                        }
                        pub->put(zenoh::Bytes(std::move(b)));
                    },
                    std::move(bytes));
            }
            else
            {
                if (this->tx_counter)
                {
                    this->tx_counter->add(bytes.size());
                }
                this->zpub.put(zenoh::Bytes(std::move(bytes)));
            }
        })}
{
}

template<typename M, typename D, int C, typename P, typename S>
BaseAdapter<M, D, C, P, S>::Publisher::Publisher(
    rclcpp::Node& node,
    zenoh::Session& zsh,
    const std::string& topic,
    const rclcpp::QoS& qos,
    BandwidthMonitor* bw) :
    state{node},
    rpub{node.create_publisher<MsgT>(topic, qos)},
    rx_counter{
        bw ? bw->addChannel(topic, BandwidthMonitor::Direction::RX) : nullptr},
    zsub{zsh.declare_subscriber(
        topic.front() == '/' ? topic.substr(1) : topic,
        [this, logger = node.get_logger(), clock = node.get_clock(), topic](
            const zenoh::Sample& sample)
        {
            ByteBuffer bytes = sample.get_payload().as_vector();
            if (this->rx_counter)
            {
                this->rx_counter->add(bytes.size());
            }
            if constexpr (USING_COMPRESSION)
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

template<typename M, typename D, int C, typename P, typename S>
typename BaseAdapter<M, D, C, P, S>::Subscriber
    BaseAdapter<M, D, C, P, S>::createSubscriber(
        rclcpp::Node& node,
        zenoh::Session& zsh,
        const std::string& topic,
        const rclcpp::QoS& qos,
        DelayQueue* delay_q,
        BandwidthMonitor* bw)
{
    return Subscriber(node, zsh, topic, qos, delay_q, bw);
}

template<typename M, typename D, int C, typename P, typename S>
typename BaseAdapter<M, D, C, P, S>::Publisher
    BaseAdapter<M, D, C, P, S>::createPublisher(
        rclcpp::Node& node,
        zenoh::Session& zsh,
        const std::string& topic,
        const rclcpp::QoS& qos,
        BandwidthMonitor* bw)
{
    return Publisher(node, zsh, topic, qos, bw);
}

template<typename M, typename D, int C, typename P, typename S>
std::shared_ptr<typename BaseAdapter<M, D, C, P, S>::Subscriber>
    BaseAdapter<M, D, C, P, S>::createSharedSubscriber(
        rclcpp::Node& node,
        zenoh::Session& zsh,
        const std::string& topic,
        const rclcpp::QoS& qos,
        DelayQueue* delay_q,
        BandwidthMonitor* bw)
{
    return std::make_shared<Subscriber>(
        std::ref(node),
        std::ref(zsh),
        std::cref(topic),
        std::cref(qos),
        delay_q,
        bw);
}

template<typename M, typename D, int C, typename P, typename S>
std::shared_ptr<typename BaseAdapter<M, D, C, P, S>::Publisher>
    BaseAdapter<M, D, C, P, S>::createSharedPublisher(
        rclcpp::Node& node,
        zenoh::Session& zsh,
        const std::string& topic,
        const rclcpp::QoS& qos,
        BandwidthMonitor* bw)
{
    return std::make_shared<Publisher>(
        std::ref(node),
        std::ref(zsh),
        std::cref(topic),
        std::cref(qos),
        bw);
}
