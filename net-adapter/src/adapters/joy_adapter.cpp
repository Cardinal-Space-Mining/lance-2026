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

#include "joy_adapter.hpp"

#include <cmath>
#include <cstdint>
#include <algorithm>

#include "mem_helpers.hpp"
#include "../ros_utils.hpp"


using namespace util;


constexpr inline size_t bytesRequiredForBits(size_t bits)
{
    return (bits / 8) + static_cast<bool>(bits % 8);
}
constexpr inline size_t calcPackedJoySize(size_t n_axes, size_t n_btns)
{
    using StampT = sensor_msgs::msg::Joy::_header_type::_stamp_type;

    return (
        sizeof(StampT::_sec_type) +      //
        sizeof(StampT::_nanosec_type) +  //
        sizeof(uint8_t) * 2 +            // two bytes for num axes and num btns
        sizeof(int16_t) * n_axes +       // 2 bytes per axis
        bytesRequiredForBits(n_btns)     // 1 bit per button
    );
}

static inline int16_t quantizeAxisVal(float v)
{
    return static_cast<int16_t>(std::lrintf(
        std::clamp(v, -1.f, 1.f) *
        static_cast<float>(std::numeric_limits<int16_t>::max())));
}
static inline float dequantizeAxisVal(int16_t v)
{
    return static_cast<float>(v) /
           static_cast<float>(std::numeric_limits<int16_t>::max());
}


JoyAdapterSubState::JoyAdapterSubState(rclcpp::Node& node)
{
    declare_param(node, "max_joy_pub_freq", this->max_pub_freq, 100.f);
}

bool JoyAdapterSubState::freqFilterStatus()
{
    const system_time t = system_clock::now();
    const auto d = std::chrono::duration_cast<std::chrono::milliseconds>(
        t - this->prev_msg_time);
    const auto f = std::chrono::milliseconds(
        static_cast<int64_t>(1000.f / this->max_pub_freq));

    if (d >= f)
    {
        this->prev_msg_time = t;
        return true;
    }
    return false;
}


JoyAdapter::JoyAdapter(rclcpp::Node& node) : BaseT{node} {}

bool JoyAdapter::serializeMsg(
    ByteBuffer& bytes,
    const MsgT& msg,
    SubStateT& state)
{
    if (!state.freqFilterStatus())
    {
        return false;
    }

    const size_t n_axes = msg.axes.size();
    const size_t n_btns = msg.buttons.size();
    bytes.resize(calcPackedJoySize(n_axes, n_btns));

    uint8_t* ptr = bytes.data();
    writeAndIncrement(ptr, msg.header.stamp.sec);
    writeAndIncrement(ptr, msg.header.stamp.nanosec);
    writeAsAndIncrement<uint8_t>(ptr, n_axes);
    writeAsAndIncrement<uint8_t>(ptr, n_btns);
    for (float axis : msg.axes)
    {
        writeAndIncrement(ptr, quantizeAxisVal(axis));
    }
    size_t bit_count = 0;
    uint8_t bits = 0;
    for (int32_t btn : msg.buttons)
    {
        bits |= (static_cast<uint8_t>(static_cast<bool>(btn)) << bit_count);
        bit_count++;

        if (bit_count >= 8)
        {
            writeAndIncrement(ptr, bits);
            bit_count = 0;
            bits = 0;
        }
    }
    if (bit_count)
    {
        writeAndIncrement(ptr, bits);
    }

    return true;
}

bool JoyAdapter::deserializeMsg(
    MsgT& msg,
    const ByteBuffer& bytes,
    PubStateT& state)
{
    (void)state;

    const uint8_t* ptr = bytes.data();

    readAndIncrement(ptr, msg.header.stamp.sec);
    readAndIncrement(ptr, msg.header.stamp.nanosec);

    uint8_t n_axes, n_btns;
    readAndIncrement(ptr, n_axes);
    readAndIncrement(ptr, n_btns);

    if (bytes.size() < calcPackedJoySize(n_axes, n_btns))
    {
        return false;
    }

    msg.axes.reserve(n_axes);
    for (uint8_t i = 0; i < n_axes; i++)
    {
        int16_t q;
        readAndIncrement(ptr, q);
        msg.axes.push_back(dequantizeAxisVal(q));
    }

    msg.buttons.reserve(n_btns);
    {
        uint8_t bits = 0;
        for (uint8_t i = 0; i < n_btns; i++)
        {
            const uint8_t off = i % 8;
            if (!off)
            {
                readAndIncrement(ptr, bits);
            }
            msg.buttons.push_back(static_cast<int32_t>((bits >> off) & 0x1));
        }
    }

    return true;
}
