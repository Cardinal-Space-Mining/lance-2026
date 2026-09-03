/*******************************************************************************
*   Copyright (C) 2025-2026 Cardinal Space Mining Club                         *
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

#include <cstdint>

#include <rclcpp/rclcpp.hpp>

#include <net_adapter/msg/bytes.hpp>

#include <csm_utils/ros_utils.hpp>


namespace lance
{

/* --- Telemetry Specification ---
 * All telemetry is packaged into a binary blob and sent from the robot
 * to the client-side decoder, where it is decoded and publishers are used as
 * needed.
 * The telemetry blob consists of any number of packets, each of which starts
 * with an id (TelemetryType) followed by the packet data, whose size can vary.
 * As such, it is the responsiblity of each individual (per telemetry type)
 * decoder to read the correct length of bytes after it's id appears in the
 * blob.
 * The control state telemetry type itself encapsulates another layer which
 * works similarly to this - that is, each controller may recursively contain
 * other controller states. The encoding and decoding process should work
 * recursively to handle decoding the overall state properly. */

enum class TelemetryType : uint8_t
{
    INVALID_ID = 0,

    ARENA_TF,
    ROBOT_STATE,
    CTRL_STATE
};

enum class ControllerType : uint8_t
{
    INVALID_ID = 0,

    TELEOP,
    AUTO,

    AUTO_MINING,
    AUTO_OFFLOAD,

    MINING,
    OFFLOAD,
    LOCALIZATION,
    TRAVERSAL
};

class TelemetryBase : public util::UsingRosAliases
{
public:
    using BytesMsg = net_adapter::msg::Bytes;
    using BytesSharedPub = rclcpp::Publisher<BytesMsg>::SharedPtr;
    using BytesSharedSub = rclcpp::Subscription<BytesMsg>::SharedPtr;

    using Bytes = BytesMsg::_data_type;
    using Byte = Bytes::value_type;
    using BytePtr = const Byte*;
    using BytePtrRef = const Byte*&;

    constexpr static uint8_t AUTO_MINING_STATE_STAGE_MASK = 0x3f;
    constexpr static uint8_t AUTO_MINING_STATE_GRID_BIT = 0x40;
    constexpr static uint8_t AUTO_MINING_STATE_PATHS_BIT = 0x80;
    constexpr static size_t AUTO_MINING_MAX_GRID_DIVS = 64;
    constexpr static size_t AUTO_MINING_MAX_PATHS = 64;
};

#ifndef ENABLE_MINING_PLANNER_DEBUG
#define ENABLE_MINING_PLANNER_DEBUG 1
#endif

};  // namespace lance
