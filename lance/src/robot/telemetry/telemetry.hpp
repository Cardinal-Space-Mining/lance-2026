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

#include <chrono>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include <tf2_ros/transform_broadcaster.hpp>

#include <net_adapter/msg/bytes.hpp>

#include "util/pub_map.hpp"

#include "robot/core/tf_cache.hpp"
#include "robot/control/robot_controller.hpp"


namespace lance
{

class TelemetryBase
{
public:
    using RclNode = rclcpp::Node;
    using BytesMsg = net_adapter::msg::Bytes;
    using BytesSharedPub = rclcpp::Publisher<BytesMsg>::SharedPtr;
    using BytesSharedSub = rclcpp::Subscription<BytesMsg>::SharedPtr;

    using Bytes = BytesMsg::_data_type;
    using Byte = Bytes::value_type;
    using BytePtr = const Byte*;
    using BytePtrRef = const Byte*&;

    static constexpr char const* TELEMETRY_TOPIC = "lance/telemetry";
};


class TelemetrySerializer : public TelemetryBase
{
    using steady_clock = std::chrono::steady_clock;
    using time_point = steady_clock::time_point;

public:
    TelemetrySerializer(RclNode& node, float pub_throttle_freq);

public:
    void update(const RobotController&);

protected:
    bool filterFreq(time_point&);

    void addArenaTf(Bytes&, const TfCache&);
    void addCollectionState(Bytes&, const CollectionState&);
    void addControlState(Bytes&, const RobotController&);

    void addTeleopController(Bytes&, const TeleopController&);
    void addAutoController(Bytes&, const AutoController&);
    void addAutoMiningController(Bytes&, const AutoMiningController&);
    void addAutoOffloadController(Bytes&, const AutoOffloadController&);
    void addMiningController(Bytes&, const MiningController&);
    void addOffloadController(Bytes&, const OffloadController&);
    void addLocController(Bytes&, const LocalizationController&);
    void addTravController(Bytes&, const TraversalController&);

protected:
    BytesSharedPub pub;

    time_point last_tf_pub, last_path_pub;

    const float throttled_pub_freq;
};


class TelemetryDeserializer : public TelemetryBase
{
    using GenericPubMap = util::GenericPubMap;
    using Tf2Broadcaster = tf2_ros::TransformBroadcaster;
    using ConstSharedClock = rclcpp::Clock::ConstSharedPtr;

public:
    TelemetryDeserializer(
        RclNode& node,
        TfCache& tf_cache);

public:
    GenericPubMap& getPubMap();

protected:
    void accept(const BytesMsg&);

protected:
    bool pubArenaTf(BytePtrRef, BytePtr);
    bool pubCollectionState(BytePtrRef, BytePtr);
    bool pubControlState(BytePtrRef, BytePtr);

    bool pubDerivedController(BytePtrRef, BytePtr);

    bool pubTeleopController(BytePtrRef, BytePtr);
    bool pubAutoController(BytePtrRef, BytePtr);
    bool pubAutoMiningController(BytePtrRef, BytePtr);
    bool pubAutoOffloadController(BytePtrRef, BytePtr);
    bool pubMiningController(BytePtrRef, BytePtr);
    bool pubOffloadController(BytePtrRef, BytePtr);
    bool pubLocController(BytePtrRef, BytePtr);
    bool pubTravController(BytePtrRef, BytePtr);

protected:
    GenericPubMap pub_map;
    TfCache& tf_cache;
    Tf2Broadcaster tf_broadcaster;
    ConstSharedClock rcl_clock;

    BytesSharedSub sub;

    std::vector<std::string> ctrl_chain;
};

};  // namespace lance
