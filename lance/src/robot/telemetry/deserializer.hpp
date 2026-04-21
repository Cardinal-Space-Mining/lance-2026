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

#include <string>
#include <vector>

#include <tf2_ros/transform_broadcaster.hpp>

#include <visualization_msgs/msg/marker_array.hpp>

#include "util/pub_map.hpp"
#include "robot/sensing/tf_cache.hpp"

#include "markers.hpp"
#include "telemetry.hpp"


namespace lance
{

class TelemetryDeserializer : public TelemetryBase
{
    using GenericPubMap = util::GenericPubMap;
    using Tf2Broadcaster = tf2_ros::TransformBroadcaster;

    using MarkerMsg = visualization_msgs::msg::Marker;
    using MarkerArrayMsg = visualization_msgs::msg::MarkerArray;

public:
    TelemetryDeserializer(RclNode&, TfCache&, MarkerManager&);

public:
    GenericPubMap& getPubMap();

protected:
    void initMarkers();
    void accept(const BytesMsg&);

protected:
    bool pubArenaTf(BytePtrRef, BytePtr);
    bool pubRobotState(BytePtrRef, BytePtr);
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
#if ENABLE_MINING_PLANNER_DEBUG
    bool pubMiningPlannerPaths(BytePtrRef, BytePtr);
    bool pubMiningPlannerGrid(BytePtrRef, BytePtr);
#endif

    void addMiningMarker(uint8_t, float);
    void addOffloadMarker(float);

protected:
    GenericPubMap pub_map;
    TfCache& tf_cache;
    MarkerManager& markers;
    Tf2Broadcaster tf_broadcaster;
    RclClock::ConstSharedPtr rcl_clock;

    BytesSharedSub sub;

    const size_t mining_marker_id;
    const size_t offload_marker_id;
#if ENABLE_MINING_PLANNER_DEBUG
    const size_t auto_mining_paths_marker_id;
    const size_t auto_mining_grid_marker_id;
    uint8_t active_mining_directions_mask{0};
#endif

    std::vector<std::string> ctrl_chain;
    float last_hopper_volume{0.f};
    float offloaded_volume{0.f};
};

};  // namespace lance
