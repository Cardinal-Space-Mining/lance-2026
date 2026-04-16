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

#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/int32.hpp>

#include <sensor_msgs/msg/joy.hpp>

#include <geometry_msgs/msg/point_stamped.hpp>

#include <net_adapter/msg/bytes.hpp>

#include "util/joy_utils.hpp"
#include "util/ros_utils.hpp"

#include "robot/core/robot_params.hpp"
#include "robot/model/geometry.hpp"
#include "robot/sensing/tf_cache.hpp"
#include "robot/telemetry/markers.hpp"
#include "robot/telemetry/deserializer.hpp"

#include "watchdog.hpp"
#include "zone_pub.hpp"


namespace lance
{

class AdvancedControls : public util::UsingRosAliases
{
    using Int32Msg = std_msgs::msg::Int32;
    using JoyMsg = sensor_msgs::msg::Joy;
    using PointStampedMsg = geometry_msgs::msg::PointStamped;
    using BytesMsg = net_adapter::msg::Bytes;

    using JoyState = util::JoyState;
    using ZoneBounds = RobotParams::ZoneBounds;

public:
    AdvancedControls(
        RclNode&,
        const TfCache&,
        MarkerManager&,
        TelemetryDeserializer&,
        WatchDog&,
        const ZoneBounds&);

protected:
    enum class State
    {
        PASSTHROUGH = 0,
        OVERRIDE,
        TRAV_CURSOR,
        MINING_CURSOR,
        OFFLOAD_CURSOR
    };

protected:
    void initMarkers();

    void handleJoy(const JoyMsg&);
    void handleClickedPoint(const PointStampedMsg&);
    void handleInterfacePubs();

    void iteratePassthroughMode();
    void iterateOverrideMode();
    bool handleCommonOverrides();

    bool isCursorState() const;

protected:
    void initTravCursorMode();
    void iterateTravCursorMode();
    void iterateTravCursorCtrl();
    void publishTravTarget();
    void publishTravVisuals();
    void updateFootprintMarkers();

protected:
    void initMiningCursorMode();
    void iterateMiningCursorMode();
    void iterateMiningCursorCtrl();
    void publishMiningTarget();
    void publishMiningVisuals();
    bool updateMiningMarkers();

protected:
    void initOffloadCursorMode();
    void recalcOffloadRange();
    void recalcOffloadTarget();
    void iterateOffloadCursorMode();
    void iterateOffloadCursorCtrl();
    void publishOffloadTarget();
    void publishOffloadVisuals();
    void updateOffloadMarkers();

private:
    const TfCache& tf_cache;
    MarkerManager& markers;
    TelemetryDeserializer& telemetry;
    WatchDog& watchdog;
    const ZoneBounds& bounds;
    RclClock::ConstSharedPtr rcl_clock;

    RclPubPtr<JoyMsg> joy_pub;
    RclSubPtr<JoyMsg> joy_sub;
    RclPubPtr<BytesMsg> commands_pub;
    RclSubPtr<PointStampedMsg> clicked_point_sub;
    RclTimer::SharedPtr interface_pub_timer;

    JoyState joy_state;
    geom::Pose3f cursor_pose;
    KeyFrame cursor_frame_id;

    geom::Vec2f offload_zone_norm;
    geom::Vec2f offload_footprint;
    geom::Box2f offload_target_range;
    geom::Pose2f offload_target;
    float offload_manual_off;
    float offload_vis_range;

    size_t footprint_markers_id;
    size_t mining_markers_id;
    size_t offload_markers_id;

    State state{State::PASSTHROUGH};
};

};  // namespace lance
