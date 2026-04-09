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

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>

#include "util/joy_utils.hpp"
#include "util/ros_utils.hpp"

#include "robot/core/robot_params.hpp"
#include "robot/sensing/tf_cache.hpp"
#include "robot/telemetry/deserializer.hpp"

#include "watchdog.hpp"
#include "zone_pub.hpp"


namespace lance
{

class InputInterface : public util::UsingRosAliases
{
    using Int32Msg = std_msgs::msg::Int32;
    using JoyMsg = sensor_msgs::msg::Joy;
    using PoseStampedMsg = geometry_msgs::msg::PoseStamped;
    using PointStampedMsg = geometry_msgs::msg::PointStamped;

    using JoyState = util::JoyState;
    using ZoneBounds = RobotParams::ZoneBounds;

public:
    InputInterface(
        RclNode&,
        const TfCache&,
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
    void handleJoy(const JoyMsg&);
    void handleClickedPoint(const PointStampedMsg&);
    void handleInterfacePubs();

    void handlePassthroughState();
    void handleOverrideState();

    void initTravCursorState();
    void initMiningCursorState();
    void initOffloadCursorState();

    void handleTravCursorState();
    void handleMiningCursorState();
    void handleOffloadCursorState();

    bool handleCommonOverrides();
    void homeTravCursor();
    void iterateTravCursor();
    void iterateMiningCursor();
    void iterateOffloadCursor();

    void publishTravTarget();
    void publishMiningTarget();
    void publishOffloadTarget();

    bool isCursorState() const;

private:
    const TfCache& tf_cache;
    TelemetryDeserializer& telemetry;
    WatchDog& watchdog;
    const ZoneBounds& bounds;

    RclPubPtr<JoyMsg> joy_pub;
    RclSubPtr<JoyMsg> joy_sub;
    RclPubPtr<PoseStampedMsg> traversal_target_pub;
    RclSubPtr<PointStampedMsg> clicked_point_sub;
    RclTimer::SharedPtr interface_pub_timer;

    JoyState joy_state;
    PoseStampedMsg traversal_cursor;

    State state{State::PASSTHROUGH};
};

};  // namespace lance
