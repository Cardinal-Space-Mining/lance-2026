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

#include "robot/sensing/tf_cache.hpp"
#include "robot/telemetry/deserializer.hpp"

#include "watchdog.hpp"


namespace lance
{

class InputInterface : public util::UsingRosAliases
{
    using Int32Msg = std_msgs::msg::Int32;
    using JoyMsg = sensor_msgs::msg::Joy;
    using PoseStampedMsg = geometry_msgs::msg::PoseStamped;
    using PointStampedMsg = geometry_msgs::msg::PointStamped;

    using JoyState = util::JoyState;

public:
    InputInterface(RclNode&, const TfCache&, TelemetryDeserializer&, WatchDog&);

protected:
    enum : uint32_t
    {
        STATE_NONE = 0,
        STATE_MC_INPUT_OVERRIDE = (1 << 0),
        STATE_CURSOR_ENABLED = (1 << 1)
    };

protected:
    void handleJoy(const JoyMsg&);
    void handleClickedPoint(const PointStampedMsg&);
    void handleInterfacePubs();

    bool hasState(uint32_t) const;
    void setState(uint32_t);
    void clearState(uint32_t);
    bool toggleState(uint32_t);

private:
    const TfCache& tf_cache;
    TelemetryDeserializer& telemetry;
    WatchDog& watchdog;

    RclPubPtr<JoyMsg> joy_pub;
    RclSubPtr<JoyMsg> joy_sub;
    RclPubPtr<PoseStampedMsg> traversal_target_pub;
    RclSubPtr<PointStampedMsg> clicked_point_sub;
    RclTimer::SharedPtr interface_pub_timer;

    JoyState joy_state;
    PoseStampedMsg traversal_cursor;

    uint32_t state{STATE_NONE};
};

};  // namespace lance
