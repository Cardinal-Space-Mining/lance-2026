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

#include "robot/control/robot_controller.hpp"
#include "robot/sensing/tf_cache.hpp"

#include "telemetry.hpp"


namespace lance
{

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
    void addRobotState(Bytes&, const RobotController&);
    void addControlState(Bytes&, const RobotController&);

    void addTeleopController(Bytes&, const TeleopController&);
    void addAutoController(Bytes&, const AutoController&);
    void addAutoMiningController(Bytes&, const AutoMiningController&);
    void addAutoOffloadController(Bytes&, const AutoOffloadController&);
    void addMiningController(Bytes&, const MiningController&);
    void addOffloadController(Bytes&, const OffloadController&);
    void addLocController(Bytes&, const LocalizationController&);
    void addTravController(Bytes&, const TraversalController&);

    void addMiningPlannerDebug(Bytes&, const AutoMiningController&, Byte&);

protected:
    BytesSharedPub pub;

    time_point last_tf_pub, last_path_pub, last_auto_mining_vis_pub;

    const float throttled_pub_freq;
};

};  // namespace lance
