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

#include "robot/core/robot_params.hpp"
#include "robot/core/motor_interface.hpp"
#include "robot/sensing/sensing_interfaces.hpp"


namespace lance
{

class LocalizationController
{
    friend class TelemetrySerializer;
    friend class TelemetryDeserializer;

    using RclNode = rclcpp::Node;

public:
    LocalizationController(
        const RobotParams&,
        SensingInterfaces&);
    ~LocalizationController() = default;

public:
    void initialize();
    bool isFinished();
    void setCancelled();

    void iterate(
        const RobotMotorStatus& motor_status,
        RobotMotorCommands& commands);

protected:
    enum class Stage
    {
        INITIALIZATION,
        SEARCHING,
        ALIGN_HEADING,
        ADJUST_RANGE,
        COMPLETE_SPIN,  // localized; finish the remainder of the 360
        FINISHED
    };

protected:
    // Accumulates how far the robot has turned in total during localization
    // (searching, aligning and completing the spin all count toward 360)
    void updateSpin(const RobotMotorStatus& motor_status);
    bool spinComplete() const;

protected:
    const RobotParams& params;
    const TfCache& tf_cache;
    ReflectorHintInterface& refl_hint_interface;

    Stage stage{Stage::FINISHED};

    // Rotation is measured two ways; the spin completes when either reaches
    // 360 deg, so a lagging source (e.g. lidar odom during fast in-place
    // turns) can't cause extra rotations
    float spin_odom_rad{0.f};
    float spin_tracks_rad{0.f};
    float spin_accum_rad{0.f};  // max of the two
    float last_yaw{0.f};
    bool have_last_yaw{false};
    bool spin_logged{false};
};

};  // namespace lance
