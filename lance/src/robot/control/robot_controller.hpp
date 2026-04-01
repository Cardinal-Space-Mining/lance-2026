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

#include "util/joy_utils.hpp"
#include "util/ros_utils.hpp"

#include "robot/core/robot_params.hpp"
#include "robot/core/robot_status.hpp"
#include "robot/core/motor_interface.hpp"
#include "robot/core/collection_state.hpp"
#include "robot/sensing/sensing_interfaces.hpp"

#include "teleop_controller.hpp"
#include "auto/auto_controller.hpp"
#include "shared/shared_controllers.hpp"


namespace lance
{

class RobotController : public util::UsingRosAliases
{
    friend class TelemetrySerializer;

    using JoyState = util::JoyState;

public:
    RobotController(RclNode&);
    ~RobotController() = default;

public:
    const HopperState& hopperState() const;
    const RobotParams& getParams() const;
    const TfCache::Tf2Buffer& getTfBuffer() const;

    void iterate(
        int32_t ctrl_status,
        const JoyState& joy,
        const RobotMotorStatus& motor_status,
        RobotMotorCommands& commands);

protected:
    const RobotMotorStatus& handleTestModeStateInjection(
        const RobotMotorStatus& ref,
        int32_t ctrl_status);

protected:
    RobotParams params;
    CollectionState collection_state;
    SensingInterfaces sensing_interfaces;
    SharedControllerCollection shared_controllers;

    AutoController auto_controller;
    TeleopController teleop_controller;

    ControlMode control_mode{ControlMode::DISABLED};
    RobotMotorStatus filtered_status;
};

};  // namespace lance
