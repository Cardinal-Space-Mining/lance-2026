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

#include <net_adapter/msg/bytes.hpp>

#include <csm_utils/joy_utils.hpp>
#include <csm_utils/ros_utils.hpp>

#include "robot/core/robot_params.hpp"
#include "robot/core/motor_interface.hpp"
#include "robot/sensing/sensing_interfaces.hpp"
#include "robot/control/shared/shared_controllers.hpp"


namespace lance
{

class TeleopController : public util::UsingRosAliases
{
    friend class TelemetrySerializer;
    friend class TelemetryDeserializer;

    using BytesMsg = net_adapter::msg::Bytes;
    using JoyState = util::JoyState;

public:
    TeleopController(
        RclNode&,
        const RobotParams&,
        SensingInterfaces&,
        SharedControllerCollection&);
    ~TeleopController() = default;

public:
    void initialize();
    void setCancelled();

    void iterate(
        uint8_t opts,
        const JoyState& joy,
        const RobotMotorStatus& motor_status,
        RobotMotorCommands& commands);

protected:
    enum class Operation
    {
        MANUAL = 0,
        ASSISTED_MINING,
        ASSISTED_OFFLOAD,

        PLANNED_TRAVERSAL,
        PLANNED_MINING_T,
        PLANNED_MINING_E,
        PLANNED_OFFLOAD_T,
        PLANNED_OFFLOAD_E
    };

protected:
    bool handleGlobalControls(const JoyState&);
    void cancelCurrentCommand();
    void clearRemoteCommand();
    void handleRemoteCommand();
    bool iterateCurrentCommand(
        const JoyState&,
        const RobotMotorStatus&,
        RobotMotorCommands&);
    void handleManualControl(
        uint8_t opts,
        const JoyState&,
        const RobotMotorStatus&,
        RobotMotorCommands&);

protected:
    bool iterateAssistedMining(
        const JoyState&,
        const RobotMotorStatus&,
        RobotMotorCommands&);
    bool iterateAssistedOffload(
        const JoyState&,
        const RobotMotorStatus&,
        RobotMotorCommands&);
    void iteratePlannedTraversal(
        const RobotMotorStatus&,
        RobotMotorCommands&);
    void iteratePlannedMining(
        const RobotMotorStatus&,
        RobotMotorCommands&);
    void iteratePlannedOffload(
        const RobotMotorStatus&,
        RobotMotorCommands&);

protected:
    const RobotParams& params;
    SensingInterfaces& sensing_interfaces;

    MiningController& mining_controller;
    OffloadController& offload_controller;
    TraversalController& traversal_controller;

    RclSubPtr<BytesMsg> remote_commands_sub;
    BytesMsg::ConstSharedPtr remote_command;

    Operation op_mode{Operation::MANUAL};
    float driving_rps_scalar;
};

};  // namespace lance
