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

/**
 * @file robot_controller.hpp
 * @brief Top-level robot control coordinator managing operational mode transitions and subsystem dispatch.
 *
 * `RobotController` serves as the central orchestration hub on the robot platform:
 *   - Parses the packed INT32 status word into high-level `ControlMode` (DISABLED, TELEOPERATED, AUTONOMOUS)
 *     and `ControlOpts` bitflags.
 *   - Detects mode state changes and fires clean transition handlers (initialize / cancel).
 *   - Maintains shared models across all control contexts:
 *       * `CollectionState`: Real-time regolith excavation accumulation and hopper belt distribution.
 *       * `StallState`: Current-limiting and velocity-deficit stall detection across tracks and trencher.
 *       * `TfCache`: Continuous spatial tracking of `map`, `odom`, and `robot` reference frames.
 *   - Delegates cycle execution to either `TeleopController` or `AutoController`.
 */

#include <cstdint>

#include <rclcpp/rclcpp.hpp>

#include <csm_utils/joy_utils.hpp>
#include <csm_utils/ros_utils.hpp>

#include "robot/core/robot_params.hpp"
#include "robot/core/robot_status.hpp"
#include "robot/core/stall_analyzer.hpp"
#include "robot/core/motor_interface.hpp"
#include "robot/core/collection_state.hpp"
#include "robot/sensing/sensing_interfaces.hpp"

#include "auto/auto_controller.hpp"
#include "shared/shared_controllers.hpp"
#include "teleop/teleop_controller.hpp"


namespace lance
{

/**
 * @class RobotController
 * @brief Master controller executing mode management, safety supervision, and actuator dispatch.
 */
class RobotController : public util::UsingRosAliases
{
    friend class TelemetrySerializer;

    using JoyState = util::JoyState;

public:
    /**
     * @brief Construct RobotController and initialize all subsystems and perception interfaces.
     * @param node Hosting ROS 2 node reference.
     */
    RobotController(RclNode& node);
    ~RobotController() = default;

public:
    /// @brief Read-only accessor for current hopper conveyor berm model.
    const HopperState& hopperState() const;

    /// @brief Read-only accessor for current motor stall diagnosis.
    const StallState& stallState() const;

    /// @brief Read-only accessor for active robot parameters.
    const RobotParams& getParams() const;

    /// @brief Read-only accessor for internal TF2 buffer.
    const TfCache::Tf2Buffer& getTfBuffer() const;

    /**
     * @brief Periodic execution cycle invoked at the control timer rate (~20 Hz).
     *
     * @param ctrl_status Packed INT32 status word from watchdog.
     * @param joy Processed operator gamepad state.
     * @param motor_status Latest feedback telemetry from motor controllers.
     * @param motor_faults Hardware fault flags from motor controllers.
     * @param[out] commands Populated motor control packet sent to hardware.
     */
    void iterate(
        int32_t ctrl_status,
        const JoyState& joy,
        const RobotMotorStatus& motor_status,
        const RobotMotorFaults& motor_faults,
        RobotMotorCommands& commands);

protected:
    /**
     * @brief Re-scale linear actuator stroke feedback if TEST_MODE is active to prevent benchtop over-travel.
     */
    const RobotMotorStatus& handleTestModeStateInjection(
        const RobotMotorStatus& ref,
        uint8_t ctrl_opts);

protected:
    RobotParams params;                          ///< Loaded ROS parameters and physical arena bounds.
    CollectionState collection_state;            ///< Regolith volume intake and hopper bed model.
    StallState stall_state;                      ///< Motor stall and jam diagnostics.
    SensingInterfaces sensing_interfaces;        ///< TF cache, path planning, and perception interfaces.
    SharedControllerCollection shared_controllers; ///< Modular sub-controllers (traversal, mining, offload, localization).

    AutoController auto_controller;              ///< Autonomous mission state machine.
    TeleopController teleop_controller;          ///< Teleoperated driver assist and direct driving handler.

    ControlMode control_mode{ControlMode::DISABLED}; ///< Currently active operating mode.
    RobotMotorStatus filtered_status;            ///< Working buffer for sanitized/remapped motor feedback.
};

};  // namespace lance
