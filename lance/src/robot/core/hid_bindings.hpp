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
 * @file hid_bindings.hpp
 * @brief Gamepad button, trigger, and axis mapping for teleoperation and mission control interaction.
 *
 * Implements standard Microsoft Xbox controller layouts mapped via Linux joystick driver (`joy_node`).
 * Supports two distinct operational contexts:
 *   1. Normal Teleoperation: Direct driving, trencher power, hopper conveyor, linear actuator tilt,
 *      and semi-autonomous mining/offloading triggers.
 *   2. Mission Control Override: Activated by holding the center Xbox button (`MissionControlOverrideButton`),
 *      allowing the operator to cycle modes (disabled, teleop, auto), place interactive waypoint cursors,
 *      and tune target offload zones.
 */

#include <csm_utils/joy_utils.hpp>


/**
 * @struct XboxControllerMappings
 * @brief Raw button indices and axis IDs for Xbox One / Series X / 360 USB controllers.
 */
struct XboxControllerMappings
{
    enum
    {
        BUTTON_A = 0,
        BUTTON_B = 1,
        BUTTON_X = 2,
        BUTTON_Y = 3,
        BUTTON_LEFT_BUMPER = 4,
        BUTTON_RIGHT_BUMPER = 5,
        BUTTON_LEFT_CENTER = 6,  // View / Back button
        BUTTON_RIGHT_CENTER = 7, // Menu / Start button
        BUTTON_CENTER = 8,       // Xbox Logo button
        BUTTON_LEFT_STICK = 9,   // Left stick click (L3)
        BUTTON_RIGHT_STICK = 10, // Right stick click (R3)

        NUM_BUTTONS = 11
    };
    enum
    {
        AXIS_LEFT_X = 0,          // Left stick horizontal (-1: left, +1: right)
        AXIS_LEFT_Y = 1,          // Left stick vertical (+1: forward, -1: reverse)
        AXIS_LEFT_TRIGGER = 2,    // Left analog trigger (-1: released, +1: pressed)
        AXIS_RIGHT_X = 3,         // Right stick horizontal
        AXIS_RIGHT_Y = 4,         // Right stick vertical
        AXIS_RIGHT_TRIGGER = 5,   // Right analog trigger (-1: released, +1: pressed)
        AXIS_DPAD_HORIZONTAL = 6, // D-pad X (-1: right, +1: left)
        AXIS_DPAD_VERTICAL = 7,   // D-pad Y (+1: up, -1: down)

        NUM_AXES = 8
    };
    enum
    {
        DPAD_UP_VAL = 1,
        DPAD_DOWN_VAL = -1,
        DPAD_LEFT_VAL = 1,
        DPAD_RIGHT_VAL = -1
    };

    // Static typed button wrappers from csm_utils
    using ButtonA = util::StaticJoyButton<BUTTON_A>;
    using ButtonB = util::StaticJoyButton<BUTTON_B>;
    using ButtonY = util::StaticJoyButton<BUTTON_Y>;
    using ButtonX = util::StaticJoyButton<BUTTON_X>;

    using ButtonLeftBumper = util::StaticJoyButton<BUTTON_LEFT_BUMPER>;
    using ButtonRightBumper = util::StaticJoyButton<BUTTON_RIGHT_BUMPER>;
    using ButtonLeftCenter = util::StaticJoyButton<BUTTON_LEFT_CENTER>;
    using ButtonRightCenter = util::StaticJoyButton<BUTTON_RIGHT_CENTER>;
    using ButtonCenter = util::StaticJoyButton<BUTTON_CENTER>;
    using ButtonLeftStick = util::StaticJoyButton<BUTTON_LEFT_STICK>;
    using ButtonRightStick = util::StaticJoyButton<BUTTON_RIGHT_STICK>;

    // Static typed axis wrappers
    using AxisLeftX = util::StaticJoyAxis<AXIS_LEFT_X>;
    using AxisLeftY = util::StaticJoyAxis<AXIS_LEFT_Y>;
    using AxisLeftTrigger = util::StaticJoyAxis<AXIS_LEFT_TRIGGER>;
    using AxisRightX = util::StaticJoyAxis<AXIS_RIGHT_X>;
    using AxisRightY = util::StaticJoyAxis<AXIS_RIGHT_Y>;
    using AxisRightTrigger = util::StaticJoyAxis<AXIS_RIGHT_TRIGGER>;

    // Static typed D-pad POV direction wrappers
    using DPadUp = util::StaticJoyPov<AXIS_DPAD_VERTICAL, DPAD_UP_VAL>;
    using DPadDown = util::StaticJoyPov<AXIS_DPAD_VERTICAL, DPAD_DOWN_VAL>;
    using DPadLeft = util::StaticJoyPov<AXIS_DPAD_HORIZONTAL, DPAD_LEFT_VAL>;
    using DPadRight = util::StaticJoyPov<AXIS_DPAD_HORIZONTAL, DPAD_RIGHT_VAL>;
};


namespace lance
{

#define Xbox XboxControllerMappings


// =============================================================================
// Direct Teleoperation Bindings (Normal Driving & Actuation)
// =============================================================================

/// Immediate stop / cancel current action
using DisableAllActionsButton = Xbox::ButtonA;

/// Speed multiplier selection (Low = 0.3x, Medium = 0.7x, High = 1.0x)
using TeleopLowSpeedButton = Xbox::ButtonB;
using TeleopMediumSpeedButton = Xbox::ButtonY;
using TeleopHighSpeedButton = Xbox::ButtonX;

/// Differential drive locomotion axes
using TeleopDriveForwardAxis = Xbox::AxisLeftY;
using TeleopDriveRotationAxis = Xbox::AxisLeftX;

/// Trencher cutter head throttle and direction reverse
using TeleopTrencherSpeedAxis = Xbox::AxisRightTrigger;
using TeleopTrencherInvertButton = Xbox::ButtonRightBumper;

/// Hopper conveyor belt throttle and direction reverse
using TeleopHopperSpeedAxis = Xbox::AxisLeftTrigger;
using TeleopHopperInvertButton = Xbox::ButtonLeftBumper;

/// Linear actuator pitch / depth adjustment
using TeleopHopperActuateAxis = Xbox::AxisRightY;

/// Semi-autonomous driver assist routine toggles
using AssistedMiningToggleButton = Xbox::ButtonLeftCenter;
using AssistedOffloadToggleButton = Xbox::ButtonRightCenter;

/// Safety constraint override toggles during assisted excavation
using ToggleMiningObstacleConstraintButton = Xbox::DPadUp;
using ToggleMiningHopperConstraintButton = Xbox::DPadRight;
using ToggleMiningZoneConstraintButton = Xbox::DPadDown;
using ToggleMiningStallConstraintButton = Xbox::DPadLeft;


// =============================================================================
// Mission Control Override Bindings (Held with Xbox Logo Button)
// =============================================================================

/// Master modifier chord button to access UI / mode switching
using MissionControlOverrideButton = Xbox::ButtonCenter;

/// System operational mode selection
using SetDisabledModeButton = Xbox::ButtonA;
using SetTeleopModeButton = Xbox::ButtonB;
using SetAutoModeButton = Xbox::ButtonX;
using ToggleTestModeButton = Xbox::ButtonY;
using ToggleQuickAutoButton = Xbox::DPadDown;
using ToggleAssistAsAutoButton = Xbox::DPadUp;

/// Mission Control interactive cursor selection & dispatch
using SetTravCursorButton = Xbox::ButtonRightBumper;
using ConfirmCursorTargetButton = Xbox::ButtonLeftBumper;
using SetMiningCursorButton = Xbox::ButtonLeftCenter;
using SetOffloadCursorButton = Xbox::ButtonRightCenter;

/// 2D Map navigation waypoint cursor placement
using TraversalCursorRotAxis = Xbox::AxisRightX;
using TraversalCursorPosAxes =
    util::StaticJoyStickAxes<Xbox::AxisLeftY::IDX, Xbox::AxisLeftX::IDX>;

/// Interactive offload zone target resizing and orientation snapping
using OffloadCursorScaleUpAxis = Xbox::AxisRightTrigger;
using OffloadCursorScaleDownAxis = Xbox::AxisLeftTrigger;
using OffloadCursorAlignButton = Xbox::ButtonLeftStick;


#undef Xbox

};  // namespace lance
