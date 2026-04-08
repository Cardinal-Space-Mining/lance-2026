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

#include "util/joy_utils.hpp"


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
        BUTTON_LEFT_CENTER = 6,
        BUTTON_RIGHT_CENTER = 7,
        BUTTON_CENTER = 8,
        BUTTON_LEFT_STICK = 9,
        BUTTON_RIGHT_STICK = 10,

        NUM_BUTTONS = 11
    };
    enum
    {
        AXIS_LEFT_X = 0,
        AXIS_LEFT_Y = 1,
        AXIS_LEFT_TRIGGER = 2,
        AXIS_RIGHT_X = 3,
        AXIS_RIGHT_Y = 4,
        AXIS_RIGHT_TRIGGER = 5,
        AXIS_DPAD_HORIZONTAL = 6,
        AXIS_DPAD_VERTICAL = 7,

        NUM_AXES = 8
    };
    enum
    {
        DPAD_UP_VAL = 1,
        DPAD_DOWN_VAL = -1,
        DPAD_LEFT_VAL = 1,
        DPAD_RIGHT_VAL = -1
    };

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

    using AxisLeftX = util::StaticJoyAxis<AXIS_LEFT_X>;
    using AxisLeftY = util::StaticJoyAxis<AXIS_LEFT_Y>;
    using AxisLeftTrigger = util::StaticJoyAxis<AXIS_LEFT_TRIGGER>;
    using AxisRightX = util::StaticJoyAxis<AXIS_RIGHT_X>;
    using AxisRightY = util::StaticJoyAxis<AXIS_RIGHT_Y>;
    using AxisRightTrigger = util::StaticJoyAxis<AXIS_RIGHT_TRIGGER>;

    using DPadUp = util::StaticJoyPov<AXIS_DPAD_VERTICAL, DPAD_UP_VAL>;
    using DPadDown = util::StaticJoyPov<AXIS_DPAD_VERTICAL, DPAD_DOWN_VAL>;
    using DPadLeft = util::StaticJoyPov<AXIS_DPAD_HORIZONTAL, DPAD_LEFT_VAL>;
    using DPadRight = util::StaticJoyPov<AXIS_DPAD_HORIZONTAL, DPAD_RIGHT_VAL>;
};


namespace lance
{

#define Xbox XboxControllerMappings


// --- Robot Controls ----------------------------------------------------------

using DisableAllActionsButton = Xbox::ButtonA;

using TeleopLowSpeedButton = Xbox::ButtonB;
using TeleopMediumSpeedButton = Xbox::ButtonY;
using TeleopHighSpeedButton = Xbox::ButtonX;

using TeleopDriveForwardAxis = Xbox::AxisLeftY;
using TeleopDriveRotationAxis = Xbox::AxisLeftX;

using TeleopTrencherSpeedAxis = Xbox::AxisRightTrigger;
using TeleopTrencherInvertButton = Xbox::ButtonRightBumper;

using TeleopHopperSpeedAxis = Xbox::AxisLeftTrigger;
using TeleopHopperInvertButton = Xbox::ButtonLeftBumper;
using TeleopHopperActuateAxis = Xbox::AxisRightY;

using AssistedMiningToggleButton = Xbox::ButtonLeftCenter;
using AssistedOffloadToggleButton = Xbox::ButtonRightCenter;

using PresetMiningInitButton = Xbox::ButtonLeftStick;
using PresetOffloadInitButton = Xbox::ButtonRightStick;

using ToggleMiningObstacleConstraintButton = Xbox::DPadUp;
using ToggleMiningHopperConstraintButton = Xbox::DPadRight;
using ToggleMiningZoneConstraintButton = Xbox::DPadDown;
using ToggleMiningStallConstraintButton = Xbox::DPadLeft;


// --- Mission Control Override Controls ---------------------------------------

using MissionControlOverrideButton = Xbox::ButtonCenter;

using SetDisabledModeButton = Xbox::ButtonA;
using SetTeleopModeButton = Xbox::ButtonB;
using SetAutoModeButton = Xbox::ButtonX;
using ToggleTestModeButton = Xbox::ButtonY;

using ToggleTraversalCursorButton = Xbox::ButtonRightBumper;
using ConfirmTraversalTargetButton = Xbox::ButtonLeftBumper;

using TraversalCursorRotAxis = Xbox::AxisRightX;
using TraversalCursorPosAxes =
    util::StaticJoyStickAxes<Xbox::AxisLeftY::IDX, Xbox::AxisLeftX::IDX>;


#undef Xbox

};  // namespace lance
