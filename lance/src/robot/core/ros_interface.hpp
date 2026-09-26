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
 * @file ros_interface.hpp
 * @brief ROS 2 topic names, service endpoints, URDF joint names, and QoS policies.
 *
 * Defines standardized string constants for:
 *   - Robot control and Talon FX motor communication topics under `lance/`.
 *   - Perception and LiDAR mapping endpoints under `cardinal_perception/`.
 *   - Operator mission control (MC) and visualization topics.
 *   - Default Quality of Service (QoS) configurations for real-time motor commands.
 */

namespace lance
{

// Topic prefix macros
#define ROBOT_TOPIC(subtopic)      "lance/" subtopic
#define PERCEPTION_TOPIC(subtopic) "cardinal_perception/" subtopic
#define TALON_CTRL_TOPIC(motor_id) ROBOT_TOPIC(motor_id "/ctrl")
#define TALON_INFO_TOPIC(motor_id) ROBOT_TOPIC(motor_id "/info")
#define COLLECTION_STATE_TOPIC(subtopic)      \
    ROBOT_TOPIC("collection_state/" subtopic)
#define STALL_STATE_TOPIC(subtopic) ROBOT_TOPIC("stall_state/" subtopic)

#define CONSTEXPR_STR constexpr inline char const*

// --- Watchdog, Mode, and Option Setters ---
CONSTEXPR_STR WATCHDOG_TOPIC = ROBOT_TOPIC("watchdog_status"); ///< INT32 status topic heartbeat.
CONSTEXPR_STR SET_TELEOP_TOPIC = ROBOT_TOPIC("set_teleop_mode");   ///< Request switch to teleoperated mode.
CONSTEXPR_STR SET_AUTO_TOPIC = ROBOT_TOPIC("set_auto_mode");       ///< Request switch to autonomous mode.
CONSTEXPR_STR SET_TEST_TOPIC = ROBOT_TOPIC("set_test_mode");       ///< Toggle bench testing depth constraint.
CONSTEXPR_STR SET_QUICK_AUTO_TOPIC = ROBOT_TOPIC("set_quick_auto"); ///< Toggle rapid scoring autonomous mode.
CONSTEXPR_STR SET_ASSIST_AUTO_TOPIC = ROBOT_TOPIC("set_assist_auto"); ///< Toggle autonomous assist routines.

// --- Human Interface Device (HID) & Remote Operator Controls ---
CONSTEXPR_STR JOY_INPUT_TOPIC = "/joy";                            ///< Raw sensor_msgs/msg/Joy from Linux joystick driver.
CONSTEXPR_STR JOY_CTRL_TOPIC = ROBOT_TOPIC("joy_ctrl");             ///< Filtered teleop joystick commands.
CONSTEXPR_STR CLICKED_POINT_TOPIC = "/clicked_point";              ///< RVIZ point click for manual goal dispatch.
CONSTEXPR_STR REMOTE_COMMANDS_TOPIC = ROBOT_TOPIC("remote_cmds");   ///< High-level operator actions (align, mine, offload).

// --- Telemetry, Diagnostics & Mission Control ---
CONSTEXPR_STR TELEMETRY_TOPIC = ROBOT_TOPIC("telemetry");           ///< Serialized binary telemetry chunk for UI.
CONSTEXPR_STR OP_STATUS_TOPIC = ROBOT_TOPIC("op_status");           ///< Human-readable operator status banner string.
CONSTEXPR_STR MC_STATE_TOPIC = ROBOT_TOPIC("mc_state");             ///< Mission control execution state.
CONSTEXPR_STR TRAVERSAL_PATH_TOPIC = ROBOT_TOPIC("traversal_path"); ///< Planned navigation path geometry.
CONSTEXPR_STR ROBOT_MARKERS_TOPIC = ROBOT_TOPIC("markers");         ///< RVIZ visualization markers for rover and berm.
CONSTEXPR_STR MC_CURSOR_TOPIC = ROBOT_TOPIC("mc_cursor");           ///< Interactive cursor position from mission control UI.
CONSTEXPR_STR ARENA_ZONES_TOPIC = "arena_zones";                    ///< Competition arena zone bounding box visualization.

// --- Joint Names ---
CONSTEXPR_STR HOPPER_JOINT_NAME = "hopper_joint";                   ///< URDF joint name for trencher tilt actuator.

// --- Perception System Integration (cardinal_perception) ---
CONSTEXPR_STR PERCEPTION_LFD_CONTROL_SRV_TOPIC =
    PERCEPTION_TOPIC("set_global_alignment");                       ///< Service to trigger LiDAR feature-based global relocalization.
CONSTEXPR_STR PERCEPTION_REFLECTOR_HINT_TOPIC =
    PERCEPTION_TOPIC("reflector_hint");                             ///< Directional search cue indicating retroreflector sector.

CONSTEXPR_STR PERCEPTION_UPDATE_MINING_EVAL_SRV_TOPIC =
    PERCEPTION_TOPIC("update_mining_eval");                         ///< Service triggering LiDAR elevation map analysis in mining zone.
CONSTEXPR_STR PERCEPTION_MINING_EVAL_RESULTS_TOPIC =
    PERCEPTION_TOPIC("mining_eval_results");                        ///< Ray-march traversability and cut depth evaluation array.

CONSTEXPR_STR PERCEPTION_PATH_TOPIC = PERCEPTION_TOPIC("planned_path");        ///< Perception node optimal path output.
CONSTEXPR_STR PERCEPTION_PPLAN_CONTROL_TOPIC =
    PERCEPTION_TOPIC("update_path_planning");                       ///< Path planning service request topic.

#undef CONSTEXPR_STR

/**
 * @def TALON_CTRL_PUBSUB_QOS
 * @brief Quality of Service policy configured for real-time motor controller updates.
 *
 * Configured as:
 *   - Depth: KeepLast(1) (drops stale packets in favor of latest setpoint)
 *   - Reliability: Best Effort (minimizes latency jitter across wireless bridge)
 *   - Durability: Volatile (new subscribers do not receive outdated commands)
 */
#define TALON_CTRL_PUBSUB_QOS                                            \
    rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile()

};  // namespace lance
