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

namespace lance
{

#define ROBOT_TOPIC(subtopic)      "lance/" subtopic
#define PERCEPTION_TOPIC(subtopic) "cardinal_perception/" subtopic
#define TALON_CTRL_TOPIC(motor_id) ROBOT_TOPIC(motor_id "/ctrl")
#define TALON_INFO_TOPIC(motor_id) ROBOT_TOPIC(motor_id "/info")
#define COLLECTION_STATE_TOPIC(subtopic)      \
    ROBOT_TOPIC("collection_state/" subtopic)
#define STALL_STATE_TOPIC(subtopic) ROBOT_TOPIC("stall_state/" subtopic)

#define CONSTEXPR_STR constexpr inline char const*

CONSTEXPR_STR WATCHDOG_TOPIC = ROBOT_TOPIC("watchdog_status");
CONSTEXPR_STR SET_TELEOP_TOPIC = ROBOT_TOPIC("set_teleop_mode");
CONSTEXPR_STR SET_AUTO_TOPIC = ROBOT_TOPIC("set_auto_mode");
CONSTEXPR_STR SET_TEST_TOPIC = ROBOT_TOPIC("set_test_mode");
CONSTEXPR_STR SET_QUICK_AUTO_TOPIC = ROBOT_TOPIC("set_quick_auto");
CONSTEXPR_STR SET_ASSIST_AUTO_TOPIC = ROBOT_TOPIC("set_assist_auto");

CONSTEXPR_STR JOY_INPUT_TOPIC = "/joy";
CONSTEXPR_STR JOY_CTRL_TOPIC = ROBOT_TOPIC("joy_ctrl");
CONSTEXPR_STR CLICKED_POINT_TOPIC = "/clicked_point";
CONSTEXPR_STR REMOTE_COMMANDS_TOPIC = ROBOT_TOPIC("remote_cmds");

CONSTEXPR_STR TELEMETRY_TOPIC = ROBOT_TOPIC("telemetry");
CONSTEXPR_STR OP_STATUS_TOPIC = ROBOT_TOPIC("op_status");
CONSTEXPR_STR MC_STATE_TOPIC = ROBOT_TOPIC("mc_state");
CONSTEXPR_STR TRAVERSAL_PATH_TOPIC = ROBOT_TOPIC("traversal_path");
CONSTEXPR_STR ROBOT_MARKERS_TOPIC = ROBOT_TOPIC("markers");
CONSTEXPR_STR MC_CURSOR_TOPIC = ROBOT_TOPIC("mc_cursor");
CONSTEXPR_STR ARENA_ZONES_TOPIC = "arena_zones";

CONSTEXPR_STR HOPPER_JOINT_NAME = "hopper_joint";


CONSTEXPR_STR PERCEPTION_LFD_CONTROL_SRV_TOPIC =
    PERCEPTION_TOPIC("set_global_alignment");
CONSTEXPR_STR PERCEPTION_REFLECTOR_HINT_TOPIC =
    PERCEPTION_TOPIC("reflector_hint");

CONSTEXPR_STR PERCEPTION_UPDATE_MINING_EVAL_SRV_TOPIC =
    PERCEPTION_TOPIC("update_mining_eval");
CONSTEXPR_STR PERCEPTION_MINING_EVAL_RESULTS_TOPIC =
    PERCEPTION_TOPIC("mining_eval_results");

CONSTEXPR_STR PERCEPTION_PATH_TOPIC = PERCEPTION_TOPIC("planned_path");
CONSTEXPR_STR PERCEPTION_PPLAN_CONTROL_TOPIC =
    PERCEPTION_TOPIC("update_path_planning");

#undef CONSTEXPR_STR

#define TALON_CTRL_PUBSUB_QOS                                            \
    rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile()

};  // namespace lance
