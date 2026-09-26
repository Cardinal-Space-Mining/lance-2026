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
 * @file robot_params.hpp
 * @brief Configuration parameters and competition arena boundary definitions for LANCE.
 *
 * Encapsulates all runtime configurable ROS parameters loaded during node startup,
 * including gamepad stick deadbands, speed multipliers, velocity limits across all actuators,
 * trencher linear actuator setpoints for each operating mode, collection hopper state model tuning,
 * stall detection thresholds, TF frame identifiers, autonomous navigation Stanley controller gains,
 * and 2D bounding boxes for NASA Lunabotics arena zones.
 */

#include <string>

#include <Eigen/Geometry>

#include <rclcpp/rclcpp.hpp>


namespace lance
{

/**
 * @struct RobotParams
 * @brief Centralized parameter registry holding all operational thresholds, gains, and geometry limits.
 *
 * Parameters are declared and retrieved from the ROS 2 parameter server in the constructor.
 * All members are declared const after initialization to prevent accidental modification at runtime.
 */
struct RobotParams
{
    using Box2f = Eigen::AlignedBox2f;

public:
    /**
     * @struct ZoneBounds
     * @brief 2D Axis-Aligned Bounding Boxes (AABB) defining the competition arena spatial zones.
     *
     * Coordinates are specified in meters relative to the arena world origin (arena_frame_id, typically "map").
     */
    struct ZoneBounds
    {
        Box2f arena_zone;        ///< Total traversable arena perimeter [xmin, ymin] to [xmax, ymax].
        Box2f mining_zone;       ///< Designated excavation area containing target regolith / BP-1 simulant.
        Box2f offload_zone;      ///< Target offload hopper / collection bin area for payload delivery.
        Box2f construction_zone; ///< Berm construction or obstacle zone where navigation is restricted.
    };

public:
    // --- Teleoperation & Gamepad Controls ---
    const float default_stick_deadzone;       ///< Gamepad thumbstick radial deadband [0.0 - 1.0].
    const float driving_magnitude_deadzone;   ///< Threshold below which composite drive commands are zeroed.
    const float driving_low_scalar;           ///< Fine-positioning drive speed scaling factor (default: 0.3).
    const float driving_medium_scalar;        ///< Standard cruise drive speed scaling factor (default: 0.7).
    const float driving_high_scalar;          ///< Full-throttle drive speed scaling factor (default: 1.0).

    // --- Actuator Velocity Limits (RPS at Motor Shaft) ---
    const float trencher_max_velocity_rps;         ///< Absolute maximum velocity limit for trencher motor (RPS).
    const float trencher_mining_velocity_rps;      ///< Nominal operating velocity for trencher during excavation (RPS).
    const float hopper_belt_max_velocity_rps;      ///< Maximum conveyor belt velocity during offload (RPS).
    const float hopper_belt_mining_velocity_rps;   ///< Slow indexing velocity for conveyor belt during mining (RPS).
    const float tracks_max_velocity_rps;           ///< Maximum track drive motor speed during transit (RPS).
    const float tracks_mining_max_velocity_rps;    ///< Reduced maximum track speed during forward trenching (RPS).
    const float tracks_offload_velocity_rps;       ///< Track speed limit during reverse alignment at offload bin (RPS).

    // --- Linear Actuator Speeds & Stroke Targets [0.0, 1.0] ---
    const float hopper_actuator_max_speed;       ///< Maximum linear actuator slew rate.
    const float hopper_actuator_plunge_speed;    ///< Controlled downward plunge speed into regolith.
    const float hopper_actuator_extract_speed;   ///< Fast retraction speed when lifting trencher clear of trench.

    const float hopper_actuator_offload_target_val;    ///< Target stroke for high elevation dump into offload bin.
    const float hopper_actuator_traversal_target_val;  ///< Target stroke for standard driving ground clearance.
    const float hopper_actuator_transport_target_val;  ///< Target stroke for low-center-of-gravity full-hopper transport.
    const float hopper_actuator_mining_target_val;     ///< Target stroke for nominal mining trench depth.
    const float hopper_actuator_mining_min_val;        ///< Lowest allowed actuator position during excavation.
    const float hopper_actuator_targetting_thresh;     ///< Closed-loop tolerance to consider actuator on-target.

    // --- Hopper Belt Indexing ---
    const float hopper_belt_mining_duty_cycle_base_seconds; ///< Base pulse duration for periodic regolith distribution along the belt.

    // --- Regolith Collection Volume Estimation Model ---
    const float collection_model_initial_volume_liters;          ///< Assumed pre-existing volume at start (Liters).
    const float collection_model_capacity_volume_liters;         ///< Maximum volumetric payload capacity (Liters).
    const float collection_model_initial_belt_footprint_meters;  ///< Linear length of belt covered at initial deposit (m).
    const float collection_model_belt_capacity_meters;           ///< Maximum conveyor belt length available for regolith bed (m).
    const float collection_model_belt_offload_length_meters;     ///< Conveyor displacement needed to purge payload (m).
    const float collection_model_transfer_efficiency;            ///< Excavation capture efficiency factor [0.0 - 1.0].

    // --- Stall Analyzer & Fault Detection Thresholds ---
    const float stall_analyzer_tracks_debounce_time_s;       ///< Time track velocity deficit must persist before flagging stall.
    const float stall_analyzer_tracks_min_vel_proportion;    ///< Min actual/commanded velocity ratio below which stall is suspected.
    const float stall_analyzer_tracks_command_deadzone_rps;  ///< Minimum commanded RPS to trigger track stall monitoring.
    const float stall_analyzer_trencher_debounce_time_s;     ///< Time trencher deficit must persist before triggering stall.
    const float stall_analyzer_trencher_min_vel_proportion;  ///< Min actual/commanded velocity ratio for trencher stall.
    const float stall_analyzer_trencher_command_deadzone_rps;///< Minimum commanded RPS to trigger trencher stall monitoring.

    // --- Timing & Coordinate Frames ---
    const float iteration_period_seconds; ///< Robot control loop tick period in seconds (default: 0.05s = 20 Hz).
    const std::string robot_frame_id;     ///< Robot base frame ID in TF tree (typically "base_link").
    const std::string odom_frame_id;      ///< Local odometry frame ID in TF tree (typically "odom").
    const std::string arena_frame_id;     ///< Global fixed reference frame ID (typically "map").

    // --- Spatial Boundaries & Preset Tasks ---
    ZoneBounds bounds;                   ///< 2D axis-aligned bounding boxes for arena zones.
    const float preset_mining_vol_l;     ///< Target volume for automated single-cut mining preset (Liters).
    const float preset_offload_backup_m; ///< Distance to reverse after completing offload sequence (meters).

    // --- Autonomous Localization & Reflector Search ---
    const int auto_localization_min_num_search_samples;           ///< Minimum LiDAR scan points required to detect retroreflector.
    const float auto_localization_search_angular_velocity_rps;   ///< Yaw rotation speed during 360-degree beacon scan.
    const float auto_localization_align_angular_velocity_rps;    ///< Slower yaw rate for precise beacon bearing alignment.
    const float auto_localization_align_angular_thresh_deg;      ///< Angular error tolerance to complete beacon alignment.
    const float auto_localization_range_target_m;                ///< Desired standoff distance from beacon target.
    const float auto_localization_range_thresh_m;                ///< Range tolerance to consider distance satisfied.

    // --- Autonomous Traversal & Stanley Path Tracking ---
    const float auto_traversal_max_track_velocity_mps;       ///< Maximum allowable track speed during autonomous navigation.
    const float auto_traversal_max_track_acceleration_mpss;  ///< Track acceleration limit (m/s^2) for smooth speed profiles.
    const float auto_traversal_max_angular_velocity_rps;     ///< Maximum yaw angular velocity during autonomous turns.
    const float auto_traversal_max_angular_accel_rpss;       ///< Angular acceleration limit (rad/s^2).
    const float auto_traversal_destination_thresh_m;         ///< Position tolerance radius for waypoint completion.
    const float auto_traversal_max_path_deviation_m;         ///< Cross-track error threshold triggering recovery/slowdown.
    const float auto_traversal_stanley_k_coeff;              ///< Stanley controller cross-track error gain k.
    const float auto_traversal_angular_kp;                   ///< Proportional gain for heading error correction.
    const float auto_traversal_min_theta_window_deg;         ///< Heading error window for transitioning between in-place and Stanley steering.
    const float auto_traversal_align_angular_thresh_deg;     ///< Final orientation alignment tolerance at waypoint.

    // --- Autonomous Mining Planner Parameters ---
    const float auto_mining_min_path_length;       ///< Minimum excavation cut length (m) for a candidate path to be viable.
    const float auto_mining_min_replan_vol_liters; ///< Remaining payload capacity threshold triggering a replan pass.
    const int auto_mining_max_iterations;          ///< Maximum successive replanning passes allowed per mining mission.

public:
    /**
     * @brief Construct RobotParams by declaring and loading all parameters from the ROS 2 node.
     * @param node Reference to the hosting rclcpp::Node instance.
     */
    RobotParams(rclcpp::Node& node);

};

};  // namespace lance
