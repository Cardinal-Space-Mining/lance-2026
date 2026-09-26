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

/**
 * @file robot_params.cpp
 * @brief Implementation of RobotParams parameter declaration, defaults, and boundary loading.
 *
 * Uses helper macros and `csm_utils::declare_and_get_param` to declare ROS parameters on the
 * hosting node and retrieve their values (or fallback to defaults if unspecified in YAML).
 */

#include "robot_params.hpp"

#include <csm_utils/ros_utils.hpp>


using namespace util;


namespace lance
{

/**
 * @def INIT_PARAM
 * @brief Helper macro to initialize a 1-level parameter: `name { declare_and_get_param(node, "name", val) }`
 */
#define INIT_PARAM(name, val, type)                        \
    name { declare_and_get_param<type>(node, #name, val) }

/**
 * @def INIT_PARAM2
 * @brief Helper macro to initialize a 2-level nested parameter: `name1_name2 { declare_and_get_param(node, "name1.name2", val) }`
 */
#define INIT_PARAM2(name1, name2, val, type)                      \
    name1##_##name2                                               \
    {                                                             \
        declare_and_get_param<type>(node, #name1 "." #name2, val) \
    }

/**
 * @def INIT_PARAM3
 * @brief Helper macro to initialize a 3-level nested parameter: `name1_name2_name3 { declare_and_get_param(node, "name1.name2.name3", val) }`
 */
#define INIT_PARAM3(name1, name2, name3, val, type)                          \
    name1##_##name2##_##name3                                                \
    {                                                                        \
        declare_and_get_param<type>(node, #name1 "." #name2 "." #name3, val) \
    }

RobotParams::RobotParams(rclcpp::Node& node) :
    // --- Joystick Deadbands & Driving Speed Scalars ---
    INIT_PARAM(default_stick_deadzone, 0.05f, float),
    INIT_PARAM(driving_magnitude_deadzone, 0.1f, float),
    INIT_PARAM(driving_low_scalar, 0.3f, float),
    INIT_PARAM(driving_medium_scalar, 0.7f, float),
    INIT_PARAM(driving_high_scalar, 1.f, float),

    // --- Subsystem Actuator Velocities (Motor Shaft RPS) ---
    INIT_PARAM2(trencher, max_velocity_rps, 80.f, float),
    INIT_PARAM2(trencher, mining_velocity_rps, 80.f, float),
    INIT_PARAM2(hopper_belt, max_velocity_rps, 45.f, float),
    INIT_PARAM2(hopper_belt, mining_velocity_rps, 10.f, float),
    INIT_PARAM2(tracks, max_velocity_rps, 125.f, float),
    INIT_PARAM2(tracks, mining_max_velocity_rps, 20.f, float),
    INIT_PARAM2(tracks, offload_velocity_rps, 30.f, float),

    // --- Trencher Linear Actuator Speeds & Stroke Setpoints ---
    INIT_PARAM2(hopper_actuator, max_speed, 1.f, float),
    INIT_PARAM2(hopper_actuator, plunge_speed, 0.4f, float),
    INIT_PARAM2(hopper_actuator, extract_speed, 0.8f, float),

    INIT_PARAM2(hopper_actuator, offload_target_val, 0.95f, float),
    INIT_PARAM2(hopper_actuator, traversal_target_val, 0.6f, float),
    INIT_PARAM2(hopper_actuator, transport_target_val, 0.55f, float),
    INIT_PARAM2(hopper_actuator, mining_target_val, 0.21f, float),
    INIT_PARAM2(hopper_actuator, mining_min_val, 0.03f, float),
    INIT_PARAM2(hopper_actuator, targetting_thresh, 0.01f, float),

    // --- Hopper Belt Indexing Pulse ---
    INIT_PARAM2(hopper_belt, mining_duty_cycle_base_seconds, 1.f, float),

    // --- Regolith Collection Volume Estimation Model Parameters ---
    INIT_PARAM2(collection_model, initial_volume_liters, 5.f, float),
    INIT_PARAM2(collection_model, capacity_volume_liters, 25.f, float),
    INIT_PARAM2(collection_model, initial_belt_footprint_meters, 0.2f, float),
    INIT_PARAM2(collection_model, belt_capacity_meters, 0.6f, float),
    INIT_PARAM2(collection_model, belt_offload_length_meters, 0.7f, float),
    INIT_PARAM2(collection_model, transfer_efficiency, 0.5f, float),

    // --- Current / Velocity Deficit Stall Detection Parameters ---
    INIT_PARAM3(stall_analyzer, tracks, debounce_time_s, 0.25f, float),
    INIT_PARAM3(stall_analyzer, tracks, min_vel_proportion, 0.20f, float),
    INIT_PARAM3(stall_analyzer, tracks, command_deadzone_rps, 0.01f, float),
    INIT_PARAM3(stall_analyzer, trencher, debounce_time_s, 0.25f, float),
    INIT_PARAM3(stall_analyzer, trencher, min_vel_proportion, 0.20f, float),
    INIT_PARAM3(stall_analyzer, trencher, command_deadzone_rps, 1.f, float),

    // --- Node Period and Coordinate Frame Identifiers ---
    INIT_PARAM(iteration_period_seconds, 0.05f, float),
    INIT_PARAM(robot_frame_id, "base_link", std::string),
    INIT_PARAM(odom_frame_id, "odom", std::string),
    INIT_PARAM(arena_frame_id, "map", std::string),

    // --- Preset Mining Mission Targets ---
    INIT_PARAM2(preset, mining_vol_l, 3.f, float),
    INIT_PARAM2(preset, offload_backup_m, 0.5f, float),

    // --- Auto Localization Beacon Search & Standoff Alignment ---
    INIT_PARAM2(auto_localization, min_num_search_samples, 100, int),
    INIT_PARAM2(auto_localization, search_angular_velocity_rps, 0.5f, float),
    INIT_PARAM2(auto_localization, align_angular_velocity_rps, 0.25f, float),
    INIT_PARAM2(auto_localization, align_angular_thresh_deg, 2.f, float),
    INIT_PARAM2(auto_localization, range_target_m, 1.05f, float),
    INIT_PARAM2(auto_localization, range_thresh_m, 0.05f, float),

    // --- Autonomous Traversal Stanley Controller & Limits ---
    INIT_PARAM2(auto_traversal, max_track_velocity_mps, 0.25f, float),
    INIT_PARAM2(auto_traversal, max_track_acceleration_mpss, 0.5f, float),
    INIT_PARAM2(auto_traversal, max_angular_velocity_rps, 1.f, float),
    INIT_PARAM2(auto_traversal, max_angular_accel_rpss, 0.5f, float),
    INIT_PARAM2(auto_traversal, destination_thresh_m, 0.03f, float),
    INIT_PARAM2(auto_traversal, max_path_deviation_m, 0.03f, float),
    INIT_PARAM2(auto_traversal, stanley_k_coeff, 1.f, float),
    INIT_PARAM2(auto_traversal, angular_kp, 1.f, float),
    INIT_PARAM2(auto_traversal, min_theta_window_deg, 2.f, float),
    INIT_PARAM2(auto_traversal, align_angular_thresh_deg, 0.5f, float),

    // --- Autonomous Mining Excavation Cut Optimization ---
    INIT_PARAM2(auto_mining, min_path_length, 2.1f, float),
    INIT_PARAM2(auto_mining, min_replan_vol_liters, 8.f, float),
    INIT_PARAM2(auto_mining, max_iterations, 5, int)
{
    std::vector<double> buff;

    // Helper macro to parse 2D AABB limits [x, y] from YAML parameter arrays
#define INIT_BOX2F(zone)                                       \
    declare_param(node, #zone "_bounds.min", buff, {0., 0.});  \
    assert(buff.size() > 1);                                   \
    this->bounds.zone.min().x() = static_cast<float>(buff[0]); \
    this->bounds.zone.min().y() = static_cast<float>(buff[1]); \
    declare_param(node, #zone "_bounds.max", buff, {0., 0.});  \
    assert(buff.size() > 1);                                   \
    this->bounds.zone.max().x() = static_cast<float>(buff[0]); \
    this->bounds.zone.max().y() = static_cast<float>(buff[1]);

    // Initialize 2D bounding boxes for all 4 competition zones
    INIT_BOX2F(arena_zone)
    INIT_BOX2F(mining_zone)
    INIT_BOX2F(offload_zone)
    INIT_BOX2F(construction_zone)
}

};  // namespace lance
