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

#include "robot_params.hpp"

#include "stall_analyzer.hpp"
#include "util/ros_utils.hpp"


using namespace util;


namespace lance
{

#define INIT_PARAM(name, val, type)                        \
    name { declare_and_get_param<type>(node, #name, val) }

#define INIT_PARAM2(name1, name2, val, type)                      \
    name1##_##name2                                               \
    {                                                             \
        declare_and_get_param<type>(node, #name1 "." #name2, val) \
    }

#define INIT_PARAM3(name1, name2, name3, val, type)                             \
    name1##_##name2##_##name3                                                   \
    {                                                                           \
        declare_and_get_param<type>(node, #name1 "." #name2 "." #name3, val)    \
    }

RobotParams::RobotParams(rclcpp::Node& node) :
    INIT_PARAM(default_stick_deadzone, 0.05f, float),
    INIT_PARAM(driving_magnitude_deadzone, 0.1f, float),
    INIT_PARAM(driving_low_scalar, 0.3f, float),
    INIT_PARAM(driving_medium_scalar, 0.7f, float),
    INIT_PARAM(driving_high_scalar, 1.f, float),

    INIT_PARAM2(trencher, max_velocity_rps, 80.f, float),
    INIT_PARAM2(trencher, mining_velocity_rps, 80.f, float),
    INIT_PARAM2(hopper_belt, max_velocity_rps, 45.f, float),
    INIT_PARAM2(hopper_belt, mining_velocity_rps, 10.f, float),
    INIT_PARAM2(tracks, max_velocity_rps, 125.f, float),
    INIT_PARAM2(tracks, mining_velocity_rps, 8.f, float),
    INIT_PARAM2(tracks, mining_adjustment_range_rps, 6.f, float),
    INIT_PARAM2(tracks, offload_velocity_rps, 30.f, float),

    INIT_PARAM2(hopper_actuator, max_speed, 1.f, float),
    INIT_PARAM2(hopper_actuator, plunge_speed, 0.4f, float),
    INIT_PARAM2(hopper_actuator, extract_speed, 0.8f, float),

    INIT_PARAM2(hopper_actuator, offload_target_val, 0.95f, float),
    INIT_PARAM2(hopper_actuator, traversal_target_val, 0.6f, float),
    INIT_PARAM2(hopper_actuator, transport_target_val, 0.55f, float),
    INIT_PARAM2(hopper_actuator, mining_target_val, 0.21f, float),
    INIT_PARAM2(hopper_actuator, mining_min_val, 0.03f, float),
    INIT_PARAM2(hopper_actuator, targetting_thresh, 0.01f, float),

    INIT_PARAM2(hopper_belt, mining_duty_cycle_base_seconds, 1.f, float),

    INIT_PARAM2(collection_model, initial_volume_liters, 5.f, float),
    INIT_PARAM2(collection_model, capacity_volume_liters, 25.f, float),
    INIT_PARAM2(collection_model, initial_belt_footprint_meters, 0.2f, float),
    INIT_PARAM2(collection_model, belt_capacity_meters, 0.6f, float),
    INIT_PARAM2(collection_model, belt_offload_length_meters, 0.7f, float),

    INIT_PARAM(iteration_period_seconds, 0.05f, float),
    INIT_PARAM2(stall_analyzer, recovery_debounce_seconds, 0.10f, float),
    INIT_PARAM3(
        stall_analyzer,
        tracks,
        acceleration_jump_rps_per_second,
        100.f,
        float),
    INIT_PARAM3(stall_analyzer, tracks, debounce_time_seconds, 0.25f, float),
    INIT_PARAM3(stall_analyzer, tracks, min_output_current_amps, 50.f, float),
    INIT_PARAM3(stall_analyzer, tracks, velocity_error_rps, 20.f, float),
    INIT_PARAM3(stall_analyzer, tracks, min_command_value, 0.01f, float),
    INIT_PARAM3(stall_analyzer, tracks, min_output_percent, 0.05f, float),
    INIT_PARAM3(stall_analyzer, tracks, min_output_voltage, 1.f, float),
    INIT_PARAM3(
        stall_analyzer,
        trencher,
        acceleration_jump_rps_per_second,
        100.f,
        float),
    INIT_PARAM3(stall_analyzer, trencher, debounce_time_seconds, 0.25f, float),
    INIT_PARAM3(stall_analyzer, trencher, min_output_current_amps, 15.f, float),
    INIT_PARAM3(stall_analyzer, trencher, velocity_error_rps, 20.f, float),
    INIT_PARAM3(stall_analyzer, trencher, min_command_value, 1.f, float),
    INIT_PARAM3(stall_analyzer, trencher, min_output_percent, 0.05f, float),
    INIT_PARAM3(stall_analyzer, trencher, min_output_voltage, 1.f, float),
    INIT_PARAM(robot_frame_id, "base_link", std::string),
    INIT_PARAM(odom_frame_id, "odom", std::string),
    INIT_PARAM(arena_frame_id, "map", std::string),

    INIT_PARAM2(auto_localization, min_num_search_samples, 100, int),
    INIT_PARAM2(auto_localization, search_angular_velocity_rps, 0.5f, float),
    INIT_PARAM2(auto_localization, align_angular_velocity_rps, 0.25f, float),
    INIT_PARAM2(auto_localization, align_angular_thresh_deg, 2.f, float),
    INIT_PARAM2(auto_localization, range_target_m, 1.05f, float),
    INIT_PARAM2(auto_localization, range_thresh_m, 0.05f, float),

    INIT_PARAM2(auto_traversal, max_track_velocity_mps, 0.25f, float),
    INIT_PARAM2(auto_traversal, max_track_acceleration_mpss, 0.5f, float),
    INIT_PARAM2(auto_traversal, max_angular_velocity_rps, 1.f, float),
    INIT_PARAM2(auto_traversal, max_angular_accel_rpss, 0.5f, float),
    INIT_PARAM2(auto_traversal, destination_thresh_m, 0.03f, float),
    INIT_PARAM2(auto_traversal, max_path_deviation_m, 0.03f, float),
    INIT_PARAM2(auto_traversal, stanley_k_coeff, 1.f, float),
    INIT_PARAM2(auto_traversal, angular_kp, 1.f, float),
    INIT_PARAM2(auto_traversal, min_theta_window_deg, 2.f, float),
    INIT_PARAM2(auto_traversal, align_angular_thresh_deg, 0.5f, float)
{
    std::vector<double> buff;

#define INIT_BOX2F(zone)                                       \
    declare_param(node, #zone "_bounds.min", buff, {0., 0.});  \
    assert(buff.size() > 1);                                   \
    this->bounds.zone.min().x() = static_cast<float>(buff[0]); \
    this->bounds.zone.min().y() = static_cast<float>(buff[1]); \
    declare_param(node, #zone "_bounds.max", buff, {0., 0.});  \
    assert(buff.size() > 1);                                   \
    this->bounds.zone.max().x() = static_cast<float>(buff[0]); \
    this->bounds.zone.max().y() = static_cast<float>(buff[1]);

    INIT_BOX2F(arena_zone)
    INIT_BOX2F(mining_zone)
    INIT_BOX2F(offload_zone)
    INIT_BOX2F(construction_zone)
}

StallAnalyzerConfig RobotParams::makeStallAnalyzerConfig() const
{
    StallAnalyzerConfig config;
    config.recovery_debounce_seconds =
        this->stall_analyzer_recovery_debounce_seconds;

    config.tracks.acceleration_jump_rps_per_second =
        this->stall_analyzer_tracks_acceleration_jump_rps_per_second;
    config.tracks.debounce_time_seconds =
        this->stall_analyzer_tracks_debounce_time_seconds;
    config.tracks.min_output_current_amps =
        this->stall_analyzer_tracks_min_output_current_amps;
    config.tracks.velocity_error_rps =
        this->stall_analyzer_tracks_velocity_error_rps;
    config.tracks.min_command_value =
        this->stall_analyzer_tracks_min_command_value;
    config.tracks.min_output_percent =
        this->stall_analyzer_tracks_min_output_percent;
    config.tracks.min_output_voltage =
        this->stall_analyzer_tracks_min_output_voltage;

    config.trencher.acceleration_jump_rps_per_second =
        this->stall_analyzer_trencher_acceleration_jump_rps_per_second;
    config.trencher.debounce_time_seconds =
        this->stall_analyzer_trencher_debounce_time_seconds;
    config.trencher.min_output_current_amps =
        this->stall_analyzer_trencher_min_output_current_amps;
    config.trencher.velocity_error_rps =
        this->stall_analyzer_trencher_velocity_error_rps;
    config.trencher.min_command_value =
        this->stall_analyzer_trencher_min_command_value;
    config.trencher.min_output_percent =
        this->stall_analyzer_trencher_min_output_percent;
    config.trencher.min_output_voltage =
        this->stall_analyzer_trencher_min_output_voltage;

    return config;
}

};  // namespace lance
