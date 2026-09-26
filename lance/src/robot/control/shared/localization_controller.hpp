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
 * @file localization_controller.hpp
 * @brief Autonomous retroreflector search and standoff alignment state machine.
 *
 * Implements the 4-phase localization sequence:
 *   1. INITIALIZATION: Raises linear actuator to traversal height for unobstructed LiDAR view.
 *   2. SEARCHING: Rotates robot in place until LiDAR retroreflector detector identifies beacon cluster.
 *   3. ALIGN_HEADING: Rotates in place to point robot heading directly at the beacon centroid.
 *   4. ADJUST_RANGE: Drives forward/reverse to establish target standoff distance (e.g. 1.05m).
 *
 * Terminates automatically when the complete global map-to-base_link transform is acquired in TfCache.
 */

#include "robot/core/robot_params.hpp"
#include "robot/core/motor_interface.hpp"
#include "robot/sensing/sensing_interfaces.hpp"


namespace lance
{

/**
 * @class LocalizationController
 * @brief Closed-loop controller for LiDAR beacon acquisition and docking.
 */
class LocalizationController
{
    friend class TelemetrySerializer;
    friend class TelemetryDeserializer;

    using RclNode = rclcpp::Node;

public:
    LocalizationController(
        const RobotParams&,
        SensingInterfaces&);
    ~LocalizationController() = default;

public:
    /// @brief Reset state machine to INITIALIZATION and clear previous hints.
    void initialize();

    /// @brief True if localization transform is verified and state machine reached FINISHED.
    bool isFinished();

    /// @brief Cancel active localization routine and disable perception hint service.
    void setCancelled();

    /**
     * @brief Execute one control iteration of the localization state machine.
     * @param motor_status Latest motor feedback.
     * @param[out] commands Motor command outputs.
     */
    void iterate(
        const RobotMotorStatus& motor_status,
        RobotMotorCommands& commands);

protected:
    /**
     * @enum Stage
     * @brief Internal execution phases for localization docking.
     */
    enum class Stage
    {
        INITIALIZATION, ///< Positioning linear actuator to clear LiDAR field of view.
        SEARCHING,      ///< 360-degree in-place yaw rotation to scan for retroreflective beacon.
        ALIGN_HEADING,  ///< Yaw correction pointing directly at reflector centroid.
        ADJUST_RANGE,   ///< Linear driving to reach exact target standoff distance.
        FINISHED        ///< Global localization transform confirmed in TF tree.
    };

protected:
    const RobotParams& params;
    const TfCache& tf_cache;
    ReflectorHintInterface& refl_hint_interface;

    Stage stage{Stage::FINISHED};
};

};  // namespace lance
