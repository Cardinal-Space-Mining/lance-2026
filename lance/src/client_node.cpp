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
 * @file client_node.cpp
 * @brief Operator station / mission control process entrypoint for LANCE.
 *
 * ## Architecture Overview
 * This node runs on the off-board operator control computer (ground station):
 *   - `WatchDog`: Broadcasts the INT32 heartbeat on `/lance/watchdog_status` to keep the robot alive.
 *   - `TelemetryDeserializer`: Listens for binary telemetry packets over `/lance/telemetry`,
 *     reconstituting robot internal state machines and publishing RVIZ diagnostic markers.
 *   - `ZonePublisher`: Periodically publishes 3D visualization boxes representing the arena boundaries,
 *     mining excavation zone, offload container, and obstacle construction areas.
 *   - `JointPublisher`: Broadcasts URDF joint positions on `/joint_states` for real-time 3D model visualization.
 *   - `AdvancedControls`: Processes gamepad inputs, interactive waypoint cursors, and operator commands,
 *     forwarding them to the rover.
 *   - `TfCache`: Caches spatial frames (`map`, `odom`, `robot`) to project coordinates between RVIZ and the robot.
 */

#include <memory>
#include <iostream>

#include <rclcpp/rclcpp.hpp>

#include <csm_utils/ros_utils.hpp>

#include "robot/sensing/tf_cache.hpp"
#include "robot/telemetry/markers.hpp"
#include "robot/telemetry/deserializer.hpp"

#include "mission_control/watchdog.hpp"
#include "mission_control/zone_pub.hpp"
#include "mission_control/joint_pub.hpp"
#include "mission_control/advanced_controls.hpp"

using namespace util;
using namespace lance;


/**
 * @class MissionControlNode
 * @brief Central ground station ROS 2 node managing operator interface, visualization, and watchdog feeding.
 */
class MissionControlNode : public rclcpp::Node, public UsingRosAliases
{
public:
    MissionControlNode();

private:
    TfCache tf_cache;                 ///< Coordinate frame listener for map, odom, and base_link.
    MarkerManager markers;             ///< RVIZ visualization marker lifecycle manager.
    TelemetryDeserializer telemetry;   ///< Unpacks binary telemetry into state displays and markers.

    WatchDog watchdog;                 ///< Operator watchdog heartbeat transmitter.
    ZonePublisher zone_publisher;      ///< Arena boundary marker broadcaster.
    JointPublisher joint_publisher;    ///< URDF joint state publisher for 3D model display.
    AdvancedControls controls;         ///< Interactive cursor and teleop command dispatcher.
};



// --- Implementation ---

MissionControlNode::MissionControlNode() :
    Node("mission_control"),
    tf_cache{
        *this,
        declare_and_get_param<std::string>(*this, "arena_frame_id", "map"),
        declare_and_get_param<std::string>(*this, "odom_frame_id", "odom"),
        declare_and_get_param<std::string>(*this, "robot_frame_id", "robot")},
    markers{},
    telemetry{*this, this->tf_cache, this->markers},

    watchdog{*this},
    zone_publisher{*this, this->tf_cache.arena_frame_id},
    joint_publisher{*this},
    controls{
        *this,
        this->tf_cache,
        this->markers,
        this->telemetry,
        this->watchdog,
        this->zone_publisher.getBounds()}
{
    std::cout << "LANCE-" << LANCE << " mission control initialized!"
              << std::endl;
}



int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MissionControlNode>());
    rclcpp::shutdown();
    return 0;
}
