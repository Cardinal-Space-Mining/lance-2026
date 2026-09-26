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
 * @file path_plan.hpp
 * @brief Perception node path planning client interface.
 *
 * Bridges the robot controller with the global obstacle-avoiding A* / RRT path planner:
 *   - Requests path computation toward destination goals (Point, Pose, or Zone centroid).
 *   - Receives calculated nav_msgs::msg::Path containing obstacle-free waypoint sequences.
 *   - Notifies perception when a traversal trajectory has been completed or aborted.
 */

#include <string_view>

#include <rclcpp/rclcpp.hpp>

#include <Eigen/Core>

#include <nav_msgs/msg/path.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>

#include <cardinal_perception/srv/update_path_planning_mode.hpp>

#include <csm_utils/ros_utils.hpp>


namespace lance
{

/**
 * @class PathPlanInterface
 * @brief ROS 2 service and subscription interface managing path requests to `cardinal_perception`.
 */
class PathPlanInterface : public util::UsingRosAliases
{
public:
    using PathMsg = nav_msgs::msg::Path;
    using PoseStampedMsg = geometry_msgs::msg::PoseStamped;
    using PointStampedMsg = geometry_msgs::msg::PointStamped;
    using UpdatePathPlanSrv = cardinal_perception::srv::UpdatePathPlanningMode;

    using Vec3f = Eigen::Vector3f;

public:
    PathPlanInterface(RclNode&);

public:
    /// @brief Dispatch path request to 3D coordinate in specified reference frame.
    void init(const Vec3f&, std::string_view);

    /// @brief Dispatch path request to full 6D stamped pose.
    void init(const PoseStampedMsg&);

    /// @brief Dispatch path request to 3D stamped point.
    void init(const PointStampedMsg&);

    /// @brief Signal perception that navigation to current destination has terminated or aborted.
    void cancel();

    /// @brief True if valid planned path has been received.
    bool hasPath() const;

    /// @brief Read-only pointer to latest received nav_msgs::Path (or nullptr if none).
    const PathMsg* getPath() const;

    /// @brief Reset and discard currently cached path message.
    void clearPath();

protected:
    RclClock::ConstSharedPtr rcl_clock;                     ///< Node clock used to stamp outgoing requests.
    RclSubPtr<PathMsg> path_sub;                            ///< Subscriber receiving computed waypoint paths.
    RclClientPtr<UpdatePathPlanSrv> pplan_control_client;  ///< Service client initiating/cancelling path searches.

    PathMsg::ConstSharedPtr last_path{nullptr};             ///< Latest received path trajectory.
};

};  // namespace lance
