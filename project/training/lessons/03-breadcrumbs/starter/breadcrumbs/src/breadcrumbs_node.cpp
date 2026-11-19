// Lesson 3: breadcrumbs_node
//
// Goal: use TF to find where the robot is, remember the path it has driven,
// and publish markers so the trail shows up in Foxglove's 3D panel.
//
// Work through the TODOs in order, following the lesson README. Build and run
// after every step. (Dependencies are already set up in package.xml and
// CMakeLists.txt this time. Take a look at what was added.)

#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdio>
#include <deque>
#include <memory>
#include <optional>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include <tf2/exceptions.hpp>
#include <tf2_ros/buffer.hpp>
#include <tf2_ros/transform_listener.hpp>

#include <geometry_msgs/msg/point.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

using namespace std::chrono_literals;

using PointMsg = geometry_msgs::msg::Point;
using MarkerMsg = visualization_msgs::msg::Marker;
using MarkerArrayMsg = visualization_msgs::msg::MarkerArray;


class BreadcrumbsNode : public rclcpp::Node
{
public:
    BreadcrumbsNode() : Node("breadcrumbs")
    {
        // TODO (Step 2): declare "fixed_frame" (default "odom") and
        //   "robot_frame" (default "base_link") parameters.

        // TODO (Step 2): create the TF buffer and listener. (Careful: read the
        //   section on member initialization order first!)

        // TODO (Step 2): create a 100ms timer that calls update().

        RCLCPP_INFO(this->get_logger(), "Breadcrumbs started!");
    }

private:
    // TODO (Step 3): write
    //   std::optional<PointMsg> lookupRobotPosition()
    //   which returns the robot's position, or std::nullopt if TF can't
    //   answer yet.

    void update()
    {
        // TODO (Step 2): look up the transform and print the robot's x and y.
        // TODO (Step 4): add breadcrumbs to the trail and track distance.
        // TODO (Step 5): publish the markers.
    }

private:
    // TODO: member variables
};


int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<BreadcrumbsNode>());
    rclcpp::shutdown();
    return 0;
}
