// Lesson 7 (capstone): goto_point_node
//
// Click a point in Foxglove's 3D panel and the robot drives there: it turns to
// face the goal, then drives toward it, steering as it goes.
//
//   /clicked_point (PointStamped) --> [goto_point] --> lance/track_*/ctrl
//
// The robot's own controller (robot_control) must NOT be running, or both
// nodes would command the tracks at once. Launch with robot_control:=null.
//
// This starter gives you the structure only. You've done every piece of this
// in earlier lessons. The README says which lesson to look back at.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <numbers>
#include <optional>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include <tf2/exceptions.hpp>
#include <tf2_ros/buffer.hpp>
#include <tf2_ros/transform_listener.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>  // lets tf2 transform geometry_msgs types

#include <geometry_msgs/msg/point_stamped.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <phoenix_ros_driver/msg/talon_ctrl.hpp>

using namespace std::chrono_literals;

using PointStampedMsg = geometry_msgs::msg::PointStamped;
using MarkerMsg = visualization_msgs::msg::Marker;
using MarkerArrayMsg = visualization_msgs::msg::MarkerArray;
using TalonCtrlMsg = phoenix_ros_driver::msg::TalonCtrl;


class GotoPointNode : public rclcpp::Node
{
    enum class State
    {
        IDLE,     // no goal: tracks stopped
        TURNING,  // turning in place to face the goal
        DRIVING   // driving toward the goal, steering as we go
    };

    struct TrackCommand
    {
        double left_mps{0.0};
        double right_mps{0.0};
    };

public:
    GotoPointNode() : Node("goto_point")
    {
        // TODO (Part 1): TF buffer + listener (Lesson 3), parameters
        // TODO (Part 1): subscribe to /clicked_point
        // TODO (Part 3): track command publishers
        // TODO (Part 2): 50ms timer calling update()

        RCLCPP_INFO(this->get_logger(), "goto_point started");
    }

private:
    // TODO (Part 1): void onGoal(const PointStampedMsg& msg)
    //   transform the goal into the fixed frame and store it

    // TODO (Part 2): void update()
    //   transform the stored goal into base_link, compute distance and
    //   heading error, run the state machine, send the track command

    // TODO (Part 2): TrackCommand runStateMachine(double distance, double heading_error)

    // TODO (Part 2): void setState(State new_state), logging the change

    // TODO (Part 3): void sendTrackCommand(const TrackCommand& cmd)

private:
    // TODO: members
    State state{State::IDLE};
};


int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GotoPointNode>());
    rclcpp::shutdown();
    return 0;
}
