// Lesson 7 reference solution: goto_point_node
//
// Click a point in Foxglove's 3D panel and the robot drives there: it turns to
// face the goal, then drives toward it, steering as it goes.
//
//   /clicked_point (PointStamped) --> [goto_point] --> lance/track_*/ctrl
//
// The robot's own controller (robot_control) must NOT be running, or both
// nodes would command the tracks at once. Launch with robot_control:=null.

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

    static const char* toString(State s)
    {
        switch (s)
        {
            case State::IDLE:
                return "IDLE";
            case State::TURNING:
                return "TURNING";
            case State::DRIVING:
                return "DRIVING";
        }
        return "?";  // unreachable, but keeps the compiler happy
    }

    struct TrackCommand
    {
        double left_mps{0.0};
        double right_mps{0.0};
    };

public:
    GotoPointNode() :
        Node("goto_point"),
        tf_buffer{std::make_unique<tf2_ros::Buffer>(this->get_clock())},
        tf_listener{std::make_shared<tf2_ros::TransformListener>(*tf_buffer)}
    {
        this->fixed_frame = this->declare_parameter<std::string>("fixed_frame", "odom");
        this->robot_frame = this->declare_parameter<std::string>("robot_frame", "base_link");
        this->max_speed_mps = this->declare_parameter<double>("max_speed_mps", 0.25);
        this->max_turn_radps = this->declare_parameter<double>("max_turn_radps", 0.6);
        this->turn_kp = this->declare_parameter<double>("turn_kp", 1.5);
        this->drive_kp = this->declare_parameter<double>("drive_kp", 0.8);
        this->heading_tolerance_rad =
            this->declare_parameter<double>("heading_tolerance_deg", 10.0) *
            (std::numbers::pi / 180.0);
        this->arrive_distance_m =
            this->declare_parameter<double>("arrive_distance_m", 0.15);
        // LANCE-2 defaults, from lance/src/robot/model/dynamics.hpp:
        //   TRACK_GEARING = 64, TRACK_EFFECTIVE_OUTPUT_RADIUS_M = 0.045
        //   -> 64 / (2 * pi * 0.045) = 226.35 motor rev per meter
        this->motor_rps_per_mps =
            this->declare_parameter<double>("motor_rps_per_mps", 226.35);
        this->track_separation_m =
            this->declare_parameter<double>("track_separation_m", 0.632);

        this->goal_sub = this->create_subscription<PointStampedMsg>(
            "/clicked_point",
            rclcpp::QoS(10),
            [this](const PointStampedMsg& msg) { this->onGoal(msg); });

        // Same QoS the robot controller uses for motor commands
        const auto ctrl_qos =
            rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile();
        this->left_pub =
            this->create_publisher<TalonCtrlMsg>("lance/track_left/ctrl", ctrl_qos);
        this->right_pub =
            this->create_publisher<TalonCtrlMsg>("lance/track_right/ctrl", ctrl_qos);
        this->markers_pub =
            this->create_publisher<MarkerArrayMsg>("goto_point/markers", rclcpp::QoS(1));

        this->timer = this->create_wall_timer(50ms, [this]() { this->update(); });

        RCLCPP_INFO(
            this->get_logger(),
            "Click a point in Foxglove (publishing to /clicked_point) to drive there");
    }

private:
    // --- New goal ------------------------------------------------------------

    void onGoal(const PointStampedMsg& msg)
    {
        // Store the goal in a FIXED frame. A goal stored relative to the robot
        // (base_link) would move every time the robot moves!
        PointStampedMsg goal = msg;
        goal.header.stamp = rclcpp::Time(0);  // 0 = "use the latest transform"
        try
        {
            this->goal_fixed =
                this->tf_buffer->transform(goal, this->fixed_frame);
        }
        catch (const tf2::TransformException& e)
        {
            RCLCPP_WARN(
                this->get_logger(),
                "Ignoring goal in frame '%s': %s",
                msg.header.frame_id.c_str(),
                e.what());
            return;
        }

        RCLCPP_INFO(
            this->get_logger(),
            "New goal: (%.2f, %.2f) in '%s'",
            this->goal_fixed->point.x,
            this->goal_fixed->point.y,
            this->fixed_frame.c_str());
        this->setState(State::TURNING);
    }

    // --- Control loop (20 Hz) ------------------------------------------------

    void update()
    {
        TrackCommand cmd;  // defaults to stopped

        if (this->state != State::IDLE && this->goal_fixed)
        {
            // Where is the goal relative to the robot RIGHT NOW?
            std::optional<PointStampedMsg> goal_robot;
            try
            {
                PointStampedMsg goal = *this->goal_fixed;
                goal.header.stamp = rclcpp::Time(0);
                goal_robot = this->tf_buffer->transform(goal, this->robot_frame);
            }
            catch (const tf2::TransformException& e)
            {
                // Don't know where we are: the only safe thing is to stop
                RCLCPP_WARN_THROTTLE(
                    this->get_logger(),
                    *this->get_clock(),
                    1000,
                    "Lost TF, stopping: %s",
                    e.what());
            }

            if (goal_robot)
            {
                // In base_link, +x is forward and +y is left
                const double x = goal_robot->point.x;
                const double y = goal_robot->point.y;
                const double distance = std::hypot(x, y);
                const double heading_error = std::atan2(y, x);  // -pi..pi, + = goal is to the left

                cmd = this->runStateMachine(distance, heading_error);
            }
        }

        this->sendTrackCommand(cmd);
        this->publishMarkers();
    }

    TrackCommand runStateMachine(double distance, double heading_error)
    {
        double forward_mps = 0.0;
        double turn_radps = 0.0;

        switch (this->state)
        {
            case State::IDLE:
                break;

            case State::TURNING:
                if (distance < this->arrive_distance_m)
                {
                    this->arrive();
                }
                else if (std::abs(heading_error) < this->heading_tolerance_rad)
                {
                    this->setState(State::DRIVING);
                }
                else
                {
                    turn_radps = this->turn_kp * heading_error;
                }
                break;

            case State::DRIVING:
                if (distance < this->arrive_distance_m)
                {
                    this->arrive();
                }
                // Only go back to turning if we're WAY off. Using a bigger
                // threshold than the one for TURNING -> DRIVING (hysteresis)
                // stops the robot flip-flopping between the two states
                else if (std::abs(heading_error) > 3.0 * this->heading_tolerance_rad)
                {
                    this->setState(State::TURNING);
                }
                else
                {
                    // Slow down when close, and steer toward the goal
                    forward_mps = std::min(this->max_speed_mps, this->drive_kp * distance);
                    turn_radps = this->turn_kp * heading_error;
                }
                break;
        }

        turn_radps = std::clamp(turn_radps, -this->max_turn_radps, this->max_turn_radps);

        // Differential drive: positive turn (counter-clockwise, to the left)
        // means the right track goes faster than the left
        TrackCommand cmd;
        cmd.left_mps = forward_mps - turn_radps * (this->track_separation_m / 2.0);
        cmd.right_mps = forward_mps + turn_radps * (this->track_separation_m / 2.0);

        // If either track would go too fast, scale BOTH down by the same
        // amount, so the robot still follows the same curve (Lesson 2's
        // applyTrackLimits does the same thing)
        const double largest = std::max(std::abs(cmd.left_mps), std::abs(cmd.right_mps));
        if (largest > this->max_speed_mps)
        {
            const double scale = this->max_speed_mps / largest;
            cmd.left_mps *= scale;
            cmd.right_mps *= scale;
        }
        return cmd;
    }

    void arrive()
    {
        RCLCPP_INFO(this->get_logger(), "Arrived!");
        this->goal_fixed.reset();
        this->setState(State::IDLE);
    }

    void setState(State new_state)
    {
        if (new_state != this->state)
        {
            RCLCPP_INFO(
                this->get_logger(),
                "%s -> %s",
                toString(this->state),
                toString(new_state));
            this->state = new_state;
        }
    }

    // --- Outputs -------------------------------------------------------------

    void sendTrackCommand(const TrackCommand& cmd)
    {
        TalonCtrlMsg left;
        left.mode = TalonCtrlMsg::VELOCITY;
        left.value = cmd.left_mps * this->motor_rps_per_mps;

        TalonCtrlMsg right;
        right.mode = TalonCtrlMsg::VELOCITY;
        right.value = cmd.right_mps * this->motor_rps_per_mps;

        // Publish EVERY cycle, even when stopped. Motor controllers keep
        // running their last command, so "stop" has to be sent too.
        this->left_pub->publish(left);
        this->right_pub->publish(right);
    }

    void publishMarkers()
    {
        MarkerArrayMsg msg;
        MarkerMsg& goal = msg.markers.emplace_back();
        goal.header.frame_id = this->fixed_frame;
        goal.header.stamp = this->now();
        goal.ns = "goal";
        goal.id = 0;
        goal.type = MarkerMsg::SPHERE;
        if (this->goal_fixed)
        {
            goal.action = MarkerMsg::ADD;
            goal.pose.position = this->goal_fixed->point;
            goal.pose.orientation.w = 1.0;
            goal.scale.x = goal.scale.y = goal.scale.z = 2.0 * this->arrive_distance_m;
            goal.color.g = 1.0f;
            goal.color.a = 0.8f;
        }
        else
        {
            goal.action = MarkerMsg::DELETE;  // remove the sphere once we arrive
        }
        this->markers_pub->publish(msg);
    }

private:
    // Declared first: the listener needs the buffer (Lesson 3!)
    std::unique_ptr<tf2_ros::Buffer> tf_buffer;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener;

    // Parameters
    std::string fixed_frame;
    std::string robot_frame;
    double max_speed_mps{0.25};
    double max_turn_radps{0.6};
    double turn_kp{1.5};
    double drive_kp{0.8};
    double heading_tolerance_rad{0.17};
    double arrive_distance_m{0.15};
    double motor_rps_per_mps{226.35};
    double track_separation_m{0.632};

    // ROS interfaces
    rclcpp::Subscription<PointStampedMsg>::SharedPtr goal_sub;
    rclcpp::Publisher<TalonCtrlMsg>::SharedPtr left_pub;
    rclcpp::Publisher<TalonCtrlMsg>::SharedPtr right_pub;
    rclcpp::Publisher<MarkerArrayMsg>::SharedPtr markers_pub;
    rclcpp::TimerBase::SharedPtr timer;

    // State
    State state{State::IDLE};
    std::optional<PointStampedMsg> goal_fixed;
};


int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GotoPointNode>());
    rclcpp::shutdown();
    return 0;
}
