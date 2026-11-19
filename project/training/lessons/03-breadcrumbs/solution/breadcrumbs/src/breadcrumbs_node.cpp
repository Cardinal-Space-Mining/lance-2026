// Lesson 3 reference solution: breadcrumbs_node
//
// Uses TF to find where the robot is, remembers the path it has driven, and
// publishes markers so the trail (and the distance driven) show up in
// Foxglove's 3D panel.

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


namespace
{

// Straight-line distance between two points, ignoring height (z), since we
// care about distance driven across the arena floor.
double distance2d(const PointMsg& a, const PointMsg& b)
{
    return std::hypot(b.x - a.x, b.y - a.y);
}

}  // namespace


class BreadcrumbsNode : public rclcpp::Node
{
public:
    BreadcrumbsNode() :
        Node("breadcrumbs"),
        // The listener needs the buffer to already exist. Members are
        // initialized in the order they are DECLARED in the class (see the
        // bottom), not the order written here, so tf_buffer must be declared
        // before tf_listener.
        tf_buffer{std::make_unique<tf2_ros::Buffer>(this->get_clock())},
        tf_listener{std::make_shared<tf2_ros::TransformListener>(*tf_buffer)}
    {
        this->fixed_frame = this->declare_parameter<std::string>("fixed_frame", "odom");
        this->robot_frame = this->declare_parameter<std::string>("robot_frame", "base_link");
        this->min_spacing_m = this->declare_parameter<double>("min_spacing_m", 0.05);
        this->max_points = static_cast<std::size_t>(
            this->declare_parameter<int>("max_points", 5000));

        this->markers_pub = this->create_publisher<MarkerArrayMsg>(
            "breadcrumbs/markers",
            rclcpp::QoS(1));

        this->update_timer =
            this->create_wall_timer(100ms, [this]() { this->update(); });

        RCLCPP_INFO(
            this->get_logger(),
            "Tracking '%s' in frame '%s'",
            this->robot_frame.c_str(),
            this->fixed_frame.c_str());
    }

private:
    // Where is the robot right now? Returns nothing (std::nullopt) if TF
    // can't answer yet, instead of crashing.
    std::optional<PointMsg> lookupRobotPosition()
    {
        try
        {
            // TimePointZero means "the most recent transform you have"
            const auto tf = this->tf_buffer->lookupTransform(
                this->fixed_frame,
                this->robot_frame,
                tf2::TimePointZero);

            PointMsg p;
            p.x = tf.transform.translation.x;
            p.y = tf.transform.translation.y;
            p.z = tf.transform.translation.z;
            return p;
        }
        catch (const tf2::TransformException& e)
        {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                2000,
                "Can't find robot position yet: %s",
                e.what());
            return std::nullopt;
        }
    }

    void update()
    {
        const std::optional<PointMsg> position = this->lookupRobotPosition();
        if (!position)
        {
            return;
        }

        // Only drop a new breadcrumb once the robot has moved far enough.
        // Otherwise a parked robot would fill the trail with duplicate points.
        if (this->trail.empty())
        {
            this->trail.push_back(*position);
        }
        else
        {
            const double moved = distance2d(this->trail.back(), *position);
            if (moved >= this->min_spacing_m)
            {
                this->trail.push_back(*position);
                this->distance_driven_m += moved;
            }
        }

        // Keep memory bounded: forget the oldest breadcrumbs
        while (this->trail.size() > this->max_points)
        {
            this->trail.pop_front();
        }

        this->publishMarkers(*position);
    }

    void publishMarkers(const PointMsg& robot_position)
    {
        MarkerArrayMsg msg;
        const auto stamp = this->now();

        // --- The trail: one line through every breadcrumb ---
        MarkerMsg& line = msg.markers.emplace_back();
        line.header.frame_id = this->fixed_frame;
        line.header.stamp = stamp;
        line.ns = "trail";
        line.id = 0;
        line.type = MarkerMsg::LINE_STRIP;
        line.action = MarkerMsg::ADD;
        line.pose.orientation.w = 1.0;  // identity rotation
        line.scale.x = 0.04;            // line width in meters
        line.color.r = 1.0f;
        line.color.g = 0.6f;
        line.color.b = 0.0f;
        line.color.a = 1.0f;
        line.points.assign(this->trail.begin(), this->trail.end());

        // --- A label floating above the robot ---
        MarkerMsg& label = msg.markers.emplace_back();
        label.header.frame_id = this->fixed_frame;
        label.header.stamp = stamp;
        label.ns = "label";
        label.id = 0;
        label.type = MarkerMsg::TEXT_VIEW_FACING;
        label.action = MarkerMsg::ADD;
        label.pose.position = robot_position;
        label.pose.position.z += 0.8;
        label.pose.orientation.w = 1.0;
        label.scale.z = 0.2;  // text height in meters
        label.color.r = 1.0f;
        label.color.g = 1.0f;
        label.color.b = 1.0f;
        label.color.a = 1.0f;

        char text[64];
        std::snprintf(text, sizeof(text), "%.2f m driven", this->distance_driven_m);
        label.text = text;

        this->markers_pub->publish(msg);
    }

private:
    // Declaration order matters: tf_buffer must come before tf_listener!
    std::unique_ptr<tf2_ros::Buffer> tf_buffer;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener;

    rclcpp::Publisher<MarkerArrayMsg>::SharedPtr markers_pub;
    rclcpp::TimerBase::SharedPtr update_timer;

    std::string fixed_frame;
    std::string robot_frame;
    double min_spacing_m{0.05};
    std::size_t max_points{5000};

    std::deque<PointMsg> trail;
    double distance_driven_m{0.0};
};


int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<BreadcrumbsNode>());
    rclcpp::shutdown();
    return 0;
}
