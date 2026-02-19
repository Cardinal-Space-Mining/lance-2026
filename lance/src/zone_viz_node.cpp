#include <chrono>
#include <memory>
#include <vector>
#include <cassert>

#include <rclcpp/rclcpp.hpp>

#include <visualization_msgs/msg/marker_array.hpp>

#include "util/ros_utils.hpp"


using namespace std::chrono_literals;
using namespace util::ros_aliases;


class ZoneVizNode : public rclcpp::Node
{
    using MarkerMsg = visualization_msgs::msg::Marker;
    using MarkerArrayMsg = visualization_msgs::msg::MarkerArray;

public:
    ZoneVizNode();

protected:
    void initMarkers();

private:
    SharedPub<MarkerArrayMsg> pub;
    RclTimer pub_timer;

    MarkerArrayMsg markers;
};



ZoneVizNode::ZoneVizNode() :
    Node("zone_visualizer"),
    pub{this->create_publisher<MarkerArrayMsg>(
        "arena_zones",
        rclcpp::SensorDataQoS{})},
    pub_timer{this->create_wall_timer(
        1s,
        [this]() { this->pub->publish(this->markers); })}
{
    this->initMarkers();
}

void ZoneVizNode::initMarkers()
{
    std::string frame_id;
    std::vector<double> min, max;

    util::declare_param(*this, "frame_id", frame_id, "map");

#define ADD_MARKER(param, NS, R, G, B, A)                            \
    {                                                                \
        util::declare_param(*this, param ".min", min, {0., 0., 0.}); \
        util::declare_param(*this, param ".max", max, {0., 0., 0.}); \
        assert(min.size() >= 3 && max.size() >= 3);                  \
        MarkerMsg& marker = this->markers.markers.emplace_back();    \
        marker.header.frame_id = frame_id;                           \
        marker.header.stamp = this->now();                           \
        marker.ns = NS;                                              \
        marker.id = this->markers.markers.size();                    \
        marker.type = MarkerMsg::CUBE;                               \
        marker.action = MarkerMsg::ADD;                              \
        marker.lifetime = rclcpp::Duration(0, 0);                    \
        marker.pose.position.x = (min[0] + max[0]) / 2.;             \
        marker.pose.position.y = (min[1] + max[1]) / 2.;             \
        marker.pose.position.z = (min[2] + max[2]) / 2.;             \
        marker.scale.x = (max[0] - min[0]);                          \
        marker.scale.y = (max[1] - min[1]);                          \
        marker.scale.z = (max[2] - min[2]);                          \
        marker.color.r = R;                                          \
        marker.color.g = G;                                          \
        marker.color.b = B;                                          \
        marker.color.a = A;                                          \
    }
    ADD_MARKER("arena_bounds", "arena", 1.f, 1.f, 1.f, 0.f);
    ADD_MARKER("mining_zone_bounds", "zones", 0.8f, 0.4f, 0.f, 0.5f);
    ADD_MARKER("offload_zone_bounds", "zones", 0.f, 0.2f, 0.8f, 0.3f);

#undef ADD_MARKER
}



int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ZoneVizNode>());
    rclcpp::shutdown();
    return 0;
}
