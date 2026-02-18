#include <chrono>
#include <memory>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>


using namespace std::chrono_literals;


class BBoxPublisher : public rclcpp::Node
{
public:
  BBoxPublisher()
  : Node("bbox_publisher")
  {
    publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
      "bounding_boxes", 10);

    timer_ = this->create_wall_timer(
      500ms, std::bind(&BBoxPublisher::publish_boxes, this));
  }

private:
  void publish_boxes()
  {
    visualization_msgs::msg::MarkerArray marker_array;

    for (int i = 0; i < 3; ++i)
    {
      visualization_msgs::msg::Marker marker;

      marker.header.frame_id = "map";
      marker.header.stamp = this->now();

      marker.ns = "bbox";
      marker.id = i;

      marker.type = visualization_msgs::msg::Marker::CUBE;
      marker.action = visualization_msgs::msg::Marker::ADD;

      // Position
      marker.pose.position.x = i * 2.0;
      marker.pose.position.y = 0.0;
      marker.pose.position.z = 1.0;

      marker.pose.orientation.x = 0.0;
      marker.pose.orientation.y = 0.0;
      marker.pose.orientation.z = 0.0;
      marker.pose.orientation.w = 1.0;

      // Box dimensions
      marker.scale.x = 1.0;  // length
      marker.scale.y = 1.5;  // width
      marker.scale.z = 2.0;  // height

      // Color (RGBA)
      marker.color.r = 0.0f;
      marker.color.g = 1.0f;
      marker.color.b = 0.0f;
      marker.color.a = 0.6f;  // transparency

      marker.lifetime = rclcpp::Duration(0, 0);  // persistent

      marker_array.markers.push_back(marker);
    }

    publisher_->publish(marker_array);
  }

  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BBoxPublisher>());
  rclcpp::shutdown();
  return 0;
}
