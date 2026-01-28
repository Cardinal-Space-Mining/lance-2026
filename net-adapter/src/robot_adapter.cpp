#include <zenoh.hxx>

#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <multiscan_driver/multiscan_spec.hpp>

#include "ros_utils.hpp"


using namespace zenoh;
using namespace util::ros_aliases;


class AdapterNode : public rclcpp::Node
{
    using ImuMsg = sensor_msgs::msg::Imu;
    using PointCloudMsg = sensor_msgs::msg::PointCloud2;

public:
    AdapterNode();

private:
    zenoh::Session zsh;

    SharedSub<ImuMsg> imu_sub;
    SharedSub<PointCloudMsg> scan_sub;
};


AdapterNode::AdapterNode() :
    Node{ "adapter_node" },
    zsh{Session::open(Config::create_default())}
{
}


int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AdapterNode>());
    rclcpp::shutdown();

    return 0;
}
