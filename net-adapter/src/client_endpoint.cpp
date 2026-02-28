#include "endpoint_def.hpp"

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<EndpointNode<NodeRole::Client>>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}