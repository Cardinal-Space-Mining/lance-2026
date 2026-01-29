#include <string>
#include <string_view>
#include <unordered_map>

#include <zenoh.hxx>

#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <multiscan_driver/multiscan_spec.hpp>

#include "ros_utils.hpp"
#include "mem_helpers.hpp"
#include "zenoh_utils.hpp"


#define CLIENT_IP_ADDRESS "10.11.11.8"

using namespace zenoh;
using namespace util;
using namespace util::ros_aliases;
using namespace util::zenoh_aliases;


class ClientAdapterNode : public rclcpp::Node
{
    using ImuMsg = sensor_msgs::msg::Imu;
    using PointField = sensor_msgs::msg::PointField;
    using PointCloudMsg = sensor_msgs::msg::PointCloud2;

public:
    ClientAdapterNode();

private:
    void imuCallback(const ImuMsg::ConstSharedPtr& msg);
    void scanCallback(const PointCloudMsg::ConstSharedPtr& msg);

private:
    Session zsh;

    ZenohPub imu_pub;
    ZenohPub scan_pub;

    SharedSub<ImuMsg> imu_sub;
    SharedSub<PointCloudMsg> scan_sub;
};


// ---

ClientAdapterNode::ClientAdapterNode() :
    Node{"robot_adapter_node"},
    zsh{Session::open(configDirectConnectTo(CLIENT_IP_ADDRESS))},
    imu_pub{zsh.declare_publisher("multiscan/imu")},
    scan_pub{zsh.declare_publisher("multiscan/lidar_scan")},
    imu_sub{this->create_subscription<ImuMsg>(
        "multiscan/imu",
        rclcpp::SensorDataQoS{},
        [this](const ImuMsg::ConstSharedPtr& msg) { this->imuCallback(msg); })},
    scan_sub{this->create_subscription<PointCloudMsg>(
        "multiscan/lidar_scan",
        rclcpp::SensorDataQoS{},
        [this](const PointCloudMsg::ConstSharedPtr& msg)
        { this->scanCallback(msg); })}
{
}

void ClientAdapterNode::imuCallback(const ImuMsg::ConstSharedPtr& msg)
{
    std::vector<uint8_t> bytes;
    bytes.resize(
        sizeof(decltype(msg->header.stamp.sec)) +
        sizeof(decltype(msg->header.stamp.nanosec)) + sizeof(float) * 7);

    uint8_t* ptr = bytes.data();
    assignAndIncrement(ptr, msg->header.stamp.sec);
    assignAndIncrement(ptr, msg->header.stamp.nanosec);
    assignAndIncrementAs<float>(ptr, msg->orientation.w);
    assignAndIncrementAs<float>(ptr, msg->orientation.x);
    assignAndIncrementAs<float>(ptr, msg->orientation.y);
    assignAndIncrementAs<float>(ptr, msg->orientation.z);
    assignAndIncrementAs<float>(ptr, msg->linear_acceleration.x);
    assignAndIncrementAs<float>(ptr, msg->linear_acceleration.y);
    assignAndIncrementAs<float>(ptr, msg->linear_acceleration.z);

    this->imu_pub.put(std::move(bytes));
}

void ClientAdapterNode::scanCallback(const PointCloudMsg::ConstSharedPtr& msg)
{
    if (msg->data.size() !=
        static_cast<size_t>(msg->height * msg->width * msg->point_step))
    {
        return;
    }

    uint32_t layer_off = 0;
    uint32_t index_off = 0;
    uint32_t range_off = 0;
    uint32_t reflector_off = 0;

    using ReqElems = std::pair<uint32_t*, uint8_t>;
    std::unordered_map<std::string, ReqElems> required_fields;
    required_fields.emplace("layer", ReqElems{&layer_off, PointField::UINT32});
    required_fields.emplace("index", ReqElems{&index_off, PointField::UINT32});
    required_fields.emplace("range", ReqElems{&range_off, PointField::FLOAT32});
    required_fields.emplace(
        "reflective",
        ReqElems{&reflector_off, PointField::FLOAT32});

    for (const PointField& field : msg->fields)
    {
        const auto iter = required_fields.find(field.name);
        if (iter != required_fields.end())
        {
            if (iter->second.second != field.datatype)
            {
                break;
            }
            *iter->second.first = field.offset;
            required_fields.erase(field.name);
        }
    }
    if (!required_fields.empty())
    {
        return;
    }

    ms136::redux::DenseBuffer dense_buff;
    for (auto p = msg->data.begin(); p < msg->data.end(); p += msg->point_step)
    {
        const uint8_t* pt_base = p.base();
        const size_t layer_i = static_cast<size_t>(
            *reinterpret_cast<const uint32_t*>(pt_base + layer_off));

        if (ms136::isHdLayer(layer_i))
        {
            continue;
        }

        const size_t index_i = static_cast<size_t>(
            *reinterpret_cast<const uint32_t*>(pt_base + index_off));
        const float range =
            *reinterpret_cast<const float*>(pt_base + range_off);
        const uint8_t reflector = static_cast<uint8_t>(
            *reinterpret_cast<const float*>(pt_base + reflector_off));

        ms136::redux::addPointToBuffer(
            dense_buff,
            layer_i,
            index_i,
            range,
            reflector);
    }

    ms136::redux::PackedBuffer packed_buff;
    ms136::redux::packBuffer(packed_buff, dense_buff);

    std::vector<uint8_t> bytes;
    bytes.resize(
        sizeof(decltype(msg->header.stamp.sec)) +
        sizeof(decltype(msg->header.stamp.nanosec)) + packed_buff.size() * 2);
    uint8_t* ptr = bytes.data();

    assignAndIncrement(ptr, msg->header.stamp.sec);
    assignAndIncrement(ptr, msg->header.stamp.nanosec);
    memcpy(ptr, packed_buff.data(), packed_buff.size() * 2);

    this->scan_pub.put(Bytes(std::move(bytes)));
}


int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ClientAdapterNode>());
    rclcpp::shutdown();

    return 0;
}
