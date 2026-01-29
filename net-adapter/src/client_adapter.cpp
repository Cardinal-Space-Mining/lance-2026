#include <zenoh.hxx>

#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <multiscan_driver/multiscan_spec.hpp>

#include "ros_utils.hpp"
#include "zenoh_utils.hpp"
#include "mem_helpers.hpp"


#define ROBOT_IP_ADDRESS "10.11.11.10"

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
    void imuCallback(const Sample& sample);
    void scanCallback(const Sample& sample);

private:
    Session zsh;

    SharedPub<ImuMsg> imu_pub;
    SharedPub<PointCloudMsg> scan_pub;

    ZenohSub imu_sub;
    ZenohSub scan_sub;
};


// ---

ClientAdapterNode::ClientAdapterNode() :
    Node{"client_adapter_node"},
    zsh{Session::open(configDirectConnectTo(ROBOT_IP_ADDRESS))},
    imu_pub{
        this->create_publisher<ImuMsg>("/redux/imu", rclcpp::SensorDataQoS{})},
    scan_pub{this->create_publisher<PointCloudMsg>(
        "/redux/lidar_scan",
        rclcpp::SensorDataQoS{})},
    imu_sub{zsh.declare_subscriber(
        "multiscan/imu",
        [this](const Sample& sample) { this->imuCallback(sample); },
        []() {})},
    scan_sub{zsh.declare_subscriber(
        "multiscan/lidar_scan",
        [this](const Sample& sample) { this->scanCallback(sample); },
        []() {})}
{
}

void ClientAdapterNode::imuCallback(const Sample& sample)
{
    std::vector<uint8_t> bytes = sample.get_payload().as_vector();

    constexpr size_t TARGET_BUFF_SIZE = 36;
    if (bytes.size() < TARGET_BUFF_SIZE)
    {
        return;
    }

    ImuMsg msg;
    uint8_t* ptr = bytes.data();

    extractAndIncrement(ptr, msg.header.stamp.sec);
    extractAndIncrement(ptr, msg.header.stamp.nanosec);
    extractAndIncrementAs<float>(ptr, msg.orientation.w);
    extractAndIncrementAs<float>(ptr, msg.orientation.x);
    extractAndIncrementAs<float>(ptr, msg.orientation.y);
    extractAndIncrementAs<float>(ptr, msg.orientation.z);
    extractAndIncrementAs<float>(ptr, msg.linear_acceleration.x);
    extractAndIncrementAs<float>(ptr, msg.linear_acceleration.y);
    extractAndIncrementAs<float>(ptr, msg.linear_acceleration.z);

    msg.header.frame_id = "lidar_link";

    this->imu_pub->publish(msg);
}

void ClientAdapterNode::scanCallback(const Sample& sample)
{
    std::vector<uint8_t> bytes = sample.get_payload().as_vector();
    uint8_t* ptr = bytes.data();

    PointCloudMsg msg;

    msg.header.frame_id = "lidar_link";
    extractAndIncrement(ptr, msg.header.stamp.sec);
    extractAndIncrement(ptr, msg.header.stamp.nanosec);

    const size_t n_packed_bytes = (bytes.end().base() - ptr);

    ms136::redux::PackedBuffer packed_buff;
    packed_buff.resize(n_packed_bytes / 2);
    memcpy(packed_buff.data(), ptr, n_packed_bytes);

    ms136::redux::DenseBuffer dense_buff;
    ms136::redux::unpackBuffer(dense_buff, packed_buff);

    constexpr size_t POINT_BYTE_LEN = 16;

    msg.data.reserve(dense_buff.size() * POINT_BYTE_LEN);
    for (size_t i = 0; i < dense_buff.size(); i++)
    {
        const uint16_t pt = dense_buff[i];
        if (pt)
        {
            const size_t prev_end_off = msg.data.size();
            msg.data.resize(msg.data.size() + POINT_BYTE_LEN);
            uint8_t* ptr = msg.data.data() + prev_end_off;

            const auto proj = ms136::redux::projectPoint(i, pt);
            const bool reflector = ms136::redux::getReflector(pt);

            assignAndIncrement(ptr, proj.x());
            assignAndIncrement(ptr, proj.y());
            assignAndIncrement(ptr, proj.z());
            assignAndIncrement(ptr, reflector ? 1.f : 0.f);
        }
    }

    msg.fields = {
        PointField{}
            .set__name("x")
            .set__datatype(PointField::FLOAT32)
            .set__count(1)
            .set__offset(0),
        PointField{}
            .set__name("y")
            .set__datatype(PointField::FLOAT32)
            .set__count(1)
            .set__offset(4),
        PointField{}
            .set__name("z")
            .set__datatype(PointField::FLOAT32)
            .set__count(1)
            .set__offset(8),
        PointField{}
            .set__name("reflective")
            .set__datatype(PointField::FLOAT32)
            .set__count(1)
            .set__offset(12)};
    msg.is_bigendian = false;
    msg.point_step = POINT_BYTE_LEN;
    msg.row_step = msg.data.size();
    msg.height = 1;
    msg.width = (msg.data.size() / POINT_BYTE_LEN);
    msg.is_dense = true;

    this->scan_pub->publish(msg);
}


int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ClientAdapterNode>());
    rclcpp::shutdown();

    return 0;
}
