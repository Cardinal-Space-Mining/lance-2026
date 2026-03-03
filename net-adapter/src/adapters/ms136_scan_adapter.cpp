/*******************************************************************************
*   Copyright (C) 2024-2026 Cardinal Space Mining Club                         *
*                                                                              *
*                                 ;xxxxxxx:                                    *
*                                ;$$$$$$$$$       ...::..                      *
*                                $$$$$$$$$$x   .:::::::::::..                  *
*                             x$$$$$$$$$$$$$$::::::::::::::::.                 *
*                         :$$$$$&X;      .xX:::::::::::::.::...                *
*                 .$$Xx++$$$$+  :::.     :;:   .::::::.  ....  :               *
*                :$$$$$$$$$  ;:      ;xXXXXXXXx  .::.  .::::. .:.              *
*               :$$$$$$$$: ;      ;xXXXXXXXXXXXXx: ..::::::  .::.              *
*              ;$$$$$$$$ ::   :;XXXXXXXXXXXXXXXXXX+ .::::.  .:::               *
*               X$$$$$X : +XXXXXXXXXXXXXXXXXXXXXXXX; .::  .::::.               *
*                .$$$$ :xXXXXXXXXXXXXXXXXXXXXXXXXXXX.   .:::::.                *
*                 X$$X XXXXXXXXXXXXXXXXXXXXXXXXXXXXx:  .::::.                  *
*                 $$$:.XXXXXXXXXXXXXXXXXXXXXXXXXXX  ;; ..:.                    *
*                 $$& :XXXXXXXXXXXXXXXXXXXXXXXX;  +XX; X$$;                    *
*                 $$$: XXXXXXXXXXXXXXXXXXXXXX; :XXXXX; X$$;                    *
*                 X$$X XXXXXXXXXXXXXXXXXXX; .+XXXXXXX; $$$                     *
*                 $$$$ ;XXXXXXXXXXXXXXX+  +XXXXXXXXx+ X$$$+                    *
*               x$$$$$X ;XXXXXXXXXXX+ :xXXXXXXXX+   .;$$$$$$                   *
*              +$$$$$$$$ ;XXXXXXx;;+XXXXXXXXX+    : +$$$$$$$$                  *
*               +$$$$$$$$: xXXXXXXXXXXXXXX+      ; X$$$$$$$$                   *
*                :$$$$$$$$$. +XXXXXXXXX;      ;: x$$$$$$$$$                    *
*                ;x$$$$XX$$$$+ .;+X+      :;: :$$$$$xX$$$X                     *
*               ;;;;;;;;;;X$$$$$$$+      :X$$$$$$&.                            *
*               ;;;;;;;:;;;;;x$$$$$$$$$$$$$$$$x.                               *
*               :;;;;;;;;;;;;.  :$$$$$$$$$$X                                   *
*                .;;;;;;;;:;;    +$$$$$$$$$                                    *
*                  .;;;;;;.       X$$$$$$$:                                    *
*                                                                              *
*   Unless required by applicable law or agreed to in writing, software        *
*   distributed under the License is distributed on an "AS IS" BASIS,          *
*   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.   *
*   See the License for the specific language governing permissions and        *
*   limitations under the License.                                             *
*                                                                              *
*******************************************************************************/

#include "ms136_scan_adapter.hpp"

#include <memory>
#include <string>
#include <cstdint>
#include <utility>
#include <unordered_map>

#include <sensor_msgs/msg/point_field.hpp>

#include <multiscan_driver/multiscan_spec.hpp>

#include "../util/ros_utils.hpp"
#include "../util/mem_helpers.hpp"


using namespace util;

using PointField = sensor_msgs::msg::PointField;

#define OUTPUT_POINT_BYTE_LEN 16
#define OUTPUT_POINT_FIELD_LIST              \
    {PointField{}                            \
         .set__name("x")                     \
         .set__datatype(PointField::FLOAT32) \
         .set__count(1)                      \
         .set__offset(0),                    \
     PointField{}                            \
         .set__name("y")                     \
         .set__datatype(PointField::FLOAT32) \
         .set__count(1)                      \
         .set__offset(4),                    \
     PointField{}                            \
         .set__name("z")                     \
         .set__datatype(PointField::FLOAT32) \
         .set__count(1)                      \
         .set__offset(8),                    \
     PointField{}                            \
         .set__name("reflective")            \
         .set__datatype(PointField::FLOAT32) \
         .set__count(1)                      \
         .set__offset(12)}



// --- MS136ScanAdapterPubState ------------------------------------------------

MS136ScanAdapterPubState::MS136ScanAdapterPubState(rclcpp::Node& node) :
    lidar_frame_id{declare_and_get_param<std::string>(
        node,
        "lidar_frame_id",
        "lidar_link")}
{
}


// --- MS136ScanAdapter --------------------------------------------------------

MS136ScanAdapter::MS136ScanAdapter(rclcpp::Node& node) : BaseT{node} {}

bool MS136ScanAdapter::serializeMsg(
    ByteBuffer& bytes,
    const MsgT& msg,
    SubStateT& state)
{
    (void)state;

    if (msg.data.size() !=
        static_cast<size_t>(msg.height * msg.width * msg.point_step))
    {
        return false;
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

    for (const PointField& field : msg.fields)
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
        return false;
    }

    ms136::redux::DenseBuffer dense_buff;
    for (auto p = msg.data.begin(); p < msg.data.end(); p += msg.point_step)
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

    bytes.resize(
        sizeof(decltype(msg.header.stamp.sec)) +      //
        sizeof(decltype(msg.header.stamp.nanosec)) +  //
        packed_buff.size() * sizeof(uint16_t));
    uint8_t* ptr = bytes.data();

    writeAndIncrement(ptr, msg.header.stamp.sec);
    writeAndIncrement(ptr, msg.header.stamp.nanosec);
    writeManyAndIncrement(ptr, packed_buff);

    return true;
}

bool MS136ScanAdapter::deserializeMsg(
    MsgT& msg,
    const ByteBuffer& bytes,
    PubStateT& state)
{
    const uint8_t* ptr = bytes.data();

    msg.header.frame_id = state.lidar_frame_id;
    readAndIncrement(ptr, msg.header.stamp.sec);
    readAndIncrement(ptr, msg.header.stamp.nanosec);

    const size_t n_packed_bytes = (bytes.end().base() - ptr);

    ms136::redux::PackedBuffer packed_buff;
    readManyAndIncrement(ptr, packed_buff, n_packed_bytes / sizeof(uint16_t));

    ms136::redux::DenseBuffer dense_buff;
    ms136::redux::unpackBuffer(dense_buff, packed_buff);

    msg.data.reserve(dense_buff.size() * OUTPUT_POINT_BYTE_LEN);
    for (size_t i = 0; i < dense_buff.size(); i++)
    {
        const uint16_t pt = dense_buff[i];
        if (pt)
        {
            const size_t prev_end_off = msg.data.size();
            msg.data.resize(msg.data.size() + OUTPUT_POINT_BYTE_LEN);
            uint8_t* ptr = msg.data.data() + prev_end_off;

            const auto proj = ms136::redux::projectPoint(i, pt);
            const bool reflector = ms136::redux::getReflector(pt);

            writeAndIncrement(ptr, proj.x());
            writeAndIncrement(ptr, proj.y());
            writeAndIncrement(ptr, proj.z());
            writeAndIncrement(ptr, reflector ? 1.f : 0.f);
        }
    }

    msg.fields = OUTPUT_POINT_FIELD_LIST;
    msg.is_bigendian = false;
    msg.point_step = OUTPUT_POINT_BYTE_LEN;
    msg.row_step = msg.data.size();
    msg.height = 1;
    msg.width = (msg.data.size() / OUTPUT_POINT_BYTE_LEN);
    msg.is_dense = true;

    return true;
}


// --- MS136SimScanAdapter (and utils) -----------------------------------------

#include <numbers>

#include <sensor_msgs/point_cloud2_iterator.hpp>


Eigen::Vector3f projectGzSimPoint(size_t i, uint16_t pt)
{
    constexpr float THETA_START = 0.f;
    constexpr float THETA_STEP =
        (2.f * std::numbers::pi_v<float>) / (ms136::POINTS_PER_LR_LAYER - 1);
    constexpr float PHI_START = ms136::redux::ELEVATION_LUT.front();
    constexpr float PHI_STEP = (ms136::redux::ELEVATION_LUT.back() -
                                ms136::redux::ELEVATION_LUT.front()) /
                               (ms136::NUM_LR_LAYERS - 1);


    const size_t layer_i = i / ms136::POINTS_PER_LR_LAYER;
    const size_t local_i = i % ms136::POINTS_PER_LR_LAYER;

    const float r = ms136::redux::getRangeMeters(pt);
    const float theta = THETA_START + THETA_STEP * local_i;
    const float phi = PHI_START + PHI_STEP * layer_i;
    const float sin_theta = std::sin(theta);
    const float cos_theta = std::cos(theta);
    const float sin_phi = std::sin(phi);
    const float cos_phi = std::cos(phi);

    return Eigen::Vector3f{
        r * cos_phi * cos_theta,
        r * cos_phi * sin_theta,
        r * sin_phi};
}


MS136SimScanAdapter::MS136SimScanAdapter(rclcpp::Node& node) : BaseT{node} {}

bool MS136SimScanAdapter::serializeMsg(
    ByteBuffer& bytes,
    const MsgT& msg,
    SubStateT& state)
{
    (void)state;

    const size_t n_pts = static_cast<size_t>(msg.height * msg.width);
    if ((n_pts != ms136::POINTS_PER_LR_SCAN) ||
        (msg.data.size() != (n_pts * msg.point_step)))
    {
        return false;
    }

    ms136::redux::DenseBuffer dense_buff;
    try
    {
        sensor_msgs::PointCloud2ConstIterator<float> x_iter(msg, "x");
        sensor_msgs::PointCloud2ConstIterator<float> y_iter(msg, "y");
        sensor_msgs::PointCloud2ConstIterator<float> z_iter(msg, "z");
        sensor_msgs::PointCloud2ConstIterator<float> rfl_iter(
            msg,
            "reflective");
        // ^ if the fields aren't FLOAT32, this silently misreads the data!

        for (size_t i = 0; i < n_pts;
             i++, ++x_iter, ++y_iter, ++z_iter, ++rfl_iter)
        {
            const float x = *x_iter;
            const float y = *y_iter;
            const float z = *z_iter;

            const float r = std::sqrt(x * x + y * y + z * z);
            const uint8_t rf = *rfl_iter == 0.f ? 0 : 1;

            dense_buff[i] = ms136::redux::reducePoint(r, rf);
        }
    }
    catch (...)
    {
        return false;
    }

    ms136::redux::PackedBuffer packed_buff;
    ms136::redux::packBuffer(packed_buff, dense_buff);

    bytes.resize(
        sizeof(decltype(msg.header.stamp.sec)) +      //
        sizeof(decltype(msg.header.stamp.nanosec)) +  //
        packed_buff.size() * sizeof(uint16_t));
    uint8_t* ptr = bytes.data();

    writeAndIncrement(ptr, msg.header.stamp.sec);
    writeAndIncrement(ptr, msg.header.stamp.nanosec);
    writeManyAndIncrement(ptr, packed_buff);

    return true;
}

bool MS136SimScanAdapter::deserializeMsg(
    MsgT& msg,
    const ByteBuffer& bytes,
    PubStateT& state)
{
    const uint8_t* ptr = bytes.data();

    msg.header.frame_id = state.lidar_frame_id;
    readAndIncrement(ptr, msg.header.stamp.sec);
    readAndIncrement(ptr, msg.header.stamp.nanosec);

    const size_t n_packed_bytes = (bytes.end().base() - ptr);

    ms136::redux::PackedBuffer packed_buff;
    readManyAndIncrement(ptr, packed_buff, n_packed_bytes / sizeof(uint16_t));

    ms136::redux::DenseBuffer dense_buff;
    ms136::redux::unpackBuffer(dense_buff, packed_buff);

    msg.data.reserve(dense_buff.size() * OUTPUT_POINT_BYTE_LEN);
    for (size_t i = 0; i < dense_buff.size(); i++)
    {
        const uint16_t pt = dense_buff[i];
        if (pt)
        {
            const size_t prev_end_off = msg.data.size();
            msg.data.resize(msg.data.size() + OUTPUT_POINT_BYTE_LEN);
            uint8_t* ptr = msg.data.data() + prev_end_off;

            const auto proj = projectGzSimPoint(i, pt);
            const bool reflector = ms136::redux::getReflector(pt);

            writeAndIncrement(ptr, proj.x());
            writeAndIncrement(ptr, proj.y());
            writeAndIncrement(ptr, proj.z());
            writeAndIncrement(ptr, reflector ? 1.f : 0.f);
        }
    }

    msg.fields = OUTPUT_POINT_FIELD_LIST;
    msg.is_bigendian = false;
    msg.point_step = OUTPUT_POINT_BYTE_LEN;
    msg.row_step = msg.data.size();
    msg.height = 1;
    msg.width = (msg.data.size() / OUTPUT_POINT_BYTE_LEN);
    msg.is_dense = true;

    return true;
}
