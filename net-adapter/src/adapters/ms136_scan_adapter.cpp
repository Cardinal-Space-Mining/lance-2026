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

#include "mem_helpers.hpp"
#include "../ros_utils.hpp"


using namespace util;


MS136ScanAdapter::MS136ScanAdapter(rclcpp::Node& node) : BaseT{node}
{
    declare_param(node, "lidar_frame_id", this->lidar_frame_id, "lidar_link");
}

bool MS136ScanAdapter::serializeMsg(
    ByteBuffer& bytes,
    const MsgT& msg,
    const SubStateT& state)
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
        sizeof(decltype(msg.header.stamp.sec)) +
        sizeof(decltype(msg.header.stamp.nanosec)) + packed_buff.size() * 2);
    uint8_t* ptr = bytes.data();

    assignAndIncrement(ptr, msg.header.stamp.sec);
    assignAndIncrement(ptr, msg.header.stamp.nanosec);
    memcpy(ptr, packed_buff.data(), packed_buff.size() * 2);

    return true;
}

bool MS136ScanAdapter::deserializeMsg(
    MsgT& msg,
    const ByteBuffer& bytes,
    const PubStateT& state)
{
    const uint8_t* ptr = bytes.data();

    msg.header.frame_id = state.lidar_frame_id;
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

    return true;
}
