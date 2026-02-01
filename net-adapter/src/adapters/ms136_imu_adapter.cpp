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

#include "ms136_imu_adapter.hpp"

#include <cstdint>

#include "mem_helpers.hpp"
#include "../ros_utils.hpp"


using namespace util;


MS136ImuAdapter::MS136ImuAdapter(rclcpp::Node& node) : BaseT{node}
{
    declare_param(node, "lidar_frame_id", this->lidar_frame_id, "lidar_link");
}

bool MS136ImuAdapter::serializeMsg(
    ByteBuffer& bytes,
    const MsgT& msg,
    const SubStateT& state)
{
    (void)state;

    bytes.resize(
        sizeof(decltype(msg.header.stamp.sec)) +
        sizeof(decltype(msg.header.stamp.nanosec)) + sizeof(float) * 7);

    uint8_t* ptr = bytes.data();
    write(ptr, msg.header.stamp.sec);
    write(ptr, msg.header.stamp.nanosec);
    write_as<float>(ptr, msg.orientation.w);
    write_as<float>(ptr, msg.orientation.x);
    write_as<float>(ptr, msg.orientation.y);
    write_as<float>(ptr, msg.orientation.z);
    write_as<float>(ptr, msg.linear_acceleration.x);
    write_as<float>(ptr, msg.linear_acceleration.y);
    write_as<float>(ptr, msg.linear_acceleration.z);

    return true;
}

bool MS136ImuAdapter::deserializeMsg(
    MsgT& msg,
    const ByteBuffer& bytes,
    const PubStateT& state)
{
    constexpr size_t TARGET_BUFF_SIZE = 36;
    if (bytes.size() < TARGET_BUFF_SIZE)
    {
        return false;
    }

    const uint8_t* ptr = bytes.data();
    read(ptr, msg.header.stamp.sec);
    read(ptr, msg.header.stamp.nanosec);
    read_as<float>(ptr, msg.orientation.w);
    read_as<float>(ptr, msg.orientation.x);
    read_as<float>(ptr, msg.orientation.y);
    read_as<float>(ptr, msg.orientation.z);
    read_as<float>(ptr, msg.linear_acceleration.x);
    read_as<float>(ptr, msg.linear_acceleration.y);
    read_as<float>(ptr, msg.linear_acceleration.z);

    msg.header.frame_id = state.lidar_frame_id;

    return true;
}
