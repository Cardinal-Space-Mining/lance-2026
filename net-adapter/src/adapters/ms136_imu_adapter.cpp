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


MS136ImuAdapterPubState::MS136ImuAdapterPubState(rclcpp::Node& node) :
    lidar_frame_id{declare_and_get_param<std::string>(
        node,
        "lidar_frame_id",
        "lidar_link")}
{
}


MS136ImuAdapter::MS136ImuAdapter(rclcpp::Node& node) : BaseT{node} {}

bool MS136ImuAdapter::serializeMsg(
    ByteBuffer& bytes,
    const MsgT& msg,
    SubStateT& state)
{
    (void)state;

    bytes.resize(
        sizeof(decltype(msg.header.stamp.sec)) +      //
        sizeof(decltype(msg.header.stamp.nanosec)) +  //
        sizeof(float) * 7);

    uint8_t* ptr = bytes.data();
    writeAndIncrement(ptr, msg.header.stamp.sec);
    writeAndIncrement(ptr, msg.header.stamp.nanosec);
    writeAsAndIncrement<float>(ptr, msg.orientation.w);
    writeAsAndIncrement<float>(ptr, msg.orientation.x);
    writeAsAndIncrement<float>(ptr, msg.orientation.y);
    writeAsAndIncrement<float>(ptr, msg.orientation.z);
    writeAsAndIncrement<float>(ptr, msg.linear_acceleration.x);
    writeAsAndIncrement<float>(ptr, msg.linear_acceleration.y);
    writeAsAndIncrement<float>(ptr, msg.linear_acceleration.z);

    return true;
}

bool MS136ImuAdapter::deserializeMsg(
    MsgT& msg,
    const ByteBuffer& bytes,
    PubStateT& state)
{
    constexpr size_t TARGET_BUFF_SIZE = 36;
    if (bytes.size() < TARGET_BUFF_SIZE)
    {
        return false;
    }

    const uint8_t* ptr = bytes.data();
    readAndIncrement(ptr, msg.header.stamp.sec);
    readAndIncrement(ptr, msg.header.stamp.nanosec);
    readAsAndIncrement<float>(ptr, msg.orientation.w);
    readAsAndIncrement<float>(ptr, msg.orientation.x);
    readAsAndIncrement<float>(ptr, msg.orientation.y);
    readAsAndIncrement<float>(ptr, msg.orientation.z);
    readAsAndIncrement<float>(ptr, msg.linear_acceleration.x);
    readAsAndIncrement<float>(ptr, msg.linear_acceleration.y);
    readAsAndIncrement<float>(ptr, msg.linear_acceleration.z);

    msg.header.frame_id = state.lidar_frame_id;

    return true;
}
