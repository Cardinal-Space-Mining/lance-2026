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

#include "path_adapter.hpp"

#include <std_msgs/msg/header.hpp>

#include "../util/ros_utils.hpp"
#include "../util/mem_helpers.hpp"


using namespace util;


PathAdapterPubState::PathAdapterPubState(rclcpp::Node& node) :
    path_frame_id{
        declare_and_get_param<std::string>(node, "path_frame_id", "odom")}
{
}


PathAdapterSubState::PathAdapterSubState(rclcpp::Node& node) :
    FrequencyFilter{declare_and_get_param(node, "max_path_pub_freq", 1.f)}
{
}


PathAdapter::PathAdapter(rclcpp::Node& node) : BaseT(node) {}

bool PathAdapter::serializeMsg(
    ByteBuffer& bytes,
    const MsgT& msg,
    SubStateT& state)
{
    if (!state.freqFilterStatus())
    {
        return false;
    }

    bytes.resize(
        sizeof(decltype(msg.header.stamp.sec)) +      //
        sizeof(decltype(msg.header.stamp.nanosec)) +  //
        msg.poses.size() * (sizeof(float) * 3));

    uint8_t* ptr = bytes.data();

    writeAndIncrement(ptr, msg.header.stamp.sec);
    writeAndIncrement(ptr, msg.header.stamp.nanosec);

    for (const auto& pose : msg.poses)
    {
        writeAsAndIncrement<float>(ptr, pose.pose.position.x);
        writeAsAndIncrement<float>(ptr, pose.pose.position.y);
        writeAsAndIncrement<float>(ptr, pose.pose.position.z);
    }

    return true;
}

bool PathAdapter::deserializeMsg(
    MsgT& msg,
    const ByteBuffer& bytes,
    PubStateT& state)
{
    constexpr size_t STAMP_SIZE =
        (sizeof(decltype(msg.header.stamp.sec)) +
         sizeof(decltype(msg.header.stamp.nanosec)));
    constexpr size_t PT_SIZE = (3 * sizeof(float));

    const uint8_t* ptr = bytes.data();

    msg.header.frame_id = state.path_frame_id;
    readAndIncrement(ptr, msg.header.stamp.sec);
    readAndIncrement(ptr, msg.header.stamp.nanosec);

    msg.poses.clear();
    msg.poses.reserve((bytes.size() - STAMP_SIZE) / PT_SIZE);
    while (ptr + PT_SIZE <= bytes.end().base())
    {
        auto& kp = msg.poses.emplace_back();
        kp.header = msg.header;
        readAsAndIncrement<float>(ptr, kp.pose.position.x);
        readAsAndIncrement<float>(ptr, kp.pose.position.y);
        readAsAndIncrement<float>(ptr, kp.pose.position.z);
    }

    return true;
}
