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

#include "mem_helpers.hpp"

// todo parametrize this?
#define FRAME_ID "odom"

using namespace util;


PathAdapter::PathAdapter(rclcpp::Node& node) : BaseT(node) {}

bool PathAdapter::serializeMsg(ByteBuffer& bytes, const MsgT& msg, SubStateT&)
{
    bytes.resize(sizeof(float) * 3 * msg.poses.size());

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

bool PathAdapter::deserializeMsg(MsgT& msg, const ByteBuffer& bytes, PubStateT&)
{
    if (bytes.size() % (sizeof(float) * 3) != 0)
    {
        return false;
    }

    const uint8_t* ptr = bytes.data();
    std_msgs::msg::Header header;

    msg.header.frame_id = FRAME_ID;
    readAndIncrement(ptr, msg.header.stamp.sec);
    readAndIncrement(ptr, msg.header.stamp.nanosec);

    msg.poses.clear();

    msg.header = header;

    size_t numPoses = bytes.size() / (sizeof(float) * 3);
    msg.poses.reserve(numPoses);

    for (size_t i = 0; i < numPoses; ++i)
    {
        geometry_msgs::msg::PoseStamped pose;
        readAsAndIncrement<float>(ptr, pose.pose.position.x);
        readAsAndIncrement<float>(ptr, pose.pose.position.y);
        readAsAndIncrement<float>(ptr, pose.pose.position.z);
        pose.header = header;
        msg.poses.push_back(pose);
    }

    return true;
}
