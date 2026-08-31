/*******************************************************************************
*   Copyright (C) 2025-2026 Cardinal Space Mining Club                         *
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

#include "remote_commands.hpp"

#include <csm_utils/mem_helpers.hpp>


using namespace util;


static void writeTraversalBytes(
    uint8_t*& ptr,
    uint8_t cmd,
    lance::KeyFrame frame_id,
    const lance::geom::Pose3f& pose)
{
    ptr[0] = cmd;
    ptr[1] = static_cast<uint8_t>(frame_id);

    ptr += 2;

    writeAndIncrement(ptr, pose.vec.x());
    writeAndIncrement(ptr, pose.vec.y());
    writeAndIncrement(ptr, pose.vec.z());
    writeAndIncrement(ptr, pose.quat.w());
    writeAndIncrement(ptr, pose.quat.x());
    writeAndIncrement(ptr, pose.quat.y());
    writeAndIncrement(ptr, pose.quat.z());
}

static void readTraversalBytes(
    const uint8_t*& ptr,
    lance::KeyFrame& frame_id,
    lance::geom::Pose3f& pose)
{
    frame_id = static_cast<lance::KeyFrame>(ptr[1]);

    ptr += 2;

    readAndIncrement(ptr, pose.vec.x());
    readAndIncrement(ptr, pose.vec.y());
    readAndIncrement(ptr, pose.vec.z());
    readAndIncrement(ptr, pose.quat.w());
    readAndIncrement(ptr, pose.quat.x());
    readAndIncrement(ptr, pose.quat.y());
    readAndIncrement(ptr, pose.quat.z());
}


namespace lance
{

void RemoteCommands::serializeTraversalCmd(
    BytesMsg& msg,
    const Pose3f& pose,
    KeyFrame frame_id)
{
    constexpr size_t RESERVE_SIZE = (sizeof(uint8_t) * 2 + sizeof(float) * 7);

    msg.data.clear();
    msg.data.resize(RESERVE_SIZE);

    auto* ptr = msg.data.data();
    writeTraversalBytes(ptr, COMMAND_TRAVERSAL, frame_id, pose);
}

void RemoteCommands::serializeMiningCmd(BytesMsg& msg, const Pose3f& pose)
{
    constexpr size_t RESERVE_SIZE = (sizeof(uint8_t) * 2 + sizeof(float) * 7);

    msg.data.clear();
    msg.data.resize(RESERVE_SIZE);

    auto* ptr = msg.data.data();
    writeTraversalBytes(ptr, COMMAND_MINING, KeyFrame::ARENA_FRAME, pose);
}

void RemoteCommands::serializeOffloadCmd(
    BytesMsg& msg,
    const Pose3f& pose,
    float dist)
{
    constexpr size_t RESERVE_SIZE = (sizeof(uint8_t) * 2 + sizeof(float) * 8);

    msg.data.clear();
    msg.data.resize(RESERVE_SIZE);

    auto* ptr = msg.data.data();
    writeTraversalBytes(ptr, COMMAND_OFFLOAD, KeyFrame::ARENA_FRAME, pose);

    writeAndIncrement(ptr, dist);
}


uint8_t RemoteCommands::getCmdType(const BytesMsg& msg)
{
    return msg.data.empty() ? static_cast<uint8_t>(COMMAND_INVALID)
                            : msg.data.front();
}

bool RemoteCommands::deserializeTraversalCmd(
    const BytesMsg& msg,
    Pose3f& pose,
    KeyFrame& frame_id)
{
    constexpr size_t REQUIRED_SIZE = (sizeof(uint8_t) * 2 + sizeof(float) * 7);

    if (msg.data.size() < REQUIRED_SIZE)
    {
        return false;
    }

    const auto* ptr = msg.data.data();
    readTraversalBytes(ptr, frame_id, pose);

    return true;
}

bool RemoteCommands::deserializeMiningCmd(const BytesMsg& msg, Pose3f& pose)
{
    constexpr size_t REQUIRED_SIZE = (sizeof(uint8_t) * 2 + sizeof(float) * 7);

    if (msg.data.size() < REQUIRED_SIZE)
    {
        return false;
    }

    const auto* ptr = msg.data.data();
    KeyFrame f;
    readTraversalBytes(ptr, f, pose);

    return true;
}

bool RemoteCommands::deserializeOffloadCmd(
    const BytesMsg& msg,
    Pose3f& pose,
    float& dist)
{
    constexpr size_t REQUIRED_SIZE = (sizeof(uint8_t) * 2 + sizeof(float) * 8);

    if (msg.data.size() < REQUIRED_SIZE)
    {
        return false;
    }

    const auto* ptr = msg.data.data();
    KeyFrame f;
    readTraversalBytes(ptr, f, pose);
    readAndIncrement(ptr, dist);

    return true;
}

};  // namespace lance
