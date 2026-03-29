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

#pragma once

#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>

#include "base_adapter.hpp"


template<typename MsgT, int Compression = 0>
class GenericAdapter :
    public BaseAdapter<MsgT, GenericAdapter<MsgT, Compression>, Compression>
{
    using BaseT =
        BaseAdapter<MsgT, GenericAdapter<MsgT, Compression>, Compression>;

    friend BaseT;

    using typename BaseT::SubStateT;
    using typename BaseT::PubStateT;
    using typename BaseT::ByteBuffer;

protected:
    GenericAdapter(rclcpp::Node&);

protected:
    static bool serializeMsg(ByteBuffer&, const MsgT&, SubStateT&);
    static bool deserializeMsg(MsgT&, const ByteBuffer&, PubStateT&);

private:
    rclcpp::Serialization<MsgT> serializer{};
};



// ---

template<typename M, int C>
GenericAdapter<M, C>::GenericAdapter(rclcpp::Node& n) : BaseT{n}
{
}

template<typename M, int C>
bool GenericAdapter<M, C>::serializeMsg(
    ByteBuffer& bytes,
    const M& msg,
    SubStateT& state)
{
    rclcpp::SerializedMessage serialized_msg;
    try
    {
        state.serializer.serialize_message(&msg, &serialized_msg);
    }
    catch (const std::exception&)
    {
        return false;
    }

    bytes.resize(serialized_msg.size());
    std::memcpy(
        bytes.data(),
        serialized_msg.get_rcl_serialized_message().buffer,
        serialized_msg.size());

    return true;
}

template<typename M, int C>
bool GenericAdapter<M, C>::deserializeMsg(
    M& msg,
    const ByteBuffer& bytes,
    PubStateT& state)
{
    rclcpp::SerializedMessage serialized_msg(bytes.size());

    auto& rcl_msg = serialized_msg.get_rcl_serialized_message();
    std::memcpy(rcl_msg.buffer, bytes.data(), bytes.size());
    rcl_msg.buffer_length = bytes.size();

    try
    {
        state.serializer.deserialize_message(&serialized_msg, &msg);
    }
    catch (const std::exception&)
    {
        return false;
    }

    return true;
}
