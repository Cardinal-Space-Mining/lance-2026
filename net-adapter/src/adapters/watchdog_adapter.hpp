#pragma once

#include <std_msgs/msg/int32.hpp>

#include "base_adapter.hpp"

class WatchdogAdapter :
    public BaseAdapter<std_msgs::msg::Int32, WatchdogAdapter>
{
    friend BaseT;

protected:
    WatchdogAdapter(rclcpp::Node&);

    static bool serializeMsg(ByteBuffer&, const MsgT&, SubStateT&);
    static bool deserializeMsg(MsgT&, const ByteBuffer&, PubStateT&);
};
