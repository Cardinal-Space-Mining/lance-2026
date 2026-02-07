#include "watchdog_adapter.hpp"

#include "mem_helpers.hpp"

using namespace util;

WatchdogAdapter::WatchdogAdapter(rclcpp::Node& node) : BaseT(node) {}

bool WatchdogAdapter::serializeMsg(
    ByteBuffer& bytes,
    const MsgT& msg,
    SubStateT&)
{
    bytes.resize(sizeof(int32_t));

    uint8_t* ptr = bytes.data();
    writeAndIncrement(ptr, msg.data);

    return true;
}

bool WatchdogAdapter::deserializeMsg(
    MsgT& msg,
    const ByteBuffer& bytes,
    PubStateT&)
{
    if (bytes.size() != sizeof(int32_t))
    {
        return false;
    }

    const uint8_t* ptr = bytes.data();
    readAndIncrement(ptr, msg.data);

    return true;
}
