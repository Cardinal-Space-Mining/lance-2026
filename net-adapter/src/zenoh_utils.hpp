#pragma once

#include <string>

#include <zenoh.hxx>


namespace util
{

namespace zenoh_aliases
{
    using ZenohPub = ::zenoh::Publisher;
    using ZenohSub = ::zenoh::Subscriber<void>;
};

inline zenoh::Config configDirectConnectTo(const std::string& hostname)
{
    zenoh::Config config = zenoh::Config::create_default();
    config.insert_json5("mode", "\"peer\"");
    config.insert_json5("scouting/multicast/enabled", "false");
    config.insert_json5("listen/endpoints", "[\"tcp/0.0.0.0:7447\"]");
    config.insert_json5("connect/endpoints", "[\"tcp/" + hostname + ":7447\"]");
    return config;
}

};  // namespace util
