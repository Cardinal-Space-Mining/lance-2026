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

#include "deserializer.hpp"

#include "util/geometry.hpp"
#include "util/time_cvt.hpp"
#include "util/mem_helpers.hpp"
#include "robot/core/ros_interface.hpp"
#include "robot/model/geometry.hpp"
#include "robot/control/robot_controller.hpp"


using namespace util;
using namespace util::geom::cvt::ops;
using namespace lance::geom;

#define AS_U8(x) static_cast<uint8_t>(x)


namespace lance
{

TelemetryDeserializer::TelemetryDeserializer(RclNode& node, TfCache& tf_cache) :
    pub_map{node, "", rclcpp::SensorDataQoS{}},
    tf_cache{tf_cache},
    tf_broadcaster{node},
    rcl_clock{node.get_clock()},
    sub{node.create_subscription<BytesMsg>(
        lance::TELEMETRY_TOPIC,
        rclcpp::SensorDataQoS{},
        [this](const BytesMsg::ConstSharedPtr& msg) { this->accept(*msg); })}
{
}


TelemetryDeserializer::GenericPubMap& TelemetryDeserializer::getPubMap()
{
    return this->pub_map;
}


void TelemetryDeserializer::accept(const BytesMsg& msg)
{
    this->ctrl_chain.clear();
    this->markers.markers.clear();
    this->tf_cache.refresh();

    const Byte* ptr = msg.data.data();
    const Byte* const end_ptr = msg.data.end().base();
    bool ok = true;
    while (ok && ptr < end_ptr)
    {
        uint8_t id = AS_U8(TelemetryType::INVALID_ID);
        readAndIncrement(ptr, id);

        switch (id)
        {
            case AS_U8(TelemetryType::ARENA_TF):
            {
                ok &= this->pubArenaTf(ptr, end_ptr);
                break;
            }
            case AS_U8(TelemetryType::COLLECTION_STATE):
            {
                ok &= this->pubCollectionState(ptr, end_ptr);
                break;
            }
            case AS_U8(TelemetryType::CTRL_STATE):
            {
                ok &= this->pubControlState(ptr, end_ptr);
                break;
            }
            case AS_U8(TelemetryType::INVALID_ID):
            default:
            {
                ok = false;
            }
        }
    }

    if (this->ctrl_chain.empty())
    {
        this->pub_map.publish<std_msgs::msg::String>(
            lance::OP_STATUS_TOPIC,
            "Disabled");
    }
    else
    {
        std::ostringstream ss;
        ss << this->ctrl_chain.front();
        for (size_t i = 1; i < this->ctrl_chain.size(); i++)
        {
            ss << " : " << this->ctrl_chain[i];
        }

        this->pub_map.publish<std_msgs::msg::String>(
            lance::OP_STATUS_TOPIC,
            ss.str());
    }

    if (!this->markers.markers.empty())
    {
        this->pub_map.publish(lance::ROBOT_MARKERS_TOPIC, this->markers);
    }
}


#define EXIT_IF_INSUFFICIENT_SIZE(x) \
    if (ptr + (x) > end)             \
    {                                \
        return false;                \
    }

bool TelemetryDeserializer::pubArenaTf(BytePtrRef ptr, BytePtr end)
{
    EXIT_IF_INSUFFICIENT_SIZE(sizeof(float) * 7);

    geometry_msgs::msg::TransformStamped msg;

    readAsAndIncrement<float>(ptr, msg.transform.translation.x);
    readAsAndIncrement<float>(ptr, msg.transform.translation.y);
    readAsAndIncrement<float>(ptr, msg.transform.translation.z);
    readAsAndIncrement<float>(ptr, msg.transform.rotation.w);
    readAsAndIncrement<float>(ptr, msg.transform.rotation.x);
    readAsAndIncrement<float>(ptr, msg.transform.rotation.y);
    readAsAndIncrement<float>(ptr, msg.transform.rotation.z);

    msg.child_frame_id = this->tf_cache.odom_frame_id;
    msg.header.frame_id = this->tf_cache.arena_frame_id;

    // NOTE: this is a really stupid hack to deal with the way that perception
    // republishes old alignment transforms (only a problem when running a sim)
    if (this->tf_cache.hasTf(ARENA_TO_ODOM_TF))
    {
        msg.header.stamp = toTimeMsg(this->tf_cache.getStamp(ARENA_TO_ODOM_TF));
    }
    else
    {
        msg.header.stamp = this->rcl_clock->now();
    }

    this->tf_broadcaster.sendTransform(msg);

    return true;
}

bool TelemetryDeserializer::pubCollectionState(BytePtrRef ptr, BytePtr end)
{
    EXIT_IF_INSUFFICIENT_SIZE(sizeof(uint8_t) + (sizeof(float) * 7))

    constexpr char const* FLOAT_TOPICS[] = {
        COLLECTION_STATE_TOPIC("volume"),
        COLLECTION_STATE_TOPIC("mining_target"),
        COLLECTION_STATE_TOPIC("offload_target"),
        COLLECTION_STATE_TOPIC("belt_pos_m"),
        COLLECTION_STATE_TOPIC("high_pos_m"),
        COLLECTION_STATE_TOPIC("low_pos_m"),
        COLLECTION_STATE_TOPIC("belt_usage_m")};

    uint8_t bool_fields{0};
    readAndIncrement(ptr, bool_fields);

    this->pub_map.publish<std_msgs::msg::Bool>(
        COLLECTION_STATE_TOPIC("is_full_volume"),
        static_cast<bool>(bool_fields & 0x1));
    this->pub_map.publish<std_msgs::msg::Bool>(
        COLLECTION_STATE_TOPIC("is_full_occ"),
        static_cast<bool>(bool_fields & 0x2));

    for (const char* TOPIC : FLOAT_TOPICS)
    {
        float val{0.f};
        readAndIncrement(ptr, val);

        this->pub_map.publish<std_msgs::msg::Float32>(TOPIC, val);
    }

    return true;
}

bool TelemetryDeserializer::pubControlState(BytePtrRef ptr, BytePtr end)
{
    EXIT_IF_INSUFFICIENT_SIZE(1)

    uint8_t ctrl_id = AS_U8(ControllerType::INVALID_ID);
    readAndIncrement(ptr, ctrl_id);

    switch (ctrl_id)
    {
        case AS_U8(ControllerType::TELEOP):
        {
            this->ctrl_chain.push_back("Teleop");
            return this->pubTeleopController(ptr, end);
        }
        case AS_U8(ControllerType::AUTO):
        {
            this->ctrl_chain.push_back("Auto");
            return this->pubAutoController(ptr, end);
        }
        default:
        {
            return false;
        }
    }
}

bool TelemetryDeserializer::pubDerivedController(BytePtrRef ptr, BytePtr end)
{
    EXIT_IF_INSUFFICIENT_SIZE(1)

    uint8_t ctrl_id = AS_U8(ControllerType::INVALID_ID);
    readAndIncrement(ptr, ctrl_id);

    switch (ctrl_id)
    {
        case AS_U8(ControllerType::TELEOP):
        {
            return this->pubTeleopController(ptr, end);
        }
        case AS_U8(ControllerType::AUTO):
        {
            return this->pubAutoController(ptr, end);
        }
        case AS_U8(ControllerType::AUTO_MINING):
        {
            return this->pubAutoMiningController(ptr, end);
        }
        case AS_U8(ControllerType::AUTO_OFFLOAD):
        {
            return this->pubAutoOffloadController(ptr, end);
        }
        case AS_U8(ControllerType::MINING):
        {
            return this->pubMiningController(ptr, end);
        }
        case AS_U8(ControllerType::OFFLOAD):
        {
            return this->pubOffloadController(ptr, end);
        }
        case AS_U8(ControllerType::LOCALIZATION):
        {
            return this->pubLocController(ptr, end);
        }
        case AS_U8(ControllerType::TRAVERSAL):
        {
            return this->pubTravController(ptr, end);
        }
        default:
        {
        }
    }

    return false;
}

bool TelemetryDeserializer::pubTeleopController(BytePtrRef ptr, BytePtr end)
{
    EXIT_IF_INSUFFICIENT_SIZE(1)

    constexpr char const* OP_TAGS[] = {
        "Manual",
        "Assisted Mining",
        "Assisted Offload",
        "Preset Mining",
        "Preset Offload",
        "Assisted Trav"};

    using Op = TeleopController::Operation;

    uint8_t stage_id;
    readAndIncrement(ptr, stage_id);

    switch (stage_id)
    {
        case AS_U8(Op::ASSISTED_MINING):
        case AS_U8(Op::PRESET_MINING):
        case AS_U8(Op::ASSISTED_OFFLOAD):
        case AS_U8(Op::PRESET_OFFLOAD):
        case AS_U8(Op::AUTO_TRAVERSAL):
        {
            this->ctrl_chain.push_back(OP_TAGS[stage_id]);

            // All these operations chain a controller which should be read
            return this->pubDerivedController(ptr, end);
        }
        case AS_U8(Op::MANUAL):
        {
            this->ctrl_chain.push_back(OP_TAGS[stage_id]);

            // Manual mode doesn't add any additional data, directly return
            return true;
        }
        default:
        {
        }
    }

    return false;
}

bool TelemetryDeserializer::pubAutoController(BytePtrRef ptr, BytePtr end)
{
    EXIT_IF_INSUFFICIENT_SIZE(1)

    constexpr char const* STAGE_TAGS[] = {
        "Localize",
        "Trav To Mining",
        "Auto Mining",
        "Trav To Offload",
        "Auto Offload"};

    using Stage = AutoController::Stage;

    uint8_t stage_id;
    readAndIncrement(ptr, stage_id);

    switch (stage_id)
    {
        case AS_U8(Stage::LOCALIZATION):
        case AS_U8(Stage::TRAVERSE_TO_MINING):
        case AS_U8(Stage::TRAVERSE_TO_OFFLOAD):
        case AS_U8(Stage::MINING):
        case AS_U8(Stage::OFFLOAD):
        {
            this->ctrl_chain.push_back(STAGE_TAGS[stage_id]);

            // All stages chain another controller
            return this->pubDerivedController(ptr, end);
        }
        default:
        {
        }
    }

    return false;
}

bool TelemetryDeserializer::pubAutoMiningController(BytePtrRef ptr, BytePtr end)
{
    EXIT_IF_INSUFFICIENT_SIZE(1)

    constexpr char const* STAGE_TAGS[] =
        {"Initializing", "Planning", "Traversing", "Mining", "Finished"};

    using Stage = AutoMiningController::Stage;

    uint8_t stage_id;
    readAndIncrement(ptr, stage_id);

    // TODO: extract planned swath

    switch (stage_id)
    {
        case AS_U8(Stage::TRAVERSING):
        case AS_U8(Stage::MINING):
        {
            this->ctrl_chain.push_back(STAGE_TAGS[stage_id]);

            // These options chain another controller
            return this->pubDerivedController(ptr, end);
        }
        case AS_U8(Stage::INITIALIZATION):
        case AS_U8(Stage::PLANNING):
        case AS_U8(Stage::FINISHED):
        {
            this->ctrl_chain.push_back(STAGE_TAGS[stage_id]);

            // These stages do not
            return true;
        }
        default:
        {
        }
    }

    return false;
}

bool TelemetryDeserializer::pubAutoOffloadController(
    BytePtrRef ptr,
    BytePtr end)
{
    EXIT_IF_INSUFFICIENT_SIZE(1)

    constexpr char const* STAGE_TAGS[] =
        {"Initializing", "Planning", "Traversing", "Offloading", "Finished"};

    using Stage = AutoOffloadController::Stage;

    uint8_t stage_id;
    readAndIncrement(ptr, stage_id);

    // TODO: extract planned zone

    switch (stage_id)
    {
        case AS_U8(Stage::TRAVERSING):
        case AS_U8(Stage::OFFLOADING):
        {
            this->ctrl_chain.push_back(STAGE_TAGS[stage_id]);

            // Read the next chained controller
            return this->pubDerivedController(ptr, end);
        }
        case AS_U8(Stage::INITIALIZATION):
        case AS_U8(Stage::PLANNING):
        case AS_U8(Stage::FINISHED):
        {
            this->ctrl_chain.push_back(STAGE_TAGS[stage_id]);

            // Nothing more to read
            return true;
        }
        default:
        {
        }
    }

    return false;
}

bool TelemetryDeserializer::pubMiningController(BytePtrRef ptr, BytePtr end)
{
    EXIT_IF_INSUFFICIENT_SIZE(sizeof(uint8_t) + sizeof(float))

    constexpr char const* STAGE_TAGS[] =
        {"Initializing", "Lowering", "Excavating", "Raising", "Finished"};

    uint8_t stage_id;
    readAndIncrement(ptr, stage_id);

    if (stage_id < (sizeof(STAGE_TAGS) / sizeof(*STAGE_TAGS)))
    {
        this->ctrl_chain.push_back(STAGE_TAGS[stage_id]);
    }

    float remaining_trav_dist;
    readAndIncrement(ptr, remaining_trav_dist);
    this->addMiningMarker(remaining_trav_dist);

    return true;
}

bool TelemetryDeserializer::pubOffloadController(BytePtrRef ptr, BytePtr end)
{
    EXIT_IF_INSUFFICIENT_SIZE(sizeof(uint8_t) + sizeof(float))

    constexpr char const* STAGE_TAGS[] = {
        "Initializing",
        "Backing Up",
        "Raising",
        "Offloading",
        "Lowering",
        "Finished"};

    uint8_t stage_id;
    readAndIncrement(ptr, stage_id);

    if (stage_id < (sizeof(STAGE_TAGS) / sizeof(*STAGE_TAGS)))
    {
        this->ctrl_chain.push_back(STAGE_TAGS[stage_id]);
    }

    float remaining_trav_dist;
    readAndIncrement(ptr, remaining_trav_dist);
    this->addOffloadMarker(remaining_trav_dist);

    return true;
}

bool TelemetryDeserializer::pubLocController(BytePtrRef ptr, BytePtr end)
{
    EXIT_IF_INSUFFICIENT_SIZE(1)

    constexpr char const* STAGE_TAGS[] = {
        "Initializing",
        "Searching",
        "Align Heading",
        "Adjust Range",
        "Finished"};

    uint8_t stage_id;
    readAndIncrement(ptr, stage_id);

    if (stage_id < (sizeof(STAGE_TAGS) / sizeof(*STAGE_TAGS)))
    {
        this->ctrl_chain.push_back(STAGE_TAGS[stage_id]);
    }

    return true;
}

bool TelemetryDeserializer::pubTravController(BytePtrRef ptr, BytePtr end)
{
    EXIT_IF_INSUFFICIENT_SIZE(1)

    constexpr char const* STAGE_TAGS[] = {
        "Initializing",
        "Tracking Path",
        "Reorienting",
        "Finished"};

    uint8_t val;
    readAndIncrement(ptr, val);

    const uint8_t stage_id = (val & 0x7f);
    if (stage_id < (sizeof(STAGE_TAGS) / sizeof(*STAGE_TAGS)))
    {
        this->ctrl_chain.push_back(STAGE_TAGS[stage_id]);
    }

    // highest bit gets set when path data is written next
    if (val & 0x80)
    {
        EXIT_IF_INSUFFICIENT_SIZE(sizeof(uint32_t))

        uint32_t n_pts;
        readAndIncrement(ptr, n_pts);

        EXIT_IF_INSUFFICIENT_SIZE(sizeof(float) * 3 * n_pts)

        if (n_pts)
        {
            nav_msgs::msg::Path msg;
            msg.header.frame_id = this->tf_cache.odom_frame_id;
            msg.header.stamp = this->rcl_clock->now();

            msg.poses.resize(n_pts);
            for (auto& p : msg.poses)
            {
                p.header = msg.header;
                readAsAndIncrement<float>(ptr, p.pose.position.x);
                readAsAndIncrement<float>(ptr, p.pose.position.y);
                readAsAndIncrement<float>(ptr, p.pose.position.z);
            }

            this->pub_map.publish(lance::TRAVERSAL_PATH_TOPIC, msg);
        }
    }

    return true;
}


void TelemetryDeserializer::addMiningMarker(float dist)
{
    if (!this->tf_cache.hasTf(ROBOT_TO_ARENA_TF))
    {
        return;
    }

    MarkerMsg& marker = this->markers.markers.emplace_back();

    marker.header.frame_id = this->tf_cache.arena_frame_id;
    marker.header.stamp = this->rcl_clock->now();

    marker.ns = "robot";
    marker.id = 1;
    marker.type = MarkerMsg::CUBE;
    marker.action = MarkerMsg::ADD;
    marker.lifetime = rclcpp::Duration(0, 100000000);

    marker.color.r = 1.f;
    marker.color.g = 0.f;
    marker.color.b = 0.f;
    marker.color.a = 0.5f;

    marker.scale.x = dist;
    marker.scale.y = lance::geom::PRIMARY_COLLISION_ZONE_WIDTH;
    marker.scale.z = lance::geom::PRIMARY_COLLISION_ZONE_HEIGHT;

    const PoseTf3f* p = this->tf_cache.getTf(ROBOT_TO_ARENA_TF);
    const Quatf flattened_q = lance::geom::flattenToYaw(p->pose.quat);
    const Vec3f pos_off =
        flattened_q * Vec3f{
                          lance::geom::FOOTPRINT_X_MAX_<float> + (dist / 2.f),
                          0.f,
                          lance::geom::PRIMARY_COLLISION_ZONE_Z_<float>};

    marker.pose.position.x = p->pose.vec.x() + pos_off.x();
    marker.pose.position.y = p->pose.vec.y() + pos_off.y();
    marker.pose.position.z = p->pose.vec.z() + pos_off.z();
    marker.pose.orientation.w = flattened_q.w();
    marker.pose.orientation.x = flattened_q.x();
    marker.pose.orientation.y = flattened_q.y();
    marker.pose.orientation.z = flattened_q.z();
}

void TelemetryDeserializer::addOffloadMarker(float dist)
{
    (void)dist;
}

};  // namespace lance
