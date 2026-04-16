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

#include <algorithm>
#include <cmath>

#include "util/geometry.hpp"
#include "util/time_cvt.hpp"
#include "util/mem_helpers.hpp"
#include "robot/core/ros_interface.hpp"
#include "robot/model/geometry.hpp"
#include "robot/control/robot_controller.hpp"


using namespace util;
using namespace util::geom::cvt::ops;
using namespace lance::geom;

using PathMsg = nav_msgs::msg::Path;
using BoolMsg = std_msgs::msg::Bool;
using ColorMsg = std_msgs::msg::ColorRGBA;
using StringMsg = std_msgs::msg::String;
using Float32Msg = std_msgs::msg::Float32;
using TransformStampedMsg = geometry_msgs::msg::TransformStamped;

#define AS_U8(x) static_cast<uint8_t>(x)


namespace lance
{

TelemetryDeserializer::TelemetryDeserializer(
    RclNode& node,
    TfCache& tf_cache,
    MarkerManager& markers) :
    pub_map{node, "", rclcpp::SensorDataQoS{}},
    tf_cache{tf_cache},
    markers{markers},
    tf_broadcaster{node},
    rcl_clock{node.get_clock()},
    sub{node.create_subscription<BytesMsg>(
        lance::TELEMETRY_TOPIC,
        rclcpp::SensorDataQoS{},
        [this](const BytesMsg::ConstSharedPtr& msg) { this->accept(*msg); })},
    mining_marker_id{markers.reserveGroup(1, "robot")},
    offload_marker_id{markers.reserveGroup(1, "robot")},
    auto_mining_paths_marker_id{
        markers.reserveGroup(AUTO_MINING_MAX_PATHS, "debug")},
    auto_mining_grid_marker_id{markers.reserveGroup(3, "debug")}
{
    this->initMarkers();
}


TelemetryDeserializer::GenericPubMap& TelemetryDeserializer::getPubMap()
{
    return this->pub_map;
}


void TelemetryDeserializer::accept(const BytesMsg& msg)
{
    constexpr size_t MAX_TELEMETRY_PACKET_BYTES = (512 * 1024);

    if (msg.data.size() > MAX_TELEMETRY_PACKET_BYTES)
    {
        std::cerr
            << "[TelemetryDeserializer] Dropping oversized telemetry packet: "
            << msg.data.size() << " bytes\n";

        return;
    }

    try
    {
        this->ctrl_chain.clear();
        this->markers.clearOutput();
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
                case AS_U8(TelemetryType::ROBOT_STATE):
                {
                    ok &= this->pubRobotState(ptr, end_ptr);
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
            this->pub_map.publish<StringMsg>(
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

            this->pub_map.publish<StringMsg>(lance::OP_STATUS_TOPIC, ss.str());
        }

        if (!this->markers.getOutputMarkers().markers.empty())
        {
            this->markers.pubOutputMarkers(
                this->pub_map.getPub<MarkerArrayMsg>(
                    lance::ROBOT_MARKERS_TOPIC),
                this->rcl_clock->now(),
                true);
        }
    }
    catch (const std::bad_alloc& e)
    {
        std::cerr
            << "[TelemetryDeserializer] bad_alloc while decoding telemetry: "
            << e.what() << "\n";
    }
    catch (const std::exception& e)
    {
        std::cerr
            << "[TelemetryDeserializer] exception while decoding telemetry: "
            << e.what() << "\n";
    }
    catch (...)
    {
        std::cerr
            << "[TelemetryDeserializer] unknown exception while decoding telemetry\n";
    }
}


void TelemetryDeserializer::initMarkers()
{
    markers.getGroup(this->mining_marker_id)
        .setFrameId(this->tf_cache.arena_frame_id)
        .setType(MarkerMsg::CUBE)
        .setDuration(RclDur{0, 100000000});
    markers.getGroup(this->offload_marker_id)
        .setFrameId(this->tf_cache.arena_frame_id)
        .setType(MarkerMsg::CUBE)
        .setDuration(RclDur{0, 100000000})
        .setColor(0.1f, 0.4f, 0.7f, 0.5f);
    {
        auto g = markers.getGroup(this->auto_mining_paths_marker_id)
                     .setFrameId(this->tf_cache.arena_frame_id)
                     .setType(MarkerMsg::ARROW)
                     .setDuration(RclDur{1, 500000000})
                     .setColor(0.f, 0.f, 0.f, 0.9f);

        for (auto itr = g.beg; itr < g.end; itr++)
        {
            itr->scale.x = 0.02f;
            itr->scale.y = 0.06f;
            itr->scale.z = 0.08f;
            itr->points.resize(2);
            itr->points[0].z = 0.05;
            itr->points[1].z = 0.05;
        }
    }
    {
        auto g = markers.getGroup(this->auto_mining_grid_marker_id)
                     .setFrameId(this->tf_cache.arena_frame_id)
                     .setDuration(RclDur{1, 500000000});

        auto& z = g[0];
        z.type = MarkerMsg::LINE_STRIP;
        z.scale.x = 0.03f;
        z.color.r = 0.2f;
        z.color.g = 0.95f;
        z.color.b = 0.2f;
        z.color.a = 0.9f;
        z.points.resize(5);
        z.points[0].z = 0.01;
        z.points[1].z = 0.01;
        z.points[2].z = 0.01;
        z.points[3].z = 0.01;

        auto& l = g[1];
        l.type = MarkerMsg::LINE_LIST;
        l.scale.x = 0.01f;
        l.color.r = 0.2f;
        l.color.g = 0.7f;
        l.color.b = 1.0f;
        l.color.a = 0.5f;

        auto& p = g[2];
        p.type = MarkerMsg::POINTS;
        p.scale.x = 0.05f;
        p.scale.y = 0.05f;
        p.color.r = 1.0f;
        p.color.g = 1.0f;
        p.color.b = 1.0f;
        p.color.a = 0.65f;
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

    TransformStampedMsg msg;

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

bool TelemetryDeserializer::pubRobotState(BytePtrRef ptr, BytePtr end)
{
    EXIT_IF_INSUFFICIENT_SIZE(sizeof(uint16_t) + (sizeof(float) * 2));

    uint16_t state{0};
    readAndIncrement(ptr, state);

    this->pub_map.publish<BoolMsg>(
        ROBOT_TOPIC("mining_constraints/stall_event"),
        static_cast<bool>(state & MiningConstraints::CONSTRAINT_MOTOR_STALL));
    this->pub_map.publish<BoolMsg>(
        ROBOT_TOPIC("mining_constraints/obstacle"),
        static_cast<bool>(state & MiningConstraints::CONSTRAINT_OBSTACLE));
    this->pub_map.publish<BoolMsg>(
        ROBOT_TOPIC("mining_constraints/hopper_model"),
        static_cast<bool>(state & MiningConstraints::CONSTRAINT_HOPPER_FULL));
    this->pub_map.publish<BoolMsg>(
        ROBOT_TOPIC("mining_constraints/zone_boundary"),
        static_cast<bool>(state & MiningConstraints::CONSTRAINT_ZONE_BOUNDARY));

    this->pub_map.publish<BoolMsg>(
        COLLECTION_STATE_TOPIC("is_full_volume"),
        static_cast<bool>(state & (1 << 8)));
    this->pub_map.publish<BoolMsg>(
        COLLECTION_STATE_TOPIC("is_full_occ"),
        static_cast<bool>(state & (1 << 9)));

    float val;
    readAndIncrement(ptr, val);
    this->offloaded_volume += std::max(this->last_hopper_volume - val, 0.f);
    this->last_hopper_volume = val;
    this->pub_map.publish<Float32Msg>(COLLECTION_STATE_TOPIC("volume"), val);
    this->pub_map.publish<Float32Msg>(
        COLLECTION_STATE_TOPIC("offloaded_volume"),
        this->offloaded_volume);

    readAndIncrement(ptr, val);
    this->pub_map.publish<Float32Msg>(
        COLLECTION_STATE_TOPIC("belt_usage"),
        val);

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
        "Assisted Trav",
        "Planned Mining : Traversal",
        "Planned Mining : Mining",
        "Planned Offload : Traversal",
        "Planned Offload : Offload"};

    using Op = TeleopController::Operation;

    uint8_t stage_id;
    readAndIncrement(ptr, stage_id);

    switch (stage_id)
    {
        case AS_U8(Op::ASSISTED_MINING):
        case AS_U8(Op::ASSISTED_OFFLOAD):
        case AS_U8(Op::PLANNED_TRAVERSAL):
        case AS_U8(Op::PLANNED_MINING_T):
        case AS_U8(Op::PLANNED_MINING_E):
        case AS_U8(Op::PLANNED_OFFLOAD_T):
        case AS_U8(Op::PLANNED_OFFLOAD_E):
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

    using Stage = AutoMiningController::Stage;

    constexpr char const* STAGE_TAGS[] =
        {"Initializing", "Planning", "Traversing", "Mining", "Finished"};

    uint8_t state_bits;
    readAndIncrement(ptr, state_bits);

    // Highest bit indicates serialized evaluated mining paths.
    if (state_bits & AUTO_MINING_STATE_PATHS_BIT)
    {
        EXIT_IF_INSUFFICIENT_SIZE(sizeof(uint32_t))

        size_t n_paths;
        readAsAndIncrement<uint32_t>(ptr, n_paths);

        constexpr size_t PATH_SIZE = ((sizeof(float) * 4) + sizeof(uint8_t));
        EXIT_IF_INSUFFICIENT_SIZE(PATH_SIZE * n_paths)

        BytePtr ptr_after_paths = ptr + (PATH_SIZE * n_paths);
        auto m = this->markers.getGroup(this->auto_mining_paths_marker_id);
        for (size_t i = 0; i < n_paths && i < AUTO_MINING_MAX_PATHS; i++)
        {
            float sx, sy, ex, ey;
            readAndIncrement(ptr, sx);
            readAndIncrement(ptr, sy);
            readAndIncrement(ptr, ex);
            readAndIncrement(ptr, ey);
            uint8_t dir_id;
            readAndIncrement(ptr, dir_id);

            const float dx = ex - sx;
            const float dy = ey - sy;
            if ((dx * dx + dy * dy) < 1e-6f)
            {
                continue;
            }

            enum class PathDir
            {
                UP,
                DOWN,
                LEFT,
                RIGHT
            };

            PathDir path_dir = PathDir::UP;
            switch (dir_id)
            {
                case AS_U8(MiningDirection::UP):
                {
                    path_dir = PathDir::UP;
                    break;
                }
                case AS_U8(MiningDirection::DOWN):
                {
                    path_dir = PathDir::DOWN;
                    break;
                }
                case AS_U8(MiningDirection::LEFT):
                {
                    path_dir = PathDir::LEFT;
                    break;
                }
                case AS_U8(MiningDirection::RIGHT):
                {
                    path_dir = PathDir::RIGHT;
                    break;
                }
                default:
                {
                    // Fallback for mixed-version payloads.
                    path_dir =
                        (std::abs(dx) >= std::abs(dy))
                            ? (dx >= 0.f ? PathDir::RIGHT : PathDir::LEFT)
                            : (dy >= 0.f ? PathDir::UP : PathDir::DOWN);
                }
            }

            constexpr float DIR_RENDER_OFFSET_M = 0.07f;
            float lateral_off_x = 0.f;
            float lateral_off_y = 0.f;
            switch (path_dir)
            {
                case PathDir::UP:
                {
                    lateral_off_x = -DIR_RENDER_OFFSET_M;
                    break;
                }
                case PathDir::DOWN:
                {
                    lateral_off_x = DIR_RENDER_OFFSET_M;
                    break;
                }
                case PathDir::LEFT:
                {
                    lateral_off_y = DIR_RENDER_OFFSET_M;
                    break;
                }
                case PathDir::RIGHT:
                {
                    lateral_off_y = -DIR_RENDER_OFFSET_M;
                    break;
                }
            }

            MarkerMsg& marker = m[i];

            switch (path_dir)
            {
                case PathDir::UP:
                {
                    marker.color.set__r(0.95f).set__g(0.95f).set__b(0.2f);
                    break;
                }
                case PathDir::DOWN:
                {
                    marker.color.set__r(0.95f).set__g(0.25f).set__b(0.25f);
                    break;
                }
                case PathDir::LEFT:
                {
                    marker.color.set__r(0.3f).set__g(0.55f).set__b(1.0f);
                    break;
                }
                case PathDir::RIGHT:
                {
                    marker.color.set__r(1.0f).set__g(0.35f).set__b(0.9f);
                    break;
                }
            }

            marker.points[0].x = sx + lateral_off_x;
            marker.points[0].y = sy + lateral_off_y;
            marker.points[1].x = ex + lateral_off_x;
            marker.points[1].y = ey + lateral_off_y;
        }

        ptr = ptr_after_paths;

        this->markers.addSubGroupToOutput(
            this->auto_mining_paths_marker_id,
            n_paths);
    }

    if (state_bits & AUTO_MINING_STATE_GRID_BIT)
    {
        EXIT_IF_INSUFFICIENT_SIZE((sizeof(float) * 5) + (sizeof(uint32_t) * 2))

        float min_x, min_y, max_x, max_y, cell_size;
        size_t x_divisions, y_divisions;
        readAndIncrement(ptr, min_x);
        readAndIncrement(ptr, min_y);
        readAndIncrement(ptr, max_x);
        readAndIncrement(ptr, max_y);
        readAndIncrement(ptr, cell_size);
        readAsAndIncrement<uint32_t>(ptr, x_divisions);
        readAsAndIncrement<uint32_t>(ptr, y_divisions);

        const size_t render_x_divisions =
            std::min(x_divisions, AUTO_MINING_MAX_GRID_DIVS);
        const size_t render_y_divisions =
            std::min(y_divisions, AUTO_MINING_MAX_GRID_DIVS);

        auto m = this->markers.getGroup(this->auto_mining_grid_marker_id);

        MarkerMsg& zone = m[0];
        zone.points[0].x = min_x;
        zone.points[0].y = min_y;
        zone.points[1].x = max_x;
        zone.points[1].y = min_y;
        zone.points[2].x = max_x;
        zone.points[2].y = max_y;
        zone.points[3].x = min_x;
        zone.points[3].y = max_y;
        zone.points[4] = zone.points[0];

        if (cell_size > 0.f)
        {
            const float grid_max_x = std::min(
                max_x,
                min_x + (static_cast<float>(render_x_divisions) * cell_size));
            const float grid_max_y = std::min(
                max_y,
                min_y + (static_cast<float>(render_y_divisions) * cell_size));

            MarkerMsg& grid_lines = m[1];
            grid_lines.points.clear();
            grid_lines.points.reserve(
                (render_x_divisions + render_y_divisions + 2) * 2);

            for (size_t ix = 0; ix <= render_x_divisions; ix++)
            {
                const float x = min_x + (static_cast<float>(ix) * cell_size);
                auto& p0 = grid_lines.points.emplace_back();
                auto& p1 = grid_lines.points.emplace_back();
                p0.x = x;
                p0.y = min_y;
                p0.z = 0.01;
                p1.x = x;
                p1.y = grid_max_y;
                p1.z = 0.01;
            }
            for (size_t iy = 0; iy <= render_y_divisions; iy++)
            {
                const float y = min_y + (static_cast<float>(iy) * cell_size);
                auto& p0 = grid_lines.points.emplace_back();
                auto& p1 = grid_lines.points.emplace_back();
                p0.x = min_x;
                p0.y = y;
                p0.z = 0.01;
                p1.x = grid_max_x;
                p1.y = y;
                p1.z = 0.01;
            }

            MarkerMsg& grid_points = m[2];
            grid_points.points.clear();
            grid_points.points.reserve(render_x_divisions * render_y_divisions);

            for (size_t ix = 0; ix < render_x_divisions; ix++)
            {
                for (size_t iy = 0; iy < render_y_divisions; iy++)
                {
                    auto& p = grid_points.points.emplace_back();
                    p.x = min_x + ((static_cast<float>(ix) + 0.5f) * cell_size);
                    p.y = min_y + ((static_cast<float>(iy) + 0.5f) * cell_size);
                    p.z = 0.015;
                }
            }

            this->markers.addGroupToOutput(this->auto_mining_grid_marker_id);
        }
        else
        {
            this->markers.addSubGroupToOutput(
                this->auto_mining_grid_marker_id,
                1);
        }
    }


    const uint8_t stage_id = (state_bits & AUTO_MINING_STATE_STAGE_MASK);
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
    EXIT_IF_INSUFFICIENT_SIZE(sizeof(uint8_t) * 2 + sizeof(float))

    constexpr char const* STAGE_TAGS[] =
        {"Initializing", "Lowering", "Excavating", "Raising", "Finished"};

    uint8_t stage_id;
    readAndIncrement(ptr, stage_id);

    if (stage_id < (sizeof(STAGE_TAGS) / sizeof(*STAGE_TAGS)))
    {
        this->ctrl_chain.push_back(STAGE_TAGS[stage_id]);
    }

    uint8_t constraint;
    float remaining_trav_dist;
    readAndIncrement(ptr, constraint);
    readAndIncrement(ptr, remaining_trav_dist);
    this->addMiningMarker(constraint, remaining_trav_dist);

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
            PathMsg msg;
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


void TelemetryDeserializer::addMiningMarker(uint8_t constraint, float dist)
{
    if (!this->tf_cache.hasTf(ROBOT_TO_ARENA_TF) || !constraint)
    {
        return;
    }

    MarkerMsg& m = this->markers.getGroup(this->mining_marker_id)[0];

    switch (constraint)
    {
        case MiningConstraints::CONSTRAINT_MOTOR_STALL:
        {
            m.color.set__r(0.9f).set__g(0.8f).set__b(0.2f).set__a(0.5f);
            break;
        }
        case MiningConstraints::CONSTRAINT_OBSTACLE:
        {
            m.color.set__r(1.f).set__g(0.f).set__b(0.f).set__a(0.5f);
            break;
        }
        case MiningConstraints::CONSTRAINT_HOPPER_FULL:
        {
            m.color.set__r(0.6f).set__g(0.2f).set__b(0.9f).set__a(0.5f);
            break;
        }
        case MiningConstraints::CONSTRAINT_ZONE_BOUNDARY:
        {
            m.color.set__r(0.8f).set__g(0.5f).set__b(0.2f).set__a(0.5f);
            break;
        }
        default:
        {
        }
    }

    m.scale.x = dist;
    m.scale.y = lance::geom::PRIMARY_COLLISION_ZONE_WIDTH;
    m.scale.z = lance::geom::PRIMARY_COLLISION_ZONE_HEIGHT;

    const PoseTf3f* p = this->tf_cache.getTf(ROBOT_TO_ARENA_TF);
    const Quatf flattened_q = lance::geom::flattenToYaw(p->pose.quat);
    const Vec3f pos_off =
        flattened_q * Vec3f{
                          lance::geom::FOOTPRINT_X_MAX_<float> + (dist / 2.f),
                          0.f,
                          lance::geom::PRIMARY_COLLISION_ZONE_Z_<float>};

    m.pose.position << Vec3f{p->pose.vec + pos_off};
    m.pose.orientation << flattened_q;

    this->markers.addGroupToOutput(this->mining_marker_id);
}

void TelemetryDeserializer::addOffloadMarker(float dist)
{
    MarkerMsg& m = this->markers.getGroup(this->offload_marker_id)[0];

    m.scale.x = lance::geom::OFFLOAD_FOOTPRINT_LENGTH;
    m.scale.y = lance::geom::OFFLOAD_FOOTPRINT_WIDTH;
    m.scale.z = 0.5;

    const PoseTf3f* p = this->tf_cache.getTf(ROBOT_TO_ARENA_TF);
    const Quatf flattened_q = lance::geom::flattenToYaw(p->pose.quat);
    const Vec3f pos_off =
        flattened_q * Vec3f{
                          -dist + lance::geom::OFFLOAD_FOOTPRINT_OFFSET_<float>,
                          0.f,
                          0.25f};

    m.pose.position << Vec3f{p->pose.vec + pos_off};
    m.pose.orientation << flattened_q;

    this->markers.addGroupToOutput(this->offload_marker_id);
}

};  // namespace lance
