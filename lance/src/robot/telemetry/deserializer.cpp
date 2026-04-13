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
    constexpr size_t MAX_TELEMETRY_PACKET_BYTES = (512 * 1024);

    try
    {
        if (msg.data.size() > MAX_TELEMETRY_PACKET_BYTES)
        {
            std::cerr << "[TelemetryDeserializer] Dropping oversized telemetry packet: "
                      << msg.data.size() << " bytes\n";
            return;
        }

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
            this->pub_map.publish<StringMsg>(lance::OP_STATUS_TOPIC, "Disabled");
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

        if (!this->markers.markers.empty())
        {
            this->pub_map.publish(lance::ROBOT_MARKERS_TOPIC, this->markers);
        }
    }
    catch (const std::bad_alloc& e)
    {
        this->ctrl_chain.clear();
        this->markers.markers.clear();
        std::cerr << "[TelemetryDeserializer] bad_alloc while decoding telemetry: "
                  << e.what() << "\n";
    }
    catch (const std::exception& e)
    {
        this->ctrl_chain.clear();
        this->markers.markers.clear();
        std::cerr << "[TelemetryDeserializer] exception while decoding telemetry: "
                  << e.what() << "\n";
    }
    catch (...)
    {
        this->ctrl_chain.clear();
        this->markers.markers.clear();
        std::cerr << "[TelemetryDeserializer] unknown exception while decoding telemetry\n";
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

    constexpr char const* FLOAT_TOPICS[] = {
        COLLECTION_STATE_TOPIC("volume"),
        COLLECTION_STATE_TOPIC("belt_usage")};
    for (const char* TOPIC : FLOAT_TOPICS)
    {
        float val{0.f};
        readAndIncrement(ptr, val);

        this->pub_map.publish<Float32Msg>(TOPIC, val);
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
        "Preset Offload",
        "Assisted Trav"};

    using Op = TeleopController::Operation;

    uint8_t stage_id;
    readAndIncrement(ptr, stage_id);

    switch (stage_id)
    {
        case AS_U8(Op::ASSISTED_MINING):
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
    constexpr uint8_t EVAL_PATHS_FLAG = 0x80;
    constexpr uint8_t GRID_INFO_FLAG = 0x40;
    constexpr uint32_t MAX_AUTO_MINING_VIS_PATHS = 256;
    constexpr uint32_t MAX_AUTO_MINING_GRID_DIVS = 256;
    constexpr size_t PATH_ENTRY_SIZE =
        ((sizeof(float) * 4) + sizeof(uint8_t));
    constexpr float DIR_RENDER_OFFSET_M = 0.07f;
    const auto path_marker_lifetime = rclcpp::Duration(1, 0);
    const auto grid_marker_lifetime = rclcpp::Duration(2, 0);

    using Stage = AutoMiningController::Stage;

    uint8_t val;
    readAndIncrement(ptr, val);

    const uint8_t stage_id = (val & 0x3f);

    // Highest bit indicates serialized evaluated mining paths.
    if (val & EVAL_PATHS_FLAG)
    {
        EXIT_IF_INSUFFICIENT_SIZE(sizeof(uint32_t))

        uint32_t raw_n_paths;
        readAndIncrement(ptr, raw_n_paths);

        const size_t bytes_left = static_cast<size_t>(end - ptr);
        if (PATH_ENTRY_SIZE == 0 || (bytes_left / PATH_ENTRY_SIZE) < raw_n_paths)
        {
            return false;
        }

        const uint32_t n_paths =
            std::min(raw_n_paths, MAX_AUTO_MINING_VIS_PATHS);

        const auto stamp = this->rcl_clock->now();
        for (uint32_t i = 0; i < raw_n_paths; i++)
        {
            float sx, sy, ex, ey;
            readAndIncrement(ptr, sx);
            readAndIncrement(ptr, sy);
            readAndIncrement(ptr, ex);
            readAndIncrement(ptr, ey);
            uint8_t dir_id;
            readAndIncrement(ptr, dir_id);

            if (i >= n_paths)
            {
                continue;
            }

            const float dx = ex - sx;
            const float dy = ey - sy;
            const float mag = std::sqrt((dx * dx) + (dy * dy));

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

            const float render_sx = sx + lateral_off_x;
            const float render_sy = sy + lateral_off_y;
            const float render_ex = ex + lateral_off_x;
            const float render_ey = ey + lateral_off_y;

            MarkerMsg& marker = this->markers.markers.emplace_back();

            marker.header.frame_id = this->tf_cache.arena_frame_id;
            marker.header.stamp = stamp;
            marker.ns = "auto_mining_eval_paths";
            marker.id = static_cast<int32_t>(1000 + i);
            marker.type = MarkerMsg::LINE_STRIP;
            marker.action = MarkerMsg::ADD;
            marker.lifetime = path_marker_lifetime;

            switch (path_dir)
            {
                case PathDir::UP:
                {
                    marker.color.r = 0.95f;
                    marker.color.g = 0.95f;
                    marker.color.b = 0.2f;
                    break;
                }
                case PathDir::DOWN:
                {
                    marker.color.r = 0.95f;
                    marker.color.g = 0.25f;
                    marker.color.b = 0.25f;
                    break;
                }
                case PathDir::LEFT:
                {
                    marker.color.r = 0.3f;
                    marker.color.g = 0.55f;
                    marker.color.b = 1.0f;
                    break;
                }
                case PathDir::RIGHT:
                {
                    marker.color.r = 1.0f;
                    marker.color.g = 0.35f;
                    marker.color.b = 0.9f;
                    break;
                }
            }
            marker.color.a = 0.9f;

            marker.scale.x = 0.035f;

            marker.points.resize(2);
            marker.points[0].x = render_sx;
            marker.points[0].y = render_sy;
            marker.points[0].z = 0.02;
            marker.points[1].x = render_ex;
            marker.points[1].y = render_ey;
            marker.points[1].z = 0.02;
            if (mag > 0.0001f)
            {
                const float arrow_len = std::min(0.6f, mag);

                MarkerMsg& arrow = this->markers.markers.emplace_back();
                arrow.header = marker.header;
                arrow.ns = "auto_mining_eval_dirs";
                arrow.id = static_cast<int32_t>(2000 + i);
                arrow.type = MarkerMsg::ARROW;
                arrow.action = MarkerMsg::ADD;
                arrow.lifetime = marker.lifetime;

                arrow.color = marker.color;
                arrow.color.a = 1.0f;

                arrow.scale.x = 0.02f;
                arrow.scale.y = 0.06f;
                arrow.scale.z = 0.08f;

                arrow.points.resize(2);
                arrow.points[0] = marker.points[0];
                arrow.points[0].z = 0.05;
                arrow.points[1].x = render_sx + (dx / mag) * arrow_len;
                arrow.points[1].y = render_sy + (dy / mag) * arrow_len;
                arrow.points[1].z = 0.05;
            }
        }
    }

    if (val & GRID_INFO_FLAG)
    {
        EXIT_IF_INSUFFICIENT_SIZE((sizeof(float) * 5) + (sizeof(uint32_t) * 2))

        float min_x, min_y, max_x, max_y, cell_size;
        uint32_t x_divisions, y_divisions;
        readAndIncrement(ptr, min_x);
        readAndIncrement(ptr, min_y);
        readAndIncrement(ptr, max_x);
        readAndIncrement(ptr, max_y);
        readAndIncrement(ptr, cell_size);
        readAndIncrement(ptr, x_divisions);
        readAndIncrement(ptr, y_divisions);

        const uint32_t render_x_divisions =
            std::min(x_divisions, MAX_AUTO_MINING_GRID_DIVS);
        const uint32_t render_y_divisions =
            std::min(y_divisions, MAX_AUTO_MINING_GRID_DIVS);

        const auto stamp = this->rcl_clock->now();

        MarkerMsg& zone = this->markers.markers.emplace_back();
        zone.header.frame_id = this->tf_cache.arena_frame_id;
        zone.header.stamp = stamp;
        zone.ns = "auto_mining_zone";
        zone.id = 3000;
        zone.type = MarkerMsg::LINE_STRIP;
        zone.action = MarkerMsg::ADD;
        zone.lifetime = grid_marker_lifetime;
        zone.scale.x = 0.03f;
        zone.color.r = 0.2f;
        zone.color.g = 0.95f;
        zone.color.b = 0.2f;
        zone.color.a = 0.9f;
        zone.points.resize(5);
        zone.points[0].x = min_x;
        zone.points[0].y = min_y;
        zone.points[0].z = 0.01;
        zone.points[1].x = max_x;
        zone.points[1].y = min_y;
        zone.points[1].z = 0.01;
        zone.points[2].x = max_x;
        zone.points[2].y = max_y;
        zone.points[2].z = 0.01;
        zone.points[3].x = min_x;
        zone.points[3].y = max_y;
        zone.points[3].z = 0.01;
        zone.points[4] = zone.points[0];

        if (cell_size > 0.f)
        {
            const float grid_max_x =
                std::min(
                    max_x,
                    min_x +
                        (static_cast<float>(render_x_divisions) * cell_size));
            const float grid_max_y =
                std::min(
                    max_y,
                    min_y +
                        (static_cast<float>(render_y_divisions) * cell_size));

            MarkerMsg& grid_lines = this->markers.markers.emplace_back();
            grid_lines.header = zone.header;
            grid_lines.ns = "auto_mining_zone_grid";
            grid_lines.id = 3001;
            grid_lines.type = MarkerMsg::LINE_LIST;
            grid_lines.action = MarkerMsg::ADD;
            grid_lines.lifetime = zone.lifetime;
            grid_lines.scale.x = 0.01f;
            grid_lines.color.r = 0.2f;
            grid_lines.color.g = 0.7f;
            grid_lines.color.b = 1.0f;
            grid_lines.color.a = 0.5f;
            grid_lines.points.reserve(
                static_cast<size_t>(
                    (render_x_divisions + render_y_divisions + 2) * 2));

            for (uint32_t ix = 0; ix <= render_x_divisions; ix++)
            {
                const float x = min_x + (static_cast<float>(ix) * cell_size);
                geometry_msgs::msg::Point p0, p1;
                p0.x = x;
                p0.y = min_y;
                p0.z = 0.01;
                p1.x = x;
                p1.y = grid_max_y;
                p1.z = 0.01;
                grid_lines.points.push_back(p0);
                grid_lines.points.push_back(p1);
            }
            for (uint32_t iy = 0; iy <= render_y_divisions; iy++)
            {
                const float y = min_y + (static_cast<float>(iy) * cell_size);
                geometry_msgs::msg::Point p0, p1;
                p0.x = min_x;
                p0.y = y;
                p0.z = 0.01;
                p1.x = grid_max_x;
                p1.y = y;
                p1.z = 0.01;
                grid_lines.points.push_back(p0);
                grid_lines.points.push_back(p1);
            }

            MarkerMsg& grid_points = this->markers.markers.emplace_back();
            grid_points.header = zone.header;
            grid_points.ns = "auto_mining_zone_points";
            grid_points.id = 3002;
            grid_points.type = MarkerMsg::POINTS;
            grid_points.action = MarkerMsg::ADD;
            grid_points.lifetime = zone.lifetime;
            grid_points.scale.x = 0.05f;
            grid_points.scale.y = 0.05f;
            grid_points.color.r = 1.0f;
            grid_points.color.g = 1.0f;
            grid_points.color.b = 1.0f;
            grid_points.color.a = 0.65f;
            grid_points.points.reserve(
                static_cast<size_t>(render_x_divisions) *
                static_cast<size_t>(render_y_divisions));

            for (uint32_t ix = 0; ix < render_x_divisions; ix++)
            {
                for (uint32_t iy = 0; iy < render_y_divisions; iy++)
                {
                    geometry_msgs::msg::Point p;
                    p.x =
                        min_x + ((static_cast<float>(ix) + 0.5f) * cell_size);
                    p.y =
                        min_y + ((static_cast<float>(iy) + 0.5f) * cell_size);
                    p.z = 0.015;
                    grid_points.points.push_back(p);
                }
            }
        }
    }

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

    MarkerMsg& marker = this->markers.markers.emplace_back();

    marker.header.frame_id = this->tf_cache.arena_frame_id;
    marker.header.stamp = this->rcl_clock->now();

    marker.ns = "robot";
    marker.id = 1;
    marker.type = MarkerMsg::CUBE;
    marker.action = MarkerMsg::ADD;
    marker.lifetime = rclcpp::Duration(0, 100000000);

    static const ColorMsg CONSTRAINT_COLORS[] = {
        ColorMsg{}.set__r(0.9f).set__g(0.8f).set__b(0.2f).set__a(0.5f),
        ColorMsg{}.set__r(1.f).set__a(0.5f),
        ColorMsg{}.set__r(0.6f).set__g(0.2f).set__b(0.9f).set__a(0.5f),
        ColorMsg{}.set__r(0.8f).set__g(0.5f).set__b(0.2f).set__a(0.5f),
    };

    marker.color = CONSTRAINT_COLORS
        [(constraint > MiningConstraints::CONSTRAINT_MOTOR_STALL) +
         (constraint > MiningConstraints::CONSTRAINT_OBSTACLE) +
         (constraint > MiningConstraints::CONSTRAINT_HOPPER_FULL)];

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

void TelemetryDeserializer::addOffloadMarker(float dist) { (void)dist; }

};  // namespace lance
