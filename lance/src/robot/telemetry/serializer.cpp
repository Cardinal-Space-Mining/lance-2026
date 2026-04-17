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

#include "serializer.hpp"

#include <algorithm>
#include <cmath>

#include "util/mem_helpers.hpp"
#include "robot/core/ros_interface.hpp"
#include "robot/model/dynamics.hpp"
#include "robot/model/geometry.hpp"


using namespace util;
using namespace lance::geom;

#define AS_U8(x) static_cast<uint8_t>(x)


namespace lance
{

TelemetrySerializer::TelemetrySerializer(
    RclNode& node,
    float pub_throttle_freq) :
    pub{node.create_publisher<BytesMsg>(
        lance::TELEMETRY_TOPIC,
        rclcpp::SensorDataQoS{})},
    throttled_pub_freq{pub_throttle_freq}
{
}

void TelemetrySerializer::update(const RobotController& robot_controller)
{
    BytesMsg msg;
    Bytes& bytes = msg.data;

    this->addArenaTf(bytes, robot_controller.sensing_interfaces.tf_cache);
    this->addRobotState(bytes, robot_controller);
    this->addControlState(bytes, robot_controller);

    this->pub->publish(msg);
}


bool TelemetrySerializer::filterFreq(time_point& tp)
{
    const time_point t = steady_clock::now();
    const auto d =
        std::chrono::duration_cast<std::chrono::milliseconds>(t - tp);
    const auto f = std::chrono::milliseconds(
        static_cast<int64_t>(1000.f / this->throttled_pub_freq));

    if (d >= f)
    {
        tp = t;
        return true;
    }
    return false;
}


void TelemetrySerializer::addArenaTf(Bytes& bytes, const TfCache& tf_cache)
{
    constexpr size_t RESERVE_SIZE = (sizeof(uint8_t) + (sizeof(float) * 7));

    // TODO: consider sending odom as well (to verify remote localization health)

    if (tf_cache.hasTf(ODOM_TO_ARENA_TF) && this->filterFreq(this->last_tf_pub))
    {
        const auto* tf = tf_cache.getTf(ODOM_TO_ARENA_TF);

        bytes.resize(bytes.size() + RESERVE_SIZE);
        Byte* ptr = (bytes.end() - RESERVE_SIZE).base();

        writeAndIncrement(ptr, AS_U8(TelemetryType::ARENA_TF));

        writeAsAndIncrement<float>(ptr, tf->pose.vec.x());
        writeAsAndIncrement<float>(ptr, tf->pose.vec.y());
        writeAsAndIncrement<float>(ptr, tf->pose.vec.z());
        writeAsAndIncrement<float>(ptr, tf->pose.quat.w());
        writeAsAndIncrement<float>(ptr, tf->pose.quat.x());
        writeAsAndIncrement<float>(ptr, tf->pose.quat.y());
        writeAsAndIncrement<float>(ptr, tf->pose.quat.z());
    }
}

void TelemetrySerializer::addRobotState(
    Bytes& bytes,
    const RobotController& robot_controller)
{
    constexpr size_t RESERVE_SIZE =
        (sizeof(uint8_t) + sizeof(uint16_t) + (sizeof(float) * 2));

    bytes.resize(bytes.size() + RESERVE_SIZE);
    Byte* ptr = (bytes.end() - RESERVE_SIZE).base();

    writeAndIncrement(ptr, AS_U8(TelemetryType::ROBOT_STATE));

    const HopperState& hopper_state =
        robot_controller.collection_state.getHopperState();
    const MiningController& mining_controller =
        robot_controller.shared_controllers.mining_controller;

    const uint16_t state =
        (static_cast<uint16_t>(
             mining_controller.constraints.enabledConstraints()) |
         (static_cast<uint16_t>(hopper_state.isVolCapacity()) << 8) |
         (static_cast<uint16_t>(hopper_state.isBeltCapacity()) << 9));
    writeAndIncrement(ptr, state);

    writeAsAndIncrement<float>(ptr, hopper_state.volume());
    writeAsAndIncrement<float>(ptr, hopper_state.beltUsagePercent());
}

void TelemetrySerializer::addControlState(
    Bytes& bytes,
    const RobotController& robot_controller)
{
    if (robot_controller.control_mode == ControlMode::DISABLED)
    {
        return;
    }

    bytes.push_back(AS_U8(TelemetryType::CTRL_STATE));

    switch (robot_controller.control_mode)
    {
        case ControlMode::TELEOPERATED:
        {
            this->addTeleopController(
                bytes,
                robot_controller.teleop_controller);
            break;
        }
        case ControlMode::AUTONOMOUS:
        {
            this->addAutoController(bytes, robot_controller.auto_controller);
            break;
        }
        default:
        {
        }
    }
}

void TelemetrySerializer::addTeleopController(
    Bytes& bytes,
    const TeleopController& controller)
{
    using Op = TeleopController::Operation;

    // TODO: pack these
    bytes.push_back(AS_U8(ControllerType::TELEOP));
    bytes.push_back(AS_U8(controller.op_mode));

    switch (controller.op_mode)
    {
        case Op::ASSISTED_MINING:
        case Op::PLANNED_MINING_E:
        {
            this->addMiningController(bytes, controller.mining_controller);
            break;
        }
        case Op::ASSISTED_OFFLOAD:
        case Op::PLANNED_OFFLOAD_E:
        {
            this->addOffloadController(bytes, controller.offload_controller);
            break;
        }
        case Op::PLANNED_TRAVERSAL:
        case Op::PLANNED_MINING_T:
        case Op::PLANNED_OFFLOAD_T:
        {
            this->addTravController(bytes, controller.traversal_controller);
            break;
        }
        default:
        {
        }
    }
}

void TelemetrySerializer::addAutoController(
    Bytes& bytes,
    const AutoController& controller)
{
    using Stage = AutoController::Stage;

    // TODO: pack these into the same byte (*reliably*)
    bytes.push_back(AS_U8(ControllerType::AUTO));
    bytes.push_back(AS_U8(controller.stage));

    switch (controller.stage)
    {
        case Stage::LOCALIZATION:
        {
            this->addLocController(bytes, controller.localization_controller);
            break;
        }
        case Stage::TRAVERSE_TO_MINING:
        {
            this->addTravController(bytes, controller.traversal_controller);
            break;
        }
        case Stage::MINING:
        {
            this->addAutoMiningController(bytes, controller.mining_controller);
            break;
        }
        case Stage::TRAVERSE_TO_OFFLOAD:
        {
            this->addTravController(bytes, controller.traversal_controller);
            break;
        }
        case Stage::OFFLOAD:
        {
            this->addAutoOffloadController(
                bytes,
                controller.offload_controller);
            break;
        }
        default:
        {
        }
    }
}

void TelemetrySerializer::addAutoMiningController(
    Bytes& bytes,
    const AutoMiningController& controller)
{
    using Stage = AutoMiningController::Stage;

    bytes.push_back(AS_U8(ControllerType::AUTO_MINING));
    bytes.push_back(AS_U8(controller.stage));

#if ENABLE_MINING_PLANNER_DEBUG
    if (this->filterFreq(this->last_auto_mining_vis_pub))
    {
        this->addMiningPlannerDebug(bytes, controller, bytes.back());
    }
#endif

    switch (controller.stage)
    {
        case Stage::TRAVERSING:
        {
            this->addTravController(bytes, controller.traversal_controller);
            break;
        }
        case Stage::MINING:
        {
            this->addMiningController(bytes, controller.mining_controller);
            break;
        }
        default:
        {
        }
    }
}

void TelemetrySerializer::addAutoOffloadController(
    Bytes& bytes,
    const AutoOffloadController& controller)
{
    using Stage = AutoOffloadController::Stage;

    // TODO: pack these
    bytes.push_back(AS_U8(ControllerType::AUTO_OFFLOAD));
    bytes.push_back(AS_U8(controller.stage));

    // TODO: push planned zone

    switch (controller.stage)
    {
        case Stage::TRAVERSING:
        {
            this->addTravController(bytes, controller.traversal_controller);
            break;
        }
        case Stage::OFFLOADING:
        {
            this->addOffloadController(bytes, controller.offload_controller);
            break;
        }
        default:
        {
        }
    }
}

void TelemetrySerializer::addMiningController(
    Bytes& bytes,
    const MiningController& controller)
{
    // TODO: pack these
    bytes.push_back(AS_U8(ControllerType::MINING));
    bytes.push_back(AS_U8(controller.stage));

    bytes.push_back(AS_U8(controller.constraints.currentConstraint()));
    bytes.resize(bytes.size() + sizeof(float));
    write(
        (bytes.end() - sizeof(float)).base(),
        controller.constraints.remainingDist());
}

void TelemetrySerializer::addOffloadController(
    Bytes& bytes,
    const OffloadController& controller)
{
    // TODO: pack these
    bytes.push_back(AS_U8(ControllerType::OFFLOAD));
    bytes.push_back(AS_U8(controller.stage));

    bytes.resize(bytes.size() + sizeof(float));
    write(
        (bytes.end() - sizeof(float)).base(),
        controller.traversal_state.remaining());
}

void TelemetrySerializer::addLocController(
    Bytes& bytes,
    const LocalizationController& controller)
{
    // TODO: pack these
    bytes.push_back(AS_U8(ControllerType::LOCALIZATION));
    bytes.push_back(AS_U8(controller.stage));
}

void TelemetrySerializer::addTravController(
    Bytes& bytes,
    const TraversalController& controller)
{
    // TODO: pack these
    bytes.push_back(AS_U8(ControllerType::TRAVERSAL));
    bytes.push_back(AS_U8(controller.state));

    if (controller.pplan_interface.hasPath() &&
        this->filterFreq(this->last_path_pub))
    {
        // controller.state only holds 5ish values so use the highest bit of
        // that byte to signal if path is present or not
        bytes.back() |= 0x80;

        const auto& poses = controller.pplan_interface.getPath()->poses;

        const size_t reserve_size =
            (poses.size() * (sizeof(float) * 3) + sizeof(uint32_t));

        bytes.resize(bytes.size() + reserve_size);
        Byte* ptr = (bytes.end() - reserve_size).base();

        writeAndIncrement(ptr, static_cast<uint32_t>(poses.size()));

        for (const auto& p : poses)
        {
            writeAsAndIncrement<float>(ptr, p.pose.position.x);
            writeAsAndIncrement<float>(ptr, p.pose.position.y);
            writeAsAndIncrement<float>(ptr, p.pose.position.z);
        }
    }
}


void TelemetrySerializer::addMiningPlannerDebug(
    Bytes& bytes,
    const AutoMiningController& controller,
    Byte& state_byte)
{
    const auto& paths = controller.mining_planner.getCachedPaths();
    const size_t n_paths = std::min(paths.size(), AUTO_MINING_MAX_PATHS);

    if (n_paths > 0)
    {
        state_byte |= AUTO_MINING_STATE_PATHS_BIT;

        const size_t reserve_size =
            sizeof(uint32_t) +
            (n_paths * ((sizeof(float) * 4) + sizeof(uint8_t)));

        bytes.resize(bytes.size() + reserve_size);
        Byte* ptr = (bytes.end() - reserve_size).base();

        writeAsAndIncrement<uint32_t>(ptr, n_paths);

        for (size_t i = 0; i < n_paths; i++)
        {
            const auto& path = paths[i];
            const DirectedMiningPath::MiningSwath swath =
                path.getPathCoordinatesInWorldFrame(
                    controller.params,
                    &controller.mining_planner.getGridGeometry());
            const float swath_len_m =
                path.getDistance() * TRACK_SEPARATION_M_<float>;
            const Vec2f end = swath.first + (swath.second * swath_len_m);

            writeAndIncrement(ptr, swath.first.x());
            writeAndIncrement(ptr, swath.first.y());
            writeAndIncrement(ptr, end.x());
            writeAndIncrement(ptr, end.y());
            writeAndIncrement(ptr, static_cast<uint8_t>(path.getDirection()));
        }
    }

    state_byte |= AUTO_MINING_STATE_GRID_BIT;

    const float r = geom::FOOTPRINT_R_MAX_<float>;
    const Vec2f min_corner_with_offset =
        controller.params.bounds.mining_zone.min() + Vec2f::Constant(r);
    const Vec2f max_corner_with_offset =
        controller.params.bounds.mining_zone.max() - Vec2f::Constant(r);

    const float actual_mining_x_length =
        max_corner_with_offset.x() - min_corner_with_offset.x();
    const float actual_mining_y_length =
        max_corner_with_offset.y() - min_corner_with_offset.y();

    const uint32_t x_divisions = std::clamp<uint32_t>(
        std::floor(actual_mining_x_length / TRACK_SEPARATION_M_<float>),
        0,
        AUTO_MINING_MAX_GRID_DIVS);
    const uint32_t y_divisions = std::clamp<uint32_t>(
        std::floor(actual_mining_y_length / TRACK_SEPARATION_M_<float>),
        0,
        AUTO_MINING_MAX_GRID_DIVS);

    const size_t grid_reserve_size =
        (sizeof(float) * 5) + (sizeof(uint32_t) * 2);
    bytes.resize(bytes.size() + grid_reserve_size);
    Byte* grid_ptr = (bytes.end() - grid_reserve_size).base();

    writeAndIncrement(grid_ptr, min_corner_with_offset.x());
    writeAndIncrement(grid_ptr, min_corner_with_offset.y());
    writeAndIncrement(grid_ptr, max_corner_with_offset.x());
    writeAndIncrement(grid_ptr, max_corner_with_offset.y());
    writeAndIncrement(grid_ptr, TRACK_SEPARATION_M_<float>);
    writeAndIncrement(grid_ptr, x_divisions);
    writeAndIncrement(grid_ptr, y_divisions);
}

};  // namespace lance
