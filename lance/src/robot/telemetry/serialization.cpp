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

#include "serialization.hpp"

#include <cstdint>

#include "util/ros_utils.hpp"
#include "util/mem_helpers.hpp"


using namespace util;


namespace lance
{

/* --- Telemetry Specification ---
 * All telemetry is packaged into a binary blob and sent from the robot
 * to the client-side decoder, where it is decoded and publishers are used as
 * needed.
 * The telemetry blob consists of any number of packets, each of which starts
 * with an id (TelemetryType) followed by the packet data, whose size can vary.
 * As such, it is the responsiblity of each individual (per telemetry type)
 * decoder to read the correct length of bytes after it's id appears in the
 * blob.
 * The control state telemetry type itself encapsulates another layer which
 * works similarly to this - that is, each controller may recursively contain
 * other controller states. The encoding and decoding process should work
 * recursively to handle decoding the overall state properly. */

enum class TelemetryType : uint8_t
{
    INVALID_ID = 0,

    MOTOR_CTRL,
    ARENA_TF,
    COLLECTION_STATE,
    CTRL_STATE
};

enum class ControllerType : uint8_t
{
    INVALID_ID = 0,

    AUTO,
    TELEOP,

    AUTO_MINING,
    AUTO_OFFLOAD,

    MINING,
    OFFLOAD,
    LOCALIZATION,
    TRAVERSAL
};



// --- Serializer --------------------------------------------------------------

TelemetrySerializer::TelemetrySerializer(RclNode& node) :
    pub{node.create_publisher<BytesMsg>(
        TELEMETRY_TOPIC,
        rclcpp::SensorDataQoS{})},
    throttled_pub_freq{
        declare_and_get_param(node, "telemetry/throttled_pub_freq", 1.f)}
{
}

void TelemetrySerializer::update(
    const RobotMotorCommands& motor_commands,
    const RobotController& robot_controller)
{
    BytesMsg msg;
    Bytes& bytes = msg.data;

    this->addMotorCommands(bytes, motor_commands);
    this->addArenaTf(bytes, robot_controller.tf_cache);
    this->addCollectionState(bytes, robot_controller.collection_state);
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


void TelemetrySerializer::addMotorCommands(
    Bytes& bytes,
    const RobotMotorCommands& motor_commands)
{
    constexpr size_t RESERVE_SIZE =
        (sizeof(uint8_t) +
         (sizeof(TalonCtrlMsg::_mode_type) + sizeof(float)) * 5);

    bytes.resize(bytes.size() + RESERVE_SIZE);
    Byte* ptr = (bytes.end() - RESERVE_SIZE).base();

    writeAndIncrement(ptr, static_cast<uint8_t>(TelemetryType::MOTOR_CTRL));

#define WRITE_MOTOR(m)                                       \
    writeAndIncrement(ptr, motor_commands.m.mode);           \
    writeAsAndIncrement<float>(ptr, motor_commands.m.value);

    WRITE_MOTOR(track_right)
    WRITE_MOTOR(track_left)
    WRITE_MOTOR(trencher)
    WRITE_MOTOR(hopper_belt)
    WRITE_MOTOR(hopper_actuator)

#undef WRITE_MOTOR
}

void TelemetrySerializer::addArenaTf(Bytes& bytes, const TfCache& tf_cache)
{
    constexpr size_t RESERVE_SIZE = (sizeof(uint8_t) + (sizeof(float) * 7));

    if (tf_cache.hasTf(ARENA_TO_ODOM_TF) && this->filterFreq(this->last_tf_pub))
    {
        const auto* tf = tf_cache.getTf(ARENA_TO_ODOM_TF);

        bytes.resize(bytes.size() + RESERVE_SIZE);
        Byte* ptr = (bytes.end() - RESERVE_SIZE).base();

        writeAndIncrement(ptr, static_cast<uint8_t>(TelemetryType::ARENA_TF));

        writeAndIncrement(ptr, tf->pose.vec.x());
        writeAndIncrement(ptr, tf->pose.vec.y());
        writeAndIncrement(ptr, tf->pose.vec.z());
        writeAndIncrement(ptr, tf->pose.quat.w());
        writeAndIncrement(ptr, tf->pose.quat.x());
        writeAndIncrement(ptr, tf->pose.quat.y());
        writeAndIncrement(ptr, tf->pose.quat.z());
    }
}

void TelemetrySerializer::addCollectionState(
    Bytes& bytes,
    const CollectionState& collection_state)
{
    constexpr size_t RESERVE_SIZE = (sizeof(uint8_t) * 2 + (sizeof(float) * 7));

    bytes.resize(bytes.size() + RESERVE_SIZE);
    Byte* ptr = (bytes.end() - RESERVE_SIZE).base();

    writeAndIncrement(
        ptr,
        static_cast<uint8_t>(TelemetryType::COLLECTION_STATE));

    const HopperState& hopper_state = collection_state.getHopperState();

    const uint8_t bool_fields =
        static_cast<uint8_t>(hopper_state.isVolCapacity()) |
        (static_cast<uint8_t>(hopper_state.isBeltCapacity()) << 1);
    writeAndIncrement(ptr, bool_fields);

    writeAsAndIncrement<float>(ptr, hopper_state.volume());
    writeAsAndIncrement<float>(ptr, hopper_state.beltPosMeters());
    writeAsAndIncrement<float>(ptr, hopper_state.startPosMeters());
    writeAsAndIncrement<float>(ptr, hopper_state.endPosMeters());
    writeAsAndIncrement<float>(ptr, hopper_state.beltUsageMeters());
    writeAsAndIncrement<float>(ptr, hopper_state.miningTargetMotorPosition());
    writeAsAndIncrement<float>(ptr, hopper_state.offloadTargetMotorPosition());
}

void TelemetrySerializer::addControlState(
    Bytes& bytes,
    const RobotController& robot_controller)
{
    if (robot_controller.control_mode == ControlMode::DISABLED)
    {
        return;
    }

    bytes.push_back(static_cast<uint8_t>(TelemetryType::CTRL_STATE));

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

void TelemetrySerializer::addAutoController(
    Bytes& bytes,
    const AutoController& controller)
{
    using Stage = AutoController::Stage;

    // TODO: pack these into the same byte (*reliably*)
    bytes.push_back(static_cast<uint8_t>(ControllerType::AUTO));
    bytes.push_back(static_cast<uint8_t>(controller.stage));

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

void TelemetrySerializer::addTeleopController(
    Bytes& bytes,
    const TeleopController& controller)
{
    using Op = TeleopController::Operation;

    // TODO: pack these
    bytes.push_back(static_cast<uint8_t>(ControllerType::TELEOP));
    bytes.push_back(static_cast<uint8_t>(controller.op_mode));

    switch (controller.op_mode)
    {
        case Op::ASSISTED_MINING:
        case Op::PRESET_MINING:
        {
            this->addMiningController(bytes, controller.mining_controller);
            break;
        }
        case Op::ASSISTED_OFFLOAD:
        case Op::PRESET_OFFLOAD:
        {
            this->addOffloadController(bytes, controller.offload_controller);
            break;
        }
        case Op::AUTO_TRAVERSAL:
        {
            this->addTravController(bytes, controller.traversal_controller);
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

    // TODO: pack these
    bytes.push_back(static_cast<uint8_t>(ControllerType::AUTO_MINING));
    bytes.push_back(static_cast<uint8_t>(controller.stage));

    // TODO: push planned swath

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
    bytes.push_back(static_cast<uint8_t>(ControllerType::AUTO_OFFLOAD));
    bytes.push_back(static_cast<uint8_t>(controller.stage));

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
    // using Stage = MiningController::Stage;

    // TODO: pack these
    bytes.push_back(static_cast<uint8_t>(ControllerType::MINING));
    bytes.push_back(static_cast<uint8_t>(controller.stage));

    bytes.resize(bytes.size() + sizeof(float));
    write(
        (bytes.end() - sizeof(float)).base(),
        controller.traversal_state.remaining());
}

void TelemetrySerializer::addOffloadController(
    Bytes& bytes,
    const OffloadController& controller)
{
    // using Stage = OffloadController::Stage;

    // TODO: pack these
    bytes.push_back(static_cast<uint8_t>(ControllerType::OFFLOAD));
    bytes.push_back(static_cast<uint8_t>(controller.stage));

    bytes.resize(bytes.size() + sizeof(float));
    write(
        (bytes.end() - sizeof(float)).base(),
        controller.traversal_state.remaining());
}

void TelemetrySerializer::addLocController(
    Bytes& bytes,
    const LocalizationController& controller)
{
    // using Stage = LocalizationController::Stage;

    // TODO: pack these
    bytes.push_back(static_cast<uint8_t>(ControllerType::LOCALIZATION));
    bytes.push_back(static_cast<uint8_t>(controller.stage));
}

void TelemetrySerializer::addTravController(
    Bytes& bytes,
    const TraversalController& controller)
{
    // using Stage = TraversalController::State;

    // TODO: pack these
    bytes.push_back(static_cast<uint8_t>(ControllerType::TRAVERSAL));
    bytes.push_back(static_cast<uint8_t>(controller.state));

    if (controller.last_path.get() && this->filterFreq(this->last_path_pub))
    {
        // controller.state only holds 5ish values so use the highest bit of
        // that byte to signal if path is present or not
        bytes.back() |= 0x80;

        const auto& poses = controller.last_path->poses;

        const size_t reserve_size =
            (poses.size() * (sizeof(float) * 3) + sizeof(uint32_t));

        bytes.resize(bytes.size() + reserve_size);
        Byte* ptr = (bytes.end() - reserve_size).base();

        writeAndIncrement(ptr, static_cast<uint8_t>(poses.size()));

        for (const auto& p : poses)
        {
            writeAsAndIncrement<float>(ptr, p.pose.position.x);
            writeAsAndIncrement<float>(ptr, p.pose.position.y);
            writeAsAndIncrement<float>(ptr, p.pose.position.z);
        }
    }
}



// --- Deserializer ------------------------------------------------------------

TelemetryDeserializer::TelemetryDeserializer(RclNode& node) :
    pub_map{node, "/lance", rclcpp::SensorDataQoS{}},
    tf_broadcaster{node},
    rcl_clock{node.get_clock()},
    sub{node.create_subscription<BytesMsg>(
        TELEMETRY_TOPIC,
        rclcpp::SensorDataQoS{},
        [this](const BytesMsg::ConstSharedPtr& msg) { this->accept(*msg); })},
    odom_frame_id{},
    arena_frame_id{}
{
}

void TelemetryDeserializer::accept(const BytesMsg& msg)
{
    this->ctrl_chain.clear();

    const Byte* ptr = msg.data.data();
    bool ok = true;
    while (ok && ptr < msg.data.end().base())
    {
        uint8_t id = static_cast<uint8_t>(TelemetryType::INVALID_ID);
        readAndIncrement(ptr, id);

        switch (id)
        {
            case static_cast<uint8_t>(TelemetryType::MOTOR_CTRL):
            {
                ok &= this->pubMotorCommands(ptr);
                break;
            }
            case static_cast<uint8_t>(TelemetryType::ARENA_TF):
            {
                ok &= this->pubArenaTf(ptr);
                break;
            }
            case static_cast<uint8_t>(TelemetryType::COLLECTION_STATE):
            {
                ok &= this->pubCollectionState(ptr);
                break;
            }
            case static_cast<uint8_t>(TelemetryType::CTRL_STATE):
            {
                ok &= this->pubControlState(ptr);
                break;
            }
            case static_cast<uint8_t>(TelemetryType::INVALID_ID):
            default:
            {
                ok = false;
            }
        }
    }

    // publish ctrl state strings from chain
}

bool TelemetryDeserializer::pubMotorCommands(ReadPtr ptr)
{
    constexpr char const* MOTOR_CTRL_TOPICS[] = {
        "track_right/ctrl",
        "track_left/ctrl",
        "trencher/ctrl",
        "hopper_belt/ctrl",
        "hopper_act/ctrl"};

    for (const char* TOPIC : MOTOR_CTRL_TOPICS)
    {
        TalonCtrlMsg msg;

        readAndIncrement(ptr, msg.mode);
        readAsAndIncrement<float>(ptr, msg.value);

        this->pub_map.publish(TOPIC, msg);
    }

    return true;
}

bool TelemetryDeserializer::pubArenaTf(ReadPtr ptr)
{
    geometry_msgs::msg::TransformStamped msg;

    readAsAndIncrement<float>(ptr, msg.transform.translation.x);
    readAsAndIncrement<float>(ptr, msg.transform.translation.y);
    readAsAndIncrement<float>(ptr, msg.transform.translation.z);
    readAsAndIncrement<float>(ptr, msg.transform.rotation.w);
    readAsAndIncrement<float>(ptr, msg.transform.rotation.x);
    readAsAndIncrement<float>(ptr, msg.transform.rotation.y);
    readAsAndIncrement<float>(ptr, msg.transform.rotation.z);

    msg.child_frame_id = this->odom_frame_id;
    msg.header.frame_id = this->arena_frame_id;
    msg.header.stamp = this->rcl_clock->now();

    this->tf_broadcaster.sendTransform(msg);

    return true;
}

bool TelemetryDeserializer::pubCollectionState(ReadPtr ptr)
{
    constexpr char const* FLOAT_TOPICS[] = {
        "collection_state/volume",
        "collection_state/mining_target",
        "collection_state/offload_target",
        "collection_state/belt_pos_m",
        "collection_state/high_pos_m",
        "collection_state/low_pos_m",
        "collection_state/belt_usage_m"};

    uint8_t bool_fields{0};
    readAndIncrement(ptr, bool_fields);

    this->pub_map.publish<std_msgs::msg::Bool>(
        "collection_state/is_full_volume",
        static_cast<bool>(bool_fields & 0x1));
    this->pub_map.publish<std_msgs::msg::Bool>(
        "collection_state/is_full_occ",
        static_cast<bool>(bool_fields & 0x2));

    for (const char* TOPIC : FLOAT_TOPICS)
    {
        float val{0.f};
        readAndIncrement(ptr, val);

        this->pub_map.publish<std_msgs::msg::Float32>(TOPIC, val);
    }

    return true;
}

bool TelemetryDeserializer::pubControlState(ReadPtr ptr)
{
    uint8_t ctrl_id = static_cast<uint8_t>(ControllerType::INVALID_ID);
    readAndIncrement(ptr, ctrl_id);

    switch(ctrl_id)
    {
        case static_cast<uint8_t>(ControllerType::TELEOP):
        {
            this->ctrl_chain.push_back("Teleop");
            return this->pubTeleopController(ptr);
        }
        case static_cast<uint8_t>(ControllerType::AUTO):
        {
            this->ctrl_chain.push_back("Auto");
            return this->pubAutoController(ptr);
        }
        default:
        {
            return false;
        }
    }
}

bool TelemetryDeserializer::pubAutoController(ReadPtr ptr)
{
    // uint8_t 
}

bool TelemetryDeserializer::pubTeleopController(ReadPtr ptr)
{

}

bool TelemetryDeserializer::pubAutoMiningController(ReadPtr ptr)
{

}

bool TelemetryDeserializer::pubAutoOffloadController(ReadPtr ptr)
{

}

bool TelemetryDeserializer::pubMiningController(ReadPtr ptr)
{

}

bool TelemetryDeserializer::pubOffloadController(ReadPtr ptr)
{

}

bool TelemetryDeserializer::pubLocController(ReadPtr ptr)
{

}

bool TelemetryDeserializer::pubTravController(ReadPtr ptr)
{

}


};  // namespace lance
