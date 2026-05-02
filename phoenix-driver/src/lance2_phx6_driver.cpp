/*******************************************************************************
*   Copyright (C) 2025-2026 Cardinal Space Mining Club                         *
*                                                                              *
*                                 ;xxxxxxx:                                   *
*                                ;$$$$$$$$$       ...::..                     *
*                                $$$$$$$$$$x   .:::::::::::..                 *
*                             x$$$$$$$$$$$$$$::::::::::::::::.                *
*                         :$$$$$&X;      .xX:::::::::::::.::...               *
*                 .$$Xx++$$$$+  :::.     :;:   .::::::.  ....  :              *
*                :$$$$$$$$$  ;:      ;xXXXXXXXx  .::.  .::::. .:.             *
*               :$$$$$$$$: ;      ;xXXXXXXXXXXXXx: ..::::::  .::.             *
*              ;$$$$$$$$ ::   :;XXXXXXXXXXXXXXXXXX+ .::::.  .:::              *
*               X$$$$$X : +XXXXXXXXXXXXXXXXXXXXXXXX; .::  .::::.              *
*                .$$$$ :xXXXXXXXXXXXXXXXXXXXXXXXXXXX.   .:::::.               *
*                 X$$X XXXXXXXXXXXXXXXXXXXXXXXXXXXXx:  .::::.                 *
*                 $$$:.XXXXXXXXXXXXXXXXXXXXXXXXXXX  ;; ..:.                   *
*                 $$& :XXXXXXXXXXXXXXXXXXXXXXXX;  +XX; X$$;                   *
*                 $$$: XXXXXXXXXXXXXXXXXXXXXX; :XXXXX; X$$;                   *
*                 X$$X XXXXXXXXXXXXXXXXXXX; .+XXXXXXX; $$$                    *
*                 $$$$ ;XXXXXXXXXXXXXXX+  +XXXXXXXXx+ X$$$+                   *
*               x$$$$$X ;XXXXXXXXXXX+ :xXXXXXXXX+   .;$$$$$$                  *
*              +$$$$$$$$ ;XXXXXXx;;+XXXXXXXXX+    : +$$$$$$$$                 *
*               +$$$$$$$$: xXXXXXXXXXXXXXX+      ; X$$$$$$$$                  *
*                :$$$$$$$$$. +XXXXXXXXX;      ;: x$$$$$$$$$                   *
*                ;x$$$$XX$$$$+ .;+X+      :;: :$$$$$xX$$$X                    *
*               ;;;;;;;;;;X$$$$$$$+      :X$$$$$$&.                           *
*               ;;;;;;;:;;;;;x$$$$$$$$$$$$$$$$x.                              *
*               :;;;;;;;;;;;;.  :$$$$$$$$$$X                                  *
*                .;;;;;;;;:;;    +$$$$$$$$$                                   *
*                  .;;;;;;.       X$$$$$$$:                                   *
*                                                                              *
*   Unless required by applicable law or agreed to in writing, software        *
*   distributed under the License is distributed on an "AS IS" BASIS,          *
*   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.   *
*   See the License for the specific language governing permissions and        *
*   limitations under the License.                                             *
*                                                                              *
*******************************************************************************/
#include "lance2_phx6_driver.hpp"
#include "lance2_phx6_mechanisms.hpp"

#include <vector>
#include <algorithm>
#include <cmath>
#include <optional>
#include <unordered_map>
#include <unordered_set>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/string.hpp>

#define Phoenix_No_WPI  // remove WPI dependencies
#include <ctre/phoenix/cci/Diagnostics_CCI.h>
#include <ctre/phoenix6/controls/DifferentialFollower.hpp>
#include <ctre/phoenix6/controls/Follower.hpp>
#include <ctre/phoenix6/TalonFX.hpp>
#include <ctre/phoenix6/unmanaged/Unmanaged.hpp>

#include "ros_utils.hpp"
#include "lance2_phx6_utils.hpp"

using namespace util;
using namespace util::ros_aliases;
using namespace std::chrono_literals;

#define TALON_CTRL_SUB_QOS                                               \
    rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile()


bool isSupportedTalonCtrlMode(int8_t mode)
{
    switch (mode)
    {
        case TalonCtrlMsg::PERCENT_OUTPUT:
        case TalonCtrlMsg::POSITION:
        case TalonCtrlMsg::VELOCITY:
        case TalonCtrlMsg::VOLTAGE:
        case TalonCtrlMsg::MUSIC_TONE:
        case TalonCtrlMsg::DISABLED:
            return true;
        default:
            return false;
    }
}


Phoenix6Driver::Phoenix6Driver() :
    Node("lance2_phoenix6_driver"),
    bus_name{declare_and_get_param(*this, "canbus", std::string("canable_A"))},
    bus(this->bus_name),
    diagnostic_server_port(declare_and_get_param(*this, "diagnostics_port", 0)),
    watchdog_status_sub{this->create_subscription<Int32Msg>(
        "lance/watchdog_status",
        rclcpp::SensorDataQoS{},
        [this](const Int32Msg& msg) { this->feedWatchdogStatus(msg.data); })},
    info_pub_timer{this->create_wall_timer(
        declare_and_get_param(*this, "info_pub_rate_ms", 50) * 1ms,
        [this]() { this->pubMotorInfo_cb(); })},
    fault_pub_timer{this->create_wall_timer(
        declare_and_get_param(*this, "fault_pub_rate_ms", 250) * 1ms,
        [this]() { this->pubMotorFault_cb(); })}
{
    // --- Init phoenix -------------------------------------------------------------
    if (diagnostic_server_port > 0)
    {
        c_Phoenix_Diagnostics_Create_On_Port(diagnostic_server_port);
    }
    else
    {
        // this might not be needed
        c_Phoenix_Diagnostics_SetSecondsToStart(-1);
    }

    parseMechanismConfigs();
    setupMotors();
    setupMechanisms();
}

Phoenix6Driver::~Phoenix6Driver() { c_Phoenix_Diagnostics_Dispose(); }

static std::optional<phx6::signals::MotorAlignmentValue> parseMotorAlignment(
    const std::string& alignment)
{
    static const std::
        unordered_map<std::string, phx6::signals::MotorAlignmentValue>
            kAlignmentMap = {
                {"Aligned", phx6::signals::MotorAlignmentValue::Aligned},
                {"Opposed", phx6::signals::MotorAlignmentValue::Opposed},
    };

    auto it = kAlignmentMap.find(alignment);
    if (it == kAlignmentMap.end())
    {
        return std::nullopt;
    }
    return it->second;
}

static std::optional<phx6::signals::MotorArrangementValue>
    parseMotorArrangement(const std::string& arrangement)
{
    static const std::unordered_map<
        std::string,
        phx6::signals::MotorArrangementValue>
        kArrangementMap = {
            {       "Disabled",phx6::signals::MotorArrangementValue::Disabled                               },
            {     "Minion_JST", phx6::signals::MotorArrangementValue::Minion_JST},
            {     "Brushed_DC", phx6::signals::MotorArrangementValue::Brushed_DC},
            {     "brushed_dc", phx6::signals::MotorArrangementValue::Brushed_DC},
            {        "NEO_JST",    phx6::signals::MotorArrangementValue::NEO_JST},
            {     "NEO550_JST", phx6::signals::MotorArrangementValue::NEO550_JST},
            {     "VORTEX_JST", phx6::signals::MotorArrangementValue::VORTEX_JST},
            {"CustomBrushless",
             phx6::signals::MotorArrangementValue::CustomBrushless              },
    };

    auto it = kArrangementMap.find(arrangement);
    if (it == kArrangementMap.end())
    {
        return std::nullopt;
    }
    return it->second;
}

void Phoenix6Driver::parseMechanismConfigs()
{
    auto mechanism_names = declare_and_get_param<std::vector<std::string>>(
        *this,
        "mechanism_names",
        {});

    std::unordered_set<std::string> mechanism_names_seen;
    for (const auto& name : mechanism_names)
    {
        if (!mechanism_names_seen.emplace(name).second)
        {
            RCLCPP_ERROR(
                get_logger(),
                "Mechanism name '%s' is configured more than once",
                name.c_str());
            continue;
        }

        const std::string param_prefix = "mechanisms." + name + ".";
        MechanismConfig config;
        config.name = name;
        config.type = declare_and_get_param<std::string>(
            *this,
            param_prefix + "type",
            "");
        config.motors = declare_and_get_param<std::vector<std::string>>(
            *this,
            param_prefix + "motors",
            {});

        const std::string alignment = declare_and_get_param<std::string>(
            *this,
            param_prefix + "alignment",
            "Aligned");
        auto parsed_alignment = parseMotorAlignment(alignment);
        if (!parsed_alignment)
        {
            RCLCPP_ERROR(
                get_logger(),
                "Mechanism %s has invalid alignment '%s'",
                name.c_str(),
                alignment.c_str());
            continue;
        }
        config.alignment = *parsed_alignment;

        config.sensor_to_differential_ratio = declare_and_get_param<double>(
            *this,
            param_prefix + "sensor_to_differential_ratio",
            1.0);
        config.closed_loop_rate_hz = declare_and_get_param<double>(
            *this,
            param_prefix + "closed_loop_rate_hz",
            100.0);
        config.update_period_ms = declare_and_get_param<int>(
            *this,
            param_prefix + "update_period_ms",
            20);
        if (config.update_period_ms <= 0)
        {
            RCLCPP_ERROR(
                get_logger(),
                "Mechanism %s has invalid update_period_ms %d; using 20 ms",
                name.c_str(),
                config.update_period_ms);
            config.update_period_ms = 20;
        }

        static const std::unordered_set<std::string> kMechanismTypes = {
            "CustomMechanism",
            "SimpleDifferentialMechanism",
        };
        if (kMechanismTypes.find(config.type) == kMechanismTypes.end())
        {
            RCLCPP_ERROR(
                get_logger(),
                "Mechanism %s has invalid type '%s'",
                name.c_str(),
                config.type.c_str());
            continue;
        }

        for (const auto& motor_name : config.motors)
        {
            if (!mechanism_motor_names.emplace(motor_name).second)
            {
                RCLCPP_ERROR(
                    get_logger(),
                    "Motor %s is owned by more than one mechanism",
                    motor_name.c_str());
            }
        }

        mechanism_configs.emplace_back(std::move(config));
    }
}

void Phoenix6Driver::setupMotors()
{
    // --- Get Motors-------------------------------------------------------------
    auto motor_names =
        declare_and_get_param<std::vector<std::string>>(*this, "names", {});

    std::unordered_map<std::string, int> ids;
    for (auto name : motor_names)
    {
        const std::string param_prefix = "motors." + name + ".";
        int id = declare_and_get_param(*this, param_prefix + "can_id", -1);

        ids.emplace(name, id);
    }

    for (auto name : motor_names)
    {
        const std::string param_prefix = "motors." + name + ".";

        // Basic Motor Info
        std::string controller = declare_and_get_param<std::string>(
            *this,
            param_prefix + "controller",
            "");
        int id = declare_and_get_param(*this, param_prefix + "can_id", -1);

        if (controller.empty() || id == -1)
        {
            RCLCPP_ERROR(
                get_logger(),
                "Motor %s is missing required parameters",
                name.c_str());
            continue;
        }

        // Motor Config, Includes all items from readme
        RclMotorConfig params;

        std::string follows = declare_and_get_param<std::string>(
            *this,
            param_prefix + "follows",
            "");
        if (follows.empty())
        {
            params.follows_id = -1;
        }
        else
        {
            auto follows_it = ids.find(follows);
            if (follows_it == ids.end())
            {
                RCLCPP_ERROR(
                    get_logger(),
                    "Motor %s follows unknown motor '%s'",
                    name.c_str(),
                    follows.c_str());
                continue;
            }
            params.follows_id = follows_it->second;
        }

        params.output_inverted = declare_and_get_param<bool>(
            *this,
            param_prefix + "output_inverted",
            false);

        std::string alignment = declare_and_get_param<std::string>(
            *this,
            param_prefix + "alignment",
            "Aligned");
        auto parsed_alignment = parseMotorAlignment(alignment);
        if (!parsed_alignment)
        {
            RCLCPP_ERROR(
                get_logger(),
                "Motor %s has invalid alignment '%s'",
                name.c_str(),
                alignment.c_str());
            continue;
        }
        params.alignment = *parsed_alignment;

        params.follower_type = declare_and_get_param<std::string>(
            *this,
            param_prefix + "follower_type",
            "normal");

        static const std::unordered_set<std::string> kFollowerTypes = {
            "normal",
            "DifferentialFollower",
            "CustomMechanism",
        };
        if (kFollowerTypes.find(params.follower_type) == kFollowerTypes.end())
        {
            RCLCPP_ERROR(
                get_logger(),
                "Motor %s has invalid follower_type '%s'",
                name.c_str(),
                params.follower_type.c_str());
            continue;
        }
        if (params.follower_type != "normal" && params.follows_id == -1)
        {
            RCLCPP_ERROR(
                get_logger(),
                "Motor %s has follower_type '%s' but does not set follows",
                name.c_str(),
                params.follower_type.c_str());
            continue;
        }
        if (params.follower_type == "CustomMechanism" && controller != "FXS")
        {
            RCLCPP_ERROR(
                get_logger(),
                "Motor %s has follower_type CustomMechanism but controller "
                "is '%s'; CustomMechanism is only supported on FXS",
                name.c_str(),
                controller.c_str());
            continue;
        }

        if (controller == "FXS")
        {
            std::string arrangement = declare_and_get_param<std::string>(
                *this,
                param_prefix + "motor_arrangement",
                "Disabled");
            auto parsed_arrangement = parseMotorArrangement(arrangement);
            if (!parsed_arrangement)
            {
                RCLCPP_ERROR(
                    get_logger(),
                    "Motor %s has invalid motor_arrangement '%s'",
                    name.c_str(),
                    arrangement.c_str());
                continue;
            }
            params.motor_arrangement = *parsed_arrangement;

            params.temp_sensor_required = declare_and_get_param<bool>(
                *this,
                param_prefix + "temp_sensor_required",
                false);
        }

        std::string sensor = declare_and_get_param<std::string>(
            *this,
            param_prefix + "sensor",
            "");

        static const std::unordered_map<std::string, SensorSource> kSensorMap =
            {
                {                   "",         SensorSource::Commutation},
                {        "Commutation",         SensorSource::Commutation},
                {  "QuadratureEncoder",   SensorSource::QuadratureEncoder},
                {  "PulseWidthEncoder",   SensorSource::PulseWidthEncoder},
                {     "RemoteCANcoder",      SensorSource::RemoteCANcoder},
                {      "FusedCANcoder",       SensorSource::FusedCANcoder},
                {       "SyncCANcoder",        SensorSource::SyncCANcoder},
                {   "RemotePigeon2Yaw",    SensorSource::RemotePigeon2Yaw},
                { "RemotePigeon2Pitch",  SensorSource::RemotePigeon2Pitch},
                {  "RemotePigeon2Roll",   SensorSource::RemotePigeon2Roll},
                {"AnalogPotentiometer", SensorSource::AnalogPotentiometer},
                {         "analog_pot", SensorSource::AnalogPotentiometer},
        };
        auto it = kSensorMap.find(sensor);
        if (it == kSensorMap.end())
        {
            RCLCPP_ERROR(
                get_logger(),
                "Unknown sensor '%s' for motor %s",
                sensor.c_str(),
                name.c_str());
            continue;
        }
        params.sensor = it->second;
        if (params.follower_type == "CustomMechanism" &&
            params.sensor != SensorSource::AnalogPotentiometer)
        {
            RCLCPP_ERROR(
                get_logger(),
                "Motor %s has follower_type CustomMechanism but sensor is "
                "not AnalogPotentiometer",
                name.c_str());
            continue;
        }

        const bool needs_remote_sensor_id =
            params.sensor == SensorSource::RemoteCANcoder ||
            params.sensor == SensorSource::FusedCANcoder ||
            params.sensor == SensorSource::SyncCANcoder ||
            params.sensor == SensorSource::RemotePigeon2Yaw ||
            params.sensor == SensorSource::RemotePigeon2Pitch ||
            params.sensor == SensorSource::RemotePigeon2Roll;

        if (needs_remote_sensor_id)
        {
            declare_param(
                *this,
                param_prefix + "remote_sensor_id",
                params.remote_sensor_id,
                -1);

            if (params.remote_sensor_id == -1)
            {
                RCLCPP_ERROR(
                    get_logger(),
                    "Motor %s is using sensor '%s' but remote_sensor_id is "
                    "not set",
                    name.c_str(),
                    sensor.c_str());
                continue;
            }
        }
        else if (params.sensor == SensorSource::AnalogPotentiometer)
        {
            declare_param(
                *this,
                param_prefix + "pot_max_v",
                params.pot.max_v,
                -1.0);
            declare_param(
                *this,
                param_prefix + "pot_inverted",
                params.pot.invert_sensor,
                false);

            if (params.pot.max_v <= 0.0)
            {
                RCLCPP_ERROR(
                    get_logger(),
                    "Motor %s is using AnalogPotentiometer but pot_max_v is "
                    "not set to a positive value",
                    name.c_str());
                continue;
            }
        }
        else
        {
            params.remote_sensor_id = -1;
        }

        declare_param(*this, param_prefix + "kP", params.kP, 0.2);
        declare_param(*this, param_prefix + "kI", params.kI, 0.05);
        declare_param(*this, param_prefix + "kD", params.kD, 0.0001);
        declare_param(*this, param_prefix + "kV", params.kV, 0.12);
        declare_param(
            *this,
            param_prefix + "neutral_deadband",
            params.neutral_deadband,
            0.05);
        declare_param(
            *this,
            param_prefix + "neutral_brake",
            params.neutral_brake,
            false);
        declare_param(
            *this,
            param_prefix + "stator_current_limit",
            params.stator_current_limit,
            30.0);
        declare_param(
            *this,
            param_prefix + "supply_current_limit",
            params.supply_current_limit,
            20.0);
        declare_param(
            *this,
            param_prefix + "voltage_limit",
            params.voltage_limit,
            12.0);

        // --- Create Motors -------------------------------------------------------------
        const bool enable_ctrl_sub =
            mechanism_motor_names.find(name) == mechanism_motor_names.end();
        if (controller == "FX")
        {
            FX_motors.emplace_back(
                std::make_unique<RclMotor<TalonFX>>(
                    this,
                    name,
                    id,
                    params,
                    bus,
                    enable_ctrl_sub));
            FX_motors_by_name.emplace(name, FX_motors.back().get());
            std::cout << "creating FX motor " << name << std::endl;
        }
        else if (controller == "FXS")
        {
            FXS_motors.emplace_back(
                std::make_unique<RclMotor<TalonFXS>>(
                    this,
                    name,
                    id,
                    params,
                    bus,
                    enable_ctrl_sub));
            FXS_motors_by_name.emplace(name, FXS_motors.back().get());

            std::cout << "creating FXS motor " << name << std::endl;
        }
        else
        {
            RCLCPP_ERROR(
                get_logger(),
                "Motor %s has unknown controller type: %s",
                name.c_str(),
                controller.c_str());
        }
    }
}

// --- RclMotor implementation --------------------------------------------------

template<typename MotorType>
Phoenix6Driver::RclMotor<MotorType>::RclMotor(
    rclcpp::Node* node,
    const std::string& name,
    int id,
    const RclMotorConfig& config,
    const CANBus& bus,
    bool enable_ctrl_sub) :
    motor(id, bus),
    name(name),
    logger(node->get_logger()),
    clock(node->get_clock()),
    config(config),
    info_pub(node->template create_publisher<TalonInfoMsg>(
        "lance/" + name + "/info",
        rclcpp::SensorDataQoS{})),
    faults_pub(node->template create_publisher<TalonFaultsMsg>(
        "lance/" + name + "/faults",
        rclcpp::SensorDataQoS{}))

{
    if (enable_ctrl_sub)
    {
        ctrl_sub = node->create_subscription<TalonCtrlMsg>(
            "lance/" + name + "/ctrl",
            TALON_CTRL_SUB_QOS,
            [this](const TalonCtrlMsg& msg) { this->executeCtrl(msg); });
    }

    // --- Init Motors -------------------------------------------------------------
    ConfigurationT phxConfig = buildMotorConfig<ConfigurationT>(
        config.kP,
        config.kI,
        config.kD,
        config.kV,
        config.neutral_deadband,
        config.neutral_brake,
        config.stator_current_limit,
        config.supply_current_limit,
        config.voltage_limit);

    if (config.output_inverted)
    {
        phxConfig.MotorOutput.Inverted = phx6::signals::InvertedValue::
            Clockwise_Positive;  //Counter Clockwise is positive by default
    }

    if constexpr (std::is_same_v<MotorType, TalonFXS>)
    {
        phxConfig.WithCommutation(
            phx6::configs::CommutationConfigs{}.WithMotorArrangement(
                config.motor_arrangement));
        phxConfig.WithExternalTemp(
            phx6::configs::ExternalTempConfigs{}.WithTempSensorRequired(
                config.temp_sensor_required
                    ? phx6::signals::TempSensorRequiredValue::Required
                    : phx6::signals::TempSensorRequiredValue::Not_Required));

        // configure feedback source if provided (FXS only)
        ExternalFeedbackConfigs feedback{};
        if (config.sensor != Phoenix6Driver::SensorSource::AnalogPotentiometer)
        {
            feedback.WithExternalFeedbackSensorSource(
                static_cast<int>(config.sensor));
            if (config.remote_sensor_id > -1)
            {
                feedback.WithFeedbackRemoteSensorID(config.remote_sensor_id);
            }
        }
        // else: AnalogPotentiometer uses default Commutation; position computed in publishInfo()

        phxConfig.WithExternalFeedback(feedback);
        phxConfig.Commutation.MotorArrangement =
            phx6::signals::MotorArrangementValue::Brushed_DC;
        phxConfig.ExternalTemp.TempSensorRequired =
            phx6::signals::TempSensorRequiredValue::Not_Required;
    }

    auto config_status = motor.GetConfigurator().Apply(phxConfig);
    if (!config_status.IsOK())
    {
        RCLCPP_ERROR(
            logger,
            "Failed to apply config for motor %s: %s (%d): %s",
            name.c_str(),
            config_status.GetName(),
            static_cast<int>(config_status),
            config_status.GetDescription());
    }

    if (config.follows_id != -1)
    {
        if (config.follower_type == "normal")
        {
            phx6::controls::Follower followerCtrl(
                config.follows_id,
                config.alignment);
            auto follower_status = motor.SetControl(followerCtrl);
            if (!follower_status.IsOK())
            {
                RCLCPP_ERROR(
                    logger,
                    "Failed to configure follower for motor %s following CAN "
                    "ID %d: %s (%d): %s",
                    name.c_str(),
                    config.follows_id,
                    follower_status.GetName(),
                    static_cast<int>(follower_status),
                    follower_status.GetDescription());
            }
        }
        else if (config.follower_type == "DifferentialFollower")
        {
            phx6::controls::DifferentialFollower followerCtrl(
                config.follows_id,
                config.alignment);
            auto follower_status = motor.SetControl(followerCtrl);
            if (!follower_status.IsOK())
            {
                RCLCPP_ERROR(
                    logger,
                    "Failed to configure differential follower for motor %s "
                    "following CAN ID %d: %s (%d): %s",
                    name.c_str(),
                    config.follows_id,
                    follower_status.GetName(),
                    static_cast<int>(follower_status),
                    follower_status.GetDescription());
            }
        }
        else if (config.follower_type == "CustomMechanism")
        {
            // CustomMechanism is paired after all FXS motors are created.
        }
    }
}

template<typename MotorType>
void Phoenix6Driver::RclMotor<MotorType>::executeCtrl(const TalonCtrlMsg& msg)
{
    if (custom_ctrl_handler && custom_ctrl_handler(msg))
    {
        return;
    }

    if (!isSupportedTalonCtrlMode(msg.mode))
    {
        RCLCPP_ERROR(
            logger,
            "Motor %s does not support control mode %d",
            name.c_str(),
            msg.mode);
        return;
    }

    auto control_status = (motor << msg);
    if (!control_status.IsOK())
    {
        RCLCPP_ERROR_THROTTLE(
            logger,
            *clock,
            5000,
            "Failed to send control mode %d to motor %s: %s (%d): %s",
            msg.mode,
            name.c_str(),
            control_status.GetName(),
            static_cast<int>(control_status),
            control_status.GetDescription());
    }

    // TODO possible add dynamic load gains, or FOC
    // TODO add new functionality for seprate slot for pos control

    // if constexpr (std::is_same_v<MotorType, TalonFX>)
    // {
    // }
    // else if constexpr (std::is_same_v<MotorType, TalonFXS>)
    // {
    // }
}

// Explicit template instantiations
template class Phoenix6Driver::RclMotor<TalonFX>;
template class Phoenix6Driver::RclMotor<TalonFXS>;

void Phoenix6Driver::feedWatchdogStatus(int32_t status)
{
    /* Watchdog feed decoding:
     * POSTIVE feed time --> enabled
     * ZERO feed time --> disabled
     * NEGATIVE feed time --> autonomous */
    const int32_t timeout_ms = status / 1000;
    if (timeout_ms == 0)
    {
        for (auto& m : this->FX_motors)
        {
            m->motor.SetControl(phx6::controls::NeutralOut{});
        }
        for (auto& m : this->FXS_motors)
        {
            m->motor.SetControl(phx6::controls::NeutralOut{});
        }
        for (auto& pair : custom_mechanisms)
        {
            pair->disable();
        }
        is_disabled = true;
    }
    else
    {
        ctre::phoenix::unmanaged::FeedEnable(std::abs(timeout_ms));
        is_disabled = false;
    }
}

void Phoenix6Driver::pubMotorInfo_cb()
{
    const rclcpp::Time stamp = this->get_clock()->now();
    TalonInfoMsg info_msg{};
    info_msg.header.stamp = stamp;
    for (auto& m : this->FX_motors)
    {
        info_msg << m->motor;
        info_msg.status |= static_cast<uint8_t>(!is_disabled);
        m->info_pub->publish(info_msg);
    }
    for (auto& m : this->FXS_motors)
    {
        info_msg << m->motor;
        info_msg.status |= static_cast<uint8_t>(!is_disabled);
        if (m->config.sensor == SensorSource::AnalogPotentiometer)
        {
            // Software handled sensor position for AnalogPotentiometer
            double v = m->motor.GetAnalogVoltage().GetValueAsDouble();
            double raw = v / m->config.pot.max_v;
            info_msg.position = m->config.pot.invert_sensor ? (1.0 - raw) : raw;
        }
        m->info_pub->publish(info_msg);
    }
    for (auto& pair : custom_mechanisms)
    {
        pair->publishInfo(stamp);
    }
}

void Phoenix6Driver::pubMotorFault_cb()
{
    const rclcpp::Time stamp = this->get_clock()->now();
    TalonFaultsMsg faults_msg{};
    faults_msg.header.stamp = stamp;

    for (auto& m : this->FX_motors)
    {
        faults_msg << m->motor;
        m->faults_pub->publish(faults_msg);
    }
    for (auto& m : this->FXS_motors)
    {
        faults_msg << m->motor;
        m->faults_pub->publish(faults_msg);
    }
    for (auto& pair : custom_mechanisms)
    {
        pair->publishFaults(stamp);
    }
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Phoenix6Driver>();
    RCLCPP_INFO(node->get_logger(), "Driver node (Phoenix6) has started");

    rclcpp::spin(node);

    RCLCPP_INFO(node->get_logger(), "Driver node (Phoenix6) shutting down...");
    rclcpp::shutdown();

    return EXIT_SUCCESS;
}
