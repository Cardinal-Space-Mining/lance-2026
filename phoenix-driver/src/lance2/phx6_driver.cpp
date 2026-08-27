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
#include "phx6_driver.hpp"

#include <algorithm>
#include <cmath>
#include <optional>

#define Phoenix_No_WPI  // remove WPI dependencies
#include <ctre/phoenix/cci/Diagnostics_CCI.h>
#include <ctre/phoenix6/controls/Follower.hpp>
#include <ctre/phoenix6/unmanaged/Unmanaged.hpp>

#include "common.hpp"
#include <csm_utils/ros_utils.hpp>
#include "phx6_utils.hpp"


using namespace util;


static std::optional<MotorAlignmentValue> parseMotorAlignment(
    const std::string& alignment)
{
    static const std::unordered_map<std::string, MotorAlignmentValue>
        kAlignmentMap = {
            {"Aligned", MotorAlignmentValue::Aligned},
            {"Opposed", MotorAlignmentValue::Opposed},
    };

    auto it = kAlignmentMap.find(alignment);
    if (it == kAlignmentMap.end())
    {
        return std::nullopt;
    }
    return it->second;
}

static std::optional<MotorArrangementValue> parseMotorArrangement(
    const std::string& arrangement)
{
    static const std::unordered_map<std::string, MotorArrangementValue>
        kArrangementMap = {
            {       "Disabled",        MotorArrangementValue::Disabled},
            {     "Minion_JST",      MotorArrangementValue::Minion_JST},
            {     "Brushed_DC",      MotorArrangementValue::Brushed_DC},
            {     "brushed_dc",      MotorArrangementValue::Brushed_DC},
            {        "NEO_JST",         MotorArrangementValue::NEO_JST},
            {     "NEO550_JST",      MotorArrangementValue::NEO550_JST},
            {     "VORTEX_JST",      MotorArrangementValue::VORTEX_JST},
            {"CustomBrushless", MotorArrangementValue::CustomBrushless},
    };

    auto it = kArrangementMap.find(arrangement);
    if (it == kArrangementMap.end())
    {
        return std::nullopt;
    }
    return it->second;
}


Phoenix6Driver::Phoenix6Driver() :
    Node("lance2_phoenix6_driver"),
    bus_name{declare_and_get_param(*this, "canbus", std::string("canable_A"))},
    bus(this->bus_name),
    diagnostic_server_port(declare_and_get_param(
        *this,
        "diagnostics_port",
        DEFAULT_DIAG_SERVER_PORT)),
    watchdog_status_sub{this->create_subscription<Int32Msg>(
        "lance/watchdog_status",
        rclcpp::SensorDataQoS{},
        [this](const Int32Msg& msg) { this->feedWatchdogStatus(msg.data); })},
    info_pub_timer{this->create_wall_timer(
        std::chrono::milliseconds(declare_and_get_param(
            *this,
            "info_pub_rate_ms",
            DEFAULT_MOTOR_INFO_PUB_DT_MS)),
        [this]() { this->pubMotorInfoCb(); })},
    fault_pub_timer{this->create_wall_timer(
        std::chrono::milliseconds(declare_and_get_param(
            *this,
            "fault_pub_rate_ms",
            DEFAULT_MOTOR_FAULTS_PUB_DT_MS)),
        [this]() { this->pubMotorFaultCb(); })}
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

    this->parseMechanismConfigs();
    this->setupMotors();
    this->setupMechanisms();
}

Phoenix6Driver::~Phoenix6Driver() { c_Phoenix_Diagnostics_Dispose(); }




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

void Phoenix6Driver::setupMechanisms()
{
    std::unordered_set<std::string> claimed_motors;
    for (const auto& config : mechanism_configs)
    {
        bool overlaps_existing_mechanism = false;
        for (const auto& motor_name : config.motors)
        {
            if (!claimed_motors.emplace(motor_name).second)
            {
                RCLCPP_ERROR(
                    get_logger(),
                    "Mechanism %s overlaps another mechanism on motor %s",
                    config.name.c_str(),
                    motor_name.c_str());
                overlaps_existing_mechanism = true;
            }
        }
        if (overlaps_existing_mechanism)
        {
            continue;
        }

        if (config.type == "CustomMechanism")
        {
            if (config.motors.size() != 2)
            {
                RCLCPP_ERROR(
                    get_logger(),
                    "CustomMechanism %s requires exactly two motors",
                    config.name.c_str());
                continue;
            }

            auto leader_it = FXS_motors_by_name.find(config.motors[0]);
            auto follower_it = FXS_motors_by_name.find(config.motors[1]);
            if (leader_it == FXS_motors_by_name.end() ||
                follower_it == FXS_motors_by_name.end())
            {
                RCLCPP_ERROR(
                    get_logger(),
                    "CustomMechanism %s requires two FXS motors",
                    config.name.c_str());
                continue;
            }

            RclMotor<TalonFXS>* leader = leader_it->second;
            RclMotor<TalonFXS>* follower = follower_it->second;
            if (leader->config.sensor != SensorSource::AnalogPotentiometer ||
                follower->config.sensor != SensorSource::AnalogPotentiometer)
            {
                RCLCPP_ERROR(
                    get_logger(),
                    "CustomMechanism %s requires analog potentiometer feedback "
                    "on both motors",
                    config.name.c_str());
                continue;
            }
            if (leader->custom_ctrl_handler || follower->custom_ctrl_handler)
            {
                RCLCPP_ERROR(
                    get_logger(),
                    "CustomMechanism %s overlaps another custom mechanism",
                    config.name.c_str());
                continue;
            }

            auto pair = std::make_unique<CustomMechanismPair>(
                this,
                config.name,
                leader,
                follower,
                config.update_period_ms,
                &is_disabled);
            CustomMechanismPair* pair_ptr = pair.get();
            leader->custom_ctrl_handler = [pair_ptr](const TalonCtrlMsg& msg)
            { return pair_ptr->executeCtrl(msg); };
            follower->custom_ctrl_handler = [pair_ptr](const TalonCtrlMsg& msg)
            { return pair_ptr->executeCtrl(msg); };
            custom_mechanisms.emplace_back(std::move(pair));
            custom_mechanisms.back()->logPotState("startup");

            RCLCPP_INFO(
                get_logger(),
                "Configured CustomMechanism %s on motors %s/%s with control "
                "topic lance/%s/ctrl and update period %d ms",
                config.name.c_str(),
                leader->name.c_str(),
                follower->name.c_str(),
                config.name.c_str(),
                config.update_period_ms);
        }
        else if (config.type == "SimpleDifferentialMechanism")
        {
            if (config.motors.size() != 2)
            {
                RCLCPP_ERROR(
                    get_logger(),
                    "SimpleDifferentialMechanism %s requires exactly two motors",
                    config.name.c_str());
                continue;
            }

            auto leader_it = FX_motors_by_name.find(config.motors[0]);
            auto follower_it = FX_motors_by_name.find(config.motors[1]);
            if (leader_it == FX_motors_by_name.end() ||
                follower_it == FX_motors_by_name.end())
            {
                RCLCPP_ERROR(
                    get_logger(),
                    "SimpleDifferentialMechanism %s requires two FX motors",
                    config.name.c_str());
                continue;
            }

            RclMotor<TalonFX>* leader = leader_it->second;
            RclMotor<TalonFX>* follower = follower_it->second;
            ctre::phoenix6::controls::DifferentialFollower follower_ctrl(
                leader->motor.GetDeviceID(),
                config.alignment);
            auto follower_status = follower->motor.SetControl(follower_ctrl);
            if (!follower_status.IsOK())
            {
                RCLCPP_ERROR(
                    get_logger(),
                    "Failed to configure SimpleDifferentialMechanism %s "
                    "follower %s following %s: %s (%d): %s",
                    config.name.c_str(),
                    follower->name.c_str(),
                    leader->name.c_str(),
                    follower_status.GetName(),
                    static_cast<int>(follower_status),
                    follower_status.GetDescription());
                continue;
            }

            RCLCPP_INFO(
                get_logger(),
                "Configured SimpleDifferentialMechanism %s on motors %s/%s",
                config.name.c_str(),
                leader->name.c_str(),
                follower->name.c_str());
        }
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

void Phoenix6Driver::pubMotorInfoCb()
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

void Phoenix6Driver::pubMotorFaultCb()
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
    RCLCPP_INFO(node->get_logger(), "LANCE-2 phoenix6 driver initialized.");

    rclcpp::spin(node);

    RCLCPP_INFO(node->get_logger(), "Phoenix6 driver shutting down...");
    rclcpp::shutdown();

    return EXIT_SUCCESS;
}
