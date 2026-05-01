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
#include "lance2_phx6_mechanisms.hpp"

#include <algorithm>
#include <unordered_set>

#include <ctre/phoenix6/controls/DifferentialFollower.hpp>

using namespace std::chrono_literals;

#define TALON_CTRL_SUB_QOS                                               \
    rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile()

Phoenix6Driver::CustomMechanismPair::CustomMechanismPair(
    const std::string& name,
    rclcpp::Node* node,
    RclMotor<TalonFXS>* leader,
    RclMotor<TalonFXS>* follower,
    int update_period_ms,
    const bool* is_disabled,
    const rclcpp::Logger& logger,
    rclcpp::Clock::SharedPtr clock) :
    name(name),
    leader(leader),
    follower(follower),
    logger(logger),
    clock(clock),
    info_pub(node->create_publisher<TalonInfoMsg>(
        "lance/" + name + "/info",
        rclcpp::SensorDataQoS{})),
    faults_pub(node->create_publisher<TalonFaultsMsg>(
        "lance/" + name + "/faults",
        rclcpp::SensorDataQoS{})),
    ctrl_sub(node->create_subscription<TalonCtrlMsg>(
        "lance/" + name + "/ctrl",
        TALON_CTRL_SUB_QOS,
        [this](const TalonCtrlMsg& msg) { this->executeCtrl(msg); })),
    update_timer(node->create_wall_timer(
        update_period_ms * 1ms,
        [this]() { this->timerUpdate(); })),
    is_disabled(is_disabled)
{
}

double Phoenix6Driver::CustomMechanismPair::potPosition(
    RclMotor<TalonFXS>& motor) const
{
    const double volts = motor.motor.GetAnalogVoltage().GetValueAsDouble();
    const double raw = volts / motor.config.pot.max_v;
    return motor.config.pot.invert_sensor ? (1.0 - raw) : raw;
}

TalonInfoMsg Phoenix6Driver::CustomMechanismPair::motorInfo(
    RclMotor<TalonFXS>& motor) const
{
    TalonInfoMsg info{};
    info << motor.motor;
    info.status |= static_cast<uint8_t>(!(*is_disabled));
    info.position = potPosition(motor);
    return info;
}

uint8_t Phoenix6Driver::CustomMechanismPair::averageUint8(uint8_t a, uint8_t b)
{
    return static_cast<uint8_t>(
        (static_cast<uint16_t>(a) + static_cast<uint16_t>(b)) / 2U);
}

void Phoenix6Driver::CustomMechanismPair::publishInfo(
    const rclcpp::Time& stamp) const
{
    const TalonInfoMsg leader_info = motorInfo(*leader);
    const TalonInfoMsg follower_info = motorInfo(*follower);

    TalonInfoMsg info{};
    info.header.stamp = stamp;
    info.position = (leader_info.position + follower_info.position) / 2.0;
    info.velocity = (leader_info.velocity + follower_info.velocity) / 2.0;
    info.acceleration =
        (leader_info.acceleration + follower_info.acceleration) / 2.0;
    info.device_temp =
        (leader_info.device_temp + follower_info.device_temp) / 2.0F;
    info.processor_temp =
        (leader_info.processor_temp + follower_info.processor_temp) / 2.0F;
    info.bus_voltage =
        (leader_info.bus_voltage + follower_info.bus_voltage) / 2.0F;
    info.supply_current =
        (leader_info.supply_current + follower_info.supply_current) / 2.0F;
    info.output_percent =
        (leader_info.output_percent + follower_info.output_percent) / 2.0F;
    info.output_voltage =
        (leader_info.output_voltage + follower_info.output_voltage) / 2.0F;
    info.output_current =
        (leader_info.output_current + follower_info.output_current) / 2.0F;
    info.motor_state =
        averageUint8(leader_info.motor_state, follower_info.motor_state);
    info.bridge_mode =
        averageUint8(leader_info.bridge_mode, follower_info.bridge_mode);
    info.control_mode =
        averageUint8(leader_info.control_mode, follower_info.control_mode);
    info.status = averageUint8(leader_info.status, follower_info.status);

    info_pub->publish(info);
}

void Phoenix6Driver::CustomMechanismPair::publishFaults(
    const rclcpp::Time& stamp) const
{
    TalonFaultsMsg leader_faults{};
    TalonFaultsMsg follower_faults{};
    leader_faults << leader->motor;
    follower_faults << follower->motor;

    TalonFaultsMsg faults{};
    faults.header.stamp = stamp;
    faults.faults = leader_faults.faults | follower_faults.faults;
    faults.sticky_faults =
        leader_faults.sticky_faults | follower_faults.sticky_faults;

    faults.hardware_fault =
        leader_faults.hardware_fault || follower_faults.hardware_fault;
    faults.proc_temp_fault =
        leader_faults.proc_temp_fault || follower_faults.proc_temp_fault;
    faults.device_temp_fault =
        leader_faults.device_temp_fault || follower_faults.device_temp_fault;
    faults.undervoltage_fault =
        leader_faults.undervoltage_fault || follower_faults.undervoltage_fault;
    faults.boot_fault = leader_faults.boot_fault || follower_faults.boot_fault;
    faults.unliscensed_fault =
        leader_faults.unliscensed_fault || follower_faults.unliscensed_fault;
    faults.bridge_brownout_fault = leader_faults.bridge_brownout_fault ||
                                   follower_faults.bridge_brownout_fault;
    faults.overvoltage_fault =
        leader_faults.overvoltage_fault || follower_faults.overvoltage_fault;
    faults.unstable_voltage_fault = leader_faults.unstable_voltage_fault ||
                                    follower_faults.unstable_voltage_fault;
    faults.stator_current_limit_fault =
        leader_faults.stator_current_limit_fault ||
        follower_faults.stator_current_limit_fault;
    faults.supply_current_limit_fault =
        leader_faults.supply_current_limit_fault ||
        follower_faults.supply_current_limit_fault;
    faults.static_brake_disabled_fault =
        leader_faults.static_brake_disabled_fault ||
        follower_faults.static_brake_disabled_fault;

    faults.sticky_hardware_fault = leader_faults.sticky_hardware_fault ||
                                   follower_faults.sticky_hardware_fault;
    faults.sticky_proc_temp_fault = leader_faults.sticky_proc_temp_fault ||
                                    follower_faults.sticky_proc_temp_fault;
    faults.sticky_device_temp_fault = leader_faults.sticky_device_temp_fault ||
                                      follower_faults.sticky_device_temp_fault;
    faults.sticky_undervoltage_fault =
        leader_faults.sticky_undervoltage_fault ||
        follower_faults.sticky_undervoltage_fault;
    faults.sticky_boot_fault =
        leader_faults.sticky_boot_fault || follower_faults.sticky_boot_fault;
    faults.sticky_unliscensed_fault = leader_faults.sticky_unliscensed_fault ||
                                      follower_faults.sticky_unliscensed_fault;
    faults.sticky_bridge_brownout_fault =
        leader_faults.sticky_bridge_brownout_fault ||
        follower_faults.sticky_bridge_brownout_fault;
    faults.sticky_overvoltage_fault = leader_faults.sticky_overvoltage_fault ||
                                      follower_faults.sticky_overvoltage_fault;
    faults.sticky_unstable_voltage_fault =
        leader_faults.sticky_unstable_voltage_fault ||
        follower_faults.sticky_unstable_voltage_fault;
    faults.sticky_stator_current_limit_fault =
        leader_faults.sticky_stator_current_limit_fault ||
        follower_faults.sticky_stator_current_limit_fault;
    faults.sticky_supply_current_limit_fault =
        leader_faults.sticky_supply_current_limit_fault ||
        follower_faults.sticky_supply_current_limit_fault;
    faults.sticky_static_brake_disabled_fault =
        leader_faults.sticky_static_brake_disabled_fault ||
        follower_faults.sticky_static_brake_disabled_fault;

    faults_pub->publish(faults);
}

void Phoenix6Driver::CustomMechanismPair::logPotState(const char* context) const
{
    const double leader_volts =
        leader->motor.GetAnalogVoltage().GetValueAsDouble();
    const double follower_volts =
        follower->motor.GetAnalogVoltage().GetValueAsDouble();

    RCLCPP_INFO(
        logger,
        "CustomMechanism %s %s: %s analog=%.3f V position=%.3f; "
        "%s analog=%.3f V position=%.3f",
        name.c_str(),
        context,
        leader->name.c_str(),
        leader_volts,
        potPosition(*leader),
        follower->name.c_str(),
        follower_volts,
        potPosition(*follower));
}

double Phoenix6Driver::CustomMechanismPair::voltageLimit() const
{
    const double leader_limit = leader->config.voltage_limit;
    const double follower_limit = follower->config.voltage_limit;
    const double configured_limit = std::min(leader_limit, follower_limit);
    return configured_limit > 0.0 ? configured_limit : 12.0;
}

void Phoenix6Driver::CustomMechanismPair::setVoltage(
    RclMotor<TalonFXS>& motor,
    double voltage) const
{
    const double limit = voltageLimit();
    const double clamped_voltage = std::clamp(voltage, -limit, limit);
    auto control_status = motor.motor.SetControl(
        phx6::controls::VoltageOut{units::voltage::volt_t{clamped_voltage}}
            .WithEnableFOC(false));
    if (!control_status.IsOK())
    {
        RCLCPP_ERROR_THROTTLE(
            logger,
            *clock,
            5000,
            "Failed to send CustomMechanism voltage control to motor %s: "
            "%s (%d): %s",
            motor.name.c_str(),
            control_status.GetName(),
            static_cast<int>(control_status),
            control_status.GetDescription());
    }
}

void Phoenix6Driver::CustomMechanismPair::sendMirroredCtrl(
    const TalonCtrlMsg& msg)
{
    const TalonCtrlMsg mirrored_msg = msg;
    std::cout << "PRESET Mode: " << static_cast<int>(mirrored_msg.mode)
          << ", Value: " << mirrored_msg.value << std::endl;

    auto leader_status = leader->motor << mirrored_msg;
    if (!leader_status.IsOK())
    {
        RCLCPP_ERROR_THROTTLE(
            logger,
            *clock,
            5000,
            "Failed to send CustomMechanism control mode %d to motor %s: "
            "%s (%d): %s",
            mirrored_msg.mode,
            leader->name.c_str(),
            leader_status.GetName(),
            static_cast<int>(leader_status),
            leader_status.GetDescription());
    }

    auto follower_status = follower->motor << mirrored_msg;
    if (!follower_status.IsOK())
    {
        RCLCPP_ERROR_THROTTLE(
            logger,
            *clock,
            5000,
            "Failed to send CustomMechanism control mode %d to motor %s: "
            "%s (%d): %s",
            mirrored_msg.mode,
            follower->name.c_str(),
            follower_status.GetName(),
            static_cast<int>(follower_status),
            follower_status.GetDescription());
    }
}

void Phoenix6Driver::CustomMechanismPair::setPosition(
    double target_position) const
{
    const double leader_error = target_position - potPosition(*leader);
    const double follower_error = target_position - potPosition(*follower);

    setVoltage(*leader, leader->config.kP * leader_error);
    setVoltage(*follower, follower->config.kP * follower_error);
}

void Phoenix6Driver::CustomMechanismPair::setVelocity(
    double target_velocity) const
{
    const double leader_position = potPosition(*leader);
    const double follower_position = potPosition(*follower);
    const double balance_error = follower_position - leader_position;

    const double base_voltage =
        ((leader->config.kV + follower->config.kV) / 2.0) * target_velocity;
    const double balance_voltage =
        ((leader->config.kP + follower->config.kP) / 4.0) * balance_error;

    setVoltage(*leader, base_voltage + balance_voltage);
    setVoltage(*follower, base_voltage - balance_voltage);
}

bool Phoenix6Driver::CustomMechanismPair::executeCtrl(const TalonCtrlMsg& msg)
{
    switch (msg.mode)
    {
        case TalonCtrlMsg::POSITION:
        case TalonCtrlMsg::VELOCITY:
            logPotState("before control");
            active_ctrl = msg;
            update();
            return true;
        case TalonCtrlMsg::DISABLED:
            active_ctrl.reset();
            leader->motor.SetControl(phx6::controls::NeutralOut{});
            follower->motor.SetControl(phx6::controls::NeutralOut{});
            return true;
        case TalonCtrlMsg::PERCENT_OUTPUT:
        case TalonCtrlMsg::VOLTAGE:
        case TalonCtrlMsg::MUSIC_TONE:
            logPotState("before mirrored control");
            active_ctrl.reset();
            sendMirroredCtrl(msg);
            return true;
        default:
            RCLCPP_ERROR(
                logger,
                "CustomMechanism does not support control mode %d",
                msg.mode);
            return true;
    }
}

void Phoenix6Driver::CustomMechanismPair::disable()
{
    active_ctrl.reset();
    leader->motor.SetControl(phx6::controls::NeutralOut{});
    follower->motor.SetControl(phx6::controls::NeutralOut{});
}

void Phoenix6Driver::CustomMechanismPair::update()
{
    if (!active_ctrl)
    {
        return;
    }

    switch (active_ctrl->mode)
    {
        case TalonCtrlMsg::POSITION:
            setPosition(active_ctrl->value);
            break;
        case TalonCtrlMsg::VELOCITY:
            setVelocity(active_ctrl->value);
            break;
        default:
            break;
    }
}

void Phoenix6Driver::CustomMechanismPair::timerUpdate()
{
    if (is_disabled && *is_disabled)
    {
        return;
    }

    update();
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
                config.name,
                this,
                leader,
                follower,
                config.update_period_ms,
                &is_disabled,
                get_logger(),
                get_clock());
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
