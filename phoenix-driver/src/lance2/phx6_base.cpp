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

#include "phx6_base.hpp"

#include <algorithm>
#include <unordered_set>

#include <ctre/phoenix6/controls/DifferentialFollower.hpp>

#include "common.hpp"


static bool isSupportedTalonCtrlMode(int8_t mode)
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

static double getPotVal(Phoenix6Base::RclMotor<TalonFXS>& motor)
{
    const double volts = motor.motor.GetAnalogVoltage().GetValueAsDouble();
    const double raw = volts / motor.config.pot.max_v;
    return motor.config.pot.invert_sensor ? (1.0 - raw) : raw;
}

static TalonInfoMsg motorInfoWithPotPosition(
    Phoenix6Base::RclMotor<TalonFXS>& motor,
    bool is_disabled)
{
    TalonInfoMsg info{};
    info << motor.motor;
    info.status |= static_cast<uint8_t>(!is_disabled);
    info.position = getPotVal(motor);
    return info;
}

static inline uint8_t averageUint8(uint8_t a, uint8_t b)
{
    return static_cast<uint8_t>(
        (static_cast<uint16_t>(a) + static_cast<uint16_t>(b)) / 2U);
}





// --- RclMotor implementation --------------------------------------------------

template<typename MotorType>
Phoenix6Base::RclMotor<MotorType>::RclMotor(
    rclcpp::Node* node,
    const std::string& name,
    int id,
    const RclMotorConfig& config,
    const CANBus& bus,
    bool enable_ctrl_sub) :
    name(name),
    config(config),
    motor(id, bus),
    logger(node->get_logger()),
    clock(node->get_clock()),
    info_pub(node->template create_publisher<TalonInfoMsg>(
        ROBOT_TOPIC("") + name + "/info",
        rclcpp::SensorDataQoS{})),
    faults_pub(node->template create_publisher<TalonFaultsMsg>(
        ROBOT_TOPIC("") + name + "/faults",
        rclcpp::SensorDataQoS{}))

{
    if (enable_ctrl_sub)
    {
        ctrl_sub = node->create_subscription<TalonCtrlMsg>(
            ROBOT_TOPIC("") + name + "/ctrl",
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
        config.output_inverted,
        config.stator_current_limit,
        config.supply_current_limit,
        config.voltage_limit);

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

        if (config.sensor != Phoenix6Base::SensorSource::AnalogPotentiometer)
        {
            ExternalFeedbackConfigs feedback{};
            feedback.WithExternalFeedbackSensorSource(
                static_cast<int>(config.sensor));
            if (config.remote_sensor_id > -1)
            {
                feedback.WithFeedbackRemoteSensorID(config.remote_sensor_id);
            }
            phxConfig.WithExternalFeedback(feedback);
        }
        // else: AnalogPotentiometer uses default Commutation; position computed in publishInfo()

        // phxConfig.Commutation.MotorArrangement =
        //     phx6::signals::MotorArrangementValue::Brushed_DC;
        // phxConfig.ExternalTemp.TempSensorRequired =
        //     phx6::signals::TempSensorRequiredValue::Not_Required;
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
void Phoenix6Base::RclMotor<MotorType>::executeCtrl(const TalonCtrlMsg& msg)
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
template class Phoenix6Base::RclMotor<TalonFX>;
template class Phoenix6Base::RclMotor<TalonFXS>;





// --- CustomMechanism ---------------------------------------------------------

Phoenix6Base::CustomMechanismPair::CustomMechanismPair(
    rclcpp::Node* node,
    const std::string& name,
    RclMotor<TalonFXS>* leader,
    RclMotor<TalonFXS>* follower,
    int update_period_ms,
    const bool* is_disabled) :
    name(name),
    leader(leader),
    follower(follower),
    logger(node->get_logger()),
    clock(node->get_clock()),
    info_pub(node->create_publisher<TalonInfoMsg>(
        ROBOT_TOPIC("") + name + "/info",
        rclcpp::SensorDataQoS{})),
    faults_pub(node->create_publisher<TalonFaultsMsg>(
        ROBOT_TOPIC("") + name + "/faults",
        rclcpp::SensorDataQoS{})),
    ctrl_sub(node->create_subscription<TalonCtrlMsg>(
        ROBOT_TOPIC("") + name + "/ctrl",
        TALON_CTRL_SUB_QOS,
        [this](const TalonCtrlMsg& msg) { this->executeCtrl(msg); })),
    update_timer(node->create_wall_timer(
        std::chrono::milliseconds(update_period_ms),
        [this]() { this->timerUpdate(); })),
    is_disabled(is_disabled)
{
}

void Phoenix6Base::CustomMechanismPair::publishInfo(
    const rclcpp::Time& stamp) const
{
    const TalonInfoMsg leader_info =
        motorInfoWithPotPosition(*leader, *is_disabled);
    const TalonInfoMsg follower_info =
        motorInfoWithPotPosition(*follower, *is_disabled);

#define AVG_FIELDS(field) ((leader_info.field + follower_info.field) / 2)
#define AVG_U8_FIELDS(field)                             \
    averageUint8(leader_info.field, follower_info.field)

    TalonInfoMsg info{};
    info.header.stamp = stamp;
    info.position = AVG_FIELDS(position);
    info.velocity = AVG_FIELDS(velocity);
    info.acceleration = AVG_FIELDS(acceleration);
    info.device_temp = AVG_FIELDS(device_temp);
    info.processor_temp = AVG_FIELDS(processor_temp);
    info.bus_voltage = AVG_FIELDS(bus_voltage);
    info.supply_current = AVG_FIELDS(supply_current);
    info.output_percent = AVG_FIELDS(output_percent);
    info.output_voltage = AVG_FIELDS(output_voltage);
    info.output_current = AVG_FIELDS(output_current);
    info.motor_state = AVG_U8_FIELDS(motor_state);
    info.bridge_mode = AVG_U8_FIELDS(bridge_mode);
    info.control_mode = AVG_U8_FIELDS(control_mode);
    info.status = AVG_U8_FIELDS(status);

#undef AVG_FIELDS
#undef AVG_U8_FIELDS

    info_pub->publish(info);
}

void Phoenix6Base::CustomMechanismPair::publishFaults(
    const rclcpp::Time& stamp) const
{
    TalonFaultsMsg leader_faults{}, follower_faults{};
    leader_faults << leader->motor;
    follower_faults << follower->motor;

#define OR_BITS_FIELDS(field) (leader_faults.field | follower_faults.field)
#define OR_BOOL_FIELDS(field) (leader_faults.field || follower_faults.field)

    TalonFaultsMsg faults{};
    faults.header.stamp = stamp;
    faults.faults = OR_BITS_FIELDS(faults);
    faults.sticky_faults = OR_BITS_FIELDS(sticky_faults);

    faults.hardware_fault = OR_BOOL_FIELDS(hardware_fault);
    faults.proc_temp_fault = OR_BOOL_FIELDS(proc_temp_fault);
    faults.device_temp_fault = OR_BOOL_FIELDS(device_temp_fault);
    faults.undervoltage_fault = OR_BOOL_FIELDS(undervoltage_fault);
    faults.boot_fault = OR_BOOL_FIELDS(boot_fault);
    faults.unliscensed_fault = OR_BOOL_FIELDS(unliscensed_fault);
    faults.bridge_brownout_fault = OR_BOOL_FIELDS(bridge_brownout_fault);
    faults.overvoltage_fault = OR_BOOL_FIELDS(overvoltage_fault);
    faults.unstable_voltage_fault = OR_BOOL_FIELDS(unstable_voltage_fault);
    faults.stator_current_limit_fault =
        OR_BOOL_FIELDS(stator_current_limit_fault);
    faults.supply_current_limit_fault =
        OR_BOOL_FIELDS(supply_current_limit_fault);
    faults.static_brake_disabled_fault =
        OR_BOOL_FIELDS(static_brake_disabled_fault);

    faults.sticky_hardware_fault = OR_BOOL_FIELDS(sticky_hardware_fault);
    faults.sticky_proc_temp_fault = OR_BOOL_FIELDS(sticky_proc_temp_fault);
    faults.sticky_device_temp_fault = OR_BOOL_FIELDS(sticky_device_temp_fault);
    faults.sticky_undervoltage_fault =
        OR_BOOL_FIELDS(sticky_undervoltage_fault);
    faults.sticky_boot_fault = OR_BOOL_FIELDS(sticky_boot_fault);
    faults.sticky_unliscensed_fault = OR_BOOL_FIELDS(sticky_unliscensed_fault);
    faults.sticky_bridge_brownout_fault =
        OR_BOOL_FIELDS(sticky_bridge_brownout_fault);
    faults.sticky_overvoltage_fault = OR_BOOL_FIELDS(sticky_overvoltage_fault);
    faults.sticky_unstable_voltage_fault =
        OR_BOOL_FIELDS(sticky_unstable_voltage_fault);
    faults.sticky_stator_current_limit_fault =
        OR_BOOL_FIELDS(sticky_stator_current_limit_fault);
    faults.sticky_supply_current_limit_fault =
        OR_BOOL_FIELDS(sticky_supply_current_limit_fault);
    faults.sticky_static_brake_disabled_fault =
        OR_BOOL_FIELDS(sticky_static_brake_disabled_fault);

#undef OR_BITS_FIELDS
#undef OR_BOOL_FIELDS

    faults_pub->publish(faults);
}

void Phoenix6Base::CustomMechanismPair::logPotState(const char* context) const
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
        getPotVal(*leader),
        follower->name.c_str(),
        follower_volts,
        getPotVal(*follower));
}

double Phoenix6Base::CustomMechanismPair::voltageLimit() const
{
    const double leader_limit = leader->config.voltage_limit;
    const double follower_limit = follower->config.voltage_limit;
    if (leader_limit > 0.0 && follower_limit > 0.0)
    {
        return std::min(leader_limit, follower_limit);
    }
    else if (leader_limit > 0.0)
    {
        return leader_limit;
    }
    else if (follower_limit > 0.0)
    {
        return follower_limit;
    }
    return 0.0;
}

void Phoenix6Base::CustomMechanismPair::setVoltage(
    RclMotor<TalonFXS>& motor,
    double voltage) const
{
    const double limit = voltageLimit();
    const double clamped_voltage =
        limit > 0.0 ? std::clamp(voltage, -limit, limit) : voltage;
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

void Phoenix6Base::CustomMechanismPair::sendMirroredCtrl(
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

void Phoenix6Base::CustomMechanismPair::setPosition(
    double target_position) const
{
    const double leader_error = target_position - getPotVal(*leader);
    const double follower_error = target_position - getPotVal(*follower);

    setVoltage(*leader, leader->config.kP * leader_error);
    setVoltage(*follower, follower->config.kP * follower_error);
}

void Phoenix6Base::CustomMechanismPair::setVelocity(
    double target_velocity) const
{
    const double leader_position = getPotVal(*leader);
    const double follower_position = getPotVal(*follower);
    const double balance_error = follower_position - leader_position;

    const double base_voltage =
        ((leader->config.kV + follower->config.kV) / 2.0) * target_velocity;
    const double balance_voltage =
        ((leader->config.kP + follower->config.kP) / 4.0) * balance_error;

    setVoltage(*leader, base_voltage + balance_voltage);
    setVoltage(*follower, base_voltage - balance_voltage);
}

bool Phoenix6Base::CustomMechanismPair::executeCtrl(const TalonCtrlMsg& msg)
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

void Phoenix6Base::CustomMechanismPair::disable()
{
    active_ctrl.reset();
    leader->motor.SetControl(phx6::controls::NeutralOut{});
    follower->motor.SetControl(phx6::controls::NeutralOut{});
}

void Phoenix6Base::CustomMechanismPair::update()
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

void Phoenix6Base::CustomMechanismPair::timerUpdate()
{
    if (is_disabled && *is_disabled)
    {
        return;
    }

    update();
}
