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

#include <vector>
#include <algorithm>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/string.hpp>

#define Phoenix_No_WPI  // remove WPI dependencies
#include <ctre/phoenix/cci/Diagnostics_CCI.h>
#include <ctre/phoenix6/TalonFX.hpp>
#include <ctre/phoenix6/unmanaged/Unmanaged.hpp>

#include "ros_utils.hpp"
#include "lance2_phx6_utils.hpp"

using namespace util;
using namespace util::ros_aliases;
using namespace std::chrono_literals;

Phoenix6Driver::Phoenix6Driver() :
    Node("lance2_phoenix6_driver"),
    bus(declare_and_get_param(
        *this,
        "can_interface",
        std::string("canable_A"))),
    diagnostic_server_port(declare_and_get_param(*this, "diagnostics_port", 0)),
    watchdog_status_sub{this->create_subscription<Int32Msg>(
        "lance/watchdog_status",
        rclcpp::SensorDataQoS{},
        [this](const Int32Msg& msg) { this->feedWatchdogStatus(msg.data); })},
    info_pub_timer{this->create_wall_timer(
        declare_and_get_param(*this, "info_pub_rate_mexts", 50) * 1ms,
        [this]() { this->pubMotorInfo_cb(); })},
    fault_pub_timer{this->create_wall_timer(
        declare_and_get_param(*this, "info_fault_rate_ms", 250) * 1ms,
        [this]() { this->pubMotorFault_cb(); })}
{
    // --- Get motors params-------------------------------------------------------------
    auto motor_names =
        declare_and_get_param<std::vector<std::string>>(*this, "names", {});

    std::unordered_map<std::string, int> ids;
    for (auto name : motor_names)
    {
        const std::string param_prefix = "motors." + name + ".";
        int id = declare_and_get_param(*this, param_prefix + "id", -1);

        ids.emplace(name, id);
    }

    for (auto name : motor_names)
    {
        const std::string param_prefix = "motors." + name + ".";

        std::string controller = declare_and_get_param<std::string>(
            *this,
            param_prefix + "controller",
            "");
        int id = declare_and_get_param(*this, param_prefix + "id", -1);

        if (controller.empty() || id == -1)
        {
            RCLCPP_ERROR(
                get_logger(),
                "Motor %s is missing required parameters",
                name.c_str());
            continue;
        }

        std::string follows = declare_and_get_param<std::string>(
            *this,
            param_prefix + "follows",
            "");
        int followsId = follows.empty() ? -1 : ids.at(follows);
        std::string sensor = declare_and_get_param<std::string>(
            *this,
            param_prefix + "sensor",
            "");

        RclMotorConfig params;

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

        if (controller == "FX")
        {
            FX_motors.emplace_back(
                std::make_unique<RclMotor<TalonFX>>(
                    this,
                    name,
                    id,
                    followsId,
                    sensor,
                    params,
                    bus));
        }
        else if (controller == "FXS")
        {
            FXS_motors.emplace_back(
                std::make_unique<RclMotor<TalonFXS>>(
                    this,
                    name,
                    id,
                    followsId,
                    sensor,
                    params,
                    bus));
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
}

Phoenix6Driver::~Phoenix6Driver()
{
    c_Phoenix_Diagnostics_Dispose();
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
    TalonInfoMsg info_msg{};
    info_msg.header.stamp = this->get_clock()->now();
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
        m->info_pub->publish(info_msg);
    }
}

void Phoenix6Driver::pubMotorFault_cb()
{
    TalonFaultsMsg faults_msg{};
    faults_msg.header.stamp = this->get_clock()->now();

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
