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

#include <vector>
#include <string>

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

class Phoenix6Driver : public rclcpp::Node
{
public:
    using Int32Msg = std_msgs::msg::Int32;

    enum class SensorSource
    {
        // TalonFXS ExternalFeedbackSensorSourceValue passthrough
        Commutation,
        QuadratureEncoder,
        PulseWidthEncoder,
        RemoteCANcoder,
        FusedCANcoder,  // Pro
        SyncCANcoder,   // Pro
        RemotePigeon2Yaw,
        RemotePigeon2Pitch,
        RemotePigeon2Roll,
        FusedPigeon2Yaw,    // Pro
        FusedPigeon2Pitch,  // Pro
        FusedPigeon2Roll,   // Pro

        // Software handled from Gadgeteer (10 pin) input
        AnalogPotentiometer,
    };

    struct RclMotorConfig
    {
        int follows_id;
        phx6::signals::MotorAlignmentValue alignment;
        SensorSource sensor;

        union
        {
            int remote_sensor_id; // For any sensor that requires an ID: Remote*, Fused*, or Sync*
            struct
            {
                int pot_max_v; // For AnalogPotentiometer sensor config
                int pot_min_v;
            };
        };

        double kP;
        double kI;
        double kD;
        double kV;
        double neutral_deadband;
        bool neutral_brake;
        double stator_current_limit;
        double supply_current_limit;
        double voltage_limit;
    };


    template<typename MotorType>
    struct RclMotor
    {
        MotorType motor;
        SharedPub<TalonInfoMsg> info_pub;
        SharedPub<TalonFaultsMsg> faults_pub;
        SharedSub<TalonCtrlMsg> ctrl_sub;

        RclMotor(
            rclcpp::Node* node,
            const std::string& name,
            int id,
            int followsId,
            const std::string& sensor,
            const RclMotorConfig& config,
            const CANBus& bus);

        void executeCtrl(const TalonCtrlMsg& msg);
    };

public:
    Phoenix6Driver();
    ~Phoenix6Driver();

private:
    void feedWatchdogStatus(int32_t status);

    void pubMotorInfo_cb();
    void pubMotorFault_cb();

private:
    CANBus bus;
    int diagnostic_server_port;

    std::vector<std::unique_ptr<RclMotor<TalonFX>>> FX_motors;
    std::vector<std::unique_ptr<RclMotor<TalonFXS>>> FXS_motors;

    SharedSub<Int32Msg> watchdog_status_sub;

    RclTimer info_pub_timer;
    RclTimer fault_pub_timer;

    bool is_disabled = false;
};

// --- Helper functions ------------------------------------------------------------

// Coverts from our SensorSource enum to Phoenix's ExternalFeedbackSensorSourceValue enum unless the sensor is software handled
// Returns std::nullopt if the source is not a valid Phoenix sensor source (e.g. AnalogPotentiometer)
static std::optional<phx6::signals::ExternalFeedbackSensorSourceValue>
    toPhoenixSensorSource(Phoenix6Driver::SensorSource source)
{
    using S = Phoenix6Driver::SensorSource;
    using V = phx6::signals::ExternalFeedbackSensorSourceValue;

    static const std::unordered_map<S, V> kMap = {
        {       S::Commutation,         V::Commutation},
        { S::QuadratureEncoder,   V::QuadratureEncoder},
        { S::PulseWidthEncoder,   V::PulseWidthEncoder},
        {    S::RemoteCANcoder,      V::RemoteCANcoder},
        {     S::FusedCANcoder,       V::FusedCANcoder},
        {      S::SyncCANcoder,        V::SyncCANcoder},
        {  S::RemotePigeon2Yaw,   V::RemotePigeon2_Yaw},
        {S::RemotePigeon2Pitch, V::RemotePigeon2_Pitch},
        { S::RemotePigeon2Roll,  V::RemotePigeon2_Roll},
        {   S::FusedPigeon2Yaw,    V::FusedPigeon2_Yaw},
        { S::FusedPigeon2Pitch,  V::FusedPigeon2_Pitch},
        {  S::FusedPigeon2Roll,   V::FusedPigeon2_Roll},
        // AnalogPotentiometer intentionally absent
    };

    auto it = kMap.find(source);
    if (it == kMap.end())
    {
        return std::nullopt;
    }
    return it->second;
}

// --- Implementation -------------------------------------------------------------

template<typename MotorType>
Phoenix6Driver::RclMotor<MotorType>::RclMotor(
    rclcpp::Node* node,
    const std::string& name,
    int id,
    const RclMotorConfig& config,
    const CANBus& bus) :
    motor(id, bus),
    info_pub(node->template create_publisher<TalonInfoMsg>(
        "lance/" + name + "/info",
        10)),
    faults_pub(node->template create_publisher<TalonFaultsMsg>(
        "lance/" + name + "/faults",
        10)),
    ctrl_sub(node->create_subscription<TalonCtrlMsg>(
        "lance/" + name + "/ctrl",
        10,
        [this](const TalonCtrlMsg& msg) { this->executeCtrl(msg); }))

{
    // --- Init motor -------------------------------------------------------------
    if constexpr (std::is_same_v<MotorType, TalonFX>)
    {
        TalonFXConfiguration FXconfig = buildMotorConfig(
            config.kP,
            config.kI,
            config.kD,
            config.kV,
            config.neutral_deadband,
            config.neutral_brake,
            config.stator_current_limit,
            config.supply_current_limit,
            config.voltage_limit);

        motor.GetConfigurator().Apply(FXconfig);

        if (config.follows_id != -1)
        {
            ctre::phoenix6::controls::Follower followerCtrl(
                config.follows_id,
                config.alignment);
            motor.SetControl(followerCtrl);
        }
    }
    else if constexpr (std::is_same_v<MotorType, TalonFXS>)
    {
        TalonFXSConfiguration FXSconfig = buildMotorConfig(
            config.kP,
            config.kI,
            config.kD,
            config.kV,
            config.neutral_deadband,
            config.neutral_brake,
            config.stator_current_limit,
            config.supply_current_limit,
            config.voltage_limit);

        // TalonFXS supports external sensors — configure feedback source if provided
        if (toPhoenixSensorSource(config.sensor))
        {
            FXSconfig.WithFeedback(
                FeedbackConfigs{}.WithExternalSensorSource(
                    *toPhoenixSensorSource(config.sensor)));
        }

        motor.GetConfigurator().Apply(FXSconfig);

        if (config.follows_id != -1)
        {
            ctre::phoenix6::controls::Follower followerCtrl(
                config.follows_id,
                config.alignment);
            motor.SetControl(followerCtrl);
        }
    }
}

template<typename MotorType>
void Phoenix6Driver::RclMotor<MotorType>::executeCtrl(const TalonCtrlMsg& msg)
{
    motor << msg;

    // TODO possible add dynamic load gains, or FOC
    // TODO add new functionality for seprate slot for pos control

    // if constexpr (std::is_same_v<MotorType, TalonFX>)
    // {
    // }
    // else if constexpr (std::is_same_v<MotorType, TalonFXS>)
    // {
    // }
}
