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

#pragma once

#include <string>
#include <optional>
#include <functional>

#include <rclcpp/rclcpp.hpp>

#include "ros_utils.hpp"
#include "phx6_utils.hpp"


using phx6::signals::MotorAlignmentValue;
using phx6::signals::MotorArrangementValue;


class Phoenix6Base
{
    using FeedbackSensor = phx6::signals::ExternalFeedbackSensorSourceValue;

public:
    enum class SensorSource : int
    {
        // TalonFXS ExternalFeedbackSensorSourceValue passthrough
        Commutation = FeedbackSensor::Commutation,
        QuadratureEncoder = FeedbackSensor::Quadrature,
        PulseWidthEncoder = FeedbackSensor::PulseWidth,
        RemoteCANcoder = FeedbackSensor::RemoteCANcoder,
        FusedCANcoder = FeedbackSensor::FusedCANcoder,  // Pro
        SyncCANcoder = FeedbackSensor::SyncCANcoder,    // Pro
        RemotePigeon2Yaw = FeedbackSensor::RemotePigeon2Yaw,
        RemotePigeon2Pitch = FeedbackSensor::RemotePigeon2Pitch,
        RemotePigeon2Roll = FeedbackSensor::RemotePigeon2Roll,
        // FusedPigeon2Yaw = FeedbackSensor::FusedPigeon2Yaw,      // Pro
        // FusedPigeon2Pitch = FeedbackSensor::FusedPigeon2Pitch,  // Pro
        // FusedPigeon2Roll = FeedbackSensor::FusedPigeon2Roll,    // Pro

        // Software handled from Gadgeteer (10 pin) input
        AnalogPotentiometer = 100
    };

public:
    struct RclMotorConfig
    {
        double kP;
        double kI;
        double kD;
        double kV;
        double neutral_deadband;
        double stator_current_limit;
        double supply_current_limit;
        double voltage_limit;

        int follows_id;
        std::string follower_type;
        MotorAlignmentValue alignment;
        MotorArrangementValue motor_arrangement;

        SensorSource sensor;
        union
        {
            int remote_sensor_id;  // For any sensor that requires an ID: Remote*, Fused*, or Sync*
            struct
            {
                double max_v;
                bool invert_sensor;
            } pot;  // For AnalogPotentiometer sensor config
        };

        bool output_inverted;
        bool neutral_brake;
        bool temp_sensor_required;
    };

public:
    template<typename T>
    struct RclMotor : public util::UsingRosAliases
    {
        static_assert(
            std::is_same_v<T, TalonFX> || std::is_same_v<T, TalonFXS>,
            "RclMotor only supports TalonFX and TalonFXS");

        using MotorT = T;
        using ConfigurationT = std::conditional_t<
            std::is_same_v<T, TalonFX>,
            TalonFXConfiguration,
            TalonFXSConfiguration>;

    public:
        std::string name;
        RclMotorConfig config;
        MotorT motor;

        rclcpp::Logger logger;
        RclClock::SharedPtr clock;
        RclPubPtr<TalonInfoMsg> info_pub;
        RclPubPtr<TalonFaultsMsg> faults_pub;
        RclSubPtr<TalonCtrlMsg> ctrl_sub;

        std::function<bool(const TalonCtrlMsg&)> custom_ctrl_handler;

    public:
        RclMotor(
            rclcpp::Node* node,
            const std::string& name,
            int id,
            const RclMotorConfig& config,
            const CANBus& bus,
            bool enable_ctrl_sub);

        void executeCtrl(const TalonCtrlMsg& msg);
    };

public:
    struct MechanismConfig
    {
        std::string name;
        std::string type;
        std::vector<std::string> motors;
        MotorAlignmentValue alignment;

        double sensor_to_differential_ratio;
        double closed_loop_rate_hz;
        int update_period_ms;
    };

public:
    struct CustomMechanismPair : public util::UsingRosAliases
    {
    public:
        std::string name;
        RclMotor<TalonFXS>* leader;
        RclMotor<TalonFXS>* follower;

        RclLogger logger;
        RclClock::SharedPtr clock;

        RclPubPtr<TalonInfoMsg> info_pub;
        RclPubPtr<TalonFaultsMsg> faults_pub;
        RclSubPtr<TalonCtrlMsg> ctrl_sub;
        RclTimer::SharedPtr update_timer;

        const bool* is_disabled;
        std::optional<TalonCtrlMsg> active_ctrl;

    public:
        CustomMechanismPair(
            rclcpp::Node* node,
            const std::string& name,
            RclMotor<TalonFXS>* leader,
            RclMotor<TalonFXS>* follower,
            int update_period_ms,
            const bool* is_disabled);

    public:
        void publishInfo(const rclcpp::Time& stamp) const;
        void publishFaults(const rclcpp::Time& stamp) const;
        void logPotState(const char* context) const;

        double voltageLimit() const;
        void setVoltage(RclMotor<TalonFXS>& motor, double voltage) const;
        void sendMirroredCtrl(const TalonCtrlMsg& msg);
        void setPosition(double target_position) const;
        void setVelocity(double target_velocity) const;

        bool executeCtrl(const TalonCtrlMsg& msg);
        void disable();
        void update();
        void timerUpdate();
    };
};
