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
    using V = phx6::signals::ExternalFeedbackSensorSourceValue;

    enum class SensorSource : int
    {
        // TalonFXS ExternalFeedbackSensorSourceValue passthrough
        Commutation = V::Commutation.value,
        QuadratureEncoder = V::QuadratureEncoder.value,
        PulseWidthEncoder = V::PulseWidthEncoder.value,
        RemoteCANcoder = V::RemoteCANcoder.value,
        FusedCANcoder = V::FusedCANcoder.value,  // Pro
        SyncCANcoder = V::SyncCANcoder.value,    // Pro
        RemotePigeon2Yaw = V::RemotePigeon2Yaw.value,
        RemotePigeon2Pitch = V::RemotePigeon2Pitch.value,
        RemotePigeon2Roll = V::RemotePigeon2Roll.value,
        FusedPigeon2Yaw = V::FusedPigeon2Yaw.value,      // Pro
        FusedPigeon2Pitch = V::FusedPigeon2Pitch.value,  // Pro
        FusedPigeon2Roll = V::FusedPigeon2Roll.value,    // Pro

        // Software handled from Gadgeteer (10 pin) input
        AnalogPotentiometer = 100,
    };

    struct RclMotorConfig
    {
        int follows_id;
        phx6::signals::MotorAlignmentValue alignment;
        SensorSource sensor;

        union
        {
            int remote_sensor_id;  // For any sensor that requires an ID: Remote*, Fused*, or Sync*
            struct
            {
                int pot_max_v;  // For AnalogPotentiometer sensor config
                bool invert_sensor;
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
        static_assert(
            std::is_same_v<MotorType, TalonFX> ||
                std::is_same_v<MotorType, TalonFXS>,
            "RclMotor only supports TalonFX and TalonFXS");

        using ConfiguratorT = std::conditional_t<
            std::is_same_v<MotorType, TalonFX>,
            phx6::TalonFXConfigurator,
            phx6::TalonFXSConfigurator>;

        MotorType motor;
        RclMotorConfig config;
        SharedPub<TalonInfoMsg> info_pub;
        SharedPub<TalonFaultsMsg> faults_pub;
        SharedSub<TalonCtrlMsg> ctrl_sub;

        RclMotor(
            rclcpp::Node* node,
            const std::string& name,
            int id,
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
