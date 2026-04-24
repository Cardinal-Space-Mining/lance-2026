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

#include <functional>
#include <vector>
#include <string>
#include <unordered_map>
#include <unordered_set>

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
        Commutation = V::Commutation,
        QuadratureEncoder = V::Quadrature,
        PulseWidthEncoder = V::PulseWidth,
        RemoteCANcoder = V::RemoteCANcoder,
        FusedCANcoder = V::FusedCANcoder,  // Pro
        SyncCANcoder = V::SyncCANcoder,    // Pro
        RemotePigeon2Yaw = V::RemotePigeon2Yaw,
        RemotePigeon2Pitch = V::RemotePigeon2Pitch,
        RemotePigeon2Roll = V::RemotePigeon2Roll,
        // FusedPigeon2Yaw = V::FusedPigeon2Yaw,      // Pro
        // FusedPigeon2Pitch = V::FusedPigeon2Pitch,  // Pro
        // FusedPigeon2Roll = V::FusedPigeon2Roll,    // Pro

        // Software handled from Gadgeteer (10 pin) input
        AnalogPotentiometer = 100,
    };

    struct RclMotorConfig
    {
        int follows_id;
        std::string follower_type;
        phx6::signals::MotorAlignmentValue alignment;
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

    struct MechanismConfig
    {
        std::string name;
        std::string type;
        std::vector<std::string> motors;
        phx6::signals::MotorAlignmentValue alignment;
        double sensor_to_differential_ratio;
        double closed_loop_rate_hz;
    };


    template<typename MotorType>
    struct RclMotor
    {
        static_assert(
            std::is_same_v<MotorType, TalonFX> ||
                std::is_same_v<MotorType, TalonFXS>,
            "RclMotor only supports TalonFX and TalonFXS");

        using ConfigurationT = std::conditional_t<
            std::is_same_v<MotorType, TalonFX>,
            phx6::configs::TalonFXConfiguration,
            phx6::configs::TalonFXSConfiguration>;

        MotorType motor;
        std::string name;
        rclcpp::Logger logger;
        rclcpp::Clock::SharedPtr clock;
        RclMotorConfig config;
        SharedPub<TalonInfoMsg> info_pub;
        SharedPub<TalonFaultsMsg> faults_pub;
        SharedSub<TalonCtrlMsg> ctrl_sub;
        std::function<bool(const TalonCtrlMsg&)> custom_ctrl_handler;

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
    Phoenix6Driver();
    ~Phoenix6Driver();

private:
    void feedWatchdogStatus(int32_t status);

    void parseMechanismConfigs();
    void setupMechanisms();
    void updateCustomMechanisms();

    void setupMotors();

    void pubMotorInfo_cb();
    void pubMotorFault_cb();

private:
    struct CustomMechanismPair;

    std::string bus_name;
    CANBus bus;
    int diagnostic_server_port;

    std::vector<std::unique_ptr<RclMotor<TalonFX>>> FX_motors;
    std::vector<std::unique_ptr<RclMotor<TalonFXS>>> FXS_motors;
    std::vector<std::unique_ptr<CustomMechanismPair>> custom_mechanisms;
    std::vector<MechanismConfig> mechanism_configs;
    std::unordered_set<std::string> mechanism_motor_names;
    std::unordered_map<std::string, RclMotor<TalonFX>*> FX_motors_by_name;
    std::unordered_map<std::string, RclMotor<TalonFXS>*> FXS_motors_by_name;

    SharedSub<Int32Msg> watchdog_status_sub;

    RclTimer info_pub_timer;
    RclTimer fault_pub_timer;
    RclTimer custom_mechanism_timer;

    bool is_disabled = false;
};
