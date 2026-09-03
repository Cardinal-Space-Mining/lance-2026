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

#include <memory>
#include <string>
#include <vector>
#include <unordered_map>
#include <unordered_set>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>

#include <csm_utils/ros_utils.hpp>
#include "phx6_base.hpp"
#include "phx6_utils.hpp"


class Phoenix6Driver :
    public rclcpp::Node,
    public Phoenix6Base,
    private util::UsingRosAliases
{
public:
    using Int32Msg = std_msgs::msg::Int32;

public:
    Phoenix6Driver();
    ~Phoenix6Driver();

private:
    void feedWatchdogStatus(int32_t status);

    void parseMechanismConfigs();
    void setupMechanisms();

    void setupMotors();

    void pubMotorInfoCb();
    void pubMotorFaultCb();

private:
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

    RclSubPtr<Int32Msg> watchdog_status_sub;

    RclTimer::SharedPtr info_pub_timer;
    RclTimer::SharedPtr fault_pub_timer;

    bool is_disabled = false;
};
