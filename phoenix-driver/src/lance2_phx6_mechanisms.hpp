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
#pragma once

#include "lance2_phx6_driver.hpp"

#include <optional>
#include <string>

struct Phoenix6Driver::CustomMechanismPair
{
    std::string name;
    RclMotor<TalonFXS>* leader;
    RclMotor<TalonFXS>* follower;
    rclcpp::Logger logger;
    rclcpp::Clock::SharedPtr clock;
    SharedPub<TalonInfoMsg> info_pub;
    SharedPub<TalonFaultsMsg> faults_pub;
    SharedSub<TalonCtrlMsg> ctrl_sub;
    RclTimer update_timer;
    const bool* is_disabled;
    std::optional<TalonCtrlMsg> active_ctrl;

    CustomMechanismPair(
        const std::string& name,
        rclcpp::Node* node,
        RclMotor<TalonFXS>* leader,
        RclMotor<TalonFXS>* follower,
        int update_period_ms,
        const bool* is_disabled,
        const rclcpp::Logger& logger,
        rclcpp::Clock::SharedPtr clock);

    double potPosition(RclMotor<TalonFXS>& motor) const;
    TalonInfoMsg motorInfo(RclMotor<TalonFXS>& motor) const;
    static uint8_t averageUint8(uint8_t a, uint8_t b);

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
