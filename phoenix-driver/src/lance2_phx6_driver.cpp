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

#include <atomic>
#include <chrono>
#include <cmath>
#include <functional>
#include <iostream>
#include <memory>
#include <string>
#include <string_view>
#include <thread>
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
#include "phx6_utils.hpp"

using namespace util;
using namespace util::ros_aliases;
using namespace std::chrono_literals;

class Phoenix6Driver : public rclcpp::Node
{
    using Int32Msg = std_msgs::msg::Int32;

    struct RclTalonFX
    {
        TalonFX motor;
        SharedPub<TalonInfoMsg> info_pub;
        SharedPub<TalonFaultsMsg> faults_pub;
        SharedSub<TalonCtrlMsg> ctrl_sub;
    };

    struct RclTalonFXS
    {
        TalonFXS motor;
        SharedPub<TalonInfoMsg> info_pub;
        SharedPub<TalonFaultsMsg> faults_pub;
        SharedSub<TalonCtrlMsg> ctrl_sub;
    };

public:
    Phoenix6Driver();
    ~Phoenix6Driver();

private:
    void initPhx();

    template <typename RclMotor>
    void initMotor(RclMotor& rcl_motor);

    void feed_watchdog_status(int32_t status);

    void pub_motor_info_cb();
    void pub_motor_fault_cb();

    template <typename RclMotor>
    void execute_ctrl(typename RclMotor::MotorType& motor, const TalonCtrlMsg& msg);

private:
    RclTalonFX track_right;
    RclTalonFX track_left;
    RclTalonFX trencher;
    RclTalonFX hopper_belt;
    RclTalonFXS hopper_act_left;
    RclTalonFXS hopper_act_right;

    SharedSub<Int32Msg> watchdog_status_sub;

    RclTimer info_pub_timer;
    RclTimer fault_pub_timer;

    bool is_disabled = false;
};

Phoenix6Driver::Phoenix6Driver() :
    Node{"phoenix6_driver"},
    {
        this->initPhx();
        this->initMotorFX();
        this->initMotorFXS();
    }

void Phoenix6Driver::initPhx()
{

}

template <typename RclMotor>
void Phoenix6Driver::initMotor(RclMotor& rcl_motor)
{

}


void Phoenix6Driver::feed_watchdog_status(int32_t status)
{

}

void Phoenix6Driver::pub_motor_info_cb()
{

}

void Phoenix6Driver::pub_motor_fault_cb()
{

}

template <typename RclMotor>
void execute_ctrl(typename RclMotor::MotorType& motor, const TalonCtrlMsg& msg);
{

}
