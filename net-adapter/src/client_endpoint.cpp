/*******************************************************************************
*   Copyright (C) 2024-2026 Cardinal Space Mining Club                         *
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

#include <string>

#include <zenoh.hxx>

#include <rclcpp/rclcpp.hpp>

#include "ros_utils.hpp"
#include "zenoh_utils.hpp"

#include "adapters/joy_adapter.hpp"
#include "adapters/ms136_imu_adapter.hpp"
#include "adapters/ms136_scan_adapter.hpp"
#include "adapters/talon_adapter.hpp"
#include "adapters/watchdog_adapter.hpp"


#define DEFAULT_ROBOT_IP_ADDRESS "10.11.11.10"

using namespace zenoh;
using namespace util;


class ClientEndpointNode : public rclcpp::Node
{
public:
    ClientEndpointNode() :
        Node{"client_redux_endpoint"},
        zsh{Session::open(configDirectConnectTo(
            declare_and_get_param<std::string>(
                *this,
                "robot_hostname",
                DEFAULT_ROBOT_IP_ADDRESS)))},
        joy_sub{JoyAdapter::createSubscriber(*this, zsh, "/joy")},
        imu_pub{MS136ImuAdapter::createPublisher(*this, zsh, "multiscan/imu")},
        scan_pub{MS136ScanAdapter::createPublisher(
            *this,
            zsh,
            "multiscan/lidar_scan")},
        watchdog_sub{WatchdogAdapter::createSubscriber(
            *this,
            zsh,
            "lance/watchdog_status")},
        track_left(*this, zsh, "track_left"),
        track_right(*this, zsh, "track_right"),
        trencher(*this, zsh, "trencher"),
        hopper_belt(*this, zsh, "hopper_belt"),
        hopper_actuator(*this, zsh, "hopper_actuator")
    {
    }

private:
    Session zsh;

    JoyAdapter::Subscriber joy_sub;
    MS136ImuAdapter::Publisher imu_pub;
    MS136ScanAdapter::Publisher scan_pub;
    WatchdogAdapter::Subscriber watchdog_sub;

    struct MotorEndpoint
    {
        // TalonCtrlAdapter::Publisher ctrl_pub;
        TalonInfoAdapter::Publisher info_pub;
        TalonFaultsAdapter::Publisher faults_pub;

        MotorEndpoint(
            rclcpp::Node& node,
            Session& zsh,
            const std::string& name) :
            // ctrl_pub{TalonCtrlAdapter::createPublisher(node, zsh, name)},
            info_pub{TalonInfoAdapter::createPublisher(node, zsh, name)},
            faults_pub{TalonFaultsAdapter::createPublisher(node, zsh, name)}
        {
        }
    };

    MotorEndpoint track_left;
    MotorEndpoint track_right;
    MotorEndpoint trencher;
    MotorEndpoint hopper_belt;
    MotorEndpoint hopper_actuator;
};


int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ClientEndpointNode>());
    rclcpp::shutdown();

    return 0;
}
