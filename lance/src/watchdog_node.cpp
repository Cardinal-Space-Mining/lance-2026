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

#include <chrono>

#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/int32.hpp>
#include <std_srvs/srv/set_bool.hpp>

#include "util/ros_utils.hpp"
#include "robot/core/robot_status.hpp"
#include "robot/core/ros_interface.hpp"


using namespace util;
using namespace lance;
using namespace std::chrono;
using namespace std::chrono_literals;


#define WATCHDOG_PUB_DT           100ms
#define WATCHDOG_TELEOP_FEED_TIME 250ms
#define WATCHDOG_AUTO_FEED_TIME   10000ms


class RobotStatusServer : public rclcpp::Node, public UsingRosAliases
{
    using Int32Msg = std_msgs::msg::Int32;
    using SetBoolSrv = std_srvs::srv::SetBool;

    using SetBoolReqPtr = SetBoolSrv::Request::SharedPtr;
    using SetBoolRespPtr = SetBoolSrv::Response::SharedPtr;

public:
    RobotStatusServer() :
        Node("robot_status"),

        watchdog_status_pub{this->create_publisher<Int32Msg>(
            lance::WATCHDOG_TOPIC,
            rclcpp::SensorDataQoS{})},

        set_teleop_srv{this->create_service<SetBoolSrv>(
            lance::SET_TELEOP_TOPIC,
            [this](SetBoolReqPtr req, SetBoolRespPtr resp)
            {
                this->ctrl_mode = req->data ? ControlMode::TELEOPERATED
                                            : ControlMode::DISABLED;
                resp->success = true;
            })},

        set_auto_srv{this->create_service<SetBoolSrv>(
            lance::SET_AUTO_TOPIC,
            [this](SetBoolReqPtr req, SetBoolRespPtr resp)
            {
                this->ctrl_mode =
                    req->data ? ControlMode::AUTONOMOUS : ControlMode::DISABLED;
                resp->success = true;
            })},

        test_mode_srv{this->create_service<SetBoolSrv>(
            lance::SET_TEST_TOPIC,
            [this](SetBoolReqPtr req, SetBoolRespPtr resp)
            {
                this->ctrl_opts = static_cast<uint8_t>(
                    req->data ? ControlOpts::TEST_MODE : ControlOpts::NONE);
                resp->success = true;
            })},

        watchdog_timer{this->create_wall_timer(
            WATCHDOG_PUB_DT,
            [this]()
            {
                this->watchdog_status_pub->publish(
                    Int32Msg{}.set__data(this->getFeedTime()));
            })}
    {
    }

protected:
    inline int32_t getFeedTime()
    {
        return ControlStatus::format(
            this->ctrl_mode,
            this->ctrl_opts,
            WATCHDOG_TELEOP_FEED_TIME,
            WATCHDOG_AUTO_FEED_TIME);
    }

protected:
    RclPubPtr<Int32Msg> watchdog_status_pub;
    RclSrvPtr<SetBoolSrv> set_teleop_srv;
    RclSrvPtr<SetBoolSrv> set_auto_srv;
    RclSrvPtr<SetBoolSrv> test_mode_srv;
    RclTimer::SharedPtr watchdog_timer;

    ControlMode ctrl_mode{ControlMode::DISABLED};
    uint8_t ctrl_opts{0};
};


int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RobotStatusServer>());
    rclcpp::shutdown();

    return 0;
}
