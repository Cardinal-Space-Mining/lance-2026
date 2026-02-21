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

#include <std_msgs/msg/int8.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/string.hpp>

#include <geometry_msgs/msg/point_stamped.hpp>

#include "ros_utils.hpp"
#include "zenoh_utils.hpp"

#include "adapters/joy_adapter.hpp"
#include "adapters/talon_adapter.hpp"
#include "adapters/generic_adapter.hpp"
#include "adapters/ms136_imu_adapter.hpp"
#include "adapters/ms136_scan_adapter.hpp"
#include "adapters/path_adapter.hpp"


#define DEFAULT_CLIENT_IP_ADDRESS "10.11.11.8"

using namespace zenoh;
using namespace util;


class RobotEndpointNode : public rclcpp::Node
{
    using StdInt8Adapter = GenericAdapter<std_msgs::msg::Int8>;
    using StdInt32Adapter = GenericAdapter<std_msgs::msg::Int32>;
    using StdStringAdapter = GenericAdapter<std_msgs::msg::String>;
    using PointStampedAdapter =
        GenericAdapter<geometry_msgs::msg::PointStamped>;

public:
    RobotEndpointNode() :
        Node{
            "robot_redux_endpoint"
    },
        zsh{Session::open(configDirectConnectTo(
            declare_and_get_param<std::string>(
                *this,
                "client_hostname",
                DEFAULT_CLIENT_IP_ADDRESS)))},

        joy_pub{JoyAdapter::createPublisher(*this, zsh, "/joy")},
        watchdog_pub{StdInt32Adapter::createPublisher(
            *this,
            zsh,
            "lance/watchdog_status")},
        clicked_point_pub{
            PointStampedAdapter::createPublisher(*this, zsh, "clicked_point")},

        imu_sub{MS136ImuAdapter::createSubscriber(*this, zsh, "multiscan/imu")},
        scan_sub{MS136ScanAdapter::createSubscriber(
            *this,
            zsh,
            "multiscan/lidar_scan")},

        talon_subs{
            *this,
            zsh,
            {"lance/track_left",
             "lance/track_right",
             "lance/trencher",
             "lance/hopper_belt",
             "lance/hopper_act"}},
        path_sub{PathAdapter::createSubscriber(
            *this,
            zsh,
            "cardinal_perception/planned_path")},

        relay_status_sub{
            StdInt8Adapter::createSubscriber(*this, zsh, "lance/relay_status")},
        op_status_sub{
            StdStringAdapter::createSubscriber(*this, zsh, "lance/op_status")}
    {
    }

private:
    Session zsh;

    JoyAdapter::Publisher joy_pub;
    StdInt32Adapter::Publisher watchdog_pub;
    PointStampedAdapter::Publisher clicked_point_pub;

    MS136ImuAdapter::Subscriber imu_sub;
    MS136ScanAdapter::Subscriber scan_sub;
    TalonFeedback::SubscriberGroup talon_subs;
    PathAdapter::Subscriber path_sub;

    StdInt8Adapter::Subscriber relay_status_sub;
    StdStringAdapter::Subscriber op_status_sub;
};


int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RobotEndpointNode>());
    rclcpp::shutdown();

    return 0;
}
