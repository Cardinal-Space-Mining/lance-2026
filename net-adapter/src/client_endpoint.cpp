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
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>

#include "ros_utils.hpp"
#include "zenoh_utils.hpp"

#include "adapters/joy_adapter.hpp"
#include "adapters/talon_adapter.hpp"
#include "adapters/generic_adapter.hpp"
#include "adapters/watchdog_adapter.hpp"
#include "adapters/ms136_imu_adapter.hpp"
#include "adapters/ms136_scan_adapter.hpp"
#include "adapters/path_adapter.hpp"


#define DEFAULT_ROBOT_IP_ADDRESS "10.11.11.10"

using namespace zenoh;
using namespace util;


class ClientEndpointNode : public rclcpp::Node
{
    using StdInt8Adapter = GenericAdapter<std_msgs::msg::Int8>;
    using StdStringAdapter = GenericAdapter<std_msgs::msg::String>;
    using PointStampedAdapter =
        GenericAdapter<geometry_msgs::msg::PointStamped>;

public:
    ClientEndpointNode() :
        Node{
            "client_redux_endpoint"
    },
        zsh{Session::open(configDirectConnectTo(
            declare_and_get_param<std::string>(
                *this,
                "robot_hostname",
                DEFAULT_ROBOT_IP_ADDRESS)))},

        imu_pub{MS136ImuAdapter::createPublisher(*this, zsh, "multiscan/imu")},
        scan_pub{MS136ScanAdapter::createPublisher(
            *this,
            zsh,
            "multiscan/lidar_scan")},

        talon_pubs{
            *this,
            zsh,
            {"lance/track_left",
             "lance/track_right",
             "lance/trencher",
             "lance/hopper_belt",
             "lance/hopper_act"}},
        path_pub{PathAdapter::createPublisher(
            *this,
            zsh,
            "cardinal_perception/planned_path")},

        joy_sub{JoyAdapter::createSubscriber(*this, zsh, "/joy")},
        watchdog_sub{WatchdogAdapter::createSubscriber(
            *this,
            zsh,
            "lance/watchdog_status")},
        clicked_point_sub{
            PointStampedAdapter::createSubscriber(*this, zsh, "clicked_point")},

        relay_status_pub{
            StdInt8Adapter::createPublisher(*this, zsh, "lance/relay_status")},
        op_status_pub{
            StdStringAdapter::createPublisher(*this, zsh, "lance/op_status")}
    {
    }

private:
    Session zsh;

    MS136ImuAdapter::Publisher imu_pub;
    MS136ScanAdapter::Publisher scan_pub;
    TalonFeedback::PublisherGroup talon_pubs;
    PathAdapter::Publisher path_pub;

    JoyAdapter::Subscriber joy_sub;
    WatchdogAdapter::Subscriber watchdog_sub;
    PointStampedAdapter::Subscriber clicked_point_sub;

    StdInt8Adapter::Publisher relay_status_pub;
    StdStringAdapter::Publisher op_status_pub;
};


int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ClientEndpointNode>());
    rclcpp::shutdown();

    return 0;
}
