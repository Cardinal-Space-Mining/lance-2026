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

#include <memory>
#include <iostream>

#include <rclcpp/rclcpp.hpp>

#include "util/ros_utils.hpp"

#include "robot/sensing/tf_cache.hpp"
#include "robot/telemetry/deserializer.hpp"

#include "mission_control/inputs.hpp"
#include "mission_control/watchdog.hpp"
#include "mission_control/zone_pub.hpp"
#include "mission_control/joint_pub.hpp"

using namespace util;
using namespace lance;


class MissionControlNode : public rclcpp::Node, public UsingRosAliases
{
public:
    MissionControlNode();

private:
    TfCache tf_cache;
    TelemetryDeserializer telemetry;

    WatchDog watchdog;
    ZonePublisher zone_publisher;
    JointPublisher joint_publisher;
    InputInterface input_interface;
};



// --- Client Node -------------------------------------------------------------

MissionControlNode::MissionControlNode() :
    Node("mission_control"),
    tf_cache{
        *this,
        declare_and_get_param<std::string>(*this, "arena_frame_id", "map"),
        declare_and_get_param<std::string>(*this, "odom_frame_id", "odom"),
        declare_and_get_param<std::string>(*this, "robot_frame_id", "robot")},
    telemetry{*this, this->tf_cache},

    watchdog{*this},
    zone_publisher{*this, this->tf_cache},
    joint_publisher{*this},
    input_interface{*this, this->tf_cache, this->telemetry, this->watchdog}
{
    std::cout << "LANCE-" << LANCE << " mission control initialized!"
              << std::endl;
}



// --- Main --------------------------------------------------------------------

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MissionControlNode>());
    rclcpp::shutdown();
    return 0;
}
