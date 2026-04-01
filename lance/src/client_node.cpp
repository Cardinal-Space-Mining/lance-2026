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
#include <memory>
#include <vector>
#include <cassert>
#include <iostream>

#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/joint_state.hpp>

#include <visualization_msgs/msg/marker_array.hpp>

#include "util/ros_utils.hpp"

#include "robot/core/ros_interface.hpp"
#include "robot/core/motor_interface.hpp"
#include "robot/model/dynamics.hpp"
#include "robot/sensing/tf_cache.hpp"
#include "robot/telemetry/deserializer.hpp"


using namespace std::chrono_literals;

using namespace util;
using namespace lance;


class ClientNode : public rclcpp::Node, public UsingRosAliases
{
    using MarkerMsg = visualization_msgs::msg::Marker;
    using MarkerArrayMsg = visualization_msgs::msg::MarkerArray;
    using JointStateMsg = sensor_msgs::msg::JointState;

public:
    ClientNode();

protected:
    void initMarkers();

private:
    TfCache tf_cache;
    TelemetryDeserializer telemetry;

    RclSubPtr<TalonInfoMsg> hopper_info_sub;
    RclPubPtr<MarkerArrayMsg> markers_pub;
    RclTimer::SharedPtr markers_pub_timer;

    MarkerArrayMsg markers;
};



ClientNode::ClientNode() :
    Node("mission_control"),
    tf_cache{
        *this,
        declare_and_get_param<std::string>(*this, "arena_frame_id", "map"),
        declare_and_get_param<std::string>(*this, "odom_frame_id", "odom"),
        declare_and_get_param<std::string>(*this, "robot_frame_id", "robot")},
    telemetry{*this, this->tf_cache},
    hopper_info_sub{this->create_subscription<TalonInfoMsg>(
        TALON_INFO_TOPIC("hopper_act"),
        rclcpp::SensorDataQoS{},
        [this](const TalonInfoMsg& info)
        {
            JointStateMsg msg;
            msg.header = info.header;
            msg.name.push_back(lance::HOPPER_JOINT_NAME);
            msg.position.push_back(
                lance::linearActuatorToJointAngle(info.position / 1000.));
            this->telemetry.getPubMap().publish("joint_states", msg);
        })},
    markers_pub{this->create_publisher<MarkerArrayMsg>(
        lance::ARENA_ZONES_TOPIC,
        rclcpp::SensorDataQoS{})},
    markers_pub_timer{this->create_wall_timer(
        1s,
        [this]() { this->markers_pub->publish(this->markers); })}
{
    this->initMarkers();

    std::cout << "LANCE-" << LANCE << " mission control initialized!"
              << std::endl;
}

void ClientNode::initMarkers()
{
    std::vector<double> min, max;

#define ADD_MARKER(param, NS, R, G, B, A)                            \
    {                                                                \
        util::declare_param(*this, param ".min", min, {0., 0., 0.}); \
        util::declare_param(*this, param ".max", max, {0., 0., 0.}); \
        assert(min.size() >= 3 && max.size() >= 3);                  \
        MarkerMsg& marker = this->markers.markers.emplace_back();    \
        marker.header.frame_id = this->tf_cache.arena_frame_id;      \
        marker.header.stamp = this->now();                           \
        marker.ns = NS;                                              \
        marker.id = this->markers.markers.size();                    \
        marker.type = MarkerMsg::CUBE;                               \
        marker.action = MarkerMsg::ADD;                              \
        marker.lifetime = rclcpp::Duration(0, 0);                    \
        marker.pose.position.x = (min[0] + max[0]) / 2.;             \
        marker.pose.position.y = (min[1] + max[1]) / 2.;             \
        marker.pose.position.z = (min[2] + max[2]) / 2.;             \
        marker.scale.x = (max[0] - min[0]);                          \
        marker.scale.y = (max[1] - min[1]);                          \
        marker.scale.z = (max[2] - min[2]);                          \
        marker.color.r = R;                                          \
        marker.color.g = G;                                          \
        marker.color.b = B;                                          \
        marker.color.a = A;                                          \
    }
    ADD_MARKER("arena_bounds", "arena", 1.f, 1.f, 1.f, 0.f);
    ADD_MARKER("mining_zone_bounds", "zones", 0.8f, 0.4f, 0.f, 0.5f);
    ADD_MARKER("offload_zone_bounds", "zones", 0.f, 0.2f, 0.8f, 0.3f);
    ADD_MARKER("construction_zone_bounds", "zones", 0.1f, 0.9f, 0.2f, 0.1f);

#undef ADD_MARKER
}



int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ClientNode>());
    rclcpp::shutdown();
    return 0;
}
