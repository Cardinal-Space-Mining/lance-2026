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

#include <chrono>
#include <vector>
#include <cassert>

#include <rclcpp/rclcpp.hpp>

#include <visualization_msgs/msg/marker_array.hpp>

#include "util/ros_utils.hpp"

#include "robot/core/ros_interface.hpp"
#include "robot/model/geometry.hpp"
#include "robot/sensing/tf_cache.hpp"


namespace lance
{

class ZonePublisher : public util::UsingRosAliases
{
    using Box2f = lance::geom::Box2f;

    using MarkerMsg = visualization_msgs::msg::Marker;
    using MarkerArrayMsg = visualization_msgs::msg::MarkerArray;

    static constexpr int64_t PUB_DT_MS = 1000;

public:
    inline ZonePublisher(RclNode& node, const TfCache& tf_cache) :
        markers_pub{node.create_publisher<MarkerArrayMsg>(
            lance::ARENA_ZONES_TOPIC,
            rclcpp::SensorDataQoS{})},
        markers_pub_timer{node.create_wall_timer(
            std::chrono::milliseconds(PUB_DT_MS),
            [this]() { this->markers_pub->publish(this->markers); })}
    {
        this->initMarkers(node, tf_cache);
    }

public:
    inline const Box2f& arenaBounds() const { return this->arena_bounds; }
    inline const Box2f& miningBounds() const { return this->mining_bounds; }
    inline const Box2f& offloadBounds() const { return this->offload_bounds; }
    inline const Box2f& constructionBounds() const
    {
        return this->construction_bounds;
    }

private:
    inline void initMarkers(RclNode& node, const TfCache& tf_cache)
    {
        std::vector<double> min, max;

#define ADD_MARKER(param, NS, R, G, B, A)                           \
    {                                                               \
        util::declare_param(node, param ".min", min, {0., 0., 0.}); \
        util::declare_param(node, param ".max", max, {0., 0., 0.}); \
        assert(min.size() >= 3 && max.size() >= 3);                 \
        MarkerMsg& marker = this->markers.markers.emplace_back();   \
        marker.header.frame_id = tf_cache.arena_frame_id;           \
        marker.header.stamp = node.now();                           \
        marker.ns = NS;                                             \
        marker.id = this->markers.markers.size();                   \
        marker.type = MarkerMsg::CUBE;                              \
        marker.action = MarkerMsg::ADD;                             \
        marker.lifetime = rclcpp::Duration(0, 0);                   \
        marker.pose.position.x = (min[0] + max[0]) / 2.;            \
        marker.pose.position.y = (min[1] + max[1]) / 2.;            \
        marker.pose.position.z = (min[2] + max[2]) / 2.;            \
        marker.scale.x = (max[0] - min[0]);                         \
        marker.scale.y = (max[1] - min[1]);                         \
        marker.scale.z = (max[2] - min[2]);                         \
        marker.color.r = R;                                         \
        marker.color.g = G;                                         \
        marker.color.b = B;                                         \
        marker.color.a = A;                                         \
    }
#define SET_BOUNDS(var)                             \
    {                                               \
        var.min().x() = static_cast<float>(min[0]); \
        var.min().y() = static_cast<float>(min[1]); \
        var.max().x() = static_cast<float>(max[0]); \
        var.max().y() = static_cast<float>(max[1]); \
    }

        ADD_MARKER("arena_bounds", "arena", 1.f, 1.f, 1.f, 0.f);
        SET_BOUNDS(this->arena_bounds)
        ADD_MARKER("mining_zone_bounds", "zones", 0.8f, 0.4f, 0.f, 0.2f);
        SET_BOUNDS(this->mining_bounds)
        ADD_MARKER("offload_zone_bounds", "zones", 0.f, 0.2f, 0.8f, 0.2f);
        SET_BOUNDS(this->offload_bounds)
        ADD_MARKER("construction_zone_bounds", "zones", 0.1f, 0.9f, 0.2f, 0.1f);
        SET_BOUNDS(this->construction_bounds)

#undef ADD_MARKER
#undef SET_BOUNDS
    }

private:
    RclPubPtr<MarkerArrayMsg> markers_pub;
    RclTimer::SharedPtr markers_pub_timer;

    MarkerArrayMsg markers;

    Box2f arena_bounds;
    Box2f mining_bounds;
    Box2f offload_bounds;
    Box2f construction_bounds;
};

};  // namespace lance
