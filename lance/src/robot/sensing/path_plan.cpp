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

#include "path_plan.hpp"

#include "robot/core/ros_interface.hpp"


namespace lance
{

PathPlanInterface::PathPlanInterface(RclNode& node, const RobotParams& params) :
    params{params},
    rcl_clock{node.get_clock()},
    path_sub{node.create_subscription<PathMsg>(
        lance::PERCEPTION_PATH_TOPIC,
        rclcpp::SensorDataQoS{},
        [this](const PathMsg::ConstSharedPtr& msg) { this->last_path = msg; })},
    pplan_control_client{node.create_client<UpdatePathPlanSrv>(
        lance::PERCEPTION_PPLAN_CONTROL_TOPIC)}
{
}


void PathPlanInterface::init(const Vec3f& arena_dest)
{
    auto req = std::make_shared<UpdatePathPlanSrv::Request>();
    req->target.header.frame_id = this->params.arena_frame_id;
    req->target.header.stamp = this->rcl_clock->now();
    req->target.pose.position.x = arena_dest.x();
    req->target.pose.position.y = arena_dest.y();
    req->target.pose.position.z = arena_dest.z();
    req->completed = false;

    this->pplan_control_client->async_send_request(
        req,
        [](RclClient<UpdatePathPlanSrv>::SharedFuture) {});
}
void PathPlanInterface::init(const PoseStampedMsg& dest)
{
    auto req = std::make_shared<UpdatePathPlanSrv::Request>();
    req->target = dest;
    req->completed = false;

    this->pplan_control_client->async_send_request(
        req,
        [](RclClient<UpdatePathPlanSrv>::SharedFuture) {});
}
void PathPlanInterface::init(const PointStampedMsg& dest)
{
    auto req = std::make_shared<UpdatePathPlanSrv::Request>();
    req->target.header = dest.header;
    req->target.pose.position = dest.point;
    req->completed = false;

    this->pplan_control_client->async_send_request(
        req,
        [](RclClient<UpdatePathPlanSrv>::SharedFuture) {});
}
void PathPlanInterface::cancel()
{
    auto req = std::make_shared<UpdatePathPlanSrv::Request>();
    req->completed = true;

    this->pplan_control_client->async_send_request(
        req,
        [this](RclClient<UpdatePathPlanSrv>::SharedFuture)
        { this->last_path = nullptr; });
}

bool PathPlanInterface::hasPath() const
{
    return this->last_path.operator bool();
}
const PathPlanInterface::PathMsg* PathPlanInterface::getPath() const
{
    return this->last_path ? this->last_path.get() : nullptr;
}

void PathPlanInterface::clearPath() { this->last_path.reset(); }

};  // namespace lance
