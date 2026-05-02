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

#include "mining_eval.hpp"

#include "robot/core/ros_interface.hpp"


using namespace lance::geom;


namespace lance
{

MiningEvalInterface::MiningEvalInterface(
    RclNode& node,
    const RobotParams& params) :
    params{params},
    rcl_clock{node.get_clock()},
    mining_eval_sub{node.create_subscription<MiningEvalResultsMsg>(
        lance::PERCEPTION_MINING_EVAL_RESULTS_TOPIC,
        rclcpp::SensorDataQoS{},
        [this](const MiningEvalResultsMsg::ConstSharedPtr& msg)
        { this->updateResult(msg); })},
    mining_eval_client{node.create_client<UpdateMiningEvalSrv>(
        lance::PERCEPTION_UPDATE_MINING_EVAL_SRV_TOPIC)}
{
}


void MiningEvalInterface::queryArenaFrame(const std::vector<Pose2f>& poses)
{
    std::cout << "Querying mining eval arena frame with " << poses.size()
              << " poses...\n";
    UpdateMiningEvalSrv::Request::SharedPtr req =
        std::make_shared<UpdateMiningEvalSrv::Request>();

    req->completed = false;
    req->queries.header.frame_id = this->params.arena_frame_id;
    req->queries.header.stamp = this->rcl_clock->now();

    req->queries.poses.reserve(poses.size());
    req->query_widths.reserve(poses.size());
    req->query_heights.reserve(poses.size());

    for (const Pose2f& pose : poses)
    {
        auto& out = req->queries.poses.emplace_back();

        const float st = std::sin(pose.z());
        const float ct = std::cos(pose.z());

        out.position.x = pose.x() + (ct * PRIMARY_COLLISION_ZONE_X -
                                     st * PRIMARY_COLLISION_ZONE_Y);
        out.position.y = pose.y() + (st * PRIMARY_COLLISION_ZONE_X +
                                     ct * PRIMARY_COLLISION_ZONE_Y);
        out.position.z = PRIMARY_COLLISION_ZONE_Z;

        out.orientation.w = std::cos(pose.z() / 2.f);
        out.orientation.x = 0.0;
        out.orientation.y = 0.0;
        out.orientation.z = std::sin(pose.z() / 2.f);

        req->query_widths.push_back(PRIMARY_COLLISION_ZONE_WIDTH_<float>);
        req->query_heights.push_back(PRIMARY_COLLISION_ZONE_HEIGHT_<float>);
    }

    this->eval_id = -1;
    this->offset_dist = PRIMARY_COLLISION_ZONE_LENGTH_OFFSET_<float>;
    this->eval_results.reset();
    this->mining_eval_client->async_send_request(
        req,
        [this](rclcpp::Client<UpdateMiningEvalSrv>::SharedFuture f)
        { this->eval_id = f.get()->query_id; });
}

void MiningEvalInterface::queryRobotFrame()
{
    UpdateMiningEvalSrv::Request::SharedPtr req =
        std::make_shared<UpdateMiningEvalSrv::Request>();

    req->completed = false;
    req->queries.header.frame_id = this->params.robot_frame_id;
    req->queries.header.stamp = this->rcl_clock->now();

    auto& pose = req->queries.poses.emplace_back();
    pose.position.x = std::min(TRACKS_X_MAX, TRENCHER_X_MAX);
    pose.position.y = PRIMARY_COLLISION_ZONE_Y;
    pose.position.z = PRIMARY_COLLISION_ZONE_Z;

    req->query_widths.push_back(PRIMARY_COLLISION_ZONE_WIDTH_<float>);
    req->query_heights.push_back(PRIMARY_COLLISION_ZONE_HEIGHT_<float>);

    this->eval_id = -1;
    this->offset_dist =
        (FOOTPRINT_X_MAX_<float> -
         std::min(TRACKS_X_MAX_<float>, TRENCHER_X_MAX_<float>));
    this->eval_results.reset();
    this->mining_eval_client->async_send_request(
        req,
        [this](rclcpp::Client<UpdateMiningEvalSrv>::SharedFuture f)
        { this->eval_id = f.get()->query_id; });
}

void MiningEvalInterface::cancelQuery()
{
    UpdateMiningEvalSrv::Request::SharedPtr req =
        std::make_shared<UpdateMiningEvalSrv::Request>();

    req->completed = true;
    this->eval_id = -1;
    this->mining_eval_client->async_send_request(
        req,
        [](rclcpp::Client<UpdateMiningEvalSrv>::SharedFuture) {});
}

bool MiningEvalInterface::hasResult() const
{
    return this->eval_id >= 0 && this->eval_results &&
           this->eval_results->query_id == this->eval_id;
}

const std::vector<float>* MiningEvalInterface::getDists() const
{
    return this->eval_results ? &this->eval_results->ranges : nullptr;
}


void MiningEvalInterface::updateResult(
    const MiningEvalResultsMsg::ConstSharedPtr& msg)
{
    if (!this->eval_results)
    {
        this->eval_results = std::make_unique<MiningEvalResultsMsg>();
    }

    this->eval_results->query_id = msg->query_id;
    this->eval_results->ranges.clear();
    this->eval_results->ranges.reserve(msg->ranges.size());
    for (const float r : msg->ranges)
    {
        this->eval_results->ranges.push_back(r - this->offset_dist);
    }
}

};  // namespace lance
