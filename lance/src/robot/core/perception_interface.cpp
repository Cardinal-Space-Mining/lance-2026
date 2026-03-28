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

#include "perception_interface.hpp"


#define PERCEPTION_UPDATE_MINING_EVAL_SRV_TOPIC \
    "/cardinal_perception/update_mining_eval"
#define PERCEPTION_MINING_EVAL_RESULTS_TOPIC   \
    "/cardinal_perception/mining_eval_results"


namespace lance
{

MiningEvalInterface::MiningEvalInterface(
    RclNode& node,
    const RobotParams& params) :
    params{params},
    rcl_clock{node.get_clock()},
    mining_eval_client{node.create_client<UpdateMiningEvalSrv>(
        PERCEPTION_UPDATE_MINING_EVAL_SRV_TOPIC)},
    mining_eval_sub{node.create_subscription<MiningEvalResultsMsg>(
        PERCEPTION_MINING_EVAL_RESULTS_TOPIC,
        rclcpp::SensorDataQoS{},
        [this](const MiningEvalResults::ConstSharedPtr& msg)
        { this->eval_results = msg; })}
{
}


void MiningEvalInterface::queryArenaFrame(const std::vector<Pose2f>& poses)
{
    UpdateMiningEvalSrv::Request::SharedPtr req =
        std::make_shared<UpdateMiningEvalSrv::Request>();

    req->completed = false;
    req->queries.header.frame_id = this->params.arena_frame_id;
    req->queries.header.stamp = this->rcl_clock->now();

    // TODO: if robot model requires multiple queries per pose, scale sizes accordingly
    req->queries.poses.reserve(poses.size());
    req->query_widths.reserve(poses.size());
    req->query_heights.reserve(poses.size());

    for (const Pose2f& pose : poses)
    {
        auto& out = req->queries.poses.emplace_back();

        // TODO: Robot model offsets and add correct widths/heights
        out.position.x = pose.x();
        out.position.y = pose.y();
        out.position.z = 0.0;
        out.orientation.w = std::cos(pose.z() / 2.f);
        out.orientation.x = 0.0;
        out.orientation.y = 0.0;
        out.orientation.z = std::sin(pose.z() / 2.f);

        // TODO: Use robot model to populate
        req->query_widths.push_back(0.f);
        req->query_heights.push_back(0.f);
    }

    this->eval_id = -1;
    this->eval_results = nullptr;
    this->mining_eval_client->async_send_request(
        req,
        [this](rclcpp::Client<UpdateMiningEvalSrv>::SharedFuture f)
        { this->eval_id = f->query_id; });
}

void MiningEvalInterface::queryRobotFrame()
{
    UpdateMiningEvalSrv::Request::SharedPtr req =
        std::make_shared<UpdateMiningEvalSrv::Request>();

    req->completed = false;
    req->queries.header.frame_id = this->params.robot_frame_id;
    req->queries.header.stamp = this->rcl_clock->now();

    // TODO: Resize to however many zones are needed per-pose
    auto& poses = req->queries.poses;
    poses.resize(1);
    poses[0].position.x = 0.0;
    poses[0].position.y = 0.0;
    poses[0].position.z = 0.0;
    poses[0].orientation.w = 1.0;
    poses[0].orientation.x = 0.0;
    poses[0].orientation.y = 0.0;
    poses[0].orientation.z = 0.0;

    req->query_widths.push_back(0.f);
    req->query_heights.push_back(0.f);

    this->eval_id = -1;
    this->eval_results = nullptr;
    this->mining_eval_client->async_send_request(
        req,
        [this](rclcpp::Client<UpdateMiningEvalSrv>::SharedFuture f)
        { this->eval_id = f->query_id; });
}

void MiningEvalInterface::cancelQuery()
{
    UpdateMiningEvalSrv::Request::SharedPtr req =
        std::make_shared<UpdateMiningEvalSrv::Request>();

    req->completed = true;
    this->eval_id = -1;
    this->mining_eval_client->async_send_request(
        req,
        [](rclcpp::Client<UpdateMiningEvalSrv>::SharedFuture f) {});
}

bool MiningEvalInterface::hasResult() const
{
    return this->eval_id >= 0 && this->eval_results.get() != nullptr &&
           this->eval_results->query_id == this->eval_id;
}

const std::vector<float>* MiningEvalInterface::getDists() const
{
    return this->eval_results ? &this->eval_results->ranges : nullptr;
}

};  // namespace lance
