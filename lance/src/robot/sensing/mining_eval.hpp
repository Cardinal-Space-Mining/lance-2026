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

#include <rclcpp/rclcpp.hpp>

#include <Eigen/Core>

#include <cardinal_perception/msg/mining_eval_results.hpp>
#include <cardinal_perception/srv/update_mining_eval_mode.hpp>

#include "util/ros_utils.hpp"
#include "robot/core/robot_params.hpp"
#include "robot/model/geometry.hpp"


namespace lance
{

class MiningEvalInterface : public util::UsingRosAliases
{
    using UpdateMiningEvalSrv = cardinal_perception::srv::UpdateMiningEvalMode;
    using MiningEvalResultsMsg = cardinal_perception::msg::MiningEvalResults;

public:
    // vec.x() -> x, vec.y() -> y, vec.z() -> theta (radians)
    using Pose2f = lance::geom::Pose2f;

public:
    MiningEvalInterface(RclNode&, const RobotParams&);

public:
    void queryArenaFrame(const std::vector<Pose2f>& poses);
    void queryRobotFrame();
    void cancelQuery();

    bool hasResult() const;
    const std::vector<float>* getDists() const;

protected:
    void updateResult(const MiningEvalResultsMsg::ConstSharedPtr& msg);

protected:
    const RobotParams& params;
    RclClock::ConstSharedPtr rcl_clock;

    RclSubPtr<MiningEvalResultsMsg> mining_eval_sub;
    RclClientPtr<UpdateMiningEvalSrv> mining_eval_client;

    MiningEvalResultsMsg::UniquePtr eval_results{nullptr};
    int32_t eval_id{-1};
    float offset_dist{0.f};
};

};  // namespace lance
