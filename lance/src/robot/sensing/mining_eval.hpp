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

/**
 * @file mining_eval.hpp
 * @brief Perception client querying traversability and excavation clearance for candidate cuts.
 *
 * Interfaces with `cardinal_perception`'s 2.5D elevation and ray-marching engine:
 *   - Submits query poses representing trench start points and headings.
 *   - Perception casts volumetric boxes along candidate cuts, evaluating obstacle collisions
 *     and soil terrain height.
 *   - Receives maximum unobstructed excavation distances (`ranges`) for each candidate cut line.
 */

#include <rclcpp/rclcpp.hpp>

#include <Eigen/Core>

#include <cardinal_perception/msg/mining_eval_results.hpp>
#include <cardinal_perception/srv/update_mining_eval_mode.hpp>

#include <csm_utils/ros_utils.hpp>

#include "robot/core/robot_params.hpp"
#include "robot/model/geometry.hpp"


namespace lance
{

/**
 * @class MiningEvalInterface
 * @brief Manages asynchronous RPC queries to `cardinal_perception/update_mining_eval`.
 */
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
    /// @brief Dispatch batch evaluation query for multiple candidate cut poses in arena (map) frame.
    void queryArenaFrame(const std::vector<Pose2f>& poses);

    /// @brief Query forward clearance directly in front of the robot's current pose.
    void queryRobotFrame();

    /// @brief Abort any pending evaluation query.
    void cancelQuery();

    /// @brief True when evaluation results for the active query ID have arrived.
    bool hasResult() const;

    /// @brief Read-only pointer to vector of valid excavation cut lengths in meters (or nullptr).
    const std::vector<float>* getDists() const;

protected:
    void updateResult(const MiningEvalResultsMsg::ConstSharedPtr& msg);

protected:
    const RobotParams& params;
    RclClock::ConstSharedPtr rcl_clock;

    RclSubPtr<MiningEvalResultsMsg> mining_eval_sub;       ///< Evaluation result subscriber.
    RclClientPtr<UpdateMiningEvalSrv> mining_eval_client;   ///< Service client sending queries.

    MiningEvalResultsMsg::UniquePtr eval_results{nullptr}; ///< Received result cache.
    int32_t eval_id{-1};                                   ///< Active query transaction ID.
    float offset_dist{0.f};                                ///< Distance subtracted to account for robot body length.
};

};  // namespace lance
