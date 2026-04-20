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

#include <vector>
#include <iostream>
#include <stdexcept>
#include <algorithm>
#include <functional>

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>

#include "robot/core/robot_params.hpp"
#include "robot/sensing/mining_eval.hpp"


namespace lance
{

enum class MiningDirection
{
    UP,
    DOWN,
    LEFT,
    RIGHT
};

struct MiningGridGeometry
{
    float mining_zone_x_length = 0.0f;
    float mining_zone_y_length = 0.0f;
    float actual_mining_x_length = 0.0f;
    float actual_mining_y_length = 0.0f;
    int x_divisions = 0;
    int y_divisions = 0;
    Eigen::Vector2f min_corner_with_offset = Eigen::Vector2f::Zero();
    Eigen::Vector2f max_corner_with_offset = Eigen::Vector2f::Zero();
};


// Struct to represent a mining path with its direction and associated matrix for
// distance calculation
class DirectedMiningPath
{
public:
    using Vec2i = Eigen::Vector2i;
    using Vec2f = Eigen::Vector2f;
    using MiningPath = std::pair<Vec2i, Vec2i>;
    using MiningSwath = std::pair<Vec2f, Vec2f>;

public:
    DirectedMiningPath(
        MiningPath p,
        MiningDirection dir,
        const Eigen::MatrixXf* mat);

public:
    MiningSwath getPathCoordinatesInWorldFrame(
        const RobotParams& robot_params,
        const MiningGridGeometry* grid_geometry = nullptr) const;

    void markMiningOnMatrix(Eigen::MatrixXi& mined_count_matrix) const;

    void print() const
    {
        std::cout << "Path from (" << path.first.x() << ", " << path.first.y()
                  << ") to (" << path.second.x() << ", " << path.second.y()
                  << ") in direction " << static_cast<int>(direction)
                  << " with distance " << distance << "\n";
    }


public:
// public but only used by MiningPlanner
    float getDistance() const;

    bool checkValidity() const;

    float getQuality(const Eigen::MatrixXi& previously_minined_locations) const;

    bool adjustForRobotClearance();

    MiningDirection getDirection() const { return direction; }

    float getRecalculatedDistance() const;

private:
    MiningPath path;
    MiningDirection direction;
    const Eigen::MatrixXf* matrix;
    float distance = -1.0;
    static constexpr float previously_mined_penalty = 0.1f;
};



class MiningPlanner
{
public:
    using Box2f = Eigen::AlignedBox2f;
    using DirectedMiningPaths = std::vector<DirectedMiningPath>;
    using Pose2f = Eigen::Vector3f;

public:
    MiningPlanner(MiningEvalInterface& mining_eval, const RobotParams& robot_params);

public:
    bool updateMappedMatrices();
    const DirectedMiningPaths& finalOutput();
    const DirectedMiningPaths& getCachedPaths() const { return all_mining_paths; }
    const MiningGridGeometry& getGridGeometry() const { return grid_geometry; }

    void markMiningOnMatrix(const DirectedMiningPath& path);
    bool hasSentRequest() const { return sent_eval_request; }

private:
    static MiningGridGeometry
        computeMiningGridGeometry(const RobotParams& robot_params);

    const std::vector<Pose2f>& getStartingLocations();

    void appendPlannedMiningPaths();

    void sortPathsByQuality();
    void removeSectionsForRobotClearance();

private:
    const RobotParams& robot_params;
    MiningEvalInterface& mining_eval;


    bool sent_eval_request{ false };
    MiningGridGeometry grid_geometry;

    // The direction is the way the the robot would be moving in reference to the
    // base frame which is MiningDirection::DOWN
    Eigen::MatrixXf strip_map_up;
    Eigen::MatrixXf strip_map_down;
    Eigen::MatrixXf strip_map_left;
    Eigen::MatrixXf strip_map_right;
    Eigen::MatrixXf strip_map_left_transposed;
    Eigen::MatrixXf strip_map_right_transposed;
    Eigen::MatrixXi previously_mined_cells;

    DirectedMiningPaths all_mining_paths;

    
};

};  // namespace lance
