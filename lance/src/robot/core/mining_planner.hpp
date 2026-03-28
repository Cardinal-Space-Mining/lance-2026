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

#include <vector>
#include <iostream>
#include <stdexcept>
#include <algorithm>
#include <functional>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "robot_params.hpp"


namespace lance
{

enum class MiningDirection
{
    UP,
    DOWN,
    LEFT,
    RIGHT
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
    float getDistance() const;

    void markMiningOnMatrix(Eigen::MatrixXi& mined_count_matrix) const;

    bool checkValidity() const;

    float getQuality(const Eigen::MatrixXi& previously_minined_locations) const;

    bool adjustForRobotClearance();

    MiningSwath getPathCoordinatesInWorldFrame() const;

    float getRecalculatedDistance() const;

private:
    MiningPath toBaseCoordinates() const;

private:
    MiningPath path;
    MiningDirection direction;
    const Eigen::MatrixXf* matrix;
    float distance = -1.0;
};



class MiningPlanner
{
public:
    using Box2f = Eigen::AlignedBox2f;
    using DirectedMiningPaths = std::vector<DirectedMiningPath>;

public:
    MiningPlanner(const RobotParams& robot_params);

public:
    void updateMappedMatrices();
    const DirectedMiningPaths& finalOutput();

    void markMiningOnMatrix(const DirectedMiningPath& path);

private:
    // Helper function to populate a strip map for a given direction
    void populdateStripMap(
        Eigen::MatrixXf& strip_map,
        MiningDirection direction);

    void appendPlannedMiningPaths(
        const Eigen::MatrixXf& mat,
        MiningDirection mining_dir);

    void sortPathsByQuality();
    void removeSectionsForRobotClearance();

private:
    const RobotParams& robot_params;

    // The direction is the way the the robot would be moving in reference to the
    // base frame which is MiningDirection::DOWN
    Eigen::MatrixXf strip_map_up;
    Eigen::MatrixXf strip_map_down;
    Eigen::MatrixXf strip_map_left;
    Eigen::MatrixXf strip_map_right;
    Eigen::MatrixXi times_mined_count_matrix;

    DirectedMiningPaths all_mining_paths;

    int mapped_matrix_width;
    int mapped_matrix_height;

    const float previously_mined_penalty = 0.1f;
};

};  // namespace lance
