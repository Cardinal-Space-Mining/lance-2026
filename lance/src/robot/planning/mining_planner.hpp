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
#include <map>
#include <iostream>
#include <stdexcept>
#include <algorithm>
#include <functional>
#include <string_view>

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>

#include "robot/core/robot_params.hpp"
#include "robot/sensing/mining_eval.hpp"


namespace lance
{

enum class MiningDirection
{
    YPLUS,
    YMINUS,
    XMINUS,
    XPLUS
};
// MiningDirection to radians
inline float miningDirectionToRadians(MiningDirection dir)
{
    switch (dir)
    {
        case MiningDirection::YPLUS:
            return M_PI / 2.0f;  // 90 degrees in radians
        case MiningDirection::YMINUS:
            return 3.0f * M_PI / 2.0f;  // 270 degrees in radians
        case MiningDirection::XMINUS:
            return M_PI;  // 180 degrees in radians
        case MiningDirection::XPLUS:
            return 0.0f;  // 0 degrees in radians
        default:
            throw std::invalid_argument("Invalid MiningDirection");
    }
}

inline constexpr std::string_view toString(MiningDirection dir)
{
    switch (dir)
    {
        case MiningDirection::YPLUS:
            return "YPLUS";
        case MiningDirection::YMINUS:
            return "YMINUS";
        case MiningDirection::XMINUS:
            return "XMINUS";
        case MiningDirection::XPLUS:
            return "XPLUS";
        default:
            return "UNKNOWN";
    }
}

// Utility to iterate over all mining directions in a range-based for loop
inline constexpr std::array<MiningDirection, 4> ALL_MINING_DIRECTIONS = {
    MiningDirection::YPLUS,
    MiningDirection::YMINUS,
    MiningDirection::XMINUS,
    MiningDirection::XPLUS,
};

using MiningZoneLimiterVector = std::array<
    float,
    4>;  // {Yplus, Yminus, Xminus, Xplus} limits to how far into the mining zone the robot can mine in each direction based on its footprint and the geometry of the zone

// test version with all geom_FOOTPRINT_R_MAX_ as limits
// inline constexpr MiningZoneLimiterVector MINING_ZONE_OFFSETS[4] = {
//     MiningZoneLimiterVector{
//         geom::FOOTPRINT_R_MAX_<float>, geom::FOOTPRINT_R_MAX_<float>, geom::FOOTPRINT_R_MAX_<float>, geom::FOOTPRINT_R_MAX_<float>,},
//     MiningZoneLimiterVector{
//         geom::FOOTPRINT_R_MAX_<float>, geom::FOOTPRINT_R_MAX_<float>, geom::FOOTPRINT_R_MAX_<float>, geom::FOOTPRINT_R_MAX_<float>,},
//     MiningZoneLimiterVector{
//         geom::FOOTPRINT_R_MAX_<float>, geom::FOOTPRINT_R_MAX_<float>, geom::FOOTPRINT_R_MAX_<float>, geom::FOOTPRINT_R_MAX_<float>,},
//     MiningZoneLimiterVector{
//         geom::FOOTPRINT_R_MAX_<float>, geom::FOOTPRINT_R_MAX_<float>, geom::FOOTPRINT_R_MAX_<float>, geom::FOOTPRINT_R_MAX_<float>,}
// };
// For ksc
inline constexpr std::array<std::array<float, 4>, 4> MINING_ZONE_OFFSETS = {
    // 4 mining limiters in order of Yplus, Yminus, Xminus, Xplus
    MiningZoneLimiterVector{
                            geom::FOOTPRINT_R_MAX_<float>,
                            0.0f,geom::FOOTPRINT_R_MAX_<float> / 2.0f,
                            0.0f, },
    MiningZoneLimiterVector{
                            0.0f, geom::FOOTPRINT_R_MAX_<float>,
                            geom::FOOTPRINT_R_MAX_<float> / 2.0f,
                            0.0f, },
    MiningZoneLimiterVector{
                            geom::FOOTPRINT_R_MAX_<float> / 2.0f,
                            geom::FOOTPRINT_R_MAX_<float> / 2.0f,
                            0.0f, 0.0f,
                            },
    MiningZoneLimiterVector{
                            geom::FOOTPRINT_R_MAX_<float> / 2.0f,
                            geom::FOOTPRINT_R_MAX_<float> / 2.0f,
                            geom::FOOTPRINT_R_MAX_<float>,
                            0.0f, }
};
// inline constexpr std::array<std::array<float, 4>, 4> MINING_ZONE_OFFSETS = {
//     // 4 mining limiters in order of Yplus, Yminus, Xminus, Xplus
//     MiningZoneLimiterVector{
//         0.0f, geom::FOOTPRINT_R_MAX_<float>, geom::FOOTPRINT_R_MAX_<float>/2.0f,0.0f,},
//     MiningZoneLimiterVector{
//         geom::FOOTPRINT_R_MAX_<float>,0.0f,geom::FOOTPRINT_R_MAX_<float>/2.0f,0.0f,},
//     MiningZoneLimiterVector{
//         geom::FOOTPRINT_R_MAX_<float>/2.0f,geom::FOOTPRINT_R_MAX_<float>/2.0f,0.0f,geom::FOOTPRINT_R_MAX_<float>},
//     MiningZoneLimiterVector{
//         geom::FOOTPRINT_R_MAX_<float>/2.0f,geom::FOOTPRINT_R_MAX_<float>/2.0f,geom::FOOTPRINT_R_MAX_<float>,0.0f,}
// };


struct MiningGridGeometry
{
    float mining_zone_x_length = 0.0f;
    float mining_zone_y_length = 0.0f;
    float actual_mining_x_length = 0.0f;
    float actual_mining_y_length = 0.0f;
    float cell_length_x = 0.0f;
    float cell_length_y = 0.0f;
    Eigen::Vector2f min_corner_with_offset = Eigen::Vector2f::Zero();
    Eigen::Vector2f max_corner_with_offset = Eigen::Vector2f::Zero();

    friend std::ostream& operator<<(
        std::ostream& os,
        const MiningGridGeometry& geom)
    {
        os << "  mining_zone_x_length: " << geom.mining_zone_x_length << "\n"
           << "  mining_zone_y_length: " << geom.mining_zone_y_length << "\n"
           << "  actual_mining_x_length: " << geom.actual_mining_x_length
           << "\n"
           << "  actual_mining_y_length: " << geom.actual_mining_y_length
           << "\n"
           << "  cell_length_x: " << geom.cell_length_x << "\n"
           << "  cell_length_y: " << geom.cell_length_y << "\n"
           << "  min_corner_with_offset: (" << geom.min_corner_with_offset.x()
           << ", " << geom.min_corner_with_offset.y() << ")\n"
           << "  max_corner_with_offset: (" << geom.max_corner_with_offset.x()
           << ", " << geom.max_corner_with_offset.y() << ")";
        return os;
    }
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
        const Eigen::MatrixXf& mat,
        const MiningGridGeometry* grid_geometry);

public:
    Vec2f getPointInWorldFrame(const Vec2i& point) const;
    static Vec2f computePointInWorldFrame(
        const Vec2i& point,
        MiningDirection dir,
        int matrix_rows,
        int matrix_cols,
        const MiningGridGeometry& geom);
    static Eigen::AlignedBox2f computeCellBoxInWorldFrame(
        const Vec2i& point,
        MiningDirection dir,
        int matrix_rows,
        int matrix_cols,
        const MiningGridGeometry& geom);
    MiningSwath getPathStartInWorldFrame() const;
    std::vector<Vec2f> getFullPathInWorldFrame() const;


    void markMiningOnMatrix(Eigen::MatrixXi& mined_count_matrix) const;

    inline void print() const
    {
        std::cout << "Path from (" << path.first.x() << ", " << path.first.y()
                  << ") to (" << path.second.x() << ", " << path.second.y()
                  << ") in direction " << static_cast<int>(direction)
                  << " with distance " << distance << "\n";
    }


public:
    // public but only used by MiningPlanner
    float getDistance() const;

    bool checkValidity();

    float getQuality(const Eigen::MatrixXi& previously_minined_locations) const;

    void adjustForRobotClearance();

    MiningDirection getDirection() const { return direction; }

    float getRecalculatedDistance() const;


private:
    MiningGridGeometry grid_geometry;
    MiningPath path;
    MiningDirection direction;
    std::reference_wrapper<const Eigen::MatrixXf> matrix;
    float distance = -1.0;
    static constexpr float PREVIOUSLY_MINED_PENALTY = 0.1f;
};



class MiningPlanner
{
public:
    using Box2f = Eigen::AlignedBox2f;
    using DirectedMiningPaths = std::vector<DirectedMiningPath>;
    using Pose2f = Eigen::Vector3f;

public:
    MiningPlanner(
        MiningEvalInterface& mining_eval,
        const RobotParams& robot_params);

public:
    bool updateMappedMatrices();
    const DirectedMiningPaths& finalOutput();
    const DirectedMiningPaths& getCachedPaths() const
    {
        return all_mining_paths;
    }



    std::unordered_map<MiningDirection, MiningGridGeometry>
        getGridGeometriesByDirection() const;


    // const MiningGridGeometry& getGridGeometry() const { return grid_geometry; }

    void markMiningOnMatrix(const DirectedMiningPath& path);
    bool hasSentRequest() const { return sent_eval_request; }

    // REMOVE LATER JUST FOR TESTING
    std::unordered_map<MiningDirection, Eigen::MatrixXi>
        getPreviouslyMinedCellsByDirection() const;
    
    bool recheckPathValidity(const DirectedMiningPath& path);

private:
    MiningGridGeometry computeMiningGridGeometry(
        const RobotParams& robot_params,
        const MiningZoneLimiterVector& direction_offset) const;

    std::vector<Pose2f> getStartingLocations();

    void appendPlannedMiningPaths();

    void sortPathsByQuality();
    void removeSectionsForRobotClearance();

private:
    const RobotParams& robot_params;
    MiningEvalInterface& mining_eval;


    bool sent_eval_request{false};
    bool sent_validity_request{false};

    std::unordered_map<MiningDirection, MiningGridGeometry> grid_geometry;
    // Each previously mined cell matrix is offset by the same amounts as it's corresponding strip map so their indices align.
    std::unordered_map<MiningDirection, Eigen::MatrixXi>
        previously_mined_cells_by_direction;

    // The direction is the way the the robot would be moving in reference to the
    // base frame which is MiningDirection::YMINUS
    Eigen::MatrixXf strip_map_yplus;
    Eigen::MatrixXf strip_map_yminus;
    Eigen::MatrixXf strip_map_xminus;
    Eigen::MatrixXf strip_map_xplus;
    Eigen::MatrixXf strip_map_xminus_transposed;
    Eigen::MatrixXf strip_map_xplus_transposed;


    DirectedMiningPaths all_mining_paths;
    // JCOMMENT: Why are these hard coded? I would expect as the mining zone size changes, these would change too
    static constexpr int x_divisions = 8;
    static constexpr int y_divisions = 12;
};

};  // namespace lance
