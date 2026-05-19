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

#include "mining_planner.hpp"

#include "robot/model/dynamics.hpp"
#include "robot/model/geometry.hpp"

namespace lance
{
// Used for testing
// inline const char* miningDirectionToString(MiningDirection dir)
// {
//     switch (dir)
//     {
//         case MiningDirection::YPLUS:
//             return "YPLUS";
//         case MiningDirection::YMINUS:
//             return "YMINUS";
//         case MiningDirection::XMINUS:
//             return "XMINUS";
//         case MiningDirection::XPLUS:
//             return "XPLUS";
//         default:
//             return "UNKNOWN";
//     }
// }

DirectedMiningPath::DirectedMiningPath(
    DirectedMiningPath::MiningPath p,
    MiningDirection dir,
    const Eigen::MatrixXf& mat,
    const MiningGridGeometry* geometry) :
    grid_geometry(*geometry),
    path(p),
    direction(dir),
    matrix(std::cref(mat))
{
    distance = this->getRecalculatedDistance();
}

float DirectedMiningPath::getDistance() const { return distance; }

void DirectedMiningPath::markMiningOnMatrix(
    Eigen::MatrixXi& mined_count_matrix) const
{
    MiningPath p = path;
    if (p.first.x() > p.second.x())
    {
        std::swap(p.first.x(), p.second.x());
    }
    if (p.first.y() > p.second.y())
    {
        std::swap(p.first.y(), p.second.y());
    }
    for (int i = p.first.x(); i <= p.second.x(); i++)
    {
        for (int j = p.first.y(); j <= p.second.y(); j++)
        {
            mined_count_matrix(i, j) += 1;
        }
    }
}

bool DirectedMiningPath::checkValidity()
{
    if (path.first.x() < 0 || path.first.y() < 0 || path.second.x() < 0 ||
        path.second.y() < 0)
    {
        return false;
    }
    if (path.first.x() >= this->matrix.get().rows() ||
        path.second.x() >= this->matrix.get().rows())
    {
        return false;
    }
    if (path.first.y() >= this->matrix.get().cols() ||
        path.second.y() >= this->matrix.get().cols())
    {
        return false;
    }

    if (!(path.first.x() == path.second.x() ||
          path.first.y() == path.second.y()))
    {
        return false;
    }

    for (int i = path.first.x(); i <= path.second.x(); ++i)
    {
        for (int j = path.first.y(); j <= path.second.y(); ++j)
        {
            if (this->matrix.get()(i, j) == 0)
            {
                return false;
            }
        }
    }
    // After checking valid path make sure it has a positive distance.
    distance = this->getRecalculatedDistance();

    if (distance <= 0.0f)
    {
        return false;
    }

    return distance >=
           0.0f;  // Could be changed to a larger value for a shortest path requirement.
}

float DirectedMiningPath::getQuality(
    const Eigen::MatrixXi& previously_mined_cells) const
{
    float quality = 0.0f;
    quality += distance;

    MiningPath p = path;
    if (p.first.x() > p.second.x())
    {
        std::swap(p.first.x(), p.second.x());
    }
    if (p.first.y() > p.second.y())
    {
        std::swap(p.first.y(), p.second.y());
    }
    for (int i = p.first.x(); i <= p.second.x(); i++)
    {
        for (int j = p.first.y(); j <= p.second.y(); j++)
        {
            quality -= previously_mined_cells(i, j) * PREVIOUSLY_MINED_PENALTY;
        }
    }

    return quality;
}

void DirectedMiningPath::adjustForRobotClearance()
{
    /* Adjusts the path for robot clearance requirements. */


    int robot_clearance_exclusion_count_x = static_cast<int>(std::ceil(
        (geom::PRIMARY_COLLISION_ZONE_LENGTH_OFFSET_<float> / 2.0f) /
        this->grid_geometry.cell_length_x));
    int robot_clearance_exclusion_count_y = static_cast<int>(std::ceil(
        (geom::PRIMARY_COLLISION_ZONE_LENGTH_OFFSET_<float> / 2.0f) /
        this->grid_geometry.cell_length_y));

    switch (direction)
    {
        case MiningDirection::YMINUS:
            if (path.first.x() != 0)
            {
                path.first.x() += robot_clearance_exclusion_count_y;
            }
            break;
        case MiningDirection::YPLUS:
            if (path.first.x() != this->matrix.get().rows() - 1)
            {
                path.first.x() -= robot_clearance_exclusion_count_y;
            }
            break;
        case MiningDirection::XPLUS:
            if (path.first.y() != 0)
            {
                path.first.y() += robot_clearance_exclusion_count_x;
            }
            break;
        case MiningDirection::XMINUS:
            if (path.first.y() != this->matrix.get().cols() - 1)
            {
                path.first.y() -= robot_clearance_exclusion_count_x;
            }
            break;
    }
}

DirectedMiningPath::Vec2f DirectedMiningPath::computePointInWorldFrame(
    const Vec2i& point,
    MiningDirection direction,
    int matrix_rows,
    int matrix_cols,
    const MiningGridGeometry& grid_geometry)
{
    Vec2f world_point;

    const float cell_size_x = grid_geometry.cell_length_x;
    const float cell_size_y = grid_geometry.cell_length_y;
    const float half_cell_x = cell_size_x / 2.0f;
    const float half_cell_y = cell_size_y / 2.0f;

    const Eigen::Vector2f min_corner_with_offset =
        grid_geometry.min_corner_with_offset;
    const Eigen::Vector2f max_corner_with_offset =
        grid_geometry.max_corner_with_offset;
    switch (direction)
    {
        case MiningDirection::YMINUS:
            world_point.x() = point.y() * grid_geometry.cell_length_x +
                              min_corner_with_offset.x() + half_cell_x;
            world_point.y() = max_corner_with_offset.y() -
                              (point.x() * grid_geometry.cell_length_y);
            break;
        case MiningDirection::YPLUS:
            world_point.x() = point.y() * grid_geometry.cell_length_x +
                              min_corner_with_offset.x() + half_cell_x;
            world_point.y() =
                min_corner_with_offset.y() +
                (matrix_rows - 1 - point.x()) * grid_geometry.cell_length_y;
            break;
        case MiningDirection::XPLUS:
            world_point.x() = point.y() * grid_geometry.cell_length_x +
                              min_corner_with_offset.x();
            world_point.y() = max_corner_with_offset.y() -
                              (point.x() * grid_geometry.cell_length_y) -
                              half_cell_y;
            break;
        case MiningDirection::XMINUS:
            world_point.x() =
                max_corner_with_offset.x() -
                ((matrix_cols - 1 - point.y()) * grid_geometry.cell_length_x);
            world_point.y() = max_corner_with_offset.y() -
                              (point.x() * grid_geometry.cell_length_y) -
                              half_cell_y;
            break;
    }
    return world_point;
}

DirectedMiningPath::Vec2f DirectedMiningPath::computeRobotOriginInWorldFrame(
    const Vec2i& point,
    MiningDirection direction,
    int matrix_rows,
    int matrix_cols,
    const MiningGridGeometry& grid_geometry)
{
    Vec2f robot_origin = computePointInWorldFrame(
        point,
        direction,
        matrix_rows,
        matrix_cols,
        grid_geometry);

    switch (direction)
    {
        case MiningDirection::YPLUS:
            robot_origin.y() -= geom::FOOTPRINT_R_MAX_<float>;
            break;
        case MiningDirection::YMINUS:
            robot_origin.y() += geom::FOOTPRINT_R_MAX_<float>;
            break;
        case MiningDirection::XPLUS:
        case MiningDirection::XMINUS:
            break;
    }

    return robot_origin;
}

Eigen::AlignedBox2f DirectedMiningPath::computeCellBoxInWorldFrame(
    const Vec2i& point,
    MiningDirection direction,
    int matrix_rows,
    int matrix_cols,
    const MiningGridGeometry& grid_geometry)
{
    Vec2f min_point = computePointInWorldFrame(
        point,
        direction,
        matrix_rows,
        matrix_cols,
        grid_geometry);
    Vec2f offset = Vec2f(0, 0);
    switch (direction)
    {
        case MiningDirection::YMINUS:
            offset = Vec2f(
                grid_geometry.cell_length_x,
                -grid_geometry.cell_length_y);
            min_point.x() -= grid_geometry.cell_length_x / 2.0f;
            break;
        case MiningDirection::YPLUS:
            offset =
                Vec2f(grid_geometry.cell_length_x, grid_geometry.cell_length_y);
            min_point.x() -= grid_geometry.cell_length_x / 2.0f;
            break;
        case MiningDirection::XPLUS:
            offset = Vec2f(grid_geometry.cell_length_x, 0);
            min_point.y() -= grid_geometry.cell_length_y / 2.0f;
            break;
        case MiningDirection::XMINUS:
            offset = Vec2f(-grid_geometry.cell_length_x, 0);
            min_point.y() -= grid_geometry.cell_length_y / 2.0f;
            break;
    }
    Vec2f max_point = min_point + offset;

    // Eigen::AlignedBox2f handles sorting the min and max corners properly
    return Eigen::AlignedBox2f(
        min_point.cwiseMin(max_point),
        min_point.cwiseMax(max_point));
}

DirectedMiningPath::Vec2f DirectedMiningPath::getPointInWorldFrame(
    const Vec2i& point) const
{
    return computePointInWorldFrame(
        point,
        this->direction,
        matrix.get().rows(),
        matrix.get().cols(),
        this->grid_geometry);
}

std::vector<DirectedMiningPath::Vec2f>
    DirectedMiningPath::getFullPathInWorldFrame() const
{
    std::vector<Vec2f> full_path;
    switch (direction)
    {
        case MiningDirection::YMINUS:
            for (int i = 0; i < path.second.x() - path.first.x(); i++)
            {
                Vec2f point =
                    getPointInWorldFrame({path.first.x() + i, path.first.y()});
                full_path.emplace_back(point);
            }
            break;
        case MiningDirection::YPLUS:
            for (int i = 0; i < path.first.x() - path.second.x(); i++)
            {
                Vec2f point =
                    getPointInWorldFrame({path.first.x() - i, path.first.y()});
                full_path.emplace_back(point);
            }
            break;
        case MiningDirection::XPLUS:
            for (int i = 0; i < path.second.y() - path.first.y(); i++)
            {
                Vec2f point =
                    getPointInWorldFrame({path.first.x(), path.first.y() + i});
                full_path.emplace_back(point);
            }
            break;
        case MiningDirection::XMINUS:
            for (int i = 0; i < path.first.y() - path.second.y(); i++)
            {
                Vec2f point =
                    getPointInWorldFrame({path.first.x(), path.first.y() - i});
                full_path.emplace_back(point);
            }
            break;
    }
    return full_path;
}
DirectedMiningPath::MiningSwath DirectedMiningPath::getPathStartInWorldFrame()
    const
{
    Vec2f world_point = computeRobotOriginInWorldFrame(
        path.first,
        this->direction,
        matrix.get().rows(),
        matrix.get().cols(),
        this->grid_geometry);

    Eigen::Vector2f target_pos = world_point;
    Eigen::Vector2f target_dir;

    switch (direction)
    {
        case MiningDirection::YMINUS:
            target_dir = Eigen::Vector2f({0.f, -1.f});
            break;
        case MiningDirection::YPLUS:
            target_dir = Eigen::Vector2f({0.f, 1.f});
            break;
        case MiningDirection::XPLUS:
            target_dir = Eigen::Vector2f({1.f, 0.f});
            break;
        case MiningDirection::XMINUS:
            target_dir = Eigen::Vector2f({-1.f, 0.f});
            break;
    }

    return {target_pos, target_dir};
}

float DirectedMiningPath::getRecalculatedDistance() const
{
    const auto& start = this->path.first;
    const auto& end = this->path.second;

    if (end.x() != start.x())
    {
        // For YPLUS/YMINUS, the distance is based on the change in y position with is x of vectors (row index)
        const float distance_in_cells =
            (end.x() - start.x()) *
                (direction == MiningDirection::YPLUS ? -1.0f : 1.0f) +
            1.0f - (1.0f - this->matrix.get()(end.x(), end.y()));
        return distance_in_cells * grid_geometry.cell_length_y;
    }
    // For XPLUS/XMINUS, the distance is based on the change in x position which is y of vectors (column index)
    const float distance_in_cells =
        (end.y() - start.y()) *
            (direction == MiningDirection::XMINUS ? -1.0f : 1.0f) +
        1.0f - (1.0f - this->matrix.get()(end.x(), end.y()));
    return distance_in_cells * grid_geometry.cell_length_x;
}

MiningGridGeometry MiningPlanner::computeMiningGridGeometry(
    const RobotParams& robot_params,
    const MiningZoneLimiterVector& direction_offset) const
{
    MiningGridGeometry geometry;

    geometry.mining_zone_x_length = robot_params.bounds.mining_zone.max().x() -
                                    robot_params.bounds.mining_zone.min().x();
    geometry.mining_zone_y_length = robot_params.bounds.mining_zone.max().y() -
                                    robot_params.bounds.mining_zone.min().y();

    // Direction offset is {Yplus, Yminus, Xminus, Xplus}
    geometry.actual_mining_x_length = std::max(
        0.0f,
        geometry.mining_zone_x_length - direction_offset[2] -
            direction_offset[3]);
    geometry.actual_mining_y_length = std::max(
        0.0f,
        geometry.mining_zone_y_length - direction_offset[0] -
            direction_offset[1]);

    geometry.cell_length_x =
        geometry.actual_mining_x_length / static_cast<float>(x_divisions);
    geometry.cell_length_y =
        geometry.actual_mining_y_length / static_cast<float>(y_divisions);

    geometry.min_corner_with_offset =
        robot_params.bounds.mining_zone.min() +
        Eigen::Vector2f(direction_offset[2], direction_offset[0]);
    geometry.max_corner_with_offset =
        robot_params.bounds.mining_zone.max() -
        Eigen::Vector2f(direction_offset[3], direction_offset[1]);

    return geometry;
}

MiningPlanner::MiningPlanner(
    MiningEvalInterface& mining_eval,
    const RobotParams& robot_params) :
    robot_params(robot_params),
    mining_eval(mining_eval),
    grid_geometry(getGridGeometriesByDirection())
{
    // UP/DOWN are (width, height) = (5, 4)
    strip_map_yplus = Eigen::MatrixXf::Zero(y_divisions, x_divisions);
    strip_map_yminus = Eigen::MatrixXf::Zero(y_divisions, x_divisions);

    // LEFT/RIGHT are swapped: (height, width) = (4, 5)
    strip_map_xminus = Eigen::MatrixXf::Zero(y_divisions, x_divisions);
    strip_map_xplus = Eigen::MatrixXf::Zero(y_divisions, x_divisions);
    strip_map_xminus_transposed =
        Eigen::MatrixXf::Zero(x_divisions, y_divisions);
    strip_map_xplus_transposed =
        Eigen::MatrixXf::Zero(x_divisions, y_divisions);

    this->previously_mined_cells_by_direction[MiningDirection::YPLUS] =
        Eigen::MatrixXi::Zero(strip_map_yplus.rows(), strip_map_yplus.cols());
    this->previously_mined_cells_by_direction[MiningDirection::YMINUS] =
        Eigen::MatrixXi::Zero(strip_map_yminus.rows(), strip_map_yminus.cols());
    this->previously_mined_cells_by_direction[MiningDirection::XMINUS] =
        Eigen::MatrixXi::Zero(strip_map_xminus.rows(), strip_map_xminus.cols());
    this->previously_mined_cells_by_direction[MiningDirection::XPLUS] =
        Eigen::MatrixXi::Zero(strip_map_xplus.rows(), strip_map_xplus.cols());
}

std::unordered_map<MiningDirection, MiningGridGeometry>
    MiningPlanner::getGridGeometriesByDirection() const
{
    std::unordered_map<MiningDirection, MiningGridGeometry> geometries;
    for (const auto& direction : ALL_MINING_DIRECTIONS)
    {
        const auto& direction_offset =
            MINING_ZONE_OFFSETS[static_cast<size_t>(direction)];
        geometries.emplace(
            direction,
            computeMiningGridGeometry(this->robot_params, direction_offset));
    }
    return geometries;
}

// Used periodically to check if the path is still valid as you traverse to the
// path with updated lidar points
bool MiningPlanner::recheckPathValidity(const DirectedMiningPath& path)
{
    if (!sent_validity_request)
    {
        const DirectedMiningPath::MiningSwath swath =
            path.getPathStartInWorldFrame();
        MiningPlanner::Pose2f start_position = Pose2f(
            swath.first.x(),
            swath.first.y(),
            miningDirectionToRadians(path.getDirection()));
        std::cout << "Loading the query with  " << swath.first.x() << " , "
                  << swath.first.y() << " , "
                  << miningDirectionToRadians(path.getDirection()) << "\n";
        mining_eval.queryArenaFrame(
            std::vector<MiningPlanner::Pose2f>{start_position});
        sent_validity_request = true;
    }
    if (!mining_eval.hasResult())
    {
        return true;  // temporarily return true while waiting for the result
    }
    sent_validity_request = false;

    const std::vector<float>* distances = mining_eval.getDists();
    if (distances == nullptr || distances->empty())
    {
        std::cerr << "Mining path validity result is missing.\n";
        return false;
    }

    const float distance = distances->front();
    constexpr float validity_tolerance_m = 0.05f;

    std::cout << "Rechecking path validity. Distance to obstacle: " << distance
              << " meters. Path distance: " << path.getDistance() << " meters.\n";
    return path.getDistance() <= distance + validity_tolerance_m;
}

bool MiningPlanner::updateMappedMatrices()
{
    const size_t grid_cell_count = static_cast<size_t>(
        this->strip_map_yplus.rows() * this->strip_map_yplus.cols());
    if (grid_cell_count == 0)
    {
        // No valid planning grid means no query/results are needed.
        this->sent_eval_request = false;
        return true;
    }

    if (!sent_eval_request)
    {
        // Sends the starting locations to the eval
        sent_validity_request = false;
        mining_eval.queryArenaFrame(this->getStartingLocations());
        sent_eval_request = true;
    }

    if (!mining_eval.hasResult())
    {
        std::cerr << "Mining evaluation data not ready yet.\n";
        return false;
    }
    sent_eval_request = false;
    const std::vector<float>* mining_eval_distances = mining_eval.getDists();
    if (mining_eval_distances == nullptr)
    {
        std::cerr << "Mining evaluation distances are missing.\n";
        return false;
    }

    const size_t expected_eval_count = 4 * grid_cell_count;
    if (mining_eval_distances->size() < expected_eval_count)
    {
        std::cerr << "Mining evaluation size mismatch: expected at least "
                  << expected_eval_count << ", got "
                  << mining_eval_distances->size() << ".\n";
        return false;
    }

    size_t mining_eval_index = 0;
    for (int i = 0; i < strip_map_yplus.cols(); i++)
    {
        for (int j = strip_map_yplus.rows() - 1; j >= 0; j--)
        {
            strip_map_yplus(j, i) = std::clamp(
                (*mining_eval_distances)[mining_eval_index++] /
                    grid_geometry[MiningDirection::YPLUS].cell_length_y,
                0.0f,
                1.0f);
        }
    }
    // DOWN
    for (int i = 0; i < strip_map_yminus.cols(); i++)
    {
        for (int j = 0; j < strip_map_yminus.rows(); j++)
        {
            strip_map_yminus(j, i) = std::clamp(
                (*mining_eval_distances)[mining_eval_index++] /
                    grid_geometry[MiningDirection::YMINUS].cell_length_y,
                0.0f,
                1.0f);
        }
    }
    // LEFT
    for (int i = strip_map_xminus.cols() - 1; i >= 0; i--)
    {
        for (int j = 0; j < strip_map_xminus.rows(); j++)
        {
            strip_map_xminus(j, i) = std::clamp(
                (*mining_eval_distances)[mining_eval_index++] /
                    grid_geometry[MiningDirection::XMINUS].cell_length_x,
                0.0f,
                1.0f);
        }
    }
    // RIGHT
    for (int i = 0; i < strip_map_xplus.cols(); i++)
    {
        for (int j = 0; j < strip_map_xplus.rows(); j++)
        {
            strip_map_xplus(j, i) = std::clamp(
                (*mining_eval_distances)[mining_eval_index++] /
                    grid_geometry[MiningDirection::XPLUS].cell_length_x,
                0.0f,
                1.0f);
        }
    }
    strip_map_xminus_transposed = strip_map_xminus.transpose();
    strip_map_xplus_transposed = strip_map_xplus.transpose();
    return true;
}
const MiningPlanner::DirectedMiningPaths& MiningPlanner::finalOutput()
{
    std::cout << "Clearing all paths\n";
    this->all_mining_paths.clear();
    std::cout << "Appending planned paths\n";
    this->appendPlannedMiningPaths();
    std::cout << "Removing sections for robot clearance\n";
    // print current length
    std::cout << "Current path count: " << this->all_mining_paths.size()
              << "\n";
    this->removeSectionsForRobotClearance();
    std::cout << "After clearance path count: " << this->all_mining_paths.size()
              << "\n";
    std::cout << "Sorting paths by quality\n";
    this->sortPathsByQuality();
    std::cout << "Final path count: " << this->all_mining_paths.size() << "\n";

    return all_mining_paths;
}

void MiningPlanner::markMiningOnMatrix(const DirectedMiningPath& path)
{
    path.markMiningOnMatrix(
        this->previously_mined_cells_by_direction[path.getDirection()]);

    // Loop through the other 3 matrices and mark the corresponding cells as mined
    //  To do this, you'll need to convert each of the indices to a box with the real world coordinates then check to see if it overlaps with any of the indices in the path
    std::vector<DirectedMiningPath::Vec2f> path_full =
        path.getFullPathInWorldFrame();
    // make one big box for the whole mining path
    Eigen::Vector2f min_point = path_full[0];
    Eigen::Vector2f max_point = path_full[0];

    for (const auto& pt : path_full)
    {
        min_point.x() = std::min(min_point.x(), pt.x());
        min_point.y() = std::min(min_point.y(), pt.y());
        max_point.x() = std::max(max_point.x(), pt.x());
        max_point.y() = std::max(max_point.y(), pt.y());
    }

    // // Expand the box slightly to encompass the full cell footprint (radius = half cell width/height)
    // min_point -= Eigen::Vector2f(path.grid_geometry.cell_length_x / 2.0f, path.grid_geometry.cell_length_y / 2.0f);
    // max_point += Eigen::Vector2f(path.grid_geometry.cell_length_x / 2.0f, path.grid_geometry.cell_length_y / 2.0f);

    Eigen::AlignedBox2f path_box(min_point, max_point);

    for (auto& [direction, matrix] : this->previously_mined_cells_by_direction)
    {
        if (direction != path.getDirection())
        {
            // loop through each cell in the matrix
            for (int i = 0; i < matrix.rows(); i++)
            {
                for (int j = 0; j < matrix.cols(); j++)
                {
                    // Get the position of the current cell in the world frame using the current direction's geometry
                    Eigen::AlignedBox2f cell_box =
                        DirectedMiningPath::computeCellBoxInWorldFrame(
                            {i, j},
                            direction,
                            matrix.rows(),
                            matrix.cols(),
                            this->grid_geometry.at(direction));

                    if (path_box.intersects(cell_box))
                    {
                        matrix(i, j) += 1;
                    }
                }
            }
        }
    }
}

std::vector<MiningPlanner::Pose2f> MiningPlanner::getStartingLocations()
{
    std::vector<MiningPlanner::Pose2f> starting_vectors;
    starting_vectors.clear();
    starting_vectors.reserve(
        static_cast<size_t>(4 * x_divisions * y_divisions));

    auto append_pose = [this, &starting_vectors](
                           MiningDirection direction,
                           const DirectedMiningPath::Vec2i& point)
    {
        const DirectedMiningPath::Vec2f world_point =
            DirectedMiningPath::computeRobotOriginInWorldFrame(
                point,
                direction,
                y_divisions,
                x_divisions,
                this->grid_geometry.at(direction));
        starting_vectors.emplace_back(
            world_point.x(),
            world_point.y(),
            miningDirectionToRadians(direction));
    };

    for (int col = 0; col < x_divisions; col++)
    {
        for (int row = y_divisions - 1; row >= 0; row--)
        {
            append_pose(MiningDirection::YPLUS, {row, col});
        }
    }
    for (int col = 0; col < x_divisions; col++)
    {
        for (int row = 0; row < y_divisions; row++)
        {
            append_pose(MiningDirection::YMINUS, {row, col});
        }
    }
    for (int col = x_divisions - 1; col >= 0; col--)
    {
        for (int row = 0; row < y_divisions; row++)
        {
            append_pose(MiningDirection::XMINUS, {row, col});
        }
    }
    for (int col = 0; col < x_divisions; col++)
    {
        for (int row = 0; row < y_divisions; row++)
        {
            append_pose(MiningDirection::XPLUS, {row, col});
        }
    }
    return starting_vectors;
}

void MiningPlanner::appendPlannedMiningPaths()
{
    int a = 0, b = 0;
    size_t pushed_paths = 0;
    size_t skipped_short_paths = 0;

    std::vector<MiningDirection> directions = {
        MiningDirection::YMINUS,
        MiningDirection::XPLUS,
        MiningDirection::YPLUS,
        MiningDirection::XMINUS};
    std::vector<Eigen::MatrixXf*> mats = {
        &strip_map_yminus,
        &strip_map_xplus_transposed,
        &strip_map_yplus,
        &strip_map_xminus_transposed};

    // Print lots of relevant info for debugging
    std::cout << "\n================ MINING PLANNER DEBUG =================\n"
              << "Mining Zone Bounds:\n"
              << "  Min: (" << robot_params.bounds.mining_zone.min().x() << ", "
              << robot_params.bounds.mining_zone.min().y() << ")\n"
              << "  Max: (" << robot_params.bounds.mining_zone.max().x() << ", "
              << robot_params.bounds.mining_zone.max().y() << ")\n\n"
              << "Calculated Grid Size:\n"
              << "  Columns = " << strip_map_yplus.cols() << "\n"
              << "  Rows    = " << strip_map_yplus.rows() << "\n\n"
              << "--- Grid Geometry By Direction ---\n";

    // print grid geometry for each direction
    for (const auto& [dir, geom] : grid_geometry)
    {
        std::cout << "[" << toString(dir) << "]\n" << geom << "\n\n";
    }

    // print all the strip maps
    std::cout << "--- Strip Maps ---\n"
              << "[YMINUS]\n"
              << strip_map_yminus << "\n\n"
              << "[YPLUS]\n"
              << strip_map_yplus << "\n\n"
              << "[XMINUS]\n"
              << strip_map_xminus << "\n\n"
              << "[XPLUS]\n"
              << strip_map_xplus << "\n\n"
              << "[XMINUS_TRANSPOSED]\n"
              << strip_map_xminus_transposed << "\n\n"
              << "[XPLUS_TRANSPOSED]\n"
              << strip_map_xplus_transposed << "\n\n";

    // Print out the previously mined cells for each direction
    std::cout << "--- Previously Mined Cells ---\n";
    for (const auto& [dir, matrix] : previously_mined_cells_by_direction)
    {
        std::cout << "[" << toString(dir) << "]\n" << matrix << "\n\n";
    }
    std::cout << "=======================================================\n";



    // Top to Bottom search (DOWN and RIGHT)
    for (int direction_index = 0; direction_index < 2; direction_index++)
    {
        Eigen::MatrixXf* mat = mats[direction_index];
        MiningDirection mining_dir = directions[direction_index];

        int width = mat->cols();
        int height = mat->rows();

        for (int i = 0; i < width; i++)
        {
            a = 0;
            b = 0;

            while (b < height)
            {
                const float va = (*mat)(a, i);
                const float vb = (*mat)(b, i);

                if (va == 0)
                {
                    a++;
                    b = a;
                }
                else if (vb != 1 || b == height - 1)
                {
                    if (vb > 0)
                    {
                        b++;
                    }

                    DirectedMiningPath::MiningPath possible_path = std::pair(
                        Eigen::Vector2i(a, i),
                        Eigen::Vector2i(b - 1, i));

                    if (MiningDirection::XPLUS == mining_dir)
                    {
                        // Undo transpose for RIGHT directional search.
                        possible_path = std::pair(
                            Eigen::Vector2i(i, possible_path.first.x()),
                            Eigen::Vector2i(i, possible_path.second.x()));
                    }

                    Eigen::MatrixXf& original_mat =
                        (MiningDirection::YMINUS == mining_dir)
                            ? strip_map_yminus
                            : strip_map_xplus;
                    DirectedMiningPath dir_mining_path(
                        possible_path,
                        mining_dir,
                        original_mat,
                        &grid_geometry[mining_dir]);

                    const float dist = dir_mining_path.getDistance();

                    if (dist >= robot_params.auto_mining_min_path_length)
                    {
                        this->all_mining_paths.emplace_back(
                            std::move(dir_mining_path));
                        pushed_paths++;
                    }
                    else
                    {
                        skipped_short_paths++;
                    }

                    if (a == b)
                    {
                        b++;
                    }
                    a = b;
                }
                else if (vb == 1)
                {
                    b++;
                }
                else
                {
                    b++;
                }
            }
        }
    }

    // Bottom to Top search (UP and LEFT)
    for (int direction_index = 2; direction_index < 4; direction_index++)
    {
        Eigen::MatrixXf* mat = mats[direction_index];
        MiningDirection mining_dir = directions[direction_index];

        int width = mat->cols();
        int height = mat->rows();

        for (int i = 0; i < width; i++)
        {
            a = height - 1;
            b = height - 1;

            while (b >= 0)
            {
                const float va = (*mat)(a, i);
                const float vb = (*mat)(b, i);

                if (va == 0)
                {
                    a--;
                    b = a;
                }
                else if (vb != 1 || b == 0)
                {
                    if (vb > 0)
                    {
                        b--;
                    }

                    DirectedMiningPath::MiningPath possible_path = std::pair(
                        Eigen::Vector2i(a, i),
                        Eigen::Vector2i(b + 1, i));

                    if (MiningDirection::XMINUS == mining_dir)
                    {
                        // Undo transpose for LEFT directional search.
                        possible_path = std::pair(
                            Eigen::Vector2i(i, possible_path.first.x()),
                            Eigen::Vector2i(i, possible_path.second.x()));
                    }

                    const Eigen::MatrixXf& original_mat =
                        (MiningDirection::YPLUS == mining_dir)
                            ? strip_map_yplus
                            : strip_map_xminus;
                    DirectedMiningPath dir_mining_path(
                        possible_path,
                        mining_dir,
                        original_mat,
                        &grid_geometry[mining_dir]);

                    const float dist = dir_mining_path.getDistance();

                    if (dist >= robot_params.auto_mining_min_path_length)
                    {
                        this->all_mining_paths.emplace_back(dir_mining_path);
                        pushed_paths++;
                    }
                    else
                    {
                        skipped_short_paths++;
                    }

                    if (a == b)
                    {
                        b--;
                    }
                    a = b;
                }
                else if (vb == 1)
                {
                    b--;
                }
                else
                {
                    b--;
                }
            }
        }
    }
}

void MiningPlanner::sortPathsByQuality()
{
    std::sort(
        this->all_mining_paths.begin(),
        this->all_mining_paths.end(),
        [this](const DirectedMiningPath& a, const DirectedMiningPath& b)
        {
            return a.getQuality(this->previously_mined_cells_by_direction
                                    [a.getDirection()]) >
                   b.getQuality(this->previously_mined_cells_by_direction
                                    [b.getDirection()]);
        });
}

void MiningPlanner::removeSectionsForRobotClearance()
{
    std::erase_if(
        this->all_mining_paths,
        [](DirectedMiningPath& path)
        {
            path.adjustForRobotClearance();
            return !path.checkValidity();
        });
}
// REMOVE LATER JUST FOR TESTING
std::unordered_map<MiningDirection, Eigen::MatrixXi>
    MiningPlanner::getPreviouslyMinedCellsByDirection() const
{
    return this->previously_mined_cells_by_direction;
}

};  // namespace lance
