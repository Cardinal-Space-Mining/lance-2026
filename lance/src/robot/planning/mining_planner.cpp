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
    const Eigen::MatrixXf* mat,
    const MiningGridGeometry* geometry) :
    grid_geometry(*geometry),
    path(std::move(p)),
    direction(dir),
    matrix(mat)
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

bool DirectedMiningPath::checkValidity() const
{
    if (matrix == nullptr)
    {
        return false;
    }

    if (path.first.x() < 0 || path.first.y() < 0 || path.second.x() < 0 ||
        path.second.y() < 0)
    {
        return false;
    }
    if (path.first.x() >= matrix->rows() || path.second.x() >= matrix->rows())
    {
        return false;
    }
    if (path.first.y() >= matrix->cols() || path.second.y() >= matrix->cols())
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
            if ((*matrix)(i, j) == 0)
            {
                return false;
            }
        }
    }
    return true;
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
            quality -= previously_mined_cells(i, j) * previously_mined_penalty;
        }
    }

    return quality;
}

bool DirectedMiningPath::adjustForRobotClearance()
{
    /* Adjusts the path for robot clearance requirements. */


    int robot_clearance_exclusion_count_x = static_cast<int>(std::ceil(
        (geom::PRIMARY_COLLISION_ZONE_LENGTH_OFFSET_<float>/2.0f) / this->grid_geometry.cell_length_x));
    int robot_clearance_exclusion_count_y = static_cast<int>(std::ceil(
        (geom::PRIMARY_COLLISION_ZONE_LENGTH_OFFSET_<float>/2.0f) / this->grid_geometry.cell_length_y));

    switch (direction)
    {
        case MiningDirection::YMINUS:
            if (path.first.x() != 0)
            {
                path.first.x() += robot_clearance_exclusion_count_y;
            }
            break;
        case MiningDirection::YPLUS:
            if (path.first.x() != matrix->rows() - 1)
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
            if (path.first.y() != matrix->cols() - 1)
            {
                path.first.y() -= robot_clearance_exclusion_count_x;
            }
            break;
    }

    if (!checkValidity())
    {
        return false;
    }
    distance = this->getRecalculatedDistance();

    if (distance <= 0.0f)
    {
        return false;
    }

    return distance >= 0.0f;
}

DirectedMiningPath::Vec2f DirectedMiningPath::getPointInWorldFrame(const Vec2i& point) const
{
    Vec2f world_point;

    const int matrix_rows = matrix->rows();
    const int matrix_cols = matrix->cols();
    const float cell_size_x = grid_geometry.cell_length_x;
    const float cell_size_y = grid_geometry.cell_length_y;
    const float half_cell_x = cell_size_x / 2.0f;
    const float half_cell_y = cell_size_y / 2.0f;

    const Eigen::Vector2f min_corner_with_offset = grid_geometry.min_corner_with_offset;
    const Eigen::Vector2f max_corner_with_offset = grid_geometry.max_corner_with_offset;
    switch (direction)
    {
        case MiningDirection::YMINUS:
            world_point.x() = path.first.y() * grid_geometry.cell_length_x + min_corner_with_offset.x() +
                   half_cell_x;
            world_point.y() = max_corner_with_offset.y() - (path.first.x() * grid_geometry.cell_length_y);
            break;
        case MiningDirection::YPLUS:
            world_point.x() = path.first.y() * grid_geometry.cell_length_x + min_corner_with_offset.x() +
                   half_cell_x;
             world_point.y() = min_corner_with_offset.y() +
                 (matrix_rows - 1 - path.first.x()) * grid_geometry.cell_length_y;
            break;
        case MiningDirection::XPLUS:
            world_point.x() = path.first.y() * grid_geometry.cell_length_x + min_corner_with_offset.x();
            world_point.y() = max_corner_with_offset.y() - (path.first.x() * grid_geometry.cell_length_y) -
                   half_cell_y;
            break;
        case MiningDirection::XMINUS:
            world_point.x() = max_corner_with_offset.x() -
                 ((matrix_cols - 1 - path.first.y()) * grid_geometry.cell_length_x);
            world_point.y() = max_corner_with_offset.y() - (path.first.x() * grid_geometry.cell_length_y) -
                   half_cell_y;
            break;
    }
    return world_point;
}

std::vector<DirectedMiningPath::Vec2f> DirectedMiningPath::getFullPathInWorldFrame() const
{
    float stdX = 0.f;
    float stdY = 0.f;
    std::vector<Vec2f> full_path;

    Eigen::Vector2f target_pos;
    Eigen::Vector2f target_dir;

    const int matrix_rows = matrix->rows();
    const int matrix_cols = matrix->cols();

    // const float half_track_sep = TRACK_SEPARATION_M_<float> / 2.0f;
    const float cell_size_x = grid_geometry.cell_length_x;
    const float cell_size_y = grid_geometry.cell_length_y;
    const float half_cell_x = cell_size_x / 2.0f;
    const float half_cell_y = cell_size_y / 2.0f;

    const Eigen::Vector2f min_corner_with_offset = grid_geometry.min_corner_with_offset;
    const Eigen::Vector2f max_corner_with_offset = grid_geometry.max_corner_with_offset;

    // const Eigen::Vector2f min_corner_with_offset =
    //     (grid_geometry != nullptr)
    //         ? grid_geometry->min_corner_with_offset
    //         : robot_params.bounds.mining_zone.min() +
    //               Eigen::Vector2f::Constant(r);

    switch (direction)
    {
        case MiningDirection::YMINUS:
            for (int i = 0; i < path.second.x() - path.first.x(); i++)
            {
                stdX = path.first.y() * grid_geometry.cell_length_x +
                             min_corner_with_offset.x() + half_cell_x;
                stdY = max_corner_with_offset.y() -
                             ((path.first.x() + i) * grid_geometry.cell_length_y);

                full_path.emplace_back(stdX, stdY);
            }
            break;
        case MiningDirection::YPLUS:
            for (int i = 0; i < path.first.x() - path.second.x(); i++) 
            {
                stdX = path.first.y() * grid_geometry.cell_length_x + min_corner_with_offset.x() +
                             half_cell_x;
                stdY = min_corner_with_offset.y() +
                             (matrix_rows - 1 - (path.first.x() - i)) * grid_geometry.cell_length_y;

                full_path.emplace_back(stdX, stdY);
            }
            break;
        case MiningDirection::XPLUS:
            for (int i = 0; i < path.second.y() - path.first.y(); i++)
            {
                stdX = path.first.y() * grid_geometry.cell_length_x + min_corner_with_offset.x() +
                              (i * grid_geometry.cell_length_x);
                stdY = max_corner_with_offset.y() - (path.first.x() * grid_geometry.cell_length_y) -
                   half_cell_y;

                full_path.emplace_back(stdX, stdY);
            }
            break;
        case MiningDirection::XMINUS:
            for (int i = 0; i < path.first.y() - path.second.y(); i++)
            {
                stdX = max_corner_with_offset.x() -
                             ((matrix_cols - 1 - path.first.y()) * grid_geometry.cell_length_x) - (i * grid_geometry.cell_length_x);
                stdY = max_corner_with_offset.y() - (path.first.x() * grid_geometry.cell_length_y) -
                   half_cell_y;

                full_path.emplace_back(stdX, stdY);
            }
            break;
    }
    return full_path;
}
DirectedMiningPath::MiningSwath
    DirectedMiningPath::getPathStartInWorldFrame() const
{


    Vec2f world_point = getPointInWorldFrame(path.first);

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
    if (this->matrix == nullptr)
    {
        return -1.0f;
    }

    const auto& start = this->path.first;
    const auto& end = this->path.second;

    if (end.x() != start.x())
    {
        // For YPLUS/YMINUS, the distance is based on the change in y position with is x of vectors (row index)
        const float distance_in_cells =
            (end.x() - start.x()) *
                (direction == MiningDirection::YPLUS ? -1.0f : 1.0f) +
            1.0f - (1.0f - (*matrix)(end.x(), end.y()));
        return distance_in_cells * grid_geometry.cell_length_y;
    }
    // For XPLUS/XMINUS, the distance is based on the change in x position which is y of vectors (column index)
    const float distance_in_cells =
        (end.y() - start.y()) *
            (direction == MiningDirection::XMINUS ? -1.0f : 1.0f) +
        1.0f - (1.0f - (*matrix)(end.x(), end.y()));
    return distance_in_cells * grid_geometry.cell_length_x;
}

MiningGridGeometry MiningPlanner::computeMiningGridGeometry(
    const RobotParams& robot_params,
    const MiningZoneLimiterVector& direction_offset
) const
{
    MiningGridGeometry geometry;

    geometry.mining_zone_x_length =
        robot_params.bounds.mining_zone.max().x() -
        robot_params.bounds.mining_zone.min().x();
    geometry.mining_zone_y_length =
        robot_params.bounds.mining_zone.max().y() -
        robot_params.bounds.mining_zone.min().y();

    // Direction offset is {Yplus, Yminus, Xminus, Xplus}
    geometry.actual_mining_x_length = std::max(
        0.0f, geometry.mining_zone_x_length - direction_offset[2] - direction_offset[3]);
    geometry.actual_mining_y_length = std::max(
        0.0f, geometry.mining_zone_y_length - direction_offset[0] - direction_offset[1]);
    
    geometry.cell_length_x = geometry.actual_mining_x_length / static_cast<float>(x_divisions);
    geometry.cell_length_y = geometry.actual_mining_y_length / static_cast<float>(y_divisions);

    geometry.min_corner_with_offset =
        robot_params.bounds.mining_zone.min() + Eigen::Vector2f(
            direction_offset[2],
            direction_offset[0]);
    geometry.max_corner_with_offset =
        robot_params.bounds.mining_zone.max() - Eigen::Vector2f(
            direction_offset[3],
            direction_offset[1]);

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
    strip_map_xminus_transposed = Eigen::MatrixXf::Zero(x_divisions, y_divisions);
    strip_map_xplus_transposed = Eigen::MatrixXf::Zero(x_divisions, y_divisions);

    this->previously_mined_cells_by_direction[MiningDirection::YPLUS] = Eigen::MatrixXi::Zero(
            strip_map_yplus.rows(),
            strip_map_yplus.cols());
    this->previously_mined_cells_by_direction[MiningDirection::YMINUS] = Eigen::MatrixXi::Zero(
            strip_map_yminus.rows(),
            strip_map_yminus.cols());
    this->previously_mined_cells_by_direction[MiningDirection::XMINUS] = Eigen::MatrixXi::Zero(
            strip_map_xminus.rows(),
            strip_map_xminus.cols());
    this->previously_mined_cells_by_direction[MiningDirection::XPLUS] = Eigen::MatrixXi::Zero(
            strip_map_xplus.rows(),
            strip_map_xplus.cols());
}

std::map<MiningDirection, MiningGridGeometry> MiningPlanner::getGridGeometriesByDirection() const
    {
        std::map<MiningDirection, MiningGridGeometry> geometries;
        for (const auto& direction : ALL_MINING_DIRECTIONS)
        {
            const auto& direction_offset =
                MINING_ZONE_OFFSETS[static_cast<size_t>(direction)];
            geometries.emplace(
                direction,
                computeMiningGridGeometry(
                    this->robot_params,
                    direction_offset));
        }
        return geometries;
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
                (*mining_eval_distances)[mining_eval_index++]/grid_geometry[MiningDirection::YPLUS].cell_length_y,
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
                (*mining_eval_distances)[mining_eval_index++]/grid_geometry[MiningDirection::YMINUS].cell_length_y,
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
                (*mining_eval_distances)[mining_eval_index++]/grid_geometry[MiningDirection::XMINUS].cell_length_x,
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
                (*mining_eval_distances)[mining_eval_index++]/grid_geometry[MiningDirection::XPLUS].cell_length_x,
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
    std::cout << "Current path count: " << this->all_mining_paths.size() << "\n";
    this->removeSectionsForRobotClearance();
    std::cout << "After clearance path count: " << this->all_mining_paths.size() << "\n";
    std::cout << "Sorting paths by quality\n";
    this->sortPathsByQuality();
    std::cout << "Final path count: " << this->all_mining_paths.size() << "\n";

    return all_mining_paths;
}

void MiningPlanner::markMiningOnMatrix(const DirectedMiningPath& path)
{
    
    path.markMiningOnMatrix(this->previously_mined_cells_by_direction[path.getDirection()]);

    // Loop through the other 3 matrices and mark the corresponding cells as mined
    //  To do this, you'll need to convert each of the indices to a box with the real world coordinates then check to see if it overlaps with any of the indices in the path 
    std::vector<DirectedMiningPath::Vec2f> path_full = path.getFullPathInWorldFrame();
    // make one big box for the whole mining path
    Eigen::Vector2f min_point = path_full[0];
    Eigen::Vector2f max_point = path_full[0];
    
    Eigen::AlignedBox2f path_box(min_point, max_point);

    for (const auto& [direction, matrix] : this->previously_mined_cells_by_direction)
    {
        if (direction != path.getDirection())
        {
            // loop through each cell in the matrix
            for (int i = 0; i < matrix.rows(); i++)
            {
                for (int j = 0; j < matrix.cols(); j++)
                {
                    // Loop through the mined cells
                    float positionx = j * grid_geometry[direction].cell_length_x + grid_geometry[direction].min_corner_with_offset.x() + grid_geometry[direction].cell_length_x / 2.0f;
                    float positiony = i * grid_geometry[direction].cell_length_y + grid_geometry[direction].min_corner_with_offset.y() + grid_geometry[direction].cell_length_y / 2.0f;
                    Eigen::Vector2f cell_pos(positionx, positiony);
                    if (path_box.contains(cell_pos))
                    {
                        // Do something with the overlapping cell
                    }
                }
            }

        }
    }
}

const std::vector<MiningPlanner::Pose2f>& MiningPlanner::getStartingLocations()
{
    static std::vector<MiningPlanner::Pose2f> starting_vectors;
    starting_vectors.clear();

    for (int x = 0; x < x_divisions; x++)
    {
        for (int y = 0; y < y_divisions; y++)
        {
            // Up
            starting_vectors.push_back(Pose2f(
                grid_geometry[MiningDirection::YMINUS].min_corner_with_offset.x() +
                    (x + 0.5f) * grid_geometry[MiningDirection::YMINUS].cell_length_x,
                grid_geometry[MiningDirection::YMINUS].min_corner_with_offset.y() + y * grid_geometry[MiningDirection::YMINUS].cell_length_y,
                90.0f * (M_PI / 180.0f)));  // Convert degrees to radians
        }
    }
    for (int x = 0; x < x_divisions; x++)
    {
        for (int y = 0; y < y_divisions; y++)
        {
            // Down
            starting_vectors.push_back(Pose2f(
                grid_geometry[MiningDirection::YPLUS].min_corner_with_offset.x() +
                    (x + 0.5f) * grid_geometry[MiningDirection::YPLUS].cell_length_x,
                grid_geometry[MiningDirection::YPLUS].max_corner_with_offset.y() - y * grid_geometry[MiningDirection::YPLUS].cell_length_y,
                270.0f * (M_PI / 180.0f)));  // Convert degrees to radians
        }
    }
    for (int x = 0; x < x_divisions; x++)
    {
        for (int y = 0; y < y_divisions; y++)
        {
            std::cout << "Adding starting vector for XMINUS at x=" << grid_geometry[MiningDirection::XMINUS].max_corner_with_offset.x() - x * grid_geometry[MiningDirection::XMINUS].cell_length_x << " y=" << grid_geometry[MiningDirection::XMINUS].max_corner_with_offset.y() - (y + 0.5f) * grid_geometry[MiningDirection::XMINUS].cell_length_y
                      << "\n";
            // Left
            starting_vectors.push_back(Pose2f(
                grid_geometry[MiningDirection::XMINUS].max_corner_with_offset.x() - x * grid_geometry[MiningDirection::XMINUS].cell_length_x,
                grid_geometry[MiningDirection::XMINUS].max_corner_with_offset.y() -
                    (y + 0.5f) * grid_geometry[MiningDirection::XMINUS].cell_length_y,
                180.0f * (M_PI / 180.0f)));  // Convert degrees to radians
        }
    }
    for (int x = 0; x < x_divisions; x++)
    {
        for (int y = 0; y < y_divisions; y++)
        {
            // Right
            std::cout << "Adding starting vector for XPLUS at x=" << grid_geometry[MiningDirection::XPLUS].min_corner_with_offset.x() + x * grid_geometry[MiningDirection::XPLUS].cell_length_x << " y=" << grid_geometry[MiningDirection::XPLUS].max_corner_with_offset.y() - (y + 0.5f) * grid_geometry[MiningDirection::XPLUS].cell_length_y
                      << "\n";
            starting_vectors.push_back(Pose2f(
                grid_geometry[MiningDirection::XPLUS].min_corner_with_offset.x() + x * grid_geometry[MiningDirection::XPLUS].cell_length_x,
                grid_geometry[MiningDirection::XPLUS].max_corner_with_offset.y() -
                    (y + 0.5f) * grid_geometry[MiningDirection::XPLUS].cell_length_y,
                0.0f * (M_PI /180.0f)));  // Convert degrees to radians (0 remains 0)
        }
    }

    return starting_vectors;
}

void MiningPlanner::appendPlannedMiningPaths()
{
    std::cerr << "[MiningPlanner][DBG] appendPlannedMiningPaths: begin\n";

    int a = 0, b = 0;
    size_t pushed_paths = 0;
    size_t skipped_short_paths = 0;

    const auto directionToString = [](MiningDirection dir)
    {
        switch (dir)
        {
            case MiningDirection::YMINUS:
                return "YMINUS";
            case MiningDirection::XPLUS:
                return "XPLUS";
            case MiningDirection::YPLUS:
                return "YPLUS";
            case MiningDirection::XMINUS:
                return "XMINUS";
            default:
                return "UNKNOWN";
        }
    };

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
    std::cout << "Mining zone bounds: min=("
              << robot_params.bounds.mining_zone.min().x() << ","
              << robot_params.bounds.mining_zone.min().y() << ") "
              << "max=(" << robot_params.bounds.mining_zone.max().x() << ","
              << robot_params.bounds.mining_zone.max().y() << ")\n";
    std::cout << "Calculated grid size: cols=" << strip_map_yplus.cols()
              << " rows=" << strip_map_yplus.rows() << "\n";
    
    // print grid geometry for each direction
    for (const auto& [dir, geom] : grid_geometry)
    {
        std::cout << "Grid geometry for direction " << directionToString(dir) << ":\n";
        std::cout << "  mining_zone_x_length: " << geom.mining_zone_x_length << "\n";
        std::cout << "  mining_zone_y_length: " << geom.mining_zone_y_length << "\n";
        std::cout << "  actual_mining_x_length: " << geom.actual_mining_x_length << "\n";
        std::cout << "  actual_mining_y_length: " << geom.actual_mining_y_length << "\n";
        std::cout << "  cell_length_x: " << geom.cell_length_x << "\n";
        std::cout << "  cell_length_y: " << geom.cell_length_y << "\n";
        std::cout << "  min_corner_with_offset: (" << geom.min_corner_with_offset.x() << ", " << geom.min_corner_with_offset.y() << ")\n"; 
        std::cout << "  max_corner_with_offset: (" << geom.max_corner_with_offset.x() << ", " << geom.max_corner_with_offset.y() << ")\n";
    }

    // print all the strip maps
    std::cout << "strip_map_yminus:\n" << strip_map_yminus << "\n";
    std::cout << "strip_map_yplus:\n" << strip_map_yplus << "\n";
    std::cout << "strip_map_xplus:\n" << strip_map_xplus << "\n";
    std::cout << "strip_map_xminus:\n" << strip_map_xminus << "\n";
    std::cout << "strip_map_xplus_transposed:\n"
              << strip_map_xplus_transposed << "\n";
    std::cout << "strip_map_xminus_transposed:\n"
              << strip_map_xminus_transposed << "\n";



    // Top to Bottom search (DOWN and RIGHT)
    for (int direction_index = 0; direction_index < 2; direction_index++)
    {
        Eigen::MatrixXf* mat = mats[direction_index];
        MiningDirection mining_dir = directions[direction_index];

        int width = mat->cols();
        int height = mat->rows();

        std::cerr << "[MiningPlanner][DBG] pass=TopToBottom dir="
                  << directionToString(mining_dir) << " width=" << width
                  << " height=" << height << "\n";

        for (int i = 0; i < width; i++)
        {
            a = 0;
            b = 0;
            int guard = 0;

            while (b < height)
            {
                if (++guard > (height * 20 + 100))
                {
                    std::cerr
                        << "[MiningPlanner][DBG][STALL] pass=TopToBottom dir="
                        << directionToString(mining_dir) << " col=" << i
                        << " a=" << a << " b=" << b << " guard=" << guard
                        << "\n";
                    break;
                }

                const float va = (*mat)(a, i);
                const float vb = (*mat)(b, i);
                if ((va != va) || (vb != vb))
                {
                    std::cerr
                        << "[MiningPlanner][DBG][NAN] pass=TopToBottom dir="
                        << directionToString(mining_dir) << " col=" << i
                        << " a=" << a << " b=" << b << " va=" << va
                        << " vb=" << vb << "\n";
                }

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

                    Eigen::MatrixXf* original_mat =
                        (MiningDirection::YMINUS == mining_dir)
                            ? &strip_map_yminus
                            : &strip_map_xplus;
                    DirectedMiningPath dir_mining_path(
                        possible_path,
                        mining_dir,
                        original_mat,
                        &grid_geometry[mining_dir]);

                    const float dist = dir_mining_path.getDistance();
                    std::cerr << "[MiningPlanner][DBG] candidate dir="
                              << directionToString(mining_dir) << " col=" << i
                              << " start=(" << possible_path.first.x() << ","
                              << possible_path.first.y() << ")"
                              << " end=(" << possible_path.second.x() << ","
                              << possible_path.second.y() << ")"
                              << " dist=" << dist << " min_required="
                              << robot_params.minimum_mining_path_length
                              << "\n";

                    if (dist >= robot_params.minimum_mining_path_length)
                    {
                        this->all_mining_paths.push_back(dir_mining_path);
                        pushed_paths++;
                    }
                    else
                    {
                        skipped_short_paths++;
                    }

                    if (a == b)
                    {
                        std::cerr
                            << "[MiningPlanner][DBG][NO_PROGRESS] pass=TopToBottom dir="
                            << directionToString(mining_dir) << " col=" << i
                            << " a=" << a << " b=" << b << " forcing b++\n";
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
                    std::cerr
                        << "[MiningPlanner][DBG][UNEXPECTED] pass=TopToBottom dir="
                        << directionToString(mining_dir) << " col=" << i
                        << " a=" << a << " b=" << b << " va=" << va
                        << " vb=" << vb << " forcing b++\n";
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

        std::cerr << "[MiningPlanner][DBG] pass=BottomToTop dir="
                  << directionToString(mining_dir) << " width=" << width
                  << " height=" << height << "\n";

        for (int i = 0; i < width; i++)
        {
            a = height - 1;
            b = height - 1;
            int guard = 0;

            while (b >= 0)
            {
                if (++guard > (height * 20 + 100))
                {
                    std::cerr
                        << "[MiningPlanner][DBG][STALL] pass=BottomToTop dir="
                        << directionToString(mining_dir) << " col=" << i
                        << " a=" << a << " b=" << b << " guard=" << guard
                        << "\n";
                    break;
                }

                const float va = (*mat)(a, i);
                const float vb = (*mat)(b, i);
                if ((va != va) || (vb != vb))
                {
                    std::cerr
                        << "[MiningPlanner][DBG][NAN] pass=BottomToTop dir="
                        << directionToString(mining_dir) << " col=" << i
                        << " a=" << a << " b=" << b << " va=" << va
                        << " vb=" << vb << "\n";
                }

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

                    Eigen::MatrixXf* original_mat =
                        (MiningDirection::YPLUS == mining_dir) ? &strip_map_yplus
                                                              : &strip_map_xminus;
                    DirectedMiningPath dir_mining_path(
                        possible_path,
                        mining_dir,
                        original_mat,
                        &grid_geometry[mining_dir]);

                    const float dist = dir_mining_path.getDistance();
                    std::cerr << "[MiningPlanner][DBG] candidate dir="
                              << directionToString(mining_dir) << " col=" << i
                              << " start=(" << possible_path.first.x() << ","
                              << possible_path.first.y() << ")"
                              << " end=(" << possible_path.second.x() << ","
                              << possible_path.second.y() << ")"
                              << " dist=" << dist << " min_required="
                              << robot_params.minimum_mining_path_length
                              << "\n";

                    if (dist >= robot_params.minimum_mining_path_length)
                    {
                        this->all_mining_paths.push_back(dir_mining_path);
                        pushed_paths++;
                    }
                    else
                    {
                        skipped_short_paths++;
                    }

                    if (a == b)
                    {
                        std::cerr
                            << "[MiningPlanner][DBG][NO_PROGRESS] pass=BottomToTop dir="
                            << directionToString(mining_dir) << " col=" << i
                            << " a=" << a << " b=" << b << " forcing b--\n";
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
                    std::cerr
                        << "[MiningPlanner][DBG][UNEXPECTED] pass=BottomToTop dir="
                        << directionToString(mining_dir) << " col=" << i
                        << " a=" << a << " b=" << b << " va=" << va
                        << " vb=" << vb << " forcing b--\n";
                    b--;
                }
            }
        }
    }

    std::cerr
        << "[MiningPlanner][DBG] appendPlannedMiningPaths: end total_paths="
        << this->all_mining_paths.size() << " pushed=" << pushed_paths
        << " skipped_short=" << skipped_short_paths << "\n";
}

void MiningPlanner::sortPathsByQuality()
{
    std::sort(
        this->all_mining_paths.begin(),
        this->all_mining_paths.end(),
        [this](const DirectedMiningPath& a, const DirectedMiningPath& b)
        {
            return a.getQuality(this->previously_mined_cells_by_direction[a.getDirection()]) >
                   b.getQuality(this->previously_mined_cells_by_direction[b.getDirection()]);
        });
}

void MiningPlanner::removeSectionsForRobotClearance()
{
    for (int i = this->all_mining_paths.size() - 1; i >= 0; --i)
    {
        if (!this->all_mining_paths[i].adjustForRobotClearance())
        {
            this->all_mining_paths.erase(this->all_mining_paths.begin() + i);
        }
    }
}

};  // namespace lance
