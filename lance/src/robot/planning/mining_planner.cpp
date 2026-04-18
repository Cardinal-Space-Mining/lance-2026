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

// How many sections to exclude from the end of a mining path to ensure the robot can fit in the spot without hitting
constexpr int ROBOT_CLEARANCE_EXCLUSION_COUNT = static_cast<int>(std::ceil(
    geom::PRIMARY_COLLISION_ZONE_LENGTH_OFFSET_<float> /
    TRACK_SEPARATION_M_<float>));

// Used for testing
// inline const char* miningDirectionToString(MiningDirection dir)
// {
//     switch (dir)
//     {
//         case MiningDirection::UP:
//             return "UP";
//         case MiningDirection::DOWN:
//             return "DOWN";
//         case MiningDirection::LEFT:
//             return "LEFT";
//         case MiningDirection::RIGHT:
//             return "RIGHT";
//         default:
//             return "UNKNOWN";
//     }
// }

DirectedMiningPath::DirectedMiningPath(
    DirectedMiningPath::MiningPath p,
    MiningDirection dir,
    const Eigen::MatrixXf* mat) :
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

    switch (direction)
    {
        case MiningDirection::DOWN:
            if (path.first.x() != 0)
            {
                path.first.x() += ROBOT_CLEARANCE_EXCLUSION_COUNT;
            }
            break;
        case MiningDirection::UP:
            if (path.first.x() != matrix->rows() - 1)
            {
                path.first.x() -= ROBOT_CLEARANCE_EXCLUSION_COUNT;
            }
            break;
        case MiningDirection::RIGHT:
            if (path.first.y() != 0)
            {
                path.first.y() += ROBOT_CLEARANCE_EXCLUSION_COUNT;
            }
            break;
        case MiningDirection::LEFT:
            if (path.first.y() != matrix->cols() - 1)
            {
                path.first.y() -= ROBOT_CLEARANCE_EXCLUSION_COUNT;
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

DirectedMiningPath::MiningSwath
    DirectedMiningPath::getPathCoordinatesInWorldFrame(
    const RobotParams& robot_params,
    const MiningGridGeometry* grid_geometry) const
{

#define CELL_SIZE TRACK_SEPARATION_M_<float>

    float stdX = path.first.x() * CELL_SIZE + geom::FOOTPRINT_R_MAX;
    float stdY = path.first.y() * CELL_SIZE;
    Eigen::Vector2f target_pos;
    Eigen::Vector2f target_dir;

    const int matrix_rows = (matrix == nullptr) ? 0 : matrix->rows();
    const int matrix_cols = (matrix == nullptr) ? 0 : matrix->cols();

    const float r = geom::FOOTPRINT_R_MAX_<float>;
    const float half_track_sep = TRACK_SEPARATION_M_<float> / 2.0f;
    const Eigen::Vector2f min_corner_with_offset =
        (grid_geometry != nullptr)
            ? grid_geometry->min_corner_with_offset
            : robot_params.bounds.mining_zone.min() +
                  Eigen::Vector2f::Constant(r);

    Eigen::Vector2f max_corner_with_offset;
    if (grid_geometry != nullptr)
    {
        max_corner_with_offset = grid_geometry->max_corner_with_offset;
    }
    else
    {
        max_corner_with_offset =
            robot_params.bounds.mining_zone.min() + Eigen::Vector2f(
                r + matrix_cols * TRACK_SEPARATION_M_<float>,
                r + matrix_rows * TRACK_SEPARATION_M_<float>);
    }

    switch (direction)
    {
        case MiningDirection::DOWN:
            stdX = path.first.y() * CELL_SIZE + min_corner_with_offset.x() +
                   half_track_sep;
            stdY = max_corner_with_offset.y() - (path.first.x() * CELL_SIZE);

            target_pos = Eigen::Vector2f(stdX, stdY);
            target_dir = Eigen::Vector2f({0.f, -1.f});
            break;
        case MiningDirection::UP:
            stdX = path.first.y() * CELL_SIZE + min_corner_with_offset.x() +
                   half_track_sep;
             stdY = min_corner_with_offset.y() +
                 (matrix_rows - 1 - path.first.x()) * CELL_SIZE;

            target_pos = Eigen::Vector2f(stdX, stdY);
            target_dir = Eigen::Vector2f({0.f, 1.f});
            break;
        case MiningDirection::RIGHT:
            stdX = path.first.y() * CELL_SIZE + min_corner_with_offset.x();
            stdY = max_corner_with_offset.y() - (path.first.x() * CELL_SIZE) -
                   half_track_sep;

            target_pos = Eigen::Vector2f(stdX, stdY);
            target_dir = Eigen::Vector2f({1.f, 0.f});
            break;
        case MiningDirection::LEFT:
            stdX = max_corner_with_offset.x() -
                 ((matrix_cols - 1 - path.first.y()) * CELL_SIZE);
            stdY = max_corner_with_offset.y() - (path.first.x() * CELL_SIZE) -
                   half_track_sep;

            target_pos = Eigen::Vector2f(stdX, stdY);
            target_dir = Eigen::Vector2f({-1.f, 0.f});
            break;
    }

#undef CELL_SIZE
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
        return 
            (end.x() - start.x()) * (direction == MiningDirection::UP ? -1 : 1) + 1 -
            (1 - (*matrix)(end.x(), end.y()));
    }
    return 
        ((end.y() - start.y()) * (direction == MiningDirection::LEFT ? -1 : 1)) + 1 - (1 - (*matrix)(end.x(), end.y()));
}

MiningGridGeometry MiningPlanner::computeMiningGridGeometry(
    const RobotParams& robot_params)
{
    MiningGridGeometry geometry;

    geometry.mining_zone_x_length =
        robot_params.bounds.mining_zone.max().x() -
        robot_params.bounds.mining_zone.min().x();
    geometry.mining_zone_y_length =
        robot_params.bounds.mining_zone.max().y() -
        robot_params.bounds.mining_zone.min().y();

    geometry.actual_mining_x_length = std::max(
        0.0f,
        geometry.mining_zone_x_length - geom::FOOTPRINT_R_MAX_<float> * 2.0f);
    geometry.actual_mining_y_length = std::max(
        0.0f,
        geometry.mining_zone_y_length - geom::FOOTPRINT_R_MAX_<float> * 2.0f);

    geometry.x_divisions = std::max(
        0,
        static_cast<int>(std::floor(
            geometry.actual_mining_x_length / TRACK_SEPARATION_M_<float>)));
    geometry.y_divisions = std::max(
        0,
        static_cast<int>(std::floor(
            geometry.actual_mining_y_length / TRACK_SEPARATION_M_<float>)));

    const float r = geom::FOOTPRINT_R_MAX_<float>;
    geometry.min_corner_with_offset =
        robot_params.bounds.mining_zone.min() + Eigen::Vector2f::Constant(r);
    geometry.max_corner_with_offset =
        robot_params.bounds.mining_zone.min() + Eigen::Vector2f(
            r + geometry.x_divisions * TRACK_SEPARATION_M_<float>,
            r + geometry.y_divisions * TRACK_SEPARATION_M_<float>);

    return geometry;
}

MiningPlanner::MiningPlanner(
    MiningEvalInterface& mining_eval,
    const RobotParams& robot_params) :
    robot_params(robot_params),
    mining_eval(mining_eval)
{
    // Now that I have taken off part of full width and max length to avoid the hitting the walls,
    // The up and down, left and right must be done separately since the offsets are opposite

    this->grid_geometry = computeMiningGridGeometry(this->robot_params);
    // const int x_divisions = this->grid_geometry.x_divisions;
    // const int y_divisions = this->grid_geometry.y_divisions;

    if (this->grid_geometry.x_divisions == 0 || this->grid_geometry.y_divisions == 0)
    {
        std::cerr
            << "Mining planner grid has no valid cells. Check bounds.mining_zone and robot footprint parameters.\n";
    }

    // UP/DOWN are (width, height) = (5, 4)
    strip_map_up = Eigen::MatrixXf::Zero(this->grid_geometry.y_divisions, this->grid_geometry.x_divisions);
    strip_map_down = Eigen::MatrixXf::Zero(this->grid_geometry.y_divisions, this->grid_geometry.x_divisions);

    // LEFT/RIGHT are swapped: (height, width) = (4, 5)
    strip_map_left = Eigen::MatrixXf::Zero(this->grid_geometry.y_divisions, this->grid_geometry.x_divisions);
    strip_map_right = Eigen::MatrixXf::Zero(this->grid_geometry.y_divisions, this->grid_geometry.x_divisions);
    strip_map_left_transposed = Eigen::MatrixXf::Zero(this->grid_geometry.x_divisions, this->grid_geometry.y_divisions);
    strip_map_right_transposed =
        Eigen::MatrixXf::Zero(this->grid_geometry.x_divisions, this->grid_geometry.y_divisions);

    // Base-coordinate mining paths are indexed in the up/down frame.
    previously_mined_cells =
        Eigen::MatrixXi::Zero(strip_map_down.rows(), strip_map_down.cols());
}

bool MiningPlanner::updateMappedMatrices()
{
    const size_t grid_cell_count = static_cast<size_t>(
        this->strip_map_up.rows() * this->strip_map_up.cols());
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
    for (int i = 0; i < strip_map_up.cols(); i++)
    {
        for (int j = strip_map_up.rows() - 1; j >= 0; j--)
        {
            strip_map_up(j, i) = std::clamp(
                (*mining_eval_distances)[mining_eval_index++],
                0.0f,
                1.0f);
        }
    }
    // DOWN
    for (int i = 0; i < strip_map_down.cols(); i++)
    {
        for (int j = 0; j < strip_map_down.rows(); j++)
        {
            strip_map_down(j, i) = std::clamp(
                (*mining_eval_distances)[mining_eval_index++],
                0.0f,
                1.0f);
        }
    }
    // LEFT
    for (int i = strip_map_left.cols() - 1; i >= 0; i--)
    {
        for (int j = 0; j < strip_map_left.rows(); j++)
        {
            strip_map_left(j, i) = std::clamp(
                (*mining_eval_distances)[mining_eval_index++],
                0.0f,
                1.0f);
        }
    }
    // RIGHT
    for (int i = 0; i < strip_map_right.cols(); i++)
    {
        for (int j = 0; j < strip_map_right.rows(); j++)
        {
            strip_map_right(j, i) = std::clamp(
                (*mining_eval_distances)[mining_eval_index++],
                0.0f,
                1.0f);
        }
    }
    strip_map_left_transposed = strip_map_left.transpose();
    strip_map_right_transposed = strip_map_right.transpose();
    return true;
}
const MiningPlanner::DirectedMiningPaths& MiningPlanner::finalOutput()
{
    std::cout << "Clearing all paths\n";
    this->all_mining_paths.clear();
    std::cout << "Appending planned paths\n";
    this->appendPlannedMiningPaths();
    std::cout << "Removing sections for robot clearance\n";
    // print current lenght
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
    path.markMiningOnMatrix(this->previously_mined_cells);
}

const std::vector<MiningPlanner::Pose2f>& MiningPlanner::getStartingLocations()
{
    static std::vector<MiningPlanner::Pose2f> starting_vectors;
    starting_vectors.clear();

    const auto& geometry = this->grid_geometry;
    const int x_divisions = geometry.x_divisions;
    const int y_divisions = geometry.y_divisions;
    const auto& min_corner_with_offset = geometry.min_corner_with_offset;
    const auto& max_corner_with_offset = geometry.max_corner_with_offset;

    for (int x = 0; x < x_divisions; x++)
    {
        for (int y = 0; y < y_divisions; y++)
        {
            // Up
            starting_vectors.push_back(Pose2f(
                min_corner_with_offset.x() +
                    (x + 0.5f) * TRACK_SEPARATION_M_<float>,
                min_corner_with_offset.y() + y * TRACK_SEPARATION_M_<float>,
                90.0f * (M_PI / 180.0f)));  // Convert degrees to radians
        }
    }
    for (int x = 0; x < x_divisions; x++)
    {
        for (int y = 0; y < y_divisions; y++)
        {
            // Down
            starting_vectors.push_back(Pose2f(
                min_corner_with_offset.x() +
                    (x + 0.5f) * TRACK_SEPARATION_M_<float>,
                max_corner_with_offset.y() - y * TRACK_SEPARATION_M_<float>,
                270.0f * (M_PI / 180.0f)));  // Convert degrees to radians
        }
    }
    for (int x = 0; x < x_divisions; x++)
    {
        for (int y = 0; y < y_divisions; y++)
        {
            std::cout << "Adding starting vector for Left at x=" << max_corner_with_offset.x() - x * TRACK_SEPARATION_M_<float> << " y=" << max_corner_with_offset.y() - (y + 0.5f) * TRACK_SEPARATION_M_<float>
                      << "\n";
            // Left
            starting_vectors.push_back(Pose2f(
                max_corner_with_offset.x() - x * TRACK_SEPARATION_M_<float>,
                max_corner_with_offset.y() -
                    (y + 0.5f) * TRACK_SEPARATION_M_<float>,
                180.0f * (M_PI / 180.0f)));  // Convert degrees to radians
        }
    }
    for (int x = 0; x < x_divisions; x++)
    {
        for (int y = 0; y < y_divisions; y++)
        {
            // Right
            std::cout << "Adding starting vector for Right at x=" << min_corner_with_offset.x() + x * TRACK_SEPARATION_M_<float> << " y=" << max_corner_with_offset.y() - (y + 0.5f) * TRACK_SEPARATION_M_<float>
                      << "\n";
            starting_vectors.push_back(Pose2f(
                min_corner_with_offset.x() + x * TRACK_SEPARATION_M_<float>,
                max_corner_with_offset.y() -
                    (y + 0.5f) * TRACK_SEPARATION_M_<float>,
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
            case MiningDirection::DOWN:
                return "DOWN";
            case MiningDirection::RIGHT:
                return "RIGHT";
            case MiningDirection::UP:
                return "UP";
            case MiningDirection::LEFT:
                return "LEFT";
            default:
                return "UNKNOWN";
        }
    };

    std::vector<MiningDirection> directions = {
        MiningDirection::DOWN,
        MiningDirection::RIGHT,
        MiningDirection::UP,
        MiningDirection::LEFT};
    std::vector<Eigen::MatrixXf*> mats = {
        &strip_map_down,
        &strip_map_right_transposed,
        &strip_map_up,
        &strip_map_left_transposed};

    // Print lots of relevant info for debugging
    std::cout << "Mining zone bounds: min=("
              << robot_params.bounds.mining_zone.min().x() << ","
              << robot_params.bounds.mining_zone.min().y() << ") "
              << "max=(" << robot_params.bounds.mining_zone.max().x() << ","
              << robot_params.bounds.mining_zone.max().y() << ")\n";
    std::cout << "Calculated grid size: cols=" << strip_map_up.cols()
              << " rows=" << strip_map_up.rows() << "\n";
    std::cout << "ROBOT_CLEARANCE_EXCLUSION_COUNT: "
              << ROBOT_CLEARANCE_EXCLUSION_COUNT << "\n";
    std::cout << "Actual mining zone x length: "
              << (robot_params.bounds.mining_zone.max().x() -
                  robot_params.bounds.mining_zone.min().x() -
                  geom::FOOTPRINT_R_MAX * 2.0f)
              << "\n";
    std::cout << "Actual mining zone y length: "
              << (robot_params.bounds.mining_zone.max().y() -
                  robot_params.bounds.mining_zone.min().y() -
                  geom::FOOTPRINT_R_MAX * 2.0f)
              << "\n";
    std::cout << "Grid cell size (TRACK_SEPARATION_M): "
              << TRACK_SEPARATION_M_<float> << "\n";
    std::cout << "Grid divisions: x=" << strip_map_up.cols()
              << " y=" << strip_map_up.rows() << "\n";
    std::cout << "Total grid cells: "
              << strip_map_up.cols() * strip_map_up.rows() << "\n";
    std::cout << "Expected mining eval count: "
              << 4 * strip_map_up.cols() * strip_map_up.rows() << "\n";
    std::cout << "Geom footprint r max: " << geom::FOOTPRINT_R_MAX << "\n";
    // print all the strip maps
    std::cout << "strip_map_down:\n" << strip_map_down << "\n";
    std::cout << "strip_map_up:\n" << strip_map_up << "\n";
    std::cout << "strip_map_right:\n" << strip_map_right << "\n";
    std::cout << "strip_map_left:\n" << strip_map_left << "\n";
    std::cout << "strip_map_right_transposed:\n"
              << strip_map_right_transposed << "\n";
    std::cout << "strip_map_left_transposed:\n"
              << strip_map_left_transposed << "\n";



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

                    if (MiningDirection::RIGHT == mining_dir)
                    {
                        // Undo transpose for RIGHT directional search.
                        possible_path = std::pair(
                            Eigen::Vector2i(i, possible_path.first.x()),
                            Eigen::Vector2i(i, possible_path.second.x()));
                    }

                    Eigen::MatrixXf* original_mat =
                        (MiningDirection::DOWN == mining_dir)
                            ? &strip_map_down
                            : &strip_map_right;
                    DirectedMiningPath dir_mining_path(
                        possible_path,
                        mining_dir,
                        original_mat);

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

                    if (MiningDirection::LEFT == mining_dir)
                    {
                        // Undo transpose for LEFT directional search.
                        possible_path = std::pair(
                            Eigen::Vector2i(i, possible_path.first.x()),
                            Eigen::Vector2i(i, possible_path.second.x()));
                    }

                    Eigen::MatrixXf* original_mat =
                        (MiningDirection::UP == mining_dir) ? &strip_map_up
                                                            : &strip_map_left;
                    DirectedMiningPath dir_mining_path(
                        possible_path,
                        mining_dir,
                        original_mat);

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
            return a.getQuality(this->previously_mined_cells) >
                   b.getQuality(this->previously_mined_cells);
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
