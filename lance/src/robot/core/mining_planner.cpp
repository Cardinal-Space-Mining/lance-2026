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

#include "robot_math.hpp"


namespace lance
{

// How many sections to exclude from the end of a mining path to ensure the robot can fit in the spot without hitting
constexpr int NUM_SECTIONS_TO_EXLUDE_FROM_ROBOT_CLEARANCE = static_cast<int>(
    std::ceil(ROBOT_LENGTH_M_<float> / TRACK_SEPARATION_M_<float>));

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
    MiningPath p,
    MiningDirection dir,
    const Eigen::MatrixXf* mat,
    const RobotParams& robot_params) :
    path(std::move(p)),
    direction(dir),
    matrix(mat),
    m_robot_params(robot_params)
{
    distance = this->getRecalculatedDistance(path, matrix);
}

float DirectedMiningPath::getDistance() const { return distance; }

void DirectedMiningPath::markMiningOnMatrix(
    Eigen::MatrixXi& mined_count_matrix) const
{
    MiningPath p = toBaseCoordinates();
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
    const Eigen::MatrixXi& previously_minined_locations) const
{
    float quality = 0.0f;
    quality += distance;

    MiningPath p = toBaseCoordinates();

    for (int i = p.first.x(); i <= p.second.x(); i++)
    {
        for (int j = p.first.y(); j <= p.second.y(); j++)
        {
            quality -=
                previously_minined_locations(i, j) * previously_mined_penalty;
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
            path.first.x() += NUM_SECTIONS_TO_EXLUDE_FROM_ROBOT_CLEARANCE;
            break;
        case MiningDirection::UP:
            path.first.x() += NUM_SECTIONS_TO_EXLUDE_FROM_ROBOT_CLEARANCE;
            break;
        case MiningDirection::RIGHT:
            path.first.x() += NUM_SECTIONS_TO_EXLUDE_FROM_ROBOT_CLEARANCE;
            break;
        case MiningDirection::LEFT:
            path.first.x() += NUM_SECTIONS_TO_EXLUDE_FROM_ROBOT_CLEARANCE;
            break;
    }

    if (!checkValidity())
    {
        return false;
    }
    distance = this->getRecalculatedDistance();
    // std::cout << "got new distance " << distance << " for point " << path.first.x() << ", " << path.first.y() << " to " << path.second.x() << ", " << path.second.y() << " after clearance adjustment: ";

    return distance >= 0.0f;
}

DirectedMiningPath::MiningSwath
    DirectedMiningPath::getPathCoordinatesInWorldFrame() const
{
    MiningPath p = toBaseCoordinates();

#define CELL_SIZE TRACK_WIDTH_M_<float>

    // Eigen::Vector2f target_pos = this->params.mining_zone_bounds.max() - Eigen::Vector2f::Constant(0.8f);
    // Eigen::Vector2f target_dir{0.f, 1.f};
    float stdX = p.first.y() * CELL_SIZE;
    float stdY = p.first.x() * CELL_SIZE;
    Eigen::Vector2f target_pos;
    Eigen::Vector2f target_dir;

    switch (direction)
    {
        case MiningDirection::DOWN:
            target_pos = Eigen::Vector2f(stdX, stdY + CELL_SIZE);
            target_dir = Eigen::Vector2f({0.f, 1.f});
            break;
        case MiningDirection::UP:
            target_pos = Eigen::Vector2f(stdX, stdY - CELL_SIZE);
            target_dir = Eigen::Vector2f({0.f, -1.f});
            break;
        case MiningDirection::RIGHT:
            target_pos = Eigen::Vector2f(stdX - CELL_SIZE, stdY);
            target_dir = Eigen::Vector2f({1.f, 0.f});
            break;
        case MiningDirection::LEFT:
            // path.second.x() -= c;
            target_pos = Eigen::Vector2f(stdX + CELL_SIZE, stdY);
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
        return std::abs(
            end.x() - start.x() + 1 - (1 - (*mat)(end.x(), end.y())));
    }
    return std::abs(end.y() - start.y() + 1 - (1 - (*mat)(end.x(), end.y())));
}

MiningPath DirectedMiningPath::toBaseCoordinates() const
{
    MiningPath p = path;

    switch (direction)
    {
        case MiningDirection::DOWN:
            break;
        case MiningDirection::UP:
            // std::swap(p.first, p.second);
            p.first.x() = matrix->rows() - 1 - p.first.x();
            p.second.x() = matrix->rows() - 1 - p.second.x();
            break;
        case MiningDirection::RIGHT:
        {
            p.first = Eigen::Vector2i(p.first.y(), p.first.x());
            p.second = Eigen::Vector2i(p.second.y(), p.second.x());
            // int first_y = p.first.y();

            // p.first.y() = matrix->cols() - 1 - p.second.y();
            // p.second.y() = matrix->cols() - 1 - first_y;

            break;
        }
        case MiningDirection::LEFT:
            p.first = Eigen::Vector2i(p.first.y(), p.first.x());
            p.second = Eigen::Vector2i(p.second.y(), p.second.x());
            std::swap(p.first, p.second);
            // You want to use the rows because left basically a transpose of down so it needs cols of down which is rows of left
            p.first.y() = matrix->rows() - 1 - p.first.y();
            p.second.y() = matrix->rows() - 1 - p.second.y();
            break;
    }

    if (p.first.x() > p.second.x() ||
        (p.first.x() == p.second.x() && p.first.y() > p.second.y()))
    {
        std::swap(p.first, p.second);
    }

    return p;
}





MiningPlanner(
    const RobotParams& robot_params,
    std::function<float(float, float, float)> perception_eval) :
    m_robot_params(robot_params),
    perception_eval(perception_eval),
    mining_zone_bounds{robot_params.mining_zone_bounds}
{
    mapped_matrix_width =
        (int)(mining_zone_bounds.max().x() / TRACK_SEPARATION_M_<float>);
    mapped_matrix_height =
        (int)(mining_zone_bounds.max().y() / TRACK_SEPARATION_M_<float>);

    // UP/DOWN are (width, height) = (5, 4)
    strip_map_up =
        Eigen::MatrixXf::Zero(mapped_matrix_width, mapped_matrix_height);
    strip_map_down =
        Eigen::MatrixXf::Zero(mapped_matrix_width, mapped_matrix_height);

    // LEFT/RIGHT are swapped: (height, width) = (4, 5)
    strip_map_left =
        Eigen::MatrixXf::Zero(mapped_matrix_height, mapped_matrix_width);
    strip_map_right =
        Eigen::MatrixXf::Zero(mapped_matrix_height, mapped_matrix_width);

    times_mined_count_matrix =
        Eigen::MatrixXi::Zero(mapped_matrix_width, mapped_matrix_height);
}


void MiningPlanner::markMiningOnMatrix(const DirectedMiningPath& path)
{
    path.markMiningOnMatrix(times_mined_count_matrix);
}

const DirectedMiningPaths& MiningPlanner::finalOutput()
{
    DirectedMiningPaths vertical_up_paths;
    DirectedMiningPaths vertical_down_paths;
    DirectedMiningPaths horizontal_left_paths;
    DirectedMiningPaths horizontal_right_paths;
    populatePlannedMiningPaths(
        vertical_up_paths,
        strip_map_up,
        MiningDirection::UP);
    populatePlannedMiningPaths(
        vertical_down_paths,
        strip_map_down,
        MiningDirection::DOWN);
    populatePlannedMiningPaths(
        horizontal_left_paths,
        strip_map_left,
        MiningDirection::LEFT);
    populatePlannedMiningPaths(
        horizontal_right_paths,
        strip_map_right,
        MiningDirection::RIGHT);

    DirectedMiningPaths all_mining_paths = vertical_up_paths;
    all_mining_paths.insert(
        all_mining_paths.end(),
        vertical_down_paths.begin(),
        vertical_down_paths.end());
    all_mining_paths.insert(
        all_mining_paths.end(),
        horizontal_left_paths.begin(),
        horizontal_left_paths.end());
    all_mining_paths.insert(
        all_mining_paths.end(),
        horizontal_right_paths.begin(),
        horizontal_right_paths.end());

    removeSectionsForRobotClearance(all_mining_paths);
    sortPathsByQuality(all_mining_paths, times_mined_count_matrix);

    return all_mining_paths;
}

void MiningPlanner::updateMappedMatrices()
{
    // This function is used to populate the 4 strip maps based on the perception evaluation function. It doesn't need to be called very often but could be to refresh data
    populdateStripMap(
        strip_map_up,
        mapped_matrix_width,
        mapped_matrix_height,
        MiningDirection::UP,
        perception_eval);
    populdateStripMap(
        strip_map_down,
        mapped_matrix_width,
        mapped_matrix_height,
        MiningDirection::DOWN,
        perception_eval);
    populdateStripMap(
        strip_map_left,
        mapped_matrix_height,
        mapped_matrix_width,
        MiningDirection::LEFT,
        perception_eval);
    populdateStripMap(
        strip_map_right,
        mapped_matrix_height,
        mapped_matrix_width,
        MiningDirection::RIGHT,
        perception_eval);
}

void MiningPlanner::sortPathsByQuality(
    DirectedMiningPaths& paths,
    const Eigen::MatrixXi& previously_mined_locations)
{
    std::sort(
        paths.begin(),
        paths.end(),
        [&previously_mined_locations](
            const DirectedMiningPath& a,
            const DirectedMiningPath& b)
        {
            return a.getQuality(previously_mined_locations) >
                   b.getQuality(previously_mined_locations);
        });
}

void MiningPlanner::populatePlannedMiningPaths(
    DirectedMiningPaths& dir_mining_paths,
    const Eigen::MatrixXf& mat,
    MiningDirection mining_dir)
{
    int a = 0, b = 0;
    int width = mat.cols();
    int height = mat.rows();
    // std::cout << "Matrix dimensions - Width: " << width << ", Height: " << height << "\n";
    // std::cout << "Mining Direction: " << miningDirectionToString(mining_dir) << "\n";
    // std::cout << mat << "\n";
    for (int i = 0; i < width; i++)
    {
        a = 0;
        b = 0;
        while (b < height)
        {
            if (mat(a, i) == 0)
            {
                a++;
                b = a;
            }
            else if (mat(b, i) != 1 || b == height - 1)
            {
                if (mat(b, i) > 0)
                {
                    b++;
                }
                MiningPath possible_path =
                    std::pair(Eigen::Vector2i(a, i), Eigen::Vector2i(b - 1, i));

                DirectedMiningPath dir_mining_path(
                    possible_path,
                    mining_dir,
                    &mat,
                    m_robot_params);

                if (get_distance_for_path(dir_mining_path) >=
                    m_robot_params.min_zone_length)
                {
                    dir_mining_paths.push_back(dir_mining_path);
                }
                a = b;
            }
            else if (mat(b, i) == 1)
            {
                b++;
            }
        }
    }
    // std::cout << "\n=== Planned Mining Paths ===\n";
    // for (auto& path : dir_mining_paths){
    //     path.print();
    // }
}

void MiningPlanner::removeSectionsForRobotClearance(
    DirectedMiningPaths& dir_mining_paths)
{
    for (int i = dir_mining_paths.size() - 1; i >= 0; --i)
    {
        // std::cout << "\n=== Adjusting Path for Robot Clearance ===\n";
        // dir_mining_paths[i].print();
        if (!dir_mining_paths[i].adjustForRobotClearance())
        {
            // std::cout << "Path removed.\n";
            dir_mining_paths.erase(dir_mining_paths.begin() + i);
        }
    }
}


// Helper function to populate a strip map for a given direction
void MiningPlanner::populdateStripMap(
    Eigen::MatrixXf& strip_map,
    int primary_dim,
    int secondary_dim,
    MiningDirection direction,
    std::function<float(float, float, float)> perception_eval)
{
    float angle =
        (direction == MiningDirection::UP || direction == MiningDirection::DOWN)
            ? 90.0f
            : 0.0f;

    for (int i = 0; i < secondary_dim; i++)
    {
        float secondary_pos = i * TRACK_SEPARATION_M_<float>;

        int row = 0;
        while (row < primary_dim)
        {
            float primary_pos = row * TRACK_SEPARATION_M;
            float distance = perception_eval(secondary_pos, primary_pos, angle);
            int cells_clear = (int)(distance / TRACK_SEPARATION_M_<float>);

            for (int j = row; j < primary_dim; j++)
            {
                float distance_to_cell_end =
                    (j - row + 1) * TRACK_SEPARATION_M_<float>;
                if (distance >= distance_to_cell_end)
                {
                    strip_map(j, i) = 1.0f;
                }
                else
                {
                    float distance_to_cell_start =
                        (j - row) * TRACK_SEPARATION_M_<float>;
                    if (distance <= distance_to_cell_start)
                    {
                        strip_map(j, i) = 0.0f;
                    }
                    else
                    {
                        strip_map(j, i) = (distance - distance_to_cell_start) /
                                          TRACK_SEPARATION_M_<float>;
                    }
                    break;
                }
            }
            row += cells_clear + 1;
        }
    }
}

};  // namespace lance
