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
    constexpr int ROBOT_CLEARANCE_EXCLUSION_COUNT =
        static_cast<int>(std::ceil(
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
        const Eigen::MatrixXf *mat) : path(std::move(p)),
                                      direction(dir),
                                      matrix(mat)
    {
        distance = this->getRecalculatedDistance();
    }

    float DirectedMiningPath::getDistance() const { return distance; }

    void DirectedMiningPath::markMiningOnMatrix(
        Eigen::MatrixXi &mined_count_matrix) const
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
        const Eigen::MatrixXi &previously_mined_cells) const
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
                quality -=
                    previously_mined_cells(i, j) * previously_mined_penalty;
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

        return distance >= 0.0f;
    }

    DirectedMiningPath::MiningSwath
    DirectedMiningPath::getPathCoordinatesInWorldFrame(const RobotParams &robot_params) const
    {

#define CELL_SIZE TRACK_SEPARATION_M_<float>

        float stdX = path.first.x() * CELL_SIZE + geom::FOOTPRINT_R_MAX;
        float stdY = path.first.y() * CELL_SIZE;
        Eigen::Vector2f target_pos;
        Eigen::Vector2f target_dir;

        const float r = geom::FOOTPRINT_R_MAX_<float>;
        const float half_track_sep = TRACK_SEPARATION_M_<float> / 2.0f;
        const auto min_corner_with_offset = robot_params.mining_zone_bounds.min() + Eigen::Vector2f::Constant(r);
        const auto max_corner_with_offset = robot_params.mining_zone_bounds.max() - Eigen::Vector2f::Constant(r);

        switch (direction)
        {

        case MiningDirection::DOWN:
            stdX = path.first.y() * CELL_SIZE + min_corner_with_offset.x() + half_track_sep;
            stdY = max_corner_with_offset.y() - (path.first.x() * CELL_SIZE);

            target_pos = Eigen::Vector2f(stdX, stdY);
            target_dir = Eigen::Vector2f({0.f, 1.f});
            break;
        case MiningDirection::UP:
            stdX = path.first.y() * CELL_SIZE + min_corner_with_offset.x() + half_track_sep;
            stdY = path.first.x() * CELL_SIZE;

            target_pos = Eigen::Vector2f(stdX, stdY);
            target_dir = Eigen::Vector2f({0.f, -1.f});
            break;
        case MiningDirection::RIGHT:
            stdX = path.first.y() * CELL_SIZE + min_corner_with_offset.x();
            stdY = max_corner_with_offset.y() - (path.first.x() * CELL_SIZE) - half_track_sep;

            target_pos = Eigen::Vector2f(stdX, stdY);
            target_dir = Eigen::Vector2f({-1.f, 0.f});
            break;
        case MiningDirection::LEFT:
            stdX = max_corner_with_offset.x() - ((matrix->cols() - 1 - path.first.y()) * CELL_SIZE);
            stdY = max_corner_with_offset.y() - (path.first.x() * CELL_SIZE) - half_track_sep;

            target_pos = Eigen::Vector2f(stdX, stdY);
            target_dir = Eigen::Vector2f({1.f, 0.f});
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

        const auto &start = this->path.first;
        const auto &end = this->path.second;

        if (end.x() != start.x())
        {
            return std::abs(
                std::abs(end.x() - start.x()) + 1 - (1 - (*matrix)(end.x(), end.y())));
        }
        return std::abs(
            std::abs(end.y() - start.y()) + 1 - (1 - (*matrix)(end.x(), end.y())));
    }

    MiningPlanner::MiningPlanner(MiningEvalInterface &mining_eval, const RobotParams &robot_params) : robot_params(robot_params), mining_eval(mining_eval)
    {
        // Now that I have taken off part of full width and max length to avoid the hitting the walls,
        // The up and down, left and right must be done separately since the offsets are opposite

        float mining_zone_x_length =
            (this->robot_params.mining_zone_bounds.max().x() -
             this->robot_params.mining_zone_bounds.min().x());
        float mining_zone_y_length =
            (this->robot_params.mining_zone_bounds.max().y() -
             this->robot_params.mining_zone_bounds.min().y());

        float actual_mining_x_length =
            mining_zone_x_length - geom::FOOTPRINT_R_MAX * 2.0f;
        float actual_mining_y_length =
            mining_zone_y_length - geom::FOOTPRINT_R_MAX * 2.0f;
        int x_divisions =
            static_cast<int>(actual_mining_x_length / TRACK_SEPARATION_M_<float>);
        int y_divisions =
            static_cast<int>(actual_mining_y_length / TRACK_SEPARATION_M_<float>);

        // UP/DOWN are (width, height) = (5, 4)
        strip_map_up = Eigen::MatrixXf::Zero(y_divisions, x_divisions);
        strip_map_down = Eigen::MatrixXf::Zero(y_divisions, x_divisions);

        // LEFT/RIGHT are swapped: (height, width) = (4, 5)
        strip_map_left = Eigen::MatrixXf::Zero(y_divisions, x_divisions);
        strip_map_right = Eigen::MatrixXf::Zero(y_divisions, x_divisions);
        strip_map_left_transposed = Eigen::MatrixXf::Zero(x_divisions, y_divisions);
        strip_map_right_transposed = Eigen::MatrixXf::Zero(x_divisions, y_divisions);

        // Base-coordinate mining paths are indexed in the up/down frame.
        previously_mined_cells = Eigen::MatrixXi::Zero(
            strip_map_down.rows(),
            strip_map_down.cols());
    }

    void MiningPlanner::updateMappedMatrices()
    {
        // Sends the starting locations to the eval
        mining_eval.queryArenaFrame(this->getStartingLocations());

        if (!mining_eval.hasResult())
        {
            std::cerr << "Mining evaluation data not ready yet.\n";
            return;
        }

        const std::vector<float> *mining_eval_distances = mining_eval.getDists();

        size_t mining_eval_index = 0;
        // UP
        for (int i = 0; i < strip_map_up.cols(); i++)
        {
            for (int j = strip_map_up.rows() - 1; j >= 0; j--)
            {
                strip_map_up(j, i) = (*mining_eval_distances)[mining_eval_index++];
            }
        }
        // DOWN
        for (int i = 0; i < strip_map_down.cols(); i++)
        {
            for (int j = 0; j < strip_map_down.rows(); j++)
            {
                strip_map_down(j, i) = (*mining_eval_distances)[mining_eval_index++];
            }
        }
        // LEFT
        for (int i = strip_map_left.cols() - 1; i >= 0; i--)
        {
            for (int j = 0; j < strip_map_left.rows(); j++)
            {
                strip_map_left(j, i) = (*mining_eval_distances)[mining_eval_index++];
            }
        }
        // RIGHT
        for (int i = 0; i < strip_map_right.cols(); i++)
        {
            for (int j = 0; j < strip_map_right.rows(); j++)
            {
                strip_map_right(j, i) = (*mining_eval_distances)[mining_eval_index++];
            }
        }
        strip_map_left_transposed = strip_map_left.transpose();
        strip_map_right_transposed = strip_map_right.transpose();
    }
    const MiningPlanner::DirectedMiningPaths &MiningPlanner::finalOutput()
    {
        this->all_mining_paths.clear();
        this->appendPlannedMiningPaths();
        this->removeSectionsForRobotClearance();
        this->sortPathsByQuality();

        return all_mining_paths;
    }

    void MiningPlanner::markMiningOnMatrix(const DirectedMiningPath &path)
    {
        path.markMiningOnMatrix(this->previously_mined_cells);
    }

    const std::vector<MiningPlanner::Pose2f> &
    MiningPlanner::getStartingLocations()
    {
        // Gets the starting locations based on the box2f mining zone and the two offsets full_width and max_length
        float mining_zone_x_length =
            (this->robot_params.mining_zone_bounds.max().x() -
             this->robot_params.mining_zone_bounds.min().x());
        float mining_zone_y_length =
            (this->robot_params.mining_zone_bounds.max().y() -
             this->robot_params.mining_zone_bounds.min().y());

        static std::vector<MiningPlanner::Pose2f> starting_vectors;
        starting_vectors.clear();

        float actual_mining_x_length =
            mining_zone_x_length - geom::FOOTPRINT_R_MAX * 2.0f;
        float actual_mining_y_length =
            mining_zone_y_length - geom::FOOTPRINT_R_MAX * 2.0f;

        const float r = geom::FOOTPRINT_R_MAX_<float>;
        const auto min_corner_with_offset = this->robot_params.mining_zone_bounds.min() + Eigen::Vector2f::Constant(r);
        const auto max_corner_with_offset = this->robot_params.mining_zone_bounds.max() - Eigen::Vector2f::Constant(r);

        int x_divisions = static_cast<int>(
            actual_mining_x_length / TRACK_SEPARATION_M_<float>);
        int y_divisions = static_cast<int>(
            actual_mining_y_length / TRACK_SEPARATION_M_<float>);

        for (int x = 0; x < x_divisions; x++)
        {
            for (int y = 0; y < y_divisions; y++)
            {
                // Up
                starting_vectors.push_back(Pose2f(
                    min_corner_with_offset.x() + (x + 0.5f) * TRACK_SEPARATION_M_<float>,
                    min_corner_with_offset.y() + y * TRACK_SEPARATION_M_<float>,
                    90.0f));
            }
        }
        for (int x = 0; x < x_divisions; x++)
        {
            for (int y = 0; y < y_divisions; y++)
            {
                // Down
                starting_vectors.push_back(Pose2f(
                    min_corner_with_offset.x() + (x + 0.5f) * TRACK_SEPARATION_M_<float>,
                    max_corner_with_offset.y() - y * TRACK_SEPARATION_M_<float>,
                    270.0f));
            }
        }
        for (int x = 0; x < x_divisions; x++)
        {
            for (int y = 0; y < y_divisions; y++)
            {
                // Left
                starting_vectors.push_back(Pose2f(
                    max_corner_with_offset.x() - x * TRACK_SEPARATION_M_<float>,
                    max_corner_with_offset.y() - (y + 0.5f) * TRACK_SEPARATION_M_<float>,
                    180.0f));
            }
        }
        for (int x = 0; x < x_divisions; x++)
        {
            for (int y = 0; y < y_divisions; y++)
            {
                // Right
                starting_vectors.push_back(Pose2f(
                    min_corner_with_offset.x() + x * TRACK_SEPARATION_M_<float>,
                    max_corner_with_offset.y() - (y + 0.5f) * TRACK_SEPARATION_M_<float>,
                    0.0f));
            }
        }
        return starting_vectors;
    }

    void MiningPlanner::appendPlannedMiningPaths()
    {
        int a = 0, b = 0;

        std::vector<MiningDirection> directions = {
            MiningDirection::DOWN,
            MiningDirection::RIGHT,
            MiningDirection::UP,
            MiningDirection::LEFT};
        std::vector<Eigen::MatrixXf *> mats = {
            &strip_map_down,
            &strip_map_right_transposed,
            &strip_map_up,
            &strip_map_left_transposed};

        // Top to Bottom search (Down and RIGHT)
        for (int direction_index = 0; direction_index < 2; direction_index++)
        {
            Eigen::MatrixXf *mat = mats[direction_index];
            MiningDirection mining_dir = directions[direction_index];

            int width = mat->cols();
            int height = mat->rows();

            for (int i = 0; i < width; i++)
            {
                a = 0;
                b = 0;
                while (b < height)
                {
                    if (mat->operator()(a, i) == 0)
                    {
                        a++;
                        b = a;
                    }
                    else if (mat->operator()(b, i) != 1 || b == height - 1)
                    {
                        if (mat->operator()(b, i) > 0)
                        {
                            b++;
                        }
                        DirectedMiningPath::MiningPath possible_path = std::pair(
                            Eigen::Vector2i(a, i),
                            Eigen::Vector2i(b - 1, i));

                        if (MiningDirection::RIGHT == mining_dir) // undoing the transpose
                        {
                            possible_path = std::pair(
                                Eigen::Vector2i(i, possible_path.first.x()),
                                Eigen::Vector2i(i, possible_path.second.x())

                            );
                        }

                        Eigen::MatrixXf *original_mat = (MiningDirection::DOWN == mining_dir) ? &strip_map_down : &strip_map_right;
                        DirectedMiningPath dir_mining_path(
                            possible_path,
                            mining_dir,
                            original_mat);

                        if (dir_mining_path.getDistance() >=
                            robot_params.minimum_mining_path_length)
                        {
                            this->all_mining_paths.push_back(dir_mining_path);
                        }
                        a = b;
                    }
                    else if (mat->operator()(b, i) == 1)
                    {
                        b++;
                    }
                }
            }
        }
        // Bottom to Top search (UP and LEFT)
        for (int direction_index = 2; direction_index < 4; direction_index++)
        {
            Eigen::MatrixXf *mat = mats[direction_index];
            MiningDirection mining_dir = directions[direction_index];

            int width = mat->cols();
            int height = mat->rows();

            for (int i = 0; i < width; i++)
            {
                a = height - 1;
                b = height - 1;
                while (b >= 0)
                {
                    if (mat->operator()(a, i) == 0)
                    {
                        a--;
                        b = a;
                    }
                    else if (mat->operator()(b, i) != 1 || b == 0)
                    {
                        if (mat->operator()(b, i) > 0)
                        {
                            b--;
                        }

                        DirectedMiningPath::MiningPath possible_path = std::pair(
                            Eigen::Vector2i(a, i),
                            Eigen::Vector2i(b + 1, i));

                        if (MiningDirection::LEFT == mining_dir) // undoing the transpose
                        {
                            possible_path = std::pair(
                                Eigen::Vector2i(i, possible_path.first.x()),
                                Eigen::Vector2i(i, possible_path.second.x())

                            );
                        }

                        Eigen::MatrixXf *original_mat = (MiningDirection::UP == mining_dir) ? &strip_map_up : &strip_map_left;
                        DirectedMiningPath dir_mining_path(
                            possible_path,
                            mining_dir,
                            original_mat);
                        if (dir_mining_path.getDistance() >=
                            robot_params.minimum_mining_path_length)
                        {
                            this->all_mining_paths.push_back(dir_mining_path);
                        }
                        a = b;
                    }
                    else if (mat->operator()(b, i) == 1)
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
            [this](const DirectedMiningPath &a, const DirectedMiningPath &b)
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
                this->all_mining_paths.erase(
                    this->all_mining_paths.begin() + i);
            }
        }
    }

}; // namespace lance
