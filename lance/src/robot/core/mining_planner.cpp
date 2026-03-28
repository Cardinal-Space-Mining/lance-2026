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
    // std::cout << "got new distance " << distance << " for point " << path.first.x() << ", "
    // << path.first.y() << " to " << path.second.x() << ", " << path.second.y() << " after clearance adjustment: ";

    return distance >= 0.0f;
}

DirectedMiningPath::MiningSwath
    DirectedMiningPath::getPathCoordinatesInWorldFrame() const
{
    MiningPath p = toBaseCoordinates();

#define CELL_SIZE TRACK_SEPARATION_M_<float>

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
            end.x() - start.x() + 1 - (1 - (*matrix)(end.x(), end.y())));
    }
    return std::abs(
        end.y() - start.y() + 1 - (1 - (*matrix)(end.x(), end.y())));
}

DirectedMiningPath::MiningPath DirectedMiningPath::toBaseCoordinates() const
{
    MiningPath p = path;

    switch (direction)
    {
        case MiningDirection::DOWN:
        {
            break;
        }
        case MiningDirection::UP:
        {
            // std::swap(p.first, p.second);
            p.first.x() = matrix->rows() - 1 - p.first.x();
            p.second.x() = matrix->rows() - 1 - p.second.x();
            break;
        }
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
        {
            p.first = Eigen::Vector2i(p.first.y(), p.first.x());
            p.second = Eigen::Vector2i(p.second.y(), p.second.x());
            std::swap(p.first, p.second);
            // You want to use the rows because left basically a transpose of down
            // so it needs cols of down which is rows of left
            p.first.y() = matrix->rows() - 1 - p.first.y();
            p.second.y() = matrix->rows() - 1 - p.second.y();
            break;
        }
    }

    if (p.first.x() > p.second.x() ||
        (p.first.x() == p.second.x() && p.first.y() > p.second.y()))
    {
        std::swap(p.first, p.second);
    }

    return p;
}





MiningPlanner::MiningPlanner(const RobotParams& robot_params) :
    robot_params(robot_params)
{
    // Now that I have taken off part of full width and max length to avoid the hitting the walls,
        // The up and down, left and right must be done separately since the offsets are opposite

    float actual_mining_x_length = (mining_zone_x_length - this->max_length) * 2.0f;
    float actual_mining_y_length = (mining_zone_y_length - this->full_width) * 2.0f;

    int x_divisions = static_cast<int>(actual_mining_x_length / TRACK_SEPARATION_M_<float>);
    int y_divisions = static_cast<int>(actual_mining_y_length / TRACK_SEPARATION_M_<float>);
    
    // UP/DOWN are (width, height) = (5, 4)
    strip_map_up =
        Eigen::MatrixXf::Zero(x_divisions, y_divisions);
    strip_map_down =
        Eigen::MatrixXf::Zero(x_divisions, y_divisions);

    actual_mining_x_length = (mining_zone_x_length - this->full_width) * 2.0f;
    actual_mining_y_length = (mining_zone_y_length - this->max_length) * 2.0f;

    x_divisions = static_cast<int>(actual_mining_x_length / TRACK_SEPARATION_M_<float>);
    y_divisions = static_cast<int>(actual_mining_y_length / TRACK_SEPARATION_M_<float>);

    // LEFT/RIGHT are swapped: (height, width) = (4, 5)
    strip_map_left =
        Eigen::MatrixXf::Zero(y_divisions, x_divisions);
    strip_map_right =
        Eigen::MatrixXf::Zero(y_divisions, x_divisions);

    times_mined_count_matrix = // same dimensions as the down strip map since it's the base coordinates
        Eigen::MatrixXi::Zero(x_divisions, y_divisions);
}


void MiningPlanner::updateMappedMatrices(const std::vector<float>* mining_eval_distances)
{
    // take the output from the perception eval (mining_eval_distances) and put all the data into the 4 strips in order
    int mining_eval_index = 0;

    // first strip_map_up

    for (int x = 0; x < strip_map_up.rows(); x++) {
        for (int y = 0; y < strip_map_up.cols(); y++){
            // Up
            strip_map_up(x, strip_map_up.cols()-y-1) = (*mining_eval_distances)[mining_eval_index];
            mining_eval_index++;

            // Down
            strip_map_down(x, y) = (*mining_eval_distances)[mining_eval_index];
            mining_eval_index++;

        }
    }

    for (int x = 0; x < strip_map_left.rows(); x++) {
        for (int y = 0; y < strip_map_left.cols(); y++){
            // Right
            strip_map_right(x, y) = (*mining_eval_distances)[mining_eval_index];
            mining_eval_index++;
            // starting_vectors.push_back(Pose2f(
            //     starting_corner_x + x * TRACK_SEPARATION_M_<float>,
            //     starting_corner_y + y * TRACK_SEPARATION_M_<float>,
            //     0.0f));
            // Left
            strip_map_left(x, y) = (*mining_eval_distances)[mining_eval_index];
            mining_eval_index++;
            // starting_vectors.push_back(Pose2f(
            //     starting_corner_x + (x + 1) * RACK_SEPARATION_M_<float>,
            //     starting_corner_y + y * TRACK_SEPARATION_M_<float>,
            //     180.0f));
        }
    }
    

    for (int i = 0; i < strip_map_up.rows(); i++)
    {
        for (int j = 0; j < strip_map_up.cols(); j++)
        {
            int index = i * strip_map_up.cols() + j;
   
    for (int i = 0; i < strip_map_left.rows(); i++)
    {
        for (int j = 0; j < strip_map_left.cols(); j++)
        {
            int index = strip_map_up.size() + strip_map_down.size() + i * strip_map_left.cols() + j;
            if (index < mining_eval_distances->size())
            {
                strip_map_left(i, j) = (*mining_eval_distances)[index];
            }
        }
    }
    for (int i = 0; i < strip_map_right.rows(); i++)
    {
        for (int j = 0; j < strip_map_right.cols(); j++)
        {
            int index = strip_map_up.size() + strip_map_down.size() + strip_map_left.size() + i * strip_map_right.cols() + j;
            if (index < mining_eval_distances->size())
            {
                strip_map_right(i, j) = (*mining_eval_distances)[index];
            }
        }
    }
}
    // This function is used to populate the 4 strip maps based on the perception
    // evaluation function. It doesn't need to be called very often but could
    // be to refresh data


//     this->populateStripMap(strip_map_up, MiningDirection::UP);
//     this->populateStripMap(strip_map_down, MiningDirection::DOWN);
//     this->populateStripMap(strip_map_left, MiningDirection::LEFT);
//     this->populateStripMap(strip_map_right, MiningDirection::RIGHT);
// }
}
const MiningPlanner::DirectedMiningPaths& MiningPlanner::finalOutput()
{
    this->all_mining_paths.clear();

    this->appendPlannedMiningPaths(strip_map_up, MiningDirection::UP);
    this->appendPlannedMiningPaths(strip_map_down, MiningDirection::DOWN);
    this->appendPlannedMiningPaths(strip_map_left, MiningDirection::LEFT);
    this->appendPlannedMiningPaths(strip_map_right, MiningDirection::RIGHT);

    this->removeSectionsForRobotClearance();
    this->sortPathsByQuality();

    return all_mining_paths;
}

void MiningPlanner::markMiningOnMatrix(const DirectedMiningPath& path)
{
    path.markMiningOnMatrix(this->times_mined_count_matrix);
}


const std::vector<MiningPlanner::Pose2f>& MiningPlanner::getStartingLocations() {
    // Gets the starting locations based on the box2f mining zone and the two offsets full_width and max_length

    float mining_zone_x_length = (this->robot_params.mining_zone_bounds.max().x()-this->robot_params.mining_zone_bounds.min().x());
    float mining_zone_y_length = (this->robot_params.mining_zone_bounds.max().y()-this->robot_params.mining_zone_bounds.min().y());

    static std::vector<MiningPlanner::Pose2f> starting_vectors;
    starting_vectors.clear();

    // First do the pos_x and neg_x - In theory you could remove one of the max_lengths for each 
        // because you could have the robot mine all the way but for now just remove both to be save
    float actual_mining_x_length = (mining_zone_x_length - this->max_length) * 2.0f;
    float actual_mining_y_length = (mining_zone_y_length - this->full_width) * 2.0f;

    float starting_corner_x = this->robot_params.mining_zone_bounds.min().x()+this->full_width;
    float starting_corner_y = this->robot_params.mining_zone_bounds.min().y()+this->max_length;

    int x_divisions = static_cast<int>(actual_mining_x_length / TRACK_SEPARATION_M_<float>);
    int y_divisions = static_cast<int>(actual_mining_y_length / TRACK_SEPARATION_M_<float>);
    


    // NEED TO FIGURE OUT WHICH WAY I WANT THE MATRICES TO FACE AND COORDINATE SYSTEM
    // CURRENTLY I DO X, Y FOR UP/DOWN BUT THAT DOESN'T FEEL RIGHT
    // RIGHT DOWN HERE IT SEEMS TO DO THE OPPOSITE AND IN
    // Areas to do 
        /*
            updateMappedMatrices
            getStartingLocations
            constructor where the strip maps are initialized and the dimensions are calculated
        
        Once all of those are done I will export this and run tests on it
        
        */

    for (int x = 0; x < x_divisions; x++) {
        for (int y = 0; y < y_divisions; y++){
            // Up
            starting_vectors.push_back(Pose2f(
                starting_corner_x + x * TRACK_SEPARATION_M_<float>,
                starting_corner_y + y * TRACK_SEPARATION_M_<float>,
                90.0f));

            // Down
            starting_vectors.push_back(Pose2f(
                starting_corner_x + x * TRACK_SEPARATION_M_<float>,
                starting_corner_y + (y + 1) * TRACK_SEPARATION_M_<float>,
                270.0f));

        }
    }

    // Now do the pos_y and neg_y
    actual_mining_x_length = (mining_zone_x_length - this->full_width) * 2.0f;
    actual_mining_y_length = (mining_zone_y_length - this->max_length) * 2.0f;

    starting_corner_x = this->robot_params.mining_zone_bounds.min().x()+this->max_length;
    starting_corner_y = this->robot_params.mining_zone_bounds.min().y()+this->full_width;

    x_divisions = static_cast<int>(actual_mining_x_length / TRACK_SEPARATION_M_<float>);
    y_divisions = static_cast<int>(actual_mining_y_length / TRACK_SEPARATION_M_<float>);

    for (int x = 0; x < x_divisions; x++) {
        for (int y = 0; y < y_divisions; y++){
            // Right
            starting_vectors.push_back(Pose2f(
                starting_corner_x + x * TRACK_SEPARATION_M_<float>,
                starting_corner_y + y * TRACK_SEPARATION_M_<float>,
                0.0f));
            // Left
            starting_vectors.push_back(Pose2f(
                starting_corner_x + (x + 1) * RACK_SEPARATION_M_<float>,
                starting_corner_y + y * TRACK_SEPARATION_M_<float>,
                180.0f));
        }
    }
    return starting_vectors;

}



void MiningPlanner::populateStripMap(Eigen::MatrixXf& strip_map, MiningDirection direction)
{
    // This function will take
}

// Helper function to populate a strip map for a given direction
// void MiningPlanner::populateStripMap(
//     Eigen::MatrixXf& strip_map,
//     MiningDirection direction)
// {
//     int primary_dim, secondary_dim;
//     float angle;
//     switch (direction)
//     {
//         case MiningDirection::UP:
//         case MiningDirection::DOWN:
//         {
//             primary_dim = this->mapped_matrix_width;
//             secondary_dim = this->mapped_matrix_height;
//             angle = 90.f;
//             break;
//         }
//         case MiningDirection::LEFT:
//         case MiningDirection::RIGHT:
//         {
//             primary_dim = this->mapped_matrix_height;
//             secondary_dim = this->mapped_matrix_width;
//             angle = 0.f;
//             break;
//         }
//     }

//     for (int i = 0; i < secondary_dim; i++)
//     {
//         float secondary_pos = i * TRACK_SEPARATION_M_<float>;

//         int row = 0;
//         while (row < primary_dim)
//         {
//             float primary_pos = row * TRACK_SEPARATION_M_<float>;
//             // float distance = perception_eval(secondary_pos, primary_pos, angle);
//             float distance = 0.f;
//             int cells_clear = (int)(distance / TRACK_SEPARATION_M_<float>);

//             for (int j = row; j < primary_dim; j++)
//             {
//                 float distance_to_cell_end =
//                     (j - row + 1) * TRACK_SEPARATION_M_<float>;
//                 if (distance >= distance_to_cell_end)
//                 {
//                     strip_map(j, i) = 1.0f;
//                 }
//                 else
//                 {
//                     float distance_to_cell_start =
//                         (j - row) * TRACK_SEPARATION_M_<float>;
//                     if (distance <= distance_to_cell_start)
//                     {
//                         strip_map(j, i) = 0.0f;
//                     }
//                     else
//                     {
//                         strip_map(j, i) = (distance - distance_to_cell_start) /
//                                           TRACK_SEPARATION_M_<float>;
//                     }
//                     break;
//                 }
//             }
//             row += cells_clear + 1;
//         }
//     }
// }

void MiningPlanner::appendPlannedMiningPaths(
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
                DirectedMiningPath::MiningPath possible_path =
                    std::pair(Eigen::Vector2i(a, i), Eigen::Vector2i(b - 1, i));

                DirectedMiningPath dir_mining_path(
                    possible_path,
                    mining_dir,
                    &mat);

                if (dir_mining_path.getDistance() >=
                    robot_params.min_zone_length)
                {
                    this->all_mining_paths.push_back(dir_mining_path);
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

void MiningPlanner::sortPathsByQuality()
{
    std::sort(
        this->all_mining_paths.begin(),
        this->all_mining_paths.end(),
        [this](const DirectedMiningPath& a, const DirectedMiningPath& b)
        {
            return a.getQuality(this->times_mined_count_matrix) >
                   b.getQuality(this->times_mined_count_matrix);
        });
}

void MiningPlanner::removeSectionsForRobotClearance()
{
    for (int i = this->all_mining_paths.size() - 1; i >= 0; --i)
    {
        // std::cout << "\n=== Adjusting Path for Robot Clearance ===\n";
        // dir_mining_paths[i].print();
        if (!this->all_mining_paths[i].adjustForRobotClearance())
        {
            // std::cout << "Path removed.\n";
            this->all_mining_paths.erase(this->all_mining_paths.begin() + i);
        }
    }
}

};  // namespace lance
