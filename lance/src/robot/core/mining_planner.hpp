#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <iostream>
#include <stdexcept>
#include <vector>
#include <functional>
#include <algorithm>

#include "robot_params.hpp"

using MiningPaths = std::vector<std::pair<Eigen::Vector2i, Eigen::Vector2i>>;
using MiningPath = std::pair<Eigen::Vector2i, Eigen::Vector2i>;
using Box2f = Eigen::AlignedBox2f;

const float previously_mined_penalty = 0.1f;



// Going to need to update Robot Params with new values
// Or need to change my code to use the names of existing values in Robot Params

struct RobotParams;
// {
// const float track_Width = 1;// 0.5-0.75
// const float full_Width = 1.0;
// const float track_to_center = full_Width/2;
// const float min_zone_length = 1.1; // 2 The minimum length that should be considered for a mining path. Longer than path length since the length of the robot isn't factored into this yet
// const float min_path_length = 2; // 2 The minimum length that the robot should be mining for, if the mining distance is less than this, it will not save that path
// // MUST KEEP AN EYE ON ROBOT_LENGTH/TRACK_WIDTH BECAUSE IF IT IS JUST OVER THEN IT COULD REDUCE LEAVE WAY TOO MUCH CLEARANCE AND WASTE MINING SECTIONS
// const float robot_length = .99;
// const int sections_to_exclude_for_robot_clearance = (int)(robot_length / track_Width)+1; // How many sections to exclude from the end of a mining path to ensure the robot can fit in the spot without hitting an obstacle with its back end
// };

enum class MiningDirection
{
    UP,
    DOWN,
    LEFT,
    RIGHT
};

inline std::string miningDirectionToString(MiningDirection dir)
{
    switch (dir)
    {
        case MiningDirection::UP:
            return "UP";
        case MiningDirection::DOWN:
            return "DOWN";
        case MiningDirection::LEFT:
            return "LEFT";
        case MiningDirection::RIGHT:
            return "RIGHT";
        default:
            return "UNKNOWN";
    }
}

using DirectedMiningPaths = std::vector<class DirectedMiningPath>;


// Struct to represent a mining path with its direction and associated matrix for distance calculation
class DirectedMiningPath
{
public:
    MiningPath path;
    MiningDirection direction;
    const Eigen::MatrixXf* matrix;
    const RobotParams& m_robot_params;
    float distance = -1.0;

    DirectedMiningPath(
        MiningPath p,
        MiningDirection dir,
        const Eigen::MatrixXf* mat,
        const RobotParams& robot_params) :
        path(std::move(p)),
        direction(dir),
        matrix(mat),
        m_robot_params(robot_params)
    {
        distance = calculate_distance(path, matrix);
    }

    // void print() const {
    //     /* Method is only for debugging purposes*/
    //     std::cout << "Start: (" << path.first.x() << ", " << path.first.y()
    //             << ") -> End: (" << path.second.x() << ", " << path.second.y()
    //             << ") | Direction: " << miningDirectionToString(direction) <<
    //             " | Distance: " << distance << "\n";
    // }

    // void print_path_in_base_frame() const {
    //     /* Method is only for debugging purposes*/
    //     MiningPath p = to_base_coordinates();
    //     std::cout << "Base Frame - Start: (" << p.first.x() << ", " << p.first.y()
    //             << ") -> End: (" << p.second.x() << ", " << p.second.y()
    //             << ") | Direction: " << miningDirectionToString(direction) <<
    //             " | Distance: " << distance << "\n";
    // }

    float get_distance() const { return distance; }

    void mark_mining_on_matrix(Eigen::MatrixXi& mined_count_matrix) const
    {
        MiningPath p = to_base_coordinates();
        for (int i = p.first.x(); i <= p.second.x(); i++)
        {
            for (int j = p.first.y(); j <= p.second.y(); j++)
            {
                mined_count_matrix(i, j) += 1;
            }
        }
    }

    static float calculate_distance(
        const MiningPath& p,
        const Eigen::MatrixXf* mat)
    {
        if (mat == nullptr)
        {
            return -1.0f;
        }

        const auto& start = p.first;
        const auto& end = p.second;

        if (end.x() != start.x())
        {
            return std::abs(
                end.x() - start.x() + 1 - (1 - (*mat)(end.x(), end.y())));
        }
        return std::abs(
            end.y() - start.y() + 1 - (1 - (*mat)(end.x(), end.y())));
    }

    bool check_validity() const
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
        if (path.first.x() >= matrix->rows() ||
            path.second.x() >= matrix->rows())
        {
            return false;
        }
        if (path.first.y() >= matrix->cols() ||
            path.second.y() >= matrix->cols())
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

    float get_qualtiy(const Eigen::MatrixXi& previously_minined_locations) const
    {
        float quality = 0.0f;
        quality += distance;

        MiningPath p = to_base_coordinates();

        for (int i = p.first.x(); i <= p.second.x(); i++)
        {
            for (int j = p.first.y(); j <= p.second.y(); j++)
            {
                quality -= previously_minined_locations(i, j) *
                           previously_mined_penalty;
            }
        }

        return quality;
    }

    bool adjust_for_robot_clearance()
    {
        /*
        Adjusts the path for robot clearance requirements.
        */
        const int c = m_robot_params.sections_to_exclude_for_robot_clearance;

        switch (direction)
        {
            case MiningDirection::DOWN:
                path.first.x() += c;
                break;
            case MiningDirection::UP:
                path.first.x() += c;
                break;
            case MiningDirection::RIGHT:
                path.first.x() += c;
                break;
            case MiningDirection::LEFT:
                // path.second.x() -= c;
                path.first.x() += c;
                break;
        }

        if (!check_validity())
        {
            return false;
        }
        distance = calculate_distance(path, matrix);
        // std::cout << "got new distance " << distance << " for point " << path.first.x() << ", " << path.first.y() << " to " << path.second.x() << ", " << path.second.y() << " after clearance adjustment: ";

        return distance >= 0.0f;
    }

    std::pair<Eigen::Vector2f, Eigen::Vector2f>
        get_path_coordinates_in_world_frame(float cell_size) const
    {
        MiningPath p = to_base_coordinates();

        // Eigen::Vector2f target_pos = this->params.mining_zone_bounds.max() - Eigen::Vector2f::Constant(0.8f);
        // Eigen::Vector2f target_dir{0.f, 1.f};
        float stdX = p.first.y() * cell_size;
        float stdY = p.first.x() * cell_size;
        Eigen::Vector2f target_pos;
        Eigen::Vector2f target_dir;

        switch (direction)
        {
            case MiningDirection::DOWN:
                target_pos = Eigen::Vector2f(stdX, stdY + cell_size);
                target_dir = Eigen::Vector2f({0.f, 1.f});
                break;
            case MiningDirection::UP:
                target_pos = Eigen::Vector2f(stdX, stdY - cell_size);
                target_dir = Eigen::Vector2f({0.f, -1.f});
                break;
            case MiningDirection::RIGHT:
                target_pos = Eigen::Vector2f(stdX - cell_size, stdY);
                target_dir = Eigen::Vector2f({1.f, 0.f});
                break;
            case MiningDirection::LEFT:
                // path.second.x() -= c;
                target_pos = Eigen::Vector2f(stdX + cell_size, stdY);
                target_dir = Eigen::Vector2f({-1.f, 0.f});
                break;
        }

        return {target_pos, target_dir};
    }

private:
    MiningPath to_base_coordinates() const
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
};

float const get_distance_for_path(const DirectedMiningPath& directed_path)
{
    return DirectedMiningPath::calculate_distance(
        directed_path.path,
        directed_path.matrix);
}

class MiningPlanner
{
private:
    RobotParams& m_robot_params;
    std::function<float(float, float, float)> perception_eval;
    Box2f mining_zone_bounds;


    // The direction is the way the the robot would be moving in reference to the base frame which is MiningDirection::DOWN
    Eigen::MatrixXf strip_map_up;
    Eigen::MatrixXf strip_map_down;
    Eigen::MatrixXf strip_map_left;
    Eigen::MatrixXf strip_map_right;
    Eigen::MatrixXi times_mined_count_matrix;


    DirectedMiningPaths all_mining_paths;

    int mapped_matrix_width;
    int mapped_matrix_height;
    int horizontal_matrix_width;
    int horizontal_matrix_height;

public:
    MiningPlanner(RobotParams& robot_params) :
        m_robot_params(robot_params),
        mapped_matrix_width(0),
        mapped_matrix_height(0),
        horizontal_matrix_width(0),
        horizontal_matrix_height(0)
    {
    }

    MiningPlanner(
        std::function<float(float, float, float)> perception_eval,
        RobotParams& robot_params,
        Box2f mining_zone_bounds) :
        m_robot_params(robot_params),
        perception_eval(perception_eval),
        mining_zone_bounds{mining_zone_bounds}
    {
        mapped_matrix_width =
            (int)(mining_zone_bounds.max().x() / robot_params.track_Width);
        mapped_matrix_height =
            (int)(mining_zone_bounds.max().y() / robot_params.track_Width);

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


    // should be REMOVED LATER
    Eigen::MatrixXi get_times_mined_count_matrix() const
    {
        return times_mined_count_matrix;
    }

    void mark_mining_on_matrix(const DirectedMiningPath& path)
    {
        path.mark_mining_on_matrix(times_mined_count_matrix);
    }

    DirectedMiningPaths final_output()
    {
        DirectedMiningPaths vertical_up_paths;
        DirectedMiningPaths vertical_down_paths;
        DirectedMiningPaths horizontal_left_paths;
        DirectedMiningPaths horizontal_right_paths;
        populate_planned_mining_paths(
            vertical_up_paths,
            strip_map_up,
            MiningDirection::UP);
        populate_planned_mining_paths(
            vertical_down_paths,
            strip_map_down,
            MiningDirection::DOWN);
        populate_planned_mining_paths(
            horizontal_left_paths,
            strip_map_left,
            MiningDirection::LEFT);
        populate_planned_mining_paths(
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

        remove_sections_for_robot_clearance(all_mining_paths);
        sort_paths_by_quality(all_mining_paths, times_mined_count_matrix);


        return all_mining_paths;
    }

    const Eigen::MatrixXf& get_strip_map(MiningDirection direction) const
    {
        /*Used for debugging only*/
        switch (direction)
        {
            case MiningDirection::UP:
                return strip_map_up;
            case MiningDirection::DOWN:
                return strip_map_down;
            case MiningDirection::LEFT:
                return strip_map_left;
            case MiningDirection::RIGHT:
                return strip_map_right;
            default:
                return strip_map_up;
        }
    }
    void update_mapped_matrices()
    {
        // This function is used to populate the 4 strip maps based on the perception evaluation function. It doesn't need to be called very often but could be to refresh data
        populate_strip_map(
            strip_map_up,
            mapped_matrix_width,
            mapped_matrix_height,
            MiningDirection::UP,
            perception_eval);
        populate_strip_map(
            strip_map_down,
            mapped_matrix_width,
            mapped_matrix_height,
            MiningDirection::DOWN,
            perception_eval);
        populate_strip_map(
            strip_map_left,
            mapped_matrix_height,
            mapped_matrix_width,
            MiningDirection::LEFT,
            perception_eval);
        populate_strip_map(
            strip_map_right,
            mapped_matrix_height,
            mapped_matrix_width,
            MiningDirection::RIGHT,
            perception_eval);
    }

private:
    void sort_paths_by_quality(
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
                return a.get_qualtiy(previously_mined_locations) >
                       b.get_qualtiy(previously_mined_locations);
            });
    }

    void populate_planned_mining_paths(
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
                    MiningPath possible_path = std::pair(
                        Eigen::Vector2i(a, i),
                        Eigen::Vector2i(b - 1, i));

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

    void remove_sections_for_robot_clearance(
        DirectedMiningPaths& dir_mining_paths)
    {
        for (int i = dir_mining_paths.size() - 1; i >= 0; --i)
        {
            // std::cout << "\n=== Adjusting Path for Robot Clearance ===\n";
            // dir_mining_paths[i].print();
            if (!dir_mining_paths[i].adjust_for_robot_clearance())
            {
                // std::cout << "Path removed.\n";
                dir_mining_paths.erase(dir_mining_paths.begin() + i);
            }
        }
    }


    // Helper function to populate a strip map for a given direction
    void populate_strip_map(
        Eigen::MatrixXf& strip_map,
        int primary_dim,
        int secondary_dim,
        MiningDirection direction,
        std::function<float(float, float, float)> perception_eval)
    {
        float angle = (direction == MiningDirection::UP ||
                       direction == MiningDirection::DOWN)
                          ? 90.0f
                          : 0.0f;

        for (int i = 0; i < secondary_dim; i++)
        {
            float secondary_pos = i * m_robot_params.track_Width;

            int row = 0;
            while (row < primary_dim)
            {
                float primary_pos = row * m_robot_params.track_Width;
                float distance =
                    perception_eval(secondary_pos, primary_pos, angle);
                int cells_clear = (int)(distance / m_robot_params.track_Width);

                for (int j = row; j < primary_dim; j++)
                {
                    float distance_to_cell_end =
                        (j - row + 1) * m_robot_params.track_Width;
                    if (distance >= distance_to_cell_end)
                    {
                        strip_map(j, i) = 1.0f;
                    }
                    else
                    {
                        float distance_to_cell_start =
                            (j - row) * m_robot_params.track_Width;
                        if (distance <= distance_to_cell_start)
                        {
                            strip_map(j, i) = 0.0f;
                        }
                        else
                        {
                            strip_map(j, i) =
                                (distance - distance_to_cell_start) /
                                m_robot_params.track_Width;
                        }
                        break;
                    }
                }
                row += cells_clear + 1;
            }
        }
    }
};
