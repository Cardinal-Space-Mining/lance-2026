#pragma once

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>

class OffloadPlanner
{
public:
    /// @brief This represents an offload action enclosing the point to drive to, then the point to back to
    struct OffloadAction
    {
        /// @brief The point to drive the robot to
        Eigen::Vector2f drive_point;
        /// @brief The point to back the robot to
        Eigen::Vector2f back_point;
    };

public:
    /// @brief Constructor
    /// @param B_ Offload Bounding Box in Arena Frame
    /// @param A_ Offload Forward Direction in Arena Frame. Must be parallel with one side of B & Axis aligned
    /// @param F_robot_ Bounding Box that describes how large the offload cell size is. Y is across the robot. X is depth
    /// @param f_robot_ The distance from the center of the robot to the center of generated offload cells
    OffloadPlanner(
        Eigen::AlignedBox2f B_,
        Eigen::Vector2f A_,
        Eigen::Vector2f F_robot_,
        double f_robot_);

    size_t num_actions() const;

    std::vector<OffloadAction> to_actions() const;

private:
    /// @brief Returns the center of the next spot to offload
    /// @return A spot in arena space
    Eigen::Vector2f next();

    OffloadAction from_vec2(const Eigen::Vector2f& vec);

    /// @brief Marks the current box as filled and advances internal state to next box
    void consume();

private:
    Eigen::AlignedBox2f B;
    Eigen::Vector2f A;
    double f_robot;

private:
    Eigen::Vector2f start_point;
    Eigen::Vector2f d_x;
    Eigen::Vector2f d_y;
    size_t n_x;
    size_t i_x{0};
    size_t n_y;
    size_t i_y{0};
};
