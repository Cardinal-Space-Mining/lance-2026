#include "offload_planner.hpp"

#include <cassert>

#include <cmath>
#include <iostream>
#include <limits>
namespace
{
/// @brief Returns true if two numbers are close
/// @param a num 1
/// @param b num 2
/// @param epsilon
/// @return true if two numbers are close
bool is_close(double a, double b, double epsilon = 1e-9)
{
    return std::abs(a - b) < epsilon;
}

bool almost_parallel(
    const Eigen::Vector2f& a,
    const Eigen::Vector2f& b,
    double epsilon = 1e-6)
{
    return is_close(a.normalized().dot(b.normalized()), 1, epsilon);
}

}  // namespace

OffloadPlanner::OffloadPlanner(
    Eigen::AlignedBox2f B_,
    Eigen::Vector2f A_,
    Eigen::Vector2f F_robot_,
    double f_robot_) :
    B(B_),
    A(A_),
    f_robot(f_robot_)
{
    Eigen::Vector2f b_right = B.corner(B.BottomRight) - B.corner(B.BottomLeft);
    Eigen::Vector2f b_up = B.corner(B.TopLeft) - B.corner(B.BottomLeft);
    // Assertions
    {
        assert(!B_.isEmpty());  // Assert box is not empty
        assert(
            almost_parallel(A, b_right) || almost_parallel(A, b_up) ||
            almost_parallel(A, -b_right) ||
            almost_parallel(A, -b_up));  // Assert A_ is along one box axis
        assert(
            B_.volume() >
            F_robot_
                .norm());  // Assert the offload zone can fit in the offload area
        assert(A.norm() == 1);  // Assert A is axis aligned and normalized
    }

    {
        if (almost_parallel(A, b_up))  // 1
        {
            this->start_point = B.corner(B.BottomLeft);
            this->n_y = std::floor(b_up.norm() / F_robot_.y());
            this->n_x = std::floor(b_right.norm() / F_robot_.x());
            this->d_y = b_up / n_y;
            this->d_x = b_right / n_x;
        }
        else if (almost_parallel(A, b_right))  // 2
        {
            this->start_point = B.corner(B.TopLeft);
            this->n_y = std::floor(b_right.norm() / F_robot_.y());
            this->n_x = std::floor(b_up.norm() / F_robot_.x());
            this->d_y = b_right / n_y;
            this->d_x = -b_up / n_x;
        }
        else if (almost_parallel(A, -b_up))  // 3
        {
            this->start_point = B.corner(B.TopRight);
            this->n_y = std::floor(b_up.norm() / F_robot_.y());
            this->n_x = std::floor(b_right.norm() / F_robot_.x());
            this->d_y = -b_up / n_y;
            this->d_x = -b_right / n_x;
        }
        else if (almost_parallel(A, -b_right))  // 4
        {
            this->start_point = B.corner(B.BottomRight);
            this->n_y = std::floor(b_right.norm() / F_robot_.y());
            this->n_x = std::floor(b_up.norm() / F_robot_.x());
            this->d_y = -b_right / n_y;
            this->d_x = b_up / n_x;
        }
        else
        {
            std::cerr
                << "We should never reach this line. However, we have. Oops."
                << __FILE__ << ':' << __LINE__ << std::endl;
            assert(false);
        }

        assert(n_x > 0);
        assert(n_y > 0);
    }
}

size_t OffloadPlanner::num_actions() const { return n_x * n_y; } // Total number of boxes that can fit in the offload zone

std::vector<OffloadPlanner::OffloadAction> OffloadPlanner::to_actions() const
{
    OffloadPlanner planner = *this;
    planner.i_x = 0;
    planner.i_y = 0;

    std::vector<OffloadPlanner::OffloadAction> actions(planner.num_actions());
    for (size_t i = 0; i < planner.num_actions(); i++)
    {
        actions[i] = planner.from_vec2(planner.next());
        planner.consume();
    }

    return actions;
}

Eigen::Vector2f OffloadPlanner::next()
{
    // where the next box should go (the / 2 factors centralize it, start point gets it to box 1, then how many boxes in both directions we are)
    return start_point + (d_y / 2) + (d_x / 2) + (i_x * d_x) + (i_y * d_y);
}

OffloadPlanner::OffloadAction OffloadPlanner::from_vec2(
    const Eigen::Vector2f& vec)
{
    return OffloadAction{.drive_point = (A * f_robot *2) + vec, .back_point = vec + (A * f_robot)};
}

void OffloadPlanner::consume()
{
    i_x++;
    if (i_x >= n_x)
    {
        i_x = 0;
        i_y++;
    }
}
