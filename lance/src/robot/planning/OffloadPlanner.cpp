#include "OffloadPlanner.hpp"

#include <cassert>

#include <cmath>
#include <limits>
namespace
{
    bool is_close(double a, double b, double epsilon = 1e-9)
    {
        return std::abs(a - b) < epsilon;
    }

    bool almost_parallel(const Eigen::Vector2d &a, const Eigen::Vector2d &b, double epsilon = 1e-9)
    {
        // Cross product of two parallel vectors is zero
        return ((a.x() * b.y()) - (a.y() * b.x())) < epsilon;
    }

}

OffloadPlanner::OffloadPlanner(Eigen::AlignedBox2d B_, Eigen::Vector2d A_, Eigen::Vector2d F_robot_, double f_robot_, double r_robot_, Eigen::Vector2d Q_arena_) : B(B_),
                                                                                                                                                                   A(A_),
                                                                                                                                                                   F_robot(F_robot_),
                                                                                                                                                                   f_robot(f_robot_),
                                                                                                                                                                   r_robot(r_robot_),
                                                                                                                                                                   Q_arena(Q_arena_)
{
    // Assertions
    {
        assert(!B_.isEmpty()); // Assert box is not empty
        auto b_up = B.corner(B.TopLeft) - B.corner(B.BottomLeft);
        auto b_right = B.corner(B.BottomRight) - B.corner(B.BottomLeft);
        assert(almost_parallel(A, b_right) || almost_parallel(A, b_up)); // Assert A_ is along one box axis
        assert(B_.volume() > F_robot.norm());                            // Assert the offload zone can fit in the offload area
        assert(A.norm() == 1);                                           // Assert A is axis aligned and normalized
    }

    {
        auto b_right = B.corner(B.BottomRight) - B.corner(B.BottomLeft);
        auto b_width = b_right.norm();
        auto b_up = B.corner(B.TopLeft) - B.corner(B.BottomLeft);
        auto b_height = b_up.norm();

        // NO IF STATEMENTS. ROTATE IN THE DIRECTION OF A

        if (A.x() > 0)
        {
            this->start_point = B.corner(B.TopRight);
            // this->d_row =
        }
        else if (A.x() < 0)
        {
            /* code */
        }
        else if (A.y() > 0)
        {
        }
        else if (A.y() < 0)
        {
        }
        else
        {
            assert(false);
        }
    }
}

Eigen::Vector2d OffloadPlanner::next()
{
    return (start_point + (d_row * row_idx) + (d_col * col_idx)) + edge_to_center;
}

OffloadPlanner::OffloadAction OffloadPlanner::from_vec2(const Eigen::Vector2d &vec)
{
    return OffloadAction{
        .drive_point = (-A.normalized()) * r_robot,
        .back_point = vec};
}

void OffloadPlanner::consume()
{
    row_idx++;
    if (row_idx >= n_row)
    {
        row_idx = 0;
        col_idx++;
    }
}
