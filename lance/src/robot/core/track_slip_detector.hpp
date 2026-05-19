#pragma once

#include <optional>
#include <vector>
#include <chrono>

#include <boost/circular_buffer.hpp>



class TrackSkidDetector
{
public:
    TrackSkidDetector();

    /// @brief Updates skid detector
    /// @param track_left_raw Left track encoder values
    /// @param track_right_raw Right track encoder values
    /// @param dlo_x Lidar global x estimation in meters
    /// @param dlo_y Lidar global y estimation in meters
    /// @param dlo_theta Lidar global x estimation in radians
    void update(
        double track_left_raw,
        double track_right_raw,
        double dlo_x,
        double dlo_y,
        double dlo_theta);


    /// @brief calculates position error between lidar and odometry
    /// @return a positive position error in meters
    double get_positional_error();

    /// @brief Calculates angular error between lidar and odometry
    /// @return a positive rotational error in radians
    double get_rotational_error();

private:
    struct ErrCache
    {
        double pose_error_m;
        double angle_error_rad;
    };

    std::optional<TrackSkidDetector::ErrCache> err_cache;

    TrackSkidDetector::ErrCache calculate_errors() const;

private:
    struct Internal
    {
        double track_left_raw;
        double track_right_raw;
        double dlo_x_m;
        double dlo_y_m;
        double dlo_theta;
        std::chrono::steady_clock::time_point time_point;
    };

    boost::circular_buffer<TrackSkidDetector::Internal> data_points;
};
