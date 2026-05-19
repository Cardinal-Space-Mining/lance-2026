#include <numbers>

#include "track_slip_detector.hpp"

#include "robot/model/dynamics.hpp"


TrackSkidDetector::TrackSkidDetector() : data_points(25) {}

void TrackSkidDetector::update(
    double track_left_raw,
    double track_right_raw,
    double dlo_x,
    double dlo_y,
    double dlo_theta)
{
    data_points.push_back(
        TrackSkidDetector::Internal{
            .track_left_raw = track_left_raw,
            .track_right_raw = track_right_raw,
            .dlo_x_m = dlo_x,
            .dlo_y_m = dlo_y,
            .dlo_theta = dlo_theta,
            .time_point = std::chrono::steady_clock::now()});
    // invalidate cache
    err_cache.reset();
}

double TrackSkidDetector::get_positional_error()
{
    if (!err_cache.has_value())
    {
        auto v = this->calculate_errors();
        err_cache = v;
    }
    return err_cache.value().pose_error_m;
}

double TrackSkidDetector::get_rotational_error()
{
    if (!err_cache.has_value())
    {
        auto v = this->calculate_errors();
        err_cache = v;
    }
    return err_cache.value().angle_error_rad;
}

TrackSkidDetector::ErrCache TrackSkidDetector::calculate_errors() const
{
    if (data_points.size() <= 1)
    {
        return ErrCache{.pose_error_m = 0, .angle_error_rad = 0};
    }


    // Set initial track pose from initial lidar pose
    double track_x = data_points.begin()->dlo_x_m;
    double track_y = data_points.begin()->dlo_y_m;
    double track_theta = data_points.begin()->dlo_theta;

    for (size_t i = 1; i < data_points.size(); i++)
    {
        // Calculate odom differential steps
        double dl_m = lance::trackMotorRpsToGroundMps(
            data_points[i].track_left_raw - data_points[i - 1].track_left_raw);
        double dr_m = lance::trackMotorRpsToGroundMps(
            data_points[i].track_right_raw -
            data_points[i - 1].track_right_raw);

        double fwd_dx_m = lance::trackVelocitiesToForwardVelocity(dl_m, dr_m);
        double d_theta = lance::trackVelocitiesToAngularVelocity(dl_m, dr_m);

        // World space differential steps
        double dx = fwd_dx_m * std::cos(track_theta);
        double dy = fwd_dx_m * std::sin(track_theta);

        // Update pose
        track_x += dx;
        track_y += dy;
        track_theta += d_theta;
    }

    double err_x = track_x - data_points.back().dlo_x_m;
    double err_y = track_y - data_points.back().dlo_y_m;
    double dist_err = std::hypot(err_x, err_y);

    double err_theta = std::fmod(
        track_theta - data_points.back().dlo_theta,
        2.0 * std::numbers::pi);
    if (err_theta < 0)
    {
        err_theta += 2 * std::numbers::pi;
    }

    return ErrCache{.pose_error_m = dist_err, .angle_error_rad = err_theta};
}
