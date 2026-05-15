#pragma once

#include <optional>
#include <utility>

#include "robot/core/motor_interface.hpp"

class LinearDriveController
{
public:
    LinearDriveController() = default;

    /// @brief Constructor
    /// @param dist_m Distance to drive in meters. Positive is fwd. Negitive is bkwd
    /// @param default_track_rps Track rotation speed
    /// @param epsilon_m_ How close is close enough, in meters
    LinearDriveController(
        float dist_m,
        float default_track_rps,
        float epsilon_m_);

    /// @brief Update function
    /// @param ltrack ltrack info
    /// @param rtrack rtrack info
    void updateOdom(
        const lance::TalonInfoMsg& ltrack,
        const lance::TalonInfoMsg& rtrack);

    /// @brief Query if the controller is done
    /// @return True if it is close enough to the back point
    bool hasRemaining() const;

    /// @brief Get how much distance is left
    /// @return Signed distance value. Positive if point is fwd of robot, negitive otherwise
    float remaining() const;

    /// @brief Gets the track velocities corrected for drift
    /// @return track velocities in the order left, right
    std::pair<float, float> get_track_velocities() const;

private:
    float dist_m;
    float rps;
    float epsilon_m;

    std::optional<float> l_track_start_pose;
    std::optional<float> l_track_prev_pose;

    std::optional<float> r_track_start_pose;
    std::optional<float> r_track_prev_pose;
};
