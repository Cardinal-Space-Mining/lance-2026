#include "robot/control/shared/linear_drive_controller.hpp"
#include "robot/model/dynamics.hpp"


LinearDriveController::LinearDriveController(
    float dist_m,
    float default_track_rps,
    float epsilon_m_) :
    dist_m(dist_m),
    rps(default_track_rps),
    epsilon_m(epsilon_m_)
{
}

bool LinearDriveController::hasRemaining() const
{
    return std::abs(this->remaining()) > this->epsilon_m;
}

float LinearDriveController::remaining() const
{
    float d_l = l_track_prev_pose.value_or(0) - l_track_start_pose.value_or(0);
    float d_r = r_track_prev_pose.value_or(0) - r_track_start_pose.value_or(0);
    float avg_d = (d_l + d_r) / 2.0f;

    return this->dist_m - avg_d;
}

void LinearDriveController::updateOdom(
    const lance::TalonInfoMsg& ltrack,
    const lance::TalonInfoMsg& rtrack)
{
    if (!l_track_start_pose.has_value())
    {
        l_track_start_pose = lance::trackMotorRpsToGroundMps(ltrack.position);
    }
    if (!this->r_track_start_pose.has_value())
    {
        r_track_start_pose = lance::trackMotorRpsToGroundMps(rtrack.position);
    }



    this->l_track_prev_pose = lance::trackMotorRpsToGroundMps(ltrack.position);
    this->r_track_prev_pose = lance::trackMotorRpsToGroundMps(rtrack.position);
}

std::pair<float, float> LinearDriveController::get_track_velocities() const
{
    auto r_dist = remaining();
    auto spd = std::signbit(r_dist) ? -rps : rps;
    return std::make_pair(spd, spd);
}
