// =============================================================================
// sim_utils.cpp - Simulation testing utilities (see sim_utils.hpp)
// =============================================================================

#include "sim_utils.hpp"

#include <cmath>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <iostream>
#include <stdexcept>


namespace mpc
{

// Frame logger

FrameLogger::FrameLogger(const std::string& filepath)
{
    if (!filepath.empty())
    {
        file_.open(filepath);
    }
}

FrameLogger::~FrameLogger() = default;

void FrameLogger::logParams(const MPCParams& p, int max_steps)
{
    std::ostringstream ss;
    ss << "{\"params\":{"
       << "\"N\":" << p.N << ",\"dt\":" << p.dt << ",\"v_max\":" << p.v_max
       << ",\"v_min\":" << p.v_min  // needed by the Python visualiser
       << ",\"omega_max\":" << p.omega_max << ",\"d_hard\":" << p.d_hard
       << ",\"max_steps\":" << max_steps << "}}";

    const std::string line = ss.str();
    // std::cout << line << "\n";
    if (file_.is_open())
    {
        file_ << line << "\n";
    }
}

void FrameLogger::logFrame(
    double sim_t,
    const State& x,
    const Control& u,
    const Path& path,
    const DebugInfo& dbg)
{
    const std::string line = buildFrame(sim_t, x, u, path, dbg);
    // std::cout << line << "\n";
    // std::cout.flush();
    if (file_.is_open())
    {
        file_ << line << "\n";
    }
}

// FrameLogger internals

void FrameLogger::jd(std::ostream& s, double v, int prec)
{
    s << std::fixed << std::setprecision(prec) << v;
}

std::string FrameLogger::buildFrame(
    double sim_t,
    const State& x,
    const Control& u,
    const Path& path,
    const DebugInfo& dbg)
{
    std::ostringstream ss;
    ss << std::fixed << std::setprecision(6) << "{";

    ss << "\"t\":";
    jd(ss, sim_t, 4);
    ss << ",";

    ss << "\"robot\":{"
       << "\"x\":" << x.x << ",\"y\":" << x.y << ",\"theta\":" << x.theta
       << "},";

    ss << "\"control\":{"
       << "\"v\":" << u.v << ",\"omega\":" << u.omega << "},";

    ss << "\"cte\":";
    jd(ss, dbg.cte_raw);
    ss << ",";

    ss << "\"proj\":[";
    jd(ss, dbg.proj_pt.x());
    ss << ",";
    jd(ss, dbg.proj_pt.y());
    ss << "],";

    ss << "\"solver_ok\":" << (dbg.solver_ok ? "true" : "false") << ",";

    ss << "\"solve_ms\":";
    jd(ss, dbg.solve_ms, 4);
    ss << ",";

    ss << "\"pred\":[";
    for (int k = 0; k < static_cast<int>(dbg.pred_traj.size()); ++k)
    {
        if (k)
        {
            ss << ",";
        }
        ss << "[" << dbg.pred_traj[k].x << "," << dbg.pred_traj[k].y << ","
           << dbg.pred_traj[k].theta << "]";
    }
    ss << "],";

    ss << "\"ref\":[";
    for (int k = 0; k < static_cast<int>(dbg.ref_snap.x_ref.size()); ++k)
    {
        if (k)
        {
            ss << ",";
        }
        ss << "[" << dbg.ref_snap.x_ref[k].x << "," << dbg.ref_snap.x_ref[k].y
           << "," << dbg.ref_snap.x_ref[k].theta << "]";
    }
    ss << "],";

    ss << "\"path\":[";
    for (int i = 0; i < static_cast<int>(path.size()); ++i)
    {
        if (i)
        {
            ss << ",";
        }
        ss << "[" << path.pts[i].pos.x() << "," << path.pts[i].pos.y() << "]";
    }
    ss << "]}";

    return ss.str();
}

}  // namespace mpc
