#pragma once
// =============================================================================
// sim_utils.hpp - Simulation testing utilities for sim_main.cpp
//
// Bundles together all test-only concerns so that sim_main.cpp can focus
// on scenario selection and the top-level simulation loop:
//
//   - plantStep()       - nonlinear ground-truth plant model
//   - Path factories    - makeStraightPath, makeSharpLPath, makeFigure8Path,
//                         loadPathFromFile
//   - StateNoise        - configurable Gaussian position/heading noise
//   - StuckDetector     - sliding-window net-displacement + speed check
//   - FrameLogger       - JSON-lines frame builder and file writer
// =============================================================================

#include "mpc/path.hpp"
#include "mpc/types.hpp"
#include "mpc/params.hpp"
#include "mpc/mpc_controller.hpp"

#include <string>
#include <vector>
#include <fstream>


namespace mpc
{

// Frame logger

// Writes simulation output as JSON-lines to stdout and optionally to a file.
//
// Two record types are emitted:
//   - A header record (once, via logParams) with MPC params for the visualiser.
//   - Per-step frame records (via logFrame) with robot state, control,
//     debug info, and the current path geometry.
//
// Both stdout and the file receive every record so that the visualiser can
// read directly from the binary's stdout or replay from the saved file.
class FrameLogger
{
public:
    // @param filepath  Path for the JSONL output file.  Pass an empty string
    //                  to suppress file output (stdout-only).
    explicit FrameLogger(const std::string& filepath = "sim_data.jsonl");
    ~FrameLogger();

    // Emit the params header record.  Call once before the sim loop.
    void logParams(const MPCParams& p, int max_steps);

    // Emit one frame record for the current simulation step.
    void logFrame(
        double sim_t,
        const State& x,
        const Control& u,
        const Path& path,
        const DebugInfo& dbg);

private:
    std::ofstream file_;

    // Internal helpers
    static std::string buildFrame(
        double sim_t,
        const State& x,
        const Control& u,
        const Path& path,
        const DebugInfo& dbg);

    static void jd(std::ostream& s, double v, int prec = 5);
};

}  // namespace mpc
