/*******************************************************************************
*   Copyright (C) 2025-2026 Cardinal Space Mining Club                         *
*                                                                              *
*                                 ;xxxxxxx:                                    *
*                                ;$$$$$$$$$       ...::..                      *
*                                $$$$$$$$$$x   .:::::::::::..                  *
*                             x$$$$$$$$$$$$$$::::::::::::::::.                 *
*                         :$$$$$&X;      .xX:::::::::::::.::...                *
*                 .$$Xx++$$$$+  :::.     :;:   .::::::.  ....  :               *
*                :$$$$$$$$$  ;:      ;xXXXXXXXx  .::.  .::::. .:.              *
*               :$$$$$$$$: ;      ;xXXXXXXXXXXXXx: ..::::::  .::.              *
*              ;$$$$$$$$ ::   :;XXXXXXXXXXXXXXXXXX+ .::::.  .:::               *
*               X$$$$$X : +XXXXXXXXXXXXXXXXXXXXXXXX; .::  .::::.               *
*                .$$$$ :xXXXXXXXXXXXXXXXXXXXXXXXXXXX.   .:::::.                *
*                 X$$X XXXXXXXXXXXXXXXXXXXXXXXXXXXXx:  .::::.                  *
*                 $$$:.XXXXXXXXXXXXXXXXXXXXXXXXXXX  ;; ..:.                    *
*                 $$& :XXXXXXXXXXXXXXXXXXXXXXXX;  +XX; X$$;                    *
*                 $$$: XXXXXXXXXXXXXXXXXXXXXX; :XXXXX; X$$;                    *
*                 X$$X XXXXXXXXXXXXXXXXXXX; .+XXXXXXX; $$$                     *
*                 $$$$ ;XXXXXXXXXXXXXXX+  +XXXXXXXXx+ X$$$+                    *
*               x$$$$$X ;XXXXXXXXXXX+ :xXXXXXXXX+   .;$$$$$$                   *
*              +$$$$$$$$ ;XXXXXXx;;+XXXXXXXXX+    : +$$$$$$$$                  *
*               +$$$$$$$$: xXXXXXXXXXXXXXX+      ; X$$$$$$$$                   *
*                :$$$$$$$$$. +XXXXXXXXX;      ;: x$$$$$$$$$                    *
*                ;x$$$$XX$$$$+ .;+X+      :;: :$$$$$xX$$$X                     *
*               ;;;;;;;;;;X$$$$$$$+      :X$$$$$$&.                            *
*               ;;;;;;;:;;;;;x$$$$$$$$$$$$$$$$x.                               *
*               :;;;;;;;;;;;;.  :$$$$$$$$$$X                                   *
*                .;;;;;;;;:;;    +$$$$$$$$$                                    *
*                  .;;;;;;.       X$$$$$$$:                                    *
*                                                                              *
*   Unless required by applicable law or agreed to in writing, software        *
*   distributed under the License is distributed on an "AS IS" BASIS,          *
*   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.   *
*   See the License for the specific language governing permissions and        *
*   limitations under the License.                                             *
*                                                                              *
*******************************************************************************/

#pragma once

#include <array>
#include <cmath>
#include <chrono>
#include <limits>
#include <vector>
#include <optional>

#include "robot/core/robot_params.hpp"
#include "robot/model/geometry.hpp"
#include "robot/sensing/mining_eval.hpp"
#include "robot/sensing/tf_cache.hpp"


namespace lance
{

/* Selects a mining route from the robot's current position. Headings are
 * swept through a full 360 degrees. Each heading's range ends at the first
 * obstacle (mining eval) or where the trencher would exit the mining zone,
 * whichever comes first.
 *
 * Routes with one turn are also considered: for every heading, the robot
 * could stop at 1/3 or 2/3 of its range, turn in place, and mine the longest
 * second leg from there. A two-leg route is only chosen if its total length
 * beats the best straight route by TWO_LEG_MIN_GAIN. */
class PolarMiningPlanner
{
public:
    using Vec2f = lance::geom::Vec2f;
    using Pose2f = lance::geom::Pose2f;

    struct Result
    {
        Vec2f origin;
        Vec2f dir;
        float range;  // total planned mining length (both legs)
        // Distance to mine along `dir` before pivoting (inf = straight route)
        float leg1_dist{std::numeric_limits<float>::infinity()};
        std::optional<Vec2f> leg2_dir;
    };

    static constexpr int STEP_DEG = 5;  // must divide 360
    static constexpr int N_HEADINGS = 360 / STEP_DEG;

    // Last completed sweep, kept for telemetry visualization
    struct Sweep
    {
        Vec2f origin{Vec2f::Zero()};
        std::array<float, N_HEADINGS> headings{};
        std::array<float, N_HEADINGS> ranges{};
        int selected{-1};  // -1 if no heading was accepted
        bool valid{false};

        // Second leg of the selected route, if a two-leg route was chosen
        bool has_leg2{false};
        Vec2f pivot{Vec2f::Zero()};
        float leg2_heading{0.f};
        float leg2_range{0.f};
    };

public:
    PolarMiningPlanner(
        MiningEvalInterface& mining_eval,
        const TfCache& tf_cache,
        const RobotParams& params);

public:
    void reset();
    /* Returns false while waiting on mining eval results. Once complete, `out`
     * holds the best heading, or nullopt if no heading is long enough (or the
     * robot is not inside the mining zone). */
    bool iterate(std::optional<Result>& out);
    /* Straight-only sweep from the robot's current pose, used at the pivot of
     * a two-leg route to re-plan leg 2 against the current terrain. Headings
     * within NO_BACKTRACK_DEG of straight back along `leg1_dir` are skipped.
     * Returns false while waiting; `out` holds the best heading, or nullopt if
     * none is at least MIN_LEG_M long. */
    bool iteratePivot(const Vec2f& leg1_dir, std::optional<Result>& out);

    const Sweep& getLastSweep() const { return this->last_sweep; }
    void clearLastSweep() { this->last_sweep.valid = false; }

protected:
    enum class State
    {
        IDLE,
        WAIT_LEG1,
        WAIT_LEG2
    };

    // One second-leg query: first-leg heading i, pivot fraction k, heading j
    struct Leg2Query
    {
        int i;
        int k;
        int j;
    };

protected:
    bool sendLeg1();
    bool processLeg1();
    void sendLeg2();
    std::optional<Result> chooseRoute(const std::vector<float>* leg2_dists);

    Vec2f headingDir(int i) const;
    float rangeToZoneEdge(const Vec2f& p, float theta) const;

protected:
    static constexpr float STEP_RAD =
        static_cast<float>(STEP_DEG) * static_cast<float>(M_PI) / 180.f;

    // A two-leg route must be at least this much longer than the best
    // straight route to be chosen (the mid-run turn costs time)
    static constexpr float TWO_LEG_MIN_GAIN = 1.2f;
    // Second legs within this angle of straight back along leg 1 are skipped
    static constexpr int NO_BACKTRACK_DEG = 30;
    // Each leg of a two-leg route must be at least this long
    static constexpr float MIN_LEG_M = 0.25f;
    // Pivot points as fractions of the first leg's range
    static constexpr std::array<float, 2> PIVOT_FRACTIONS{1.f / 3.f, 2.f / 3.f};

    MiningEvalInterface& mining_eval;
    const TfCache& tf_cache;
    const RobotParams& params;

    State state{State::IDLE};
    Vec2f query_origin{Vec2f::Zero()};
    std::array<float, N_HEADINGS> query_headings{};
    std::array<float, N_HEADINGS> leg1_ranges{};
    std::vector<Leg2Query> leg2_queries;
    std::chrono::steady_clock::time_point leg2_sent_time;
    Sweep last_sweep;
};

};  // namespace lance
