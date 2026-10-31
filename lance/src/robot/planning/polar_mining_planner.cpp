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

#include "polar_mining_planner.hpp"

#include <limits>
#include <vector>
#include <iostream>
#include <algorithm>


namespace lance
{

PolarMiningPlanner::PolarMiningPlanner(
    MiningEvalInterface& mining_eval,
    const TfCache& tf_cache,
    const RobotParams& params) :
    mining_eval{mining_eval},
    tf_cache{tf_cache},
    params{params}
{
}

void PolarMiningPlanner::reset() { this->state = State::IDLE; }

bool PolarMiningPlanner::iterate(std::optional<Result>& out)
{
    switch (this->state)
    {
        case State::IDLE:
        {
            out.reset();
            if (!this->sendLeg1())
            {
                return true;
            }
            this->state = State::WAIT_LEG1;
            [[fallthrough]];
        }
        case State::WAIT_LEG1:
        {
            if (!this->mining_eval.hasResult())
            {
                return false;
            }
            if (!this->processLeg1())
            {
                this->state = State::IDLE;
                return true;
            }

            this->sendLeg2();
            if (this->leg2_queries.empty())
            {
                this->state = State::IDLE;
                out = this->chooseRoute(nullptr);
                return true;
            }
            this->state = State::WAIT_LEG2;
            return false;
        }
        case State::WAIT_LEG2:
        {
            if (!this->mining_eval.hasResult())
            {
                return false;
            }
            this->state = State::IDLE;

            const auto wait_ms =
                std::chrono::duration_cast<std::chrono::milliseconds>(
                    std::chrono::steady_clock::now() - this->leg2_sent_time)
                    .count();
            std::cout << "[PolarMiningPlanner] Second-leg eval: "
                      << this->leg2_queries.size() << " queries in "
                      << wait_ms << " ms\n";

            const std::vector<float>* dists = this->mining_eval.getDists();
            if (dists == nullptr || dists->size() < this->leg2_queries.size())
            {
                std::cout << "[PolarMiningPlanner] Second-leg eval returned "
                          << (dists ? dists->size() : 0) << " ranges, expected "
                          << this->leg2_queries.size()
                          << ". Considering straight routes only.\n";
                dists = nullptr;
            }

            out = this->chooseRoute(dists);
            return true;
        }
    }

    return true;
}

bool PolarMiningPlanner::sendLeg1()
{
    if (!this->tf_cache.hasTf(ROBOT_TO_ARENA_TF))
    {
        std::cout << "[PolarMiningPlanner] No robot pose available.\n";
        return false;
    }

    this->query_origin =
        this->tf_cache.getTf(ROBOT_TO_ARENA_TF)->pose.vec.template head<2>();

    if (!this->params.bounds.mining_zone.contains(this->query_origin))
    {
        std::cout << "[PolarMiningPlanner] Robot is not inside the mining "
                     "zone.\n";
        return false;
    }

    std::vector<Pose2f> poses;
    poses.reserve(N_HEADINGS);
    for (int i = 0; i < N_HEADINGS; i++)
    {
        const float theta = i * STEP_RAD;
        this->query_headings[i] = theta;
        poses.emplace_back(this->query_origin.x(), this->query_origin.y(), theta);
    }

    this->mining_eval.queryArenaFrame(poses);
    return true;
}

bool PolarMiningPlanner::processLeg1()
{
    const std::vector<float>* dists = this->mining_eval.getDists();
    if (dists == nullptr || dists->size() < N_HEADINGS)
    {
        std::cout << "[PolarMiningPlanner] Mining eval returned "
                  << (dists ? dists->size() : 0) << " ranges, expected "
                  << N_HEADINGS << ".\n";
        return false;
    }

    this->last_sweep.origin = this->query_origin;
    this->last_sweep.headings = this->query_headings;
    this->last_sweep.selected = -1;
    this->last_sweep.has_leg2 = false;
    this->last_sweep.valid = true;

    for (int i = 0; i < N_HEADINGS; i++)
    {
        const float theta = this->query_headings[i];
        // Path ends at the first obstacle or when leaving the mining zone
        const float obstacle_range = (*dists)[i];
        const float zone_range =
            this->rangeToZoneEdge(this->query_origin, theta);
        const float range = std::max(std::min(obstacle_range, zone_range), 0.f);
        this->leg1_ranges[i] = range;
        this->last_sweep.ranges[i] = range;

        std::cout << "[PolarMiningPlanner] heading " << (i * STEP_DEG)
                  << " deg: obstacle " << obstacle_range << " m, zone edge "
                  << zone_range << " m -> " << range << " m\n";
    }

    return true;
}

void PolarMiningPlanner::sendLeg2()
{
    constexpr int BACKTRACK_STEPS = NO_BACKTRACK_DEG / STEP_DEG;

    this->leg2_queries.clear();
    std::vector<Pose2f> poses;

    for (int i = 0; i < N_HEADINGS; i++)
    {
        const int reverse_j = (i + N_HEADINGS / 2) % N_HEADINGS;

        for (int k = 0; k < static_cast<int>(PIVOT_FRACTIONS.size()); k++)
        {
            const float d1 = PIVOT_FRACTIONS[k] * this->leg1_ranges[i];
            if (d1 < MIN_LEG_M)
            {
                continue;
            }
            const Vec2f pivot = this->query_origin + this->headingDir(i) * d1;

            for (int j = 0; j < N_HEADINGS; j++)
            {
                // Skip headings that double back over leg 1
                int diff = std::abs(j - reverse_j);
                diff = std::min(diff, N_HEADINGS - diff);
                if (diff <= BACKTRACK_STEPS)
                {
                    continue;
                }

                this->leg2_queries.push_back({i, k, j});
                poses.emplace_back(pivot.x(), pivot.y(), j * STEP_RAD);
            }
        }
    }

    if (!poses.empty())
    {
        this->mining_eval.queryArenaFrame(poses);
        this->leg2_sent_time = std::chrono::steady_clock::now();
    }
}

std::optional<PolarMiningPlanner::Result> PolarMiningPlanner::chooseRoute(
    const std::vector<float>* leg2_dists)
{
    // Best straight route
    int best_i = -1;
    float best_straight = -std::numeric_limits<float>::infinity();
    for (int i = 0; i < N_HEADINGS; i++)
    {
        if (this->leg1_ranges[i] > best_straight)
        {
            best_straight = this->leg1_ranges[i];
            best_i = i;
        }
    }

    // Best two-leg route
    int best_q = -1;
    float best_total = -std::numeric_limits<float>::infinity();
    float best_d1 = 0.f;
    float best_r2 = 0.f;
    Vec2f best_pivot{Vec2f::Zero()};
    if (leg2_dists)
    {
        for (size_t q = 0; q < this->leg2_queries.size(); q++)
        {
            const auto& lq = this->leg2_queries[q];
            const float d1 = PIVOT_FRACTIONS[lq.k] * this->leg1_ranges[lq.i];
            const Vec2f pivot = this->query_origin + this->headingDir(lq.i) * d1;
            const float r2 = std::max(
                std::min(
                    (*leg2_dists)[q],
                    this->rangeToZoneEdge(pivot, lq.j * STEP_RAD)),
                0.f);
            if (r2 < MIN_LEG_M)
            {
                continue;
            }

            if (d1 + r2 > best_total)
            {
                best_total = d1 + r2;
                best_q = static_cast<int>(q);
                best_d1 = d1;
                best_r2 = r2;
                best_pivot = pivot;
            }
        }
    }

    const bool use_two_leg =
        best_q >= 0 && best_total >= TWO_LEG_MIN_GAIN * best_straight;

    std::cout << "[PolarMiningPlanner] Straight best " << best_straight
              << " m @ " << (best_i * STEP_DEG) << " deg; ";
    if (best_q >= 0)
    {
        const auto& lq = this->leg2_queries[best_q];
        std::cout << "two-leg best " << best_total << " m ("
                  << (lq.i * STEP_DEG) << " deg for " << best_d1 << " m, then "
                  << (lq.j * STEP_DEG) << " deg for " << best_r2 << " m)";
    }
    else
    {
        std::cout << "no two-leg route";
    }
    std::cout << " -> " << (use_two_leg ? "TWO-LEG" : "straight") << "\n";

    const float total = use_two_leg ? best_total : best_straight;
    if (best_i < 0 || total < this->params.auto_mining_min_path_length)
    {
        std::cout << "[PolarMiningPlanner] No route reaches minimum path "
                     "length ("
                  << this->params.auto_mining_min_path_length << " m).\n";
        return std::nullopt;
    }

    Result res;
    res.origin = this->query_origin;
    res.range = total;

    if (use_two_leg)
    {
        const auto& lq = this->leg2_queries[best_q];
        res.dir = this->headingDir(lq.i);
        res.leg1_dist = best_d1;
        res.leg2_dir = this->headingDir(lq.j);

        this->last_sweep.selected = lq.i;
        this->last_sweep.has_leg2 = true;
        this->last_sweep.pivot = best_pivot;
        this->last_sweep.leg2_heading = lq.j * STEP_RAD;
        this->last_sweep.leg2_range = best_r2;
    }
    else
    {
        res.dir = this->headingDir(best_i);
        this->last_sweep.selected = best_i;
    }

    return res;
}

bool PolarMiningPlanner::iteratePivot(
    const Vec2f& leg1_dir,
    std::optional<Result>& out)
{
    switch (this->state)
    {
        case State::IDLE:
        {
            out.reset();
            if (!this->sendLeg1())
            {
                return true;
            }
            this->state = State::WAIT_LEG1;
            [[fallthrough]];
        }
        case State::WAIT_LEG1:
        {
            if (!this->mining_eval.hasResult())
            {
                return false;
            }
            this->state = State::IDLE;
            if (!this->processLeg1())
            {
                return true;
            }
            break;
        }
        default:
        {
            // Not expected (no second-leg query is sent from a pivot)
            this->state = State::IDLE;
            return false;
        }
    }

    constexpr float NO_BACKTRACK_RAD =
        static_cast<float>(NO_BACKTRACK_DEG) * static_cast<float>(M_PI) / 180.f;
    const float back_theta = std::atan2(-leg1_dir.y(), -leg1_dir.x());

    int best_i = -1;
    float best_range = -std::numeric_limits<float>::infinity();
    for (int i = 0; i < N_HEADINGS; i++)
    {
        const float d = this->query_headings[i] - back_theta;
        if (std::abs(std::atan2(std::sin(d), std::cos(d))) <= NO_BACKTRACK_RAD)
        {
            continue;
        }
        if (this->leg1_ranges[i] > best_range)
        {
            best_range = this->leg1_ranges[i];
            best_i = i;
        }
    }

    if (best_i < 0 || best_range < MIN_LEG_M)
    {
        std::cout << "[PolarMiningPlanner] Pivot re-sweep: no second leg of at "
                     "least "
                  << MIN_LEG_M << " m\n";
        return true;
    }

    std::cout << "[PolarMiningPlanner] Pivot re-sweep: second leg "
              << (best_i * STEP_DEG) << " deg for " << best_range << " m\n";

    this->last_sweep.selected = best_i;

    Result res;
    res.origin = this->query_origin;
    res.dir = this->headingDir(best_i);
    res.range = best_range;
    out = res;
    return true;
}

PolarMiningPlanner::Vec2f PolarMiningPlanner::headingDir(int i) const
{
    const float theta = i * STEP_RAD;
    return Vec2f{std::cos(theta), std::sin(theta)};
}

float PolarMiningPlanner::rangeToZoneEdge(const Vec2f& p, float theta) const
{
    // Same boundary distance MiningController uses to stop mining
    // (CONSTRAINT_ZONE_BOUNDARY), so the planned range matches the actual run
    return lance::geom::distToBounds(
               Pose2f{p.x(), p.y(), theta},
               this->params.bounds.mining_zone) -
           lance::geom::TRENCHER_X_MAX_<float>;
}

};  // namespace lance
