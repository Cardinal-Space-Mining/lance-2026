#pragma once

#include <cmath>
#include <limits>
#include <vector>
#include <algorithm>
#include <stdexcept>

#include <Eigen/Dense>


/* =============================================================================
 * PathSmoother
 *
 * Given a piecewise-linear path as a sequence of 2D waypoints, computes the
 * optimal smoothing radius at each interior junction for a differential-drive
 * robot subject to:
 *
 *   k_max     - max protrusion of any arc beyond its junction point
 *   omega_max - max angular velocity of the robot (rad/s)
 *   v_max     - max tangential velocity of the robot (m/s)
 *
 * The output radius r_i at each interior point P_i is the largest value that
 * satisfies all constraint families:
 *
 *   1. Per-junction cap:   r_i <= k_max * rho(theta_i)
 *   2. Speed cap:          r_i <= v_max / omega_max
 *   3. Endpoint cap:       l_0  <= |S_0|,   l_{m-1} <= |S_{n-2}|
 *                          (first / last junction's unpaired segment)
 *   4. Non-overlap:        l_i + l_{i+1} <= |S|    for each shared segment
 *
 * Solved via a symmetric forward-backward sweep that distributes segment
 * budget equally between competing junctions, converging in a small fixed
 * number of passes.
 * ========================================================================== */

namespace traj_gen
{


/* Robot constraints */
struct Constraints
{
    float k_max;      // max arc protrusion (metres)
    float omega_max;  // max angular velocity (rad/s)
    float v_max;      // max tangential velocity (m/s)

    // Radius at which bend speed saturates to v_max
    float r_sat() const { return v_max / omega_max; }
};


/* Per-junction result */
struct JunctionResult
{
    float theta;  // turning angle (rad); 0 = straight, pi = U-turn
    float r;      // optimal smoothing radius (metres)
    float l;      // tangent-point offset along each adjacent segment (metres)
    float k;      // arc protrusion from junction point (metres)
    float v;      // achievable bend speed at this junction (m/s)
};


/* Full result: one entry per interior point (indices 1 .. n-2 of the path).
 * junctions[j] corresponds to path[j+1]. */
struct SmoothedPath
{
    std::vector<JunctionResult> junctions;
};


/* Trig helpers  (theta = turning angle; 0 = straight, pi = U-turn) */
namespace detail
{

inline float alpha(float theta) { return std::cos(theta * 0.5f); }
inline float beta(float theta) { return std::sin(theta * 0.5f); }
inline float gamma(float theta) { return std::tan(theta * 0.5f); }
inline float rho(float theta)
{
    const float a = alpha(theta);
    return a / (1.f - a);
}
inline float sigma(float theta)
{
    const float a = alpha(theta);
    return beta(theta) / (1.f - a);
}

/* k from (r, l) - numerically stable form avoids catastrophic cancellation
 * when l << r (shallow bends). Derived from
 * sqrt(r²+l²) - r = l²/(sqrt(r²+l²)+r). */
inline float k_from_rl(float r, float l)
{
    return (l * l) / (std::sqrt(r * r + l * l) + r);
}

}  // namespace detail


/* Path preprocessing
 *
 * Removes waypoints that would be no-ops for the smoother:
 *
 *   - Collinear waypoints (theta < straight_threshold): carry no geometric
 *     information and artificially shorten adjacent segments, reducing the
 *     radius the flanking junctions can achieve.
 *
 *   - Near-U-turn waypoints (theta > pi - uturn_threshold): rho -> 0 so
 *     r_ub = 0 regardless of k_max; the robot must stop there anyway.
 *     If these are artifacts of the path generator rather than intentional
 *     hard constraints, removing them lets flanking junctions see the full
 *     combined segment length.
 *
 *   - Zero-length segments (duplicate waypoints): silently dropped, as they
 *     would corrupt theta computation via normalized() returning a zero vector.
 *
 * Both angle thresholds are in radians. */
inline std::vector<Eigen::Vector2f> preprocess(
    const std::vector<Eigen::Vector2f>& path,
    float extremes_threshold = 1e-4f)
{
    if (path.size() < 2)
    {
        return path;
    }

    // First pass: drop exact duplicates so normalized() is always safe
    std::vector<Eigen::Vector2f> deduped = {path[0]};
    for (int i = 1; i < (int)path.size(); ++i)
    {
        if ((path[i] - path[i - 1]).norm() > 0.0f)
        {
            deduped.push_back(path[i]);
        }
    }

    if (deduped.size() < 3)
    {
        return deduped;
    }

    const float cos_extreme_thresh = std::cos(extremes_threshold);

    // Second pass: drop collinear and near-U-turn interior waypoints
    std::vector<Eigen::Vector2f> out = {deduped[0]};
    for (int i = 1; i < (int)deduped.size() - 1; ++i)
    {
        const Eigen::Vector2f s_in = (deduped[i] - deduped[i - 1]).normalized();
        const Eigen::Vector2f s_out =
            (deduped[i + 1] - deduped[i]).normalized();
        const float cos_t = std::clamp(s_in.dot(s_out), -1.0f, 1.0f);

        if (std::abs(cos_t) < cos_extreme_thresh)
        {
            out.push_back(deduped[i]);
        }
    }
    out.push_back(deduped.back());
    return out;
}


/* Main solver */
inline SmoothedPath smooth(
    const std::vector<Eigen::Vector2f>& path,
    const Constraints& constraints,
    float straight_threshold =
        1e-4f,            // junctions with theta < this are skipped
    int sweep_passes = 5  // forward-backward passes (3-5 is plenty)
)
{
    const int n = static_cast<int>(path.size());
    if (n < 3)
    {
        throw std::invalid_argument("Path must have at least 3 points.");
    }

    const int m = n - 2;  // number of interior junctions

    // Pre-compute segment lengths and per-junction geometry
    std::vector<float> seg_len(n - 1);
    for (int i = 0; i < n - 1; ++i)
    {
        seg_len[i] = (path[i + 1] - path[i]).norm();
        if (seg_len[i] == 0.0f)
        {
            throw std::invalid_argument(
                "Duplicate waypoints at index " + std::to_string(i) + " and " +
                std::to_string(i + 1) + " (zero-length segment).");
        }
    }

    const float cos_straight_threshold = std::cos(straight_threshold);
    std::vector<float> theta(m), gam(m), r_ub(m);

    for (int j = 0; j < m; ++j)
    {
        // Junction j <-> path point path[j+1]
        // Incoming unit segment: path[j+1] - path[j]
        // Outgoing unit segment: path[j+2] - path[j+1]
        const Eigen::Vector2f s_in = (path[j + 1] - path[j]).normalized();
        const Eigen::Vector2f s_out = (path[j + 2] - path[j + 1]).normalized();

        const float cos_t = std::clamp(s_in.dot(s_out), -1.0f, 1.0f);
        theta[j] = std::acos(cos_t);

        // theta ~ 0: straight through, no arc needed
        // theta ~ pi: U-turn, r collapses to 0 via rho -> 0.
        // In either case, abs(cos(theta)) -> 1
        if (std::abs(cos_t) > cos_straight_threshold)
        {
            gam[j] = 0.0f;
            r_ub[j] = 0.0f;
            continue;
        }

        gam[j] = detail::gamma(theta[j]);

        // Start with the speed and k_max per-junction caps (constraints 1 & 2)
        r_ub[j] = std::min(
            constraints.k_max * detail::rho(theta[j]),
            constraints.r_sat());

        // --- Endpoint constraint (constraint family 3) ---
        // The first junction's left segment (seg_len[0]) and the last
        // junction's right segment (seg_len[n-2]) are each shared with no
        // other junction, so l_j <= that segment length must hold unilaterally.
        if (j == 0)
        {
            r_ub[j] = std::min(r_ub[j], seg_len[0] / gam[j]);
        }
        if (j == m - 1)
        {
            r_ub[j] = std::min(r_ub[j], seg_len[n - 2] / gam[j]);
        }
    }

    // Initialise radii to their unconstrained upper bounds
    std::vector<float> r(r_ub);

    // Symmetric enforce: when two junctions compete for a shared segment
    // of length L, each is offered half the budget (l_i = l_{i+1} = L/2).
    // If one side is already under half, the remainder is given to the other.
    // This distributes the segment fairly rather than penalising only the
    // downstream junction (which creates alternating spike patterns).
    auto enforce = [&](int j_left, int j_right, int seg_idx)
    {
        const float budget = seg_len[seg_idx];
        const float l_left = r[j_left] * gam[j_left];
        const float l_right = r[j_right] * gam[j_right];

        if (l_left + l_right <= budget)
        {
            return;  // constraint already satisfied
        }

        const float half = budget * 0.5f;

        if (l_left <= half)
        {
            // Left is under its half; give the remainder of the budget to right
            if (gam[j_right] > 0.0f)
            {
                r[j_right] =
                    std::min(r[j_right], (budget - l_left) / gam[j_right]);
            }
        }
        else if (l_right <= half)
        {
            // Right is under its half; give the remainder of the budget to left
            if (gam[j_left] > 0.0f)
            {
                r[j_left] =
                    std::min(r[j_left], (budget - l_right) / gam[j_left]);
            }
        }
        else
        {
            // Both exceed their half; split evenly
            if (gam[j_left] > 0.0f)
            {
                r[j_left] = std::min(r[j_left], half / gam[j_left]);
            }
            if (gam[j_right] > 0.0f)
            {
                r[j_right] = std::min(r[j_right], half / gam[j_right]);
            }
        }
    };

    // Sweep: forward then backward, repeated sweep_passes times.
    // For a chain graph a single pass resolves symmetric problems exactly;
    // a few extra passes handle asymmetric cases.
    for (int pass = 0; pass < sweep_passes; ++pass)
    {
        for (int j = 0; j < m - 1; ++j)  // forward
        {
            enforce(j, j + 1, j + 1);
        }
        for (int j = m - 2; j >= 0; --j)  // backward
        {
            enforce(j, j + 1, j + 1);
        }
    }

    // Build output
    SmoothedPath result;
    result.junctions.resize(m);

    for (int j = 0; j < m; ++j)
    {
        JunctionResult& out = result.junctions[j];
        out.theta = theta[j];
        out.r = r[j];
        out.l = r[j] * gam[j];
        out.k = (theta[j] < straight_threshold)
                    ? 0.0f
                    : detail::k_from_rl(r[j], out.l);
        // Straight-through waypoints (theta ~ 0) have no centripetal
        // constraint - the robot passes through at full speed.
        out.v = (theta[j] < straight_threshold)
                    ? constraints.v_max
                    : std::min(constraints.v_max, r[j] * constraints.omega_max);
    }

    return result;
}



// =============================================================================
// keypoint_planner.hpp
//
// Companion to path_smoother.hpp.
//
// Reconstructs the continuous path geometry from a smoothed waypoint list,
// then produces time-uniform MPC keypoints using an exact analytical velocity
// profile - no intermediate dense grid required.
//
// Pipeline:
//   1. build_segments()   – geometry: straight runs + circular arcs
//   2. plan_keypoints()   – velocity profile + time-uniform sampling (exact)
//
// Velocity profile:
//   Breakpoints are placed at every segment boundary (where v_limit changes)
//   and at the path end (v=0).  A forward then backward pass over these
//   breakpoints produces exact planned speeds.  Between any two consecutive
//   breakpoints the profile is piecewise: accelerate → plateau at v_lim →
//   decelerate, each piece having a closed-form arc-length and time integral.
//   MPC keypoints at uniform dt are found by inverting these closed forms
//   directly - no approximation error.
//
// Coordinate convention:
//   Right-handed, +x forward, +y left (standard robot body frame).
//   heading = 0 → moving along +x, heading = +pi/2 → moving along +y.
//   Curvature κ > 0 for left turns (CCW when viewed from above, +z up).
//   Cross product sign: d_in × d_out > 0 ↔ left turn ↔ κ > 0.
//
// Usage:
//   auto segs = traj_gen::build_segments(path, smooth_result.junctions, constraints.v_max);
//   auto kps  = traj_gen::plan_keypoints(segs, {a_max, v_init, dt});
// =============================================================================


/* A single segment of the reconstructed path. */
struct PathSegment
{
    bool is_arc = false;

    // Straight fields
    Eigen::Vector2f p0, p1;
    float length = 0.f;

    // Arc fields
    Eigen::Vector2f center;
    float radius = 0.f;
    float start_angle = 0.f;  // angle of entry tangent-point from centre (rad)
    float sweep_angle =
        0.f;  // signed span: >0 → left turn (CCW, +z up, right-handed)
    float arc_length = 0.f;  // radius * |sweep_angle|

    // Common
    float v_limit = 0.f;  // max speed on this segment (m/s)
};


/* A single MPC keypoint. */
struct KeyPoint
{
    Eigen::Vector2f position;
    float heading =
        0.f;  // tangent direction (rad); right-handed, +x=0, +y=+pi/2
    float curvature = 0.f;    // signed κ = 1/r (m⁻¹); positive = left turn
    float arc_length = 0.f;   // cumulative path length from start (m)
    float speed_limit = 0.f;  // segment v_limit (m/s)
    float velocity = 0.f;     // planned speed (m/s)
};


/* Planner configuration. */
struct PlannerConfig
{
    float a_max;   // max tangential accel/decel (m/s²)
    float v_init;  // speed at the path start    (m/s)
    float dt;      // MPC timestep               (s)
};

/*  */
inline std::vector<PathSegment> build_segments(
    const std::vector<Eigen::Vector2f>& path,
    const std::vector<JunctionResult>& junctions,
    float v_max)
{
    const int n = static_cast<int>(path.size());
    const int m = n - 2;
    constexpr float ST = 1e-4f;

    struct ArcGeom
    {
        bool valid = false;
        Eigen::Vector2f C, T_in, T_out;
        float r = 0.f, sa = 0.f, sweep = 0.f, arc_len = 0.f, v_lim = 0.f;
    };
    std::vector<ArcGeom> arcs(m);

    for (int j = 0; j < m; ++j)
    {
        const auto& jn = junctions[j];
        if (jn.r < 1e-9f || jn.theta < ST)
        {
            continue;
        }

        const Eigen::Vector2f P = path[j + 1];
        const Eigen::Vector2f d_in = (path[j + 1] - path[j]).normalized();
        const Eigen::Vector2f d_out = (path[j + 2] - path[j + 1]).normalized();

        const Eigen::Vector2f T_in = P - jn.l * d_in;
        const Eigen::Vector2f T_out = P + jn.l * d_out;

        // cross_z > 0 → left turn (CCW) in right-handed frame (+x fwd, +y left)
        const float cross_z = d_in.x() * d_out.y() - d_in.y() * d_out.x();
        const bool left = cross_z > 0.f;

        // Centre is to the left of d_in for a left turn, right for a right turn
        const Eigen::Vector2f normal =
            left ? Eigen::Vector2f(
                       d_in.y(),
                       -d_in.x())  // left of d_in  (+y left convention)
                 : Eigen::Vector2f(-d_in.y(), d_in.x());  // right of d_in

        const Eigen::Vector2f C = T_in + jn.r * normal;
        const float sa = std::atan2(T_in.y() - C.y(), T_in.x() - C.x());
        // sweep > 0 → CCW (left turn) in right-handed frame
        const float sweep = left ? jn.theta : -jn.theta;

        arcs[j] =
            {true, C, T_in, T_out, jn.r, sa, sweep, jn.r * jn.theta, jn.v};
    }

    std::vector<PathSegment> segments;
    Eigen::Vector2f prev = path[0];

    for (int j = 0; j < m; ++j)
    {
        const ArcGeom& arc = arcs[j];
        const Eigen::Vector2f& dst = arc.valid ? arc.T_in : path[j + 1];
        const float d = (dst - prev).norm();
        if (d > 1e-6f)
        {
            PathSegment seg;
            seg.is_arc = false;
            seg.p0 = prev;
            seg.p1 = dst;
            seg.length = d;
            seg.v_limit = v_max;
            segments.push_back(seg);
        }
        if (arc.valid)
        {
            PathSegment seg;
            seg.is_arc = true;
            seg.center = arc.C;
            seg.radius = arc.r;
            seg.start_angle = arc.sa;
            seg.sweep_angle = arc.sweep;
            seg.arc_length = arc.arc_len;
            seg.v_limit = arc.v_lim;
            segments.push_back(seg);
            prev = arc.T_out;
        }
        else
        {
            prev = path[j + 1];
        }
    }
    const float d = (path[n - 1] - prev).norm();
    if (d > 1e-6f)
    {
        PathSegment seg;
        seg.is_arc = false;
        seg.p0 = prev;
        seg.p1 = path[n - 1];
        seg.length = d;
        seg.v_limit = v_max;
        segments.push_back(seg);
    }
    return segments;
}

/* exact analytical velocity profile + time-uniform output */
inline std::vector<KeyPoint> plan_keypoints(
    const std::vector<PathSegment>& segments,
    const PlannerConfig& cfg)
{
    if (segments.empty())
    {
        return {};
    }
    if (cfg.dt <= 0.f)
    {
        throw std::invalid_argument("dt must be positive.");
    }
    if (cfg.a_max <= 0.f)
    {
        throw std::invalid_argument("a_max must be positive.");
    }

    const float a = cfg.a_max;
    const int N = static_cast<int>(segments.size());

    // 1.  Breakpoints: one per segment boundary + path end.
    //     The velocity cap at a boundary is the min of adjacent segment limits,
    //     ensuring the robot satisfies both the exiting and entering constraints
    //     at that point.  The final breakpoint has cap = 0 (must stop).
    std::vector<float> bp_s(N + 1, 0.f);
    std::vector<float> bp_cap(N + 1, 0.f);
    {
        float s = 0.f;
        bp_cap[0] = segments[0].v_limit;
        for (int i = 0; i < N; ++i)
        {
            s += segments[i].is_arc ? segments[i].arc_length
                                    : segments[i].length;
            bp_s[i + 1] = s;
            bp_cap[i + 1] =
                (i < N - 1)
                    ? std::min(segments[i].v_limit, segments[i + 1].v_limit)
                    : 0.f;
        }
    }

    // 2.  Forward pass: max achievable speed at each breakpoint.
    std::vector<float> v_fwd(N + 1);
    v_fwd[0] = std::min(cfg.v_init, bp_cap[0]);
    for (int i = 0; i < N; ++i)
    {
        const float ds = bp_s[i + 1] - bp_s[i];
        v_fwd[i + 1] = std::min(
            bp_cap[i + 1],
            std::sqrt(v_fwd[i] * v_fwd[i] + 2.f * a * ds));
    }

    // 3.  Backward pass: latest speed at each breakpoint to still stop in time.
    std::vector<float> v_bwd(N + 1);
    v_bwd[N] = 0.f;
    for (int i = N - 1; i >= 0; --i)
    {
        const float ds = bp_s[i + 1] - bp_s[i];
        v_bwd[i] = std::min(
            bp_cap[i],
            std::sqrt(v_bwd[i + 1] * v_bwd[i + 1] + 2.f * a * ds));
    }

    // Planned breakpoint speeds
    std::vector<float> v_bp(N + 1);
    for (int i = 0; i <= N; ++i)
    {
        v_bp[i] = std::min(v_fwd[i], v_bwd[i]);
    }

    // 4.  Per-interval zone geometry.
    //
    //     Between breakpoints i and i+1, the exact velocity profile is:
    //       [s0, s_flat_lo]       accel from v0  at rate +a
    //       [s_flat_lo, s_flat_hi] cruise at v_flat (= segment v_limit)
    //       [s_flat_hi, s1]       decel  to v1   at rate -a
    //
    //     If the parabolas from each side intersect below v_limit, there is no
    //     cruise zone (s_flat_lo == s_flat_hi == peak position).
    struct Zones
    {
        float s_flat_lo, s_flat_hi;
        float v_flat;
    };

    auto compute_zones = [&](int i) -> Zones
    {
        const float s0 = bp_s[i], s1 = bp_s[i + 1];
        const float v0 = v_bp[i], v1 = v_bp[i + 1];
        const float v_lim = segments[i].v_limit;
        const float L = s1 - s0;

        // Arc-length of the parabola intersection
        const float s_peak = s0 + (2.f * a * L + v1 * v1 - v0 * v0) / (4.f * a);
        const float v_peak =
            std::sqrt(std::max(0.f, v0 * v0 + 2.f * a * (s_peak - s0)));

        if (v_peak <= v_lim + 1e-6f)
        {
            return {s_peak, s_peak, v_peak};
        }

        const float slo =
            s0 + std::max(0.f, (v_lim * v_lim - v0 * v0) / (2.f * a));
        const float shi =
            s1 - std::max(0.f, (v_lim * v_lim - v1 * v1) / (2.f * a));
        return {std::min(slo, shi), std::max(slo, shi), v_lim};
    };

    struct IData
    {
        Zones z;
        float t_total;
    };
    std::vector<IData> idata(N);

    auto interval_time = [&](int i, const Zones& z) -> float
    {
        constexpr float eps = 1e-9f;
        float t = 0.f;
        if (z.s_flat_lo > bp_s[i] + eps)
        {
            t += (z.v_flat - v_bp[i]) / a;
        }
        if (z.s_flat_hi > z.s_flat_lo + eps)
        {
            t += (z.s_flat_hi - z.s_flat_lo) / z.v_flat;
        }
        if (bp_s[i + 1] > z.s_flat_hi + eps)
        {
            t += (z.v_flat - v_bp[i + 1]) / a;
        }
        return t;
    };

    float T_total = 0.f;
    for (int i = 0; i < N; ++i)
    {
        idata[i].z = compute_zones(i);
        idata[i].t_total = interval_time(i, idata[i].z);
        T_total += idata[i].t_total;
    }

    // 5.  Geometry from segment si at local parameter t ∈ [0,1].
    //     bp_s[i] is already the cumulative arc-length at the start of segment i,
    //     so no second accumulator is needed.
    auto geom_from_seg = [&](int si, float t, float s_abs) -> KeyPoint
    {
        const auto& seg = segments[si];
        KeyPoint kp;
        kp.arc_length = s_abs;
        kp.speed_limit = seg.v_limit;
        if (!seg.is_arc)
        {
            const Eigen::Vector2f dir = (seg.p1 - seg.p0).normalized();
            kp.position = seg.p0 + t * (seg.p1 - seg.p0);
            kp.heading = std::atan2(dir.y(), dir.x());
            kp.curvature = 0.f;
        }
        else
        {
            const float phi = seg.start_angle + t * seg.sweep_angle;
            kp.position =
                seg.center +
                seg.radius * Eigen::Vector2f(std::cos(phi), std::sin(phi));
            kp.heading = seg.sweep_angle > 0.f
                             ? phi + static_cast<float>(M_PI_2)
                             : phi - static_cast<float>(M_PI_2);
            kp.curvature =
                seg.sweep_angle > 0.f ? 1.f / seg.radius : -1.f / seg.radius;
        }
        return kp;
    };

    // 6.  Exact s(t) and v(t) within a breakpoint interval.
    auto s_and_v_in_interval = [&](int i,
                                   float t_into) -> std::pair<float, float>
    {
        constexpr float eps = 1e-9f;
        const Zones& z = idata[i].z;
        const float v0 = v_bp[i];
        const float s0 = bp_s[i];

        const float t_accel =
            (z.s_flat_lo > s0 + eps) ? (z.v_flat - v0) / a : 0.f;
        if (t_into <= t_accel + eps)
        {
            const float t = std::min(t_into, t_accel);
            return {s0 + v0 * t + 0.5f * a * t * t, v0 + a * t};
        }
        float t_rem = t_into - t_accel;

        const float flat_len = z.s_flat_hi - z.s_flat_lo;
        const float t_flat = (flat_len > eps) ? flat_len / z.v_flat : 0.f;
        if (t_rem <= t_flat + eps)
        {
            const float t = std::min(t_rem, t_flat);
            return {z.s_flat_lo + z.v_flat * t, z.v_flat};
        }
        t_rem -= t_flat;

        return {
            z.s_flat_hi + z.v_flat * t_rem - 0.5f * a * t_rem * t_rem,
            std::max(0.f, z.v_flat - a * t_rem)};
    };

    // 7.  Single forward pass: emit one keypoint per dt tick.
    //
    //     Two monotone cursors advance together - neither ever resets:
    //       cur_iv : breakpoint interval == segment index (one per segment)
    //       k      : keypoint index
    //
    //     bp_s[cur_iv] is the segment start arc-length - the same value that
    //     seg_s0 used to duplicate.  One accumulator, one cursor.
    //     Total work is O(N_segments + N_keypoints).
    const int n_steps = static_cast<int>(std::ceil(T_total / cfg.dt));
    std::vector<KeyPoint> out;
    out.reserve(n_steps + 2);

    float t_iv_start = 0.f;  // time at start of cur_iv
    int cur_iv = 0;

    for (int k = 0; k <= n_steps; ++k)
    {
        const float t_target =
            std::min(static_cast<float>(k) * cfg.dt, T_total);

        // Advance cursor (monotone - O(N) total across all k)
        while (cur_iv < N - 1 &&
               t_target > t_iv_start + idata[cur_iv].t_total - 1e-9f)
        {
            t_iv_start += idata[cur_iv].t_total;
            ++cur_iv;
        }

        const float t_into =
            std::clamp(t_target - t_iv_start, 0.f, idata[cur_iv].t_total);
        const auto [s_abs, v] = s_and_v_in_interval(cur_iv, t_into);

        // bp_s[cur_iv] is the segment start - no separate seg_s0 needed
        const float seg_len = segments[cur_iv].is_arc
                                  ? segments[cur_iv].arc_length
                                  : segments[cur_iv].length;
        const float t_param = std::clamp(
            (s_abs - bp_s[cur_iv]) / std::max(seg_len, 1e-9f),
            0.f,
            1.f);

        KeyPoint kp = geom_from_seg(cur_iv, t_param, s_abs);
        kp.velocity = v;
        out.push_back(kp);
    }

    // Exact endpoint (velocity = 0)
    if (out.empty() || out.back().arc_length < bp_s[N] - 1e-6f)
    {
        KeyPoint kp = geom_from_seg(N - 1, 1.f, bp_s[N]);
        kp.velocity = 0.f;
        out.push_back(kp);
    }

    return out;
}

}  // namespace traj_gen




#include <variant>

namespace util
{

class PathSampler
{
    using Vec2f = Eigen::Vector2f;
    using Vec3f = Eigen::Vector3f;
    using Path2f = std::vector<Vec2f>;

public:
    PathSampler() = default;

public:
    void setConstraints(float k_max, float omega_max, float v_max, float a_max);
    void setParams(int smoothing_passes);

    bool update(const Path2f& path);

    const Path2f& getPath() const;

protected:
    struct Junction
    {
        float theta;
        float r;
        float l;
        float v;
    };
    struct LineSegment
    {
        Vec2f start, end;
        float length{0.f};
        float v_max{0.f};
    };
    struct ArcSegment
    {
        Vec2f center;
        float radius{0.f};
        float start_angle{0.f};
        float sweep_angle{0.f};
        float v_max{0.f};

        inline float length() const { return radius * sweep_angle; }
    };

    using PathJunctions = std::vector<Junction>;
    using PathSegments = std::vector<std::variant<LineSegment, ArcSegment>>;

protected:
    static float alpha(float theta);
    static float beta(float theta);
    static float gamma(float theta);
    static float rho(float theta);
    static float sigma(float theta);
    static float k_from_rl(float r, float l);
    float r_sat() const;

    bool filterAndUpdate(const Path2f& path);
    bool updateJunctions();
    bool buildSegments();

    void optimizeJunctions(size_t seg_i);

protected:
    Path2f path;
    PathJunctions junctions;
    PathSegments segments;

    struct Tmp
    {
        std::vector<float> seg_len;
        std::vector<float> gammas;
    }  //
    tmp;

    float k_max;
    float omega_max;
    float v_max;
    float a_max;

    size_t smoothing_passes;
};




void PathSampler::setConstraints(
    float k_max,
    float omega_max,
    float v_max,
    float a_max)
{
    this->k_max = k_max;
    this->omega_max = omega_max;
    this->v_max = v_max;
    this->a_max = a_max;
}
void PathSampler::setParams(size_t smoothing_passes)
{
    this->smoothing_passes = smoothing_passes;
}

bool PathSampler::update(const Path2f& path)
{
    if (!this->filterAndUpdate(path))
    {
        return false;
    }
    if (!this->updateJunctions())
    {
        return false;
    }
    if (!this->buildSegments())
    {
        return false;
    }

    return true;
}


float PathSampler::alpha(float theta) { return std::cos(theta * 0.5f); }
float PathSampler::beta(float theta) { return std::sin(theta * 0.5f); }
float PathSampler::gamma(float theta) { return std::tan(theta * 0.5f); }
float PathSampler::rho(float theta)
{
    const float a = alpha(theta);
    return (a / (1.f - a));
}
float PathSampler::sigma(float theta)
{
    return (beta(theta) / (1.f - alpha(theta)));
}
float PathSampler::k_from_rl(float r, float l)
{
    return (l * l) / (std::sqrt(r * r + l * l) + r);
}
float PathSampler::r_sat() const { return this->v_max / this->omega_max; }


bool PathSampler::filterAndUpdate(const Path2f& p)
{
    constexpr float DIST_EPSILON = 1e-6f;
    constexpr float THETA_EPSILON = 1e-4f;
    constexpr float COS_THETA_EPSILON = std::cos(THETA_EPSILON);

    if (p.size() < 2)
    {
        return false;
    }

    this->path.clear();
    this->path.reserve(p.size());
    this->path.push_back(p.front());

    for (size_t i = 1; i < p.size(); i++)
    {
        if ((p[i] - this->path.back()).norm() > DIST_EPSILON)
        {
            const size_t j = this->path.size();
            if (j > 1)
            {
                const Vec2f s_in =
                    (this->path.back() - this->path(j - 2)).normalized();
                const Vec2f s_out = (p[i] - this->path.back()).normalized();
                if (std::abs(s_in.dot(s_out)) < COS_THETA_EPSILON)
                {
                    this->path.back() = p[i];
                }
            }
            else
            {
                this->path.push_back(p[i]);
            }
        }
    }
}
bool PathSampler::updateJunctions()
{
    const size_t n_pts = this->path.size();
    if (n_pts < 3)
    {
        return false;
    }
    const size_t n_segs = (n_pts - 1);
    const size_t n_juncs = (n_pts - 2);

    this->tmp.seg_len.clear();
    this->tmp.seg_len.resize(n_segs);
    for (size_t i = 0; i < n_segs; i++)
    {
        this->tmp.seg_len[i] = (this->path[i + 1] - this->path[i]).norm();
    }

    this->tmp.gammas.clear();
    this->junctions.clear();
    this->tmp.gammas.resize(n_juncs);
    this->junctions.resize(n_juncs);
    for (size_t j = 0; j < n_juncs; j++)
    {
        Junction& jn = this->junctions[j];
        float& gam = this->tmp.gammas[j];

        const Vec2f s_in = (this->path[j + 1] - this->path[j]).normalized();
        const Vec2f s_out =
            (this->path[j + 2] - this->path[j + 1]).normalized();

        const float cos_theta = s_in.dot(s_out);

        jn.theta = std::acos(cos_theta);
        gam = gamma(jn.theta);
        jn.r = std::min(this->k_max * rho(jn.theta), this->r_sat());

        if (j == 0)
        {
            jn.r = std::min(jn.r, this->tmp.seg_len[0] / gam);
        }
        if (j == n_juncs - 1)
        {
            jn.r = std::min(jn.r, this->tmp.seg_len[n_juncs] / gam);
        }
    }

    for (size_t i = 0; i < this->smoothing_passes; i++)
    {
        for (size_t s = 1; s < n_segs - 1; s++)
        {
            this->optimizeJunctions(s);
        }
        for (size_t s = n_segs - 1; s > 1; s--)
        {
            this->optimizeJunctions(s - 1);
        }
    }

    for (size_t j = 0; j < n_juncs; j++)
    {
        Junction& jn = this->junctions[j];

        jn.l = jn.r * this->tmp.gammas[j];
        jn.v = std::min(this->v_max, jn.r * this->omega_max);
    }
}

bool PathSampler::buildSegments()
{
    const size_t n_pts = this->path.size();
    const size_t n_segs = n_pts - 1;
    const size_t n_juncs = n_pts - 2;

    this->segments.clear();
    this->segments.reserve(n_segs + n_juncs);
    for()
}


void PathSampler::optimizeJunctions(size_t seg_i)
{
    const size_t j_left = seg_i - 1;
    const size_t j_right = seg_i;
    Junction& jn_l = this->junctions[j_left];
    Junction& jn_r = this->junctions[j_right];
    const float gam_l = this->tmp.gammas[j_left];
    const float gam_r = this->tmp.gammas[j_right];

    const float budget = this->tmp.seg_len[seg_i];
    const float l_left = jn_l.r * gam_l;
    const float l_right = jn_r.r * gam_r;

    if (l_left + l_right <= budget)
    {
        return;
    }

    const float half = budget * 0.5f;

    if (l_left <= half)
    {
        // Left is under its half; give the remainder of the budget to right
        if (gam_r > 0.0f)
        {
            jn_r.r = std::min(jn_r.r, (budget - l_left) / gam_r);
        }
    }
    else if (l_right <= half)
    {
        // Right is under its half; give the remainder of the budget to left
        if (gam_l > 0.0f)
        {
            jn_l.r = std::min(jn_l.r, (budget - l_right) / gam_l);
        }
    }
    else
    {
        // Both exceed their half; split evenly
        if (gam_l > 0.0f)
        {
            jn_l.r = std::min(jn_l.r, half / gam_l);
        }
        if (gam_r > 0.0f)
        {
            jn_r.r = std::min(jn_r.r, half / gam_r);
        }
    }
}

};  // namespace util
