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

#include "path_sampler.hpp"

#include <cmath>
#include <numbers>


constexpr inline float alpha(float theta) { return std::cos(theta * 0.5f); }
constexpr inline float beta(float theta) { return std::sin(theta * 0.5f); }
constexpr inline float gamma(float theta) { return std::tan(theta * 0.5f); }
constexpr inline float rho(float theta)
{
    const float a = alpha(theta);
    return (a / (1.f - a));
}
constexpr inline float k_from_rl(float r, float l)
{
    return (l * l) / (std::sqrt(r * r + l * l) + r);
}


namespace util
{

// ---- setters ----------------------------------------------------------------

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

void PathSampler::setParams(size_t smoothing_sweep_pairs)
{
    this->smoothing_sweep_pairs = smoothing_sweep_pairs;
}

// ---- update -----------------------------------------------------------------

// Find the index of the first waypoint that the robot has not yet passed,
// using the same projection test as the external planner but augmented with
// a bend-bisector condition for sharp corners.
//
// For bends sharper than 90° the smoothed arc pulls the robot inside the
// corner, so the raw projection onto the incoming edge never reaches 1 even
// though the robot has clearly passed the junction. The bisector of the bend
// at each waypoint points "forward through the junction" regardless of angle
// and gives a correct advancement condition in all cases.
//
// The robot is at the origin in the path's coordinate frame.
static size_t findTrimStart(const PathSampler::Path2f& path)
{
    using Vec2f = PathSampler::Vec2f;

    for (size_t i = 0; i + 1 < path.size(); i++)
    {
        const Vec2f& A = path[i];
        const Vec2f& B = path[i + 1];
        const Vec2f diff = B - A;
        const float len2 = diff.squaredNorm();
        if (len2 < 1e-8f)
        {
            continue;  // degenerate edge — always advance
        }

        // Primary: standard projection test
        const float proj = diff.dot(-A) / len2;
        if (proj >= 1.f)
        {
            continue;
        }

        // Secondary: bisector test for sharp bends
        if (i + 2 < path.size())
        {
            const Vec2f& C = path[i + 2];
            const Vec2f s1 = diff.normalized();
            const Vec2f s2 = (C - B).normalized();
            const Vec2f bisector_sum = s1 + s2;

            if (bisector_sum.norm() > 0.1f)
            {
                // Normal bend: advance when robot is in forward half-plane of
                // the bisector at B.
                const Vec2f bisector = bisector_sum.normalized();
                if (bisector.dot(-B) >= 0.f)
                {
                    continue;
                }
            }
            else
            {
                // Near-180° U-turn: bisector is degenerate. Advance when the
                // robot has positive projection along the outgoing edge from B.
                if (s2.dot(-B) >= 0.f)
                {
                    continue;
                }
            }
        }
        else
        {
            // Last waypoint: perpendicular-at-B test.
            if (diff.dot(-B) >= 0.f)
            {
                continue;
            }
        }

        return i;
    }
    return path.size() > 0 ? path.size() - 1 : 0;
}

bool PathSampler::update(const Path2f& path)
{
    this->path.clear();
    this->junctions.clear();
    this->segments.clear();

    // Trim any waypoints the robot has already passed before building the
    // smoothed path. This mirrors the external planner's trimming logic and
    // prevents the sampler from targeting already-passed segments on sharp bends.
    const size_t trim_start = findTrimStart(path);
    const Path2f trimmed(path.begin() + trim_start, path.end());

    if (!this->filterAndUpdate(trimmed))
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

// ---- sampling helpers -------------------------------------------------------

void PathSampler::sampleLine(
    StanleySample& out,
    const LineSegment& seg,
    float t) const
{
    const Vec2f diff = seg.end - seg.start;
    const Vec2f closest_pt = seg.start + diff * t;

    out.heading_error = std::atan2(diff.y(), diff.x());
    out.lateral_error =
        std::signbit(closest_pt.y()) ? -closest_pt.norm() : closest_pt.norm();
    out.v_max = this->v_max;
    out.path_curvature = 0.f;
}

void PathSampler::sampleArc(
    StanleySample& out,
    const ArcSegment& seg,
    float theta,
    float theta_tan) const
{
    // Arc point on the path centerline at angle theta.
    const Vec2f pt =
        seg.center +
        Vec2f{std::cos(theta) * seg.radius, std::sin(theta) * seg.radius};

    // Closest point is approximated by projecting onto the tangent line at pt.
    // This is intentionally a linear approximation: the target arc is treated
    // as locally straight, consistent with the Stanley controller's assumption.
    const Vec2f tan_dir = Vec2f{std::cos(theta_tan), std::sin(theta_tan)};
    const Vec2f closest_pt = pt + tan_dir * (tan_dir.dot(-pt));

    out.heading_error = theta_tan;
    out.lateral_error =
        std::signbit(closest_pt.y()) ? -closest_pt.norm() : closest_pt.norm();
    out.v_max = seg.v_max;
    // Signed curvature: positive = CCW (left turn), negative = CW (right turn).
    out.path_curvature = (seg.sweep_angle > 0.f ? 1.f : -1.f) / seg.radius;
}

// ---- sampleStanley ----------------------------------------------------------

void PathSampler::findCurrentSegment(
    size_t& i,
    size_t& adj_seg_i,
    float& offset,
    float& len_to_next,
    bool& exhausted,
    StanleySample& out) const
{
    constexpr float PI2_F = std::numbers::pi_v<float> * 2.f;
    constexpr float PIH_F = std::numbers::pi_v<float> * 0.5f;

    exhausted = false;

    // Walk forward until we find the first segment that hasn't been fully
    // passed. If offset > 0 the sample point is pushed ahead along the path.
    for (; i < this->segments.size(); i++)
    {
        const auto& var = this->segments[i];

        if (std::holds_alternative<LineSegment>(var))
        {
            const LineSegment& seg = std::get<LineSegment>(var);
            const Vec2f diff = seg.end - seg.start;
            const float t = (diff.dot(-seg.start)) / (seg.length * seg.length);

            if (t >= 1.f)
            {
                continue;  // segment fully behind us
            }

            adj_seg_i = i;
            const float t_off = offset / seg.length;

            if (t + t_off <= 1.f)
            {
                this->sampleLine(out, seg, t + t_off);
                len_to_next = (1.f - t - t_off) * seg.length;
                offset = 0.f;
            }
            else
            {
                offset -= (1.f - t) * seg.length;
            }
            break;
        }
        else
        {
            const ArcSegment& seg = std::get<ArcSegment>(var);
            const float end_angle = seg.end_angle();
            const float lower = std::min(seg.start_angle, end_angle);
            const float higher = std::max(seg.start_angle, end_angle);

            // Resolve robot angle onto the arc's angular range, trying the
            // raw angle first then +/- 2pi aliases. Use closed interval
            // [lower, higher] to avoid missing the arc endpoint.
            float theta = std::atan2(-seg.center.y(), -seg.center.x());
            if (const float tp = theta + PI2_F; lower <= tp && tp <= higher)
            {
                theta = tp;
            }
            else if (const float tm = theta - PI2_F;
                     lower <= tm && tm <= higher)
            {
                theta = tm;
            }
            else if (!(lower <= theta && theta <= higher))
            {
                continue;
            }

            // DOES NOT HANDLE LAST SEGMENT IS ARC
            // (this is usually handled externally)
            const float dist = seg.center.norm();
            if (dist <= 1e-4f)
            {
                continue;
            }

            adj_seg_i = i;

            // offset is scaled by dist (robot-to-center distance) rather than
            // seg.radius so that the lookahead length reflects the arc actually
            // traced by the vehicle at its lateral position, not the centerline.
            const float off_theta = offset / dist;
            const bool ccw = (seg.sweep_angle > 0.f);
            const float theta2 = ccw ? theta + off_theta : theta - off_theta;
            const bool within =
                ccw ? (theta2 < end_angle) : (theta2 > end_angle);

            if (within)
            {
                const float theta_tan = theta2 + (ccw ? PIH_F : -PIH_F);
                this->sampleArc(out, seg, theta2, theta_tan);
                len_to_next = std::abs(end_angle - theta2) * seg.radius;
                offset = 0.f;
            }
            else
            {
                offset = std::abs(theta2 - end_angle) * dist;
            }
            break;
        }
    }

    // If the loop consumed all segments without settling, the robot is past the
    // end of the path. Snap the reference to the last segment endpoint so the
    // controller has a valid, stable target to track while decelerating.
    if (i >= this->segments.size() && !this->segments.empty())
    {
        exhausted = true;
        adj_seg_i = this->segments.size() - 1;
        len_to_next = 0.f;
        offset = 0.f;

        const auto& var = this->segments.back();
        if (std::holds_alternative<LineSegment>(var))
        {
            const LineSegment& seg = std::get<LineSegment>(var);
            this->sampleLine(out, seg, 1.f);
        }
        else
        {
            const ArcSegment& seg = std::get<ArcSegment>(var);
            const bool ccw = (seg.sweep_angle > 0.f);
            const float theta = seg.end_angle();
            const float theta_tan = theta + (ccw ? PIH_F : -PIH_F);
            this->sampleArc(out, seg, theta, theta_tan);
        }
    }
}

void PathSampler::consumeOffsetIntoSegments(
    size_t& i,
    float& offset,
    float& len_to_next,
    StanleySample& out) const
{
    constexpr float PIH_F = std::numbers::pi_v<float> * 0.5f;

    while (offset > 0.f && i + 1 < this->segments.size())
    {
        i++;
        const auto& var = this->segments[i];
        const bool is_last = (i + 1 == this->segments.size());

        if (std::holds_alternative<LineSegment>(var))
        {
            const LineSegment& seg = std::get<LineSegment>(var);
            if (offset < seg.length || is_last)
            {
                const float t = (seg.end - seg.start).dot(-seg.start) /
                                (seg.length * seg.length);
                this->sampleLine(out, seg, t);
                len_to_next = seg.length - offset;
                offset = 0.f;
            }
            else
            {
                offset -= seg.length;
            }
        }
        else
        {
            const ArcSegment& seg = std::get<ArcSegment>(var);
            if (offset < seg.length() || is_last)
            {
                const float theta_off =
                    std::min(offset / seg.radius, std::abs(seg.sweep_angle));
                const bool ccw = (seg.sweep_angle > 0.f);
                const float theta =
                    seg.start_angle + (ccw ? theta_off : -theta_off);
                const float theta_tan = theta + (ccw ? PIH_F : -PIH_F);

                this->sampleArc(out, seg, theta, theta_tan);
                len_to_next =
                    (std::abs(seg.sweep_angle) - theta_off) * seg.radius;
                offset = 0.f;
            }
            else
            {
                offset -= seg.length();
            }
        }
    }
}

void PathSampler::applyVelocityLookahead(
    size_t i,
    float len_to_next,
    StanleySample& out) const
{
    // Scan ahead by the kinematic stopping distance (v^2 / 2a) and tighten
    // out.v_max to respect upcoming arc constraints, accounting for
    // the acceleration budget available before each arc is reached.
    const float search_dist = (this->v_max * this->v_max) / (2.f * this->a_max);
    float total_dist = len_to_next;

    for (i++; total_dist < search_dist && i < this->segments.size(); i++)
    {
        const auto& var = this->segments[i];
        if (std::holds_alternative<LineSegment>(var))
        {
            total_dist += std::get<LineSegment>(var).length;
        }
        else
        {
            const ArcSegment& seg = std::get<ArcSegment>(var);
            out.v_max = std::min(
                out.v_max,
                std::sqrt(
                    seg.v_max * seg.v_max + 2.f * total_dist * this->a_max));
            total_dist += seg.length();
        }
    }

    // If we exhausted all segments before reaching search_dist, clamp for
    // the path end (decelerate to zero).
    if (i >= this->segments.size() && total_dist < search_dist)
    {
        out.v_max =
            std::min(out.v_max, std::sqrt(2.f * total_dist * this->a_max));
    }
}

void PathSampler::populateBehindInfo(StanleySample& out) const
{
    // Path points are in the vehicle frame, so path.front() is the vector from
    // the robot to the first keypoint. The robot-to-keypoint vector is its
    // negation — but since we want bearing and distance TO the keypoint, we
    // use path.front() directly: it already points from robot origin to the pt.
    const Vec2f& kp = this->path.front();
    out.is_behind = true;
    out.dist_to_keypoint = kp.norm();
    out.theta_to_keypoint = std::atan2(kp.y(), kp.x());
}

void PathSampler::sampleStanley(StanleySample& out, float offset) const
{
    constexpr float PI_F = std::numbers::pi_v<float>;
    constexpr float PI2_F = PI_F * 2.f;

    // Early-out when there are no segments (path has fewer than 3 points or
    // update() failed). Write a safe stopped state so the caller doesn't read
    // stale data. is_last_seg=true signals the controller to halt.
    if (this->segments.empty())
    {
        out = StanleySample{};
        out.is_last_seg = true;
        if (!this->path.empty())
        {
            this->populateBehindInfo(out);
        }
        return;
    }

    size_t i = 0, adj_seg_i = 0;
    float len_to_next = 0.f;
    bool exhausted = false;

    this->findCurrentSegment(i, adj_seg_i, offset, len_to_next, exhausted, out);

    // Populate behind-keypoint info when the robot has not yet reached the
    // start of the first segment, indicated by t < 0 on the first line segment
    // (the segment loop will have broken immediately at i=0 without skipping).
    // We detect this by checking whether the first segment start is still ahead
    // of us: if adj_seg_i is still 0 and the path front is in front of origin.
    if (!exhausted && !this->path.empty())
    {
        const auto& var = this->segments.front();
        if (std::holds_alternative<LineSegment>(var))
        {
            const LineSegment& seg = std::get<LineSegment>(var);
            const Vec2f diff = seg.end - seg.start;
            const float t = (diff.dot(-seg.start)) / (seg.length * seg.length);
            if (t < 0.f)
            {
                this->populateBehindInfo(out);
            }
        }
        // For an arc-first path, behind-detection is handled externally
        // per the existing contract noted in findCurrentSegment.
    }

    out.is_last_seg =
        (this->path.size() < 2 || adj_seg_i + 1 == this->segments.size() ||
         exhausted);

    this->consumeOffsetIntoSegments(i, offset, len_to_next, out);
    this->applyVelocityLookahead(i, len_to_next, out);

    // Wrap heading error to [-pi, pi]
    while (out.heading_error < -PI_F)
    {
        out.heading_error += PI2_F;
    }
    while (out.heading_error > PI_F)
    {
        out.heading_error -= PI2_F;
    }
}

const PathSampler::Path2f& PathSampler::getPath() const { return this->path; }

// ---- path building ----------------------------------------------------------

bool PathSampler::filterAndUpdate(const Path2f& p)
{
    constexpr float DIST_EPSILON = 1e-6f;
    constexpr float THETA_EPSILON = 1e-4f;
    const float COS_THETA_EPSILON = std::cos(THETA_EPSILON);

    if (p.size() < 2)
    {
        return false;
    }

    this->path.reserve(p.size());
    this->path.push_back(p.front());

    for (size_t i = 1; i < p.size(); i++)
    {
        if ((p[i] - this->path.back()).norm() <= DIST_EPSILON)
        {
            continue;
        }

        const size_t j = this->path.size();
        if (j > 1)
        {
            const Vec2f s_in =
                (this->path.back() - this->path[j - 2]).normalized();
            const Vec2f s_out = (p[i] - this->path.back()).normalized();
            if (std::abs(s_in.dot(s_out)) > COS_THETA_EPSILON)
            {
                // Incoming and outgoing directions are collinear: replace the
                // intermediate waypoint so no degenerate zero-length segment
                // is created.
                this->path.back() = p[i];
            }
            else
            {
                this->path.push_back(p[i]);
            }
        }
        else
        {
            this->path.push_back(p[i]);
        }
    }

    return true;
}

bool PathSampler::updateJunctions()
{
    const size_t n_pts = this->path.size();
    if (n_pts < 3)
    {
        return false;
    }

    const size_t n_segs = n_pts - 1;
    const size_t n_juncs = n_pts - 2;

    this->tmp.segment_lengths.assign(n_segs, 0.f);
    for (size_t i = 0; i < n_segs; i++)
    {
        this->tmp.segment_lengths[i] =
            (this->path[i + 1] - this->path[i]).norm();
    }

    this->tmp.half_tangents.resize(n_juncs);
    this->junctions.resize(n_juncs);
    for (size_t j = 0; j < n_juncs; j++)
    {
        Junction& jn = this->junctions[j];
        float& ht = this->tmp.half_tangents[j];

        const Vec2f s_in = (this->path[j + 1] - this->path[j]).normalized();
        const Vec2f s_out =
            (this->path[j + 2] - this->path[j + 1]).normalized();

        jn.theta = std::acos(s_in.dot(s_out));
        ht = gamma(jn.theta);
        jn.radius = std::min(
            this->k_max * rho(jn.theta),
            this->v_max / this->omega_max);

        if (j == 0)
        {
            jn.radius = std::min(jn.radius, this->tmp.segment_lengths[0] / ht);
        }
        if (j == n_juncs - 1)
        {
            jn.radius =
                std::min(jn.radius, this->tmp.segment_lengths[n_juncs] / ht);
        }
    }

    for (size_t i = 0; i < this->smoothing_sweep_pairs; i++)
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
        jn.tan_off = jn.radius * this->tmp.half_tangents[j];
        jn.v_max = std::min(this->v_max, jn.radius * this->omega_max);
    }

    return true;
}

void PathSampler::pushLine(const Vec2f& start, const Vec2f& end)
{
    auto& seg =
        std::get<LineSegment>(this->segments.emplace_back(LineSegment{}));
    seg.start = start;
    seg.end = end;
    seg.length = (end - start).norm();
}

bool PathSampler::buildSegments()
{
    const size_t n_pts = this->path.size();
    const size_t n_segs = n_pts - 1;
    const size_t n_juncs = n_pts - 2;

    if (this->junctions.size() != n_juncs)
    {
        return false;  // sanity check
    }

    this->segments.reserve(n_segs + n_juncs);

    Vec2f prev_arc_end = this->path.front();
    for (size_t i = 0; i < n_juncs; i++)
    {
        const Vec2f& prev_pt = this->path[i];
        const Vec2f& curr_pt = this->path[i + 1];
        const Vec2f& next_pt = this->path[i + 2];
        const Junction& jn = this->junctions[i];

        if (jn.radius < 1e-7f || jn.theta < 1e-4f)
        {
            if ((curr_pt - prev_arc_end).norm() > 1e-7f)
            {
                pushLine(prev_arc_end, curr_pt);
                prev_arc_end = curr_pt;
            }
        }
        else
        {
            const Vec2f s1_dir = (curr_pt - prev_pt).normalized();
            const Vec2f s2_dir = (next_pt - curr_pt).normalized();
            Vec2f arc_beg = curr_pt - s1_dir * jn.tan_off;

            // Guard against floating-point overshoot when two adjacent arcs
            // fully saturate the segment between them: if arc_beg has crept
            // behind prev_arc_end along s1_dir, snap it forward so no
            // negative-length line segment or arc position drift occurs.
            if (s1_dir.dot(arc_beg - prev_arc_end) < 0.f)
            {
                arc_beg = prev_arc_end;
            }

            if ((arc_beg - prev_arc_end).norm() > 1e-7f)
            {
                pushLine(prev_arc_end, arc_beg);
            }

            auto& arc =
                std::get<ArcSegment>(this->segments.emplace_back(ArcSegment{}));

            // 2d cross product: positive means center is to the left
            const bool center_on_left =
                (s1_dir.x() * s2_dir.y()) > (s1_dir.y() * s2_dir.x());
            const Vec2f to_center_dir = center_on_left
                                            ? Vec2f{-s1_dir.y(), s1_dir.x()}
                                            : Vec2f{s1_dir.y(), -s1_dir.x()};

            arc.center = arc_beg + to_center_dir * jn.radius;
            arc.radius = jn.radius;
            arc.start_angle = std::atan2(
                arc_beg.y() - arc.center.y(),
                arc_beg.x() - arc.center.x());
            arc.sweep_angle = center_on_left ? jn.theta : -jn.theta;
            arc.v_max = jn.v_max;

            prev_arc_end = curr_pt + s2_dir * jn.tan_off;
        }

        if (i + 1 == n_juncs && (next_pt - prev_arc_end).norm() > 1e-7f)
        {
            pushLine(prev_arc_end, next_pt);
        }
    }

    return true;
}

void PathSampler::optimizeJunctions(size_t seg_i)
{
    const size_t j_left = seg_i - 1;
    const size_t j_right = seg_i;
    Junction& jn_l = this->junctions[j_left];
    Junction& jn_r = this->junctions[j_right];
    const float gam_l = this->tmp.half_tangents[j_left];
    const float gam_r = this->tmp.half_tangents[j_right];

    const float budget = this->tmp.segment_lengths[seg_i];
    const float tan_l = jn_l.radius * gam_l;
    const float tan_r = jn_r.radius * gam_r;

    if (tan_l + tan_r <= budget)
    {
        return;
    }

    const float half = budget * 0.5f;

    if (tan_l <= half)
    {
        // Left is under its half; give the remainder of the budget to right
        if (gam_r > 0.f)
        {
            jn_r.radius = std::min(jn_r.radius, (budget - tan_l) / gam_r);
        }
    }
    else if (tan_r <= half)
    {
        // Right is under its half; give the remainder of the budget to left
        if (gam_l > 0.f)
        {
            jn_l.radius = std::min(jn_l.radius, (budget - tan_r) / gam_l);
        }
    }
    else
    {
        // Both exceed their half; split evenly
        if (gam_l > 0.f)
        {
            jn_l.radius = std::min(jn_l.radius, half / gam_l);
        }
        if (gam_r > 0.f)
        {
            jn_r.radius = std::min(jn_r.radius, half / gam_r);
        }
    }
}

};  // namespace util
