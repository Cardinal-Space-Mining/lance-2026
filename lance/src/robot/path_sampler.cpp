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


constexpr inline float alpha(float theta) { return std::cos(theta * 0.5f); }
constexpr inline float beta(float theta) { return std::sin(theta * 0.5f); }
constexpr inline float gamma(float theta) { return std::tan(theta * 0.5f); }
constexpr inline float rho(float theta)
{
    const float a = alpha(theta);
    return (a / (1.f - a));
}
constexpr inline float sigma(float theta)
{
    return (beta(theta) / (1.f - alpha(theta)));
}
constexpr inline float k_from_rl(float r, float l)
{
    return (l * l) / (std::sqrt(r * r + l * l) + r);
}


namespace util
{

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
    this->path.clear();
    this->junctions.clear();
    this->segments.clear();

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

void PathSampler::sampleStanley(StanleySample& out, float offset) const
{
    constexpr float PI_F = std::numbers::pi_v<float>;
    constexpr float PI2_F = std::numbers::pi_v<float> * 2.f;
    constexpr float PIH_F = std::numbers::pi_v<float> * 0.5f;

    size_t i = 0, adj_seg_i = 0;
    float len_to_next = 0.f;
    for (; i < this->segments.size(); i++)
    {
        const auto& var = this->segments[i];
        if (std::holds_alternative<LineSegment>(var))
        {
            const LineSegment& seg = std::get<LineSegment>(var);
            const Vec2f diff = seg.end - seg.start;
            const float t = (diff.dot(-seg.start)) / (seg.length * seg.length);

            if (t < 1.f)
            {
                adj_seg_i = i;
                const float t_off = offset / seg.length;
                if (t + t_off <= 1.f)
                {
                    const Vec2f closest_pt = seg.start + diff * t;

                    out.heading_error = std::atan2(diff.y(), diff.x());
                    out.lateral_error = closest_pt.norm();
                    if (std::signbit(closest_pt.y()))
                    {
                        out.lateral_error *= -1.f;
                    }
                    out.v_max = this->v_max;
                    offset = 0.f;
                    len_to_next = (1.f - t - t_off) * seg.length;
                }
                else
                {
                    offset -= (1.f - t) * seg.length;
                }
                break;
            }
        }
        else
        {
            const ArcSegment& seg = std::get<ArcSegment>(var);

            const float end_angle = seg.end_angle();
            const float lower = std::min(seg.start_angle, end_angle);
            const float higher = std::max(seg.start_angle, end_angle);

            float theta = std::atan2(-seg.center.y(), -seg.center.x());
            if (lower <= theta && theta < higher)
            {
            }
            else if (const float theta_p = (theta + PI2_F);
                     (lower <= theta_p && theta_p < higher))
            {
                theta = theta_p;
            }
            else if (const float theta_m = (theta - PI2_F);
                     (lower <= theta_m && theta_m < higher))
            {
                theta = theta_m;
            }
            else
            {
                continue;
            }

            // DOES NOT HANDLE LAST SEGMENT IS RADIUS
            // (this is usually handled externally though)

            const float dist = seg.center.norm();
            if (dist > 1e-4f)
            {
                adj_seg_i = i;
                const float off_theta = offset / dist;
                if (seg.sweep_angle > 0)
                {
                    const float theta2 = theta + off_theta;
                    if (theta2 < end_angle)
                    {
                        const float theta3 = theta2 + PIH_F;
                        const Vec2f pt =
                            seg.center + Vec2f{
                                             std::cos(theta2) * seg.radius,
                                             std::sin(theta2) * seg.radius};
                        const Vec2f tan_dir =
                            Vec2f{std::cos(theta3), std::sin(theta3)};
                        const Vec2f closest_pt =
                            (pt + tan_dir * (tan_dir.dot(-pt)));

                        out.heading_error = theta3;
                        out.lateral_error = closest_pt.norm();
                        if (std::signbit(closest_pt.y()))
                        {
                            out.lateral_error *= -1.f;
                        }
                        out.v_max = seg.v_max;
                        offset = 0.f;
                        len_to_next = (end_angle - theta2) * seg.radius;
                    }
                    else
                    {
                        offset = (theta2 - end_angle) * dist;
                    }
                }
                else
                {
                    const float theta2 = theta - off_theta;
                    if (theta2 > end_angle)
                    {
                        const float theta3 = theta - PIH_F;
                        const Vec2f pt =
                            seg.center + Vec2f{
                                             std::cos(theta2) * seg.radius,
                                             std::sin(theta2) * seg.radius};
                        const Vec2f tan_dir =
                            Vec2f{std::cos(theta3), std::sin(theta3)};
                        const Vec2f closest_pt =
                            (pt + tan_dir * (tan_dir.dot(-pt)));

                        out.heading_error = theta3;
                        out.lateral_error = closest_pt.norm();
                        if (std::signbit(closest_pt.y()))
                        {
                            out.lateral_error *= -1.f;
                        }
                        out.v_max = seg.v_max;
                        offset = 0.f;
                        len_to_next = (theta2 - end_angle) * seg.radius;
                    }
                    else
                    {
                        offset = (end_angle - theta2) * dist;
                    }
                }
            }
            else
            {
                continue;
            }
        }
    }

    out.is_last_seg =
        (this->path.size() < 2 || adj_seg_i + 1 == this->segments.size());

    while (offset > 0.f)
    {
        i++;
        const auto& var = this->segments[i];
        if (std::holds_alternative<LineSegment>(var))
        {
            const LineSegment& seg = std::get<LineSegment>(var);
            if (offset < seg.length || i + 1 == this->segments.size())
            {
                const Vec2f diff = seg.end - seg.start;
                const float t =
                    (diff.dot(-seg.start)) / (seg.length * seg.length);
                const Vec2f closest_pt = seg.start + diff * t;

                out.heading_error = std::atan2(diff.y(), diff.x());
                out.lateral_error = closest_pt.norm();
                if (std::signbit(closest_pt.y()))
                {
                    out.lateral_error *= -1.f;
                }
                out.v_max = this->v_max;
                offset = 0.f;
                len_to_next = seg.length - offset;
            }
            else
            {
                offset -= seg.length;
            }
        }
        else
        {
            const ArcSegment& seg = std::get<ArcSegment>(var);
            if (offset < seg.length() || i + 1 == this->segments.size())
            {
                const float theta_off =
                    std::min(offset / seg.radius, std::abs(seg.sweep_angle));
                if (seg.sweep_angle > 0.f)
                {
                    const float theta = seg.start_angle + theta_off;
                    const float theta2 = theta + PIH_F;
                    const Vec2f pt =
                        seg.center + Vec2f{
                                         std::cos(theta) * seg.radius,
                                         std::sin(theta) * seg.radius};
                    const Vec2f tan_dir =
                        Vec2f{std::cos(theta2), std::sin(theta2)};
                    const Vec2f closest_pt =
                        (pt + tan_dir * (tan_dir.dot(-pt)));

                    out.heading_error = theta2;
                    out.lateral_error = closest_pt.norm();
                    if (std::signbit(closest_pt.y()))
                    {
                        out.lateral_error *= -1.f;
                    }
                    out.v_max = seg.v_max;
                    offset = 0.f;
                    len_to_next = (seg.sweep_angle - theta_off) * seg.radius;
                }
                else
                {
                    const float theta = seg.start_angle - theta_off;
                    const float theta2 = theta - PIH_F;
                    const Vec2f pt =
                        seg.center + Vec2f{
                                         std::cos(theta) * seg.radius,
                                         std::sin(theta) * seg.radius};
                    const Vec2f tan_dir =
                        Vec2f{std::cos(theta2), std::sin(theta2)};
                    const Vec2f closest_pt =
                        (pt + tan_dir * (tan_dir.dot(-pt)));

                    out.heading_error = theta2;
                    out.lateral_error = closest_pt.norm();
                    if (std::signbit(closest_pt.y()))
                    {
                        out.lateral_error *= -1.f;
                    }
                    out.v_max = seg.v_max;
                    offset = 0.f;
                    len_to_next = (theta_off - seg.sweep_angle) * seg.radius;
                }
            }
            else
            {
                offset -= seg.length();
            }
        }
    }

    const float search_dist = (this->v_max * this->v_max) / (2.f * this->a_max);
    float total_dist = len_to_next;
    for (i++; total_dist < search_dist && i < this->segments.size(); i++)
    {
        const auto& var = this->segments[i];
        if (std::holds_alternative<LineSegment>(var))
        {
            const LineSegment& seg = std::get<LineSegment>(var);
            total_dist += seg.length;
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
    if (i >= this->segments.size() && total_dist < search_dist)
    {
        out.v_max =
            std::min(out.v_max, std::sqrt(2.f * total_dist * this->a_max));
    }

    while (out.heading_error < -PI_F)
    {
        out.heading_error += PI2_F;
    }
    while (out.heading_error > PI_F)
    {
        out.heading_error -= PI2_F;
    }
}

const PathSampler::Path2f& PathSampler::getPath() const
{
    return this->path;
}


float PathSampler::Junction::k() const { return k_from_rl(radius, tan_off); }


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
        if ((p[i] - this->path.back()).norm() > DIST_EPSILON)
        {
            const size_t j = this->path.size();
            if (j > 1)
            {
                const Vec2f s_in =
                    (this->path.back() - this->path[j - 2]).normalized();
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

    return true;
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
        jn.radius = std::min(
            this->k_max * rho(jn.theta),
            (this->v_max / this->omega_max));

        if (j == 0)
        {
            jn.radius = std::min(jn.radius, this->tmp.seg_len[0] / gam);
        }
        if (j == n_juncs - 1)
        {
            jn.radius = std::min(jn.radius, this->tmp.seg_len[n_juncs] / gam);
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

        jn.tan_off = jn.radius * this->tmp.gammas[j];
        jn.v_max = std::min(this->v_max, jn.radius * this->omega_max);
    }

    return true;
}

bool PathSampler::buildSegments()
{
    const size_t n_pts = this->path.size();
    const size_t n_segs = n_pts - 1;
    const size_t n_juncs = n_pts - 2;

    // sanity check
    if (this->junctions.size() != n_juncs)
    {
        return false;
    }

    this->segments.reserve(n_segs + n_juncs);

    float start_off = 0.f;
    Vec2f prev_arc_end = this->path.front();
    for (size_t i = 0; i < n_juncs; i++)
    {
        const Vec2f& prev_pt = this->path[i];
        const Vec2f& curr_pt = this->path[i + 1];
        const Vec2f& next_pt = this->path[i + 2];
        const Junction& jn = this->junctions[i];

        if (jn.radius < 1e-7f || jn.theta < 1e-4f)
        {
            // don't add arc segment
            if ((curr_pt - prev_arc_end).norm() > 1e-7f)
            {
                auto& seg = std::get<LineSegment>(
                    this->segments.emplace_back(LineSegment{}));
                seg.start = prev_arc_end;
                seg.end = curr_pt;
                seg.length = (seg.end - seg.start).norm();
                seg.start_off = start_off;

                start_off += seg.length;
                prev_arc_end = curr_pt;
            }
        }
        else
        {
            const Vec2f s1_dir = (curr_pt - prev_pt).normalized();
            const Vec2f s2_dir = (next_pt - curr_pt).normalized();
            const Vec2f arc_beg = curr_pt - (s1_dir * jn.tan_off);

            if ((arc_beg - prev_arc_end).norm() > 1e-7f)
            {
                auto& seg = std::get<LineSegment>(
                    this->segments.emplace_back(LineSegment{}));
                seg.start = prev_arc_end;
                seg.end = arc_beg;
                seg.length = (seg.end - seg.start).norm();
                seg.start_off = start_off;
                start_off += seg.length;
            }

            auto& arc =
                std::get<ArcSegment>(this->segments.emplace_back(ArcSegment{}));

            // 2d cross product > 0
            const bool center_on_left =
                (s1_dir.x() * s2_dir.y()) > (s1_dir.y() * s2_dir.x());
            const Vec2f to_center_dir = center_on_left
                                            ? Vec2f{s1_dir.y(), -s1_dir.x()}
                                            : Vec2f{-s1_dir.y(), s1_dir.x()};

            arc.center = arc_beg + to_center_dir * jn.radius;
            arc.radius = jn.radius;
            arc.start_angle = std::atan2(
                arc_beg.y() - arc.center.y(),
                arc_beg.x() - arc.center.x());
            arc.sweep_angle = center_on_left ? jn.theta : -jn.theta;
            arc.v_max = jn.v_max;
            arc.start_off = start_off;

            start_off += arc.length();
            prev_arc_end = curr_pt + (s2_dir * jn.tan_off);
        }

        if (i + 1 == n_juncs && (next_pt - prev_arc_end).norm() > 1e-7f)
        {
            auto& last_seg = std::get<LineSegment>(
                this->segments.emplace_back(LineSegment{}));
            last_seg.start = prev_arc_end;
            last_seg.end = next_pt;
            last_seg.length = (last_seg.end - last_seg.start).norm();
            last_seg.start_off = start_off;
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
    const float gam_l = this->tmp.gammas[j_left];
    const float gam_r = this->tmp.gammas[j_right];

    const float budget = this->tmp.seg_len[seg_i];
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
        if (gam_r > 0.0f)
        {
            jn_r.radius = std::min(jn_r.radius, (budget - tan_l) / gam_r);
        }
    }
    else if (tan_r <= half)
    {
        // Right is under its half; give the remainder of the budget to left
        if (gam_l > 0.0f)
        {
            jn_l.radius = std::min(jn_l.radius, (budget - tan_r) / gam_l);
        }
    }
    else
    {
        // Both exceed their half; split evenly
        if (gam_l > 0.0f)
        {
            jn_l.radius = std::min(jn_l.radius, half / gam_l);
        }
        if (gam_r > 0.0f)
        {
            jn_r.radius = std::min(jn_r.radius, half / gam_r);
        }
    }
}

};  // namespace util
