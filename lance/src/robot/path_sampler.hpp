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

#include <cmath>
#include <limits>
#include <vector>
#include <numbers>
#include <variant>

#include <Eigen/Dense>


namespace util
{

class PathSampler
{
public:
    using Vec2f = Eigen::Vector2f;
    using Vec3f = Eigen::Vector3f;
    using Path2f = std::vector<Vec2f>;

public:
    struct StanleySample
    {
        float heading_error{0.f};
        float lateral_error{0.f};
        float v_max{0.f};
        bool  is_last_seg{false};

        // Signed path curvature κ = ±1/radius at the matched sample point.
        // Positive = left turn (CCW), negative = right turn (CW), zero on
        // straight segments. Use for curvature feedforward:
        //   W_ff = k_ff * V_forward * path_curvature
        float path_curvature{0.f};

        // Populated when the robot is behind the nearest path keypoint.
        // dist_to_keypoint is the straight-line distance to it;
        // theta_to_keypoint is its bearing angle in the vehicle frame.
        bool  is_behind{false};
        float dist_to_keypoint{0.f};
        float theta_to_keypoint{0.f};
    };

public:
    PathSampler() = default;

public:
    void setConstraints(float k_max, float omega_max, float v_max, float a_max);
    void setParams(size_t smoothing_sweep_pairs);

    bool update(const Path2f& path);

    void sampleStanley(StanleySample& out, float offset = 0.f) const;

    const Path2f& getPath() const;

protected:
    struct Junction
    {
        float theta;
        float radius;
        float tan_off;
        float v_max;
    };
    struct LineSegment
    {
        Vec2f start{Vec2f::Zero()}, end{Vec2f::Zero()};
        float length{0.f};
    };
    struct ArcSegment
    {
        Vec2f center{Vec2f::Zero()};
        float radius{0.f};
        float start_angle{0.f};
        float sweep_angle{0.f};
        float v_max{0.f};

        inline float length() const { return radius * std::abs(sweep_angle); }
        inline float end_angle() const { return start_angle + sweep_angle; }
    };

    using PathJunctions = std::vector<Junction>;
    using PathSegments = std::vector<std::variant<LineSegment, ArcSegment>>;

protected:
    bool filterAndUpdate(const Path2f& path);
    bool updateJunctions();
    bool buildSegments();

    void optimizeJunctions(size_t seg_i);

    // Writes heading/lateral errors into out from a line segment at parameter t.
    void sampleLine(StanleySample& out, const LineSegment& seg, float t) const;

    // Writes heading/lateral errors into out from an arc at the given angle.
    // theta_tan is the precomputed tangent direction angle (theta +/- pi/2).
    void sampleArc(StanleySample& out, const ArcSegment& seg,
                   float theta, float theta_tan) const;

    // Phase 1: walk segments forward to find the one the robot currently sits
    // on, consuming offset along the way. Sets adj_seg_i to the matched index.
    // Sets exhausted=true if all segments were fully passed (robot is past end).
    void findCurrentSegment(
        size_t& i, size_t& adj_seg_i,
        float& offset, float& len_to_next,
        bool& exhausted,
        StanleySample& out) const;

    // Phase 2: forward any remaining offset into subsequent segments after the
    // current one has been identified.
    void consumeOffsetIntoSegments(
        size_t& i, float& offset, float& len_to_next,
        StanleySample& out) const;

    // Phase 3: scan ahead by the kinematic stopping distance and tighten
    // out.v_max to respect upcoming arc constraints.
    void applyVelocityLookahead(
        size_t i, float len_to_next,
        StanleySample& out) const;

    // Populates out.is_behind, out.dist_to_keypoint, and out.theta_to_keypoint
    // using the robot-to-first-keypoint vector when the robot is behind the
    // nearest segment start. The first keypoint is always at the origin in the
    // vehicle frame, so the vector is simply -path.front() (negated because
    // path points are expressed relative to the robot).
    void populateBehindInfo(StanleySample& out) const;

    // Appends a LineSegment to this->segments from start to end.
    void pushLine(const Vec2f& start, const Vec2f& end);

protected:
    Path2f path;
    PathJunctions junctions;
    PathSegments segments;

    struct Tmp
    {
        // Length of each polyline segment between path waypoints
        std::vector<float> segment_lengths;
        // tan(theta/2) for each junction, cached to avoid recomputation
        std::vector<float> half_tangents;
    }  //
    tmp;

    float k_max{0.05f};
    float omega_max{1.f};
    float v_max{0.5f};
    float a_max{1.f};

    // Number of forward+backward smoothing sweep pairs to run
    size_t smoothing_sweep_pairs{5};
};

};  // namespace util
