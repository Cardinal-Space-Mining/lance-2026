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
    using Vec2f = Eigen::Vector2f;
    using Vec3f = Eigen::Vector3f;
    using Path2f = std::vector<Vec2f>;

public:
    struct StanleySample
    {
        float heading_error{0.f};
        float lateral_error{0.f};
        float v_max{0.f};
    };

public:
    PathSampler() = default;

public:
    void setConstraints(float k_max, float omega_max, float v_max, float a_max);
    void setParams(size_t smoothing_passes);

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

        inline float k() const { return k_from_rl(radius, tan_off); }
    };
    struct LineSegment
    {
        Vec2f start{Vec2f::Zero()}, end{Vec2f::Zero()};
        float length{0.f};
        float start_off{0.f};
    };
    struct ArcSegment
    {
        Vec2f center{Vec2f::Zero()};
        float radius{0.f};
        float start_angle{0.f};
        float sweep_angle{0.f};
        float v_max{0.f};
        float start_off{0.f};

        inline float length() const { return radius * sweep_angle; }
        inline float end_angle() const { return start_angle + sweep_angle; }
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

    float k_max{0.05f};
    float omega_max{1.f};
    float v_max{0.5f};
    float a_max{1.f};

    size_t smoothing_passes{5};
};

};  // namespace util
