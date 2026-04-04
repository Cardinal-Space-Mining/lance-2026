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
#include <algorithm>


namespace util
{

/* Kinematics Utilities */
namespace kmx
{

/* Solve the kinematics equation `Vf^2 = Vi^2 + 2ax` for `Vi` such that the target
 * `Vf` is attained within `x` while decelerating at `a` (gives the maximum `Vi`). */
template<typename F>
inline F maxStartVel(F end_vel, F dist, F max_decell)
{
    return std::sqrt((end_vel * end_vel) + (2 * dist * std::abs(max_decell)));
}

/* Compute the distance it will take to decellerate to `Vf = 0` */
template<typename F>
inline F decellDist(F vel, F max_decell)
{
    // x = v^2 / 2a
    return (vel * vel) / (2 * std::abs(max_decell));
}

/* Limit track velocities while maintaining trajectory curvature. */
template<typename F>
inline void
    applyTrackLimits(F Vl_target, F Vr_target, F V_max, F& Vl_out, F& Vr_out)
{
    const F Vl_target_abs = std::abs(Vl_target);
    const F Vr_target_abs = std::abs(Vr_target);
    const F s = std::min(
        {static_cast<F>(1), (V_max / Vl_target_abs), (V_max / Vr_target_abs)});
    Vl_out = Vl_target * s;
    Vr_out = Vr_target * s;
}

/* WARNING: BROKEN.
 * Constrains the target left/right track velocities such that Vmax and Vdelta
 * are within the specified limits, while minimizing the error in trajectory
 * curvature. Previous velocities MUST be within the given limits, otherwise
 * this may create a positive feedback loop. */
template<typename F>
inline void applyTrackLimits(
    F Vl_target,
    F Vr_target,
    F Vl_prev,
    F Vr_prev,
    F V_max,
    F Vd_max,
    F& Vl_out,
    F& Vr_out)
{
    const F Vl_min = std::clamp(Vl_prev - Vd_max, -V_max, V_max);
    const F Vl_max = std::clamp(Vl_prev + Vd_max, -V_max, V_max);
    const F Vr_min = std::clamp(Vr_prev - Vd_max, -V_max, V_max);
    const F Vr_max = std::clamp(Vr_prev + Vd_max, -V_max, V_max);

    // 1. If target is already in attainable range, short-circuit.
    if ((Vl_target <= Vl_max && Vl_target >= Vl_min) &&
        (Vr_target <= Vr_max && Vr_target >= Vr_min))
    {
        Vl_out = Vl_target;
        Vr_out = Vr_target;
        return;
    }

    const F Vl_target_abs = std::abs(Vl_target);
    const F Vr_target_abs = std::abs(Vr_target);

    // 2. If targetting no movement, pick edge point that's closest
    if (Vl_target_abs <= 1e-7 && Vr_target_abs <= 1e-7)
    {
        Vl_out = std::clamp(Vl_target, Vl_min, Vl_max);
        Vr_out = std::clamp(Vr_target, Vr_min, Vr_max);
        return;
    }

    // 3. If constant-curvature line intersects range, pick intersection point
    //    that is closest
    {
        F t_min = -std::numeric_limits<F>::max();
        F t_max = std::numeric_limits<F>::max();
        if (Vl_target_abs > 1e-7)
        {
            const F a = (Vl_min / Vl_target);
            const F b = (Vl_max / Vl_target);
            t_min = std::max(t_min, std::min(a, b));
            t_max = std::min(t_max, std::max(a, b));
        }
        else
        {
            // L-component is zero, thus equality line is the R-axis. If range
            // doesn't intersect L=0, then there cannot be an intersection.
            if (Vl_min > 0 || Vl_max < 0)
            {
                goto SKIP_INTERSECT_L;
            }
        }
        if (Vr_target_abs > 1e-7)
        {
            const F a = (Vr_min / Vr_target);
            const F b = (Vr_max / Vr_target);
            t_min = std::max(t_min, std::min(a, b));
            t_max = std::min(t_max, std::max(a, b));
        }
        else
        {
            // R-componenet is zero, thus equality line is the L-axis. If range
            // doesn't intersect R=0, then there cannot be an intersection.
            if (Vr_min > 0 || Vr_max < 0)
            {
                goto SKIP_INTERSECT_L;
            }
        }

        // Pick intersection point that's closest to the target
        if (t_min <= t_max)
        {
            const F t =
                (std::abs(t_min - 1) < std::abs(t_max - 1)) ? t_min : t_max;
            Vl_out = Vl_target * t;
            Vr_out = Vr_target * t;
            return;
        }
    }

SKIP_INTERSECT_L:

    // 4. Use the corner with minimum curvature error
    //    Pick C or R form based on which is most stable across all corners.
    {
        // Clamp to nearest as a default if anything fails...
        Vl_out = std::clamp(Vl_target, Vl_min, Vl_max);
        Vr_out = std::clamp(Vr_target, Vr_min, Vr_max);

        const F Vlr_sum = Vl_target + Vr_target;
        const F Vlr_diff = Vl_target - Vr_target;
        const bool R_ok = std::abs(Vlr_diff) > 1e-7;
        const bool C_ok = std::abs(Vlr_sum) > 1e-7;
        if (!R_ok && !C_ok)
        {
            // No valid target metric at all - use clamped values
            return;
        }

        // Find worst-case (minimum) denominator for each form across all corners
        F C_denorm_min = std::numeric_limits<F>::max();
        F R_denom_min = std::numeric_limits<F>::max();
        for (size_t i = 0; i < 4; i++)
        {
            const F l = (i & 0b01) ? Vl_min : Vl_max;
            const F r = (i & 0b10) ? Vr_min : Vr_max;
            C_denorm_min = std::min(C_denorm_min, std::abs(l + r));
            R_denom_min = std::min(R_denom_min, std::abs(l - r));
        }

        // Pick the single most stable form, respecting validity
        // Then compute target reference value with that form
        const bool use_R = R_ok && (!C_ok || R_denom_min >= C_denorm_min);
        const F target_ref = use_R ? (Vlr_sum / Vlr_diff)   // R_target
                                   : (Vlr_diff / Vlr_sum);  // C_target

        // Evaluate all 4 corners with the chosen form
        F err_min = std::numeric_limits<F>::max();
        for (size_t i = 0; i < 4; i++)
        {
            const F l = (i & 0b01) ? Vl_min : Vl_max;
            const F r = (i & 0b10) ? Vr_min : Vr_max;
            const F denom = use_R ? (l - r) : (l + r);
            const F numer = use_R ? (l + r) : (l - r);

            if (std::abs(denom) <= 1e-7)
            {
                // singular corner for chosen form, skip
                continue;
            }

            const F err = std::abs(numer / denom - target_ref);
            if (err < err_min)
            {
                err_min = err;
                Vl_out = l;
                Vr_out = r;
            }
        }
    }
}

};  // namespace kmx

};  // namespace util
