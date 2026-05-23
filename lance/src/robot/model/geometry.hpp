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

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "util/geometry.hpp"


#ifndef LANCE
    #define LANCE 2
#endif

#define CONSTEXPR_VAL_TEMPLATE(name, val)             \
    template<typename T>                              \
    inline constexpr T name##_ = static_cast<T>(val); \
    inline constexpr double name = name##_<double>;

namespace lance
{

namespace geom
{

#if LANCE <= 1
CONSTEXPR_VAL_TEMPLATE(FOOTPRINT_R_MAX, 0.790)
CONSTEXPR_VAL_TEMPLATE(FOOTPRINT_X_MAX, 0.735)
CONSTEXPR_VAL_TEMPLATE(FOOTPRINT_X_MIN, -0.765)
CONSTEXPR_VAL_TEMPLATE(FOOTPRINT_Y_MAX, 0.369)
CONSTEXPR_VAL_TEMPLATE(FOOTPRINT_Y_MIN, -0.369)
CONSTEXPR_VAL_TEMPLATE(COLLISION_Z_MAX, 0.675)
CONSTEXPR_VAL_TEMPLATE(COLLISION_Z_MIN, -0.102)

CONSTEXPR_VAL_TEMPLATE(TRACKS_X_MAX, 0.375)
CONSTEXPR_VAL_TEMPLATE(TRENCHER_X_MAX, 0.557)

CONSTEXPR_VAL_TEMPLATE(OFFLOAD_FOOTPRINT_OFFSET, -0.7)
CONSTEXPR_VAL_TEMPLATE(OFFLOAD_FOOTPRINT_WIDTH, 0.4)
CONSTEXPR_VAL_TEMPLATE(OFFLOAD_FOOTPRINT_LENGTH, 0.3)
#elif LANCE >= 2
CONSTEXPR_VAL_TEMPLATE(FOOTPRINT_R_MAX, 0.695)
CONSTEXPR_VAL_TEMPLATE(FOOTPRINT_X_MAX, 0.591)
CONSTEXPR_VAL_TEMPLATE(FOOTPRINT_X_MIN, -0.490)  // -0.640 min of upper section
CONSTEXPR_VAL_TEMPLATE(FOOTPRINT_Y_MAX, 0.362)
CONSTEXPR_VAL_TEMPLATE(FOOTPRINT_Y_MIN, -0.362)
CONSTEXPR_VAL_TEMPLATE(COLLISION_Z_MAX, 0.810)
CONSTEXPR_VAL_TEMPLATE(COLLISION_Z_MIN, -0.102)

CONSTEXPR_VAL_TEMPLATE(TRACKS_X_MAX, 0.466)
CONSTEXPR_VAL_TEMPLATE(TRENCHER_X_MAX, 0.590)

CONSTEXPR_VAL_TEMPLATE(OFFLOAD_FOOTPRINT_OFFSET, -0.6)
CONSTEXPR_VAL_TEMPLATE(OFFLOAD_FOOTPRINT_WIDTH, 0.4)
CONSTEXPR_VAL_TEMPLATE(OFFLOAD_FOOTPRINT_LENGTH, 0.3)
#endif

CONSTEXPR_VAL_TEMPLATE(PRIMARY_COLLISION_ZONE_X, FOOTPRINT_X_MIN)
CONSTEXPR_VAL_TEMPLATE(
    PRIMARY_COLLISION_ZONE_Y,
    ((FOOTPRINT_Y_MAX + FOOTPRINT_Y_MIN) / 2))
CONSTEXPR_VAL_TEMPLATE(
    PRIMARY_COLLISION_ZONE_Z,
    ((COLLISION_Z_MAX + COLLISION_Z_MIN) / 2))

CONSTEXPR_VAL_TEMPLATE(
    PRIMARY_COLLISION_ZONE_LENGTH_OFFSET,
    (FOOTPRINT_X_MAX - FOOTPRINT_X_MIN))
CONSTEXPR_VAL_TEMPLATE(
    PRIMARY_COLLISION_ZONE_WIDTH,
    (FOOTPRINT_Y_MAX - FOOTPRINT_Y_MIN))
CONSTEXPR_VAL_TEMPLATE(
    PRIMARY_COLLISION_ZONE_HEIGHT,
    (COLLISION_Z_MAX - COLLISION_Z_MIN))



template<typename T>
using Vec2 = Eigen::Vector2<T>;
template<typename T>
using Vec3 = Eigen::Vector3<T>;
template<typename T>
using Pose2 = Vec3<T>;
template<typename T>

using Quat = Eigen::Quaternion<T>;
template<typename T>
using Iso3 = Eigen::Transform<T, 3, Eigen::Isometry>;
template<typename T>
using Pose3 = util::geom::Pose3<T>;
template<typename T>
using PoseTf3 = util::geom::PoseTf3<T>;
template<typename T>

using Box2 = Eigen::AlignedBox<T, 2>;
template<typename T>
using Box3 = Eigen::AlignedBox<T, 3>;


using Vec2f = Vec2<float>;
using Vec3f = Vec3<float>;
using Pose2f = Pose2<float>;

using Quatf = Quat<float>;
using Iso3f = Iso3<float>;
using Pose3f = util::geom::Pose3f;
using PoseTf3f = util::geom::PoseTf3f;

using Box2f = Eigen::AlignedBox2f;
using Box3f = Eigen::AlignedBox3f;


template<typename T>
inline Quat<T> yawToQuat(const T theta)
{
    return Quat<T>{std::cos(theta / 2), 0.f, 0.f, std::sin(theta / 2)};
}
template<typename T>
inline T quatToYaw(const Quat<T>& q)
{
    return (q.w() * q.w() + q.z() * q.z()) < static_cast<T>(1e-6)
               ? 0
               : std::remainder(
                     2 * std::atan2(q.z(), q.w()),
                     std::numbers::pi_v<T> * 2);
}
template<typename T>
inline Quat<T> flattenToYaw(const Quat<T>& q)
{
    const T sq_mag = (q.w() * q.w() + q.z() * q.z());
    return sq_mag < static_cast<T>(1e-6)
               ? Quat<T>::Identity()
               : Quat<T>{q.w(), 0, 0, q.z()}.normalized();
}
template<typename T>
inline Pose2<T> flattenPose(const Pose3<T>& p)
{
    return Pose2<T>{p.vec.x(), p.vec.y(), quatToYaw(p.quat)};
}
template<typename T>
inline Pose3<T> expandPose(const Pose2<T>& p, T z = 0)
{
    Pose3<T> p3;
    p3.vec.x() = p.x();
    p3.vec.y() = p.y();
    p3.vec.z() = z;
    p3.quat.w() = std::cos(p.z() / 2);
    p3.quat.x() = 0;
    p3.quat.y() = 0;
    p3.quat.z() = std::sin(p.z() / 2);
    return p3;
}

/* Obtain the raw distance from the pose origin to the nearest bounary. */
template<typename T>
inline T distToBounds(const Pose2<T>& p, const Box2<T>& b)
{
    if (b.contains(p.template head<2>()))
    {
        const T dx = std::cos(p.z());
        const T dy = std::sin(p.z());
        T tx = std::numeric_limits<T>::max();
        T ty = std::numeric_limits<T>::max();

        if (std::abs(dx) > 1e-6)
        {
            tx = ((dx > 0 ? b.max().x() : b.min().x()) - p.x()) / dx;
        }
        if (std::abs(dy) > 1e-6)
        {
            ty = ((dy > 0 ? b.max().y() : b.min().y()) - p.y()) / dy;
        }

        return std::min(tx, ty);
    }
    return std::numeric_limits<T>::max();
}
/* Flattends 3d pose to 2d and applies distToBounds() overload for Pose2. */
template<typename T>
inline T distToBounds(const Pose3<T>& p, const Box2<T>& b)
{
    return distToBounds(flattenPose(p), b);
}
/* Flattends 3d pose to 2d and applies distToBounds() overload for Pose2. */
template<typename T>
inline T distToBounds(const PoseTf3<T>& p, const Box2<T>& b)
{
    return distToBounds(p.pose, b);
}

template<typename T>
inline Vec2<T> innerZoneNormalDir(const Box2<T>& outer, const Box2<T>& inner)
{
    const Vec2<T> inner_size = inner.sizes();
    const Vec2<T> center_diff = inner.center() - outer.center();

    // Does not explicitly handle when x and y are identical
    if (inner_size.x() > inner_size.y())
    {
        // normal will be +/-y
        if (center_diff.y() > 0)
        {
            // inner less positive than outer --> point towards positive
            return Vec2<T>{0, 1};
        }
        else
        {
            // inner more positive than outer --> point towards negative
            return Vec2<T>{0, -1};
        }
    }
    else
    {
        // normal will be +/-x
        if (center_diff.x() > 0)
        {
            // inner less positive than outer --> point towards positive
            return Vec2<T>{1, 0};
        }
        else
        {
            // inner more positive than outer --> point towards negative
            return Vec2<T>{-1, 0};
        }
    }
}


};  // namespace geom

};  // namespace lance

#undef CONSTEXPR_VAL_TEMPLATE
