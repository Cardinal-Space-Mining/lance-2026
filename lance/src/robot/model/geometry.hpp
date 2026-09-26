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

/**
 * @file geometry.hpp
 * @brief Robot bounding envelopes, 2D/3D pose representations, and spatial collision queries.
 *
 * Provides:
 *   - Platform physical envelope dimensions for LANCE-1 and LANCE-2:
 *       * Swept turning radius (FOOTPRINT_R_MAX) for turn-in-place clearance.
 *       * Chassis bounding box extents [X_MIN, X_MAX], [Y_MIN, Y_MAX], [Z_MIN, Z_MAX].
 *       * Offload dump footprint projection behind the vehicle.
 *   - Eigen-based geometric type aliases (Pose2, Pose3, Box2, Box3).
 *   - Conversions between planar Euler yaw angles and quaternions.
 *   - Ray-box boundary intersection distance calculations (`distToBounds`).
 *   - Arena zone approach normal determination (`innerZoneNormalDir`).
 */

#include <cmath>
#include <limits>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <csm_utils/geometry.hpp>


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
// =============================================================================
// LANCE-1 Geometric Boundaries & Footprint (meters)
// =============================================================================
CONSTEXPR_VAL_TEMPLATE(FOOTPRINT_R_MAX, 0.790)  ///< Maximum circumscribed turning circle radius.
CONSTEXPR_VAL_TEMPLATE(FOOTPRINT_X_MAX, 0.735)  ///< Forward chassis extent from base_link.
CONSTEXPR_VAL_TEMPLATE(FOOTPRINT_X_MIN, -0.765) ///< Rear chassis extent from base_link.
CONSTEXPR_VAL_TEMPLATE(FOOTPRINT_Y_MAX, 0.369)  ///< Left chassis lateral extent.
CONSTEXPR_VAL_TEMPLATE(FOOTPRINT_Y_MIN, -0.369) ///< Right chassis lateral extent.
CONSTEXPR_VAL_TEMPLATE(COLLISION_Z_MAX, 0.675)  ///< Upper vertical height of robot envelope.
CONSTEXPR_VAL_TEMPLATE(COLLISION_Z_MIN, -0.102) ///< Lower vertical clearance (into trench).

CONSTEXPR_VAL_TEMPLATE(TRACKS_X_MAX, 0.375)     ///< Track front edge x-coordinate.
CONSTEXPR_VAL_TEMPLATE(TRENCHER_X_MAX, 0.557)   ///< Outermost trencher tooth tip reach.

CONSTEXPR_VAL_TEMPLATE(OFFLOAD_FOOTPRINT_OFFSET, -0.7) ///< Offset to offload discharge deposit center.
CONSTEXPR_VAL_TEMPLATE(OFFLOAD_FOOTPRINT_WIDTH, 0.4)   ///< Width of regolith offload deposit.
CONSTEXPR_VAL_TEMPLATE(OFFLOAD_FOOTPRINT_LENGTH, 0.3)  ///< Length of regolith offload deposit.

#elif LANCE >= 2
// =============================================================================
// LANCE-2 Geometric Boundaries & Footprint (meters)
// =============================================================================
CONSTEXPR_VAL_TEMPLATE(FOOTPRINT_R_MAX, 0.695)  ///< Maximum circumscribed turning circle radius.
CONSTEXPR_VAL_TEMPLATE(FOOTPRINT_X_MAX, 0.591)  ///< Forward chassis extent from base_link.
CONSTEXPR_VAL_TEMPLATE(FOOTPRINT_X_MIN, -0.490) ///< Rear chassis extent from base_link (-0.640 upper section).
CONSTEXPR_VAL_TEMPLATE(FOOTPRINT_Y_MAX, 0.362)  ///< Left chassis lateral extent.
CONSTEXPR_VAL_TEMPLATE(FOOTPRINT_Y_MIN, -0.362) ///< Right chassis lateral extent.
CONSTEXPR_VAL_TEMPLATE(COLLISION_Z_MAX, 0.810)  ///< Upper vertical height of robot envelope.
CONSTEXPR_VAL_TEMPLATE(COLLISION_Z_MIN, -0.102) ///< Lower vertical clearance (into trench).

CONSTEXPR_VAL_TEMPLATE(TRACKS_X_MAX, 0.466)     ///< Track front edge x-coordinate.
CONSTEXPR_VAL_TEMPLATE(TRENCHER_X_MAX, 0.590)   ///< Outermost trencher tooth tip reach.

CONSTEXPR_VAL_TEMPLATE(OFFLOAD_FOOTPRINT_OFFSET, -0.6) ///< Offset to offload discharge deposit center.
CONSTEXPR_VAL_TEMPLATE(OFFLOAD_FOOTPRINT_WIDTH, 0.4)   ///< Width of regolith offload deposit.
CONSTEXPR_VAL_TEMPLATE(OFFLOAD_FOOTPRINT_LENGTH, 0.3)  ///< Length of regolith offload deposit.
#endif

/// Primary rectangular collision box origin and extents
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


// =============================================================================
// Geometric Type Aliases
// =============================================================================

template<typename T>
using Vec2 = Eigen::Vector2<T>;
template<typename T>
using Vec3 = Eigen::Vector3<T>;
/// Planar 2D pose packed as [x, y, yaw_radians]
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


// =============================================================================
// Pose & Quaternion Conversion Functions
// =============================================================================

/**
 * @brief Construct an Eigen::Quaternion representing a pure yaw rotation about the Z axis.
 * @param theta Yaw angle in radians.
 * @return Quaternion with w = cos(theta/2), z = sin(theta/2), x = 0, y = 0.
 */
template<typename T>
inline Quat<T> yawToQuat(const T theta)
{
    return Quat<T>{std::cos(theta / 2), 0.f, 0.f, std::sin(theta / 2)};
}

/**
 * @brief Extract planar yaw heading in radians [-pi, pi] from a 3D quaternion.
 *
 * Discards pitch and roll components; robust against zero magnitude degeneracies.
 *
 * @param q 3D orientation quaternion.
 * @return Yaw angle in radians.
 */
template<typename T>
inline T quatToYaw(const Quat<T>& q)
{
    return (q.w() * q.w() + q.z() * q.z()) < static_cast<T>(1e-6)
               ? 0
               : std::remainder(
                     2 * std::atan2(q.z(), q.w()),
                     std::numbers::pi_v<T> * 2);
}

/**
 * @brief Flatten a 3D orientation quaternion into a normalized pure-yaw quaternion (zero roll and pitch).
 */
template<typename T>
inline Quat<T> flattenToYaw(const Quat<T>& q)
{
    const T sq_mag = (q.w() * q.w() + q.z() * q.z());
    return sq_mag < static_cast<T>(1e-6)
               ? Quat<T>::Identity()
               : Quat<T>{q.w(), 0, 0, q.z()}.normalized();
}

/**
 * @brief Project a full 3D spatial pose (vec3 + quat) onto a 2D planar pose [x, y, yaw].
 */
template<typename T>
inline Pose2<T> flattenPose(const Pose3<T>& p)
{
    return Pose2<T>{p.vec.x(), p.vec.y(), quatToYaw(p.quat)};
}

/**
 * @brief Expand a 2D planar pose [x, y, yaw] into a 3D pose with specified elevation z.
 */
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


// =============================================================================
// Ray-Casting & Spatial Boundary Queries
// =============================================================================

/**
 * @brief Calculate distance along the robot's heading ray from pose origin to the nearest boundary of an AABB.
 *
 * If the pose origin is inside box `b`, casts a forward ray along heading angle theta = p.z():
 *   ray(t) = [p.x + t * cos(theta), p.y + t * sin(theta)]
 * Finds the intersection distance t > 0 with the bounding box walls.
 * Returns std::numeric_limits<T>::max() if the origin is outside the box.
 *
 * @param p 2D Pose [x, y, theta_yaw].
 * @param b 2D Axis-aligned bounding box.
 * @return Distance in meters to the nearest perimeter wall along heading.
 */
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

/**
 * @brief Overload computing boundary distance from a 3D Pose3.
 */
template<typename T>
inline T distToBounds(const Pose3<T>& p, const Box2<T>& b)
{
    return distToBounds(flattenPose(p), b);
}

/**
 * @brief Overload computing boundary distance from a PoseTf3.
 */
template<typename T>
inline T distToBounds(const PoseTf3<T>& p, const Box2<T>& b)
{
    return distToBounds(p.pose, b);
}

/**
 * @brief Compute the cardinal normal vector pointing from an inner sub-zone towards the arena center.
 *
 * Used to orient entry into the mining or offload zone from the open arena.
 *
 * @param outer Enclosing arena bounding box.
 * @param inner Sub-zone bounding box.
 * @return 2D unit direction vector [dx, dy] pointing into the active region.
 */
template<typename T>
inline Vec2<T> innerZoneNormalDir(const Box2<T>& outer, const Box2<T>& inner)
{
    const Vec2<T> inner_size = inner.sizes();
    const Vec2<T> center_diff = inner.center() - outer.center();

    if (inner_size.x() > inner_size.y())
    {
        // Normal along +/- Y axis
        if (center_diff.y() > 0)
        {
            return Vec2<T>{0, 1};
        }
        else
        {
            return Vec2<T>{0, -1};
        }
    }
    else
    {
        // Normal along +/- X axis
        if (center_diff.x() > 0)
        {
            return Vec2<T>{1, 0};
        }
        else
        {
            return Vec2<T>{-1, 0};
        }
    }
}

};  // namespace geom

};  // namespace lance

#undef CONSTEXPR_VAL_TEMPLATE
