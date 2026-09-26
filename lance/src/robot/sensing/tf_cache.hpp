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
 * @file tf_cache.hpp
 * @brief Thread-safe transform listener and cached transform chain for arena, odom, and robot frames.
 *
 * Wraps tf2_ros::Buffer and TransformListener with pre-composed transforms:
 *   - Key frames: ARENA_FRAME (map), ODOM_FRAME (odom), ROBOT_FRAME (base_link).
 *   - Avoids repetitive lookupTransform overhead by caching active transforms on every control tick.
 *   - Automatically chains: `robot_to_arena = (arena_to_odom)^-1 * (odom_to_robot)^-1`.
 *   - Provides zero-allocation lookups returning Eigen isometry representations.
 */

#include <mutex>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include <tf2_ros/buffer.hpp>
#include <tf2_ros/transform_listener.hpp>

#include <csm_utils/time_cvt.hpp>
#include <csm_utils/ros_utils.hpp>

#include "robot/core/robot_params.hpp"
#include "robot/model/geometry.hpp"


namespace lance
{

/**
 * @enum KeyFrame
 * @brief Discrete enumerated IDs for primary system reference frames.
 */
enum KeyFrame
{
    INVALID_FRAME = 0,
    ARENA_FRAME = 1, ///< World fixed competition arena frame (typically "map").
    ODOM_FRAME = 2,  ///< Continuous wheel-odometry / visual odometry frame ("odom").
    ROBOT_FRAME = 3  ///< Robot base chassis kinematic origin ("base_link" or "robot").
};

/**
 * @enum KeyTf
 * @brief Packed directional transform identifiers: `(from << 2) | to`.
 */
enum KeyTf
{
    INVALID_TF = 0,
    ARENA_TO_ODOM_TF = (ARENA_FRAME << 2 | ODOM_FRAME),
    ARENA_TO_ROBOT_TF = (ARENA_FRAME << 2 | ROBOT_FRAME),
    ODOM_TO_ARENA_TF = (ODOM_FRAME << 2 | ARENA_FRAME),
    ODOM_TO_ROBOT_TF = (ODOM_FRAME << 2 | ROBOT_FRAME),
    ROBOT_TO_ARENA_TF = (ROBOT_FRAME << 2 | ARENA_FRAME),
    ROBOT_TO_ODOM_TF = (ROBOT_FRAME << 2 | ODOM_FRAME)
};

/**
 * @brief Bit-pack source and target KeyFrames into a single KeyTf enum value.
 */
inline constexpr KeyTf composeKeyTf(KeyFrame from, KeyFrame to)
{
    return static_cast<KeyTf>(from << 2 | to);
}


/**
 * @class TfCache
 * @brief Synchronized cache of critical coordinate transforms between world, odom, and robot.
 */
class TfCache : public util::UsingRosAliases
{
public:
    using Tf2Buffer = tf2_ros::Buffer;
    using Tf2Listener = tf2_ros::TransformListener;
    using PoseTf = lance::geom::PoseTf3f;

public:
    const std::string arena_frame_id; ///< Frame string for ARENA_FRAME ("map").
    const std::string odom_frame_id;  ///< Frame string for ODOM_FRAME ("odom").
    const std::string robot_frame_id; ///< Frame string for ROBOT_FRAME ("base_link" / "robot").

public:
    TfCache(RclNode&, const RobotParams&);
    TfCache(
        RclNode&,
        const std::string& arena_frame_id,
        const std::string& odom_frame_id,
        const std::string& robot_frame_id);

public:
    /**
     * @brief Poll TF2 buffer and refresh cached forward and inverse isometry transforms.
     * Called at start of each control cycle.
     */
    void refresh();

    /// @brief Access underlying TF2 buffer.
    Tf2Buffer& getBuffer();
    const Tf2Buffer& getBuffer() const;

    /// @brief Check if valid transform has been received for the specified KeyTf.
    bool hasTf(KeyTf k) const;

    /// @brief Overload accepting either KeyFrame enums or frame name strings.
    template<typename KeyOrStr1, typename KeyOrStr2>
    bool hasTf(KeyOrStr1&& from, KeyOrStr2&& to) const;

    /// @brief Retrieve timestamp (seconds) of the latest update for transform k.
    double getStamp(KeyTf k) const;

    /// @brief Overload retrieving timestamp using frame names or enums.
    template<typename KeyOrStr1, typename KeyOrStr2>
    double getStamp(KeyOrStr1&& from, KeyOrStr2&& to) const;

    /// @brief Retrieve pointer to cached forward PoseTf, or nullptr if unavailable.
    const PoseTf* getTf(KeyTf k) const;

    /// @brief Overload retrieving PoseTf using frame names or enums.
    template<typename KeyOrStr1, typename KeyOrStr2>
    const PoseTf* getTf(KeyOrStr1&& from, KeyOrStr2&& to) const;

    /// @brief Map a frame ID string or enum to KeyFrame.
    template<typename T>
    KeyFrame resolveKeyFrame(T&& val) const;

    /// @brief Map KeyFrame enum to its configured ROS frame string.
    const std::string& getFrameId(KeyFrame f) const;

protected:
    /**
     * @struct TfLink
     * @brief Pair of forward and inverse transforms with timestamp.
     */
    struct TfLink
    {
        PoseTf tf;        ///< Forward transform.
        PoseTf inv_tf;    ///< Precomputed matrix inverse.
        double stamp{-1.};///< ROS epoch timestamp in seconds.
    };

protected:
    Tf2Buffer tf_buffer;       ///< TF2 buffer accumulating transform broadcasts.
    Tf2Listener tf_listener;   ///< Listener thread receiving TF2 messages.

    TfLink arena_to_odom;      ///< Cached arena -> odom link.
    TfLink odom_to_robot;      ///< Cached odom -> robot link.
    TfLink robot_to_arena;     ///< Synthesized robot -> arena global link.

    mutable std::mutex mtx;    ///< Protects concurrent cache access.
};



// --- Template Method Implementations ---

#include <string_view>
#include <type_traits>


template<typename T1, typename T2>
bool TfCache::hasTf(T1&& from, T2&& to) const
{
    return this->hasTf(composeKeyTf(
        this->resolveKeyFrame(std::forward<T1>(from)),
        this->resolveKeyFrame(std::forward<T2>(to))));
}

template<typename T1, typename T2>
double TfCache::getStamp(T1&& from, T2&& to) const
{
    return this->getStamp(composeKeyTf(
        this->resolveKeyFrame(std::forward<T1>(from)),
        this->resolveKeyFrame(std::forward<T2>(to))));
}

template<typename T1, typename T2>
const TfCache::PoseTf* TfCache::getTf(T1&& from, T2&& to) const
{
    return this->getTf(composeKeyTf(
        this->resolveKeyFrame(std::forward<T1>(from)),
        this->resolveKeyFrame(std::forward<T2>(to))));
}

template<typename T>
KeyFrame TfCache::resolveKeyFrame(T&& val) const
{
    if constexpr (std::is_same_v<std::remove_cvref_t<T>, KeyFrame>)
    {
        return val;
    }
    if constexpr (std::is_constructible_v<std::string_view, T>)
    {
        std::string_view tag{val};

        if (tag == this->robot_frame_id)
        {
            return ROBOT_FRAME;
        }
        if (tag == this->odom_frame_id)
        {
            return ODOM_FRAME;
        }
        if (tag == this->arena_frame_id)
        {
            return ARENA_FRAME;
        }
    }
    return INVALID_FRAME;
}

};  // namespace lance
