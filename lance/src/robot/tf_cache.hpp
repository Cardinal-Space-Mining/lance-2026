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

#include <mutex>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include <tf2_ros/buffer.h>

#include "robot_params.hpp"
#include "../util/geometry.hpp"
#include "../util/time_cvt.hpp"


namespace lance
{

enum KeyFrame
{
    INVALID_FRAME = 0,
    ARENA_FRAME = 1,
    ODOM_FRAME = 2,
    ROBOT_FRAME = 3
};
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


inline constexpr KeyTf composeKeyTf(KeyFrame from, KeyFrame to)
{
    return static_cast<KeyTf>(from << 2 | to);
}


class TfCache
{
    using Tf2Buffer = tf2_ros::Buffer;
    using PoseTf = util::geom::PoseTf3f;

public:
    const std::string arena_frame_id;
    const std::string odom_frame_id;
    const std::string robot_frame_id;

public:
    TfCache(rclcpp::Node&, const RobotParams&);

public:
    void refresh();

    Tf2Buffer& getBuffer();
    const Tf2Buffer& getBuffer() const;

    bool hasTf(KeyTf k) const;
    template<typename KeyOrStr1, typename KeyOrStr2>
    bool hasTf(KeyOrStr1&& from, KeyOrStr2&& to) const;

    double getStamp(KeyTf k) const;
    template<typename KeyOrStr1, typename KeyOrStr2>
    double getStamp(KeyOrStr1&& from, KeyOrStr2&& to) const;

    const PoseTf* getTf(KeyTf k) const;
    template<typename KeyOrStr1, typename KeyOrStr2>
    const PoseTf* getTf(KeyOrStr1&& from, KeyOrStr2&& to) const;

protected:
    struct TfLink
    {
        PoseTf tf;
        PoseTf inv_tf;

        double stamp{-1.};
    };

protected:
    template<typename T>
    KeyFrame resolveKeyFrame(T&& val) const;

protected:
    Tf2Buffer tf_buffer;

    TfLink arena_to_odom;
    TfLink odom_to_robot;
    TfLink robot_to_arena;

    mutable std::mutex mtx;
};



// ---

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
