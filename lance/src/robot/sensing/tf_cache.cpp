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

#include "tf_cache.hpp"

#include <csm_utils/geometry.hpp>
#include <csm_utils/time_cvt.hpp>


using namespace util::geom::cvt::ops;


namespace lance
{

TfCache::TfCache(RclNode& node, const RobotParams& params) :
    arena_frame_id{params.arena_frame_id},
    odom_frame_id{params.odom_frame_id},
    robot_frame_id{params.robot_frame_id},
    tf_buffer{node.get_clock()},
    // LYRICAL DEPRECATION: tf_listener{this->tf_buffer, tf2_ros::TransformListener::RequiredInterfaces{node}}
    tf_listener{this->tf_buffer, &node}
{
}
TfCache::TfCache(
    RclNode& node,
    const std::string& arena_frame_id,
    const std::string& odom_frame_id,
    const std::string& robot_frame_id) :
    arena_frame_id{arena_frame_id},
    odom_frame_id{odom_frame_id},
    robot_frame_id{robot_frame_id},
    tf_buffer{node.get_clock()},
    // LYRICAL DEPRECATION: tf_listener{this->tf_buffer, tf2_ros::TransformListener::RequiredInterfaces{node}}
    tf_listener{this->tf_buffer, &node}
{
}

void TfCache::refresh()
{
    std::unique_lock lock{this->mtx};

    try
    {
        auto tf_msg = this->tf_buffer.lookupTransform(
            this->odom_frame_id,
            this->arena_frame_id,
            tf2::TimePointZero);

        double ts = util::toFloatSeconds(tf_msg.header.stamp);
        if (ts > this->arena_to_odom.stamp)
        {
            this->arena_to_odom.tf.pose << tf_msg.transform;
            this->arena_to_odom.tf.tf << this->arena_to_odom.tf.pose;
            this->arena_to_odom.inv_tf.tf = this->arena_to_odom.tf.tf.inverse();
            this->arena_to_odom.inv_tf.pose << this->arena_to_odom.inv_tf.tf;
            this->arena_to_odom.stamp = ts;
        }
    }
    catch (...)
    {
    }

    try
    {
        auto tf_msg = this->tf_buffer.lookupTransform(
            this->robot_frame_id,
            this->odom_frame_id,
            tf2::TimePointZero);

        double ts = util::toFloatSeconds(tf_msg.header.stamp);
        if (ts > this->odom_to_robot.stamp)
        {
            this->odom_to_robot.tf.pose << tf_msg.transform;
            this->odom_to_robot.tf.tf << this->odom_to_robot.tf.pose;
            this->odom_to_robot.inv_tf.tf = this->odom_to_robot.tf.tf.inverse();
            this->odom_to_robot.inv_tf.pose << this->odom_to_robot.inv_tf.tf;
            this->odom_to_robot.stamp = ts;
        }
    }
    catch (...)
    {
    }

    if ((this->arena_to_odom.stamp >= 0. && this->odom_to_robot.stamp >= 0.) &&
        (this->arena_to_odom.stamp > this->robot_to_arena.stamp ||
         this->odom_to_robot.stamp > this->robot_to_arena.stamp))
    {
        this->robot_to_arena.tf.tf =
            (this->arena_to_odom.inv_tf.tf * this->odom_to_robot.inv_tf.tf);
        this->robot_to_arena.inv_tf.tf =
            (this->odom_to_robot.tf.tf * this->arena_to_odom.tf.tf);
        this->robot_to_arena.tf.pose << this->robot_to_arena.tf.tf;
        this->robot_to_arena.inv_tf.pose << this->robot_to_arena.inv_tf.tf;
        this->robot_to_arena.stamp =
            std::max(this->arena_to_odom.stamp, this->odom_to_robot.stamp);
    }
}

TfCache::Tf2Buffer& TfCache::getBuffer() { return this->tf_buffer; }
const TfCache::Tf2Buffer& TfCache::getBuffer() const { return this->tf_buffer; }

bool TfCache::hasTf(KeyTf k) const { return this->getStamp(k) >= 0.; }

double TfCache::getStamp(KeyTf k) const
{
    std::unique_lock lock{this->mtx};

    switch (k)
    {
        case ARENA_TO_ODOM_TF:
        case ODOM_TO_ARENA_TF:
        {
            return this->arena_to_odom.stamp;
        }
        case ODOM_TO_ROBOT_TF:
        case ROBOT_TO_ODOM_TF:
        {
            return this->odom_to_robot.stamp;
        }
        case ROBOT_TO_ARENA_TF:
        case ARENA_TO_ROBOT_TF:
        {
            return this->robot_to_arena.stamp;
        }
        default:
        {
            return -1.;
        }
    }
}

const TfCache::PoseTf* TfCache::getTf(KeyTf k) const
{
    std::unique_lock lock{this->mtx};

    switch (k)
    {
        case ARENA_TO_ODOM_TF:
        {
            return &this->arena_to_odom.tf;
        }
        case ODOM_TO_ARENA_TF:
        {
            return &this->arena_to_odom.inv_tf;
        }
        case ODOM_TO_ROBOT_TF:
        {
            return &this->odom_to_robot.tf;
        }
        case ROBOT_TO_ODOM_TF:
        {
            return &this->odom_to_robot.inv_tf;
        }
        case ROBOT_TO_ARENA_TF:
        {
            return &this->robot_to_arena.tf;
        }
        case ARENA_TO_ROBOT_TF:
        {
            return &this->robot_to_arena.inv_tf;
        }
        default:
        {
            return nullptr;
        }
    }
}

const std::string& TfCache::getFrameId(KeyFrame f) const
{
    static const std::string EMPTY = "";

    switch (f)
    {
        case ARENA_FRAME:
        {
            return this->arena_frame_id;
        }
        case ODOM_FRAME:
        {
            return this->odom_frame_id;
        }
        case ROBOT_FRAME:
        {
            return this->robot_frame_id;
        }
        default:
        {
            return EMPTY;
        }
    }
}

};  // namespace lance
