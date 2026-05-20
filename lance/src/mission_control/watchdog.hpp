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

#include <chrono>

#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/int32.hpp>
#include <std_srvs/srv/set_bool.hpp>

#include "util/ros_utils.hpp"

#include "robot/core/robot_status.hpp"
#include "robot/core/ros_interface.hpp"


namespace lance
{

#define UPDATE_BITFIELD(val, var, bit) \
    if (val)                      \
    {                             \
        var |= (bit);             \
    }                             \
    else                          \
    {                             \
        var &= ~(bit);            \
    }

class WatchDog : public util::UsingRosAliases
{
    using Int32Msg = std_msgs::msg::Int32;
    using SetBoolSrv = std_srvs::srv::SetBool;

    using SetBoolReqPtr = SetBoolSrv::Request::SharedPtr;
    using SetBoolRespPtr = SetBoolSrv::Response::SharedPtr;

    static constexpr int64_t PUB_DT_MS = 100;
    static constexpr int64_t TELEOP_FEED_TIME_MS = 250;
    static constexpr int64_t AUTO_FEED_TIME_MS = 10000;

public:
    inline WatchDog(RclNode& node) :
        watchdog_status_pub{node.create_publisher<Int32Msg>(
            lance::WATCHDOG_TOPIC,
            rclcpp::SensorDataQoS{})},

        set_teleop_srv{node.create_service<SetBoolSrv>(
            lance::SET_TELEOP_TOPIC,
            [this](SetBoolReqPtr req, SetBoolRespPtr resp)
            {
                this->ctrl_mode = req->data ? ControlMode::TELEOPERATED
                                            : ControlMode::DISABLED;
                resp->success = true;
            })},

        set_auto_srv{node.create_service<SetBoolSrv>(
            lance::SET_AUTO_TOPIC,
            [this](SetBoolReqPtr req, SetBoolRespPtr resp)
            {
                this->ctrl_mode =
                    req->data ? ControlMode::AUTONOMOUS : ControlMode::DISABLED;
                resp->success = true;
            })},

        test_mode_srv{node.create_service<SetBoolSrv>(
            lance::SET_TEST_TOPIC,
            [this](SetBoolReqPtr req, SetBoolRespPtr resp)
            {
                UPDATE_BITFIELD(
                    req->data,
                    this->ctrl_opts,
                    static_cast<uint8_t>(ControlOpts::TEST_MODE))
                resp->success = true;
            })},
        quick_auto_srv{node.create_service<SetBoolSrv>(
            lance::SET_QUICK_AUTO_TOPIC,
            [this](SetBoolReqPtr req, SetBoolRespPtr resp)
            {
                UPDATE_BITFIELD(
                    req->data,
                    this->ctrl_opts,
                    static_cast<uint8_t>(ControlOpts::QUICK_AUTO))
                resp->success = true;
            })},
        assist_auto_srv{node.create_service<SetBoolSrv>(
            lance::SET_ASSIST_AUTO_TOPIC,
            [this](SetBoolReqPtr req, SetBoolRespPtr resp)
            {
                UPDATE_BITFIELD(
                    req->data,
                    this->ctrl_opts,
                    static_cast<uint8_t>(ControlOpts::ASSIST_AS_AUTO))
                resp->success = true;
            })},

        watchdog_timer{node.create_wall_timer(
            std::chrono::milliseconds(PUB_DT_MS),
            [this]()
            {
                this->watchdog_status_pub->publish(
                    Int32Msg{}.set__data(this->getFeedTime()));
            })}
    {
    }

public:
    inline ControlMode getCtrl() const { return this->ctrl_mode; }
    inline bool isCtrl(ControlMode c) const { return this->ctrl_mode == c; }
    inline void setCtrl(ControlMode c) { this->ctrl_mode = c; }

    inline uint8_t getOpts() const { return this->ctrl_opts; }
    inline bool hasOpt(uint8_t opt) const { return this->ctrl_opts & opt; }
    inline void setOpt(uint8_t opt) { this->ctrl_opts |= opt; }
    inline bool toggleOpt(uint8_t opt)
    {
        return (this->ctrl_opts ^= opt) & opt;
    }
    inline void clearOpt(uint8_t opt) { this->ctrl_opts &= ~opt; }

private:
    inline int32_t getFeedTime() const
    {
        return ControlStatus::format<TELEOP_FEED_TIME_MS, AUTO_FEED_TIME_MS>(
            this->ctrl_mode,
            this->ctrl_opts);
    }

private:
    RclPubPtr<Int32Msg> watchdog_status_pub;
    RclSrvPtr<SetBoolSrv> set_teleop_srv;
    RclSrvPtr<SetBoolSrv> set_auto_srv;
    RclSrvPtr<SetBoolSrv> test_mode_srv;
    RclSrvPtr<SetBoolSrv> quick_auto_srv;
    RclSrvPtr<SetBoolSrv> assist_auto_srv;
    RclTimer::SharedPtr watchdog_timer;

    ControlMode ctrl_mode{ControlMode::DISABLED};
    uint8_t ctrl_opts{0};
};

};  // namespace lance
