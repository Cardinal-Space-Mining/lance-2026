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

#include <rclcpp/rclcpp.hpp>

#include <std_srvs/srv/set_bool.hpp>

#include <cardinal_perception/msg/reflector_hint.hpp>

#include "util/pub_map.hpp"

#include "robot/core/tf_cache.hpp"
#include "robot/core/robot_params.hpp"
#include "robot/core/motor_interface.hpp"


namespace lance
{

class LocalizationController
{
    friend class TelemetrySerializer;
    friend class TelemetryDeserializer;

    using RclNode = rclcpp::Node;
    using SetBoolSrv = std_srvs::srv::SetBool;
    using ReflectorHintMsg = cardinal_perception::msg::ReflectorHint;
    using GenericPubMap = util::GenericPubMap;

    template<typename T>
    using RclSubPtr = typename rclcpp::Subscription<T>::SharedPtr;
    template<typename T>
    using RclClientPtr = typename rclcpp::Client<T>::SharedPtr;

public:
    LocalizationController(
        RclNode&,
        GenericPubMap&,
        const RobotParams&,
        const TfCache&);
    ~LocalizationController() = default;

public:
    void initialize();
    bool isFinished();
    void setCancelled();

    void iterate(
        const RobotMotorStatus& motor_status,
        RobotMotorCommands& commands);

protected:
    enum class Stage
    {
        INITIALIZATION,
        SEARCHING,
        ALIGN_HEADING,
        ADJUST_RANGE,
        FINISHED
    };

protected:
    void setLfdControl(bool enabled);

protected:
    GenericPubMap& pub_map;
    const RobotParams& params;
    const TfCache& tf_cache;

    RclSubPtr<ReflectorHintMsg> hint_sub;
    RclClientPtr<SetBoolSrv> lfd_control_client;

    Stage stage{Stage::FINISHED};
    ReflectorHintMsg::ConstSharedPtr last_hint{nullptr};
};

};  // namespace lance
