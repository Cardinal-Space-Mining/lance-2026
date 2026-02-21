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

#include <geometry_msgs/msg/point_stamped.hpp>

#include "../robot_params.hpp"
#include "../motor_interface.hpp"
#include "../../util/pub_map.hpp"
#include "../../util/joy_utils.hpp"

#include "mining_controller.hpp"
#include "shared_controllers.hpp"
#include "offload_controller.hpp"
#include "traversal_controller.hpp"


class TeleopController
{
    using RclNode = rclcpp::Node;
    using PointStampedMsg = geometry_msgs::msg::PointStamped;
    using JoyState = util::JoyState;
    using GenericPubMap = util::GenericPubMap;

    template<typename T>
    using RclSubPtr = typename rclcpp::Subscription<T>::SharedPtr;

public:
    TeleopController(
        RclNode&,
        GenericPubMap&,
        const RobotParams&,
        SharedControllerCollection&);
    ~TeleopController() = default;

public:
    void initialize();
    void setCancelled();

    void iterate(
        const JoyState& joy,
        const RobotMotorStatus& motor_status,
        RobotMotorCommands& commands);

protected:
    enum class Operation
    {
        MANUAL = 0,
        ASSISTED_MINING,
        ASSISTED_OFFLOAD,
        PRESET_MINING,
        PRESET_OFFLOAD,
        AUTO_TRAVERSAL
    };

protected:
    bool handleGlobalInputs(const JoyState& joy);
    bool handleClickedPoint(bool can_apply);
    void handleTeleopInputs(const JoyState& joy, RobotMotorCommands& commands);
    void publishState();

protected:
    GenericPubMap& pub_map;
    const RobotParams& params;

    RclSubPtr<PointStampedMsg> clicked_point_sub;
    PointStampedMsg::ConstSharedPtr clicked_point;

    Operation op_mode{Operation::MANUAL};
    float driving_rps_scalar;

    MiningController& mining_controller;
    OffloadController& offload_controller;
    TraversalController& traversal_controller;
};
