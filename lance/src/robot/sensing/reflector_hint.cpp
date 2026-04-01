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

#include "reflector_hint.hpp"

#include "robot/core/ros_interface.hpp"


namespace lance
{

ReflectorHintInterface::ReflectorHintInterface(RclNode& node) :
    hint_sub{node.create_subscription<ReflectorHintMsg>(
        lance::PERCEPTION_REFLECTOR_HINT_TOPIC,
        rclcpp::SensorDataQoS{},
        [this](const ReflectorHintMsg::ConstSharedPtr& msg)
        { this->last_hint = msg; })},
    lfd_control_client{
        node.create_client<SetBoolSrv>(lance::PERCEPTION_LFD_CONTROL_SRV_TOPIC)}
{
}

void ReflectorHintInterface::setEnableSrv(bool enabled)
{
    auto req = std::make_shared<SetBoolSrv::Request>();
    req->data = enabled;
    this->lfd_control_client->async_send_request(
        req,
        [this, enabled](RclClient<SetBoolSrv>::SharedFuture)
        {
            if (!enabled)
            {
                this->last_hint = nullptr;
            }
        });
}

bool ReflectorHintInterface::hasHint() const
{
    return this->last_hint.operator bool();
}
const ReflectorHintInterface::ReflectorHintMsg*
    ReflectorHintInterface::getLatestHint() const
{
    return this->last_hint ? this->last_hint.get() : nullptr;
}

void ReflectorHintInterface::clearHint() { this->last_hint.reset(); }

};  // namespace lance
