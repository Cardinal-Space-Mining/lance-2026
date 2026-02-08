/*******************************************************************************
*   Copyright (C) 2024-2026 Cardinal Space Mining Club                         *
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

#include <vector>

#include <phoenix_ros_driver/msg/talon_ctrl.hpp>
#include <phoenix_ros_driver/msg/talon_info.hpp>
#include <phoenix_ros_driver/msg/talon_faults.hpp>

#include "base_adapter.hpp"


class TalonCtrlAdapter :
    public BaseAdapter<phoenix_ros_driver::msg::TalonCtrl, TalonCtrlAdapter>
{
    friend BaseT;

public:
    TalonCtrlAdapter(rclcpp::Node& node);

protected:
    static bool serializeMsg(ByteBuffer&, const MsgT&, SubStateT&);
    static bool deserializeMsg(MsgT&, const ByteBuffer&, PubStateT&);
};


class TalonInfoAdapter :
    public BaseAdapter<phoenix_ros_driver::msg::TalonInfo, TalonInfoAdapter>
{
    friend BaseT;

public:
    TalonInfoAdapter(rclcpp::Node& node);

protected:
    static bool serializeMsg(ByteBuffer&, const MsgT&, SubStateT&);
    static bool deserializeMsg(MsgT&, const ByteBuffer&, PubStateT&);
};


class TalonFaultsAdapter :
    public BaseAdapter<phoenix_ros_driver::msg::TalonFaults, TalonFaultsAdapter>
{
    friend BaseT;

public:
    TalonFaultsAdapter(rclcpp::Node& node);

protected:
    static bool serializeMsg(ByteBuffer&, const MsgT&, SubStateT&);
    static bool deserializeMsg(MsgT&, const ByteBuffer&, PubStateT&);
};



struct TalonFeedback
{
    struct Subscriber
    {
        Subscriber(
            rclcpp::Node&,
            zenoh::Session&,
            const std::string&,
            const rclcpp::QoS& = rclcpp::SensorDataQoS{});

        TalonInfoAdapter::Subscriber info_sub;
        TalonFaultsAdapter::Subscriber faults_sub;
    };

    struct Publisher
    {
        Publisher(
            rclcpp::Node&,
            zenoh::Session&,
            const std::string&,
            const rclcpp::QoS& = rclcpp::SensorDataQoS{});

        TalonInfoAdapter::Publisher info_pub;
        TalonFaultsAdapter::Publisher faults_pub;
    };

    struct SubscriberGroup
    {
        SubscriberGroup(
            rclcpp::Node&,
            zenoh::Session&,
            const std::vector<std::string>&,
            const rclcpp::QoS& = rclcpp::SensorDataQoS{});

        std::vector<Subscriber> subs;
    };

    struct PublisherGroup
    {
        PublisherGroup(
            rclcpp::Node&,
            zenoh::Session&,
            const std::vector<std::string>&,
            const rclcpp::QoS& = rclcpp::SensorDataQoS{});

        std::vector<Publisher> pubs;
    };
};
