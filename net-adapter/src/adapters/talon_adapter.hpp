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

#include <chrono>
#include <string>

#include "base_adapter.hpp"

#include "phoenix_ros_driver/msg/talon_ctrl.hpp"
#include "phoenix_ros_driver/msg/talon_info.hpp"
#include "phoenix_ros_driver/msg/talon_faults.hpp"


class TalonAdapterSubState
{
    friend class TalonCtrlAdapter;
    friend class TalonInfoAdapter;
    friend class TalonFaultsAdapter;

    using system_clock = std::chrono::system_clock;
    using system_time = system_clock::time_point;

public:
    TalonAdapterSubState(rclcpp::Node& n, float max_pub_freq = 100.f);

protected:
    bool freqFilterStatus();

protected:
    float max_pub_freq{100.f};
    system_time prev_msg_time{};
};


class TalonCtrlAdapter :
    public BaseAdapter<
        phoenix_ros_driver::msg::TalonCtrl,
        TalonCtrlAdapter,
        void,
        TalonAdapterSubState>
{
    friend BaseT;

public:
    TalonCtrlAdapter(rclcpp::Node& node, const std::string& motor_name);

protected:
    static bool serializeMsg(ByteBuffer&, const MsgT&, SubStateT&);
    static bool deserializeMsg(MsgT&, const ByteBuffer&, PubStateT&);

private:
    std::string topic_name;
};


class TalonInfoAdapter :
    public BaseAdapter<
        phoenix_ros_driver::msg::TalonInfo,
        TalonInfoAdapter,
        void,
        TalonAdapterSubState>
{
    friend BaseT;

public:
    TalonInfoAdapter(rclcpp::Node& node, const std::string& motor_name);

protected:
    static bool serializeMsg(ByteBuffer&, const MsgT&, SubStateT&);
    static bool deserializeMsg(MsgT&, const ByteBuffer&, PubStateT&);

private:
    std::string topic_name;
};


class TalonFaultsAdapter :
    public BaseAdapter<
        phoenix_ros_driver::msg::TalonFaults,
        TalonFaultsAdapter,
        void,
        TalonAdapterSubState>
{
    friend BaseT;

public:
    TalonFaultsAdapter(rclcpp::Node& node, const std::string& motor_name);

protected:
    static bool serializeMsg(ByteBuffer&, const MsgT&, SubStateT&);
    static bool deserializeMsg(MsgT&, const ByteBuffer&, PubStateT&);

private:
    std::string topic_name;
};
