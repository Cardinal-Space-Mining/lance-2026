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

#include "talon_adapter.hpp"
#include <cstring>
#include <algorithm>

#include "mem_helpers.hpp"

using namespace util;

// ------------------ TalonAdapterSubState ------------------
TalonAdapterSubState::TalonAdapterSubState(rclcpp::Node&, float max_pub_freq) :
    max_pub_freq(max_pub_freq)
{
}

bool TalonAdapterSubState::freqFilterStatus()
{
    const auto t = system_clock::now();
    const auto d = std::chrono::duration_cast<std::chrono::milliseconds>(
        t - prev_msg_time);
    const auto f =
        std::chrono::milliseconds(static_cast<int64_t>(1000.f / max_pub_freq));

    if (d >= f)
    {
        prev_msg_time = t;
        return true;
    }
    return false;
}

// ------------------ TalonCtrlAdapter ------------------
TalonCtrlAdapter::TalonCtrlAdapter(
    rclcpp::Node& node,
    const std::string& motor_name) :
    BaseT{node},
    topic_name("/" + motor_name + "/ctrl")
{
}

bool TalonCtrlAdapter::serializeMsg(
    ByteBuffer& bytes,
    const MsgT& msg,
    SubStateT& state)
{
    if (!state.freqFilterStatus())
    {
        return false;
    }

    bytes.resize(sizeof(uint8_t) + sizeof(float));  // mode + value as float32
    uint8_t* ptr = bytes.data();

    writeAndIncrement(ptr, msg.mode);
    writeAsAndIncrement<float>(ptr, static_cast<float>(msg.value));

    return true;
}

bool TalonCtrlAdapter::deserializeMsg(
    MsgT& msg,
    const ByteBuffer& bytes,
    PubStateT&)
{
    if (bytes.size() < sizeof(uint8_t) + sizeof(float))
    {
        return false;
    }

    const uint8_t* ptr = bytes.data();
    readAndIncrement(ptr, msg.mode);
    readAsAndIncrement<float>(ptr, msg.value);

    return true;
}


// ------------------ TalonInfoAdapter ------------------
TalonInfoAdapter::TalonInfoAdapter(
    rclcpp::Node& node,
    const std::string& motor_name) :
    BaseT{node},
    topic_name("/" + motor_name + "/info")
{
}

bool TalonInfoAdapter::serializeMsg(
    ByteBuffer& bytes,
    const MsgT& msg,
    SubStateT& state)
{
    if (!state.freqFilterStatus())
    {
        return false;
    }

    bytes.resize(
        sizeof(uint32_t) * 2 +  // header: sec + nanosec
        sizeof(float) * 12 +  // position/velocity/acc + temps/voltages/currents
        sizeof(uint8_t) * 4   // motor_state, bridge_mode, control_mode, status
    );

    uint8_t* ptr = bytes.data();

    // header
    writeAndIncrement(ptr, msg.header.stamp.sec);
    writeAndIncrement(ptr, msg.header.stamp.nanosec);

    // position/velocity/acceleration as float32
    writeAsAndIncrement<float>(ptr, static_cast<float>(msg.position));
    writeAsAndIncrement<float>(ptr, static_cast<float>(msg.velocity));
    writeAsAndIncrement<float>(ptr, static_cast<float>(msg.acceleration));

    // device/processor temps, voltages, currents
    writeAndIncrement(ptr, msg.device_temp);
    writeAndIncrement(ptr, msg.processor_temp);
    writeAndIncrement(ptr, msg.bus_voltage);
    writeAndIncrement(ptr, msg.supply_current);

    writeAndIncrement(ptr, msg.output_percent);
    writeAndIncrement(ptr, msg.output_voltage);
    writeAndIncrement(ptr, msg.output_current);

    // motor_state, bridge_mode, control_mode, status
    writeAndIncrement(ptr, msg.motor_state);
    writeAndIncrement(ptr, msg.bridge_mode);
    writeAndIncrement(ptr, msg.control_mode);
    writeAndIncrement(ptr, msg.status);

    return true;
}

bool TalonInfoAdapter::deserializeMsg(
    MsgT& msg,
    const ByteBuffer& bytes,
    PubStateT&)
{
    constexpr size_t EXPECTED_SIZE =
        sizeof(uint32_t) * 2 + sizeof(float) * 12 + sizeof(uint8_t) * 4;

    if (bytes.size() < EXPECTED_SIZE)
    {
        return false;
    }

    const uint8_t* ptr = bytes.data();

    readAndIncrement(ptr, msg.header.stamp.sec);
    readAndIncrement(ptr, msg.header.stamp.nanosec);

    readAsAndIncrement<float>(ptr, msg.position);
    readAsAndIncrement<float>(ptr, msg.velocity);
    readAsAndIncrement<float>(ptr, msg.acceleration);

    readAndIncrement(ptr, msg.device_temp);
    readAndIncrement(ptr, msg.processor_temp);
    readAndIncrement(ptr, msg.bus_voltage);
    readAndIncrement(ptr, msg.supply_current);

    readAndIncrement(ptr, msg.output_percent);
    readAndIncrement(ptr, msg.output_voltage);
    readAndIncrement(ptr, msg.output_current);

    readAndIncrement(ptr, msg.motor_state);
    readAndIncrement(ptr, msg.bridge_mode);
    readAndIncrement(ptr, msg.control_mode);
    readAndIncrement(ptr, msg.status);

    return true;
}


// ------------------ TalonFaultsAdapter ------------------
TalonFaultsAdapter::TalonFaultsAdapter(
    rclcpp::Node& node,
    const std::string& motor_name) :
    BaseT{node},
    topic_name("/" + motor_name + "/faults")
{
}

bool TalonFaultsAdapter::serializeMsg(
    ByteBuffer& bytes,
    const MsgT& msg,
    SubStateT& state)
{
    if (!state.freqFilterStatus())
    {
        return false;
    }

    bytes.resize(
        sizeof(uint32_t) * 2 +  // header: sec + nanosec
        sizeof(uint32_t) * 2 +  // packed faults + sticky_faults
        sizeof(uint32_t) * 2    // packed booleans + sticky booleans
    );

    uint8_t* ptr = bytes.data();

    // header
    writeAndIncrement(ptr, msg.header.stamp.sec);
    writeAndIncrement(ptr, msg.header.stamp.nanosec);

    // faults
    writeAndIncrement(ptr, msg.faults);
    writeAndIncrement(ptr, msg.sticky_faults);

    // pack 12 booleans into a uint32
    uint32_t fault_bits = 0;
    fault_bits |= (msg.hardware_fault << 0);
    fault_bits |= (msg.proc_temp_fault << 1);
    fault_bits |= (msg.device_temp_fault << 2);
    fault_bits |= (msg.undervoltage_fault << 3);
    fault_bits |= (msg.boot_fault << 4);
    fault_bits |= (msg.unliscensed_fault << 5);
    fault_bits |= (msg.bridge_brownout_fault << 6);
    fault_bits |= (msg.overvoltage_fault << 7);
    fault_bits |= (msg.unstable_voltage_fault << 8);
    fault_bits |= (msg.stator_current_limit_fault << 9);
    fault_bits |= (msg.supply_current_limit_fault << 10);
    fault_bits |= (msg.static_brake_disabled_fault << 11);
    writeAndIncrement(ptr, fault_bits);

    // pack sticky booleans into a uint32
    uint32_t sticky_bits = 0;
    sticky_bits |= (msg.sticky_hardware_fault << 0);
    sticky_bits |= (msg.sticky_proc_temp_fault << 1);
    sticky_bits |= (msg.sticky_device_temp_fault << 2);
    sticky_bits |= (msg.sticky_undervoltage_fault << 3);
    sticky_bits |= (msg.sticky_boot_fault << 4);
    sticky_bits |= (msg.sticky_unliscensed_fault << 5);
    sticky_bits |= (msg.sticky_bridge_brownout_fault << 6);
    sticky_bits |= (msg.sticky_overvoltage_fault << 7);
    sticky_bits |= (msg.sticky_unstable_voltage_fault << 8);
    sticky_bits |= (msg.sticky_stator_current_limit_fault << 9);
    sticky_bits |= (msg.sticky_supply_current_limit_fault << 10);
    sticky_bits |= (msg.sticky_static_brake_disabled_fault << 11);
    writeAndIncrement(ptr, sticky_bits);

    return true;
}

bool TalonFaultsAdapter::deserializeMsg(
    MsgT& msg,
    const ByteBuffer& bytes,
    PubStateT&)
{
    if (bytes.size() < sizeof(uint32_t) * 6)
    {
        return false;
    }

    const uint8_t* ptr = bytes.data();

    readAndIncrement(ptr, msg.header.stamp.sec);
    readAndIncrement(ptr, msg.header.stamp.nanosec);

    readAndIncrement(ptr, msg.faults);
    readAndIncrement(ptr, msg.sticky_faults);

    uint32_t fault_bits = 0;
    readAndIncrement(ptr, fault_bits);
    msg.hardware_fault = (fault_bits >> 0) & 1;
    msg.proc_temp_fault = (fault_bits >> 1) & 1;
    msg.device_temp_fault = (fault_bits >> 2) & 1;
    msg.undervoltage_fault = (fault_bits >> 3) & 1;
    msg.boot_fault = (fault_bits >> 4) & 1;
    msg.unliscensed_fault = (fault_bits >> 5) & 1;
    msg.bridge_brownout_fault = (fault_bits >> 6) & 1;
    msg.overvoltage_fault = (fault_bits >> 7) & 1;
    msg.unstable_voltage_fault = (fault_bits >> 8) & 1;
    msg.stator_current_limit_fault = (fault_bits >> 9) & 1;
    msg.supply_current_limit_fault = (fault_bits >> 10) & 1;
    msg.static_brake_disabled_fault = (fault_bits >> 11) & 1;

    uint32_t sticky_bits = 0;
    readAndIncrement(ptr, sticky_bits);
    msg.sticky_hardware_fault = (sticky_bits >> 0) & 1;
    msg.sticky_proc_temp_fault = (sticky_bits >> 1) & 1;
    msg.sticky_device_temp_fault = (sticky_bits >> 2) & 1;
    msg.sticky_undervoltage_fault = (sticky_bits >> 3) & 1;
    msg.sticky_boot_fault = (sticky_bits >> 4) & 1;
    msg.sticky_unliscensed_fault = (sticky_bits >> 5) & 1;
    msg.sticky_bridge_brownout_fault = (sticky_bits >> 6) & 1;
    msg.sticky_overvoltage_fault = (sticky_bits >> 7) & 1;
    msg.sticky_unstable_voltage_fault = (sticky_bits >> 8) & 1;
    msg.sticky_stator_current_limit_fault = (sticky_bits >> 9) & 1;
    msg.sticky_supply_current_limit_fault = (sticky_bits >> 10) & 1;
    msg.sticky_static_brake_disabled_fault = (sticky_bits >> 11) & 1;

    return true;
}
