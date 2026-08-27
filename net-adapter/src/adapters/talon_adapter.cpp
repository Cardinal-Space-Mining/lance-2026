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

#include <csm_utils/mem_helpers.hpp>


using namespace util;


// --- TalonCtrlAdapter --------------------------------------------------------

TalonCtrlAdapter::TalonCtrlAdapter(rclcpp::Node& node) : BaseT{node} {}

constexpr size_t CTRL_PACKED_SIZE =
    (sizeof(uint8_t) + sizeof(float));  // mode + value as float32

bool TalonCtrlAdapter::serializeMsg(
    ByteBuffer& bytes,
    const MsgT& msg,
    SubStateT&)
{
    bytes.resize(CTRL_PACKED_SIZE);
    uint8_t* ptr = bytes.data();

    writeAndIncrement(ptr, msg.mode);
    writeAsAndIncrement<float>(ptr, msg.value);

    return true;
}

bool TalonCtrlAdapter::deserializeMsg(
    MsgT& msg,
    const ByteBuffer& bytes,
    PubStateT&)
{
    if (bytes.size() < CTRL_PACKED_SIZE)
    {
        return false;
    }

    const uint8_t* ptr = bytes.data();
    readAndIncrement(ptr, msg.mode);
    readAsAndIncrement<float>(ptr, msg.value);

    return true;
}



// --- TalonInfoAdapter --------------------------------------------------------

TalonInfoAdapter::TalonInfoAdapter(rclcpp::Node& node) : BaseT{node} {}

constexpr size_t INFO_PACKED_SIZE =
    (sizeof(uint32_t) * 2 +  // header: sec + nanosec
     sizeof(float) * 12 +    // position/velocity/acc + temps/voltages/currents
     sizeof(uint8_t) * 4     // motor_state, bridge_mode, control_mode, status
    );

bool TalonInfoAdapter::serializeMsg(
    ByteBuffer& bytes,
    const MsgT& msg,
    SubStateT&)
{
    bytes.resize(INFO_PACKED_SIZE);

    uint8_t* ptr = bytes.data();

    // header
    writeAndIncrement(ptr, msg.header.stamp.sec);
    writeAndIncrement(ptr, msg.header.stamp.nanosec);

    // position/velocity/acceleration as float32
    writeAsAndIncrement<float>(ptr, msg.position);
    writeAsAndIncrement<float>(ptr, msg.velocity);
    writeAsAndIncrement<float>(ptr, msg.acceleration);

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
    if (bytes.size() < INFO_PACKED_SIZE)
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



// --- TalonFaultsAdapter ------------------------------------------------------

TalonFaultsAdapter::TalonFaultsAdapter(rclcpp::Node& node) : BaseT{node} {}

constexpr size_t FAULTS_PACKED_SIZE =
    (sizeof(uint32_t) * 2 +  // header: sec + nanosec
     sizeof(uint32_t) * 2 +  // packed faults + sticky_faults
     sizeof(uint8_t) * 3     // packed booleans + sticky booleans
    );

bool TalonFaultsAdapter::serializeMsg(
    ByteBuffer& bytes,
    const MsgT& msg,
    SubStateT&)
{
    bytes.resize(FAULTS_PACKED_SIZE);

    uint8_t* ptr = bytes.data();

    // header
    writeAndIncrement(ptr, msg.header.stamp.sec);
    writeAndIncrement(ptr, msg.header.stamp.nanosec);

    // faults
    writeAndIncrement(ptr, msg.faults);
    writeAndIncrement(ptr, msg.sticky_faults);

    uint8_t bits = 0;
    bits |= (msg.hardware_fault << 0);
    bits |= (msg.proc_temp_fault << 1);
    bits |= (msg.device_temp_fault << 2);
    bits |= (msg.undervoltage_fault << 3);
    bits |= (msg.boot_fault << 4);
    bits |= (msg.unliscensed_fault << 5);
    bits |= (msg.bridge_brownout_fault << 6);
    bits |= (msg.overvoltage_fault << 7);
    writeAndIncrement(ptr, bits);
    bits = 0;
    bits |= (msg.unstable_voltage_fault << 0);
    bits |= (msg.stator_current_limit_fault << 1);
    bits |= (msg.supply_current_limit_fault << 2);
    bits |= (msg.static_brake_disabled_fault << 3);
    bits |= (msg.sticky_hardware_fault << 4);
    bits |= (msg.sticky_proc_temp_fault << 5);
    bits |= (msg.sticky_device_temp_fault << 6);
    bits |= (msg.sticky_undervoltage_fault << 7);
    writeAndIncrement(ptr, bits);
    bits = 0;
    bits |= (msg.sticky_boot_fault << 0);
    bits |= (msg.sticky_unliscensed_fault << 1);
    bits |= (msg.sticky_bridge_brownout_fault << 2);
    bits |= (msg.sticky_overvoltage_fault << 3);
    bits |= (msg.sticky_unstable_voltage_fault << 4);
    bits |= (msg.sticky_stator_current_limit_fault << 5);
    bits |= (msg.sticky_supply_current_limit_fault << 6);
    bits |= (msg.sticky_static_brake_disabled_fault << 7);
    writeAndIncrement(ptr, bits);

    return true;
}

bool TalonFaultsAdapter::deserializeMsg(
    MsgT& msg,
    const ByteBuffer& bytes,
    PubStateT&)
{
    if (bytes.size() < FAULTS_PACKED_SIZE)
    {
        return false;
    }

    const uint8_t* ptr = bytes.data();

    readAndIncrement(ptr, msg.header.stamp.sec);
    readAndIncrement(ptr, msg.header.stamp.nanosec);

    readAndIncrement(ptr, msg.faults);
    readAndIncrement(ptr, msg.sticky_faults);

    uint8_t bits = 0;
    readAndIncrement(ptr, bits);
    msg.hardware_fault = (bits >> 0) & 1;
    msg.proc_temp_fault = (bits >> 1) & 1;
    msg.device_temp_fault = (bits >> 2) & 1;
    msg.undervoltage_fault = (bits >> 3) & 1;
    msg.boot_fault = (bits >> 4) & 1;
    msg.unliscensed_fault = (bits >> 5) & 1;
    msg.bridge_brownout_fault = (bits >> 6) & 1;
    msg.overvoltage_fault = (bits >> 7) & 1;
    readAndIncrement(ptr, bits);
    msg.unstable_voltage_fault = (bits >> 0) & 1;
    msg.stator_current_limit_fault = (bits >> 1) & 1;
    msg.supply_current_limit_fault = (bits >> 2) & 1;
    msg.static_brake_disabled_fault = (bits >> 3) & 1;
    msg.sticky_hardware_fault = (bits >> 4) & 1;
    msg.sticky_proc_temp_fault = (bits >> 5) & 1;
    msg.sticky_device_temp_fault = (bits >> 6) & 1;
    msg.sticky_undervoltage_fault = (bits >> 7) & 1;
    readAndIncrement(ptr, bits);
    msg.sticky_boot_fault = (bits >> 0) & 1;
    msg.sticky_unliscensed_fault = (bits >> 1) & 1;
    msg.sticky_bridge_brownout_fault = (bits >> 2) & 1;
    msg.sticky_overvoltage_fault = (bits >> 3) & 1;
    msg.sticky_unstable_voltage_fault = (bits >> 4) & 1;
    msg.sticky_stator_current_limit_fault = (bits >> 5) & 1;
    msg.sticky_supply_current_limit_fault = (bits >> 6) & 1;
    msg.sticky_static_brake_disabled_fault = (bits >> 7) & 1;

    return true;
}
