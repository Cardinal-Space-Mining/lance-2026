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

/**
 * @file motor_interface.hpp
 * @brief Abstraction structures and helper methods interfacing with CTRE Talon FX motor controllers.
 *
 * Interfaces with the `phoenix_ros_driver` package, wrapping:
 *   - TalonInfo messages into `RobotMotorStatus` (positions, velocities, currents, temperatures).
 *   - TalonFaults messages into `RobotMotorFaults` (stalls, hardware limits, undervoltage).
 *   - TalonCtrl messages into `RobotMotorCommands` (velocity, position, percent output, voltage modes).
 *
 * The robot actuator complement comprises 5 motor channels:
 *   1. `track_right`: Right side differential drive tracks.
 *   2. `track_left`: Left side differential drive tracks.
 *   3. `trencher`: Continuous bucket excavation cutting head.
 *   4. `hopper_belt`: Payload distribution and offload discharge conveyor.
 *   5. `hopper_actuator`: Linear actuator governing trencher tilt angle and digging depth.
 */

#include <phoenix_ros_driver/msg/talon_ctrl.hpp>
#include <phoenix_ros_driver/msg/talon_faults.hpp>
#include <phoenix_ros_driver/msg/talon_info.hpp>


namespace lance
{

using TalonCtrlMsg = phoenix_ros_driver::msg::TalonCtrl;
using TalonFaultsMsg = phoenix_ros_driver::msg::TalonFaults;
using TalonInfoMsg = phoenix_ros_driver::msg::TalonInfo;


/**
 * @struct RobotMotorStatus
 * @brief Aggregated feedback telemetry across all 5 robot drive motors.
 */
struct RobotMotorStatus
{
    TalonInfoMsg track_right;     ///< Right drive track motor telemetry (rotations, RPS, current).
    TalonInfoMsg track_left;      ///< Left drive track motor telemetry (rotations, RPS, current).
    TalonInfoMsg trencher;        ///< Trencher cutter head motor telemetry.
    TalonInfoMsg hopper_belt;     ///< Hopper conveyor belt motor telemetry.
    TalonInfoMsg hopper_actuator; ///< Trencher tilt linear actuator telemetry.

    /**
     * @brief Normalized position of linear actuator [0.0 = fully retracted, 1.0 = fully extended].
     */
    inline double getHopperActNormalizedValue() const
    {
        return this->hopper_actuator.position;
    }
};

/**
 * @struct RobotMotorFaults
 * @brief Hardware error status, current limit flags, and thermal warnings for each motor channel.
 */
struct RobotMotorFaults
{
    TalonFaultsMsg track_right;     ///< Right track motor controller faults.
    TalonFaultsMsg track_left;      ///< Left track motor controller faults.
    TalonFaultsMsg trencher;        ///< Trencher motor controller faults.
    TalonFaultsMsg hopper_belt;     ///< Hopper belt motor controller faults.
    TalonFaultsMsg hopper_actuator; ///< Linear actuator controller faults.
};

/**
 * @struct RobotMotorCommands
 * @brief Output command payload sent to CTRE Talon FX motor controllers.
 */
struct RobotMotorCommands
{
    TalonCtrlMsg track_right;     ///< Right track control packet.
    TalonCtrlMsg track_left;      ///< Left track control packet.
    TalonCtrlMsg trencher;        ///< Trencher control packet.
    TalonCtrlMsg hopper_belt;     ///< Hopper conveyor belt control packet.
    TalonCtrlMsg hopper_actuator; ///< Linear actuator control packet.

    /**
     * @brief Command closed-loop velocity targets to differential drive tracks.
     * @param left_rps Commanded shaft rotations per second for left track motor.
     * @param right_rps Commanded shaft rotations per second for right track motor.
     */
    inline void setTracksVelocity(double left_rps, double right_rps)
    {
        this->track_left.set__mode(TalonCtrlMsg::VELOCITY).set__value(left_rps);
        this->track_right.set__mode(TalonCtrlMsg::VELOCITY)
            .set__value(right_rps);
    }

    /**
     * @brief Command closed-loop velocity target to trencher cutter head.
     * @param rps Commanded shaft rotations per second.
     */
    inline void setTrencherVelocity(double rps)
    {
        this->trencher.set__mode(TalonCtrlMsg::VELOCITY).set__value(rps);
    }

    /**
     * @brief Command closed-loop velocity target to hopper conveyor belt.
     * @param rps Commanded shaft rotations per second.
     */
    inline void setHopperBeltVelocity(double rps)
    {
        this->hopper_belt.set__mode(TalonCtrlMsg::VELOCITY).set__value(rps);
    }

    /**
     * @brief Command closed-loop position target to linear actuator.
     * @param val Target position in rotations / normalized stroke.
     */
    inline void setHppperActPosition(double val)
    {
        this->hopper_actuator.set__mode(TalonCtrlMsg::POSITION).set__value(val);
    }

    /**
     * @brief Command velocity target to linear actuator.
     * @param val Actuator velocity.
     */
    inline void setHopperActSpeed(double val)
    {
        this->hopper_actuator.set__mode(TalonCtrlMsg::VELOCITY).set__value(val);
    }

    /**
     * @brief Command open-loop voltage directly to linear actuator.
     * @param volts Target bus voltage applied to motor terminals.
     */
    inline void setHopperActVoltage(double volts)
    {
        this->hopper_actuator.set__mode(TalonCtrlMsg::VOLTAGE)
            .set__value(volts);
    }

    /**
     * @brief Put both drive track motor controllers into neutral / disabled state.
     */
    inline void disableTracks()
    {
        this->track_left.set__mode(TalonCtrlMsg::DISABLED).set__value(0.);
        this->track_right.set__mode(TalonCtrlMsg::DISABLED).set__value(0.);
    }

    /**
     * @brief Put trencher motor controller into neutral / disabled state.
     */
    inline void disableTrencher()
    {
        this->trencher.set__mode(TalonCtrlMsg::DISABLED).set__value(0.);
    }

    /**
     * @brief Put hopper conveyor motor controller into neutral / disabled state.
     */
    inline void disableHopperBelt()
    {
        this->hopper_belt.set__mode(TalonCtrlMsg::DISABLED).set__value(0.);
    }

    /**
     * @brief Put linear actuator motor controller into neutral / disabled state.
     */
    inline void disableHopperAct()
    {
        this->hopper_actuator.set__mode(TalonCtrlMsg::DISABLED).set__value(0.);
    }

    /**
     * @brief Disable and de-energize all 5 robot motor channels simultaneously.
     */
    inline void disableAll()
    {
        this->disableTracks();
        this->disableTrencher();
        this->disableHopperBelt();
        this->disableHopperAct();
    }
};

};  // namespace lance
