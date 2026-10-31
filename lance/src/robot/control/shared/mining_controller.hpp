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
#include <limits>

#include <csm_utils/joy_utils.hpp>

#include "robot/core/robot_params.hpp"
#include "robot/core/stall_analyzer.hpp"
#include "robot/core/motor_interface.hpp"
#include "robot/core/collection_state.hpp"
#include "robot/sensing/sensing_interfaces.hpp"


namespace lance
{

class MiningConstraints
{
    using JoyState = util::JoyState;

public:
    enum
    {
        CONSTRAINT_NONE = 0,
        CONSTRAINT_MOTOR_STALL = (1 << 0),
        CONSTRAINT_OBSTACLE = (1 << 1),
        CONSTRAINT_HOPPER_FULL = (1 << 2),
        CONSTRAINT_ZONE_BOUNDARY = (1 << 3),
        CONSTRAINT_DISTANCE_LIMIT = (1 << 4),

        ALL_CONSTRAINTS =
            (CONSTRAINT_MOTOR_STALL | CONSTRAINT_OBSTACLE |
             CONSTRAINT_HOPPER_FULL | CONSTRAINT_ZONE_BOUNDARY |
             CONSTRAINT_DISTANCE_LIMIT)
    };

public:
    MiningConstraints(
        const RobotParams&,
        const HopperState&,
        const StallState&);

public:
    void updateSettings(const JoyState&);
    void resetState(
        float target_fullness = 0.f,
        float max_distance_m = std::numeric_limits<float>::infinity());
    void updateState(
        const RobotMotorStatus&,
        const TfCache&,
        const MiningEvalInterface&);

    float remainingDist() const;
    bool hasRemaining() const;

    uint8_t enabledConstraints() const;
    uint8_t currentConstraint() const;
    bool isConstraintEnabled(uint8_t) const;

protected:
    void updateOdom(const RobotMotorStatus&);

protected:
    const RobotParams& params;
    const HopperState& hopper_state;
    const StallState& stall_state;

    float remaining_dist{0.f};
    float prev_odom{0.f};
    float target_fullness{0.f};
    // Stop mining after driving this far (inf = no limit)
    float distance_limit{std::numeric_limits<float>::infinity()};
    float start_odom{std::numeric_limits<float>::infinity()};

    uint8_t enabled_constraints{ALL_CONSTRAINTS};
    uint8_t current_constraint{CONSTRAINT_NONE};
};


class MiningController
{
    friend class AutoMiningController;
    friend class TelemetrySerializer;
    friend class TelemetryDeserializer;

    using JoyState = util::JoyState;

public:
    MiningController(
        const RobotParams&,
        const HopperState&,
        const StallState&,
        SensingInterfaces&);
    ~MiningController() = default;

public:
    /* Restart the routine. If target fullness is provided, the routine
     * will be limited using the following behavior: negative values - collect
     * this much MORE volume, positive values - collect until this absolute
     * amount, zero - fill to default capacity. If max_distance_m is finite,
     * mining also stops after driving that far. */
    void initialize(
        float target_fullness = 0.f,
        float max_distance_m = std::numeric_limits<float>::infinity());
    /* Check if the command is finished, either as a result
     * of being cancelled or automatically shutting down
     * due to a stop state. */
    bool isFinished();
    /* Mark the command as cancelled, i.e. it will no longer be
     * executed. */
    void setCancelled();

    /* Iterate the controller in "full auto" mode (no user input). */
    void iterate(
        const RobotMotorStatus& motor_status,
        RobotMotorCommands& commands);
    /* Iterate the controller in "assisted" mode (user input). */
    void iterate(
        const JoyState& joy,
        const RobotMotorStatus& motor_status,
        RobotMotorCommands& commands);

    void updateConstraints(const JoyState& joy);

protected:
    enum class Stage
    {
        INITIALIZATION,
        LOWERING,
        TRAVERSING,
        RAISING,
        FINISHED
    };

protected:
    struct BeltDutyCycleState
    {
        void setMoved();
        void setStopped();
        bool canMove(float thresh_s);

    private:
        std::chrono::system_clock::time_point prev_belt_stop_time;
        bool belt_moving{false};
    };

protected:
    void iterate(
        const JoyState* joy,
        const RobotMotorStatus& motor_status,
        RobotMotorCommands& commands);

protected:
    const RobotParams& params;
    const HopperState& hopper_state;
    const TfCache& tf_cache;
    MiningEvalInterface& mining_eval_interface;

    MiningConstraints constraints;
    BeltDutyCycleState belt_duty_cycle{};

    Stage stage{Stage::FINISHED};
    float run_start_odom{0.f};
};

};  // namespace lance
