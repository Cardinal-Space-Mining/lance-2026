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
 * @file robot_status.hpp
 * @brief Robot operational status, watchdog encoding, and control mode bitfield definitions.
 *
 * ## Robot Status INT32 Protocol
 *
 * To maintain safety over wireless links, the robot status message is sent as a compact
 * single INT32 integer over the `/robot_status` topic. This encodes:
 *
 *   1. **Watchdog Timeout & Operating Mode (Sign and Quotient / 1000):**
 *      - `status / 1000 > 0`: **TELEOPERATED** mode. Magnitude is watchdog feed timeout in ms (default: 250 ms).
 *      - `status / 1000 < 0`: **AUTONOMOUS** mode. Magnitude is watchdog feed timeout in ms (default: 10,000 ms).
 *      - `status / 1000 == 0`: **DISABLED** mode. Actuators remain unpowered.
 *
 *   2. **Augmentation & Test Options (Remainder % 1000):**
 *      - `abs(status) % 1000` is a bitfield (9 available bits, 0-511) containing flags such as:
 *          * `TEST_MODE`: Prevents full plunge depth during lab testing.
 *          * `QUICK_AUTO`: Minimizes auto duration to harvest quick competition points.
 *          * `ASSIST_AS_AUTO`: Forces assisted driver-assist modes into fully autonomous routines.
 */

#include <chrono>
#include <limits>
#include <cstdint>


namespace lance
{

/**
 * @enum ControlMode
 * @brief High-level robot execution state.
 */
enum class ControlMode : uint8_t
{
    DISABLED = 0,     ///< Robot e-stopped or idle; zero motor output.
    TELEOPERATED = 1, ///< Direct operator control via joystick/teleop commands.
    AUTONOMOUS = 2    ///< Closed-loop state machine executing navigation/mining missions.
};

/**
 * @enum ControlOpts
 * @brief Bitfield flags modifying operational constraints or autonomous behaviors.
 */
enum class ControlOpts : uint8_t
{
    NONE = 0,
    /// Limits minimum trencher actuator height for benchtop testing and sim sanity.
    TEST_MODE = (1 << 0),
    /// Prioritizes fast traversal and mining return for maximum points under time limit.
    QUICK_AUTO = (1 << 1),
    /// Promotes operator-assisted sub-routines (e.g. alignment) into autonomous execution.
    ASSIST_AS_AUTO = (1 << 2)
};


/**
 * @class ControlStatus
 * @brief Encoder and decoder utility methods for the INT32 `/robot_status` protocol.
 */
class ControlStatus
{
private:
    template<typename R, typename P>
    using Duration = std::chrono::duration<R, P>;
    using Milliseconds = std::chrono::milliseconds;

    static constexpr int64_t DEFAULT_TELEOP_FEED_TIME_MS = 250;   ///< Nominal 4 Hz watchdog feed for teleoperation.
    static constexpr int64_t DEFAULT_AUTO_FEED_TIME_MS = 10000;  ///< Extended 10s watchdog feed for autonomous execution.

    /**
     * @brief Safely clamp duration to prevent 32-bit integer overflow when scaled by 1000.
     */
    template<typename R, typename P>
    static constexpr inline int32_t getClampedFeedTimeUs(Duration<R, P> dur)
    {
        using namespace std::chrono;

        const int64_t clamped_ms = std::min(
            std::abs(duration_cast<milliseconds>(dur).count()),
            duration_cast<milliseconds>(
                microseconds(std::numeric_limits<int32_t>::max()))
                .count());

        return static_cast<int32_t>(
            duration_cast<microseconds>(milliseconds(clamped_ms)).count());
    }

public:
    /**
     * @brief Encode control mode, option bitflags, and custom feed times into an INT32 word.
     *
     * @param mode Target ControlMode (DISABLED, TELEOPERATED, AUTONOMOUS).
     * @param opts Bitwise OR of ControlOpts flags.
     * @param teleop_feed_time Watchdog duration for teleoperated mode.
     * @param auto_feed_time Watchdog duration for autonomous mode.
     * @return Packed INT32 status value.
     */
    template<typename R1, typename P1, typename R2, typename P2>
    static constexpr inline int32_t format(
        ControlMode mode,
        uint8_t opts = static_cast<uint8_t>(ControlOpts::NONE),
        Duration<R1, P1> teleop_feed_time =
            Milliseconds(DEFAULT_TELEOP_FEED_TIME_MS),
        Duration<R2, P2> auto_feed_time =
            Milliseconds(DEFAULT_AUTO_FEED_TIME_MS))
    {
        int32_t v = 0;
        switch (mode)
        {
            case ControlMode::DISABLED:
            {
                v = static_cast<int32_t>(opts);
                break;
            }
            case ControlMode::TELEOPERATED:
            {
                v = getClampedFeedTimeUs(teleop_feed_time) +
                    static_cast<int32_t>(opts);
                break;
            }
            case ControlMode::AUTONOMOUS:
            {
                v = (getClampedFeedTimeUs(auto_feed_time) +
                     static_cast<int32_t>(opts)) *
                    -1;
                break;
            }
        }
        return v;
    }

    /**
     * @brief Overload using compile-time default feed times in milliseconds.
     */
    template<
        int64_t Teleop_Feed_Time_Ms = DEFAULT_TELEOP_FEED_TIME_MS,
        int64_t Auto_Feed_Time_Ms = DEFAULT_AUTO_FEED_TIME_MS>
    static constexpr inline int32_t format(
        ControlMode mode,
        uint8_t opts = ControlOpts::NONE)
    {
        using namespace std::chrono;

        return format(
            mode,
            opts,
            milliseconds(Teleop_Feed_Time_Ms),
            milliseconds(Auto_Feed_Time_Ms));
    }

    /**
     * @brief Extract ControlMode from an INT32 status word.
     *
     * Evaluates the sign of status / 1000:
     *   > 0 -> TELEOPERATED
     *   < 0 -> AUTONOMOUS
     *   == 0 -> DISABLED
     */
    static constexpr inline ControlMode getMode(int32_t status)
    {
        const int32_t watchdog = status / 1000;
        return (watchdog > 0) ? ControlMode::TELEOPERATED
                               : ((watchdog < 0) ? ControlMode::AUTONOMOUS
                                                 : ControlMode::DISABLED);
    }

    /**
     * @brief Extract watchdog timeout period in milliseconds from an INT32 status word.
     */
    static constexpr inline uint32_t getTimeoutMs(int32_t status)
    {
        return std::abs(status / 1000);
    }

    /**
     * @brief Extract option bitfield byte from an INT32 status word.
     */
    static constexpr inline uint8_t getOpts(int32_t status)
    {
        return static_cast<uint8_t>(std::abs(status) % 1000);
    }

    /**
     * @brief Check whether a specific ControlOpts flag is enabled in an INT32 status word.
     */
    template<ControlOpts Opt_V>
    static constexpr inline bool hasOpt(int32_t status)
    {
        return !(getOpts(status) ^ static_cast<uint8_t>(Opt_V));
    }

};

};  // namespace lance
