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
#include <cstdint>


namespace lance
{

enum class ControlMode : uint8_t
{
    DISABLED = 0,
    TELEOPERATED = 1,
    AUTONOMOUS = 2
};
enum class ControlOpts : uint8_t
{
    NONE = 0,
    TEST_MODE = 1
};


class ControlStatus
{
private:
    template<typename R, typename P>
    using Duration = std::chrono::duration<R, P>;
    using Milliseconds = std::chrono::milliseconds;

    static constexpr int64_t DEFAULT_TELEOP_FEED_TIME_MS = 250;
    static constexpr int64_t DEFAULT_AUTO_FEED_TIME_MS = 10000;

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

    static constexpr inline ControlMode getMode(int32_t status)
    {
        const int32_t watchdog = status / 1000;
        return (watchdog > 0) ? ControlMode::TELEOPERATED
                              : ((watchdog < 0) ? ControlMode::AUTONOMOUS
                                                : ControlMode::DISABLED);
    }
    static constexpr inline uint32_t getTimeoutMs(int32_t status)
    {
        return std::abs(status / 1000);
    }
    static constexpr inline uint8_t getOpts(int32_t status)
    {
        return static_cast<uint8_t>(std::abs(status) % 1000);
    }
    template<ControlOpts Opt_V>
    static constexpr inline bool hasOpt(int32_t status)
    {
        return !(getOpts(status) ^ static_cast<uint8_t>(Opt_V));
    }

};  // namespace ctrl_status
};  // namespace lance
