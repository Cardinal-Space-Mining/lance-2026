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

#include <deque>
#include <mutex>
#include <atomic>
#include <chrono>
#include <vector>
#include <utility>

#include <zenoh.hxx>

#include "delay_queue.hpp"


class LatencyPing
{
    using steady_clock = std::chrono::steady_clock;
    using steady_clock_time = steady_clock::time_point;

    using Ping = std::pair<uint32_t, steady_clock_time>;
    using PingResult = std::pair<steady_clock_time, steady_clock_time>;

    static constexpr char const* PING_TOPIC = "latency_ping";

public:
    LatencyPing(zenoh::Session&, DelayQueue* = nullptr, uint32_t = 0);

public:
    void ping();
    void clearResults();

    bool hasResults() const;
    template<typename DurT = std::chrono::milliseconds>
    DurT avgLatency() const;
    double avgLatencySeconds() const;

protected:
    void callback(const zenoh::Sample&);

protected:
    DelayQueue* dq{nullptr};
    const uint32_t id;

    zenoh::Publisher pub;
    zenoh::Subscriber<void> sub;

    std::deque<Ping> active_pings;
    std::vector<PingResult> completed_pings;

    std::atomic<uint32_t> ping_count{0};
    mutable std::mutex mtx;
};



template<typename D>
D LatencyPing::avgLatency() const
{
    std::unique_lock l{this->mtx};

    D sum{0};
    for (const PingResult& p : this->completed_pings)
    {
        sum += std::chrono::duration_cast<D>(p.second - p.first);
    }

    return (sum / this->completed_pings.size());
}
