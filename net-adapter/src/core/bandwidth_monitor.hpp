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

#include <mutex>
#include <atomic>
#include <chrono>
#include <deque>
#include <string>
#include <cstddef>
#include <cstdint>

#include <net_adapter/msg/net_bandwidth.hpp>


/* Counts the bytes each adapter channel sends to / receives from zenoh, and
 * turns them into per-period rates for the NetBandwidth message.
 *
 * Channels register once at construction (addChannel) and keep the returned
 * Counter pointer, which stays valid for the monitor's lifetime. Counting is
 * lock-free: add() is called from the ROS executor (outgoing messages), zenoh
 * threads (incoming samples) and the delay queue thread (delayed sends), so
 * the counters are relaxed atomics. sample() is meant to be called from a
 * single thread (the publish timer). */
class BandwidthMonitor
{
public:
    enum class Direction
    {
        TX,
        RX
    };

    class Counter
    {
        friend BandwidthMonitor;

    public:
        inline void add(size_t bytes)
        {
            this->bytes.fetch_add(bytes, std::memory_order_relaxed);
            this->msgs.fetch_add(1, std::memory_order_relaxed);
        }

    private:
        std::atomic<uint64_t> bytes{0};
        std::atomic<uint64_t> msgs{0};
    };

public:
    BandwidthMonitor();

    /* Thread-safe. The returned pointer is never invalidated. */
    Counter* addChannel(const std::string& topic, Direction dir);

    /* Fill `msg` with rates since the previous call (or construction) and
     * running totals. `msg.stamp` is left to the caller. */
    void sample(net_adapter::msg::NetBandwidth& msg);

private:
    struct Channel
    {
        std::string topic;
        Direction dir;
        Counter counter;
        uint64_t last_bytes{0};
        uint64_t last_msgs{0};
    };

    std::mutex mtx;
    std::deque<Channel> channels;  // deque: stable element addresses
    std::chrono::steady_clock::time_point last_sample;
};
