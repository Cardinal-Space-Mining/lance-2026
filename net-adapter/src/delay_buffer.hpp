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
#include <deque>
#include <functional>
#include <mutex>
#include <string>
#include <vector>

#include <zenoh.hxx>


class DelayBuffer
{
public:
    using ByteBuffer = std::vector<uint8_t>;
    using Clock = std::chrono::steady_clock;
    using TimePoint = Clock::time_point;
    using PublishFn = std::function<void(ByteBuffer)>;

private:
    struct Entry
    {
        TimePoint deadline;
        PublishFn publish;  // captures the specific zenoh::Publisher*
        ByteBuffer bytes;
    };

    std::deque<Entry> queue_;
    std::mutex mutex_;
    std::chrono::milliseconds delay_;

public:
    explicit DelayBuffer(std::chrono::milliseconds delay) : delay_{delay} {}

    /* Change delay at runtime (takes effect for future enqueues). */
    void set_delay(std::chrono::milliseconds delay)
    {
        std::lock_guard<std::mutex> lk{mutex_};
        delay_ = delay;
    }

    std::chrono::milliseconds get_delay() const { return delay_; }

    /* Enqueue a message.  Called from ROS subscriber callbacks. */
    void enqueue(PublishFn publish_fn, ByteBuffer bytes)
    {
        Entry e;
        e.deadline = Clock::now() + delay_;
        e.publish = std::move(publish_fn);
        e.bytes = std::move(bytes);

        std::lock_guard<std::mutex> lk{mutex_};
        queue_.push_back(std::move(e));
    }

    /* Forward all entries whose deadline has passed.
     * Call this from an rclcpp::Timer callback at a rate finer than the
     * desired delay resolution (e.g. every 10 ms). */
    void drain()
    {
        const TimePoint now = Clock::now();

        // Collect under the lock, publish outside it.
        std::vector<Entry> ready;
        {
            std::lock_guard<std::mutex> lk{mutex_};
            while (!queue_.empty() && queue_.front().deadline <= now)
            {
                ready.push_back(std::move(queue_.front()));
                queue_.pop_front();
            }
        }

        for (auto& e : ready)
        {
            e.publish(std::move(e.bytes));
        }
    }
};
