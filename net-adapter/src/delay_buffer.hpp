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
#include <chrono>
#include <string>
#include <vector>
#include <functional>

#include <zenoh.hxx>


class DelayBuffer
{
public:
    using Clock = std::chrono::steady_clock;
    using TimePoint = Clock::time_point;
    using ByteBuffer = std::vector<uint8_t>;
    using PublishFn = std::function<void(ByteBuffer&&)>;

    using Milliseconds = std::chrono::milliseconds;

public:
    template<typename R, typename P>
    explicit DelayBuffer(std::chrono::duration<R, P> delay);

public:
    /* Change delay at runtime (takes effect for future enqueues). */
    template<typename R, typename P>
    void setDelay(std::chrono::duration<R, P> delay);

    Milliseconds getDelay() const;

    /* Enqueue a message.  Called from ROS subscriber callbacks. */
    void enqueue(PublishFn publish_fn, ByteBuffer&& bytes);

    /* Forward all entries whose deadline has passed.
     * Call this from an rclcpp::Timer callback at a rate finer than the
     * desired delay resolution (e.g. every 10 ms). */
    void drain();

private:
    struct Entry
    {
        TimePoint deadline;
        PublishFn publish;  // captures the specific zenoh::Publisher*
        ByteBuffer bytes;
    };

    std::deque<Entry> queue;
    std::mutex mtx;
    Milliseconds delay;
};



// --- Implementation ----------------------------------------------------------

template<typename R, typename P>
DelayBuffer::DelayBuffer(std::chrono::duration<R, P> d) :
    delay{std::chrono::duration_cast<std::chrono::milliseconds>(d)}
{}

template<typename R, typename P>
void DelayBuffer::setDelay(std::chrono::duration<R, P> d)
{
    std::lock_guard<std::mutex> lk{this->mtx};
    this->delay = std::chrono::duration_cast<std::chrono::milliseconds>(d);
}

DelayBuffer::Milliseconds DelayBuffer::getDelay() const
{
    return this->delay;
}

void DelayBuffer::enqueue(PublishFn publish_fn, ByteBuffer&& bytes)
{
    Entry e;
    e.deadline = Clock::now() + this->delay;
    e.publish = std::move(publish_fn);
    e.bytes = std::move(bytes);

    std::lock_guard<std::mutex> lk{this->mtx};
    this->queue.emplace_back(std::move(e));
}

void DelayBuffer::drain()
{
    const TimePoint now = Clock::now();

    // Collect under the lock, publish outside it.
    std::vector<Entry> ready;
    {
        std::lock_guard<std::mutex> lk{this->mtx};
        while (!this->queue.empty() && this->queue.front().deadline <= now)
        {
            ready.emplace_back(std::move(this->queue.front()));
            this->queue.pop_front();
        }
    }

    for (auto& e : ready)
    {
        e.publish(std::move(e.bytes));
    }
}
