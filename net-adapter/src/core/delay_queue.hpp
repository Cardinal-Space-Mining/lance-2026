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
#include <queue>
#include <atomic>
#include <chrono>
#include <thread>
#include <vector>
#include <functional>
#include <condition_variable>


class DelayQueue
{
    using ClockT = std::chrono::steady_clock;
    using TimePointT = ClockT::time_point;
    using DurationT = std::chrono::nanoseconds;

    using ByteBuffer = std::vector<uint8_t>;
    using PublishFn = std::function<void(ByteBuffer&&)>;

public:
    template<typename R, typename P>
    explicit DelayQueue(std::chrono::duration<R, P> delay);
    ~DelayQueue();

public:
    template<typename R, typename P>
    void setDelay(std::chrono::duration<R, P> delay);
    DurationT getDelay() const;

    void startThread();
    void stopThread();

    void push(PublishFn pub_fn, ByteBuffer&& payload);

private:
    struct QueuedMessage
    {
        TimePointT pub_time;
        ByteBuffer payload;
        PublishFn pub_fn;

        bool operator<(const QueuedMessage&) const;
        bool operator>(const QueuedMessage&) const;
    };

private:
    TimePointT delayFromNow() const;

    void thread_worker();

private:
    DurationT delay;

    std::priority_queue<
        QueuedMessage,
        std::vector<QueuedMessage>,
        std::greater<QueuedMessage>>
        msg_queue;

    std::thread thread;
    std::mutex mtx;
    std::atomic_bool is_running{false};
    std::condition_variable msg_notifier;
};



// --- Implementation (templated methods) --------------------------------------

template<typename R, typename P>
DelayQueue::DelayQueue(std::chrono::duration<R, P> d) :
    delay{std::chrono::duration_cast<DurationT>(d)}
{
}

template<typename R, typename P>
void DelayQueue::setDelay(std::chrono::duration<R, P> d)
{
    std::lock_guard lock{this->mtx};
    this->delay = std::chrono::duration_cast<DurationT>(d);
}
