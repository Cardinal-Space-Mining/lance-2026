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
#include <functional>
#include <condition_variable>

#include <zenoh.hxx>


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



// --- Implementation ----------------------------------------------------------

template<typename R, typename P>
DelayQueue::DelayQueue(std::chrono::duration<R, P> d) :
    delay{std::chrono::duration_cast<DurationT>(d)}
{
}

DelayQueue::~DelayQueue()
{
    this->stopThread();
}

template<typename R, typename P>
void DelayQueue::setDelay(std::chrono::duration<R, P> d)
{
    std::lock_guard lock{this->mtx};
    this->delay = std::chrono::duration_cast<DurationT>(d);
}

DelayQueue::DurationT DelayQueue::getDelay() const { return this->delay; }

void DelayQueue::startThread()
{
    if (!this->is_running)
    {
        this->is_running = true;
        this->thread = std::thread{&DelayQueue::thread_worker, this};
    }
}
void DelayQueue::stopThread()
{
    this->is_running = false;
    if (this->thread.joinable())
    {
        this->msg_notifier.notify_one();
        this->thread.join();
    }
}

void DelayQueue::push(PublishFn pub_fn, ByteBuffer&& payload)
{
    std::lock_guard lock{this->mtx};

    this->msg_queue.emplace(
        this->delayFromNow(),
        std::move(payload),
        std::move(pub_fn));

    this->msg_notifier.notify_one();
}


bool DelayQueue::QueuedMessage::operator<(const QueuedMessage& m) const
{
    return this->pub_time < m.pub_time;
}
bool DelayQueue::QueuedMessage::operator>(const QueuedMessage& m) const
{
    return this->pub_time > m.pub_time;
}


DelayQueue::TimePointT DelayQueue::delayFromNow() const
{
    return (ClockT::now() + this->delay);
}

void DelayQueue::thread_worker()
{
    std::mutex tmp_mtx;
    std::unique_lock<std::mutex> tmp_lock{tmp_mtx};

    // First loop iteration has no delay and this gets initialized from the
    // queue top or with a dummy delay if the queue is empty.
    TimePointT next{};
    do
    {
        // Normally we would check the status to see if the wait timed out,
        // was triggered by another thread, or spurriously awoken, but in all
        // cases we might as well recheck the queue; thus it doesn't matter.
        this->msg_notifier.wait_until(tmp_lock, next);

        std::lock_guard lock{this->mtx};
        while(!this->msg_queue.empty())
        {
            const TimePointT rn = ClockT::now();
            const QueuedMessage& msg = this->msg_queue.top();
            if(msg.pub_time <= rn)
            {
                msg.pub_fn(std::move(const_cast<QueuedMessage&>(msg).payload));
                this->msg_queue.pop();
            }
            else
            {
                next = msg.pub_time;
                break;
            }
        }
        if (this->msg_queue.empty())
        {
            next = this->delayFromNow();
        }
    }  //
    while (this->is_running.load());
}
