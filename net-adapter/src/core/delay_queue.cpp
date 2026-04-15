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

#include "delay_queue.hpp"


DelayQueue::~DelayQueue()
{
    this->stopThread();
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

    // First loop iteration has no delay - this gets initialized from the
    // top queue message or with a dummy delay if the queue is empty.
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
                // This is acceptable since the move doesn't modify anything
                // which determines the queue order (this is why top() is only
                // const&), and we immediately remove the message from the queue.
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
