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
#include <vector>
#include <utility>

#include <zenoh.hxx>

#include "../util/delay_queue.hpp"
#include "../util/mem_helpers.hpp"


class LatencyPing
{
    using steady_clock = std::chrono::steady_clock;
    using steady_clock_time = steady_clock::time_point;

    using Ping = std::pair<steady_clock_time, steady_clock_time>;
    using PingResults = std::vector<Ping>;

    static constexpr char const* PING_TOPIC = "latency_ping";

    enum : uint8_t
    {
        PING_REQUEST = 0,
        PING_REPLY = 1
    };

public:
    LatencyPing(zenoh::Session&, DelayQueue* = nullptr);

public:
    void ping();

    bool hasResults() const;
    const PingResults& results() const;

    template<typename DurT = std::chrono::milliseconds>
    DurT avgLatency() const;
    double avgLatencySeconds() const;

    void clearResults() const;

protected:
    static zenoh::SubscriberOptions getIgnoreLocalOpts();

    void callback(const zenoh::Sample&);

protected:
    DelayQueue* dq{nullptr};

    zenoh::Publisher pub;
    zenoh::Subscriber<void> sub;

    std::deque<steady_clock_time> active_pings;
    PingResults completed_pings;

    size_t ping_count{0};
    std::mutex mtx;
};


LatencyPing::LatencyPing(zenoh::Session& zsh, DelayQueue* delay_q) :
    dq{delay_q},
    pub{zsh.declare_publisher(PING_TOPIC)},
    sub{zsh.declare_subscription(
        PING_TOPIC,
        [this](const zenoh::Sample& sample) { this->callback(sample); },
        getIgnoreLocalOpts())}
{
}


void LatencyPing::ping()
{
    std::vector<uint8_t> msg;
    msg.resize(sizeof(size_t));

    size_t val = (this->ping_count++ << 1);
    util::write(msg.data(), val);

    {
        std::unique_lock l{this->mtx};
        this->active_pings.emplace_back(steady_clock::now());
    }

    if (this->dq)
    {
        this->dq->push(
            [this](std::vector<uint8_t>&& b)
            { this->pub.put(zenoh::Bytes(std::move(b))); },
            std::move(msg));
    }
    else
    {
        this->pub.put(zenoh::Bytes(std::move(msg)));
    }
}

bool LatencyPing::hasResults() const
{
    std::unique_lock l{this->mtx};
    return !this->completed_pings.empty();
}

const LatencyPing::PingResults& LatencyPing::results() const
{
    return this->completed_pings;
}

template<typename D>
D LatencyPing::avgLatency() const
{
    std::unique_lock l{this->mtx};

    D sum;
    for (const Ping& p : this->completed_pings)
    {
        sum += std::chrono::duration_cast<D>(p.second - p.first);
    }

    return (sum / this->completed_pings.size());
}

double LatencyPing::avgLatencySeconds() const
{
    return this->avgLatency<std::chrono::duration<double>>().count();
}

void LatencyPing::clearResults()
{
    std::unique_lock l{this->mtx};
    this->completed_pings.clear();
}


zenoh::SubscriberOptions LatencyPing::getIgnoreLocalOpts()
{
    zenoh::SubscriberOptions opts;
    opts.allowed_origin = zenoh::Locality::Z_LOCALITY_REMOTE;
    return opts;
}

void LatencyPing::callback(const zenoh::Sample& sample)
{
    const steady_clock_time n = steady_clock::now();

    std::vector<uint8_t> msg = sample.get_payload().as_vector();

    size_t val;
    util::read(msg.data(), val);

    if(val & PING_REQUEST)
    {
        // publish mirrored msg
    }
    else
    {
        std::unique_lock l{this->mtx};

        this->completed_pings.emplace_back(this->active_pings.front(), n);
        this->active_pings.pop_front();
    }
}
