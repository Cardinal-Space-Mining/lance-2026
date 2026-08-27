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

#include "latency_ping.hpp"

#include <random>

#include <csm_utils/mem_helpers.hpp>
#include "../util/zenoh_utils.hpp"


inline static uint32_t getId(uint32_t id)
{
    if (id > 0)
    {
        return id;
    }
    else
    {
        std::mt19937_64 gen{std::random_device{}()};
        std::uniform_int_distribution<uint32_t> d;
        return d(gen);
    }
}


LatencyPing::LatencyPing(
    zenoh::Session& zsh,
    DelayQueue* delay_q,
    uint32_t id) :
    dq{delay_q},
    id{getId(id)},
    pub{zsh.declare_publisher(PING_TOPIC)},
    sub{zsh.declare_subscriber(
        PING_TOPIC,
        [this](const zenoh::Sample& sample) { this->callback(sample); },
        []() {})}
{
}


void LatencyPing::ping()
{
    const uint32_t ping_id = this->ping_count++;
    if (this->ping_count >=
        (1U << (sizeof(decltype(this->ping_count)) * 8 - 1)))
    {
        this->ping_count = 0;
    }

    std::vector<uint8_t> bytes;
    bytes.resize(sizeof(uint32_t) * 2);
    util::write(bytes.data(), (ping_id << 1));
    util::write(bytes.data() + sizeof(uint32_t), this->id);

    {
        std::unique_lock l{this->mtx};
        this->active_pings.emplace_back(ping_id, steady_clock::now());
    }

    if (this->dq)
    {
        this->dq->push(
            [this](std::vector<uint8_t>&& b)
            { this->pub.put(zenoh::Bytes(std::move(b))); },
            std::move(bytes));
    }
    else
    {
        this->pub.put(zenoh::Bytes(std::move(bytes)));
    }
}

bool LatencyPing::hasResults() const
{
    std::unique_lock l{this->mtx};
    return !this->completed_pings.empty();
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


void LatencyPing::callback(const zenoh::Sample& sample)
{
    const steady_clock_time n = steady_clock::now();

    std::vector<uint8_t> bytes = sample.get_payload().as_vector();
    if (bytes.size() < (sizeof(uint32_t) * 2))
    {
        return;
    }

    uint32_t val, id;
    util::read(bytes.data(), val);
    util::read(bytes.data() + sizeof(uint32_t), id);

    if (id == this->id)
    {
        return;
    }

    if (val & 0x1)
    {
        std::unique_lock l{this->mtx};

        while (!this->active_pings.empty())
        {
            if ((val >> 1) == this->active_pings.front().first)
            {
                this->completed_pings.emplace_back(
                    this->active_pings.front().second,
                    n);
                this->active_pings.pop_front();
                break;
            }
            else
            {
                this->active_pings.pop_front();
            }
        }
    }
    else
    {
        util::write(bytes.data(), (val | 0x1));
        util::write(bytes.data() + sizeof(uint32_t), this->id);

        if (this->dq)
        {
            this->dq->push(
                [this](std::vector<uint8_t>&& b)
                { this->pub.put(zenoh::Bytes(std::move(b))); },
                std::move(bytes));
        }
        else
        {
            this->pub.put(zenoh::Bytes(std::move(bytes)));
        }
    }
}
