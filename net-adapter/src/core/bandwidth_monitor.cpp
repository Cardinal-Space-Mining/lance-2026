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

#include "bandwidth_monitor.hpp"


BandwidthMonitor::BandwidthMonitor() :
    last_sample{std::chrono::steady_clock::now()}
{
}

BandwidthMonitor::Counter* BandwidthMonitor::addChannel(
    const std::string& topic,
    Direction dir)
{
    std::unique_lock l{this->mtx};
    Channel& c = this->channels.emplace_back();
    c.topic = topic;
    c.dir = dir;
    return &c.counter;
}

void BandwidthMonitor::sample(net_adapter::msg::NetBandwidth& msg)
{
    const auto now = std::chrono::steady_clock::now();
    const double period_s =
        std::chrono::duration<double>(now - this->last_sample).count();
    const double inv_period = period_s > 0. ? 1. / period_s : 0.;
    this->last_sample = now;

    msg.period_s = static_cast<float>(period_s);
    msg.tx_bytes_per_s = 0.;
    msg.rx_bytes_per_s = 0.;
    msg.tx_bytes_total = 0;
    msg.rx_bytes_total = 0;

    std::unique_lock l{this->mtx};
    msg.channels.resize(this->channels.size());
    for (size_t i = 0; i < this->channels.size(); i++)
    {
        Channel& c = this->channels[i];
        const uint64_t bytes =
            c.counter.bytes.load(std::memory_order_relaxed);
        const uint64_t msgs = c.counter.msgs.load(std::memory_order_relaxed);

        auto& out = msg.channels[i];
        out.topic = c.topic;
        out.tx = (c.dir == Direction::TX);
        out.bytes_per_s = double(bytes - c.last_bytes) * inv_period;
        out.msgs_per_s = double(msgs - c.last_msgs) * inv_period;
        out.bytes_total = bytes;
        out.msgs_total = msgs;
        c.last_bytes = bytes;
        c.last_msgs = msgs;

        if (out.tx)
        {
            msg.tx_bytes_per_s += out.bytes_per_s;
            msg.tx_bytes_total += bytes;
        }
        else
        {
            msg.rx_bytes_per_s += out.bytes_per_s;
            msg.rx_bytes_total += bytes;
        }
    }
}
