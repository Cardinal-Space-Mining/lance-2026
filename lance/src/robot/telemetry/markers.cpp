/*******************************************************************************
*   Copyright (C) 2025-2026 Cardinal Space Mining Club                         *
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

#include "markers.hpp"


namespace lance
{

size_t MarkerManager::MarkerGroup::size() const
{
    return this->end - this->beg;
}

MarkerManager::MarkerMsg& MarkerManager::MarkerGroup::operator[](size_t i)
{
    auto x = this->beg + i;
    return (x < this->end) ? *x : *this->beg;
}

MarkerManager::MarkerGroup& MarkerManager::MarkerGroup::setFrameIds(
    std::string_view frame_id)
{
    for (auto itr = this->beg; itr != this->end; itr++)
    {
        itr->header.frame_id = frame_id;
    }
    return *this;
}

MarkerManager::MarkerGroup& MarkerManager::MarkerGroup::setTypes(int32_t type)
{
    for (auto itr = this->beg; itr != this->end; itr++)
    {
        itr->type = type;
    }
    return *this;
}

MarkerManager::MarkerGroup& MarkerManager::MarkerGroup::setDurations(RclDur dur)
{
    for (auto itr = this->beg; itr != this->end; itr++)
    {
        itr->lifetime = dur;
    }
    return *this;
}



size_t MarkerManager::reserveGroup(size_t n, std::string_view ns)
{
    this->all_markers.markers.reserve(this->all_markers.markers.size() + n);
    for (size_t i = 0; i < n; i++)
    {
        auto& m = this->all_markers.markers.emplace_back();
        m.ns = ns;
        m.id = static_cast<int32_t>(this->all_markers.markers.size());
        m.action = MarkerMsg::ADD;
    }

    this->alloc_indices.push_back(this->all_markers.markers.size());

    return this->alloc_indices.size() - 1;
}

MarkerManager::MarkerGroup MarkerManager::getGroup(size_t i)
{
    if (i >= this->alloc_indices.size())
    {
        return MarkerGroup{
            .beg = this->all_markers.markers.end(),
            .end = this->all_markers.markers.end()};
    }

    const size_t b = i > 0 ? this->alloc_indices[i - 1] : 0;
    const size_t e = this->alloc_indices[i];

    return MarkerGroup{
        .beg = this->all_markers.markers.begin() + b,
        .end = this->all_markers.markers.begin() + e};
}

void MarkerManager::clearAll()
{
    this->all_markers.markers.clear();
    this->output_markers.markers.clear();
}

void MarkerManager::clearOutput() { this->output_markers.markers.clear(); }

void MarkerManager::addGroupToOutput(size_t i)
{
    if (i >= this->alloc_indices.size())
    {
        return;
    }

    MarkerGroup g = this->getGroup(i);
    this->output_markers.markers.insert(
        this->output_markers.markers.end(),
        g.beg,
        g.end);
}

const MarkerManager::MarkerArrayMsg& MarkerManager::getAllMarkers() const
{
    return this->all_markers;
}

const MarkerManager::MarkerArrayMsg& MarkerManager::getOutputMarkers() const
{
    return this->output_markers;
}

void MarkerManager::pubAllMarkers(RclPubPtr<MarkerArrayMsg>& pub, RclTime stamp)
{
    for (auto& m : this->all_markers.markers)
    {
        m.header.stamp = stamp;
    }

    pub->publish(this->all_markers);
}

void MarkerManager::pubOutputMarkers(
    RclPubPtr<MarkerArrayMsg>& pub,
    RclTime stamp,
    bool clear)
{
    for (auto& m : this->output_markers.markers)
    {
        m.header.stamp = stamp;
    }

    pub->publish(this->output_markers);

    if (clear)
    {
        this->output_markers.markers.clear();
    }
}

};  // namespace lance
