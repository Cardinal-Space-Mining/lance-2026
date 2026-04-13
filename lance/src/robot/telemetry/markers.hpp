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

#pragma once

#include <vector>
#include <string_view>

#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "util/ros_utils.hpp"

#include "robot/model/geometry.hpp"


namespace lance
{

class MarkerManager : public util::UsingRosAliases
{
public:
    using MarkerMsg = visualization_msgs::msg::Marker;
    using MarkerArrayMsg = visualization_msgs::msg::MarkerArray;

    struct MarkerGroup
    {
        using MarkerIter = MarkerArrayMsg::_markers_type::iterator;

        MarkerIter beg, end;

        size_t size() const;
        MarkerMsg& operator[](size_t i);

        MarkerGroup& setFrameId(std::string_view frame_id);
        MarkerGroup& setType(int32_t type);
        MarkerGroup& setDuration(RclDur dur);
        MarkerGroup& setColor(float r, float g, float b, float a);

        // MarkerGroup& updateMiningSweep(const geom::Pose2f& p, float dist);
        // MarkerGroup& updateMiningSweep(const geom::Pose3f& p, float dist);
    };

public:
    MarkerManager() = default;

public:
    size_t reserveGroup(size_t n, std::string_view ns = "default_ns");
    MarkerGroup getGroup(size_t i);

    void clearAll();
    void clearOutput();
    void addGroupToOutput(size_t i);

    const MarkerArrayMsg& getAllMarkers() const;
    const MarkerArrayMsg& getOutputMarkers() const;

    void pubAllMarkers(RclPubPtr<MarkerArrayMsg> pub, RclTime stamp);
    void pubOutputMarkers(
        RclPubPtr<MarkerArrayMsg> pub,
        RclTime stamp,
        bool clear = true);

protected:
    MarkerArrayMsg all_markers;
    MarkerArrayMsg output_markers;

    std::vector<size_t> alloc_indices;
};

};  // namespace lance
