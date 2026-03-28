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
*               x$$$$$X ;XXXXXXXXXXX+ :xXXXXXX+   .;$$$$$$                   *
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
#include <cstdint>

#include <zstd.h>

namespace util
{

/* Compresses buf in-place using zstd. The zstd frame header stores the
 * original size, so no manual prefix is needed.
 *
 * level follows zstd conventions: 1 = fastest, 3 = default (good balance),
 * 6+ = better ratio at increasing CPU cost. */
inline bool zstdCompress(std::vector<uint8_t>& buf, int level = ZSTD_CLEVEL_DEFAULT)
{
    if (buf.empty())
    {
        return true;
    }

    const size_t dst_cap = ZSTD_compressBound(buf.size());
    std::vector<uint8_t> tmp(dst_cap);

    const size_t compressed =
        ZSTD_compress(tmp.data(), dst_cap, buf.data(), buf.size(), level);

    if (ZSTD_isError(compressed))
    {
        return false;
    }

    tmp.resize(compressed);
    buf = std::move(tmp);
    return true;
}

/* Decompresses buf in-place. Reads original size from the zstd frame header. */
inline bool zstdDecompress(std::vector<uint8_t>& buf)
{
    if (buf.empty())
    {
        return true;
    }

    const unsigned long long orig_size =
        ZSTD_getFrameContentSize(buf.data(), buf.size());

    if (orig_size == ZSTD_CONTENTSIZE_ERROR ||
        orig_size == ZSTD_CONTENTSIZE_UNKNOWN)
    {
        return false;
    }

    if (orig_size == 0)
    {
        buf.clear();
        return true;
    }

    std::vector<uint8_t> tmp(static_cast<size_t>(orig_size));

    const size_t result =
        ZSTD_decompress(tmp.data(), static_cast<size_t>(orig_size), buf.data(), buf.size());

    if (ZSTD_isError(result))
    {
        return false;
    }

    buf = std::move(tmp);
    return true;
}

}  // namespace util
