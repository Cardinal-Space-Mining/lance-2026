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
*               x$$$$$X ;XXXXXXXXXXX+ :xXXXXXX+     .;$$$$$$                   *
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

#include "zstd_ctx.hpp"


namespace util
{

ZstdCompressor::ZstdCompressor() : ctx{ZSTD_createCCtx()} {}
ZstdCompressor::~ZstdCompressor() { ZSTD_freeCCtx(ctx); }

bool ZstdCompressor::compress(std::vector<uint8_t>& buf, int level)
{
    if (buf.empty())
    {
        return true;
    }

    const size_t dst_cap = ZSTD_compressBound(buf.size());
    std::vector<uint8_t> tmp(dst_cap);

    const size_t compressed = ZSTD_compressCCtx(
        ctx,
        tmp.data(),
        dst_cap,
        buf.data(),
        buf.size(),
        level);

    if (ZSTD_isError(compressed))
    {
        return false;
    }

    tmp.resize(compressed);
    buf = std::move(tmp);
    return true;
}


ZstdDecompressor::ZstdDecompressor() : ctx{ZSTD_createDCtx()} {}
ZstdDecompressor::~ZstdDecompressor() { ZSTD_freeDCtx(ctx); }

bool ZstdDecompressor::decompress(std::vector<uint8_t>& buf)
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

    const size_t result = ZSTD_decompressDCtx(
        ctx,
        tmp.data(),
        static_cast<size_t>(orig_size),
        buf.data(),
        buf.size());

    if (ZSTD_isError(result))
    {
        return false;
    }

    buf = std::move(tmp);
    return true;
}

};  // namespace util
