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

#include <vector>
#include <cstddef>
#include <cstdint>

#include <multiscan_driver/multiscan_spec.hpp>


/* Lossy (QUANT_MM range step), context-modeled rANS codec for MS136 low-res
 * DenseBuffers. Each scan is independently decodable.
 *
 * Wire format (little-endian):
 *   u8  FORMAT_VERSION
 *   u8  QUANT_MM
 *   u16 TABLE_VERSION
 *   u16 rans_len
 *   u8  rans[rans_len]      (rANS stream, initial state first)
 *   u8  raw[...]            (raw low bits, LSB-first, zero padded)
 *
 * See ms136_codec_model.hpp for the model and tools/ms136_codec_train.cpp for
 * how the probability tables are generated. */
namespace util::ms136_codec
{

constexpr uint8_t FORMAT_VERSION = 1;
constexpr size_t HEADER_BYTES = 6;

/* Encode a dense buffer and append the result to `out`. Cells whose range
 * bits are zero are treated as null (their reflector bit is dropped). */
void encodeScan(const ms136::redux::DenseBuffer& dense, std::vector<uint8_t>& out);

/* Decode one scan. Returns false (leaving `dense` unspecified) if the payload
 * is malformed, truncated, or was produced with a different format version,
 * quantization step, or probability table version. */
bool decodeScan(
    const uint8_t* data,
    size_t len,
    ms136::redux::DenseBuffer& dense);

};  // namespace util::ms136_codec
