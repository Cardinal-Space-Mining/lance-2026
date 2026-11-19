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

#include <array>
#include <bit>
#include <cstddef>
#include <cstdint>

#include <multiscan_driver/multiscan_spec.hpp>


/* Shared probability-model definition for the MS136 scan codec.
 *
 * Everything here is used identically by the encoder, the decoder, and the
 * offline table trainer (tools/ms136_codec_train.cpp). Any change to the
 * context computation MUST be followed by retraining the tables and bumping
 * TABLE_VERSION, otherwise compression degrades (decoding stays correct as
 * long as encoder and decoder are built from the same sources).
 *
 * Points are coded in dense order (layer-major, by elevation). Each point
 * codes up to three things:
 *   1. valid bit            - binary symbol, NUM_VALID_CTX contexts
 *   2. reflector bit        - binary symbol, NUM_REFL_CTX contexts (valid only)
 *   3. quantized range      - residual vs. a MED (LOCO-I) prediction from the
 *                             left / below / below-left neighbors; the zigzag
 *                             residual's bit-length ("size class") is a symbol
 *                             under NUM_RANGE_CTX contexts and the remaining
 *                             low bits are written raw. If no neighbor is
 *                             available, RAW_RANGE_BITS raw bits are used. */
namespace util::ms136_codec
{

// --- Layout ------------------------------------------------------------------
constexpr size_t NUM_LAYERS = ms136::NUM_LR_LAYERS;
constexpr size_t LAYER_WIDTH = ms136::POINTS_PER_LR_LAYER;
constexpr size_t NUM_CELLS = ms136::POINTS_PER_LR_SCAN;
static_assert(NUM_CELLS == NUM_LAYERS * LAYER_WIDTH);

// --- Quantization ------------------------------------------------------------
/* Range quantization step. Measured sensor noise is ~3.6 mm (1 sigma), so a
 * 2 mm step adds ~0.58 mm RMS error (+1.3% total). */
constexpr uint16_t QUANT_MM = 2;
constexpr uint16_t MAX_QRANGE =
    (ms136::redux::RANGE_MASK + QUANT_MM / 2) / QUANT_MM;
constexpr uint32_t RAW_RANGE_BITS = std::bit_width(MAX_QRANGE);
static_assert(RAW_RANGE_BITS <= 16);

inline constexpr uint16_t quantizeRange(uint16_t mm)
{
    return mm ? static_cast<uint16_t>((mm + QUANT_MM / 2) / QUANT_MM) : 0;
}
inline constexpr uint16_t dequantizeRange(uint16_t q)
{
    const uint32_t mm = static_cast<uint32_t>(q) * QUANT_MM;
    return static_cast<uint16_t>(
        mm > ms136::redux::RANGE_MASK ? ms136::redux::RANGE_MASK : mm);
}

// --- Model dimensions ----------------------------------------------------------
constexpr uint32_t PROB_BITS = 12;
constexpr uint32_t PROB_SCALE = 1U << PROB_BITS;

constexpr size_t NUM_VALID_CTX = 16;
constexpr size_t NUM_REFL_CTX = 9;
constexpr size_t NUM_ACT_BUCKETS = 15;
constexpr size_t NUM_DIST_BUCKETS = 15;
constexpr size_t NUM_RANGE_CTX = NUM_ACT_BUCKETS * NUM_DIST_BUCKETS * 2;
/* zigzag residual < 2 * MAX_QRANGE  ->  bit length <= RAW_RANGE_BITS */
constexpr size_t NUM_RANGE_SYM = RAW_RANGE_BITS + 1;

using BinaryFreqs = std::array<uint16_t, 2>;
using RangeFreqs = std::array<uint16_t, NUM_RANGE_SYM>;

// --- Neighborhood ------------------------------------------------------------
namespace detail
{
inline constexpr std::array<size_t, NUM_LAYERS> computePrevLayerOffsets()
{
    std::array<size_t, NUM_LAYERS> off{};
    constexpr int W = static_cast<int>(LAYER_WIDTH);
    for (size_t l = 0; l < NUM_LAYERS; l++)
    {
        off[l] = static_cast<size_t>(
            ((ms136::redux::PREV_LAYER_AZ_SHIFT[l] % W) + W) % W);
    }
    return off;
}
inline constexpr uint32_t floorLog2(uint32_t v)
{
    return static_cast<uint32_t>(std::bit_width(v)) - 1;  // v > 0
}
};  // namespace detail

/* Non-negative azimuth index offset into the previous layer */
constexpr std::array<size_t, NUM_LAYERS> PREV_LAYER_OFFSET =
    detail::computePrevLayerOffsets();

struct PointContext
{
    uint16_t pred;       // predicted quantized range, 0 = no prediction
    uint8_t valid_ctx;   // [0, NUM_VALID_CTX)
    uint8_t refl_ctx;    // [0, NUM_REFL_CTX)
    uint16_t range_ctx;  // [0, NUM_RANGE_CTX)
};

/* Compute contexts and prediction for cell (l, a) from causal data only.
 *  q    : quantized ranges (0 = invalid); cells before (l, a) must be final
 *  refl : reflector bits (0/1) for the same cells
 *  last : last valid quantized range before (l, a) in coding order (0 = none) */
inline PointContext computeContext(
    const uint16_t* q,
    const uint8_t* refl,
    size_t l,
    size_t a,
    uint16_t last)
{
    const size_t i = l * LAYER_WIDTH + a;

    uint32_t L = 0, U = 0, UL = 0, UR = 0;
    uint32_t fl = 0, fu = 0;
    if (a > 0)
    {
        L = q[i - 1];
        fl = refl[i - 1];
    }
    if (l > 0)
    {
        const size_t base = (l - 1) * LAYER_WIDTH;
        size_t u = a + PREV_LAYER_OFFSET[l];
        if (u >= LAYER_WIDTH)
        {
            u -= LAYER_WIDTH;
        }
        U = q[base + u];
        fu = refl[base + u];
        if (a > 0)
        {
            UL = q[base + (u == 0 ? LAYER_WIDTH - 1 : u - 1)];
        }
        if (a + 1 < LAYER_WIDTH)
        {
            UR = q[base + (u + 1 == LAYER_WIDTH ? 0 : u + 1)];
        }
    }

    PointContext c;
    c.valid_ctx = static_cast<uint8_t>(
        (L > 0) | ((U > 0) << 1) | ((UR > 0) << 2) | ((last > 0) << 3));
    c.refl_ctx =
        static_cast<uint8_t>((L ? 1 + fl : 0) * 3 + (U ? 1 + fu : 0));

    const bool full = L && U && UL;
    uint32_t pred;
    int32_t act = -1;
    if (full)
    {
        const uint32_t mx = L > U ? L : U;
        const uint32_t mn = L > U ? U : L;
        pred = (UL >= mx) ? mn : (UL <= mn) ? mx : (L + U - UL);
        act = static_cast<int32_t>((L > UL ? L - UL : UL - L) +
                                   (U > UL ? U - UL : UL - U));
    }
    else
    {
        pred = L ? L : U ? U : last;
        if (L && U)
        {
            act = static_cast<int32_t>(L > U ? L - U : U - L);
        }
    }

    uint32_t act_b = 0;
    if (act >= 0)
    {
        const uint32_t lg = detail::floorLog2(static_cast<uint32_t>(act) + 1);
        act_b = 1 + (lg < NUM_ACT_BUCKETS - 2 ? lg : NUM_ACT_BUCKETS - 2);
    }
    uint32_t dist_b = 0;
    if (pred)
    {
        const uint32_t lg = detail::floorLog2(pred);
        dist_b = lg < NUM_DIST_BUCKETS - 1 ? lg : NUM_DIST_BUCKETS - 1;
    }

    c.pred = static_cast<uint16_t>(pred);
    c.range_ctx = static_cast<uint16_t>(
        (act_b * NUM_DIST_BUCKETS + dist_b) * 2 + (full ? 1 : 0));
    return c;
}

// --- Residual mapping ----------------------------------------------------------
inline constexpr uint32_t zigzag(int32_t e)
{
    return e >= 0 ? (static_cast<uint32_t>(e) << 1)
                  : ((static_cast<uint32_t>(-e) << 1) - 1);
}
inline constexpr int32_t unzigzag(uint32_t u)
{
    return (u & 1) ? -static_cast<int32_t>((u + 1) >> 1)
                   : static_cast<int32_t>(u >> 1);
}
/* Size class of a zigzag residual: 0 for u == 0, else bit length of u.
 * Class k > 0 is followed by (k - 1) raw bits: u - 2^(k-1). */
inline constexpr uint32_t sizeClass(uint32_t u)
{
    return static_cast<uint32_t>(std::bit_width(u));
}

};  // namespace util::ms136_codec
