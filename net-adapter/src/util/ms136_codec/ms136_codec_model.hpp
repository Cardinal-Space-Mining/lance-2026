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


/* =============================================================================
 *  MS136 scan codec: the model (what gets coded, and with which probabilities)
 * =============================================================================
 *
 * Read ms136_codec.hpp first for the big picture. This file defines
 * everything that must be identical in the encoder, the decoder and the
 * offline trainer (tools/ms136_codec_train.cpp). ms136_codec.cpp only turns
 * the symbols defined here into bytes.
 *
 * If you change anything in computeContext() or the residual mapping, you
 * must retrain the tables and bump TABLE_VERSION / FORMAT_VERSION. Decoding
 * stays correct as long as both ends are built from the same sources, but
 * compression silently gets worse when the tables no longer match the model.
 *
 *
 * --- Coding order and causality ----------------------------------------------
 *
 * Cells are visited layer by layer, starting from dense layer 0 (lowest
 * elevation), and within a layer by azimuth index a = 0 .. 359. When cell X is
 * coded, every cell before it is already known to the decoder:
 *
 *                     a-1    a    a+1
 *       layer l        L     X     .      L  = left neighbour, same layer
 *       layer l-1      UL    U     UR     the whole previous layer is known
 *
 * "U" is not simply the same array index one layer down: each layer starts at
 * a different azimuth (AZIMUTH_OFFSET_LUT). Layer 7, for example, starts
 * ~180 deg away from layer 6. PREV_LAYER_OFFSET[l] is the index shift that
 * lands on the azimuthally closest beam in layer l-1. The shift wraps around
 * the 360 cells, which is safe because the whole previous layer is known.
 *
 * Within a layer, L is not taken across the 359 -> 0 wrap: for a == 0, cell
 * 359 of the same layer hasn't been decoded yet.
 *
 *
 * --- What is coded per cell --------------------------------------------------
 *
 *   1. valid bit              always            binary, 16 contexts
 *   2. reflector bit          if valid          binary, 9 contexts
 *   3. range, as a residual   if valid          size class under 450
 *      against a prediction                     contexts, then raw low bits
 *
 * Typical cost on the competition bags: ~146 B/scan for all valid bits,
 * ~24 B for reflectors, ~3.1 KB for ranges. That is ~6.3 bits per valid
 * point, nearly all of it in the range residuals, which are dominated by
 * sensor noise that no model can predict. */
namespace util::ms136_codec
{

// --- Layout ------------------------------------------------------------------
constexpr size_t NUM_LAYERS = ms136::NUM_LR_LAYERS;
constexpr size_t LAYER_WIDTH = ms136::POINTS_PER_LR_LAYER;
constexpr size_t NUM_CELLS = ms136::POINTS_PER_LR_SCAN;
static_assert(NUM_CELLS == NUM_LAYERS * LAYER_WIDTH);

// --- Quantization ------------------------------------------------------------
/* Range quantization step: q = round(mm / QUANT_MM), with q == 0 reserved for
 * "no return". With QUANT_MM = 2, valid q is in [1, 16384] and needs 15 bits.
 *
 * Why 2 mm: measured sensor noise is ~3.6 mm (1 sigma; 3.2-4.7 mm depending on
 * range, ~0.5 mm more with the trencher running). Uniform rounding to a step
 * of Q adds Q / sqrt(12) RMS error, 0.58 mm here, so the combined error grows
 * from 3.60 to 3.65 mm (+1.3%). Each doubling of the step saves ~1 bit per
 * point, because the bits removed are noise. 5 mm would still add only ~8%.
 *
 * dequantizeRange() clamps to RANGE_MASK: q = 16384 would become 32768 mm,
 * which would spill into the reflector bit of the DenseBuffer format. */
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

// --- Model dimensions --------------------------------------------------------
/* Probabilities are integer frequencies out of PROB_SCALE = 4096: P(s) =
 * freq[s] / 4096, each freq >= 1, and each table row sums to exactly 4096.
 * Consequences:
 *  - The cheapest symbol costs -log2(4095/4096) ~ 0.0004 bit and the most
 *    expensive costs 12 bits, which bounds the damage of a badly mispredicted
 *    symbol.
 *  - 12 bits is the usual choice for a 32-bit rANS state with byte-wise
 *    renormalization (see ms136_codec.cpp): the arithmetic fits in 32 bits
 *    and each symbol emits at most 2 bytes. */
constexpr uint32_t PROB_BITS = 12;
constexpr uint32_t PROB_SCALE = 1U << PROB_BITS;

/* Number of contexts, i.e. separate probability distributions, per symbol
 * kind. How each index is formed is documented in computeContext(). */
constexpr size_t NUM_VALID_CTX = 16;     // 4 binary neighbour flags
constexpr size_t NUM_REFL_CTX = 9;       // 3 states x 3 states
constexpr size_t NUM_ACT_BUCKETS = 15;   // 0 = unknown, 1..14 = log2 classes
constexpr size_t NUM_DIST_BUCKETS = 15;  // floor(log2(pred)), capped
constexpr size_t NUM_RANGE_CTX = NUM_ACT_BUCKETS * NUM_DIST_BUCKETS * 2;
/* Range symbols are size classes 0..15: with q and pred both in
 * [1, MAX_QRANGE], |residual| < MAX_QRANGE, so zigzag(residual) <
 * 2 * MAX_QRANGE = 2^15 and its bit length is <= RAW_RANGE_BITS (15). */
constexpr size_t NUM_RANGE_SYM = RAW_RANGE_BITS + 1;

/* One table row = the distribution for one context */
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

/* PREV_LAYER_AZ_SHIFT (multiscan_spec.hpp) can be negative; this is the same
 * shift as a non-negative offset mod LAYER_WIDTH, so U's index is just
 * (a + offset), minus LAYER_WIDTH if it went past the end. For example, layer
 * 1 has shift -11: cell a of layer 1 points the same way as cell a - 11 of
 * layer 0. */
constexpr std::array<size_t, NUM_LAYERS> PREV_LAYER_OFFSET =
    detail::computePrevLayerOffsets();

struct PointContext
{
    uint16_t pred;       // predicted quantized range, 0 = no prediction
    uint8_t valid_ctx;   // [0, NUM_VALID_CTX)
    uint8_t refl_ctx;    // [0, NUM_REFL_CTX)
    uint16_t range_ctx;  // [0, NUM_RANGE_CTX)
};

/* Compute the prediction and the three context indices for cell (l, a),
 * using only causal data (see the diagram at the top of this file).
 *
 *  q    : quantized ranges (0 = invalid); cells before (l, a) must be final.
 *         Cells at or after (l, a) are never read.
 *  refl : reflector bits (0/1) for the same cells
 *  last : last valid quantized range before (l, a) in coding order
 *         (0 = none yet)
 *
 * Valid-bit context (16): four "is it valid?" flags,
 *     bit 0: L     bit 1: U     bit 2: UR     bit 3: any earlier valid point
 *   Nulls come in runs (sky, beyond max range, the robot's own body), so
 *   valid neighbours make "valid" very likely and vice versa. On the bags this
 *   costs ~0.2 bit per cell on average.
 *
 * Reflector context (9): each of L and U is in one of three states, 0 = null,
 *   1 = valid non-reflector, 2 = valid reflector; ctx = state(L) * 3 +
 *   state(U). Reflective targets cover several adjacent beams, so a
 *   reflective neighbour makes this cell much more likely to be reflective.
 *
 * Range prediction: MED, the "median edge detector" from LOCO-I / JPEG-LS.
 *   With all of L, U and UL valid:
 *     UL >= max(L, U)  -> min(L, U)    edge: X is on the low side with
 *                                       whichever of L/U is smaller
 *     UL <= min(L, U)  -> max(L, U)    edge, X on the high side
 *     otherwise        -> L + U - UL   smooth: the value that makes the 2x2
 *                                       patch {UL, U, L, X} a plane
 *   (This equals the median of {L, U, L + U - UL}, hence the name.) It is
 *   cheap and handles both smooth ground and depth discontinuities. Without
 *   the full neighbourhood it falls back to L, then U, then `last`. Only the
 *   first valid point of a scan has no prediction at all; it is sent as
 *   RAW_RANGE_BITS raw bits.
 *
 * Range context (450) = (activity bucket, distance bucket, full flag):
 *   activity = how much the neighbours disagree, a local gradient magnitude:
 *     |L - UL| + |U - UL| with a full neighbourhood, |L - U| with only L and
 *     U, unknown otherwise. Flat patches give small residuals and edges give
 *     big ones. Bucketed as 1 + floor(log2(act + 1)), capped at 14, because
 *     the residual scale grows roughly in proportion to the activity.
 *     Bucket 0 = unknown.
 *   distance = floor(log2(pred)), capped at 14. Farther points are noisier,
 *     and the 1 deg beam spacing covers more ground (~17 cm at 10 m), so
 *     residuals grow with range.
 *   full = 1 if MED was used, 0 for a fallback. Fallback predictions are less
 *     reliable, so their residual distribution is different.
 *   Index = (activity * NUM_DIST_BUCKETS + distance) * 2 + full. */
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

// --- Residual mapping --------------------------------------------------------
/* A residual e = q - pred is coded in two parts:
 *
 * 1. zigzag maps signed to unsigned so small magnitudes get small values:
 *        e :  0  -1  +1  -2  +2  -3  +3 ...
 *        u :  0   1   2   3   4   5   6 ...
 *
 * 2. u is split into a size class k (its bit length), which is entropy coded
 *    under the range context, and the k - 1 bits below its leading 1, which
 *    are written raw. The leading 1 itself is implied by k:
 *
 *        u          k    raw bits                      e
 *        0          0    none                          0
 *        1          1    none                          -1
 *        2..3       2    1    (u - 2)                  +1, -2
 *        4..7       3    2    (u - 4)                  +2 .. -4
 *        8..15      4    3    (u - 8)                  +4 .. -8
 *        ...
 *        16384..    15   14
 *
 *    Example: e = +5 -> u = 10 = 0b1010 -> k = 4, raw = 0b010 (3 bits).
 *
 * Why split: the size class carries nearly all the predictable structure (how
 * big is the error likely to be here?), while the bits below the leading 1 are
 * close to uniformly random, i.e. sensor noise. Modelling them would grow the
 * tables a lot and save almost nothing. The same idea appears as Elias-gamma
 * codes and as JPEG's "category + extra bits". It keeps the range alphabet to
 * 16 symbols. */
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
