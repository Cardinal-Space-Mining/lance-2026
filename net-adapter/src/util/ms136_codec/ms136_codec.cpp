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

#include "ms136_codec.hpp"

#include <array>
#include <limits>

#include "ms136_codec_model.hpp"
#include "ms136_codec_tables.hpp"


namespace util::ms136_codec
{

namespace
{

// --- Table preprocessing -----------------------------------------------------
/* The trained tables store a frequency f[s] for each symbol. Both coder
 * directions also need each symbol's cumulative start c[s] = f[0] + ... +
 * f[s-1], so that symbol s owns the slot range [c[s], c[s] + f[s]) of the
 * PROB_SCALE slots. A cumulative row has one extra entry, c[N] = PROB_SCALE,
 * so f[s] = cum[s + 1] - cum[s] and the decoder's search needs no bounds
 * check. Example: f = {3000, 1096} gives cum = {0, 3000, 4096}; slots
 * 0..2999 decode as symbol 0 and 3000..4095 as symbol 1.
 *
 * All of this runs at compile time. tablesValid() checks the only properties
 * the coder needs from the tables for correctness:
 *  - every frequency > 0: a zero-frequency symbol could never be encoded and
 *    would make the encoder divide by zero;
 *  - each row sums to exactly PROB_SCALE: otherwise slots would be missing
 *    or overlap. */

template<size_t N>
using CumRow = std::array<uint16_t, N + 1>;

template<size_t R, size_t N>
constexpr std::array<CumRow<N>, R> makeCum(
    const std::array<std::array<uint16_t, N>, R>& freqs)
{
    std::array<CumRow<N>, R> cum{};
    for (size_t r = 0; r < R; r++)
    {
        uint32_t c = 0;
        for (size_t s = 0; s < N; s++)
        {
            cum[r][s] = static_cast<uint16_t>(c);
            c += freqs[r][s];
        }
        cum[r][N] = static_cast<uint16_t>(c);
    }
    return cum;
}

template<size_t R, size_t N>
constexpr bool tablesValid(const std::array<std::array<uint16_t, N>, R>& freqs)
{
    for (const auto& row : freqs)
    {
        uint32_t sum = 0;
        for (uint16_t f : row)
        {
            if (f == 0)
            {
                return false;
            }
            sum += f;
        }
        if (sum != PROB_SCALE)
        {
            return false;
        }
    }
    return true;
}

static_assert(
    tablesValid(VALID_FREQS),
    "VALID_FREQS rows must be > 0 and sum to PROB_SCALE");
static_assert(
    tablesValid(REFL_FREQS),
    "REFL_FREQS rows must be > 0 and sum to PROB_SCALE");
static_assert(
    tablesValid(RANGE_FREQS),
    "RANGE_FREQS rows must be > 0 and sum to PROB_SCALE");

constexpr auto VALID_CUM = makeCum(VALID_FREQS);
constexpr auto REFL_CUM = makeCum(REFL_FREQS);
constexpr auto RANGE_CUM = makeCum(RANGE_FREQS);


// --- rANS (32-bit state, byte-wise renormalization) --------------------------
/* rANS ("range asymmetric numeral systems", J. Duda, 2009) packs a sequence of
 * symbols into one growing integer state x. This is the common byte-wise
 * variant with a 32-bit state, as in F. Giesen's public-domain rans_byte.h.
 *
 * Let M = PROB_SCALE, and let symbol s have frequency f and cumulative start
 * c (see above), so P(s) = f / M.
 *
 *   encode s:   x' = (x / f) * M + (x % f) + c
 *
 *   decode:     slot = x' % M        s = the symbol with c <= slot < c + f
 *               x    = f * (x' / M) + (slot - c)
 *
 * Why it inverts: x' / M gives back x / f and slot - c gives back x % f,
 * which together rebuild x exactly. Encoding multiplies x by about
 * M / f = 1 / P(s), so x grows by log2(1 / P(s)) bits per symbol: exactly
 * the ideal code length, fractional bits included. Likely symbols barely
 * grow x; unlikely ones grow it a lot.
 *
 * Two practical wrinkles:
 *
 * 1. It's a stack. Decoding pops the most recently encoded symbol first. The
 *    decoder has to go forward, since it needs earlier points to compute the
 *    contexts. So the encoder records symbols forward (RansEncoder::put),
 *    then encodes them last-to-first (RansEncoder::finish).
 *
 * 2. Renormalization keeps x in 32 bits. Between symbols, x stays in
 *    [RANS_L, 256 * RANS_L) = [2^23, 2^31).
 *     - Encoder: before encoding s, while x >= x_max = (RANS_L / M) * 256 * f
 *       (= 2^19 * f), emit x's low byte and shift it out. Then x / f < 2^19,
 *       so x' < 2^19 * M = 2^31. Also x >= x_max / 256 makes x' >= 2^23.
 *       Either way, x' is back in range.
 *     - Decoder: after decoding s, while x < RANS_L, shift the next byte in.
 *       This exactly undoes the bytes the encoder emitted for that step.
 *    Because the encoder runs backwards, it fills its byte buffer from the
 *    end toward the front, and the decoder reads front to back. Last of all,
 *    the encoder writes its final x (4 bytes) at the front, which becomes the
 *    decoder's starting state.
 *
 * A free integrity check: the encoder starts from x = RANS_L, so decoding a
 * well-formed stream ends with x == RANS_L exactly when its last byte is
 * consumed (RansDecoder::finishedCleanly). A corrupted rANS byte derails every
 * symbol after it and in practice never satisfies this (0 of 2000 random bit
 * flips in the tests).
 *
 * Output bound: one encode step emits at most 2 bytes. The worst case is
 * f = 1: x < 2^31 must be shifted below x_max = 2^19, which takes two byte
 * shifts. So a scan's stream is at most 2 bytes per symbol plus the 4-byte
 * state, which always fits the u16 rans_len header field. */

constexpr uint32_t RANS_L = 1U << 23;  // lower bound of the normalized state

/* At most 3 symbols per cell: valid bit, reflector bit, range size class */
constexpr size_t MAX_SYMBOLS = 3 * NUM_CELLS;
constexpr size_t MAX_RANS_BYTES = 2 * MAX_SYMBOLS + 4;
static_assert(PROB_BITS == 12);
static_assert(MAX_RANS_BYTES <= std::numeric_limits<uint16_t>::max());

struct SymbolRef
{
    uint16_t start;
    uint16_t freq;
};

/* rANS encodes in reverse: put() records symbols in coding order, and
 * finish() encodes them last-to-first so the decoder can read forward. */
class RansEncoder
{
public:
    RansEncoder() { syms.reserve(MAX_SYMBOLS); }

    /* cum: cumulative frequencies of one context row (size = n_symbols + 1) */
    template<size_t M>
    inline void put(const std::array<uint16_t, M>& cum, uint32_t s)
    {
        syms.push_back(
            SymbolRef{cum[s], static_cast<uint16_t>(cum[s + 1] - cum[s])});
    }

    /* Append the finished stream to out, return number of bytes written */
    size_t finish(std::vector<uint8_t>& out) const
    {
        // Worst-case-sized stack buffer (~30 KB): no allocation per scan.
        // Filled from the end toward the front (see rANS notes above).
        std::array<uint8_t, MAX_RANS_BYTES> buf;
        uint8_t* const end = buf.data() + buf.size();
        uint8_t* ptr = end;
        uint32_t x = RANS_L;  // known start state, checked by the decoder
        for (auto it = syms.rbegin(); it != syms.rend(); ++it)
        {
            const uint32_t freq = it->freq;
            // renormalize: shift out bytes until encoding fits in 31 bits
            const uint32_t x_max = ((RANS_L >> PROB_BITS) << 8) * freq;
            while (x >= x_max)
            {
                *--ptr = static_cast<uint8_t>(x & 0xFF);
                x >>= 8;
            }
            // x' = (x / f) * M + (x % f) + c
            x = ((x / freq) << PROB_BITS) + (x % freq) + it->start;
        }
        // final state goes in front: it is the decoder's initial state
        ptr -= 4;
        ptr[0] = static_cast<uint8_t>(x);
        ptr[1] = static_cast<uint8_t>(x >> 8);
        ptr[2] = static_cast<uint8_t>(x >> 16);
        ptr[3] = static_cast<uint8_t>(x >> 24);

        out.insert(out.end(), ptr, end);
        return static_cast<size_t>(end - ptr);
    }

private:
    std::vector<SymbolRef> syms;
};

/* Decodes forward. Every byte read is bounds-checked. After the first
 * failure `ok` stays false, so every later call fails immediately. On hostile
 * input the state arithmetic may wrap, which is well-defined for unsigned
 * integers: it produces garbage symbols that the caller's range checks or the
 * final-state check reject. */
class RansDecoder
{
public:
    RansDecoder(const uint8_t* begin, const uint8_t* end) : ptr{begin}, end{end}
    {
        if (end - begin >= 4)
        {
            x = static_cast<uint32_t>(ptr[0]) |
                (static_cast<uint32_t>(ptr[1]) << 8) |
                (static_cast<uint32_t>(ptr[2]) << 16) |
                (static_cast<uint32_t>(ptr[3]) << 24);
            ptr += 4;
            ok = x >= RANS_L;  // the encoder never produces a smaller state
        }
    }

    template<size_t M>
    inline bool get(const std::array<uint16_t, M>& cum, uint32_t& s)
    {
        if (!ok)
        {
            return false;
        }
        // slot = x % M identifies the symbol
        const uint32_t c = x & (PROB_SCALE - 1);
        // Linear search: rows have at most 16 symbols and nearly all hits are
        // in the first few (small size classes), so this beats a 4096-entry
        // slot -> symbol lookup table per context.
        s = 0;
        while (cum[s + 1] <= c)  // cum[M - 1] == PROB_SCALE > c terminates this
        {
            s++;
        }
        // x = f * (x' / M) + (slot - c)
        x = (cum[s + 1] - cum[s]) * (x >> PROB_BITS) + c - cum[s];
        // renormalize: pull bytes back in, undoing the encoder's output
        while (x < RANS_L)
        {
            if (ptr == end)
            {
                return (ok = false);
            }
            x = (x << 8) | *ptr++;
        }
        return true;
    }

    /* A well-formed stream ends exactly on its last byte, back in the
     * encoder's initial state RANS_L */
    bool finishedCleanly() const { return ok && ptr == end && x == RANS_L; }

private:
    const uint8_t* ptr;
    const uint8_t* const end;
    uint32_t x = 0;
    bool ok = false;
};


// --- Raw bit stream (LSB-first) ----------------------------------------------
/* The low bits of range residuals (see "Residual mapping" in
 * ms136_codec_model.hpp) bypass rANS. They are close to uniform noise, so
 * entropy coding them would gain nothing. Also, with 12-bit probabilities
 * rANS can code at most 12 bits per symbol. Instead they go in a plain bit
 * stream appended after the rANS bytes.
 *
 * Packing is LSB-first: the first value written occupies the lowest bits of
 * the first byte. A 64-bit accumulator holds pending bits; at most 7 are
 * pending when a value of at most 16 bits is added, so it never overflows.
 *
 * The encoder writes raw values during its forward pass, and the decoder reads
 * them in the same order, interleaved with its rANS reads. So neither stream
 * needs position markers; the only boundary is rans_len in the header. */

class BitWriter
{
public:
    explicit BitWriter(std::vector<uint8_t>& out) : out{out} {}

    inline void put(uint32_t v, uint32_t bits)
    {
        acc |= static_cast<uint64_t>(v) << n;
        n += bits;
        while (n >= 8)
        {
            out.push_back(static_cast<uint8_t>(acc));
            acc >>= 8;
            n -= 8;
        }
    }
    void flush()
    {
        if (n)
        {
            out.push_back(static_cast<uint8_t>(acc));
        }
        acc = 0;
        n = 0;
    }

private:
    std::vector<uint8_t>& out;
    uint64_t acc = 0;
    uint32_t n = 0;
};

class BitReader
{
public:
    BitReader(const uint8_t* begin, const uint8_t* end) : ptr{begin}, end{end}
    {
    }

    inline bool get(uint32_t bits, uint32_t& v)
    {
        while (n < bits)
        {
            if (ptr == end)
            {
                return false;
            }
            acc |= static_cast<uint64_t>(*ptr++) << n;
            n += 8;
        }
        v = static_cast<uint32_t>(acc & ((1ULL << bits) - 1));
        acc >>= bits;
        n -= bits;
        return true;
    }

    /* All bytes consumed, and the final padding bits (zeros from
     * BitWriter::flush) are still zero. Anything else means corruption. */
    bool finishedCleanly() const { return ptr == end && acc == 0; }

private:
    const uint8_t* ptr;
    const uint8_t* const end;
    uint64_t acc = 0;
    uint32_t n = 0;
};


inline void writeU16(std::vector<uint8_t>& out, size_t at, uint16_t v)
{
    out[at] = static_cast<uint8_t>(v);
    out[at + 1] = static_cast<uint8_t>(v >> 8);
}
inline uint16_t readU16(const uint8_t* p)
{
    return static_cast<uint16_t>(p[0] | (p[1] << 8));
}

}  // namespace


/* Encoding happens in two passes:
 *  1. Forward over all cells in coding order: compute each context exactly as
 *     the decoder will, record the rANS symbols, and write raw bits directly.
 *  2. RansEncoder::finish() runs the actual rANS encode, backwards.
 * The output is then assembled as header | rANS bytes | raw bytes. */
void encodeScan(
    const ms136::redux::DenseBuffer& dense,
    std::vector<uint8_t>& out)
{
    // Quantize up front. Contexts must be computed from quantized values,
    // because that is all the decoder will ever see. A reflector bit on a
    // null cell is dropped, since the format can't represent it.
    std::array<uint16_t, NUM_CELLS> q;
    std::array<uint8_t, NUM_CELLS> f;
    for (size_t i = 0; i < NUM_CELLS; i++)
    {
        q[i] = quantizeRange(ms136::redux::getRangeMMeters(dense[i]));
        f[i] = (q[i] && ms136::redux::getReflector(dense[i])) ? 1 : 0;
    }

    RansEncoder rans;
    std::vector<uint8_t> raw;
    raw.reserve(NUM_CELLS * 2);
    BitWriter bits{raw};

    uint16_t last = 0;  // last valid q in coding order (fallback predictor)
    for (size_t l = 0; l < NUM_LAYERS; l++)
    {
        for (size_t a = 0; a < LAYER_WIDTH; a++)
        {
            const size_t i = l * LAYER_WIDTH + a;
            const PointContext c =
                computeContext(q.data(), f.data(), l, a, last);
            const uint16_t v = q[i];

            // 1. valid bit
            rans.put(VALID_CUM[c.valid_ctx], v ? 1 : 0);
            if (!v)
            {
                continue;
            }
            // 2. reflector bit
            rans.put(REFL_CUM[c.refl_ctx], f[i]);
            // 3. range: size class of the zigzag residual (rANS), then the
            //    bits below its leading 1 (raw). k = 0 or 1 has no raw bits.
            if (c.pred)
            {
                const uint32_t u = zigzag(
                    static_cast<int32_t>(v) - static_cast<int32_t>(c.pred));
                const uint32_t k = sizeClass(u);
                rans.put(RANGE_CUM[c.range_ctx], k);
                if (k > 1)
                {
                    bits.put(u - (1U << (k - 1)), k - 1);
                }
            }
            else
            {
                // first valid point of the scan: nothing to predict from
                bits.put(v, RAW_RANGE_BITS);
            }
            last = v;
        }
    }
    bits.flush();

    // Header first. rans_len is only known after finish(), so it is patched
    // in afterwards.
    const size_t header_at = out.size();
    out.resize(header_at + HEADER_BYTES);
    out[header_at] = FORMAT_VERSION;
    out[header_at + 1] = static_cast<uint8_t>(QUANT_MM);
    writeU16(out, header_at + 2, TABLE_VERSION);
    const size_t rans_len = rans.finish(out);
    writeU16(out, header_at + 4, static_cast<uint16_t>(rans_len));
    out.insert(out.end(), raw.begin(), raw.end());
}

/* Mirror image of encodeScan()'s forward pass. Every read is bounds-checked
 * and every decoded value is range-checked, so arbitrary input bytes can only
 * make this return false; it never reads out of bounds. It stops at the first
 * inconsistency. There is no checksum: a flipped bit in the raw region gives a
 * slightly wrong range rather than an error. Transport integrity is left to
 * zenoh. */
bool decodeScan(
    const uint8_t* data,
    size_t len,
    ms136::redux::DenseBuffer& dense)
{
    if (!data || len < HEADER_BYTES)
    {
        return false;
    }
    if (data[0] != FORMAT_VERSION || data[1] != QUANT_MM ||
        readU16(data + 2) != TABLE_VERSION)
    {
        return false;
    }
    const size_t rans_len = readU16(data + 4);
    if (HEADER_BYTES + rans_len > len)
    {
        return false;
    }
    const uint8_t* const rans_begin = data + HEADER_BYTES;
    const uint8_t* const raw_begin = rans_begin + rans_len;
    RansDecoder rans{rans_begin, raw_begin};
    BitReader bits{raw_begin, data + len};

    // Decoded quantized values, rebuilt cell by cell so computeContext() sees
    // exactly what the encoder saw. It never reads cells that haven't been
    // decoded yet; zero-init only keeps this deterministic.
    std::array<uint16_t, NUM_CELLS> q{};
    std::array<uint8_t, NUM_CELLS> f{};
    uint16_t last = 0;
    for (size_t l = 0; l < NUM_LAYERS; l++)
    {
        for (size_t a = 0; a < LAYER_WIDTH; a++)
        {
            const size_t i = l * LAYER_WIDTH + a;
            const PointContext c =
                computeContext(q.data(), f.data(), l, a, last);

            uint32_t valid = 0, refl = 0;
            if (!rans.get(VALID_CUM[c.valid_ctx], valid))
            {
                return false;
            }
            if (!valid)
            {
                dense[i] = 0;
                continue;
            }
            if (!rans.get(REFL_CUM[c.refl_ctx], refl))
            {
                return false;
            }

            uint32_t v = 0;
            if (c.pred)
            {
                uint32_t k = 0, low = 0;
                if (!rans.get(RANGE_CUM[c.range_ctx], k) ||
                    (k > 1 && !bits.get(k - 1, low)))
                {
                    return false;
                }
                // restore the implied leading 1 (k = 1 -> u = 1, k = 0 -> 0)
                const uint32_t u = k ? ((1U << (k - 1)) | low) : 0;
                const int32_t val = static_cast<int32_t>(c.pred) + unzigzag(u);
                // The encoder never produces values outside [1, MAX_QRANGE].
                // Seeing one means corruption. A 0 here would also break the
                // contexts of later cells, which treat 0 as "null".
                if (val < 1 || val > static_cast<int32_t>(MAX_QRANGE))
                {
                    return false;
                }
                v = static_cast<uint32_t>(val);
            }
            else
            {
                if (!bits.get(RAW_RANGE_BITS, v) || v < 1 || v > MAX_QRANGE)
                {
                    return false;
                }
            }

            q[i] = static_cast<uint16_t>(v);
            f[i] = static_cast<uint8_t>(refl);
            last = q[i];
            dense[i] = static_cast<uint16_t>(
                dequantizeRange(q[i]) |
                (refl ? ms136::redux::REFLECTOR_MASK : 0));
        }
    }

    // Both streams must end exactly where the encoder ended them
    return rans.finishedCleanly() && bits.finishedCleanly();
}

};  // namespace util::ms136_codec
