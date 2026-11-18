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

// --- Table preprocessing -------------------------------------------------------

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

static_assert(tablesValid(VALID_FREQS), "VALID_FREQS rows must be > 0 and sum to PROB_SCALE");
static_assert(tablesValid(REFL_FREQS), "REFL_FREQS rows must be > 0 and sum to PROB_SCALE");
static_assert(tablesValid(RANGE_FREQS), "RANGE_FREQS rows must be > 0 and sum to PROB_SCALE");

constexpr auto VALID_CUM = makeCum(VALID_FREQS);
constexpr auto REFL_CUM = makeCum(REFL_FREQS);
constexpr auto RANGE_CUM = makeCum(RANGE_FREQS);


// --- rANS (32-bit state, byte-wise renormalization) ----------------------------

constexpr uint32_t RANS_L = 1U << 23;  // lower bound of the normalized state

/* Each symbol emits at most 2 bytes with PROB_BITS = 12 */
constexpr size_t MAX_SYMBOLS = 3 * NUM_CELLS;
constexpr size_t MAX_RANS_BYTES = 2 * MAX_SYMBOLS + 4;
static_assert(PROB_BITS == 12);
static_assert(MAX_RANS_BYTES <= std::numeric_limits<uint16_t>::max());

struct SymbolRef
{
    uint16_t start;
    uint16_t freq;
};

/* rANS encodes in reverse: collect symbols forward, then emit backwards so
 * the decoder can read forward. */
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
        std::array<uint8_t, MAX_RANS_BYTES> buf;
        uint8_t* const end = buf.data() + buf.size();
        uint8_t* ptr = end;
        uint32_t x = RANS_L;
        for (auto it = syms.rbegin(); it != syms.rend(); ++it)
        {
            const uint32_t freq = it->freq;
            const uint32_t x_max = ((RANS_L >> PROB_BITS) << 8) * freq;
            while (x >= x_max)
            {
                *--ptr = static_cast<uint8_t>(x & 0xFF);
                x >>= 8;
            }
            x = ((x / freq) << PROB_BITS) + (x % freq) + it->start;
        }
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
            ok = x >= RANS_L;
        }
    }

    template<size_t M>
    inline bool get(const std::array<uint16_t, M>& cum, uint32_t& s)
    {
        if (!ok)
        {
            return false;
        }
        const uint32_t c = x & (PROB_SCALE - 1);
        s = 0;
        while (cum[s + 1] <= c)  // cum[M - 1] == PROB_SCALE > c terminates this
        {
            s++;
        }
        x = (cum[s + 1] - cum[s]) * (x >> PROB_BITS) + c - cum[s];
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

    /* A well-formed stream ends exactly on its last byte in the initial state */
    bool finishedCleanly() const { return ok && ptr == end && x == RANS_L; }

private:
    const uint8_t* ptr;
    const uint8_t* const end;
    uint32_t x = 0;
    bool ok = false;
};


// --- Raw bit stream (LSB-first) ------------------------------------------------

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
    BitReader(const uint8_t* begin, const uint8_t* end) : ptr{begin}, end{end} {}

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

    /* All bytes consumed and the final padding bits are zero */
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


void encodeScan(const ms136::redux::DenseBuffer& dense, std::vector<uint8_t>& out)
{
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

    uint16_t last = 0;
    for (size_t l = 0; l < NUM_LAYERS; l++)
    {
        for (size_t a = 0; a < LAYER_WIDTH; a++)
        {
            const size_t i = l * LAYER_WIDTH + a;
            const PointContext c =
                computeContext(q.data(), f.data(), l, a, last);
            const uint16_t v = q[i];

            rans.put(VALID_CUM[c.valid_ctx], v ? 1 : 0);
            if (!v)
            {
                continue;
            }
            rans.put(REFL_CUM[c.refl_ctx], f[i]);
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
                bits.put(v, RAW_RANGE_BITS);
            }
            last = v;
        }
    }
    bits.flush();

    const size_t header_at = out.size();
    out.resize(header_at + HEADER_BYTES);
    out[header_at] = FORMAT_VERSION;
    out[header_at + 1] = static_cast<uint8_t>(QUANT_MM);
    writeU16(out, header_at + 2, TABLE_VERSION);
    const size_t rans_len = rans.finish(out);
    writeU16(out, header_at + 4, static_cast<uint16_t>(rans_len));
    out.insert(out.end(), raw.begin(), raw.end());
}

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
                const uint32_t u = k ? ((1U << (k - 1)) | low) : 0;
                const int32_t val = static_cast<int32_t>(c.pred) + unzigzag(u);
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

    return rans.finishedCleanly() && bits.finishedCleanly();
}

};  // namespace util::ms136_codec
