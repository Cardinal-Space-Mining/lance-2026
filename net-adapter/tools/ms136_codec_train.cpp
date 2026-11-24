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

/* Offline trainer for the MS136 scan codec probability tables.
 *
 * Input files are raw little-endian uint16 DenseBuffers (N x NUM_CELLS), as
 * produced by tools/ms136_extract_scans.py.
 *
 * Usage:
 *   ms136_codec_train [-o tables.hpp] [--version N] --train a.bin [b.bin ...]
 *                     [--eval c.bin ...]
 *
 * Writes the generated tables header (if -o is given) and prints the
 * estimated mean encoded size per scan for the training set and for any
 * --eval sets (cross-entropy under the quantized 12-bit tables, which rANS
 * achieves to within a few bytes).
 *
 * --- How training works ------------------------------------------------------
 *
 * The codec's "model" is just one probability distribution per context (see
 * ms136_codec_model.hpp). Training walks every scan exactly as the encoder
 * does and counts how often each symbol occurs in each context. The observed
 * frequency is the best estimate of the true probability, and coding with it
 * minimizes the expected size on data like the training data. There is
 * nothing iterative: one counting pass, then normalization into 12-bit
 * integer frequencies.
 *
 * Generalization: with ~500 small contexts and ~20k training scans, the
 * tables barely overfit. Tables trained on five bags and evaluated on the
 * sixth come within ~1.5% of tables trained on that bag itself. To check this
 * after retraining, run with one bag held out and pass it to --eval.
 *
 * Typical retraining run (ROS sourced, built with ./src/build.sh --tools,
 * i.e. -DNET_ADAPTER_BUILD_TOOLS=ON):
 *
 * Easiest: run tools/ms136_extract_scans.py with no arguments. Its GUI picks
 * bags, extracts (cached), trains, writes the header and optionally rebuilds
 * and runs the test. By hand:
 *
 *   1. For each bag directory BAG:
 *        tools/ms136_extract_scans.py BAG scans/BAG_NAME.bin --stride 5
 *   2. ms136_codec_train -o src/util/ms136_codec/ms136_codec_tables.hpp
 *          --version <TABLE_VERSION + 1> --train scans/a.bin scans/b.bin ...
 *   3. Rebuild and run test_ms136_codec. */

#include <array>
#include <cmath>
#include <cstdio>
#include <string>
#include <vector>
#include <fstream>
#include <cstdint>
#include <iostream>
#include <algorithm>

#include "../src/util/ms136_codec/ms136_codec_model.hpp"

using namespace util::ms136_codec;

namespace
{

template<size_t N>
using CountRow = std::array<double, N>;

struct Counts
{
    std::array<CountRow<2>, NUM_VALID_CTX> valid{};
    std::array<CountRow<2>, NUM_REFL_CTX> refl{};
    std::array<CountRow<NUM_RANGE_SYM>, NUM_RANGE_CTX> range{};
    double raw_bits = 0;
    size_t n_scans = 0;
};

using Scan = std::array<uint16_t, NUM_CELLS>;

std::vector<Scan> loadScans(const std::string& path)
{
    std::ifstream f(path, std::ios::binary | std::ios::ate);
    if (!f)
    {
        throw std::runtime_error("cannot open " + path);
    }
    const size_t bytes = static_cast<size_t>(f.tellg());
    if (bytes % sizeof(Scan))
    {
        throw std::runtime_error(path + ": size is not a multiple of a scan");
    }
    std::vector<Scan> scans(bytes / sizeof(Scan));
    f.seekg(0);
    f.read(reinterpret_cast<char*>(scans.data()), bytes);
    // input is little-endian; this tool assumes a little-endian host
    static_assert(std::endian::native == std::endian::little);
    return scans;
}

/* Visit every coded symbol of a scan exactly as the encoder emits them.
 *
 * MAINTENANCE: this mirrors the forward pass of encodeScan()
 * (ms136_codec.cpp). Both call the shared computeContext(), but the loop
 * itself is duplicated, so a change to what encodeScan() emits (order, which
 * symbols, raw-bit counts) must be made here too. If they drift apart, the
 * tables still decode correctly but no longer match the data, and
 * compression silently degrades. test_ms136_codec's size check would
 * catch it.
 *
 * on_bin(which, ctx, bit)    which: 0 = valid bit, 1 = reflector bit
 * on_range(ctx, k, raw_bits) ctx = -1 for the unpredicted first point */
template<typename VisitBin, typename VisitRange>
void traverse(const Scan& dense, VisitBin&& on_bin, VisitRange&& on_range)
{
    std::array<uint16_t, NUM_CELLS> q;
    std::array<uint8_t, NUM_CELLS> f;
    for (size_t i = 0; i < NUM_CELLS; i++)
    {
        q[i] = quantizeRange(dense[i] & ms136::redux::RANGE_MASK);
        f[i] = q[i] ? static_cast<uint8_t>(dense[i] >> 15) : 0;
    }
    uint16_t last = 0;
    for (size_t l = 0; l < NUM_LAYERS; l++)
    {
        for (size_t a = 0; a < LAYER_WIDTH; a++)
        {
            const size_t i = l * LAYER_WIDTH + a;
            const PointContext c = computeContext(q.data(), f.data(), l, a, last);
            const uint16_t v = q[i];
            on_bin(0, c.valid_ctx, v ? 1 : 0);
            if (v)
            {
                on_bin(1, c.refl_ctx, f[i]);
                if (c.pred)
                {
                    const uint32_t k = sizeClass(zigzag(
                        static_cast<int32_t>(v) - static_cast<int32_t>(c.pred)));
                    on_range(c.range_ctx, k, k ? k - 1 : 0);
                }
                else
                {
                    on_range(-1, 0, RAW_RANGE_BITS);
                }
                last = v;
            }
        }
    }
}

void accumulate(Counts& c, const std::vector<Scan>& scans)
{
    for (const Scan& s : scans)
    {
        traverse(
            s,
            [&](int which, size_t ctx, uint32_t bit)
            { (which == 0 ? c.valid[ctx] : c.refl[ctx])[bit] += 1; },
            [&](int ctx, uint32_t k, uint32_t raw)
            {
                if (ctx >= 0)
                {
                    c.range[static_cast<size_t>(ctx)][k] += 1;
                }
                c.raw_bits += raw;
            });
        c.n_scans++;
    }
}

/* Smoothed counts -> frequencies summing to PROB_SCALE, each >= 1.
 *
 * Smoothing (adding `alpha` to every count before normalizing) handles
 * symbols that never occurred in training. The raw estimate for those would be
 * probability 0, i.e. unencodable, so every symbol gets a small share.
 * alpha = 0.5 for the binary tables is the classic Krichevsky-Trofimov
 * estimator. The 16-symbol range rows use a smaller 0.05, because many
 * contexts are sparse and a larger alpha would noticeably flatten their
 * distributions. A context never seen at all ends up uniform.
 *
 * Rounding to integers: each frequency is round(p * 4096), at least 1. The
 * rounded values may not sum to exactly 4096, so the difference is added to
 * the most probable symbol. That symbol is always large enough to absorb it,
 * and a small change to a high probability costs the least. */
template<size_t N>
std::array<uint16_t, N> normalize(const CountRow<N>& counts, double alpha)
{
    double total = 0;
    for (double v : counts)
    {
        total += v + alpha;
    }
    std::array<uint16_t, N> freq{};
    int64_t sum = 0;
    for (size_t s = 0; s < N; s++)
    {
        const double p = (counts[s] + alpha) / total;
        const int64_t f = std::max<int64_t>(1, std::llround(p * PROB_SCALE));
        freq[s] = static_cast<uint16_t>(f);
        sum += f;
    }
    // fix rounding drift on the most probable symbol (always >= PROB_SCALE/N)
    const size_t top = static_cast<size_t>(
        std::max_element(freq.begin(), freq.end()) - freq.begin());
    freq[top] = static_cast<uint16_t>(
        static_cast<int64_t>(freq[top]) + (PROB_SCALE - sum));
    return freq;
}

struct Tables
{
    std::array<BinaryFreqs, NUM_VALID_CTX> valid;
    std::array<BinaryFreqs, NUM_REFL_CTX> refl;
    std::array<RangeFreqs, NUM_RANGE_CTX> range;
};

Tables buildTables(const Counts& c)
{
    Tables t;
    for (size_t i = 0; i < NUM_VALID_CTX; i++)
    {
        t.valid[i] = normalize(c.valid[i], 0.5);
    }
    for (size_t i = 0; i < NUM_REFL_CTX; i++)
    {
        t.refl[i] = normalize(c.refl[i], 0.5);
    }
    for (size_t i = 0; i < NUM_RANGE_CTX; i++)
    {
        t.range[i] = normalize(c.range[i], 0.05);
    }
    return t;
}

/* Mean estimated bytes per scan (payload + fixed codec overhead).
 *
 * This is the cross-entropy of the data under the quantized tables: each
 * modelled symbol costs -log2(freq / 4096) bits and each raw bit costs 1. rANS
 * gets within a fraction of a byte per scan of this, so the estimate predicts
 * real encodeScan() output closely (it matched to within 1 byte on the test
 * fixture). That makes --eval a quick way to try model changes without
 * touching the codec. */
double estimate(const Tables& t, const std::vector<Scan>& scans)
{
    // header (6 B) + rANS final state (4 B) + raw-bit stream padding (~0.5 B)
    constexpr double OVERHEAD_BYTES = 6 + 4 + 0.5;
    double bits = 0;
    for (const Scan& s : scans)
    {
        traverse(
            s,
            [&](int which, size_t ctx, uint32_t bit)
            {
                const uint16_t f = (which == 0 ? t.valid[ctx] : t.refl[ctx])[bit];
                bits -= std::log2(static_cast<double>(f) / PROB_SCALE);
            },
            [&](int ctx, uint32_t k, uint32_t raw)
            {
                if (ctx >= 0)
                {
                    bits -= std::log2(
                        static_cast<double>(t.range[static_cast<size_t>(ctx)][k]) /
                        PROB_SCALE);
                }
                bits += raw;
            });
    }
    return scans.empty() ? 0 : bits / 8 / scans.size() + OVERHEAD_BYTES;
}

template<size_t N>
void writeRows(std::ostream& o, const char* name, const char* type,
               const char* dim, const auto& rows)
{
    o << "constexpr std::array<" << type << ", " << dim << "> " << name
      << "{{\n";
    for (const auto& r : rows)
    {
        o << "    {";
        for (size_t s = 0; s < N; s++)
        {
            o << (s ? ", " : "") << r[s];
        }
        o << "},\n";
    }
    o << "}};\n\n";
}

void writeHeader(const std::string& path, const Tables& t, int version,
                 const std::vector<std::string>& train_files, size_t n_scans)
{
    std::ofstream o(path);
    if (!o)
    {
        throw std::runtime_error("cannot write " + path);
    }
    o << "/* GENERATED by tools/ms136_codec_train -- do not edit by hand.\n"
         " * Trained on "
      << n_scans << " scans from:\n";
    for (const auto& f : train_files)
    {
        o << " *   " << f.substr(f.find_last_of('/') + 1) << "\n";
    }
    o << " */\n\n#pragma once\n\n#include <array>\n#include <cstdint>\n\n"
         "#include \"ms136_codec_model.hpp\"\n\n"
         "namespace util::ms136_codec\n{\n\n"
         "constexpr uint16_t TABLE_VERSION = "
      << version << ";\n\n";
    writeRows<2>(o, "VALID_FREQS", "BinaryFreqs", "NUM_VALID_CTX", t.valid);
    writeRows<2>(o, "REFL_FREQS", "BinaryFreqs", "NUM_REFL_CTX", t.refl);
    writeRows<NUM_RANGE_SYM>(o, "RANGE_FREQS", "RangeFreqs", "NUM_RANGE_CTX", t.range);
    o << "};  // namespace util::ms136_codec\n";
}

}  // namespace


int main(int argc, char** argv)
{
    std::string out_path;
    int version = 1;
    std::vector<std::string> train_files, eval_files;
    std::vector<std::string>* cur = nullptr;
    for (int i = 1; i < argc; i++)
    {
        const std::string a = argv[i];
        if (a == "-o" && i + 1 < argc)
        {
            out_path = argv[++i];
        }
        else if (a == "--version" && i + 1 < argc)
        {
            version = std::stoi(argv[++i]);
        }
        else if (a == "--train")
        {
            cur = &train_files;
        }
        else if (a == "--eval")
        {
            cur = &eval_files;
        }
        else if (cur)
        {
            cur->push_back(a);
        }
        else
        {
            std::cerr << "usage: " << argv[0]
                      << " [-o tables.hpp] [--version N] --train a.bin ..."
                         " [--eval b.bin ...]\n";
            return 1;
        }
    }
    if (train_files.empty())
    {
        std::cerr << "no training files given\n";
        return 1;
    }

    try
    {
        Counts counts;
        std::vector<Scan> train_all;
        for (const auto& f : train_files)
        {
            auto s = loadScans(f);
            accumulate(counts, s);
            train_all.insert(train_all.end(), s.begin(), s.end());
        }
        const Tables t = buildTables(counts);
        std::printf("trained on %zu scans: %.0f B/scan (in-sample)\n",
                    counts.n_scans, estimate(t, train_all));
        for (const auto& f : eval_files)
        {
            const auto s = loadScans(f);
            std::printf("eval %-40s %5zu scans: %.0f B/scan\n",
                        f.substr(f.find_last_of('/') + 1).c_str(), s.size(),
                        estimate(t, s));
        }
        if (!out_path.empty())
        {
            writeHeader(out_path, t, version, train_files, counts.n_scans);
            std::printf("wrote %s (TABLE_VERSION %d)\n", out_path.c_str(), version);
        }
    }
    catch (const std::exception& e)
    {
        std::cerr << "error: " << e.what() << "\n";
        return 1;
    }
    return 0;
}
