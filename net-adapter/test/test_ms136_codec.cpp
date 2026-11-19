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

#include <array>
#include <random>
#include <vector>
#include <cstdint>
#include <fstream>
#include <numeric>

#include <gtest/gtest.h>

#include "../src/util/ms136_codec/ms136_codec.hpp"
#include "../src/util/ms136_codec/ms136_codec_model.hpp"

using ms136::redux::DenseBuffer;
using ms136::redux::REFLECTOR_MASK;
using ms136::redux::getRangeMMeters;
using ms136::redux::getReflector;
namespace codec = util::ms136_codec;

#ifndef MS136_FIXTURE_PATH
#error "MS136_FIXTURE_PATH must be defined"
#endif

namespace
{

std::vector<DenseBuffer> loadFixture()
{
    std::ifstream f(MS136_FIXTURE_PATH, std::ios::binary | std::ios::ate);
    EXPECT_TRUE(f) << "cannot open " << MS136_FIXTURE_PATH;
    const size_t bytes = static_cast<size_t>(f.tellg());
    std::vector<DenseBuffer> scans(bytes / sizeof(DenseBuffer));
    f.seekg(0);
    f.read(reinterpret_cast<char*>(scans.data()), scans.size() * sizeof(DenseBuffer));
    return scans;
}

std::vector<uint8_t> encode(const DenseBuffer& d)
{
    std::vector<uint8_t> out;
    codec::encodeScan(d, out);
    return out;
}

/* Decoded scan must match the input up to range quantization */
void expectRoundTrip(const DenseBuffer& in)
{
    const std::vector<uint8_t> bytes = encode(in);
    DenseBuffer out;
    out.fill(0xFFFF);
    ASSERT_TRUE(codec::decodeScan(bytes.data(), bytes.size(), out));
    for (size_t i = 0; i < in.size(); i++)
    {
        const uint16_t r_in = getRangeMMeters(in[i]);
        const uint16_t r_out = getRangeMMeters(out[i]);
        if (!r_in)
        {
            ASSERT_EQ(out[i], 0) << "cell " << i;
            continue;
        }
        ASSERT_NE(r_out, 0) << "cell " << i;
        ASSERT_LE(std::abs(int(r_in) - int(r_out)), codec::QUANT_MM / 2)
            << "cell " << i << " in " << r_in << " out " << r_out;
        ASSERT_EQ(getReflector(in[i]), getReflector(out[i])) << "cell " << i;
    }
}

DenseBuffer randomScan(std::mt19937& rng, double valid_p, uint16_t lo, uint16_t hi)
{
    std::bernoulli_distribution valid(valid_p), refl(0.2);
    std::uniform_int_distribution<int> range(lo, hi);
    DenseBuffer d;
    for (auto& v : d)
    {
        v = valid(rng) ? static_cast<uint16_t>(range(rng) | (refl(rng) ? REFLECTOR_MASK : 0)) : 0;
    }
    return d;
}

}  // namespace


TEST(MS136Codec, ModelConstants)
{
    // previous-layer azimuth alignment derived from AZIMUTH_OFFSET_LUT
    constexpr std::array<int, 14> expected{0, -11, -3, 29, -29, 3, 10, 180, -11, -4, 31, -33, 3, 13};
    for (size_t l = 0; l < expected.size(); l++)
    {
        EXPECT_EQ(ms136::redux::PREV_LAYER_AZ_SHIFT[l], expected[l]) << "layer " << l;
    }
    EXPECT_EQ(codec::MAX_QRANGE, 16384);
    EXPECT_EQ(codec::RAW_RANGE_BITS, 15U);
    EXPECT_EQ(codec::dequantizeRange(codec::quantizeRange(0x7FFF)), 0x7FFF);
}

TEST(MS136Codec, RoundTripRecordedScans)
{
    const auto scans = loadFixture();
    ASSERT_GE(scans.size(), 12U);
    for (const auto& s : scans)
    {
        expectRoundTrip(s);
    }
}

TEST(MS136Codec, RoundTripEdgeCases)
{
    DenseBuffer d;

    d.fill(0);
    expectRoundTrip(d);

    d.fill(0x7FFF | REFLECTOR_MASK);  // all max range, all reflectors
    expectRoundTrip(d);

    d.fill(1);  // smallest representable range
    expectRoundTrip(d);

    d.fill(0);
    d.front() = 1234;
    expectRoundTrip(d);

    d.fill(0);
    d.back() = 0x7FFF | REFLECTOR_MASK;
    expectRoundTrip(d);

    d.fill(0);  // reflector bit on a null cell must decode as null
    d[100] = REFLECTOR_MASK;
    d[101] = 5000;
    expectRoundTrip(d);

    for (size_t i = 0; i < d.size(); i++)  // alternating extremes
    {
        d[i] = (i % 2) ? 1 : 0x7FFF;
    }
    expectRoundTrip(d);
}

TEST(MS136Codec, RoundTripRandomScans)
{
    std::mt19937 rng(1234);
    for (int t = 0; t < 50; t++)
    {
        const double p = (t % 5) / 4.0;
        expectRoundTrip(randomScan(rng, p, 1, 0x7FFF));      // incompressible
        expectRoundTrip(randomScan(rng, p, 2000, 2100));     // narrow band
    }
}

TEST(MS136Codec, CompressedSize)
{
    const auto scans = loadFixture();
    size_t total = 0;
    for (const auto& s : scans)
    {
        total += encode(s).size();
    }
    const double mean = double(total) / scans.size();
    std::printf("mean encoded size: %.0f B/scan over %zu recorded scans\n", mean, scans.size());
    EXPECT_LT(mean, 4000.0);  // ~3.4 KB expected (6.6 KB with the old packing + zstd)
}

TEST(MS136Codec, RejectsHeaderMismatch)
{
    const auto scans = loadFixture();
    ASSERT_FALSE(scans.empty());
    const std::vector<uint8_t> good = encode(scans.front());
    DenseBuffer out;
    for (size_t byte : {0, 1, 2, 3})
    {
        auto bad = good;
        bad[byte] ^= 0x01;
        EXPECT_FALSE(codec::decodeScan(bad.data(), bad.size(), out)) << "header byte " << byte;
    }
    EXPECT_FALSE(codec::decodeScan(nullptr, 0, out));
}

TEST(MS136Codec, RejectsTruncatedOrPadded)
{
    const auto scans = loadFixture();
    ASSERT_FALSE(scans.empty());
    const std::vector<uint8_t> good = encode(scans.front());
    DenseBuffer out;
    for (size_t n = 0; n < good.size(); n++)
    {
        std::vector<uint8_t> cut(good.begin(), good.begin() + n);  // exact-size buffer for ASan
        EXPECT_FALSE(codec::decodeScan(cut.data(), cut.size(), out)) << "length " << n;
    }
    auto padded = good;
    padded.push_back(0);
    EXPECT_FALSE(codec::decodeScan(padded.data(), padded.size(), out));
}

TEST(MS136Codec, SurvivesCorruptInput)
{
    // Corrupt payloads must never crash or read out of bounds (run under ASan
    // to verify). There is no checksum: flips in the raw low-bit region decode
    // to slightly different ranges by design (transport integrity is zenoh's
    // job), but flips in the rANS stream desynchronize the decoder and should
    // almost always be caught by the final-state check.
    const auto scans = loadFixture();
    ASSERT_FALSE(scans.empty());
    const std::vector<uint8_t> good = encode(scans.front());
    const size_t rans_len = good[4] | (good[5] << 8);
    const size_t raw_begin = codec::HEADER_BYTES + rans_len;
    ASSERT_LT(raw_begin, good.size());

    std::mt19937 rng(42);
    DenseBuffer out;
    const auto flipAndDecode = [&](size_t lo, size_t hi)
    {
        auto bad = good;
        bad[lo + rng() % (hi - lo)] ^= static_cast<uint8_t>(1U << (rng() % 8));
        return codec::decodeScan(bad.data(), bad.size(), out);
    };

    const int trials = 2000;
    int rans_accepted = 0;
    for (int t = 0; t < trials; t++)
    {
        rans_accepted += flipAndDecode(codec::HEADER_BYTES, raw_begin);
        (void)flipAndDecode(raw_begin, good.size());
    }

    int junk_accepted = 0;
    for (int t = 0; t < 500; t++)  // pure garbage behind a valid header
    {
        std::vector<uint8_t> junk(good.begin(), good.begin() + codec::HEADER_BYTES);
        junk.resize(codec::HEADER_BYTES + rng() % 8000);
        for (size_t i = codec::HEADER_BYTES; i < junk.size(); i++)
        {
            junk[i] = static_cast<uint8_t>(rng());
        }
        junk[4] = static_cast<uint8_t>(rng());
        junk[5] = static_cast<uint8_t>(rng() % 0x40);
        junk_accepted += codec::decodeScan(junk.data(), junk.size(), out);
    }
    std::printf("accepted: rANS bit-flips %d / %d, garbage %d / 500\n",
                rans_accepted, trials, junk_accepted);
    EXPECT_LT(rans_accepted, trials / 100);
    EXPECT_EQ(junk_accepted, 0);
}
