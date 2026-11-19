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


/* =============================================================================
 *  MS136 scan codec: overview
 * =============================================================================
 *
 * Compresses one MS136 low-res scan for the robot <-> client link. The input
 * is a ms136::redux::DenseBuffer: 14 layers x 360 azimuth cells, each a
 * 15-bit range in mm plus a reflector bit, with 0 meaning "no return". Every
 * scan is encoded independently (no state carries over between scans), so a
 * dropped message never affects any other scan.
 *
 * Measured on the 2026 competition bags (~4,200 valid points per scan):
 *
 *   old format (gap-skip packing + zstd -15)   ~6,600 B/scan
 *   this codec                                 ~3,300-3,500 B/scan
 *   encode / decode time                       ~0.2 ms each, per scan
 *   range error vs. input DenseBuffer          <= 1 mm (QUANT_MM / 2)
 *   range error vs. true range                 0.76 mm RMS, <= 1.5 mm, unbiased
 *   valid / null pattern, reflector bits       exact
 *
 *
 * --- Why it beats a general-purpose compressor -------------------------------
 *
 * Information theory: if the decoder expected a value with probability p, the
 * value can be sent in -log2(p) bits, and no scheme does better on average.
 * So output size is set entirely by how well both sides can *predict* each
 * value before seeing it. zstd predicts by finding repeated byte strings,
 * which lidar ranges almost never contain (their low bits are sensor noise),
 * so it only manages ~1.3x. This codec uses knowledge of the data instead:
 *
 *  1. Quantize. Ranges are rounded to QUANT_MM (2 mm) steps. The sensor's own
 *     noise is ~3.6 mm (1 sigma, measured from stationary periods in the
 *     bags), so the dropped bit was mostly noise anyway: this saves ~1 bit
 *     per point and raises total error from 3.61 to 3.68 mm (+1.9%; see
 *     the error budget in ms136_codec_model.hpp).
 *
 *  2. Predict. Neighbouring beams usually hit nearby surfaces, so each range
 *     is predicted from already-coded neighbours and only the residual
 *     (actual - predicted) is sent. Residuals cluster tightly around zero, so
 *     they are far more predictable than the raw ranges.
 *
 *  3. Model in context. How big a residual is likely to be depends on the
 *     neighbourhood: tiny on flat ground, large at an object edge, larger
 *     far away. The model chooses one of several hundred probability
 *     distributions from features of already-coded neighbours (the
 *     "context"). The valid and reflector bits are modelled the same way and
 *     become nearly free (~170 B/scan together, versus 2 bytes per gap marker
 *     plus 1 bit per point before).
 *
 *  4. Entropy code. An rANS coder (a fast form of arithmetic coding) turns the
 *     stream of (symbol, probability) pairs into bytes at essentially the
 *     ideal -log2(p) each, fractional bits included. That matters because
 *     valid bits average ~0.2 bit each, which a code that spends at least 1
 *     bit per symbol (like Huffman) cannot express.
 *
 * The probabilities are fixed tables trained offline on recorded bags
 * (ms136_codec_tables.hpp). Robot and client compile in the same tables, so no
 * model data is ever transmitted. Table quality affects only output size: any
 * tables that pass the static_asserts in ms136_codec.cpp decode correctly.
 *
 * The one invariant everything depends on: for every point, the decoder must
 * compute exactly the same prediction and context the encoder used. That is
 * only possible if both are derived from data the decoder already has, i.e.
 * points earlier in coding order ("causal" data). computeContext() in
 * ms136_codec_model.hpp is the single definition of this; the encoder,
 * decoder and offline trainer all call it.
 *
 *
 * --- Files -------------------------------------------------------------------
 *
 *   ms136_codec.hpp          this API and overview
 *   ms136_codec_model.hpp    quantization, prediction, contexts and residual
 *                            mapping ("what is coded"). Start here to
 *                            understand or change the model.
 *   ms136_codec.cpp          rANS coder, raw bit stream, encode/decode loops
 *                            ("how it is coded")
 *   ms136_codec_tables.hpp   GENERATED probability tables; do not hand-edit
 *   tools/ms136_extract_scans.py, tools/ms136_codec_train.cpp
 *                            regenerate the tables from recorded bags
 *   test/test_ms136_codec.cpp
 *                            round-trip, corrupt-input and size tests
 *
 *
 * --- Wire format (all multi-byte fields little-endian) ----------------------
 *
 *   offset      size      field
 *   0           1         FORMAT_VERSION   bump when byte layout/model changes
 *   1           1         QUANT_MM
 *   2           2         TABLE_VERSION    bump when tables are retrained
 *   4           2         rans_len
 *   6           rans_len  rANS stream: 4-byte initial decoder state, then
 *                         renormalization bytes in the order the decoder
 *                         reads them
 *   6+rans_len  rest      raw low bits of range residuals, packed LSB-first,
 *                         last byte zero-padded
 *
 * The decoder rejects any payload whose header fields differ from its own
 * build, so mismatched robot/client builds drop scans instead of decoding
 * garbage. Deploy robot and client together.
 *
 *
 * --- Changing things ---------------------------------------------------------
 *
 *  - Retraining (new sensor settings, new environments): extract scans with
 *    tools/ms136_extract_scans.py, then run ms136_codec_train with
 *    --version <TABLE_VERSION + 1>. Its --eval option reports expected
 *    bytes/scan on held-out bags. Then run the tests.
 *  - Model changes (contexts, predictor, QUANT_MM): edit
 *    ms136_codec_model.hpp, adjust the table dimensions, retrain, and bump
 *    both TABLE_VERSION and FORMAT_VERSION. The trainer's --eval output shows
 *    whether a change helps before anything is deployed. */
namespace util::ms136_codec
{

constexpr uint8_t FORMAT_VERSION = 1;
constexpr size_t HEADER_BYTES = 6;

/* Encode a dense buffer and append the result to `out`. Cells whose range
 * bits are zero are treated as null (their reflector bit is dropped). */
void encodeScan(
    const ms136::redux::DenseBuffer& dense,
    std::vector<uint8_t>& out);

/* Decode one scan. Returns false (leaving `dense` unspecified) if the payload
 * is malformed, truncated, or was produced with a different format version,
 * quantization step, or probability table version. */
bool decodeScan(
    const uint8_t* data,
    size_t len,
    ms136::redux::DenseBuffer& dense);

};  // namespace util::ms136_codec
