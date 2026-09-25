// src/layer/src_elev.cpp - see g2m/layer/src_elev.h and
// docs/formats/src_elev.md.
//
// Samples section: per sample in row-major order, the residual
// d = v - p (uint32 wrap-around arithmetic) of the value against its
// prediction, zigzag-mapped so small |d| become small unsigned values, then
// byte-plane shuffled (all byte 0s, then all byte 1s, ...), which leaves
// the mostly-zero high planes as long runs for zstd. A NoData sample
// carries the prediction itself (d = 0) and is marked in the NoData bitmap;
// the decoder keeps the prediction chain running through it and writes
// kHeightNoData to the output afterwards.

#include "g2m/layer/src_elev.h"

#include "g2m/core/bytes.h"
#include "georef_io.h"

namespace g2m {
namespace {

using layer_detail::read_georef;
using layer_detail::write_georef;

inline std::uint32_t predict(const std::uint32_t* cur, const std::uint32_t* prev, int c, ElevPredictor pred) {
    if (c > 0) {
        if (prev && pred == ElevPredictor::Gradient) return cur[c - 1] + prev[c] - prev[c - 1];
        return cur[c - 1];
    }
    return prev ? prev[0] : 0u;
}

} // namespace

Result<TileBody> encode_src_elev(const ElevTile& tile, ElevPredictor predictor) {
    const RasterGeoref& g = tile.georef;
    if (!g.valid() || g.width != tile.heights.width() || g.height != tile.heights.height()) {
        return make_error(ErrorCode::InvalidArgument, "src.elev: georef invalid or does not match the raster");
    }
    if (static_cast<std::int64_t>(g.width) * g.height > layer_detail::kMaxRasterSamples) {
        return make_error(ErrorCode::LimitExceeded, "src.elev: raster too large");
    }
    if (predictor != ElevPredictor::Left && predictor != ElevPredictor::Gradient) {
        return make_error(ErrorCode::InvalidArgument, "src.elev: unknown predictor");
    }
    const auto w = static_cast<std::size_t>(g.width);
    const auto h = static_cast<std::size_t>(g.height);
    const std::size_t n = w * h;
    const auto src = tile.heights.data();

    // Reconstructed values (NoData replaced by its prediction), for the chain.
    std::vector<std::uint32_t> rec(n);
    std::vector<std::uint8_t> planes(4 * n);
    std::vector<std::uint8_t> nodata((n + 7) / 8, 0);
    bool any_nodata = false;
    for (std::size_t r = 0; r < h; ++r) {
        std::uint32_t* cur = rec.data() + r * w;
        const std::uint32_t* prev = r > 0 ? cur - w : nullptr;
        for (std::size_t c = 0; c < w; ++c) {
            const std::size_t i = r * w + c;
            const std::uint32_t p = predict(cur, prev, static_cast<int>(c), predictor);
            std::uint32_t v = static_cast<std::uint32_t>(src[i]);
            if (src[i] == kHeightNoData) {
                any_nodata = true;
                nodata[i >> 3] = static_cast<std::uint8_t>(nodata[i >> 3] | (1u << (i & 7)));
                v = p;
            }
            cur[c] = v;
            const std::uint32_t z = zigzag_encode32(static_cast<std::int32_t>(v - p));
            planes[i] = static_cast<std::uint8_t>(z);
            planes[n + i] = static_cast<std::uint8_t>(z >> 8);
            planes[2 * n + i] = static_cast<std::uint8_t>(z >> 16);
            planes[3 * n + i] = static_cast<std::uint8_t>(z >> 24);
        }
    }

    TileBody body;
    body.encoding = static_cast<std::uint16_t>(LayerEncoding::SrcElev);
    body.encoding_version = kSrcElevEncodingVersion;
    ByteWriter meta;
    write_georef(meta, g);
    meta.u8(static_cast<std::uint8_t>(predictor));
    meta.u8(any_nodata ? 1 : 0);
    body.sections.push_back(Section{src_elev::kMeta, SectionCodec::None, meta.take()});
    body.sections.push_back(Section{src_elev::kSamples, SectionCodec::Zstd, std::move(planes)});
    if (any_nodata) body.sections.push_back(Section{src_elev::kNoData, SectionCodec::Zstd, std::move(nodata)});
    return body;
}

Result<ElevTile> decode_src_elev(const TileBody& body) {
    if (body.encoding != static_cast<std::uint16_t>(LayerEncoding::SrcElev)) {
        return make_error(ErrorCode::Unsupported, "not a src.elev body");
    }
    if (body.encoding_version != kSrcElevEncodingVersion) return make_error(ErrorCode::Unsupported, "src.elev version");
    const Section* meta = body.find(src_elev::kMeta);
    const Section* samples = body.find(src_elev::kSamples);
    const Section* nd = body.find(src_elev::kNoData);
    if (!meta || !samples) return make_error(ErrorCode::Malformed, "src.elev: missing section");
    if (body.sections.size() != (nd ? 3u : 2u)) return make_error(ErrorCode::Malformed, "src.elev: unknown section");

    ByteReader r(meta->data);
    auto g = read_georef(r);
    if (!g) return g.error();
    const std::uint8_t pred_raw = r.u8();
    const std::uint8_t has_nodata = r.u8();
    if (!r.ok()) return make_error(ErrorCode::Truncated, "src.elev: meta truncated");
    if (!r.at_end()) return make_error(ErrorCode::Malformed, "src.elev: meta has trailing bytes");
    if (pred_raw != 1 && pred_raw != 2) return make_error(ErrorCode::Unsupported, "src.elev: unknown predictor");
    if (has_nodata > 1 || (has_nodata == 1) != (nd != nullptr)) {
        return make_error(ErrorCode::Malformed, "src.elev: NoData flag and section disagree");
    }
    const auto predictor = static_cast<ElevPredictor>(pred_raw);
    const auto w = static_cast<std::size_t>(g->width);
    const auto h = static_cast<std::size_t>(g->height);
    const std::size_t n = w * h;
    if (samples->data.size() != 4 * n) return make_error(ErrorCode::Malformed, "src.elev: sample section size");
    if (nd) {
        if (nd->data.size() != (n + 7) / 8) return make_error(ErrorCode::Malformed, "src.elev: NoData bitmap size");
        bool any = false;
        for (std::uint8_t b : nd->data) any = any || b != 0;
        if (!any) return make_error(ErrorCode::Malformed, "src.elev: empty NoData bitmap");
        if ((n & 7) != 0 && (nd->data.back() >> (n & 7)) != 0) {
            return make_error(ErrorCode::Malformed, "src.elev: NoData padding bits set");
        }
    }

    ElevTile out;
    out.georef = *g;
    out.heights = HeightRaster(g->width, g->height);
    const auto dst = out.heights.data();
    const std::uint8_t* pl = samples->data.data();
    std::vector<std::uint32_t> rec(n);
    for (std::size_t r2 = 0; r2 < h; ++r2) {
        std::uint32_t* cur = rec.data() + r2 * w;
        const std::uint32_t* prev = r2 > 0 ? cur - w : nullptr;
        for (std::size_t c = 0; c < w; ++c) {
            const std::size_t i = r2 * w + c;
            const std::uint32_t z = static_cast<std::uint32_t>(pl[i]) | (static_cast<std::uint32_t>(pl[n + i]) << 8) |
                                    (static_cast<std::uint32_t>(pl[2 * n + i]) << 16) |
                                    (static_cast<std::uint32_t>(pl[3 * n + i]) << 24);
            const std::uint32_t p = predict(cur, prev, static_cast<int>(c), predictor);
            const std::uint32_t v = p + static_cast<std::uint32_t>(zigzag_decode32(z));
            cur[c] = v;
            const bool is_nd = nd && ((nd->data[i >> 3] >> (i & 7)) & 1u) != 0;
            if (is_nd) {
                if (z != 0) return make_error(ErrorCode::Malformed, "src.elev: NoData sample with a nonzero residual");
                dst[i] = kHeightNoData;
            } else {
                if (static_cast<std::int32_t>(v) == kHeightNoData) {
                    return make_error(ErrorCode::Malformed, "src.elev: NoData value outside the NoData bitmap");
                }
                dst[i] = static_cast<std::int32_t>(v);
            }
        }
    }
    return out;
}

} // namespace g2m
