// src/layer/src_landcover.cpp - see g2m/layer/src_landcover.h and
// docs/formats/src_landcover.md.

#include "g2m/layer/src_landcover.h"

#include "g2m/core/bytes.h"
#include "georef_io.h"

#include <algorithm>

namespace g2m {
namespace {

bool valid_scheme(std::string_view s) {
    return !s.empty() && s.size() <= kMaxClassSchemeLength && std::all_of(s.begin(), s.end(), [](char c) {
        return (c >= 'a' && c <= 'z') || (c >= '0' && c <= '9') || c == '_' || c == '.';
    });
}

} // namespace

Result<TileBody> encode_src_landcover(const LandcoverTile& tile) {
    const RasterGeoref& g = tile.georef;
    if (!g.valid() || g.width != tile.classes.width() || g.height != tile.classes.height()) {
        return make_error(ErrorCode::InvalidArgument, "src.landcover: georef invalid or does not match the raster");
    }
    if (static_cast<std::int64_t>(g.width) * g.height > layer_detail::kMaxRasterSamples) {
        return make_error(ErrorCode::LimitExceeded, "src.landcover: raster too large");
    }
    if (!valid_scheme(tile.class_scheme)) return make_error(ErrorCode::InvalidArgument, "src.landcover: bad class scheme");
    TileBody body;
    body.encoding = static_cast<std::uint16_t>(LayerEncoding::SrcLandcover);
    body.encoding_version = kSrcLandcoverEncodingVersion;
    ByteWriter meta;
    layer_detail::write_georef(meta, g);
    meta.u8(tile.nodata_class);
    meta.u8(static_cast<std::uint8_t>(tile.class_scheme.size()));
    meta.bytes(tile.class_scheme.data(), tile.class_scheme.size());
    body.sections.push_back(Section{src_landcover::kMeta, SectionCodec::None, meta.take()});
    const auto cls = tile.classes.data();
    body.sections.push_back(Section{src_landcover::kClasses, SectionCodec::Zstd, {cls.begin(), cls.end()}});
    return body;
}

Result<LandcoverTile> decode_src_landcover(const TileBody& body) {
    if (body.encoding != static_cast<std::uint16_t>(LayerEncoding::SrcLandcover)) {
        return make_error(ErrorCode::Unsupported, "not a src.landcover body");
    }
    if (body.encoding_version != kSrcLandcoverEncodingVersion) {
        return make_error(ErrorCode::Unsupported, "src.landcover version");
    }
    const Section* meta = body.find(src_landcover::kMeta);
    const Section* cls = body.find(src_landcover::kClasses);
    if (!meta || !cls || body.sections.size() != 2) return make_error(ErrorCode::Malformed, "src.landcover: sections");
    ByteReader r(meta->data);
    auto g = layer_detail::read_georef(r);
    if (!g) return g.error();
    LandcoverTile t;
    t.georef = *g;
    t.nodata_class = r.u8();
    const std::uint8_t len = r.u8();
    t.class_scheme = std::string(r.string(len));
    if (!r.ok()) return make_error(ErrorCode::Truncated, "src.landcover: meta truncated");
    if (!r.at_end()) return make_error(ErrorCode::Malformed, "src.landcover: meta has trailing bytes");
    if (!valid_scheme(t.class_scheme)) return make_error(ErrorCode::Malformed, "src.landcover: bad class scheme");
    const std::size_t n = static_cast<std::size_t>(g->width) * static_cast<std::size_t>(g->height);
    if (cls->data.size() != n) return make_error(ErrorCode::Malformed, "src.landcover: class section size");
    t.classes = ClassRaster(g->width, g->height);
    std::copy(cls->data.begin(), cls->data.end(), t.classes.data().begin());
    return t;
}

} // namespace g2m
