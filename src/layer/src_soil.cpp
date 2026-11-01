#include "g2m/layer/src_soil.h"
#include "g2m/core/bytes.h"
#include "g2m/core/geo/utm.h"
#include <algorithm>
#include <cmath>
#include <limits>

namespace g2m {
namespace {
Error malformed(std::string message) { return make_error(ErrorCode::Malformed, "src.soil: " + message); }
bool valid_scheme(std::string_view s) {
    return !s.empty() && s.size() <= 64 && std::all_of(s.begin(), s.end(), [](char c) {
        return (c >= 'a' && c <= 'z') || (c >= '0' && c <= '9') || c == '_' || c == '.';
    });
}
} // namespace

SoilWindow soil_source_window(const TileKey& key, std::int32_t cells_per_degree) {
    const geo::Utm utm;
    const double n = static_cast<double>(cells_per_degree);
    const std::int64_t size = key.size_m(), e0 = key.min_easting(), n0 = key.min_northing();
    double lon_lo = std::numeric_limits<double>::infinity(), lon_hi = -lon_lo, lat_lo = lon_lo, lat_hi = -lon_lo;
    for (int i = 0; i <= 4; ++i) {
        for (int j = 0; j <= 4; ++j) {
            const geo::TmReverse r = utm.reverse(key.zone, static_cast<double>(e0 + size * i / 4), static_cast<double>(n0 + size * j / 4));
            lon_lo = std::min(lon_lo, r.lon); lon_hi = std::max(lon_hi, r.lon);
            lat_lo = std::min(lat_lo, r.lat); lat_hi = std::max(lat_hi, r.lat);
        }
    }
    SoilWindow w;
    w.col0 = static_cast<std::int64_t>(std::floor(lon_lo * n)) - 1;
    w.row0 = static_cast<std::int64_t>(std::floor(lat_lo * n)) - 1;
    w.width = static_cast<int>(static_cast<std::int64_t>(std::floor(lon_hi * n)) + 1 - w.col0 + 1);
    w.height = static_cast<int>(static_cast<std::int64_t>(std::floor(lat_hi * n)) + 1 - w.row0 + 1);
    return w;
}

Result<TileBody> encode_src_soil(const SoilSourceTile& t) {
    if (!t.key.valid() || t.key.level != 2) return make_error(ErrorCode::InvalidArgument, "src.soil: key must be a valid level-2 key");
    if (!valid_scheme(t.scheme)) return make_error(ErrorCode::InvalidArgument, "src.soil: bad scheme");
    if (t.cells_per_degree < 1 || t.cells_per_degree > 100000) return make_error(ErrorCode::InvalidArgument, "src.soil: cells_per_degree");
    if (t.width < 1 || t.height < 1 || t.width > kSoilMaxWindow || t.height > kSoilMaxWindow)
        return make_error(ErrorCode::InvalidArgument, "src.soil: window size");
    const std::size_t cells = static_cast<std::size_t>(t.width) * static_cast<std::size_t>(t.height);
    if (t.values.size() != cells * kSoilPlanes) return make_error(ErrorCode::InvalidArgument, "src.soil: value count does not match the window");
    if (cells * kSoilPlanes * 2 > kMaxSectionRawSize) return make_error(ErrorCode::LimitExceeded, "src.soil: window too large");
    for (std::uint16_t v : t.values)
        if (v > 32767 && v != kSoilNoData) return make_error(ErrorCode::InvalidArgument, "src.soil: value above 32767 that is not the no-data marker");
    ByteWriter meta;
    meta.u64(t.key.packed());
    meta.u8(static_cast<std::uint8_t>(t.scheme.size())); meta.bytes(t.scheme.data(), t.scheme.size());
    meta.varint(static_cast<std::uint64_t>(t.cells_per_degree));
    meta.svarint(t.col0); meta.svarint(t.row0);
    meta.varint(static_cast<std::uint64_t>(t.width)); meta.varint(static_cast<std::uint64_t>(t.height));
    ByteWriter vals;
    for (std::uint16_t v : t.values) vals.u16(v);
    TileBody body;
    body.encoding = static_cast<std::uint16_t>(LayerEncoding::SrcSoil);
    body.encoding_version = kSrcSoilEncodingVersion;
    body.sections.push_back(Section{src_soil::kMeta, SectionCodec::None, meta.take()});
    body.sections.push_back(Section{src_soil::kValues, SectionCodec::Zstd, vals.take()});
    return body;
}

Result<SoilSourceTile> decode_src_soil(const TileBody& body) {
    if (body.encoding != static_cast<std::uint16_t>(LayerEncoding::SrcSoil)) return make_error(ErrorCode::Unsupported, "not a src.soil body");
    if (body.encoding_version != kSrcSoilEncodingVersion) return make_error(ErrorCode::Unsupported, "src.soil version");
    const Section* meta = body.find(src_soil::kMeta);
    const Section* vals = body.find(src_soil::kValues);
    if (!meta || !vals || body.sections.size() != 2) return malformed("sections");
    ByteReader r(meta->data);
    SoilSourceTile t;
    const auto key = TileKey::unpack(r.u64());
    if (!key || key->level != 2) return malformed("key");
    t.key = *key;
    const std::uint8_t len = r.u8();
    t.scheme = std::string(r.string(len));
    const auto n = r.varint();
    t.col0 = r.svarint(); t.row0 = r.svarint();
    const auto w = r.varint(), h = r.varint();
    if (!r.ok()) return make_error(ErrorCode::Truncated, "src.soil: meta truncated");
    if (!r.at_end()) return malformed("meta has trailing bytes");
    if (!valid_scheme(t.scheme)) return malformed("scheme");
    if (n < 1 || n > 100000) return malformed("cells_per_degree");
    if (w < 1 || h < 1 || w > static_cast<std::uint64_t>(kSoilMaxWindow) || h > static_cast<std::uint64_t>(kSoilMaxWindow)) return malformed("window size");
    constexpr std::int64_t kColMax = std::int64_t{360} * 100000 + 8, kRowMax = std::int64_t{180} * 100000 + 8;
    if (t.col0 < -kColMax || t.col0 > kColMax || t.row0 < -kRowMax || t.row0 > kRowMax) return malformed("window origin");
    t.cells_per_degree = static_cast<std::int32_t>(n);
    t.width = static_cast<int>(w); t.height = static_cast<int>(h);
    const std::size_t cells = static_cast<std::size_t>(w) * static_cast<std::size_t>(h);
    if (vals->data.size() != cells * kSoilPlanes * 2) return malformed("value section size");
    ByteReader vr(vals->data);
    t.values.resize(cells * kSoilPlanes);
    for (auto& v : t.values) {
        v = vr.u16();
        if (v > 32767 && v != kSoilNoData) return malformed("value above 32767 that is not the no-data marker");
    }
    if (!vr.at_end()) return malformed("values truncated");
    return t;
}
} // namespace g2m
