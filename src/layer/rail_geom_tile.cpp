// Railways R1/R2: the g2m.rail.geom tile body (encode / decode / validate), layer descriptor and config text.
// See g2m/layer/rail_geom_tile.h and docs/formats/rail_geom.md.
#include "g2m/layer/rail_geom_tile.h"
#include "g2m/core/bytes.h"
#include "g2m/core/hash.h"
#include "g2m/layer/road_dem_dependencies.h"
#include "g2m/layer/terrain_layers.h"
#include <algorithm>
#include <limits>

namespace g2m {
namespace {
constexpr std::int64_t kCoordLimit = std::int64_t{1} << 40;      // mm
constexpr std::int32_t kZLimit = std::int32_t{1} << 27;          // 1/256 m
Error bad(const std::string& message) { return make_error(ErrorCode::Malformed, "rail.geom: " + message); }
bool owned_by(const TileKey& key, const geom::PointMm& p) {
    const auto size = static_cast<std::int64_t>(key.size_m()) * 1000;
    return floor_div(p.x, size) == key.x && floor_div(p.y, size) == key.y;
}
}

Result<void> validate_rail_geom_tile(const RailGeomTile& tile) {
    if (!tile.key.valid() || tile.key.level != 2) return bad("level-2 key required");
    if (tile.pieces.size() > kRailGeomMaxPieces) return make_error(ErrorCode::LimitExceeded, "rail.geom: piece count");
    std::size_t points = 0;
    const RailPiece* previous = nullptr;
    for (const auto& p : tile.pieces) {
        if (previous && !(previous->way_id < p.way_id || (previous->way_id == p.way_id && previous->piece < p.piece)))
            return bad("pieces not strictly ascending by (way, piece)");
        previous = &p;
        if (p.cls > kRailClassMax) return bad("class out of range");
        if (static_cast<std::uint8_t>(p.condition) > 1) return bad("condition out of range");
        if ((p.flags & ~kRailFlagMask) != 0) return bad("unknown flag bits");
        if ((p.flags & kRailBridge) && (p.flags & kRailTunnel)) return bad("bridge and tunnel together");
        if (p.layer < -5 || p.layer > 5) return bad("layer out of range");
        if (p.tracks < 1 || p.tracks > 16) return bad("tracks out of range");
        if (static_cast<std::uint8_t>(p.service) > kRailServiceMax || static_cast<std::uint8_t>(p.usage) > kRailUsageMax) return bad("service or usage out of range");
        if (p.gauge_mm > 3000) return bad("gauge out of range");
        if (p.xy.size() < 2 || p.xy.size() > kRailGeomMaxPiecePoints || p.xy.size() != p.z.size()) return bad("point count");
        points += p.xy.size();
        if (points > kRailGeomMaxPoints) return make_error(ErrorCode::LimitExceeded, "rail.geom: point count");
        for (const auto& q : p.xy) if (q.x <= -kCoordLimit || q.x >= kCoordLimit || q.y <= -kCoordLimit || q.y >= kCoordLimit) return bad("coordinate out of range");
        for (const auto h : p.z) if (h <= -kZLimit || h >= kZLimit) return bad("height out of range");
        if ((p.flags & kRailDeclined) && std::any_of(p.z.begin(), p.z.end(), [](auto h) { return h != 0; })) return bad("declined piece with heights");
        if (!owned_by(tile.key, p.xy.front())) return bad("piece starts outside the tile");
    }
    return {};
}

Result<TileBody> encode_rail_geom(const RailGeomTile& tile) {
    const auto valid = validate_rail_geom_tile(tile); if (!valid) return valid.error();
    ByteWriter meta, ids, attrs, counts, xy, z;
    std::size_t points = 0;
    std::int64_t previous_way = 0;
    for (std::size_t i = 0; i < tile.pieces.size(); ++i) {
        const auto& p = tile.pieces[i];
        if (i == 0) ids.svarint(p.way_id); else ids.varint(static_cast<std::uint64_t>(p.way_id - previous_way));
        previous_way = p.way_id;
        ids.varint(p.piece);
        attrs.u8(p.cls); attrs.u8(static_cast<std::uint8_t>(p.condition)); attrs.u16(p.flags);
        attrs.u8(static_cast<std::uint8_t>(p.layer)); attrs.u8(p.tracks); attrs.u8(static_cast<std::uint8_t>(p.service));
        attrs.u8(static_cast<std::uint8_t>(p.usage)); attrs.u16(p.gauge_mm);
        counts.varint(p.xy.size());
        points += p.xy.size();
        for (std::size_t k = 0; k < p.xy.size(); ++k) {
            const auto px = k == 0 ? 0 : p.xy[k - 1].x, py = k == 0 ? 0 : p.xy[k - 1].y;
            xy.svarint(p.xy[k].x - px); xy.svarint(p.xy[k].y - py);
            z.svarint(static_cast<std::int64_t>(p.z[k]) - (k == 0 ? 0 : p.z[k - 1]));
        }
    }
    meta.u32(static_cast<std::uint32_t>(tile.pieces.size()));
    meta.u32(static_cast<std::uint32_t>(points));
    TileBody body;
    body.encoding = static_cast<std::uint16_t>(LayerEncoding::RailGeom);
    body.encoding_version = kRailGeomEncodingVersion;
    body.sections.push_back(Section{rail_geom::kMeta, SectionCodec::None, meta.take()});
    body.sections.push_back(Section{rail_geom::kIds, SectionCodec::Zstd, ids.take()});
    body.sections.push_back(Section{rail_geom::kAttrs, SectionCodec::Zstd, attrs.take()});
    body.sections.push_back(Section{rail_geom::kCounts, SectionCodec::Zstd, counts.take()});
    body.sections.push_back(Section{rail_geom::kXy, SectionCodec::Zstd, xy.take()});
    body.sections.push_back(Section{rail_geom::kZ, SectionCodec::Zstd, z.take()});
    return body;
}

Result<RailGeomTile> decode_rail_geom(const TileBody& body, const TileKey& key) {
    if (body.encoding != static_cast<std::uint16_t>(LayerEncoding::RailGeom)) return make_error(ErrorCode::Unsupported, "not a rail.geom body");
    if (body.encoding_version != kRailGeomEncodingVersion) return make_error(ErrorCode::Unsupported, "rail.geom: encoding version");
    if (body.sections.size() != 6) return bad("sections");
    const Section* s[6] = {body.find(rail_geom::kMeta), body.find(rail_geom::kIds), body.find(rail_geom::kAttrs),
        body.find(rail_geom::kCounts), body.find(rail_geom::kXy), body.find(rail_geom::kZ)};
    for (auto* x : s) if (!x) return bad("sections");
    ByteReader meta(s[0]->data);
    const std::uint32_t n = meta.u32(), total = meta.u32();
    if (!meta.ok()) return make_error(ErrorCode::Truncated, "rail.geom: meta truncated");
    if (!meta.at_end()) return bad("meta has trailing bytes");
    if (n > kRailGeomMaxPieces || total > kRailGeomMaxPoints) return make_error(ErrorCode::LimitExceeded, "rail.geom: counts");
    if (total < 2ull * n) return bad("point total smaller than two per piece");
    RailGeomTile tile; tile.key = key;
    // Every piece costs at least one byte per column: reject a count the section cannot hold before reserving.
    if (n > s[1]->data.size() || n > s[3]->data.size() || std::size_t{n} * 10 != s[2]->data.size()) return bad("section sizes");
    tile.pieces.resize(n);
    ByteReader ids(s[1]->data), attrs(s[2]->data), counts(s[3]->data), xy(s[4]->data), z(s[5]->data);
    std::int64_t way = 0;
    std::size_t points = 0;
    for (std::uint32_t i = 0; i < n; ++i) {
        auto& p = tile.pieces[i];
        if (i == 0) way = ids.svarint(); else {
            const std::uint64_t delta = ids.varint();
            if (delta > static_cast<std::uint64_t>(std::numeric_limits<std::int64_t>::max()) || way > std::numeric_limits<std::int64_t>::max() - static_cast<std::int64_t>(delta))
                return bad("way id overflow");
            way += static_cast<std::int64_t>(delta);
        }
        p.way_id = way;
        const std::uint64_t piece = ids.varint();
        if (piece > std::numeric_limits<std::uint32_t>::max()) return bad("piece index");
        p.piece = static_cast<std::uint32_t>(piece);
        p.cls = attrs.u8(); p.condition = static_cast<RailCondition>(attrs.u8()); p.flags = attrs.u16();
        p.layer = static_cast<std::int8_t>(attrs.u8()); p.tracks = attrs.u8(); p.service = static_cast<RailService>(attrs.u8());
        p.usage = static_cast<RailUsage>(attrs.u8()); p.gauge_mm = attrs.u16();
        const std::uint64_t count = counts.varint();
        if (!counts.ok() || count < 2 || count > kRailGeomMaxPiecePoints) return bad("point count");
        points += count;
        if (points > total) return bad("point total");
        p.xy.resize(count); p.z.resize(count);
        std::int64_t x = 0, y = 0, h = 0;
        for (std::uint64_t k = 0; k < count; ++k) {
            const std::int64_t dx = xy.svarint(), dy = xy.svarint(), dh = z.svarint();
            if (!xy.ok() || !z.ok()) return make_error(ErrorCode::Truncated, "rail.geom: truncated column");
            if (dx <= -kCoordLimit * 2 || dx >= kCoordLimit * 2 || dy <= -kCoordLimit * 2 || dy >= kCoordLimit * 2 || dh <= -std::int64_t{kZLimit} * 2 || dh >= std::int64_t{kZLimit} * 2)
                return bad("delta out of range");
            x += dx; y += dy; h += dh;
            if (x <= -kCoordLimit || x >= kCoordLimit || y <= -kCoordLimit || y >= kCoordLimit || h <= -kZLimit || h >= kZLimit) return bad("value out of range");
            p.xy[k] = {x, y}; p.z[k] = static_cast<std::int32_t>(h);
        }
    }
    if (!ids.ok() || !attrs.ok() || !counts.ok() || !xy.ok() || !z.ok()) return make_error(ErrorCode::Truncated, "rail.geom: truncated column");
    if (!ids.at_end() || !attrs.at_end() || !counts.at_end() || !xy.at_end() || !z.at_end()) return bad("trailing bytes in a column");
    if (points != total) return bad("point total");
    const auto valid = validate_rail_geom_tile(tile); if (!valid) return valid.error();
    return tile;
}

LayerDesc rail_geom_desc() {
    // Inputs ascending: elev.base, src.osm (coverage follows the second, RailGeomDeriver::coverage_inputs).
    return {{std::string(kRailGeomLayer), 1}, LayerKind::Baked, static_cast<std::uint16_t>(LayerEncoding::RailGeom), kRailGeomEncodingVersion,
        level_bit(2), {std::string(kElevBaseLayer), std::string(kSrcOsmLayer)},
        "railway centrelines in pieces with a height profile from the line itself (bridge/tunnel chains anchored at ground), "
        "structure flags and attributes"};
}

std::string rail_geom_config_text() {
    const auto& rules = builtin_rail_rules_de();
    return "g2m.rail.geom/1 enc=1 owner=first-vertex-mm piece=cut-at-first-vertex-ge-max-length"
           " profile=ground:smoothed-triangular+grade-limit-pinned-ends structure:chain-linear-dem-anchors window=src.osm-3x3"
           " portal-anchor=7a1"
           " rules=de:" + to_hex(std::span<const std::uint8_t>(rules.content_sha256.data(), rules.content_sha256.size()));
}
ContentHash RailGeomDeriver::config_hash() const { return Sha256::of(rail_geom_config_text()); }
}
