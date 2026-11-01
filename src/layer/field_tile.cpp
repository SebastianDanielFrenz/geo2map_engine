#include "g2m/layer/field_tile.h"
#include "g2m/core/bytes.h"
#include "g2m/core/geo/angle.h"
#include "g2m/core/hash.h"
#include "g2m/layer/src_osm.h"
#include <algorithm>
#include <cmath>
#include <limits>
#include <tuple>

namespace g2m {
namespace {
using geom::PointMm;
Error malformed(std::string message) { return make_error(ErrorCode::Malformed, "fields.geom: " + message); }
std::int64_t cdiv(std::int64_t a, std::int64_t b) { // b > 0
    const std::int64_t q = a / b;
    return (a % b != 0 && a > 0) ? q + 1 : q;
}
// ceil of the x where edge (a, b) meets the horizontal line y; requires a.y != b.y.
std::int64_t crossing_ceil_x(PointMm a, PointMm b, std::int64_t y) {
    std::int64_t num = (y - a.y) * (b.x - a.x), den = b.y - a.y;
    if (den < 0) { num = -num; den = -den; }
    return a.x + cdiv(num, den);
}
bool point_less(const PointMm& a, const PointMm& b) { return a.x != b.x ? a.x < b.x : a.y < b.y; }
void put_string(ByteWriter& w, const std::string& s) { w.varint(s.size()); w.bytes(s.data(), s.size()); }
void put_ring(ByteWriter& w, const geom::Ring& ring) {
    w.varint(ring.size());
    PointMm prev{0, 0};
    for (std::size_t i = 0; i < ring.size(); ++i) {
        w.svarint(i == 0 ? ring[i].x : ring[i].x - prev.x);
        w.svarint(i == 0 ? ring[i].y : ring[i].y - prev.y);
        prev = ring[i];
    }
}
bool get_ring(ByteReader& r, geom::Ring& ring, std::size_t& vertex_budget) {
    const auto n = r.varint();
    if (!r.ok() || n < 3 || n > vertex_budget || n > r.remaining() / 2) return false;
    vertex_budget -= static_cast<std::size_t>(n);
    ring.reserve(static_cast<std::size_t>(n));
    PointMm prev{0, 0};
    for (std::uint64_t i = 0; i < n; ++i) {
        const auto dx = r.svarint(), dy = r.svarint();
        if (!r.ok()) return false;
        PointMm p{i == 0 ? dx : prev.x + dx, i == 0 ? dy : prev.y + dy};
        constexpr auto bound = std::int64_t{1} << 40;
        if (p.x <= -bound || p.x >= bound || p.y <= -bound || p.y >= bound) return false;
        ring.push_back(p); prev = p;
    }
    return true;
}
bool ring_canonical(const geom::Ring& ring, bool ccw) {
    if (ring.size() < 3) return false;
    const std::int64_t a = ring_twice_area_mm2(ring);
    if (ccw ? a <= 0 : a >= 0) return false;
    for (std::size_t i = 1; i < ring.size(); ++i) if (point_less(ring[i], ring[0])) return false;
    return true;
}
constexpr std::uint32_t kPlausibilityFlags = field_flag::SmallArea | field_flag::LargeArea | field_flag::Thin;
double ring_length_mm(const geom::Ring& ring) {
    double total = 0;
    for (std::size_t i = 0; i < ring.size(); ++i) {
        const auto& a = ring[i]; const auto& b = ring[(i + 1) % ring.size()];
        const double dx = static_cast<double>(b.x - a.x), dy = static_cast<double>(b.y - a.y);
        total += std::sqrt(dx * dx + dy * dy);
    }
    return total;
}
} // namespace

std::optional<std::uint64_t> pack_field_id(FieldSource source, std::int64_t osm_id, std::uint32_t piece) {
    if (static_cast<std::uint8_t>(source) > 2 || osm_id < 0 || osm_id > kFieldMaxOsmId || piece > 0xffffu) return std::nullopt;
    return (static_cast<std::uint64_t>(source) << 62) | (static_cast<std::uint64_t>(osm_id) << 16) | piece;
}
std::optional<FieldIdParts> unpack_field_id(std::uint64_t id) {
    const auto source = static_cast<std::uint8_t>(id >> 62);
    if (source > 2) return std::nullopt;
    return FieldIdParts{static_cast<FieldSource>(source), static_cast<std::int64_t>((id >> 16) & ((std::uint64_t{1} << 46) - 1)),
        static_cast<std::uint16_t>(id & 0xffffu)};
}

std::int64_t ring_twice_area_mm2(const geom::Ring& ring) {
    if (ring.size() < 3) return 0;
    std::int64_t sum = 0;
    const PointMm o = ring[0];
    for (std::size_t i = 1; i + 1 < ring.size(); ++i)
        sum += (ring[i].x - o.x) * (ring[i + 1].y - o.y) - (ring[i].y - o.y) * (ring[i + 1].x - o.x);
    return sum;
}
std::uint64_t field_polygon_area_mm2(const FieldPolygon& polygon) {
    std::int64_t twice = ring_twice_area_mm2(polygon.outer);
    for (const auto& h : polygon.holes) twice += ring_twice_area_mm2(h); // holes are clockwise: negative
    return twice > 0 ? static_cast<std::uint64_t>(twice / 2) : 0;
}
geom::BoxMm ring_bbox(const geom::Ring& ring) {
    geom::BoxMm b{std::numeric_limits<std::int64_t>::max(), std::numeric_limits<std::int64_t>::max(),
        std::numeric_limits<std::int64_t>::min(), std::numeric_limits<std::int64_t>::min()};
    for (const auto& p : ring) { b.w = std::min(b.w, p.x); b.e = std::max(b.e, p.x); b.s = std::min(b.s, p.y); b.n = std::max(b.n, p.y); }
    return b;
}
void normalise_field_ring(geom::Ring& ring, bool counter_clockwise) {
    if (ring.size() < 3) return;
    if ((ring_twice_area_mm2(ring) > 0) != counter_clockwise) std::reverse(ring.begin(), ring.end());
    const auto it = std::min_element(ring.begin(), ring.end(), point_less);
    std::rotate(ring.begin(), it, ring.end());
}
bool field_ring_contains(const geom::Ring& ring, PointMm p) {
    bool inside = false;
    const std::size_t n = ring.size();
    for (std::size_t i = 0; i < n; ++i) {
        const PointMm a = ring[i], b = ring[(i + 1) % n];
        const int o = geom::orient(a, b, p);
        if (o == 0 && p.x >= std::min(a.x, b.x) && p.x <= std::max(a.x, b.x) && p.y >= std::min(a.y, b.y) && p.y <= std::max(a.y, b.y))
            return true;
        if (a.y <= p.y && p.y < b.y) { if (o > 0) inside = !inside; }       // upward edge, p on its left
        else if (b.y <= p.y && p.y < a.y) { if (o < 0) inside = !inside; }  // downward edge, p on its left
    }
    return inside;
}
bool field_polygon_contains(const FieldPolygon& polygon, PointMm p) {
    // Even-odd over every ring; a point on any ring edge is inside.
    bool inside = false;
    const auto visit = [&](const geom::Ring& ring) -> bool {
        const std::size_t n = ring.size();
        for (std::size_t i = 0; i < n; ++i) {
            const PointMm a = ring[i], b = ring[(i + 1) % n];
            const int o = geom::orient(a, b, p);
            if (o == 0 && p.x >= std::min(a.x, b.x) && p.x <= std::max(a.x, b.x) && p.y >= std::min(a.y, b.y) && p.y <= std::max(a.y, b.y))
                return true;
            if (a.y <= p.y && p.y < b.y) { if (o > 0) inside = !inside; }
            else if (b.y <= p.y && p.y < a.y) { if (o < 0) inside = !inside; }
        }
        return false;
    };
    if (visit(polygon.outer)) return true;
    for (const auto& h : polygon.holes) if (visit(h)) return true;
    return inside;
}
void field_row_direction(const geom::Ring& outer, std::uint16_t& cdeg, std::uint32_t& edge_mm) {
    cdeg = 0; edge_mm = 0;
    if (outer.size() < 3) return;
    std::int64_t best = -1; PointMm a{}, b{};
    for (std::size_t i = 0; i < outer.size(); ++i) {
        const PointMm p = outer[i], q = outer[(i + 1) % outer.size()];
        const std::int64_t dx = q.x - p.x, dy = q.y - p.y, l2 = dx * dx + dy * dy;
        if (l2 > best) { best = l2; a = p; b = q; }
    }
    double angle = geo::atan2d(static_cast<double>(b.y - a.y), static_cast<double>(b.x - a.x));
    if (angle < 0) angle += 180.0;
    if (angle >= 180.0) angle -= 180.0;
    auto c = static_cast<std::int64_t>(std::floor(angle * 100.0 + 0.5));
    if (c >= 18000) c = 0;
    cdeg = static_cast<std::uint16_t>(c);
    edge_mm = static_cast<std::uint32_t>(std::sqrt(static_cast<double>(best)) + 0.5);
}
PointMm field_anchor(const FieldPolygon& polygon) {
    const auto bb = ring_bbox(polygon.outer);
    constexpr int kLines = 9;
    std::int64_t best_width = -1; PointMm best = polygon.outer.empty() ? PointMm{} : polygon.outer[0];
    std::vector<std::int64_t> xs;
    for (int k = 0; k < kLines; ++k) {
        const std::int64_t y = bb.s + ((2 * k + 1) * (bb.n - bb.s)) / (2 * kLines);
        xs.clear();
        const auto collect = [&](const geom::Ring& ring) {
            for (std::size_t i = 0; i < ring.size(); ++i) {
                const PointMm a = ring[i], b = ring[(i + 1) % ring.size()];
                if (a.y == b.y) continue;
                const std::int64_t lo = std::min(a.y, b.y), hi = std::max(a.y, b.y);
                if (lo <= y && y < hi) xs.push_back(crossing_ceil_x(a, b, y));
            }
        };
        collect(polygon.outer);
        for (const auto& h : polygon.holes) collect(h);
        std::sort(xs.begin(), xs.end());
        for (std::size_t i = 0; i + 1 < xs.size(); i += 2) {
            const std::int64_t width = xs[i + 1] - 1 - xs[i];
            if (width > best_width) { best_width = width; best = PointMm{xs[i] + width / 2, y}; }
        }
    }
    return best;
}
bool field_polygon_touches_box(const FieldPolygon& polygon, const geom::BoxMm& box) {
    const auto touches = [&](const geom::Ring& ring) {
        for (std::size_t i = 0; i < ring.size(); ++i) {
            const PointMm a = ring[i], b = ring[(i + 1) % ring.size()];
            if (geom::segment_touches_box(a.x, a.y, b.x, b.y, box.w, box.s, box.e, box.n)) return true;
        }
        return false;
    };
    if (touches(polygon.outer)) return true;
    for (const auto& h : polygon.holes) if (touches(h)) return true;
    return field_polygon_contains(polygon, PointMm{box.w, box.s});
}
void finalize_field_record(FieldRecord& r, const FieldRules& rules) {
    r.area_mm2 = field_polygon_area_mm2(r.polygon);
    r.bbox = ring_bbox(r.polygon.outer);
    field_row_direction(r.polygon.outer, r.row_dir_cdeg, r.row_edge_mm);
    r.anchor = field_anchor(r.polygon);
    r.flags &= ~kPlausibilityFlags;
    const double area_m2 = static_cast<double>(r.area_mm2) / 1e6;
    if (area_m2 < static_cast<double>(rules.plausible_min_area_m2)) r.flags |= field_flag::SmallArea;
    if (area_m2 > static_cast<double>(rules.plausible_max_area_m2)) r.flags |= field_flag::LargeArea;
    double perimeter = ring_length_mm(r.polygon.outer);
    for (const auto& h : r.polygon.holes) perimeter += ring_length_mm(h);
    if (perimeter <= 0 || 2.0 * static_cast<double>(r.area_mm2) / perimeter < static_cast<double>(rules.plausible_min_width_mm))
        r.flags |= field_flag::Thin;
}

// ---- codec -------------------------------------------------------------------------------------------------
Result<TileBody> encode_fields_geom(const FieldTile& tile) {
    const auto valid = validate_field_tile(tile); if (!valid) return valid.error();
    ByteWriter w;
    w.u64(tile.key.packed());
    w.varint(tile.counts.blocks); w.varint(tile.counts.subdivided); w.varint(tile.counts.slivers_dropped);
    w.varint(tile.counts.obstacles_ignored); w.varint(tile.counts.sliver_blocks);
    w.varint(tile.fields.size());
    for (const auto& f : tile.fields) {
        w.u64(f.id); w.u8(f.landuse); w.varint(f.flags); put_string(w, f.crop);
        w.varint(f.area_mm2); w.u16(f.row_dir_cdeg); w.varint(f.row_edge_mm);
        w.svarint(f.anchor.x); w.svarint(f.anchor.y);
        w.varint(f.polygon.holes.size());
        put_ring(w, f.polygon.outer);
        for (const auto& h : f.polygon.holes) put_ring(w, h);
    }
    w.varint(tile.declines.size());
    for (const auto& d : tile.declines) { w.u8(static_cast<std::uint8_t>(d.source)); w.svarint(d.osm_id); w.u8(static_cast<std::uint8_t>(d.reason)); }
    if (w.size() > kMaxSectionRawSize) return make_error(ErrorCode::LimitExceeded, "fields.geom: section size");
    return TileBody{kFieldsGeomEncoding, 1, {{1, SectionCodec::Zstd, w.take()}}};
}

Result<FieldTile> decode_fields_geom(const TileBody& body) {
    if (body.encoding != kFieldsGeomEncoding || body.encoding_version != 1)
        return make_error(ErrorCode::Unsupported, "fields.geom: encoding/version");
    if (body.sections.size() != 1 || body.sections[0].type != 1 || body.sections[0].data.size() > kMaxSectionRawSize)
        return malformed("sections");
    ByteReader r(body.sections[0].data);
    const auto key = TileKey::unpack(r.u64()); if (!key) return malformed("key");
    FieldTile tile; tile.key = *key;
    const auto counter = [&](std::uint32_t& out) {
        const auto v = r.varint();
        if (!r.ok() || v > std::numeric_limits<std::uint32_t>::max()) return false;
        out = static_cast<std::uint32_t>(v); return true;
    };
    if (!counter(tile.counts.blocks) || !counter(tile.counts.subdivided) || !counter(tile.counts.slivers_dropped) ||
        !counter(tile.counts.obstacles_ignored) || !counter(tile.counts.sliver_blocks)) return malformed("counts");
    const auto n = r.varint();
    if (!r.ok() || n > kFieldsMaxPerTile || n > r.remaining() / 20) return malformed("field count");
    tile.fields.reserve(static_cast<std::size_t>(n));
    std::size_t vertex_budget = kFieldsMaxVertices;
    for (std::uint64_t i = 0; i < n; ++i) {
        FieldRecord f;
        f.id = r.u64(); f.landuse = r.u8();
        const auto flags = r.varint(); if (!r.ok() || flags > field_flag::All) return malformed("flags");
        f.flags = static_cast<std::uint32_t>(flags);
        const auto crop_len = r.varint();
        if (!r.ok() || crop_len > kFieldsMaxCrop || crop_len > r.remaining()) return malformed("crop length");
        f.crop = std::string(r.string(static_cast<std::size_t>(crop_len)));
        f.area_mm2 = r.varint(); f.row_dir_cdeg = r.u16();
        const auto edge = r.varint(); if (!r.ok() || edge > std::numeric_limits<std::uint32_t>::max()) return malformed("row edge");
        f.row_edge_mm = static_cast<std::uint32_t>(edge);
        f.anchor.x = r.svarint(); f.anchor.y = r.svarint();
        const auto holes = r.varint();
        if (!r.ok() || holes > vertex_budget / 3) return malformed("hole count");
        if (!get_ring(r, f.polygon.outer, vertex_budget)) return malformed("outer ring");
        for (std::uint64_t h = 0; h < holes; ++h) {
            geom::Ring ring;
            if (!get_ring(r, ring, vertex_budget)) return malformed("hole ring");
            f.polygon.holes.push_back(std::move(ring));
        }
        const auto parts = unpack_field_id(f.id); if (!parts) return malformed("id");
        f.source = parts->source; f.osm_id = parts->osm_id; f.piece = parts->piece;
        f.bbox = ring_bbox(f.polygon.outer);
        tile.fields.push_back(std::move(f));
    }
    const auto declines = r.varint();
    if (!r.ok() || declines > r.remaining() / 3) return malformed("decline count");
    for (std::uint64_t i = 0; i < declines; ++i) {
        FieldDeclineRecord d;
        const auto source = r.u8(); d.osm_id = r.svarint(); const auto reason = r.u8();
        if (!r.ok() || source > 2 || reason >= kFieldDeclineCount) return malformed("decline record");
        d.source = static_cast<FieldSource>(source); d.reason = static_cast<FieldDeclineReason>(reason);
        tile.declines.push_back(d);
    }
    if (!r.at_end()) return malformed("truncated or trailing bytes");
    const auto valid = validate_field_tile(tile); if (!valid) return valid.error();
    return tile;
}

Result<void> validate_field_tile(const FieldTile& tile) {
    if (!tile.key.valid() || tile.key.level != 2) return malformed("key must be a valid level-2 key");
    if (tile.fields.size() > kFieldsMaxPerTile) return make_error(ErrorCode::LimitExceeded, "fields.geom: field count");
    const std::int64_t size = tile.key.size_m() * 1000;
    const geom::BoxMm box{tile.key.min_easting() * 1000, tile.key.min_northing() * 1000, tile.key.min_easting() * 1000 + size,
        tile.key.min_northing() * 1000 + size};
    std::size_t vertices = 0;
    for (std::size_t i = 0; i < tile.fields.size(); ++i) {
        const auto& f = tile.fields[i];
        if (i && tile.fields[i - 1].id >= f.id) return malformed("field order or duplicate id");
        const auto parts = unpack_field_id(f.id);
        if (!parts || parts->source != f.source || parts->osm_id != f.osm_id || parts->piece != f.piece) return malformed("id does not match its parts");
        if (f.crop.size() > kFieldsMaxCrop) return malformed("crop length");
        if ((f.flags & ~field_flag::All) != 0) return malformed("flags");
        if (!ring_canonical(f.polygon.outer, true)) return malformed("outer ring must be counter-clockwise, at least 3 vertices, smallest vertex first");
        vertices += f.polygon.outer.size();
        for (std::size_t h = 0; h < f.polygon.holes.size(); ++h) {
            if (!ring_canonical(f.polygon.holes[h], false)) return malformed("hole must be clockwise, at least 3 vertices, smallest vertex first");
            if (h && !point_less(f.polygon.holes[h - 1][0], f.polygon.holes[h][0])) return malformed("hole order");
            vertices += f.polygon.holes[h].size();
            if (!field_ring_contains(f.polygon.outer, f.polygon.holes[h][0])) return malformed("hole outside its outer ring");
        }
        if (vertices > kFieldsMaxVertices) return make_error(ErrorCode::LimitExceeded, "fields.geom: vertex count");
        if (f.area_mm2 != field_polygon_area_mm2(f.polygon) || f.area_mm2 == 0) return malformed("area does not match the rings");
        if (!(f.bbox == ring_bbox(f.polygon.outer))) return malformed("bbox");
        std::uint16_t cdeg; std::uint32_t edge;
        field_row_direction(f.polygon.outer, cdeg, edge);
        if (cdeg != f.row_dir_cdeg || edge != f.row_edge_mm) return malformed("row direction does not match the outer ring");
        if (!(f.anchor == field_anchor(f.polygon)) || !field_polygon_contains(f.polygon, f.anchor)) return malformed("anchor");
        const bool anchor_in_tile = f.anchor.x >= box.w && f.anchor.x < box.e && f.anchor.y >= box.s && f.anchor.y < box.n;
        if (((f.flags & field_flag::Owned) != 0) != anchor_in_tile) return malformed("Owned flag does not match the anchor");
        if (!field_polygon_touches_box(f.polygon, box)) return malformed("field does not touch the tile");
    }
    for (std::size_t i = 0; i < tile.declines.size(); ++i) {
        const auto& d = tile.declines[i];
        if (static_cast<std::uint8_t>(d.reason) >= kFieldDeclineCount || static_cast<std::uint8_t>(d.source) > 2) return malformed("decline record");
        if (i) {
            const auto& p = tile.declines[i - 1];
            if (std::tie(p.source, p.osm_id) >= std::tie(d.source, d.osm_id)) return malformed("decline order or duplicate");
        }
    }
    return {};
}

std::optional<std::size_t> field_at(const FieldTile& tile, PointMm p) {
    for (std::size_t i = 0; i < tile.fields.size(); ++i) {
        const auto& f = tile.fields[i];
        if (p.x < f.bbox.w || p.x > f.bbox.e || p.y < f.bbox.s || p.y > f.bbox.n) continue;
        if (field_polygon_contains(f.polygon, p)) return i;
    }
    return std::nullopt;
}

FieldMatch match_field(const FieldRef& saved, std::span<const FieldRecord> candidates, const FieldRules& rules) {
    const auto id = pack_field_id(saved.source, saved.osm_id, saved.piece);
    std::optional<std::size_t> same_id, same_block_hit, moved, nearest;
    std::int64_t nearest_d2 = 0;
    const std::int64_t max_mm = static_cast<std::int64_t>(rules.match_max_anchor_distance_m) * 1000;
    const auto lowest = [&](std::optional<std::size_t>& slot, std::size_t i) {
        if (!slot || candidates[i].id < candidates[*slot].id) slot = i;
    };
    for (std::size_t i = 0; i < candidates.size(); ++i) {
        const auto& c = candidates[i];
        const bool contains = field_polygon_contains(c.polygon, saved.anchor);
        const bool same_element = c.source == saved.source && c.osm_id == saved.osm_id;
        if (id && c.id == *id && contains) lowest(same_id, i);
        if (same_element && contains) lowest(same_block_hit, i);
        if (contains && c.landuse == saved.landuse && !same_element) lowest(moved, i);
        if (same_element) {
            const std::int64_t dx = c.anchor.x - saved.anchor.x, dy = c.anchor.y - saved.anchor.y, d2 = dx * dx + dy * dy;
            if (d2 <= max_mm * max_mm && (!nearest || d2 < nearest_d2 || (d2 == nearest_d2 && c.id < candidates[*nearest].id))) { nearest = i; nearest_d2 = d2; }
        }
    }
    if (same_id) return {FieldMatchKind::SameId, *same_id};
    if (same_block_hit) return {FieldMatchKind::SameBlock, *same_block_hit};
    if (moved) return {FieldMatchKind::Moved, *moved};
    if (nearest) return {FieldMatchKind::SameBlock, *nearest};
    return {};
}

LayerDesc fields_geom_desc() {
    return {{std::string(kFieldsGeomLayer), 1}, LayerKind::Baked, kFieldsGeomEncoding, 1, level_bit(2), {std::string(kSrcOsmLayer)},
        "farm field polygons: OSM agricultural blocks cut along tracks, roads, ditches, hedges and around water and buildings"};
}
} // namespace g2m
