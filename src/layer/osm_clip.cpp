// src/layer/osm_clip.cpp - see g2m/layer/osm_clip.h.

#include "g2m/layer/osm_clip.h"

#include "g2m/core/geo/utm.h"

#include <algorithm>
#include <cmath>
#include <optional>

namespace g2m {
namespace {

using namespace g2m::osm;

// ---- exact sign of a*b - c*d for int64 operands (portable 128-bit) ----

struct U128 {
    std::uint64_t hi, lo;
};

U128 umul(std::uint64_t a, std::uint64_t b) {
    const std::uint64_t a0 = a & 0xffffffffu, a1 = a >> 32, b0 = b & 0xffffffffu, b1 = b >> 32;
    const std::uint64_t p00 = a0 * b0, p01 = a0 * b1, p10 = a1 * b0, p11 = a1 * b1;
    const std::uint64_t mid = (p00 >> 32) + (p01 & 0xffffffffu) + (p10 & 0xffffffffu);
    return U128{p11 + (p01 >> 32) + (p10 >> 32) + (mid >> 32), (mid << 32) | (p00 & 0xffffffffu)};
}

int ucmp(U128 x, U128 y) {
    if (x.hi != y.hi) return x.hi < y.hi ? -1 : 1;
    if (x.lo != y.lo) return x.lo < y.lo ? -1 : 1;
    return 0;
}

std::uint64_t mag(std::int64_t v) { return v < 0 ? 0 - static_cast<std::uint64_t>(v) : static_cast<std::uint64_t>(v); }

// sign(a*b - c*d)
int sign_diff_products(std::int64_t a, std::int64_t b, std::int64_t c, std::int64_t d) {
    const U128 p = umul(mag(a), mag(b));
    const U128 q = umul(mag(c), mag(d));
    const bool pz = p.hi == 0 && p.lo == 0, qz = q.hi == 0 && q.lo == 0;
    const int ps = pz ? 0 : ((a < 0) != (b < 0) ? -1 : 1);
    const int qs = qz ? 0 : ((c < 0) != (d < 0) ? -1 : 1);
    if (ps != qs) return ps > qs ? 1 : -1;
    if (ps == 0) return 0;
    const int m = ucmp(p, q);
    return ps > 0 ? m : -m;
}

struct Pt {
    std::int64_t x, y;
};

} // namespace

bool segment_touches_box(std::int64_t ax, std::int64_t ay, std::int64_t bx, std::int64_t by, std::int64_t x0,
                         std::int64_t y0, std::int64_t x1, std::int64_t y1) {
    // Separating axes: x, y, and the segment's normal.
    if (std::max(ax, bx) < x0 || std::min(ax, bx) > x1 || std::max(ay, by) < y0 || std::min(ay, by) > y1) return false;
    const std::int64_t dx = bx - ax, dy = by - ay;
    int pos = 0, neg = 0;
    const std::int64_t cx[4] = {x0, x1, x1, x0};
    const std::int64_t cy[4] = {y0, y0, y1, y1};
    for (int i = 0; i < 4; ++i) {
        const int sg = sign_diff_products(dx, cy[i] - ay, dy, cx[i] - ax);
        if (sg == 0) return true; // a corner on the line (and the boxes overlap)
        (sg > 0 ? pos : neg)++;
    }
    return pos > 0 && neg > 0;
}

Result<OsmTile> clip_osm(const OsmData& set, const TileKey& key, std::uint32_t halo_m) {
    if (!key.valid()) return make_error(ErrorCode::InvalidArgument, "clip_osm: invalid tile key");
    if (auto v = set.validate(); !v) return make_error(ErrorCode::InvalidArgument, "clip_osm: " + v.error().message);

    const geo::Utm utm(geo::kGRS80);
    const double lon0 = geo::utm_central_meridian(key.zone.zone);
    const std::int64_t halo_mm = static_cast<std::int64_t>(halo_m) * 1000;
    const std::int64_t x0 = key.min_easting() * 1000 - halo_mm;
    const std::int64_t y0 = key.min_northing() * 1000 - halo_mm;
    const std::int64_t x1 = (key.min_easting() + key.size_m()) * 1000 + halo_mm;
    const std::int64_t y1 = (key.min_northing() + key.size_m()) * 1000 + halo_mm;

    // 1. project every node (index-aligned with set.nodes).
    std::vector<std::optional<Pt>> pos(set.nodes.size());
    std::vector<char> node_in(set.nodes.size(), 0);
    for (std::size_t i = 0; i < set.nodes.size(); ++i) {
        const Node& n = set.nodes[i];
        const double lat = static_cast<double>(n.lat) / 1e7;
        const double lon = static_cast<double>(n.lon) / 1e7;
        double dlon = lon - lon0;
        if (dlon > 180.0) dlon -= 360.0;
        if (dlon < -180.0) dlon += 360.0;
        if (dlon > 30.0 || dlon < -30.0) continue; // far
        const geo::UtmForward f = utm.forward(key.zone, lat, lon);
        if (!std::isfinite(f.easting) || !std::isfinite(f.northing)) continue;
        const Pt p{static_cast<std::int64_t>(std::floor(f.easting * 1000.0)),
                   static_cast<std::int64_t>(std::floor(f.northing * 1000.0))};
        pos[i] = p;
        node_in[i] = (p.x >= x0 && p.x <= x1 && p.y >= y0 && p.y <= y1) ? 1 : 0;
    }
    const auto node_index = [&](std::int64_t id) -> std::size_t {
        return static_cast<std::size_t>(set.find_node(id) - set.nodes.data());
    };

    // 2. ways.
    std::vector<char> way_in(set.ways.size(), 0);
    for (std::size_t wi = 0; wi < set.ways.size(); ++wi) {
        const auto refs = set.refs_of(set.ways[wi]);
        bool hit = false;
        for (std::size_t k = 0; k < refs.size() && !hit; ++k) {
            const std::size_t a = node_index(refs[k]);
            if (node_in[a]) {
                hit = true;
                break;
            }
            if (k + 1 < refs.size()) {
                const std::size_t b = node_index(refs[k + 1]);
                if (pos[a] && pos[b]) {
                    hit = segment_touches_box(pos[a]->x, pos[a]->y, pos[b]->x, pos[b]->y, x0, y0, x1, y1);
                }
            }
        }
        way_in[wi] = hit ? 1 : 0;
    }
    // 3. nodes: IN plus all refs of included ways.
    std::vector<char> node_keep = node_in;
    for (std::size_t wi = 0; wi < set.ways.size(); ++wi) {
        if (!way_in[wi]) continue;
        for (std::int64_t ref : set.refs_of(set.ways[wi])) node_keep[node_index(ref)] = 1;
    }
    // 4. relations, transitively.
    std::vector<char> rel_in(set.relations.size(), 0);
    for (bool changed = true; changed;) {
        changed = false;
        for (std::size_t ri = 0; ri < set.relations.size(); ++ri) {
            if (rel_in[ri]) continue;
            for (const Member& m : set.members_of(set.relations[ri])) {
                bool inc = false;
                if (m.type == MemberType::Node) {
                    const Node* n = set.find_node(m.ref);
                    inc = n && node_keep[static_cast<std::size_t>(n - set.nodes.data())];
                } else if (m.type == MemberType::Way) {
                    const Way* w = set.find_way(m.ref);
                    inc = w && way_in[static_cast<std::size_t>(w - set.ways.data())];
                } else {
                    const Relation* r = set.find_relation(m.ref);
                    inc = r && rel_in[static_cast<std::size_t>(r - set.relations.data())];
                }
                if (inc) {
                    rel_in[ri] = 1;
                    changed = true;
                    break;
                }
            }
        }
    }

    // 5. assemble, re-interning strings (set.strings is sorted, so keeping
    // used indices in index order keeps the new table sorted).
    std::vector<char> used(set.strings.size(), 0);
    const auto mark = [&](std::span<const Tag> tags) {
        for (const Tag& t : tags) used[t.key] = used[t.value] = 1;
    };
    for (std::size_t i = 0; i < set.nodes.size(); ++i) {
        if (node_keep[i]) mark(set.tags_of(set.nodes[i]));
    }
    for (std::size_t i = 0; i < set.ways.size(); ++i) {
        if (way_in[i]) mark(set.tags_of(set.ways[i]));
    }
    for (std::size_t i = 0; i < set.relations.size(); ++i) {
        if (!rel_in[i]) continue;
        mark(set.tags_of(set.relations[i]));
        for (const Member& m : set.members_of(set.relations[i])) used[m.role] = 1;
    }
    OsmTile tile;
    tile.halo_m = halo_m;
    OsmData& d = tile.data;
    std::vector<std::uint32_t> remap(set.strings.size(), 0);
    for (std::size_t i = 0; i < set.strings.size(); ++i) {
        if (!used[i]) continue;
        remap[i] = static_cast<std::uint32_t>(d.strings.size());
        d.strings.push_back(set.strings[i]);
    }
    const auto copy_tags = [&](std::span<const Tag> tags, std::uint32_t& begin, std::uint32_t& count) {
        begin = static_cast<std::uint32_t>(d.tags.size());
        count = static_cast<std::uint32_t>(tags.size());
        for (const Tag& t : tags) d.tags.push_back(Tag{remap[t.key], remap[t.value]});
    };
    for (std::size_t i = 0; i < set.nodes.size(); ++i) {
        if (!node_keep[i]) continue;
        Node n = set.nodes[i];
        copy_tags(set.tags_of(set.nodes[i]), n.tags_begin, n.tags_count);
        d.nodes.push_back(n);
    }
    for (std::size_t i = 0; i < set.ways.size(); ++i) {
        if (!way_in[i]) continue;
        Way w = set.ways[i];
        const auto refs = set.refs_of(set.ways[i]);
        w.refs_begin = static_cast<std::uint32_t>(d.way_refs.size());
        d.way_refs.insert(d.way_refs.end(), refs.begin(), refs.end());
        copy_tags(set.tags_of(set.ways[i]), w.tags_begin, w.tags_count);
        d.ways.push_back(w);
    }
    for (std::size_t i = 0; i < set.relations.size(); ++i) {
        if (!rel_in[i]) continue;
        Relation r = set.relations[i];
        const auto mem = set.members_of(set.relations[i]);
        r.members_begin = static_cast<std::uint32_t>(d.members.size());
        for (Member m : mem) {
            m.role = remap[m.role];
            d.members.push_back(m);
        }
        copy_tags(set.tags_of(set.relations[i]), r.tags_begin, r.tags_count);
        d.relations.push_back(r);
    }
    return tile;
}

} // namespace g2m
