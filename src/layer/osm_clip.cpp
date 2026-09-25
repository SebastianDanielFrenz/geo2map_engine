// src/layer/osm_clip.cpp - see g2m/layer/osm_clip.h.

#include "g2m/layer/osm_clip.h"

#include "g2m/core/geo/utm.h"
#include "g2m/core/geom/exact.h"
#include "g2m/core/geom/project.h"

#include <optional>

namespace g2m {
namespace {

using namespace g2m::osm;
using Pt = geom::PointMm;

} // namespace

Result<OsmTile> clip_osm(const OsmData& set, const TileKey& key, std::uint32_t halo_m) {
    if (!key.valid()) return make_error(ErrorCode::InvalidArgument, "clip_osm: invalid tile key");
    if (auto v = set.validate(); !v) return make_error(ErrorCode::InvalidArgument, "clip_osm: " + v.error().message);

    const geo::Utm utm(geo::kGRS80);
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
        const std::optional<Pt> p = geom::project_e7_to_mm(utm, key.zone, n.lat, n.lon);
        if (!p) continue; // far, or not finite
        pos[i] = p;
        node_in[i] = (p->x >= x0 && p->x <= x1 && p->y >= y0 && p->y <= y1) ? 1 : 0;
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
