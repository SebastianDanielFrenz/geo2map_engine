// src/layer/osm_clip_index.cpp - see g2m/layer/osm_clip.h's OsmClipIndex
// (PLAN.md G1c I4). clip_osm() (osm_clip.cpp) remains untouched as the
// byte-for-byte oracle this index must reproduce; see that file's own
// five-step algorithm, which every comment below cross-references by step
// number.

#include "g2m/layer/osm_clip.h"

#include "g2m/core/geo/utm.h"
#include "g2m/core/geom/exact.h"
#include "g2m/core/geom/project.h"

#include <algorithm>
#include <optional>

namespace g2m {
namespace {

using namespace g2m::osm;
using Pt = geom::PointMm;

// The fixed 1024 m (level kRawTileLevel) bucket grid, in millimetres -
// exactly TileKey{.., kRawTileLevel, 0, 0}.size_m() * 1000, computed
// without needing a zone.
constexpr std::int64_t kBucketSizeMm = (TileKey::kLevel0SizeM << kRawTileLevel) * 1000;

// Floor/ceil division for b > 0 (floor_div is g2m::floor_div, tile_key.h).
constexpr std::int64_t ceil_div(std::int64_t a, std::int64_t b) { return -floor_div(-a, b); }

struct AxisRange {
    std::int64_t lo, hi;
};

// The exact (tight, not padded) inclusive range of bucket indices ix such
// that coordinate `c` lies within `halo_mm` of bucket ix's own
// [ix*size, ix*size+size) span - i.e. every ix whose dilated box
// (see bucket_box below) could contain `c`. Derived from
//   ix*size - halo <= c <= ix*size + size + halo
// so at most 2 wide whenever halo_mm < size_mm (always true here:
// kDefaultOsmHaloM/128/1024 m halos are all « the 1024 m bucket size),
// matching the plan's own "<=4 candidate buckets per node" (2 x 2).
AxisRange candidate_range(std::int64_t c, std::int64_t halo_mm, std::int64_t size_mm) {
    return {ceil_div(c - halo_mm, size_mm) - 1, floor_div(c + halo_mm, size_mm)};
}

struct BoxMm64 {
    std::int64_t x0, y0, x1, y1;
};

BoxMm64 bucket_box(std::int64_t ix, std::int64_t iy, std::int64_t halo_mm, std::int64_t size_mm) {
    return {ix * size_mm - halo_mm, iy * size_mm - halo_mm, ix * size_mm + size_mm + halo_mm,
            iy * size_mm + size_mm + halo_mm};
}

bool index_in_range(std::int64_t ix) { return ix >= TileKey::kIndexMin && ix <= TileKey::kIndexMax; }

} // namespace

OsmClipIndex OsmClipIndex::build(const osm::OsmData& set, geo::UtmZone zone, std::uint32_t halo_m) {
    OsmClipIndex idx;
    idx.set_ = &set;
    idx.zone_ = zone;
    idx.halo_m_ = halo_m;

    const geo::Utm utm(geo::kGRS80);
    const std::int64_t halo_mm = static_cast<std::int64_t>(halo_m) * 1000;
    const std::int64_t size_mm = kBucketSizeMm;

    // Project every node once (clip_osm's own step 1, done here instead of
    // once per clip() call).
    std::vector<std::optional<Pt>> pos(set.nodes.size());
    for (std::size_t i = 0; i < set.nodes.size(); ++i) {
        const Node& n = set.nodes[i];
        pos[i] = geom::project_e7_to_mm(utm, zone, n.lat, n.lon);
    }

    const auto node_index = [&](std::int64_t id) -> std::optional<std::uint32_t> {
        const Node* p = set.find_node(id);
        if (p == nullptr) return std::nullopt;
        return static_cast<std::uint32_t>(p - set.nodes.data());
    };

    // Reverse maps: node->ways (every way's own refs), and each of
    // node/way/relation -> the relations that directly reference it as a
    // member (node_to_relations_/way_to_relations_/relation_parents_ - the
    // plan's "node->ways, way->relations, relation->parents, member-node->
    // relations"). Built once, in a single deterministic pass over
    // ways/relations in ascending index order.
    for (std::size_t wi = 0; wi < set.ways.size(); ++wi) {
        for (std::int64_t ref : set.refs_of(set.ways[wi])) {
            if (const auto ni = node_index(ref)) idx.node_to_ways_[*ni].push_back(static_cast<std::uint32_t>(wi));
        }
    }
    idx.relation_parents_.assign(set.relations.size(), {});
    for (std::size_t ri = 0; ri < set.relations.size(); ++ri) {
        for (const Member& m : set.members_of(set.relations[ri])) {
            if (m.type == MemberType::Node) {
                if (const auto ni = node_index(m.ref)) {
                    idx.node_to_relations_[*ni].push_back(static_cast<std::uint32_t>(ri));
                }
            } else if (m.type == MemberType::Way) {
                const Way* w = set.find_way(m.ref);
                if (w != nullptr) {
                    const auto wi = static_cast<std::uint32_t>(w - set.ways.data());
                    idx.way_to_relations_[wi].push_back(static_cast<std::uint32_t>(ri));
                }
            } else {
                const Relation* r = set.find_relation(m.ref);
                if (r != nullptr) {
                    const auto child_ri = static_cast<std::size_t>(r - set.relations.data());
                    idx.relation_parents_[child_ri].push_back(static_cast<std::uint32_t>(ri));
                }
            }
        }
    }

    // Node pass: for every candidate bucket a node's position could be IN
    // (clip_osm step 1's node_in test, restricted to a handful of buckets
    // instead of the one query tile), register the node, and - via
    // node_to_ways_ - every way that has it as a ref (clip_osm step 2's
    // "some ref is IN" half of the way-inclusion test).
    for (std::size_t i = 0; i < set.nodes.size(); ++i) {
        if (!pos[i]) continue;
        const Pt p = *pos[i];
        const AxisRange rx = candidate_range(p.x, halo_mm, size_mm);
        const AxisRange ry = candidate_range(p.y, halo_mm, size_mm);
        for (std::int64_t ix = rx.lo; ix <= rx.hi; ++ix) {
            if (!index_in_range(ix)) continue;
            for (std::int64_t iy = ry.lo; iy <= ry.hi; ++iy) {
                if (!index_in_range(iy)) continue;
                const BoxMm64 box = bucket_box(ix, iy, halo_mm, size_mm);
                if (p.x < box.x0 || p.x > box.x1 || p.y < box.y0 || p.y > box.y1) continue; // safety re-check
                const TileKey key{zone, kRawTileLevel, static_cast<std::int32_t>(ix), static_cast<std::int32_t>(iy)};
                if (!key.valid()) continue;
                Bucket& bucket = idx.buckets_[key.packed()];
                bucket.node_in.push_back(static_cast<std::uint32_t>(i));
                if (const auto it = idx.node_to_ways_.find(static_cast<std::uint32_t>(i));
                    it != idx.node_to_ways_.end()) {
                    for (std::uint32_t wi : it->second) bucket.way_in.push_back(wi);
                }
            }
        }
    }

    // Way pass: clip_osm step 2's "some segment touches B" half of the
    // way-inclusion test, restricted to the candidate buckets spanning
    // each segment's own two endpoints (a segment far longer than one
    // bucket still gets every bucket it could touch, since the candidate
    // rectangle covers the whole endpoint-to-endpoint span, not just the
    // area near each endpoint).
    for (std::size_t wi = 0; wi < set.ways.size(); ++wi) {
        const auto refs = set.refs_of(set.ways[wi]);
        for (std::size_t k = 0; k + 1 < refs.size(); ++k) {
            const auto ai = node_index(refs[k]);
            const auto bi = node_index(refs[k + 1]);
            if (!ai || !bi || !pos[*ai] || !pos[*bi]) continue;
            const Pt a = *pos[*ai];
            const Pt b = *pos[*bi];
            const AxisRange rxa = candidate_range(a.x, halo_mm, size_mm);
            const AxisRange rxb = candidate_range(b.x, halo_mm, size_mm);
            const AxisRange rya = candidate_range(a.y, halo_mm, size_mm);
            const AxisRange ryb = candidate_range(b.y, halo_mm, size_mm);
            const std::int64_t ix_lo = std::min(rxa.lo, rxb.lo), ix_hi = std::max(rxa.hi, rxb.hi);
            const std::int64_t iy_lo = std::min(rya.lo, ryb.lo), iy_hi = std::max(rya.hi, ryb.hi);
            for (std::int64_t ix = ix_lo; ix <= ix_hi; ++ix) {
                if (!index_in_range(ix)) continue;
                for (std::int64_t iy = iy_lo; iy <= iy_hi; ++iy) {
                    if (!index_in_range(iy)) continue;
                    const BoxMm64 box = bucket_box(ix, iy, halo_mm, size_mm);
                    if (!geom::segment_touches_box(a.x, a.y, b.x, b.y, box.x0, box.y0, box.x1, box.y1)) continue;
                    const TileKey key{zone, kRawTileLevel, static_cast<std::int32_t>(ix),
                                       static_cast<std::int32_t>(iy)};
                    if (!key.valid()) continue;
                    idx.buckets_[key.packed()].way_in.push_back(static_cast<std::uint32_t>(wi));
                }
            }
        }
    }

    // Finalize: each bucket's node_in/way_in may have duplicate entries
    // (a node touches up to 4 buckets; a way can be registered into the
    // same bucket from more than one segment, or from both the node and
    // way pass) and are not necessarily in ascending order (way_in mixes
    // contributions from two passes) - sort+dedupe once here so clip()
    // itself never has to. Independent per bucket, so safe despite
    // iterating an unordered_map (PLAN.md D7): the sorted CONTENT of each
    // bucket does not depend on the order buckets are visited in.
    for (auto& [key, bucket] : idx.buckets_) {
        std::sort(bucket.node_in.begin(), bucket.node_in.end());
        bucket.node_in.erase(std::unique(bucket.node_in.begin(), bucket.node_in.end()), bucket.node_in.end());
        std::sort(bucket.way_in.begin(), bucket.way_in.end());
        bucket.way_in.erase(std::unique(bucket.way_in.begin(), bucket.way_in.end()), bucket.way_in.end());
    }

    return idx;
}

osm::OsmData OsmClipIndex::clip(const TileKey& key) const {
    const osm::OsmData& set = *set_;
    osm::OsmData out;

    static const Bucket kEmptyBucket{};
    const auto found = buckets_.find(key.packed());
    const Bucket& bucket = (found != buckets_.end()) ? found->second : kEmptyBucket;

    // clip_osm step 3: node_keep = node_in (bucket.node_in) plus every ref
    // of an included way.
    std::vector<std::uint32_t> node_keep(bucket.node_in.begin(), bucket.node_in.end());
    for (std::uint32_t wi : bucket.way_in) {
        for (std::int64_t ref : set.refs_of(set.ways[wi])) {
            const Node* n = set.find_node(ref);
            if (n != nullptr) node_keep.push_back(static_cast<std::uint32_t>(n - set.nodes.data()));
        }
    }
    std::sort(node_keep.begin(), node_keep.end());
    node_keep.erase(std::unique(node_keep.begin(), node_keep.end()), node_keep.end());
    const std::vector<std::uint32_t>& way_in = bucket.way_in; // already sorted+deduped at build time

    // clip_osm step 4: relations, transitively - here as a bounded BFS
    // (reverse-containment reachability from the tile's own node_keep/
    // way_in, closed upward through relation_parents_) instead of a
    // repeated full scan over every relation in the set.
    std::vector<char> rel_in(set.relations.size(), 0);
    std::vector<std::uint32_t> queue;
    const auto seed = [&](std::uint32_t ri) {
        if (rel_in[ri] == 0) {
            rel_in[ri] = 1;
            queue.push_back(ri);
        }
    };
    for (std::uint32_t ni : node_keep) {
        if (const auto it = node_to_relations_.find(ni); it != node_to_relations_.end()) {
            for (std::uint32_t ri : it->second) seed(ri);
        }
    }
    for (std::uint32_t wi : way_in) {
        if (const auto it = way_to_relations_.find(wi); it != way_to_relations_.end()) {
            for (std::uint32_t ri : it->second) seed(ri);
        }
    }
    for (std::size_t qi = 0; qi < queue.size(); ++qi) {
        for (std::uint32_t parent : relation_parents_[queue[qi]]) seed(parent);
    }

    // clip_osm step 5: assemble, re-interning strings - identical logic,
    // restricted to node_keep/way_in/rel_in instead of a full 0..N scan
    // (node_keep/way_in are already ascending original-index order, same
    // order a full scan would visit them in).
    std::vector<char> used(set.strings.size(), 0);
    const auto mark = [&](std::span<const Tag> tags) {
        for (const Tag& t : tags) {
            used[t.key] = 1;
            used[t.value] = 1;
        }
    };
    for (std::uint32_t ni : node_keep) mark(set.tags_of(set.nodes[ni]));
    for (std::uint32_t wi : way_in) mark(set.tags_of(set.ways[wi]));
    for (std::size_t ri = 0; ri < set.relations.size(); ++ri) {
        if (rel_in[ri] == 0) continue;
        mark(set.tags_of(set.relations[ri]));
        for (const Member& m : set.members_of(set.relations[ri])) used[m.role] = 1;
    }

    std::vector<std::uint32_t> remap(set.strings.size(), 0);
    for (std::size_t i = 0; i < set.strings.size(); ++i) {
        if (used[i] == 0) continue;
        remap[i] = static_cast<std::uint32_t>(out.strings.size());
        out.strings.push_back(set.strings[i]);
    }
    const auto copy_tags = [&](std::span<const Tag> tags, std::uint32_t& begin, std::uint32_t& count) {
        begin = static_cast<std::uint32_t>(out.tags.size());
        count = static_cast<std::uint32_t>(tags.size());
        for (const Tag& t : tags) out.tags.push_back(Tag{remap[t.key], remap[t.value]});
    };
    for (std::uint32_t ni : node_keep) {
        Node n = set.nodes[ni];
        copy_tags(set.tags_of(set.nodes[ni]), n.tags_begin, n.tags_count);
        out.nodes.push_back(n);
    }
    for (std::uint32_t wi : way_in) {
        Way w = set.ways[wi];
        const auto refs = set.refs_of(set.ways[wi]);
        w.refs_begin = static_cast<std::uint32_t>(out.way_refs.size());
        out.way_refs.insert(out.way_refs.end(), refs.begin(), refs.end());
        copy_tags(set.tags_of(set.ways[wi]), w.tags_begin, w.tags_count);
        out.ways.push_back(w);
    }
    for (std::size_t ri = 0; ri < set.relations.size(); ++ri) {
        if (rel_in[ri] == 0) continue;
        Relation r = set.relations[ri];
        const auto mem = set.members_of(set.relations[ri]);
        r.members_begin = static_cast<std::uint32_t>(out.members.size());
        for (Member m : mem) {
            m.role = remap[m.role];
            out.members.push_back(m);
        }
        copy_tags(set.tags_of(set.relations[ri]), r.tags_begin, r.tags_count);
        out.relations.push_back(r);
    }
    return out;
}

} // namespace g2m
