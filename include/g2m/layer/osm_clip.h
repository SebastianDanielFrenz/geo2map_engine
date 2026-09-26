// g2m/layer/osm_clip.h - cuts one g2m.src.osm tile out of a larger OSM set
// (docs/formats/src_osm.md, "Selection contract").
//
// With B = the tile's square on its UTM grid, grown by halo_m on every side
// (a closed box, integer millimetres):
//  1. a node is IN if its projected position lies in B;
//  2. a way is included if any of its segments touches B (a one-node way:
//     if the node is IN); it keeps its full ref list;
//  3. the tile's nodes are the IN nodes plus every node any included way
//     refers to (so every way is complete and has its full geometry);
//  4. a relation is included if any member is an included node, way or
//     relation (closed transitively: a parent of an included relation is
//     included); its member list is kept complete, but members that are
//     not otherwise included are NOT added;
//  5. strings are re-interned to what the tile uses.
// Positions are OSM's 1e-7 degree values projected with g2m's own
// deterministic transverse Mercator (GRS80, the tile's zone) and floored to
// whole millimetres; all tests after that are exact integer arithmetic. A
// node more than 30 degrees of longitude from the zone's central meridian
// has no position ("far"): it is never IN, and a segment ending in it
// touches B only if its other end is IN.
//
// Deterministic: the output depends only on (set, key, halo_m).
#pragma once

#include "g2m/core/geo/utm.h"
#include "g2m/core/geom/exact.h"
#include "g2m/core/result.h"
#include "g2m/core/tile_key.h"
#include "g2m/layer/osm_data.h"
#include "g2m/layer/src_osm.h"

#include <cstdint>
#include <unordered_map>
#include <vector>

namespace g2m {

inline constexpr std::uint32_t kDefaultOsmHaloM = 128;

// `set` must satisfy OsmData::validate() (in particular: ways complete).
Result<OsmTile> clip_osm(const osm::OsmData& set, const TileKey& key, std::uint32_t halo_m = kDefaultOsmHaloM);

// Exposed for tests: does segment (ax,ay)-(bx,by) touch the closed box
// [x0,x1] x [y0,y1]? Exact for all int64 inputs with |value| < 2^62.
// (Now implemented in g2m::geom, core/geom/exact.h - PLAN G1c I1; brought
// into g2m so existing unqualified callers, e.g. tests/layer/test_osm.cpp's
// `using namespace g2m;`, keep working unchanged.)
using g2m::geom::segment_touches_box;

// A precomputed index over one OsmData set (PLAN.md G1c I4) that makes
// repeated clip_osm-equivalent cuts fast enough for a real import (tens of
// thousands of level-2 region tiles), instead of clip_osm's own O(whole
// set) work per call. `clip_osm` itself remains the byte-for-byte ORACLE:
// every OsmClipIndex::clip(key) result, re-wrapped as
// OsmTile{halo_m(), clip(key)}, must encode (encode_src_osm) identically to
// clip_osm(set, key, halo_m()) - see tests/unit/test_osm_clip_index.cpp.
//
// Built once from a 1024 m (TileKey level kRawTileLevel) bucket grid of
// projected node positions and way segments, plus reverse maps
// (node->ways, node/way->direct-member relations, relation->parent
// relations) that turn clip_osm's own O(all relations) fixpoint scan into a
// bounded BFS seeded from the tile's own included nodes/ways. `set` must
// satisfy OsmData::validate() - a documented precondition, exactly like
// clip_osm's own `set` parameter, not re-checked at runtime by either
// function; unlike clip_osm, clip() itself never fails (it has no error
// channel - build() does the one-time O(whole set) work, so clip() can be
// a pure lookup). `set` must outlive the index (only a pointer is kept -
// the whole point is to avoid copying a possibly-huge region set).
//
// Every `key` passed to clip() must share build()'s own `zone` (matching
// clip_osm's per-call `key.zone` projection, but done once here instead of
// once per call - correct only because every tile of one imported region
// shares one UTM zone/scheme, PLAN.md's own region model). clip() is
// thread-safe: it only reads state fixed at build() time, so any number of
// threads may call it concurrently on the same instance (the I4 concurrent-
// clip test relies on this).
class OsmClipIndex {
public:
    static OsmClipIndex build(const osm::OsmData& set, geo::UtmZone zone, std::uint32_t halo_m = kDefaultOsmHaloM);

    osm::OsmData clip(const TileKey& key) const;

    std::uint32_t halo_m() const { return halo_m_; }
    geo::UtmZone zone() const { return zone_; }

private:
    OsmClipIndex() = default;

    struct Bucket {
        std::vector<std::uint32_t> node_in; // ascending original node index, sorted+deduped at build time
        std::vector<std::uint32_t> way_in;  // ascending original way index, sorted+deduped at build time
    };

    const osm::OsmData* set_ = nullptr;
    geo::UtmZone zone_{};
    std::uint32_t halo_m_ = 0;

    // Lookup-only after build() (never iterated to decide output order or
    // content - only ever queried by packed TileKey/index), so an
    // unordered_map here does not make clip()'s result order-dependent.
    std::unordered_map<std::uint64_t, Bucket> buckets_;
    std::unordered_map<std::uint32_t, std::vector<std::uint32_t>> node_to_ways_;
    std::unordered_map<std::uint32_t, std::vector<std::uint32_t>> node_to_relations_;
    std::unordered_map<std::uint32_t, std::vector<std::uint32_t>> way_to_relations_;
    std::vector<std::vector<std::uint32_t>> relation_parents_; // size == set.relations.size()
};

} // namespace g2m
