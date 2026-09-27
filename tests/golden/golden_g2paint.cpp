// tests/golden/golden_g2paint.cpp - G2paint golden hashes (G2.5a-grip
// commit G-a): g2m::paint_road_segments/paint_road_blocks/
// rasterize_road_blocks/RoadValueLut/raster_params_for_render_level
// (include/g2m/layer/osm_roads.h) - a SEPARATE golden set from "g2roads"
// (which already covers extract_road_segments + rasterize_road_segments;
// this one covers everything G-a added on top of it): proves the LUT
// painter, the owner-rule block partition (both block levels) and the
// moved render-LOD policy are all byte-identical Windows/Linux;
// testdata/golden/g2paint_golden.txt.

#include "golden.h"

#include "g2m/core/geo/utm.h"
#include "g2m/core/hash.h"
#include "g2m/core/tile_key.h"
#include "g2m/layer/osm_data.h"
#include "g2m/layer/osm_roads.h"

#include <algorithm>
#include <cstdint>
#include <limits>
#include <span>

namespace g2m_golden {
namespace {

using g2m::Fnv1a64;
using g2m::LandClass;
using g2m::Lattice;
using g2m::RasterParams;
using g2m::RoadBlockLookupFn;
using g2m::RoadBlockLookupResult;
using g2m::RoadBlockStats;
using g2m::RoadBlockStatus;
using g2m::RoadSegment;
using g2m::RoadStyle;
using g2m::RoadValueLut;
using g2m::SurfaceKind;
using g2m::TileKey;
namespace geo = g2m::geo;
namespace osm = g2m::osm;

// Same shape as golden_g2roads.cpp's own file-local Acc (deliberately not
// shared - see that file's header comment).
class Acc {
public:
    void u(std::uint64_t v) {
        h_.u64(v);
        ++n_;
    }
    Entry done(std::string name) const { return {std::move(name), n_, h_.value()}; }

private:
    Fnv1a64 h_;
    std::uint64_t n_ = 0;
};

// --- paint_road_segments over a small dedicated synthetic set ---------

// A handful of ways distinct from golden_g2roads.cpp's own synthetic set
// (that one already proves extract_road_segments + rasterize_road_segments;
// this file only needs enough ways to exercise paint_road_segments' LUT
// resolution and its own 0.71 m minimum clamp meaningfully): a default-width
// paved motorway, a track explicitly tagged far thinner than 710 mm (so the
// minimum clamp is the only thing making it paint at all), and an unpaved
// gravel secondary.
osm::OsmData build_paint_synthetic_set() {
    osm::OsmBuilder b;

    b.add_node(1001, 501000000, 87000000);
    b.add_node(1002, 501000000, 87010000);
    b.add_way(100, {1001, 1002}, {{"highway", "motorway"}}); // paved, default width 12 m

    b.add_node(1011, 501020000, 87000000);
    b.add_node(1012, 501020000, 87002000);
    b.add_way(101, {1011, 1012}, {{"highway", "track"}, {"width", "0.1"}}); // half_width 50 mm: below the 710 mm floor

    b.add_node(1021, 501040000, 87000000);
    b.add_node(1022, 501040000, 87008000);
    b.add_way(102, {1021, 1022}, {{"highway", "secondary"}, {"surface", "gravel"}}); // unpaved/gravel

    auto set = b.build();
    return set ? std::move(*set) : osm::OsmData{};
}

// Same shape as golden_g2roads.cpp's own file-local lattice_for (deliberately
// not shared - see that file's header comment): a lattice tightly covering
// every segment's own bounding box, computed from the segments themselves.
Lattice lattice_for(const std::vector<RoadSegment>& segs, std::int64_t spacing_mm) {
    std::int64_t min_x = std::numeric_limits<std::int64_t>::max();
    std::int64_t max_x = std::numeric_limits<std::int64_t>::min();
    std::int64_t min_y = std::numeric_limits<std::int64_t>::max();
    std::int64_t max_y = std::numeric_limits<std::int64_t>::min();
    for (const RoadSegment& s : segs) {
        const std::int64_t margin = s.half_width_mm + spacing_mm * 4;
        min_x = std::min({min_x, s.a.x - margin, s.b.x - margin});
        max_x = std::max({max_x, s.a.x + margin, s.b.x + margin});
        min_y = std::min({min_y, s.a.y - margin, s.b.y - margin});
        max_y = std::max({max_y, s.a.y + margin, s.b.y + margin});
    }
    Lattice lat;
    lat.spacing_mm = spacing_mm;
    lat.origin_e_mm = (min_x / spacing_mm) * spacing_mm;
    lat.origin_n_mm = (min_y / spacing_mm) * spacing_mm;
    lat.nx = static_cast<int>((max_x - lat.origin_e_mm) / spacing_mm) + 2;
    lat.ny = static_cast<int>((max_y - lat.origin_n_mm) / spacing_mm) + 2;
    return lat;
}

Entry painted_entry() {
    const osm::OsmData set = build_paint_synthetic_set();
    const geo::UtmZone zone{32, geo::Hemisphere::North};
    const std::vector<RoadSegment> segs = g2m::extract_road_segments(set, zone, RoadStyle::default_style());
    const Lattice lat = lattice_for(segs, /*spacing_mm=*/1000);

    const RoadValueLut lut = RoadValueLut::by_land_class(0, {{LandClass::PavedRoad, 100}, {LandClass::UnpavedRoad, 50}});
    std::vector<std::uint16_t> painted(static_cast<std::size_t>(lat.nx) * static_cast<std::size_t>(lat.ny), 0);
    g2m::paint_road_segments(segs, lat, RasterParams{}, lut, painted, static_cast<std::size_t>(lat.nx));

    Acc a;
    a.u(segs.size());
    a.u(static_cast<std::uint64_t>(lat.nx));
    a.u(static_cast<std::uint64_t>(lat.ny));
    for (std::uint16_t v : painted) a.u(v);
    return a.done("g2paint.painted");
}

// --- paint_road_blocks / rasterize_road_blocks: the owner-rule ---------

// Three adjacent L2 (1024 m) tiles at level 2, x = 100..102, y = 200: tile
// 100 and 102 each have their own hand-built segment list (never a merge of
// the two - the whole point of the owner rule), tile 101 genuinely has none
// (RoadBlockStatus::None); anything else the lookup is asked for (there is
// nothing else in this lattice's own range) falls through to Missing.
struct BlockCtx {
    TileKey tile_100, tile_101, tile_102;
    RoadSegment segs_100[2];
    RoadSegment segs_102[2];
};

RoadBlockLookupResult block_lookup(void* ctx_v, const TileKey& key) {
    const auto* ctx = static_cast<const BlockCtx*>(ctx_v);
    if (key == ctx->tile_100) return {RoadBlockStatus::Segments, std::span<const RoadSegment>(ctx->segs_100, 2)};
    if (key == ctx->tile_101) return {RoadBlockStatus::None, {}};
    if (key == ctx->tile_102) return {RoadBlockStatus::Segments, std::span<const RoadSegment>(ctx->segs_102, 2)};
    return {RoadBlockStatus::Missing, {}};
}

BlockCtx build_block_ctx() {
    const geo::UtmZone zone{32, geo::Hemisphere::North};
    BlockCtx ctx;
    ctx.tile_100 = TileKey{zone, 2, 100, 200};
    ctx.tile_101 = TileKey{zone, 2, 101, 200};
    ctx.tile_102 = TileKey{zone, 2, 102, 200};
    ctx.segs_100[0] = RoadSegment{{102500000, 205000000}, {102900000, 205000000}, 4000,
                                  LandClass::PavedRoad, SurfaceKind::Asphalt, 7};
    ctx.segs_100[1] = RoadSegment{{102600000, 205100000}, {102700000, 205300000}, 6000,
                                  LandClass::UnpavedRoad, SurfaceKind::Gravel, 2};
    ctx.segs_102[0] = RoadSegment{{104600000, 205000000}, {105000000, 205050000}, 3000,
                                  LandClass::PavedRoad, SurfaceKind::Concrete, 6};
    ctx.segs_102[1] = RoadSegment{{104700000, 205200000}, {104900000, 205200000}, 5000,
                                  LandClass::UnpavedRoad, SurfaceKind::Dirt, 1};
    return ctx;
}

// Spans tile 100's own west edge through tile 102's own east edge (crossing
// BOTH the 100|101 and 101|102 boundaries), a modest N slice (8 rows) well
// inside every tile's own y=200 span - keeps the golden computation small
// while still exercising a 3-tile partition at both block levels below.
Lattice block_lattice() {
    Lattice lat;
    lat.origin_e_mm = 100 * 1024 * 1000; // tile 100's west edge, exactly
    lat.origin_n_mm = 200 * 1024 * 1000; // tile *00's south edge, exactly
    lat.spacing_mm = 8000;
    lat.nx = (3 * 1024 * 1000) / lat.spacing_mm; // 100..103*1024 m: the whole 3-tile span
    lat.ny = 8;
    return lat;
}

Entry paint_blocks_entry(const char* name, int block_level) {
    BlockCtx ctx = build_block_ctx();
    const geo::UtmZone zone{32, geo::Hemisphere::North};
    const Lattice lat = block_lattice();
    const RoadValueLut lut = RoadValueLut::by_land_class(0, {{LandClass::PavedRoad, 100}, {LandClass::UnpavedRoad, 50}});
    std::vector<std::uint16_t> out(static_cast<std::size_t>(lat.nx) * static_cast<std::size_t>(lat.ny), 0xFFFF);

    const RoadBlockStats stats =
        g2m::paint_road_blocks(lat, block_level, zone, RasterParams{}, lut, &block_lookup, &ctx, out,
                                static_cast<std::size_t>(lat.nx));

    Acc a;
    a.u(static_cast<std::uint64_t>(lat.nx));
    a.u(static_cast<std::uint64_t>(lat.ny));
    a.u(static_cast<std::uint64_t>(stats.segments_blocks));
    a.u(static_cast<std::uint64_t>(stats.none_blocks));
    a.u(static_cast<std::uint64_t>(stats.missing_blocks));
    for (std::uint16_t v : out) a.u(v);
    return a.done(name);
}

Entry raster_blocks_entry() {
    BlockCtx ctx = build_block_ctx();
    const geo::UtmZone zone{32, geo::Hemisphere::North};
    const Lattice lat = block_lattice();
    // Ties rasterize_road_blocks to the real render L2 policy
    // (raster_params_for_render_level below) instead of a bare RasterParams{}.
    const RasterParams l2_params = g2m::raster_params_for_render_level(2).value();
    const std::size_t n = static_cast<std::size_t>(lat.nx) * static_cast<std::size_t>(lat.ny);
    std::vector<std::uint8_t> land_class(n, static_cast<std::uint8_t>(LandClass::Unknown));
    std::vector<std::uint8_t> surface(n, static_cast<std::uint8_t>(SurfaceKind::Unknown));

    const RoadBlockStats stats =
        g2m::rasterize_road_blocks(lat, /*block_level=*/2, zone, l2_params, &block_lookup, &ctx, land_class, surface);

    Acc a;
    a.u(static_cast<std::uint64_t>(lat.nx));
    a.u(static_cast<std::uint64_t>(lat.ny));
    a.u(static_cast<std::uint64_t>(stats.segments_blocks));
    a.u(static_cast<std::uint64_t>(stats.none_blocks));
    a.u(static_cast<std::uint64_t>(stats.missing_blocks));
    for (std::uint8_t v : land_class) a.u(v);
    for (std::uint8_t v : surface) a.u(v);
    return a.done("g2paint.raster_blocks_l2");
}

// --- raster_params_for_render_level: pins the moved render-LOD policy -

Entry render_params_entry() {
    Acc a;
    for (int level : {-1, 0, 1, 2, 3, 4, 5, 10}) {
        const std::optional<RasterParams> p = g2m::raster_params_for_render_level(level);
        a.u(p.has_value() ? 1 : 0);
        a.u(static_cast<std::uint64_t>(p ? p->min_rank : 0));
        a.u(static_cast<std::uint64_t>(p ? p->min_half_width_mm : 0));
    }
    return a.done("g2paint.render_params");
}

} // namespace

std::vector<Entry> compute_g2paint() {
    std::vector<Entry> out;
    out.push_back(painted_entry());
    out.push_back(paint_blocks_entry("g2paint.blocks_paint_l2", /*block_level=*/2));
    out.push_back(paint_blocks_entry("g2paint.blocks_paint_l0", /*block_level=*/0));
    out.push_back(raster_blocks_entry());
    out.push_back(render_params_entry());

    Fnv1a64 all;
    std::uint64_t n = 0;
    for (const Entry& e : out) {
        all.str(e.name).u64(e.count).u64(e.hash);
        n += e.count;
    }
    out.push_back({"all", n, all.value()});
    return out;
}

} // namespace g2m_golden
