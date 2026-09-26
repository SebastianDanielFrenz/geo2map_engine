// tests/golden/golden_g2roads.cpp - G2roads golden hashes (PLAN.md 6.5,
// roads_plan.md): g2m::extract_road_segments + g2m::rasterize_road_segments
// (include/g2m/layer/osm_roads.h) over a fixed SYNTHETIC OsmData set (no
// testdata/fixtures dependency - a hand-built OsmBuilder set, same shape as
// golden_g2mesh.cpp's own analytic-source approach) - proves the segment
// extraction (incl. SurfaceKind / paved-vs-unpaved LandClass resolution) and
// the exact rasteriser are byte-identical Windows/Linux;
// testdata/golden/g2roads_golden.txt.
//
// The synthetic set exercises: a paved default-width way (motorway), an
// unpaved default-width way (track) whose `surface=asphalt` tag overrides it
// back to paved/Asphalt, a `width`-tag override, a `lanes`-tag override, a
// `tunnel=yes` way (skipped), an `area=yes` way (skipped), a non-drivable
// highway value (footway, skipped) and a `bridge=yes` way (NOT skipped -
// painted on the ground under it until G3, osm_roads.h's own RoadStyle
// comment).

#include "golden.h"

#include "g2m/core/geo/utm.h"
#include "g2m/core/hash.h"
#include "g2m/layer/osm_data.h"
#include "g2m/layer/osm_roads.h"

#include <algorithm>
#include <cstdint>
#include <limits>

namespace g2m_golden {
namespace {

using g2m::Fnv1a64;
using g2m::LandClass;
using g2m::RasterParams;
using g2m::RoadSegment;
using g2m::RoadStyle;
using g2m::SurfaceKind;
namespace geo = g2m::geo;
namespace osm = g2m::osm;

// Same shape as golden_g2mesh.cpp's own file-local Acc (deliberately not
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

// Fixed synthetic OSM data: a handful of ways near 50.1N/8.6E (well inside
// UTM zone 32N's 30-degree central-meridian bound), built in a DELIBERATELY
// scrambled id/insertion order - extract_road_segments' own ordering
// contract (rank, way id, segment index) must not depend on it (OsmBuilder
// itself always canonicalizes, but this keeps the golden set an honest proof
// of that too, not just an accident of insertion order).
osm::OsmData build_synthetic_set() {
    osm::OsmBuilder b;

    // Way 10: highway=motorway, 3 nodes / 2 segments - default width (paved).
    b.add_node(103, 501000000, 86000000);
    b.add_node(101, 501010000, 86000000);
    b.add_node(102, 501020000, 86005000);
    b.add_way(10, {101, 102}, {{"highway", "motorway"}});

    // Way 5: highway=track, surface=asphalt - overrides UnpavedRoad back to
    // PavedRoad/Asphalt.
    b.add_node(201, 501000000, 86020000);
    b.add_node(202, 501008000, 86020000);
    b.add_way(5, {201, 202}, {{"highway", "track"}, {"surface", "asphalt"}});

    // Way 20: highway=residential, width=8 (metres) - explicit width tag.
    b.add_node(301, 501030000, 86000000);
    b.add_node(302, 501038000, 86000000);
    b.add_way(20, {301, 302}, {{"highway", "residential"}, {"width", "8"}});

    // Way 15: highway=secondary, lanes=4, surface=gravel - lanes-derived
    // width (4 * 3.25 m) plus a surface override to UnpavedRoad/Gravel.
    b.add_node(401, 501030000, 86020000);
    b.add_node(402, 501038000, 86020000);
    b.add_way(15, {401, 402}, {{"highway", "secondary"}, {"lanes", "4"}, {"surface", "gravel"}});

    // Way 30: highway=residential, tunnel=yes - skipped entirely.
    b.add_node(501, 501050000, 86000000);
    b.add_node(502, 501058000, 86000000);
    b.add_way(30, {501, 502}, {{"highway", "residential"}, {"tunnel", "yes"}});

    // Way 31: highway=service, area=yes - skipped entirely.
    b.add_node(601, 501050000, 86020000);
    b.add_node(602, 501058000, 86020000);
    b.add_way(31, {601, 602}, {{"highway", "service"}, {"area", "yes"}});

    // Way 32: highway=footway - not in RoadStyle::default_style()'s table,
    // skipped.
    b.add_node(701, 501060000, 86000000);
    b.add_node(702, 501068000, 86000000);
    b.add_way(32, {701, 702}, {{"highway", "footway"}});

    // Way 25: highway=primary, bridge=yes - NOT skipped (painted on the
    // ground under the bridge, roads_plan.md).
    b.add_node(801, 501010000, 86040000);
    b.add_node(802, 501018000, 86040000);
    b.add_way(25, {801, 802}, {{"highway", "primary"}, {"bridge", "yes"}});

    auto set = b.build();
    return set ? std::move(*set) : osm::OsmData{};
}

Entry segments_entry(const std::vector<RoadSegment>& segs) {
    Acc a;
    a.u(segs.size());
    for (const RoadSegment& s : segs) {
        a.u(static_cast<std::uint64_t>(s.a.x));
        a.u(static_cast<std::uint64_t>(s.a.y));
        a.u(static_cast<std::uint64_t>(s.b.x));
        a.u(static_cast<std::uint64_t>(s.b.y));
        a.u(static_cast<std::uint64_t>(s.half_width_mm));
        a.u(static_cast<std::uint64_t>(s.land_class));
        a.u(static_cast<std::uint64_t>(s.surface));
        a.u(static_cast<std::uint64_t>(s.rank));
    }
    return a.done("g2roads.segments");
}

// Builds a lattice tightly covering every segment's own bounding box
// (widened by its half width, plus a fixed margin) - computed from the
// segments themselves so this file never hand-derives a UTM-projected
// coordinate. All-integer, so the lattice itself is exactly reproducible.
g2m::Lattice lattice_for(const std::vector<RoadSegment>& segs, std::int64_t spacing_mm) {
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
    g2m::Lattice lat;
    lat.spacing_mm = spacing_mm;
    lat.origin_e_mm = (min_x / spacing_mm) * spacing_mm;
    lat.origin_n_mm = (min_y / spacing_mm) * spacing_mm;
    lat.nx = static_cast<int>((max_x - lat.origin_e_mm) / spacing_mm) + 2;
    lat.ny = static_cast<int>((max_y - lat.origin_n_mm) / spacing_mm) + 2;
    return lat;
}

Entry raster_entry(const char* name, const std::vector<RoadSegment>& segs, const RasterParams& params,
                    std::int64_t spacing_mm) {
    const g2m::Lattice lat = lattice_for(segs, spacing_mm);
    std::vector<std::uint8_t> land_class(static_cast<std::size_t>(lat.nx) * static_cast<std::size_t>(lat.ny),
                                          static_cast<std::uint8_t>(LandClass::Unknown));
    std::vector<std::uint8_t> surface(land_class.size(), static_cast<std::uint8_t>(SurfaceKind::Unknown));
    g2m::rasterize_road_segments(segs, lat, params, land_class, surface);

    Acc a;
    a.u(static_cast<std::uint64_t>(lat.nx));
    a.u(static_cast<std::uint64_t>(lat.ny));
    for (std::uint8_t v : land_class) a.u(v);
    for (std::uint8_t v : surface) a.u(v);
    return a.done(name);
}

} // namespace

std::vector<Entry> compute_g2roads() {
    const osm::OsmData set = build_synthetic_set();
    const geo::UtmZone zone{32, geo::Hemisphere::North};
    const std::vector<RoadSegment> segs = g2m::extract_road_segments(set, zone, RoadStyle::default_style());

    std::vector<Entry> out;
    out.push_back(segments_entry(segs));
    out.push_back(raster_entry("g2roads.raster_l0", segs, RasterParams{/*min_rank=*/0, /*min_half_width_mm=*/0},
                                /*spacing_mm=*/1000));
    // roads_plan.md R-2's "L2: tertiary and above" LOD policy (rank 5, see
    // osm_roads.h's own RoadStyle comment) on a coarser lattice, plus
    // min_half_width_mm so the still-included track-turned-asphalt way (rank
    // 0, dropped by min_rank here anyway) is not the only thing exercising
    // that widening path - exercised instead by the secondary/gravel way at
    // a lattice coarse enough that its own half width alone would otherwise
    // skip samples.
    out.push_back(raster_entry("g2roads.raster_l2", segs, RasterParams{/*min_rank=*/5, /*min_half_width_mm=*/3000},
                                /*spacing_mm=*/4000));

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
