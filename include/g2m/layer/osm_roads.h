// g2m/layer/osm_roads.h - client-side OSM road extraction and exact
// rasteriser (PLAN.md 6.5's "client-side road raster ships first" note,
// 2026-09-27; see roads_plan.md, approach (a')). racing_game calls
// extract_road_segments/rasterize_road_segments per render chunk and writes
// the result into g2m::mesh::build_chunk's existing ClassWindow / land_class
// path (racing_game currently passes nullptr there). Unchanged later, the
// SAME two functions feed G2.5's TerrainClassDeriver and the physics
// bridge's cell_surface_ids - which is why SurfaceKind extraction and the
// paved/unpaved LandClass split must be correct now even though grip is not
// wired to either until that step (open question, roads_plan.md).
//
// Determinism: both functions must be byte-identical Windows/Linux
// (tests/golden's "g2roads" set proves it). geom::project_e7_to_mm's own
// floor-to-mm projection is the only floating-point step; every geometric
// DECISION after that (rasterize_road_segments' inside test) is exact
// integer arithmetic (g2m::geom::sign_diff_products, PLAN G1c I1) - no
// floating point anywhere in the decision.
#pragma once

#include "g2m/core/geo/utm.h"
#include "g2m/core/geom/exact.h"
#include "g2m/layer/classifier.h"
#include "g2m/layer/osm_data.h"

#include <cstdint>
#include <span>
#include <string_view>
#include <vector>

namespace g2m {

// One entry of RoadStyle::highways: the default style for a `highway` tag
// value.
struct HighwayStyleEntry {
    std::string_view highway;   // e.g. "motorway", "residential", "track"
    LandClass land_class;       // PavedRoad or UnpavedRoad, before any `surface` override
    int rank;                   // ascending = more important (see RoadStyle's own comment)
    double default_width_m;     // used unless the way's own `width` or `lanes` tag overrides it
};

// One entry of RoadStyle::surfaces: the SurfaceKind and paved/unpaved a
// `surface` tag value maps to.
struct SurfaceStyleEntry {
    std::string_view surface;
    SurfaceKind surface_kind;
    LandClass land_class; // PavedRoad or UnpavedRoad
};

// A DATA TABLE (never code branches - PLAN.md 2.5's own IClassifier
// philosophy) mapping OSM tags to road style.
//
// `default_style()` is the brief's own default set: highway motorway
// through service and living_street are paved unless `surface` overrides,
// plus track (unpaved). A `highway` value not in `highways` - including
// footway, cycleway, path, steps, bridleway and pedestrian - is not
// drivable and is skipped by extract_road_segments, as is any way with
// `area=yes` or `tunnel=yes`. `bridge=yes` is NOT skipped: painted on the
// ground under the bridge until G3 (PLAN.md 6.5).
//
// Rank is ascending by importance (track=0 .. motorway=9, a `*_link` way
// shares its parent tier's rank) so a caller's level policy is a plain
// `rank >= threshold` test (roads_plan.md R-2: "L2: tertiary and above",
// "L3: primary and above") and so rasterize_road_segments' "later segments
// overwrite earlier ones" rule paints the more important road on top
// wherever two overlap.
struct RoadStyle {
    std::span<const HighwayStyleEntry> highways;
    std::span<const SurfaceStyleEntry> surfaces;

    static const RoadStyle& default_style();

    // nullptr if `highway` is not in `highways`.
    const HighwayStyleEntry* find_highway(std::string_view highway) const;
    // nullptr if `surface` is not in `surfaces`.
    const SurfaceStyleEntry* find_surface(std::string_view surface) const;
};

// One projected road segment: one edge (consecutive node pair) of a
// drivable way's polyline, ready to rasterise. `half_width_mm`/
// `land_class`/`surface` already reflect the way's own tags (width/lanes/
// surface overrides resolved by extract_road_segments); `rank` is copied
// from the matched HighwayStyleEntry.
struct RoadSegment {
    geom::PointMm a, b;
    std::int64_t half_width_mm = 0;
    LandClass land_class = LandClass::Unknown;
    SurfaceKind surface = SurfaceKind::Unknown;
    int rank = 0;

    friend bool operator==(const RoadSegment&, const RoadSegment&) = default;
};

// Extracts one RoadSegment per consecutive node pair of every drivable way
// in `set` (RoadStyle's skip rules above). Endpoints are projected with
// geom::project_e7_to_mm; a segment with either end "far" from `zone`
// (project_e7_to_mm's own 30-degree rule) is dropped - no real OSM way
// spans that far within one UTM zone.
//
// Width: the way's own `width` tag (metres - a clean std::strtod parse of
// the whole tag value, matching this codebase's src/decode/tiff/tfw.cpp
// precedent; a unit-suffixed or otherwise non-clean value is treated as
// absent, not an error) if present and > 0, else `lanes` (a clean-parsed
// integer-valued tag) x 3.25 m if present and > 0, else the matched
// HighwayStyleEntry::default_width_m. half_width_mm = the resulting width
// in metres, halved, converted to millimetres and rounded to the nearest
// whole mm (std::llround - exact for any finite double at this magnitude,
// and not on tools/ci.ps1's banned-libm-call list, unlike round/nearbyint/
// rint, which are not used here).
//
// SurfaceKind / paved-vs-unpaved: the way's own `surface` tag, if present
// and matched by RoadStyle::find_surface, gives both surface and
// land_class (overriding the highway's own default land_class - e.g. a
// residential way tagged `surface=dirt` becomes UnpavedRoad/Dirt, and a
// track tagged `surface=asphalt` becomes PavedRoad/Asphalt); otherwise
// surface is SurfaceKind::Unknown and land_class is the matched
// HighwayStyleEntry's own default.
//
// Output sorted by (rank, way id, segment index) - so
// rasterize_road_segments, which trusts the given array order for its own
// "later overwrites earlier" rule, needs no sort of its own, and the
// result does not depend on `set`'s own way/node storage order
// (tests/layer/test_osm_roads.cpp's shuffled-input-order case rebuilds
// `set` with ways/nodes added in a different order and checks the two
// extract_road_segments outputs are identical).
std::vector<RoadSegment> extract_road_segments(const osm::OsmData& set, geo::UtmZone zone, const RoadStyle& style);

// An axis-aligned grid of sample points in integer millimetres:
// sample(i, j) = {origin_e_mm + i * spacing_mm, origin_n_mm + j * spacing_mm},
// i in [0, nx), j in [0, ny). land_class/surface below are row-major over
// this grid, index = j * nx + i (row 0 = the origin's own northing, same
// "no implicit flip" convention as g2m::mesh::ClassWindow - a caller
// mapping this onto a ClassWindow's own south-origin rows picks
// origin_e_mm/origin_n_mm accordingly; this file has no ClassWindow
// dependency).
struct Lattice {
    std::int64_t origin_e_mm = 0, origin_n_mm = 0;
    std::int64_t spacing_mm = 0; // > 0
    int nx = 0, ny = 0;
};

// Rasterisation parameters.
struct RasterParams {
    // Segments with rank < min_rank are skipped entirely (a caller's LOD/
    // level policy, e.g. roads_plan.md's "tertiary and above" at L2).
    int min_rank = 0;
    // Every segment's half_width_mm is widened to at least this before
    // rasterising, so a thin road (e.g. a track) stays continuous on a
    // coarse lattice instead of falling between samples.
    std::int64_t min_half_width_mm = 0;
};

// Paints `segs` onto `lattice`, in ARRAY ORDER: a later segment overwrites
// an earlier one at any sample both cover, so a caller that needs "higher
// rank always wins" must pre-sort by rank first - extract_road_segments
// already does, so its own output needs no re-sort here.
// `land_class`/`surface` must each have exactly `lattice.nx * lattice.ny`
// elements (the row-major layout Lattice's own comment describes); only
// samples a segment actually covers are written, every other sample is
// left at whatever `land_class`/`surface` already held (the caller's job
// to pre-fill, e.g. all-Unknown/all-Unknown).
//
// For each segment, only the lattice samples inside its integer bounding
// box - widened by max(segment.half_width_mm, min_half_width_mm) on every
// side - are visited (g2m::floor_div, core/tile_key.h, for the box-to-
// index-range conversion). The inside test is ROUND END CAPS, via a
// clamped projection onto the segment (the two phrasings describe the
// same region: the clamped-nearest-point distance test traces exactly a
// stadium/capsule shape - a rectangle with a round cap at each end) -
// EXACT integer arithmetic throughout, no floating point in the decision:
//
//   w = sample - a, d = b - a, hw = max(segment.half_width_mm,
//   min_half_width_mm);
//   dot_wd = w.x*d.x + w.y*d.y, dot_dd = d.x*d.x + d.y*d.y;
//     - dot_wd <= 0: nearest point on the segment is `a`;
//       inside iff w.x*w.x + w.y*w.y <= hw*hw;
//     - dot_wd >= dot_dd: nearest point is `b`; let u = sample - b;
//       inside iff u.x*u.x + u.y*u.y <= hw*hw;
//     - otherwise (the perpendicular case): inside iff
//       cross*cross <= hw*hw*dot_dd, where cross = w.x*d.y - w.y*d.x -
//       the Lagrange identity (w.x*w.x+w.y*w.y)*dot_dd - dot_wd*dot_wd ==
//       cross*cross turns the division-by-dot_dd perpendicular-distance
//       test into this multiplication-only form. Checked via
//       g2m::geom::sign_diff_products(cross, cross, hw*hw, dot_dd), which
//       computes sign(cross*cross - hw*hw*dot_dd) using a 128-bit
//       intermediate product on each side (core/geom/exact.h/.cpp) -
//       needed because cross*cross and hw*hw*dot_dd both exceed int64
//       range for realistic segment lengths, even though cross, hw*hw and
//       dot_dd each individually fit.
//
// Documented assumption (not checked - true of every real OSM way edge
// between consecutive nodes, and of any sane min_half_width_mm): segment
// length (|d|) and the sample-to-`a` offset (|w|, bounded by the search
// box above) both stay well under 100,000,000 mm (100 km). Then dot_wd,
// dot_dd and w.x*w.x+w.y*w.y (each a sum of two products of ~1e8-magnitude
// int64 values, so up to ~2e16) stay comfortably inside int64 (max
// ~9.2e18), and cross (a difference of two such products, up to ~2e16)
// squared (~4e32) stays comfortably inside the 128-bit unsigned range
// sign_diff_products' own magnitude arithmetic uses (max ~3.4e38).
//
// Allocation-free apart from `land_class`/`surface` (owned by the caller).
void rasterize_road_segments(std::span<const RoadSegment> segs, const Lattice& lattice, const RasterParams& params,
                              std::span<std::uint8_t> land_class, std::span<std::uint8_t> surface);

} // namespace g2m
