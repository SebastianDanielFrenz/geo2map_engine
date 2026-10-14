// g2m/layer/osm_roads.h - client-side OSM road extraction and exact
// rasteriser (PLAN.md 6.5's "client-side road raster ships first" note,
// 2026-09-27; see roads_plan.md, approach (a')). racing_game calls
// extract_road_segments/rasterize_road_segments per render chunk and writes
// the result into g2m::mesh::build_chunk's existing ClassWindow / land_class
// path (racing_game currently passes nullptr there). The physics bridge's
// cell_surface_ids (G2.5a-grip) instead uses paint_road_segments/
// paint_road_blocks below, which share rasterize_road_segments' own inner
// loop and clamps - which is why SurfaceKind extraction and the
// paved/unpaved LandClass split must be correct now even though grip is not
// wired to either until that step (open question, roads_plan.md).
//
// Determinism: every function here must be byte-identical Windows/Linux
// (tests/golden's "g2roads"/"g2paint" sets prove it). geom::project_e7_to_mm's
// own floor-to-mm projection is the only floating-point step; every
// geometric DECISION after that (rasterize_road_segments' inside test) is
// exact integer arithmetic (g2m::geom::sign_diff_products, PLAN G1c I1) - no
// floating point anywhere in the decision.
#pragma once

#include "g2m/core/geo/utm.h"
#include "g2m/core/geom/exact.h"
#include "g2m/core/tile_key.h"
#include "g2m/layer/classifier.h"
#include "g2m/layer/osm_data.h"

#include <array>
#include <cstdint>
#include <initializer_list>
#include <optional>
#include <span>
#include <string_view>
#include <utility>
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
enum class SpeedLimitKind { Unknown, Numeric, Unrestricted };
struct RoadSpeedLimit {
    SpeedLimitKind kind = SpeedLimitKind::Unknown;
    double kph = 0.0;
    bool conditional = false;
    friend bool operator==(const RoadSpeedLimit&, const RoadSpeedLimit&) = default;
};
RoadSpeedLimit parse_road_speed_limit(std::string_view text) noexcept;

struct RoadSegment {
    geom::PointMm a, b;
    std::int64_t half_width_mm = 0;
    LandClass land_class = LandClass::Unknown;
    SurfaceKind surface = SurfaceKind::Unknown;
    int rank = 0;
    std::int64_t way_id = 0;
    RoadSpeedLimit forward_limit{}, backward_limit{};
    bool bridge = false;

    friend bool operator==(const RoadSegment&, const RoadSegment&) = default;
};

struct RoadSpeedMatch {
    RoadSpeedLimit limit{};
    std::int64_t way_id = 0;
    double distance_mm = 1e100;
};
// Allocation-free ground-road match. Bridge decks require future 3D matching.
RoadSpeedMatch match_road_speed_limit(std::span<const RoadSegment> roads,
    geom::PointMm point, double heading_x, double heading_y) noexcept;

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

// A row-major (LandClass, SurfaceKind) -> resolved-surface-id lookup table,
// index = land_class * 16 + surface_kind (both enums are fixed at <= 16
// values by the g2m.terrain.class encoding, classifier.h's own "never
// renumber, only append" rule) - a physics-side caller's SurfaceTable
// resolution (paved -> "asphalt" id, unpaved -> "dirt" id, ...) folded into
// one flat array so `paint_road_segments` below never branches on the enum
// values themselves.
struct RoadValueLut {
    // Bumps here mean classifier.h grew past 16 LandClass or SurfaceKind
    // values - index() then needs a wider field (17 would already collide
    // with the next land_class's surface 1).
    static_assert(static_cast<int>(LandClass::SnowIce) < 16, "LandClass grew past 16 values");
    static_assert(static_cast<int>(SurfaceKind::Water) < 16, "SurfaceKind grew past 16 values");

    std::array<std::uint16_t, 256> v{};

    static constexpr std::size_t index(LandClass land_class, SurfaceKind surface) {
        return static_cast<std::size_t>(land_class) * 16 + static_cast<std::size_t>(surface);
    }
    std::uint16_t at(LandClass land_class, SurfaceKind surface) const { return v[index(land_class, surface)]; }

    // Every entry defaults to `off_road`; each (land_class, value) pair then
    // overwrites every SurfaceKind slot of that land_class - a LUT keyed by
    // LandClass alone (roads_plan.md's "unpaved by LandClass only" default:
    // this feature does not yet distinguish surfaces within a land class).
    static RoadValueLut by_land_class(std::uint16_t off_road,
                                       std::initializer_list<std::pair<LandClass, std::uint16_t>> class_values);
};

// Same painting rule as rasterize_road_segments above (shares its inner
// loop exactly - the exact integer inside test, round end caps and "later
// segments overwrite earlier ones" rank order are identical; see that
// function's own comment for the geometry), but resolves each covered
// sample to one `lut.at(seg.land_class, seg.surface)` id instead of a
// (land_class, surface) byte pair: out[j * row_stride + i] = that id.
// `row_stride` need not equal `lattice.nx` - a caller painting into one
// sub-rectangle of a larger owning buffer (e.g. paint_road_blocks below, or
// a physics tile's own cell_surface_ids) passes that buffer's own row width
// and an `out` already offset to the sub-rectangle's own corner; `out` must
// have at least `row_stride * (lattice.ny - 1) + lattice.nx` elements from
// its own start. Untouched samples are left at whatever `out` already held.
//
// Two clamps beyond rasterize_road_segments' own `params.min_half_width_mm`
// are always applied to every segment's half_width_mm, in this order (widen
// then cap):
//   - minimum 710 mm (0.71 m): the one-cell proof (osm_roads.h's own PLAN
//     reference, roads_plan.md section 2) - a physics cell centre and its
//     render-quad corner are never more than half a metre apart on either
//     axis, and half_width_mm >= 500*sqrt(2) ~= 707.1 mm is the exact
//     threshold at which "centre inside the capsule" implies "at least one
//     corner inside" (and the dual); 710 is a documented, slightly
//     conservative rounding of that bound;
//   - maximum kDefaultOsmHaloM * 1000 = 128,000 mm (128 m, g2m/layer/
//     osm_clip.h): the owner rule's own correctness bound - a tile's own
//     OSM extraction only ever keeps a way within its 128 m halo, so a
//     half-width past that could reach a point this call was never given
//     the covering segment for (relevant once this function's segments come
//     from a single tile's own list, e.g. via paint_road_blocks below).
//
// Allocation-free apart from `out` (owned by the caller).
void paint_road_segments(std::span<const RoadSegment> segs, const Lattice& lattice, const RasterParams& params,
                          const RoadValueLut& lut, std::span<std::uint16_t> out, std::size_t row_stride);

// What one L2 tile's road segments are, for the block helpers below. Only
// `segments` is meaningful when status == Segments; the tile is the OWNER
// (roads_plan.md section 2's OWNER RULE) of every lattice sample that maps
// to it - never a merge with any neighbour's list.
enum class RoadBlockStatus : std::uint8_t {
    Segments, // segments is that tile's own extract_road_segments() output
    None,     // that tile genuinely has no drivable ways (not a failure)
    Missing,  // not resident / load failed - the caller's own fill-miss case
};

struct RoadBlockLookupResult {
    RoadBlockStatus status = RoadBlockStatus::None;
    std::span<const RoadSegment> segments; // valid only if status == Segments
};

// Allocation-free lookup seam: a plain function pointer plus an opaque
// context instead of std::function, so the block helpers below never
// allocate or take a virtual call to find a tile's segments. Called once
// per BLOCK (not once per sample).
using RoadBlockLookupFn = RoadBlockLookupResult (*)(void* ctx, const TileKey& l2_key);

// How many blocks of each RoadBlockStatus a paint_road_blocks/
// rasterize_road_blocks call touched - lets a caller fold Missing/None into
// its own fill-miss counters without re-deriving the block partition.
struct RoadBlockStats {
    int segments_blocks = 0;
    int none_blocks = 0;
    int missing_blocks = 0;
};

// Partitions `lattice` into an aligned TileKey block grid at `block_level`
// (0 = 256 m, physics; 2 = 1024 m, render - roads_plan.md section 2), each
// lattice sample's easting/northing floored to whole metres first
// (g2m::floor_div - a Lattice sample position need not itself sit on an
// exact metre, e.g. a DGM1/physics cell-centre lattice sits at X.5 m).
//
// For each block that owns at least one lattice sample: derives that
// block's OWNER L2 tile (itself if block_level == 2, else its TileKey::
// parent() ancestor two levels up - the OWNER RULE), calls `lookup` for it
// exactly once, and - iff the result is RoadBlockStatus::Segments - paints
// only that block's own sub-rectangle of `out` with only that L2 tile's own
// segments, via paint_road_segments above (so the same 0.71 m minimum / 128
// m halo maximum clamps apply here too). A None or Missing block leaves its
// own sub-rectangle of `out` untouched (the caller's job to pre-fill, e.g.
// all-off_road). Never merges more than one tile's segment list into a
// single sample, unlike a cross-tile concatenation followed by a rank sort
// - so the result at any sample cannot depend on any other tile's own data
// or on lookup call order (roads_plan.md section 2's own contrast with
// R/core/src/road_classes.cpp's current stable_sort-by-rank-alone render
// merge).
//
// `row_stride`/`out` describe the SAME full buffer paint_road_segments
// would need for the whole `lattice` at once (row_stride >= lattice.nx,
// out.size() >= row_stride * (lattice.ny - 1) + lattice.nx).
// Allocation-free apart from `out` (owned by the caller).
RoadBlockStats paint_road_blocks(const Lattice& lattice, int block_level, geo::UtmZone zone, const RasterParams& params,
                                  const RoadValueLut& lut, RoadBlockLookupFn lookup, void* ctx,
                                  std::span<std::uint16_t> out, std::size_t row_stride);

// Same partition / owner rule / allocation-free contract as
// paint_road_blocks above, producing the render (land_class, surface) byte
// pair instead of a LUT id - `land_class`/`surface` must each have exactly
// `lattice.nx * lattice.ny` elements (rasterize_road_segments' own
// row-major layout, row_stride == lattice.nx). Applies the same 128 m halo
// MAXIMUM clamp as paint_road_blocks (the owner rule's correctness bound
// above applies here too), but NOT the 710 mm minimum - a render-resolution
// lattice does not need the physics one-cell proof.
RoadBlockStats rasterize_road_blocks(const Lattice& lattice, int block_level, geo::UtmZone zone,
                                      const RasterParams& params, RoadBlockLookupFn lookup, void* ctx,
                                      std::span<std::uint8_t> land_class, std::span<std::uint8_t> surface);

// The render LOD policy mapping a mesh chunk level to a RasterParams -
// moved here from racing_game's own road_raster_params_for_level
// (R/core/src/road_classes.cpp, which becomes a thin forwarder) so it is
// available to any g2m_layer caller directly. L0/L1: every drivable segment
// (min_rank 0, no widening). L2: tertiary and above (roads_plan.md R-2).
// L3: primary and above, widened to 3/4 of that level's own lattice spacing
// so a road stays continuous on that coarse a lattice. L4 and up:
// std::nullopt - no road classes at all at that level.
std::optional<RasterParams> raster_params_for_render_level(int level);

// The same render LOD policy as caller data (owner rule, G4 slice 5: LOD
// detail is an in-game setting the game passes in). levels[k] is the policy
// of chunk level k; a level past the end, or one with `enabled == false`,
// gets no road classes. default_render_road_lod() reproduces the compiled
// policy above exactly (L0-L1 rank 0; L2 tertiary; L3 primary, 6000 mm
// half-width floor = 3/4 of the 8 m spacing; none from L4).
struct RenderRoadLodLevel {
    bool enabled = true;
    int min_rank = 0;
    std::int64_t min_half_width_mm = 0;
    friend bool operator==(const RenderRoadLodLevel&, const RenderRoadLodLevel&) = default;
};
struct RenderRoadLod {
    std::vector<RenderRoadLodLevel> levels;
    friend bool operator==(const RenderRoadLod&, const RenderRoadLod&) = default;
};
const RenderRoadLod& default_render_road_lod();
std::optional<RasterParams> raster_params_for_render_level(int level, const RenderRoadLod& lod);
// Deterministic cache key of the policy AT ONE LEVEL (FNV-1a of the effective
// RasterParams or "none"): a render chunk's road classes depend only on its
// key and this value, so a settings change rebuilds only the chunk levels
// whose key changed.
std::uint64_t render_road_lod_key(const RenderRoadLod& lod, int level);

} // namespace g2m
