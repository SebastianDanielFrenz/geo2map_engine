// Street fidelity P2 slice 12, request G-6 (terrain migration phase 6): census of ground stretches that pass over a
// lower road which is not tagged as a bridge or tunnel crossing (the B8 underpass case, vault G2M-011).
//
// The served carve fits a ground stretch's profile to the bare-earth DEM (elev.base). Where a lower road runs under it
// in a cutting or an untagged underpass, the DEM holds the lower road's surface, not the upper road's deck, so the
// upper profile dips. The census lists every such crossing in a roads.geom tile and measures the dip as the profile
// height against a straight chord over +-chord_half_m along the upper reference line.
//
// Upper: an entry with a profile, neither bridge nor tunnel. Lower: any way of the tile's graph whose segment crosses
// the upper stretch's node polyline (an exact integer-mm segment test, no shared node), on a lower OSM `layer` than the
// upper way or tagged tunnel, and not tagged bridge (a bridge is above). Same-layer crossings without a shared node are
// counted apart (`same_layer`). The station of a crossing is the reference station nearest the intersection point
// (1 m scan, then a 1 cm refinement). The chord is clamped to the stretch; a crossing with less than `min_side_m` of
// stretch on either side is counted as `short_side` and not measured.
#pragma once
#include "g2m/layer/road_geom_tile.h"
#include "g2m/layer/road_dem_sampler.h"
#include "g2m/layer/vertical_profile.h"
#include "g2m/layer/deriver.h"
#include <cstddef>
#include <cstdint>
#include <vector>

namespace g2m {
struct UnderpassCrossing {
    std::size_t upper_entry = 0;
    std::int64_t upper_way = 0, lower_way = 0;
    int upper_layer = 0, lower_layer = 0;
    bool lower_tunnel = false;
    double x_m = 0, y_m = 0;          // zone metres of the intersection
    double station_m = 0;             // on the upper reference line
    double profile_m = 0, chord_m = 0, dip_m = 0;   // dip = chord - profile (positive: the profile sags)
};
struct UnderpassCensusOptions {
    double chord_half_m = 60;
    double min_side_m = 20;
    double threshold_m = 0.3;
};
struct UnderpassCensus {
    std::size_t ground_entries = 0;   // entries examined as upper
    std::size_t same_layer = 0;       // crossings without a node on the same layer, neither tunnel nor bridge
    std::size_t short_side = 0;       // lower crossings too close to a stretch end to measure
    std::size_t over_threshold = 0;   // measured crossings with dip_m > threshold_m
    double max_dip_m = 0;
    std::vector<UnderpassCrossing> crossings;   // measured, sorted by (upper_way, upper_entry, station, lower_way)
};
UnderpassCensus road_underpass_census(const RoadGeomTile& tile, const UnderpassCensusOptions& options = {});

// ---- Underpass decks (owner decision 2026-10-15, racing_game PR #5; decisions D53-D58) ----
//
// Where a ground stretch crosses a lower road (the census rule above: a lower layer or a tunnel, never a bridge, no
// shared node) and the DEM holds a cutting there, the upper profile ignores the cutting (mask, D54) and the span over
// the cutting is served as a deck (roads.geom encoding 9, section 8; D56). Per crossing (D55):
//   1. station s_c: the reference station nearest the intersection of the two node polylines;
//   2. h(k), k = -W..W (W = rules.window_m, integer metres): the median of 7 lateral DEM samples (the profile's own
//      sampling) at s_c + k, beyond a stretch end on the tangent of the end pose;
//   3. rims a < 0 < b: the hull vertices around k = 0 of the upper convex hull of h(k) - k^2 / (2 sag_radius_m) (exact
//      int64; a sag of that radius or wider keeps the rims at the cutting edges); a vertex at 0 = no depression
//      (`shallow`), a rim at the window edge = a valley, not a cutting (`no_rim`);
//   4. depth d(k) = chord(k) - h(k) under the straight rim chord; d(0) < min_depth_mm: `shallow`;
//   5. deck = the lower road's footprint along the upper reference (half width + lower_margin_mm, plus the upper half
//      width along the crossing angle, sin clamped at 0.25) united with the contiguous k around 0 where
//      d > depth_tol_mm, widened by abutment_mm on each side, clipped to [a, b]; longer than max_span_m: `too_long`;
//      closer than end_clearance_mm to a stretch end: `reaches_end` (the deck must never be the whole stretch);
//   6. overlapping decks of one stretch merge (`merged`); the mask over a merged deck takes the highest chord.
// Tag rules (D61, owner answers on geo2map PR #13):
//   - a parking-structure storey is never an upper road nor a lower road (`in_structure`): a way tagged
//     service=parking_aisle or carrying a level=* tag. Its OSM layer describes the storey, not the ground, and the
//     bare-earth DEM holds neither the deck nor the space under it (the Kalbach and Main-Taunus-Zentrum car parks);
//   - a span whose lower ways are all tagged tunnel (any value but "no") is `covered`: the upper road stays on the
//     ground, so the profile is still masked over a DEM cutting, but no deck is served and the carve fills over the
//     tunnel (its floor and roof come from the tunnel itself; the A 66 over the continuous service tunnel). One lower
//     way that is not a tunnel makes the span a deck.
// A pure function of the stretch's reference, the crossing ways (complete in `crossings`, road_crossing_dependencies)
// and the DEM within window_m + 3 m of the stretch's own way (inside the 65 m DEM margin), so tile-invariant.
struct RoadUnderpassStats {
    std::size_t entries = 0;        // ground stretches searched
    std::size_t crossings = 0;      // lower crossings found on them
    std::size_t served = 0;         // crossings that gave a deck (before merging)
    std::size_t shallow = 0, no_rim = 0, too_long = 0, reaches_end = 0, dem_missing = 0;
    std::size_t merged = 0;         // decks merged into the previous one of the same stretch
    std::size_t decks = 0;          // decks stored (after merging)
    std::size_t covered = 0;        // D61: spans whose lower ways are all tunnels: masked, no deck (ground covers them)
    std::size_t in_structure = 0;   // D61: upper stretches and lower crossings skipped as parking-structure storeys
};
struct RoadUnderpassChord { double from_m = 0, to_m = 0, from_height_m = 0, to_height_m = 0; };
struct RoadUnderpassSpanFit {
    double start_m = 0, end_m = 0;           // deck, mm-rounded stations
    std::int32_t depth_mm = 0;
    std::vector<std::int64_t> lower_ways;    // ascending, unique
    std::vector<RoadUnderpassChord> chords;  // one per crossing, the rim chord its mask uses
    bool covered = false;                    // D61: every lower way is a tunnel - mask only, no deck
};
class RoadUnderpassFinder {
public:
    // `crossings` must hold every way crossing a segment of the ways asked about, and those ways themselves; the
    // references are kept (not owned).
    RoadUnderpassFinder(const RoadGraph& crossings, const RoadDemSampler& dem, geo::UtmZone zone, const UnderpassRules& rules);
    const UnderpassRules& rules() const noexcept { return rules_; }
    // The deck spans of one ground stretch (empty for an unknown way, a bridge or tunnel way, or rules off), sorted.
    std::vector<RoadUnderpassSpanFit> spans(std::int64_t way_id, RoadStretch stretch, const RoadReferenceLine& reference,
        RoadUnderpassStats* stats = nullptr) const;
    // The mask (D54): every sample strictly inside (0, length) whose station lies in a span takes the highest chord
    // covering it; a span without an interior sample gets one at its middle where that keeps 0.5 m to both neighbours.
    static void mask(const std::vector<RoadUnderpassSpanFit>& spans, std::vector<VerticalSample>& samples);
    // spans() then mask(): what build_road_profile and the completion tier call.
    void mask_samples(std::int64_t way_id, RoadStretch stretch, const RoadReferenceLine& reference, std::vector<VerticalSample>& samples) const;
private:
    struct Seg { std::size_t way; std::size_t index; };
    const RoadGraph& graph_;
    const RoadDemSampler& dem_;
    geo::UtmZone zone_;
    UnderpassRules rules_;
    std::vector<std::pair<std::uint64_t, Seg>> cells_;   // 64 m grid cell -> lower segment, sorted by cell then (way, index)
};
// The roads.graph level-2 tiles a segment of a `tile_graph` way passes through (segment_touches_box on each tile's
// square), `exclude` omitted; sorted, unique. A way crossing such a segment passes through the tile holding the
// intersection point, so that tile's roads.graph holds it complete (D57).
Result<std::vector<InputTileRef>> road_crossing_dependencies(const RoadGraph& tile_graph, geo::UtmZone zone,
    const TileKey* exclude = nullptr, std::size_t max_tiles = 1024);
} // namespace g2m
