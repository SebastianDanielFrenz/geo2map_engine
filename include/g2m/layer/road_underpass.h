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
} // namespace g2m
