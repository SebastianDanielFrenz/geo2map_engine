// Street fidelity P1: country road rules as data (data/road_rules/de.json).
// Lane counts, lane/shoulder/verge widths, taper lengths and the added-lane
// side that the cross-section resolver falls back to when OSM is silent. Every
// number is an integer (millimetres or metres) so a decision made from them is
// platform independent. The JSON is embedded into the library at build time;
// its SHA-256 enters the roads.geom config hash.
#pragma once
#include "g2m/core/result.h"
#include "g2m/layer/tile_container.h"
#include <array>
#include <cstddef>
#include <cstdint>
#include <span>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace g2m {
enum class AddedLaneSide : std::uint8_t { Left = 0, Right = 1 };
enum class ParkingOrientation : std::uint8_t { Parallel = 0, Diagonal = 1, Perpendicular = 2 };

// Vertical traffic-calming and similar features at a way node (design 8.3, owner decision Q2). The values are the
// encoded kind byte of roads.geom section 4 and the order of RoadRules::traffic_calming.
enum class ProfileFeatureKind : std::uint8_t { Bump = 0, MiniBumps = 1, RumbleStrip = 2, Cushion = 3, Hump = 4,
    Table = 5, Chicane = 6, Choker = 7, Island = 8, Dip = 9 };
inline constexpr std::size_t kProfileFeatureKindCount = 10;
// Defaults of one traffic_calming=* value. A kind with height_mm > 0 is carved into the road heights; height 0 is
// render only (the lateral kinds: chicane, choker, island). width_mm 0 = across the full carriageway.
struct TrafficCalmingRules {
    std::int32_t length_mm = 0, height_mm = 0, width_mm = 0;
    friend bool operator==(const TrafficCalmingRules&, const TrafficCalmingRules&) = default;
};

// Junction geometry parameters (P2 design 3 and 8.1). Lengths are integer millimetres or metres as the key says.
// The loader enforces the tile-invariance proof of design 5.2/5.3 (cap sums against reach and carver radius).
struct JunctionRules {
    std::int32_t trim_min_mm = 0;              // smallest approach trim before the margin
    std::int32_t trim_margin_mm = 0;           // added to the farthest corner station
    std::int32_t edge_station_spacing_mm = 0;  // spacing of the sampled approach edge polylines
    std::int32_t link_trim_cap_m = 0;          // cap of a single trim in the cluster link test
    std::int32_t cluster_max_extent_m = 0;     // bounding box limit per axis
    std::int32_t cluster_max_nodes = 0;
    std::int32_t reach_max_m = 0;              // farthest ring vertex from the nearest cluster node
    std::int32_t bowtie_rounds = 0;            // trim-extension rounds before the hull fallback
    std::int32_t bowtie_step_mm = 0;           // trim extension per round
    // Corner fillets (kerb returns): each block corner of a junction ring is rounded with the radius of the lower of
    // its two approaches' classes (the road bed edge; the paved and outer edges follow concentrically).
    std::int32_t corner_radius_default_mm = 0;
    std::vector<std::pair<std::string, std::int32_t>> corner_radius_mm;   // highway -> radius, ascending; 0 = sharp
    std::int32_t corner_min_wedge_deg = 0;      // narrower wedges (gores, shallow merges) keep their sharp corner
    std::int32_t corner_tangent_max_percent = 0;// tangent length cap, in percent of the radius (acute wedges)
    std::int32_t corner_oneway_pair_radius_mm = 0;      // nose radius between two one-way approaches ...
    std::int32_t corner_oneway_pair_max_wedge_deg = 0;  // ... whose wedge is narrower than this (carriageway splits)
    // The corner radius of a highway class (the default for a class without its own entry).
    std::int32_t corner_radius_for(std::string_view highway) const;
    friend bool operator==(const JunctionRules&, const JunctionRules&) = default;
};
// Painted pattern of a crossing (design 2). The values are the encoded pattern byte of roads.geom section 6.
enum class CrossingPattern : std::uint8_t { None = 0, Zebra = 1, Dashes = 2, Ladder = 3, Lines = 4, Surface = 5 };
inline constexpr std::size_t kCrossingPatternCount = 6;
struct CrossingPatternEntry {
    std::string value;                          // the OSM tag value (key of the JSON map)
    CrossingPattern pattern = CrossingPattern::None;
    friend bool operator==(const CrossingPatternEntry&, const CrossingPatternEntry&) = default;
};
// Junction furniture parameters (P2 design 7 and 8.1).
struct FurnitureRules {
    std::int32_t crossing_zebra_mm = 0, crossing_signalised_mm = 0, crossing_other_mm = 0; // crossing length along the road
    std::int32_t stop_line_before_crossing_mm = 0;
    std::int32_t junction_stop_setback_mm = 0, sign_line_setback_mm = 0;
    std::int32_t line_stop_mm = 0, line_give_way_mm = 0, line_signal_mm = 0; // line width along the road
    std::int32_t signal_assoc_max_m = 0, sign_assoc_max_m = 0;
    std::int32_t signal_lateral_offset_mm = 0, signal_left_min_lanes = 0;
    std::vector<CrossingPatternEntry> crossing_markings; // crossing:markings=* value -> pattern, ascending value
    std::vector<CrossingPatternEntry> crossing_kind;     // crossing=* value -> pattern, ascending value
    friend bool operator==(const FurnitureRules&, const FurnitureRules&) = default;

    // The pattern a crossing:markings / crossing value maps to (nullptr for a value without an entry).
    const CrossingPattern* find_marking(std::string_view value) const;
    const CrossingPattern* find_kind(std::string_view value) const;
};
// Tunnel profiles (owner ruling 2026-10-15: long tunnels follow straight lines). A tunnel stretch at least
// straight_grade_min_length_m long takes one constant grade between its two portal heights instead of the DEM floor
// (served through RoadCompletionOptions::tunnel_chord_min_length_m, served_road_completion_options()); 0 = never.
struct TunnelRules {
    std::int32_t straight_grade_min_length_m = 0;
    // D65: a tunnel stretch at most cut_max_length_m long (0 = never) whose DEM rises between cut_hump_min_mm and
    // cut_hump_max_mm above the straight chord of its portal heights is an open cut through an embankment (a road
    // under a railway on a bank, tagged tunnel in OSM): its floor is the chord, and the terrain carve cuts the bank.
    std::int32_t cut_max_length_m = 0, cut_hump_min_mm = 0, cut_hump_max_mm = 0;
    friend bool operator==(const TunnelRules&, const TunnelRules&) = default;
};
// Underpass decks (owner decision 2026-10-15, racing_game PR #5; design D53-D58): where a ground stretch crosses a
// lower road (a lower OSM layer or a tunnel, not a bridge) over a DEM cutting, the profile ignores the cutting and the
// span over it is served as a deck (roads.geom encoding 9, section 8). All lengths integer mm or m; road_underpass.h
// uses them. enabled = false: no span is searched, the profile follows the DEM as before.
struct UnderpassRules {
    bool enabled = false;
    std::int32_t min_depth_mm = 0;       // depth of the DEM under the rim chord at the crossing, below it: no deck
    std::int32_t window_m = 0;           // half-width of the DEM window along the upper road (<= 60: the 65 m DEM margin)
    std::int32_t sag_radius_m = 0;       // the rim search tolerates a vertical sag of this radius (0 = plain hull)
    std::int32_t max_span_m = 0;         // longest deck; a longer one is declined (never a whole stretch)
    std::int32_t abutment_mm = 0;        // deck length added beyond the depression on each side
    std::int32_t depth_tol_mm = 0;       // the depression ends where the DEM is within this of the rim chord
    std::int32_t lower_margin_mm = 0;    // added to the lower road's half width for the deck core
    std::int32_t end_clearance_mm = 0;   // a deck must end at least this far from both stretch ends
    bool overpass = true;                // D64: mask the hump under an upper bridge (not in de.json: the tests' sabotage switch)
    friend bool operator==(const UnderpassRules&, const UnderpassRules&) = default;
};
// Constants the junction block is validated against (design 5.2, 5.3): the widest carve blend and the radius
// of roads.geom tiles a carved L0 tile reads.
inline constexpr std::int32_t kCarveBlendMaxM = 15;
inline constexpr std::int32_t kCarverRadiusM = 192;

struct RoadTaperRules {
    std::int32_t lane_add_m = 0;      // length of a counted lane that starts or ends
    std::int32_t width_change_m = 0;  // length of a width change of a matched band
    std::int32_t edge_band_m = 0;     // length of a non-counted band that starts or ends
    std::int32_t shift_m_per_m = 0;   // lateral shift length per metre of offset
    std::int32_t shift_min_m = 0, shift_max_m = 0;
    friend bool operator==(const RoadTaperRules&, const RoadTaperRules&) = default;
};
struct RoadClassRules {
    std::string highway;
    bool single_lane_both = false;    // untagged roads have one unmarked lane for both directions
    std::int32_t oneway_lanes = 1, twoway_lanes = 2; // default counted lanes (twoway = total)
    std::int32_t lane_mm = 0;         // lane width; for single_lane_both classes equal to single_lane_mm
    std::int32_t outer_lane_mm = 0;   // 0 = none; rightmost counted lane of a oneway road
    std::int32_t single_lane_mm = 0;  // 0 unless single_lane_both
    std::int32_t shoulder_outer_mm = 0, shoulder_inner_mm = 0, verge_mm = 0;
    bool markings_default = false;
    RoadTaperRules taper;
    std::int32_t min_edge_radius_m = 0; // the radius the taper was validated against
    friend bool operator==(const RoadClassRules&, const RoadClassRules&) = default;
};
struct RoadRules {
    std::string country;
    std::int32_t min_lane_mm = 0;
    AddedLaneSide added_lane_side = AddedLaneSide::Right;
    std::int32_t cycle_lane_mm = 0, sidewalk_mm = 0;
    std::int32_t tagged_shoulder_mm = 0; // shoulder=* asks for a shoulder where the class has none
    std::array<std::int32_t, 3> parking_mm{}; // indexed by ParkingOrientation
    std::array<TrafficCalmingRules, kProfileFeatureKindCount> traffic_calming{}; // indexed by ProfileFeatureKind
    JunctionRules junction;
    FurnitureRules furniture;
    TunnelRules tunnel;
    UnderpassRules underpass;
    std::vector<RoadClassRules> classes;      // ascending highway value
    ContentHash content_sha256{};             // SHA-256 of the JSON bytes the rules were loaded from

    // nullptr for a highway value without rules.
    const RoadClassRules* find_class(std::string_view highway) const;
    std::int32_t parking_width_mm(ParkingOrientation o) const { return parking_mm[static_cast<std::size_t>(o)]; }
    const TrafficCalmingRules& calming(ProfileFeatureKind k) const { return traffic_calming[static_cast<std::size_t>(k)]; }
};

// Strict loader: format "g2m.road_rules/1", integers only, unknown or missing
// keys rejected, every highway class of the default RoadStyle present, widths
// 0..20000 mm, tapers consistent with min_edge_radius_m, junction and furniture
// blocks (P2) with every length in (0, 200] m and the cluster caps consistent
// with reach_max_m and the carver radius, and the tunnel block (straight-grade
// threshold 0..100000 m, 0 = never).
Result<RoadRules> load_road_rules(std::span<const std::byte> json);
// The embedded data/road_rules/de.json, parsed once. A corrupt build aborts.
const RoadRules& builtin_road_rules_de();
// Bytes of the embedded file (for the pin test and diagnostics).
std::span<const std::byte> builtin_road_rules_de_bytes();
} // namespace g2m
