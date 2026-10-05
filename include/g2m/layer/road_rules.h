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
    std::vector<RoadClassRules> classes;      // ascending highway value
    ContentHash content_sha256{};             // SHA-256 of the JSON bytes the rules were loaded from

    // nullptr for a highway value without rules.
    const RoadClassRules* find_class(std::string_view highway) const;
    std::int32_t parking_width_mm(ParkingOrientation o) const { return parking_mm[static_cast<std::size_t>(o)]; }
    const TrafficCalmingRules& calming(ProfileFeatureKind k) const { return traffic_calming[static_cast<std::size_t>(k)]; }
};

// Strict loader: format "g2m.road_rules/1", integers only, unknown or missing
// keys rejected, every highway class of the default RoadStyle present, widths
// 0..20000 mm, tapers consistent with min_edge_radius_m.
Result<RoadRules> load_road_rules(std::span<const std::byte> json);
// The embedded data/road_rules/de.json, parsed once. A corrupt build aborts.
const RoadRules& builtin_road_rules_de();
// Bytes of the embedded file (for the pin test and diagnostics).
std::span<const std::byte> builtin_road_rules_de_bytes();
} // namespace g2m
