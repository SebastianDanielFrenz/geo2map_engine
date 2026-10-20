// Railways R1/R2 (PLAN.md 6.6): rail rules as data (data/rail_rules/de.json), embedded at build time like the road
// rules. Which railway=* classes the g2m.rail.geom layer serves, their gauge / track / grade defaults, how a way is cut
// into pieces and how a bridge or tunnel chain is anchored. Every number is an integer (metres, millimetres or
// permille) so a decision made from them is platform independent. The file's SHA-256 enters the layer's config hash.
#pragma once
#include "g2m/core/result.h"
#include "g2m/layer/tile_container.h"
#include <cstddef>
#include <cstdint>
#include <span>
#include <string>
#include <string_view>
#include <vector>

namespace g2m {
struct RailClassRules {
    std::string railway;                // OSM value: rail, light_rail, subway, tram
    std::uint8_t id = 0;                // the encoded class byte, append-only (0 rail, 1 light_rail, 2 subway, 3 tram)
    bool enabled = false;               // served in the layer; a disabled class is only counted by the census tool
    std::int32_t gauge_mm = 0;          // default when the way has no usable gauge tag
    std::int32_t tracks = 1;            // default track count when the way has no tracks tag
    std::int32_t max_grade_permille = 0;
    friend bool operator==(const RailClassRules&, const RailClassRules&) = default;
};
struct RailRules {
    std::vector<RailClassRules> classes;      // ascending id
    bool construction_enabled = false;        // railway=construction + construction=<class> are served (condition 1)
    std::int32_t piece_max_length_m = 0;      // a way is cut at the first vertex this far from the piece start
    std::int32_t ground_spacing_m = 0;        // ground pieces are resampled to at most this vertex spacing
    std::int32_t smoothing_m = 0;             // half-width of the triangular smoothing of the ground profile
    std::int32_t piece_max_points = 0;        // hard cap on a piece's vertex count (spacing is widened to respect it)
    std::int32_t chain_walk_budget_m = 0;     // a bridge / tunnel chain is walked through connected ways up to this length
    std::int32_t chain_max_ways = 0;
    std::vector<std::string> bridge_not;      // bridge=<value> that is NOT a bridge
    std::vector<std::string> tunnel_not;      // tunnel=<value> that is NOT a tunnel
    std::int32_t dependency_margin_m = 0;     // DEM tiles are listed within this distance of every sampled segment
    ContentHash content_sha256{};             // of the file bytes
    const RailClassRules* find_class(std::string_view railway) const;
    const RailClassRules* find_class_id(std::uint8_t id) const;
    friend bool operator==(const RailRules&, const RailRules&) = default;
};
Result<RailRules> load_rail_rules(std::span<const std::byte> bytes);
std::span<const std::byte> builtin_rail_rules_de_bytes();
const RailRules& builtin_rail_rules_de();
inline constexpr std::uint8_t kRailClassMax = 3;
} // namespace g2m
