// g2m/layer/road_restriction.h - OSM turn restrictions (type=restriction
// relations) as data of the road graph (G4 slice 3, docs/routing_design.md).
//
// Source: canonical OSM (g2m.src.osm tiles keep every relation that has a
// kept member, complete with all members). The g2m.roads.graph deriver reads
// them with extract_turn_restrictions and stores the ones that concern a tile
// in the tile (encoding 6); g2m_route applies them in the search.
#pragma once

#include "g2m/core/result.h"
#include "g2m/layer/osm_data.h"

#include <cstdint>
#include <string>
#include <string_view>
#include <vector>

namespace g2m {

enum class RestrictionKind : std::uint8_t { No = 0, Only = 1 };

// The ten OSM restriction values a car rule can have, in a fixed order (the
// roads.graph encoding stores the index).
inline constexpr std::size_t kRestrictionValueCount = 10;
std::string_view restriction_value_name(std::size_t code);          // empty for code >= kRestrictionValueCount
bool restriction_value_code(std::string_view value, std::size_t& code); // false for an unknown value
RestrictionKind restriction_value_kind(std::size_t code);

// One from -> via -> to rule for cars. no_entry / no_exit relations (several
// from or to ways) are expanded to one No rule per (from, to) pair. Exactly
// one of via_node (non-zero) and via_ways (non-empty) is set; via_ways is in
// relation member order.
struct TurnRestriction {
    std::int64_t relation_id = 0;
    RestrictionKind kind = RestrictionKind::No;
    std::string value; // the OSM value, e.g. "no_left_turn", "only_straight_on"
    std::int64_t from_way = 0;
    std::int64_t via_node = 0;
    std::vector<std::int64_t> via_ways;
    std::int64_t to_way = 0;
    friend bool operator==(const TurnRestriction&, const TurnRestriction&) = default;
};

// Canonical order of a rule list: (relation id, from way, to way).
bool restriction_key_less(const TurnRestriction& a, const TurnRestriction& b);

struct RestrictionSet {
    // Sorted by (relation_id, from_way, to_way); unique.
    std::vector<TurnRestriction> restrictions;
    std::size_t relations_seen = 0;   // type=restriction relations looked at
    std::size_t not_for_cars = 0;     // restriction:<other vehicle> only, or except=*motorcar*
    std::size_t conditional_only = 0; // only restriction:conditional (not evaluated)
    std::size_t unknown_value = 0;    // value not in the no_* / only_* list
    std::size_t malformed = 0;        // member roles/types/counts do not fit the value
};

// Pure function of the relations in `data` (nodes and ways need not be
// present). Tag rules: restriction:motorcar wins over restriction; an
// `except` value listing motorcar (or motor_vehicle) drops the relation for
// cars; restriction:conditional is not evaluated. Never fails on bad data
// (counted instead); fails only on more than `max_restrictions` rules.
Result<RestrictionSet> extract_turn_restrictions(const osm::OsmData& data, std::size_t max_restrictions = 1u << 20);

} // namespace g2m
