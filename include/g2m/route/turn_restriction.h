// g2m/route/turn_restriction.h - OSM turn restrictions (type=restriction
// relations) as routing input (G4 slice 1, docs/routing_design.md).
//
// Source: canonical OSM (g2m.src.osm tiles keep every relation that has a
// kept member, so a restriction whose via node lies in a tile is in that
// tile's src.osm). roads.graph does NOT carry relations yet; see the design
// doc's "Turn restrictions" section for the planned roads.graph section.
#pragma once

#include "g2m/core/result.h"
#include "g2m/layer/osm_data.h"

#include <cstdint>
#include <string>
#include <vector>

namespace g2m::route {

enum class RestrictionKind : std::uint8_t { No = 0, Only = 1 };

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

} // namespace g2m::route
