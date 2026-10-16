// g2m/route/turn_restriction.h - OSM turn restrictions as routing input (G4
// slice 1, docs/routing_design.md). The record types and the extraction from
// canonical OSM live in g2m_layers (g2m/layer/road_restriction.h) since slice
// 3, because the g2m.roads.graph deriver stores them in the tile; this header
// keeps the names the routing code has always used.
#pragma once

#include "g2m/layer/road_restriction.h"

namespace g2m::route {

using g2m::extract_turn_restrictions;
using g2m::RestrictionKind;
using g2m::RestrictionSet;
using g2m::TurnRestriction;

} // namespace g2m::route
