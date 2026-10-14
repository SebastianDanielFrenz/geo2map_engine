// g2m/route/route_guidance.h - turn-by-turn manoeuvre list for a route
// (G4 slice 4, docs/routing_design.md section 5).
//
// A pure function of the route, the routing graph and the ways' tags. A
// manoeuvre is emitted only where the driver has a choice (a node with three
// or more edge ends) and something needs saying: a turn, a fork, a ramp, a
// roundabout, a U-turn, or a new road name/ref. Angle classes are the
// RouteConfig ones the cost model uses (heading change, positive = left).
// Instruction text and localisation stay in the game.
#pragma once

#include "g2m/layer/road_graph.h"
#include "g2m/route/route_search.h"

#include <cstdint>
#include <string>
#include <vector>

namespace g2m::route {

enum class ManeuverKind : std::uint8_t {
    Depart = 0,
    Continue,        // straight on onto a road with another name/ref
    SlightLeft, SlightRight,
    Left, Right,
    SharpLeft, SharpRight,
    KeepLeft, KeepRight, // a fork: another allowed exit lies within fork_max_deg of the chosen one
    UTurn,
    RoundaboutEnter,     // exit_number = the exit to take
    RoundaboutExit,      // only when the route starts on the roundabout
    RampOff,             // from a motorway/trunk onto a *_link
    RampOn,              // from another road onto a *_link
    Merge,               // from a *_link onto a motorway/trunk
    Arrive,
};
const char* to_string(ManeuverKind kind);

struct Maneuver {
    ManeuverKind kind = ManeuverKind::Depart;
    std::uint32_t step = 0;            // index into Route::arcs of the arc driven after the manoeuvre (Arrive: last arc)
    std::uint32_t node = kNoArc;       // graph node (kNoArc for Depart / Arrive)
    geom::PointMm position;
    std::uint64_t distance_from_start_mm = 0;
    std::uint64_t distance_to_next_mm = 0; // to the next manoeuvre (0 for Arrive)
    std::int32_t angle_deg = 0;        // heading change at the node, rounded, positive = left
    std::uint8_t exit_number = 0;      // roundabouts
    std::string name, ref;             // of the road driven after the manoeuvre
    std::string destination, destination_ref; // its signposting (OSM destination, destination:ref), mostly on links
};

struct GuidanceConfig {
    double fork_max_deg = 45.0; // another allowed exit this close to the chosen one makes a straight/slight move a fork
};

// `roads` supplies the way tags (name, ref, highway, junction) by way id: the
// RoadGraph the routing graph was built from. InvalidArgument for an
// unreachable or empty route or a way missing from `roads`.
Result<std::vector<Maneuver>> build_guidance(const RoutingGraph& graph, const RoadGraph& roads, const Route& route,
                                             const GuidanceConfig& config = {});

} // namespace g2m::route
