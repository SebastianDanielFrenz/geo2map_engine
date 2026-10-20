// g2m/layer/structure_floor.h - the design floor of a tunnel network (terrain migration phase 7a-2,
// docs/structure_classification.md 3.2). structure_classify.h's design_floor() is the floor of ONE chain with two end
// flags; this is the same rule on the graph of a connected tunnel component: the served floor is a function of the
// component, not of a tile and not of a chain, so one value stands at every shared node and the floor of a junction
// inside the tunnel network is the same whichever stretch is asked.
//
// The rule, restated on a graph (stations are 1 m apart along each stretch, the last one at the stretch end):
//  - portals are the nodes where the tunnel meets the surface (a joining ground road, a dead end); each carries an
//    anchor height. Every other node is interior: it is no anchor and constrains nothing;
//  - chord(p): the lowest straight-line bound over the pairs of ADJACENT portals (a, b) (a route a to b that passes no
//    other portal) with the station on a shortest such route: z_a + (z_b - z_a) * d(a, p) / d(a, b). On a chain with two
//    portals this is the chord of design_floor();
//  - u(p) = min(chord, ground - needed), or just the chord within needed / max_grade (graph distance) of a portal;
//  - f = the greatest max_grade-Lipschitz function below u in the graph metric, found by relaxing the stretches to a
//    fixed point (stretches in the order given, stations ascending), then raised to every portal cone
//    z_a - max_grade * d(a, p).
// Plain + - * / min max and comparisons on doubles, no libm: deterministic. The result does not depend on the order
// the stretches are given in (the relaxation reaches the greatest fixed point whatever the order), only the cost does.
//
// Infeasible anchors: two adjacent portals whose heights differ by more than a floor can climb between them cannot both
// be met. The reach rule of phase 7a-1 (portal_shift) is the handler: an excess over reach_factor * max_grade * d(a, b)
// is removed by moving the anchors toward each other (a free end alone if only one of the two is free, else half each),
// a node taking the largest move among its pairs. The moved anchors are returned; the caller re-fits whatever joins
// there. Pairs that are still beyond max_grade after one move (a node with several partners) are counted, not hidden.
#pragma once

#include <cstddef>
#include <vector>

namespace g2m::structure {

struct FloorStretch {
    int from = 0, to = 0;          // node indices in [0, node_count)
    double length_m = 0;           // > 0
    std::vector<double> ground;    // station_count(length_m) values, the lowest bare-earth height across the tube width; NaN = none
};
struct FloorPortal {
    int node = 0;
    double z_m = 0;                // the anchor height (the ground road's end level, or the DEM at a dead end)
    bool free_end = false;         // nothing joins here: it gives way first when the anchors cannot all be met
};
struct FloorNetwork {
    int node_count = 0;
    std::vector<FloorStretch> stretches;   // the caller's canonical order (ascending way id, stretch)
    std::vector<FloorPortal> portals;      // at most one per node
};
struct NetworkFloorParams {
    double max_grade = 0.05;
    double needed = 5.7;           // clearance + roof
    double reach_factor = 0.9;     // share of max_grade a floor is asked to climb between two anchored portals
    double quantum_m = 0;          // > 0: the move of an anchor is rounded to a multiple of it (1/256 m for the served heights)
};
struct NetworkFloor {
    bool ok = false;                              // false: no finite floor (no ground and no anchor in the component)
    bool converged = false;
    std::size_t passes = 0;
    std::vector<std::vector<double>> floor;       // per stretch, per station
    std::vector<double> node_floor;               // per node: the floor there (NaN for a node no stretch touches)
    std::vector<double> portal_z_m;               // per portal: the anchor after the reach handler
    std::vector<double> portal_shift_m;           // per portal: how far the handler moved it
    std::size_t pairs = 0;                        // adjacent portal pairs examined
    std::size_t pairs_shifted = 0;                // ... beyond the reach (the handler moved their anchors)
    std::size_t pairs_beyond_grade = 0;           // ... beyond max_grade outright before the handler
    std::size_t pairs_still_beyond_grade = 0;     // ... still beyond max_grade after it
    std::size_t anchor_misses = 0;                // portals whose floor ends above/below the moved anchor by more than 1 um
};

std::size_t station_count(double length_m);       // ceil(length) + 1
double station_m(double length_m, std::size_t k); // min(k, length)

NetworkFloor network_design_floor(const FloorNetwork& network, const NetworkFloorParams& params);

} // namespace g2m::structure
