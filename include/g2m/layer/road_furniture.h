// Street fidelity P2 slice 7 (design section 7): junction furniture computed from node tags and the P1 cross-sections:
// crossings (7.1), stop and give-way lines (7.2) and signals (7.3), each as a record with its own decline reason. Pure
// functions of a RoadGeomTile (entries with profile and cross-section), the context graph (node degrees, incident ways) and
// the rules. Slice 8 serves them: build_road_geom_tile stores the result in RoadGeomTile::furniture and roads.geom encoding 7
// section 6 carries it (docs/formats/road_geom.md).
//
// Where a record lives (design 5.1). Furniture on an entry (crossings and the lines and signals of nodes inside a stretch
// or at a degree-one or degree-two stretch end) belongs to ONE entry, chosen by a rule that needs only the context graph
// (see build_road_furniture), and is emitted by every tile that holds that entry, identical in all of them. Furniture
// derived from a junction node (degree >= 3) belongs to the junction and is emitted only where the junction record is
// passed in, i.e. by the owner tile.
#pragma once
#include "g2m/layer/road_junction.h"
#include <array>
#include <cstdint>
#include <span>
#include <utility>
#include <vector>

namespace g2m {
// The record types (RoadFurniture, FurnitureKind, FurnitureDecline, FurnitureFlag) are in road_geom_records.h:
// RoadGeomTile carries the records (encoding 7, slice 8).

// Counters of one build, for the measurement tables and the tests.
struct FurnitureBuildStats {
    std::size_t crossing_nodes = 0;                       // nodes of the tile graph with highway=crossing
    std::size_t crossing_not_ours = 0;                    // ... whose chosen entry is not in this tile (another tile emits them)
    std::size_t crossing_placed = 0;
    std::array<std::size_t, kFurnitureDeclineCount> crossing_declined{};   // by FurnitureDecline value (index 0 unused)
    std::array<std::size_t, 6> crossing_patterns{};       // placed crossings by CrossingPattern
    // Control nodes (highway=traffic_signals or a signalised crossing / stop / give_way) handled by this build: those on an entry
    // (counted once, also when the chosen entry is another tile's) and those of a listed junction (once, by that junction; D31:
    // slice 7 counted a junction node twice).
    std::size_t signal_nodes = 0, sign_nodes = 0;
    std::size_t junction_controls_not_owned = 0;          // control nodes of degree >= 3 that no junction of the list contains (ClusterTooLarge records excluded)
    std::size_t controls_not_ours = 0;                    // entry-bound control nodes whose chosen entry is not in this tile
    std::size_t lines_stop = 0, lines_give_way = 0, lines_signal = 0;   // placed line records (one per contiguous lane group)
    std::size_t line_sets = 0;                            // (entry or approach, travel) pairs with at least one placed line
    std::size_t signals = 0, left_posts = 0;              // placed Signal records (left_posts included in signals)
    std::size_t junction_lines = 0;                       // lines derived from a junction (included in the line counts)
    std::size_t suppressed_junction_lines = 0;            // an approach already controlled by a node on its entry (7.2: case (a) wins)
    std::size_t moved_before_crossing = 0;                // lines moved upstream of a crossing
    std::size_t kept_without_lanes = 0;                   // ... not moved: the moved station has no lane of the travel (D31)
    std::array<std::size_t, kFurnitureDeclineCount> line_declined{};
};

// Every furniture record of the tile, sorted by (junction_index, entry_index, kind, station, node_id, travel, t_from).
// `context` must hold every way incident to the nodes of the tile's ways (as for build_node_junctions); `junctions` are the
// records of the tile's own build_road_junctions (any subset: furniture of a junction not in the list is not produced).
//
// Crossings (7.1): one record per node with highway=crossing, attached to ONE entry: for a node inside a stretch that
// entry; for a stretch end (degree 1 or 2) the incident stretch end that starts at the node (smallest way id, then ref;
// else the one that ends there), at its end station; for a node of degree >= 3 (JunctionNode decline) the smallest
// (way id, ref, at_end) incident stretch end. The choice reads only the context graph, so every tile that holds the entry
// agrees; a tile without that entry emits nothing for the node.
// Lines and signals (7.2, 7.3): see the design. Documented deviations: decisions D27 and D28 in the design document.
Result<std::vector<RoadFurniture>> build_road_furniture(const RoadGeomTile& tile, const RoadReferenceBuilder& context,
    std::span<const RoadJunction> junctions, const RoadRules& rules = builtin_road_rules_de(),
    FurnitureBuildStats* stats = nullptr);

// The world quad of a placed crossing or line: (s0, t_from), (s1, t_from), (s1, t_to), (s0, t_to) with s0/s1 =
// station -+ length/2, in the UTM frame (metres). A station outside the reference line [0, L] continues along the end
// tangent (a crossing at a way join extends back over the join). For a Signal record all four points are the post.
Result<std::array<LateralPoint, 4>> furniture_quad(const RoadGeomTile& tile, const RoadFurniture& f);
// The contiguous driving-lane groups of `travel` at station s of a cross-section (s clamped to [0, length_m]), ascending
// by t, unrounded; lanes closer than 0.5 mm form one group. A placed line record spans exactly one group, mm-rounded
// (the acceptance rule of design 7.2, which the roads.geom validation checks, 9.2). Returns the lane count.
std::size_t furniture_lane_groups(const RoadCrossSection& xs, double length_m, double station_m, LaneDirection travel,
    std::vector<std::pair<double, double>>& groups);
// The position of a signal post and the unit vector it faces (opposite to the travel direction, so towards the
// approaching traffic). Not stored: derived from the record and the entry.
struct SignalPost { double x = 0, y = 0, face_x = 0, face_y = 0; };
Result<SignalPost> signal_post(const RoadGeomTile& tile, const RoadFurniture& f);

// The station of a way node on a stretch's reference line (the shared helper of the traffic-calming and furniture
// stages): the start of the first straight piece at or after ref index `ref_index`, or the middle of the rounded corner
// arc just before it, whichever lies nearer the node.
std::optional<double> road_node_station(const RoadReferenceLine& ref, std::uint32_t ref_index, const geom::PointMm& node);
} // namespace g2m
