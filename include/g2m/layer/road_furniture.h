// Street fidelity P2 slice 7 (design section 7): junction furniture computed from node tags and the P1 cross-sections:
// crossings (7.1), stop and give-way lines (7.2) and signals (7.3), each as a record with its own decline reason. Pure
// functions of a RoadGeomTile (entries with profile and cross-section), the context graph (node degrees, incident ways) and
// the rules. Library functions only: RoadGeomTile does not carry the records yet and no served byte changes (encoding 7,
// slice 8, stores them).
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
#include <vector>

namespace g2m {
enum class FurnitureKind : std::uint8_t { Crossing = 0, StopLine = 1, GiveWayLine = 2, Signal = 3 };
enum class FurnitureDecline : std::uint16_t {
    None = 0,
    StretchDeclined = 1,       // the node's stretch has no profile or no cross-section
    Tunnel = 2,
    JunctionNode = 3,          // a crossing on a degree >= 3 node: the crossing direction is unknown
    DirectionUnresolved = 4,   // a stop or give-way sign without a direction and without a junction end within reach
    NoLanes = 5,               // driving_lanes_at returns 0 for the travel direction
    Ambiguous = 6,             // a stop/give_way on a junction node whose approaches all have the same rank
    JunctionDeclined = 7,      // a junction-node signal or sign at a declined junction
};
inline constexpr std::size_t kFurnitureDeclineCount = 8;
enum FurnitureFlag : std::uint8_t {
    kFurnitureSignalised = 1,      // a crossing with crossing=traffic_signals
    kFurnitureLoweredKerb = 2,     // kerb=lowered or flush
    kFurnitureSpansJoin = 4,       // a crossing at a degree-two way join: the quad extends back over the join
    kFurnitureInsideJunction = 8,  // reserved: never set by slice 7 (it needs the junction, which only its owner tile builds, D27)
    kFurnitureLeftPost = 16,       // the second signal post, on the travel-left side
    kFurnitureDefinedMask = 31,
};
// One item. A declined record keeps its identity (kind, node, entry, travel) and has no extent (t_from = t_to = length = 0).
struct RoadFurniture {
    FurnitureKind kind = FurnitureKind::Crossing;
    FurnitureDecline decline = FurnitureDecline::None;
    std::int64_t node_id = 0;          // the source OSM node; the junction's owner node for a line or signal derived from a junction
    std::size_t entry_index = 0;       // the stretch it sits on (0 for a record of a declined junction)
    double station_m = 0;              // centre along the entry's reference line
    LaneDirection travel = LaneDirection::Forward;  // lines and signals: the direction controlled (Forward or Backward)
    double t_from = 0, t_to = 0;       // lateral extent in the reference frame, mm-rounded (signal: both = t of the post)
    double length_m = 0;               // along the road: crossing width, line width (a signal: 0), mm-rounded
    CrossingPattern pattern = CrossingPattern::None;
    std::uint8_t flags = 0;
    std::uint32_t junction_index = 0;  // 1 + the junction's index in the list given to build_road_furniture, else 0
    friend bool operator==(const RoadFurniture&, const RoadFurniture&) = default;
};
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
// The position of a signal post and the unit vector it faces (opposite to the travel direction, so towards the
// approaching traffic). Not stored: derived from the record and the entry.
struct SignalPost { double x = 0, y = 0, face_x = 0, face_y = 0; };
Result<SignalPost> signal_post(const RoadGeomTile& tile, const RoadFurniture& f);

// The station of a way node on a stretch's reference line (the shared helper of the traffic-calming and furniture
// stages): the start of the first straight piece at or after ref index `ref_index`, or the middle of the rounded corner
// arc just before it, whichever lies nearer the node.
std::optional<double> road_node_station(const RoadReferenceLine& ref, std::uint32_t ref_index, const geom::PointMm& node);
} // namespace g2m
