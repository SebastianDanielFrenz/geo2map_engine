// G3 foundation: identity-preserving road topology, before profile fitting,
// carving or routing. Built from canonical OSM, never raster capsules.
#pragma once
#include "g2m/layer/osm_roads.h"
#include <string>

namespace g2m {
struct RoadGraphNode {
    std::int64_t osm_id = 0;
    geom::PointMm position;
    friend bool operator==(const RoadGraphNode&, const RoadGraphNode&) = default;
};
struct RoadGraphWay {
    std::int64_t osm_id = 0;
    int rank = 0;
    // Original ordered refs and ALL tags: bridge/tunnel/layer, lane and turn
    // metadata, direction, width, incline and surface remain available.
    std::vector<std::int64_t> node_ids;
    std::vector<std::pair<std::string, std::string>> tags;
    friend bool operator==(const RoadGraphWay&, const RoadGraphWay&) = default;
};
struct RoadGraphEdge {
    // Stable identity across source tiles: (way id, original ref-pair index).
    // Every OSM shape node is kept; degree-two collapsing is a later view.
    std::int64_t way_id = 0;
    std::uint32_t segment_index = 0;
    std::int64_t from_node = 0, to_node = 0;
    friend bool operator==(const RoadGraphEdge&, const RoadGraphEdge&) = default;
};
struct RoadGraph {
    std::vector<RoadGraphNode> nodes; // ascending OSM node id
    std::vector<RoadGraphWay> ways;   // ascending OSM way id
    std::vector<RoadGraphEdge> edges; // ascending (way id, segment index)
    std::size_t far_zone_ways = 0;   // rejected as a WHOLE, never disconnected fragments
    std::size_t repeated_ref_pairs = 0; // consecutive identical ids omitted from edges
    friend bool operator==(const RoadGraph&, const RoadGraph&) = default;
};

// Keeps highway values accepted by style, excluding area=yes. Bridges and
// tunnels are retained with their tags, without painting them onto terrain.
// Connectivity uses OSM node ids: projected crossings or coincident positions
// alone never create a junction. This is an undirected geometric graph;
// routing directions/restrictions, fitting, wire formats and tile ownership
// are deliberately separate G3/G4 steps. Input must validate as canonical OSM.
Result<RoadGraph> build_road_graph(const osm::OsmData& input, geo::UtmZone zone,
                                 const RoadStyle& style = RoadStyle::default_style());
} // namespace g2m
