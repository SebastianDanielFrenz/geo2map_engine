#pragma once
#include "g2m/layer/deriver.h"
#include "g2m/layer/road_graph.h"
#include "g2m/layer/road_restriction.h"

namespace g2m {
inline constexpr std::string_view kRoadGraphLayer = "g2m.roads.graph";
// Format ids (a tile body's first u16, per layer): 4 = nodes without tags, 5 = nodes
// carry the whitelisted tags (street fidelity P1 slice 2), 6 = 5 plus the turn
// restrictions of the tile (G4 slice 3). The deriver and encode_road_graph's default write 6;
// 4 and 5 are read. The section embedded in roads.geom encodings 6-9 stays 5 (roads.geom
// does not carry restrictions), in encoding 5 it stays 4.
inline constexpr std::uint16_t kRoadGraphEncodingLegacy = 4;
inline constexpr std::uint16_t kRoadGraphEncodingTags = 5;
inline constexpr std::uint16_t kRoadGraphEncoding = 6;
inline constexpr std::size_t kRoadGraphMaxRestrictions = 200000, kRoadGraphMaxViaWays = 16;
inline constexpr std::uint32_t kRoadGraphHaloM = 128;
struct RoadGraphTile {
    TileKey key;
    std::uint32_t halo_m = kRoadGraphHaloM;
    RoadGraph graph;
    // Indices into graph.edges, strictly ascending. The containing tile of
    // each edge's floor-rounded integer-mm midpoint is its unique owner.
    std::vector<std::uint32_t> owned_edges;
    // Turn restrictions for cars (encoding 6; empty when decoded from 4 or 5). A rule is stored
    // in every tile whose graph holds its via node (via_node != 0) or at least one of its via
    // ways; its from and to ways need not be in the tile (they usually are). Sorted by
    // (relation id, from way, to way), unique. The region union is deduplicated on that key.
    std::vector<TurnRestriction> restrictions;
    friend bool operator==(const RoadGraphTile&, const RoadGraphTile&) = default;
};
// Canonical, bounded encoding 6/version 1 (5 or 4 on request: encoding 5 drops the restrictions,
// encoding 4 also the node tags).
// Complete way geometry and tags remain available beyond the halo; ownership
// never changes edge identities. decode accepts 4 (nodes carry no tags), 5 and 6.
Result<TileBody> encode_road_graph(const RoadGraphTile& tile, std::uint16_t encoding = kRoadGraphEncoding);
Result<RoadGraphTile> decode_road_graph(const TileBody& body);
Result<void> validate_road_graph_tile(const RoadGraphTile& tile);
Result<void> validate_road_graph(const RoadGraph& graph);
// `restrictions` is the world (or region) rule list; the tile keeps the ones it holds (see RoadGraphTile).
Result<RoadGraphTile> select_road_graph_tile(const RoadGraph& graph, const TileKey& key,
    std::span<const TurnRestriction> restrictions = {});
// Stretch context (street fidelity P1 slice 1). A tile graph stops at its halo,
// so a node of a complete way far outside the tile can miss its other incident
// ways. Every way incident to a node N touches the level-2 tile that contains
// N, hence lies in that tile's roads.graph. road_context_dependencies lists
// those tiles (sorted, unique, `exclude` omitted); merge_road_graphs unions
// complete graphs (equal ways and nodes are shared, conflicting ones are
// Malformed) into one canonical graph whose node degrees are world degrees.
// More than max_tiles distinct tiles is an explicit LimitExceeded.
Result<std::vector<InputTileRef>> road_context_dependencies(const RoadGraph& graph, geo::UtmZone zone,
    const TileKey* exclude = nullptr, std::size_t max_tiles = 512);
Result<RoadGraph> merge_road_graphs(std::span<const RoadGraph* const> graphs);
LayerDesc road_graph_desc();
class RoadGraphDeriver final : public IDeriver {
public:
    RoadGraphDeriver() : desc_(road_graph_desc()) {}
    const LayerDesc& output() const override { return desc_; }
    ContentHash config_hash() const override;
    std::vector<InputTileRef> inputs_for(const TileKey& target) const override;
    Result<DeriveResult> derive(const DeriveRequest& request) const override;
private:
    LayerDesc desc_;
};
} // namespace g2m
