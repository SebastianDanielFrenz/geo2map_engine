#pragma once
#include "g2m/layer/deriver.h"
#include "g2m/layer/road_graph.h"

namespace g2m {
inline constexpr std::string_view kRoadGraphLayer = "g2m.roads.graph";
inline constexpr std::uint16_t kRoadGraphEncoding = 4;
inline constexpr std::uint32_t kRoadGraphHaloM = 128;
struct RoadGraphTile {
    TileKey key;
    std::uint32_t halo_m = kRoadGraphHaloM;
    RoadGraph graph;
    // Indices into graph.edges, strictly ascending. The containing tile of
    // each edge's floor-rounded integer-mm midpoint is its unique owner.
    std::vector<std::uint32_t> owned_edges;
    friend bool operator==(const RoadGraphTile&, const RoadGraphTile&) = default;
};
// Canonical, bounded encoding 4/version 1. Complete way geometry and tags
// remain available beyond the halo; ownership never changes edge identities.
Result<TileBody> encode_road_graph(const RoadGraphTile& tile);
Result<RoadGraphTile> decode_road_graph(const TileBody& body);
Result<void> validate_road_graph_tile(const RoadGraphTile& tile);
Result<void> validate_road_graph(const RoadGraph& graph);
Result<RoadGraphTile> select_road_graph_tile(const RoadGraph& graph, const TileKey& key);
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
