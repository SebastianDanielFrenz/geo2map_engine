#pragma once
#include "g2m/layer/road_graph_tile.h"
#include "g2m/layer/road_profile.h"
#include <optional>
namespace g2m {
inline constexpr std::string_view kRoadGeomLayer = "g2m.roads.geom";
inline constexpr std::uint16_t kRoadGeomEncoding = 5;
struct RoadGeomEntry {
    std::int64_t way_id = 0;
    RoadStretch stretch;
    std::optional<RoadProfile> profile;
    std::optional<Error> decline;
};
// Every graph stretch has exactly one accepted profile or explicit decline.
// Consumers must inspect declines; this is never an implicitly complete mesh.
struct RoadGeomTile { RoadGraphTile source; std::vector<RoadGeomEntry> entries; };
Result<RoadGeomTile> build_road_geom_tile(const RoadGraphTile& graph,
    const RoadDemSampler& dem, const RoadProfileLimits& limits = {});
Result<void> validate_road_geom_tile(const RoadGeomTile& tile);
Result<TileBody> encode_road_geom(const RoadGeomTile& tile);
Result<RoadGeomTile> decode_road_geom(const TileBody& body);
LayerDesc road_geom_desc();
class RoadGeomDeriver final : public IDeriver {
public:
    RoadGeomDeriver() : desc_(road_geom_desc()) {}
    const LayerDesc& output() const override { return desc_; }
    ContentHash config_hash() const override;
    std::span<const std::string> coverage_inputs() const override { return std::span<const std::string>(desc_.inputs).first(1); }
    std::vector<InputTileRef> inputs_for(const TileKey& target) const override;
    Result<std::vector<InputTileRef>> additional_inputs_for(const DeriveRequest& request) const override;
    Result<DeriveResult> derive(const DeriveRequest& request) const override;
private:
    LayerDesc desc_;
};
}
