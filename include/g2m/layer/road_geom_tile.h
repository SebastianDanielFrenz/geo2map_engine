#pragma once
#include "g2m/layer/road_graph_tile.h"
#include "g2m/layer/road_cross_section.h"
#include "g2m/layer/road_profile.h"
#include <optional>
namespace g2m {
inline constexpr std::string_view kRoadGeomLayer = "g2m.roads.geom";
inline constexpr std::uint16_t kRoadGeomEncoding = 6;
// The previous body format (profiles only, no cross-sections); still decoded, never written.
inline constexpr std::uint16_t kRoadGeomEncodingLegacy = 5;
struct RoadGeomEntry {
    std::int64_t way_id = 0;
    RoadStretch stretch;
    std::optional<RoadProfile> profile;
    std::optional<Error> decline;
    // Encoding 6. Exactly one of the two is set for an entry with a profile, neither for a declined
    // entry (no profile means no reference to place a cross-section on) and neither for an encoding-5 tile.
    std::optional<RoadCrossSection> cross_section;
    std::optional<CrossSectionDecline> cross_section_decline;
};
// Vertical traffic-calming and similar features at a way node (design 8.3, owner decision Q2: every feature that
// has a height is carved; lateral kinds render only). Order is the encoded kind byte.
enum class ProfileFeatureKind : std::uint8_t { Bump = 0, MiniBumps = 1, RumbleStrip = 2, Cushion = 3, Hump = 4,
    Table = 5, Chicane = 6, Choker = 7, Island = 8, Dip = 9 };
struct ProfileFeature {
    std::size_t entry_index = 0;     // into RoadGeomTile::entries; the entry has a cross-section
    std::int64_t node_id = 0;
    double station_m = 0;            // along the entry's reference line
    ProfileFeatureKind kind = ProfileFeatureKind::Bump;
    bool carve = false;              // true: part of the carved height; false: render only
    std::int32_t length_mm = 0, height_mm = 0, width_mm = 0; // width 0 = full carriageway
    Provenance provenance = Provenance::Inferred;
    friend bool operator==(const ProfileFeature&, const ProfileFeature&) = default;
};
// Every graph stretch has exactly one accepted profile or explicit decline.
// Consumers must inspect declines; this is never an implicitly complete mesh.
struct RoadGeomTile {
    RoadGraphTile source;
    std::vector<RoadGeomEntry> entries;
    std::vector<ProfileFeature> features; // sorted by (entry_index, station bits, node_id)
    // 6 for everything built or encoded; 5 for a decoded legacy body, which has no cross-sections or
    // features (consumers fall back to the symmetric RoadAttributes::width_mm path).
    std::uint16_t encoding = kRoadGeomEncoding;
};
// `context` (optional) is a canonical graph holding every way incident to the
// nodes of graph.graph's ways (merge_road_graphs over the tiles named by
// road_context_dependencies). Stretch extents, join headings and profiles then
// depend on the world, not on the tile. Without it the tile graph itself is the
// context, which is only exact for a graph that is complete at its own nodes.
Result<RoadGeomTile> build_road_geom_tile(const RoadGraphTile& graph,
    const RoadDemSampler& dem, const RoadProfileLimits& limits = {}, const RoadGraph* context = nullptr,
    const RoadRules& rules = builtin_road_rules_de());
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
