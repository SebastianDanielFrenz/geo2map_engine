// Railways R1/R2 (PLAN.md 6.6): the served layer g2m.rail.geom, level-2 tiles (1024 m), encoding 1 in that layer's
// own id space (docs/formats/rail_geom.md).
//
// A tile holds PIECES of OSM railway ways. A way is cut deterministically from its COMPLETE node list into pieces
// (RailRules::piece_max_length_m); the piece whose FIRST vertex lies in the tile (floor on integer millimetres, the
// roads.geom junction owner rule) belongs to the tile. So every piece lives in exactly one tile and its profile is a
// function of the 3 x 3 src.osm window around the tile and the DEM only. A piece extends at most one piece length
// past its tile; a consumer that draws pieces needs the tile ring that covers that.
//
// Every piece carries the track centreline (integer mm, UTM zone of the key) and a height profile in 1/256 m, plus
// class, condition (in service / under construction), structure flags (bridge, tunnel, embankment, cutting), layer,
// track count, gauge, service, usage and electrification. The height profile comes from the line itself, never from
// the bare-earth DEM under a span: a bridge or tunnel piece is linear between its chain's two ground anchors (the DEM
// at the ends of the connected bridge / tunnel ways), a ground piece is the smoothed, grade-limited DEM along it with
// both end values pinned to the raw DEM so neighbouring pieces meet exactly.
#pragma once
#include "g2m/core/geom/exact.h"
#include "g2m/layer/deriver.h"
#include "g2m/layer/height_tile.h"
#include "g2m/layer/osm_data.h"
#include "g2m/layer/rail_rules.h"
#include "g2m/layer/road_dem_sampler.h"
#include <cstdint>
#include <map>
#include <optional>
#include <span>
#include <string>
#include <string_view>
#include <vector>

namespace g2m {
inline constexpr std::string_view kRailGeomLayer = "g2m.rail.geom";
inline constexpr std::uint16_t kRailGeomEncodingVersion = 1;
inline constexpr std::size_t kRailGeomMaxPieces = 200000, kRailGeomMaxPoints = 4000000, kRailGeomMaxPiecePoints = 4096;
namespace rail_geom {
inline constexpr std::uint16_t kMeta = 1, kIds = 2, kAttrs = 3, kCounts = 4, kXy = 5, kZ = 6;
}
// Structure and state flags of a piece.
inline constexpr std::uint16_t kRailBridge = 1, kRailTunnel = 2, kRailEmbankment = 4, kRailCutting = 8, kRailDeclined = 16,
    kRailElectrified = 32, kRailFlagMask = 63;
enum class RailCondition : std::uint8_t { InService = 0, Construction = 1 };
enum class RailService : std::uint8_t { Main = 0, Siding = 1, Yard = 2, Spur = 3, Crossover = 4 };
enum class RailUsage : std::uint8_t { None = 0, Main = 1, Branch = 2, Industrial = 3, Military = 4, Tourism = 5, Test = 6 };
inline constexpr std::uint8_t kRailServiceMax = 4, kRailUsageMax = 6;
// One piece. xy and z have the same count (>= 2). z is 1/256 m in the layer's datum (the elev.base datum of the tile
// header); a Declined piece (no usable DEM under a needed sample) has all z = 0.
struct RailPiece {
    std::int64_t way_id = 0;
    std::uint32_t piece = 0;               // index within the way, from its first vertex
    std::uint8_t cls = 0;                  // RailClassRules::id
    RailCondition condition = RailCondition::InService;
    std::uint16_t flags = 0;
    std::int8_t layer = 0;                 // OSM layer, -5..5
    std::uint8_t tracks = 1;
    RailService service = RailService::Main;
    RailUsage usage = RailUsage::None;
    std::uint16_t gauge_mm = 0;
    std::vector<geom::PointMm> xy;
    std::vector<std::int32_t> z;
    friend bool operator==(const RailPiece&, const RailPiece&) = default;
};
struct RailGeomTile {
    TileKey key;
    std::vector<RailPiece> pieces;         // strictly ascending (way_id, piece), every first vertex inside `key`
    friend bool operator==(const RailGeomTile&, const RailGeomTile&) = default;
};
Result<void> validate_rail_geom_tile(const RailGeomTile& tile);
// The body of `tile`. The key is not stored (the tile header carries it); decode needs it for the owner check.
Result<TileBody> encode_rail_geom(const RailGeomTile& tile);
Result<RailGeomTile> decode_rail_geom(const TileBody& body, const TileKey& key);

// ---- building ----
// The attributes a way contributes to every one of its pieces.
struct RailAttrs {
    std::uint8_t cls = 0;
    RailCondition condition = RailCondition::InService;
    std::uint16_t flags = 0;               // kRailBridge / kRailTunnel / kRailEmbankment / kRailCutting / kRailElectrified
    std::int8_t layer = 0;
    std::uint8_t tracks = 1;
    RailService service = RailService::Main;
    RailUsage usage = RailUsage::None;
    std::uint16_t gauge_mm = 0;
    friend bool operator==(const RailAttrs&, const RailAttrs&) = default;
};
// The attributes of an OSM way, or nullopt when it is not a served railway (class not enabled, area, a construction
// way while construction is off, a class not in the rules). Tunnel wins over bridge when both are tagged.
std::optional<RailAttrs> resolve_rail_attrs(const osm::OsmData& data, const osm::Way& way, const RailRules& rules);

struct RailWay {
    std::int64_t id = 0;
    RailAttrs attrs;
    std::vector<std::int64_t> refs;        // consecutive duplicates removed
    std::vector<geom::PointMm> pts;        // same count as refs
    friend bool operator==(const RailWay&, const RailWay&) = default;
};
// The served railway ways of a set of src.osm tiles (complete ways; equal copies merge, conflicting ones are
// Malformed) plus the way index per node used by the chain walk.
struct RailWorld {
    std::map<std::int64_t, RailWay> ways;
    std::map<std::int64_t, std::vector<std::int64_t>> node_ways;   // node id -> way ids having it as ANY vertex, ascending
};
Result<RailWorld> collect_rail_world(std::span<const osm::OsmData* const> tiles, geo::UtmZone zone, const RailRules& rules);

// A piece cut from a way: vertices [first, last] of RailWay::pts, `arc_start_mm` = arc length of vertex `first` from the
// way's first vertex.
struct RailPiecePlan {
    const RailWay* way = nullptr;
    std::uint32_t index = 0;
    std::size_t first = 0, last = 0;
    double arc_start_mm = 0;
};
// All pieces of a way (not only the owned ones), in order. Pure function of the way and the rules.
std::vector<RailPiecePlan> plan_way_pieces(const RailWay& way, const RailRules& rules);
// The pieces whose first vertex lies in `key`, ascending (way, index).
std::vector<RailPiecePlan> plan_owned_pieces(const RailWorld& world, const TileKey& key, const RailRules& rules);

// The two ground anchors of a bridge / tunnel way: the node reached at each end by walking through connected ways of
// the same structure (a node where exactly one other served way touches, and it ends there with the same structure
// flag), up to the rules' budget. `ext_*_mm` is the length of the ways walked through; `chain_mm` the whole chain.
struct RailChain {
    std::int64_t node_a = 0, node_b = 0;
    geom::PointMm pos_a, pos_b;
    double ext_a_mm = 0, ext_b_mm = 0, chain_mm = 0;
};
RailChain rail_chain(const RailWorld& world, const RailWay& way, const RailRules& rules);

// The elev.base level-0 tiles needed to profile the pieces owned by `key`: the tiles within dependency_margin_m of
// every owned piece segment and of both anchors of every owned structure piece. Sorted, unique; more than
// max_tiles is LimitExceeded.
// `portal_anchor` (7a-1): also the tiles within the margin of the joining ground way's first 9 m at every chain end and at
// every ground piece end that meets a bridge or tunnel (what the portal approach reads).
Result<std::vector<InputTileRef>> rail_dem_dependencies(const RailWorld& world, const TileKey& key, const RailRules& rules,
    std::size_t max_tiles = 1024, bool portal_anchor = false);

// The ground profile (1/256 m) of resampled vertices at arc positions `s_m` (metres, ascending) from the raw DEM
// values: triangular smoothing of half-width smoothing_m (fading to the raw value over the same distance from both
// ends), then a forward and a backward grade limit pass; the first and the last value stay the raw DEM. Doubles with
// + - * / and sqrt only.
std::vector<std::int32_t> rail_ground_profile(std::span<const double> s_m, std::span<const std::int32_t> raw,
    std::int32_t smoothing_m, std::int32_t max_grade_permille);

// Builds the tile: the owned pieces with their profiles. A needed DEM sample that is missing declines that piece.
// `portal_anchor` (7a-1, true when served): a chain end and the ground piece ending at the same node take the approach level
// of the joining ground way instead of the DEM sample at the node (portal_approach.h); false = the previous behaviour.
Result<RailGeomTile> build_rail_geom_tile(const RailWorld& world, const TileKey& key, const RailRules& rules, const RoadDemSampler& dem,
    bool portal_anchor = false);

LayerDesc rail_geom_desc();
std::string rail_geom_config_text();

// g2m.rail.geom: first stage = the 3 x 3 g2m.src.osm tiles around the target, then the elev.base level-0 tiles under the
// owned pieces. Coverage follows g2m.src.osm. Header datum = the elev.base datum (None for a tile without pieces).
class RailGeomDeriver final : public IDeriver {
public:
    RailGeomDeriver() : desc_(rail_geom_desc()) {}
    const LayerDesc& output() const override { return desc_; }
    ContentHash config_hash() const override;
    std::span<const std::string> coverage_inputs() const override { return std::span<const std::string>(desc_.inputs).subspan(1, 1); }
    std::vector<InputTileRef> inputs_for(const TileKey& target) const override;
    Result<std::vector<InputTileRef>> additional_inputs_for(const DeriveRequest& request) const override;
    Result<DeriveResult> derive(const DeriveRequest& request) const override;
private:
    LayerDesc desc_;
};
} // namespace g2m
