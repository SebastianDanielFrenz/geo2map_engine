// g2m/route/route_map.h - client map geometry for the minimap and the full
// map (G4 slice 5, docs/routing_design.md section 6).
//
// A MapTile is a client-side product of g2m.roads.graph tiles: no new server
// layer. One MapTile per tile key per LOD:
//   detail  level 2 (1 km)   every road class      1 m Douglas-Peucker
//   mid     level 4 (4 km)   tertiary and above     8 m
//   far     level 6 (16 km)  primary and above     40 m
// (MapConfig, default_map_config). A tile draws the edges OWNED by the
// level-2 roads.graph tiles inside it (RoadGraphTile::owned_edges: the tile
// containing the edge midpoint), so neighbouring map tiles never draw an
// edge twice and a tile of any LOD is assembled from level-2 tiles alone.
// Owned edges of one way that follow each other form one polyline, also
// across level-2 tile borders inside the map tile.
//
// Points are int32 decimetres relative to the tile's south-west corner.
// Every polyline keeps its (way id, segment range), so a route highlight can
// pick the same records; route_polyline gives the exact route line in mm.
// Deterministic: integer input, IEEE double arithmetic without libm, output
// sorted by (way id, first segment); the input tile order does not matter.
#pragma once

#include "g2m/core/tile_key.h"
#include "g2m/layer/road_graph_tile.h"
#include "g2m/route/route_search.h"

#include <cstdint>
#include <memory>
#include <span>
#include <string>
#include <vector>

namespace g2m::route {

enum class MapLod : std::uint8_t { Detail = 0, Mid = 1, Far = 2 };
inline constexpr std::size_t kMapLodCount = 3;
const char* to_string(MapLod lod);

// Every LOD parameter is caller data (owner rule: map detail is an in-game
// setting the game passes in and may change at runtime). See
// docs/routing_design.md "LOD settings contract".
struct MapLodConfig {
    int level = 2;                     // tile level of this LOD (>= 2: built from level-2 tiles)
    int min_rank = 0;                  // RoadStyle rank (track 0 .. motorway 9; links share their tier's)
    std::int64_t tolerance_mm = 1000;  // Douglas-Peucker tolerance at detail_scale 1
    double switch_distance_m = 2000;   // view radius up to which this LOD is used (select_map_lod)
};
struct MapConfig {
    MapLodConfig lods[kMapLodCount];
    // Aggressiveness: > 1 = more detail (tolerances divided by it, switch
    // distances multiplied by it), < 1 = coarser and cheaper. 1 = as given.
    double detail_scale = 1.0;
    std::size_t max_lines = 1u << 20;  // per tile; LimitExceeded beyond (not part of any key)
    std::size_t max_points = 1u << 24;
    RouteConfig directions = default_route_config(); // oneway rules (implied_oneway)
};
// detail L2/rank 0/1 m/2 km, mid L4/tertiary/8 m/12 km, far L6/primary/40 m
// (switch distance unbounded), scale 1; ranks looked up in
// RoadStyle::default_style().
const MapConfig& default_map_config();
// InvalidArgument unless: levels 2..TileKey::kMaxLevel and non-decreasing,
// min_rank 0..127, tolerances 0..1e9 mm, switch distances positive and
// non-decreasing (not NaN; +inf allowed), detail_scale finite in [0.05, 20].
Result<void> validate_map_config(const MapConfig& config);
// The effective parameters of one LOD (detail_scale applied: tolerance =
// round(tolerance_mm / scale), switch distance * scale).
MapLodConfig effective_lod(const MapConfig& config, MapLod lod);
// The LOD for a view whose visible radius is view_radius_m: the first whose
// effective switch distance is >= the radius, else the last.
MapLod select_map_lod(const MapConfig& config, double view_radius_m);
// Deterministic content key of one LOD's tiles: FNV-1a over a format tag,
// the LOD, its effective level, min_rank and tolerance and the oneway rules.
// Two configs with equal keys build identical tiles of that LOD; switch
// distances and limits are not part of it (they do not change a tile).
std::uint64_t map_lod_key(const MapConfig& config, MapLod lod);

struct MapPoint {
    std::int32_t x = 0, y = 0; // decimetres east/north of the tile's SW corner
    friend bool operator==(const MapPoint&, const MapPoint&) = default;
};

enum MapLineFlags : std::uint8_t {
    kMapOneway = 1u << 0,        // drivable in way order only
    kMapOnewayReverse = 1u << 1, // drivable against way order only
    kMapBridge = 1u << 2,
    kMapTunnel = 1u << 3,
    kMapLink = 1u << 4,          // highway=*_link
    kMapRoundabout = 1u << 5,
    kMapNoCars = 1u << 6,        // neither direction (oneway=reversible/alternating)
};
inline constexpr std::uint32_t kMapNoString = 0xffffffffu;

struct MapLine {
    std::int64_t way_id = 0;
    std::uint32_t seg_first = 0, seg_last = 0; // RoadGraphEdge::segment_index range (inclusive)
    std::uint32_t point_first = 0, point_count = 0; // into MapTile::points, way order, >= 2
    std::int8_t rank = 0;
    std::uint8_t flags = 0;
    std::uint32_t name = kMapNoString, ref = kMapNoString; // into MapTile::strings
    MapPoint anchor;                     // label anchor: midpoint of the longest piece
    std::int32_t anchor_dx = 0, anchor_dy = 0; // that piece's direction (dm, way order)
    friend bool operator==(const MapLine&, const MapLine&) = default;
};

struct MapTile {
    TileKey key;
    MapLod lod = MapLod::Detail;
    std::uint64_t params_key = 0;   // map_lod_key of the config it was built with
    std::vector<MapLine> lines;     // sorted by (way id, seg_first)
    std::vector<MapPoint> points;
    std::vector<std::string> strings; // ascending, unique
    std::size_t byte_size() const;    // memory held by the records, points and strings
    friend bool operator==(const MapTile&, const MapTile&) = default;
};

struct MapTileStats {
    std::size_t source_tiles = 0;   // level-2 tiles inside the key
    std::size_t owned_edges = 0;    // owned by those tiles
    std::size_t drawn_edges = 0;    // after the class filter
    std::size_t points_in = 0;      // polyline points before simplification
    std::size_t points_out = 0;
    std::size_t collapsed = 0;      // polylines shorter than one decimetre, dropped
};

// `tiles`: decoded level-2 roads.graph tiles (any order; tiles outside `key`
// are ignored, absent ones simply draw nothing). InvalidArgument when key's
// level is not the LOD's effective level, a tile is not level 2 or the
// config does not validate; Malformed when an
// owned edge's way or node is missing; LimitExceeded past the config limits.
Result<MapTile> build_map_tile(const TileKey& key, MapLod lod, std::span<const RoadGraphTile* const> tiles,
                               const MapConfig& config = default_map_config(), MapTileStats* stats = nullptr);

// Douglas-Peucker on a polyline (mm): keeps the first and last point and
// every point needed so that no dropped point lies farther than
// tolerance_mm from the kept polyline. Deterministic (the first farthest
// point wins a tie).
std::vector<geom::PointMm> simplify_polyline(std::span<const geom::PointMm> points, std::int64_t tolerance_mm);

// The driven line of a route in mm: the partial first and last arcs cut at
// the route's offsets (the same positions as RoutingGraph::point_at), every
// shape point in between, consecutive duplicates dropped.
std::vector<geom::PointMm> route_polyline(const RoutingGraph& graph, const Route& route);

// A map tile cache that follows the game's settings: set_config() drops only
// the tiles of LODs whose map_lod_key changed (a mid tolerance change keeps
// every detail and far tile). Not thread-safe: one per owner thread, or
// guard it. Deterministic: get() builds exactly what build_map_tile builds.
class MapTileCache {
public:
    explicit MapTileCache(MapConfig config = default_map_config());
    // InvalidArgument for an invalid config (the old one stays). Returns the
    // number of cached tiles dropped.
    Result<std::size_t> set_config(const MapConfig& config);
    const MapConfig& config() const { return config_; }
    // The cached tile, or builds it from `tiles` (the level-2 tiles inside
    // `key`) and caches it. The pointer stays valid until the entry is dropped.
    Result<const MapTile*> get(const TileKey& key, MapLod lod, std::span<const RoadGraphTile* const> tiles);
    const MapTile* find(const TileKey& key, MapLod lod) const;
    std::size_t size() const;
    void clear();
    std::size_t builds() const { return builds_; }

private:
    MapConfig config_;
    std::uint64_t keys_[kMapLodCount]{};
    std::vector<std::pair<TileKey, std::unique_ptr<MapTile>>> tiles_[kMapLodCount]; // sorted by key
    std::size_t builds_ = 0;
};

} // namespace g2m::route
