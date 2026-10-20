// g2m/layer/layer_id.h - layer names, versions and descriptors (PLAN.md
// 2.5, 3.2).
//
// A layer name is namespaced: dot-separated components of [a-z0-9_], each
// starting with a letter, at least two components, at most 64 bytes.
// "g2m.*" is reserved for built-ins; anyone else uses their own vendor
// prefix ("rg.race_routes", "fs.field_parcels").
#pragma once

#include "g2m/core/tile_key.h"

#include <compare>
#include <cstdint>
#include <string>
#include <string_view>
#include <vector>

namespace g2m {

inline constexpr std::size_t kMaxLayerNameLength = 64;

bool is_valid_layer_name(std::string_view name);
// True for "g2m.<...>" (reserved for built-in layers).
bool is_builtin_layer_name(std::string_view name);

struct LayerId {
    std::string name;
    std::uint32_t version = 1;

    friend bool operator==(const LayerId&, const LayerId&) = default;
    friend auto operator<=>(const LayerId&, const LayerId&) = default;
};

enum class LayerKind : std::uint8_t {
    Raw = 0,    // source data (imported or fetched), never derived
    Baked = 1,  // produced by an IDeriver from input layers
};

// Body encodings (docs/formats/). The id is what a tile body's first u16
// says; each encoding has its own version. Ids are chosen per layer: the road
// layers changed their format more than once, so roads.graph 5 and roads.geom 5
// are different encodings of different layers (a decoder checks the layer first).
enum class LayerEncoding : std::uint16_t {
    SrcElev = 1,       // docs/formats/src_elev.md
    SrcLandcover = 2,  // docs/formats/src_landcover.md
    SrcOsm = 3,        // docs/formats/src_osm.md
    RoadGraph = 4,     // g2m.roads.graph, nodes without tags (read-only legacy); docs/formats/road_graph.md
    RoadGraphNodeTags = 5, // g2m.roads.graph, nodes carry whitelisted tags (written since street fidelity P1 slice 2)
    RoadGeom = 5,      // g2m.roads.geom, sections 1-2 only (read-only once encoding 6 lands); docs/formats/road_geom.md
    RoadGeomCrossSection = 6, // g2m.roads.geom with cross-section and feature sections (street fidelity P1 slice 5; still written when RoadGeomTile::encoding is 6)
    RoadGeomJunctions = 7, // g2m.roads.geom plus junction, furniture and provenance sections (street fidelity P2 slice 8; still written when RoadGeomTile::encoding is 7)
    RoadGeomTunnels = 8,   // g2m.roads.geom encoding 7 plus provenance 4, TunnelFloor (terrain migration phase 4, decision D47)
    RoadGeomUnderpasses = 9, // g2m.roads.geom encoding 8 plus section 8, underpass deck spans (decisions D53-D58)
    TerrainClass = 7,      // g2m.terrain.class (encoding 7 in that layer's id space, design 9.4); docs/formats/terrain_class.md
    RailGeom = 1,          // g2m.rail.geom (encoding 1 in that layer's id space, railways R1/R2); docs/formats/rail_geom.md
};

// Levels a layer has tiles at, as a bit mask over TileKey levels 0..20.
using LevelMask = std::uint32_t;
constexpr LevelMask level_bit(int level) { return LevelMask{1} << level; }
constexpr bool has_level(LevelMask mask, int level) {
    return level >= 0 && level <= TileKey::kMaxLevel && (mask & level_bit(level)) != 0;
}

struct LayerDesc {
    LayerId id;
    LayerKind kind = LayerKind::Raw;
    std::uint16_t encoding = 0;          // LayerEncoding value, or a vendor encoding id >= 0x8000
    std::uint16_t encoding_version = 1;
    LevelMask levels = 0;
    // Input layer names (baked layers only), sorted, unique.
    std::vector<std::string> inputs;
    // One line for humans (manifest, docs).
    std::string content;

    friend bool operator==(const LayerDesc&, const LayerDesc&) = default;
};

// Raw source layers (PLAN.md 3.2). Tiling: TileKey level 2 (1024 m) on the
// UTM grid for all three; see docs/formats/ for the per-encoding grids.
inline constexpr int kRawTileLevel = 2;
inline constexpr std::string_view kSrcOsmLayer = "g2m.src.osm";
inline constexpr std::string_view kSrcLandcoverLayer = "g2m.src.landcover";
inline constexpr std::string_view kSrcElevPrefix = "g2m.src.elev.";
inline constexpr std::string_view kSrcElevDgm1Layer = "g2m.src.elev.dgm1";

// "g2m.src.elev.<source>" for any valid source component ("dgm1",
// "glo30", ...). Returns a descriptor with an empty name if source is not
// a valid name component.
LayerDesc make_raw_elev_layer(std::string_view source);
LayerDesc make_raw_osm_layer();
LayerDesc make_raw_landcover_layer();

} // namespace g2m
