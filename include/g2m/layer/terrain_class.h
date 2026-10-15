// Street fidelity P2 slice 10 (design sections 6.5 and 9.4; terrain migration phase 6): the surface classification
// layer g2m.terrain.class.
//
// A tile is node-registered: 257 x 257 samples at the integer-metre lattice of its level (spacing 2^L m), both tile
// edges included, so the level-0 samples are exactly the physics cell centres (D11). Four planes: land_class
// (LandClass), surface (SurfaceKind), role (TerrainRole) and rank (0 = no road, else the owning way's rank).
//
// Level 0 classifies every node under the SAME carve ownership as g2m.terrain.carved (CarveModel::evaluate +
// classify): junction cores, approach cores by their band, blends, terrain, bridge footprints. A node no road owns takes
// the base class: the g2m.src.landcover 4 m sample containing it (the source's floor edge rule) through the embedded
// ESA WorldCover classifier table. Levels 1-6 take, per node, the level L-1 sample of highest rank among the 3 x 3
// nodes around the coincident one (ties to the coincident centre, then the first in row-major order, south row first,
// west first), so roads stay visible at coarse levels.
//
// Every value is a function of the world position: neighbouring tiles agree on their shared edge nodes. Level 0 reads
// the same 3 x 3 elev.base window and roads.geom tiles as terrain.carved (a rejected roads.geom input counts as "no
// roads", G-1) plus the g2m.src.landcover level-2 tiles holding any of its nodes (the ancestor, and its east / north
// neighbours when the tile touches the ancestor's east or north edge: decision D49). Levels 1-6 read the 4 x 4
// same-layer tiles of level L-1 around the 4 children (the 3 x 3 window of an edge node reaches one node into the
// neighbouring children: D49).
#pragma once
#include "g2m/core/result.h"
#include "g2m/core/tile_key.h"
#include "g2m/layer/carve_params.h"
#include "g2m/layer/classifier.h"
#include "g2m/layer/deriver.h"
#include "g2m/layer/landcover_classifier.h"
#include "g2m/layer/src_landcover.h"
#include "g2m/layer/terrain_carve.h"
#include "g2m/layer/tile_container.h"
#include <cstddef>
#include <cstdint>
#include <span>
#include <string>
#include <string_view>
#include <vector>

namespace g2m {
inline constexpr int kTerrainClassSamples = 257;
inline constexpr std::size_t kTerrainClassNodes = static_cast<std::size_t>(kTerrainClassSamples) * kTerrainClassSamples;
inline constexpr std::string_view kTerrainClassScheme = "g2m.landclass.v1";
inline constexpr std::uint16_t kTerrainClassEncodingVersion = 1;
inline constexpr std::uint8_t kLandClassMax = 13;     // LandClass::SnowIce; append-only (classifier.h)
inline constexpr std::uint8_t kSurfaceKindMax = 11;   // SurfaceKind::Water
namespace terrain_class {
inline constexpr std::uint16_t kMeta = 1;
inline constexpr std::uint16_t kLandClass = 2;
inline constexpr std::uint16_t kSurface = 3;
inline constexpr std::uint16_t kRole = 4;
inline constexpr std::uint16_t kRank = 5;
} // namespace terrain_class

// One g2m.terrain.class tile. Node (i, j) lies at (min_easting + i * 2^L, min_northing + j * 2^L); planes are row-major
// with row 0 = south.
struct TerrainClassTile {
    TileKey key;
    std::vector<std::uint8_t> land_class, surface, role, rank;   // kTerrainClassNodes each
    static constexpr std::size_t index(int i, int j) noexcept {
        return static_cast<std::size_t>(j) * static_cast<std::size_t>(kTerrainClassSamples) + static_cast<std::size_t>(i);
    }
    void reset(const TileKey& k);   // every plane zero (Unknown, Unknown, Terrain, no road)
    friend bool operator==(const TerrainClassTile&, const TerrainClassTile&) = default;
};

// The georef of a class tile: node registration, 257 x 257, spacing 2^L m, SouthToNorth.
RasterGeoref terrain_class_georef(const TileKey& key);
Result<TileBody> encode_terrain_class(const TerrainClassTile& tile);
// Rejects a body without all five sections, unknown sections, a georef other than terrain_class_georef(expect), another
// scheme, wrong plane sizes, land_class > 13, surface > 11, role > 7.
Result<void> decode_terrain_class(const TileBody& body, const TileKey& expect, TerrainClassTile& out);

// The base class of a level-0 node from the decoded g2m.src.landcover level-2 tiles (absent = Unknown).
class LandcoverBase {
public:
    LandcoverBase(std::span<const LandcoverTile> tiles, const LandcoverClassifier& classifier) : tiles_(tiles), classifier_(classifier) {}
    // The node at zone metres (e, n): the 4 m (any spacing) cell containing it, floor on both axes.
    void at(std::int64_t e, std::int64_t n, LandClass& land_class, SurfaceKind& surface) const;
private:
    std::span<const LandcoverTile> tiles_;
    const LandcoverClassifier& classifier_;
};

// The level-0 class tile `key` (design 6.5): CarveTerrain + CarveModel as terrain.carved, then evaluate + classify at
// every node; base classes from `base`.
Result<void> classify_terrain_tile(TerrainClassTile& out, const TileKey& key, std::span<const HeightTile* const> dem,
    std::span<const CarveGeomInput> roads, const LandcoverBase& base, const CarveParams& params = builtin_carve_default(),
    const CarveOptions& options = {});

// The rank-max pyramid step (design 6.5): `window[(dy + 1) * 4 + (dx + 1)]`, dx, dy in -1..2, is the level L-1 tile at
// (2 x + dx, 2 y + dy) of `parent` (nullptr = absent: its nodes are not candidates).
void downsample_terrain_class(const TileKey& parent, std::span<const TerrainClassTile* const> window, TerrainClassTile& out);

// The g2m.src.landcover level-2 keys whose cells hold a node of the level-0 tile, ascending (1, 2 or 4).
std::vector<TileKey> terrain_class_landcover_keys(const TileKey& level0);

std::string terrain_class_config_text();

// g2m.terrain.class. L0: the inputs of terrain.carved L0 plus terrain_class_landcover_keys; L1-L6: the 4 x 4 level L-1
// tiles of the same layer around the children. Header datum None, offset 0. Coverage follows g2m.elev.base.
class TerrainClassDeriver final : public IDeriver {
public:
    TerrainClassDeriver();
    const LayerDesc& output() const override { return desc_; }
    ContentHash config_hash() const override;
    std::span<const std::string> coverage_inputs() const override { return std::span<const std::string>(desc_.inputs).first(1); }
    std::vector<InputTileRef> inputs_for(const TileKey& target) const override;
    bool rejected_input_is_absent(const InputTileRef& ref) const override;   // roads.geom, as terrain.carved (G-1)
    Result<DeriveResult> derive(const DeriveRequest& request) const override;
private:
    LayerDesc desc_;
};
} // namespace g2m
