// g2m/layer/terrain_layers.h - the terrain L0 layer descriptors (PLAN.md
// G2.1): g2m.elev.base (native DGM1 resolution, level 0 only) and
// g2m.terrain.height (the height pyramid, L0-L6 - only L0 is derived in
// this increment, G2.2 adds the mean-of-4 downsample for L1-L6).
// g2m.terrain.class (the surface-classification pyramid) is declared here
// too, for the layer name/slot to exist, but has no deriver until G2.5.
//
// Both height layers reuse body encoding 1 (SrcElev, docs/formats/
// src_elev.md) at a 256 x 256 Cell-registered, SouthToNorth georef
// (height_tile.h) - no new codec, no new fuzz target.
#pragma once

#include "g2m/layer/layer_id.h"

#include <string_view>

namespace g2m {

inline constexpr std::string_view kElevBaseLayer = "g2m.elev.base";
inline constexpr std::string_view kTerrainHeightLayer = "g2m.terrain.height";
inline constexpr std::string_view kTerrainClassLayer = "g2m.terrain.class";

// Samples per side of a terrain tile at any level (Cell-registered: level's
// own footprint / kTerrainTileSamples cells, 1 m cells at level 0).
inline constexpr int kTerrainTileSamples = 256;
// Highest level the terrain height/class pyramids reach (PLAN.md G2.2/G2.5).
inline constexpr int kTerrainMaxLevel = 6;

// g2m.elev.base: a direct, unresampled 256 m window of g2m.src.elev.dgm1
// (native ~1 m DGM1 resolution), level 0 only.
LayerDesc elev_base_desc();
// g2m.terrain.height: the mean-of-4 height pyramid, levels 0-6. Level 0 is
// byte-identical to g2m.elev.base (TerrainHeightDeriver, derive_terrain.h);
// levels 1-6 are PLAN.md G2.2, not implemented by this increment's deriver.
LayerDesc terrain_height_desc();
// g2m.terrain.class: surface classification (g2m.landclass.v1 scheme),
// levels 0-6. No deriver until PLAN.md G2.5 - declared here so the layer
// name/slot exists.
LayerDesc terrain_class_desc();

} // namespace g2m
