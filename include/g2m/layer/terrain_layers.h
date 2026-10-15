// g2m/layer/terrain_layers.h - the terrain layer descriptors (PLAN.md G2.1/
// G2.2): g2m.elev.base (native DGM1 resolution, level 0 only) and
// g2m.terrain.height (the height pyramid, L0-L6 - L0 a byte-identical
// passthrough of g2m.elev.base, L1-L6 the mean-of-4 downsample pyramid).
// g2m.terrain.class (the surface-classification pyramid) is declared here
// too; its deriver is TerrainClassDeriver (terrain_class.h, P2 slice 10).
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
// byte-identical to g2m.elev.base; levels 1-6 are the half-even mean-of-4
// downsample of their own 4 same-layer children (TerrainHeightDeriver,
// derive_terrain.h; height_tile.h's downsample_2x2() - PLAN.md G2.2).
LayerDesc terrain_height_desc();
// g2m.terrain.class: surface classification (g2m.landclass.v1 scheme),
// levels 0-6, node-registered 257 x 257 (encoding TerrainClass,
// docs/formats/terrain_class.md); deriver in terrain_class.h.
LayerDesc terrain_class_desc();

} // namespace g2m
