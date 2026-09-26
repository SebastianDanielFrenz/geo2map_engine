// src/layer/terrain_layers.cpp - see g2m/layer/terrain_layers.h.

#include "g2m/layer/terrain_layers.h"

namespace g2m {

LayerDesc elev_base_desc() {
    LayerDesc d;
    d.id = LayerId{std::string(kElevBaseLayer), 1};
    d.kind = LayerKind::Baked;
    d.encoding = static_cast<std::uint16_t>(LayerEncoding::SrcElev);
    d.encoding_version = 1;
    d.levels = level_bit(0);
    d.inputs = {std::string(kSrcElevDgm1Layer)};
    d.content = "terrain elevation at native DGM1 resolution (int32 1/256 m), level 0 only";
    return d;
}

LayerDesc terrain_height_desc() {
    LayerDesc d;
    d.id = LayerId{std::string(kTerrainHeightLayer), 1};
    d.kind = LayerKind::Baked;
    d.encoding = static_cast<std::uint16_t>(LayerEncoding::SrcElev);
    d.encoding_version = 1;
    LevelMask mask = 0;
    for (int l = 0; l <= kTerrainMaxLevel; ++l) mask |= level_bit(l);
    d.levels = mask;
    d.inputs = {std::string(kElevBaseLayer)};
    d.content = "terrain height pyramid, levels 0-6, mean-of-4 downsample (int32 1/256 m); "
                "level 0 only as of PLAN.md G2.1, levels 1-6 are G2.2";
    return d;
}

LayerDesc terrain_class_desc() {
    LayerDesc d;
    d.id = LayerId{std::string(kTerrainClassLayer), 1};
    d.kind = LayerKind::Baked;
    d.encoding = static_cast<std::uint16_t>(LayerEncoding::SrcLandcover);
    d.encoding_version = 1;
    LevelMask mask = 0;
    for (int l = 0; l <= kTerrainMaxLevel; ++l) mask |= level_bit(l);
    d.levels = mask;
    d.inputs = {std::string(kSrcLandcoverLayer)};
    d.content = "terrain surface classification (g2m.landclass.v1 scheme), levels 0-6; no deriver until PLAN.md G2.5";
    return d;
}

} // namespace g2m
