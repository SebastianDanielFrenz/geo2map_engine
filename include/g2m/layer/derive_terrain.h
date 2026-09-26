// g2m/layer/derive_terrain.h - the built-in terrain derivers (PLAN.md G2.1):
// ElevBaseDeriver (g2m.elev.base, level 0 only) and TerrainHeightDeriver
// (g2m.terrain.height; level 0 only in this increment - PLAN.md G2.2 adds
// the mean-of-4 pyramid for levels 1-6).
//
// Both are pure/thread-safe per IDeriver's contract (deriver.h): derive()
// depends only on its inputs, config_hash() and the target key, and may be
// called concurrently for different targets from any thread.
#pragma once

#include "g2m/core/result.h"
#include "g2m/layer/deriver.h"
#include "g2m/layer/terrain_layers.h"

#include <memory>
#include <vector>

namespace g2m {

namespace layer_detail {
class DecodedElevMemo;
}

// g2m.elev.base: an unresampled 256 x 256 (kTerrainTileSamples^2) window of
// its ancestor g2m.src.elev.dgm1 level-2 (1024 m) tile - g2m.elev.base
// shares DGM1's own ~1 m Cell lattice exactly (lattice_offset(), raster.h),
// just re-tiled to g2m's level-0 size and row order (South-to-North vs.
// DGM1's North-to-South). A missing/out-of-coverage input yields an
// all-NoData tile, never an error.
class ElevBaseDeriver final : public IDeriver {
public:
    ElevBaseDeriver();
    ~ElevBaseDeriver() override;
    ElevBaseDeriver(const ElevBaseDeriver&) = delete;
    ElevBaseDeriver& operator=(const ElevBaseDeriver&) = delete;

    const LayerDesc& output() const override;
    ContentHash config_hash() const override;
    std::vector<InputTileRef> inputs_for(const TileKey& target) const override;
    Result<DeriveResult> derive(const DeriveRequest& request) const override;

private:
    LayerDesc desc_;
    std::unique_ptr<layer_detail::DecodedElevMemo> memo_;
};

// g2m.terrain.height: level 0 is byte-identical to its g2m.elev.base input
// (the store therefore dedups the two tiles' bodies). Levels 1-6 (PLAN.md
// G2.2's mean-of-4 downsample pyramid) are NOT implemented here:
// inputs_for()/derive() both treat any other level as absent/an error.
class TerrainHeightDeriver final : public IDeriver {
public:
    TerrainHeightDeriver();

    const LayerDesc& output() const override;
    ContentHash config_hash() const override;
    std::vector<InputTileRef> inputs_for(const TileKey& target) const override;
    Result<DeriveResult> derive(const DeriveRequest& request) const override;

private:
    LayerDesc desc_;
};

} // namespace g2m
