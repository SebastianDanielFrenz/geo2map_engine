// g2m/layer/deriver.h - IDeriver: a named, versioned layer as a pure
// function of its input tiles (PLAN.md 2.5, 6). Declarations only in G1;
// the first built-in derivers arrive in G2.
//
// Contract:
//  - derive() is a pure function of (its input tiles, the configuration
//    named by config_hash(), the world seed, the target key): same inputs ->
//    same body bytes on every supported compiler/OS (PLAN.md 2.4);
//  - inputs_for() names the first-stage input tiles, including the fixed
//    halo. additional_inputs_for() may discover further INPUT dependencies
//    from those resolved bytes (e.g. DEM over complete road geometry).
//    No stage reads neighbouring outputs or ambient stores;
//  - it may run on any thread, concurrently for different targets; it must
//    not keep mutable state between calls.
#pragma once

#include "g2m/core/result.h"
#include "g2m/core/tile_key.h"
#include "g2m/layer/layer_id.h"
#include "g2m/layer/tile_container.h"

#include <cstdint>
#include <optional>
#include <span>
#include <string>
#include <vector>

namespace g2m {

struct InputTileRef {
    std::string layer;
    TileKey key;
    friend bool operator==(const InputTileRef&, const InputTileRef&) = default;
    friend auto operator<=>(const InputTileRef& a, const InputTileRef& b) {
        if (auto c = a.layer <=> b.layer; c != 0) return c;
        return a.key <=> b.key;
    }
};

// One resolved input: its header and raw body bytes, or nothing if the
// input tile does not exist in the release (outside its coverage).
struct InputTile {
    InputTileRef ref;
    std::optional<TileHeader> header;
    std::vector<std::uint8_t> body;
};

struct DeriveRequest {
    TileKey target;
    std::uint64_t world_seed = 0;
    std::span<const InputTile> inputs; // in inputs_for() order
};

struct DeriveResult {
    std::vector<std::uint8_t> body;  // an encoded TileBody (tile_container.h)
    VerticalDatum datum = VerticalDatum::None;
    std::int32_t height_offset = 0;
};

class IDeriver {
public:
    virtual ~IDeriver() = default;

    // The output layer (kind Baked; inputs = the layer names it reads).
    virtual const LayerDesc& output() const = 0;
    // SHA-256 of the deriver's configuration (tables, parameters); part of
    // the release id.
    virtual ContentHash config_hash() const = 0;
    // Coverage-driving inputs may exclude supporting context layers. Default
    // preserves the historical union policy; every name must be declared.
    virtual std::span<const std::string> coverage_inputs() const { return output().inputs; }
    // Every input tile needed for `target`, sorted, unique.
    virtual std::vector<InputTileRef> inputs_for(const TileKey& target) const = 0;
    // Called once after first-stage resolution, before derive(). Returns
    // sorted unique additional refs whose layers appear in output().inputs.
    // Existing first-stage refs are deduplicated. Combined inputs are sorted,
    // unique and limited to 4096. Every present body hash enters the header.
    // Missing dependencies remain explicit absent InputTiles. Discovery errors
    // and unavailable dependencies prevent derivation and cache writes.
    virtual Result<std::vector<InputTileRef>> additional_inputs_for(const DeriveRequest&) const {
        return std::vector<InputTileRef>{};
    }
    virtual Result<DeriveResult> derive(const DeriveRequest& request) const = 0;
};

} // namespace g2m
