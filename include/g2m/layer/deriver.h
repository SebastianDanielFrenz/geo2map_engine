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

#include <algorithm>
#include <cstdint>
#include <optional>
#include <span>
#include <string>
#include <utility>
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
// input tile does not exist in the release (outside its coverage). `rejected`
// marks an input whose own derivation failed deterministically and that the
// deriver accepts as absent (IDeriver::rejected_input_is_absent); header and
// body are then empty, as for an absent tile.
struct InputTile {
    InputTileRef ref;
    std::optional<TileHeader> header;
    std::vector<std::uint8_t> body;
    bool rejected = false;
};

// The value a server puts into a derived tile's header input list for a
// rejected input (terrain migration phase 6, G-1): SHA-256 of
// "g2m.rejected-input/1 <layer> <key>". It flags the tile as built without
// that input, and is never the body hash of a real tile.
inline ContentHash rejected_input_marker(const InputTileRef& ref) {
    const std::string text = "g2m.rejected-input/1 " + ref.layer + " " + to_string(ref.key);
    return content_hash(std::span<const std::uint8_t>(reinterpret_cast<const std::uint8_t*>(text.data()), text.size()));
}
inline bool header_has_rejected_input(const TileHeader& header, const InputTileRef& ref) {
    const ContentHash m = rejected_input_marker(ref);
    return std::find(header.inputs.begin(), header.inputs.end(), m) != header.inputs.end();
}

struct DeriveRequest {
    TileKey target;
    std::uint64_t world_seed = 0;
    std::span<const InputTile> inputs; // in inputs_for() order
};

struct DeriveResult {
    std::vector<std::uint8_t> body;  // an encoded TileBody (tile_container.h)
    VerticalDatum datum = VerticalDatum::None;
    std::int32_t height_offset = 0;
    // Named counts of source data the deriver left out of this tile (never part of the body, the
    // header or any hash). Empty when it left nothing out. Servers do not read it; a caller that
    // calls derive() directly (tools, tests) can report it. Names are stable, e.g.
    // "restrictions_dropped_over_via_ways".
    std::vector<std::pair<std::string, std::uint64_t>> diagnostics{};
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
    // Number of discovery stages (1..kMaxDiscoveryStages). With more than one,
    // additional_inputs_for is called again with every input resolved so far
    // (the first-stage refs plus every earlier stage's discoveries, sorted)
    // until a stage names no ref that is not already an input, or the count is
    // reached. Each call must be a pure function of the inputs it is given, so
    // the resolved set is too. The default (1) is the historical single call.
    // roads.geom uses up to 24 (decision D35: the far end of incident bridges is only
    // known once the stretch-context tiles are resolved).
    static constexpr std::size_t kMaxDiscoveryStages = 32;
    virtual std::size_t discovery_stages() const { return 1; }
    // Whether an input whose own derivation failed (a deterministic deriver
    // error, e.g. a roads.geom tile rejected by validation) is treated as
    // absent instead of failing this tile (terrain migration phase 6, G-1).
    // The server then passes it with `rejected` set and adds
    // rejected_input_marker(ref) to the output header's inputs. Unavailable
    // or degraded inputs (transient) still fail the tile. Default false.
    virtual bool rejected_input_is_absent(const InputTileRef&) const { return false; }
    virtual Result<DeriveResult> derive(const DeriveRequest& request) const = 0;
};

} // namespace g2m
