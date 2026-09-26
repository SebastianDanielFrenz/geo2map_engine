// g2m/import/bake.h - g2m_tiler's `bake` subcommand (PLAN.md G2.2): prebake
// every tile of one or more built-in terrain layers over a source store's
// snapshot, level-major, so a deployment can serve them Prebaked instead of
// OnDemand.
//
// PLAN.md decision 10 ("prebake and on-demand share the same resolver"):
// bake() is deliberately NOT a second implementation of the pyramid
// algorithm. It builds the exact same local release
// (BuiltinTerrainDerivers/builtin_local_release_params, server.h/
// builtin_release.h - the same wiring golden_g2.cpp and the game's own
// in-process server use) and then calls Server::tile() for every target
// tile, so the stored bytes are byte-for-byte what an on-demand resolve of
// that same tile would have produced - literally "the resolver called in a
// good order".
#pragma once

#include "g2m/core/result.h"

#include <cstdint>
#include <string>
#include <vector>

namespace g2m {
class TileStore;
}

namespace g2m::import {

struct BakeOptions {
    // Output (Baked) layer names to bake, e.g. {"g2m.terrain.height"}. Each
    // must be one of BuiltinTerrainDerivers::ordered()'s outputs; an unknown
    // name fails the whole call before anything is written.
    std::vector<std::string> layers;
    int min_level = 0;
    int max_level = 6;
    int jobs = 1;
    // Pin every tile bake() writes into `derived` (TileStore::set_pinned),
    // so a later size-capped eviction never drops it - mirrors
    // import::StoreSink's own `pin` parameter (sink.h).
    bool pin = false;
};

struct BakeLevelReport {
    int level = 0;
    std::uint64_t tiles = 0;
    // Sum of each tile's encoded container size (header + body, the exact
    // bytes Server::tile() returned and the derived store now holds) -
    // not the store's own deduplicated body_bytes stat (TileStore::stats()
    // reports that separately, after bake() returns, for a store-wide view).
    std::uint64_t container_bytes = 0;
};

struct BakeLayerReport {
    std::string layer;
    // In ascending level order, BakeOptions::min_level..max_level.
    std::vector<BakeLevelReport> levels;
    std::uint64_t tiles_total = 0;
    std::uint64_t container_bytes_total = 0;
};

struct BakeReport {
    // The local release's own id (ReleaseManifest::rid, hex-encoded) that
    // `derived`'s tiles were actually written under (Server's own
    // derived-store addressing, TileAddress{rid_hex, layer, key} - the same
    // string bake()'s --pin path already uses). Without this a caller has no
    // way to know what scope to pass to TileStore::index()/get() when
    // reading `derived` back later (e.g. to compare two bakes' index-hash,
    // or to open the baked store for serving) - `scope` (BakeOptions'
    // sibling parameter) names `sources`' release, not `derived`'s.
    std::string release_id_hex;
    // In BakeOptions::layers order.
    std::vector<BakeLayerReport> layers;
};

// Bakes every tile of `options.layers` at levels [min_level, max_level] of
// `scope`'s local release in `sources` into `derived`. Deterministic
// regardless of `options.jobs`: level-major (min_level first; PLAN.md G2.2
// - a level's own pyramid tiles depend only on the previous level's, never
// a later one, and baking L0 first warms the derived cache so every later
// level's recursive resolve hits Cache instead of re-deriving), and within
// a level sorted by (the target's level-2 ancestor, the target itself) -
// grouping same-parent tiles together for ElevBaseDeriver's 8-entry
// DecodedElevMemo's hit locality (decoded_elev_memo.h) even though only
// `--jobs`-many tiles are ever produced ahead of the next one consumed
// (import/ordered_parallel.h's own bounded-window contract), so the exact
// per-level task list - not just its final report - is independent of
// `jobs`.
//
// `sources` is read-only in intent: bake() never calls a mutating TileStore
// method on it (only get()/index(), both read-only, via make_local_release
// and Server::tile's own upstream reads). bake() itself takes an already-
// opened TileStore&, not a config, so it cannot enforce this at the type
// level - the caller decides how `sources` was opened. g2m_tiler's own
// `bake` subcommand opens it with TileStoreConfig::read_only set (PLAN.md
// G2.0's addition), which is only valid for a FINISHED, PINNED store
// (tile_store.h's own doc comment) - exactly the shape a real bake source
// (an already-imported, --pin'd release scope) has.
Result<BakeReport> bake(TileStore& sources, TileStore& derived, const std::string& scope, const BakeOptions& options);

} // namespace g2m::import
