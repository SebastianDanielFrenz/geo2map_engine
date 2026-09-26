// g2m/import/dem.h - DGM1 -> g2m.src.elev.dgm1 tile import (PLAN.md G1c I3).
//
// Assembles each of a resolved region's level-2 (1024 m) tiles from the up
// to four 1 km DGM1 squares that touch it (dgm1::Catalog), quantises to the
// project's own int32 height units, encodes as g2m.src.elev.dgm1 and writes
// through ITileSink. See dem.cpp's own header comment for the per-tile
// algorithm and the determinism argument (output is independent of `jobs`
// and `km_cache_tiles`, even though DemImportReport's own counters are not).
#pragma once

#include "g2m/core/result.h"
#include "g2m/decode/dgm1.h"
#include "g2m/import/sink.h"
#include "g2m/layer/region.h"
#include "g2m/layer/src_elev.h"

#include <cstddef>
#include <cstdint>
#include <string>
#include <string_view>
#include <vector>

namespace g2m::import {

struct DemImportOptions {
    // 0 = std::thread::hardware_concurrency() (at least 1) - see
    // ordered_parallel.h's own `jobs` contract.
    unsigned jobs = 0;
    // Bounded LRU of decoded 1 km DGM1 squares kept resident at once (each
    // is 1000*1000*4 bytes = ~4 MB raw plus overhead - 256 entries is
    // ~1 GB, PLAN.md's own RSS budget). Never 0 (treated as 1).
    std::size_t km_cache_tiles = 256;
    ElevPredictor predictor = ElevPredictor::Gradient;
};

// Everything import_dem measured/decided about one run. Deterministic in
// the counts a caller can reconstruct purely from the region + catalog
// (`tiles_written`/`tiles_partial`/`tiles_skipped_*`) - NOT deterministic
// across different `km_cache_tiles` values for the scheduling-dependent
// counters (`dup_*`, `km_decoded`, `km_cache_hits`, `differing`): a smaller
// cache can evict and later re-decode the same 1 km square more than once
// in a single run, re-running (and re-counting/re-reporting) its duplicate-
// copy comparison each time. The actual TILE OUTPUT (every sink.put() call)
// is unaffected either way - see dem.cpp's header comment.
struct DemImportReport {
    std::uint64_t tiles_written = 0;           // level-2 tiles written (full or partial)
    std::uint64_t tiles_partial = 0;           // subset of tiles_written with some NoData samples
    std::uint64_t tiles_skipped_nocov = 0;     // no DGM1 coverage touches this tile at all
    std::uint64_t tiles_skipped_allnodata = 0; // covered, but every sample decoded to NoData

    std::uint64_t dup_keys = 0;      // 1 km keys with more than one copy, decoded at least once this run
    std::uint64_t dup_copies = 0;    // total extra copies beyond copies()[0], summed over every decode
    std::uint64_t dup_identical = 0; // dup decodes whose every copy agreed sample-for-sample
    std::uint64_t dup_differing = 0; // dup decodes where at least one copy disagreed (copies[0] wins)
    std::vector<std::string> differing; // "e_km/n_km: <describe(copies[0])> != <describe(other)> ..."

    std::uint64_t km_decoded = 0;    // dgm1::Catalog::decode() calls actually made
    std::uint64_t km_cache_hits = 0; // km lookups served without a fresh decode (in cache or in flight)

    double wall_ms = 0.0;
};

// Imports every level-2 tile in `region.tiles` (region.h's tile_keys())
// from `catalog` into `sink` as layer g2m.src.elev.dgm1. `scope` names the
// destination for the caller's own logging only (the sink itself - e.g.
// StoreSink - already knows its destination; a future multi-scope importer
// may use it directly, so it stays part of the signature now). Returns an
// error only for a genuine decode/encode/sink failure - a region tile with
// no DGM1 coverage is not an error (see tiles_skipped_nocov).
Result<DemImportReport> import_dem(const ResolvedRegion& region, const g2m::decode::dgm1::Catalog& catalog,
                                    ITileSink& sink, std::string_view scope, const DemImportOptions& options = {});

} // namespace g2m::import
