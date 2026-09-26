// g2m/import/osm.h - PBF -> g2m.src.osm tile import (PLAN.md G1c I4).
//
// Two stages, kept separate because they have very different cost
// profiles on a real (country-scale) PBF:
//  1. extract_region_set() reads the PBF a handful of times (N1/W/R/N2,
//     see osm.cpp's own header comment) and materialises S_region: the
//     much smaller in-memory osm::OsmData subset a region actually needs.
//  2. import_osm() builds one OsmClipIndex (osm_clip.h) over S_region and
//     clips every one of the region's own level-2 tiles out of it via
//     ordered_for (jobs-independent output).
#pragma once

#include "g2m/core/result.h"
#include "g2m/import/sink.h"
#include "g2m/layer/osm_data.h"
#include "g2m/layer/region.h"

#include <cstddef>
#include <cstdint>
#include <string>
#include <string_view>

namespace g2m::import {

// Everything extract_region_set measured about one run. nodes_total/
// nodes_in/ways_in/ways_incomplete/relations_in are deterministic (pure
// functions of the PBF's own content and the region); the *_ms timings are
// not, but are still worth reporting (real-data run, PLAN.md's own "report
// per-pass times").
struct OsmExtractReport {
    std::uint64_t nodes_total = 0;      // every node id seen in the PBF (N1 pass)
    std::uint64_t nodes_in = 0;         // nodes whose position lies in some region tile's own dilated box
    std::uint64_t ways_in = 0;          // ways kept in S_region
    std::uint64_t ways_incomplete = 0;  // ways that touched the region but had an unresolvable node ref - dropped
    std::uint64_t relations_in = 0;     // relations kept in S_region
    std::uint64_t index_bytes = 0;      // rough N1 SoA footprint: nodes_total * 16

    double n1_ms = 0.0;
    double w_ms = 0.0;
    double r_ms = 0.0;
    double n2_ms = 0.0;
};

// Reads `pbf` (a .osm.pbf file) and extracts S_region: the subset of its
// nodes/ways/relations an OsmClipIndex (osm_clip.h) needs to reproduce,
// byte-for-byte, clip_osm's own cut of any of `region`'s own level-2 tiles
// at kDefaultOsmHaloM - i.e. clip_osm(S_region, k) == clip_osm(S_full, k)
// for every tile k in region.tiles (S_full = every node/way/relation the
// whole PBF has). `region.scheme` must parse (geo::parse_utm_scheme);
// `pbf`'s own node blocks must be strictly increasing by id (true of every
// real PBF writer, including this project's own g2m_fixture_cut) - a
// violation is ErrorCode::Malformed ("PBF not sorted"), everything else a
// genuine read failure is ErrorCode::Io. See osm.cpp for the N1/W/R/N2
// per-pass algorithm.
Result<osm::OsmData> extract_region_set(const std::string& pbf, const ResolvedRegion& region,
                                         OsmExtractReport* report = nullptr);

struct OsmImportOptions {
    // 0 = std::thread::hardware_concurrency() (at least 1) - ordered_for's
    // own `jobs` contract (ordered_parallel.h).
    unsigned jobs = 0;
};

struct OsmImportReport {
    std::uint64_t tiles_written = 0; // level-2 tiles written (non-empty clip)
    std::uint64_t tiles_empty = 0;   // clip produced no nodes/ways/relations at all - not written
    double wall_ms = 0.0;
};

// Imports every level-2 tile in `region.tiles` (region.h's tile_keys()) out
// of `region_set` (typically extract_region_set's own output, but any
// osm::OsmData satisfying OsmData::validate() works) into `sink` as layer
// g2m.src.osm (docs/formats/src_osm.md), datum VerticalDatum::None (OSM
// tiles carry no elevation datum). Builds one OsmClipIndex over
// `region_set` up front, then clips every tile via ordered_for - the SET of
// sink.put() calls (and their content) is identical for any `jobs` value.
// `scope` names the destination for the caller's own logging only (mirrors
// import_dem's own `scope` parameter, dem.h).
Result<OsmImportReport> import_osm(const ResolvedRegion& region, const osm::OsmData& region_set, ITileSink& sink,
                                    std::string_view scope, const OsmImportOptions& options = {});

} // namespace g2m::import
