// tools/osm_tag_probe/scan.h - streams an OSM PBF once per entity type and feeds
// everything inside a resolved region into a tag_probe::Accumulator.
//
// Passes (each restricted to one entity type, the same convention as
// g2m::import::resolve_region / extract_region_set):
//   1. relations: remember multipolygon relations tagged building / forest
//      (their member way ids, to fetch the ways' node lists in pass 3);
//   2. nodes: a sorted 16 B/node location index (ids must be strictly
//      increasing, as in every real PBF) and the tagged nodes of interest;
//   3. ways: lengths, areas, node degrees, relation member node lists.
// Then the remembered relations are assembled into rings with
// g2m::geom::assemble_rings (the region resolver's own routine). That routine
// rejects any crossing or overlapping ring - hand-mapped forests with many
// rings often do - so a rejected relation falls back to a lenient role-based
// ring chaining for its area (checked equal to the exact areas on the ~4000
// relations where both work, except one, 0.1 percent off).
//
// Region membership is exact integer point-in-polygon (g2m::geom::contains)
// on the resolved region's rings, with the region's interior tiles as a fast
// accept. Everything is single-threaded and independent of hash/iteration
// order, so the result is a pure function of the PBF and the region.
#pragma once

#include "g2m/core/result.h"
#include "g2m/layer/region.h"
#include "tag_probe.h"

#include <string>

namespace g2m::tag_probe {

// `region` must carry its in-memory ring geometry (it does when it comes from
// g2m::import::resolve_region). Errors: unreadable/malformed PBF, node ids not
// sorted, unparsable region scheme.
Result<ScanStats> scan_pbf(const std::string& pbf_path, const ResolvedRegion& region, Accumulator& acc);

// Provenance for the reports.
ReportMeta make_meta(const ResolvedRegion& region, const ScanStats& stats);

} // namespace g2m::tag_probe
