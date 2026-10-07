// g2m/import/region_resolve.h - resolves a human-authored RegionSpec
// (g2m/layer/region.h) against one specific OSM PBF snapshot into a
// ResolvedRegion (G1c I2). See docs/formats/regions.md for the on-disk
// formats and the ring-assembly rules this builds on (g2m::geom, I1).
//
// An `osm_admin` part is resolved with three passes over the PBF (each
// restricted to the one entity type it needs, g2m::decode::osm::PbfEntities
// - cheap even against a whole-country extract): relations (find the target
// relation, its outer/inner member way ids), ways (those ids' node refs),
// nodes (those ids' positions). A `utm_bbox` part needs no PBF data at all.
#pragma once

#include "g2m/core/result.h"
#include "g2m/layer/region.h"

#include <cstdint>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

namespace g2m::import {

// Resolves every part of `spec` against the PBF at `pbf_path` (only read if
// `spec` has at least one osm_admin part) and computes the region's level-2
// tile footprint (RegionSpec::halo_m applied per part, see
// g2m::geom::tiles_near). Errors (Result): an osm_admin part's relation is
// not found in the PBF; a member way id is not found; a member way
// references a node id not found; the assembled ring geometry is invalid
// (crossing/overlap/unclosed - g2m::geom::assemble_rings); expect_name/
// expect_admin_level set and mismatched; the PBF file cannot be opened or
// is malformed; `spec.scheme` does not parse (defensive - cannot happen for
// a RegionSpec that came from parse_regions).
Result<ResolvedRegion> resolve_region(const RegionSpec& spec, const std::string& pbf_path);

// Several PBF extracts (e.g. Hessen + Regierungsbezirk Karlsruhe, which meet
// at the Hessen/Baden-Wuerttemberg border). Each osm_admin part is resolved
// against the FIRST file (in the given order) that holds its relation with
// every member way and node; a file that lacks the relation, or holds it
// only in part (an extract clips relations at its border), is skipped, and
// the error of the LAST file is reported if none resolves it. `pbf_name` is
// the file names joined by '+' and `pbf_sha256` is the SHA-256 of the files'
// own 32-byte digests concatenated in the given order (for exactly one file
// that is the file's own digest, so the one-file overload is unchanged).
Result<ResolvedRegion> resolve_region(const RegionSpec& spec, const std::vector<std::string>& pbf_paths);

// One administrative-boundary relation candidate (`find_admin` below).
struct AdminCandidate {
    std::int64_t relation_id = 0;
    std::string name;               // relation's "name" tag, "" if absent
    std::optional<int> admin_level; // relation's "admin_level" tag, parsed
};

// Single pass over RELATIONS ONLY: every relation tagged
// boundary=administrative, optionally filtered to a `name` tag containing
// `name_contains` (case-sensitive substring) and/or an exact
// `admin_level`. Sorted by relation_id. Throws DecodeError if the file
// cannot be opened or is malformed (same contract as
// g2m::decode::osm::read_pbf). Used by `g2m_tiler find-admin` to locate a
// regions.json osm_admin part's `relation` id by hand, and by
// tests/support/fixture_cut to pick a small fixture relation.
std::vector<AdminCandidate> find_admin(const std::string& pbf_path,
                                        std::optional<std::string_view> name_contains = std::nullopt,
                                        std::optional<int> admin_level = std::nullopt);

} // namespace g2m::import
