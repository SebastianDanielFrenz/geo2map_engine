// g2m/layer/region.h - user-authored region definitions (regions.json,
// format "g2m.regions/1") and the resolved-region artefact
// ("g2m.resolved_region/1") that OsmAdminPart resolution (I2,
// region_resolve.h) produces from them. PLAN G1c I1/I2;
// docs/formats/regions.md is the normative spec for both formats.
#pragma once

#include "g2m/core/geo/utm.h"
#include "g2m/core/geom/ring_set.h"
#include "g2m/core/result.h"
#include "g2m/core/tile_key.h"
#include "g2m/layer/release.h"        // CoverageRun
#include "g2m/layer/tile_container.h" // ContentHash

#include <cstdint>
#include <optional>
#include <string>
#include <string_view>
#include <utility>
#include <variant>
#include <vector>

namespace g2m {

// A region part sourced from an OSM administrative-boundary relation,
// resolved against a full PBF extract by I2's region_resolve.h
// (assemble_rings over the relation's "outer"/"inner" way members).
// `expect_name`/`expect_admin_level`, when set, are checked against the
// resolved relation's own tags and reported as a resolution error on
// mismatch, not silently ignored - a drift check for when the upstream OSM
// data changes under the region author (PLAN D6: identity is the relation
// id, not a name lookup, but the expected name/level are still worth
// pinning).
struct OsmAdminPart {
    std::int64_t relation = 0;
    std::optional<std::string> expect_name;
    std::optional<int> expect_admin_level;

    friend bool operator==(const OsmAdminPart&, const OsmAdminPart&) = default;
};

// A literal UTM bounding box, in whole metres, in the enclosing
// RegionSpec's own `scheme` zone. Closed box: valid iff west < east and
// south < north (checked by parse_regions, not here - this struct can be
// built directly by a test with any values).
struct UtmBboxPart {
    std::int64_t west = 0, south = 0, east = 0, north = 0;

    friend bool operator==(const UtmBboxPart&, const UtmBboxPart&) = default;
};

using RegionPart = std::variant<OsmAdminPart, UtmBboxPart>;

// One named region, as authored by a human (regions.json). `scheme`
// ("utm32n", ...) is the UTM zone every UtmBboxPart's metres - and every
// resolved tile key - are expressed in; see g2m/core/geo/utm.h's
// utm_scheme_name/parse_utm_scheme. `halo_m` widens every part's own tile
// footprint (g2m::geom::tiles_near's own halo_m argument, PLAN D4).
struct RegionSpec {
    std::string name;
    std::string scheme;
    std::uint32_t halo_m = 0;
    std::vector<RegionPart> parts;

    friend bool operator==(const RegionSpec&, const RegionSpec&) = default;
};

// Parses a regions.json document (format "g2m.regions/1", top-level
// {"format":"g2m.regions/1","regions":[...]}). Each region: `name` matches
// `[a-z0-9_]{1,64}` and is unique within the document; `scheme` must parse
// via geo::parse_utm_scheme; `halo_m` in [0, 20000]; `parts` has at least
// one entry, each an object {"kind":"osm_admin",...} or
// {"kind":"utm_bbox",...}; a utm_bbox part must have west < east and
// south < north. Every violation is ErrorCode::InvalidArgument except a
// structurally-broken/non-object document or an unparseable "format",
// which are Malformed/Unsupported (matching release_manifest_from_json's
// own convention). Unknown keys ANYWHERE in the document are REJECTED
// (the opposite of release_manifest_from_json's forward-compatible
// "ignore unknown keys": a regions.json is hand-authored, so a typo'd key
// should fail loudly instead of being silently dropped).
Result<std::vector<RegionSpec>> parse_regions(std::string_view json);

// Splits a CLI region selector "path#name" into {path, name}, at the
// FIRST '#' (a region name never contains one - see parse_regions' name
// pattern - so a path itself containing '#' still splits correctly).
// InvalidArgument if there is no '#', or either half would be empty.
Result<std::pair<std::string, std::string>> split_selector(std::string_view selector);

// Which kind of RegionPart a ResolvedPart was built from.
enum class ResolvedPartKind : std::uint8_t {
    OsmAdmin = 0,
    UtmBbox = 1,
};

// One resolved region part: the assembled ring geometry (`rings`,
// `vertices`) plus enough identity to audit it against its source.
// `rings` is IN-MEMORY ONLY - deliberately not part of the JSON
// representation (to_json/parse_resolved_region below): only
// `geometry_hash` (Fnv1a64 over the canonical ring bytes - each outer
// then each inner ring's point count and x,y's, in `rings`' own
// already-canonical order, see g2m::geom::assemble_rings) is persisted,
// keeping the resolved-region artefact small and auditable rather than a
// raw geometry dump (PLAN D4: "auditable, importers independent,
// golden-hashed").
struct ResolvedPart {
    ResolvedPartKind kind = ResolvedPartKind::UtmBbox;
    std::int64_t id = 0;            // relation id (OsmAdmin), else 0
    std::string name;               // resolved relation name (OsmAdmin), else ""
    std::optional<int> admin_level; // OsmAdmin only
    geom::RingSet rings;            // in-memory only, see above
    std::uint64_t vertices = 0;
    std::uint64_t geometry_hash = 0;

    friend bool operator==(const ResolvedPart&, const ResolvedPart&) = default;
};

// The "g2m.resolved_region/1" artefact: a region's geometry, already
// resolved against one specific PBF snapshot (`pbf_name`/`pbf_sha256`),
// plus its level-2 tile footprint. `tiles` is the union of every part's
// own tiles_near() result (Boundary or Interior - i.e. the whole
// footprint); `interior` is the subset that is Interior for at least one
// part (PLAN: "interior if any part says Interior") - useful for a
// consumer that wants to skip boundary-only handling for tiles fully
// inside the region. `level` is always 2 today (the tiling grid every
// source layer uses); carried explicitly, rather than assumed, so a
// resolution at a different level still round-trips through this same
// format.
struct ResolvedRegion {
    std::string name;
    std::string scheme;
    int level = 2;
    std::uint32_t halo_m = 0;
    std::vector<ResolvedPart> parts;
    std::string pbf_name;
    ContentHash pbf_sha256{};
    std::vector<CoverageRun> tiles;
    std::vector<CoverageRun> interior;
    std::uint64_t tile_count = 0;

    friend bool operator==(const ResolvedRegion&, const ResolvedRegion&) = default;
};

// Canonical JSON (sorted keys, no whitespace, hashes as lower-case hex -
// release_manifest's own rules, docs/formats/release_manifest.md).
// `rings` is not part of this representation (see ResolvedPart above).
std::string to_json(const ResolvedRegion& r);
// Strict parse: unknown keys are rejected (same rationale as
// parse_regions - a resolved-region file is a build artefact checked into
// a repo, e.g. racing_game's, so a corrupted or hand-edited one should
// fail loudly, not silently accept a typo).
Result<ResolvedRegion> parse_resolved_region(std::string_view json);

// Every TileKey in `r.tiles` (the region's full level-2 footprint, not
// just `r.interior`), reconstructed from the CoverageRun list via
// `r.scheme`/`r.level`. Empty if `r.scheme` does not parse (defensive -
// cannot happen for a ResolvedRegion that came from parse_resolved_region,
// which already validates `scheme`).
std::vector<TileKey> tile_keys(const ResolvedRegion& r);

} // namespace g2m
