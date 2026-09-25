// g2m/layer/release.h - release ids and release manifests (PLAN.md 3.5;
// docs/formats/release_manifest.md).
//
// A release id (rid) is the SHA-256 of a canonical serialisation of:
//  - the source snapshot index: every raw source tile's (layer, key, body hash);
//  - every layer's version and deriver config hash;
//  - the classifier tables (name, table hash);
//  - the world seed.
// compute_release_id() sorts every list first, so the rid does not depend
// on the order the caller collected them in. The integrated server computes
// a local rid the same way from whatever was imported.
#pragma once

#include "g2m/core/result.h"
#include "g2m/core/tile_key.h"
#include "g2m/layer/layer_id.h"
#include "g2m/layer/tile_container.h"

#include <cstdint>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

namespace g2m {

struct SourceTileRef {
    std::string layer;
    std::uint64_t key_packed = 0; // TileKey::packed()
    ContentHash body_hash{};
    friend bool operator==(const SourceTileRef&, const SourceTileRef&) = default;
    friend auto operator<=>(const SourceTileRef&, const SourceTileRef&) = default;
};

struct LayerVersionRef {
    std::string layer;
    std::uint32_t version = 0;
    ContentHash config_hash{}; // all zero for layers without configuration (raw layers)
    friend bool operator==(const LayerVersionRef&, const LayerVersionRef&) = default;
    friend auto operator<=>(const LayerVersionRef&, const LayerVersionRef&) = default;
};

struct ClassifierRef {
    std::string name;
    ContentHash table_hash{};
    friend bool operator==(const ClassifierRef&, const ClassifierRef&) = default;
    friend auto operator<=>(const ClassifierRef&, const ClassifierRef&) = default;
};

struct ReleaseInputs {
    std::vector<SourceTileRef> sources;
    std::vector<LayerVersionRef> layers;
    std::vector<ClassifierRef> classifiers;
    std::uint64_t world_seed = 0;
};

// Fails on duplicates: two sources with the same (layer, key), two layer
// entries or two classifiers with the same name.
Result<ReleaseId> compute_release_id(const ReleaseInputs& inputs);

// "3f5a..." (64 lower-case hex digits) and back.
std::string to_hex(const ContentHash& h);
std::optional<ContentHash> content_hash_from_hex(std::string_view hex);

// ---- manifest ----

struct CoverageRun {
    std::int32_t y = 0;   // level-2 (1 km) tile row
    std::int32_t x0 = 0;  // first column, inclusive
    std::int32_t x1 = 0;  // last column, inclusive
    friend bool operator==(const CoverageRun&, const CoverageRun&) = default;
    friend auto operator<=>(const CoverageRun&, const CoverageRun&) = default;
};

struct ZoneCoverage {
    geo::UtmZone zone;
    std::vector<CoverageRun> runs; // sorted by (y, x0), non-overlapping, non-adjacent
    friend bool operator==(const ZoneCoverage&, const ZoneCoverage&) = default;
};

// Where a layer has data: a set of level-2 (1024 m) tiles per UTM zone, or
// everywhere.
struct Coverage {
    bool global = false;
    std::vector<ZoneCoverage> zones; // sorted by zone

    // Builds run-length coverage from level-2 tile keys (any order,
    // duplicates allowed).
    static Coverage from_level2_tiles(std::vector<TileKey> keys);
    // True if any level-2 tile under/over `key` is covered.
    bool covers(const TileKey& key) const;
    friend bool operator==(const Coverage&, const Coverage&) = default;
};

enum class Availability : std::uint8_t {
    Prebaked = 0,    // every covered tile exists
    OnDemand = 1,    // derived when first requested (may answer 503 + Retry-After)
    Unavailable = 2, // listed for completeness, not served
};

struct ReleaseSource {
    std::string id;            // e.g. "dgm1_nrw"
    std::string snapshot_date; // ISO 8601 date
    std::string licence;       // e.g. "dl-de/zero-2-0"
    std::string attribution;
    bool share_alike = false;
    friend bool operator==(const ReleaseSource&, const ReleaseSource&) = default;
};

struct ReleaseLayer {
    LayerDesc desc;
    ContentHash config_hash{};
    Availability availability = Availability::Prebaked;
    Coverage coverage;
    friend bool operator==(const ReleaseLayer&, const ReleaseLayer&) = default;
};

struct ReleaseManifest {
    ReleaseId rid{};
    std::uint8_t container_major = kContainerMajor;
    std::uint64_t world_seed = 0;
    std::vector<ReleaseSource> sources;   // sorted by id
    std::vector<ReleaseLayer> layers;     // sorted by desc.id.name
    std::vector<ClassifierRef> classifiers; // sorted by name

    const ReleaseLayer* find_layer(std::string_view name) const;
    friend bool operator==(const ReleaseManifest&, const ReleaseManifest&) = default;
};

// Canonical JSON: keys sorted, no whitespace, hashes as lower-case hex.
// Identical bytes for identical manifests on every platform.
std::string to_json(const ReleaseManifest& m);
// Strict parse: unknown keys are ignored (forward compatibility), missing
// or mistyped required keys fail.
Result<ReleaseManifest> release_manifest_from_json(std::string_view json);

} // namespace g2m
