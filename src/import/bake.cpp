// src/import/bake.cpp - see g2m/import/bake.h.

#include "g2m/import/bake.h"

#include "g2m/import/ordered_parallel.h"
#include "g2m/layer/layer_id.h"
#include "g2m/layer/release.h"
#include "g2m/server/builtin_release.h"
#include "g2m/server/server.h"
#include "g2m/server/tile_store.h"
#include "g2m/server/upstream.h"

#include <algorithm>
#include <optional>
#include <set>
#include <utility>

namespace g2m::import {
namespace {

// The level-`level` ancestor of `target` (its own key if already at or
// above `level`) - same rule as derive_terrain.cpp's file-local
// `ancestor_at` (duplicated, not shared: that one is private to
// g2m_layers, and bake.cpp lives in g2m_import).
TileKey ancestor_at(TileKey cur, int level) {
    while (cur.level < level) cur = cur.parent();
    return cur;
}

// Every level-2 (1024 m) tile a Coverage lists, in no particular order
// (release.h's own ZoneCoverage/CoverageRun fields are public exactly so
// callers like this one can walk them directly).
Result<std::vector<TileKey>> level2_tiles_of(const Coverage& cov) {
    if (cov.global) {
        return make_error(ErrorCode::InvalidArgument, "bake: cannot enumerate a global coverage");
    }
    std::vector<TileKey> out;
    for (const ZoneCoverage& zc : cov.zones) {
        for (const CoverageRun& run : zc.runs) {
            for (std::int32_t x = run.x0; x <= run.x1; ++x) {
                TileKey k;
                k.zone = zc.zone;
                k.level = kRawTileLevel;
                k.x = x;
                k.y = run.y;
                out.push_back(k);
            }
        }
    }
    return out;
}

// Every level-`level` tile touched by `l2_tiles` (a layer's covered
// level-2 set): for level < kRawTileLevel, every descendant; for
// level == kRawTileLevel, the tiles themselves; for level > kRawTileLevel,
// the distinct ancestors - matching Coverage::covers()'s own "any
// descendant/ancestor L2 tile is covered" rule (release.h), so this is
// exactly the set Server::tile() would answer Ok for at `level`.
std::vector<TileKey> targets_at_level(const std::vector<TileKey>& l2_tiles, int level) {
    if (level == kRawTileLevel) return l2_tiles;
    if (level > kRawTileLevel) {
        std::set<TileKey> ancestors;
        for (const TileKey& t2 : l2_tiles) ancestors.insert(ancestor_at(t2, level));
        return std::vector<TileKey>(ancestors.begin(), ancestors.end());
    }
    // level < kRawTileLevel: expand each L2 tile down (kRawTileLevel - level)
    // steps of 4 children each. Disjoint L2 tiles have disjoint descendants,
    // so no dedup is needed.
    const int steps = kRawTileLevel - level;
    std::vector<TileKey> cur = l2_tiles;
    for (int s = 0; s < steps; ++s) {
        std::vector<TileKey> next;
        next.reserve(cur.size() * 4);
        for (const TileKey& k : cur) {
            next.push_back(k.child(0, 0));
            next.push_back(k.child(1, 0));
            next.push_back(k.child(0, 1));
            next.push_back(k.child(1, 1));
        }
        cur = std::move(next);
    }
    return cur;
}

} // namespace

Result<BakeReport> bake(TileStore& sources, TileStore& derived, const std::string& scope, const BakeOptions& options) {
    if (options.min_level < 0 || options.max_level > TileKey::kMaxLevel || options.min_level > options.max_level) {
        return make_error(ErrorCode::InvalidArgument, "bake: invalid [min_level, max_level] range");
    }
    if (options.layers.empty()) {
        return make_error(ErrorCode::InvalidArgument, "bake: options.layers is empty");
    }

    const bool include_geometry = std::find(options.layers.begin(), options.layers.end(), std::string(kRoadGeomLayer)) != options.layers.end();
    const bool include_roads = include_geometry || std::find(options.layers.begin(), options.layers.end(), std::string(kRoadGraphLayer)) != options.layers.end();
    BuiltinTerrainDerivers derivers;
    LocalReleaseParams params = builtin_local_release_params(scope, derivers, /*sources=*/{}, include_roads, include_geometry);
    Result<ReleaseManifest> manifest_r = make_local_release(sources, params);
    if (!manifest_r) return manifest_r.error();
    const ReleaseManifest manifest = std::move(manifest_r).value();
    const std::string rid_hex = to_hex(manifest.rid);

    LocalSourceUpstream local_upstream(sources);
    ServerConfig config;
    config.server_id = "g2m_tiler.bake";
    config.offline = true;
    config.derived_store = &derived;
    config.derivers = derivers.ordered(include_roads, include_geometry);
    config.upstreams = {&local_upstream};
    Server server(config);
    Result<void> added = server.add_release(manifest, scope);
    if (!added) return added.error();

    BakeReport report;
    report.release_id_hex = rid_hex;
    for (const std::string& layer : options.layers) {
        const ReleaseLayer* rl = manifest.find_layer(layer);
        if (!rl) {
            return make_error(ErrorCode::InvalidArgument, "bake: unknown layer '" + layer + "'");
        }
        Result<std::vector<TileKey>> l2_tiles = level2_tiles_of(rl->coverage);
        if (!l2_tiles) return l2_tiles.error();

        BakeLayerReport layer_report;
        layer_report.layer = layer;
        for (int level = options.min_level; level <= options.max_level; ++level) {
            if (!has_level(rl->desc.levels, level)) continue;
            std::vector<TileKey> targets = targets_at_level(*l2_tiles, level);
            std::sort(targets.begin(), targets.end(), [](const TileKey& a, const TileKey& b) {
                const TileKey pa = ancestor_at(a, kRawTileLevel);
                const TileKey pb = ancestor_at(b, kRawTileLevel);
                if (pa != pb) return pa < pb;
                return a < b;
            });

            BakeLevelReport level_report;
            level_report.level = level;
            std::optional<Error> first_error;
            if (!targets.empty()) {
                ordered_for(
                    targets.size(), options.jobs,
                    [&](std::size_t i) -> TileResponse {
                        TileRequest req;
                        req.rid = manifest.rid;
                        req.layer = layer;
                        req.key = targets[i];
                        TileResponse resp = server.tile(req);
                        if (resp.meta.status == Status::Ok && options.pin) {
                            (void)derived.set_pinned(TileAddress{rid_hex, layer, targets[i]}, true);
                        }
                        return resp;
                    },
                    [&](std::size_t i, TileResponse&& resp) {
                        if (resp.meta.status != Status::Ok) {
                            if (!first_error) {
                                first_error = make_error(ErrorCode::Io, "bake: " + layer + " level " +
                                                                                  std::to_string(level) + " key " +
                                                                                  std::to_string(targets[i].packed()) +
                                                                                  ": " + resp.meta.message);
                            }
                            return;
                        }
                        ++level_report.tiles;
                        level_report.container_bytes += resp.container.size();
                    });
            }
            if (first_error) return *first_error;
            layer_report.levels.push_back(level_report);
            layer_report.tiles_total += level_report.tiles;
            layer_report.container_bytes_total += level_report.container_bytes;
        }
        report.layers.push_back(std::move(layer_report));
    }
    return report;
}

} // namespace g2m::import
