// g2m/server/builtin_release.h - wires every built-in terrain deriver into
// a LocalReleaseParams (PLAN.md G2.1), shared by the tiler's local bake and
// the game's own in-process server: identical params (same source_scope,
// world_seed, sources) give an identical release id either way
// (compute_release_id, release.h).
#pragma once

#include "g2m/layer/derive_terrain.h"
#include "g2m/layer/road_graph_tile.h"
#include "g2m/layer/road_geom_tile.h"
#include "g2m/server/server.h"

#include <string>
#include <vector>

namespace g2m {

// Every built-in terrain deriver, in dependency order: make_local_release()
// (server.h) accumulates each deriver's output coverage from its inputs'
// already-computed level-2 coverage, in `ordered()`'s list order - elev
// must therefore run before height (g2m.terrain.height reads
// g2m.elev.base), and any future entry reading one of these two must be
// appended after it.
struct BuiltinTerrainDerivers {
    ElevBaseDeriver elev;
    TerrainHeightDeriver height;
    // PLAN.md G2.5: TerrainClassDeriver for g2m.terrain.class goes here,
    // appended last in ordered() (its input is g2m.src.landcover, a raw
    // layer, so ordering relative to elev/height does not matter, but a
    // fixed slot keeps this struct's layout stable across increments).

    RoadGraphDeriver roads;
    RoadGeomDeriver geometry;
    // Explicit opt-in preserves existing terrain-only release identities.
    std::vector<const IDeriver*> ordered(bool include_roads = false, bool include_geometry = false) const {
        std::vector<const IDeriver*> result{&elev, &height};
        if (include_roads || include_geometry) result.push_back(&roads);
        if (include_geometry) result.push_back(&geometry);
        return result;
    }
};

// A LocalReleaseParams wired to every built-in terrain deriver
// (`derivers.ordered()`) plus `sources` for attribution; `world_seed`,
// `classifiers` and `registry` are left at their defaults (0 / empty /
// LayerRegistry::builtin()) - set them on the returned struct before
// calling make_local_release() if the caller needs otherwise.
LocalReleaseParams builtin_local_release_params(std::string scope, const BuiltinTerrainDerivers& derivers,
                                                std::vector<ReleaseSource> sources, bool include_roads = false, bool include_geometry = false);

} // namespace g2m
