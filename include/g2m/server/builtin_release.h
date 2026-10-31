// g2m/server/builtin_release.h - wires every built-in terrain deriver into
// a LocalReleaseParams (PLAN.md G2.1), shared by the tiler's local bake and
// the game's own in-process server: identical params (same source_scope,
// world_seed, sources) give an identical release id either way
// (compute_release_id, release.h).
#pragma once

#include "g2m/layer/derive_terrain.h"
#include "g2m/layer/field_tile.h"
#include "g2m/layer/road_graph_tile.h"
#include "g2m/layer/soil_tile.h"
#include "g2m/layer/road_geom_tile.h"
#include "g2m/layer/terrain_carve.h"
#include "g2m/layer/terrain_class.h"
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
    RoadGraphDeriver roads;
    RoadGeomDeriver geometry;
    // Street fidelity P2 slice 9: g2m.terrain.carved (reads elev.base and roads.geom; coverage follows elev.base).
    TerrainCarvedDeriver carved;
    // Street fidelity P2 slice 10 (terrain migration phase 6, G-5): g2m.terrain.class (reads elev.base, roads.geom and
    // src.landcover; coverage follows elev.base).
    TerrainClassDeriver classes;
    // Farm field polygons (g2m.fields.geom): reads g2m.src.osm only; coverage follows it. Opt-in, independent of the road layers.
    FieldsGeomDeriver fields;
    // Soil facts per 32 m cell (g2m.soil.grid): reads g2m.src.soil (and g2m.src.landcover when present); coverage follows g2m.src.soil.
    // Opt-in, independent of every other derived layer.
    SoilGridDeriver soil;
    // Explicit opt-in preserves existing terrain-only release identities. The carved layer implies both road layers
    // (one release, one geometry: design 6.1); the class layer implies the carved layer too (its ownership is the
    // carve's), while a carved-only release keeps its slice-9 identity (D49).
    std::vector<const IDeriver*> ordered(bool include_roads = false, bool include_geometry = false, bool include_carved = false,
                                         bool include_classes = false, bool include_fields = false, bool include_soil = false) const {
        include_carved = include_carved || include_classes;
        std::vector<const IDeriver*> result{&elev, &height};
        if (include_roads || include_geometry || include_carved) result.push_back(&roads);
        if (include_geometry || include_carved) result.push_back(&geometry);
        if (include_carved) result.push_back(&carved);
        if (include_classes) result.push_back(&classes);
        if (include_fields) result.push_back(&fields);
        if (include_soil) result.push_back(&soil);
        return result;
    }
};

// A LocalReleaseParams wired to every built-in terrain deriver
// (`derivers.ordered()`) plus `sources` for attribution; `world_seed`,
// `classifiers` and `registry` are left at their defaults (0 / empty /
// LayerRegistry::builtin()) - set them on the returned struct before
// calling make_local_release() if the caller needs otherwise.
LocalReleaseParams builtin_local_release_params(std::string scope, const BuiltinTerrainDerivers& derivers,
                                                std::vector<ReleaseSource> sources, bool include_roads = false, bool include_geometry = false,
                                                bool include_carved = false, bool include_classes = false, bool include_fields = false,
                                                bool include_soil = false);

} // namespace g2m
