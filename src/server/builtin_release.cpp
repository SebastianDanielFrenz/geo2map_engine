// src/server/builtin_release.cpp - see g2m/server/builtin_release.h.

#include "g2m/server/builtin_release.h"

#include <utility>

namespace g2m {

LocalReleaseParams builtin_local_release_params(std::string scope, const BuiltinTerrainDerivers& derivers,
                                                 std::vector<ReleaseSource> sources, bool include_roads, bool include_geometry,
                                                 bool include_carved, bool include_classes, bool include_fields,
                                                 bool include_soil) {
    LocalReleaseParams params;
    params.source_scope = std::move(scope);
    params.derivers = derivers.ordered(include_roads, include_geometry, include_carved, include_classes, include_fields, include_soil);
    params.sources = std::move(sources);
    return params;
}

} // namespace g2m
