// src/server/builtin_release.cpp - see g2m/server/builtin_release.h.

#include "g2m/server/builtin_release.h"

#include <utility>

namespace g2m {

LocalReleaseParams builtin_local_release_params(std::string scope, const BuiltinTerrainDerivers& derivers,
                                                 std::vector<ReleaseSource> sources) {
    LocalReleaseParams params;
    params.source_scope = std::move(scope);
    params.derivers = derivers.ordered();
    params.sources = std::move(sources);
    return params;
}

} // namespace g2m
