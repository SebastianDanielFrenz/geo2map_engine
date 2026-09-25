// g2m/layer/layer_registry.h - the set of layers a server or client knows
// (PLAN.md 2.5). builtin() is a static registration of the built-in
// layers (G1: the three raw source layers); a server/client copies it and
// adds its own (vendor layers, further elevation sources).
//
// Thread safety: a const LayerRegistry may be read from any number of
// threads; add() needs exclusive access. builtin() is initialised once,
// thread-safely (function-local static).
#pragma once

#include "g2m/core/result.h"
#include "g2m/layer/layer_id.h"

#include <string_view>
#include <vector>

namespace g2m {

class LayerRegistry {
public:
    LayerRegistry() = default;

    // g2m.src.elev.dgm1, g2m.src.landcover, g2m.src.osm.
    static const LayerRegistry& builtin();

    // Fails on an invalid name, a duplicate name, an empty level mask, or
    // unsorted/duplicate inputs.
    Result<void> add(LayerDesc desc);

    const LayerDesc* find(std::string_view name) const;
    // Sorted by name (the iteration order is part of the output of e.g.
    // the release manifest, so it is fixed).
    const std::vector<LayerDesc>& all() const { return layers_; }

private:
    std::vector<LayerDesc> layers_; // sorted by id.name
};

} // namespace g2m
