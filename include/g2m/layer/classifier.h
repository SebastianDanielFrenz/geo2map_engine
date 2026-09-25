// g2m/layer/classifier.h - IClassifier: source facts -> class ids (PLAN.md
// 2.5). Declarations only in G1; the built-in default classifier arrives
// with g2m.terrain.class (G2+).
//
// The numeric values of LandClass and SurfaceKind are part of the
// g2m.terrain.class encoding: never renumber, only append.
#pragma once

#include "g2m/layer/tile_container.h"

#include <cstdint>
#include <span>
#include <string_view>
#include <utility>

namespace g2m {

enum class LandClass : std::uint8_t {
    Unknown = 0,
    PavedRoad = 1,
    UnpavedRoad = 2,
    Rail = 3,
    Water = 4,
    Building = 5,
    TreeCover = 6,
    Shrub = 7,
    Grassland = 8,
    Cropland = 9,
    Bare = 10,
    BuiltUp = 11,
    Wetland = 12,
    SnowIce = 13,
};

enum class SurfaceKind : std::uint8_t {
    Unknown = 0,
    Asphalt = 1,
    Concrete = 2,
    PavingStones = 3,
    Sett = 4,
    Cobblestone = 5,
    Gravel = 6,
    Compacted = 7,
    Dirt = 8,
    Grass = 9,
    Sand = 10,
    Water = 11,
};

// What a classifier sees for one feature or cell. Any field may be empty.
struct ClassifierFacts {
    std::span<const std::pair<std::string_view, std::string_view>> osm_tags; // key, value
    std::string_view landcover_scheme;  // e.g. "esa_worldcover_v200"
    std::uint8_t landcover_class = 0;   // in that scheme
};

// Thread safety: const methods may be called concurrently.
class IClassifier {
public:
    virtual ~IClassifier() = default;

    // Stable name, e.g. "g2m.default" or "rg.surfaces".
    virtual std::string_view name() const = 0;
    // SHA-256 of the classification tables; part of the release id.
    virtual ContentHash table_hash() const = 0;

    virtual LandClass land_class(const ClassifierFacts& facts) const = 0;
    virtual SurfaceKind surface(const ClassifierFacts& facts) const = 0;
};

} // namespace g2m
