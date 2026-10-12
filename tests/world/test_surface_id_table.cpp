// g2m::classes::SurfaceIdTable (include/g2m/world/surface_id_table.h; terrain
// migration plan phase 1, vault G2M-020, request G-3). New API: there is no
// game original to pin against (the game maps three names through its
// RoadSurfaceMap); the resolve rule is documented in the header and pinned here.
#include "g2m/world/surface_id_table.h"

#include "g2m/layer/classifier.h"

#include <catch2/catch_test_macros.hpp>

#include <cstdint>

using g2m::LandClass;
using g2m::SurfaceKind;
using g2m::classes::kLandClassCount;
using g2m::classes::kSurfaceKindCount;
using g2m::classes::SurfaceIdTable;

namespace {

// Distinct ids per entry: surface kinds 100 + i, land classes 200 + i.
SurfaceIdTable distinct_table() {
    return SurfaceIdTable::build([](SurfaceKind k) { return 100 + static_cast<int>(k); },
                                 [](LandClass c) { return 200 + static_cast<int>(c); });
}

} // namespace

TEST_CASE("SurfaceIdTable: a known surface on a road cell wins, everything else follows the land class",
          "[world][surface_id_table]") {
    const SurfaceIdTable t = distinct_table();
    // Road cells with a known kind: by surface kind.
    CHECK(t.resolve(LandClass::PavedRoad, SurfaceKind::Asphalt) == 100 + static_cast<int>(SurfaceKind::Asphalt));
    CHECK(t.resolve(LandClass::UnpavedRoad, SurfaceKind::Gravel) == 100 + static_cast<int>(SurfaceKind::Gravel));
    CHECK(t.resolve(LandClass::PavedRoad, SurfaceKind::Gravel) == 100 + static_cast<int>(SurfaceKind::Gravel));
    // Road cells with no known kind: the land class (the paved/unpaved choice).
    CHECK(t.resolve(LandClass::PavedRoad, SurfaceKind::Unknown) == 200 + static_cast<int>(LandClass::PavedRoad));
    CHECK(t.resolve(LandClass::UnpavedRoad, SurfaceKind::Unknown) ==
          200 + static_cast<int>(LandClass::UnpavedRoad));
    // Every other cell ignores the surface kind.
    for (std::size_t lc = 0; lc < kLandClassCount; ++lc) {
        const auto land = static_cast<LandClass>(lc);
        if (SurfaceIdTable::is_road(land)) continue;
        for (std::size_t sk = 0; sk < kSurfaceKindCount; ++sk) {
            CHECK(t.resolve(land, static_cast<SurfaceKind>(sk)) == 200 + lc);
        }
    }
}

TEST_CASE("SurfaceIdTable: accessors, bounds and equality", "[world][surface_id_table]") {
    const SurfaceIdTable t = distinct_table();
    for (std::size_t i = 0; i < kSurfaceKindCount; ++i) {
        CHECK(t.for_surface_kind(static_cast<SurfaceKind>(i)) == 100 + i);
    }
    for (std::size_t i = 0; i < kLandClassCount; ++i) {
        CHECK(t.for_land_class(static_cast<LandClass>(i)) == 200 + i);
    }
    // An out-of-range enum value reads as Unknown (index 0).
    CHECK(t.for_surface_kind(static_cast<SurfaceKind>(200)) == 100);
    CHECK(t.for_land_class(static_cast<LandClass>(200)) == 200);
    CHECK(t == distinct_table());
    CHECK_FALSE(t == SurfaceIdTable::uniform(7));
    const SurfaceIdTable u = SurfaceIdTable::uniform(7);
    CHECK(u.for_surface_kind(SurfaceKind::Water) == 7);
    CHECK(u.for_land_class(LandClass::SnowIce) == 7);
}

TEST_CASE("SurfaceIdTable::to_road_value_lut folds the same rule into the flat lookup", "[world][surface_id_table]") {
    const SurfaceIdTable t = distinct_table();
    const g2m::RoadValueLut lut = t.to_road_value_lut();
    for (std::size_t lc = 0; lc < 16; ++lc) {
        for (std::size_t sk = 0; sk < 16; ++sk) {
            const auto land = static_cast<LandClass>(lc < kLandClassCount ? lc : 0);
            const auto kind = static_cast<SurfaceKind>(sk < kSurfaceKindCount ? sk : 0);
            INFO("land_class " << lc << " surface " << sk);
            CHECK(lut.v[lc * 16 + sk] == t.resolve(land, kind));
        }
    }
    // Spot values.
    CHECK(lut.v[static_cast<std::size_t>(LandClass::PavedRoad) * 16 + static_cast<std::size_t>(SurfaceKind::Asphalt)] ==
          100 + static_cast<int>(SurfaceKind::Asphalt));
    CHECK(lut.v[static_cast<std::size_t>(LandClass::PavedRoad) * 16] ==
          200 + static_cast<int>(LandClass::PavedRoad));
    // Out-of-enum slots resolve like Unknown/Unknown.
    CHECK(lut.v[15 * 16 + 15] == 200);
}
