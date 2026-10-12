// g2m/world/surface_id_table.h - full SurfaceKind / LandClass -> consumer
// surface id tables (terrain migration plan phase 1; vault G2M-020, request
// G-3). Header-only, engine-neutral.
//
// The consumer (physics_sim's SurfaceTable in the game) owns the NAMES and the
// ids; geo2map owns the table type and the rule that turns a classified cell
// into an id. Class mode (g2m.terrain.class) needs a full table, not the
// paved/unpaved/off_road choice the road-raster mode takes through
// RoadValueLut::by_land_class.
//
// Resolution rule (resolve):
//   - a road cell (LandClass PavedRoad or UnpavedRoad) with a known surface
//     (SurfaceKind other than Unknown) takes by_surface_kind[kind];
//   - every other cell, and a road cell whose surface is Unknown, takes
//     by_land_class[land_class]. For a road that is the paved/unpaved choice
//     (asphalt vs dirt), so an untagged road never falls onto the "Unknown"
//     surface entry.
// An out-of-range enum value reads as Unknown (index 0) in either table.
#pragma once

#include "g2m/layer/classifier.h"
#include "g2m/layer/osm_roads.h"

#include <array>
#include <cstddef>
#include <cstdint>

namespace g2m::classes {

inline constexpr std::size_t kSurfaceKindCount = 12; // SurfaceKind::Unknown .. Water
inline constexpr std::size_t kLandClassCount = 14;   // LandClass::Unknown .. SnowIce

static_assert(static_cast<std::size_t>(SurfaceKind::Water) + 1 == kSurfaceKindCount,
              "SurfaceKind grew: extend SurfaceIdTable (append only, classifier.h)");
static_assert(static_cast<std::size_t>(LandClass::SnowIce) + 1 == kLandClassCount,
              "LandClass grew: extend SurfaceIdTable (append only, classifier.h)");

struct SurfaceIdTable {
    std::array<std::uint16_t, kSurfaceKindCount> by_surface_kind{};
    std::array<std::uint16_t, kLandClassCount> by_land_class{};

    std::uint16_t for_surface_kind(SurfaceKind kind) const {
        const auto i = static_cast<std::size_t>(kind);
        return by_surface_kind[i < kSurfaceKindCount ? i : 0];
    }
    std::uint16_t for_land_class(LandClass land_class) const {
        const auto i = static_cast<std::size_t>(land_class);
        return by_land_class[i < kLandClassCount ? i : 0];
    }

    static constexpr bool is_road(LandClass land_class) {
        return land_class == LandClass::PavedRoad || land_class == LandClass::UnpavedRoad;
    }

    // The id for one classified cell (rule in the file comment).
    std::uint16_t resolve(LandClass land_class, SurfaceKind kind) const {
        if (is_road(land_class) && kind != SurfaceKind::Unknown) {
            return for_surface_kind(kind);
        }
        return for_land_class(land_class);
    }

    // The same table folded into the flat (land_class, surface) lookup that
    // paint_road_segments / fill_physics_surfaces read. Slots beyond the
    // defined enum values (up to 16 each) resolve like Unknown.
    RoadValueLut to_road_value_lut() const {
        RoadValueLut lut;
        for (std::size_t lc = 0; lc < 16; ++lc) {
            for (std::size_t sk = 0; sk < 16; ++sk) {
                const auto land_class = static_cast<LandClass>(lc < kLandClassCount ? lc : 0);
                const auto kind = static_cast<SurfaceKind>(sk < kSurfaceKindCount ? sk : 0);
                lut.v[lc * 16 + sk] = resolve(land_class, kind);
            }
        }
        return lut;
    }

    // Every entry `id`.
    static SurfaceIdTable uniform(std::uint16_t id) {
        SurfaceIdTable t;
        t.by_surface_kind.fill(id);
        t.by_land_class.fill(id);
        return t;
    }

    // Fills both tables from two callables: surface_id(SurfaceKind) and
    // land_id(LandClass), each returning an id convertible to uint16_t. The
    // caller maps names to ids (a game's SurfaceTable) inside them.
    template <class SurfaceFn, class LandFn>
    static SurfaceIdTable build(SurfaceFn&& surface_id, LandFn&& land_id) {
        SurfaceIdTable t;
        for (std::size_t i = 0; i < kSurfaceKindCount; ++i) {
            t.by_surface_kind[i] = static_cast<std::uint16_t>(surface_id(static_cast<SurfaceKind>(i)));
        }
        for (std::size_t i = 0; i < kLandClassCount; ++i) {
            t.by_land_class[i] = static_cast<std::uint16_t>(land_id(static_cast<LandClass>(i)));
        }
        return t;
    }

    friend bool operator==(const SurfaceIdTable&, const SurfaceIdTable&) = default;
};

} // namespace g2m::classes
