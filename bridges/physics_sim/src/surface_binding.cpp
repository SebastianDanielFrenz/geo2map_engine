#include "g2m/ps_bridge/surface_binding.h"

#include <cstdint>
#include <optional>

namespace g2m::ps_bridge {

namespace {

// physics.road_surfaces.{field} -> a SurfaceId valid as a grip value (the game's resolve_grip_surface, verbatim
// messages): present in the SurfaceTable AND < 255.
Result<ps::SurfaceId> resolve_grip_surface(const ps::io::SurfaceTable& table, const char* field, const std::string& name) {
    const ps::SurfaceId id = table.id_for(name);
    if (id == ps::kInvalidSurfaceId) {
        return make_error(ErrorCode::InvalidArgument,
                          std::string("Session: physics.road_surfaces.") + field + " '" + name + "' is not in the surface table");
    }
    if (id >= 255) {
        return make_error(ErrorCode::InvalidArgument, std::string("Session: physics.road_surfaces.") + field + " '" + name +
                                                          "' has surface id " + std::to_string(id) + " (must be < 255)");
    }
    return id;
}

struct GripIds {
    ps::SurfaceId paved = ps::kInvalidSurfaceId, unpaved = ps::kInvalidSurfaceId, off_road = ps::kInvalidSurfaceId;
};

// paved, unpaved, off_road in this order: the first failure is the one reported.
Result<GripIds> resolve_grip_ids(const ps::io::SurfaceTable& table, const SurfaceNames& names) {
    GripIds ids;
    auto paved = resolve_grip_surface(table, "paved", names.paved);
    if (!paved) return paved.error();
    ids.paved = *paved;
    auto unpaved = resolve_grip_surface(table, "unpaved", names.unpaved);
    if (!unpaved) return unpaved.error();
    ids.unpaved = *unpaved;
    auto off_road = resolve_grip_surface(table, "off_road", names.off_road);
    if (!off_road) return off_road.error();
    ids.off_road = *off_road;
    return ids;
}

} // namespace

Result<ClassSurfaceConfig> bind_class_surfaces(const ps::io::SurfaceTable& table, const SurfaceNames& names) {
    const auto ids = resolve_grip_ids(table, names);
    if (!ids) return ids.error();
    if (!names.kind_name) return make_error(ErrorCode::InvalidArgument, "bind_class_surfaces: SurfaceNames::kind_name is empty");
    const ps::SurfaceId paved = ids->paved, unpaved = ids->unpaved, off_road = ids->off_road;
    // SurfaceIdTable::build calls the kind lambda in SurfaceKind order; the first unresolvable name is the error (the
    // game threw from inside the lambda at that same point).
    std::optional<Error> failure;
    const auto classes = g2m::classes::SurfaceIdTable::build(
        [&](g2m::SurfaceKind kind) -> std::uint16_t {
            if (kind == g2m::SurfaceKind::Unknown) return static_cast<std::uint16_t>(paved);
            if (failure) return 0;
            const std::string name = names.kind_name(kind);
            const ps::SurfaceId id = table.id_for(name);
            if (id == ps::kInvalidSurfaceId) {
                failure = make_error(ErrorCode::InvalidArgument,
                                     "Session: terrain.class surface kind '" + name + "' is not in the surface table");
                return 0;
            }
            return static_cast<std::uint16_t>(id);
        },
        [paved, unpaved, off_road](g2m::LandClass land) -> std::uint16_t {
            if (land == g2m::LandClass::PavedRoad) return static_cast<std::uint16_t>(paved);
            if (land == g2m::LandClass::UnpavedRoad) return static_cast<std::uint16_t>(unpaved);
            return static_cast<std::uint16_t>(off_road);
        });
    if (failure) return *failure;
    return ClassSurfaceConfig{classes, off_road};
}

Result<RoadSurfaceConfig> bind_road_surfaces(const ps::io::SurfaceTable& table, const SurfaceNames& names) {
    const auto ids = resolve_grip_ids(table, names);
    if (!ids) return ids.error();
    const g2m::RoadValueLut lut = g2m::RoadValueLut::by_land_class(
        ids->off_road, {{g2m::LandClass::PavedRoad, ids->paved}, {g2m::LandClass::UnpavedRoad, ids->unpaved}});
    return RoadSurfaceConfig{lut, ids->off_road};
}

} // namespace g2m::ps_bridge
