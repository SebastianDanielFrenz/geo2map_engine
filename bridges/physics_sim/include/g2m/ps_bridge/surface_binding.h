// g2m/ps_bridge/surface_binding.h - binding geo2map's surface classes to a physics_sim SurfaceTable (terrain
// migration phase 9, moved from racing_game's Session::setup_terrain and resolve_grip_surface).
//
// The consumer (the game) owns the surface NAMES (its data/surfaces/surfaces.json) and the SurfaceTable ids; geo2map
// owns the rule that turns a classified cell into an id (g2m::classes::SurfaceIdTable, g2m::RoadValueLut). These two
// functions are the glue: they resolve the NAMES through the table and build the config G2mTerrainSource takes.
//
//   * bind_road_surfaces: road mode (RoadSurfaceConfig): paved / unpaved / off_road by LandClass only.
//   * bind_class_surfaces: class mode (ClassSurfaceConfig): every SurfaceKind through names.kind_name, and the land
//     classes through paved / unpaved / off_road. SurfaceKind::Unknown takes the paved id without asking kind_name
//     (a classifier with no facts on a road cell).
//
// Every resolved id must be present in the table AND < 255: the Jolt heightfield material index is 8 bits and slot
// 255 is reserved for ps::kInvalidSurfaceId (physics_sim core/src/backend/jolt/jolt_backend.cpp's
// shared_surface_material_list()). Both failures are returned as ErrorCode::InvalidArgument with the game's own
// message text ("Session: physics.road_surfaces.<field> '<name>' is not in the surface table", "... has surface id N
// (must be < 255)", "Session: terrain.class surface kind '<name>' is not in the surface table"). The uniform
// physics.terrain_surface path has no such check and keeps none: callers look that one name up with
// SurfaceTable::id_for directly.
#pragma once

#include "g2m/core/result.h"
#include "g2m/ps_bridge/g2m_terrain_source.h"

#include "ps/io/surface_table.h"

#include <functional>
#include <string>

namespace g2m::ps_bridge {

struct SurfaceNames {
    std::string paved, unpaved, off_road;
    // SurfaceKind -> SurfaceTable entry name (class mode only; the game's rg::surface_name_for_kind). Never called
    // for SurfaceKind::Unknown. May be empty for road mode.
    std::function<std::string(g2m::SurfaceKind)> kind_name;
};

[[nodiscard]] Result<ClassSurfaceConfig> bind_class_surfaces(const ps::io::SurfaceTable& table, const SurfaceNames& names);
[[nodiscard]] Result<RoadSurfaceConfig> bind_road_surfaces(const ps::io::SurfaceTable& table, const SurfaceNames& names);

} // namespace g2m::ps_bridge
