// Engine-neutral procedural building appearance. Algorithmic reference:
// BeamNG.Earth buildings.py, MIT (c) 2026 TecnaGamer; see THIRD_PARTY_NOTICES.md.
#pragma once
#include "g2m/world/buildings.h"
#include <array>
#include <cstdint>
#include <vector>

namespace g2m::world {
enum class BuildingMaterial : std::uint8_t { Wall, Roof, Glass, Frame };
enum class RoofForm : std::uint8_t { Flat, Gabled, Hipped, Skillion, Pyramidal };
struct BuildingVertex {
    std::array<float, 3> position{}, normal{}, colour{};
    std::array<float, 2> uv{};
    BuildingMaterial material = BuildingMaterial::Wall;
};
struct BuildingMeshOptions {
    // Inputs are relative to this absolute UTM origin, z stays orthometric.
    double origin_e = 0, origin_n = 0;
    double storey_height_m = 3, bay_width_m = 2.8, eave_m = 0.2;
    std::uint64_t seed = 0;
    bool facade_detail = true;
    bool infer_residential_roofs = true;
    std::uint32_t max_windows = 512; // per building; bound cost independently of height
};
struct BuildingMesh {
    std::vector<BuildingVertex> vertices; // triangles, CCW in E/N/up
    RoofForm roof = RoofForm::Flat;
    bool inferred_roof = false;
    bool unsupported_roof = false; // mapped shape/orientation/footprint not representable
    std::uint32_t windows = 0;
};
// Pure render geometry, never collision. Total mapped height is an upper bound,
// not eave height. Courtyards remain open. Complex footprints use an exact flat
// roof rather than a bounding-box roof spanning outside the mapped building.
BuildingMesh build_building_mesh(const Building&, const BuildingMeshOptions& = {});
} // namespace g2m::world
