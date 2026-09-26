// g2m/ps_bridge/version.h - g2m_ps_bridge scaffolding (PLAN.md R2.2 section
// 1b, G5). Placeholder proving the bridge target actually builds and links
// against both physics_sim's ps_core (Jolt included) and geo2map_engine's
// own g2m_phys, from a target outside either project - mirrors physics_sim's
// own ps/version.h (core/include/ps/version.h), which serves the identical
// purpose one level down. No simulation logic lives here; G2mTerrainSource
// (g2m_terrain_source.h, G6) is the real deliverable.
#pragma once

namespace g2m::ps_bridge {

// Returns the g2m_ps_bridge library version as "MAJOR.MINOR.PATCH".
const char* bridge_version_string();

} // namespace g2m::ps_bridge
