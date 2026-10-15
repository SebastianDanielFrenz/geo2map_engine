#!/usr/bin/env bash
# Writes the racing_game 5a3baed sources the phase-3b pins (tests/world/view_pins.inc, tests/world/building_pins.inc)
# were recorded from, extracted VERBATIM from the commit (not the working tree) into <out dir>:
#   whole files: rg/buildings.h, buildings.cpp, rg/terrain_render.h, rg/road_classes.h, rg/terrain_stream.h,
#                rg/terrain_view_streamer.h, terrain_view_streamer.cpp;
#   by pattern range: world_terrain_view.cpp (world_terrain.cpp's colour, carve and chunk-build helpers and the
#                public view functions, without the WorldTerrain class), terrain_stream_part.cpp (terrain_stream.cpp's
#                constants, reselect_distance_for_speed, chunk_time_to_reach_s, sort_adds_by_urgency),
#                lod_settings_part.cpp (changed_road_lod_levels);
#   stubs: rg/world_terrain.h (the real header's TileCarveFn and TerrainViewSource text plus the declarations of the
#          view functions), rg/lod_settings.h (the changed_road_lod_levels declaration).
# Usage: extract_view_ref.sh <racing_game checkout> <out dir>; then configure with -DG2M_WORLD_VIEW_REFGEN_DIR=<out dir>,
# build g2m_world_view_refgen and run it: its stdout is view_pins.inc followed by building_pins.inc (split at the
# "// ---- building_pins.inc" line).
set -euo pipefail
GAME="$1"
OUT="$2"
REV=5a3baed
show() { git -C "$GAME" show "$REV:$1"; }
mkdir -p "$OUT/rg"
for h in buildings terrain_render road_classes terrain_stream terrain_view_streamer; do
  show "core/include/rg/$h.h" > "$OUT/rg/$h.h"
done
show core/src/buildings.cpp > "$OUT/buildings.cpp"
show core/src/terrain_view_streamer.cpp > "$OUT/terrain_view_streamer.cpp"
{
  echo '#pragma once'
  echo '#include "rg/road_classes.h"'
  echo '#include "rg/terrain_render.h"'
  echo '#include "g2m/layer/osm_roads.h"'
  echo '#include "g2m/mesh/lod_select.h"'
  echo '#include "g2m/mesh/terrain_chunk.h"'
  echo '#include "g2m/world/chunk_road_classes.h"'
  echo '#include <atomic>'
  echo '#include <memory>'
  echo '#include <vector>'
  echo 'namespace rg {'
  show core/include/rg/world_terrain.h | sed -n '/^using TileCarveFn/p'
  cat <<'DECL'
ChunkCarve chunk_window_carve(const g2m::mesh::ChunkKey& key, void* ctx, TileCarveFn fn);
void build_static_view_from_lookup(double cam_x, double cam_y, const g2m::mesh::LodParams& params, double e0,
                                   double n0, g2m::mesh::TileLookup lookup, void* ctx, unsigned thread_count,
                                   std::vector<RenderChunk>& out, const g2m::world::ClassLookup& class_lookup = {},
                                   const RoadSurfaceMap& surfaces = {}, TileCarveFn carve_state = nullptr);
void select_view_keys(double cam_x, double cam_y, const g2m::mesh::LodParams& params, double e0, double n0,
                      std::vector<g2m::mesh::ChunkKey>& out);
bool build_render_chunks(const std::vector<g2m::mesh::ChunkKey>& keys, g2m::mesh::TileLookup lookup, void* ctx,
                         double e0, double n0, unsigned thread_count, std::vector<RenderChunk>& out,
                         const std::atomic<bool>* cancel = nullptr, const g2m::world::ClassLookup& class_lookup = {},
                         const RoadSurfaceMap& surfaces = {}, TileCarveFn carve_state = nullptr);
DECL
  show core/include/rg/world_terrain.h | sed -n '/^struct TerrainViewSource {/,/^};/p'
  echo '}'
} > "$OUT/rg/world_terrain.h"
cat > "$OUT/rg/lod_settings.h" <<'STUB'
#pragma once
#include "g2m/layer/osm_roads.h"
#include <vector>
namespace rg { std::vector<int> changed_road_lod_levels(const g2m::RenderRoadLod* a, const g2m::RenderRoadLod* b, int max_level); }
STUB
{
  echo '#include "rg/lod_settings.h"'
  echo 'namespace rg {'
  show core/src/lod_settings.cpp | sed -n '/^std::vector<int> changed_road_lod_levels/,/^}/p'
  echo '}'
} > "$OUT/lod_settings_part.cpp"
{
  echo '#include "rg/terrain_stream.h"'
  echo '#include <algorithm>'
  echo '#include <cmath>'
  echo 'namespace rg {'
  echo 'namespace {'
  show core/src/terrain_stream.cpp | sed -n '/^constexpr double kMinDirectionSpeedMps/,/^double clamp01/p'
  echo '}'
  show core/src/terrain_stream.cpp | sed -n '/^double reselect_distance_for_speed/,/^LodCoverage lod_coverage_ahead/p' | sed '$d'
  echo '}'
} > "$OUT/terrain_stream_part.cpp"
{
  show core/src/world_terrain.cpp | sed -n '/^#include/p' | grep -v 'rg/route_check.h'
  echo '#include <atomic>'
  echo 'namespace rg {'
  echo 'namespace {'
  show core/src/world_terrain.cpp | sed -n '/^std::uint32_t pack_rgba/,/^const g2m::HeightTile\* world_terrain_lookup/p' | sed '$d'
  echo '}'
  show core/src/world_terrain.cpp | sed -n '/^ChunkCarve chunk_window_carve/,/^WorldTerrain::~WorldTerrain/p' | sed '$d'
  echo '}'
} > "$OUT/world_terrain_view.cpp"
