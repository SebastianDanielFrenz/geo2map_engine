#!/usr/bin/env bash
# Writes the racing_game 0c027dc sources the phase-3a mesh pins (tests/world/road_mesh_pins.inc) were recorded from,
# extracted VERBATIM from the commit (not the working tree) into <out dir>:
#   rg/road_structures.h, rg/road_visual.h, road_structures.cpp, road_visual.cpp (whole files),
#   rg/npc_traffic.h (a stub declaring traffic_road_rank) and traffic_rank.cpp (npc_traffic.cpp's tag() and
#   traffic_road_rank(), by line range).
# Usage: extract_mesh_ref.sh <racing_game checkout> <out dir>; then configure with -DG2M_WORLD_MESH_REFGEN_DIR=<out dir>,
# build g2m_world_mesh_refgen and run it: its stdout is road_mesh_pins.inc.
set -euo pipefail
GAME="$1"
OUT="$2"
REV=0c027dc
show() { git -C "$GAME" show "$REV:$1"; }
mkdir -p "$OUT/rg"
show core/include/rg/road_structures.h > "$OUT/rg/road_structures.h"
show core/include/rg/road_visual.h > "$OUT/rg/road_visual.h"
show core/src/road_structures.cpp > "$OUT/road_structures.cpp"
show core/src/road_visual.cpp > "$OUT/road_visual.cpp"
cat > "$OUT/rg/npc_traffic.h" <<'STUB'
#pragma once
#include "g2m/layer/road_graph.h"
namespace rg { int traffic_road_rank(const g2m::RoadGraphWay&); }
STUB
{
  echo '#include "rg/npc_traffic.h"'
  echo '#include <map>'
  echo '#include <string>'
  echo 'namespace rg {'
  show core/src/npc_traffic.cpp | sed -n '13p;104,111p'   # tag(), traffic_road_rank()
  echo '}'
} > "$OUT/traffic_rank.cpp"
