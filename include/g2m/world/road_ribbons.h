// g2m/world/road_ribbons.h - the road ribbon mesh of one level-2 tile (terrain migration plan phase 3a): ribbons per
// paved stretch at 1 m stations, draped on the ground or on a structure deck, lane/marking flags from the served
// cross-section and the tags, turn arrows, and (owner tile only) the served junctions: asphalt patches over the road bed
// rings cut on the terrain lattice, the gravel verge around them, the through road's markings continued over them,
// gores, turn guide lines and the stop, give-way and crossing lines. Ported from racing_game core/src/road_visual.cpp
// (0c027dc) with det math. Render only; colours and the GPU upload stay with the consumer. Engine-neutral.
#pragma once

#include "g2m/core/tile_key.h"
#include "g2m/layer/road_geom_tile.h"
#include "g2m/world/road_structures.h"

#include <cstdint>
#include <functional>
#include <optional>
#include <string_view>
#include <vector>

namespace g2m::world {

// flags (bits, read by the consumer's road shader): 1 lane markings, 2 edge lines, 4 solid separator, 8 fast road,
// 16 solid paint, 32 overlay (only the paint is drawn), 64 / 128 left / right edge line broken (a junction mouth),
// 512 / 1024 left / right edge line not drawn, 2048 paving stones (no painted lines), 4096 concrete, 8192 unpaved (a
// junction patch piece at a track's mouth, drawn like the terrain's dirt), 16384 with 2048 = sett / cobblestone (natural
// stone). Bit 256 is left free for the consumer (the game's adapter sets it for a verge vertex).
// x, y: metres from the tile's south-west corner; z: absolute height. u: lateral offset from the centre of the counted
// lanes, v: station, width: the counted lanes' width; verge 1 = the gravel strip beside the road bed (no asphalt, no
// paint).
struct RoadRibbonVertex {
    float x, y, z, u, v, width, fade, lanes, forward, flags, verge = 0;
};
// The RoadRibbonTile product: one mesh per level-2 tile (indices: triangles into `vertices`).
struct RoadRibbonTile {
    TileKey key;
    std::vector<RoadRibbonVertex> vertices;
    std::vector<std::int32_t> indices;
    // Turn-arrow diagnostics (not part of the mesh): lanes that got an arrow, merge arrows among them (Verschwenkungspfeil),
    // and turn:lanes values of the lanes painted that are no OSM value (they draw nothing).
    std::size_t turn_arrow_lanes = 0, merge_arrows = 0, unknown_turn_tokens = 0;
    // Lattice drape counters (diagnostics, vault G2M-025): ribbon triangles fed in, triangles out (of which `uncut`: emitted whole,
    // inside the tolerance), pieces dropped for lack of ground, and lattice nodes read from the caller's ground function.
    std::uint32_t drape_in = 0, drape_out = 0, drape_uncut = 0, drape_dropped = 0, drape_node_reads = 0;
};
// Glyphs of the turn arrows (bits). Left/right: the full bent arrow; slight_*: a shallower bend; sharp_*: a bend back
// toward the side; merge_*: the diagonal straight arrow (Verschwenkungspfeil, StVO Zeichen 297.1) of the lane that ends.
enum ArrowGlyph : std::uint16_t {
    kArrowThrough = 1u << 0, kArrowLeft = 1u << 1, kArrowRight = 1u << 2, kArrowSlightLeft = 1u << 3, kArrowSlightRight = 1u << 4,
    kArrowSharpLeft = 1u << 5, kArrowSharpRight = 1u << 6, kArrowMergeLeft = 1u << 7, kArrowMergeRight = 1u << 8 };
// The glyphs painted for ONE lane of a turn:lanes value (the text between two '|', ';' joins values of the lane): through,
// left and right as before, slight_* and sharp_* their own bends, merge_to_* only when no other direction shares the
// lane (that lane ends). none and reverse paint nothing. A value that is not an OSM turn value (parse_turn_token) paints
// nothing and adds one to *unknown_tokens. Every value is matched whole - never as a substring.
std::uint16_t lane_arrow_glyphs(std::string_view lane_value, std::size_t* unknown_tokens = nullptr);
// Ground height (absolute metres) at absolute UTM (easting, northing); nullopt = no ground there (NoData, not loaded).
using RoadRibbonGround = std::function<std::optional<double>(double, double)>;

// How a ribbon vertex finds its height on the ground (vault G2M-025).
//   PointSampled  every vertex = `ground` at its own position (the pre-G2M-025 behaviour: bilinear if the caller's
//                 function is the bilinear sampler). Between its vertices a ribbon triangle is a plane that the terrain
//                 mesh does not follow, so on a steep bank the terrain stands above the road.
//   Lattice       the ribbon is draped on the SAME triangle surface the terrain mesh and the physics ground have: the
//                 1 m lattice (cell centres at x.5 m absolute) with the NW-SE diagonal of height_sampler.h
//                 (sample_height_exact). `ground` is read only at lattice nodes. Every ground-drawn ribbon triangle is
//                 cut where it crosses a lattice line or diagonal (vertices inserted there, shared between neighbours,
//                 so the mesh stays watertight), so no terrain triangle can stand above the road inside it.
enum class RibbonDrape : std::uint8_t { PointSampled = 0, Lattice = 1 };
struct RoadRibbonOptions {
    RibbonDrape drape = RibbonDrape::Lattice;
    // Lattice drape: a ribbon triangle is emitted uncut when its edges and every point where the terrain bends inside it
    // stay within this many metres of the triangle's own plane (cracks are impossible: an edge is judged on its own).
    // 0 cuts every triangle on every lattice line and diagonal.
    double drape_tolerance_m = 0.008;
    // The same for the markings, which sit a few mm above the road bed and must not fall under it.
    double drape_mark_tolerance_m = 0.003;
};

// The ribbons of `geometry` inside `key` (each 1 m row and each marking is kept by the tile holding its centre), lifted
// `lift_m` above the surface. Structure decks come from build_road_decks(geometry, separation); a stretch is drawn on a
// deck of its own (way, stretch start_ref) where one covers the station.
RoadRibbonTile build_road_ribbons(const RoadGeomTile& geometry, const TileKey& key, const RoadRibbonGround& ground, double lift_m,
                                  const RoadSeparation& separation = {}, const RoadRibbonOptions& options = {});
// The junction patches alone (appended to `mesh`), in tile-local metres like build_road_ribbons.
void add_junction_surfaces(RoadRibbonTile& mesh, const RoadGeomTile& geometry, const TileKey& key, const RoadRibbonGround& ground, double lift_m,
                           const RoadRibbonOptions& options = {});

// Priority rank of a road for right of way (StVO 18(3) / 8): motorway 14, trunk 12, primary 10, secondary 8,
// tertiary 6, unclassified / road 4, residential 2, living_street and anything else 0, every *_link 1. The ribbons pick
// the through road and the implied give-way lines with it.
int road_priority_rank(const RoadGraphWay& way);

} // namespace g2m::world
