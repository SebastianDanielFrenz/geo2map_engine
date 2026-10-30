# Terrain and roads: how the ground is built and how roads sit on it (text twin)

Back to [pipeline overview](README.md). Illustrated version: [terrain_and_roads.html](terrain_and_roads.html) (open from disk, prints). This file is the text twin for agents. It has the same sections and anchors, with every diagram turned into structured records. The plan for moving the remaining game code is [../terrain_migration_plan.md](../terrain_migration_plan.md), section 6.

Status: 2026-10-20. geo2map origin/main 407380b. racing_game origin/codex/powertrain-swaps 326f2df. Open PRs are listed where they change a stage.

## 0. How to read this file

- One stage per `###` heading. Each stage has the fields `stage`, `input`, `output`, `owner`, `anchor`, `config`, `tickets`, plus a short `notes`.
- Owner keywords (stable, grep for them):
  - `OWNER=geo2map`: the code lives in geo2map.
  - `OWNER=racing_game`: the code lives in the game.
  - `OWNER=moving`: the code lives in the game today, and the migration plan moves it to geo2map.
  - `OWNER=seam`: the physics_sim seam. It is drawn as its own box (section 5).
- Anchor form: `repo:path:line`. `g2m:` is geo2map at 407380b. `rg:` is racing_game at 326f2df. `rg#46:` and similar is a racing_game PR branch, `g2m#20:` a geo2map PR branch.
- `UNVERIFIED` marks a statement taken from a comment or doc that I did not trace in code.
- D-numbers (D42, D47, ...) are design decisions recorded in `docs/pipeline/roads.md` and the format docs. G2M-/RACE-/PHYS- ids are knowledge-vault tickets.

## 1. Overview

```
SERVER (geo2map derivers, cached, part of the release id)
  g2m.src.elev.dgm1 -> g2m.elev.base -> g2m.terrain.height L0-L6 ----------------------------+
  g2m.src.osm -> g2m.roads.graph -> g2m.roads.geom (enc 9) -> g2m.terrain.carved L0-L6 ------+--> served tiles
                                                         \-> g2m.terrain.class L0-L6 -------+
  g2m.src.osm -> g2m.rail.geom (opt-in, g2m#22) ----------------------------------------------+
CLIENT (in the game process)
  g2m_world (geo2map):  TerrainService, ViewStreamer chunks, RoadRibbonTile, StructureSet decks, buildings
  rg::WorldTerrain (game facade): game carve RoadSurfacePatch, game completion, route surfaces, layer resolution, rail meshes
  g2m_phys + g2m_ps_bridge (geo2map, opt-in bridge): physics tile grid, resident heights, ITerrainSource
  rg::Session + DeckInstaller (game): surface binding, start-up gate, deck colliders     <- the SEAM (section 5)
OUTPUTS
  physics ground = ps heightfield tiles (255 m)   collision decks = ps MeshShape bodies   render = chunks + ribbons + decks
```

Key fact today (2026-10-20): the game does NOT read `g2m.terrain.carved` or the served smoothing. It fetches the raw `g2m.terrain.height`, smooths it with its own Gaussian and carves it with its own `RoadSurfacePatch`. Physics and render both use that game-carved tile. The served carve is ready but has no consumer yet. The switch is migration phase 7.

## 2. Terrain stages

### T1 Raw DGM1 source
- stage: `g2m.src.elev.dgm1`, level 2, a 1024 m tile with 1024 x 1024 samples of 1 m.
- input: Hessen DGM1 (DHHN2016 normal heights).
- output: source tiles in the store.
- owner: OWNER=geo2map (source importer).
- anchor: `g2m:src/layer/src_elev.cpp`
- notes: bare earth. Bridge decks are removed and their gap is interpolated, which leaves a hump under every overpass (this is what D64 deals with).

### T2 Base elevation
- stage: `g2m.elev.base` L0: 256 m, 256 x 256 samples, 1 m cells, cell-registered (each sample at its cell centre), row 0 = south.
- input: the one L2 ancestor of `g2m.src.elev.dgm1` (16 windows per source tile).
- output: int32 heights in 1/256 m. NoData = INT32_MIN.
- owner: OWNER=geo2map.
- anchor: `g2m:src/layer/derive_terrain.cpp:45` (ElevBaseDeriver).
- config: config text `g2m.elev.base/1 dgm1-only`.

### T3 Height pyramid (served levels)
- stage: `g2m.terrain.height` L0-L6. L0 is the same bytes as elev.base. Each level above is the mean of 4 children, rounded half to even. A child with any NoData gives NoData.
- cell size: L0 1 m, L1 2 m, L2 4 m, ... L6 64 m. Every tile has 256 x 256 samples.
- owner: OWNER=geo2map.
- anchor: `g2m:src/layer/derive_terrain.cpp:131` (TerrainHeightDeriver); `g2m:include/g2m/layer/height_tile.h` (`downsample_2x2`).
- notes: no roads are in this layer. It is what the game fetches today.

### T4 Smoothing (two implementations)
- stage: soften the 1 m DEM noise before roads are carved.
- served version: OWNER=geo2map. An integer kernel [138, 621, 1024, 621, 138] at strength 35, inside `CarveTerrain`. Anchor `g2m:src/layer/terrain_carve.cpp:82` (`smooth_cell`).
- game version: OWNER=moving. A separable Gaussian using `std::exp`: radius 2 m, strength 0.35, 1 pass, with a halo taken from neighbour tiles. It runs at every level. Anchors `rg:core/src/terrain_smoothing.cpp` (61 lines) and `rg:core/include/rg/terrain_smoothing.h:8-10`. Called from `rg:core/src/world_terrain.cpp:248`.
- config: game `terrain_smoothing.{enabled,radius_m,strength,passes}` (`rg:core/src/world_config.cpp:496-501`); geo2map `data/carve/carve_default.json`.
- tickets: G2M-017 (the carve switch deletes the game's version with no fallback).

### T5 Carve (two implementations)
- stage: pull the smoothed terrain onto the road surface along each road, then blend back to the terrain.
- served version: OWNER=geo2map. `g2m.terrain.carved` L0 per 1 m cell:
  1. junction cores (D42 Shepard surface via `junction_height_m`, `g2m:src/layer/terrain_carve.cpp:743`) beat approach cores, and the lowest core wins;
  2. otherwise the blend of the largest weight. Blend width is 3-15 m, widened to a 1:1.5 slope;
  3. the deck clearance clamp keeps the ground 0.55 m under a bridge, ramped over 4 m from each end (`:860-872`);
  4. D59: a carve that ends at an underpass deck keeps steep walls, fading over 1 m (`:20-21`, `:795-797`);
  5. bridges, tunnels and OnStructure junctions are never carved.
  - With g2m#20 there is one exception, D65: a short straight tunnel under a bank is carved as an open cut.
  - L1-L6 are the mean of 4 of the carved level below.
  - Deriver anchor: `g2m:src/layer/terrain_carve.cpp:1135`.
- game version: OWNER=moving. `RoadSurfacePatch::apply`, `rg:core/src/road_surface.cpp` (228 lines, +53 in rg#46):
  1. 1 m segments of every accepted PavedRoad or tunnel profile, in 16 m bins (`:46`). Bridges are skipped (`:45`). Segments under inferred decks are skipped (`:64-67`).
  2. Per cell, every road near the cell computes its own surface height. The crown is `crown*(sqrt(l^2+0.25)-0.5)` (`:141`), or `g2m::road_crown_m` / `road_verge_m` when a cross-section exists (`:142-148`).
  3. RoadSeparation (`:155-173`) drops a road that is more than 2.5 m above or below another road's protected footprint.
  4. rg#46 junction tie-in (section 3, R10).
  5. The best road wins by normalised score. Its weight is a smoothstep over the blend width (`:174-180`). A 0.35 transition blend runs across roads of the same layer (`:186-198`).
  6. Result: `llround(height*256)` (`:199`).
  - Deck clearance: decks are cut to `top - 0.55*smooth(edge/4)` (`:221`). Tunnel-floor decks are excluded (`:19`).
  - It runs only for `key.level <= carve_max_level_` (default 2, `rg:core/include/rg/world_terrain.h:340`). L1 and L2 fall back to the smoothed tile and are healed later (`rg:core/src/world_terrain.cpp:637-670`).
  - It uses libm (`std::hypot`, `sqrt`, `llround`).
- config: game `road_visuals.verge_drop_m` (default 0.06, range 0-0.15; `rg:core/src/world_config.cpp:480`) and `road_separation.{clearance_m 2.5, margin_m 3.0, approach_reach_m 60}` (`:487-492`, `rg:core/include/rg/world_config.h:123-127`). Served: the config text `g2m.terrain.carved/1 ... junction=D42 underpass-deck=1 underpass-walls=steep/1 carve=<sha256>`.
- tickets: G2M-017, RACE-012 S12, RACE-023 (tie-in).
- notes, measured from code:
  - The game carves tunnel stretches into the terrain. `road_surface.cpp:46` admits tunnel entries as carve segments, so the hill is cut down to the tunnel floor along its footprint.
  - Lines 19, 102, 149 and 156 only exclude tunnels from the deck clearance cut, from the bed points, from footprint protection and from grade separation.
  - The served carve does the opposite: it never carves tunnels except D65 cuts.
  - This difference must be decided before phase 7. See plan section 6.4, owner question Q6.

### T6 Terrain class
- stage: `g2m.terrain.class` L0-L6. 257 x 257 nodes, with the planes land_class, surface, role and rank.
- L0 uses the carve ownership of T5 (`CarveModel::classify`, `g2m:src/layer/terrain_carve.cpp:879`). Roles: Terrain, RoadBed, Sidewalk, Verge, JunctionRoadBed, JunctionSidewalk, Blend, BridgeDeck. Land cover comes from `g2m.src.landcover`.
- L1-L6: rank-max over a 3 x 3 window.
- owner: OWNER=geo2map.
- anchor: `g2m:src/layer/terrain_class.cpp:234`; format `docs/formats/terrain_class.md`.
- consumers: the render ClassWindow (`g2m:include/g2m/world/chunk_terrain_classes.h:56`, D62) and physics class mode (`G2mTerrainSource` with `ClassSurfaceConfig`).
- game step: OWNER=moving. The game stamps its own route surfaces into the served L0 class tile (`stamp_route_surface_classes`, `rg:core/src/route_surface_mesh.cpp:376`, called at `rg:core/src/world_terrain.cpp:841`). Rules: a 0.45 m height gate against the ground, and priority paved 3 > walkway 2 > other 1.

### T7 Physics height tile
- stage: the ground the car drives on. A 255 m node-registered grid of 256 x 256 samples, so neighbouring tiles share their border samples. It is filled from the resident 256 m cell-registered tiles.
- input: the decoded height tile the game hands in. Today that is the game-carved tile (`rg:core/src/terrain_mode.cpp:26`, `HeightTileSharedFetch::fetch`), plus road segments or class tiles for grip.
- output: `ps::terrain::TileSample` (heights and per-cell SurfaceId) for the physics_sim TileManager.
- owner: OWNER=geo2map (`g2m_phys`) + OWNER=seam (`g2m_ps_bridge`).
- anchor:
  - `g2m:include/g2m/phys/physics_grid.h`
  - `g2m:include/g2m/phys/fill_physics_heights.h:47`
  - `g2m:include/g2m/phys/fill_physics_surfaces.h:132` (road mode)
  - `g2m:include/g2m/phys/fill_physics_classes.h:51` (class mode)
  - `g2m:src/phys/physics_streamer.cpp` (the gate)
  - `g2m:bridges/physics_sim/src/g2m_terrain_source.cpp:113` (`fill_from`), `:166` (`make_terrain_config`)
- config: game `physics.{radius_m 400, max_tile_fills_per_tick 1, loader_workers 2, lookahead_s 3.0, lookahead_cap_m, startup_timeout_s 60, startup_max_s, prefetch_margin_tiles, prefetch_max_tiles, road_surfaces.{enabled,paved,unpaved,off_road}, terrain_surface}` (`rg:core/include/rg/world_config.h:88-104`).
- notes:
  - `make_terrain_config` sets `suppress_terrain_under_static_geometry = false`, so the heightfield is never holed under decks.
  - The pool is (2*ceil(r/255)+3)^2 tiles, and the game doubles it for a second interest point (`rg:core/src/session.cpp:489-491`).

### T8 Render terrain chunks
- stage: LOD chunk meshes of the carved heights, coloured by class id.
- owner: OWNER=geo2map (`g2m_world` ViewStreamer, `g2m:src/mesh/terrain_chunk.cpp:210` `build_chunk`, `g2m:src/world/view_streamer.cpp`).
- Colours stay with the game: OWNER=racing_game, `rg:core/src/terrain_colours.cpp` (129 lines), applied through the ViewSource colour hook.
- Godot upload: OWNER=racing_game, `rg:godot_ext/src/rg_terrain_view.cpp` (1023 lines) and `terrain_surface_pack.cpp` (86).
- notes: the chunk mesh, the Jolt heightfield and `sample_height_exact` share one surface, a 1 m lattice of cell centres with each cell split along the anti-diagonal (G2M-025). `sample_height_bilinear` (`g2m:src/world/height_sampler.cpp:41`) is a different surface; `sample_height_exact` is `:89`.

## 3. Road stages

### R1 OSM road graph
- stage: `g2m.roads.graph` L2, with a 128 m halo, complete ways and midpoint ownership. It carries node tags, turn restrictions and via ways.
- input: `g2m.src.osm`.
- owner: OWNER=geo2map. Anchors `g2m:src/layer/derive_road_graph.cpp:103`, `g2m:src/layer/road_graph.cpp`, `road_attributes.cpp`.
- config: graph text `g2m.roads.graph/1 complete-ways default-RoadStyle halo=128 midpoint-mm-owner ... enc=6`.

### R2 Reference line, lanes and widths
- stage: per way stretch, a smooth reference line plus the cross-section bands (carriageway lanes, shoulders, verges, sidewalks, cycle lanes) from the tags (`lanes`, `width`, `sidewalk`, `cycleway`, `turn:lanes`) and `data/road_rules/de.json`.
- output: the `g2m.roads.geom` sections reference line, cross-section, features.
- owner: OWNER=geo2map. Anchors `g2m:src/layer/road_reference.cpp:284` (`fit_stretch`), `g2m:src/layer/road_cross_section.cpp`, `road_cross_section_join.cpp`, `road_features.cpp`.
- in flight:
  - g2m#21 bounds the reference line's distance from its OSM chord (G2M-024; the 32 m bow) and takes the turn-arrow tokens from a table.
  - The drape worktree adds a merge-lane taper (uncommitted, `road_cross_section_join.cpp`).
- tickets: G2M-024.

### R3 Vertical profile
- stage: a height profile along the reference line, fitted to the DEM samples (5 m spacing) with class grade and curvature limits.
- owner: OWNER=geo2map. Anchors `g2m:src/layer/road_profile.cpp`, `g2m:src/layer/vertical_profile.cpp:30` (`fit_vertical_profile`), `road_dem_sampler.cpp`, `road_dem_dependencies.cpp`.
- config: geom text `spacing=5 max-samples=2048 iterations=4096 ... sloping-plane-r3 approach=32`.
- notes: the strict pass declines a stretch it cannot fit. Bridges and tunnels return Unsupported here and are completed in R4.

### R4 Completion: bridges, tunnels, D64/D65, declines
- stage: give a profile to the stretches the strict pass left without one.
- owner: OWNER=geo2map (served since migration phases 4/5, D47/D48). Anchor `g2m:src/layer/road_completion.cpp:155`:
  - tunnels `:311`;
  - ground `:383`;
  - retry `:181`.
- What it does:
  - Accepted-bridge anchors (D47).
  - A bridge spans between abutments found by a walk of up to 200 m with a 205 m margin (D48).
  - TunnelFloor follows the DEM at 5 m spacing, at most 30 % grade, radius at least 20 m. From 200 m length it uses one straight grade between the portals (D52).
  - D43 ground ease.
  - A 20 m retry for "bounded solver" / "endpoint curvature".
  - Underpass decks D53-D58: a ground stretch over a lower road in a DEM cutting gets a deck span on the rim chord, at most 80 m, with 2 m abutments. Anchor `g2m:src/layer/road_underpass.cpp` (`spans :472`, `mask :571`).
  - D61 tags, D63 joins.
- in flight, g2m#20:
  - D64: a road under an upper bridge ignores the bridge deck's DEM hump (`RoadUnderpassFinder::overpass_spans`).
  - D65: a tunnel stretch of at most `tunnel.cut_max_length_m` 80 m, whose DEM rises 1 to 8 m above the chord of its portals, is an open cut. Its floor is that chord, and the carve cuts the bank along it unless a ground road crosses it in plan.
- game copy: OWNER=moving, not yet deleted.
  - `complete_road_profiles`, `rg:core/src/road_structures.cpp:56-161`: tunnel floors `:69-90`, bridges with a 32 m-budget abutment walk `:21-53` and `:91-121`, ground only for declined stretches `:122-161`.
  - The 20 m retry, `rg:core/src/world_terrain.cpp:369-381`.
  - Both run on the served geometry after fetch. They only touch entries the server left without a profile (`road_structures.cpp:129`).
- config: `de.json` `tunnel.straight_grade_min_length_m` 200, `tunnel.cut_max_length_m` 80, `cut_hump_min_mm`, `cut_hump_max_mm` (g2m#20). Geom text `completion=1 completion-retry=20/16384 completion-abutment=32/200 ... tunnel-floor=5/0.3/20/chord:200 tunnel-cut=80/1/8 ... overpass-mask=D64` (g2m#20).
- tickets: G2M-019 (ground ease served, the game trusts it).

### R5 Carve and imprint
- See T5. The road imprint on the ground is the carve. A bridge leaves no imprint, only the clearance clamp under its deck. A tunnel leaves none in the served carve, but a full cut in the game carve (T5 notes).

### R6 Structures and decks
- stage: deck meshes for bridge and tunnel profiles, for served underpass decks, for inferred tunnel roofs (an upper road of rank >= 5 on a higher layer over a tunnel way; extent max(20, (w/2+4)/sine) m) and for inferred approach decks (RoadSeparation).
- The mesh has 5 vertices across per 1 m station (rounded crown), a 0.5 m slab and fascia sides.
- owner: OWNER=geo2map (`g2m_world`, ported in phase 3a). Anchors `g2m:src/world/road_structures.cpp:242` (`build_road_decks`), `:228` (`road_deck_height`); header `g2m:include/g2m/world/road_structures.h`.
- consumers:
  1. the game carve's deck clearance cut;
  2. the render deck upload, `rg:godot_ext/src/rg_terrain_view.cpp:711-727`: colours 0x4c4c52 paved / 0x5c4729 unpaved, 2 per frame;
  3. the collision decks (section 5).
- notes: the game publishes decks from `produce_road_surface` into `published_decks_` keyed (way, start, end) (`rg:core/src/world_terrain.cpp:394`). A deck exists only after the game derived that L2 road surface.

### R7 Ribbons and markings
- stage: the visible road surface. Carriageway ribbon, verge strips, cross-section bands, lane paint, turn arrows, German junction markings and junction patches, sampled per 1 m.
- owner: OWNER=geo2map (`g2m_world`). Anchor `g2m:src/world/road_ribbons.cpp:353` (`build_road_ribbons`).
- ground: the game passes `sample_height_bilinear` over the game-carved tile (`rg:core/src/world_terrain.cpp:467-559`). Bridge and tunnel stretches are drawn by the deck path, not as draped ribbons (`g2m:src/world/road_ribbons.cpp:441`, comment).
- in flight: the drape branch (`claude/ribbon-drape-taper`, commit d436eb1, G2M-025) drapes ribbons on the terrain triangulation instead of bilinear sampling. Before it, the terrain stood up to 0.3-0.4 m over the road bed on steep banks, and up to 2.8 m at the Main-Taunus-Zentrum cloverleaf. Tolerance 8 mm for the bed, 3 mm for paint. `RoadRibbonOptions::drape` selects Lattice or PointSampled.
- tickets: G2M-025 (proposed in the branch), RACE-027.

### R8 Route surfaces (game)
- stage: surfaces the ribbon does not draw:
  - unpaved roads, sidewalk and cycle bands as 1 m strips, trimmed out of junctions, with height from the deck, the profile or the ground;
  - OSM foot, cycle and path ways that are not in the road graph.
- owner: OWNER=moving. Anchors `rg:core/src/route_surface_mesh.cpp:150` (`append_route_surface_mesh`), `:294` (`append_standalone_path_mesh`), `:376` (`stamp_route_surface_classes`). 463 lines.
- flags: kStone 2048, kConcrete 4096, kUnpaved 8192, kSett 16384, kWalkway 32768.

### R9 Surface layer resolution (game, rg#51)
- stage: the visual layers never overlap.
- Rank: verge 0 < walkway 1 < unpaved strip 2 < carriageway 3 < junction patch 4. A lower-ranked triangle is cut against every higher-ranked one within 0.25 m height and keeps only the uncovered part.
- Where two carriageways overlap at the same height, the one with more lanes wins.
- Decals are lifted 25 mm instead (`kDecalLiftM`).
- owner: OWNER=moving (open PR). Anchors `rg#51:core/include/rg/surface_layers.h` (36 lines), `rg#51:core/src/surface_layers.cpp` (223), called at `rg#51:core/src/world_terrain.cpp:513`.
- tickets: RACE-027.
- notes: it addresses layer-vs-layer mismatch (2-24 mm measured). The drape (R7) addresses ribbon-vs-terrain. Both are needed. The 25 mm decal lift should be measured again after the drape lands.

### R10 Junctions
- stage: one surface where roads meet.
- served version: OWNER=geo2map. `build_road_junctions` (`g2m:src/layer/road_junction.cpp:1014`) and `junction_height_m` (`:1386`). D42 mixes the approach bases, and each approach's crown and verge relief fades out from its trim over 3-15 m and 1.5 m. It is used by the served carve (T5) and by the ribbons' junction patches.
- game version: OWNER=moving (rg#46, RACE-023).
  - A narrower road meeting a wider one at more than 30 degrees, whose half width is more than 0.25 m smaller, takes the wider carriageway's crowned surface.
  - It returns to its own profile over the wider road's shoulder plus at most 3 m.
  - It applies only at grade: fully up to 0.5 m height gap, fading to none at 1.0 m.
  - Anchor `rg#46:core/src/road_surface.cpp` (+53 lines). It misses a third road at the same node (the example in rg#47 PLAN at UTM 467919.5 / 5550447.5).
- plan: rg#47 and g2m#19 record that G3-D junction surfaces replace the tie-in. See plan section 6.5.

### R11 Rail (in flight)
- served: OWNER=geo2map, opt-in. `g2m.rail.geom` v1, L2: a centreline plus a height profile taken from the rail line itself (g2m#22, G2M-026).
- game: OWNER=moving (rg#52, RACE-028). It builds:
  - the visual meshes: deck slab, ballast trough, parapets, rails;
  - the collision decks per bridge piece, dropping every span whose underside is less than `collision.min_clearance_m` 5.0 m above the ground (`collision.edge_margin_m` 0.5).
  - Anchors `rg#52:core/include/rg/rail_meshes.h` (126), `core/src/rail_meshes.cpp` (440), `rail_decks_near` in `core/src/world_terrain.cpp` (+111). It converts rail decks to `RoadDeck` and feeds them into `sync_road_decks` (`rg#52:core/src/session.cpp:1104-1106`).
- config: `data/world/rail.json` (rg.rail/1).

## 4. The three outputs

### O1 Physics ground
- what: physics_sim heightfield tiles, 255 m, 256 x 256 node-registered, filled from the resident 256 m tiles. Today these are the game-carved heights. After phase 7 they will be the served carved heights.
- grip:
  - uniform mode: one SurfaceId;
  - road mode: road segments rasterised by LandClass to paved, unpaved or off_road (`RoadValueLut`);
  - class mode: `g2m.terrain.class` through `SurfaceIdTable`.
- chain: `HeightTileSharedFetch` (`rg:core/src/terrain_mode.cpp:26`) -> `HeightTileLoader` -> `ResidentHeightSet` -> `PhysicsTerrainStreamer` gate -> `G2mTerrainSource::fill_from` -> `ps::World::set_terrain_source`.
- owner: OWNER=geo2map + OWNER=seam.

### O2 Collision decks
- what: one static `ps::MeshShape` body per deck (bridge, tunnel floor, underpass deck, inferred roof, approach deck, rail bridge span). The surface is the paved or unpaved road_surfaces name.
- chain: `WorldTerrain::road_decks()` (`rg:core/src/world_terrain.cpp:439`) -> `Session::sync_road_decks` (`rg:core/src/session.cpp:1089-1129`) -> `DeckInstaller` (`rg:core/src/deck_installer.cpp`) -> `World::create_shape` on 2 worker threads, `World::create_body` in DeckKey order.
- Radii: required = physics radius + 255 m; built ahead = a further 600 m; 16 installs per tick.
- owner: OWNER=seam (game code today, plan phase 9).

### O3 Render meshes
- terrain chunks (T8), road ribbons plus route surfaces (R7-R9), deck meshes (R6), buildings (`g2m_world` BuildingSet; tessellated in `rg:game/scripts/building_stream.gd:120-227`), rail meshes (R11).
- Streaming policy: geo2map `RingStreamer`, wrapped by `rg:godot_ext/src/rg_ring_streamer.cpp` and used by `road_stream.gd`, `building_stream.gd` and `rail_stream.gd`.
- owner: OWNER=geo2map (builders) + OWNER=racing_game (upload, colours, materials) + OWNER=moving (route surfaces, layers, rail, building tessellation).

## 5. The physics seam (OWNER=seam)

The seam is where geo2map data becomes physics_sim objects. G2M-001: geo2map core never depends on physics_sim. Only the opt-in target `bridges/physics_sim` (`g2m_ps_bridge`, pinned by `bridges/physics_sim/physics_sim.pin`) sees ps types.

| Seam piece | Where now | Lines | Logic or glue | Anchor |
|---|---|---|---|---|
| S1 `G2mTerrainSource` (ITerrainSource: uniform, road and class modes; prefetch Snapshot) + `make_terrain_config` | geo2map bridge | 363 (177 h + 186 cpp) | logic, already shared | `g2m:bridges/physics_sim/src/g2m_terrain_source.cpp` |
| S2 Physics terrain stack (grid, resident, loader, streamer, source, in destruction order) | game `Session::Terrain` | 71 | mostly glue, about 40 lines of composition any game needs | `rg:core/src/session.cpp:91-161` |
| S3 Surface binding (road_surfaces names -> SurfaceTable ids, < 255 check, SurfaceKind -> name -> id, LandClass -> paved/unpaved/off_road) | game `resolve_grip_surface` + `setup_terrain` | 15 + about 45 | logic | `rg:core/src/session.cpp:193-207`, `:430-475` |
| S4 Start-up gate and priming (retry failed, timeout verdict, prime ticks ceil(square/fills)+1, resident/starved/miss checks) | game `setup_terrain`, `gate_check` | about 70 of 172 + 40 | logic about 70, the rest progress/trace/cancel glue | `rg:core/src/session.cpp:401-572`, `:808-847` |
| S5 Deck selection (required vs prefetch radius, DeckKey order, surface per land class) | game `sync_road_decks` | 41 | logic about 30 | `rg:core/src/session.cpp:1089-1129` |
| S6 `DeckInstaller` + `make_deck_mesh_shape` (off-thread shapes, fixed install order, budget, wait, legacy path) | game | 371 (152 h + 219 cpp) | logic; depends only on g2m `RoadDeck` and ps | `rg:core/include/rg/deck_installer.h`, `rg:core/src/deck_installer.cpp` |
| S7 Height fetch adapter `HeightTileSharedFetch : g2m::phys::IHeightTileFetch` + `make_terrain_mode` | game | 214 (125 h + 89 cpp) | glue to the facade | `rg:core/src/terrain_mode.cpp` |
| S8 Rail decks -> RoadDeck | game (rg#52) | about 60 in `rail_decks_near` | logic | `rg#52:core/src/world_terrain.cpp` |
| S9 Surface names per SurfaceKind | game `surface_kinds.h` | 47 | game data | `rg:core/include/rg/surface_kinds.h` |

Measured totals:
- The game-side seam is about 984 lines (S2-S9). About 525 of them are logic that any game on geo2map plus physics_sim needs (S3, S4, S5 and S6 without its legacy path, plus the S2 composition). About 460 are game glue: progress atomics, tracing, env overrides, error text, cancel.
- The bridge (S1) adds 363.

Recommendation: outcome (a). Grow `g2m_ps_bridge` into the reusable adapter. Details are in plan section 6.6 (phase 9).

## 6. Cross-sections

Each record: case, which code sets the heights, where it is drawn, where it collides. "Today" means the game-carved ground. After phase 7 the served carve replaces it, with the same deck path.

### X1 Road on ground
- heights: profile R3/R4 (`g2m:src/layer/vertical_profile.cpp:30`), then the carve T5. Today `rg:core/src/road_surface.cpp`; after phase 7 `g2m:src/layer/terrain_carve.cpp`.
- drawn: ribbon R7 on the carved ground (bilinear today, lattice drape after G2M-025) + terrain chunk T8.
- collides: the heightfield O1 (carved heights), grip by class or road raster.

### X2 Bridge over road
- heights:
  - upper road: deck from the bridge profile (R4 bridge fit with abutment walk, D47/D48), mesh by `g2m:src/world/road_structures.cpp:242`;
  - lower road: R3/R4 (D64 masks the DEM hump, g2m#20);
  - ground under the deck: clamped to deck top - 0.55 m ramped over 4 m (game `road_surface.cpp:221`, served `terrain_carve.cpp:860-872`).
- drawn: deck mesh (render upload `rg_terrain_view.cpp:711`), lower ribbon on the carved ground.
- collides: deck = MeshShape body (O2), lower road = heightfield (O1).

### X3 Road under bridge
- heights: lower road from its own profile. The bare-earth hump of the removed deck is masked by D64 (g2m#20) instead of declining the way.
- RoadSeparation keeps the upper approach from lifting the lower road's ground (game `road_surface.cpp:155-173`; served: the lowest core wins). Whether the two are equivalent is UNVERIFIED.
- drawn: lower ribbon on the carved ground, deck above.
- collides: heightfield below. The rail bridge spans of rg#52 collide only with at least 5 m of clearance.

### X4 Short tunnel / open cut (D65)
- heights: TunnelFloor profile (R4). With D65 (g2m#20) the floor is the portal chord, and the served carve cuts the bank along it.
- Today the game already carves every tunnel (T5 notes), so D65 changes nothing for the game until phase 7.
- drawn: deck mesh with `tunnel_floor` = true (render), plus the cut ground. The rail or road bank above is drawn by its own deck if it is a bridge.
- collides: tunnel-floor deck (O2) and the cut heightfield.

### X5 Long tunnel
- heights: TunnelFloor. Straight grade between the portals from 200 m (D52), else the DEM at 5 m spacing, at most 30 % grade.
- An inferred roof deck is added where a rank >= 5 road on a higher layer crosses.
- game today: the hill is carved down to the floor (an open trench, measured from code, not seen in a drive).
- served carve: the hill stays. The physics heightfield would then lie over the tunnel floor deck, because heightfields are not holed (`make_terrain_config`). This is the open phase 7 question (plan 6.4, Q6).
- drawn: tunnel-floor deck + roof deck + terrain (carved today).
- collides: floor deck + roof deck + heightfield.

### X6 Junction with crown
- heights:
  - served: D42 junction surface (`junction_height_m`), approach relief fading over 3-15 m;
  - game today: the per-cell best-road rule, plus rg#46 tie-in onto the wider carriageway's crossfall (at grade only).
- drawn: junction patch (ribbons, rank 4 in rg#51) + markings.
- collides: heightfield with carved heights.

### X7 Verge and walkway
- heights:
  - verge: the verge drop (game 0.06 m by default, smoothstep over the shoulder; served `road_verge_m`);
  - sidewalk and cycle band: the cross-section crown at that offset;
  - standalone paths: the ground.
- drawn: ribbon verge strips + route surfaces R8, cut by layer rank R9.
- collides: heightfield. Grip is stamped by `stamp_route_surface_classes` (paved > walkway > unpaved, 0.45 m gate) into the class tile.

## 7. Index of config keys and tickets

- geo2map data: `data/road_rules/de.json` (tunnel.*, junction, furniture), `data/carve/carve_default.json`.
- game config:
  - `terrain_smoothing.*`, `road_visuals.verge_drop_m`, `road_separation.*`;
  - `physics.*` (section T7), `physics.road_surfaces.*`;
  - `lod.max_distance_m`, `rail_served` / `rail_config_path` (rg#52);
  - `data/world/rail.json`, `data/surfaces/surfaces.json`.
- tickets: G2M-001 (bridge boundary), G2M-010 (grip and render share one function), G2M-017 (carve switch, no fallback), G2M-019 (ground ease served), G2M-020 (chunk road classes moved), G2M-024 (reference bow), G2M-025 (lattice drape, proposed), G2M-026 (rail layer), G2M-027 (this re-inventory, proposed), RACE-012, RACE-023, RACE-027, RACE-028, PHYS-047 (phantom wheel casts on large shapes).
