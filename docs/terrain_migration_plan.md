# Terrain migration plan: racing_game -> geo2map

Status (2026-10-14): phases 0-2 done (game re-fit dropped; geo2map 6be9d3f phase 1, 6b6c621 phase 2, 8987291 msvc fix; racing_game switched to g2m::world::TerrainService). Phase 3a done (`StructureSet` / `RoadRibbonTile` builders in g2m_world, byte-pinned against the game; racing_game builds decks and ribbons through them); phase 3b (streamers, `BuildingSet`) is next. Owner answers to section 5 are at the end of this file. Originally written 2026-10-12 in geo2map out/ (gitignored); the files listed under Sources that live in out/ folders exist only on the original machine.

Owner decision, 2026-10-12, verbatim: "move the ENTIRE terrain logic into geo2map. It is needed for future projects."

Constraints:
- G2M-001: geo2map never depends on physics_sim or any game. The opt-in `bridges/physics_sim` target (`g2m_ps_bridge`) is the only place that sees ps types.
- G2M-010: grip and render share the same OSM road data through the same function.
- G2M-017 / RACE-012 S12: the carve switch deletes the game's carve with no fallback. Work-in-progress states are acceptable (Q1). Relaxed profiles are served flagged (Q2).
- G2M-019: D43 ground-ease is served. The game should trust served profiles instead of porting the ease.

Pins at the time of writing:
- racing_game a8a1b13
- geo2map 06b3b1e (D43, release id c5236c35103b)
- physics_sim 441e16d

Sources:
- racing_game/out/racing_game_context.md
- racing_game/out/racing_game_queue.md (2026-10-12 entries)
- racing_game/out/road_completion_gap.md
- geo2map/out/geo2map_context.md
- geo2map/docs/pipeline/README.md
- vault G2M-001, -010, -017, -019, RACE-012

Size classes used below:
- S: under about 300 changed lines
- M: 300-1000 lines
- L: over 1000 lines

---

## 1. Inventory

Columns:
- "Deps": what the piece needs. ps = physics_sim types, Godot = Godot types, cfg = game world config.
- "geo2map today": whether an equivalent already exists.
- Paths: racing_game paths unless prefixed `g2m:`. `h` = `core/include/rg/<file>.h`, `cpp` = `core/src/<file>.cpp`.

### 1.1 Terrain service facade (WorldTerrain)

| Piece | Anchors | What it does | Deps | geo2map today |
|---|---|---|---|---|
| `WorldTerrain::open` | world_terrain.cpp:211 | Opens source/derived stores. Calls `builtin_local_release_params(scope, derivers, sources, road_profiles_enabled, ...)`, builds an offline `g2m::server::Server` + `InProcessTransport` + `SessionFrame`. Takes `lod_params_.max_distance_m` from config. | cfg (world_config.cpp:449 `lod.max_distance_m`, store paths, `physics.road_surfaces.enabled`) | Server, transport and release params exist. **No client facade.** |
| Raw/decoded height fetch | `fetch_raw_decode` cpp:324, `raw_height_tile_shared` cpp:348, `fetch_and_decode` cpp:354, `cached_height_tile` cpp:607, `height_tile_shared` cpp:613 | Fetches `g2m.terrain.height` L0-L6 and caches the decoded tile. `fetch_and_decode` applies the game's smoothing at every level and, at L0, the game's road carve (`road_surface_patch` of the L2 ancestor). | none beyond g2m | Fetch exists (`client/cache.h`). Smoothing+carve equivalent = `g2m.terrain.carved` (opt-in, no consumer). |
| `decode_height_tile_container` | cpp:511 | Parses the container, checks layer/key, decodes, applies `height_offset` in int64. | none | Partly (layer decode exists). The container+offset helper is game-side. |
| `fetch_height_tile_cached` | cpp:551 | Free helper with the same cache. | none | No |
| `road_surface_patch` | cpp:369-449 | Runs **under `geometry_mutex_` (global, serialised)**. Fetches `g2m.roads.geom` L2. Maps "outside coverage" / "required dependency absent" / "deriver rejected" to an empty patch. Fetches DEM deps (`road_dem_dependencies`) and builds `RoadDemSampler`. Calls `complete_road_profiles`, then the 20 m retry (spacing 20, 16384 iterations, curvature exceptions, only on "bounded solver" / "endpoint curvature"). Caches the geometry, builds `RoadSurfacePatch(geometry, verge_drop_m_)` and publishes decks into `published_decks_` under `decks_mutex_`. | cfg (`road_verge_drop_m`, `road_surfaces.enabled`) | DEM sampler, dependencies, completion and retry exist (`g2m:layer/road_dem_sampler.h`, `road_dem_dependencies.h`, `road_completion.h`). Patch equivalent = carved deriver. **The serialisation is the cold-start bottleneck: 1.2-2.6 s per roads.geom tile, about 20 s gate in release, over 30 s in debug.** |
| Road graph / geom / decks accessors | `road_graph_tile` cpp:451, `road_decks` cpp:486, `cached_road_geometry_at` cpp:495 (try_lock), `road_geometry_at` cpp:504 | Accessors over the caches above. | none | Partly (fetch only) |
| Road segments | `RoadFetchStatus`, `decode_road_segments_response` cpp:672, `fetch_road_segments_cached` cpp:713, `road_segments_shared` cpp:744, `road_segments` cpp:770 | `osm_roads` segment extraction per tile, cached. Used for grip (via the physics loader), speed limit and road-ahead. | none | Extraction exists (`g2m:layer/osm_roads.h`). Fetch+cache is game-side. |
| OSM / visual / buildings accessors | `source_osm_tile` cpp:589, `road_visual_tile` cpp:583, `buildings_tile` cpp:597 | Fetch the source OSM tile; build the road ribbon / buildings for an L2 key (see 1.5, 1.6). | cfg (building heights) | Source fetch only |
| Static view | `build_static_view_from_lookup` h:70, `select_view_keys` h:81 / cpp:148, `build_render_chunks` h:95 / cpp:153 (std::thread fan-out, cancel flag), `build_static_view` cpp:645 (up to 8 threads), `view_source` cpp:653, `TerrainViewSource` h:105 | LOD chunk selection and chunk mesh build for the whole view. **The 20 km render view derives thousands of height/elev tiles with the main thread blocked (about 15 min in debug after an id change).** | cfg (`lod.max_distance_m` 6000 default, configs use 20000) | `g2m:mesh/lod_select.h` (`select_chunks`, `LodParams`), `g2m:mesh/terrain_chunk.h` (`gather_window`, `build_chunk`, `TerrainChunkMesh`, `ClassWindow`). No scheduler. |
| `chunk_vertex_colors`, `build_one_chunk` | cpp:68, cpp:116 | Hypsometric ramp plus paved/unpaved colours keyed by surface name. One chunk build. | cfg (surface names) | No (geo2map has ClassWindow, no colouring) |

### 1.2 Road profiles and structures

| Piece | Anchors | What it does | Deps | geo2map today |
|---|---|---|---|---|
| `complete_road_profiles` | road_structures.cpp:131-234 | (a) Anchors from accepted bridges, 136-143. (b) **Tunnel floors**, 146-165: fit_stretch, ceil(len/5) samples, DEM floor fit at max_grade 0.30 / min radius 20. (c) Bridges, 166-196: abutment walk plus anchors, fit at max_grade max(.12, class) / min radius min(250, class), secant fallback. (d) Ground loop, 197-233: 40 m spacing, 32 m ramp, +-0.10 clamp. **Line 201 re-fits an already-profiled entry whose end node is in `anchors` = the regression.** | none (pure g2m types) | `g2m:layer/road_completion.cpp`: strict anchors (184-196), bridges, ground completion (236-288), retry (175). **Tunnels are skipped (`road_completion.cpp:203`, `tunnel_skipped`).** |
| `abutment()` walk | road_structures.cpp:28-60 | 32 m ground budget, 100 iterations, dot > 0.25, walks through bridges with **no total cap**. | none | `Walk` (road_completion.cpp:61-125), bounded by `abutment_path_m` 60 (max `kRoadCompletionMaxAbutmentPathM` 61.5, DEM margin 65 m, 128 m halo). |
| `RoadDeck` + `deck()` mesh builder | road_structures.h; road_structures.cpp:61-122 | Deck mesh: 5 vertices across, 0.5 m slab, fascia sides, max 1000 m. | none | No |
| `road_deck_height` | road_structures.cpp:124 | Height on a deck at XY. | none | No (carve has a deck clamp, `g2m:layer/terrain_carve.h`) |
| `build_road_decks` | road_structures.cpp:235-276 | Decks for bridge/tunnel profiles, plus **inferred tunnel roof decks** where a lower tunnel way crosses an upper road of rank >= 5 on a higher layer (extent max(20, (w/2+4)/sine)). | none | No. G-6 (census of untagged underpass dips) is related. |
| `DeckInstaller` | deck_installer.h/.cpp (200 lines) | Builds `ps::MeshShape` / `ShapeHandle` off the sim thread and installs bodies in DeckKey order. | **ps** | Stays in the game (glue). |

### 1.3 Ground carve, smoothing and DEM sampling

| Piece | Anchors | What it does | Deps | geo2map today |
|---|---|---|---|---|
| `RoadSurfacePatch` ctor + `apply()` | road_surface.h; road_surface.cpp (144 lines) | Decks and deck cuts; 1 m segments of accepted non-bridge PavedRoad/tunnel profiles in 16 m bins. Nearest-road blend: crown, verge drop, smoothstep, 0.35 transition blend across same-layer roads, deck clearance cut (top - 0.55*smooth(edge/4)). Uses libm (`std::sqrt`, `std::hypot`, `std::llround`). | cfg (verge drop) | `g2m:layer/road_surface.h` (`road_surface_height_m`, crown_round 0.5, verge 0.06) plus `terrain_carve.h` (`CarveTerrain`, deck clamp, D42 junction fade) |
| Terrain smoothing | terrain_smoothing.h/.cpp (61 lines) | Separable Gaussian with `std::exp`; radius 2 m, strength .35, 1 pass, halo via lookup. | cfg (`terrain_smoothing`) | Integer-kernel port in `CarveTerrain` ([138,621,1024,621,138], strength 35) |
| `RoadDemSampler` usage | world_terrain.cpp:369-449 | Builds the sampler for completion. | none | Exists (`g2m:layer/road_dem_sampler.h`) |
| Height elevation derivation | (server-side) | `g2m.elev.base` L0 -> `g2m.terrain.height` L0-L6 | none | Exists (server derivers) |

### 1.4 Physics path (start-up gate, tile set, surfaces, adapter)

| Piece | Anchors | What it does | Deps | geo2map today |
|---|---|---|---|---|
| `HeightTileSharedFetch : g2m::phys::IHeightTileFetch` | terrain_mode.h/.cpp (59 lines) | Passes the height fetch through `WorldTerrain`. In road mode it fetches L2 = `key.parent().parent()` segments; Failed becomes null roads. | none | Interface + `TransportHeightTileFetch` exist (`g2m:phys/height_tile_loader.h:80,104`) |
| `TerrainModeConfig`, `make_terrain_mode` | terrain_mode.h | Wires frame, fetch, spawn, physics config, prime ticks. | cfg, ps (via bridge) | Partly |
| `Session::Terrain`, `setup_terrain` | session.cpp:65-108, 310-422 | Builds a `RoadValueLut` from the SurfaceTable by land class, `make_terrain_config` with `max_resident_tiles *= 2`, `set_terrain_source`. **Start-up gate:** loop with `retry_failed` + `startup_timeout_s`, then `sync_road_decks()`, priming ticks ceil(square/fills)+1, resident/starved/fill-miss checks. | **ps**, cfg | `g2m:phys/` (`HeightTileLoader`, `ResidentHeightSet`, `PhysicsTerrainStreamer` gate, `PhysicsGrid`), bridge `G2mTerrainSource`, `make_terrain_config`, `RoadSurfaceConfig` |
| Interest points | session.cpp:563-603 | Player point 0, parked car/walker point 1, followed point. | ps | No (game policy, stays) |
| `gate_check`, step loop | session.cpp:659-690, 751 | `gate_check() && sync_road_decks(1)` | ps | Streamer gate exists |
| `sync_road_decks` | session.cpp:833-873 | Required radius = physics radius + 255 + prefetch margin. Surface id from road_surfaces names. DeckInstaller update / wait. **Decks only exist after `road_surface_patch` ran for that L2, so the gate implicitly includes roads.geom derivation + completion for every gate tile.** | ps, cfg | No |
| ITerrainSource adapter | g2m:`bridges/physics_sim/g2m_terrain_source.h/.cpp` | `G2mTerrainSource` (uniform/road mode), `fill_physics_heights`, `fill_physics_surfaces`, prefetch snapshot. | ps (opt-in target) | **Already in geo2map** (per G2M-001), pinned via `bridges/physics_sim/physics_sim.pin` |
| Surface classification | road_classes.h/.cpp (92 lines): `RoadSurfaceMap`, `ClassLookup`, `road_raster_params_for_level`, `road_class_lattice`, `src_osm_tiles_for_chunk`, `rasterize_chunk_road_classes`; surface_kinds.h: `surface_name_for_kind` (Water -> low_mu placeholder) | Maps road/land classes to game surface names. Chunk rasterisation forwards to `g2m::rasterize_road_blocks` and `raster_params_for_render_level`. | cfg (surface names) | Rasterisation, `RoadValueLut`, `SurfaceKind`/`LandClass` (`classifier.h`) exist. **G-3 (full SurfaceKind 0-11 / LandClass 0-13 -> SurfaceId tables) is open.** |
| Height sampling | route_check.cpp:444 `sample_l0_height`; callers world_terrain (road_visual_tile, buildings_tile), godot_ext rg_simulation.cpp:634-637, rg_simulation_shell.cpp:169 | Bilinear over the cell-centre lattice. **This differs from Jolt's diagonal triangulation** (G-4). | none | No public helper. The bridge test `test_render_vs_jolt` compares the two. |

### 1.5 Render-side map logic

| Piece | Anchors | What it does | Deps | geo2map today |
|---|---|---|---|---|
| `TerrainViewStreamer` | terrain_view_streamer.h/.cpp (205 lines) | Background worker. Re-selects LOD chunks when the focus moves more than 128 m, diffs, builds only added chunks, commits adds before removes. 2 build threads. | cfg | Selection/build exist, streamer does not |
| `RenderChunk` | terrain_render.h | key, mesh, origin_session, rgba | none | `TerrainChunkMesh` (no colours) |
| `build_road_visual` | road_visual.h/.cpp (99 lines) | Ribbon mesh at 1 m stations draped on ground or deck height. Lane/marking flags from tags (lanes, oneway, turn:lanes). Junction fade 3 + 8 m. Calls `build_road_decks` again. | none | No (`road_furniture.h` / `road_junction.h` carry the data) |
| `extract_buildings` | buildings.h/.cpp | OSM -> Building records (min/fallback/storey height, ground fn). | cfg (heights) | No |
| `BuildingFootprints` / `BuildingObstacles`, `shot_obstacles.h` | building_footprints.h/.cpp | Camera obstacles | none | Stays in the game (camera feature) |
| `road_stream.gd` | game/scripts/road_stream.gd (119 lines) | Worker thread schedules L2 road visual tiles within distance_m (512-6000, default 2500) + 725 m; evicts beyond + 2048 m. **Tile-selection policy in GDScript.** | Godot | No |
| `building_stream.gd` | game/scripts/building_stream.gd (230 lines) | Same pattern for buildings | Godot | No |
| Godot upload | godot_ext rg_terrain_view.cpp/.h: `upload_one_chunk`, `get_road_visual_tile` 153, buildings dictionary 166, `sync_road_decks` 364 (deck colours 0x4c4c52ff / 0x5c4729ff, 2 per frame); shaders road/terrain.gdshader | RenderingServer upload | Godot | Stays (glue) |

### 1.6 Map logic that is not terrain (scope note)

These stay in the game for now and are listed only so nothing is overlooked:
- `road_ahead` (camera).
- `road_network` / `plan_path` (S3 routing). geo2map G4 routing is planned.
- npc traffic / truck routes (session.cpp 1277, 1532).
- Speed limit (session.cpp 1176-1225). This already uses `g2m::match_road_speed_limit`.

All of them consume the facade. They only change the type they call.

---

## 2. Boundary and API

### 2.1 Rule

Everything that turns map data into heights, road profiles, structures, surfaces, LOD selection, meshes or tile-selection policy goes to geo2map. It must be engine-neutral, carry no ps and no Godot types, and stay deterministic where it feeds physics.

The game keeps:
1. **physics_sim glue** in `session.cpp` and `deck_installer.cpp`:
   - SurfaceTable name -> SurfaceId mapping, fed into geo2map's class tables;
   - TerrainConfig sizing;
   - interest points;
   - gate timeout and priming policy;
   - `DeckInstaller` (ps shapes).
2. **Godot upload**: rg_terrain_view.cpp, shaders, the GDScript that only uploads.
3. **Game config**: world_config.json values passed into geo2map option structs.

**Adapter location.** The `ps::terrain::ITerrainSource` adapter (`G2mTerrainSource`) already lives in geo2map's opt-in `g2m_ps_bridge` target. G2M-001 placed it there, and geo2map's core still does not depend on physics_sim. This plan keeps it there. The brief's wording "game keeps the ITerrainSource adapter" is satisfied by the game owning only its wiring (session.cpp). If the owner wants the adapter physically inside racing_game, see Q5. The cost is that a farming sim would have to copy it.

### 2.2 New geo2map module `g2m::world` (library target `g2m_world`)

The new target links `g2m_client`, `g2m_server`, `g2m_layer`, `g2m_mesh` and `g2m_phys`. It replaces `rg::WorldTerrain`, the view streamer and the stream scripts' policy.

```cpp
namespace g2m::world {

// ---- configuration (all game values are plain fields; no JSON parsing of game files) ----
struct StoreConfig { std::filesystem::path source_store, derived_store; std::string source_scope; };
struct LayerOptIn  { bool roads = true, geometry = true, carved = false, classes = false; };
struct TerrainServiceConfig {
    StoreConfig stores;
    LayerOptIn layers;                 // feeds builtin_local_release_params -> release id
    int derive_workers = 0;            // 0 = max(1, hardware_concurrency - 2)
    std::size_t cache_bytes = 1u << 30;
};

enum class Priority : std::uint8_t { Gate = 0, Physics = 1, Prefetch = 2, Render = 3, Warm = 4 };
enum class TileState : std::uint8_t { Pending, Ready, Absent, Failed, Cancelled };

template <class T> class Ticket {          // cheap to copy; shared state
public:
    TileState state() const noexcept;      // never blocks
    std::shared_ptr<const T> value() const noexcept;   // null unless Ready
    const std::string& message() const noexcept;       // Absent/Failed reason
    bool wait_for(std::chrono::milliseconds) const;    // tools/tests only; never on a render/sim thread
    void cancel() noexcept;                // drops this caller's interest; job stops if no one else waits
};

// Engine-neutral payloads (float32 positions relative to a chunk/tile origin in SessionFrame metres)
struct HeightTile;          // decoded g2m.terrain.height or g2m.terrain.carved, offset applied
struct RoadGeomTile;        // completed g2m.roads.geom (served profiles; no client re-fit)
struct RoadSegmentsTile;    // osm_roads segments for grip/speed limit
struct StructureSet;        // per L2: StructureMesh{kind Deck|Slab|Fascia|TunnelFloor|Roof, way_id, land_class,
                            // bounds, positions, normals, indices, surface_kind} + deck height query
struct RoadRibbonTile;      // per L2: ribbon mesh + lane/marking flags + junction fade
struct BuildingSet;         // per L2: footprints, heights (BuildingOptions)
struct RenderChunk;         // key, TerrainChunkMesh, per-vertex SurfaceKind/LandClass + optional rgba

class TerrainService {
public:
    static std::unique_ptr<TerrainService> open(const TerrainServiceConfig&);  // throws on bad stores
    std::string release_id_hex() const;
    const SessionFrame& frame() const;

    // async, thread-safe, any thread; never derive on the calling thread
    Ticket<HeightTile>       height(TileKey, Priority);
    Ticket<RoadGeomTile>     road_geom(TileKey l2, Priority);
    Ticket<RoadSegmentsTile> road_segments(TileKey, Priority);
    Ticket<OsmTile>          source_osm(TileKey, Priority);
    Ticket<StructureSet>     structures(TileKey l2, Priority);
    Ticket<RoadRibbonTile>   road_ribbons(TileKey l2, Priority, const RibbonOptions&);
    Ticket<BuildingSet>      buildings(TileKey l2, Priority, const BuildingOptions&);

    // non-blocking peeks: return a cached value or null, never start work
    std::shared_ptr<const HeightTile>   peek_height(TileKey) const noexcept;
    std::shared_ptr<const RoadGeomTile> peek_road_geom(TileKey) const noexcept;
    std::shared_ptr<const StructureSet> peek_structures(TileKey) const noexcept;

    // sampling (pure, on resident data): physics-exact (Jolt diagonal, G-4) and bilinear
    std::optional<double> surface_height_exact(double x, double y) const noexcept;
    std::optional<double> surface_height_bilinear(double x, double y) const noexcept;

    // warm-up after a release-id change (whole region or a key set), progress + cancel
    WarmJob warm(const WarmRequest&);      // runs at Priority::Warm on the same pool

    // physics: IHeightTileFetch over this service (replaces rg::HeightTileSharedFetch)
    std::shared_ptr<phys::IHeightTileFetch> physics_fetch(bool with_road_segments);
};

// LOD view streamer (moved from rg::TerrainViewStreamer + the select/build free functions)
struct ViewStreamerConfig { mesh::LodParams lod; double reselect_m = 128; int build_workers = 2; };
class ViewStreamer {
public:
    ViewStreamer(TerrainService&, ViewStreamerConfig);
    void set_focus(double x, double y) noexcept;          // any thread, non-blocking
    ViewDiff take_diff();                                 // main thread: adds before removes, ready chunks only
};

// Generic ring streamer for per-L2 products (moved policy of road_stream.gd / building_stream.gd)
template <class Product> class RingStreamer;   // load radius, evict radius, Priority::Render
}

namespace g2m::classes {   // G-3: SurfaceKind 0-11 / LandClass 0-13 -> consumer id
struct SurfaceIdTable { std::array<std::uint16_t, 12> by_surface_kind; std::array<std::uint16_t, 14> by_land_class; };
}
```

### 2.3 Threading model

- **One worker pool per service**, size `derive_workers`. Work is a priority queue (Gate before Physics before Prefetch before Render before Warm). Different keys derive in parallel.
- **Per-key in-flight dedup.** Each (layer, key) has at most one running job, and later requests attach to its shared state.
  - This replaces `geometry_mutex_` and the server's current duplicate derivation, where two threads may derive the same tile and both write identical bytes.
  - It must also be added inside `server::Server`'s resolver (per (layer, key), "join a running job, never wait on a queued one"). Recursive dependency resolution inside one worker then cannot deadlock: a waiter only waits on a thread that is executing, and the derivation DAG is acyclic.
- **No main-thread blocking.** Render and sim threads only call tickets' `state()`/`value()` and `peek_*`.
  - The start-up gate polls with the game's timeout and never calls `wait_for` on the sim thread.
  - The view streamer and ring streamers do their own polling on their worker.
- **Deterministic outputs.** Job order never influences bytes; that is already the server's contract. Structure and ribbon meshes are pure functions of their inputs and use det math where they feed physics (decks become ps meshes).
- **Cancellation**: a ticket's job is dropped once every holder cancelled and it has not started. A running derive finishes and its result is cached.

### 2.4 Release-id rules in this design

- Moving client-side code (meshes, sampling, scheduling, streamers) never changes the release id.
- The id changes only when a deriver's layer version or config text changes, or when the game changes its layer opt-ins. That happens in phases 4, 5 and 7 below.

---

## 3. Overlaps: which version survives

Gap file numbers are corridor first, Hessen in brackets.

| # | Overlap | Game | geo2map | Survives | Action |
|---|---|---|---|---|---|
| O1 | Profile completion as a whole | `complete_road_profiles` (road_structures.cpp:131-234) + retry (world_terrain.cpp:369-449) | `road_completion.cpp` | **geo2map** | Port the three gaps below; then delete the game function, call site and retry. |
| O2 | **Tunnel floors** (GAP 1) | Fitted at 146-165 | Skipped (`road_completion.cpp:203`) | **The game algorithm, ported into geo2map** | 1 368 entries / 841 ways / 31.2 km (914 / 604) unprofiled on the served side; 33 drivable (rank >= 5) tunnels; 102 tiles. Port into step 2 in place of `tunnel_skipped`: fit_stretch with curvature exceptions; skip < 0.5 m; count max(1, ceil(len/5)), skip > 2047; DEM at every station, decline if any missing; VerticalFitLimits{0.30, 20 m, 16384, clamped secant end grades}; new provenance `TunnelFloor = 4`; options `tunnel_spacing_m 5`, `tunnel_max_grade 0.30`, `tunnel_min_radius_m 20`. **Long tunnels:** the DEM-as-floor rule is wrong for deep tunnels (539 / 499 / 498 m examples; 36 over 100 m, 9 over 300 m). Recommended: above 150 m fit a chord between the DEM-supported portal anchors, clamped to max_grade (Q1). Check that encoding-7 decoders accept provenance 4; otherwise bump to encoding 8. |
| O3 | **Bridge anchor reuse** (GAP 2a) | Anchors from accepted bridges (136-143) | Strict anchors only at bridging nodes | **Ported into geo2map** | Covers 24/63 (14/35) of the 77 (40) missing bridge entries. Add accepted-bridge endpoint heights to `anchor_of` (road_completion.cpp:189-196) ahead of the walk. |
| O4 | **Abutment walk length** (GAP 2b) | Unbounded through bridges, 32 m ground | Bounded 60 m (max 61.5 m, DEM margin 65 m, halo 128 m) | **geo2map's bounded walk, with a larger bound** | An unbounded walk breaks the dependency bound that makes tiles derivable independently, so it is not ported. Need over 60 m: 39/63; over 200 m: 15; over 500 m: 4; max 1 136 m (Hessen 552 m). Recommended: `abutment_path_m` 200 (48/63 covered), DEM margin about 205 m, context halo at least 256 m. Measure the cost with G-7. The rest keeps the secant fallback, flagged relaxed (Q2 of G2M-017). Chain fitting is owner question Q2. |
| O5 | **20 m retry** | Game retry after completion | geo2map retry (spacing 20, 16384, road_completion.cpp:175) | **geo2map** | The same rule already exists. GAP 3's retry case (way 285188051, utm32n/2/464/5394) and the two ground ways (25589596 31.9 m, 320190187 6.4 m) are expected to be input differences that GAP 2a fixes. Investigate with an input diff in the probe; do not add a second retry. |
| O6 | **Ground re-fit of served profiles** | road_structures.cpp:197-233, the `anchors` re-fit at :201 | Served Strict/GroundCompleted (D43 ease) | **geo2map; the game behaviour is a regression and is NOT ported** | 472 entries / 350 ways (399 / 302). max dz p99 2.2 m, max 2.8-4.24 m. GroundCompleted median DEM RMS 0.168 -> 0.330 m; worst way 4000565: 0.043 m served vs 1.611 m game. Delete the re-fit first (phase 0). |
| O7 | Smoothing | Gaussian with `std::exp` (terrain_smoothing.cpp) | Integer kernel inside `CarveTerrain` | **geo2map** | Deleted from the game at the carve switch. |
| O8 | Road carve / blend / deck cut | `RoadSurfacePatch` (road_surface.cpp), libm | `road_surface.h` + `terrain_carve.h` (D42 fade, deck clamp) | **geo2map** (`g2m.terrain.carved`) | Owner decision: no fallback. The game's `road_verge_drop_m` becomes a carve config value (Q3 covers the default). Note: game L1-L6 today are smoothed uncarved tiles; carved L1-L6 is a carved pyramid. That is a visual change, accepted under "work-in-progress states are acceptable". |
| O9 | Structure meshes and deck height | `deck()`, `road_deck_height`, `build_road_decks` | none | **Game code, moved into geo2map** | Pure move (phase 3). |
| O10 | Tunnel roof inference | `build_road_decks` inferred roofs | none (G-6 census open) | **Moved as-is** client-side into `StructureSet` | G-6 decides later whether it becomes a served layer or is dropped (Q4). |
| O11 | Road ribbon junction fade | 3 + 8 m in road_visual.cpp | D42 carve fade | **Both kept for now** (ribbon vs ground) | Align the ribbon fade to D42's numbers in the move commit only if a probe shows no visual regression. Otherwise keep the game numbers. |
| O12 | Road class rasterisation | Wrappers in road_classes.cpp | `rasterize_road_blocks` etc. | **geo2map** | Delete the wrappers (phase 1). |
| O13 | Height sampling | Bilinear `sample_l0_height` | none public; Jolt-diagonal in the bridge test | **geo2map both** | Physics-consistent callers (camera ground, building base, ribbon drape) switch to the exact sampler (G-4). Bilinear stays for render-only users. |
| O14 | Surface name mapping | `surface_kinds.h`, `RoadSurfaceMap` | `RoadValueLut`, classifier enums | **geo2map tables (G-3)** | Surface names stay game config; the table type is geo2map's. |

---

## 4. Phases

Every phase works the same way:
1. A geo2map commit, with tests, on its own CI topic. `docs/pipeline/README.md` is updated in the same commit when the pipeline changes.
2. A racing_game commit that bumps the geo2map pin, switches callers, deletes the game copy, and is test-driven on the corridor.

The game must stay playable after each step.

Rules for the roadcmp probe:
- The probe is `racing_game/out/roadcmp`: probe_src + instrumentation.patch, `RG_ROADCMP=1`.
- It must show **zero game-only differences** before any game profile code is deleted.
- It runs on both corridor (155 tiles) and Hessen (134 tiles).

### Phase 0: drop the game's re-fit regression (game only, S)

- **Change:** road_structures.cpp:197-233. Skip every entry that already carries a served profile, i.e. remove the `anchors` re-fit at :201. Keep tunnel floors, bridges and genuine ground completion.
- **Verification:**
  - The probe's "game re-fits served profiles" class drops from 472 (399) to 0.
  - The GAP 1-3 counts are unchanged.
  - Corridor test drive over the worst way 4000565.
- **Release id:** unchanged. Game-side ground heights change in the warm cache, but the derived store is not affected.

### Phase 1: client helpers and sampling into geo2map (no id change, M)

- **geo2map, new `g2m_world` target:**
  - container decode with height_offset (port of cpp:511);
  - `fetch_*_cached`;
  - road-segment fetch/decode (cpp:672-770);
  - `surface_height_exact` (G-4, Jolt diagonal) and bilinear sampler;
  - `classes::SurfaceIdTable` (G-3).
- **Tests:** decode round trips; exact sampler equal to `test_render_vs_jolt`'s reference; bilinear equal to the game's `sample_l0_height` on recorded tiles.
- **Game:**
  - WorldTerrain calls these.
  - Delete `decode_height_tile_container`, `fetch_*_cached`, `decode_road_segments_response`, route_check.cpp:444, and the road_classes.cpp wrappers.
  - Switch rg_simulation.cpp:634 and rg_simulation_shell.cpp:169 to the exact sampler.
- **Verification:** game unit tests; camera and building bases on the corridor.
- **Release id:** unchanged.

### Phase 2: TerrainService with parallel, off-thread derivation (no id change, L)

This fixes the 2026-10-12 queue items (b) and (c).

- **geo2map:**
  - `TerrainService` (open, tickets, priorities, peeks, `warm`, `physics_fetch`);
  - per-key in-flight dedup in `server::Server`'s resolver.
- **Game:**
  - `WorldTerrain` becomes a thin owner of a `TerrainService`.
  - The game's still-existing completion + `RoadSurfacePatch` runs as a per-L2 job: one shared future per key in place of `geometry_mutex_`.
  - `HeightTileSharedFetch` is replaced by `physics_fetch`.
  - `build_static_view` and the gate use tickets.
  - A release-build warm pass runs through `warm()` after a pin bump (queue item a).
- **Tests:**
  - dedup (N threads, one derivation; count via a test deriver);
  - no deadlock under a 1-worker pool with recursive dependencies;
  - byte equality vs serial derivation;
  - cancellation;
  - linux-tsan leg.
- **Verification:**
  - Cold gate time on the corridor, release and debug, against today's about 20 s / over 30 s. The target is gate time limited by parallel roads.geom derivation; record the measured number.
  - The main thread never blocks: frame-time log during a cold start.
  - The probe still shows identical game profiles.
- **Release id:** unchanged.

### Phase 3: meshes, structures and streamers into geo2map (no id change, M+M)

Split into two game PRs if needed.

- **geo2map:**
  - 3a: `StructureSet` (port of `deck()`, `road_deck_height`, `build_road_decks` incl. roof inference) and `RoadRibbonTile` (port of road_visual.cpp), built from a `RoadGeomTile` handed in by the caller. In phase 3 that is the game's completed tile; from phase 7 it is served.
  - 3b: `ViewStreamer` (port of terrain_view_streamer.cpp + `select_view_keys` / `build_render_chunks` / `build_one_chunk`), with vertex class ids and an optional colour hook; `RingStreamer` (policy of road_stream.gd / building_stream.gd: load radius + 725 m, evict + 2048 m); `BuildingSet` (port of buildings.cpp).
  - Libm in ported mesh code is replaced by det math where the output feeds physics (deck meshes).
- **Tests:** mesh byte pins against meshes recorded from the game before the move; streamer diff order (adds before removes); ring eviction.
- **Game:**
  - Delete road_structures.cpp's mesh half, road_visual.cpp, buildings.cpp, terrain_view_streamer.cpp, and the world_terrain.cpp view functions (h:70-105, cpp:68-211).
  - road_stream.gd and building_stream.gd shrink to upload loops over `RingStreamer` diffs.
  - `chunk_vertex_colors` becomes a game-side colour table keyed by class id (presentation).
- **Verification:**
  - Deck ps meshes are bit-identical; `DeckInstaller` hash pins unchanged.
  - Visual A/B screenshots on the corridor.
- **Release id:** unchanged.

### Phase 4: completion parity, tunnels + bridge anchors (id bump #1, M)

- **geo2map:**
  - O2 tunnel floors with the long-tunnel rule (Q1);
  - O3 anchor reuse;
  - layer version / config text of `g2m.roads.geom`;
  - provenance `TunnelFloor`;
  - docs/pipeline/README.md and a D-entry.
- **Tests:**
  - per-rule unit tests;
  - the 3 long tunnels;
  - a determinism pin at 1 vs N workers;
  - G2M golden tiles regenerated.
- **Game:** pin bump only. The game's completion still runs, but it now finds tunnels/bridges already profiled and leaves them alone (phase 0 rule).
- **Verification:**
  - Probe GAP 1 count 0.
  - GAP 2 count drops by 24 (14).
  - The game-only list contains only GAP 2b and GAP 3 cases.
- **Release id:** **bump.** The corridor and Hessen derived stores go cold for roads.geom and its dependents. Warm with `warm()` (phase 2) or the geo2map bake tooling before the test drive.

### Phase 5: abutment walk length + margin (id bump #2, M)

- **geo2map:**
  - O4: `abutment_path_m` to the chosen bound (Q2; 200 m recommended);
  - raise `kRoadCompletionMaxAbutmentPathM`, `kRoadCompletionDemMarginMm` and the context halo accordingly;
  - discovery stages rechecked (D35);
  - G-7 cost measurement.
  - GAP 3 input diff: fix if a deterministic input difference remains, otherwise document.
- **Tests:** dependency-bound test (every DEM read stays inside the margin); walk lengths at 60/200/500 m fixtures; release-cost benchmark.
- **Game:**
  - Pin bump.
  - When the probe shows zero game-only differences, **delete `complete_road_profiles`, the 20 m retry, `RoadDemSampler`/dependency use and `published_decks_` filling from game profiles.**
  - `StructureSet`/ribbons now consume the served `RoadGeomTile` directly.
- **Verification:**
  - Probe: zero game-only entries on both regions, which is the deletion gate.
  - Corridor and Hessen drive over the listed bridges (63 / 35 unique).
- **Release id:** **bump.** Phases 4 and 5 can be one bump if the owner prefers fewer cold stores (Q3).

### Phase 6: served carve readiness (geo2map only, no game id change yet, M)

This covers G-1, G-2, G-5 and slice 10.

- **geo2map:**
  - G-1: carved L0 treats a rejected roads.geom input (validation error, e.g. utm32n/2/452/5428 "reference endpoints") like an absent one: no roads, flagged. Today a non-NotFound input fails the dependent tile (server.cpp resolver), which would freeze the game's gate.
  - G-2: class-mode FetchResult keeps RoadSegment metadata.
  - G-5: carved/class deriver + prefetch flags in `G2mTerrainSource`.
  - Slice 10: `g2m.terrain.class` deriver.
  - G-6 census.
- **Tests:** a carved tile over a rejected roads.geom tile is Found; class layer goldens; bridge prefetch with carved tiles.
- **Release id:** unchanged for the game. It does not opt in yet, and opt-ins are part of its id.

### Phase 7: carve switch (id bump #3, L)

- **Game:**
  - `LayerOptIn{carved = true, classes = true}`.
  - Physics `G2mTerrainSource` reads carved heights and class surfaces through `SurfaceIdTable`.
  - Render reads carved L0-L6.
  - **Delete:** `RoadSurfacePatch` (road_surface.*), terrain_smoothing.*, the carve branch of `fetch_and_decode`, `road_verge_drop_m`/`terrain_smoothing` config keys (moved into the release config if kept), `geometry_mutex_` leftovers, surface_kinds.h.
- **geo2map:** release config carries the game's verge/smoothing values if they differ from defaults (Q3).
- **Verification:**
  - geo2map `test_render_vs_jolt` with carved tiles.
  - Game: gate on corridor/Hessen cold and warm.
  - Drive the 33 drivable tunnels and the 63 bridges.
  - Fall detector silent on the autopilot route.
  - Grip equals the render class at sample points (G2M-010).
- **Release id:** **bump** (opt-in change). This is the largest cold-store cost: carved L0-L6 for the whole 20 km view. Warm before merge.

### Phase 8: remove the facade remnant (no id change, S)

- `rg::WorldTerrain` disappears. Session and godot_ext hold `g2m::world::TerrainService` directly.
- road_ahead, road_network, npc traffic and speed limit read `RoadSegmentsTile` / `RoadGeomTile` tickets.
- **Verification:** game build + test drive.

### Release-id cost

| Plan variant | Bumps |
|---|---|
| As written | **3** (phase 4, phase 5, phase 7) |
| Phases 4+5 batched | **2** |
| A tunnel or anchor rework after review | +1 each (likely) |

Cold-store consequence per bump:
- The corridor gate re-derives about 20 s in release (less after phase 2).
- The full view re-derives on demand off-thread (after phase 2), or ahead of time with `warm()`.

### Size summary

| Phase | Size |
|---|---|
| 0 | S |
| 1 | M |
| 2 | L |
| 3 | 2 x M |
| 4 | M |
| 5 | M |
| 6 | M |
| 7 | L |
| 8 | S |

Phases 1-3 and 6 are independent of each other once phase 2's ticket type exists. Phase 3 can start in parallel with phase 4.

---

## 5. Owner questions

1. **Long tunnels.** For tunnels over about 150 m, follow the DEM (wrong for deep tunnels: 36 over 100 m, 9 over 300 m) or fit a chord between the portals clamped to max grade? The plan recommends the chord.
2. **Bridge reach.** Should the abutment walk be bounded at 200 m (48/63 corridor bridges completed, bigger dependency margin and halo, more cold-derive cost) or 500 m (59/63, much larger halo)? Should a chain of bridge ways be fitted as one structure? The plan recommends 200 m and no chain fit; the rest stay relaxed-flagged.
3. **Release-id batching.** Use three small bumps (phases 4, 5, 7) or two (phases 4+5 together)? Each bump cools the corridor and Hessen stores. Related: should the game's `road_verge_drop_m` and smoothing values survive as release config, or does the game accept geo2map's defaults (verge 0.06 m, integer kernel)?
4. **Tunnel roofs.** The game infers roof decks where a tunnel passes under a higher road. Should geo2map serve them (after the G-6 census), keep them as a client-side product, or drop them?
5. **Adapter location.** Keep the `ITerrainSource` adapter in geo2map's opt-in physics_sim bridge, as G2M-001 already decided and this plan assumes? The alternative is to move it physically into racing_game, as the brief's wording suggests.

## Owner answers (2026-10-12)
- Q1 long tunnels: NOT decided. Owner will test tricky Frankfurt cases later, incl. a tunnel with significant turns and slopes. Phase 4 must handle curved, sloped tunnels; keep the straight-fit threshold configurable and do not lock in either option until those tests run.
- Q2 bridge walk: 200 m.
- Q3 release-id bumps: three (phases 4 and 5 stay separate).
- Q4 tunnel roofs: geo2map serves them.
- Q5 ITerrainSource adapter: stays in geo2map's opt-in physics_sim bridge (G2M-001, already there); no question to the owner needed.
- Phase 0 (game regression) started 2026-10-12.
