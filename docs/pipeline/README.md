# geo2map pipeline: from raw geodata to a finished map

This is the one entry point. Start here if you have never seen the code.
It shows the whole path with diagrams, then links to the detail pages.

- Status date: 2026-10-05. Verified against the code of `geo2map_engine` master at that date.
- Scope: `geo2map_engine` (this repo) and how its two consumers use it: `racing_game` (render + game) and `physics_sim` (terrain tiles for the physics heightfield). Both consumers are separate repos and are only described here.
- Rule: any change that adds or changes a pipeline stage, layer, encoding, data source or consumer updates this folder in the same commit (also in `CLAUDE.md`).
- Reading the diagrams: **green** = implemented and in use. **amber dashed** = downloaded or designed, not used yet. **grey dashed** = planned. Mermaid renders on GitHub and in most editors.

Contents:

1. [Overview diagram](#1-overview)
2. [Stages, one by one](#2-stages-one-by-one)
3. [Layer dependencies](#3-layer-dependencies)
4. [Runtime: what happens when a tile is requested](#4-runtime-tile-request)
5. [Data sources](#5-data-sources)
6. [Roadmap](#6-roadmap)
7. [How to rebuild and run](#7-how-to-rebuild-and-run)
8. [Glossary](#8-glossary)
9. [Open questions and contradictions](#9-open-questions-and-contradictions)
10. [Index of all other documents](#10-index-of-all-documents)

---

## 1. Overview

The idea in one paragraph. Raw files (a height model, an OSM extract, a land cover raster) are **imported once** into a store of small tiles. Everything else is **derived** from those tiles by pure functions: same input, same bytes, on every machine. A server (in-process today) answers "give me layer X, tile Y" by looking in the cache, deriving the tile if missing, or asking an upstream. The game then turns tiles into render meshes and physics heightfields on its own side.

```mermaid
%%{init: {"flowchart": {"wrappingWidth": 440, "useMaxWidth": false}}}%%
flowchart TB
  SRC["<b>1. Raw sources</b> - files on S:<br/>DGM1 1 m heights - HVBG<br/>OSM Hessen extract - Geofabrik<br/>ESA WorldCover 10 m<br/><i>downloaded, not used yet:</i> GLO-30, LoD2, DOM1, DOP20, Basis-DLM, tree data"]
  IMP["<b>2. Import</b> - g2m_tiler import<br/>resolve region to level-2 tiles<br/>import-dem, import-osm, import-landcover"]
  RAW["<b>3. Raw layers</b> - level 2, 1024 m, source store<br/>g2m.src.elev.dgm1 / g2m.src.osm / g2m.src.landcover"]
  BAKE["<b>4. Derived layers</b> - pure derivers, g2m_tiler bake<br/>g2m.elev.base L0 / g2m.terrain.height L0-L6<br/>g2m.roads.graph L2 / g2m.roads.geom L2<br/><i>planned:</i> terrain.class, buildings, trees, markings"]
  SRV["<b>5. Server and cache</b> - in-process today<br/>g2m::Server: cache, derive on miss, upstream<br/>tiles.sqlite3 + blob files, release id<br/><i>planned:</i> HTTP server, packs, hosted CDN"]
  GAME["<b>6a. racing_game</b> WorldTerrain<br/>render chunks, road raster and grip,<br/>road carving at L0, NPC traffic"]
  PHYS["<b>6b. physics_sim</b><br/>g2m_ps_bridge G2mTerrainSource<br/>TileManager Jolt heightfield pool"]
  SRC --> IMP --> RAW --> BAKE --> SRV --> GAME --> PHYS
  RAW -. "src.osm and src.landcover are also served raw" .-> SRV

  classDef done fill:#d8f0d8,stroke:#2e7d32,color:#000,text-align:left;
  class SRC,IMP,RAW,BAKE,SRV,GAME,PHYS done;
```

What the diagram hides, in short:

- **Terrain heights are not carved on the server.** `g2m.terrain.height` is a plain DGM1 pyramid. Road carving into the height tile happens in `racing_game` today (stage "client derivations" below). A server-side carving step is planned (G3-D); once it ships the game consumes the carved heights and its own carve step is removed (owner decision 2026-10-05).
- **`roads.graph` and `roads.geom` are opt-in.** They are only added to a release when asked for, so the release id of terrain-only releases did not change when they were added.
- **Some things never travel over the wire.** Render meshes, LOD selection, the road colour raster and physics surface ids are computed in the client from tiles.

---

## 2. Stages, one by one

Each stage has: inputs, code, output, versioning, tiling, tests and consumers. Detail pages hold the long text:
[raw layers](raw_layers.md), [terrain](terrain.md), [roads](roads.md), [client side](client_side.md).

### 2.0 Common rules (apply to every stage)

| Rule | Where |
|---|---|
| Layer = name + version + level set + encoding + inputs. Raw layers are imported. Baked layers are made by an `IDeriver`. | `include/g2m/layer/layer_id.h`, `deriver.h` |
| A deriver is a pure function of its input tile bytes and its config hash. No clock, no libm, integer geometry, sorted iteration. | `include/g2m/layer/deriver.h`, `CLAUDE.md` (determinism rules) |
| A tile is a `g2m.tile/1` container: header + body, typed zstd sections, zstd pinned (1.5.7, level 12). | [formats/tile_container.md](../formats/tile_container.md) |
| Tile key: UTM zone + level + x + y. Level L is a square of 256 * 2^L m. Raw layers live at level 2 (1024 m). | `include/g2m/core/tile_key.h`, [formats/README.md](../formats/README.md) |
| A release = a frozen set of tiles and layer versions. Its id is a SHA-256. | [formats/release_manifest.md](../formats/release_manifest.md) |
| Golden hashes must match on Windows and Linux. | `testdata/golden/*.txt`, `tests/golden/` |

### 2.1 Region resolve and import (raw layers)

| | |
|---|---|
| Inputs | A region definition (`g2m.regions/1` JSON, for the home area `racing_game/data/world/regions.json#home`), the OSM PBF to cut admin boundaries, the DGM1 ZIPs, the OSM PBF, the WorldCover GeoTIFFs. |
| Code | `apps/g2m_tiler/main.cpp` (subcommands `find-admin`, `resolve`, `import-dem`, `import-osm`, `import-landcover`, `import`); `include/g2m/import/{dem,osm,landcover}.h`; decoders in `include/g2m/decode/`. |
| Output | Three raw layers, level 2, in the **source store** (`tiles.sqlite3` + `tiles_blobs/`), scope e.g. `home-2026-09r1`. An `import_report_<scope>.json` is written next to the store. |
| Encodings | `g2m.src.elev.dgm1` body encoding 1 ([src_elev.md](../formats/src_elev.md)); `g2m.src.landcover` encoding 2 ([src_landcover.md](../formats/src_landcover.md)); `g2m.src.osm` encoding 3 ([src_osm.md](../formats/src_osm.md)). |
| Tiling | Level 2 only (1024 m). DGM1: 1024 x 1024 samples per tile, 1/256 m int32, NoData = INT32_MIN. OSM: ways are clipped with a 128 m halo but kept complete. Landcover: 4 m cells, 256 x 256 samples, nearest neighbour. |
| Config hash / release id | Raw tiles are content-addressed (SHA-256 of body). They are the input of the release id. `--pin` marks a scope as a finished release source. |
| Tests | `tests/unit/test_import_*.cpp`, `test_decode_*.cpp`, `tests/layer/test_osm.cpp`, `test_raster_encodings.cpp`, `test_region.cpp`, fuzz targets in `tests/fuzz` and `tests/fuzz_formats`. Goldens: `g1_golden.txt`, `g1c_golden.txt`. |
| Consumers | Every derived layer; `racing_game` also reads `g2m.src.osm` directly for road classes, grip and render colours, and `g2m.src.landcover` for terrain colour and grip. |

Details: [raw_layers.md](raw_layers.md).

### 2.2 Terrain heights: `g2m.elev.base` and `g2m.terrain.height`

| | |
|---|---|
| Inputs | `g2m.src.elev.dgm1` (level 2). |
| Code | `src/layer/derive_terrain.cpp`, `src/layer/terrain_layers.cpp`, `include/g2m/layer/{terrain_layers,derive_terrain,height_tile}.h`. |
| Output | `g2m.elev.base` v1: level 0 only, a direct 256 m window of DGM1 (no resampling). `g2m.terrain.height` v1: levels 0 to 6. Level 0 is byte-identical to `elev.base`. Levels 1 to 6 are the mean of the 4 child tiles of the layer below (half-to-even rounding). Both use body encoding 1. |
| Tile shape | 256 x 256 samples at every level, cell-registered (sample at the cell centre). 1 m cells at level 0, 64 m cells at level 6. |
| Config hash | `ElevBaseDeriver`: `g2m.elev.base/1 dgm1-only`. `TerrainHeightDeriver`: `g2m.terrain.height/2 L0-passthrough-of-elev.base; L1-L6 mean-of-4 half-even pyramid`. The config text is hashed into the release id. |
| Halo | None. Neighbouring tiles are not read. A missing child counts as all-NoData. |
| Tests | `tests/layer/test_derive_terrain.cpp`, `test_terrain_pyramid.cpp`; golden `g2_golden.txt`. |
| Consumers | `racing_game` `WorldTerrain` (render and physics); `g2m_ps_bridge` (physics). |
| Not done | `g2m.terrain.class` has a declared slot and no deriver. No multi-source blending (GLO-30 etc.) and no halo blending: PLAN.md 6.1 describes more than the code does. |

Details: [terrain.md](terrain.md).

### 2.3 Roads: `g2m.roads.graph` and `g2m.roads.geom`

| | |
|---|---|
| Inputs | `roads.graph`: `g2m.src.osm` for the 3 x 3 level-2 neighbourhood. `roads.geom`: the same-tile `roads.graph`, the `roads.graph` tiles that hold every way incident to a node of its ways (stretch context), plus every `g2m.elev.base` level-0 tile the road segments touch (margin 65 m). |
| Code | `src/layer/derive_road_graph.cpp`, `road_graph.cpp`, `road_attributes.cpp`, `osm_roads.cpp`; `derive_road_geom.cpp`, `road_reference.cpp`, `road_profile.cpp`, `vertical_profile.cpp`, `road_dem_dependencies.cpp`, `road_dem_sampler.cpp`; `road_rules.cpp` with `data/road_rules/de.json` (embedded, loader only so far). Builtin wiring: `include/g2m/server/builtin_release.h` (`BuiltinTerrainDerivers::ordered(include_roads, include_geometry)`). |
| Output | `roads.graph` v1, encoding 5 (nodes carry whitelisted tags; 4 still decodes; [road_graph.md](../formats/road_graph.md)), level 2. `roads.geom` v1, encoding 6 ([road_geom.md](../formats/road_geom.md)), level 2: reference line, vertical profile, declined stretches with reasons, plus (street fidelity P1 slice 5) one lane-aware cross-section per profiled stretch (bands, anchor, ramps, offset correction, join records) and a feature section for vertical traffic-calming features (slice 7: carved bumps, humps, tables, dips; lateral kinds render only). Encoding 5 bodies still decode (no cross-sections). |
| Halo | `roads.graph` uses a 128 m halo and complete ways. An edge belongs to the tile that contains its midpoint (integer mm). `roads.geom` reads DEM tiles up to 65 m outside the road. |
| Config hash | graph: `g2m.roads.graph/1 complete-ways default-RoadStyle halo=128 midpoint-mm-owner raw-nine-neighbours node-tags=1 enc=5`. geom: `g2m.roads.geom/1 class-policy-2026-10-02 spacing=5 max-samples=2048 iterations=4096 curvature-exceptions=true sloping-plane-r3 approach=32 height-correction=.25 merge-agreement=.5 incline-soft=5 dependency-margin=65 coverage=graph xs=1 enc=6 rules=de:<sha256 of de.json> join-bisector=1 context=nodes`. The rules hash ties a release id to the exact `data/road_rules/de.json` bytes. |
| Tests | `tests/layer/test_road_*.cpp`, `test_vertical_profile.cpp`; goldens `g2roads_golden.txt` and `g3xs_golden.txt` (encoding-6 bodies and cross-section boundaries of the two committed OSM fixtures, compared Windows = Linux); probe `tools/road_probe`. |
| Consumers | `racing_game`: road surface patch (carving into the level-0 height tile), road visuals, bridge structures, NPC traffic and trucks. |
| Status | Slices A and B of G3 implemented. C has a tested foundation, acceptance open. D to F open. See [roads.md](roads.md) and [../g3_road_pipeline.md](../g3_road_pipeline.md). |

### 2.4 Client-side derivations (not served)

| What | Where | Note |
|---|---|---|
| Render chunk mesh (65 x 65 vertices from a 67 x 67 halo window, same triangle diagonal as Jolt) | `include/g2m/mesh/terrain_chunk.h`, `src/mesh/terrain_chunk.cpp` | Bit-identical across chunk borders (test). |
| LOD quadtree selection | `include/g2m/mesh/lod_select.h` | |
| OSM road raster (colour) and per-cell road surface ids (grip) | `include/g2m/layer/osm_roads.h`: `extract_road_segments`, `rasterize_road_segments`, `paint_road_segments`, `RoadStyle` table; goldens `g2roads`, `g2paint` | Colour and grip come from `g2m.src.osm` (+ `g2m.src.landcover` for the base class), not from a baked layer. |
| Physics tile fill (255 m tiles, 256 samples, node-registered, anchored on DGM1 cell centres by integer offset copy) | `include/g2m/phys/fill_physics_heights.h` | No interpolation. |
| Physics surface ids per cell | `include/g2m/phys/fill_physics_surfaces.h` | |
| Residency, gate, prefetch, look-ahead | `include/g2m/phys/{resident_heights,height_tile_loader,physics_streamer}.h` | |
| Road carving into a copy of the level-0 height tile | `racing_game/core/src/world_terrain.cpp` (`fetch_and_decode`) | Stays in the game until G3-D ships server-side carved heights; then the game step is removed, not kept as a fallback (owner decision 2026-10-05). |

Details: [client_side.md](client_side.md).

### 2.5 Server, cache, releases

| | |
|---|---|
| Code | `include/g2m/server/{server,builtin_release,protocol,tile_store}.h`, `src/server/*.cpp`, `include/g2m/client/{cache,in_process_transport}.h`. |
| Behaviour | `Server::handle()` routes a protocol v1 request. Resolver chain per layer: Cache (derived store under the release id), Derive (inputs resolved recursively), Upstream (local source store, later network). Results of Derive are written to the derived store. |
| Store | SQLite + blob files, content-addressed: [formats/cache_db.md](../formats/cache_db.md). |
| Release | Release id = SHA-256 over source tile hashes, layer versions and config hashes, classifiers and the world seed: [formats/release_manifest.md](../formats/release_manifest.md). Manifest `g2m.release/1`; server manifest `g2m.server/1` in [formats/protocol.md](../formats/protocol.md). |
| Status | In-process only. `apps/g2m_server` (HTTP) and `g2m_cli` do not exist yet. Offline mode returns 503 degraded instead of touching the network. |
| Tests | `tests/server/test_server.cpp`, `test_tile_store.cpp`, `test_protocol.cpp`, `test_client_cache.cpp`, `tests/layer/test_release.cpp`, `test_pack.cpp`. |

---

## 3. Layer dependencies

Which tile of which layer is needed to build a tile of another layer. The labels carry the halo and neighbour rules.

```mermaid
flowchart TD
  OSM["g2m.src.osm - L2 1024 m"]
  DEM["g2m.src.elev.dgm1 - L2 1024 m"]
  LC["g2m.src.landcover - L2 1024 m"]
  EB["g2m.elev.base - L0 256 m"]
  H0["g2m.terrain.height L0 - copy of elev.base"]
  H1["g2m.terrain.height L1 to L6"]
  RG["g2m.roads.graph - L2"]
  RM["g2m.roads.geom - L2"]
  TC["g2m.terrain.class - L0 to L6 - no deriver"]
  CARVE["server-side carving G3-D - planned"]
  BLD["buildings from LoD2 - planned"]
  TRE["trees and vegetation - planned"]

  DEM -->|"one L2 ancestor tile, 16 L0 tiles per source tile"| EB
  EB -->|"same key, byte-identical"| H0
  H0 -->|"4 children of the level below"| H1
  OSM -->|"own tile + 8 neighbours, 128 m halo"| RG
  RG -->|"same key"| RM
  EB -->|"every L0 tile near a road, margin 65 m"| RM
  LC -.->|"planned"| TC
  RM -.->|"planned"| CARVE
  EB -.-> CARVE
  CARVE -.-> H0
  OSM -.-> BLD
  DEM -.-> TRE

  classDef done fill:#d8f0d8,stroke:#2e7d32,color:#000;
  classDef plan fill:#eeeeee,stroke:#888888,stroke-dasharray:4 3,color:#000;
  class OSM,DEM,LC,EB,H0,H1,RG,RM done;
  class TC,CARVE,BLD,TRE plan;
```

Tile-level rules worth remembering:

- `elev.base` has **no halo**. Two neighbouring tiles share no samples. Seams are handled by the consumers (render halo window, physics edge weld).
- The terrain pyramid needs all four children even for a tile at the data edge. A missing child counts as NoData, never as an error.
- `roads.geom` is the only built-in layer whose input list depends on the data (`additional_inputs_for`): the DEM tiles come from the graph body.
- A tile in the 1 km halo of the region may have no DGM1 coverage: 85 such tiles were skipped at import (`import_report_home-2026-09r1.json`).

---

## 4. Runtime tile request

What happens when the game needs a terrain tile. This is the physics path with prefetch. The render path is the same up to "decode"; it then builds a chunk mesh instead.

```mermaid
sequenceDiagram
  participant PS as physics_sim TileManager
  participant SRC as G2mTerrainSource
  participant STR as PhysicsTerrainStreamer
  participant LD as HeightTileLoader worker
  participant WT as racing_game WorldTerrain
  participant SV as g2m::Server
  participant ST as tile store
  participant DV as deriver

  Note over STR: every tick between World steps: update()
  STR->>LD: request L0 keys of keep set, gate keys first
  LD->>WT: fetch height tile
  WT->>SV: TileRequest terrain.height L0 key
  SV->>ST: cache lookup under release id
  alt hit
    ST-->>SV: container bytes
  else miss
    SV->>DV: derive from inputs, resolved recursively
    DV-->>SV: new tile
    SV->>ST: write tile
  end
  SV-->>WT: container
  WT->>WT: decode, smoothing, carve road patch at L0
  WT-->>LD: HeightTile
  LD-->>STR: poll() completion, Ready or Absent or Failed
  STR->>STR: install in ResidentHeightSet
  STR-->>PS: gate ready, World may step
  PS->>SRC: acquire_prefetch_inputs(key) on tick thread
  SRC-->>PS: immutable snapshot of resident tiles
  PS->>SRC: fill_tile_from(snapshot) on background lane
  SRC-->>PS: 256 x 256 heights, node-registered
  PS->>PS: install spare into Jolt heightfield body
  Note over PS,SRC: stale snapshot - prefetch_inputs_current false - falls back to a synchronous fill
```

Notes:

- The tick-time path never calls the server. Only the loader workers do.
- The gate (clock freeze) is a function of position and radius only. Velocity and timing never change which tiles are required, so the simulation stays deterministic.
- Render chunks use the same decoded tile from `WorldTerrain::height_tile()` (cached, mutex-guarded), then `gather_window` and `build_chunk`.
- Code: `include/g2m/phys/physics_streamer.h` (the header comment documents the order), `bridges/physics_sim/include/g2m/ps_bridge/g2m_terrain_source.h`, `racing_game/core/src/world_terrain.cpp`, `physics_sim` `core/src/terrain/tile_manager.cpp` (prefetch tickets). Test: `bridges/physics_sim/tests/test_prefetch_snapshot.cpp`.

---

## 5. Data sources

Files are under `S:\claude_code\geo2map_data\downloads\` (not in git). The full licence and URL text is in `S:\claude_code\geo2map_data\SOURCES.md`; this table is the summary. "Used" means a pipeline stage reads it today.

| Dataset | Provider | Licence | Attribution duty | Local path | Stage | Status |
|---|---|---|---|---|---|---|
| DGM1, 1 m terrain heights | HVBG Hessen | dl-de/zero-2.0 | Optional (we keep it) | `downloads/dgm1/PKT_*.zip` (8 files, 8.5 GB) | `import-dem` -> `g2m.src.elev.dgm1` | **Used** |
| OSM Hessen extract | Geofabrik / OpenStreetMap | ODbL 1.0 | **Required**: (c) OpenStreetMap contributors; share-alike for derived databases (affects served packs, G7) | `downloads/hessen-latest.osm.pbf` | `import-osm` -> `g2m.src.osm`; roads, grip, colours | **Used** |
| ESA WorldCover 2021 v200 | ESA | CC BY 4.0 | **Required** | `downloads/ESA_WorldCover_10m_2021_v200_N48E006_Map.tif` | `import-landcover` -> `g2m.src.landcover` | **Used** |
| Copernicus GLO-30 DEM tiles N49 and N50 | Copernicus / ESA | Copernicus DEM licence | **Required** (DLR, Airbus, EU, ESA text in SOURCES.md) | `downloads/Copernicus_DSM_COG_10_N49_00_E008_00_DEM.tif`, `..._N50_...` | Planned fallback for DGM1 gaps | Downloaded, not used |
| HVBG LoD2 3D buildings | HVBG | **dl-de/by-2.0** | **Required** (HVBG source line, SOURCES.md) | `downloads/hvbg_lod2/` (1.2 GB) | Planned: buildings (street fidelity P4) | Downloaded, not used |
| HVBG DOM1 surface model | HVBG | dl-de/zero-2.0 | Optional | `downloads/hvbg_dom1/` (4.2 GB) | Planned: trees (P5) | Downloaded, not used |
| HVBG DOP20 orthophotos | HVBG | dl-de/zero-2.0 | Optional | `downloads/hvbg_dop20/` (4.8 GB) | Planned: colours, QA (P5, P6) | Downloaded, not used |
| ATKIS Basis-DLM Hessen | HVBG | dl-de/zero-2.0 | Optional | `downloads/hvbg_basis_dlm/` (298 MB) | Planned: road width, lanes (P1) | Downloaded, not used |
| Frankfurt Baumkataster | Stadt Frankfurt | dl-de/by-2.0 | **Required** | `downloads/frankfurt_baumkataster/` | Planned: trees (P5) | Downloaded, not used |
| Thuenen dominant tree species | Thuenen Institute | CC BY 4.0 | **Required** (citation with DOI in SOURCES.md) | `downloads/thuenen_tree_species/` | Planned: trees (P5, P6) | Downloaded, not used |
| Copernicus HRL tree cover and leaf type | Copernicus Land Monitoring Service / EEA | Free and open (Regulation (EU) 1159/2013) | **Required** (EU, Copernicus, EEA) | `downloads/copernicus_hrl/` | Planned: trees (P5) | Downloaded, not used |
| DWD phenology | Deutscher Wetterdienst | GeoNutzV | **Required**: source reference | `downloads/dwd_phenology/` | Planned: seasons (P6) | Downloaded, not used |

Attribution status in the game: the `racing_game` manifest lists only the DGM1 attribution today. OSM, WorldCover and the others are not in a credits screen yet (R5 is not done). See [open questions](#9-open-questions-and-contradictions).

---

## 6. Roadmap

Two plans meet here: the engine phases G0 to G8 (in `racing_game/PLAN.md` section 12 and `PLAN.md` here) and the street fidelity phases P0 to P8 ([../street_fidelity/plan.md](../street_fidelity/plan.md)).

```mermaid
flowchart TB
  G0["G0 skeleton, geodesy - done"]
  G1["G1 server core, raw layers, tiler, cache - done"]
  G2["G2 terrain pyramid, render chunks, physics bridge - done for height"]
  G2C["G2 terrain.class deriver - open"]
  G3A["G3 A,B roads.graph and roads.geom codec - done"]
  G3C["G3 C profile acceptance - in progress"]
  G3D["G3 D junction surfaces and carving - open"]
  G3E["G3 E ribbons, markings, bridge bodies - open"]
  G3F["G3 F route acceptance - open"]
  G4["G4 routing - planned"]
  G5["G5 land cover, buildings, water, vegetation - planned"]
  G6["G6 HTTP server and packs - planned"]
  G7["G7 hosted server - planned"]
  G8["G8 worldwide - planned"]
  P0["P0 OSM tag coverage probe - done"]
  P1["P1 lane-aware cross-section - slices 1-5 done, 6-7 open"]
  P2["P2 junction surfaces - planned"]
  P3["P3 markings - planned"]
  P4["P4 buildings - planned"]
  P5["P5 trees - planned"]
  P6["P6 seasons - planned"]
  P7["P7 facades - planned"]
  P8["P8 imagery QA - optional"]

  G0 --> G1 --> G2 --> G3A --> G3C --> G3D --> G3E --> G3F
  G2 -.-> G2C
  G3F -.-> G4
  G3D -.-> G5
  G1 -.-> G6 --> G7 --> G8
  P0 --> P1 --> P2 --> P3
  P1 -.-> G3D
  P4 --> P5 --> P6 --> P7 --> P8

  classDef done fill:#d8f0d8,stroke:#2e7d32,color:#000;
  classDef part fill:#fff3cd,stroke:#b58900,stroke-dasharray:4 3,color:#000;
  classDef plan fill:#eeeeee,stroke:#888888,stroke-dasharray:4 3,color:#000;
  class G0,G1,G2,G3A,P0 done;
  class G3C,P1 part;
  class G2C,G3D,G3E,G3F,G4,G5,G6,G7,G8,P2,P3,P4,P5,P6,P7,P8 plan;
```

| Item | Status | Where it is described |
|---|---|---|
| Terrain pyramid, render chunks, physics bridge, prefetch ring | Implemented | [terrain.md](terrain.md), `racing_game/PLAN.md` G2/R2 |
| `terrain.class` deriver | Open (slot only) | `PLAN.md` G2.5 |
| G3 A: identity-preserving graph | Implemented | [../g3_road_pipeline.md](../g3_road_pipeline.md) |
| G3 B: graph and geom tile encoding, derivation, opt-in serving | Implemented | [../g3_road_pipeline.md](../g3_road_pipeline.md), [../formats/road_geom.md](../formats/road_geom.md) |
| G3 C: reference lines and vertical profiles | Foundation implemented, acceptance open | [../g3_profile_foundation.md](../g3_profile_foundation.md), [../g3_reference_lines.md](../g3_reference_lines.md) |
| G3 D: junction surfaces and server-side carving | Open | [../g3_road_pipeline.md](../g3_road_pipeline.md) section D |
| G3 E: ribbons, markings, bridge bodies | Open | section E |
| G3 F: route acceptance | Open | section F |
| Street fidelity P0 (tag probe) | Done | [../osm_tag_coverage_home.md](../osm_tag_coverage_home.md) |
| Street fidelity P1 (lane-aware cross-section) | Slices 1 to 6 implemented (context, node tags, base and join resolution, encoding 6, consumer API: carve footprint, lane queries, grip-raster envelope); profile features open | [../street_fidelity/p1_cross_section_design.md](../street_fidelity/p1_cross_section_design.md) |
| Street fidelity P2 to P8 | Planned, nothing written | [../street_fidelity/plan.md](../street_fidelity/plan.md) section 6 |
| HTTP server, packs, hosted CDN (G6, G7) | Planned | [../formats/protocol.md](../formats/protocol.md), [../formats/pack.md](../formats/pack.md) |

---

## 7. How to rebuild and run

Only commands that exist in the code as of this date. Run from `S:\claude_code\geo2map_engine` in a VS 2022 developer shell (`tools\ci.ps1` enters one itself).

### 7.1 Build and test the library

```powershell
cmake --preset release
cmake --build --preset release
ctest --preset release --output-on-failure
```

CI (see `tools/ci.ps1` header and `CLAUDE.md` for the exact legs):

```powershell
tools\ci.ps1                 # default fast run
tools\ci.ps1 -Full           # every leg, every test
```

There is no docs-only mode in `tools/ci.ps1`. A docs-only change needs no build.

### 7.2 Import raw data into a store

The tiler is built by default (`G2M_BUILD_APPS` is ON). All subcommands are in `apps/g2m_tiler/main.cpp` (`usage()`).

```powershell
g2m_tiler resolve regions.json#home --pbf hessen-latest.osm.pbf --out home.resolved.json
g2m_tiler import regions.json#home --pbf hessen-latest.osm.pbf --store STORE_DIR --scope SCOPE `
    --dem-data DGM1_DIR --wc-data WORLDCOVER_DIR --jobs N --pin
g2m_tiler stats --store STORE_DIR --scope SCOPE
g2m_tiler index-hash --store STORE_DIR --scope SCOPE
```

`import` runs resolve + `import-dem` + `import-osm` + `import-landcover` and writes `import_report_<scope>.json`. The individual `import-*` subcommands take `--resolved` instead of the region argument. `diff-scopes` compares two stores tile by tile. `check-dem-nodata` reports DEM tiles with no data. The real home import (scope `home-2026-09r1`) took 131 s and wrote 3170 tiles (807 MB) per its report.

### 7.3 Bake derived layers

```powershell
g2m_tiler bake --store STORE_DIR --scope SCOPE --derived DERIVED_DIR `
    --layers g2m.elev.base,g2m.terrain.height [--levels 0-6] [--jobs N] [--pin]
```

`--store` must be a finished, `--pin`ned scope. The result does not depend on `--jobs`. Without `bake`, a consumer derives tiles on demand through `g2m::Server` into its own derived store (`racing_game` does this for the home region, store `S:\claude_code\geo2map_cache\home-r1\baked`). Levels a layer does not have are skipped (`roads.*` exist only at level 2). Naming `g2m.roads.graph` in `--layers` adds the road graph deriver to the release; naming `g2m.roads.geom` adds both (`src/import/bake.cpp`: `include_roads` / `include_geometry`). Adding them changes the release id; terrain-only bakes keep the old one.

### 7.4 Probes

```powershell
g2m_osm_tag_probe --pbf F --region REGIONS.json#NAME --out-md F --out-json F
g2m_road_probe STORE_DIR SCOPE utm32n/2/455/5421 out.csv [--exceptions] [--profiles]
```

Both are under `G2M_BUILD_APPS`. See `tools/osm_tag_probe` and `tools/road_probe/README.md`. The probes never write to the tile cache.

### 7.5 Use it from the game

`racing_game` reads `data/world/world_config.json`: source store `${RG_G2M_HOME}` (default `S:\claude_code\geo2map_cache\home-r1`), scope `home-2026-09r1`, derived store `${RG_G2M_HOME}/baked`. Nothing in that repo bakes it. See `racing_game/CLAUDE.md` section "World config".

### 7.6 Where the release id changes

Any change to a deriver's config text, a layer version, an encoding version, a classifier or a source tile changes the release id. That is by design: a new id means new derived tiles. Changing only docs or tests does not.

---

## 8. Glossary

| Term | Meaning |
|---|---|
| Layer | A named, versioned set of tiles, e.g. `g2m.terrain.height`. Raw (imported) or baked (derived). |
| Raw layer | Imported from a source file. Names start with `g2m.src.`. Level 2. |
| Baked layer | Made by a pure deterministic deriver from other layers. |
| Deriver (`IDeriver`) | A pure function: input tiles + config -> output tile bytes. |
| Tile | One cell of a layer, stored as a `g2m.tile/1` container. |
| Tile key | `utm<zone>n/<level>/<x>/<y>`. Packed into 64 bits for storage. |
| Tile level | Level L covers 256 * 2^L metres per side. Level 0 = 256 m, level 2 = 1024 m, level 6 = 16384 m. |
| Cell-registered | A sample sits at the centre of its cell (half-metre offsets for 1 m cells). Used by render and the stored height tiles. |
| Node-registered | A sample sits on a lattice corner. Used by the physics heightfield tiles (255 m, 256 samples). |
| Halo | Extra data around a tile that is read but not owned, so neighbours agree. `roads.graph`: 128 m. Render chunk: a 67 x 67 window around 65 x 65 vertices. |
| Stretch | A section of one road between two junctions or structure boundaries. The unit of horizontal and vertical fitting in `roads.geom`. |
| Reference line | The fitted horizontal centre line of a stretch (straights, clothoids, arcs). |
| Profile | The vertical line of a road: height along the reference line, bounded by class grade and curvature. |
| Complete way | An OSM way with all its nodes, even outside the tile. Needed so a profile never fits a clipped fragment. |
| Declined stretch | A stretch the fitter refused, kept in `roads.geom` with an explicit reason. |
| Encoding | The byte format of a tile body (1 elev, 2 landcover, 3 osm, 4 road graph, 5 road geom). |
| Config hash | SHA-256 of a deriver's own config text. Part of the release id. |
| Release | A frozen, named set of layers and source tiles that the server serves. |
| Release id (rid) | SHA-256 over source tile hashes, layer versions and config hashes, classifiers, world seed. |
| Scope | A name for one import into a store, e.g. `home-2026-09r1`. |
| Source store / derived store | The imported raw tiles / the cache of derived tiles. |
| Pin | Marks a scope as finished and protected from eviction. |
| Resolve | Turn a region definition into a list of level-2 tiles (interior runs + halo). |
| Resolver chain | Cache, then Derive, then Upstream, inside `g2m::Server`. |
| Gate | The set of physics tiles that must be resident before the simulation clock may advance. |
| Prefetch ring | Extra tiles loaded around the gate set, plus a look-ahead square along the velocity. |
| Snapshot input | An immutable copy of resident tiles that a background thread uses to fill a physics tile. |
| Golden hash | A recorded hash of an encoder or deriver output. Must equal on Windows and Linux. |
| NoData | `INT32_MIN` in height tiles. |
| Datum | Vertical reference: DHHN2016 normal heights for DGM1. |
| Street fidelity P0 to P8 | The phased plan for lane, marking, building and tree detail. |

---

## 9. Open questions and contradictions

These are listed, not resolved. Each needs an owner decision or a check.

1. **G3 endpoint heights vs class grade.** [../g3_road_pipeline.md](../g3_road_pipeline.md) (checkpoint, top) says the priority between endpoint heights and class grade "needs an owner choice". [../g3_profile_foundation.md](../g3_profile_foundation.md) says endpoint heights and class bounds "retain priority". `racing_game/PLAN.md` G3 status says the grade-policy question "is withdrawn" (it came from reading an underpass as a deck slope). The geo2map docs still show it pending. **Resolved 2026-10-05 (owner):** measured endpoint heights win. Where honouring them needs a grade steeper than the class bound, the profile keeps the endpoint heights and the stretch is flagged as a grade exception (the existing exception mechanism), not clamped. [../g3_road_pipeline.md](../g3_road_pipeline.md) and [../g3_profile_foundation.md](../g3_profile_foundation.md) say so now; the `racing_game` note about a withdrawn question stays true for the underpass case.
2. **HVBG LoD2 licence.** `geo2map_data/SOURCES.md` records dl-de/**by**-2.0 (attribution required) after reading the primary metadata. [../street_fidelity/plan.md](../street_fidelity/plan.md) and `PLAN.md` 3.9, 5.3 and 13 plan with dl-de/**zero**-2.0 for HVBG products. **Resolved 2026-10-05 (documents):** `street_fidelity/plan.md` and `PLAN.md` 3.9, 5.3 and 13 now plan with dl-de/by-2.0 and attribution for LoD2; the attribution list must include it before buildings ship.
3. **Attribution is not shown to players.** `racing_game`'s manifest lists only the DGM1 attribution. OSM (ODbL), WorldCover (CC BY 4.0) and Copernicus are in use or downloaded, but the credits screen (R5) is not built.
4. **Where road carving lives.** Today `racing_game` carves a road patch into the level-0 height tile after decode. The roadmap (G3-D) says the server derives carved heights and `terrain.class` from the same geometry. Until G3-D, two consumers that skip the game's step would see uncarved terrain. **Resolved 2026-10-05 (owner):** carving stays in `racing_game` (`WorldTerrain::fetch_and_decode`, level 0) until G3-D ships server-side carved heights. When G3-D lands, the game consumes the server's carved heights and its own carve step is REMOVED, not kept as a fallback: one source of truth for render and physics (same principle as the G2M-010 grip rule).
5. **`CLAUDE.md` of this repo is partly stale.** It still describes `elev.base`, `terrain.height`, `bake`, roads and the road probe as not built or planned in places. This docs folder is checked against code; `CLAUDE.md` should be reconciled.
6. **PLAN.md vs code, terrain.** PLAN.md 6.1 describes multi-source blending and a 16 m halo for `elev.base`. The code is DGM1-only without halo. `g2m.terrain.class` has no deriver although the slot exists.
7. **PLAN.md vs code, CLI.** PLAN.md 3.10 and 5.5 describe a CLI that differs from the real `g2m_tiler` subcommands. `apps/g2m_server` and `g2m_cli` are not built.
8. **`LayerEncoding` enum.** Resolved 2026-10-05: the enum now lists the road encodings (`RoadGraph` 4, `RoadGraphNodeTags` 5, `RoadGeom` 5, `RoadGeomCrossSection` 6). Ids are per layer, so the same number can name two layers' encodings.
9. **Height quantiser unification (Q3).** PLAN.md records it as undecided: the half-even quantiser (DGM1 importer) and the older ties-away-from-zero `quantise_height` (everything else) both exist.
10. **85 DGM1 halo tiles without coverage.** Accepted until the GLO-30 fallback exists. Heights there are NoData.
11. **Pin values differ between documents.** `bridges/physics_sim/physics_sim.pin` holds `b90a531...`; older PLAN text mentions `5c648a5` and `be36545`; `racing_game` pins other commits. Which is the supported combination? Check `git submodule status` in `racing_game`.
12. **Is roads baked for the home store?** `bake` accepts the road layers (code verified), but this review did not run it and did not check whether `S:\claude_code\geo2map_cache\home-r1\baked` already holds `roads.*` tiles.
13. **Street fidelity plan is partly outdated.** Resolved 2026-10-05: its section 9 now says what has been downloaded. Since 2026-10-05 LoD2, DOM1, DOP20, Basis-DLM, tree datasets, HRL and phenology are downloaded ([section 5](#5-data-sources)).
14. **Road curvature exceptions: decided or not?** `tools/road_probe/README.md` says the owner must choose how served geometry represents curvature exceptions. [../g3_reference_lines.md](../g3_reference_lines.md) says the owner continuation selects "preservation with explicit curvature exceptions", and the `roads.geom` config hash text contains `curvature-exceptions=true`. The probe README reads as undecided, the other two as decided.
15. **Stale bake flags in a format doc.** Resolved 2026-10-05: road_graph.md and g3_road_pipeline.md now say `--levels 2-2` (the real tiler flag, `apps/g2m_tiler/main.cpp`).

---

## 10. Index of all documents

| Topic | Document |
|---|---|
| This overview | `docs/pipeline/README.md` |
| Detail pages | [raw_layers.md](raw_layers.md), [terrain.md](terrain.md), [roads.md](roads.md), [client_side.md](client_side.md) |
| Byte formats | [../formats/README.md](../formats/README.md): [tile_container](../formats/tile_container.md), [src_elev](../formats/src_elev.md), [src_landcover](../formats/src_landcover.md), [src_osm](../formats/src_osm.md), [pack](../formats/pack.md), [release_manifest](../formats/release_manifest.md), [regions](../formats/regions.md), [protocol](../formats/protocol.md), [cache_db](../formats/cache_db.md), [road_graph](../formats/road_graph.md), [road_geom](../formats/road_geom.md) |
| G3 road pipeline | [../g3_road_pipeline.md](../g3_road_pipeline.md), [../g3_profile_foundation.md](../g3_profile_foundation.md), [../g3_reference_lines.md](../g3_reference_lines.md) |
| Street fidelity | [../street_fidelity/plan.md](../street_fidelity/plan.md), [../street_fidelity/p1_cross_section_design.md](../street_fidelity/p1_cross_section_design.md), [../osm_tag_coverage_home.md](../osm_tag_coverage_home.md) |
| Design plan | `PLAN.md` (this repo), `racing_game/PLAN.md` section 12 (milestones) |
| Architecture map and rules | `CLAUDE.md` |
| Road probe | `tools/road_probe/README.md` |
| Source data, licences | `S:\claude_code\geo2map_data\SOURCES.md` |
