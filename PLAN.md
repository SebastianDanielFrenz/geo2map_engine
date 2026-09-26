# geo2map_engine: implementation plan

- **Owner:** Sebastian.
- **Written:** 2026-09-25 by the planning session (Opus 5.5).
- **How:** read-only research against:
  - `S:\claude_code\physics_sim` at `be36545`. The other session's uncommitted drivetrain work was not read.
  - the knowledge vault: PHYS-003, -008, -009, -011, -021, -025, -026, -032, -035, -042 and TOOL-023.
  - the web: three research agents; their sources are listed under References.
- **Nothing was built, installed, downloaded or created.** Only this file was written.
- **Markers:**
  - **[unverified]**: not confirmed against code or a primary source.
  - **[estimate]**: my own arithmetic, to be replaced by a measurement at the milestone named next to it.

Three repositories:

| Repo | Role | Depends on |
|---|---|---|
| `S:\claude_code\physics_sim` (exists) | vehicle and rigid-body physics (`ps_core`) | Jolt, nlohmann/json |
| `S:\claude_code\geo2map_engine` (new) | geography to game map: library + map server. C++ namespace `g2m`, target prefix `g2m_` | third-party libs only; never physics_sim, never a game |
| `S:\claude_code\racing_game` (new) | the game: Godot project + engine-neutral game logic | pins both repos above |

A future farming simulator is a fourth repo that pins the same two libraries.

---

> **Split note.** This file holds the library and server parts (sections 2-8, plus the dependency list from 13) of the joint plan written on 2026-09-25. The cross-repo schedule (milestones G0-G8 and R0-R10), the risks, the owner decisions and the game-side sections (1, 9-15) live in `S:\claude_code
acing_game\PLAN.md`. Section numbers are kept identical in both files, so a reference such as "(11.7)" or "D12" points there.

## 0. Key decisions (details in the sections named)

1. **geo2map_engine is a standalone engine-neutral C++20 library plus a map server built from the same code (sections 2 and 3).**
   - It has no dependency on physics_sim or on any game.
   - It exposes generic typed map data plus extension points; games add their meaning through those.
2. **Map data is served by a map server, with one protocol and three deployments (section 3):**
   - the owner's hosted default server, which the game ships pointing at;
   - a custom endpoint set in config;
   - an integrated server: the same server code in-process, or as a local sidecar, for offline play and development.
   - The protocol is HTTP GET on immutable, release-pinned tile URLs, with a manifest endpoint.
3. **Served data: baked (derived) generic layers by default. Raw source layers are also on the protocol, and every layer can be derived on the client as a fallback (section 3.3; owner decision D2).**
   - Baked tiles cost about the same bandwidth as raw data.
   - They save client CPU, are identical for every player by construction, and cache across players on a CDN.
   - Render meshes, vegetation instances and physics tiles are never served. The client builds them cheaply from the served layers.
4. **Geodata acquisition (section 5.5):**
   - Bulk downloads converted by our own offline tiler: HVBG DGM1, Geofabrik OSM extracts.
   - Range-read cloud-optimised files from public open-data buckets: Copernicus GLO-30, ESA WorldCover.
   - Never live queries against OSM services.
5. **Grid and frame: UTM (section 4).**
   - Tiles are keyed by (UTM zone, level, easting index, northing index).
   - The session frame is the UTM grid minus an integer-kilometre origin; z is orthometric height.
   - Hessen DGM1 is published natively on integer UTM32 metres, so the home region needs no resampling.
6. **Physics gets terrain and roads through the heightfield (section 8).**
   - Roads are carved into a 1 m height raster with per-cell surface ids.
   - Only bridges (later also tunnels and buildings) are streamed static mesh bodies.
   - The glue is `g2m_ps_bridge`, an opt-in target in the geo2map_engine repo. It is built only when the consumer already provides `ps_core`.
7. **physics_sim needs a few additive changes (section 9).**
   - R1 (the ±200 m height clamp) and R3 (the tile-seam contact spike) are done in physics_sim 5c648a5, which racing_game pins. One boundary is still open: a heightfield-vs-static-mesh boundary spike (162 m/s² at test_ground x = −500). Our bridges and structures create the same kind of boundary; see G3.
8. **The game builds one GDExtension DLL (`rg_godot`, section 10).**
   - It contains `ps_core`, `g2m`, the bridge, and the game's own engine-neutral `rg_core`.
   - Both libraries are pinned as git submodules at explicit commits. Pinning a sibling path is not an option: it would build against another session's uncommitted work.
9. **Milestones interleave library (G) and game (R) steps (section 12).**
   - Everything up to real roads uses the integrated in-process server only.
   - The HTTP transport and sidecar follow once the layer set is stable. The hosted default server comes after that.

---

## 2. geo2map_engine: library

### 2.1 Scope

**In scope:**
- fetching raw sources, and caching;
- geodesy;
- deterministic derivation of generic map layers;
- the map server and its protocol;
- the client (streaming, cache, mesh builders, routing);
- the C API;
- thin engine helpers.

**Out of scope (game semantics):** what is drivable, grip values, races, farming fields, soil models, traffic policy.
- The library exposes facts. Examples: a road with `surface=sett`, 2 lanes, maxspeed 50; a polygon tagged `landuse=farmland`; a cell classified `tree_cover`.
- Games attach meaning through extension points (2.5).

### 2.2 Repository layout (`S:\claude_code\geo2map_engine`)

```
geo2map_engine/
  CMakeLists.txt, CMakePresets.json    same compiler/preset matrix as physics_sim (clang-cl, cl, WSL clang)
  CLAUDE.md, README.md                 map only; rationale goes into the vault
  docs/
    api/                               C and C++ API guide (Doxygen from headers + hand-written guide)
    protocol/                          map-server protocol v1 spec, examples, conformance rules
    formats/                           g2m.tile/1, layer encodings, cache db schema, release manifest
    licences.md                        per source: licence, attribution text, obligations
  include/g2m/                         public C++ API (no third-party types)
    core/      det_math.h, geo/ (ellipsoid, tm, utm, ecef), tile_key.h, raster.h, polyline.h, hash.h
    layer/     layer_id.h, layer_registry.h, deriver.h (IDeriver), classifier.h (IClassifier), release.h
    server/    server.h (g2m::Server), transport.h (ITransport), upstream.h (IUpstream), config.h
    client/    client.h (g2m::Client), streamer.h (interest points), cache.h, fetcher.h (IFetcher)
    graph/     road_graph.h, routing.h
    mesh/      terrain_chunk.h, road_mesh.h, building_mesh.h, instances.h, map_geometry.h
  include/g2m_c/g2m.h                  flat C API, stable ABI
  src/                                 mirrors include/
  third_party/musl_math/               deterministic libm port (2.4)
  apps/
    g2m_server/                        server binary: sidecar or hosted (HTTP transport)
    g2m_tiler/                         OFFLINE importer: DGM GeoTIFF / OSM PBF / COGs -> raw source tiles
    g2m_cli/                           derive/dump/hash tiles, run golden checks, pack/prebake a region, conformance client
  bridges/physics_sim/                 g2m_ps_bridge (opt-in, 2.6)
  adapters/godot/                      g2m_godot (static lib): IFetcher over HTTPClient, mesh/texture upload helpers
  adapters/unreal/                     later: UE5 plugin skeleton over the C API
  tests/       unit/ golden/ determinism/ protocol/ fuzz/ bench/
  testdata/    small committed fixtures: a 2 x 2 km DGM1 cut + matching OSM cut, golden hashes
  tools/       setup_dev_env.ps1, ci.ps1
```

### 2.3 Targets

| Target | Contents | Third-party (all static) |
|---|---|---|
| `g2m_core` | deterministic math, geodesy, tile keys, rasters, geometry, hashing | none |
| `g2m_layers` | layer registry, raw-source decoders, all built-in derivers | zstd, libdeflate, Clipper2, earcut.hpp |
| `g2m_server` (lib) | `g2m::Server`: request router, resolver, source store, derived store, upstreams | SQLite |
| `g2m_client` | `g2m::Client`: streamer, client cache, mesh builders, routing, in-process transport | SQLite |
| `g2m_c` | C API over client + in-process server | — |
| `g2m_http` | HTTP transport: server side (cpp-httplib) and a libcurl `IFetcher` for tools/server upstreams | cpp-httplib, libcurl |
| `g2m_ps_bridge` | `ITerrainSource` + static-body feeder | consumer's `ps_core` |
| `g2m_godot` | Godot helpers | consumer's godot-cpp |
| apps | `g2m_server`, `g2m_tiler` (+ libosmium, protozero), `g2m_cli` | |

- `g2m_warnings` and `g2m_options` mirror physics_sim's `ps_warnings`/`ps_options`: warnings as errors in own code, `/arch:AVX2`, `/fp:precise`, no FMA contraction.
- Same static CRT as physics_sim, so one game DLL can link both.

### 2.4 Determinism rules

**Goal:** the same input bytes, library derivation version, layer configuration and seed give bit-identical layer tiles on every supported compiler and OS.

This matters more than in physics_sim. The hosted server (Linux) and the integrated server inside a Windows client must produce the same bytes, or a client-derived tile differs from what peers download (3.6).

Rules, adopted from physics_sim D7/D8 and vault PHYS-009/-026:
- **No platform libm transcendentals in derivation code.**
  - g2m carries its own musl-derived port (MIT).
  - It adds `sinh`, `cosh`, `asinh`, `atanh` and `hypot`, which transverse Mercator needs.
  - This deliberately duplicates physics_sim's `ps::math` port, because g2m must not depend on physics_sim.
  - CI greps for `std::sin` and similar, as physics_sim does.
- **Floating point:** `/fp:precise` with FMA contraction off. No `-ffast-math`, no SIMD reductions whose order depends on vector width.
- **Integer geometry wherever topology matters:** Clipper2's Int64 paths at 1 mm resolution.
- **No iteration over unordered containers in anything that decides output.** Sort keys before use (PHYS-026).
- **Parallelism only across whole tiles.** A derivation's inputs are its own input tiles plus a fixed halo read from neighbouring *input* tiles. It never reads neighbouring *outputs*, so results never depend on load order.
- **Randomness** (vegetation scatter, building variation) comes from a counter-based hash of (world seed, tile key, feature id, stream). There is no stateful generator.
- **Quantised outputs** (6.1): heights are stored as integers at 1/256 m, so tiny FP differences cannot leak in via the least significant bits.
- **Golden tiles:** every CI leg checks a committed hash per golden tile, and Windows and Linux hashes are compared.
- **Fuzzing:** every decoder of server- or file-supplied bytes is fuzzed (libFuzzer on the clang legs), because a custom endpoint is untrusted input.

### 2.5 Extension points

- **`IClassifier`** maps source facts to class ids.
  - The built-in default produces a generic `g2m::LandClass`: `paved_road, unpaved_road, rail, water, building, tree_cover, shrub, grassland, cropland, bare, built_up, wetland, ...`.
  - It also produces a `g2m::SurfaceKind`: `asphalt, concrete, paving_stones, sett, cobblestone, gravel, compacted, dirt, grass, sand, ...`.
  - Consumers supply tables or code for their own ids.
- **`IDeriver`** declares a named, versioned layer as a pure function: `(input layer tiles + halo, config, seed) -> tile bytes`.
  - It declares its inputs, tiling, levels and version.
  - The same registration works in the server (baked) and in the client (local derivation). See 3.3 for where custom layers run.
- **`IUpstream`** is a raw data source adapter: a local source store, a COG over HTTP, a PMTiles archive, or another g2m server. SoilGrids for the farming sim would be one more upstream.
- **Style and material tables** for mesh builders and map geometry are data files, not code.

Layer names are namespaced: `g2m.*` for built-ins, `<vendor>.*` for others (e.g. `rg.race_routes`, `fs.field_parcels`).

### 2.6 Where the physics bridge lives

`g2m_ps_bridge` lives in `geo2map_engine/bridges/physics_sim/` and is built only `if(TARGET ps_core)`.

- geo2map_engine never pins or fetches physics_sim, so there is no diamond dependency. The bridge always compiles against the physics_sim commit the consuming game pinned.
- Both games reuse it.
- geo2map_engine's CI builds it against a pinned physics_sim commit, to catch API drift early.
- It contains mechanism only:
  - sampling the served terrain layer into `TileSample`;
  - streaming bridge and building bodies;
  - mapping class ids to `SurfaceId` through a consumer-supplied name table.
- Which physics surface a class becomes is game data.

Rejected alternatives:
- **a copy per game:** it would drift between the racing and farming games;
- **inside physics_sim as P11's `ps_geo`:** physics_sim would pin geo2map_engine while the game pins both, which is a real diamond. It would also put code into a repo another session owns.

### 2.7 Relationship to PLAN.md P11 (owner decision D1)

PLAN.md P11 plans a `ps_geo` target inside physics_sim, fed by an offline importer that bakes a bounding box. The owner's 2026-09-25 rules supersede it: an on-demand world, reusable across engines and projects, served by a map server.

**Recommended reconciliation:** P11 becomes "physics_sim consumes geo2map_engine through `g2m_ps_bridge`".
- **physics_sim keeps only physics:**
  - `ITerrainSource` (exists);
  - the per-tile height datum (R1) and prebuilt static shapes (R2);
  - `IWorldShape` with the flat implementation, and later a WGS84 curved one;
  - per-body gravity.
- **P11's acceptance tests move to geo2map_engine and the bridge:** heights match the DEM; no steps at road/terrain seams; footprints within 0.1 m; a 50 km drive without hitches.
- **The curved world shape stays a physics task.** g2m already provides the geodesy it needs: ECEF, ellipsoid normals, local ENU frames at any grid point.
- **The offline bbox bake survives as `g2m_tiler`,** but only to fill the server's raw source store. The game never consumes a bake directly.

---

## 3. The map server

### 3.1 Three deployments, one code path

```
                +---------------------------- g2m::Server (one implementation) ---------------------------+
                |  router -> resolver: cache hit? -> derive from inputs (IDeriver)? -> ask upstream?       |
                |  source store (raw, content-addressed)   derived store (baked tiles, content-addressed)  |
                |  upstreams: local source store | public COG/PMTiles buckets | another g2m server        |
                +-----------------------------------------------------------------------------------------+
                      ^ in-process transport          ^ HTTP transport (g2m_http)
                      |                               |
  Integrated (in-process, default for dev/offline)    Sidecar (same binary on localhost)   Hosted default / custom endpoint
  game DLL links g2m_server lib                       g2m_server.exe --cache dir --port 0  g2m_server behind reverse proxy + CDN
```

- **Integrated server.**
  - A `g2m::Server` object created by the game (C API `g2m_server_create(config)`) with a local cache directory and an upstream list.
  - The client talks to it through the in-process transport. Requests and responses are the same typed objects the HTTP transport serialises, minus the socket.
  - Offline mode is simply the integrated server with network upstreams disabled: only cached or imported data is available.
- **Sidecar.**
  - The same `g2m_server` binary, started by the game or a tool on localhost with a random port that it reports on stdout.
  - Useful when a Godot editor, the game and the tools share one cache, or to keep heavy I/O out of the game process.
- **Hosted default.**
  - The same binary on the owner's infrastructure, behind a reverse proxy and a CDN (3.7).
  - The game build ships this URL.
- **Custom endpoint.**
  - Any server that speaks protocol v1: the reference binary run by a modder or studio, or their own implementation.
  - Set via the game's settings file or command line (`--map-server <url>`).
  - The client checks compatibility against the manifest (3.4) before use.

**Upstream chaining is what makes the three deployments one design.**
- An integrated server's upstream can be the hosted server: it fetches baked tiles, falls back to fetching raw layers and deriving locally, or goes fully offline.
- The hosted server's upstreams are its own raw-data mirrors.
- The per-layer policy (3.3) is simply the order in which the resolver tries these options.

### 3.2 What is a "layer"

Every piece of served data is a tile of a named, versioned layer:

| Layer | Kind | Tiling / levels | Encoding | Content |
|---|---|---|---|---|
| `g2m.src.elev.<source>` | raw | the source's own tiling: DGM1 1 km UTM32 tiles; GLO-30 1° tiles re-chunked to 0.25° | int32 1/256 m, row delta, zstd | native DEM samples + datum id |
| `g2m.src.osm` | raw | 1 km UTM tiles + halo membership | own columnar format, zstd | OSM nodes/ways/relations clipped per tile, with ids, tags and relation membership (topology preserved, unlike MVT) |
| `g2m.src.landcover` | raw | 1 km UTM tiles | uint8 classes, zstd | WorldCover, reprojected nearest-neighbour |
| `g2m.elev.base` | baked | L0 1 m ... L6 64 m, 256-cell tiles | int32 1/256 m, zstd | blended multi-source DEM on the g2m grid, before roads |
| `g2m.roads.graph` | baked | 1 km tiles + coarse hierarchy (motorway/trunk/primary) | own format, zstd | nodes, edges (reference line), attributes, lanes, restrictions, junction topology (6.2) |
| `g2m.roads.geom` | baked | 1 km tiles | own format | per-edge vertical profile and superelevation, junction polygons and heights, bridge decks (6.3) |
| `g2m.terrain.height` | baked | L0 1 m ... L6, 256-cell tiles (256² cell-centre samples, no shared borders; 4.9) | int32 1/256 m relative to tile datum, zstd | `elev.base` + road carving |
| `g2m.terrain.class` | baked | L0 1 m, L3 8 m | uint8 LandClass + uint8 SurfaceKind, zstd | per-cell classes |
| `g2m.features` | baked | 1 km tiles | own format | building footprints + heights + roof shape, water polygons, land-use polygons, point features (trees, signals) |

**Never served; derived on the client from the layers above:**
- render meshes (terrain chunks, road ribbons, buildings);
- vegetation instances;
- physics tiles (the bridge);
- minimap and full-map geometry;
- routing search state;
- all game-specific layers, unless deployed as server plugins (3.3).

### 3.3 Baked vs raw (owner decision D2: recommendation plus complexity)

| Criterion | Serve baked generic layers | Serve raw source layers, derive on client |
|---|---|---|
| Bandwidth | ~same: carved heights compress like the raw DEM; the road graph and geometry are about OSM-sized [estimate, measured in G3] | ~same |
| Client CPU | low: copy, mesh, scatter | high: road profile fitting, junction polygons and carving on the client, competing with 240 Hz physics on 12 cores |
| Server CPU / storage | derivation (on demand or pre-baked) plus derived storage | raw storage only |
| Determinism | identical bytes for all players by construction; the server is the authority | every client compiler must reproduce the server's bytes exactly (a larger CI matrix; any drift becomes a multiplayer desync) |
| Versioning | one release id pins sources and pipeline | same release id, plus the client's own deriver version must match |
| Cross-player cache hits | high: immutable URLs on a CDN | raw tiles cache equally well; derivation work is repeated per player |
| Improving derivation | redeploy the server; clients get it on the next release | ship a new game build |
| Game-specific layers | only via server plugins | natural: the client derives them |
| Offline and custom endpoints | needs a server with the layer (or the integrated server) | a raw-only server suffices |

**Recommendation:**
- Default to **baked generic layers**, with **raw layers also on the protocol and every built-in layer derivable locally.**
- **Per-layer resolver policy**, configurable per deployment:
  - Hosted default: `baked`.
  - Integrated server, online: `upstream-baked, else upstream-raw + local derive`.
  - Integrated server, offline: `cache, else local derive from cached raw, else degraded`.
  - Custom endpoint: whatever its manifest says it has.
- **Game-specific layers are derived on the client** over the served generic layers, through `IDeriver` registrations linked into the game.
  - If such a layer is expensive and identical for all players, the owner can deploy the same deriver as a server plugin (a static registration in a server build).
  - The hosted server then serves it baked under its vendor namespace. It is the same code, because the server and client derivers are one library.

**What supporting both costs:**
- **Code: small.** The client already links the whole derivation pipeline, because the integrated server is in-process. The resolver's per-layer fallback chain is roughly a few hundred lines [estimate].
- **Test matrix: the real cost.** Every compiler that ships a client (clang-cl, cl, Linux clang; later UE5's toolchains and consoles) must be hash-compared against the server's builds for every derived layer. That is CI time plus discipline (2.4).
- **Hosting:** raw layers must be served as well. Raw storage is needed anyway to derive, so the only extra is egress when clients use raw mode.
- **Licensing:** serving raw OSM-derived tiles is also publishing a Derivative Database under ODbL (3.9). This is the same obligation as for baked tiles, not an extra one.

**Cheaper alternative, if the owner wants less:** remote servers serve baked layers only; raw layers exist only between the integrated server and its own upstreams.
- Clients then never derive what a peer might download.
- The integrated server still derives locally, for offline and dev.
- Cost: no load shedding from the hosted server onto clients.

### 3.4 Protocol v1

This is plain HTTP GET, because it is engine-neutral, CDN-cacheable and trivial to implement for a custom server or a static bucket. Alternatives considered:
- **gRPC:** not cacheable by CDNs, needs HTTP/2 end-to-end and code generation, and its streaming is not needed.
- **WebSocket:** stateful, no caching.
- **Plain MVT/PMTiles:** loses road topology and restrictions (research: MVT drops topology).

Endpoints (all under `{base}/g2m/v1`):

```
GET /manifest
    JSON, Cache-Control: max-age=300. Server id, protocol versions supported, default release id,
    list of live releases, tile base URL (may point at a CDN host), auth mode, rate-limit policy, contact.
GET /releases/{rid}/manifest
    JSON, immutable. For the release:
      - sources: id, snapshot date, licence, attribution text, share-alike flag, coverage
      - layers: name, version, kind (raw|baked), tiling scheme, levels, encoding, inputs (for local derivation),
                deriver version hash, availability (pre-baked coverage | on-demand | unavailable)
      - coverage: per layer, a coarse bitmap per UTM zone (1 km tiles), or "global"
GET /releases/{rid}/tiles/{layer}/{scheme}/{level}/{x}/{y}
    One tile. Headers: ETag "<sha256 of body>", Cache-Control: public, max-age=31536000, immutable.
    404 = outside coverage (definitive for this release), 503 + Retry-After = on-demand derivation queued.
GET /releases/{rid}/packs/{layer}/{scheme}/{level}/{x}/{y}
    A pack of 16 x 16 tiles (one request, one CDN object) with a directory of per-tile hashes;
    used for region pinning/offline downloads and for pre-baked regions stored as static files.
GET /releases/{rid}/hashes/{layer}/{scheme}/{level}/{x}/{y}
    The per-tile hash list for one pack area (small), for verification without download (3.6).
```

- **Tile address.**
  - `scheme` = `utm{zone}{n|s}`, e.g. `utm32n`.
  - `x = floor(E / (256 m · 2^level))` and `y = floor(N / (256 m · 2^level))`, on the zone's false-easting grid (500 km E).
  - Levels are shared by all raster layers. 1 km vector tiles use `level = 2` (1024 m).
  - The client's cache key and the physics bridge use the same integers (4.4).
- **Tile container `g2m.tile/1`:**
  - header: magic, container version, layer name and version, release id, scheme/level/x/y, datum (heights), input content hashes, body hash;
  - typed sections, zstd-compressed.
  - Content-addressed: the ETag is the body hash.
- **Compression:** tiles are already zstd-compressed, so HTTP compression is off.
- **Compatibility:** a client accepts a release if it supports the container major version and each layer's encoding version. The manifest lists both.
- **Transport-level extras:** HTTP/2 or HTTP/3 through the CDN, invisible to the protocol.

### 3.5 Releases, pinning and versioning

- **Release id** `rid` = hash over:
  - the source snapshot index (each raw source tile's content hash);
  - each layer's deriver version and config hash;
  - the classifier tables;
  - the world seed.
- **A release is immutable.** A tile URL never changes content, which is what makes CDN caching and multiplayer sharing safe.
- **Source snapshots are content-addressed.** A new OSM snapshot changes only the raw tiles that changed; unchanged tiles are shared between releases. The hosted server can therefore keep several releases live cheaply, and derive old-release tiles on demand as long as their raw snapshot is retained.
- **The game pins a release per session.**
  - Single player: the game build's default release, or "latest release compatible with my g2m version".
  - Multiplayer: the host's release (3.6).
  - The derivation pipeline version is part of `rid`, so a server-side derivation fix is a new release, never a silent change.
- **Cadence:** see 5.5 and decision D11.
- **Retention:** the hosted server keeps the last N releases live (recommend N = 3). Older ones return 410 with the newest compatible `rid`.
- **Integrated server:** a local release id is computed the same way from whatever the user imported, so offline tiles are just as pinned.

### 3.6 Multiplayer and verification

- Peers agree on `rid` in the session handshake. Tiles from any server serving that `rid` are interchangeable, because they are content-addressed.
- A peer that derived a tile locally (integrated or offline) can verify it against `/hashes` when online.
- Recommended policy (decision D12): multiplayer uses server-baked or hash-verified tiles only. Unverified local derivations are single-player only. This removes the cross-compiler determinism risk from multiplayer.
- Physics determinism across peers additionally needs identical physics_sim versions and identical vehicle setups. The game handshake already has to cover those.

### 3.7 Hosted default server: architecture and operations

- **Static layer: pre-baked packs** as files in object storage behind a CDN. There is no compute on the request path, so this is where almost all traffic lands.
- **Dynamic layer:** `g2m_server` instances for on-demand derivation of areas not yet pre-baked.
  - Results are written back into object storage, so the next request is static.
  - A queue with priority: tiles near active players first, then prefetch along their heading.
- **Raw mirrors:** the server holds its own data (5.5).
  - Never proxy public OSM services: Overpass, tile.openstreetmap.org and Nominatim policies forbid game-client and bulk use (research, 5.2).
- **Auth, rate limiting and abuse:**
  - CDN-cached tile and pack reads: anonymous, with a per-IP rate limit at the CDN. Scraping baked tiles is acceptable; they are open data under ODbL anyway.
  - On-demand derivation (a CDN miss that reaches `g2m_server`): per-client API key embedded in the game build, free and revocable, plus per-key and per-IP compute quotas. `User-Agent` carries game and version.
  - Oversized, malformed or out-of-coverage requests are rejected before any derivation.
  - The server publishes its policy in `/manifest`, so custom endpoints can advertise theirs.
- **Privacy:** server logs contain IP addresses, and the owner is in Germany, so the GDPR applies. Needed: a privacy notice, short log retention (7 days recommended), no player-level analytics without consent (decision D15).
- **Tech choice: C++, sharing the libraries.**
  - `g2m_server` is `g2m_server` (lib) + `g2m_http` (cpp-httplib: MIT, header-only, thread-per-connection, adequate behind a reverse proxy).
  - TLS, HTTP/2/3 and edge rate limiting come from a reverse proxy/CDN (Caddy or Cloudflare; decision D3).
  - Rejected: a Go or Rust front end calling C++ through the C API. It would add a toolchain and a second language for little gain; the expensive part is derivation, which must be the shared C++ code anyway for bit-identical results.

### 3.8 Hosting estimates [estimate, to be replaced by measurements in G3/G7]

- **Baked layer size per km²:**

  | Content | Size per km² |
  |---|---|
  | `terrain.height` L0 on lidar DEM (1 M samples at ~4–7 bits after delta+zstd) | 0.4–0.9 MB |
  | Coarse levels | +0.1–0.3 MB |
  | `terrain.class` | 0.02–0.06 MB |
  | Roads and features | 0.02–0.1 MB (urban higher) |
  | **Total, lidar areas** | **≈ 0.5–1.1 MB** |
  | **Total, GLO-30-only areas** (upsampled, smooth, compresses far better) | **≈ 0.1–0.3 MB** |

- **Per region:**

  | Region | Area | Baked size |
  |---|---|---|
  | Home region (~1,000 km² incl. margin, 5.4) | ~1,000 km² | 0.5–1.1 GB |
  | Hessen | ~21,100 km² | 11–23 GB |
  | Germany | ~357,600 km² | 180–400 GB |
  | Planet land at GLO-30 level | ~149 M km² | 15–45 TB (hence on-demand beyond pre-baked countries) |

- **Raw mirrors:**
  - German DGM1 as delivered: 1.2–2.7 MB/km² (NRW's float LZW tiles, research), about 0.4–1 TB; re-encoded as `g2m.src.elev`, 150–320 GB.
  - OSM planet PBF: about 80–90 GB [unverified].
  - WorldCover: 124 GB (research).
  - Mapterhorn planet z0–12: about 120 GB (research).
  - GLO-30: range-read from AWS rather than mirrored at first.
- **Egress per player-hour.**
  - Driving new roads at 60 km/h:
    - L0 band ±1 km: 120 km²/h × 0.45–0.9 MB = 55–110 MB;
    - far field at 8 m out to 20 km: about 20–35 MB;
    - vectors ±3 km: about 10–35 MB;
    - total ≈ **0.1–0.2 GB/h**.
  - Motorway or hypercar speeds: ≈ 0.2–0.5 GB/h.
  - Revisited areas: ≈ 0 (client cache).
  - Requests: about 3–6 k tiles per new player-hour.
- **Cost.**
  - Cloudflare R2: $0.015 per GB-month, zero egress (research). Germany baked: about $3–6 per month in storage.
  - Operations are billed per million reads [price unverified]; at ~5 k reads per player-hour, pennies per thousand player-hours.
  - A classic CDN at about $0.08/GB [unverified] would cost roughly $0.01–0.04 per player-hour.
- **Derivation CPU:** assume 0.5–3 CPU-seconds per km² (dominated by road profile fitting and carving).
  - Pre-baking Germany: 50–300 CPU-hours, a few hours on one 32-core machine.
  - Europe: 1.5–9 k CPU-hours.
  - Planet: 20–125 k CPU-hours, hence on demand plus pre-baking popular regions.
  - One core derives ≥ 1 km² per few seconds, while a hypercar needs about 3 km² per minute of new L0. A single on-demand VM therefore serves tens of simultaneous explorers in un-baked areas. Beyond that, the resolver sheds load to client-side derivation from raw layers (3.3).

### 3.9 Licensing obligations of serving (the owner must act; this is my reading, not legal advice)

- **OSM (ODbL 1.0).**
  - Baked and raw OSM-derived tiles are a Derivative Database that is publicly used.
  - The owner must:
    - offer it under ODbL, or offer the alteration method (ODbL 4.6);
    - attribute "© OpenStreetMap contributors" with a link to the licence.
  - Rendered meshes and screenshots are likely Produced Works: attribution only (OSMF Produced Work guideline).
  - Practical plan:
    - publish each release's OSM-derived packs as a download under ODbL;
    - keep `g2m_tiler` and the derivers reproducible;
    - consider open-sourcing geo2map_engine, which is the simplest way to satisfy "method" (decision D7).
  - ODbL covers the data, not the game code or art.
- **Copernicus DEM:** attribution text for modified data ("produced using Copernicus WorldDEM-30 ..." per the licence; exact wording from the licence PDF [unverified: the PDF did not parse]).
- **ESA WorldCover (CC BY 4.0):** attribution plus indication of changes.
- **Hessen DGM1/LoD2 (dl-de/zero-2-0):** no conditions. If credited ("ATKIS-DGM1 © HVBG"), modifications must be marked.
- **Other German states:** dl-de/by-2-0 or CC BY 4.0 require attribution. The manifest carries it per source.
- **Mapterhorn:** per-source licences in its `attribution.json`.
- **Custom endpoints** are responsible for their own data. The client simply displays whatever attributions their manifest lists.


### 3.10 G1a as built: deviations from 2.2, 3.2, 3.4 and 7.2 (2026-09-25)

The normative byte formats are in `docs/formats/`. The design records are
in the vault.

- **Raw tile keys.** All three raw layers use level-2 keys (1024 m) on the
  g2m UTM grid, not the source's own tiling (3.2 said "DGM1 1 km tiles").
  Each `src.elev` tile carries the source's sample lattice in its georef
  block, so DGM1 stays a 1 m cell-registered raster. The G1c tiler cuts
  sources into these keys, and the data is never resampled.
- **`src.elev` encoding.** Instead of 3.2's plain "row delta", each sample is
  predicted from its neighbours (`Gradient`, planar
  `west + north - northwest`, is the default; `Left` = row delta). The
  residual is zigzag-coded, byte-plane shuffled, then zstd. NoData goes in a
  separate bitmap section.
- **Protocol statuses (in-process).** Besides 404/410/503, the server also
  answers:
  - 304 (If-None-Match equal to the body hash);
  - 400 (malformed request);
  - 500 (store I/O or deriver error).

  Offline without a cached tile gives 503 with `degraded` set and no
  Retry-After. G1 derives on-demand tiles inline, inside the request, so a
  503 with Retry-After only means a transient failure. The queued "503 +
  Retry-After = derivation queued" of 3.4 arrives with the hosted server.
  Packs answer 200 / 404 / 503, with a state per entry (NotFound, Present,
  Unavailable, Degraded).
  (See 3.11 for G1c's `g2m_tiler`/`g2m_import`, which populate the source
  store this section's resolver reads from.)
- **Tile rid.** A tile response's container always carries the requested
  release's rid, even when the body came from the cache or an upstream
  under another release. Bodies are shared, headers are per release.
- **Header locations.** `IFetcher` is in `include/g2m/server/fetcher.h`,
  not `client/`: the server's network upstreams use it, and the client will
  too. `ServerConfig` is in `server.h`; there is no `config.h`. The protocol
  objects are in `server/protocol.h`. The protocol paths and statuses are
  documented in `docs/formats/protocol.md`; the full spec
  (`docs/protocol/`) comes with G6.
- **Cache schema (7.2).** One schema serves the client cache and the
  server's stores:
  - `tiles(scope, layer, key, hash, header, last_used, pinned)`, where
    `key` is `TileKey::packed()` and the scope is the rid in hex for the
    cache and derived store, or the snapshot name for the source store;
  - `bodies(hash, size, refs, data)`, which holds each body once
    (deduplicated), in the database or, over 256 KiB, in a blob file;
  - the `sources` column is gone, because a tile's provenance is its
    header's input hashes;
  - the LRU clock is a counter in the database, not wall time.

  A damaged body drops every entry that shares it.
- **Fuzzing directory.** The fuzz targets for these formats are in
  `tests/fuzz_formats/`, because `tests/fuzz/` belongs to G1b's decoders.
  They run on the `linux-release` and `linux-asan-ubsan` legs, not on
  `linux-tsan`.

### 3.11 G1c as built: deviations from 2.2, 5.4 and 5.5 (2026-09-26)

Seven increments (I1-I7): exact polygon geometry and the region format
(I1); the `g2m_import` library skeleton, `apps/g2m_tiler` and the region
resolver (I2); nested-zip access, the DGM1 catalogue and the DEM importer
(I3); `OsmClipIndex` and the OSM importer (I4); the WorldCover importer
(I5); the G1c golden, fuzz seeds and end-to-end docs (I6); the real home
region import, this section, and marking 5.5 done (I7). The design
records (D1-D24) are in the vault (G2M-005); this section is deviations
and measured numbers only.

- **Repository layout.** `g2m_import` (`include/g2m/import/`,
  `src/import/`) is a new library 2.2/2.3 did not anticipate - region
  resolution, the tile-write seam (`ITileSink`/`StoreSink`/`MemorySink`),
  the ordered-parallel driver and the three importers (dem/osm/landcover),
  kept out of `g2m_layers`/`g2m_server` on purpose (D22) so importers never
  pull in server/decode-thread concerns. `include/g2m/core/geom/` (exact
  integer predicates, ring assembly) and `include/g2m/layer/region.h` are
  likewise new. Of 2.2's planned `apps/`, only `apps/g2m_tiler` is built;
  `g2m_server` and `g2m_cli` remain unbuilt placeholders - everything 2.3
  assigned to `g2m_cli` ("derive/dump/hash tiles, run golden checks,
  pack/prebake a region") that G1c needed lives in `g2m_tiler`'s own
  subcommands instead (`stats`, `index-hash`).
- **Geometry (D1, D2).** Polygon assembly, point-in-polygon and
  tile/segment/distance tests are our own exact-integer code
  (`core/geom/exact.h`, `ring_set.h`, `ring_assembly.h`), not Clipper2
  (double/int64, missing predicates), Boost.Geometry (heavy) or osmium's
  area assembler (heavy, version-dependent) - it only needed the handful
  of predicates `osm_clip.cpp` already had, generalised. Ring nesting is
  even-odd depth, not trusted OSM member roles (D2): robust to real role
  errors, which are reported (`role_mismatches`) rather than trusted. The
  three real home relations resolve with `role_mismatches == 0` (a hard
  CHECK since I3), but the mismatch-tolerant path is what fixture tests
  exercise.
- **Region identity and Höchst (D6, D7).** `g2m.regions/1` identifies an
  `osm_admin` part by relation id, with `expect_name`/`expect_admin_level`
  as a drift check, not a name lookup; unknown JSON keys are rejected
  (D5, a hand-written config). Frankfurt-Höchst is the Stadtteil relation
  4209939 (admin_level 10, ~5 km², matching 5.4's "~5 km²" estimate), not
  the Ortsbezirk 6 "West" relation 11042428 (admin_level 9, ~30 km²,
  covering nine Stadtteile) - the Stadtteil adds only 56 tiles with the
  2 km halo.
- **`regions.json`'s location.** 5.5 step 3 names a repo-local
  `data/world/regions.json`; geo2map_engine has no such directory for
  game-world content, and the coordinator owns `racing_game` too, so I7
  commits `S:\claude_code\racing_game\data\world\regions.json` there
  instead (`racing_game/CLAUDE.md` gained one Layout line for it).
- **CLI shape (deviation from 5.5 step 3's literal command).** The step-3
  text `g2m_tiler import --region data/world/regions.json#home` does not
  match the CLI as built: the region selector is a positional `PATH#NAME`
  argument, there is no `--region` flag, and there is no `$G2M_DATA_DIR`-
  style default data directory - every source path is explicit. I7's
  actual invocation (paths shortened):
  ```
  g2m_tiler import "racing_game/data/world/regions.json#home" \
    --pbf ".../hessen-latest.osm.pbf" --store ".../geo2map_cache/home" \
    --scope home-2026-09 --dem-data ".../dgm1" --tif ".../ESA_WorldCover_...tif" \
    --jobs 12 --pin
  ```
  This also means `g2m_tiler import` writes all three raw layers
  (elevation, OSM, landcover) in one pinned pass; 5.5's own text has
  landcover arrive later, lazily, via range-read derivation (step 4) -
  G1c's offline tiler eagerly imports a local WorldCover COG instead,
  since the hosted/integrated server's range-read path does not exist yet.
- **Pin flag (I2-I6 said "add when needed").** I2-I6 hardcoded
  `StoreSink`'s `pin` to `false`. I7 adds a `--pin` boolean CLI switch
  (default off) to `import`/`import-dem`/`import-osm`/`import-landcover`,
  plumbed straight to the existing `StoreSink(store, scope, pin)`
  constructor from I6. The argv parser moved out of `main.cpp` into a new
  header-only `apps/g2m_tiler/cli_args.h` (so `tests/unit/
  test_tiler_cli_args.cpp` can include it directly, no second CMake
  target) and gained an opt-in `bool_flags` list, making a named flag a
  presence-only switch instead of consuming the next token as its value;
  every pre-existing value-flag's behaviour is unchanged (regression-
  tested). Two more I7 additions: `g2m_tiler stats` now also runs and
  reports `check_consistency` (previously a separate, unexposed call);
  a new `index-hash --store DIR --scope SCOPE` subcommand prints a
  per-layer and overall FNV-1a64 over the store's `(layer, key,
  body_hash)` index in its own sorted order, for the `--jobs`-independence
  check below.
- **Quantisation (Q3, unresolved).** Both quantisers are kept: the
  pre-existing `quantise_height` (`llround`, ties away from zero) is
  untouched everywhere else; the DGM1 importer uses the new half-even
  `quantise_height_half_even` (ties to even, rounding-mode independent).
  Unifying the two was explicitly deferred to a later increment - **open
  item**, not attempted in G1c.
- **OSM (D13, D14, D15).** `extract_region_set` builds its own sorted
  structure-of-arrays node-location index (16 B/node, ~0.6 GB for all of
  Hessen) instead of osmium's `NodeLocationsForWays` - deterministic and
  measurable, but does not scale to all of Germany (revisit in G8).
  `OsmClipIndex` is a precomputed per-tile-clip index equivalence-tested
  byte-for-byte against `clip_osm` (kept as the oracle), roughly 1000x
  less work than calling `clip_osm` per tile. Incomplete ways (an
  unresolvable node ref) are dropped and counted (`ways_incomplete`),
  never repaired.
- **Landcover fixtures.** Two committed WorldCover fixture TIFFs, not one
  (`worldcover/fixture_N50E008_hoechst.tif`, `..._bbox.tif`) - the fixture
  region's two disjoint shapes (a Höchst-relation polygon and a separate
  `utm_bbox` part, ~11 km apart) do not both fit inside one realistically
  sized ESA WorldCover crop.
- **Fixtures and fuzz seeds (D19-D21).** Real, small, committed cuts (a
  test-only `g2m_fixture_cut` tool, never shipped) rather than synthetic
  data alone: 6 fixture files, 7,855,376 bytes, under the 8 MiB test
  budget; committed fuzz seed corpora for zip/tiff/pbf
  (`testdata/fuzz_seeds/`).
- **G1c golden.** `tests/golden/golden_g1c.cpp` /
  `testdata/golden/g1c_golden.txt`: `all 122 e7947b8adb8e5b47`, identical
  on all 7 CI legs since I6.

**I7 measured numbers** (home region: Main-Taunus-Kreis + Hochtaunuskreis
+ Frankfurt-Höchst, halo 2000 m; release build, `--jobs 12`, real data
under `S:\claude_code\geo2map_data` (read-only), store
`S:\claude_code\geo2map_cache\home`, scope `home-2026-09`):

| Measurement | Expected (I7 plan) | Measured | Note |
|---|---|---|---|
| Tile count | 1085 ± 15 | **1085** | exact |
| Interior tiles | >= 520 | **533** (51 interior runs) | matches I2/I3's own dress-rehearsal figure exactly |
| DEM full / partial / no-coverage | ~900 / ~100 / ~85 | **850 / 150 / 85** | no-coverage exact; full/partial split off by 50 tiles each (~4.6%) - see below |
| DEM NoData in interior tiles | 0 | **0** | `tiles_skipped_allnodata=0` (no tile was ALL NoData) - but the stronger claim this row was read as, "interior tiles all have full km-tile coverage", was never actually checked (neither `stats` nor `index-hash` decodes a tile body) and was **false**: see the "G2.0 correction" addendum below - 29 of the 533 interior tiles carried a real ~97.7%-NoData stripe. |
| DEM duplicate keys / needed km tiles | ~412 keys | **412 keys**, `km_decoded=1176` | both match the plan's measured inputs exactly (412 needed keys duplicated; home needs 1176 km tiles) |
| DEM dup_copies / dup_identical / dup_differing | "~1490 copies", dup_differing reported | **490 extra copies** (902 total copies over the 412 dup keys), **396 identical**, **16 differing** | differing copies are logged by name (PKT source + inner-zip entry); see Deviations below for the "1490" reconciliation |
| OSM ways_incomplete | 0 | **0** | matches I4 exactly |
| OSM nodes/ways/relations | (I4 baseline) | 31,958,253 total / 2,310,058 in / 439,145 ways / 8,448 relations | bit-for-bit same as I4's dress rehearsal |
| Landcover tiles written | ~1085 | **1085** | `tiles_skipped_allzero=0` |
| Peak RSS (every importer) | < 4 GB | **1873 MiB** combined run; 1433 MiB (DEM alone), 1864 MiB (OSM alone) | well under bound |
| Wall time (jobs 12, total) | < 5 min | **126.96 s** (~2.1 min) | resolve 10.7 s, dem 58.3 s, osm 55.1 s, landcover 2.9 s |
| Store size: DEM / OSM / landcover | 0.5-1.1 GB / 30-100 MB / 5-30 MB | **689.6 MB / 80.2 MB / 1.42 MB** | DEM and OSM in range; **landcover below range** - see below |
| `check_consistency` | OK | **OK** | |
| All tiles pinned | yes | **yes** (`pinned_tiles=3170 == tiles=3170`) | |
| `--jobs 1` vs `--jobs 12` index-hash | identical | **identical** (all layers and the overall `all` hash) | see `index-hash` below |

Two measurements needed a short investigation (not threshold-tuning):

- **DEM full/partial split (850/150 vs "~900/~100").** The combined
  covered count (1000) and the no-coverage count (85) both match the plan
  exactly; only the split between "every km tile present" (full) and
  "some km tiles missing" (partial) shifted by 50 tiles either way. The
  home region resolves against real, irregular admin-boundary geometry
  (not a synthetic bounding box), which puts more L2 tiles astride the
  2 km-halo edge - where a km tile is more likely to be missing from the
  DGM1 catalogue's coverage - than the plan's estimate assumed. Not
  investigated further: both halves of Q2's ruling (accept the halo gaps
  until GLO-30 fills them in G2) already cover this range either way.
- **Landcover store size (1.42 MB vs the planned 5-30 MB).** Every
  landcover tile got its own body (`bodies=1085` for `tiles=1085` - no
  cross-tile deduplication happened), so the small size is compression,
  not dedup. A level-2 tile is 1024 m at WorldCover's 10 m pixel size, so
  roughly 100x100 = 10,000 one-byte class cells (~10 KB raw) per tile,
  ~11 MB raw for 1085 tiles - in the planned range. The real Rhein-Main
  landscape (large homogeneous farmland/forest/built-up patches) is
  evidently compressing at roughly 8x under the store's generic body
  compression, landing well under the plan's estimate. Not a bug, and not
  investigated further - a data-dependent compression ratio, consistent
  with DEM (continuous, less compressible terrain data landing at the
  upper part of its own range) and OSM (near the upper end of its range
  too) behaving as expected.
- Reconciling "~1490 copies": the plan's own shorthand almost certainly
  meant "1000 available km keys, realized as 1490 total copies when
  duplicates are summed" (1000 + 490 extra = 1490) rather than
  `dup_copies` alone (490, the count of *extra* copies beyond the first);
  `dup_keys=412` matches either reading exactly. Not chased further since
  neither reading changes any pass/fail outcome.

**`index-hash` determinism check (Step 3).** A second import, same
inputs, `--jobs 1` (473.9 s wall, vs. 127.0 s at `--jobs 12` - single-
threaded DEM decode dominates), into a scratch store under
`geo2map_engine/out/scratch_home_jobs1` (deleted immediately after this
comparison; store stats identical too: `tiles=3170 body_bytes=771279516
pinned_tiles=3170`, `check_consistency OK`):

```
                     jobs=12 and jobs=1 (identical):
g2m.src.elev.dgm1 1000 ac05f9dc40e9c159
g2m.src.landcover 1085 358b31f4156733cd
g2m.src.osm       1085 788472aac3d7543d
all               3170 3c62cd447a6082e1
```

Every layer hash and the overall `all` hash matched exactly between the
two runs - the importer's output is `--jobs`-independent, as designed
(`ordered_for`, D12).

**Open items.**
- Unify (or formally keep) the two height quantisers (Q3) - undecided.
- The DGM1 halo gaps (85 no-coverage L2 tiles, 99 partial as of the G2.0
  fix below, genuinely all in the 2 km halo - see the "G2.0 correction"
  addendum) are accepted until GLO-30 fills them in G2 (Q2) - not fixed
  here.
- 5.5 steps 4-5 (GLO-30/WorldCover on-demand derivation via range-reads,
  `g2m_cli prebake`) and `apps/g2m_server`/`apps/g2m_cli` themselves remain
  unbuilt; only step 3 (the tiler import) is done - see the 5.5 annotation
  below.
- `g2m_fixture_cut` (a maintainer-only tool, never CTest-run) still
  catches an exception by reference and calls `.what()` on it (TOOL-019
  hazard) - inert because it never runs under ASan in CI, left as-is per
  I6's own notes.

**G2.0 correction (2026-09-26): the "interior tiles have full coverage"
claim above was false.** A level-2 tile (1024 m) can span THREE DGM1 1 km
squares on an axis, not just two, whenever its west/south edge falls in
km-remainder 977..999 - the DEM importer (`src/import/dem.cpp`) computed
the correct min/max square span (`e0`/`e1`, unchanged by this fix) but only
ever fetched the 2x2 box's 4 CORNER squares, silently skipping the middle
square for every 3-square-span tile. Those corner squares each only
overlap a thin sliver (1-23 columns/rows) of the destination raster at the
tile's own edges - the middle square supplies essentially the entire tile
- so an affected tile came out ~97.7% NoData (1,024,000/1,048,576 samples)
while nothing checked or reported that: `stats`'s `check_consistency` only
checks ref counts and blob presence, and `index-hash` fingerprints
`(layer, key, body_hash)` - neither decodes a tile body or counts NoData
samples, so the "0 NoData" measurement above never actually looked.

Fixed via `dgm1_km_span()`/`KmSpan` (`include/g2m/import/dem.h`,
`src/import/dem.cpp`): the importer now visits the full `[lo, hi] x [lo,
hi]` rectangle, up to 3x3 = 9 squares, instead of the 2x2 box's 4 corners.
Two new regression tools guard against this class of bug recurring
silently: `ResolvedRegion::interior_tile_keys()`
(`include/g2m/layer/region.h`) plus `g2m_tiler check-dem-nodata --store DIR
--scope SCOPE --resolved PATH [--interior-only]`, which decodes every real
`g2m.src.elev.dgm1` body and counts actual NoData samples (exit 1 on any
interior-tile NoData); and `g2m_tiler diff-scopes --a DIR:SCOPE --b
DIR:SCOPE`, a per-tile complement to `index-hash` (two differing hashes
say only that something changed, never what) that merge-diffs two stores'/
scopes' sorted `TileStore::index()` output into per-layer added/removed/
changed/unchanged counts.

Confirmed against the real existing home store (`S:\claude_code\geo2map_cache\home`,
scope `home-2026-09`, read-only - never modified by this investigation):
`check-dem-nodata --interior-only` found exactly **29 of the 533 interior
tiles** affected (x=446-452 at y=5416, plus x=458 at y=5424 and
y=5427-5447), each 1,024,000/1,048,576 NoData samples
(`total_nodata_samples=29,696,000`, exactly 29 x 1,024,000) - precisely the
bug's predicted signature.

**Re-import with the fix** (same inputs/region/`--jobs 12`, new store
`S:\claude_code\geo2map_cache\home-r1`, scope `home-2026-09r1` - the old
store above was left untouched):

| Measurement | Old (buggy) | New (fixed) | Note |
|---|---|---|---|
| DEM full / partial / no-coverage | 850 / 150 / 85 | **901 / 99 / 85** | no-coverage unchanged (a square's coverage existence doesn't depend on this fix); 51 tiles moved partial -> full - their middle square *was* covered all along, just never fetched |
| DEM NoData in interior tiles (`check-dem-nodata --interior-only`) | **29 tiles**, 29,696,000 samples - FAILED | **0 tiles**, 0 samples (533 checked, 558,891,008 total samples) - OK | the fix's whole point; now a regression-guarded pass/fail |
| `diff-scopes` old vs. new | - | `g2m.src.elev.dgm1 added=0 removed=0 changed=51 unchanged=949`; landcover/osm both `changed=0` (untouched by this fix); `total changed=51` | the 51 changed DEM tiles are exactly the full/partial-split delta above, and are a superset of the 29 visibly-NoData ones (all lie along the same x=441-456@y=5416 row and x=458@y=5417-5451 column - a DGM1 km-grid seam) |
| `body_bytes` | 771,279,516 | **807,912,301** (+36.6 MB) | real height data replacing former NoData-stamped bytes in the 51 changed tiles |
| Wall time (jobs 12, total) | 126.96 s | **131.18 s** | dem sub-step 58.3 s -> 61.8 s (visiting up to 9 vs. 4 km squares); resolve/osm/landcover unchanged |
| Peak RSS | 1873 MiB | **1872 MiB** | unchanged |
| `stats` | `tiles=3170 pinned_tiles=3170` OK | `tiles=3170 pinned_tiles=3170` OK | unchanged shape |
| `index-hash` (jobs 12) | (superseded) | `g2m.src.elev.dgm1 1000 76f8296375013744` / `g2m.src.landcover 1085 358b31f4156733cd` / `g2m.src.osm 1085 788472aac3d7543d` / `all 3170 ecd52efb1294e764` | landcover/osm hashes identical to the old store (as `diff-scopes` also shows), confirming the fix touches DEM only |
| `--jobs 1` vs `--jobs 12` index-hash | - | **identical** (all layers and `all`) | second import, same inputs, `--jobs 1` (486.2 s wall vs. 131.2 s at `--jobs 12` - single-threaded DEM decode dominates), into a scratch store under `geo2map_engine/out/scratch_home_jobs1` (deleted immediately after this comparison); `stats` identical too (`tiles=3170 body_bytes=807912301 pinned_tiles=3170`, `check_consistency OK`; report's own `tiles_partial=99` matches); every layer hash and `all` bit-identical to the jobs-12 run above - the fix did not disturb `ordered_for`'s `--jobs`-independence (D12) |

Spot-checked 3 of the originally-affected keys (`utm32n/2/446/5416`,
`utm32n/2/458/5424`, `utm32n/2/458/5440`) individually against
`diff-scopes`' per-tile output (all three `changed`) and against the new
store's `check-dem-nodata --interior-only` result (0 tiles with NoData
across all 533 interior tiles, so these three specifically now read
0/1,048,576 NoData) - consistent with the aggregate numbers above.

---

## 4. Geodesy, frames, floating origin

1. **Grid.** UTM on GRS80/WGS84; the metre-level ETRS89–WGS84 offset is ignored.
   - Transverse Mercator via Krüger's 6th-order series (Karney 2011).
   - Error ≤ 5 nm within 35° of the central meridian; do not use beyond ~75° (GeographicLib docs).
   - Implemented in-house on the deterministic math, and cross-checked against GeographicLib reference values (test data only; GeographicLib itself calls platform libm, so it is not deterministic).
2. **Session frame.**
   - Chosen from the spawn point's zone. Home region: zone 32N (EPSG:25832).
   - Physics x = grid E − E0; y = grid N − N0; z = orthometric height. E0/N0 are multiples of 1 km.
   - The world is flat in this frame (PLAN.md P11's flat shape), with constant −z gravity.
3. **Why UTM.**
   - It is the official grid of all German geodata (DGM1, LoD2, ALKIS), so there is no resampling at home.
   - Stable, cacheable tile keys.
   - Costs:
     - scale about 0.9996 near the central meridian (≈ 0.99962 in the region, about 0.5° west of 9° E [estimate]), i.e. about 3.8 m short per 10 km, invisible and self-consistent;
     - grid north differs from true north by about 0.4° [estimate]; the compass and map correct for it.
4. **Keys shared everywhere.**
   - g2m tiles, the client cache, the server URLs and the bridge's `TileKey.packed` all derive from integer grid metres.
   - Data tiles (L0, 256 m) are **cell-registered**, exactly like Hessen DGM1: 256×256 samples at half-metre cell centres (E + 0.5, N + 0.5).
   - Physics tiles (255 m, 256 samples, 1 m spacing, node-registered with shared borders, as physics_sim needs) have origins at **half-metre** positions, so every physics sample lies on a data-tile sample. `fill_tile` is therefore an integer-offset copy (`lattice_offset`) from up to 4 g2m tiles, never an interpolation. (Ruling 2026-09-25, see 4.9 and vault G2M-003.)
5. **Zone boundaries.**
   - A session keeps its zone. Tiles beyond the boundary are derived in the extended zone, where the series stays accurate far outside the 6° strip, and cached under that zone's key.
   - Sessions more than about 1,500 km from their central meridian are out of scope; start a new session.
6. **Curvature.**
   - Ignored in the flat shape: about 196 m drop at 50 km is not represented.
   - The WGS84 curved world shape (physics R8) can come later without changing keys; g2m already returns ECEF for any grid point.
7. **Vertical datums.**
   - DGM1 uses DHHN2016; Copernicus uses EGM2008.
   - At coverage seams the base DEM removes the local median offset before blending (6.1).
8. **Precision.**
   - Physics uses double positions (Jolt `DOUBLE_PRECISION`).
   - Rendering uses the existing origin rebasing (`origin_rebase.h`, 500 m threshold; 8.6 µm error at 50 km per PHYS-011).
   - Every g2m client item carries a double origin, with float vertices relative to it.
   - Official Godot builds are single precision, and double builds need a custom engine build (research), so floating origin is mandatory.
9. **Rulings after G0 (2026-09-25, vault G2M-003).**
   - **Round-trip target.** Geodetic → TM → geodetic round trip ≤ 5 nm (was < 1 nm). Measured: 3.16 nm for g2m, 3.17 nm for GeographicLib itself in double. 1 nm is out of reach for a 6th-order series in double, so the `[!mayfail]` 1 nm test is removed.
   - **Sample lattice.** Data tiles are cell-registered (the native DGM1 lattice, no resampling). Physics tiles are node-registered, 255 m / 256 samples, anchored on the half-metre cell centres (see 4.4).
   - Rejected alternatives:
     - resampling to integer-metre nodes: it adds interpolation error, and 257 samples is odd, which Jolt rejects;
     - 258-sample 256 m physics tiles: not a 1 m spacing.

---

## 5. Data sources and acquisition

### 5.1 Elevation

| Source | Licence | Access | Resolution / CRS | Use |
|---|---|---|---|---|
| **Hessen DGM1** (HVBG) | dl-de/zero-2-0 (open since 1 Feb 2022); no attribution required | Download-Center gds.hessen.de, by Kreis/Gemeinde, ZIP of GeoTIFFs. JS portal; no WCS/ATOM found, so manual download. Tile size probably 1×1 km [unverified] | 1 m, float32 LZW, NoData −9999, EPSG:25832 + DHHN2016, ±0.3 m (95 %), 6-year update cycle | home region, primary |
| Other German states' DGM1 | dl-de/zero (HE, NW, BE), dl-de/by-2-0 (BW, BB, HH, RP, SL, SN, ST, TH), CC BY 4.0 (BY, HB, MV, NI, SH) | varies. NRW: plain HTTPS directory of 1 km tiles (automatable); Bavaria open since 2023 | 1 m | Germany expansion (G8) |
| **Copernicus GLO-30** | Copernicus DEM licence: free, attribution; commercial use believed allowed [unverified wording] | AWS `copernicus-dem-30m`, 1° COGs, no account, range reads; 2021 release, no planned updates; N50E008 covers most of the region | 1″, EPSG:4326 + EGM2008. **A DSM**: forests appear as 20–30 m plateaus | worldwide fallback, margins |
| Mapterhorn | code BSD-3; data per source (553 sources incl. Hessen DGM1, all German states, GLO-30 elsewhere) | terrarium WebP 512 px; `tiles.mapterhorn.com`; PMTiles planet z0–12 ~120 GB, regional z13–17; volunteer service with no usage policy | Web Mercator | server-side mirror/extract option for worldwide DEM (decision D13); never hotlinked by clients |
| AWS Terrain Tiles (Tilezen) | mixed attribution | terrarium PNG to z15 on S3 | effectively SRTM ~30 m in Germany | not used (Copernicus/Mapterhorn are better) |
| EU-DEM | discontinued | — | — | not used |
| FABDEM (bare-earth GLO-30) | non-commercial [unverified] | — | 30 m | not used unless the licence allows |
| MapTiler terrain-rgb | proprietary, API key | — | — | not used |

### 5.2 Vectors (OSM and others)

- **OSM via Geofabrik extracts.** `hessen-latest.osm.pbf` is 329 MB, updated daily, ODbL.
  - There is no Regierungsbezirk extract; the region is cut by the tiler.
  - Geofabrik advises modest download volumes, so it suits the pipeline, not clients.
- **Forbidden for game clients:** Overpass, tile.openstreetmap.org, Nominatim (OSMF policies).
- **Overture Maps** (ODbL for transportation): explicit segment/connector topology, speed limits, prohibited transitions, surface, width, but no lanes. GeoParquet on S3, monthly releases.
  - A candidate second input for the worldwide pipeline (G8).
  - Parquet is converted offline by the tiler, never read at runtime.
- **Protomaps / OpenMapTiles:** road layers lack lanes/maxspeed/surface, and MVT loses topology. Not used for the road graph; possibly for a far-field basemap later.

### 5.3 Land cover and buildings

- **ESA WorldCover 10 m** (CC BY 4.0).
  - Bucket `esa-worldcover`: 3×3° COGs, about 47 MB each on average.
  - Tile N48E006 covers the region.
  - 11 classes (tree cover, shrubland, grassland, cropland, built-up, bare, snow, water, wetland, mangroves, moss).
  - Versions: 2020 v100 and 2021 v200; static.
- **Buildings:** OSM footprints.
  - Hessen LoD2 CityGML 1.0 (dl-de/zero, EPSG:25832 + DHHN2016, standardised roofs, annual updates) is an optional later upstream for exact roofs in Hessen [size per Kreis unverified].

### 5.4 Home region: area and first-download estimate

- **Area.**
  - Main-Taunus-Kreis ~222 km², Hochtaunuskreis ~482 km², Frankfurt-Höchst ~5 km² [unverified, from memory].
  - With a 2 km margin: about **1,000 km²**. The bounding box is about 1,800 km².
  - Höchst's DGM1 comes in the Frankfurt am Main package (~248 km² [unverified]).
  - District boundaries come from OSM admin boundaries (`admin_level=6`) in the same extract, so no extra source is needed.
- **Downloads (one-off, integrated server raw store):**
  - DGM1 for MTK + HTK + Frankfurt: ~950 km² × 1.2–2.7 MB = **1.1–2.6 GB** as delivered (by NRW analogy). Re-encoded: 0.4–0.9 GB.
  - Geofabrik Hessen PBF: 329 MB. Region cut: ~20–40 MB [estimate].
  - GLO-30 N50E008 (+ N49E008 if the southern margin crosses 50° N): 25–40 MB each [unverified].
  - WorldCover N48E006 v200 map: ~47 MB average [research estimate].
- **Client cache for the pinned home region (baked):** 0.5–1.1 GB. The source cache adds 0.5–1 GB when running integrated.

### 5.5 How each data type is obtained, per deployment

Principles:
- Bulk files are converted by our own offline tiler.
- Cloud-optimised public files (COGs on AWS open data) are range-read directly.
- No live queries against OSM services.
- Every source enters a content-addressed raw store with its licence and attribution attached.

**Elevation (licences: dl-de/zero for HE DGM1; Copernicus DEM licence; others per 5.1).**
- **Development now:**
  - HVBG DGM1: manual Kreis ZIP download (MTK, HTK, Frankfurt), then `g2m_tiler import-dem`.
  - Margins: GLO-30 COG range reads from AWS.
- **Integrated server (shipped game):** upstream order:
  1. cache;
  2. the hosted server (baked, else raw);
  3. direct GLO-30 COG range reads;
  4. anything the user imported with the tiler.
- **Hosted server (later):**
  - own mirror of national 1 m DGMs (NRW scripted over HTTPS; other states per portal);
  - a Mapterhorn extract (D13);
  - GLO-30.
- **Cadence:** DGM1 on a 6-year cycle (check yearly); GLO-30 static (2021).

**Roads and OSM vectors (licence: ODbL 1.0).**
- **Development now:** Geofabrik `hessen-latest.osm.pbf` (329 MB), downloaded once. Then `g2m_tiler import-osm` cuts the districts + 2 km.
- **Integrated server (shipped game):**
  - from the hosted server (baked, or raw `src.osm`);
  - offline or custom: the user imports a Geofabrik extract via `g2m_cli import`;
  - never Overpass or the OSM API.
- **Hosted server (later):**
  - Geofabrik country extracts or planet PBF, converted by the tiler;
  - optionally Overture GeoParquet converted offline.
- **Cadence:** quarterly releases (D11). Geofabrik updates daily, but we do not need that.

**Land cover (licence: CC BY 4.0).**
- **Development now:** WorldCover N48E006 v200 by range read (~47 MB file), plus OSM landuse.
- **Integrated server:** direct COG range reads, or via the hosted server.
- **Hosted server:** COG range reads, or a mirror (124 GB).
- **Cadence:** static (2021 v200).

**Buildings (licences: ODbL; dl-de/zero for LoD2).**
- **Development now:** OSM footprints, via the same extract as roads.
- **Integrated server:** same as roads.
- **Hosted server:** same as roads, plus optional Hessen LoD2 (manual) later.
- **Cadence:** with the OSM releases; LoD2 annual.

**Getting the first region into the cache (G1/R2):**
1. The owner downloads the DGM1 ZIPs for Main-Taunus-Kreis, Hochtaunuskreis and Frankfurt am Main from the HVBG Download-Center. This is a manual step, because the portal is JavaScript-only and no automatable URL was found.
2. `setup_dev_env.ps1 -FetchData` (after approval) downloads the Geofabrik Hessen PBF.
3. `g2m_tiler import --region data/world/regions.json#home`:
   - takes the district polygons from the extract's admin boundaries plus a 2 km margin;
   - writes `g2m.src.elev.dgm1` and `g2m.src.osm` tiles into the integrated server's source store, marked pinned.
   - **Done (G1c I7, 2026-09-26): see 3.11.** The actual command differs
     from this literal text (positional region selector, no `--region`
     flag, explicit source paths, `racing_game/data/world/regions.json`
     instead of a repo-local path) and also imports landcover in the same
     pass rather than deferring it to step 4 - see 3.11's CLI-shape and
     landcover-timing deviations. Steps 4-5 below are NOT done.
4. On first derivation, GLO-30 and WorldCover are range-read for the region and cached as pinned raw tiles.
5. `g2m_cli prebake --region home` derives all baked layers into the derived store (0.5–1.1 GB, pinned). The region is then fully offline-capable.

Later, with the hosted server, the same result comes from the settings menu's "download region", which fetches packs.

---

## 6. Derivation pipeline

Every layer is a pure function (2.4). Terms:
- **L0:** 1 m cells in 256-cell tiles.
- **Coarse levels:** box-filtered powers of two.
- **Graph tiles:** 1 km.

### 6.1 `g2m.elev.base`

1. Assemble the tile plus a 16 m halo from the best available source. Priority: DGM1 > other national 1 m DGMs > GLO-30.
   - DGM1 in UTM32 needs no resampling.
   - GLO-30 is resampled bicubically in fixed evaluation order.
2. **Coverage seams:** remove the median vertical offset over the seam band, then blend over 200 m with smoothstep.
3. **DSM correction (GLO-30 only):**
   - Where WorldCover says tree cover or built-up, replace heights by an inpainting from surrounding non-canopy cells (harmonic fill, fixed iteration count).
   - This is imperfect, because GLO-30 canopy error under forest is ~13 m median (research).
4. **Quantise** to 1/256 m.
5. **Coarse levels** are box-filtered from L0 where L0 exists, so all LODs agree.

### 6.2 `g2m.roads.graph`

Built from `g2m.src.osm`.

- **Nodes:** junction nodes, i.e. way endpoints and nodes shared by at least two road ways. They keep their OSM node id; shape points go into edge polylines.
- **Edges:** one per way segment between junction nodes, stored as a smoothed **reference line** (6.3 step 1). Attributes:
  - `highway` class; `oneway` (incl. `-1`, roundabouts);
  - `lanes`, `lanes:forward/backward`, `turn:lanes`, `width`;
  - `maxspeed` (+ `:forward/:backward`, parsed units, implicit defaults per country as data);
  - `surface`, `smoothness`, `bridge`/`tunnel`/`layer`;
  - `access`/`motor_vehicle`, `name`, `ref`, `junction`, `lit`.
  - Missing values get documented defaults from tables (e.g. German lane width 3.25 m), each flagged "inferred".
- **Restrictions:** `type=restriction` relations (via node or via way), stored as `TurnRestriction{from_edge, via, to_edge, kind}`.
- **Junctions:**
  - Clusters of junction nodes within ~25 m belonging to one intersection (dual carriageways crossing) form one `Junction`, with ordered edge ends.
  - A roundabout forms one junction ring.
  - This consolidation is the A/B Street mitigation for bow-tie junction polygons (research).
- **Tiling:**
  - Each edge is owned by the tile containing its lower-id end node.
  - Edges crossing tiles reference far nodes by global OSM id, so tiles stitch without a global pass.
  - A coarse hierarchy for major classes supports long routes (Valhalla-like levels).
- **Stable ids:**
  - `NodeId` = OSM node id; `EdgeId` = hash(way id, segment index). Stable within a release.
  - Saved routes store coordinates and are re-matched across releases.
- **Representation** mirrors OpenDRIVE concepts: reference line, elevation polynomial in arc length s, superelevation. An OpenDRIVE export for external tools stays possible later.

### 6.3 `g2m.roads.geom` and carving into `g2m.terrain.height`

Order matters for determinism and continuity. Each step is a pure function of a canonical scope, never of the requesting tile.

1. **Reference line (per edge).**
   - Resample the OSM polyline at 0.5 m.
   - Smooth it, with the curvature limited per class. Keep the original vertices within 0.5 m for major roads.
   - Fit a G1 chain of lines, arcs and clothoids (Bertolazzi–Frego approach; implemented in-house on the deterministic math, since the `Clothoids` library's licence is [unverified]).
2. **Junction heights (per junction).** A robust median of `elev.base` over the junction footprint, depending only on the junction's own position and the DEM.
3. **Vertical profile (per edge).**
   - Sample `elev.base` along the reference line: median across a cross-section window, to reject vehicles and vegetation returns.
   - Fit cubic segments in s by constrained least squares:
     - endpoints fixed to the two junction heights;
     - max grade per class (motorway 6 %, others 12–20 %; tagged `incline` respected);
     - minimum vertical-curve radius.
   - Bridge spans (`bridge=yes`) are excluded from the DEM fit and interpolated between abutments.
4. **Cross-section.**
   - Width = lanes × lane width, or `width`.
   - Crown 2.5 % on undivided roads; superelevation from curvature, capped at 4 %.
   - Shoulder 1.5 m.
   - Then a class-dependent **domain of influence** (3–15 m) blending to the untouched terrain, with a maximum cut/fill slope of 1:1.5 (the BeamNG Road Architect model).
5. **Junction surfaces.**
   - Offset the incoming edge boundaries (Clipper2, miter with bevel fallback), intersect adjacent offsets, and trim edges back to the collision points (A/B Street's algorithm).
   - The junction polygon's height is a least-squares plane through the incoming edge ends.
   - Triangulate with earcut after Clipper cleanup (earcut needs non-self-intersecting rings).
6. **Carving (per `terrain.height` tile, reading every edge and junction touching the tile plus halo).**
   - Rasterise road surfaces (inside the footprint the road wins), then shoulders, then the blend band.
   - Where two roads' influence overlaps, the lowest road wins (BeamNG), producing embankments that a bridge then spans.
   - Write `SurfaceKind` per cell.
   - Order: road class, then `EdgeId`.
7. **Bridges.**
   - The deck follows the fitted profile between the abutments. The terrain beneath is not carved (DGM1 is bare earth).
   - Deck plus parapets become a render mesh and a static collision mesh.
   - Deck ends overlap the carved approach by 1 m and sit 2 cm below it, so a wheel rolls on without a step.
8. **Tunnels:** v1 renders the road through, with the terrain above kept and the tunnel not drivable. Later: heightfield holes at the portals plus a tunnel mesh. How many tunnels the region has on through routes is [unverified]; the tiler reports statistics in G3.
9. **Markings (client-side render only):** lane, edge and stop lines from lanes/oneway/turn:lanes; patterns by class from German marking rules as data.

On lidar data this mostly removes noise. On GLO-30 it is what makes a road drivable at all.

### 6.4 `g2m.terrain.class` and `g2m.features`

- **Land class:**
  - OSM `landuse`/`natural`/`leisure` polygons first; WorldCover where OSM has nothing; water and buildings from OSM.
  - 1 m near roads, 8 m elsewhere.
- **Buildings:**
  - Footprints with `height`; else `building:levels × 3 m` + roof; else a per-type default with deterministic variation.
  - `roof:shape` for simple footprints; base height = minimum terrain under the footprint.
- **Water:** polygons with a surface height = minimum shoreline height.

### 6.5 Client-side derivations (never served)

- **Terrain render chunks:**
  - quadtree LOD, 64×64 quads per chunk at every level, skirts;
  - normals, and 4 material weights from land class;
  - L0 only within ~150–250 m, coarser outward to 20 km.
  - They use the same `terrain.height` samples physics uses ("no second height source").
- **Road ribbons** per edge and junction: u across, v in metres along; material by surface; marking decal mesh.
- **Buildings:** merged mesh per 256 m tile, 3 LODs.
- **Vegetation:** blue-noise scatter from `terrain.class` plus `natural=tree`/`tree_row`, with a seeded hash. Output: instance lists (position, yaw, scale, species).
- **Map geometry:** simplified polylines by road class with widths and names, land-class polygons, water, building outlines at high zoom, label anchors.
- **Physics tiles:** in the bridge (8.1).

---

## 7. Client: streaming, cache, budgets

### 7.1 Interest points and radii (initial values; tuned in R9)

| Product | Source layer(s) | Radius per interest point |
|---|---|---|
| Physics heightfield (bridge) | `terrain.height` L0, `terrain.class` | 400 m per vehicle (derived data kept to 600 m, plus look-ahead along velocity) |
| Bridge and building collision | `roads.geom`, `features` | 300 m per vehicle |
| Road graph | `roads.graph` | 3 km, plus on-demand along an active route |
| Terrain render | `terrain.height` L0–L6 | L0 ≤ 250 m ... L6 to 20 km |
| Road meshes | `roads.graph`, `roads.geom` | 2 km |
| Buildings | `features` | 3 km (3 LODs) |
| Vegetation | `terrain.class`, `features` | 1.5 km, plus an impostor band |
| Minimap / full map | graph, class, features | 2 km / on demand |

- The game sets one interest point per vehicle and one for the camera.
- Vehicle points are also forwarded to `World::set_terrain_interest_point` (the adapter already does this per vehicle).

### 7.2 Client cache

- **One SQLite database** (public domain, single file, crash-safe) with table `tiles(rid, layer, scheme, level, x, y, hash, bytes|blob_ref, last_used, pinned, sources)`.
  - Blobs over 256 KB go to a content-addressed file store next to it (`ab/cd/<hash>`).
- **Dedup:** tiles unchanged between releases share their blob by hash.
- **Eviction:** LRU with a size cap (default 5 GB, set in settings). Pinned regions (e.g. home, downloaded via packs from the settings menu) are never evicted.
- **Integrity:** the hash is checked on every read. A mismatch drops the entry and refetches.
- **The integrated server has its own stores** (raw source store and derived store) in the same cache directory, with their own caps (default 20 GB for sources).
- **Offline or degraded:** if a tile is unavailable, fall back to the next coarser level of the same layer, or to `elev.base` without roads. The item is flagged `degraded` for the UI.
  - Physics never receives a guessed tile (8.2).

### 7.3 Threading and budgets on the Ryzen 9 5900X (12 cores / 24 threads)

- **physics_sim:** its pool defaults to physical cores − 2 = 10 workers, plus the sim thread.
- **Godot:** main thread plus render thread.
- **g2m:** its own pool of 3 low-priority workers for network I/O completion, decoding, local derivation and mesh building.
  - physics_sim's job system has no priorities or background jobs (PLAN.md D9), and multi-millisecond derivation jobs would delay 240 Hz ticks.
- **Main thread:**
  - GPU uploads through `RenderingServer` (Godot's thread-safety rules; multithreaded mesh calls have known deadlocks, research);
  - capped at ~1 ms per frame, measured as a telemetry channel next to the adapter's own frame time;
  - PLAN.md's adapter budget is < 1 ms of its own work.
- **Physics:**
  - `max_tile_fills_per_tick` of 1–2 for 256² tiles; the `SetHeights` cost for 256² is unmeasured (the existing bench uses smaller tiles).
  - At most 1 prebuilt static body add per tick (R2).
  - A hypercar at ~100 m/s crosses a 255 m tile every 2.5 s: ~2 new tiles/s on the front of a 400 m radius, far within 240 fills/s.
- **Fixed material set,** plus Godot ≥ 4.4 ubershaders and background pipeline compilation, to avoid first-draw stutter.

### 7.4 Loading and teleports

- A session start or teleport waits until three things are resident:
  - physics tiles under every vehicle;
  - the road graph tile;
  - L0–L2 render chunks around the camera.
- The loading screen shows progress per layer and the attribution for the area.

### 7.5 C API boundary (for UE5 and other engines)

- **ABI.**
  - Plain C (`g2m_c/g2m.h`), opaque handles, POD structs.
  - Every struct starts with a `size` field and is append-only; new fields never break old plugins.
  - `g2m_version()` returns semver.
- **Lifecycle.**
  - `g2m_server_create(config_json)` gives an in-process server.
  - `g2m_client_create(config_json, transport)`, where the transport is in-process (a server handle) or HTTP (a base URL + host fetcher).
- **Host fetcher.**
  - Remote servers are reached through callbacks the host provides: `fetch(url, headers, ticket)`, completed later via `g2m_fetch_complete(ticket, status, headers, bytes, len)`.
  - Godot implements it with `HTTPClient`, Unreal with `FHttpModule`, and tools with the bundled libcurl.
  - The library needs no TLS stack in the game.
- **Threading.**
  - The library's own pool does all work. Host callbacks are never invoked on worker threads.
  - The host calls `g2m_client_poll(max_items, budget_us)` from one thread and receives completed items there, in deterministic order: product, then level, then key.
  - All other calls are thread-safe.
- **Memory.**
  - Every produced item (terrain chunk, road mesh, instance list, map geometry, graph tile) is immutable, refcounted and owned by the library.
  - `g2m_item_view()` returns pointers plus counts, plus a `double origin[3]` in session-frame metres, valid until `g2m_item_release()`.
  - The host copies into GPU buffers and releases. No allocator is imposed.
- **Errors.** Only setup calls fail with an error code plus message. Streaming reports per-item status; no exception ever crosses the C boundary.
- **Async requests.**
  - `g2m_client_set_interest(id, xyz, radii_by_product)` and `g2m_client_request(layer, key) -> ticket` (the latter for tools and the bridge).
  - `g2m_client_cancel(ticket)`.

---

## 8. Physics bridge and the collision decision

### 8.1 `G2mTerrainSource : ps::terrain::ITerrainSource`

- `tile_size_m() = 255`, `samples_per_tile() = 256`.
- `TileKey.packed` = (ix, iy) of the session 255 m grid, whose origin is offset by half a metre so samples sit on DGM1 cell centres (4.4, 4.9); the zone is fixed per session.
- `fill_tile`:
  - copies samples from the resident immutable `terrain.height` tiles (up to 4 g2m tiles per physics tile);
  - writes ABSOLUTE heights (orthometric, session frame). physics_sim's TileManager chooses a per-tile datum itself (R1, physics_sim 1f1e0f8: the midpoint of the tile's range snapped to 128 m, subtracted before upload, added back via the body pose) and counts tiles with more than ±200 m of local relief (`World::terrain_relief_overflow_count`). Measured at 1 m spacing: exact on flat ground, ≤ 1.15 mm on a 30 % slope, a 5 mm step kept to ~10 µm. Error grows with local relief inside a Jolt block, not with absolute height;
  - maps `SurfaceKind`/`LandClass` to `SurfaceId` by name through the game's table.
- `TerrainConfig`:
  - `suppress_terrain_under_static_geometry = false`;
  - `max_resident_tiles` sized from the radius (~40 at 400 m);
  - `max_tile_fills_per_tick` from the G2 measurement.

### 8.2 Guarantees

- **A vehicle only runs where its physics tiles are filled.** If a tile is late (network, derivation queue), the game freezes the session clock and shows "streaming" instead of letting the car fall. Tile availability then never changes physics results, only when the session advances.
- **Streamed static bodies** (bridge decks, building shells) are added and removed between ticks on the sim thread, in sorted key order, at ticks decided by vehicle position (deterministic).

### 8.3 Roads in the heightfield vs static road meshes (decision)

| | Carve into heightfield (chosen) | Static road mesh bodies |
|---|---|---|
| Geometry | 1 m cells; crown and banking fine; kerbs not representable | exact ribbon, kerbs possible |
| Seams | none added | road/terrain coincident surfaces: the PHYS-021 flicker class; suppression only works at z = 0 today |
| Streaming cost | none beyond terrain tiles | mesh bodies per tile; Jolt BVH build on the sim thread (until R2); broadphase churn; whole-cache suppression wipe per add |
| Surface ids | per 1 m cell (stair-stepped at road edges) | per triangle |
| Physics changes | R1 | R1, R2, plus non-zero-height suppression |

The carve-in wins for v1 because it adds no new seam or streaming risk. Bridges stay meshes because they lie above other surfaces, and they are rare. Revisit for a track mode that needs kerbs.

### 8.4 Surfaces

- The game ships its own surfaces file, a superset of physics_sim's names. It adds `cobblestone`, `concrete`, `gravel` and `paving_stones`, with sourced λµ/crr values.
- It also ships a `g2m SurfaceKind/LandClass -> surface name` table.
- Ids are resolved by name at load. The haptics surface table is name-keyed too (PHYS-042), so new surfaces only need rows.

### 8.5 G2.4 as built: the physics_sim bridge (R2.2 G1-G8, 2026-09-26) - **done**

Implemented against the separate, coordinator-approved plan
`racing_game/PLAN.md` R2.2 section 1 (commits `c2f3959`..`584fb91`, milestone
schedule item R2.2 G1-G8). This subsection is the "as built" record 8.1-8.3
described in advance; the normative code comments are on the headers cited
below.

**What exists:**
- `g2m_phys` (ps-free, `include/g2m/phys/`, `src/phys/`, built unconditionally
  on every leg): `PhysicsTileGrid` (255 m / 256-sample physics tiles anchored
  on DGM1 cell centres, `physics_grid.h`), `ResidentHeightSet` (sorted-cache
  of loaded `HeightTile`s with an owner-thread-only mutation contract,
  `resident_heights.h`), `fill_physics_heights<Real>` (header-only,
  allocation-free fill from the resident set, `fill_physics_heights.h`),
  `HeightTileLoader` + `TransportHeightTileFetch` + `DelayedFetch` (async
  fetch/decode/retry, `height_tile_loader.h`), `PhysicsTerrainStreamer`
  (gate/keep/evict + the clock-freeze `GateStatus`, `physics_streamer.h`).
- `g2m_ps_bridge` (`bridges/physics_sim/`, opt-in behind
  `if(TARGET ps_core)`/`G2M_PS_SOURCE_DIR`): `G2mTerrainSource :
  ps::terrain::ITerrainSource` and `make_terrain_config(interest_radius_m,
  fills_per_tick)` (`include/g2m/ps_bridge/g2m_terrain_source.h`).
- The `bridge-release` CI leg (`CMakePresets.json`, `tools/ci.ps1`, reachable
  only via `-Only bridge-release`): builds and tests the bridge against a
  local, offline clone of physics_sim pinned by `bridges/physics_sim/
  physics_sim.pin` (`5c648a5`, equal to racing_game's own submodule sha).

**Measured acceptance (PLAN.md 4a/4b numbers, `g2m_phys_tests` +
`g2m_bridge_tests`, bridge-release leg):**
- **Render vs Jolt** (`test_render_vs_jolt.cpp`): synthetic checkerboard
  saddle max 0.506 mm (bound <= 2 mm; the wrong triangle diagonal is off by
  388.6 mm, kept as a negative control >= 100 mm); the PKT DGM1 fixture max
  2.913 mm / p99 1.279 mm (bound <= 5 mm / <= 2 mm).
- **Determinism** (`test_determinism.cpp`): a box + 16 spheres pushed across
  physics-tile seams for 2400 ticks give the identical per-tick state_hash
  sequence (final `e1a5c9079787bd41`) at 1/4/10 workers, and between
  up-front installs and gated delayed installs (one tile released per 3
  frozen attempts behind a test-local racing_game-style gate); `fill_miss_
  count() == 0` throughout.
- **Residency** (`test_residency.cpp`): on the fixture at r = 255 m, after
  priming, `starved`/`relief_overflow`/`fill_miss` are all 0; a 404 area
  120 m from the northern coverage edge installs as Absent (holes in both
  Jolt and `fill_tile`) with no freeze.
- **Seam vs seamless** (`test_seam_crossing.cpp`): a sliding sphere/box at
  5/20/40 m/s over the tiled 255 m ground vs one 1024-sample seamless
  ground, `+x` and 4-tile-corner paths, physics_sim's own
  `test_terrain_seam_crossing.cpp` bounds (`|az| <= 2*seamless + 30` and
  `<= 120 m/s^2`). Passes on the `+x` path; on the corner path the seamless
  reference itself reaches up to ~175 m/s^2 (1/256 m-quantised terrain,
  diagonal at 40 m/s), so the absolute cap there is relaxed to
  `max(120, seamless + 30)` (a corner-path-only change, `abs_bound()` in the
  test) - see the "deviations" bullet below.
- **`bench_bridge_fill`** and the **F decision**: `fill_tile` 0.030 ms;
  `update_heightfield_body(256^2)` (Jolt `SetHeights`/`SetMaterials`) 2.262 ms
  mean at pool 49; cross-tile weld overhead 0.006 ms/tick; end-to-end fill
  ticks 2.41 ms at F = 1 vs up to 5.0 ms at F = 2. Plan 1b's rule (F = 2 only
  if one `SetHeights` + weld stays under 1.0 ms) gives 2.268 ms >= 1.0 ms, so
  **F stays 1**; `make_terrain_config`'s own header records the decision.

**Deviations from the plan:**
- **Margin interpretation** (`physics_streamer.h`/`.cpp`, section 1a step
  3): the plan left open which radius the look-ahead square uses. The
  implementation uses the *gate* radius (TileManager's own `r` +
  `gate_margin_tiles`), not TileManager's bare radius, around
  `p + v*min(lookahead_s, lookahead_cap_m)` - chosen so the default
  `max_resident_l0 = 160` still leaves headroom over gate + prefetch + look-
  ahead at the plan's own worked radii.
- **Failed-key memory + `retry_failed()`** (section 1a step 1/4): a key
  whose loader outcome is `Failed` is remembered and reported in
  `GateStatus::failed` rather than either (a) re-requested every tick
  (would hammer a failing server) or (b) installed as an `Absent` hole
  (would silently create a hole where coverage genuinely exists). It is
  forgotten, and so retried, once it leaves the keep set, or on an explicit
  `PhysicsTerrainStreamer::retry_failed()` call - not specified in the
  original plan text.
- **Corner-path seam cap** (`test_seam_crossing.cpp`, section 4b): the
  plan's literal seam-vs-seamless bound (`<= 2*seamless + 30` and
  `<= 120 m/s^2`, copied from physics_sim's own test) is relaxed to
  `max(120, seamless + 30)` on the 4-tile-corner path only, because the
  *seamless* reference itself exceeds 120 m/s^2 there (terrain
  quantisation, not the bridge's own weld) - flagged for the physics
  session as a candidate follow-up (weld behaviour at a 4-tile corner at
  low speed), not fixed here.
- **All ten `FETCHCONTENT_SOURCE_DIR_*` overrides, not two** (section 1c,
  `tools/ci.ps1`): `FETCHCONTENT_FULLY_DISCONNECTED` is a configure-global
  CMake switch, not scoped to the two dependencies the bridge itself needs
  (JoltPhysics, nlohmann_json, from physics_sim's own `_deps`). The
  `bridge-release` leg therefore also redirects geo2map_engine's own eight
  FetchContent dependencies (Catch2, zstd, SQLite, nlohmann_json,
  libdeflate, zlib-ng, protozero, libosmium) at this repo's own `release`
  preset's `_deps`, and fails with an actionable message if any of the ten
  is missing, rather than a bare FetchContent network error.
- **Pool size 49, not "~40"**: 8.1's estimate is superseded by
  `make_terrain_config`'s exact formula, `(2*ceil(r/255)+3)^2`; at the
  plan's own r = 400 m example this is 49, not ~40.

**What remains for racing_game (R1 onward, unchanged from the R2.2 plan,
section 6):** bump the physics_sim/geo2map_engine submodules to this
commit and link `g2m_phys` + `g2m_ps_bridge` into `rg_core` (R1);
`FixedRateLoop` (R2); `WorldConfig::physics` and
`WorldTerrain::height_tile_shared` (R3); `Session`'s terrain mode - the
`try_step` gate, start-up priming and spawn-ray-cast, `StreamingStatus`
(R4); `DriveScript`/autopilot and `tools/terrain_drive` (R5); `[realdata]`
render-vs-physics and seam acceptance (R6); the `RgSimulation`/
`RgTerrainView` Godot API (R7); `TerrainViewStreamer` + `RenderChunk::key`
render-LOD-follows-the-car (R8); the `--drive` scene, HUD and smoke tests
(R9); R10's measurements and its own "R2.2 as built" note in
`racing_game/PLAN.md`.

**Deferred to R2.3+** (R2.2 plan section 4c, unchanged): the 30 km MTK/HTK
scripted drive; streaming at 100 m/s (tested here only up to a 100 m/s
random-walk interest point in the queued-key invariant test, never as an
acceptance bound); land-class surfaces (`G2mTerrainSource` writes one fixed
`ps::SurfaceId` per instance, default `"asphalt"`, resolved by the caller
through `SurfaceTable` - see 8.4); an LRU for the `WorldTerrain` decoded-tile
cache; the PHYS-045 mesh-spike follow-up implied by the corner-path
deviation above.

---

## 13. Downloads and installs for owner approval

Nothing below has been downloaded or installed. The setup scripts will install or fetch only items on an approved list. Sizes are approximate; [unverified] where no primary figure was found.

**Already installed (verified this session): no action.** VS 2022 Build Tools (clang-cl, cl), CMake, Ninja, Git 2.54, Python (`py`), `curl.exe`, Godot 4.7.2 (winget), WSL with clang/lld.

**Source dependencies (FetchContent at configure time; pinned tags):**

| Item | Licence | Approx. size | Used by |
|---|---|---|---|
| godot-cpp branch 4.5 | MIT | ~30 MB source [unverified] | game extension (physics_sim already fetches it) |
| SQLite amalgamation 3.x | public domain | ~3 MB zip [unverified] | client cache, server stores |
| zstd | BSD (dual BSD/GPLv2; use BSD) | ~2.5 MB [unverified] | tile compression |
| libdeflate | MIT | ~0.5 MB [unverified] | DEFLATE in TIFF/COG |
| Clipper2 | BSL-1.0 | ~1 MB [unverified] | road offsets, junctions |
| earcut.hpp | ISC | < 0.1 MB | polygon triangulation |
| cpp-httplib | MIT | ~0.5 MB, one header | HTTP server (sidecar/hosted) |
| libcurl (Schannel on Windows, OpenSSL on Linux) | curl (MIT-like) | ~4 MB source [unverified] | tools, server upstream fetches (not in the game) |
| libosmium + protozero | BSL-1.0 / BSD-2 | ~2 MB [unverified] | `g2m_tiler` only |
| zlib-ng | zlib | ~1 MB | libosmium PBF (tiler only) |
| GeographicLib (test data generation only) | MIT | ~2 MB [unverified] | G0 reference values |
| later, only if needed: libwebp (decode) | BSD-3 | ~4 MB [unverified] | Mapterhorn WebP (G8) |
| later: vtzero + protozero | BSD-2 | < 1 MB | MVT/PMTiles basemaps, if ever |

**Datasets (dev machine, integrated server; 5.4/5.5):**

| Item | Licence | Approx. size | How |
|---|---|---|---|
| Hessen DGM1: Main-Taunus-Kreis, Hochtaunuskreis, Frankfurt am Main | dl-de/zero-2-0 | 1.1–2.6 GB [estimate] | manual download from gds.hessen.de (JS portal, no automatable URL found) |
| Geofabrik `hessen-latest.osm.pbf` | ODbL 1.0 | 329 MB | setup script, once (Geofabrik fair use) |
| Copernicus GLO-30 N50E008 (+ N49E008 if needed) | Copernicus DEM licence | 25–40 MB each [unverified] | range reads (or one download) from AWS open data |
| ESA WorldCover v200 N48E006 map | CC BY 4.0 | ~47 MB average [estimate] | range reads (or one download) from AWS open data |
| CC0 dev textures (13 sets, 2K) from ambientCG / Poly Haven | CC0 1.0 | 150–400 MB [estimate] | setup script, listed per asset |
| Hessen LoD2 for the region (optional, G5+) | dl-de/zero-2-0 | unknown [unverified] | manual |

**Later (hosted server, G7/G8; each approved separately at that time):**
- Germany DGM1, all states: 0.4–1 TB as delivered [estimate].
- Geofabrik Germany extract (~4 GB [unverified]) or the OSM planet (~80–90 GB [unverified]).
- WorldCover global: 124 GB.
- Mapterhorn planet PMTiles z0–12: ~120 GB.
- GLO-30 global mirror (size [unverified]).
- Accounts/services: object storage + CDN, compute VM(s), domain + TLS. Choice per D3.

---

## References

**physics_sim (read-only):**
- `PLAN.md`: sections 1, 3 (D1–D10), 6 seams, P2 front-end targets, M1, P4 HUD, P8 seam spike, P9 budgets, P11, section 8.
- `core/include/ps/terrain/terrain_source.h`, `terrain_config.h`
- `core/src/terrain/tile_manager.{h,cpp}`
- `core/src/backend/jolt/jolt_backend.cpp`
- `core/include/ps/backend/shape_desc.h`
- `core/include/ps/world/world.h`
- `core/src/io/vehicle_io.cpp`
- `adapters/godot/{CMakeLists.txt, src/*, demo/*}`
- `data/models/car_hyper/car_hyper.rig.json`, `data/vehicles/car_sedan.json`, `data/surfaces/surfaces.json`
- Vault: PHYS-003, -008, -009, -011, -021, -025, -026, -032, -035, -042; TOOL-023.

**OSM and vector data:**
- https://wiki.openstreetmap.org/wiki/Overpass_API
- https://operations.osmfoundation.org/policies/tiles/
- https://operations.osmfoundation.org/policies/nominatim/
- https://opendatacommons.org/licenses/odbl/1-0/
- https://osmfoundation.org/wiki/Licence/Community_Guidelines/Produced_Work_-_Guideline
- https://osmfoundation.org/wiki/Licence/Attribution_Guidelines
- https://download.geofabrik.de/europe/germany/hessen.html
- https://download.geofabrik.de/technical.html
- https://docs.overturemaps.org/guides/transportation/
- https://docs.overturemaps.org/schema/reference/transportation/segment/
- https://docs.overturemaps.org/attribution/
- https://docs.protomaps.com/basemaps/downloads
- https://docs.protomaps.com/basemaps/layers
- https://github.com/mapbox/vector-tile-spec
- https://openmaptiles.org/schema/
- https://valhalla.github.io/valhalla/concepts/tiles/
- https://developers.cloudflare.com/r2/pricing/

**Elevation and land cover:**
- https://hvbg.hessen.de/landesvermessung/geotopographie/3d-daten/digitale-gelaendemodelle
- https://hvbg.hessen.de/geoinformation/open-data
- https://metaver.de/trefferanzeige?docuuid=dbf48a95-b44d-48b3-a5b4-981e4c1bd8e6
- https://hvbg.hessen.de/landesvermessung/geotopographie/3d-daten/3d-gebaeudemodelle
- https://gds.hessen.de/INTERSHOP/web/WFS/HLBG-Geodaten-Site/de_DE/-/EUR/ViewDownloadcenter-Start
- https://www.opengeodata.nrw.de/produkte/geobasis/hm/dgm1_tiff/dgm1_tiff/
- https://geodaten.bayern.de/opengeodata/OpenDataDetail.html?pn=dgm1
- https://registry.opendata.aws/copernicus-dem/
- https://copernicus-dem-30m.s3.amazonaws.com/readme.html
- https://dataspace.copernicus.eu/explore-data/data-collections/copernicus-contributing-missions/collections-description/COP-DEM
- https://registry.opendata.aws/terrain-tiles/
- https://github.com/tilezen/joerd/blob/master/docs/formats.md
- https://mapterhorn.com/data-access/
- https://download.mapterhorn.com/attribution.json
- https://protomaps.com/blog/mapterhorn-terrain/
- https://registry.opendata.aws/esa-worldcover-vito/
- https://esa-worldcover.s3.eu-central-1.amazonaws.com/readme.html
- https://land.copernicus.eu/en/products/products-that-are-no-longer-disseminated-on-the-clms-website
- https://meetingorganizer.copernicus.org/EGU22/EGU22-8994.html

**Libraries:**
- https://geographiclib.sourceforge.io/C++/doc/classGeographicLib_1_1TransverseMercator.html (Karney, J. Geodesy 85(8), 2011)
- https://proj.org/en/stable/install.html
- https://github.com/AngusJohnson/Clipper2
- https://github.com/mapbox/earcut.hpp
- https://github.com/mapbox/vtzero
- https://github.com/mapbox/protozero
- https://github.com/ebiggers/libdeflate
- https://github.com/randy408/libspng
- https://github.com/webmproject/libwebp
- https://gdal.org/en/stable/drivers/raster/cog.html
- https://github.com/yhirose/cpp-httplib
- https://github.com/curl/curl/blob/master/docs/INSTALL-CMAKE.md
- https://github.com/facebook/zstd

**Road geometry:**
- https://a-b-street.github.io/docs/tech/map/geometry/index.html
- https://documentation.beamng.com/world_editor/tools/road_architect/terraforming/
- https://publications.pages.asam.net/standards/ASAM_OpenDRIVE/ASAM_OpenDRIVE_Specification/latest/specification/10_roads/10_05_elevation.html
- https://github.com/ebertolazzi/Clothoids
- https://arxiv.org/pdf/1209.0910
- https://github.com/tordanik/OSM2World/pull/176
- https://carla.readthedocs.io/en/0.9.11/tuto_G_openstreetmap/

**Godot and textures:**
- https://docs.godotengine.org/en/4.4/tutorials/physics/large_world_coordinates.html
- https://docs.godotengine.org/en/latest/tutorials/performance/thread_safe_apis.html
- https://github.com/godotengine/godot/issues/90461
- https://docs.godotengine.org/en/stable/tutorials/performance/pipeline_compilations.html
- https://github.com/TokisanGames/Terrain3D
- https://docs.godotengine.org/en/4.4/classes/class_httpclient.html
- https://dev.epicgames.com/documentation/en-us/unreal-engine/API/Runtime/HTTP/FHttpModule
- https://docs.ambientcg.com/license/
- https://polyhaven.com/license
