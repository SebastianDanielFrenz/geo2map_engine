# geo2map_engine - architecture map

This file only maps what exists and how to build, test and run it.
- Rationale and design decisions live in the knowledge vault
  (`S:\claude_knowledge_base`), not here.
- The design and phase plan is `PLAN.md` (plus racing_game `PLAN.md` section 12
  for the milestones).
- **Standing rule:** any change that adds or changes a pipeline stage, layer,
  encoding, data source or consumer updates `docs/pipeline/` in the same
  commit. Start reading there: `docs/pipeline/README.md`.
- Milestone state: **G0 done**. G0 covers the library skeleton, deterministic
  math, geodesy, tile keys, rasters, hashes and golden infrastructure.
  **G1a done**: the layer model, the tile formats, releases, the SQLite tile
  stores, the in-process server and the client cache. **G1b done**: pure,
  bounds-checked decoders for the source formats (ZIP, TIFF/GeoTIFF/BigTIFF/
  COG, Hessen DGM1, OSM PBF) - see `g2m_decode` below. **G1c done**: exact
  polygon geometry (`g2m::geom`), region definitions and resolution
  (`g2m/layer/region.h`, `g2m_import`'s `region_resolve`), the DGM1/OSM/
  WorldCover importers, the offline tiler CLI (`apps/g2m_tiler`), and the
  real home-region import into the integrated server's source store
  (PLAN.md 3.11, 5.5 step 3) - see `g2m_import`/`g2m_tiler` below.
  **G6 done** (2026-10-15): protocol v1 over HTTP (`g2m_http`) and the
  sidecar server `apps/g2m_server` - see `g2m_http` below; not wired into
  racing_game; TLS/auth/hosting are G7 (`docs/protocol/README.md`).

C++20, namespace `g2m`, MIT. g2m never includes or links anything from
physics_sim.

## Layout

```
geo2map_engine/
  CMakeLists.txt, CMakePresets.json   top-level build (presets below)
  CLAUDE.md, README.md, PLAN.md, LICENSE, THIRD_PARTY_NOTICES.md
  cmake/deps_core.cmake               FetchContent for zstd / SQLite / nlohmann_json (G1a; pins in its header table)
  cmake/deps_http.cmake               FetchContent for cpp-httplib (G6; only with G2M_BUILD_HTTP; pin in its header table)
  include/g2m/core/                   public API of g2m_core (no third-party types)
    result.h                          Result<T> / Result<void>, Error{code, message}, ErrorCode, make_error (no exceptions)
    bytes.h                           ByteWriter / ByteReader: little-endian, canonical LEB128 varint/svarint, zigzag;
                                      the reader is bounds-checked with a sticky failure flag (fuzz-safe)
    det_math.h                        g2m::det: sin cos sincos tan asin acos atan atan2 exp expm1 log log1p pow
                                      sinh cosh tanh asinh atanh hypot (musl port), sqrt (std, IEEE-exact)
    version.h                         g2m::version_string() (= CMake PROJECT_VERSION)
    hash.h                            Fnv1a64 (explicit little-endian serialisation, doubles by bit pattern,
                                      length-prefixed strings), Sha256 (FIPS 180-4), to_hex
    tile_key.h                        TileKey{zone, level, x, y}: 256 m * 2^level squares on the UTM grid,
                                      packed() 64-bit layout, parent/child, floor_div, tile_key_at, to_string
    raster.h                          Raster<T> (row-major), HeightRaster (int32, 1/256 m), ClassRaster (uint8),
                                      quantise/dequantise_height, SampleRegistration {Node, Cell},
                                      RowOrder {SouthToNorth, NorthToSouth}, RasterGeoref (integer mm),
                                      tile_georef, lattice_offset
    geo/ellipsoid.h                   Ellipsoid{a, f}; kGRS80 (default everywhere), kWGS84
    geo/angle.h                       degree helpers: sincosd, atan2d, atand, ang_normalize, ang_diff, kDegree
    geo/tm.h                          TransverseMercator(ellipsoid, k0): forward/reverse (6th-order Krueger series)
    geo/utm.h                         UtmZone, Hemisphere, utm_central_meridian, utm_false_northing,
                                      utm_latitude_band, utm_standard_zone (Norway/Svalbard), utm_scheme_name, Utm
    geo/ecef.h                        geodetic_to_ecef, EnuFrame/enu_frame
    geo/session_frame.h               SessionFrame (UTM zone + integer-km origin): geodetic/grid <-> LocalXY,
                                      tm_at, tile_at, tile_origin; SessionFrame::around(lat, lon)
  include/g2m/layer/                  public API of g2m_layers (PLAN.md 2.5, 3.2; formats: docs/formats/)
    layer_id.h                        layer names (is_valid_layer_name), LayerId, LayerKind, LayerEncoding, LevelMask,
                                      LayerDesc, make_raw_{elev,osm,landcover}_layer (g2m.src.elev.<source>, g2m.src.osm,
                                      g2m.src.landcover; level 2)
    layer_registry.h                  LayerRegistry (sorted; builtin() = the three raw layers, elev as dgm1)
    deriver.h                         IDeriver + InputTileRef/InputTile/DeriveRequest/DeriveResult (declarations; no built-ins)
    classifier.h                      IClassifier, LandClass, SurfaceKind, ClassifierFacts (declarations)
    tile_container.h                  g2m.tile/1: TileHeader, TileBody/Section/SectionCodec, VerticalDatum, content_hash,
                                      encode/decode_header, encode/decode_body, assemble_container, parse_container
    src_elev.h                        ElevTile, ElevPredictor {Left, Gradient}, encode/decode_src_elev
    src_landcover.h                   LandcoverTile, encode/decode_src_landcover
    osm_data.h                        osm::OsmData (canonical model: sorted strings/ids, pooled tags/refs/members), OsmBuilder
    src_osm.h                         OsmTile, encode/decode_src_osm
    osm_clip.h                        clip_osm (per-tile selection with halo), segment_touches_box
    osm_roads.h                        client-side OSM road raster (PLAN.md 6.5, roads_plan.md): RoadStyle (data-table
                                      highway->LandClass/rank/width, surface->SurfaceKind), extract_road_segments,
                                      Lattice/RasterParams, rasterize_road_segments (exact integer inside test, round
                                      end caps via clamped projection); render road LOD as caller data: RenderRoadLod
                                      (per chunk level enabled/min_rank/min_half_width_mm), default_render_road_lod
                                      (== the compiled raster_params_for_render_level), render_road_lod_key(policy, level)
    release.h                         ReleaseInputs -> compute_release_id, Coverage, ReleaseManifest + to_json/from_json,
                                      to_hex / content_hash_from_hex
    pack.h                            PackDirectory/PackEntry/PackTileState, pack_of/pack_tile, encode/decode_pack,
                                      encode/decode_pack_directory (G6: `G2MH`, the /hashes body over HTTP)
  include/g2m/server/                 public API of g2m_server (PLAN.md 3.1-3.4)
    tile_store.h                      TileStore (SQLite + blob files; CONCURRENCY CONTRACT in the header), TileStoreConfig
                                      (incl. read_only - G2.0: SQLite immutable-mode URI, no directory creation, no
                                      write-path pragmas, every mutator rejected -> ErrorCode::ReadOnly),
                                      TileAddress, StoredTile, stats/check_consistency, index() (TileIndexEntry{layer,
                                      key, body_hash}, sorted by (layer, key) - G2.0's diff-scopes merge-diffs two
                                      calls' output directly, no extra sort needed)
    protocol.h                        protocol v1 request/response objects, Status, ResponseMeta, ServerManifest JSON,
                                      to_path / parse_path
    transport.h                       ITransport
    fetcher.h                         IFetcher (the only way to the network; absent = offline by construction)
    upstream.h                        IUpstream, UpstreamContext/UpstreamReply, LocalSourceUpstream (a source TileStore)
    server.h                          Server (router + resolver, per-layer policy chain Cache -> Derive -> Upstream),
                                      ServerConfig, LayerPolicy, make_local_release
  include/g2m/client/                 public API of g2m_client
    cache.h                           ClientCache (TileStore named "cache": dedup, blobs > 256 KiB, LRU cap, pinning,
                                      integrity check on read)
    in_process_transport.h            InProcessTransport (ITransport -> Server::handle)
  src/core/                           implementation (mirrors include/); det_math.cpp forwards g2m::det -> g2m::musl
  src/layer/                          g2m_layers implementation; zstd_codec.h/.cpp (INTERNAL: pinned zstd), georef_io.h
                                      (INTERNAL: georef block)
  src/server/                         g2m_server: tile_store.cpp, protocol.cpp, server.cpp
  src/client/                         g2m_client: cache.cpp
  include/g2m/http/                   public API of g2m_http (G6; no httplib type; binding spec docs/protocol/README.md)
    http_wire.h                       the pure HTTP mapping of protocol v1: request_target/request_headers and
                                      decode_response (client), parse_request/encode_response/error_response/
                                      handle_request (server), Message/Header, header-text escaping, ETag, base path,
                                      parse_http_url, kMaxTargetLength 1024; status mapping (429/5xx -> 503, 4xx -> 400,
                                      bad header or payload -> 500, transport failure -> 503)
    http_transport.h                  HttpTransport (ITransport over HTTP/1.1, keep-alive pool, timeouts, max body,
                                      never follows redirects), HttpTransportConfig
    http_server.h                     HttpServer::start(ITransport&, HttpServerConfig) (127.0.0.1, port 0 = any,
                                      base path, thread pool, one listener per port), port/base_url/stats/stop
  src/http/                           g2m_http: http_wire.cpp (also compiled into the fuzz build), http_transport.cpp,
                                      http_server.cpp (the only files that include httplib.h)
  apps/g2m_server/                    sidecar map server (G6): main.cpp - one local release over a read-only source
                                      store (builtin_local_release_params, --layers terrain|roads|geometry|carved),
                                      offline (no fetcher), --bind 127.0.0.1 --port 0 --base-path --threads,
                                      prints `G2M_SERVER_READY url=... port=... rid=...`, SIGINT/SIGTERM or
                                      --exit-on-stdin-eof stop it; binary g2m_server (CMake target g2m_server_app)
  include/g2m/mesh/                   public API of g2m_mesh (PLAN.md G2.3): engine-neutral terrain render
                                      chunks + LOD selection, POD-shaped for a later C API
    terrain_chunk.h                  ChunkKey, HeightWindow (67x67 halo)/ClassWindow, TileLookup,
                                      gather_window, TerrainChunkMesh, build_chunk (triangulates with the
                                      SAME diagonal as physics_sim's Jolt HeightFieldShape - see the header
                                      comment on build_chunk for the full derivation)
    lod_select.h                     LodParams, select_chunks (distance-based split + restricted-quadtree
                                      2:1 balance, output sorted by level desc/cy/cx)
  src/mesh/                           g2m_mesh implementation: terrain_chunk.cpp, lod_select.cpp
  include/g2m/phys/                   public API of g2m_phys (PLAN.md R2.2 section 1a): ps-free physics-tile
                                      grid + resident height cache, shared by g2m_ps_bridge (G6+) and any
                                      other physics_sim peer - no physics_sim/Jolt/Godot type anywhere here,
                                      and g2m_phys never links physics_sim
    physics_grid.h                   kPhysSamples (256, tile edge 255 m, 1 m spacing), PhysTileIndex,
                                      PhysicsTileGrid (index_for_local/origin_local, pack/unpack - same
                                      layout as physics_sim's ProceduralTerrainSource - l0_keys_for_square:
                                      the DGM1 L0 data-tile keys a square of physics tiles can touch);
                                      anchored so every physics sample sits exactly on a DGM1 cell centre
                                      (see the header's own derivation and its [AMEND] test)
    resident_heights.h                ResidentHeightSet (PLAN.md R2.2 section 1a): a sorted-by-packed()
                                      vector<Entry> cache of loaded HeightTiles; find (binary search,
                                      returns a raw HeightTile* - no refcount traffic), install (nullptr =
                                      confirmed Absent/404), retain_only (eviction). Threading contract
                                      written into the header: only the owner mutates it, and only between
                                      World steps - find() takes no lock
    fill_physics_heights.h            FillResult (Ok/HadNoData/MissingTile), fill_physics_heights<Real>
                                      (header-only template): fills one physics tile's samples from a
                                      ResidentHeightSet, NO allocation for any sample count (incl. 1024),
                                      NO row flip (out is row-major [j*n+k], j north, matching
                                      HeightTile::h's own row-0-is-south layout directly). A NoData sample
                                      or an Absent tile writes `hole`; Absent alone leaves the result Ok
                                      (a design choice - see the header's own note), a resident tile's own
                                      NoData sample sets HadNoData, and any tile missing from the set sets
                                      MissingTile (priority over HadNoData)
    height_tile_loader.h              async L0 HeightTile loading: FetchResult, IHeightTileFetch (blocking,
                                      thread-safe), TransportHeightTileFetch (ITransport -> parse_container
                                      -> decode_body -> decode_height_tile, applies header height_offset),
                                      DelayedFetch (test/smoke: base + key/seed-hashed jitter),
                                      LoadOutcome/LoadedHeightTile/LoaderConfig/LoaderStats,
                                      HeightTileLoader (worker pool; request() replaces the want-list,
                                      dedups vs in-flight/done; poll() sorted by key; retry/backoff;
                                      destructor joins). Status mapping and threading contract in the header
    physics_streamer.h                InterestPoint, StreamerConfig, GateStatus, PhysicsTerrainStreamer:
                                      update() (owner thread, between steps) polls/installs completions,
                                      builds the gate set (physics_sim TileManager's square
                                      r = ceil(radius/ts) + gate_margin_tiles around index_for_local) and
                                      keep set (+ prefetch square + look-ahead square), retain_only(keep)
                                      only over max_resident_l0 (gate keys never evicted), requests missing
                                      keys gate-first by distance/key, reports ready; remembers Failed keys
                                      (retry_failed()); gate_keys()/keep_keys() accessors
  src/phys/                           g2m_phys implementation: physics_grid.cpp, resident_heights.cpp,
                                      height_tile_loader.cpp, physics_streamer.cpp
  include/g2m/world/                  public API of g2m_world (terrain migration phase 1, vault G2M-020): the
                                      engine-neutral helpers the game's WorldTerrain/route_check/road_classes
                                      hold, ported bit for bit - no physics_sim/Jolt/Godot type
    tile_fetch.h                     decode_height_tile_container (parse_container, layer/key check,
                                      height_offset in int64, reject <= kHeightNoData or > INT32_MAX),
                                      fetch_height_tile (one transport fetch), fetch_height_tile_cached
                                      (double-checked insert, fn outside the lock, failures not cached);
                                      RoadFetchStatus Ok/Absent("outside coverage")/Failed,
                                      decode_road_segments_response, fetch_road_segments(_cached)
    height_sampler.h                 sample_height_exact (Jolt/build_chunk triangulation: NW-SE diagonal,
                                      a triangle solid only if its 3 corners are valid) and
                                      sample_height_bilinear (render only), L0TileLookupFn; session-local
                                      metres, lattice (c - e0 + 0.5)
    chunk_road_classes.h             ClassLookup/RoadSegmentsFn, road_class_lattice,
                                      src_osm_tiles_for_chunk, rasterize_chunk_road_classes (owner rule;
                                      ClassLookup::lod = the caller's RenderRoadLod, null = default)
    chunk_terrain_classes.h          (P2 slice 11, D62) the served alternative: TerrainClassLookup/
                                      TerrainClassTileFn, terrain_class_tile_for_chunk, terrain_class_window
                                      (vertex = the class node at floor of its position, road nodes filtered
                                      by the RenderRoadLod on the rank plane), fill_chunk_terrain_classes;
                                      ViewSource::terrain_classes takes precedence over class_lookup
    surface_id_table.h               g2m::classes::SurfaceIdTable (SurfaceKind + LandClass -> consumer
                                      id; road cell with a known kind -> kind id, else land class id;
                                      to_road_value_lut)
    ticket.h                         Priority (Gate, Physics, Prefetch, Render, Warm), TileState
                                      (Pending/Ready/Absent/Failed/Cancelled), Ticket<T> (state()/value()
                                      lock-free reads, wait_for for tools, cancel = drop of the last copy)
    products.h                       Outcome<T>, RoadSegmentsTile, RawTile, ProductOptions; the phase-3
                                      payloads (StructureSet, RoadRibbonTile, BuildingSet) are the types below
    road_structures.h                (phase 3a) RoadDeck (incl. start_ref), road_deck_height, build_road_decks
                                      (bridge / tunnel-floor decks + inferred tunnel roofs, client-side until
                                      served, owner Q4; + approach decks of grade-separated crossings under the
                                      caller's RoadSeparation), StructureSet + build_structure_set(RoadGeomTile)
    road_ribbons.h                   (phase 3a) RoadRibbonVertex/RoadRibbonTile, build_road_ribbons(geometry,
                                      key, ground fn, lift) incl. junction surfaces and markings,
                                      add_junction_surfaces, road_priority_rank (StVO 18(3) rank)
    buildings.h                      (phase 3b) Building/BuildingPoint/BuildingGround/BuildingParams,
                                      extract_buildings, BuildingSet + build_building_set(OsmTile, ground)
    view_streamer.h                  (phase 3b) render LOD around a focus: ViewChunk (mesh + class ids, carve
                                      state, session origin, consumer rgba), ViewSource (lod params, lookups,
                                      carve state fn, optional colour hook, road LOD), chunk_window_carve,
                                      select_view_keys, build_view_chunks, build_static_view, speed/urgency
                                      helpers, changed_road_lod_levels, ViewStreamer (one worker, one ViewDiff in
                                      flight, adds-before-removals contract in the header)
    ring_streamer.h                  (phase 3b) RingStreamer(Config): level-2 tile tiers of ring products (road
                                      ribbons, buildings): jobs nearest first (behind penalty), next_job() any
                                      thread, accept(), frame() -> shown/downgraded/evicted changes
    terrain_service.h                TerrainService (terrain migration phase 2): open (stores, layer opt-ins,
                                      release, in-process server, ONE worker pool), async height/road_geom/
                                      road_segments/source_osm/raw_tile/terrain_class (D62)/structures/road_ribbons/buildings ->
                                      Ticket<T>, peek_* (never start work), surface_height_exact/bilinear on
                                      resident tiles, warm() -> WarmJob, physics_fetch() -> PhysicsFetch
                                      (IHeightTileFetch; blocks on the caller, which is a loader thread),
                                      product<T>(kind, key, ...) + ProductContext for caller producers
  src/world/                          g2m_world implementation: road_structures.cpp, road_ribbons.cpp (phase 3a,
                                      ported from racing_game 0c027dc + 945455a, libm -> det), buildings.cpp,
                                      view_streamer.cpp, ring_streamer.cpp (phase 3b, from racing_game 5a3baed),
                                      terrain_service.cpp, job_scheduler.{h,cpp}
                                      (internal: priority queue, per-job registry, interest counting, nested
                                      join / inline run), tile_cache.h (internal: byte-weighted LRU of
                                      decoded tiles), tile_fetch.cpp, height_sampler.cpp,
                                      chunk_road_classes.cpp
  include/g2m/route/                  public API of g2m_route (G4 routing, docs/routing_design.md): engine-neutral,
                                      Result<T>, integer mm / ms costs; links g2m_layers only
    turn_restriction.h               TurnRestriction (No/Only, from way, via node or via ways, to way), RestrictionSet
                                      (counts: not_for_cars, conditional_only, unknown_value, malformed),
                                      extract_turn_restrictions(osm::OsmData) (restriction:motorcar > restriction, except,
                                      no_entry/no_exit expanded per pair)
    routing_graph.h                  RouteConfig/default_route_config (class speeds, zone speeds, blocked access, implied
                                      oneway, turn-cost angle classes, limits), RoutingGraph (decision nodes, shape nodes
                                      collapsed, arcs = edge directions keyed (way, seg_first..seg_last), CSR out-arcs,
                                      via-node rules per node; successors() = restrictions + dead-end-only U-turn;
                                      edge shapes; snap(point, SnapOptions) -> RoutePoint with per-direction candidates),
                                      build_routing_graph(RoadGraph, restrictions, config), RouteBuildStats
    route_search.h                   RouteLandmarks (ALT: farthest-point landmarks, [node][landmark] uint32 tables,
                                      bound(v, t)), RouteSearcher (reusable edge-based A* workspace, generation stamps,
                                      heuristic = max(chord at max_speed, ALT), reopening, ties by (f, g, arc), stop when
                                      min f >= best goal; find(RouteQuery) node to node, find(RoutePoint, RoutePoint)
                                      with partial first/last arcs), Route (start/end offsets), RouteSearchStats
    region_graph.h                   merge_region_graphs (one copy per OSM id, conflicts Malformed, order-independent,
                                      no per-tile limits), assemble_region_graph(keys, RoadGraphTileFetch),
                                      merge_turn_restrictions, build_region_routing (heap-held graph + landmarks)
    route_guidance.h                 build_guidance(graph, roads, route, GuidanceConfig) -> Maneuver list (G4 slice 4:
                                      depart/arrive, continue on a new name/ref, slight/turn/sharp by the cost angle
                                      classes, keep left/right forks within fork_max_deg, U-turn, roundabout enter with
                                      exit number, ramp off/on, merge; name/ref/destination of the road entered;
                                      distances in mm), to_string(ManeuverKind)
    route_map.h                      client map geometry (G4 slice 5): build_map_tile(key, MapLod detail L2 / mid L4 /
                                      far L6, level-2 roads.graph tiles, MapConfig) -> MapTile (lines over the owned edges,
                                      runs merged per way, Douglas-Peucker 1 / 8 / 40 m, int32 dm points from the SW corner,
                                      rank, flags, name/ref string indices, label anchor, (way, segment range));
                                      simplify_polyline, route_polyline (driven line cut at the snapped ends); every LOD
                                      parameter is caller data (levels, ranks, tolerances, switch distances, detail_scale):
                                      validate_map_config, effective_lod, select_map_lod, map_lod_key (per-LOD content key,
                                      MapTile::params_key), MapTileCache (set_config drops only LODs whose key changed)
  src/route/                          g2m_route implementation: turn_restriction.cpp, routing_graph.cpp (incl. snapping:
                                      sorted cell index, heading-aware snap, point_at, partial_cost_ms), route_search.cpp,
                                      region_graph.cpp, route_guidance.cpp, route_map.cpp
  bridges/physics_sim/                 g2m_ps_bridge (PLAN.md R2.2 section 1b, G5/G6): the seam between
                                      g2m_phys's ps-free physics-tile data and physics_sim's own
                                      ps::terrain::ITerrainSource - the only place the two projects meet.
                                      Reached only via the root CMakeLists.txt's `if(TARGET ps_core)`
                                      guard, itself only true when the top level was configured with
                                      G2M_PS_SOURCE_DIR (see "CMake options" below)
    physics_sim.pin                   the exact physics_sim commit this bridge is built/tested against
                                      (read by tools/ci.ps1's bridge-release leg only, never enforced by
                                      CMake itself); must equal racing_game's own physics_sim submodule sha
    include/g2m/ps_bridge/
      version.h                       bridge_version_string() - scaffolding proving the target links
                                      against both ps_core (Jolt included) and g2m_phys (G5)
      g2m_terrain_source.h            G2mTerrainSource : ps::terrain::ITerrainSource (G6) - every override
                                      is a thin, allocation-free translation between
                                      g2m::phys::PhysicsTileGrid's own opaque tile-index packing and
                                      ps::terrain::TileKey; fill_tile defers to
                                      g2m::phys::fill_physics_heights against the ResidentHeightSet the
                                      source holds a shared_ptr to. fill_miss_count()/nodata_fill_count()
                                      (relaxed atomics) surface g2m::phys::FillResult::MissingTile/HadNoData
                                      for the streaming determinism witness (PLAN.md R2.2 section 2).
                                      make_terrain_config(interest_radius_m, fills_per_tick) builds a
                                      ps::terrain::TerrainConfig with suppression/precompute off and
                                      max_resident_tiles = (2*ceil(r/255)+3)^2.
                                      Second constructor, road mode (PLAN.md G2.5a-grip, G-c):
                                      G2mTerrainSource(grid, resident, RoadSurfaceConfig{lut, off_road}) -
                                      fill_tile instead defers to g2m::phys::fill_physics_surfaces (per-cell
                                      SurfaceId from OSM roads riding on the same ResidentHeightSet entries);
                                      a road block's own Missing count folds into the same
                                      fill_miss_count(). static_assert(ps::SurfaceId == uint16_t) guards the
                                      LUT's value type. The uniform (flat SurfaceId) constructor and its
                                      fill_tile path are unchanged
    src/                               version.cpp, g2m_terrain_source.cpp
    tests/                             g2m_bridge_tests (bridge-release leg only): test_smoke.cpp (links
                                      against ps_core/Jolt and g2m_phys), test_g2m_terrain_source.cpp
                                      (contract: samples even/tile_size 255/key_offset+origin round-trip/
                                      fill_tile never resizes; fill_tile zero-alloc under
                                      ps::jobs::debug_alloc across ps::jobs::JobSystem::parallel_batches,
                                      plus its negative control - one ::operator new per job IS counted, G7).
                                      G7 acceptance (PLAN.md R2.2 4b): test_render_vs_jolt.cpp (4096 LCG
                                      points strictly inside L0 quads, Jolt ray_cast through a primed World
                                      vs barycentric interpolation in g2m_mesh build_chunk's own triangles:
                                      +-51/256 m checkerboard saddle max <= 2 mm, the other diagonal >= 0.1 m
                                      as negative control; PKT fixture max <= 5 mm, p99 <= 2 mm),
                                      test_residency.cpp (fixture, r 255, real loader+streamer+
                                      TransportHeightTileFetch near the northern coverage edge: gate opens
                                      with 404 keys Absent, starved/relief_overflow/fill_miss 0, holes in
                                      Jolt and in fill_tile beyond the edge), test_determinism.cpp (a box +
                                      16 spheres pushed across seams for 2400 ticks at pool 49/F 1: per-tick
                                      state_hash equal at 1/4/10 workers and up-front vs gated delayed
                                      installs - TokenFetch releases one tile per 3 frozen attempts behind a
                                      minimal test-local racing_game-style gate). bridge_test_support.h:
                                      synthetic L0 tiles from a per-cell height function (SyntheticTileCache,
                                      install_square), a fixed MMIX LCG, TempDir. Also links g2m_mesh and
                                      g2m_fixture_release (tests/support).
                                      G8: test_seam_crossing.cpp (sliding sphere/box at 5/20/40 m/s over
                                      the 255 m tiles vs ONE 1024-sample static heightfield on the same
                                      lattice, flat + wavy field, +x and 4-tile-corner paths; physics_sim's
                                      test_terrain_seam_crossing.cpp bounds, the 120 m/s^2 cap adapted on
                                      the corner path). bench_bridge_fill.cpp -> bench_bridge_fill (plain
                                      main, CTest smoke, labels bench;bridge, asserts no fill miss/starved
                                      tile, never timing): ms per fill_tile, per update_heightfield_body at
                                      pool 49, weld overhead per tick, end-to-end fill ticks at F 1/2, and
                                      the plan-1b F decision (make_terrain_config's doc records it), plus an
                                      A-road case (G-c): the same fill_tile measurement in road mode, budget
                                      ~0.25 ms extra per fill over the uniform-mode A case.
                                      Deliberately does not link tests/mesh/alloc_counter.cpp or
                                      tests/phys/alloc_counter.cpp (both override global operator new,
                                      which clashes with ps_core's own override)
                                      G-c (PLAN.md G2.5a-grip): test_g2m_terrain_source_roads.cpp (road-mode
                                      shape cast through a real World/TileManager/Jolt heightfield reads the
                                      paved id on-road and off_road 30 m off; fill_tile zero-alloc in road
                                      mode across concurrent jobs, plus its negative control),
                                      test_render_vs_physics_roads.cpp (render-vs-physics cell invariant:
                                      every physics cell fill_physics_surfaces paints as road has a
                                      same-class render-L0 corner among rasterize_road_blocks's own output,
                                      and vice versa - no physics_sim/Jolt type, pure g2m_phys/g2m_layer
                                      comparison). test_determinism.cpp additionally covers road-mode
                                      determinism (gated delayed installs, roads riding on the same
                                      TokenFetch/HeightTileLoader/PhysicsTerrainStreamer machinery,
                                      LoaderConfig::require_roads = true): sampled ps::ShapeCastHit::surface
                                      at fixed on-road/off-road points identical at 1/4/10 workers - generic
                                      rigid-body dynamics do not depend on SurfaceId, so state_hash alone
                                      would not catch a road-painting bug. bridge_test_support.h gained
                                      install_square_with_roads (install_square's road-carrying counterpart).
  docs/protocol/README.md             protocol v1 over HTTP (G6): targets, headers, bodies, content types, Cache-Control,
                                      client status mapping, connections, the sidecar, what G7 needs
  docs/formats/                       normative byte formats: README (conventions), tile_container, src_elev,
                                      src_landcover, src_osm, road_graph, road_geom, terrain_carved, pack,
                                      release_manifest, regions, protocol, cache_db
  docs/pipeline/                      THE human entry point: README.md (overview + layer-dependency + runtime-sequence + roadmap
                                      Mermaid diagrams, per-stage tables, data sources, runbook, glossary, open questions) and
                                      raw_layers.md, terrain.md, roads.md, client_side.md
  docs/street_fidelity/               plan.md (street-level fidelity sources/architecture/phases P0-P8), p1_cross_section_design.md
                                      (P1 design); moved from out/ (out/ keeps one-line stubs)
  third_party/musl_math/              musl libm v1.2.5 port, namespace g2m::musl (OBJECT library, compiled into g2m_core)
  include/g2m/decode/                 public API of g2m_decode (no third-party types); milestone G1b
    byte_source.h                     IByteSource (abstract random-access read), FileByteSource, MemoryByteSource,
                                      DecodeError (every decoder's one error channel)
    zip/zip_reader.h                  ZipReader: EOCD/ZIP64/central directory, stored+deflate, CRC-32 (libdeflate),
                                      raw (non-UTF-8-tolerant) filenames, read_entry_as_source (nested-zip re-parse)
    tiff/tiff_types.h                 Ifd/StripOrTile/tag enums (compression, predictor, sample format, PlanarConfig)
    tiff/tiff_reader.h                TiffReader: IFD chain (classic + BigTIFF), strips/tiles, LZW/Deflate/PackBits,
                                      predictor 1/2/3, decode_window_f32 (bounds-checked sub-rectangle read)
    tiff/geotransform.h               GeoTransform (pixel-centre affine) from GeoTIFF tags; to_raster_georef()
                                      (UTM-only bridge to g2m::RasterGeoref, NorthToSouth + exact-mm alignment)
    tiff/tfw.h                        World-file (.tfw) parser; rejects rotation/non-north-up
    dgm1.h                            Hessen DGM1: enumerate_tiles/decode_tile over nested PKT_*.zip deliveries -
                                      see this file's own header comment for the real filename convention (G2M-003)
    dgm1_bw.h                         Baden-Wuerttemberg DGM1 (LGL, 2 km zips of 1 km ASCII .xyz tiles, cell-centre samples placed
                                      by their own coordinates, border tiles cropped -> unlisted sample = -9999 "not BW");
                                      Catalog::open/copies/decode, parse_xyz_tile (2026-10-07 corridor import)
    osm/pbf_reader.h                  PbfHandler (virtual-dispatch seam, no osmium type in this header),
                                      BoundingBox, read_pbf (libosmium+protozero+zlib-ng, single-threaded,
                                      streamed in file order, bbox prefilter)
  src/decode/                         implementation (mirrors include/g2m/decode/); CMakeLists.txt defines g2m_decode
    tiff/lzw.h+.cpp                   TIFF-flavored LZW decoder (ClearCode/EOI, early-change code-width growth)
    tiff/predictor.h+.cpp             Predictor 2 (horizontal differencing) / 3 (floating-point byte-plane) undo
    tiff/packbits.h+.cpp              PackBits decoder
  include/g2m/core/geom/               exact integer polygon geometry (milestone G1c I1), part of g2m_core
    exact.h                           point-in-polygon/segment/box/distance predicates, 128-bit-intermediate exact
    project.h                         OSM fixed-point lat/lon -> integer-mm UTM (shared by osm_clip.cpp and ring assembly)
    ring_set.h                        a set of closed rings + point-in-polygon / tile-vs-boundary tests
    ring_assembly.h                   assembles OSM-relation way members into simple rings (chain/split/even-odd nesting)
  src/core/geom/                       implementation (exact.cpp, ring_set.cpp, ring_assembly.cpp; project.h is header-only);
                                       compiled into g2m_core, listed in src/CMakeLists.txt
  include/g2m/layer/region.h           g2m.regions/1 (OsmAdminPart/UtmBboxPart RegionSpec) and g2m.resolved_region/1
                                       (ResolvedRegion: tile_count/interior runs/parts/geometry_hash) - milestone G1c I1/I2;
                                       interior_tile_keys(ResolvedRegion) (G2.0: the interior/halo tile keys a
                                       verification pass needs - added after "interior tiles have full DEM coverage"
                                       turned out to be an unverified, and false, claim - see PLAN.md's "G2.0
                                       correction" addendum and check-dem-nodata below)
  src/layer/osm_clip_index.cpp         OsmClipIndex (milestone G1c I4): per-tile OSM selection index ~1000x cheaper
                                       than calling clip_osm() (osm_clip.cpp, unchanged, kept as the byte-for-byte oracle)
  include/g2m/import/                  public API of g2m_import (milestone G1c I2-I6); importers stay free of
                                       server/decode/thread dependencies (layers-only + g2m_decode's readers)
    region_resolve.h                  resolve_region(spec, pbf_path): RegionSpec -> ResolvedRegion via ring assembly;
                                       overload over several PBFs (first file holding a relation wins, joined name, combined
                                       sha); part kind utm_corridor = one rectangle per polyline segment
    sink.h                            ITileSink, StoreSink(TileStore&, scope, pin=false) (G1c I7: --pin plumbed from
                                       g2m_tiler), MemorySink (std::map<{layer,key},Entry>, no SQLite needed for tests)
    report.h                          ImportReport{layer, tiles_written, tiles_skipped, warnings, wall_seconds, messages}
    ordered_parallel.h                ordered_for: fan produce(i) over up to `jobs` worker threads, deliver to
                                       consume(i, ...) in ascending index order - importer output is --jobs-independent
    dem.h                             DGM1 -> g2m.src.elev.dgm1 (I3): DemImportReport incl. tiles_written/partial/
                                       skipped_nocov/skipped_allnodata and dup_keys/dup_copies/dup_identical/dup_differing;
                                       dgm1_km_span(min_m, extent_m) -> KmSpan{lo, hi} (G2.0 fix): a level-2 tile can
                                       span up to THREE DGM1 1 km squares per axis (km-remainder 977..999), not two -
                                       the importer visits the full [lo,hi]x[lo,hi] rectangle (up to 3x3=9 squares),
                                       not just a 2x2 box's 4 corners (the old code's bug - see PLAN.md's "G2.0
                                       correction" addendum); DemSources{hessen, bw} + import_dem overload: a square in both
                                       is merged per sample (a sample BW lists wins, else Hessen, never averaged), report
                                       km_bw_only/km_merged/border_samples_*/border_max_abs_diff_units
    osm.h                             PBF -> g2m.src.osm (I4): two-stage extract (own sorted-SoA node-location index,
                                       D13) + per-tile clip (OsmClipIndex), OsmExtractReport (nodes/ways/ways_incomplete)
                                       merge_osm_data/extract_region_set(vector<pbf>): ids in both extracts kept once (nodes/ways:
                                       first file, relations: longer member list), OsmMergeReport
    landcover.h                       ESA WorldCover COG -> g2m.src.landcover (I5)
  src/import/                          g2m_import implementation (mirrors include/g2m/import/); CMakeLists.txt
                                       defines g2m_import (STATIC)
  apps/g2m_tiler/                      offline importer CLI (milestone G1c I2-I7)
    main.cpp                          subcommands: find-admin, resolve, import-dem, import-osm, import-landcover,
                                       import (all three layers via one resolved region), stats (+ check_consistency,
                                       G1c I7), index-hash (per-layer + "all" FNV-1a64 over the store index, G1c I7),
                                       diff-scopes --a DIR:SCOPE --b DIR:SCOPE (G2.0: per-tile complement to
                                       index-hash - merge-diffs two stores'/scopes' sorted TileStore::index() output
                                       into per-layer added/removed/changed/unchanged counts + per-tile lines),
                                       check-dem-nodata --store DIR --scope SCOPE --resolved PATH [--interior-only]
                                       (G2.0: decodes real g2m.src.elev.dgm1 bodies and counts actual NoData samples -
                                       exit 1 on any interior-tile NoData; see PLAN.md's "G2.0 correction" addendum
                                       for why stats/index-hash alone could not catch this)
    cli_args.h                        header-only argv parser (Args/parse_args) - extracted in I7 so a unit test can
                                       include it directly; `bool_flags` makes a flag (e.g. --pin) a presence-only switch;
                                       split_dir_scope(s) (G2.0: splits a "DIR:SCOPE" argument on the LAST ':' so a
                                       Windows drive letter's own colon doesn't confuse the split) - also header-only
                                       for the same direct-unit-test reason
    rss.h+.cpp                        peak resident-set-size sampling for each subcommand's own diagnostic summary line
  testdata/fixtures/                   committed small REAL cuts for import tests (milestone G1c I6, D19): dgm1/
                                       (PKT_fixture.zip), osm/ (frankfurt_hoechst.osm.pbf relation-closure cut,
                                       unsorted_nodes.osm.pbf), worldcover/ (two fixture TIFFs - a home-region tiepoint-
                                       aligned crop and a bbox crop, I5 deviation: one WorldCover source tile could not
                                       cover both test shapes), regions.json (the g2m.regions/1 fixture, incl. one
                                       utm_bbox part), README.md
  testdata/fuzz_seeds/                 committed corpora (D21): pbf/, tiff/, zip/ - seed inputs for the G1c decoders'
                                       fuzz targets (tests/fuzz_formats, tests/fuzz)
  tests/
    support/                          test-only: headless_env (Windows CRT dialogs off), catch_main, splitmix64.h,
                                      synthetic.h (SplitMix64 elev/landcover tiles and OSM sets for tests and the g1 golden)
    layer/                            g2m_layer_tests: test_container, test_raster_encodings, test_osm, test_release, test_pack
    server/                           g2m_server_tests: test_tile_store, test_protocol, test_server, test_client_cache
                                      (server_test_util.h: TempDir, make_body/make_header)
    fuzz_formats/                     fuzz target bodies (fuzz_run.cpp) + seeds (fuzz_seeds.cpp); g2m_fuzz_smoke_tests
                                      (every leg); fuzz_<name> libFuzzer executables + g2m_fuzz_seeds (G2M_BUILD_FUZZERS);
                                      G6 added packDirectory, httpRequest, httpResponse (http_wire.cpp compiled in)
    http/                             g2m_http_tests (G6, tag [http]): test_http_wire.cpp (pure mapping: header text,
                                      ETag/URL/base path, canonical targets, encode->decode identity of every server
                                      response, foreign statuses, broken payloads, pack directory), test_http_server.cpp
                                      (real sockets: HTTP vs in-process byte-identical for 27 requests covering every
                                      endpoint and status, with and without a base path; malformed targets/headers/
                                      methods/raw garbage; HEAD; 8-thread concurrency; transport failure -> 503; port in
                                      use; stop() with idle keep-alive; pooled connection replaced); http_test_world.h
    unit/                             g2m_unit_tests: test_det_math, test_tm_reference, test_geo_frames,
                                      test_tile_raster_hash, test_decode_zip, test_decode_tiff, test_decode_osm,
                                      test_decode_real_data (gated on env var G2M_DATA_DIR, SKIP when unset)
    route/                            g2m_route_tests (G4): test_turn_restrictions, test_route_search (synthetic cross/oneway/
                                      restriction/U-turn/limit cases, A* == Dijkstra on random restricted grids, hidden
                                      [.][bench] 90k-node grid timing incl. a test-only ALT experiment),
                                      test_route_fixture (both OSM fixtures with their restriction relations: 400 random
                                      pairs each A* == Dijkstra, restriction negative control), test_route_slice2 (snapping,
                                      point routes, ALT == Dijkstra on random grids and on fixture point queries, region
                                      graph from roads.graph tiles == whole-world graph), test_route_guidance (a scene per
                                      manoeuvre kind, invariants on 148 fixture routes, a pinned 21-manoeuvre
                                      hoechst_a66 route), test_route_map (simplification bounds, ownership/LOD/flag scene,
                                      route_polyline, every fixture LOD tiled with coverage, accuracy, size/time print);
                                      route_fixture_io.h = read_route_fixture (OSM fixture with
                                      relations); route_reference.h = the test-only
                                      Dijkstra (node and point), route validators and GraphBuilder; route_scenes.h = the
                                      synthetic cross and random restricted grids
    golden/                           golden.h/.cpp + golden_g1.cpp + golden_g1c.cpp (g2m_golden, milestone G1c I6),
                                      dump_main.cpp (g2m_golden_dump [g0|g1|g1c|...|g3c]), test_golden.cpp (g2m_golden_tests)
    fuzz/                             libFuzzer targets for g2m_decode (fuzz_zip, fuzz_tiff, fuzz_pbf) over a
                                      fuzz-instrumented copy of G2M_DECODE_SOURCES; same G2M_BUILD_FUZZERS gate and
                                      CTest `fuzz` label as fuzz_formats (WSL clang legs)
  testdata/
    geodesy/                          GeographicLib reference values (written by tools/refgen; see below)
    golden/g0_golden.txt              committed G0 golden hashes
    golden/g1_golden.txt              committed G1 golden hashes (encoded bytes of every G1 format)
    golden/g1c_golden.txt             committed G1c golden hashes (milestone G1c I6; `all 122 e7947b8adb8e5b47`)
  cmake/deps_decoders.cmake           FetchContent for g2m_decode's deps (libdeflate, zlib-ng, protozero, libosmium)
  tools/
    ci.ps1                            CI (below)
    setup_dev_env.ps1                 verify (-CheckOnly) or install the toolchain
    tsan_suppressions.txt             TSan suppressions, loaded only by the linux-tsan test preset (SQLite wal-index header reads)
    refgen/                           g2m_refgen: GeographicLib reference-value generator (G2M_BUILD_REFGEN=ON only)
    osm_tag_probe/                    street-fidelity plan P0: g2m_osm_tag_probe, OSM tag-coverage report (road classes, crossings/signals, buildings, trees) for a regions.json region cut by admin boundaries. Library g2m_osm_tag_probe_lib (tag_probe.* pure aggregation + md/json writers, scan.* three PBF passes over a ResolvedRegion), exe under G2M_BUILD_APPS, home_probe_region.json (Main-Taunus, Hochtaunus, Frankfurt), test tests/unit/test_osm_tag_probe.cpp [osm][probe]. Run: `g2m_osm_tag_probe --pbf F --region REGIONS.json#NAME --out-md F --out-json F`; deterministic (sorted maps). Snapshot: docs/osm_tag_coverage_home.md
```

Not built yet. These are placeholders only (PLAN.md 2.2/2.3; a comment at the
end of `CMakeLists.txt`):
- the other core headers `polyline.h`, `graph/`, `mesh/`;
- `include/g2m_c/`;
- `apps/g2m_cli` (`apps/g2m_tiler` - G1c - and `apps/g2m_server` - G6 - are built);
- `adapters/`;
- `tests/{determinism,protocol,bench}`;
- the targets `g2m_c` and `g2m_godot` (`g2m_http` is built - G6; `bridges/physics_sim` /
  `g2m_ps_bridge` is now built - PLAN.md R2.2 G5/G6, see "Layout" above).

## Targets

- `g2m_core` (STATIC): `src/core/**` plus the musl objects.
  - Public include dir: `include/`. `third_party/musl_math` is PRIVATE.
  - No third-party dependency.
- `g2m_musl_math` (OBJECT): `third_party/musl_math/*.cpp`, used only through
  `$<TARGET_OBJECTS:...>` in `g2m_core`.
- `g2m_layers` (STATIC): `src/layer/*.cpp`; PUBLIC `g2m_core`, PRIVATE
  `g2m_dep_zstd`, `g2m_dep_json`. Its source list is the cache variable
  `G2M_LAYERS_SOURCES` (the fuzz build compiles it again, instrumented).
- `g2m_server` (STATIC): `src/server/*.cpp`; PUBLIC `g2m_layers`, PRIVATE
  `g2m_dep_sqlite3`, `g2m_dep_json`.
- `g2m_client` (STATIC): `src/client/*.cpp`; PUBLIC `g2m_server`.
- `g2m_http` (STATIC, G6, only with `G2M_BUILD_HTTP`): `src/http/*.cpp`; PUBLIC
  `g2m_server`, PRIVATE `g2m_dep_httplib` (cpp-httplib, INTERFACE, `cmake/deps_http.cmake`),
  `g2m_warnings`, `g2m_options`. `g2m_server_app` (executable `g2m_server`, `apps/g2m_server/`,
  with `G2M_BUILD_HTTP` and `G2M_BUILD_APPS`): links `g2m_http` + `g2m_client`, reuses
  `apps/g2m_tiler/cli_args.h`. Tests `g2m_http_tests` (`tests/http/`, with `G2M_BUILD_HTTP`).
- `g2m_mesh` (STATIC, PLAN.md G2.3): `src/mesh/*.cpp` (terrain_chunk.cpp,
  lod_select.cpp); PUBLIC `g2m_core` + `g2m_layers` (for `HeightTile`), no
  other dependency - no rendering-engine or third-party type anywhere in
  `include/g2m/mesh/`.
- `g2m_phys` (STATIC, PLAN.md R2.2 section 1a): `src/phys/*.cpp`
  (physics_grid.cpp, resident_heights.cpp, height_tile_loader.cpp,
  physics_streamer.cpp -
  fill_physics_heights.h is a header-only template, no .cpp of its own);
  PUBLIC `g2m_client` (transitively `g2m_server` / `g2m_layers` /
  `g2m_core`, for `HeightTile` and `ITransport`), PRIVATE `Threads::Threads`.
  Built UNCONDITIONALLY, not gated by `G2M_BUILD_IMPORT` - every CI leg,
  including the sanitizer legs, exercises it. No physics_sim/Jolt/Godot type
  anywhere in `include/g2m/phys/`, and it never links physics_sim.
- `g2m_world` (STATIC, `src/world/*.cpp`, terrain migration phase 1): PUBLIC
  `g2m_client` + `g2m_mesh` + `g2m_phys`, PRIVATE `g2m_warnings`/`g2m_options`.
  Built unconditionally. No physics_sim/Jolt/Godot type; never links
  physics_sim. Tests `g2m_world_tests` (tests/world: test_tile_fetch,
  test_height_sampler, test_chunk_road_classes, test_surface_id_table; tag
  `[world]`): values pinned from racing_game a8a1b13's own code (verbatim
  extraction in `tests/world/refgen/`, not built by default), ported game
  cases, `sample_height_exact` against `build_chunk`'s triangles and (bridge
  leg, `test_render_vs_jolt.cpp`) against a Jolt ray cast.
  Phase 2 (`TerrainService`): links `g2m_server` too and PRIVATE
  `Threads::Threads`. Tests `test_terrain_service.cpp` (tag `[terrain_service]`;
  synthetic 5 x 5 DEM + service-road world): one derivation for concurrent
  requests, priority order (equal priority = request order, a re-request of a
  queued tile raises it), cancel/drop/shutdown without leak or deadlock,
  bit-identity of every byte with the plain serial server, warm fills the
  derived store, nested producers on a pool of one. The server resolver now
  deduplicates per (release, layer, key) (`server.h`; test
  `test_server.cpp` "concurrent cold requests derive each tile exactly once").
  `g2m_terrain_service_bench` (tests/world/bench_terrain_service.cpp, not a
  CTest test): cold-store throughput, serial server vs pool vs warm.
  Phase 3a (`road_structures`, `road_ribbons`): tests `test_road_meshes.cpp`
  (tags `[road_structures]`, `[road_ribbons]`, `[junction]`): deck and ribbon
  bytes of 55 cases (`road_mesh_test_data.h`: three synthetic scenes incl. a
  motorway overpass, every tile of both OSM fixtures, smooth and holed ground,
  grade separation off and on) equal `road_mesh_pins.inc`, recorded from
  racing_game 945455a's own code (the cases without separation are unchanged
  from 0c027dc) (`refgen/extract_mesh_ref.sh` +
  `refgen/refgen_meshes.cpp`, target `g2m_world_mesh_refgen` only with
  `-DG2M_WORLD_MESH_REFGEN_DIR=<dir>`), plus the game's two junction-surface
  tests. g2m_world_tests links `g2m_decode` and reads the OSM fixtures.
  Phase 3b (`buildings`, `view_streamer`, `ring_streamer`): tests
  `test_buildings.cpp` (`[buildings]`; synthetic tag scene + every fixture tile
  with buildings == `building_pins.inc`), `test_view_streamer.cpp`
  (`[view_streamer]`; static builds, key selection, urgency order and every step
  of a streamer drive incl. velocity, road-LOD and view-distance changes ==
  `view_pins.inc`, 1 vs 3 threads, colour hook, cancel), both recorded from
  racing_game 5a3baed's own C++ (`refgen/extract_view_ref.sh` +
  `refgen/refgen_view.cpp`, target `g2m_world_view_refgen` only with
  `-DG2M_WORLD_VIEW_REFGEN_DIR=<dir>`); `test_ring_streamer.cpp`
  (`[ring_streamer]`) replays `ring_pins.inc`, recorded by running the game's
  road_stream.gd / building_stream.gd headless in Godot over 8 focus steps
  (`refgen/extract_ring_ref.sh` + `refgen/ring_harness.gd`; job order within
  1 m because GDScript works in float32), plus reject/downgrade/evict cases.
- `g2m_ps_bridge` (STATIC, `bridges/physics_sim/`, PLAN.md R2.2 section 1b,
  G5/G6): `src/version.cpp`, `src/g2m_terrain_source.cpp`; PUBLIC `ps_core`
  (physics_sim's own target, embedded via `G2M_PS_SOURCE_DIR` - see "CMake
  options" below) + `g2m_phys`, PRIVATE `g2m_warnings`/`g2m_options`. Only
  ever `add_subdirectory`'d behind the root CMakeLists.txt's
  `if(TARGET ps_core)` guard, so it silently does not exist in any
  configuration that did not embed physics_sim.
- Third-party (`cmake/deps_core.cmake`, compiled as our own targets, no
  g2m_warnings/options): `g2m_dep_zstd` (STATIC, C), `g2m_dep_sqlite3`
  (STATIC, C), `g2m_dep_json` (INTERFACE, header-only).
- `g2m_warnings` (INTERFACE): `/W4 /WX`, or `-Wall -Wextra -Wpedantic -Werror`.
  Linked into our own targets only, never into third-party ones.
- `g2m_options` (INTERFACE): FMA contraction is off everywhere. No fast-math.
  - MSVC family: `/arch:AVX2 /fp:precise /EHsc`, plus
    `/clang:-ffp-contract=off` for clang-cl.
  - Otherwise: `-mavx2 -mfma -ffp-contract=off`.
- Static CRT (`CMAKE_MSVC_RUNTIME_LIBRARY` MultiThreaded[Debug]), set before
  `project()`, the same as physics_sim.
- Test-only targets:
  - `g2m_headless_env`, `g2m_test_catch_main` (tests/support);
  - `g2m_unit_tests`, `g2m_layer_tests`, `g2m_server_tests`, `g2m_mesh_tests`
    (tests/mesh, PLAN.md G2.3 - also has its own `alloc_counter.cpp`, a
    global operator new/delete counting override scoped to this one
    executable, for `build_chunk`'s zero-allocation-on-repeat-call check),
    `g2m_phys_tests` (tests/phys, PLAN.md R2.2 section 1a/4a -
    test_physics_grid.cpp, test_resident_heights.cpp,
    test_fill_physics_heights.cpp, test_height_tile_loader.cpp (scripted
    fetch + fake ITransport, no network/store files),
    test_physics_streamer.cpp (incl. a TileManager pending-queue model for
    the queued-key invariant, plus a gate_margin_tiles = 0 negative
    control); also its own `alloc_counter.cpp` for
    `fill_physics_heights`'s zero-allocation check at samples = 256 and
    1024, same pattern as `g2m_mesh_tests`), `g2m_fuzz_smoke_tests`,
    `g2m_golden_tests` (Catch2;
    tags become CTest labels through `ADD_TAGS_AS_LABELS`);
  - `g2m_fixture_release` (STATIC, `tests/support/fixture_release.{h,cpp}`,
    PLAN.md R2.2 section 1c/G5): `make_fixture_release(tmpdir)` builds the
    PKT DGM1 fixture release (import -> store -> server) once, shared by
    `test_derive_terrain.cpp` and the bridge tests below; PUBLIC
    `g2m_layers`/`g2m_decode`/`g2m_import`/`g2m_server`/`g2m_client` +
    `Catch2::Catch2` (`REQUIRE` inside `make_fixture_release` itself);
  - `g2m_bridge_tests` and `bench_bridge_fill` (`bridges/physics_sim/tests/`,
    PLAN.md R2.2 section 1c/4b, G5-G8 - **built only as part of the
    `bridge-release` preset/leg**, see "Presets"/"CI" below): Catch2
    `g2m_bridge_tests` (test_smoke.cpp, test_g2m_terrain_source.cpp,
    test_render_vs_jolt.cpp, test_residency.cpp, test_determinism.cpp,
    test_seam_crossing.cpp; links `g2m_ps_bridge` + `g2m_mesh` +
    `g2m_fixture_release` + `g2m_test_catch_main`) and the plain-`main`
    `bench_bridge_fill` (CTest smoke only, label `bench;bridge`; links
    `g2m_ps_bridge` + `g2m_headless_env`) - see the "Layout" entry above for
    what each test file covers. Deliberately does not link
    `tests/mesh/alloc_counter.cpp` or `tests/phys/alloc_counter.cpp` (both
    override global operator new/delete, which would clash with `ps_core`'s
    own override);
  - `g2m_golden` (STATIC, links g2m_layers + g2m_server + g2m_mesh) and
    `g2m_golden_dump` (exe; sets: `g0`, `g1`, `g1c`, `g2`, `g2mesh`, `g2roads`);
  - `g2m_fuzz_run`, `g2m_fuzz_seeds_lib` (STATIC);
  - with `G2M_BUILD_FUZZERS`: `g2m_fuzz_instrumented` (OBJECT:
    `G2M_LAYERS_SOURCES` + `src/server/protocol.cpp` + `src/http/http_wire.cpp` + `fuzz_run.cpp`,
    `-fsanitize=fuzzer-no-link`), `fuzz_{container,elev,landcover,osm,pack,manifest,path,region,
    roadgraph,roadgeom,roadCrossSectionEval,packDirectory,httpRequest,httpResponse}`
    (`-fsanitize=fuzzer`), `g2m_fuzz_seeds` (writes the corpus; CTest
    fixture). The CTest tests `fuzz_<name>` run 10 s each
    (`G2M_FUZZ_SECONDS`, `-seed=1`), label `fuzz`.
- `g2m_refgen` (exe, `G2M_BUILD_REFGEN=ON` only): links GeographicLib.
- `g2m_decode` (STATIC, milestone G1b): `src/decode/**`, links `g2m_core`
  PRIVATE plus `libdeflate::libdeflate_static` and (via `g2m_dep_libosmium`)
  `zlib`/`g2m_dep_protozero`. No third-party type in any public header
  (`include/g2m/decode/`).
- `tests/fuzz` (`G2M_BUILD_FUZZERS`, default OFF): libFuzzer targets for
  `g2m_decode`; clang only, not clang-cl (not exercised by the Windows
  presets).
- `g2m_import` (STATIC, milestone G1c I2-I6): `src/import/*.cpp`
  (region_resolve, sink, report, dem, osm, landcover -
  `ordered_parallel.h` is header-only, no library code); PUBLIC `g2m_server`
  (for `sink.h`'s `StoreSink`/`TileStore`), PRIVATE `g2m_decode` (D22 - no
  osmium/protozero/TIFF type in any `g2m/import` public header).
- `g2m_tiler` (executable, `apps/g2m_tiler/`, milestone G1c I2-I7): links
  `g2m_import` only (+ its own `rss.cpp`); gated by `G2M_BUILD_APPS`
  independently of `G2M_BUILD_TESTS`, so it never depends on
  `tests/support`. See its own Layout entry above for subcommands.

## CMake options

- `G2M_BUILD_TESTS` (ON): Catch2 v3.16.0 via FetchContent, plus `tests/`.
- `G2M_BUILD_IMPORT` (ON, G2.0): gates the whole decoder/importer stack -
  `cmake/deps_decoders.cmake`'s FetchContent (libdeflate/zlib-ng/protozero/
  libosmium), `src/decode`, `src/import`, `apps/g2m_tiler` - behind one
  switch, for a future pure consumer of the tile-serving stack (e.g. a
  game bridge, see `bridges/physics_sim` above) that never needs it.
  `G2M_BUILD_APPS`/`G2M_BUILD_TESTS` both `FATAL_ERROR` if ON while this is
  OFF (both need `apps/g2m_tiler`/its tests). The `consumer-check` preset
  (below) proves OFF actually builds - only `g2m_core`/`g2m_layers`/
  `g2m_server`/`g2m_client` and their own deps (zstd, SQLite, nlohmann_json)
  compile, no Catch2/decoder deps fetched.
- `G2M_BUILD_HTTP` (default `PROJECT_IS_TOP_LEVEL`, G6): fetches cpp-httplib and builds
  `g2m_http`, `tests/http` and (with `G2M_BUILD_APPS`) `apps/g2m_server`. OFF in a consumer
  that `add_subdirectory`s geo2map_engine (racing_game), so it fetches nothing new there.
- `G2M_BUILD_REFGEN` (OFF): fetches GeographicLib v2.7
  (`GEOGRAPHICLIB_PRECISION=3`, long double) and builds `tools/refgen`.
  - Only on Linux x86-64, because it needs an 80-bit long double.
- `G2M_SANITIZER_BUILD` (OFF): set ON by the `asan`, `linux-asan-ubsan` and
  `linux-tsan` presets.
- `G2M_BUILD_FUZZERS` (OFF): clang only; set ON by the `linux-release` and
  `linux-asan-ubsan` presets.
- Configure fails on purpose if an optimised build type has no `-O`/`/O` flag
  (a stale cache). Fix it with `cmake --preset <name> --fresh`.
- `G2M_PS_SOURCE_DIR` (empty, PLAN.md R2.2 section 1b/1c, only used by the
  `bridge-release` preset): path to a physics_sim checkout at
  `bridges/physics_sim/physics_sim.pin`'s own commit. When set, the top
  level `add_subdirectory`s it (as `ps`, `EXCLUDE_FROM_ALL`) BEFORE its own
  Catch2 `FetchContent_Declare` (mirrors `racing_game/CMakeLists.txt`'s
  ordering so nlohmann/Catch2 target names don't clash), and
  `bridges/physics_sim` (`g2m_ps_bridge`) is only `add_subdirectory`'d once
  `ps_core` exists as a target. Empty skips both entirely - every other
  preset never touches physics_sim.

## Dependencies (FetchContent, pinned)

| Dependency | Pin | Use |
|---|---|---|
| Catch2 | `v3.16.0` | tests only; `/EHsc` on its targets under MSVC, `-mllvm -asan-stack=0` under clang-cl ASan |
| GeographicLib | `v2.7` | `tools/refgen` only; never linked into g2m_core or a test |
| zstd | `v1.5.7` (release tarball, SHA-256) | g2m_layers; static-asserted version; level 12, every parameter pinned |
| SQLite | `3.53.4` amalgamation (SHA3-256) | g2m_server tile stores |
| nlohmann_json | `v3.12.0` (release tarball, SHA-256) | g2m_layers / g2m_server, PRIVATE only |
| libdeflate | `v1.26` | `g2m_decode`: DEFLATE + CRC-32 for ZIP and TIFF/COG |
| zlib-ng | `2.3.3` | `g2m_decode`: zlib-API DEFLATE for libosmium's PBF blob decompression only |
| protozero | `v1.8.2` | `g2m_decode`: header-only protobuf decode, under libosmium |
| libosmium | `v2.23.1` | `g2m_decode`: OSM PBF parsing (`src/decode/osm/`) |
| cpp-httplib | `v0.58.0` (git commit `4f3f9ef1`) | `g2m_http` only (PRIVATE, header-only, no TLS/zlib), `G2M_BUILD_HTTP` |

## Presets

- Every preset sets `CMAKE_C_COMPILER` too (zstd and SQLite are C).
- `G2M_BUILD_HTTP` is on in every top-level preset except `consumer-check` and `bridge-release`
  (both set it OFF: no extra fetch in a consumer configuration, and `bridge-release` is fully
  disconnected).
- Windows (Ninja, from a VS 2022 Build Tools environment):
  - `debug`, `release`, `relwithdebinfo`, `asan` (clang-cl);
  - `msvc-debug`, `msvc-release` (cl);
  - `consumer-check` (clang-cl, G2.0): `CMAKE_BUILD_TYPE=Release`,
    `G2M_BUILD_IMPORT=OFF`, `G2M_BUILD_TESTS=OFF`, `G2M_BUILD_APPS=OFF` -
    proves a pure consumer configuration builds `g2m_layers`/`g2m_server`/
    `g2m_client` alone; `tools/ci.ps1`'s own leg builds exactly those three
    targets and nothing else.
  - `bridge-release` (clang-cl, PLAN.md R2.2 G5/G6): `release` plus
    `G2M_BUILD_IMPORT=ON`, `G2M_BUILD_TESTS=ON`, `PS_BUILD_TESTS=OFF`,
    `PS_BUILD_GODOT_ADAPTER=OFF`, `G2M_PS_SOURCE_DIR=out/ps_pin/src` and
    `FETCHCONTENT_FULLY_DISCONNECTED=ON` with `FETCHCONTENT_SOURCE_DIR_*`
    overrides for every one of physics_sim's AND this repo's own
    FetchContent deps (JoltPhysics/nlohmann_json redirected at physics_sim's
    own already-built `_deps`; Catch2 and the decoder-stack deps redirected
    at this repo's own `release` preset's `_deps`) - `FULLY_DISCONNECTED` is
    a global switch, so every FetchContent dependency in the whole configure
    needs an offline source, not just the two `g2m_ps_bridge` itself needs.
    Not in the default `tools/ci.ps1` run (see "CI" below).
  - build dirs are `out/build/<preset>`.
- WSL (clang + lld):
  - `linux-release`, `linux-asan-ubsan`, `linux-tsan`;
  - build dirs are `~/build/geo2map_engine/<preset>`, never under `/mnt/s`;
  - `linux-tsan` symbolizes with `/usr/bin/addr2line` and loads
    `tools/tsan_suppressions.txt`.

## Conventions (what the code does)

- **Deterministic math**
  - Code under `include/` and `src/` calls `g2m::det::*`, never libm
    transcendentals or `<complex>`. The `ci.ps1` grep enforces this.
  - Exempt from the grep: `det_math.h`, `det_math.cpp` and
    `third_party/musl_math`.
  - IEEE-exact functions (sqrt, fabs, floor, ceil, trunc, fmod, remainder,
    copysign, scalbn) may be called from `std::`.
- **Transverse Mercator**
  - Degrees in and out. `forward(lon0, lat, lon)` returns
    `{x, y, gamma, k}`, and `reverse(lon0, x, y)` returns
    `{lat, lon, gamma, k}`.
  - `x`/`y` have no false offsets.
  - `Utm` adds 500 km false easting and, for the south, 10,000 km false
    northing.
  - Any zone works for any point (extended zone).
- **TileKey**
  - Level L is 256·2^L m.
  - Indices are `floor_div(floor(E), size)` in integers, which is exact.
  - x/y are signed 24-bit.
  - Packed layout: zone bits 54-59, hemisphere bit 53 (1 = south), level
    bits 48-52, y + 2^23 in bits 24-47, x + 2^23 in bits 0-23.
  - The numeric order of `packed()` equals `operator<=>`, which orders by
    (zone, hemi, level, y, x).
  - URL form: `utm32n/<level>/<x>/<y>`.
- **SessionFrame**
  - Local x = E − E0 and y = N − N0.
  - E0 and N0 are multiples of 1000 m.
  - It is computed from TM x/y minus integer offsets, not from the full
    northing.
  - `tile_at` uses `floor(local) + origin` in integers.
- **Raster georeference**
  - The footprint's SW corner and spacing are in integer mm.
  - Sample positions are in integer half-mm.
  - `Node` places a sample on a lattice node: n cells give n + 1 samples,
    shared borders.
  - `Cell` places a sample at the cell centre, half a spacing in; this is
    Hessen DGM1.
  - `RowOrder` is explicit.
  - `lattice_offset(a, b)` returns the integer (dcol, drow, row_flip) when
    every sample of b lies on a's lattice (copy without resampling), and is
    empty otherwise.
- **Heights**: int32 in 1/256 m units, `kHeightNoData = INT32_MIN`.
- **Errors**: library code returns `Result<T>` and never throws; JSON is
  parsed with exceptions off.
- **Formats**: little-endian, canonical varints, pinned zstd; decoders of
  untrusted bytes are bounds-checked and fuzzed (`docs/formats/`).
- **Offline**: `ServerConfig::offline` drops the fetcher, so no network
  call is reachable.
- **Test inputs**: from `g2m_test::SplitMix64`, never `std::`
  distributions. Draw one value per statement, because argument evaluation
  order differs between ABIs.

## Building and testing

```powershell
# from a VS 2022 developer shell (tools\ci.ps1 enters one itself)
cmake --preset release
cmake --build --preset release
ctest --preset release --output-on-failure
.\out\build\release\tests\golden\g2m_golden_dump.exe      # prints the golden hashes
```

```bash
# inside WSL
cd /mnt/s/claude_code/geo2map_engine
cmake --preset linux-release && cmake --build --preset linux-release && ctest --preset linux-release
```

Regenerate the reference values (WSL only; commands also in
`tools/refgen/CMakeLists.txt`):

```bash
cmake -S . -B ~/build/geo2map_engine/refgen -G Ninja -DCMAKE_BUILD_TYPE=Release \
      -DCMAKE_CXX_COMPILER=clang++ -DG2M_BUILD_TESTS=OFF -DG2M_BUILD_REFGEN=ON
cmake --build ~/build/geo2map_engine/refgen --target g2m_refgen
~/build/geo2map_engine/refgen/tools/refgen/g2m_refgen testdata/geodesy
```

`testdata/geodesy/*.txt` holds one record per line, with hex doubles and each
result stored as hi + lo:
- `tm_grs80.txt`: 10,000 forward and 10,000 reverse points, including ±9°;
- `tm_wgs84.txt`: 1,000 points;
- `ecef_grs80.txt`: 1,000 points;
- `utm_zone.txt`: 4,000 points.

Golden hashes are in `testdata/golden/g0_golden.txt` and `g1_golden.txt`,
one line per entry: `name count fnv64`, where `all` covers every entry of
the set. Regenerate them only after an intended output change: run
`g2m_golden_dump <g0|g1> <file>`, then re-add the header comment.

Fuzzers (WSL, `linux-release` or `linux-asan-ubsan` build dir):
`ctest -L fuzz`, or run one by hand from `tests/fuzz_formats`:
`./fuzz_osm -max_total_time=60 corpus/work/osm corpus/seeds/osm`.

## CI

```powershell
tools\ci.ps1                               # every leg + grep + golden compare
tools\ci.ps1 -SkipWsl                      # Windows legs only
tools\ci.ps1 -SkipWindows                  # WSL legs only
tools\ci.ps1 -Only release,linux-release   # named legs only
```

- **Legs:** `debug`, `release`, `asan`, `msvc-release` (Windows);
  `linux-release`, `linux-asan-ubsan`, `linux-tsan` (WSL). The legs run one
  after another. A `consumer-check` step (G2.0) runs right after the
  Windows legs: configures+builds the `consumer-check` preset
  (`G2M_BUILD_IMPORT=OFF`), only `g2m_layers`/`g2m_server`/`g2m_client` -
  no `ctest`/golden dump (tests are OFF in that configuration).
- **`bridge-release` (PLAN.md R2.2 G5/G6):** NOT in the default set above -
  reachable only via `tools\ci.ps1 -Only bridge-release[,release]`. Before
  the generic Windows-leg runner, it stages `out/ps_pin/src` as a
  `git clone --local` of `S:\claude_code\physics_sim` checked out to
  `bridges/physics_sim/physics_sim.pin` (reused as-is if already at that
  pin) - a filesystem-local clone, no network, `physics_sim` itself
  untouched. A missing offline dependency directory (see the preset's own
  entry above) is an actionable configure-time failure, not a silent
  network fetch.
- **Per leg:** configure, build, `ctest` (every test), then
  `g2m_golden_dump <set>` into `out/ci_golden/<leg>.<set>.txt` for each set
  (g0, g1). On `linux-release` and `linux-asan-ubsan` the ctest run includes
  the libFuzzer runs (label `fuzz`), and a `fuzz` result line counts them.
- **Checks:**
  - libm grep over `include/` and `src/`, with a pattern self-test;
  - golden compare: every leg's dump of each set must equal
    `testdata/golden/<set>_golden.txt`.
- **Output:** logs go to `out/ci_logs/`, and the summary prints PASS/FAIL per
  leg and stage. The exit code is non-zero on any failure.

## Dev environment

```powershell
tools\setup_dev_env.ps1 -CheckOnly   # verify only, installs nothing
tools\setup_dev_env.ps1              # install what is missing (winget / apt)
```

## Terrain prefetch snapshots (2026-10-01)

ResidentHeightSet uses copy-on-write entry storage. PhysicsTerrainStreamer
publishes an immutable ResidentHeightSet view after its owner-thread update and
purges retired views there, outside World::step. Tick-side snapshot acquisition
copies only a shared_ptr. Old views retain height and road objects until their
last reader releases them AND the owner purges them; ticket release cannot
free storage inside the physics tick. Mutating published HeightTile or road
objects through a separate mutable alias is forbidden.

G2mTerrainSource advertises Snapshot support. acquire_prefetch_inputs declines
unpublished or incomplete footprints (missing heights, or missing road lists
in road mode). Confirmed Absent tiles and successful empty roads are valid.
fill_tile_from reads only the captured view. prefetch_inputs_current compares
lookup state plus height and road pointer identities for the tile's complete
L0 footprint; unrelated residency changes do not invalidate a build.
make_terrain_config accepts optional prefetch margin and ticket cap (zero/off
by default). Bridge physics pin is b90a531.

Snapshot tests cover replacement/eviction lifetime, owner purge, concurrent
read/fill against live mutation, relevant height/road invalidation, zero
allocation callbacks and on/off per-tick crossing hashes at 1/4/24 workers.

## G3 road graph foundation (2026-10-02)

`g2m/layer/road_graph.h` exposes `RoadGraph` and `build_road_graph` from validated
canonical OSM. It preserves node/way ids, reference order and all way tags,
including bridge/tunnel metadata, with stable per-ref-pair edge ids. Output is
sorted; shared node ids define connectivity. It lives in g2m_layers and changes
no existing served layer, height raster or runtime behavior. Tests are in
`tests/layer/test_road_graph.cpp`, including the real Frankfurt-Hoechst fixture.
Remaining slices and acceptance are in `docs/g3_road_pipeline.md`; G3 is not yet
complete. The opt-in g2m.roads.graph layer now has a bounded codec and nine-input deriver.
Physical tags, complete DEM dependency enumeration, immutable DEM sampling,
G1 clothoid fitting and constrained cubic profiles are implemented foundations.
IDeriver::additional_inputs_for provides pure discovery stages (`discovery_stages()`, default 1) for distant
DEM inputs. See docs/g3_profile_foundation.md for limits and project defaults.
## Street fidelity P1 (cross-section), in progress (2026-10-05)

Design: `docs/street_fidelity/p1_cross_section_design.md` (owner decisions at its end are binding).
Slice 1: `roads.geom` also depends on the `roads.graph` tiles holding every way incident to a node of
the tile's ways (`road_context_dependencies`, `merge_road_graphs`, `RoadReferenceBuilder::degree/join`),
so stretches are tile-invariant, and both stretches of a degree-two way join end on one shared bisector
heading. Slice 2: `roads.graph` encoding 5 (node tags, whitelist in `road_node_tag_key_allowed`; 4 still
decodes), `include/g2m/layer/road_rules.h` + `src/layer/road_rules.cpp` (strict loader of
`data/road_rules/de.json`, embedded through a CMake-generated `road_rules_de.inc`, `builtin_road_rules_de()`,
`.gitattributes` keeps the JSON byte-exact). Tests: `test_road_context.cpp`, `test_road_rules.cpp`,
`test_road_graph_tile.cpp` (node tags), shared helpers in `tests/support/road_fixture.h`.
Slice 3: `include/g2m/layer/road_cross_section.h` + `src/layer/road_cross_section.cpp`: `RoadCrossSection`
(bands, anchor, offset, joins; pure `band_width_m`/`offset_m`/`boundaries_at`, `road_smooth_step`) and
`resolve_cross_section_base` (tags + rules -> bands, anchor, field declines; join ramps and offsets come in
slice 4). Test: `test_road_cross_section.cpp`.
Slice 4: `src/layer/road_cross_section_join.cpp`: `resolve_cross_section(builder, way, stretch, length_m, rules)`
(join resolution in the lower way id's frame so both stretches derive one outcome; lane add/drop ramps,
width ramps, absorber offsets Ramps/Hold/Transition, ramp clipping to the stretch length) and
`lateral_point(reference, s, t)`. Second committed fixture `testdata/fixtures/osm/hoechst_a66.osm.pbf`
(see testdata/fixtures/README.md). Test: `test_road_joins.cpp` (synthetic joins, design F1-F10/H1-H7,
edge continuity over every degree-2 join of both fixtures, context invariance).
Slice 5: `roads.geom` encoding 6 (`kRoadGeomEncoding`; 5 still decodes, `RoadGeomTile::encoding`): section 1 is the
roads.graph encoding-5 payload, section 2 unchanged, section 3 one cross-section record per entry
(`RoadGeomEntry::cross_section` / `cross_section_decline`, built by `build_road_geom_tile` via
`resolve_cross_section` with `builtin_road_rules_de()`), section 4 profile features (`ProfileFeature`, filled
in slice 7). Validation per design 7.2 in `validate_road_geom_tile`. Config hash carries `xs=1 enc=6
rules=de:<hex>`. Golden `g3xs` (`tests/golden/golden_g3xs.cpp`, `testdata/golden/g3xs_golden.txt`, Windows =
Linux), fuzz `Target::RoadCrossSectionEval`. Tests: `test_road_geom_xs.cpp`. Roadgeom golden body SHA is now
`17d9b7c7...5ee2f9` (was `4dcfcbca...445f8`, the encoding-5 body, still reproduced by the test).
Slice 6: consumer API. `road_cross_section.h`: `carriageway_edges`, `carve_footprint` (`CarveFootprint`, `CrownKind`,
kerb line, `kRoadKerbHeightM`), `driving_lanes_at` (`LaneQuery`); implemented in
`src/layer/road_cross_section_api.cpp`. `include/g2m/layer/road_envelope.h` / `src/layer/road_envelope.cpp`:
`envelope_road_segments(tile, l0_key, spacing_m)` (grip-raster capsules from the same footprint; half-width = mean over
the station pair, not the design text's max, because the round caps already overpaint). Test: `test_road_api.cpp`.
Slice 7: profile features (D-SF8, owner decision Q2). `src/layer/road_features.cpp`: `resolve_profile_features` (called by
`build_road_geom_tile` per entry with a cross-section) turns `traffic_calming` on the interior nodes of a stretch into
`ProfileFeature` records; defaults per kind live in `data/road_rules/de.json` (`traffic_calming`, `TrafficCalmingRules`,
`RoadRules::calming`; `ProfileFeatureKind` moved to `road_rules.h`). Every kind with a height is carved (bump,
mini_bumps, rumble_strip, cushion, hump, table, dip as a negative depth), chicane/choker/island are render only. The
height function `feature_height_m` / `feature_height_contribution_m` is a plateau with two C1 smooth-step ramps;
`kFeatureMinRampMm` = 1000 (1 m ramp per side, grid property, not a country rule), tables use 1:10 ramps
(`kFeatureTableRampRatio`); carved length = `max(length, 2 * ramp)` computed, nominal length stored. Features are
shortened to stay inside the stretch (Derived) or dropped under 1 m of room. The `de.json` hash changed
(`7e387db8...b69a`), so the config hash and release id changed; golden `g3xs` changed in exactly one tile (the
frankfurt_hoechst tile that holds the fixture's one island node, +22 bytes). Tests: `test_road_features.cpp`, rules tests
in `test_road_rules.cpp`; `[.][diag]` case prints the effective carved shapes on the 1 m grid.

## Street fidelity P2 (junctions, carving, furniture; design `docs/street_fidelity/p2_junction_design.md`)

Slice 1 (rules and parameter files, no new stage): `data/road_rules/de.json` gained the `junction` and `furniture`
blocks (`JunctionRules`, `FurnitureRules`, `CrossingPattern`, `RoadRules::junction`/`::furniture` in `road_rules.h`;
every key is in integer mm or counts, the loader proves `cluster_max_extent + 2*link_trim_cap <= reach_max` and
`reach_max + kCarveBlendMaxM + extent <= kCarverRadiusM`). The de.json SHA moved to `186e11c4...8435` (was
`7e387db8...b69a`), which moves the roads.geom config hash and release id; goldens did not move. New embedded data:
`data/carve/carve_default.json` (`g2m.carve/1`, `include/g2m/layer/carve_params.h`, `load_carve_params`,
`builtin_carve_default()`, deliberately NOT in the roads.geom config hash) and
`data/classifiers/esa_worldcover_v200.json` (`g2m.classifier_table/1`, `landcover_classifier.h`, `LandcoverClassifier`,
its file SHA is the classifier identity for the later terrain.class config hash). The shared strict JSON reader moved
to `src/layer/strict_json.h`; `src/layer/CMakeLists.txt` embeds all three JSON files through `g2m_embed_json`.
Tests: `test_road_rules.cpp` (junction/furniture cases), `test_carve_params.cpp`, `test_landcover_classifier.cpp`.

Slice 2: `include/g2m/layer/road_surface.h` / `src/layer/road_surface.cpp`: `road_surface_height_m(tile, entry, s, t, params)`
is the one carved road height (design 4.1: `p(s) + C(s,t) - C(s,0) + V(s,t) + F(s,t)`; helpers `road_crown_m`, `road_verge_m`).
`RoadSurfaceParams{crown_round_m, verge_drop_m}` defaults equal `carve_default.json` (`CarveParams::surface_params()`); the
crossfall is the profile's `attributes.crown_per_mille`. Features act over the carriageway (width 0 and cushions) or a width
centred on it, with no lateral blend; NaN for an entry without profile or cross-section. No encoding change. Test:
`test_road_surface.cpp` (T1, sabotage: dropping `-C(s,0)` breaks the reference-equals-profile checks).

Slice 3: `include/g2m/layer/road_junction.h` / `src/layer/road_junction.cpp`: `build_node_junction(tile, context, node_id, rules, stats)`
and `build_node_junctions(...)` (offset-intersect-trim per design 3.3-3.8, pure, no tile or layer output yet). Per node: approaches
sorted by azimuth, three rings (RoadBed, Paved, Outer) from the P1 `carve_footprint`/`lateral_point` edges as integer-mm polylines,
corners by exact int64 segment intersection (`geom::orient`/`segments_intersect`), trims from the Outer layer, bow-tie repair by
1 m extension rounds of the involved approaches (`junction.bowtie_rounds`), convex-hull fallback with `JunctionFlags::HullFallback`,
reach cap (`ReachExceeded`), declines `ClusterTooLarge/ReachExceeded/SingleApproach/Degenerate/Underground`, per-approach status
(Used/NoProfile/NoCrossSection/Tunnel...). A node is built only when the tile holds every approach of it (else skipped, counted).
`junction_ring_valid` is the simple-and-counter-clockwise check. Test: `test_road_junction.cpp` (X/T/Y/fork/star scenes, Hoechst
and Frankfurt fixture census, multi-tile identity). Sabotages: swapping left/right in `edge_t` fails the corner/trim checks;
disabling bow-tie repair and hull fallback fails the repair/hull cases and the census. No served bytes change.

Slice 4: same files. `build_road_junctions(tile, context, rules, stats, options)` adds trim-overlap clusters (design 3.2,
decisions D16-D21): links over one entry when the capped single-node trims cover it (roundabout ways never link), bounded BFS
with caps 48 m / 12 nodes, owner = smallest node. A cluster's ring is the outer face of the darts' rotation system (absorbed
stretches are two darts, D16). A node whose discovery reaches a possible link to an incomplete node is skipped
(`skipped_unresolved`); an over-cap discovery keeps the singles plus a `ClusterTooLarge` record from its smallest node (D19).
All builders now close obtuse open pairs on cap segments at the node and send every layer to the hull when one needs it (D17,
ring nesting). `JunctionBuildOptions` switches exist for tests only. Tests: `test_road_junction.cpp` T4 cases, D17 cases,
cluster census with multi-tile identity and its negative control (`unknown_links_are_absent`), a `[.][diag]` for two-stretch
links. No served bytes change.

Slice 5: `road_junction.h` / `road_junction.cpp`: junction heights and raised tables (design 4.2-4.4, decisions D22-D24).
`make_junction_surface(tile, junction)` builds a `JunctionSurface` (per used approach: the 0.5 m reference polyline to trim + 1, the
mm-rounded outer trim segment, an optional ramp record); `junction_height_m(surface, tile, x, y)` = sum of w_i z_i with the product
form of inverse squared distances to the trim segments (exact on a trim line, C1, convex), z_i = `road_surface_height_m` at the
projection on the approach's reference polyline with a tangent extension before station 0 (`junction_approach_height_m`,
`junction_weights`); pure, allocation-free, det::sqrt only. A table or hump tag on a junction node sets `JunctionFlags::RaisedTable`,
`raised_height_m` and `raised_node` (greatest in a cluster; other vertical kinds raise nothing and count `calming_ignored`);
the raise enters through per-approach ramp records over the carriageway only (D24), not as a ring-wide constant.
`adjust_features_for_junctions(tile, junctions)` returns the kept features (an approach feature whose footprint reaches the trim is
suppressed, D23), the ramp records (kind Table, plateau [0, trim], ramp max(10 x height, 1 m), D22) and `core_features()` for the
carver. `tests/support/road_fixture.h` `geometry_in_tile` takes an optional DEM height function. Test:
`test_road_junction_height.cpp` (T5, T6; sabotages: plain-distance weights, plain average, a 2 cm lifted profile, a constant raise).
No served bytes change.

Slice 6: `road_completion.h` / `road_completion.cpp`: the profile completion tier of design 6.6, a port of racing_game's
`complete_road_profiles` and its 20 m retry, opt-in (`RoadCompletionOptions::enabled`, default off; `build_road_geom_tile` takes the
options and an optional `RoadCompletionStats*`; the deriver passes neither, so no served bytes change). Steps: (1) retry of strict
declines "bounded solver" / "endpoint curvature" at 20 m / 16384 iterations (`Retry`), (2) bridge span re-fit between abutment
planes at max(12 %, class) grade and min(250, class) radius with the secant-grade fallback (`BridgeCompleted`), (3) re-fit of
still-declined ground stretches with 40 m DEM samples, end grades clamped to +-10 %, 32 m anchor ramps, max(12 %, class) /
min(500, class) (`GroundCompleted`). `RoadGeomTile::origins` (one `RoadProfileOrigin{provenance, strict_decline_code}` per entry,
empty without completion) is validated (non-Strict needs a profile and a decline code, the vertical bounds are relaxed per
provenance) and `encode_road_geom` refuses a non-Strict origin (encoding 7 carries it, slice 8). `road_bridge_anchor(graph, dem,
zone, node)` is a pure function of the node (decision D25: a bounded 60 m walk away from the first incident bridge edge, the DEM
plane at the 32 m support carried back), so the result of an entry depends only on its strict outcome, the context graph within
60 m of its end nodes and the DEM within 63 m, never on other entries or the tile. D26: strict ground profiles are not refitted at
bridge nodes. Tests `test_road_completion.cpp` (T18, T6 completion variant, order independence, two-tile and fixture tile
invariance with negative controls) against the frozen game code in `tests/support/road_completion_reference.h` (libm allowed there;
do not "fix" it). `tests/support/road_fixture.h` `geometry_in_tile_with(world, key, GeomOptions)` takes the DEM, completion options
and stats. Fixture A / B with the flat DEM: ground 12 / 5 and bridge 4 / 2 of the strict declines completed (bridge 3 / 1 stay declined, no support); see D25-D26.

Slice 7: `road_furniture.h` / `road_furniture.cpp`: junction furniture of design section 7 as library functions (no stage, no served bytes,
no change to `RoadGeomTile`; encoding 7 stores it in slice 8). `build_road_furniture(tile, context builder, junctions, rules, stats)` returns
one `RoadFurniture` per source node and item (crossing, stop line, give-way line, signal post), placed or declined
(`FurnitureDecline`: StretchDeclined, Tunnel, JunctionNode, DirectionUnresolved, NoLanes, Ambiguous, JunctionDeclined), sorted by
(junction_index, entry_index, kind, station, node_id, travel, t_from, flags). Ownership (D27): entry furniture belongs to one entry chosen
from the context graph (interior node: its entry; stretch end of degree 1 or 2: the stretch starting at the node, else the one ending;
degree >= 3: smallest (way, ref, at_end)) and is emitted by every tile holding it, identical everywhere; furniture derived from a junction
node only where its `RoadJunction` is passed in (owner tile). Lines span exactly `driving_lanes_at` of the controlled direction (one record
per contiguous lane group), signal posts stand at the travel-right edge + `signal_lateral_offset_mm` (a left post only on a one-way with
>= 3 lanes, flag bit 4), `signal_post` derives the facing (opposite to travel), `furniture_quad` the world quad of a crossing or line (a
crossing at a way join extends back over it, flag spans-join). Flag bit 3 (inside a junction) is reserved and never set (D28). Also
`road_node_station` (public wrapper of road_features.cpp's `station_of`). Test: `test_road_furniture.cpp` (T9-T11 plus fixture tile
invariance with a negative control; sabotages lanes -> `carriageway_edges`, no crossing rule, post offset 0, tile graph as context).
Fixture A / B: crossing nodes 123 / 52, placed 122 / 50, declined 1 StretchDeclined / 2 JunctionNode; stop lines 61 / 73, give-way 1 / 1,
signal posts 61 / 71.

Slice 8: `roads.geom` encoding 7 (`kRoadGeomEncoding` 7, `LayerEncoding::RoadGeomJunctions`; 6 is still written when
`RoadGeomTile::encoding` is 6, 6 and 5 decode). The P2 records moved to `include/g2m/layer/road_geom_records.h` and
`RoadGeomTile` carries `junctions`, `furniture` and `origins` (always one per entry). Section 5 junctions (only those whose
owner node `nodes.front()` lies in the tile by `road_tile_contains`, half-open floor on integer mm), section 6 furniture
(entry items in every tile holding the entry, junction items once with their junction), section 7 provenance; byte layout
in `docs/formats/road_geom.md`. `build_road_geom_tile` builds junctions with the world context, furniture and completion;
the deriver passes `served_road_completion_options()`. Validation of design 9.2 in `validate_road_geom_tile` (indices,
sort orders, ring simplicity and nesting within 2 mm, trims, flags, furniture lane spans, reserved bit 3 rejected, D33).
D34: offset rings that do not nest or exceed 1024 vertices take the all-layer hull, else Degenerate. Config text gains
`enc=7 junction=1 furniture=1 completion=1 completion-retry=20/16384 completion-abutment=32/60 completion-ground=40/32
strict-anchor=1 ring-nesting=2mm bridge-far-end=1 ground-ease=8/0.05` (`road_geom_config_text()`). Tests: `test_road_geom_enc7.cpp` (T8: one rejection per
rule with its own malformed body, round trip, truncation; T7: ownership and cross-tile identity with the tile-graph
negative control), `test_road_geom_xs.cpp` (encoding 6/7, pinned config text), `tests/support/road_junction_scene.h`
(standalone scene also used by the fuzz seeds), fuzz seeds for encodings 6 and 7. Goldens: `g3xs` re-recorded (served
tiles now carry completed entries), new `g3j` (`golden_g3j.cpp`, `testdata/golden/g3j_golden.txt`, Windows = Linux).
Roadgeom test body SHA for encoding 7 `a453aed5...a618` (encoding 6 `17d9b7c7...` still reproduced).
D35: `IDeriver::discovery_stages()` (default 1, max 4; the server re-calls `additional_inputs_for` with every input resolved
so far until a stage adds nothing); roads.geom uses 2 - stage 2 adds every bridge incident to a node of the tile's ways
(`road_bridge_dependencies`: DEM along it + 65 m, graph tiles of its nodes), closing the D32 far-end read; `derive()`
rejects a stage-1-only input set. Test helper `g2m_test::road_geom_inputs(world, key, bridge_far_end)`.

Slice 9: `g2m.terrain.carved` v1, L0-L6, body encoding 1 (`docs/formats/terrain_carved.md`). `include/g2m/layer/terrain_carve.h`
/ `src/layer/terrain_carve.cpp`: `CarveTerrain` (the 3 x 3 elev.base window, on-demand integer smoothing [138,621,1024,621,138]
strength 35), `CarveModel` (cores, D40 band-rectangle/ring blends, deck clamp; buckets of `kCarveBucketM` 16 m), `CarveOptions`
(sabotage switches `blend_from_raw`, `first_come_ownership`, `deck_clamp`, `candidate_order_seed`), `carve_terrain_tile`,
`carve_road_geom_keys` (roads.geom L2 tiles within `kCarveRoadRadiusM` 192 m, half-open), `TerrainCarvedDeriver` (L1-L6 =
`downsample_2x2`), `terrain_carved_config_text()`. Opt-in: `BuiltinTerrainDerivers::ordered(..., include_carved)` (implies both
road layers), `builtin_local_release_params(..., include_carved)`, `g2m_tiler bake --layers g2m.terrain.carved`; release ids
without it unchanged. Decisions D36-D41 in `docs/street_fidelity/p2_junction_design.md`. Tests `tests/layer/test_terrain_carve.cpp`
(T12-T15, T17, T19, smoothing parity, server + 4-worker bake), fixture `tests/support/carve_fixture.h` (`CarveFixture`,
`fixture_world`, `carve_tilted_dem`); golden `g3d` (`tests/golden/golden_g3d.cpp`, `testdata/golden/g3d_golden.txt`, 4 tiles,
1 vs 4 workers checked, Windows = Linux).

Terrain migration phase 4 (design D47, 2026-10-15): `roads.geom` encoding 8 (`kRoadGeomEncoding` 8,
`LayerEncoding::RoadGeomTunnels`; the byte layout of 7, which still decodes and is written on request without tunnel floors).
`complete_road_profiles` step 2a builds tunnel floors (`ProfileProvenance::TunnelFloor` = 4; the game's DEM-floor step,
options `tunnel_floors`, `tunnel_spacing_m` 5, `tunnel_max_grade` 0.30, `tunnel_min_radius_m` 20,
`tunnel_chord_min_length_m`, struct default 0 = never; served value from de.json since D52, see below); tunnel profiles are never carved, never paint grip and stay `Tunnel`
approaches. `bridge_anchor_reuse`: the end plane of an accepted incident bridge stretch (recomputed tile-invariantly, one
level) wins over the base anchor; a ground stretch at a bridging node with no accepted bridge ends on the DEM plane, and
falls back to the base anchors when those planes leave no cubic (`ground_base_fallback`). `Walk::anchor` tries every incident
bridge edge. Config text `enc=8 ... tunnel-floor=5/0.29999999999999999/20/chord:0 bridge-anchor-reuse=1` (release id bump);
terrain.carved names `roads.geom enc=8`. Tests `[D47]` (test_road_completion.cpp, against the game's live completion in
`tests/support/road_completion_reference.h` `complete_road_profiles_live`; roadcmp on both fixtures 0 game-only entries).

Terrain migration phase 5 (design D48, 2026-10-15): the completion's abutment walk is served at 200 m
(`kRoadCompletionMaxAbutmentPathM` 201.5); `road_bridge_dependencies(..., walk_margin_mm = kRoadBridgeWalkMarginMm)` (205 m)
adds, in the second discovery stage, the elev.base and roads.graph tiles of a 205 m square around every node of every bridge
incident to the tile's nodes (0 = the pre-D48 set). Config text `completion-abutment=32/200 ... bridge-walk-margin=205`
(release id bump #2). Tests `[D48]` (fence, 60/200/500 m walks, fixture cost) in test_road_completion.cpp.

Terrain migration phase 6 (served carve readiness, designs D49-D51, 2026-10-15): G-1 / D50: `IDeriver::rejected_input_is_absent(ref)`,
`InputTile::rejected`, `rejected_input_marker(ref)` (server.cpp): a roads.geom input of terrain.carved / terrain.class that fails to
resolve counts as absent (no roads) and its marker goes into the output header inputs; errors are not cached. Slice 10 / D49:
`g2m.terrain.class` v1 L0-L6, body encoding 7 (`include/g2m/layer/terrain_class.h`, `src/layer/terrain_class.cpp`,
`docs/formats/terrain_class.md`): 257 x 257 nodes at integer metres, planes land_class / surface / role (`TerrainRole`) / rank; L0
from the carve ownership (`CarveModel::classify`) over ESA land cover, L1-L6 rank-max 3 x 3 over a 4 x 4 window of L-1 tiles.
Opt-in (`include_classes` in `builtin_local_release_params` / `BuiltinTerrainDerivers::ordered`, bake `--layers g2m.terrain.class`,
implies carved + roads); release ids without it unchanged. D51 / G-2/G-3/G-5: `FetchResult::classes`, `TransportHeightTileFetch(...,
layer, class_layer)`, `LoaderConfig::require_classes`, `ResidentHeightSet` entries carry the class tile (4-arg install/find),
`include/g2m/phys/fill_physics_classes.h` (cell centre -> class node, full `SurfaceIdTable` resolve), bridge
`ClassSurfaceConfig` ctor of `G2mTerrainSource` (Snapshot prefetch compares the class tile identity). G-6:
`include/g2m/layer/road_underpass.h` `road_underpass_census` (dip = chord over +-60 m minus profile at grade-separated crossings),
`g2m_road_probe --underpasses`. Tests: `tests/layer/test_terrain_class.cpp` ([terrain_class]), G-1 case in test_terrain_carve.cpp,
`tests/layer/test_road_underpass.cpp` ([underpass]), `tests/phys/test_class_mode.cpp` ([classes]),
`bridges/physics_sim/tests/test_carved_prefetch.cpp` ([classes], bridge leg); golden `g3c` (`tests/golden/golden_g3c.cpp`,
`testdata/golden/g3c_golden.txt`). Open: the home-region G-6 run.

Slice 11 render class window (design D62, 2026-10-16): `include/g2m/world/chunk_terrain_classes.h` /
`src/world/chunk_terrain_classes.cpp` build a chunk's `ClassWindow` from the `g2m.terrain.class` tile of its level (one tile per
chunk: (L, floor(cx/4), floor(cy/4)); the vertex at (64 cx + i) 2^L + 0.5 m takes node 64 cx + i, its south-west node; road
nodes with rank below the `RenderRoadLod` level's min_rank, or at a level without a policy, become Unknown). `ViewSource::
terrain_classes` (precedence over `class_lookup`), `TerrainService::terrain_class` / `peek_terrain_class` (job kind 10). Library
only: no layer, config text, release id or golden change. Tests `tests/world/test_chunk_terrain_classes.cpp`
(`[terrain_class_window]`) and the served case in `test_terrain_service.cpp`.

Junction gradient (design D42, after slice 9): the junction surface blends v_i = b_i + phi_i (p_i - b_i) + psi_i (z_i - p_i)
instead of z_i (b base, p paved band only, z full approach surface; `junction_approach_heights` returns all three), with
phi over l_i = clamp(2 relief_i / slope, min, max) and psi over the verge fade (<= min, so z_J stays convex). Parameters:
`JunctionFadeParams`, the `junction_fade` block of `carve_default.json` (`slope_per_mille` 10, `min_mm` 3000, `max_mm` 15000,
`verge_mm` 1500; `CarveParams::junction_fade_params()`), `make_junction_surface(tile, junction, rules, params, fade)`;
`CarveOptions::junction_relief_fade` = false is the D22-D24 sabotage. Config text gains `junction=D42`. Test
`tests/layer/test_junction_gradient.cpp` (`[layer][carve][junction]`): on the road bed and paved rings of every junction touching
the busy tiles, flat and tilted DEM, gradient <= max(B, 2 F_paved) + 0.02 at >= 2 m from every trim and
<= max(B, own, 2 F_all) + 0.02 nearer (B steepest approach grade + crossfall, F forced gradient between trim points); the
sabotage case expects >= 10 flat junctions over; `[.][diag]` cases print per-junction tables, distance bins and a probe.

Kerb returns (design D44): `fillet_corner` in `src/layer/road_junction.cpp` rounds every stub-stub block corner of the
junction rings with a circular arc (rational quadratic Bezier), radius per highway class from `de.json`
(`JunctionRules::corner_radius_mm`, `corner_radius_for`, `corner_min_wedge_deg`, `corner_tangent_max_percent`,
`corner_oneway_pair_*`), concentric over the three layers; trims extend past the tangent points. A fillet that breaks
simplicity, 2 mm nesting or the vertex budget is dropped per corner (full, half radius, then sharp).
`JunctionBuildOptions::corner_fillets`, stats `fillets`/`filleted_junctions`/`fillet_fallbacks`. Config text
` corner-fillet=1`; de.json SHA `1677d98b...fc27` (`2a5997c5...c4e4` since D52). Tests `tests/layer/test_junction_fillets.cpp` ([fillet]); hidden
`[.][diag][junction_dump]` (`tests/layer/test_junction_dump.cpp`, env G2M_JUNCTION_DUMP) + `tools/junction_view/render.py`
draw rings and approach edges as PNGs.

Width tapers (design D46): `taper_corner` in `src/layer/road_junction.cpp` replaces the cap-corner step where a road
continues at a different width with a straight 1:5 taper (at least 1 m) on every layer; `JunctionBuildOptions::width_tapers`,
config text ` width-taper=5`, test `[taper]` in `tests/layer/test_junction_fillets.cpp`.

Straight-grade tunnels (design D52, owner ruling 2026-10-15): `data/road_rules/de.json` block `tunnel`
(`straight_grade_min_length_m` 200, `TunnelRules`, `RoadRules::tunnel`; de.json SHA `2a5997c5...c4e4`);
`served_road_completion_options()` sets `tunnel_chord_min_length_m` from it, so a tunnel stretch of 200 m or more takes one
constant grade between its portal samples (DEM floor below, or when steeper than 30 %). Config text `tunnel-floor=.../chord:200`
(release id bump; no golden moved, the fixture tunnels are all under 60 m; phase-3a mesh pins of `structure_scene` regenerated
with the game's mesh code). Tests `[tunnel]` in test_road_completion.cpp (tile edge invariance + controls, fixture measurement)
and test_road_rules.cpp.

Underpass decks (design D53-D58, owner decision 2026-10-15, racing_game PR #5): `data/road_rules/de.json` block `underpass`
(`UnderpassRules`: enabled, min_depth 1 m, window 50 m, sag radius 300 m, max span 80 m, abutment 2 m, depth tol 0.1 m,
lower margin 1 m, end clearance 1 m; de.json SHA `2fe0826f...dfa0`). `include/g2m/layer/road_underpass.h`
`RoadUnderpassFinder` (spans per ground stretch: crossing of a lower-layer/tunnel way without a shared node, rims from the
upper hull of the window minus a 300 m sag, deck = lower footprint + depression + abutments clipped to the rims; mask =
rim chord over the span, applied via `RoadProfileLimits::underpass` in strict/Retry and GroundCompleted) and
`road_crossing_dependencies` (D57: roads.graph tiles of every tile-way segment; separate crossings graph,
`RoadUnderpassOptions{enabled, crossings, stats}` of `build_road_geom_tile`, library default off). roads.geom encoding 9
(`kRoadGeomEncoding`, 8 = `kRoadGeomEncodingTunnels` still written when asked, no decks): section 8 `RoadGeomTile::underpasses`
(`RoadUnderpass{entry_index, start_m, end_m, depth_mm, flags, lower_ways}`). Consumers: terrain.carved skips deck spans
on the ground line and adds a bridge line per deck (`CarveOptions::underpass_decks` sabotage), the road envelope marks
deck capsules bridge, `g2m::world::build_road_decks` adds an inferred deck per span (no tunnel roof under it). D59 (owner: steep walls): ground
blends fade out within 1 m past the deck end planes (`Station::deck_lo/deck_hi`, `kUnderpassWallFadeM`, sabotage `CarveOptions::underpass_steep_walls`), so the cutting
keeps its own walls under the deck. D60: parameters kept as served (owner 2026-10-15). D61 (owner's tag answers): parking-structure storeys
(`service=parking_aisle`, `level=*`) are neither upper nor lower roads (`RoadUnderpassStats::in_structure`); a span over tunnel lower ways only is
`covered` (`RoadUnderpassSpanFit::covered`: masked, no deck). D63 (A 66 way boundary): `RoadUnderpassFinder::join(node)` - at a degree-two node of two ground ways, a covered span
reaching the node (window centred on it, both ways' polylines) gives both stretches one rim-chord end anchor (strict and completion, before the grade checks) and
its mask (`RoadUnderpassStats::joined/join_spans`); the finder reads the stretch context; `road_crossing_dependencies` adds the 61 m squares
(`kRoadUnderpassJoinMarginMm`) around every tile way's end nodes. Config
texts: roads.geom `enc=9 ... underpass=1 crossing-context=1 underpass-tags=D61 underpass-join=D63/61000`, carved/class `enc=9 ... underpass-deck=1 underpass-walls=steep/1`. Tests
`tests/layer/test_road_underpass.cpp` (`[underpass][deck]`: bounds + ground outside, non-deck controls, near end,
tile invariance + negative control, encoding 9 round trip/validation, carve no-slot + sabotage, `[walls]` steep walls + sabotage, `[d63]` way-boundary support + controls + invariance, `[d61]` tag rules on the scene and on the owner's five fixture crossings with tag-stripped controls; hidden `[.][diag][tags]` OSM tags at those crossings, hidden `[.][diag][d61]` decks/covered spans of both fixtures on the golden DEM, `[.][diag]` WGS84 crossing list and `[.][diag]` example-4
CSV), `tests/world/test_road_meshes.cpp` (D56 deck; D61: a tunnel lower road gets no served deck), scene `tests/support/underpass_scene.h`.

Owner report 2026-10-19 (two Frankfurt-west sites, decisions D64/D65, branch claude/rail-bridge-underpass). D64 (road under an upper bridge): DGM1 is bare earth, a bridge deck is
removed and its gap interpolated, so a ground road under a `bridge=yes` way with a higher `layer` and no shared node sees a hump in the DEM (site 2: 108.2 -> 114.4 -> 108.2 m over
39 m); the strict profile followed it and the junction planes at the way's end nodes took the hump's wall, so the way was declined (no ribbon, no carve: "the tunnel ends in a wall").
`RoadUnderpassFinder::overpass_spans` finds the bridges over the stretch (`crossings_of(..., over=true)`), fits the NEGATED window with `fit_crossing` (a hump is a cutting of the
negated DEM), and returns `covered` spans (mask only, no deck); `overpass_end_plane` turns a span reaching a stretch end into a `RoadJunctionPlane` of the rim chord; used by
`build_road_profile`'s end anchor and the completion tier's `plane_at` (step 3, the one that matters for the real spot). `UnderpassRules::overpass` (true; not in de.json) is the tests'
switch. D65 (short tunnel under a bank, site 1: a 46 m `tunnel=yes` road under a railway embankment, the DEM floor climbed the ridge at 30 %): completion step 2a takes the portal
chord for a tunnel stretch under `tunnel_chord_min_length_m` when it is at most `tunnel.cut_max_length_m` (80 m) long and the DEM rises `cut_hump_min_mm`..`cut_hump_max_mm` (1..8 m)
over the chord (`RoadCompletionStats::tunnel_cut`); `terrain_carve.cpp` `is_tunnel_cut` carves exactly those floors (length bound and floor straight to 1 cm) as an open cut, every
other tunnel stays uncarved (`CarveOptions::tunnel_cut` sabotage); a cut candidate that a ground road crosses in plan (A66 carriageways over a service tunnel; "lowest core wins" would slot the road above) stays uncarved too (`crossed_by_ground_road`, `CarveOptions::tunnel_cut_crossing_check` sabotage). Provenance stays `TunnelFloor`, so encoding 9 is unchanged. Config texts: roads.geom `... chord:200 tunnel-cut=80/1/8 ...
underpass-join=D63/61000 overpass-mask=D64`, carved `... underpass-walls=steep/1 tunnel-cut=D65/80 carve=...` (release ids change; `/80+hump` since 7a-2). Rail ways are NOT in roads.graph: the railway
over the road is not modelled in geo2map at all (the racing_game side must source it). Tests `tests/layer/test_road_overpass.cpp` (`[overpass][d64]`, `[d65]`: synthetic scenes, controls,
recorded sabotages `UnderpassRules.overpass = false`, `tunnel_cut_max_length_m = 0`, `CarveOptions.tunnel_cut = false`), pins `test_road_geom_xs.cpp`, `test_terrain_carve.cpp`,
`test_road_rules.cpp` (de.json hash is of the LF bytes), goldens g3c/g3d (one hoechst_a66 tile). Tools `g2m_osm_near`, `g2m_spot_dump` (docs/pipeline/README.md 7.4).

## Portal continuity 7a-1 (2026-10-20, branch claude/portal-continuity, unmerged until phase 7)

docs/structure_classification.md section 5. A tunnel floor's portal anchor is no longer `dem.sample` at the portal node (often a bank
face) but the adjacent ground road's level: `road_completion.cpp` `portal_plane(node)` (memoised, a pure function of the node) takes
(1) the end height and grade of an accepted ground profile ending there (strict, else the 20 m retry; `underpass = nullptr`), else
(2) the approach level of the joining ground way (`portal_approach.{h,cpp}`: median of the DEM 2-8 m out, detrended to the node, grade
clamped to 0.10), else (3) for tunnel/tunnel nodes the junction plane. Step 2a ramps the DEM samples to the anchor height over
`ground_ramp_m` and holds the joining road's grade; a chord tunnel (>= 200 m) stays on its chord within `kPortalGradeSlack` 0.015 with
end ramps of at most a quarter of its length; a D65 cut is exactly the chord (the carve's 1 cm straightness test), so a cut's grade can
miss the ground road's by more than 0.02. Fallback chain: anchored -> floor between the two anchors -> straight chord
(`portal_tunnel_straight`) -> DEM (`portal_tunnel_unfit`). Step 3's `plane_at` gains the tunnel portal plane after D64: strict, then the
D43 ease with the chord mismatch waived (`ground_tunnel_chord`), then the DEM planes (`ground_tunnel_unfit`); `ground_tunnel_anchored`
counts the successes. Tile invariance: `road_portal_dependencies` (`road_dem_dependencies.h`, stage 2 with the bridge dependencies) adds
the DEM within 8.5 m of every portal node and along 9 m of each joining ground way. Rail: `rail_geom_build.cpp` `rail_node_anchor` gives
chain ends and ground-piece ends that touch a structure the same approach level; `rail_dem_dependencies(..., portal_anchor)` adds the
approach segments. Option `RoadCompletionOptions::tunnel_portal_anchor` (struct default false, true in `served_road_completion_options()`;
`g2m_test::served_without_portal_anchor()` for the previous behaviour); config token ` portal-anchor=7a1` in roads.geom and rail.geom
(only when on; release ids change only through it, de.json and data ids untouched). New fixture `testdata/fixtures/osm/unterliederbach_tunnel.osm.pbf`
(way 4812754, tile utm32n/2/456/5420; synthetic bank DEM in the test). Tests `tests/layer/test_portal_continuity.cpp` (`[roads][structure][portal]`:
fixtures flat/tilted, 4812754 NW mouth, carved axis, rail; recorded sabotages A portal anchor back to the node's DEM sample, B no refit of
the declined stretch to the tunnel plane); goldens g3c/g3d (hoechst_a66) re-recorded; `test_terrain_carve.cpp` T12 and `test_junction_gradient.cpp`
D42 each allow the one hoechst_a66 1820/21684 tilted case that appears because a formerly declined stretch now completes. `g2m_spot_dump --portals`
is the census. Reach rule (review of ffd575d): `portal_shift(node)` in `road_completion.cpp` - where a tunnel stretch's two raw portal heights differ by more than
`kPortalReach` 0.9 x `tunnel_max_grade` x its reference length, each end moves toward the other by half the excess (pure per node, largest shift
wins at a node on several stretches; an end nothing joins is FREE: step 2a moves its floor end to the nearest height within kPortalReach of the anchored end - stat portal_free_end - and nothing shifts or refits; the first attempt shifted the joined node instead and pushed five short ground stubs back into a decline); `portal_plane` returns the moved plane, which anchors the floor in step 2a and which step 3 refits the joining
ground stretches to (declined ones, and retry-accepted ones - `refit`, stat `ground_portal_refit`, provenance becomes GroundCompleted and keeps its
strict decline code). A strict-accepted ground profile is never re-fitted (format: non-strict provenance needs a strict decline code); a ground
stretch that cannot reach the moved plane is refitted to the UNMOVED raw plane instead (stat `ground_portal_unreachable`; the floor stays on the moved plane, so a step remains and is reported - both show as census steps) ("unreachable" lines of `g2m_spot_dump --portals`, which also
lists the decline text per declined portal node; test `unreachable_dz`). Stat `portal_clamped` counts moved nodes; the DEM floor `portal_tunnel_unfit`
stays as a counted last resort. Test `test_road_completion.cpp` "portal reach (7a-1)" (5 m stub between 3.6 m of terrain rise: both mouths step 0,
floor rise 1.348 of 1.5 m); a second part covers the dead-end stub. Recorded sabotages C (`kPortalReach` 1e9 = no clamp) and D (an unjoined far end not moved: the stub keeps its DEM height) fail it.Known pre-existing failure on the base: `test_road_rules.cpp` "The tunnel block" (its mutation string predates D65's three extra keys).

## Design floor and component solve 7a-2, first half (2026-10-20, branch claude/design-floor, stacked on claude/portal-continuity)

docs/structure_classification.md sections 3, 4.5, 9. Replaces the D47/D52/D65 tunnel floors in `road_completion.cpp` step 2a by one
floor per connected tunnel COMPONENT. Library: `include/g2m/layer/structure_floor.h`, `src/layer/structure_floor.cpp`
(`network_design_floor(FloorNetwork, NetworkFloorParams) -> NetworkFloor`, pure, no libm): per 1 m station the upper bound is
u = min(chord, ground - needed) (ground = minimum DEM at the centre and at +-width/2, `structure_clearance_m` + `structure_roof_m`
= needed; the ground term is skipped within needed/G of a portal), the floor is the greatest G-Lipschitz profile below u raised to the
portal cones (anchors from 7a-1 stay exact, G = `structure_max_grade`), the network chord is the minimum over adjacent portal pairs.
`structure_classify` (census rule) now lives in `src/layer/structure_classify.*` and is linked by `tools/structure_census`.
`road_completion.cpp`: `component_for(stretch)` joins tunnel stretches at shared nodes by BFS in canonical (way id, start ref) order;
a portal is a node with a non-tunnel edge or a single tunnel end (anchor = `portal_raw`, else a free anchor at the DEM sample), interior
nodes carry no anchor; `FloorComponent` is memoised per component (a function of the component, never of the tile). Each entry is fitted
by `fit_vertical_profile` on the floor samples with fixed end heights; end grade = ground road at a portal, mean of the two floor slopes
at a degree-2 interior junction (C1), own-side slope otherwise; a straight floor is fitted as the exact chord. Where the anchors are not
reachable under G the 7a-1 reach rule stays the handler with `tunnel_max_grade` replaced by G (`pairs_beyond_grade`,
`pairs_shifted`, `pairs_still_beyond_grade`, `anchor_misses`; stats `floor_*`, `FloorComponentReport`). A strict-accepted ground end beside
a shifted portal stays stepped (provenance validator) and is counted (`ground_portal_unreachable`). Dependencies:
`road_component_dependencies` (`road_dem_dependencies.cpp`, one round) and `component_dependencies` in `derive_road_geom.cpp` iterate as
the context grows (graph tiles of the component, DEM along every floor station) inside the discovery stages (`discovery_stages` 24 for
roads.geom, `kMaxDiscoveryStages` 32); `complete=false` until the needed graph tiles are present; derive requires the recomputed set to
equal `request.inputs`. Options: `RoadCompletionOptions::tunnel_design_floor` (struct default false = D47/D52/D65 path kept, true in
`served_road_completion_options()`); config token ` tunnel-design-floor=7a2/<clearance>/<roof>/<max_grade>/reach:0.9 component-walk=1`
(only when on). Release ids: roads.geom changes through this token and through the de.json `structure` block sha (batched bump #3); rail.geom
only through the de.json sha (rail floor at 4 % and served runs are the second half). de.json `structure` block: see docs/structure_classification.md
sections 3.2 and 3.3 (`eec3ec54...b813` is the pinned sha). Carving of design floors, served runs (roads.geom enc 11, rail.geom enc 2) and the Above-input
problem are NOT in this half. Tests: `tests/unit/test_structure_floor.cpp` (`[structure][floor]`, 18 cases, sabotages S5-S7),
`tests/layer/test_road_completion.cpp` Y component (shared-node equality, recorded sabotage "no component walk": 14 failures), four-tile invariance,
golden pin 0x3028837f4a976ec2; `test_portal_continuity` prints "[portal 7a-2 ...]" (components, unsolved, pairs beyond grade, anchor moves, fit deviation).
D65 under the design floor (7a-2, measured on the home store): the design floor of a short tunnel stretch is its chord by construction, and the carve recognises
an open cut by a floor straight within 1 cm and at most `tunnel.cut_max_length_m` long, so left alone 1329 stretches were carved as cuts (7a-1: 378). The completion
therefore takes the exact chord only for D65's hump rule (ground over the chord rising by `cut_hump_min/max_mm`, on the per-station ground the floor was solved against;
`FloorComponent::ground`) and gives every other short stretch the portal grade itself (no `kPortalGradeSlack`), and `terrain_carve.cpp` `tunnel_cut_hump_ok`
(`CarveOptions::tunnel_cut_hump_check`, tolerance `kTunnelCutHumpToleranceM` 0.25, cell DEM at 1 m stations; config token `+hump` in the terrain.carved text) refuses a
straight floor without the hump. Census now: 186 carved as cuts, 54 left uncarved because a ground road crosses. `tunnel_entry_is_carved(..., const CutGroundAt*)` and
`g2m_spot_dump --census --csv` (`carved=`, `carved_without_hump_check=`) measure it. Phase 7a-3 replaces this test. Test `test_road_overpass.cpp` "tunnel cut (7a-2)" (sabotage: hump check off trenches a 15 m hill).
Fixture helper `tests/support/road_fixture.h` `road_geom_inputs(..., components=true)` runs the component walk. Goldens g3xs/g3j/g3d and the road mesh pins
(`tests/world/road_mesh_pins.inc`, re-recorded from `g2m::world` with the hidden `[.][regen]` test) moved with the tunnel floors. Cost:
`g2m_structure_census --component-cost` (City-Tunnel: 37 chains, 19474 stations, 16 portals, 4 passes, 7.38 ms; all 1210 components 15.5 ms).

## Railways R1/R2 (2026-10-20, decision G2M-026)

`g2m.rail.geom` v1, an OPT-IN level-2 baked layer (body encoding `LayerEncoding::RailGeom` = 1 in its own id space):
every OSM railway way as pieces (cut at the first vertex at or past `piece.max_length_m` 512 m, owned by the tile that
holds the first vertex, the roads.geom junction rule) with class (rail, light_rail, subway; tram class id 3 reserved but
disabled in the rules), condition (in service / construction), flags (bridge, tunnel, embankment, cutting, declined,
electrified), layer, tracks, service, usage, gauge and a z profile in 1/256 m taken from the line itself: bridge and
tunnel pieces are linear between the two ground anchors of the chain of connected same-structure ways (budget 3000 m,
16 ways), so a deck never follows the DEM into a cutting; ground pieces are resampled at 20 m, triangular-smoothed
(40 m), grade limited (40 permille rail, 60 light_rail/subway) with both ends pinned to the raw DEM. A missing DEM tile
declines only the pieces that need it (flag 16, all z = 0). Does NOT touch roads.geom encoding 9, so carved/class tiles
and every existing release id are unchanged; only a release that opts in (`--layers g2m.rail.geom`, `LayerOptIn::rail`,
`include_rail` as the trailing `builtin_release` parameter) gets a new id. Files: `include/g2m/layer/rail_rules.h` /
`src/layer/rail_rules.cpp` (strict loader of `data/rail_rules/de.json`, embedded, its SHA-256 in the config hash),
`include/g2m/layer/rail_geom_tile.h` (public API), `src/layer/rail_geom_tile.cpp` (codec, validator, descriptor, config
text), `rail_geom_build.cpp` (attributes, piece plan, chain, profile, DEM dependencies), `derive_rail_geom.cpp`
(`RailGeomDeriver`: 3x3 `src.osm` window, stage-2 DEM tiles incl. the chain anchors). `TerrainService`: typed ticket
(job kind 11), `peek_rail_geom`, `ProductContext::rail_geom`. Tool `g2m_rail_census` (`tools/rail_census/main.cpp`,
read-only census of ways, bridges, crossings with roads, baked tiles). Tests `tests/layer/test_rail_geom.cpp`
(`[rail]`, every property with a recorded sabotage), `tests/world/test_terrain_service.cpp` (rail case), fuzz target
`railGeom`, golden `g3rail` (`tests/golden/golden_g3rail.cpp`, `testdata/golden/g3rail_golden.txt`, Windows = Linux).
Docs `docs/formats/rail_geom.md`, `docs/pipeline/rail.md` (decisions, home-region census). Not in v1: embankment/
cutting geometry, carving under a rail bridge, level crossing records, rail.graph, catenary, trains, tram.

## G4 routing, slice 1 (2026-10-14)

Design, owner questions and the slice plan: `docs/routing_design.md`. `g2m_route` (`include/g2m/route/`, `src/route/`)
builds an in-memory routable car graph from a complete `RoadGraph` (`merge_road_graphs` of roads.graph tiles, or
`build_road_graph` of OSM) plus `extract_turn_restrictions` of canonical OSM (src.osm keeps restriction relations;
roads.graph does not carry them yet, planned as encoding 6 in slice 3). Edge-based A* with via-node restrictions,
oneway, access and integer ms costs; via-way rules are counted, not applied. Tests `tests/route/`
(`g2m_route_tests "[route]"`; timing: `g2m_route_tests "[bench]" -s`). Plain A* misses 10 ms at 30 km on the
90k-node stand-in grid (about 30 ms); ALT landmarks are slice 2.
Slice 2 (2026-10-14): ALT landmarks (`RouteLandmarks`, 16 by default; 30 km on the stand-in grid: 1.2 ms mean,
1,695 settled vs 78,464 plain), heading-aware snapping and point-to-point routes, region graph assembly from
roads.graph tiles (`region_graph.h`). Defaults settled: tracks routable at 15 km/h, time restrictions ignored.
Slice 4 (2026-10-14): guidance manoeuvre list (`route_guidance.h`, `g2m_route_tests "[guidance]"`); lane hints (`Maneuver::lanes`, `LaneHint`, `parse_turn_lanes`; from the approach way's `turn:lanes`[`:forward/backward`], empty without a usable tag; `"[lanes]"`). Slice 5
(2026-10-14): client map tiles at three LODs from level-2 roads.graph tiles (`route_map.h`, `"[map]"`; fixture
tiles <= 22 KiB, < 5 ms). Owner rules from PR #2: turn costs final at the defaults; every map and render-road LOD
parameter is a runtime setting the game passes in, with per-LOD/per-level cache keys (docs/routing_design.md 6.1,
tests `"[lod_settings]"`). Slice 3 (2026-10-16): `roads.graph` encoding 6 carries the turn restrictions a tile holds (`include/g2m/layer/road_restriction.h`, `RoadGraphTile::restrictions`, deriver extracts them from the 9 src.osm inputs; roads.geom still embeds encoding 5); `RoutingGraph` applies via-way rules with ghost search states (`state_count()`, `arcs()` = real arcs), `assemble_region` returns graph + merged restrictions. Release ids of roads.graph and roads.geom changed (terrain unchanged); tests `test_via_way.cpp`, `test_route_restrictions.cpp`, `tests/route/via_reference.h`. Next: real home-store runs.
