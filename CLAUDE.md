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

C++20, namespace `g2m`, MIT. g2m never includes or links anything from
physics_sim.

## Layout

```
geo2map_engine/
  CMakeLists.txt, CMakePresets.json   top-level build (presets below)
  CLAUDE.md, README.md, PLAN.md, LICENSE, THIRD_PARTY_NOTICES.md
  cmake/deps_core.cmake               FetchContent for zstd / SQLite / nlohmann_json (G1a; pins in its header table)
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
                                      end caps via clamped projection)
    release.h                         ReleaseInputs -> compute_release_id, Coverage, ReleaseManifest + to_json/from_json,
                                      to_hex / content_hash_from_hex
    pack.h                            PackDirectory/PackEntry/PackTileState, pack_of/pack_tile, encode/decode_pack
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
  docs/formats/                       normative byte formats: README (conventions), tile_container, src_elev,
                                      src_landcover, src_osm, pack, release_manifest, protocol, cache_db
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
    region_resolve.h                  resolve_region(spec, pbf_path): RegionSpec -> ResolvedRegion via ring assembly
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
                                       correction" addendum)
    osm.h                             PBF -> g2m.src.osm (I4): two-stage extract (own sorted-SoA node-location index,
                                       D13) + per-tile clip (OsmClipIndex), OsmExtractReport (nodes/ways/ways_incomplete)
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
                                      (every leg); fuzz_<name> libFuzzer executables + g2m_fuzz_seeds (G2M_BUILD_FUZZERS)
    unit/                             g2m_unit_tests: test_det_math, test_tm_reference, test_geo_frames,
                                      test_tile_raster_hash, test_decode_zip, test_decode_tiff, test_decode_osm,
                                      test_decode_real_data (gated on env var G2M_DATA_DIR, SKIP when unset)
    golden/                           golden.h/.cpp + golden_g1.cpp + golden_g1c.cpp (g2m_golden, milestone G1c I6),
                                      dump_main.cpp (g2m_golden_dump [g0|g1|g1c]), test_golden.cpp (g2m_golden_tests)
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
- `apps/g2m_server`, `apps/g2m_cli` (`apps/g2m_tiler` is now built - G1c);
- `adapters/`;
- `tests/{determinism,protocol,bench}`;
- the targets `g2m_c`, `g2m_http` and `g2m_godot` (`bridges/physics_sim` /
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
    `G2M_LAYERS_SOURCES` + `src/server/protocol.cpp` + `fuzz_run.cpp`,
    `-fsanitize=fuzzer-no-link`), `fuzz_{container,elev,landcover,osm,pack,manifest,path}`
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

## Presets

- Every preset sets `CMAKE_C_COMPILER` too (zstd and SQLite are C).
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
IDeriver::additional_inputs_for provides one pure discovery stage for distant
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
