# geo2map_engine - architecture map

This file only maps what exists and how to build, test and run it.
- Rationale and design decisions live in the knowledge vault
  (`S:\claude_knowledge_base`), not here.
- The design and phase plan is `PLAN.md` (plus racing_game `PLAN.md` section 12
  for the milestones).
- Milestone state: **G0 done**. G0 covers the library skeleton, deterministic
  math, geodesy, tile keys, rasters, hashes and golden infrastructure.
  **G1a done**: the layer model, the tile formats, releases, the SQLite tile
  stores, the in-process server and the client cache. **G1b done**: pure,
  bounds-checked decoders for the source formats (ZIP, TIFF/GeoTIFF/BigTIFF/
  COG, Hessen DGM1, OSM PBF) - see `g2m_decode` below. G1c (tiler plus
  import) comes next.

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
    release.h                         ReleaseInputs -> compute_release_id, Coverage, ReleaseManifest + to_json/from_json,
                                      to_hex / content_hash_from_hex
    pack.h                            PackDirectory/PackEntry/PackTileState, pack_of/pack_tile, encode/decode_pack
  include/g2m/server/                 public API of g2m_server (PLAN.md 3.1-3.4)
    tile_store.h                      TileStore (SQLite + blob files; CONCURRENCY CONTRACT in the header), TileStoreConfig,
                                      TileAddress, StoredTile, stats/check_consistency
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
  docs/formats/                       normative byte formats: README (conventions), tile_container, src_elev,
                                      src_landcover, src_osm, pack, release_manifest, protocol, cache_db
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
    golden/                           golden.h/.cpp + golden_g1.cpp (g2m_golden), dump_main.cpp (g2m_golden_dump [g0|g1]),
                                      test_golden.cpp (g2m_golden_tests)
    fuzz/                             libFuzzer targets for g2m_decode (fuzz_zip, fuzz_tiff, fuzz_pbf) over a
                                      fuzz-instrumented copy of G2M_DECODE_SOURCES; same G2M_BUILD_FUZZERS gate and
                                      CTest `fuzz` label as fuzz_formats (WSL clang legs)
  testdata/
    geodesy/                          GeographicLib reference values (written by tools/refgen; see below)
    golden/g0_golden.txt              committed G0 golden hashes
    golden/g1_golden.txt              committed G1 golden hashes (encoded bytes of every G1 format)
  cmake/deps_decoders.cmake           FetchContent for g2m_decode's deps (libdeflate, zlib-ng, protozero, libosmium)
  tools/
    ci.ps1                            CI (below)
    setup_dev_env.ps1                 verify (-CheckOnly) or install the toolchain
    tsan_suppressions.txt             TSan suppressions, loaded only by the linux-tsan test preset (SQLite wal-index header reads)
    refgen/                           g2m_refgen: GeographicLib reference-value generator (G2M_BUILD_REFGEN=ON only)
```

Not built yet. These are placeholders only (PLAN.md 2.2/2.3; a comment at the
end of `CMakeLists.txt`):
- the other core headers `polyline.h`, `graph/`, `mesh/`;
- `include/g2m_c/`;
- `apps/` (`g2m_server`, `g2m_tiler`, `g2m_cli`);
- `bridges/physics_sim/`;
- `adapters/`;
- `tests/{determinism,protocol,bench}`;
- the targets `g2m_c`, `g2m_http`, `g2m_ps_bridge` and `g2m_godot`.

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
  - `g2m_unit_tests`, `g2m_layer_tests`, `g2m_server_tests`,
    `g2m_fuzz_smoke_tests`, `g2m_golden_tests` (Catch2; tags become CTest
    labels through `ADD_TAGS_AS_LABELS`);
  - `g2m_golden` (STATIC, links g2m_layers + g2m_server) and
    `g2m_golden_dump` (exe);
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

## CMake options

- `G2M_BUILD_TESTS` (ON): Catch2 v3.16.0 via FetchContent, plus `tests/`.
- `G2M_BUILD_REFGEN` (OFF): fetches GeographicLib v2.7
  (`GEOGRAPHICLIB_PRECISION=3`, long double) and builds `tools/refgen`.
  - Only on Linux x86-64, because it needs an 80-bit long double.
- `G2M_SANITIZER_BUILD` (OFF): set ON by the `asan`, `linux-asan-ubsan` and
  `linux-tsan` presets.
- `G2M_BUILD_FUZZERS` (OFF): clang only; set ON by the `linux-release` and
  `linux-asan-ubsan` presets.
- Configure fails on purpose if an optimised build type has no `-O`/`/O` flag
  (a stale cache). Fix it with `cmake --preset <name> --fresh`.

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
  after another.
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
