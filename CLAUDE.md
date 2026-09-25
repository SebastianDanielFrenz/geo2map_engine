# geo2map_engine - architecture map

This file only maps what exists and how to build, test and run it.
- Rationale and design decisions live in the knowledge vault
  (`S:\claude_knowledge_base`), not here.
- The design and phase plan is `PLAN.md` (plus racing_game `PLAN.md` section 12
  for the milestones).
- Milestone state: **G0 done**. G0 covers the library skeleton, deterministic
  math, geodesy, tile keys, rasters, hashes and golden infrastructure.
  **G1b done** (branch `g1b-decoders`, unmerged as of this entry): pure,
  bounds-checked decoders for the source formats (ZIP, TIFF/GeoTIFF/BigTIFF/
  COG, Hessen DGM1, OSM PBF) - see `g2m_decode` below.

C++20, namespace `g2m`, MIT. g2m never includes or links anything from
physics_sim.

## Layout

```
geo2map_engine/
  CMakeLists.txt, CMakePresets.json   top-level build (presets below)
  CLAUDE.md, README.md, PLAN.md, LICENSE, THIRD_PARTY_NOTICES.md
  include/g2m/core/                   public API of g2m_core (no third-party types)
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
  src/core/                           implementation (mirrors include/); det_math.cpp forwards g2m::det -> g2m::musl
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
    support/                          test-only: headless_env (Windows CRT dialogs off), catch_main, splitmix64.h
    unit/                             g2m_unit_tests: test_det_math, test_tm_reference, test_geo_frames,
                                      test_tile_raster_hash, test_decode_zip, test_decode_tiff, test_decode_osm,
                                      test_decode_real_data (gated on env var G2M_DATA_DIR, SKIP when unset)
    golden/                           golden.h/.cpp (g2m_golden), dump_main.cpp (g2m_golden_dump),
                                      test_golden.cpp (g2m_golden_tests)
    fuzz/                             libFuzzer targets for g2m_decode (fuzz_zip, fuzz_tiff, fuzz_pbf); built only
                                      under clang (not clang-cl), gated by G2M_BUILD_FUZZERS (OFF by default)
  testdata/
    geodesy/                          GeographicLib reference values (written by tools/refgen; see below)
    golden/g0_golden.txt              committed G0 golden hashes
  cmake/deps_decoders.cmake           FetchContent for g2m_decode's deps (libdeflate, zlib-ng, protozero, libosmium)
  tools/
    ci.ps1                            CI (below)
    setup_dev_env.ps1                 verify (-CheckOnly) or install the toolchain
    refgen/                           g2m_refgen: GeographicLib reference-value generator (G2M_BUILD_REFGEN=ON only)
  docs/                               empty until later milestones (PLAN.md 2.2)
```

Not built yet. These are placeholders only (PLAN.md 2.2/2.3; a comment at the
end of `CMakeLists.txt`):
- the other core headers `polyline.h`, `layer/`, `server/`, `client/`,
  `graph/`, `mesh/`;
- `include/g2m_c/`;
- `apps/` (`g2m_server`, `g2m_tiler`, `g2m_cli`);
- `bridges/physics_sim/`;
- `adapters/`;
- `tests/{determinism,protocol,fuzz,bench}`;
- the targets `g2m_layers`, `g2m_server`, `g2m_client`, `g2m_c`, `g2m_http`,
  `g2m_ps_bridge` and `g2m_godot`.

## Targets

- `g2m_core` (STATIC): `src/core/**` plus the musl objects.
  - Public include dir: `include/`. `third_party/musl_math` is PRIVATE.
  - No third-party dependency.
- `g2m_musl_math` (OBJECT): `third_party/musl_math/*.cpp`, used only through
  `$<TARGET_OBJECTS:...>` in `g2m_core`.
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
  - `g2m_unit_tests`, `g2m_golden_tests` (Catch2; tags become CTest labels
    through `ADD_TAGS_AS_LABELS`);
  - `g2m_golden` (STATIC) and `g2m_golden_dump` (exe).
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
- Configure fails on purpose if an optimised build type has no `-O`/`/O` flag
  (a stale cache). Fix it with `cmake --preset <name> --fresh`.

## Dependencies (FetchContent, pinned)

| Dependency | Pin | Use |
|---|---|---|
| Catch2 | `v3.16.0` | tests only; `/EHsc` on its targets under MSVC, `-mllvm -asan-stack=0` under clang-cl ASan |
| GeographicLib | `v2.7` | `tools/refgen` only; never linked into g2m_core or a test |
| libdeflate | `v1.26` | `g2m_decode`: DEFLATE + CRC-32 for ZIP and TIFF/COG |
| zlib-ng | `2.3.3` | `g2m_decode`: zlib-API DEFLATE for libosmium's PBF blob decompression only |
| protozero | `v1.8.2` | `g2m_decode`: header-only protobuf decode, under libosmium |
| libosmium | `v2.23.1` | `g2m_decode`: OSM PBF parsing (`src/decode/osm/`) |

## Presets

- Windows (Ninja, from a VS 2022 Build Tools environment):
  - `debug`, `release`, `relwithdebinfo`, `asan` (clang-cl);
  - `msvc-debug`, `msvc-release` (cl);
  - build dirs are `out/build/<preset>`.
- WSL (clang + lld):
  - `linux-release`, `linux-asan-ubsan`, `linux-tsan`;
  - build dirs are `~/build/geo2map_engine/<preset>`, never under `/mnt/s`;
  - `linux-tsan` symbolizes with `/usr/bin/addr2line`.

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

Golden hashes are in `testdata/golden/g0_golden.txt`, one line per entry:
`name count fnv64`, where `all` covers every entry. Regenerate them only
after an intended output change: run `g2m_golden_dump <file>`, then re-add
the header comment.

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
  `g2m_golden_dump` into `out/ci_golden/<leg>.txt`.
- **Checks:**
  - libm grep over `include/` and `src/`, with a pattern self-test;
  - golden compare: every leg's dump must equal `g0_golden.txt`.
- **Output:** logs go to `out/ci_logs/`, and the summary prints PASS/FAIL per
  leg and stage. The exit code is non-zero on any failure.

## Dev environment

```powershell
tools\setup_dev_env.ps1 -CheckOnly   # verify only, installs nothing
tools\setup_dev_env.ps1              # install what is missing (winget / apt)
```
