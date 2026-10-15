# Third-party notices

## musl libm (partial port) - linked into `g2m_core`

`third_party/musl_math/` is a C++ port of double-precision functions from
**musl libc's math library**, exposed as `g2m::det` (`include/g2m/core/det_math.h`).
It exists so that derivation code never calls the platform C library's
transcendentals, which round differently on Windows (ucrtbase) and Linux
(glibc) (PLAN.md 2.4).

- Upstream: <https://git.musl-libc.org/cgit/musl/>
- Version: **musl 1.2.5**, tag `v1.2.5`, commit
  `0784374d561435f7c787a555aeab8ede699ed298`.
- Files ported (musl paths, under `src/math/`):
  - `__rem_pio2.c`, `__rem_pio2_large.c`, `__sin.c`, `__cos.c`, `__tan.c`;
  - `sin.c`, `cos.c`, `tan.c`, `sincos.c`;
  - `asin.c`, `acos.c`, `atan.c`, `atan2.c`;
  - `exp.c`, `exp_data.c`, `expm1.c`;
  - `log.c`, `log_data.c`, `log1p.c`;
  - `pow.c`, `pow_data.c`;
  - `sinh.c`, `cosh.c`, `tanh.c`, `asinh.c`, `atanh.c`, `__expo2.c`;
  - `hypot.c`;
  - the bit-level helper macros from `src/internal/libm.h`.
- Only the `double` code paths are ported. Branches that differ on this
  project's targets are left out, and each ported file says which it left out:
  - the `__FP_FAST_FMA` branches (FMA contraction is off project-wide);
  - `FLT_EVAL_METHOD != 0` (x86-64 SSE2 never evaluates in extended precision).
- Every ported file keeps its upstream copyright header: Sun Microsystems /
  SunSoft / SunPro (fdlibm origin, via FreeBSD) or Arm Limited.
- Licence: MIT, text below. The Sun notices ("Permission to use, copy, modify,
  and distribute this software is freely granted, provided that this notice
  is preserved") and the Arm notices (SPDX: MIT) in the individual files are
  part of the same permissive licensing (see musl's `COPYRIGHT`).

### musl licence text (musl `COPYRIGHT`, at the commit above)

```
musl as a whole is licensed under the following standard MIT license:

----------------------------------------------------------------------
Copyright (C) 2005-2020 Rich Felker, et al.

Permission is hereby granted, free of charge, to any person obtaining
a copy of this software and associated documentation files (the
"Software"), to deal in the Software without restriction, including
without limitation the rights to use, copy, modify, merge, publish,
distribute, sublicense, and/or sell copies of the Software, and to
permit persons to whom the Software is furnished to do so, subject to
the following conditions:

The above copyright notice and this permission notice shall be
included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY
CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
----------------------------------------------------------------------
```

## GeographicLib - reference values only, never linked

Transverse Mercator in `src/core/geo/tm.cpp` implements the Krüger series as
published by C. F. F. Karney, "Transverse Mercator with an accuracy of a few
nanometers", J. Geodesy 85(8), 475-485 (2011). Its structure follows
GeographicLib's `TransverseMercator.cpp` (v2.7, MIT, Copyright (c) 2008-2025
Charles Karney), including the series-coefficient polynomials
(`alpcoeff`/`betcoeff`/`b1coeff` at order 6).

GeographicLib itself is fetched by CMake `FetchContent` (tag `v2.7`) only
when `G2M_BUILD_REFGEN=ON`, for `tools/refgen`, which writes the reference
values in `testdata/geodesy/`. It is never linked into `g2m_core` or any
test, and no GeographicLib source is vendored in this repository.

```
The MIT License (MIT).

Copyright (c) 2008-2025, Charles Karney

Permission is hereby granted, free of charge, to any person
obtaining a copy of this software and associated documentation
files (the "Software"), to deal in the Software without
restriction, including without limitation the rights to use, copy,
modify, merge, publish, distribute, sublicense, and/or sell copies
of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be
included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
NONINFRINGEMENT.  IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
DEALINGS IN THE SOFTWARE.
```

## Catch2 - tests only

Fetched by CMake `FetchContent` (tag `v3.16.0`) for the test executables.
Not vendored, never linked into `g2m_core`. Licence: BSL-1.0, at its upstream
repository.

## zstd - linked into `g2m_layers`

Zstandard compression for tile-container sections
(`docs/formats/tile_container.md`). It is fetched by CMake `FetchContent`
from the release tarball
`https://github.com/facebook/zstd/releases/download/v1.5.7/zstd-1.5.7.tar.gz`
(SHA-256 `eb33e51f49a15e023950cd7825ca74a4a2b43db8354825ac24fc1b7ee09e6fa3`;
tag `v1.5.7`, commit `f8745da6ff1ad1e7bab384bd1f9d742439278e99`). It is
compiled as our own static target `g2m_dep_zstd` (`cmake/deps_core.cmake`).
The version is pinned exactly: `src/layer/zstd_codec.cpp` static-asserts
1.5.7, because another version may compress to different bytes. Not
vendored. Licence: BSD-3-Clause (zstd is dual-licensed BSD/GPLv2; g2m uses
it under BSD):

```
BSD License

For Zstandard software

Copyright (c) Meta Platforms, Inc. and affiliates. All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

 * Redistributions of source code must retain the above copyright notice, this
   list of conditions and the following disclaimer.

 * Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

 * Neither the name Facebook, nor Meta, nor the names of its contributors may
   be used to endorse or promote products derived from this software without
   specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
```

## SQLite - linked into `g2m_server`

The tile stores (`docs/formats/cache_db.md`). The amalgamation
**3.53.4** is fetched by CMake `FetchContent` from
`https://sqlite.org/2026/sqlite-amalgamation-3530400.zip` (SHA3-256
`628a44cfe82c66aed1ccbbe85a562d2e33ebe64b3288981ed76285612227934e`) and
compiled as our own static target `g2m_dep_sqlite3`. Not vendored.
Licence: **public domain**. The SQLite authors disclaim copyright; the
source carries this blessing in place of a licence:

```
The author disclaims copyright to this source code.  In place of
a legal notice, here is a blessing:

   May you do good and not evil.
   May you find forgiveness for yourself and forgive others.
   May you share freely, never taking more than you give.
```

## nlohmann/json - linked into `g2m_layers` and `g2m_server` (header-only)

Release- and server-manifest JSON (`docs/formats/release_manifest.md`,
`protocol.md`). The single-header release **v3.12.0** is fetched by CMake
`FetchContent` from
`https://github.com/nlohmann/json/releases/download/v3.12.0/json.tar.xz`
(SHA-256 `42f6e95cad6ec532fd372391373363b62a14af6d771056dbfc86160e6dfff7aa`),
interface target `g2m_dep_json`, included PRIVATE only (no JSON type
appears in a public header). Not vendored. Licence: MIT:

```
MIT License

Copyright (c) 2013-2025 Niels Lohmann

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
```

## libdeflate - linked into `g2m_decode`

DEFLATE (de)compression and CRC-32, used by `src/decode/zip/` (ZIP stored/
deflate entries) and `src/decode/tiff/` (TIFF/COG Deflate-compressed
strips/tiles).

- Upstream: <https://github.com/ebiggers/libdeflate>
- Version: **v1.26**, fetched via CMake `FetchContent` (`cmake/deps_decoders.cmake`),
  built as a static library only (no shared lib, gzip CLI, or its own tests).
  Not vendored; no source committed to this repository.
- Licence: MIT, at its upstream repository (`COPYING`).

## zlib-ng - linked into `g2m_decode` (transitively, via libosmium)

A zlib-API-compatible (`ZLIB_COMPAT=ON`) DEFLATE implementation, used only to
satisfy libosmium's own `#include <zlib.h>` for PBF blob decompression
(`osmium/io/detail/zlib.hpp`) - g2m's own ZIP/TIFF decompression goes through
libdeflate above, not this.

- Upstream: <https://github.com/zlib-ng/zlib-ng>
- Version: **2.3.3**, fetched via CMake `FetchContent`, built static
  (`BUILD_SHARED_LIBS` forced `OFF` - see `cmake/deps_decoders.cmake`'s own
  comment for why this is load-bearing: left unset, zlib-ng's CMakeLists
  builds a shared DLL by default and every `g2m_decode` test executable fails
  to start at all). Not vendored; no source committed to this repository.
- Licence: zlib, at its upstream repository (`LICENSE.md`).

## protozero - header-only, linked into `g2m_decode` (transitively, via libosmium)

Minimal protobuf decoder libosmium's own PBF reader is built on; not called
directly by any g2m code.

- Upstream: <https://github.com/mapbox/protozero>
- Version: **v1.8.2**, fetched via CMake `FetchContent_Populate` (source only,
  no `add_subdirectory` - its own CMakeLists builds fuzzers/tests this
  project does not need), exposed as an `INTERFACE` target with `SYSTEM`
  include dirs. Not vendored; no source committed to this repository.
- Licence: BSD-2-Clause, at its upstream repository (`LICENSE.md`).

## libosmium - header-only, linked into `g2m_decode`

OSM PBF parsing (`src/decode/osm/`). Only the node/way/relation/tag
PBF-reading path is used (via `osmium/io/detail/pbf_input.hpp`, not
`any_input.hpp`, to avoid a transitive `bzlib.h` dependency this project does
not want); no osmium type appears in any g2m public header - `g2m::decode::
osm::PbfHandler` is g2m's own virtual-dispatch seam.

- Upstream: <https://github.com/osmcode/libosmium>
- Version: **v2.23.1**, fetched via CMake `FetchContent_Populate` (source
  only, no `add_subdirectory` - its own CMakeLists needs Boost/GDAL/
  FindOsmium.cmake for its full test/example suite, none of which g2m_decode
  needs), exposed as an `INTERFACE` target with `SYSTEM` include dirs. Not
  vendored; no source committed to this repository.
- Licence: BSL-1.0, at its upstream repository (`LICENSE`).

## cpp-httplib - header-only, linked into `g2m_http` (G6)

HTTP/1.1 client and server under `HttpTransport` and `HttpServer`
(`src/http/http_transport.cpp`, `src/http/http_server.cpp` are the only
files that include it; no httplib type appears in `include/g2m/http/`).
Built without TLS (no `CPPHTTPLIB_OPENSSL_SUPPORT`) and without zlib/brotli.

- Upstream: <https://github.com/yhirose/cpp-httplib>
- Version: **v0.58.0**, commit `4f3f9ef19be83ae97a5d9a059432dc00e445b7ab`,
  fetched via CMake `FetchContent` (`cmake/deps_http.cmake`, git clone of
  that exact commit; its own CMakeLists is not used), exposed as the
  `INTERFACE` target `g2m_dep_httplib` with a `SYSTEM` include dir, linked
  PRIVATE. Only fetched when `G2M_BUILD_HTTP` is ON (default: only when
  geo2map_engine is the top-level project). Not vendored; no source
  committed to this repository.
- Licence: MIT (upstream `LICENSE`):

```
The MIT License (MIT)

Copyright (c) 2017 yhirose

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
```

## Data

Small derived cuts of third-party geodata are committed under
`testdata/fixtures/` and `testdata/fuzz_seeds/` for tests and fuzzing (see
`testdata/fixtures/README.md` for exact source files, cut commands, sizes
and SHA-256 hashes of every committed file). None of this data is used at
runtime by any shipped `g2m` binary - fixtures and fuzz seeds only.

- **DGM1 (digital terrain model)**: © HVBG - Hessisches Landesamt für
  Bodenmanagement und Geoinformation, licensed
  [Datenlizenz Deutschland – Zero – Version 2.0](https://www.govdata.de/dl-de/zero-2-0)
  (`dl-de/zero-2-0`) - equivalent to a public-domain dedication; no
  attribution is legally required, but source is credited anyway in
  `testdata/fixtures/README.md`. Covers `testdata/fixtures/dgm1/
  PKT_fixture.zip` and the DGM1-shaped synthetic fuzz seed
  `testdata/fuzz_seeds/tiff/dgm1_shape_32x32_lzw_pred2_f32.tif` (values are
  synthetic, but the file's LZW+predictor-2 float32 encoding shape matches
  a real DGM1 tile).
- **OpenStreetMap (via Geofabrik's `hessen-latest.osm.pbf` extract)**:
  © OpenStreetMap contributors, licensed
  [ODbL 1.0](https://opendatacommons.org/licenses/odbl/1-0/). Covers
  `testdata/fixtures/osm/frankfurt_hoechst.osm.pbf`,
  `testdata/fixtures/osm/hoechst_a66.osm.pbf` (SHA-256 `8556244549478372cccdead6d3fd59638b5e313fc9358ccb9a1a4b7ac2947dd8`, 799,511 bytes) and
  `testdata/fuzz_seeds/pbf/tiny_bbox_cut.osm.pbf` (all are small
  bbox/relation-closure cuts of the same source, produced by
  `g2m_fixture_cut cut-pbf`; `testdata/fixtures/osm/unsorted_nodes.osm.pbf`
  is hand-built test data, not derived from this source, and carries no
  OSM copyright). **Produced Work / Derivative Database note**: a bbox+
  relation-closure extract that preserves the original entities'
  attribution-bearing structure (nodes/ways/relations with their original
  ids and tags, not merely rendered/aggregated output) is a Derivative
  Database under ODbL 1.0 §2.3/§4, not a Produced Work under §2.4 - it must
  therefore itself be licensed under ODbL 1.0 (as it is: this notice is the
  required attribution, and the extract is redistributed under the same
  ODbL 1.0 terms as its source, satisfying the share-alike requirement).
- **ESA WorldCover** (`ESA_WorldCover_10m_2021_v200_N48E006_Map.tif`):
  © ESA WorldCover project 2021 / Contains modified Copernicus Sentinel
  data, licensed [CC BY 4.0](https://creativecommons.org/licenses/by/4.0/).
  Covers `testdata/fixtures/worldcover/fixture_N50E008_hoechst.tif`,
  `testdata/fixtures/worldcover/fixture_N50E008_bbox.tif` (cropped windows,
  re-encoded uncompressed - see `testdata/fixtures/README.md`'s
  `cut-worldcover` section for the exact georeferencing/tag handling) and
  `testdata/fuzz_seeds/tiff/worldcover_crop_64x64_u8.tif` (a further 64x64
  crop of the first fixture, re-encoded uncompressed).
