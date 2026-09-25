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
