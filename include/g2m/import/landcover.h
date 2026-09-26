// g2m/import/landcover.h - ESA WorldCover COG -> g2m.src.landcover tile
// import (PLAN.md G1c I5).
//
// Each input TIFF (`tifs`) is expected to be a WorldCover v200-shaped 3x3
// degree COG: single-band uint8 land-cover classes, geographic EPSG:4326,
// GeoTIFF PixelIsArea, exactly 1/N degree square pixels for an integer N
// (N = 12000 for the real product). IFD 0 of every file is validated up
// front (landcover.cpp's validate_ifd0) - any file failing validation fails
// the whole call with ErrorCode::Unsupported before any tile is processed,
// rather than surfacing per-tile.
//
// `tifs` is searched IN THE GIVEN ORDER for each tile's samples (see
// landcover.cpp's own header comment for the per-tile/per-file algorithm) -
// callers with more than one file covering a region (e.g. one spanning a
// 3-degree WorldCover tile boundary) should sort `tifs` so the search order
// is deterministic and meaningful; a single-file caller (this project's
// current scope: all of Hessen fits inside one real WorldCover tile,
// N48E006) does not need to think about the order at all.
#pragma once

#include "g2m/core/result.h"
#include "g2m/import/sink.h"
#include "g2m/layer/region.h"

#include <cstdint>
#include <filesystem>
#include <string>
#include <string_view>
#include <vector>

namespace g2m::import {

struct LandcoverImportOptions {
    // 0 = std::thread::hardware_concurrency() (at least 1) - ordered_for's
    // own `jobs` contract (ordered_parallel.h).
    unsigned jobs = 0;
};

// Everything import_landcover measured about one run. Deterministic in both
// fields (a pure function of region + the resolved file contents) -
// unlike DemImportReport there is no scheduling-dependent counter here.
struct LandcoverImportReport {
    std::uint64_t tiles_written = 0;         // level-2 tiles written (>= 1 non-zero class sample)
    std::uint64_t tiles_skipped_allzero = 0; // every one of the tile's 65536 samples resolved to class 0
    double wall_ms = 0.0;
};

// Imports every level-2 tile in `region.tiles` (region.h's tile_keys()) from
// `tifs` into `sink` as layer g2m.src.landcover, class_scheme
// "esa_worldcover_v200", nodata_class 0, datum VerticalDatum::None. `scope`
// names the destination for the caller's own logging only (mirrors
// import_dem/import_osm's own `scope` parameter). Returns
// ErrorCode::Unsupported if any file in `tifs` fails IFD-0 validation;
// otherwise an error only for a genuine decode/encode/sink failure - a tile
// with no covering file at all is not an error (see tiles_skipped_allzero).
Result<LandcoverImportReport> import_landcover(const ResolvedRegion& region,
                                                std::vector<std::filesystem::path> tifs, ITileSink& sink,
                                                std::string_view scope,
                                                const LandcoverImportOptions& options = {});

} // namespace g2m::import
