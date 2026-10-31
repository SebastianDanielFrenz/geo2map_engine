// g2m/import/soil.h - soil source rasters -> g2m.src.soil tile import (docs/pipeline/soil.md).
//
// Input: one or more DATA SETS, each a directory holding the 42 GeoTIFFs `<prop>_<depth>_mean.tif` (prop = sand, silt,
// clay, soc, phh2o, cfvo, bdod; depth = 0-5cm, 5-15cm, 15-30cm, 30-60cm, 60-100cm, 100-200cm - the SoilGrids 2.0 names;
// tools/soil_fetch/fetch_soilgrids.py writes such a directory). Every file of a set must be a single-band 16-bit
// (int16 or uint16) GeoTIFF in geographic EPSG:4326, PixelIsArea, square pixels of exactly 1/N degree with N =
// rules.cells_per_degree (480), its corner on the 1/N lattice, and all 42 files must share size and corner. Anything
// else fails the whole call (ErrorCode::Unsupported) before any tile is processed.
//
// Per level-2 tile: the lattice window soil_source_window(key, N) is filled from the first set (in the given order) that
// covers each cell; a cell no set covers, or where the file's GDAL_NODATA value (or any negative value) is read,
// becomes the no-data marker 65535. A tile whose window no set covers at all is not written. The values are copied
// as they are: nothing is resampled and no datum shift is applied (the lattice IS the source grid).
#pragma once

#include "g2m/core/result.h"
#include "g2m/import/sink.h"
#include "g2m/layer/region.h"

#include <cstdint>
#include <filesystem>
#include <string_view>
#include <vector>

namespace g2m::import {

struct SoilImportOptions {
    unsigned jobs = 0; // 0 = hardware concurrency (ordered_for's contract)
};

struct SoilImportReport {
    std::uint64_t tiles_written = 0;
    std::uint64_t tiles_skipped_nocoverage = 0; // no data set covers any cell of the tile's window
    std::uint64_t cells_nodata = 0;             // window cells inside a written tile that no set covers (65535)
    double wall_ms = 0.0;
};

Result<SoilImportReport> import_soil(const ResolvedRegion& region, std::vector<std::filesystem::path> data_sets, ITileSink& sink,
                                      std::string_view scope, const SoilImportOptions& options = {});

} // namespace g2m::import
