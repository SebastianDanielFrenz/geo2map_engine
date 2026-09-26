// src/import/landcover.cpp - see g2m/import/landcover.h.
//
// Per-tile algorithm (PLAN.md G1c I5):
//  1. A level-2 tile's own georef is tile_georef(k, cells_per_side=256,
//     Cell, SouthToNorth) - 4 m cells (WorldCover's own ~9.26 m pixels at
//     this latitude are coarser than our 4 m grid, so several of our
//     samples legitimately read the same source pixel - a deliberate
//     nearest-neighbour resample, not a bug; docs/formats/src_landcover.md).
//  2. For each of the tile's 65536 (r, c) samples, the cell CENTRE's UTM
//     easting/northing (RasterGeoref::sample_easting_m/sample_northing_m -
//     already exactly 1024x + 4c + 2 / 1024y + 4r + 2 for this georef, no
//     separate formula needed) is converted to lon/lat via the existing
//     deterministic g2m::geo::Utm::reverse (D16: no new numerics - the same
//     series already locked cross-platform by the g0 golden's tm.reverse
//     entry).
//  3. lon/lat is turned into a pixel index against one candidate file's own
//     validated (N, C0, C1): col = floor(lon*N) - C0, row = C1 - 1 -
//     floor(lat*N) (D17: one rounding; a sample sitting exactly on a pixel
//     edge resolves to the east/south pixel - an accepted, documented
//     edge rule, not a bug).
//  4. Multi-file search: `tifs` is tried IN ORDER. For the common case (one
//     file fully covers the tile - true for every real region this project
//     imports today, since Hessen sits entirely inside WorldCover tile
//     N48E006), every one of the 65536 indices resolves against tifs[0] and
//     the whole tile is served by exactly one decode_window_u8 call. In
//     general: for each file in turn, every sample NOT YET resolved by an
//     earlier file gets its pixel index recomputed against THIS file's own
//     (N, C0, C1); samples that land inside [0, width) x [0, height) are
//     resolved from one bounding-box decode_window_u8 call over just those
//     samples; anything still unresolved after every file has had a turn is
//     class 0 (the scheme's own "no data" sentinel - the same value a
//     sample the source itself reports 0 for would also get, so this
//     import path never has to distinguish "no coverage" from "coverage
//     says 0" downstream).
//  5. A tile where EVERY sample is 0 is skipped, not written (matches
//     DEM's tiles_skipped_* convention of never writing all-background
//     tiles); otherwise the raster is encoded via encode_src_landcover and
//     written as g2m.src.landcover, datum VerticalDatum::None (land cover
//     carries no elevation datum).
//
// Determinism (PLAN.md's own "output independent of --jobs"): the sequence
// of sink.put() calls is a pure function of (region, the validated file
// list's own content) - ordered_for delivers per-tile results to `consume`
// in ascending tile-index order regardless of scheduling, and every
// per-tile computation here touches no shared mutable state.
//
// Geodetic note (PLAN.md 4.1, docs/formats/regions.md, src_landcover.md):
// ESA WorldCover's coordinates are WGS84; this project's own UTM math uses
// GRS80/ETRS89 (locked by the g0 golden). The two differ by up to ~0.5 m at
// Hessen's latitude (frame realisation drift since 1989, not an ellipsoid
// shape difference - GRS80 and WGS84 are equal to within a fraction of a
// millimetre in flattening). At WorldCover's own ~10 m native resolution
// this is sub-pixel and is deliberately ignored, exactly as PLAN.md 4.1
// specifies - no reprojection/datum-shift step exists in this importer.
#include "g2m/import/landcover.h"

#include "g2m/core/geo/utm.h"
#include "g2m/core/tile_key.h"
#include "g2m/decode/byte_source.h"
#include "g2m/decode/tiff/tiff_reader.h"
#include "g2m/import/ordered_parallel.h"
#include "g2m/layer/src_landcover.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>
#include <stdexcept>
#include <thread>
#include <vector>

namespace g2m::import {
namespace {

using g2m::decode::DecodeError;
using g2m::decode::FileByteSource;
using g2m::decode::CrsKind;
using g2m::decode::tiff::Ifd;
using g2m::decode::tiff::RasterType;
using g2m::decode::tiff::SampleFormat;
using g2m::decode::tiff::TiffReader;

constexpr int kCellsPerSide = 256; // WorldCover import tiles at 1024 m / 256 = 4 m cells

// A validated WorldCover-shaped input file: its IFD-0 pixel geometry
// (N, C0, C1, width, height) plus the reader itself (kept alive - decode
// calls happen later, per tile).
struct FileInfo {
    std::filesystem::path path;
    std::shared_ptr<TiffReader> reader;
    std::int64_t width = 0;
    std::int64_t height = 0;
    std::int64_t pixels_per_degree = 0; // N
    std::int64_t c0 = 0;                // lon0 * N (tiepoint corner, west edge)
    std::int64_t c1 = 0;                // lat0 * N (tiepoint corner, north edge)
};

Result<FileInfo> open_and_validate(const std::filesystem::path& path) {
    const std::string path_str = path.string();
    std::shared_ptr<TiffReader> reader;
    try {
        auto source = std::make_shared<FileByteSource>(path_str);
        reader = std::make_shared<TiffReader>(source);
    } catch (const DecodeError& e) {
        return make_error(ErrorCode::Io, "landcover: " + path_str + ": " + e.what());
    }
    if (reader->ifds().empty()) {
        return make_error(ErrorCode::Unsupported, "landcover: " + path_str + ": no IFD");
    }
    const Ifd& ifd = reader->ifds()[0];
    if (ifd.sample_format != SampleFormat::UnsignedInt || ifd.bits_per_sample != 8) {
        return make_error(ErrorCode::Unsupported,
                           "landcover: " + path_str + ": IFD0 must be uint8 (got sample_format=" +
                               std::to_string(static_cast<int>(ifd.sample_format)) +
                               " bits_per_sample=" + std::to_string(ifd.bits_per_sample) + ")");
    }
    if (ifd.samples_per_pixel != 1) {
        return make_error(ErrorCode::Unsupported, "landcover: " + path_str + ": IFD0 must be single-band");
    }
    if (!ifd.geo_transform) {
        return make_error(ErrorCode::Unsupported, "landcover: " + path_str + ": missing GeoTIFF georeference");
    }
    if (ifd.raster_type != RasterType::PixelIsArea) {
        return make_error(ErrorCode::Unsupported, "landcover: " + path_str + ": GTRasterTypeGeoKey must be PixelIsArea");
    }
    if (ifd.crs.kind != CrsKind::GeographicEpsg || ifd.crs.epsg != 4326) {
        return make_error(ErrorCode::Unsupported,
                           "landcover: " + path_str + ": CRS must be geographic EPSG:4326 (got kind=" +
                               std::to_string(static_cast<int>(ifd.crs.kind)) +
                               " epsg=" + std::to_string(ifd.crs.epsg) + ")");
    }
    const double sx = ifd.geo_transform->pixel_width;
    const double sy = ifd.geo_transform->pixel_height;
    if (!(sx > 0.0) || sx != sy) {
        return make_error(ErrorCode::Unsupported,
                           "landcover: " + path_str + ": ModelPixelScale must be equal and positive on both axes");
    }
    const auto n = static_cast<std::int64_t>(std::floor(1.0 / sx + 0.5));
    if (n <= 0 || 1.0 / static_cast<double>(n) != sx) {
        return make_error(ErrorCode::Unsupported,
                           "landcover: " + path_str + ": ModelPixelScale is not an exact 1/N degree for integer N");
    }
    const double c0d = ifd.geo_transform->corner_x * static_cast<double>(n);
    const double c1d = ifd.geo_transform->corner_y * static_cast<double>(n);
    if (std::floor(c0d) != c0d || std::floor(c1d) != c1d) {
        return make_error(ErrorCode::Unsupported,
                           "landcover: " + path_str + ": tiepoint corner is not an integer multiple of 1/N degree");
    }
    if (ifd.gdal_nodata && *ifd.gdal_nodata != "0") {
        return make_error(ErrorCode::Unsupported,
                           "landcover: " + path_str + ": GDAL_NODATA must be \"0\" (got \"" + *ifd.gdal_nodata + "\")");
    }

    FileInfo info;
    info.path = path;
    info.reader = std::move(reader);
    info.width = static_cast<std::int64_t>(ifd.width);
    info.height = static_cast<std::int64_t>(ifd.height);
    info.pixels_per_degree = n;
    info.c0 = static_cast<std::int64_t>(c0d);
    info.c1 = static_cast<std::int64_t>(c1d);
    return info;
}

struct TileOutcome {
    bool skip_allzero = false;
    std::vector<std::uint8_t> encoded_body; // set only when written
};

} // namespace

Result<LandcoverImportReport> import_landcover(const ResolvedRegion& region, std::vector<std::filesystem::path> tifs,
                                                ITileSink& sink, std::string_view scope,
                                                const LandcoverImportOptions& options) {
    (void)scope; // informational only - see landcover.h's own doc comment
    const auto t0 = std::chrono::steady_clock::now();

    LandcoverImportReport report;
    const std::vector<TileKey> keys = tile_keys(region);
    if (keys.empty()) return report;

    std::vector<FileInfo> files;
    files.reserve(tifs.size());
    for (const auto& path : tifs) {
        Result<FileInfo> info = open_and_validate(path);
        if (!info) return info.error();
        files.push_back(std::move(*info));
    }

    unsigned jobs = options.jobs;
    if (jobs == 0) jobs = std::max(1u, std::thread::hardware_concurrency());

    const geo::Utm utm;

    const auto produce = [&](std::size_t idx) -> TileOutcome {
        const TileKey& key = keys[idx];
        const std::optional<RasterGeoref> dest_opt =
            tile_georef(key, kCellsPerSide, SampleRegistration::Cell, RowOrder::SouthToNorth);
        if (!dest_opt) {
            throw std::runtime_error("landcover: tile_georef failed for a resolved region's own tile key");
        }
        const RasterGeoref& dest_georef = *dest_opt;
        const int width = dest_georef.width;
        const int height = dest_georef.height;
        const std::size_t total = static_cast<std::size_t>(width) * static_cast<std::size_t>(height);

        ClassRaster classes(width, height, /*fill=*/std::uint8_t{0});
        std::vector<bool> resolved(total, false);
        std::size_t remaining = total;

        struct Hit {
            int r, c;
            std::int64_t col, row;
        };
        std::vector<Hit> hits;

        for (const FileInfo& file : files) {
            if (remaining == 0) break;
            hits.clear();
            hits.reserve(remaining);
            for (int r = 0; r < height; ++r) {
                const double northing_m = dest_georef.sample_northing_m(r);
                for (int c = 0; c < width; ++c) {
                    const std::size_t index = static_cast<std::size_t>(r) * static_cast<std::size_t>(width) + c;
                    if (resolved[index]) continue;
                    const double easting_m = dest_georef.sample_easting_m(c);
                    const geo::TmReverse rev = utm.reverse(dest_georef.zone, easting_m, northing_m);
                    const std::int64_t col = static_cast<std::int64_t>(
                                                  std::floor(rev.lon * static_cast<double>(file.pixels_per_degree))) -
                                              file.c0;
                    const std::int64_t row = file.c1 - 1 -
                                              static_cast<std::int64_t>(
                                                  std::floor(rev.lat * static_cast<double>(file.pixels_per_degree)));
                    if (col < 0 || col >= file.width || row < 0 || row >= file.height) continue;
                    hits.push_back(Hit{r, c, col, row});
                }
            }
            if (hits.empty()) continue;

            std::int64_t min_col = hits.front().col, max_col = hits.front().col;
            std::int64_t min_row = hits.front().row, max_row = hits.front().row;
            for (const Hit& h : hits) {
                min_col = std::min(min_col, h.col);
                max_col = std::max(max_col, h.col);
                min_row = std::min(min_row, h.row);
                max_row = std::max(max_row, h.row);
            }
            const int win_w = static_cast<int>(max_col - min_col + 1);
            const int win_h = static_cast<int>(max_row - min_row + 1);

            Raster<std::uint8_t> window;
            try {
                window = file.reader->decode_window_u8(0, min_col, min_row, win_w, win_h);
            } catch (const DecodeError& e) {
                throw std::runtime_error("landcover: " + file.path.string() + ": " + e.what());
            }
            for (const Hit& h : hits) {
                classes.at(h.c, h.r) = window.at(static_cast<int>(h.col - min_col), static_cast<int>(h.row - min_row));
                resolved[static_cast<std::size_t>(h.r) * static_cast<std::size_t>(width) + h.c] = true;
            }
            remaining -= hits.size();
        }

        TileOutcome outcome;
        const bool all_zero = std::all_of(classes.data().begin(), classes.data().end(),
                                           [](std::uint8_t v) { return v == 0; });
        if (all_zero) {
            outcome.skip_allzero = true;
            return outcome;
        }

        LandcoverTile tile;
        tile.georef = dest_georef;
        tile.classes = std::move(classes);
        tile.class_scheme = "esa_worldcover_v200";
        tile.nodata_class = 0;
        Result<TileBody> body = encode_src_landcover(tile);
        if (!body) throw std::runtime_error("landcover: encode_src_landcover: " + body.error().message);
        Result<std::vector<std::uint8_t>> encoded = encode_body(*body);
        if (!encoded) throw std::runtime_error("landcover: encode_body: " + encoded.error().message);
        outcome.encoded_body = std::move(*encoded);
        return outcome;
    };

    const auto consume = [&](std::size_t idx, TileOutcome&& outcome) {
        if (outcome.skip_allzero) {
            ++report.tiles_skipped_allzero;
            return;
        }
        ++report.tiles_written;

        TileHeader header;
        header.layer = std::string(kSrcLandcoverLayer);
        header.layer_version = 1;
        header.key = keys[idx];
        header.datum = VerticalDatum::None;
        header.height_offset = 0;
        const Result<void> put_result = sink.put(header.layer, keys[idx], header, outcome.encoded_body);
        if (!put_result) throw std::runtime_error("landcover: sink.put: " + put_result.error().message);
    };

    try {
        ordered_for(keys.size(), static_cast<int>(jobs), produce, consume);
    } catch (const std::exception& e) {
        return make_error(ErrorCode::Io, std::string("landcover: ") + e.what());
    }

    const auto t1 = std::chrono::steady_clock::now();
    report.wall_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
    return report;
}

} // namespace g2m::import
