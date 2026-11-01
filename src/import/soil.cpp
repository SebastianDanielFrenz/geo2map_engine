// src/import/soil.cpp - see g2m/import/soil.h. Same shape as landcover.cpp: validate every file up front, then one pure
// per-tile function whose results are written in ascending tile order (ordered_for), so the output does not depend on --jobs.
#include "g2m/import/soil.h"

#include "g2m/core/tile_key.h"
#include "g2m/decode/byte_source.h"
#include "g2m/decode/tiff/tiff_reader.h"
#include "g2m/import/ordered_parallel.h"
#include "g2m/layer/soil_rules.h"
#include "g2m/layer/src_soil.h"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <memory>
#include <stdexcept>
#include <thread>

namespace g2m::import {
namespace {
using g2m::decode::CrsKind;
using g2m::decode::DecodeError;
using g2m::decode::FileByteSource;
using g2m::decode::tiff::Ifd;
using g2m::decode::tiff::RasterType;
using g2m::decode::tiff::SampleFormat;
using g2m::decode::tiff::TiffReader;

struct DataSet {
    std::filesystem::path dir;
    std::array<std::shared_ptr<TiffReader>, kSoilPlanes> readers;
    std::array<std::int64_t, kSoilPlanes> nodata{}; // INT64_MIN = none declared
    bool is_signed = true;
    std::int64_t width = 0, height = 0;
    std::int64_t c0 = 0;    // lattice column of the west edge
    std::int64_t r_top = 0; // lattice line index of the north edge (file row i is lattice row r_top - 1 - i)
};

std::string file_name(int prop, int depth) {
    return std::string(kSoilPropNames[prop]) + "_" + std::string(kSoilDepthNames[depth]) + "_mean.tif";
}

Result<DataSet> open_set(const std::filesystem::path& dir, std::int32_t n) {
    DataSet set;
    set.dir = dir;
    bool first = true;
    for (int p = 0; p < kSoilProps; ++p) {
        for (int d = 0; d < kSoilDepths; ++d) {
            const std::filesystem::path path = dir / file_name(p, d);
            const std::string where = "soil: " + path.string() + ": ";
            std::shared_ptr<TiffReader> reader;
            try {
                reader = std::make_shared<TiffReader>(std::make_shared<FileByteSource>(path.string()));
            } catch (const DecodeError& e) {
                return make_error(ErrorCode::Io, where + e.what());
            }
            if (reader->ifds().empty()) return make_error(ErrorCode::Unsupported, where + "no IFD");
            const Ifd& ifd = reader->ifds()[0];
            if (ifd.bits_per_sample != 16 || (ifd.sample_format != SampleFormat::SignedInt && ifd.sample_format != SampleFormat::UnsignedInt))
                return make_error(ErrorCode::Unsupported, where + "IFD0 must be int16 or uint16");
            if (ifd.samples_per_pixel != 1) return make_error(ErrorCode::Unsupported, where + "IFD0 must be single-band");
            if (!ifd.geo_transform) return make_error(ErrorCode::Unsupported, where + "missing GeoTIFF georeference");
            if (ifd.raster_type != RasterType::PixelIsArea) return make_error(ErrorCode::Unsupported, where + "GTRasterTypeGeoKey must be PixelIsArea");
            if (ifd.crs.kind != CrsKind::GeographicEpsg || ifd.crs.epsg != 4326)
                return make_error(ErrorCode::Unsupported, where + "CRS must be geographic EPSG:4326");
            const double sx = ifd.geo_transform->pixel_width, sy = ifd.geo_transform->pixel_height;
            const double want = 1.0 / static_cast<double>(n);
            if (!(sx > 0.0) || std::fabs(sx - want) > want * 1e-9 || std::fabs(sy - want) > want * 1e-9)
                return make_error(ErrorCode::Unsupported, where + "pixel size must be exactly 1/" + std::to_string(n) + " degree");
            const double cx = ifd.geo_transform->corner_x * n, cy = ifd.geo_transform->corner_y * n;
            const double rx = std::floor(cx + 0.5), ry = std::floor(cy + 0.5);
            if (std::fabs(cx - rx) > 1e-6 || std::fabs(cy - ry) > 1e-6)
                return make_error(ErrorCode::Unsupported, where + "tiepoint corner is not on the 1/" + std::to_string(n) + " degree lattice");
            std::int64_t nodata = INT64_MIN;
            if (ifd.gdal_nodata) {
                char* end = nullptr;
                const long long v = std::strtoll(ifd.gdal_nodata->c_str(), &end, 10);
                if (end == ifd.gdal_nodata->c_str() || *end != '\0') return make_error(ErrorCode::Unsupported, where + "GDAL_NODATA must be an integer");
                nodata = v;
            }
            const bool is_signed = ifd.sample_format == SampleFormat::SignedInt;
            if (first) {
                set.width = static_cast<std::int64_t>(ifd.width); set.height = static_cast<std::int64_t>(ifd.height);
                set.c0 = static_cast<std::int64_t>(rx); set.r_top = static_cast<std::int64_t>(ry); set.is_signed = is_signed;
                first = false;
            } else if (set.width != static_cast<std::int64_t>(ifd.width) || set.height != static_cast<std::int64_t>(ifd.height) ||
                       set.c0 != static_cast<std::int64_t>(rx) || set.r_top != static_cast<std::int64_t>(ry) || set.is_signed != is_signed) {
                return make_error(ErrorCode::Unsupported, where + "size, corner or sample type differs from the first file of the set");
            }
            set.readers[static_cast<std::size_t>(soil_plane(p, d))] = std::move(reader);
            set.nodata[static_cast<std::size_t>(soil_plane(p, d))] = nodata;
        }
    }
    return set;
}

struct TileOutcome {
    bool skip = false;
    std::uint64_t nodata_cells = 0;
    std::vector<std::uint8_t> encoded_body;
};
} // namespace

Result<SoilImportReport> import_soil(const ResolvedRegion& region, std::vector<std::filesystem::path> data_sets, ITileSink& sink,
                                      std::string_view scope, const SoilImportOptions& options) {
    (void)scope;
    const auto t0 = std::chrono::steady_clock::now();
    SoilImportReport report;
    const std::vector<TileKey> keys = tile_keys(region);
    if (keys.empty()) return report;
    if (data_sets.empty()) return make_error(ErrorCode::InvalidArgument, "soil: no data set given");
    const SoilRules& rules = builtin_soil_rules();
    const std::int32_t n = rules.cells_per_degree;
    std::vector<DataSet> sets;
    for (const auto& dir : data_sets) {
        Result<DataSet> s = open_set(dir, n);
        if (!s) return s.error();
        sets.push_back(std::move(*s));
    }
    unsigned jobs = options.jobs;
    if (jobs == 0) jobs = std::max(1u, std::thread::hardware_concurrency());

    const auto produce = [&](std::size_t idx) -> TileOutcome {
        const TileKey& key = keys[idx];
        const SoilWindow win = soil_source_window(key, n);
        SoilSourceTile tile;
        tile.key = key; tile.scheme = rules.source_scheme; tile.cells_per_degree = n;
        tile.col0 = win.col0; tile.row0 = win.row0; tile.width = win.width; tile.height = win.height;
        const std::size_t cells = static_cast<std::size_t>(win.width) * static_cast<std::size_t>(win.height);
        tile.values.assign(cells * kSoilPlanes, kSoilNoData);
        std::vector<bool> resolved(cells, false);
        for (const DataSet& set : sets) {
            // Lattice cells (col, row) the file covers: col in [c0, c0 + width), row in [r_top - height, r_top).
            const std::int64_t col_lo = std::max(win.col0, set.c0), col_hi = std::min(win.col0 + win.width, set.c0 + set.width);
            const std::int64_t row_lo = std::max(win.row0, set.r_top - set.height), row_hi = std::min(win.row0 + win.height, set.r_top);
            if (col_lo >= col_hi || row_lo >= row_hi) continue;
            const int w = static_cast<int>(col_hi - col_lo), h = static_cast<int>(row_hi - row_lo);
            const std::int64_t file_x = col_lo - set.c0, file_y = set.r_top - row_hi; // north-most row of the overlap in the file
            const auto cell_of = [&](int x, int y) { // window cell of overlap column x and overlap row y (row 0 = north of the overlap)
                const std::int64_t lat_row = row_hi - 1 - y;
                return static_cast<std::size_t>(lat_row - win.row0) * static_cast<std::size_t>(win.width) + static_cast<std::size_t>(col_lo + x - win.col0);
            };
            std::vector<std::size_t> owned; // cells of the overlap no earlier set resolved
            for (int y = 0; y < h; ++y) for (int x = 0; x < w; ++x) if (!resolved[cell_of(x, y)]) owned.push_back(cell_of(x, y));
            if (owned.empty()) continue;
            for (int plane = 0; plane < kSoilPlanes; ++plane) {
                const auto& reader = set.readers[static_cast<std::size_t>(plane)];
                const std::int64_t nodata = set.nodata[static_cast<std::size_t>(plane)];
                std::vector<std::int32_t> window(static_cast<std::size_t>(w) * static_cast<std::size_t>(h));
                try {
                    if (set.is_signed) {
                        const auto raster = reader->decode_window_i16(0, file_x, file_y, w, h);
                        for (int y = 0; y < h; ++y) for (int x = 0; x < w; ++x) window[static_cast<std::size_t>(y) * static_cast<std::size_t>(w) + static_cast<std::size_t>(x)] = raster.at(x, y);
                    } else {
                        const auto raster = reader->decode_window_u16(0, file_x, file_y, w, h);
                        for (int y = 0; y < h; ++y) for (int x = 0; x < w; ++x) window[static_cast<std::size_t>(y) * static_cast<std::size_t>(w) + static_cast<std::size_t>(x)] = raster.at(x, y);
                    }
                } catch (const DecodeError& e) {
                    throw std::runtime_error("soil: " + set.dir.string() + ": " + e.what());
                }
                for (int y = 0; y < h; ++y) {
                    for (int x = 0; x < w; ++x) {
                        const std::size_t cell = cell_of(x, y);
                        if (resolved[cell]) continue;
                        const std::int32_t v = window[static_cast<std::size_t>(y) * static_cast<std::size_t>(w) + static_cast<std::size_t>(x)];
                        const bool nd = v < 0 || (nodata != INT64_MIN && v == nodata) || v > 32767;
                        tile.values[static_cast<std::size_t>(plane) * cells + cell] = nd ? kSoilNoData : static_cast<std::uint16_t>(v);
                    }
                }
            }
            for (std::size_t cell : owned) resolved[cell] = true;
        }
        TileOutcome outcome;
        const std::size_t resolved_count = static_cast<std::size_t>(std::count(resolved.begin(), resolved.end(), true));
        if (resolved_count == 0) { outcome.skip = true; return outcome; }
        outcome.nodata_cells = cells - resolved_count;
        Result<TileBody> body = encode_src_soil(tile);
        if (!body) throw std::runtime_error("soil: encode_src_soil: " + body.error().message);
        Result<std::vector<std::uint8_t>> encoded = encode_body(*body);
        if (!encoded) throw std::runtime_error("soil: encode_body: " + encoded.error().message);
        outcome.encoded_body = std::move(*encoded);
        return outcome;
    };

    const auto consume = [&](std::size_t idx, TileOutcome&& outcome) {
        if (outcome.skip) { ++report.tiles_skipped_nocoverage; return; }
        ++report.tiles_written;
        report.cells_nodata += outcome.nodata_cells;
        TileHeader header;
        header.layer = std::string(kSrcSoilLayer);
        header.layer_version = 1;
        header.key = keys[idx];
        header.datum = VerticalDatum::None;
        header.height_offset = 0;
        const Result<void> put_result = sink.put(header.layer, keys[idx], header, outcome.encoded_body);
        if (!put_result) throw std::runtime_error("soil: sink.put: " + put_result.error().message);
    };

    try {
        ordered_for(keys.size(), static_cast<int>(jobs), produce, consume);
    } catch (const std::exception& e) {
        return make_error(ErrorCode::Io, std::string("soil: ") + e.what());
    }
    report.wall_ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
    return report;
}
} // namespace g2m::import
