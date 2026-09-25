// g2m/decode/tiff/tiff_reader.h - classic TIFF + BigTIFF + GeoTIFF subset
// reader (PLAN.md 13, milestone G1b), reading through IByteSource so a COG
// window on a remote/large file only pulls the bytes it needs.
//
// Scope: little/big endian, classic and BigTIFF; strips and tiles;
// compression none/LZW/Deflate(8 and 32946)/PackBits; predictor 1/2/3;
// sample formats uint8/int16/uint16/int32/float32; PlanarConfig 1 with
// SamplesPerPixel == 1 (anything else is rejected with a clear error, per
// PLAN.md - this reader is single-band only); the IFD chain (every IFD is
// enumerated, e.g. COG overviews); GeoTIFF ModelPixelScale/ModelTiepoint,
// GeoKeyDirectory (RasterTypeGeoKey, ProjectedCSTypeGeoKey,
// GeographicTypeGeoKey) and the GDAL_NODATA ASCII tag.
#pragma once

#include "g2m/core/raster.h"
#include "g2m/decode/byte_source.h"
#include "g2m/decode/tiff/geotransform.h"
#include "g2m/decode/tiff/tiff_types.h"

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace g2m::decode::tiff {

// One IFD's parsed baseline + GeoTIFF tags. Only what this reader's scope
// needs; an unrecognised/unsupported tag value is simply not exposed here
// (decode_window() is where an unsupported combination throws).
struct Ifd {
    std::uint64_t width = 0;
    std::uint64_t height = 0;
    unsigned bits_per_sample = 0;
    unsigned samples_per_pixel = 1;
    Compression compression = Compression::None;
    SampleFormat sample_format = SampleFormat::UnsignedInt;
    Predictor predictor = Predictor::None;
    PlanarConfig planar_config = PlanarConfig::Chunky;

    // Strip layout (tile_width == 0 selects this).
    std::uint64_t rows_per_strip = 0;
    std::vector<std::uint64_t> strip_offsets;
    std::vector<std::uint64_t> strip_byte_counts;

    // Tile layout (tile_width != 0 selects this; TileLength/TileWidth are
    // required to both be present together per the TIFF6 tiling extension).
    std::uint64_t tile_width = 0;
    std::uint64_t tile_height = 0;
    std::vector<std::uint64_t> tile_offsets;
    std::vector<std::uint64_t> tile_byte_counts;

    bool is_tiled() const { return tile_width != 0; }

    // GeoTIFF (absent when the file carries none of these tags - PLAN.md:
    // "some DGM1 copies lack GeoKeys").
    std::optional<GeoTransform> geo_transform; // from ModelPixelScale + ModelTiepoint
    Crs crs;                                    // CrsKind::Absent if no GeoKeyDirectory tag
    RasterType raster_type = RasterType::Unknown;
    std::optional<std::string> gdal_nodata; // tag 42113, raw ASCII (e.g. "-9999")

    // Number of bytes one decoded sample occupies (bits_per_sample / 8,
    // always byte-aligned for every format this reader supports).
    unsigned bytes_per_sample() const { return bits_per_sample / 8; }
};

class TiffReader {
public:
    explicit TiffReader(std::shared_ptr<IByteSource> source);

    // Every IFD in the file's IFD chain, in file order (index 0 = the first
    // IFD; a COG's reduced-resolution overviews are the later ones).
    const std::vector<Ifd>& ifds() const { return ifds_; }

    // Decodes a window [x, y, x+w) x [y, y+h) of IFD `ifd_index` into a
    // Raster<T>. Throws DecodeError if:
    //  - the window is out of bounds;
    //  - samples_per_pixel != 1 or planar_config != Chunky ("single band
    //    only", PLAN.md);
    //  - the compression/predictor/sample-format/bit-depth combination is
    //    not supported;
    //  - T's size/signedness does not match the IFD's actual
    //    bits_per_sample/sample_format (callers are expected to check
    //    ifd.sample_format/bits_per_sample first - this is a hard
    //    precondition check, not a conversion).
    Raster<float> decode_window_f32(std::size_t ifd_index, std::int64_t x, std::int64_t y, int w, int h) const;
    Raster<std::uint8_t> decode_window_u8(std::size_t ifd_index, std::int64_t x, std::int64_t y, int w, int h) const;
    Raster<std::uint16_t> decode_window_u16(std::size_t ifd_index, std::int64_t x, std::int64_t y, int w,
                                             int h) const;
    Raster<std::int16_t> decode_window_i16(std::size_t ifd_index, std::int64_t x, std::int64_t y, int w,
                                            int h) const;
    Raster<std::int32_t> decode_window_i32(std::size_t ifd_index, std::int64_t x, std::int64_t y, int w,
                                            int h) const;

private:
    std::shared_ptr<IByteSource> source_;
    std::vector<Ifd> ifds_;
    bool big_endian_ = false;
    bool big_tiff_ = false;

    // Raw decode: fills `out` (already sized w*h*bytes_per_sample, row
    // major) with the IFD's native sample bytes for the window, in the
    // reader's own host byte order (little-endian - see byte_source.h's
    // host-endianness assumption), predictor already undone.
    void decode_window_raw(const Ifd& ifd, std::int64_t x, std::int64_t y, int w, int h,
                            std::vector<std::byte>& out) const;
};

} // namespace g2m::decode::tiff
