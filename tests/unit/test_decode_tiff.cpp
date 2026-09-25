// tests/unit/test_decode_tiff.cpp - TiffReader: synthetic in-test fixtures
// covering compression none/LZW/Deflate/PackBits, predictor 1/2/3, sample
// formats uint8/uint16/int16/int32/float32, GeoTIFF tag parsing, .tfw
// parsing, and the georeference bridge to RasterGeoref.

#include "g2m/core/geo/utm.h"
#include "g2m/decode/byte_source.h"
#include "g2m/decode/tiff/geotransform.h"
#include "g2m/decode/tiff/tfw.h"
#include "g2m/decode/tiff/tiff_reader.h"

#include <libdeflate.h>

#include <catch2/catch_test_macros.hpp>

#include <cstdint>
#include <cstring>
#include <memory>
#include <vector>

using namespace g2m;
using namespace g2m::decode;
using namespace g2m::decode::tiff;

namespace {

// --- little helpers -------------------------------------------------------

std::vector<std::byte> le_u16(std::uint16_t v) {
    std::vector<std::byte> b(2);
    std::memcpy(b.data(), &v, 2);
    return b;
}
std::vector<std::byte> le_u32(std::uint32_t v) {
    std::vector<std::byte> b(4);
    std::memcpy(b.data(), &v, 4);
    return b;
}
std::vector<std::byte> le_f64_array(const std::vector<double>& vals) {
    std::vector<std::byte> b(vals.size() * 8);
    std::memcpy(b.data(), vals.data(), b.size());
    return b;
}
std::vector<std::byte> le_u16_array(const std::vector<std::uint16_t>& vals) {
    std::vector<std::byte> b(vals.size() * 2);
    std::memcpy(b.data(), vals.data(), b.size());
    return b;
}

std::vector<std::byte> deflate_zlib_compress(const std::vector<std::byte>& in) {
    libdeflate_compressor* c = libdeflate_alloc_compressor(6);
    REQUIRE(c != nullptr);
    std::vector<std::byte> out(in.size() + 4096);
    const std::size_t n = libdeflate_zlib_compress(c, in.data(), in.size(), out.data(), out.size());
    libdeflate_free_compressor(c);
    REQUIRE(n != 0);
    out.resize(n);
    return out;
}

// PackBits encoder: emits every byte as its own 1-byte literal run (valid,
// just maximally inefficient - fine for a unit fixture).
std::vector<std::byte> packbits_encode_trivial(const std::vector<std::byte>& in) {
    std::vector<std::byte> out;
    out.reserve(in.size() * 2);
    for (std::byte b : in) {
        out.push_back(std::byte{0}); // literal run of 1 byte (ctrl 0 -> n+1=1)
        out.push_back(b);
    }
    return out;
}

// TIFF-flavoured LZW encoder emitting every symbol as a LITERAL code only
// (never a dictionary match) - still exercises MSB-first bit packing and
// the early-change code-width growth rule exactly the way lzw_decode
// expects (a dictionary entry is pushed for every non-first code
// regardless of whether it was itself a literal or a match), just without
// covering the dictionary-hit/KwKwK decode paths. Those paths are instead
// covered by the real-data DGM1 cross-check (PLAN.md/this milestone's
// final report).
std::vector<std::byte> lzw_encode_literal_only(const std::vector<std::byte>& in) {
    constexpr int kClearCode = 256, kEoiCode = 257, kFirstFreeCode = 258, kMaxCodeBits = 12;
    std::vector<std::byte> out;
    std::size_t bitpos = 0;
    auto emit = [&](std::uint32_t code, int nbits) {
        for (int b = nbits - 1; b >= 0; --b) {
            const bool bit = ((code >> b) & 1u) != 0;
            const std::size_t byte_idx = bitpos >> 3;
            if (byte_idx >= out.size()) out.push_back(std::byte{0});
            if (bit) out[byte_idx] |= static_cast<std::byte>(1u << (7 - (bitpos & 7)));
            ++bitpos;
        }
    };
    int nbits = 9;
    emit(kClearCode, nbits);
    std::size_t table_size = 0;
    bool have_prev = false;
    for (std::byte bb : in) {
        emit(static_cast<std::uint8_t>(bb), nbits);
        if (!have_prev) {
            have_prev = true;
        } else {
            ++table_size;
            if (table_size + kFirstFreeCode + 1 >= (std::uint64_t{1} << nbits) && nbits < kMaxCodeBits) ++nbits;
        }
    }
    emit(kEoiCode, nbits);
    return out;
}

// Encodes predictor 2 (horizontal differencing) over one row, in place - the
// exact inverse of undo_predictor2 (tested indirectly via the round trip).
void apply_predictor2_encode(std::vector<std::byte>& row, int width, unsigned bits_per_sample) {
    switch (bits_per_sample) {
        case 8: {
            auto* p = reinterpret_cast<std::uint8_t*>(row.data());
            for (int i = width - 1; i >= 1; --i) p[i] = static_cast<std::uint8_t>(p[i] - p[i - 1]);
            break;
        }
        case 16: {
            auto* p = reinterpret_cast<std::uint16_t*>(row.data());
            for (int i = width - 1; i >= 1; --i) p[i] = static_cast<std::uint16_t>(p[i] - p[i - 1]);
            break;
        }
        case 32: {
            auto* p = reinterpret_cast<std::uint32_t*>(row.data());
            for (int i = width - 1; i >= 1; --i) p[i] = static_cast<std::uint32_t>(p[i] - p[i - 1]);
            break;
        }
        default: FAIL("unsupported bits_per_sample in test helper");
    }
}

// Encodes predictor 3 (floating point) over one row - the exact inverse of
// undo_predictor3: split into big-endian byte planes, then byte-wise diff.
std::vector<std::byte> apply_predictor3_encode(const std::vector<std::byte>& host_order_row, int width,
                                                unsigned bytes_per_sample) {
    const std::size_t n = static_cast<std::size_t>(width) * bytes_per_sample;
    std::vector<std::uint8_t> big_endian_samples(n);
    for (int c = 0; c < width; ++c) {
        for (unsigned b = 0; b < bytes_per_sample; ++b) {
            // host is little-endian; big-endian byte b (0 = most significant)
            // of sample c comes from host byte (bytes_per_sample-1-b).
            big_endian_samples[static_cast<std::size_t>(c) * bytes_per_sample + b] =
                static_cast<std::uint8_t>(host_order_row[static_cast<std::size_t>(c) * bytes_per_sample +
                                                           (bytes_per_sample - 1 - b)]);
        }
    }
    std::vector<std::uint8_t> planes(n);
    for (int c = 0; c < width; ++c) {
        for (unsigned b = 0; b < bytes_per_sample; ++b) {
            planes[static_cast<std::size_t>(b) * static_cast<std::size_t>(width) + static_cast<std::size_t>(c)] =
                big_endian_samples[static_cast<std::size_t>(c) * bytes_per_sample + b];
        }
    }
    std::vector<std::byte> out(n);
    out[0] = static_cast<std::byte>(planes[0]);
    for (std::size_t i = 1; i < n; ++i) {
        out[i] = static_cast<std::byte>(static_cast<std::uint8_t>(planes[i] - planes[i - 1]));
    }
    return out;
}

// --- classic little-endian TIFF builder ------------------------------------

class TiffBuilder {
public:
    void add_short(std::uint16_t tag, std::uint16_t value) { add_inline(tag, 3, 1, le_u16(value)); }
    void add_long(std::uint16_t tag, std::uint32_t value) { add_inline(tag, 4, 1, le_u32(value)); }
    void add_short_array(std::uint16_t tag, const std::vector<std::uint16_t>& values) {
        add_external(tag, 3, values.size(), le_u16_array(values));
    }
    void add_double_array(std::uint16_t tag, const std::vector<double>& values) {
        add_external(tag, 12, values.size(), le_f64_array(values));
    }
    void add_ascii(std::uint16_t tag, const std::string& s) {
        std::string with_nul = s;
        with_nul.push_back('\0');
        std::vector<std::byte> bytes(with_nul.size());
        std::memcpy(bytes.data(), with_nul.data(), with_nul.size());
        add_external(tag, 2, with_nul.size(), bytes);
    }

    // Builds the full file: header + one IFD (entries added so far, plus
    // StripOffsets/StripByteCounts for `strip_data`) + external value blocks
    // + strip data.
    std::vector<std::byte> build(const std::vector<std::byte>& strip_data) const {
        std::vector<Entry> all = entries_;
        const std::size_t ifd_header_size = 2 + 12 * (all.size() + 2) + 4;
        const std::uint32_t base_offset = 8 + static_cast<std::uint32_t>(ifd_header_size);

        std::uint32_t running = 0;
        std::vector<std::uint32_t> external_offsets(all.size(), 0);
        for (std::size_t i = 0; i < all.size(); ++i) {
            if (!all[i].external.empty()) {
                external_offsets[i] = base_offset + running;
                running += static_cast<std::uint32_t>(all[i].external.size());
            }
        }
        const std::uint32_t strip_offset = base_offset + running;

        Entry strip_offsets_entry;
        strip_offsets_entry.tag = 273;
        strip_offsets_entry.type = 4;
        strip_offsets_entry.count = 1;
        strip_offsets_entry.inline_value = le_u32(strip_offset);
        Entry strip_bytecounts_entry;
        strip_bytecounts_entry.tag = 279;
        strip_bytecounts_entry.type = 4;
        strip_bytecounts_entry.count = 1;
        strip_bytecounts_entry.inline_value = le_u32(static_cast<std::uint32_t>(strip_data.size()));
        all.push_back(strip_offsets_entry);
        all.push_back(strip_bytecounts_entry);

        std::vector<std::byte> out;
        out.push_back(std::byte{'I'});
        out.push_back(std::byte{'I'});
        auto app32 = [&](std::vector<std::byte>& v, const std::vector<std::byte>& b) {
            v.insert(v.end(), b.begin(), b.end());
        };
        app32(out, le_u16(42));
        app32(out, le_u32(8));
        app32(out, le_u16(static_cast<std::uint16_t>(all.size())));
        for (std::size_t i = 0; i < all.size(); ++i) {
            const Entry& e = all[i];
            app32(out, le_u16(e.tag));
            app32(out, le_u16(e.type));
            app32(out, le_u32(e.count));
            if (!e.external.empty()) {
                app32(out, le_u32(external_offsets[i]));
            } else {
                std::vector<std::byte> v = e.inline_value;
                v.resize(4, std::byte{0});
                app32(out, v);
            }
        }
        app32(out, le_u32(0)); // next IFD
        for (std::size_t i = 0; i < all.size(); ++i) {
            if (!all[i].external.empty()) app32(out, all[i].external);
        }
        app32(out, strip_data);
        return out;
    }

private:
    struct Entry {
        std::uint16_t tag;
        std::uint16_t type;
        std::uint32_t count;
        std::vector<std::byte> inline_value; // used when external is empty
        std::vector<std::byte> external;     // used when the value doesn't fit in 4 bytes
    };
    void add_inline(std::uint16_t tag, std::uint16_t type, std::uint32_t count, std::vector<std::byte> value) {
        entries_.push_back(Entry{tag, type, count, std::move(value), {}});
    }
    void add_external(std::uint16_t tag, std::uint16_t type, std::size_t count, std::vector<std::byte> value) {
        entries_.push_back(Entry{tag, type, static_cast<std::uint32_t>(count), {}, std::move(value)});
    }
    std::vector<Entry> entries_;
};

std::vector<std::byte> make_raster_bytes_u8(int w, int h) {
    std::vector<std::byte> v(static_cast<std::size_t>(w) * h);
    for (std::size_t i = 0; i < v.size(); ++i) v[i] = static_cast<std::byte>(i % 251);
    return v;
}
std::vector<std::byte> make_raster_bytes_f32(int w, int h) {
    std::vector<float> f(static_cast<std::size_t>(w) * h);
    for (std::size_t i = 0; i < f.size(); ++i) f[i] = static_cast<float>(i) * 0.5f - 3.0f;
    std::vector<std::byte> v(f.size() * 4);
    std::memcpy(v.data(), f.data(), v.size());
    return v;
}

} // namespace

TEST_CASE("tiff: uncompressed uint8, no predictor", "[decode][tiff]") {
    const int w = 8, h = 4;
    auto raw = make_raster_bytes_u8(w, h);
    TiffBuilder b;
    b.add_short(256, static_cast<std::uint16_t>(w));
    b.add_short(257, static_cast<std::uint16_t>(h));
    b.add_short(258, 8);
    b.add_short(259, 1); // Compression::None
    b.add_short(277, 1);
    b.add_long(278, static_cast<std::uint32_t>(h));
    b.add_short(284, 1);
    b.add_short(339, 1); // SampleFormat::UnsignedInt
    auto bytes = b.build(raw);
    auto src = std::make_shared<MemoryByteSource>(std::move(bytes));
    TiffReader reader(src);
    REQUIRE(reader.ifds().size() == 1);
    CHECK(reader.ifds()[0].width == static_cast<std::uint64_t>(w));
    auto r = reader.decode_window_u8(0, 0, 0, w, h);
    for (int y = 0; y < h; ++y)
        for (int x = 0; x < w; ++x) CHECK(r.at(x, y) == static_cast<std::uint8_t>((y * w + x) % 251));
}

TEST_CASE("tiff: float32 uncompressed, no predictor, windowed decode", "[decode][tiff]") {
    const int w = 6, h = 5;
    auto raw = make_raster_bytes_f32(w, h);
    TiffBuilder b;
    b.add_short(256, static_cast<std::uint16_t>(w));
    b.add_short(257, static_cast<std::uint16_t>(h));
    b.add_short(258, 32);
    b.add_short(259, 1);
    b.add_short(277, 1);
    b.add_long(278, static_cast<std::uint32_t>(h));
    b.add_short(284, 1);
    b.add_short(339, 3); // Float
    auto bytes = b.build(raw);
    auto src = std::make_shared<MemoryByteSource>(std::move(bytes));
    TiffReader reader(src);
    auto full = reader.decode_window_f32(0, 0, 0, w, h);
    std::vector<float> expected(w * h);
    std::memcpy(expected.data(), raw.data(), raw.size());
    for (int y = 0; y < h; ++y)
        for (int x = 0; x < w; ++x) CHECK(full.at(x, y) == expected[static_cast<std::size_t>(y) * w + x]);

    // Windowed decode: a 3x2 sub-window should match the same sub-rectangle.
    auto win = reader.decode_window_f32(0, 2, 1, 3, 2);
    for (int y = 0; y < 2; ++y)
        for (int x = 0; x < 3; ++x) CHECK(win.at(x, y) == expected[static_cast<std::size_t>(y + 1) * w + (x + 2)]);
}

TEST_CASE("tiff: PackBits uint8", "[decode][tiff]") {
    const int w = 10, h = 3;
    auto raw = make_raster_bytes_u8(w, h);
    TiffBuilder b;
    b.add_short(256, static_cast<std::uint16_t>(w));
    b.add_short(257, static_cast<std::uint16_t>(h));
    b.add_short(258, 8);
    b.add_short(259, 32773); // PackBits
    b.add_short(277, 1);
    b.add_long(278, static_cast<std::uint32_t>(h));
    b.add_short(284, 1);
    b.add_short(339, 1);
    auto bytes = b.build(packbits_encode_trivial(raw));
    auto src = std::make_shared<MemoryByteSource>(std::move(bytes));
    TiffReader reader(src);
    auto r = reader.decode_window_u8(0, 0, 0, w, h);
    for (int y = 0; y < h; ++y)
        for (int x = 0; x < w; ++x) CHECK(r.at(x, y) == static_cast<std::uint8_t>((y * w + x) % 251));
}

TEST_CASE("tiff: LZW (literal-only stream), predictor none, uint16", "[decode][tiff]") {
    const int w = 5, h = 3;
    std::vector<std::uint16_t> vals(static_cast<std::size_t>(w) * h);
    for (std::size_t i = 0; i < vals.size(); ++i) vals[i] = static_cast<std::uint16_t>(1000 + i * 7);
    std::vector<std::byte> raw(vals.size() * 2);
    std::memcpy(raw.data(), vals.data(), raw.size());

    TiffBuilder b;
    b.add_short(256, static_cast<std::uint16_t>(w));
    b.add_short(257, static_cast<std::uint16_t>(h));
    b.add_short(258, 16);
    b.add_short(259, 5); // LZW
    b.add_short(277, 1);
    b.add_long(278, static_cast<std::uint32_t>(h));
    b.add_short(284, 1);
    b.add_short(339, 1);
    auto bytes = b.build(lzw_encode_literal_only(raw));
    auto src = std::make_shared<MemoryByteSource>(std::move(bytes));
    TiffReader reader(src);
    auto r = reader.decode_window_u16(0, 0, 0, w, h);
    for (int y = 0; y < h; ++y)
        for (int x = 0; x < w; ++x) CHECK(r.at(x, y) == vals[static_cast<std::size_t>(y) * w + x]);
}

TEST_CASE("tiff: Deflate + horizontal-differencing predictor (2), float32 - the DGM1 shape", "[decode][tiff]") {
    const int w = 4, h = 3;
    auto raw = make_raster_bytes_f32(w, h);
    std::vector<std::byte> encoded(raw.size());
    for (int y = 0; y < h; ++y) {
        std::vector<std::byte> row(raw.begin() + static_cast<std::ptrdiff_t>(y) * w * 4,
                                    raw.begin() + static_cast<std::ptrdiff_t>(y + 1) * w * 4);
        // DGM1 fact (G2M-002): predictor 2 over float32 LZW/Deflate data is a
        // cumulative sum over the raw uint32 WORD, not bytes - encode the
        // same way.
        apply_predictor2_encode(row, w, 32);
        std::memcpy(encoded.data() + static_cast<std::ptrdiff_t>(y) * w * 4, row.data(), row.size());
    }

    TiffBuilder b;
    b.add_short(256, static_cast<std::uint16_t>(w));
    b.add_short(257, static_cast<std::uint16_t>(h));
    b.add_short(258, 32);
    b.add_short(259, 8); // Deflate
    b.add_short(277, 1);
    b.add_long(278, static_cast<std::uint32_t>(h));
    b.add_short(284, 1);
    b.add_short(317, 2); // Predictor::HorizontalDifferencing
    b.add_short(339, 3); // Float
    auto bytes = b.build(deflate_zlib_compress(encoded));
    auto src = std::make_shared<MemoryByteSource>(std::move(bytes));
    TiffReader reader(src);
    auto r = reader.decode_window_f32(0, 0, 0, w, h);
    std::vector<float> expected(static_cast<std::size_t>(w) * h);
    std::memcpy(expected.data(), raw.data(), raw.size());
    for (int y = 0; y < h; ++y)
        for (int x = 0; x < w; ++x) CHECK(r.at(x, y) == expected[static_cast<std::size_t>(y) * w + x]);
}

TEST_CASE("tiff: Deflate + floating-point predictor (3), float32", "[decode][tiff]") {
    const int w = 4, h = 2;
    auto raw = make_raster_bytes_f32(w, h);
    std::vector<std::byte> encoded(raw.size());
    for (int y = 0; y < h; ++y) {
        std::vector<std::byte> row(raw.begin() + static_cast<std::ptrdiff_t>(y) * w * 4,
                                    raw.begin() + static_cast<std::ptrdiff_t>(y + 1) * w * 4);
        auto enc_row = apply_predictor3_encode(row, w, 4);
        std::memcpy(encoded.data() + static_cast<std::ptrdiff_t>(y) * w * 4, enc_row.data(), enc_row.size());
    }
    TiffBuilder b;
    b.add_short(256, static_cast<std::uint16_t>(w));
    b.add_short(257, static_cast<std::uint16_t>(h));
    b.add_short(258, 32);
    b.add_short(259, 8);
    b.add_short(277, 1);
    b.add_long(278, static_cast<std::uint32_t>(h));
    b.add_short(284, 1);
    b.add_short(317, 3); // Predictor::FloatingPoint
    b.add_short(339, 3);
    auto bytes = b.build(deflate_zlib_compress(encoded));
    auto src = std::make_shared<MemoryByteSource>(std::move(bytes));
    TiffReader reader(src);
    auto r = reader.decode_window_f32(0, 0, 0, w, h);
    std::vector<float> expected(static_cast<std::size_t>(w) * h);
    std::memcpy(expected.data(), raw.data(), raw.size());
    for (int y = 0; y < h; ++y)
        for (int x = 0; x < w; ++x) CHECK(r.at(x, y) == expected[static_cast<std::size_t>(y) * w + x]);
}

TEST_CASE("tiff: GeoTIFF ModelPixelScale/Tiepoint + GeoKeyDirectory + GDAL_NODATA", "[decode][tiff]") {
    const int w = 2, h = 2;
    auto raw = make_raster_bytes_f32(w, h);
    TiffBuilder b;
    b.add_short(256, static_cast<std::uint16_t>(w));
    b.add_short(257, static_cast<std::uint16_t>(h));
    b.add_short(258, 32);
    b.add_short(259, 1);
    b.add_short(277, 1);
    b.add_long(278, static_cast<std::uint32_t>(h));
    b.add_short(284, 1);
    b.add_short(339, 3);
    b.add_double_array(33550, {1.0, 1.0, 0.0}); // ModelPixelScaleTag: 1m pixels
    b.add_double_array(33922, {0.0, 0.0, 0.0, 436000.5, 5560999.5, 0.0}); // ModelTiepointTag, PixelIsPoint convention
    // GeoKeyDirectory: header{1,1,0,2} + RasterTypeGeoKey=2 (Point) +
    // ProjectedCSTypeGeoKey=25832 (ETRS89 / UTM 32N).
    b.add_short_array(34735, {1, 1, 0, 2, 1025, 0, 1, 2, 3072, 0, 1, 25832});
    b.add_ascii(42113, "-9999");
    auto bytes = b.build(raw);
    auto src = std::make_shared<MemoryByteSource>(std::move(bytes));
    TiffReader reader(src);
    const auto& ifd = reader.ifds()[0];
    REQUIRE(ifd.geo_transform.has_value());
    CHECK(ifd.geo_transform->origin_x == 436000.5);
    CHECK(ifd.geo_transform->origin_y == 5560999.5);
    CHECK(ifd.geo_transform->pixel_width == 1.0);
    CHECK(ifd.raster_type == RasterType::PixelIsPoint);
    CHECK(ifd.crs.kind == CrsKind::Utm);
    CHECK(ifd.crs.utm_zone == 32);
    CHECK_FALSE(ifd.crs.utm_south);
    REQUIRE(ifd.gdal_nodata.has_value());
    CHECK(*ifd.gdal_nodata == "-9999");

    auto georef = to_raster_georef(*ifd.geo_transform, w, h, SampleRegistration::Cell, RowOrder::NorthToSouth,
                                    geo::UtmZone{32, geo::Hemisphere::North});
    REQUIRE(georef.has_value());
    // Point tiepoint at (436000.5, 5560999.5) IS the row-0/col-0 sample
    // centre already (PixelIsPoint semantics): matches DGM1's own stated
    // convention directly.
    CHECK(georef->sample_easting_m(0) == 436000.5);
    CHECK(georef->sample_northing_m(0) == 5560999.5);
}

TEST_CASE("tiff: rejects SamplesPerPixel != 1", "[decode][tiff]") {
    TiffBuilder b;
    b.add_short(256, 2);
    b.add_short(257, 2);
    b.add_short(258, 8);
    b.add_short(259, 1);
    b.add_short(277, 3); // 3 bands - unsupported
    b.add_long(278, 2);
    b.add_short(284, 1);
    b.add_short(339, 1);
    auto bytes = b.build(std::vector<std::byte>(2 * 2 * 3, std::byte{0}));
    auto src = std::make_shared<MemoryByteSource>(std::move(bytes));
    CHECK_THROWS_AS(TiffReader(src), DecodeError);
}

TEST_CASE("tiff: decode window out of bounds throws", "[decode][tiff]") {
    const int w = 4, h = 4;
    auto raw = make_raster_bytes_u8(w, h);
    TiffBuilder b;
    b.add_short(256, static_cast<std::uint16_t>(w));
    b.add_short(257, static_cast<std::uint16_t>(h));
    b.add_short(258, 8);
    b.add_short(259, 1);
    b.add_short(277, 1);
    b.add_long(278, static_cast<std::uint32_t>(h));
    b.add_short(284, 1);
    b.add_short(339, 1);
    auto bytes = b.build(raw);
    auto src = std::make_shared<MemoryByteSource>(std::move(bytes));
    TiffReader reader(src);
    CHECK_THROWS_AS(reader.decode_window_u8(0, 2, 2, 4, 4), DecodeError);
}

TEST_CASE("tiff: fuzz_tiff regression - crafted IFD (Count=0 tag, huge entry/field counts) throws DecodeError, not std::terminate",
          "[decode][tiff]") {
    // 60-byte crafted classic TIFF from the G1c step 0 fuzz_tiff crash (WSL
    // linux-asan-ubsan/linux-release fuzz legs, ~a few seconds in): before
    // the fix, an ImageWidth entry with Count=0 made
    // read_uint_array(...).at(0) throw std::out_of_range, and other entries
    // in the same file carry implausibly large Count/entry-count values
    // that reached out.reserve()/raw_entries.reserve() directly - none of
    // that is a g2m::decode::DecodeError, so it escaped every catch
    // boundary as std::terminate instead of a clean decode failure.
    const std::vector<std::byte> bytes = {
        std::byte{0x49}, std::byte{0x49}, std::byte{0x2a}, std::byte{0x00}, std::byte{0x04}, std::byte{0x00},
        std::byte{0x00}, std::byte{0x00}, std::byte{0x00}, std::byte{0x00}, std::byte{0x00}, std::byte{0x00},
        std::byte{0x2a}, std::byte{0x00}, std::byte{0x04}, std::byte{0x00}, std::byte{0x00}, std::byte{0x00},
        std::byte{0x00}, std::byte{0x01}, std::byte{0x01}, std::byte{0x00}, std::byte{0x00}, std::byte{0x00},
        std::byte{0x00}, std::byte{0x00}, std::byte{0x00}, std::byte{0x00}, std::byte{0x00}, std::byte{0x2a},
        std::byte{0x00}, std::byte{0x04}, std::byte{0x00}, std::byte{0x00}, std::byte{0x00}, std::byte{0x00},
        std::byte{0x01}, std::byte{0x01}, std::byte{0x00}, std::byte{0x49}, std::byte{0x2a}, std::byte{0x00},
        std::byte{0x04}, std::byte{0x00}, std::byte{0x00}, std::byte{0xc9}, std::byte{0xc9}, std::byte{0xc9},
        std::byte{0xc9}, std::byte{0xc9}, std::byte{0xc9}, std::byte{0xc9}, std::byte{0xc9}, std::byte{0x08},
        std::byte{0x00}, std::byte{0x03}, std::byte{0x2a}, std::byte{0x00}, std::byte{0x04}, std::byte{0x00},
    };
    auto src = std::make_shared<MemoryByteSource>(bytes);
    CHECK_THROWS_AS(TiffReader(src), DecodeError);
}

TEST_CASE("tfw: parses a well-formed world file", "[decode][tiff][io]") {
    const std::string text = "1.0\n0.0\n0.0\n-1.0\n436000.5\n5560999.5\n";
    auto t = parse_tfw(text);
    REQUIRE(t.has_value());
    CHECK(t->origin_x == 436000.5);
    CHECK(t->origin_y == 5560999.5);
    CHECK(t->pixel_width == 1.0);
    CHECK(t->pixel_height == 1.0);
}

TEST_CASE("tfw: rejects rotation terms and non-north-up orientation", "[decode][tiff][io]") {
    CHECK_FALSE(parse_tfw("1.0\n0.1\n0.0\n-1.0\n0.0\n0.0\n").has_value()); // rotation != 0
    CHECK_FALSE(parse_tfw("1.0\n0.0\n0.0\n1.0\n0.0\n0.0\n").has_value());  // e >= 0 (not north-up)
    CHECK_FALSE(parse_tfw("not a number\n").has_value());
}
