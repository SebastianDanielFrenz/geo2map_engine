// src/decode/tiff/tiff_reader.cpp - see g2m/decode/tiff/tiff_reader.h.

#include "g2m/decode/tiff/tiff_reader.h"

#include "lzw.h"
#include "packbits.h"
#include "predictor.h"

#include <libdeflate.h>

#include <algorithm>
#include <cstring>

namespace g2m::decode::tiff {
namespace {

std::uint16_t bswap16(std::uint16_t v) { return static_cast<std::uint16_t>((v >> 8) | (v << 8)); }
std::uint32_t bswap32(std::uint32_t v) {
    return (v >> 24) | ((v >> 8) & 0x0000FF00u) | ((v << 8) & 0x00FF0000u) | (v << 24);
}
std::uint64_t bswap64(std::uint64_t v) {
    return (static_cast<std::uint64_t>(bswap32(static_cast<std::uint32_t>(v))) << 32) |
           bswap32(static_cast<std::uint32_t>(v >> 32));
}

// Byte-swaps `n` consecutive `word_size`-byte words in place (word_size in
// {1,2,4,8}; 1 is a no-op).
void swap_words_inplace(std::span<std::byte> data, unsigned word_size) {
    if (word_size <= 1) return;
    auto* p = data.data();
    const std::size_t count = data.size() / word_size;
    for (std::size_t i = 0; i < count; ++i) {
        std::byte* w = p + i * word_size;
        std::reverse(w, w + word_size);
    }
}

// TIFF tag value type ids (TIFF6 table 1 + BigTIFF's LONG8/SLONG8/IFD8).
enum class FieldType : std::uint16_t {
    Byte = 1,
    Ascii = 2,
    Short = 3,
    Long = 4,
    Rational = 5,
    SByte = 6,
    Undefined = 7,
    SShort = 8,
    SLong = 9,
    SRational = 10,
    Float = 11,
    Double = 12,
    Ifd = 13,
    Long8 = 16,
    SLong8 = 17,
    Ifd8 = 18,
};

std::uint64_t field_type_size(FieldType t) {
    switch (t) {
        case FieldType::Byte: case FieldType::Ascii: case FieldType::SByte: case FieldType::Undefined: return 1;
        case FieldType::Short: case FieldType::SShort: return 2;
        case FieldType::Long: case FieldType::SLong: case FieldType::Float: case FieldType::Ifd: return 4;
        case FieldType::Rational: case FieldType::SRational: case FieldType::Double:
        case FieldType::Long8: case FieldType::SLong8: case FieldType::Ifd8: return 8;
        default: throw DecodeError("tiff: unsupported field type " + std::to_string(static_cast<int>(t)));
    }
}

struct RawEntry {
    std::uint16_t tag = 0;
    FieldType type = FieldType::Byte;
    std::uint64_t count = 0;
    std::vector<std::byte> inline_or_offset_bytes; // 4 bytes (classic) or 8 bytes (BigTIFF), file byte order
};

class Endian {
public:
    explicit Endian(bool big) : big_(big) {}
    std::uint16_t u16(const std::byte* p) const {
        std::uint16_t v;
        std::memcpy(&v, p, 2);
        return big_ ? bswap16(v) : v;
    }
    std::uint32_t u32(const std::byte* p) const {
        std::uint32_t v;
        std::memcpy(&v, p, 4);
        return big_ ? bswap32(v) : v;
    }
    std::uint64_t u64(const std::byte* p) const {
        std::uint64_t v;
        std::memcpy(&v, p, 8);
        return big_ ? bswap64(v) : v;
    }
    bool big() const { return big_; }

private:
    bool big_;
};

// Every file-derived element/byte count is validated here before it drives a
// reserve()/resize()/vector construction. A crafted count near UINT64_MAX
// (or one that only overflows a downstream byte-count multiplication) must
// never reach an allocation call directly: std::vector::reserve/resize
// throws std::length_error or std::bad_alloc for an implausible size, and
// that is not a DecodeError - it would escape every "catch (const
// DecodeError&)" fuzz/caller boundary as an uncaught std::terminate
// (G1c step 0 item 3; found via fuzz_tiff). The bound is deliberately loose
// (a count may never need more elements than there are bytes in the whole
// source, since even the smallest element is 1 byte) so it rejects only
// implausible/overflow-driven values, never a legitimately large real file.
void check_file_derived_count(std::uint64_t count, std::uint64_t available_bytes, const char* what) {
    if (count > available_bytes) {
        throw DecodeError(std::string("tiff: ") + what + " count implausible for a " +
                           std::to_string(available_bytes) + "-byte source");
    }
}

// Reads one field's values as unsigned integers (valid for
// Byte/SByte/Short/SShort/Long/SLong/Long8/SLong8/Ifd/Ifd8 - i.e. every
// integer-ish type this reader's recognised tags actually use).
std::vector<std::uint64_t> read_uint_array(const IByteSource& src, const Endian& en, const RawEntry& e) {
    const std::uint64_t elem_size = field_type_size(e.type);
    const std::uint64_t total = elem_size * e.count;
    const std::uint64_t inline_capacity = e.inline_or_offset_bytes.size();
    std::vector<std::byte> bytes;
    if (total <= inline_capacity) {
        bytes.assign(e.inline_or_offset_bytes.begin(), e.inline_or_offset_bytes.begin() + static_cast<std::ptrdiff_t>(total));
    } else {
        const std::uint64_t offset =
            inline_capacity == 8 ? en.u64(e.inline_or_offset_bytes.data()) : en.u32(e.inline_or_offset_bytes.data());
        bytes = src.read(offset, total);
    }
    std::vector<std::uint64_t> out;
    check_file_derived_count(e.count, src.size(), "field value array");
    out.reserve(e.count);
    for (std::uint64_t i = 0; i < e.count; ++i) {
        const std::byte* p = bytes.data() + i * elem_size;
        switch (elem_size) {
            case 1: out.push_back(static_cast<std::uint8_t>(*p)); break;
            case 2: out.push_back(en.u16(p)); break;
            case 4: out.push_back(en.u32(p)); break;
            case 8: out.push_back(en.u64(p)); break;
            default: throw DecodeError("tiff: unexpected integer field width");
        }
    }
    return out;
}

// A required scalar tag (ImageWidth, Compression, ...) must carry at least
// one value; a crafted zero-Count entry would otherwise make
// read_uint_array(...).at(0) throw std::out_of_range - again not a
// DecodeError, escaping uncaught (found via fuzz_tiff on a 60-byte crafted
// TIFF with ImageWidth Count=0).
std::uint64_t read_uint_scalar(const IByteSource& src, const Endian& en, const RawEntry& e, const char* tag_name) {
    const std::vector<std::uint64_t> v = read_uint_array(src, en, e);
    if (v.empty()) throw DecodeError(std::string("tiff: ") + tag_name + " tag has Count 0");
    return v.front();
}

std::vector<double> read_double_array(const IByteSource& src, const Endian& en, const RawEntry& e) {
    const std::uint64_t elem_size = field_type_size(e.type);
    const std::uint64_t total = elem_size * e.count;
    const std::uint64_t inline_capacity = e.inline_or_offset_bytes.size();
    std::vector<std::byte> bytes;
    if (total <= inline_capacity) {
        bytes.assign(e.inline_or_offset_bytes.begin(), e.inline_or_offset_bytes.begin() + static_cast<std::ptrdiff_t>(total));
    } else {
        const std::uint64_t offset =
            inline_capacity == 8 ? en.u64(e.inline_or_offset_bytes.data()) : en.u32(e.inline_or_offset_bytes.data());
        bytes = src.read(offset, total);
    }
    std::vector<double> out;
    check_file_derived_count(e.count, src.size(), "field value array");
    out.reserve(e.count);
    if (e.type == FieldType::Double) {
        for (std::uint64_t i = 0; i < e.count; ++i) {
            const std::uint64_t bits = en.u64(bytes.data() + i * 8);
            double d;
            std::memcpy(&d, &bits, 8);
            out.push_back(d);
        }
    } else if (e.type == FieldType::Float) {
        for (std::uint64_t i = 0; i < e.count; ++i) {
            const std::uint32_t bits = en.u32(bytes.data() + i * 4);
            float f;
            std::memcpy(&f, &bits, 4);
            out.push_back(static_cast<double>(f));
        }
    } else if (e.type == FieldType::Rational) {
        for (std::uint64_t i = 0; i < e.count; ++i) {
            const std::uint32_t num = en.u32(bytes.data() + i * 8);
            const std::uint32_t den = en.u32(bytes.data() + i * 8 + 4);
            out.push_back(den != 0 ? static_cast<double>(num) / static_cast<double>(den) : 0.0);
        }
    } else {
        throw DecodeError("tiff: field is not a floating-point type");
    }
    return out;
}

std::string read_ascii(const IByteSource& src, const Endian& en, const RawEntry& e) {
    const std::uint64_t total = e.count;
    const std::uint64_t inline_capacity = e.inline_or_offset_bytes.size();
    std::vector<std::byte> bytes;
    if (total <= inline_capacity) {
        bytes.assign(e.inline_or_offset_bytes.begin(), e.inline_or_offset_bytes.begin() + static_cast<std::ptrdiff_t>(total));
    } else {
        const std::uint64_t offset =
            inline_capacity == 8 ? en.u64(e.inline_or_offset_bytes.data()) : en.u32(e.inline_or_offset_bytes.data());
        bytes = src.read(offset, total);
    }
    std::string s(reinterpret_cast<const char*>(bytes.data()), bytes.size());
    while (!s.empty() && s.back() == '\0') s.pop_back();
    return s;
}

Compression to_compression(std::uint64_t v) {
    switch (v) {
        case 1: return Compression::None;
        case 5: return Compression::Lzw;
        case 8: return Compression::Deflate;
        case 32946: return Compression::DeflateOld;
        case 32773: return Compression::PackBits;
        default: throw DecodeError("tiff: unsupported Compression " + std::to_string(v));
    }
}

} // namespace

TiffReader::TiffReader(std::shared_ptr<IByteSource> source) : source_(std::move(source)) {
    if (!source_) throw DecodeError("tiff: null byte source");
    const auto hdr = source_->read(0, 16 <= source_->size() ? 16 : source_->size());
    if (hdr.size() < 8) throw DecodeError("tiff: file too small for a TIFF header");
    if (hdr[0] == std::byte{'I'} && hdr[1] == std::byte{'I'}) {
        big_endian_ = false;
    } else if (hdr[0] == std::byte{'M'} && hdr[1] == std::byte{'M'}) {
        big_endian_ = true;
    } else {
        throw DecodeError("tiff: not a TIFF file (bad byte-order mark)");
    }
    Endian en(big_endian_);
    const std::uint16_t magic = en.u16(&hdr[2]);
    std::uint64_t first_ifd_offset;
    if (magic == 42) {
        big_tiff_ = false;
        first_ifd_offset = en.u32(&hdr[4]);
    } else if (magic == 43) {
        big_tiff_ = true;
        if (hdr.size() < 16) throw DecodeError("tiff: file too small for a BigTIFF header");
        const std::uint16_t offset_size = en.u16(&hdr[4]);
        if (offset_size != 8) throw DecodeError("tiff: unsupported BigTIFF offset size");
        first_ifd_offset = en.u64(&hdr[8]);
    } else {
        throw DecodeError("tiff: bad magic number");
    }

    const std::uint64_t entry_size = big_tiff_ ? 20 : 12;
    const std::uint64_t inline_bytes = big_tiff_ ? 8 : 4;
    const std::uint64_t count_field_size = big_tiff_ ? 8 : 2;
    const std::uint64_t next_ptr_size = big_tiff_ ? 8 : 4;

    std::uint64_t ifd_offset = first_ifd_offset;
    int guard = 0;
    while (ifd_offset != 0) {
        if (++guard > 4096) throw DecodeError("tiff: IFD chain too long (or cyclic)");
        const auto count_bytes = source_->read(ifd_offset, count_field_size);
        const std::uint64_t entry_count =
            count_field_size == 8 ? en.u64(count_bytes.data()) : en.u16(count_bytes.data());
        // Validated BEFORE `entry_count * entry_size` below: that
        // multiplication itself can overflow uint64 for a crafted huge
        // entry_count, wrapping to a small value that then passes
        // source_->read()'s own bounds check while entry_count stays huge
        // for reserve()/the entry loop.
        check_file_derived_count(entry_count, source_->size() / entry_size, "IFD entry");

        std::vector<RawEntry> raw_entries;
        raw_entries.reserve(entry_count);
        const std::uint64_t entries_off = ifd_offset + count_field_size;
        const auto entries_block = source_->read(entries_off, entry_count * entry_size);
        for (std::uint64_t i = 0; i < entry_count; ++i) {
            const std::byte* p = entries_block.data() + i * entry_size;
            RawEntry re;
            re.tag = en.u16(p);
            re.type = static_cast<FieldType>(en.u16(p + 2));
            re.count = big_tiff_ ? en.u64(p + 4) : en.u32(p + 4);
            const std::byte* value_field = p + (big_tiff_ ? 12 : 8);
            re.inline_or_offset_bytes.assign(value_field, value_field + inline_bytes);
            raw_entries.push_back(std::move(re));
        }
        const std::uint64_t next_off = entries_off + entry_count * entry_size;
        const auto next_bytes = source_->read(next_off, next_ptr_size);
        ifd_offset = next_ptr_size == 8 ? en.u64(next_bytes.data()) : en.u32(next_bytes.data());

        Ifd ifd;
        for (const RawEntry& e : raw_entries) {
            switch (e.tag) {
                case tag::kImageWidth: ifd.width = read_uint_scalar(*source_, en, e, "ImageWidth"); break;
                case tag::kImageLength: ifd.height = read_uint_scalar(*source_, en, e, "ImageLength"); break;
                case tag::kBitsPerSample: ifd.bits_per_sample = static_cast<unsigned>(read_uint_scalar(*source_, en, e, "BitsPerSample")); break;
                case tag::kCompression: ifd.compression = to_compression(read_uint_scalar(*source_, en, e, "Compression")); break;
                case tag::kSamplesPerPixel: ifd.samples_per_pixel = static_cast<unsigned>(read_uint_scalar(*source_, en, e, "SamplesPerPixel")); break;
                case tag::kRowsPerStrip: ifd.rows_per_strip = read_uint_scalar(*source_, en, e, "RowsPerStrip"); break;
                case tag::kStripOffsets: ifd.strip_offsets = read_uint_array(*source_, en, e); break;
                case tag::kStripByteCounts: ifd.strip_byte_counts = read_uint_array(*source_, en, e); break;
                case tag::kPlanarConfiguration: ifd.planar_config = static_cast<PlanarConfig>(read_uint_scalar(*source_, en, e, "PlanarConfiguration")); break;
                case tag::kPredictor: ifd.predictor = static_cast<Predictor>(read_uint_scalar(*source_, en, e, "Predictor")); break;
                case tag::kTileWidth: ifd.tile_width = read_uint_scalar(*source_, en, e, "TileWidth"); break;
                case tag::kTileLength: ifd.tile_height = read_uint_scalar(*source_, en, e, "TileLength"); break;
                case tag::kTileOffsets: ifd.tile_offsets = read_uint_array(*source_, en, e); break;
                case tag::kTileByteCounts: ifd.tile_byte_counts = read_uint_array(*source_, en, e); break;
                case tag::kSampleFormat: ifd.sample_format = static_cast<SampleFormat>(read_uint_scalar(*source_, en, e, "SampleFormat")); break;
                case tag::kGdalNoData: ifd.gdal_nodata = read_ascii(*source_, en, e); break;
                default: break; // geo tags handled in a second pass below (need width/height context-free values only)
            }
        }
        // Second pass: GeoTIFF tags (independent of the switch above so
        // ordering of tags within the IFD does not matter).
        std::vector<double> pixel_scale, tiepoint;
        std::vector<std::uint64_t> geo_keys;
        for (const RawEntry& e : raw_entries) {
            if (e.tag == tag::kModelPixelScaleTag) pixel_scale = read_double_array(*source_, en, e);
            else if (e.tag == tag::kModelTiepointTag) tiepoint = read_double_array(*source_, en, e);
            else if (e.tag == tag::kGeoKeyDirectoryTag) geo_keys = read_uint_array(*source_, en, e);
        }
        if (!geo_keys.empty() && geo_keys.size() >= 4) {
            const std::uint64_t num_keys = geo_keys[3];
            for (std::uint64_t k = 0; k < num_keys; ++k) {
                const std::uint64_t base = 4 + 4 * k;
                if (base + 3 >= geo_keys.size()) break;
                const std::uint16_t key_id = static_cast<std::uint16_t>(geo_keys[base]);
                const std::uint16_t loc = static_cast<std::uint16_t>(geo_keys[base + 1]);
                const std::uint16_t value = static_cast<std::uint16_t>(geo_keys[base + 3]);
                if (loc != 0) continue; // only directly-inlined SHORT keys are consumed (our 3 keys of interest)
                if (key_id == geokey::kGTRasterTypeGeoKey) {
                    ifd.raster_type = static_cast<RasterType>(value);
                } else if (key_id == geokey::kProjectedCSTypeGeoKey && value != 0 && value != 32767) {
                    ifd.crs.kind = CrsKind::ProjectedEpsg;
                    ifd.crs.epsg = value;
                    if (value >= 25801 && value <= 25860) { // ETRS89 / UTM zone NN N
                        ifd.crs.kind = CrsKind::Utm;
                        ifd.crs.utm_zone = value - 25800;
                        ifd.crs.utm_south = false;
                    } else if (value >= 32601 && value <= 32660) { // WGS84 / UTM zone NN N
                        ifd.crs.kind = CrsKind::Utm;
                        ifd.crs.utm_zone = value - 32600;
                        ifd.crs.utm_south = false;
                    } else if (value >= 32701 && value <= 32760) { // WGS84 / UTM zone NN S
                        ifd.crs.kind = CrsKind::Utm;
                        ifd.crs.utm_zone = value - 32700;
                        ifd.crs.utm_south = true;
                    }
                } else if (key_id == geokey::kGeographicTypeGeoKey && value != 0 && value != 32767 &&
                           ifd.crs.kind == CrsKind::Absent) {
                    ifd.crs.kind = CrsKind::GeographicEpsg;
                    ifd.crs.epsg = value;
                }
            }
        }
        if (pixel_scale.size() >= 2 && tiepoint.size() >= 6) {
            const bool area = ifd.raster_type != RasterType::PixelIsPoint; // Area is the GeoTIFF default
            const double scale_x = pixel_scale[0];
            const double scale_y = pixel_scale[1];
            const double raster_i = tiepoint[0];
            const double raster_j = tiepoint[1];
            const double model_x = tiepoint[3];
            const double model_y = tiepoint[4];
            GeoTransform t;
            t.pixel_width = scale_x;
            t.pixel_height = scale_y;
            t.origin_x = model_x - raster_i * scale_x + (area ? 0.5 * scale_x : 0.0);
            t.origin_y = model_y + raster_j * scale_y - (area ? 0.5 * scale_y : 0.0);
            if (t.valid()) ifd.geo_transform = t;
        }

        if (ifd.samples_per_pixel != 1) {
            throw DecodeError("tiff: only single-band (SamplesPerPixel == 1) images are supported, got " +
                               std::to_string(ifd.samples_per_pixel));
        }
        if (ifd.planar_config != PlanarConfig::Chunky) {
            throw DecodeError("tiff: only PlanarConfig 1 (Chunky) is supported");
        }
        ifds_.push_back(std::move(ifd));
    }
    if (ifds_.empty()) throw DecodeError("tiff: no IFDs found");
}

namespace {

std::vector<std::byte> decompress_unit(const Ifd& ifd, std::span<const std::byte> compressed,
                                        std::size_t expected_bytes) {
    switch (ifd.compression) {
        case Compression::None: {
            if (compressed.size() != expected_bytes) throw DecodeError("tiff: uncompressed unit size mismatch");
            return std::vector<std::byte>(compressed.begin(), compressed.end());
        }
        case Compression::Lzw:
            return tiff::lzw_decode(compressed, expected_bytes);
        case Compression::PackBits:
            return tiff::packbits_decode(compressed, expected_bytes);
        case Compression::Deflate:
        case Compression::DeflateOld: {
            std::vector<std::byte> out(expected_bytes);
            libdeflate_decompressor* dec = libdeflate_alloc_decompressor();
            if (!dec) throw DecodeError("tiff: libdeflate_alloc_decompressor failed");
            std::size_t actual = 0;
            const libdeflate_result r =
                libdeflate_zlib_decompress(dec, compressed.data(), compressed.size(), out.data(), out.size(), &actual);
            libdeflate_free_decompressor(dec);
            if (r != LIBDEFLATE_SUCCESS || actual != out.size()) {
                throw DecodeError("tiff: Deflate decompression failed");
            }
            return out;
        }
    }
    throw DecodeError("tiff: unhandled compression");
}

// Undoes the predictor and normalises sample byte order to host
// (little-endian), one row (of `row_width` samples) at a time, in place.
void fix_up_rows(const Ifd& ifd, std::span<std::byte> unit, std::uint64_t row_width, std::uint64_t rows,
                  bool file_big_endian) {
    const unsigned bps = ifd.bits_per_sample;
    const unsigned bytes = bps / 8;
    const std::uint64_t row_bytes = row_width * bytes;
    for (std::uint64_t r = 0; r < rows; ++r) {
        std::span<std::byte> row(unit.data() + r * row_bytes, row_bytes);
        if (ifd.predictor == Predictor::HorizontalDifferencing) {
            if (file_big_endian) swap_words_inplace(row, bytes);
            undo_predictor2(row, static_cast<int>(row_width), bps);
        } else if (ifd.predictor == Predictor::FloatingPoint) {
            undo_predictor3(row, static_cast<int>(row_width), bytes);
            swap_words_inplace(row, bytes); // predictor 3's output is always big-endian; swap to host
        } else {
            if (file_big_endian) swap_words_inplace(row, bytes);
        }
    }
}

} // namespace

void TiffReader::decode_window_raw(const Ifd& ifd, std::int64_t x, std::int64_t y, int w, int h,
                                    std::vector<std::byte>& out) const {
    if (w <= 0 || h <= 0) throw DecodeError("tiff: empty decode window");
    if (x < 0 || y < 0 || static_cast<std::uint64_t>(x) + static_cast<std::uint64_t>(w) > ifd.width ||
        static_cast<std::uint64_t>(y) + static_cast<std::uint64_t>(h) > ifd.height) {
        throw DecodeError("tiff: decode window out of bounds");
    }
    const unsigned bytes = ifd.bytes_per_sample();
    if (bytes == 0) throw DecodeError("tiff: unsupported BitsPerSample " + std::to_string(ifd.bits_per_sample));
    out.assign(static_cast<std::size_t>(w) * static_cast<std::size_t>(h) * bytes, std::byte{0});

    auto copy_rect = [&](const std::vector<std::byte>& unit, std::uint64_t unit_width, std::int64_t unit_x0,
                          std::int64_t unit_y0, std::uint64_t unit_rows) {
        // Intersect [unit_x0, unit_x0+unit_width) x [unit_y0, unit_y0+unit_rows)
        // with the requested window [x, x+w) x [y, y+h).
        const std::int64_t ix0 = std::max<std::int64_t>(x, unit_x0);
        const std::int64_t ix1 = std::min<std::int64_t>(x + w, unit_x0 + static_cast<std::int64_t>(unit_width));
        const std::int64_t iy0 = std::max<std::int64_t>(y, unit_y0);
        const std::int64_t iy1 = std::min<std::int64_t>(y + h, unit_y0 + static_cast<std::int64_t>(unit_rows));
        if (ix1 <= ix0 || iy1 <= iy0) return;
        for (std::int64_t row = iy0; row < iy1; ++row) {
            const std::byte* src_row = unit.data() + (static_cast<std::uint64_t>(row - unit_y0) * unit_width +
                                                        static_cast<std::uint64_t>(ix0 - unit_x0)) * bytes;
            std::byte* dst_row =
                out.data() + (static_cast<std::uint64_t>(row - y) * static_cast<std::uint64_t>(w) +
                              static_cast<std::uint64_t>(ix0 - x)) * bytes;
            std::memcpy(dst_row, src_row, static_cast<std::size_t>(ix1 - ix0) * bytes);
        }
    };

    // A generous absolute cap on one decompressed strip/tile unit. Below
    // this, real strip/tile geometry from any file this project reads (COG
    // tiles are typically 256x256, DGM1 strips are 1000 samples wide) stays
    // untouched; above it, a crafted TileWidth/TileLength/RowsPerStrip (or
    // the full image Width for a stripped read) would otherwise reach
    // decompress_unit's out.reserve()/vector(N) with an implausible size -
    // std::length_error/bad_alloc, not a DecodeError, escaping uncaught
    // (G1c step 0 item 3, found via fuzz_tiff).
    constexpr std::size_t kMaxDecodedUnitBytes = std::size_t{1} << 30; // 1 GiB

    if (ifd.is_tiled()) {
        if (ifd.tile_offsets.empty() || ifd.tile_offsets.size() != ifd.tile_byte_counts.size()) {
            throw DecodeError("tiff: tiled image missing tile offsets/byte counts");
        }
        if (ifd.tile_height == 0) throw DecodeError("tiff: TileLength is 0");
        const std::uint64_t tiles_across = (ifd.width + ifd.tile_width - 1) / ifd.tile_width;
        const std::uint64_t tiles_down = (ifd.height + ifd.tile_height - 1) / ifd.tile_height;
        const std::int64_t tx0 = x / static_cast<std::int64_t>(ifd.tile_width);
        const std::int64_t tx1 = (x + w - 1) / static_cast<std::int64_t>(ifd.tile_width);
        const std::int64_t ty0 = y / static_cast<std::int64_t>(ifd.tile_height);
        const std::int64_t ty1 = (y + h - 1) / static_cast<std::int64_t>(ifd.tile_height);
        for (std::int64_t ty = ty0; ty <= ty1; ++ty) {
            for (std::int64_t tx = tx0; tx <= tx1; ++tx) {
                const std::uint64_t tile_index =
                    static_cast<std::uint64_t>(ty) * tiles_across + static_cast<std::uint64_t>(tx);
                if (tile_index >= ifd.tile_offsets.size()) throw DecodeError("tiff: tile index out of range");
                (void)tiles_down;
                const auto compressed =
                    source_->read(ifd.tile_offsets[tile_index], ifd.tile_byte_counts[tile_index]);
                const std::size_t expected =
                    static_cast<std::size_t>(ifd.tile_width) * static_cast<std::size_t>(ifd.tile_height) * bytes;
                if (expected > kMaxDecodedUnitBytes) throw DecodeError("tiff: tile too large");
                auto unit = decompress_unit(ifd, compressed, expected);
                fix_up_rows(ifd, unit, ifd.tile_width, ifd.tile_height, big_endian_);
                copy_rect(unit, ifd.tile_width, tx * static_cast<std::int64_t>(ifd.tile_width),
                          ty * static_cast<std::int64_t>(ifd.tile_height), ifd.tile_height);
            }
        }
    } else {
        if (ifd.strip_offsets.empty() || ifd.strip_offsets.size() != ifd.strip_byte_counts.size() ||
            ifd.rows_per_strip == 0) {
            throw DecodeError("tiff: stripped image missing strip offsets/byte counts/RowsPerStrip");
        }
        const std::int64_t s0 = y / static_cast<std::int64_t>(ifd.rows_per_strip);
        const std::int64_t s1 = (y + h - 1) / static_cast<std::int64_t>(ifd.rows_per_strip);
        for (std::int64_t s = s0; s <= s1; ++s) {
            if (static_cast<std::size_t>(s) >= ifd.strip_offsets.size()) throw DecodeError("tiff: strip index out of range");
            const std::uint64_t strip_row0 = static_cast<std::uint64_t>(s) * ifd.rows_per_strip;
            const std::uint64_t rows_here = std::min<std::uint64_t>(ifd.rows_per_strip, ifd.height - strip_row0);
            const auto compressed =
                source_->read(ifd.strip_offsets[static_cast<std::size_t>(s)], ifd.strip_byte_counts[static_cast<std::size_t>(s)]);
            const std::size_t expected = static_cast<std::size_t>(ifd.width) * static_cast<std::size_t>(rows_here) * bytes;
            if (expected > kMaxDecodedUnitBytes) throw DecodeError("tiff: strip too large");
            auto unit = decompress_unit(ifd, compressed, expected);
            fix_up_rows(ifd, unit, ifd.width, rows_here, big_endian_);
            copy_rect(unit, ifd.width, 0, static_cast<std::int64_t>(strip_row0), rows_here);
        }
    }
}

namespace {
template <class T>
void check_type(const Ifd& ifd, SampleFormat expect_fmt, unsigned expect_bits) {
    if (ifd.sample_format != expect_fmt || ifd.bits_per_sample != expect_bits) {
        throw DecodeError("tiff: requested Raster<T> type does not match the IFD's SampleFormat/BitsPerSample");
    }
    static_assert(sizeof(T) * 8 == 0 || true); // T's size is checked by the caller passing matching expect_bits
}
} // namespace

Raster<float> TiffReader::decode_window_f32(std::size_t ifd_index, std::int64_t x, std::int64_t y, int w,
                                             int h) const {
    const Ifd& ifd = ifds_.at(ifd_index);
    check_type<float>(ifd, SampleFormat::Float, 32);
    std::vector<std::byte> raw;
    decode_window_raw(ifd, x, y, w, h, raw);
    Raster<float> r(w, h);
    std::memcpy(r.data().data(), raw.data(), raw.size());
    return r;
}

Raster<std::uint8_t> TiffReader::decode_window_u8(std::size_t ifd_index, std::int64_t x, std::int64_t y, int w,
                                                   int h) const {
    const Ifd& ifd = ifds_.at(ifd_index);
    check_type<std::uint8_t>(ifd, SampleFormat::UnsignedInt, 8);
    std::vector<std::byte> raw;
    decode_window_raw(ifd, x, y, w, h, raw);
    Raster<std::uint8_t> r(w, h);
    std::memcpy(r.data().data(), raw.data(), raw.size());
    return r;
}

Raster<std::uint16_t> TiffReader::decode_window_u16(std::size_t ifd_index, std::int64_t x, std::int64_t y, int w,
                                                     int h) const {
    const Ifd& ifd = ifds_.at(ifd_index);
    check_type<std::uint16_t>(ifd, SampleFormat::UnsignedInt, 16);
    std::vector<std::byte> raw;
    decode_window_raw(ifd, x, y, w, h, raw);
    Raster<std::uint16_t> r(w, h);
    std::memcpy(r.data().data(), raw.data(), raw.size());
    return r;
}

Raster<std::int16_t> TiffReader::decode_window_i16(std::size_t ifd_index, std::int64_t x, std::int64_t y, int w,
                                                    int h) const {
    const Ifd& ifd = ifds_.at(ifd_index);
    check_type<std::int16_t>(ifd, SampleFormat::SignedInt, 16);
    std::vector<std::byte> raw;
    decode_window_raw(ifd, x, y, w, h, raw);
    Raster<std::int16_t> r(w, h);
    std::memcpy(r.data().data(), raw.data(), raw.size());
    return r;
}

Raster<std::int32_t> TiffReader::decode_window_i32(std::size_t ifd_index, std::int64_t x, std::int64_t y, int w,
                                                    int h) const {
    const Ifd& ifd = ifds_.at(ifd_index);
    check_type<std::int32_t>(ifd, SampleFormat::SignedInt, 32);
    std::vector<std::byte> raw;
    decode_window_raw(ifd, x, y, w, h, raw);
    Raster<std::int32_t> r(w, h);
    std::memcpy(r.data().data(), raw.data(), raw.size());
    return r;
}

} // namespace g2m::decode::tiff
