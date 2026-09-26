// src/decode/zip/zip_reader.cpp - see g2m/decode/zip/zip_reader.h.
//
// Format references: PKWARE APPNOTE.TXT 6.3.x. Byte layout notes are given
// inline as (offset, size) pairs from each record's signature.

#include "g2m/decode/zip/zip_reader.h"

#include <libdeflate.h>

#include <algorithm>
#include <cstring>

namespace g2m::decode {
namespace {

constexpr std::uint32_t kEocdSig = 0x06054b50;
constexpr std::uint32_t kEocd64LocatorSig = 0x07064b50;
constexpr std::uint32_t kEocd64Sig = 0x06064b50;
constexpr std::uint32_t kCentralDirSig = 0x02014b50;
constexpr std::uint32_t kLocalHeaderSig = 0x04034b50;
constexpr std::uint16_t kZip64ExtraId = 0x0001;

std::uint16_t rd_u16(const std::byte* p) {
    std::uint16_t v;
    std::memcpy(&v, p, 2);
    return v; // little-endian host assumed (see FileByteSource header comment - Win/WSL x86_64 only)
}
std::uint32_t rd_u32(const std::byte* p) {
    std::uint32_t v;
    std::memcpy(&v, p, 4);
    return v;
}
std::uint64_t rd_u64(const std::byte* p) {
    std::uint64_t v;
    std::memcpy(&v, p, 8);
    return v;
}

// Searches the last `search_window` bytes of `src` for the EOCD signature,
// scanning backward so the LAST (innermost, correct) match wins even if the
// file comment happens to contain the 4-byte signature earlier in the
// window. The EOCD record is 22 bytes plus a 0-65535 byte comment, so the
// signature can be at most 22 + 65535 bytes from EOF.
std::uint64_t find_eocd(const IByteSource& src) {
    const std::uint64_t size = src.size();
    if (size < 22) throw DecodeError("zip: file too small for an EOCD record");
    const std::uint64_t max_back = std::min<std::uint64_t>(size, 22 + 65535);
    const std::uint64_t window_start = size - max_back;
    const auto window = src.read(window_start, max_back);
    for (std::int64_t i = static_cast<std::int64_t>(window.size()) - 22; i >= 0; --i) {
        if (rd_u32(&window[static_cast<std::size_t>(i)]) == kEocdSig) {
            return window_start + static_cast<std::uint64_t>(i);
        }
    }
    throw DecodeError("zip: end-of-central-directory record not found");
}

struct Eocd {
    std::uint64_t central_dir_offset;
    std::uint64_t central_dir_count;
};

Eocd parse_eocd(const IByteSource& src, std::uint64_t eocd_off) {
    const auto rec = src.read(eocd_off, 22);
    const std::uint16_t this_disk = rd_u16(&rec[4]);
    const std::uint16_t cd_disk = rd_u16(&rec[6]);
    if (this_disk != 0 || cd_disk != 0) throw DecodeError("zip: split/spanned archives are not supported");
    std::uint16_t count16 = rd_u16(&rec[10]);
    std::uint32_t cd_size32 = rd_u32(&rec[12]);
    std::uint32_t cd_off32 = rd_u32(&rec[16]);

    // ZIP64: if the classic fields are saturated (0xFFFF/0xFFFFFFFF), the
    // real values live in the ZIP64 EOCD record, found via the locator that
    // immediately precedes this EOCD (locator is a fixed 20 bytes).
    const bool maybe_zip64 = count16 == 0xFFFF || cd_off32 == 0xFFFFFFFF || cd_size32 == 0xFFFFFFFF;
    if (!maybe_zip64) {
        return Eocd{cd_off32, count16};
    }
    if (eocd_off < 20) throw DecodeError("zip: ZIP64 EOCD locator would be before start of file");
    const auto loc = src.read(eocd_off - 20, 20);
    if (rd_u32(&loc[0]) != kEocd64LocatorSig) {
        throw DecodeError("zip: ZIP64 fields indicated but no ZIP64 EOCD locator found");
    }
    const std::uint64_t eocd64_off = rd_u64(&loc[8]);
    const auto rec64_hdr = src.read(eocd64_off, 56);
    if (rd_u32(&rec64_hdr[0]) != kEocd64Sig) throw DecodeError("zip: malformed ZIP64 EOCD record");
    const std::uint64_t count = rd_u64(&rec64_hdr[32]);
    const std::uint64_t cd_off = rd_u64(&rec64_hdr[48]);
    return Eocd{cd_off, count};
}

// Reads the ZIP64 extra field (id 0x0001) values that are present per the
// APPNOTE rule: only fields whose classic-header counterpart was saturated
// appear, in the fixed order uncompressed_size, compressed_size,
// local_header_offset, disk_start_number. `have_*` says which of the
// classic fields were saturated (so which ZIP64 fields to expect).
struct Zip64Extra {
    std::uint64_t uncompressed_size = 0;
    std::uint64_t compressed_size = 0;
    std::uint64_t local_header_offset = 0;
};

Zip64Extra parse_zip64_extra(std::span<const std::byte> extra, bool have_uncompressed, bool have_compressed,
                              bool have_offset) {
    Zip64Extra out{};
    std::size_t pos = 0;
    while (pos + 4 <= extra.size()) {
        const std::uint16_t id = rd_u16(&extra[pos]);
        const std::uint16_t len = rd_u16(&extra[pos + 2]);
        if (pos + 4 + len > extra.size()) break;
        if (id == kZip64ExtraId) {
            std::size_t p = pos + 4;
            const std::byte* base = extra.data();
            if (have_uncompressed && p + 8 <= pos + 4 + len) { out.uncompressed_size = rd_u64(base + p); p += 8; }
            if (have_compressed && p + 8 <= pos + 4 + len) { out.compressed_size = rd_u64(base + p); p += 8; }
            if (have_offset && p + 8 <= pos + 4 + len) { out.local_header_offset = rd_u64(base + p); p += 8; }
            return out;
        }
        pos += 4 + len;
    }
    throw DecodeError("zip: ZIP64 sizes/offset indicated but no ZIP64 extra field present");
}

} // namespace

std::string to_lossy_display_name(const std::string& raw_name) {
    std::string out;
    out.reserve(raw_name.size());
    const auto* p = reinterpret_cast<const unsigned char*>(raw_name.data());
    const std::size_t n = raw_name.size();
    std::size_t i = 0;
    auto push_replacement = [&out]() { out += "\xEF\xBF\xBD"; }; // U+FFFD, UTF-8
    while (i < n) {
        const unsigned char c0 = p[i];
        std::size_t len = 0;
        std::uint32_t cp = 0;
        if (c0 < 0x80) { len = 1; cp = c0; }
        else if ((c0 & 0xE0) == 0xC0) { len = 2; cp = c0 & 0x1F; }
        else if ((c0 & 0xF0) == 0xE0) { len = 3; cp = c0 & 0x0F; }
        else if ((c0 & 0xF8) == 0xF0) { len = 4; cp = c0 & 0x07; }
        else { push_replacement(); ++i; continue; }
        if (i + len > n) { push_replacement(); ++i; continue; }
        bool ok = true;
        for (std::size_t k = 1; k < len; ++k) {
            const unsigned char ck = p[i + k];
            if ((ck & 0xC0) != 0x80) { ok = false; break; }
            cp = (cp << 6) | (ck & 0x3F);
        }
        // Reject overlong encodings and surrogate/out-of-range code points.
        static constexpr std::uint32_t kMin[5] = {0, 0, 0x80, 0x800, 0x10000};
        if (!ok || cp < kMin[len] || cp > 0x10FFFF || (cp >= 0xD800 && cp <= 0xDFFF)) {
            push_replacement();
            ++i;
            continue;
        }
        out.append(raw_name, i, len);
        i += len;
    }
    return out;
}

ZipReader::ZipReader(std::shared_ptr<IByteSource> source) : source_(std::move(source)) {
    if (!source_) throw DecodeError("zip: null byte source");
    const std::uint64_t eocd_off = find_eocd(*source_);
    const Eocd eocd = parse_eocd(*source_, eocd_off);

    entries_.reserve(static_cast<std::size_t>(std::min<std::uint64_t>(eocd.central_dir_count, 1u << 20)));
    std::uint64_t pos = eocd.central_dir_offset;
    for (std::uint64_t i = 0; i < eocd.central_dir_count; ++i) {
        const auto hdr = source_->read(pos, 46);
        if (rd_u32(&hdr[0]) != kCentralDirSig) throw DecodeError("zip: malformed central directory entry");
        const std::uint16_t method = rd_u16(&hdr[10]);
        const std::uint32_t crc = rd_u32(&hdr[16]);
        std::uint32_t comp32 = rd_u32(&hdr[20]);
        std::uint32_t uncomp32 = rd_u32(&hdr[24]);
        const std::uint16_t name_len = rd_u16(&hdr[28]);
        const std::uint16_t extra_len = rd_u16(&hdr[30]);
        const std::uint16_t comment_len = rd_u16(&hdr[32]);
        std::uint32_t local_off32 = rd_u32(&hdr[42]);

        const auto name_bytes = source_->read(pos + 46, name_len);
        std::string name(reinterpret_cast<const char*>(name_bytes.data()), name_bytes.size());

        ZipEntry entry;
        entry.name = name;
        entry.method = static_cast<ZipMethod>(method);
        entry.crc32 = crc;
        entry.compressed_size = comp32;
        entry.uncompressed_size = uncomp32;
        entry.local_header_offset = local_off32;
        entry.is_directory = !name.empty() && name.back() == '/' && uncomp32 == 0;

        if (extra_len > 0) {
            const auto extra = source_->read(pos + 46 + name_len, extra_len);
            const bool need_u = uncomp32 == 0xFFFFFFFF;
            const bool need_c = comp32 == 0xFFFFFFFF;
            const bool need_o = local_off32 == 0xFFFFFFFF;
            if (need_u || need_c || need_o) {
                const Zip64Extra z = parse_zip64_extra(extra, need_u, need_c, need_o);
                if (need_u) entry.uncompressed_size = z.uncompressed_size;
                if (need_c) entry.compressed_size = z.compressed_size;
                if (need_o) entry.local_header_offset = z.local_header_offset;
            }
        }

        if (method != 0 && method != 8) {
            throw DecodeError("zip: unsupported compression method " + std::to_string(method) + " for entry '" +
                               to_lossy_display_name(name) + "'");
        }

        entries_.push_back(std::move(entry));
        pos += 46 + name_len + extra_len + comment_len;
    }
}

std::vector<std::byte> ZipReader::read_entry(const ZipEntry& entry) const {
    // Locate the actual compressed-data offset via the local header (its
    // name/extra field lengths can differ from the central directory's, so
    // the data does not start at local_header_offset + 30 in general).
    const auto lhdr = source_->read(entry.local_header_offset, 30);
    if (rd_u32(&lhdr[0]) != kLocalHeaderSig) throw DecodeError("zip: malformed local file header");
    const std::uint16_t name_len = rd_u16(&lhdr[26]);
    const std::uint16_t extra_len = rd_u16(&lhdr[28]);
    const std::uint64_t data_off = entry.local_header_offset + 30 + name_len + extra_len;

    if (entry.is_directory || entry.uncompressed_size == 0) return {};

    const auto compressed = source_->read(data_off, entry.compressed_size);

    std::vector<std::byte> out(entry.uncompressed_size);
    if (entry.method == ZipMethod::Stored) {
        if (entry.compressed_size != entry.uncompressed_size) {
            throw DecodeError("zip: stored entry size mismatch");
        }
        out = compressed;
    } else {
        libdeflate_decompressor* dec = libdeflate_alloc_decompressor();
        if (!dec) throw DecodeError("zip: libdeflate_alloc_decompressor failed");
        std::size_t actual_out = 0;
        const libdeflate_result r = libdeflate_deflate_decompress(
            dec, compressed.data(), compressed.size(), out.data(), out.size(), &actual_out);
        libdeflate_free_decompressor(dec);
        if (r != LIBDEFLATE_SUCCESS || actual_out != out.size()) {
            throw DecodeError("zip: DEFLATE decompression failed for '" + to_lossy_display_name(entry.name) + "'");
        }
    }

    const std::uint32_t crc = libdeflate_crc32(0, out.data(), out.size());
    if (crc != entry.crc32) {
        throw DecodeError("zip: CRC-32 mismatch for '" + to_lossy_display_name(entry.name) + "'");
    }
    return out;
}

std::shared_ptr<MemoryByteSource> ZipReader::read_entry_as_source(const ZipEntry& entry) const {
    return std::make_shared<MemoryByteSource>(read_entry(entry));
}

Result<std::shared_ptr<IByteSource>> ZipReader::entry_source(const ZipEntry& entry,
                                                               std::uint64_t max_inflate_bytes) const {
    if (entry.is_directory || entry.uncompressed_size == 0) {
        return std::shared_ptr<IByteSource>(std::make_shared<MemoryByteSource>(std::vector<std::byte>{}));
    }

    if (entry.method == ZipMethod::Deflate) {
        if (entry.uncompressed_size > max_inflate_bytes) {
            return make_error(ErrorCode::LimitExceeded,
                               "zip: entry '" + to_lossy_display_name(entry.name) + "' would inflate to " +
                                   std::to_string(entry.uncompressed_size) + " bytes, exceeding the " +
                                   std::to_string(max_inflate_bytes) + "-byte cap");
        }
        return std::shared_ptr<IByteSource>(std::make_shared<MemoryByteSource>(read_entry(entry)));
    }
    if (entry.method != ZipMethod::Stored) {
        throw DecodeError("zip: unsupported compression method for entry_source '" +
                           to_lossy_display_name(entry.name) + "'");
    }
    if (entry.compressed_size != entry.uncompressed_size) {
        throw DecodeError("zip: stored entry size mismatch");
    }

    // Locate the actual compressed-data offset via the local header - same
    // reasoning as read_entry() above (the central directory's own name/
    // extra field lengths can differ from the local header's).
    const auto lhdr = source_->read(entry.local_header_offset, 30);
    if (rd_u32(&lhdr[0]) != kLocalHeaderSig) throw DecodeError("zip: malformed local file header");
    const std::uint16_t name_len = rd_u16(&lhdr[26]);
    const std::uint16_t extra_len = rd_u16(&lhdr[28]);
    const std::uint64_t data_off = entry.local_header_offset + 30 + name_len + extra_len;

    // Verify the CRC-32 once here, streaming 1 MiB chunks straight from the
    // archive's own byte source rather than materialising the whole entry
    // (PLAN.md D8: bounded memory even for a several-hundred-MB inner zip).
    // A STORED entry's compressed bytes ARE its uncompressed bytes, so this
    // reads exactly what the returned SubrangeByteSource will ever expose.
    constexpr std::uint64_t kChunk = std::uint64_t{1} << 20; // 1 MiB
    std::uint32_t crc = 0;
    std::vector<std::byte> chunk;
    for (std::uint64_t off = 0; off < entry.uncompressed_size;) {
        const std::uint64_t len = std::min(kChunk, entry.uncompressed_size - off);
        source_->read(data_off + off, len, chunk);
        crc = libdeflate_crc32(crc, chunk.data(), chunk.size());
        off += len;
    }
    if (crc != entry.crc32) {
        throw DecodeError("zip: CRC-32 mismatch for '" + to_lossy_display_name(entry.name) + "'");
    }

    return std::shared_ptr<IByteSource>(
        std::make_shared<SubrangeByteSource>(source_, data_off, entry.uncompressed_size));
}

} // namespace g2m::decode
