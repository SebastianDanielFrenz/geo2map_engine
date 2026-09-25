// g2m/decode/zip/zip_reader.h - a pure, bounds-checked ZIP reader (PLAN.md
// 13, milestone G1b).
//
// Scope (matches what the Hessen DGM1 delivery ZIPs and their nested inner
// ZIPs actually use): end-of-central-directory search (with a ZIP64
// locator/record), the central directory, stored and DEFLATE entries
// (DEFLATE via libdeflate), CRC-32 verification, ZIP64 extra fields for
// sizes/offsets past the 32-bit classic-ZIP limit.
//
// NOT implemented (throws DecodeError if encountered): encryption, methods
// other than stored/DEFLATE, split/spanned archives, data descriptors
// without a preceding local-header-size (we always trust the CENTRAL
// directory's sizes, which are mandatory and authoritative, and only use
// the local header to locate the compressed-data offset - so a data
// descriptor's own field values are never needed).
//
// Filenames are RAW BYTES (ZipEntry::name), never assumed to be UTF-8: the
// Hessen municipality ZIPs are not. to_lossy_display_name() is a
// best-effort UTF-8 interpretation (invalid sequences become U+FFFD) for
// logging/UI only - never use it as a lookup key.
#pragma once

#include "g2m/decode/byte_source.h"

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace g2m::decode {

enum class ZipMethod : std::uint16_t {
    Stored = 0,
    Deflate = 8,
};

struct ZipEntry {
    std::string name;               // raw bytes, exactly as stored (see file header comment)
    ZipMethod method = ZipMethod::Stored;
    std::uint64_t compressed_size = 0;
    std::uint64_t uncompressed_size = 0;
    std::uint32_t crc32 = 0;
    std::uint64_t local_header_offset = 0; // into the archive's own byte source
    bool is_directory = false;             // name ends in '/' with size 0 (a ZIP convention, not a flag byte)
};

// Best-effort UTF-8 decode of a raw entry name for display only: each byte
// sequence that is not valid UTF-8 is replaced by U+FFFD (one replacement
// per invalid byte, so byte length differences are visible rather than
// silently collapsed). Valid UTF-8 input round-trips unchanged.
std::string to_lossy_display_name(const std::string& raw_name);

// Parses one ZIP archive's end-of-central-directory + central directory.
// Does not decompress anything until read_entry() is called for a specific
// entry. Throws DecodeError on a structurally invalid archive.
class ZipReader {
public:
    explicit ZipReader(std::shared_ptr<IByteSource> source);

    const std::vector<ZipEntry>& entries() const { return entries_; }

    // Decompresses one entry fully into memory and verifies its CRC-32.
    // Throws DecodeError on a CRC mismatch, an unsupported method, or a
    // malformed local header.
    std::vector<std::byte> read_entry(const ZipEntry& entry) const;

    // Convenience for the nested-ZIP case (PLAN.md: "iterate inner entries
    // without extracting to disk"): decompresses the entry and wraps it as
    // a MemoryByteSource, ready to hand to another ZipReader or TiffReader.
    std::shared_ptr<MemoryByteSource> read_entry_as_source(const ZipEntry& entry) const;

private:
    std::shared_ptr<IByteSource> source_;
    std::vector<ZipEntry> entries_;
};

} // namespace g2m::decode
