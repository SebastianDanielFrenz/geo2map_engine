// tests/support/tiff_builder.h - a classic little-endian TIFF writer
// covering exactly what TiffReader needs to parse (IFD entries, external
// value blocks, strip data), plus a TIFF-flavoured LZW literal-only
// encoder.
//
// Promoted UNCHANGED from tests/unit/test_decode_tiff.cpp (PLAN.md G1c I2,
// tests/support/fixture_cut) so a test-only tool needing a synthetic TIFF
// fixture (fixture_cut's future cut-worldcover subcommand, PLAN.md G1c I3+)
// can reuse it instead of re-deriving the format by hand; test_decode_tiff.cpp
// itself now includes this header instead of defining TiffBuilder/
// lzw_encode_literal_only locally.
#pragma once

#include <cstdint>
#include <cstring>
#include <string>
#include <vector>

namespace g2m::test_support {

// --- little helpers -------------------------------------------------------

inline std::vector<std::byte> tiff_le_u16(std::uint16_t v) {
    std::vector<std::byte> b(2);
    std::memcpy(b.data(), &v, 2);
    return b;
}
inline std::vector<std::byte> tiff_le_u32(std::uint32_t v) {
    std::vector<std::byte> b(4);
    std::memcpy(b.data(), &v, 4);
    return b;
}
inline std::vector<std::byte> tiff_le_f64_array(const std::vector<double>& vals) {
    std::vector<std::byte> b(vals.size() * 8);
    std::memcpy(b.data(), vals.data(), b.size());
    return b;
}
inline std::vector<std::byte> tiff_le_u16_array(const std::vector<std::uint16_t>& vals) {
    std::vector<std::byte> b(vals.size() * 2);
    std::memcpy(b.data(), vals.data(), b.size());
    return b;
}

// TIFF-flavoured LZW encoder emitting every symbol as a LITERAL code only
// (never a dictionary match) - still exercises MSB-first bit packing and
// the early-change code-width growth rule exactly the way lzw_decode
// expects (a dictionary entry is pushed for every non-first code
// regardless of whether it was itself a literal or a match), just without
// covering the dictionary-hit/KwKwK decode paths.
inline std::vector<std::byte> lzw_encode_literal_only(const std::vector<std::byte>& in) {
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

// --- classic little-endian TIFF builder ------------------------------------

class TiffBuilder {
public:
    void add_short(std::uint16_t tag, std::uint16_t value) { add_inline(tag, 3, 1, tiff_le_u16(value)); }
    void add_long(std::uint16_t tag, std::uint32_t value) { add_inline(tag, 4, 1, tiff_le_u32(value)); }
    void add_short_array(std::uint16_t tag, const std::vector<std::uint16_t>& values) {
        add_external(tag, 3, values.size(), tiff_le_u16_array(values));
    }
    void add_double_array(std::uint16_t tag, const std::vector<double>& values) {
        add_external(tag, 12, values.size(), tiff_le_f64_array(values));
    }
    void add_ascii(std::uint16_t tag, const std::string& s) {
        std::string with_nul = s;
        with_nul.push_back('\0');
        std::vector<std::byte> bytes(with_nul.size());
        std::memcpy(bytes.data(), with_nul.data(), with_nul.size());
        // TIFF6 "value fits in 4 bytes -> stored inline, not at an offset"
        // (tiff_reader.cpp's read_ascii follows this exactly: it reads the
        // raw 4-byte value/offset field itself as the string's bytes
        // whenever count <= that field's size, never dereferencing it as an
        // offset in that case). A short ASCII value - GDAL_NODATA "0" is
        // the one every real WorldCover/DGM1 file in this project actually
        // uses, count 2 incl. NUL - written externally here used to leave
        // the reader interpreting a raw file offset as if it were the
        // string's own bytes instead of following it, corrupting the value
        // silently (found via test_import_landcover.cpp's synthetic
        // all-zero-tile test surfacing "GDAL_NODATA must be \"0\" (got
        // \"\")" - PLAN.md G1c I5).
        if (bytes.size() <= 4) {
            // Compute the count BEFORE the call: passing `bytes.size()` and
            // `std::move(bytes)` as two arguments of the SAME call left the
            // count argument reading a moved-from (empty) vector on this
            // compiler - argument evaluation order is unspecified, and nothing
            // stops the callee's by-value parameter from being move-
            // constructed before a sibling argument expression is evaluated.
            const auto count = static_cast<std::uint32_t>(bytes.size());
            add_inline(tag, 2, count, std::move(bytes));
        } else {
            const auto count = with_nul.size();
            add_external(tag, 2, count, std::move(bytes));
        }
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
        strip_offsets_entry.inline_value = tiff_le_u32(strip_offset);
        Entry strip_bytecounts_entry;
        strip_bytecounts_entry.tag = 279;
        strip_bytecounts_entry.type = 4;
        strip_bytecounts_entry.count = 1;
        strip_bytecounts_entry.inline_value = tiff_le_u32(static_cast<std::uint32_t>(strip_data.size()));
        all.push_back(strip_offsets_entry);
        all.push_back(strip_bytecounts_entry);

        std::vector<std::byte> out;
        out.push_back(std::byte{'I'});
        out.push_back(std::byte{'I'});
        auto app32 = [&](std::vector<std::byte>& v, const std::vector<std::byte>& b) {
            v.insert(v.end(), b.begin(), b.end());
        };
        app32(out, tiff_le_u16(42));
        app32(out, tiff_le_u32(8));
        app32(out, tiff_le_u16(static_cast<std::uint16_t>(all.size())));
        for (std::size_t i = 0; i < all.size(); ++i) {
            const Entry& e = all[i];
            app32(out, tiff_le_u16(e.tag));
            app32(out, tiff_le_u16(e.type));
            app32(out, tiff_le_u32(e.count));
            if (!e.external.empty()) {
                app32(out, tiff_le_u32(external_offsets[i]));
            } else {
                std::vector<std::byte> v = e.inline_value;
                v.resize(4, std::byte{0});
                app32(out, v);
            }
        }
        app32(out, tiff_le_u32(0)); // next IFD
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

} // namespace g2m::test_support

