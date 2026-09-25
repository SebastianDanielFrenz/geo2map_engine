// src/decode/tiff/lzw.cpp - see lzw.h.

#include "lzw.h"

#include <cstdint>

namespace g2m::decode::tiff {
namespace {

constexpr int kClearCode = 256;
constexpr int kEoiCode = 257;
constexpr int kFirstFreeCode = 258;
constexpr int kMaxCodeBits = 12;

// Reads a big-endian 32-bit word starting at byte offset `byte_off` of
// `data`, treating any byte at or past data.size() as 0 (matches the
// reference's data[...].ljust(4, b'\0') padding - a truncated final code at
// end-of-stream reads as zero bits rather than faulting).
std::uint32_t read_be32_padded(std::span<const std::byte> data, std::size_t byte_off) {
    std::uint32_t v = 0;
    for (int k = 0; k < 4; ++k) {
        const std::size_t idx = byte_off + static_cast<std::size_t>(k);
        const std::uint8_t b = idx < data.size() ? static_cast<std::uint8_t>(data[idx]) : 0;
        v = (v << 8) | b;
    }
    return v;
}

} // namespace

std::vector<std::byte> lzw_decode(std::span<const std::byte> compressed, std::size_t expected_output_size) {
    std::vector<std::byte> out;
    out.reserve(expected_output_size);

    // table[code] for code >= 258 is a dictionary entry; codes 0-255 are
    // literal single bytes and are never stored here (looked up directly).
    std::vector<std::vector<std::uint8_t>> table;
    table.reserve(4096);

    std::size_t bitpos = 0;
    const std::size_t n_bits_total = compressed.size() * 8;
    int nbits = 9;
    bool have_prev = false;
    std::vector<std::uint8_t> prev;

    auto literal = [](int code) -> std::vector<std::uint8_t> { return {static_cast<std::uint8_t>(code)}; };
    auto table_entry_size = [&](int code) -> const std::vector<std::uint8_t>* {
        // code < 256: literal (handled by caller); 256/257 handled by
        // caller; code >= 258 indexes `table` (offset by kFirstFreeCode).
        const std::size_t idx = static_cast<std::size_t>(code - kFirstFreeCode);
        return idx < table.size() ? &table[idx] : nullptr;
    };

    while (bitpos + static_cast<std::size_t>(nbits) <= n_bits_total &&
           (expected_output_size == 0 || out.size() < expected_output_size)) {
        const std::size_t byte = bitpos >> 3;
        const int shift = static_cast<int>(bitpos & 7);
        const std::uint32_t chunk = read_be32_padded(compressed, byte);
        const int code = static_cast<int>((chunk >> (32 - shift - nbits)) & ((1u << nbits) - 1u));
        bitpos += static_cast<std::size_t>(nbits);

        if (code == kClearCode) {
            table.clear();
            nbits = 9;
            have_prev = false;
            continue;
        }
        if (code == kEoiCode) break;

        std::vector<std::uint8_t> entry;
        if (!have_prev) {
            if (code >= 256) throw DecodeError("tiff/lzw: first code after clear is not a literal");
            entry = literal(code);
            out.insert(out.end(), reinterpret_cast<const std::byte*>(entry.data()),
                       reinterpret_cast<const std::byte*>(entry.data()) + entry.size());
            prev = std::move(entry);
            have_prev = true;
            continue;
        }

        if (code < 256) {
            entry = literal(code);
        } else if (code >= kFirstFreeCode) {
            if (const auto* e = table_entry_size(code)) {
                entry = *e;
            } else if (static_cast<std::size_t>(code - kFirstFreeCode) == table.size()) {
                // KwKwK case: the code being requested is exactly the next
                // one about to be added.
                entry = prev;
                entry.push_back(prev.front());
            } else {
                throw DecodeError("tiff/lzw: invalid code");
            }
        } else {
            throw DecodeError("tiff/lzw: invalid code");
        }

        // New dictionary entry: prev + entry's first byte.
        std::vector<std::uint8_t> new_entry = prev;
        new_entry.push_back(entry.front());
        table.push_back(std::move(new_entry));

        out.insert(out.end(), reinterpret_cast<const std::byte*>(entry.data()),
                   reinterpret_cast<const std::byte*>(entry.data()) + entry.size());
        prev = std::move(entry);

        // Early-change code-width growth: mirrors the reference exactly.
        // table.size() here already reflects this iteration's append, so
        // "len(table) + 1" in the Python reference (which counts the two
        // reserved placeholder slots 256/257 that this C++ table never
        // stores) becomes "table.size() + kFirstFreeCode + 1" here.
        if (table.size() + kFirstFreeCode + 1 >= (1u << nbits) && nbits < kMaxCodeBits) {
            ++nbits;
        }
    }

    return out;
}

} // namespace g2m::decode::tiff
