// src/decode/tiff/packbits.cpp - see packbits.h.

#include "packbits.h"

namespace g2m::decode::tiff {

std::vector<std::byte> packbits_decode(std::span<const std::byte> compressed, std::size_t expected_output_size) {
    std::vector<std::byte> out;
    out.reserve(expected_output_size);
    std::size_t i = 0;
    const std::size_t n = compressed.size();
    while (i < n && out.size() < expected_output_size) {
        const auto ctrl = static_cast<std::int8_t>(compressed[i]);
        ++i;
        if (ctrl >= 0) {
            const std::size_t count = static_cast<std::size_t>(ctrl) + 1;
            if (i + count > n) throw DecodeError("tiff/packbits: literal run past end of input");
            out.insert(out.end(), compressed.begin() + static_cast<std::ptrdiff_t>(i),
                       compressed.begin() + static_cast<std::ptrdiff_t>(i + count));
            i += count;
        } else if (ctrl != -128) {
            if (i >= n) throw DecodeError("tiff/packbits: replicate run past end of input");
            const std::size_t count = static_cast<std::size_t>(1 - static_cast<int>(ctrl));
            out.insert(out.end(), count, compressed[i]);
            ++i;
        }
        // ctrl == -128: no-op byte, skip.
    }
    if (out.size() != expected_output_size) {
        throw DecodeError("tiff/packbits: decoded size mismatch");
    }
    return out;
}

} // namespace g2m::decode::tiff
