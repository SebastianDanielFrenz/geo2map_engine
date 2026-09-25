// src/decode/tiff/predictor.cpp - see predictor.h.

#include "predictor.h"

#include "g2m/decode/byte_source.h"

#include <cstring>
#include <string>
#include <vector>

namespace g2m::decode::tiff {

void undo_predictor2(std::span<std::byte> row, int width, unsigned bits_per_sample) {
    if (width <= 0) return;
    switch (bits_per_sample) {
        case 8: {
            auto* p = reinterpret_cast<std::uint8_t*>(row.data());
            for (int i = 1; i < width; ++i) p[i] = static_cast<std::uint8_t>(p[i] + p[i - 1]);
            break;
        }
        case 16: {
            auto* p = reinterpret_cast<std::uint16_t*>(row.data());
            for (int i = 1; i < width; ++i) p[i] = static_cast<std::uint16_t>(p[i] + p[i - 1]);
            break;
        }
        case 32: {
            auto* p = reinterpret_cast<std::uint32_t*>(row.data());
            for (int i = 1; i < width; ++i) p[i] = static_cast<std::uint32_t>(p[i] + p[i - 1]);
            break;
        }
        default:
            throw DecodeError("tiff: predictor 2 unsupported for " + std::to_string(bits_per_sample) +
                               "-bit samples");
    }
}

void undo_predictor3(std::span<std::byte> row, int width, unsigned bytes_per_sample) {
    if (width <= 0) return;
    const std::size_t n = static_cast<std::size_t>(width) * bytes_per_sample;
    // Step 1: byte-wise cumulative sum across the whole row.
    auto* bytes = reinterpret_cast<std::uint8_t*>(row.data());
    for (std::size_t i = 1; i < n; ++i) bytes[i] = static_cast<std::uint8_t>(bytes[i] + bytes[i - 1]);

    // Step 2: de-interleave byte planes (plane 0 = most-significant byte of
    // every sample, stored first) back into big-endian per-sample bytes.
    std::vector<std::uint8_t> tmp(n);
    for (int c = 0; c < width; ++c) {
        for (unsigned b = 0; b < bytes_per_sample; ++b) {
            tmp[static_cast<std::size_t>(c) * bytes_per_sample + b] =
                bytes[static_cast<std::size_t>(b) * static_cast<std::size_t>(width) + static_cast<std::size_t>(c)];
        }
    }
    std::memcpy(bytes, tmp.data(), n);
}

} // namespace g2m::decode::tiff
