// src/layer/zstd_codec.h - INTERNAL: the one place g2m calls zstd.
//
// Compression is a pure function of (zstd version, parameters, input): every
// parameter that could otherwise be chosen at run time (by CPU features,
// thread count or defaults that may change) is set explicitly in
// zstd_codec.cpp, and compression is always single-threaded. See
// docs/formats/tile_container.md, "Compression".
#pragma once

#include "g2m/core/result.h"

#include <cstddef>
#include <cstdint>
#include <span>
#include <vector>

namespace g2m::layer_detail {

// The pinned compression level (docs/formats/tile_container.md).
inline constexpr int kZstdLevel = 12;
// Decoder window limit: 2^27 = 128 MiB, never exceeded by our encoder
// (sections are at most kMaxSectionRawSize).
inline constexpr int kZstdWindowLogMax = 27;

// One zstd frame: content size recorded, no checksum (the container's
// SHA-256 covers the bytes), no dictionary id.
std::vector<std::uint8_t> zstd_compress(std::span<const std::uint8_t> raw, int level = kZstdLevel);

// Accepts exactly one complete frame whose declared content size equals
// expected_raw_size and whose decoded size matches it; nothing may follow
// the frame. Fails by value on anything else.
Result<std::vector<std::uint8_t>> zstd_decompress(std::span<const std::uint8_t> frame, std::size_t expected_raw_size);

// "1.5.7" - for the release manifest and the golden file header.
const char* zstd_version();

} // namespace g2m::layer_detail
