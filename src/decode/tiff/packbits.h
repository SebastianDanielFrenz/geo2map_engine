// src/decode/tiff/packbits.h - INTERNAL: PackBits decompression (TIFF6
// section 9 / Apple TN1023).
#pragma once

#include "g2m/decode/byte_source.h"

#include <cstddef>
#include <span>
#include <vector>

namespace g2m::decode::tiff {

// Decodes one PackBits-compressed strip/tile. Control byte n (signed
// int8): 0..127 -> copy the next n+1 literal bytes; -1..-127 -> repeat the
// next single byte (1-n) times; -128 is a no-op. Throws DecodeError if the
// stream runs out of input mid-run or does not exactly fill
// expected_output_size.
std::vector<std::byte> packbits_decode(std::span<const std::byte> compressed, std::size_t expected_output_size);

} // namespace g2m::decode::tiff
