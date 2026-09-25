// src/decode/tiff/lzw.h - INTERNAL: TIFF-flavoured LZW decompression
// (TIFF6 section 13). MSB-first bit packing, "early change" code-width
// growth (widens one code early, at table size 2^n - 1 instead of 2^n),
// 9-12 bit codes, code 256 = ClearCode, 257 = EndOfInformation.
//
// This mirrors, bit-for-bit, the coordinator's independent pure-Python
// reference decoder (verified against real Hessen DGM1 tiles before this
// was written) rather than a generic textbook LZW - the early-change
// threshold in particular has more than one "obviously right" reading of
// the TIFF6 spec text, and this is the one that is known to match the real
// files.
#pragma once

#include "g2m/decode/byte_source.h"

#include <cstddef>
#include <span>
#include <vector>

namespace g2m::decode::tiff {

// Decodes one LZW-compressed strip/tile. Throws DecodeError on a malformed
// bitstream (an EOI is not required - decoding also stops cleanly once
// `expected_output_size` bytes have been produced, matching libtiff's own
// tolerance for a truncated final EOI in some encoders).
std::vector<std::byte> lzw_decode(std::span<const std::byte> compressed, std::size_t expected_output_size);

} // namespace g2m::decode::tiff
