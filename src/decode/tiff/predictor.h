// src/decode/tiff/predictor.h - INTERNAL: TIFF predictors 2 (horizontal
// differencing) and 3 (floating point), decode direction (TIFF6 Technical
// Note 3 / Note 2).
#pragma once

#include <cstddef>
#include <cstdint>
#include <span>

namespace g2m::decode::tiff {

// Undoes horizontal differencing (Predictor 2) on one decompressed strip
// row, in place. The differencing word size matches bits_per_sample
// (TIFF6 13.3): 8-bit -> byte-wise mod-256 cumulative sum, 16-bit ->
// uint16 mod-65536, 32-bit -> uint32 mod-2^32 (this is why DGM1's
// float32-with-predictor-2 tiles are differenced as uint32 WORDS over the
// float bit pattern, not as bytes - PLAN.md/G2M-002). `width` is samples
// per row (SamplesPerPixel == 1 is this reader's whole scope, so no
// per-component stride is needed). `row` must be exactly
// width * (bits_per_sample / 8) bytes.
void undo_predictor2(std::span<std::byte> row, int width, unsigned bits_per_sample);

// Undoes the floating-point predictor (Predictor 3, TIFF Technical Note 3),
// in place: (1) byte-wise mod-256 cumulative sum over the whole row, then
// (2) de-interleave from "byte planes" (most-significant byte of every
// sample first) back into individual big-endian sample byte sequences.
// `row` must be exactly width * bytes_per_sample bytes; on return, sample c
// occupies row[c*bytes_per_sample .. (c+1)*bytes_per_sample) as a
// BIG-ENDIAN value of that width (per spec, independent of the file's own
// byte order - the caller byte-swaps to host order itself if needed).
void undo_predictor3(std::span<std::byte> row, int width, unsigned bytes_per_sample);

} // namespace g2m::decode::tiff
