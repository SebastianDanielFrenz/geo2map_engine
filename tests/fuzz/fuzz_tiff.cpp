// tests/fuzz/fuzz_tiff.cpp - libFuzzer entry point for TiffReader (see
// tests/fuzz/CMakeLists.txt: only built under G2M_BUILD_FUZZERS, clang++
// only).
#include "g2m/decode/byte_source.h"
#include "g2m/decode/tiff/tiff_reader.h"

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <memory>
#include <vector>

extern "C" int LLVMFuzzerTestOneInput(const std::uint8_t* data, std::size_t size) {
    std::vector<std::byte> bytes(size);
    if (size > 0) std::memcpy(bytes.data(), data, size);
    auto src = std::make_shared<g2m::decode::MemoryByteSource>(std::move(bytes));
    try {
        g2m::decode::tiff::TiffReader reader(src);
        for (std::size_t i = 0; i < reader.ifds().size(); ++i) {
            const auto& ifd = reader.ifds()[i];
            // Bound the decode window so a huge claimed width/height doesn't
            // turn one fuzz iteration into an unbounded allocation.
            const int w = static_cast<int>(std::min<std::uint64_t>(ifd.width, 64));
            const int h = static_cast<int>(std::min<std::uint64_t>(ifd.height, 64));
            if (w <= 0 || h <= 0) continue;
            try {
                switch (ifd.sample_format) {
                    case g2m::decode::tiff::SampleFormat::Float:
                        if (ifd.bits_per_sample == 32) (void)reader.decode_window_f32(i, 0, 0, w, h);
                        break;
                    case g2m::decode::tiff::SampleFormat::UnsignedInt:
                        if (ifd.bits_per_sample == 8) (void)reader.decode_window_u8(i, 0, 0, w, h);
                        else if (ifd.bits_per_sample == 16) (void)reader.decode_window_u16(i, 0, 0, w, h);
                        break;
                    case g2m::decode::tiff::SampleFormat::SignedInt:
                        if (ifd.bits_per_sample == 16) (void)reader.decode_window_i16(i, 0, 0, w, h);
                        else if (ifd.bits_per_sample == 32) (void)reader.decode_window_i32(i, 0, 0, w, h);
                        break;
                }
            } catch (const g2m::decode::DecodeError&) {
                // expected for malformed/unsupported combinations
            }
        }
    } catch (const g2m::decode::DecodeError&) {
        // expected for malformed input
    }
    return 0;
}
