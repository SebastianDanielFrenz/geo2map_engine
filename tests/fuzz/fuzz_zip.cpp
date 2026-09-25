// tests/fuzz/fuzz_zip.cpp - libFuzzer entry point for ZipReader (see
// tests/fuzz/CMakeLists.txt: only built under G2M_BUILD_FUZZERS, clang++
// only).
#include "g2m/decode/byte_source.h"
#include "g2m/decode/zip/zip_reader.h"

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
        g2m::decode::ZipReader reader(src);
        for (const auto& entry : reader.entries()) {
            // Bound per-entry decompressed size so a crafted "zip bomb"
            // doesn't turn one fuzz iteration into an unbounded allocation.
            if (entry.uncompressed_size > (16u << 20)) continue;
            try {
                (void)reader.read_entry(entry);
            } catch (const g2m::decode::DecodeError&) {
                // expected for malformed/corrupt entries
            }
        }
    } catch (const g2m::decode::DecodeError&) {
        // expected for malformed input
    }
    return 0;
}
