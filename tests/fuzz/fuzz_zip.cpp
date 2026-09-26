// tests/fuzz/fuzz_zip.cpp - libFuzzer entry point for ZipReader (see
// tests/fuzz/CMakeLists.txt: only built under G2M_BUILD_FUZZERS, clang++
// only).
//
// PLAN.md G1c I6 item B: also exercises entry_source() (the zero-copy nested-
// ZIP path a real DGM1 delivery uses, dgm1.h's Catalog) on every entry, and
// recurses one level into any entry whose name ends ".zip" - matching the
// real outer-ZIP/inner-ZIP nesting depth this project's own fixtures use
// (testdata/fixtures/dgm1/PKT_fixture.zip), never deeper.
#include "g2m/decode/byte_source.h"
#include "g2m/decode/zip/zip_reader.h"

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <memory>
#include <string>
#include <string_view>
#include <vector>

namespace {

constexpr std::uint64_t kMaxInflateBytes = 16u << 20; // 16 MiB
constexpr int kMaxDepth = 2;                          // outer + one nested level

bool has_zip_suffix(const std::string& name) {
    constexpr std::string_view kSuffix = ".zip";
    return name.size() >= kSuffix.size() &&
           name.compare(name.size() - kSuffix.size(), kSuffix.size(), kSuffix) == 0;
}

void fuzz_entries(const std::shared_ptr<g2m::decode::IByteSource>& source, int depth) {
    try {
        g2m::decode::ZipReader reader(source);
        for (const auto& entry : reader.entries()) {
            // Bound per-entry decompressed size so a crafted "zip bomb"
            // doesn't turn one fuzz iteration into an unbounded allocation.
            if (entry.uncompressed_size > kMaxInflateBytes) continue;
            try {
                (void)reader.read_entry(entry);
            } catch (const g2m::decode::DecodeError&) {
                // expected for malformed/corrupt entries
            }

            std::shared_ptr<g2m::decode::IByteSource> nested;
            try {
                auto result = reader.entry_source(entry, kMaxInflateBytes);
                if (result) nested = *result;
                // !result: caller-chosen cap exceeded (LimitExceeded) - not
                // a crash, nothing more to do for this entry.
            } catch (const g2m::decode::DecodeError&) {
                // expected for malformed/corrupt entries
                continue;
            }
            if (nested && depth < kMaxDepth && has_zip_suffix(entry.name)) {
                fuzz_entries(nested, depth + 1);
            }
        }
    } catch (const g2m::decode::DecodeError&) {
        // expected for malformed input
    }
}

} // namespace

extern "C" int LLVMFuzzerTestOneInput(const std::uint8_t* data, std::size_t size) {
    std::vector<std::byte> bytes(size);
    if (size > 0) std::memcpy(bytes.data(), data, size);
    auto src = std::make_shared<g2m::decode::MemoryByteSource>(std::move(bytes));
    fuzz_entries(src, 1);
    return 0;
}
