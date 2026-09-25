// tests/fuzz/fuzz_pbf.cpp - libFuzzer entry point for read_pbf (see
// tests/fuzz/CMakeLists.txt: only built under G2M_BUILD_FUZZERS, clang++
// only). libosmium's Reader only reads from a path (or file descriptor),
// not an in-memory buffer, so each iteration writes the fuzz input to a
// scratch file - accepted cost for a libFuzzer target that is never run on
// the Windows build this milestone's own verification uses.
#include "g2m/decode/byte_source.h"
#include "g2m/decode/osm/pbf_reader.h"

#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <string>

// G2M_BUILD_FUZZERS is clang-only (WSL legs), never the Windows presets, so
// a POSIX getpid() is always available here.
#include <unistd.h>

namespace {
std::string scratch_path() {
    // Unique per process: tools/ci.ps1's linux-release and linux-asan-ubsan
    // fuzz legs can run concurrently in the same WSL instance (G1c step 0
    // item 4c), and a single fixed filename let one leg's scratch file be
    // truncated/rewritten mid-read by the other leg's fuzzer process.
    static const std::filesystem::path dir = std::filesystem::temp_directory_path();
    static const std::string path =
        (dir / ("g2m_fuzz_pbf_input_" + std::to_string(::getpid()) + ".osm.pbf")).string();
    return path;
}
} // namespace

extern "C" int LLVMFuzzerTestOneInput(const std::uint8_t* data, std::size_t size) {
    const std::string path = scratch_path();
    {
        std::ofstream out(path, std::ios::binary | std::ios::trunc);
        if (size > 0) out.write(reinterpret_cast<const char*>(data), static_cast<std::streamsize>(size));
    }
    class NullHandler : public g2m::decode::osm::PbfHandler {};
    NullHandler handler;
    try {
        (void)g2m::decode::osm::read_pbf(path, handler);
    } catch (const g2m::decode::DecodeError&) {
        // expected for malformed input
    } catch (const std::exception&) {
        // libosmium/protozero may throw their own exception types directly
        // for some malformed inputs; still an expected outcome for a fuzz
        // target, not a crash.
    }
    std::remove(path.c_str());
    return 0;
}
