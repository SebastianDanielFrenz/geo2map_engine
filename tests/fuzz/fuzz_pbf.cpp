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

namespace {
std::string scratch_path() {
    static const std::filesystem::path dir = std::filesystem::temp_directory_path();
    return (dir / "g2m_fuzz_pbf_input.osm.pbf").string();
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
