// tests/unit/test_decode_osm.cpp - minimal PbfReader unit coverage: error
// path and BoundingBox logic. Full parse correctness against a real PBF
// (dictionary-bearing, multi-block) is covered by the gated real-data smoke
// test (G2M_DATA_DIR / hessen-latest.osm.pbf) rather than a hand-built
// synthetic protobuf fixture here.

#include "g2m/decode/osm/pbf_reader.h"

#include <catch2/catch_test_macros.hpp>

#include <chrono>
#include <filesystem>
#include <fstream>
#include <string>
#include <thread>

using namespace g2m::decode;
using namespace g2m::decode::osm;

TEST_CASE("osm: reading a nonexistent path throws DecodeError", "[decode][osm]") {
    class NullHandler : public PbfHandler {};
    NullHandler handler;
    CHECK_THROWS_AS(read_pbf("Z:/definitely/does/not/exist.osm.pbf", handler), DecodeError);
}

TEST_CASE("osm: reading a garbage .osm.pbf file throws DecodeError, not a crash", "[decode][osm]") {
    // A real extension (so libosmium's format detection picks the PBF
    // reader) with content that is not a valid PBF blob stream at all.
    const std::string path = (std::filesystem::temp_directory_path() / "g2m_test_garbage.osm.pbf").string();
    {
        std::ofstream out(path, std::ios::binary | std::ios::trunc);
        out << "this is not a protobuf-framed OSM PBF file, just plain garbage bytes";
    }
    class NullHandler : public PbfHandler {};
    NullHandler handler;
    CHECK_THROWS_AS(read_pbf(path, handler), DecodeError);

    // libosmium's osmium::io::Reader tears itself down via an internal
    // decompression/prefetch thread pool that has not necessarily joined
    // (and released its file handle) by the moment read_pbf's exception
    // propagates out of its own stack-unwind - so an IMMEDIATE delete can
    // still race Windows' exclusive-handle rules even though the Reader
    // object itself has already gone out of scope. Retry briefly rather
    // than treat this scratch-file cleanup race as a test failure.
    for (int attempt = 0; attempt < 20; ++attempt) {
        std::error_code ec;
        if (std::filesystem::remove(path, ec) || ec == std::errc::no_such_file_or_directory) break;
        std::this_thread::sleep_for(std::chrono::milliseconds(25));
    }
}

TEST_CASE("osm: fuzz_pbf regression - crafted BlobHeader.datasize exceeding remaining file bytes throws DecodeError, not UBSan null-memcpy",
          "[decode][osm]") {
    // 16-byte crafted PBF from the G1c step 0 fuzz_pbf UBSan crash
    // (linux-asan-ubsan leg): BlobHeader.datasize (field 3) decodes to
    // 15015, but only 4 bytes remain in the whole 16-byte file. Before
    // validate_pbf_framing() existed, this reached libosmium's
    // pbf_input_format.hpp with a short/empty read and it built a Blob
    // buffer around it, landing on a memcpy with a null source pointer and
    // a non-zero length ("null pointer passed as argument 2").
    const std::string path = (std::filesystem::temp_directory_path() / "g2m_test_fuzz_pbf_regression.osm.pbf").string();
    {
        const unsigned char bytes[] = {0x00, 0x00, 0x00, 0x08, 0xf8, 0x20, 0xf7, 0x04,
                                        0x18, 0xa7, 0xf5, 0x00, 0x00, 0x08, 0x00, 0x00};
        std::ofstream out(path, std::ios::binary | std::ios::trunc);
        out.write(reinterpret_cast<const char*>(bytes), sizeof(bytes));
    }
    class NullHandler : public PbfHandler {};
    NullHandler handler;
    CHECK_THROWS_AS(read_pbf(path, handler), DecodeError);

    for (int attempt = 0; attempt < 20; ++attempt) {
        std::error_code ec;
        if (std::filesystem::remove(path, ec) || ec == std::errc::no_such_file_or_directory) break;
        std::this_thread::sleep_for(std::chrono::milliseconds(25));
    }
}

TEST_CASE("osm: fuzz_pbf regression - BlobHeader.datasize field with a non-varint wire type throws DecodeError, not abort",
          "[decode][osm]") {
    // 13-byte crafted PBF found a few minutes into the G1c step 0 item 5
    // fuzzing pass, ON validate_pbf_framing() itself (added for the
    // previous regression above): field 3 (BlobHeader.datasize, expected
    // varint) is encoded with a different wire type. protozero's
    // pbf_reader::get_int32() ASSERTS has_wire_type(varint) internally
    // rather than throwing, so calling it unconditionally reached a raw
    // assert()/abort() in a Debug (NDEBUG-less) build - not a
    // g2m::decode::DecodeError, escaping uncaught exactly like the framing
    // bugs this validator exists to reject.
    const std::string path = (std::filesystem::temp_directory_path() / "g2m_test_fuzz_pbf_wiretype.osm.pbf").string();
    {
        const unsigned char bytes[] = {0x00, 0x00, 0x00, 0x08, 0xf8, 0x27, 0xff, 0x18,
                                        0x19, 0xa7, 0xda, 0x12, 0x49};
        std::ofstream out(path, std::ios::binary | std::ios::trunc);
        out.write(reinterpret_cast<const char*>(bytes), sizeof(bytes));
    }
    class NullHandler : public PbfHandler {};
    NullHandler handler;
    CHECK_THROWS_AS(read_pbf(path, handler), DecodeError);

    for (int attempt = 0; attempt < 20; ++attempt) {
        std::error_code ec;
        if (std::filesystem::remove(path, ec) || ec == std::errc::no_such_file_or_directory) break;
        std::this_thread::sleep_for(std::chrono::milliseconds(25));
    }
}

TEST_CASE("osm: fuzz_pbf regression - BlobHeader missing the required type field throws DecodeError, not abort",
          "[decode][osm]") {
    // 39-byte crafted PBF found during the SAME G1c step 0 item 5 fuzzing
    // pass, run AGAIN after the two fixes above landed: a well-framed
    // BlobHeader (blob_header_len=11, correctly bounded) whose 11 bytes
    // repeat field 3 (datasize=24, matching the 24 bytes actually left in
    // the file) four times and never carry field 1 (BlobHeader.type) at
    // all. validate_pbf_framing() accepted this (datasize was valid and in
    // range), so it reached libosmium's own decode_blob_header()
    // (pbf_input_format.hpp), whose blob_header_type stays a
    // default-constructed (nullptr-data) protozero::data_view when field 1
    // never appears, then is unconditionally std::strncmp()'d against -
    // glibc's strncmp marks its second argument nonnull regardless of the
    // length passed, so this reached a real UBSan "null pointer passed as
    // argument 2" report (pbf_input_format.hpp:171) under linux-asan-ubsan,
    // not a g2m::decode::DecodeError.
    const std::string path = (std::filesystem::temp_directory_path() / "g2m_test_fuzz_pbf_missing_type.osm.pbf").string();
    {
        const unsigned char bytes[] = {0x00, 0x00, 0x00, 0x0b, 0xe0, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18,
                                        0x18, 0x18, 0x18, 0x18, 0x1a, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18,
                                        0x18, 0x18, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                                        0x18, 0x62, 0x62, 0x18, 0x25, 0x00};
        std::ofstream out(path, std::ios::binary | std::ios::trunc);
        out.write(reinterpret_cast<const char*>(bytes), sizeof(bytes));
    }
    class NullHandler : public PbfHandler {};
    NullHandler handler;
    CHECK_THROWS_AS(read_pbf(path, handler), DecodeError);

    for (int attempt = 0; attempt < 20; ++attempt) {
        std::error_code ec;
        if (std::filesystem::remove(path, ec) || ec == std::errc::no_such_file_or_directory) break;
        std::this_thread::sleep_for(std::chrono::milliseconds(25));
    }
}

TEST_CASE("osm: fuzz_pbf regression - full CI crash artifact (fuzz_pbf-crash-38f97870...) throws DecodeError, not UBSan",
          "[decode][osm]") {
    // 52-byte artifact from the linux-asan-ubsan leg's full tools/ci.ps1 run
    // that motivated this whole G1c step 0 fix set (also saved under
    // ~/build/geo2map_engine/linux-asan-ubsan/tests/fuzz/corpus/fuzz_pbf-crash-38f97870...).
    // Base64: AAAAGBgYGBgpKSkpKSkpKSkpKSkpKRgYGBgYGA4YGBgYBCIBAGJaWlpaWjd6WlpaWlpaWg==
    // Same root cause as the "missing required type field" case above (also
    // pbf_input_format.hpp:171's std::strncmp(expected_type,
    // blob_header_type.data(), ...) with a null blob_header_type.data()):
    // a well-framed 24-byte BlobHeader (length prefix 0x18=24) whose fields
    // are three repeats of field 3 (BlobHeader.datasize, varint, decoding to
    // 24 each time - consistent with the 24 bytes actually remaining) and
    // two repeats of an unrelated field 5 (fixed64, skipped) - field 1
    // (BlobHeader.type) never appears. Kept as its own regression case,
    // distinct from the hand-minimised 39-byte one above, because it is the
    // literal artifact tools/ci.ps1 flagged and libFuzzer's own crash
    // minimisation does not always converge on the same byte pattern twice.
    const std::string path = (std::filesystem::temp_directory_path() / "g2m_test_fuzz_pbf_ci_crash.osm.pbf").string();
    {
        const unsigned char bytes[] = {0x00, 0x00, 0x00, 0x18, 0x18, 0x18, 0x18, 0x18, 0x29, 0x29, 0x29, 0x29,
                                        0x29, 0x29, 0x29, 0x29, 0x29, 0x29, 0x29, 0x29, 0x29, 0x29, 0x18, 0x18,
                                        0x18, 0x18, 0x18, 0x18, 0x0e, 0x18, 0x18, 0x18, 0x18, 0x04, 0x22, 0x01,
                                        0x00, 0x62, 0x5a, 0x5a, 0x5a, 0x5a, 0x5a, 0x37, 0x7a, 0x5a, 0x5a, 0x5a,
                                        0x5a, 0x5a, 0x5a, 0x5a};
        std::ofstream out(path, std::ios::binary | std::ios::trunc);
        out.write(reinterpret_cast<const char*>(bytes), sizeof(bytes));
    }
    class NullHandler : public PbfHandler {};
    NullHandler handler;
    CHECK_THROWS_AS(read_pbf(path, handler), DecodeError);

    for (int attempt = 0; attempt < 20; ++attempt) {
        std::error_code ec;
        if (std::filesystem::remove(path, ec) || ec == std::errc::no_such_file_or_directory) break;
        std::this_thread::sleep_for(std::chrono::milliseconds(25));
    }
}

#if defined(__linux__)
#include <cstdlib>
#include <dirent.h>

namespace {
// Counts this process's currently-open file descriptors via /proc/self/fd -
// Linux only (matches the item 4a requirement: run the failing-parse loop
// everywhere, but only assert fd-count non-growth where /proc exists).
std::size_t count_open_fds() {
    DIR* dir = ::opendir("/proc/self/fd");
    if (dir == nullptr) return 0;
    std::size_t count = 0;
    while (::readdir(dir) != nullptr) ++count;
    ::closedir(dir);
    return count;
}
} // namespace
#endif

TEST_CASE("osm: repeated failing parses do not leak file descriptors/handles", "[decode][osm]") {
    // G1c step 0 item 4a: fuzz_pbf hit "Too many open files" after ~3s of
    // continuous failing parses (a leaked fd/handle per failed
    // osmium::io::Reader construction/read whose reader.close() was never
    // reached on the exception path). Run the loop on every platform (so a
    // regression here is at least exercised everywhere), but only assert
    // the fd count on Linux, where /proc/self/fd gives a cheap, reliable
    // count; there is no equivalent zero-cost handle-count API exercised
    // here on Windows.
    const std::string path = (std::filesystem::temp_directory_path() / "g2m_test_fd_leak.osm.pbf").string();
    {
        std::ofstream out(path, std::ios::binary | std::ios::trunc);
        out << "this is not a protobuf-framed OSM PBF file, just plain garbage bytes";
    }

    class NullHandler : public PbfHandler {};
    NullHandler handler;

#if defined(__linux__)
    // Warm up (first few iterations may open additional fds, e.g. for lazy
    // library initialization) before taking the baseline.
    for (int i = 0; i < 10; ++i) {
        CHECK_THROWS_AS(read_pbf(path, handler), DecodeError);
    }
    const std::size_t baseline = count_open_fds();
#endif

    constexpr int kIterations = 2000;
    for (int i = 0; i < kIterations; ++i) {
        CHECK_THROWS_AS(read_pbf(path, handler), DecodeError);
    }

#if defined(__linux__)
    const std::size_t after = count_open_fds();
    // Allow a small constant slack (e.g. a transient fd during teardown of
    // the very last iteration) but reject any growth proportional to
    // kIterations, which is what a per-call leak would produce.
    CHECK(after <= baseline + 8);
#endif

    std::error_code ec;
    std::filesystem::remove(path, ec);
}

TEST_CASE("osm: BoundingBox::contains", "[decode][osm]") {
    BoundingBox bbox{49.5, 50.5, 8.0, 9.0};
    CHECK(bbox.contains(50.10, 8.55));
    CHECK(bbox.contains(49.5, 8.0));  // inclusive lower corner
    CHECK(bbox.contains(50.5, 9.0));  // inclusive upper corner
    CHECK_FALSE(bbox.contains(49.49, 8.55));
    CHECK_FALSE(bbox.contains(50.10, 9.01));
    CHECK_FALSE(bbox.contains(-10.0, 8.55));
}

TEST_CASE("osm: default BoundingBox contains everything", "[decode][osm]") {
    BoundingBox bbox;
    CHECK(bbox.contains(89.9, 179.9));
    CHECK(bbox.contains(-89.9, -179.9));
    CHECK(bbox.contains(0.0, 0.0));
}
