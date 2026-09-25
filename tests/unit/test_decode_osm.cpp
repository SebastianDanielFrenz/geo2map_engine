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
