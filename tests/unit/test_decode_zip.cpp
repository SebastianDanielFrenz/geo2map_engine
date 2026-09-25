// tests/unit/test_decode_zip.cpp - ZipReader: synthetic in-test fixtures
// (stored, DEFLATE, ZIP64-flagged sizes, nested ZIP, CRC rejection).
//
// ZipBuilder itself now lives in tests/support/zip_builder.h (promoted
// unchanged, PLAN.md G1c I2, for tests/support/fixture_cut's future
// cut-worldcover/repack-dgm1 subcommands to reuse) - only this file's own
// test-case-local helpers (to_bytes) stay here.

#include "g2m/decode/byte_source.h"
#include "g2m/decode/zip/zip_reader.h"
#include "zip_builder.h"

#include <catch2/catch_test_macros.hpp>

#include <cstring>
#include <memory>
#include <string>
#include <vector>

using namespace g2m::decode;
using g2m::test_support::ZipBuilder;

namespace {

std::vector<std::byte> to_bytes(const std::string& s) {
    std::vector<std::byte> v(s.size());
    std::memcpy(v.data(), s.data(), s.size());
    return v;
}

} // namespace

TEST_CASE("zip: stored entry round-trips", "[decode][zip]") {
    ZipBuilder b;
    b.add(ZipBuilder::Item{"hello.txt", to_bytes("hello, DGM1 world"), false, false});
    auto bytes = b.build();
    auto src = std::make_shared<MemoryByteSource>(std::move(bytes));
    ZipReader reader(src);
    REQUIRE(reader.entries().size() == 1);
    CHECK(reader.entries()[0].name == "hello.txt");
    CHECK(reader.entries()[0].method == ZipMethod::Stored);
    auto content = reader.read_entry(reader.entries()[0]);
    CHECK(std::string(reinterpret_cast<const char*>(content.data()), content.size()) == "hello, DGM1 world");
}

TEST_CASE("zip: deflate entry round-trips", "[decode][zip]") {
    std::string payload;
    for (int i = 0; i < 500; ++i) payload += "the quick brown fox jumps over the lazy dog ";
    ZipBuilder b;
    b.add(ZipBuilder::Item{"deflate.bin", to_bytes(payload), true, false});
    auto src = std::make_shared<MemoryByteSource>(b.build());
    ZipReader reader(src);
    REQUIRE(reader.entries().size() == 1);
    CHECK(reader.entries()[0].method == ZipMethod::Deflate);
    auto content = reader.read_entry(reader.entries()[0]);
    CHECK(std::string(reinterpret_cast<const char*>(content.data()), content.size()) == payload);
}

TEST_CASE("zip: forced ZIP64 fields parse correctly", "[decode][zip]") {
    ZipBuilder b;
    b.add(ZipBuilder::Item{"z64.txt", to_bytes("zip64 path, tiny payload"), false, true});
    auto src = std::make_shared<MemoryByteSource>(b.build());
    ZipReader reader(src);
    REQUIRE(reader.entries().size() == 1);
    CHECK(reader.entries()[0].uncompressed_size == std::string("zip64 path, tiny payload").size());
    auto content = reader.read_entry(reader.entries()[0]);
    CHECK(std::string(reinterpret_cast<const char*>(content.data()), content.size()) == "zip64 path, tiny payload");
}

TEST_CASE("zip: nested zip (zip-in-zip, PLAN.md's DGM1 delivery shape)", "[decode][zip]") {
    ZipBuilder inner;
    inner.add(ZipBuilder::Item{"dgm1_436_5560.tif", to_bytes("pretend tiff bytes"), false, false});
    inner.add(ZipBuilder::Item{"dgm1_436_5560.tfw", to_bytes("1.0\n0.0\n0.0\n-1.0\n436000.5\n5560999.5\n"), true, false});
    auto inner_bytes = inner.build();

    ZipBuilder outer;
    outer.add(ZipBuilder::Item{"436_5560.zip", inner_bytes, false, false});
    auto src = std::make_shared<MemoryByteSource>(outer.build());

    ZipReader outer_reader(src);
    REQUIRE(outer_reader.entries().size() == 1);
    auto inner_source = outer_reader.read_entry_as_source(outer_reader.entries()[0]);
    ZipReader inner_reader(inner_source);
    REQUIRE(inner_reader.entries().size() == 2);
    CHECK(inner_reader.entries()[0].name == "dgm1_436_5560.tif");
    CHECK(inner_reader.entries()[1].name == "dgm1_436_5560.tfw");
    auto tfw_bytes = inner_reader.read_entry(inner_reader.entries()[1]);
    CHECK(std::string(reinterpret_cast<const char*>(tfw_bytes.data()), tfw_bytes.size()) ==
          "1.0\n0.0\n0.0\n-1.0\n436000.5\n5560999.5\n");
}

TEST_CASE("zip: corrupted entry bytes fail CRC-32 verification", "[decode][zip]") {
    ZipBuilder b;
    b.add(ZipBuilder::Item{"a.bin", to_bytes("some content to corrupt"), false, false});
    auto bytes = b.build();
    // Flip a byte inside the stored payload (after the local header + name).
    const std::string needle = "some content";
    for (std::size_t i = 0; i + needle.size() <= bytes.size(); ++i) {
        if (std::memcmp(&bytes[i], needle.data(), needle.size()) == 0) {
            bytes[i] = static_cast<std::byte>(static_cast<unsigned char>(bytes[i]) ^ 0xFF);
            break;
        }
    }
    auto src = std::make_shared<MemoryByteSource>(std::move(bytes));
    ZipReader reader(src);
    REQUIRE(reader.entries().size() == 1);
    CHECK_THROWS_AS(reader.read_entry(reader.entries()[0]), DecodeError);
}

TEST_CASE("zip: raw filename bytes are not assumed UTF-8", "[decode][zip]") {
    // A Latin-1 "ä" (0xE4) is not valid UTF-8 on its own.
    std::string raw_name = "stra";
    raw_name.push_back(static_cast<char>(0xE4));
    raw_name += "e.txt";
    ZipBuilder b;
    b.add(ZipBuilder::Item{raw_name, to_bytes("content"), false, false});
    auto src = std::make_shared<MemoryByteSource>(b.build());
    ZipReader reader(src);
    REQUIRE(reader.entries().size() == 1);
    CHECK(reader.entries()[0].name == raw_name); // raw bytes preserved exactly
    const std::string display = to_lossy_display_name(raw_name);
    CHECK(display != raw_name); // the invalid byte was replaced
    CHECK(display.find("\xEF\xBF\xBD") != std::string::npos); // U+FFFD present
}

TEST_CASE("zip: unsupported compression method is rejected with a clear error", "[decode][zip]") {
    // Hand-build a central directory entry claiming method 99 (invalid),
    // reusing ZipBuilder's stored-entry layout as a base then patching the
    // method field in both the local and central headers.
    ZipBuilder b;
    b.add(ZipBuilder::Item{"weird.bin", to_bytes("x"), false, false});
    auto bytes = b.build();
    // Local header method field is at offset 8 (2 bytes); central directory
    // method field is 46 bytes further into the local file header's own
    // layout duplicated in the central dir - patch both occurrences of the
    // 2-byte LE value 0x0000 at the known local-header offset 8 and search
    // for the matching central-directory signature to patch its method too.
    bytes[8] = static_cast<std::byte>(99);
    for (std::size_t i = 0; i + 4 <= bytes.size(); ++i) {
        if (bytes[i] == static_cast<std::byte>(0x50) && bytes[i + 1] == static_cast<std::byte>(0x4b) &&
            bytes[i + 2] == static_cast<std::byte>(0x01) && bytes[i + 3] == static_cast<std::byte>(0x02)) {
            bytes[i + 10] = static_cast<std::byte>(99);
            break;
        }
    }
    auto src = std::make_shared<MemoryByteSource>(std::move(bytes));
    CHECK_THROWS_AS(ZipReader(src), DecodeError);
}
