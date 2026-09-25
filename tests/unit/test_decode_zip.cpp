// tests/unit/test_decode_zip.cpp - ZipReader: synthetic in-test fixtures
// (stored, DEFLATE, ZIP64-flagged sizes, nested ZIP, CRC rejection).

#include "g2m/decode/byte_source.h"
#include "g2m/decode/zip/zip_reader.h"

#include <libdeflate.h>

#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <cstring>
#include <memory>
#include <string>
#include <vector>

using namespace g2m::decode;

namespace {

void put_u16(std::vector<std::byte>& v, std::uint16_t x) {
    v.push_back(static_cast<std::byte>(x & 0xFF));
    v.push_back(static_cast<std::byte>((x >> 8) & 0xFF));
}
void put_u32(std::vector<std::byte>& v, std::uint32_t x) {
    for (int i = 0; i < 4; ++i) v.push_back(static_cast<std::byte>((x >> (8 * i)) & 0xFF));
}
void put_bytes(std::vector<std::byte>& v, const std::string& s) {
    for (char c : s) v.push_back(static_cast<std::byte>(c));
}

std::vector<std::byte> deflate_compress(const std::vector<std::byte>& in) {
    libdeflate_compressor* c = libdeflate_alloc_compressor(6);
    REQUIRE(c != nullptr);
    std::vector<std::byte> out(in.size() + 4096);
    const std::size_t n = libdeflate_deflate_compress(c, in.data(), in.size(), out.data(), out.size());
    libdeflate_free_compressor(c);
    REQUIRE(n != 0);
    out.resize(n);
    return out;
}

// A minimal, hand-rolled ZIP writer covering exactly what ZipReader needs to
// parse: stored/deflate entries, and an optional forced-ZIP64 mode (sizes
// written as 0xFFFFFFFF in the classic fields with a ZIP64 extra field
// carrying the real values, plus an EOCD64 record/locator) even for a tiny
// archive - exercises the ZIP64 *parsing* path without needing an actual
// >4 GiB fixture.
class ZipBuilder {
public:
    struct Item {
        std::string name;
        std::vector<std::byte> content; // uncompressed
        bool deflate = false;
        bool force_zip64 = false;
    };

    void add(Item item) { items_.push_back(std::move(item)); }

    std::vector<std::byte> build() const {
        std::vector<std::byte> data;
        struct CdRec {
            std::string name;
            std::uint16_t method;
            std::uint32_t crc;
            std::uint64_t comp_size;
            std::uint64_t uncomp_size;
            std::uint64_t local_offset;
            bool force_zip64;
        };
        std::vector<CdRec> cd;

        for (const Item& it : items_) {
            const std::vector<std::byte> compressed = it.deflate ? deflate_compress(it.content) : it.content;
            const std::uint32_t crc = libdeflate_crc32(0, it.content.data(), it.content.size());
            const std::uint64_t local_offset = data.size();

            std::vector<std::byte> zip64_extra;
            if (it.force_zip64) {
                put_u16(zip64_extra, 0x0001);
                put_u16(zip64_extra, 16);
                for (int i = 0; i < 8; ++i)
                    zip64_extra.push_back(static_cast<std::byte>((it.content.size() >> (8 * i)) & 0xFF));
                for (int i = 0; i < 8; ++i)
                    zip64_extra.push_back(static_cast<std::byte>((compressed.size() >> (8 * i)) & 0xFF));
            }

            // Local file header.
            put_u32(data, 0x04034b50);
            put_u16(data, 20); // version needed
            put_u16(data, 0);  // flags
            put_u16(data, it.deflate ? 8 : 0);
            put_u16(data, 0); // mod time
            put_u16(data, 0); // mod date
            put_u32(data, crc);
            put_u32(data, it.force_zip64 ? 0xFFFFFFFFu : static_cast<std::uint32_t>(compressed.size()));
            put_u32(data, it.force_zip64 ? 0xFFFFFFFFu : static_cast<std::uint32_t>(it.content.size()));
            put_u16(data, static_cast<std::uint16_t>(it.name.size()));
            put_u16(data, static_cast<std::uint16_t>(zip64_extra.size()));
            put_bytes(data, it.name);
            data.insert(data.end(), zip64_extra.begin(), zip64_extra.end());
            data.insert(data.end(), compressed.begin(), compressed.end());

            cd.push_back(CdRec{it.name, static_cast<std::uint16_t>(it.deflate ? 8 : 0), crc, compressed.size(),
                                it.content.size(), local_offset, it.force_zip64});
        }

        const std::uint64_t cd_start = data.size();
        for (const CdRec& r : cd) {
            std::vector<std::byte> zip64_extra;
            if (r.force_zip64) {
                put_u16(zip64_extra, 0x0001);
                put_u16(zip64_extra, 24);
                for (int i = 0; i < 8; ++i) zip64_extra.push_back(static_cast<std::byte>((r.uncomp_size >> (8 * i)) & 0xFF));
                for (int i = 0; i < 8; ++i) zip64_extra.push_back(static_cast<std::byte>((r.comp_size >> (8 * i)) & 0xFF));
                for (int i = 0; i < 8; ++i) zip64_extra.push_back(static_cast<std::byte>((r.local_offset >> (8 * i)) & 0xFF));
            }
            put_u32(data, 0x02014b50);
            put_u16(data, 20); // version made by
            put_u16(data, 20); // version needed
            put_u16(data, 0);  // flags
            put_u16(data, r.method);
            put_u16(data, 0); // mod time
            put_u16(data, 0); // mod date
            put_u32(data, r.crc);
            put_u32(data, r.force_zip64 ? 0xFFFFFFFFu : static_cast<std::uint32_t>(r.comp_size));
            put_u32(data, r.force_zip64 ? 0xFFFFFFFFu : static_cast<std::uint32_t>(r.uncomp_size));
            put_u16(data, static_cast<std::uint16_t>(r.name.size()));
            put_u16(data, static_cast<std::uint16_t>(zip64_extra.size()));
            put_u16(data, 0); // comment length
            put_u16(data, 0); // disk number start
            put_u16(data, 0); // internal attrs
            put_u32(data, 0); // external attrs
            put_u32(data, r.force_zip64 ? 0xFFFFFFFFu : static_cast<std::uint32_t>(r.local_offset));
            put_bytes(data, r.name);
            data.insert(data.end(), zip64_extra.begin(), zip64_extra.end());
        }
        const std::uint64_t cd_size = data.size() - cd_start;

        // EOCD (+ ZIP64 EOCD/locator when forced).
        const bool any_zip64 = std::any_of(items_.begin(), items_.end(), [](const Item& i) { return i.force_zip64; });
        if (any_zip64) {
            const std::uint64_t eocd64_off = data.size();
            put_u32(data, 0x06064b50);
            for (int i = 0; i < 8; ++i) data.push_back(static_cast<std::byte>((std::uint64_t{44} >> (8 * i)) & 0xFF));
            put_u16(data, 20);
            put_u16(data, 20);
            put_u32(data, 0);
            put_u32(data, 0);
            for (int i = 0; i < 8; ++i) data.push_back(static_cast<std::byte>((cd.size() >> (8 * i)) & 0xFF));
            for (int i = 0; i < 8; ++i) data.push_back(static_cast<std::byte>((cd.size() >> (8 * i)) & 0xFF));
            for (int i = 0; i < 8; ++i) data.push_back(static_cast<std::byte>((cd_size >> (8 * i)) & 0xFF));
            for (int i = 0; i < 8; ++i) data.push_back(static_cast<std::byte>((cd_start >> (8 * i)) & 0xFF));

            put_u32(data, 0x07064b50);
            put_u32(data, 0);
            for (int i = 0; i < 8; ++i) data.push_back(static_cast<std::byte>((eocd64_off >> (8 * i)) & 0xFF));
            put_u32(data, 1);
        }
        put_u32(data, 0x06054b50);
        put_u16(data, 0);
        put_u16(data, 0);
        put_u16(data, any_zip64 ? 0xFFFF : static_cast<std::uint16_t>(cd.size()));
        put_u16(data, any_zip64 ? 0xFFFF : static_cast<std::uint16_t>(cd.size()));
        put_u32(data, any_zip64 ? 0xFFFFFFFFu : static_cast<std::uint32_t>(cd_size));
        put_u32(data, any_zip64 ? 0xFFFFFFFFu : static_cast<std::uint32_t>(cd_start));
        put_u16(data, 0);
        return data;
    }

private:
    std::vector<Item> items_;
};

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
