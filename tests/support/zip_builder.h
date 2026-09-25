// tests/support/zip_builder.h - a minimal, hand-rolled ZIP writer covering
// exactly what ZipReader needs to parse: stored/deflate entries, and an
// optional forced-ZIP64 mode (sizes written as 0xFFFFFFFF in the classic
// fields with a ZIP64 extra field carrying the real values, plus an EOCD64
// record/locator) even for a tiny archive - exercises the ZIP64 *parsing*
// path without needing an actual >4 GiB fixture.
//
// Promoted UNCHANGED from tests/unit/test_decode_zip.cpp (PLAN.md G1c I2,
// tests/support/fixture_cut) so any test-only tool needing a synthetic ZIP
// fixture (fixture_cut's future cut-worldcover/repack-dgm1 subcommands,
// PLAN.md G1c I3+) can reuse it instead of re-deriving the format by hand;
// test_decode_zip.cpp itself now includes this header instead of defining
// ZipBuilder locally.
#pragma once

#include <libdeflate.h>

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace g2m::test_support {

inline void zip_put_u16(std::vector<std::byte>& v, std::uint16_t x) {
    v.push_back(static_cast<std::byte>(x & 0xFF));
    v.push_back(static_cast<std::byte>((x >> 8) & 0xFF));
}
inline void zip_put_u32(std::vector<std::byte>& v, std::uint32_t x) {
    for (int i = 0; i < 4; ++i) v.push_back(static_cast<std::byte>((x >> (8 * i)) & 0xFF));
}
inline void zip_put_bytes(std::vector<std::byte>& v, const std::string& s) {
    for (char c : s) v.push_back(static_cast<std::byte>(c));
}

inline std::vector<std::byte> zip_deflate_compress(const std::vector<std::byte>& in) {
    libdeflate_compressor* c = libdeflate_alloc_compressor(6);
    std::vector<std::byte> out(in.size() + 4096);
    const std::size_t n = libdeflate_deflate_compress(c, in.data(), in.size(), out.data(), out.size());
    libdeflate_free_compressor(c);
    out.resize(n);
    return out;
}

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
            const std::vector<std::byte> compressed = it.deflate ? zip_deflate_compress(it.content) : it.content;
            const std::uint32_t crc = libdeflate_crc32(0, it.content.data(), it.content.size());
            const std::uint64_t local_offset = data.size();

            std::vector<std::byte> zip64_extra;
            if (it.force_zip64) {
                zip_put_u16(zip64_extra, 0x0001);
                zip_put_u16(zip64_extra, 16);
                for (int i = 0; i < 8; ++i)
                    zip64_extra.push_back(static_cast<std::byte>((it.content.size() >> (8 * i)) & 0xFF));
                for (int i = 0; i < 8; ++i)
                    zip64_extra.push_back(static_cast<std::byte>((compressed.size() >> (8 * i)) & 0xFF));
            }

            // Local file header.
            zip_put_u32(data, 0x04034b50);
            zip_put_u16(data, 20); // version needed
            zip_put_u16(data, 0);  // flags
            zip_put_u16(data, it.deflate ? 8 : 0);
            zip_put_u16(data, 0); // mod time
            zip_put_u16(data, 0); // mod date
            zip_put_u32(data, crc);
            zip_put_u32(data, it.force_zip64 ? 0xFFFFFFFFu : static_cast<std::uint32_t>(compressed.size()));
            zip_put_u32(data, it.force_zip64 ? 0xFFFFFFFFu : static_cast<std::uint32_t>(it.content.size()));
            zip_put_u16(data, static_cast<std::uint16_t>(it.name.size()));
            zip_put_u16(data, static_cast<std::uint16_t>(zip64_extra.size()));
            zip_put_bytes(data, it.name);
            data.insert(data.end(), zip64_extra.begin(), zip64_extra.end());
            data.insert(data.end(), compressed.begin(), compressed.end());

            cd.push_back(CdRec{it.name, static_cast<std::uint16_t>(it.deflate ? 8 : 0), crc, compressed.size(),
                                it.content.size(), local_offset, it.force_zip64});
        }

        const std::uint64_t cd_start = data.size();
        for (const CdRec& r : cd) {
            std::vector<std::byte> zip64_extra;
            if (r.force_zip64) {
                zip_put_u16(zip64_extra, 0x0001);
                zip_put_u16(zip64_extra, 24);
                for (int i = 0; i < 8; ++i) zip64_extra.push_back(static_cast<std::byte>((r.uncomp_size >> (8 * i)) & 0xFF));
                for (int i = 0; i < 8; ++i) zip64_extra.push_back(static_cast<std::byte>((r.comp_size >> (8 * i)) & 0xFF));
                for (int i = 0; i < 8; ++i) zip64_extra.push_back(static_cast<std::byte>((r.local_offset >> (8 * i)) & 0xFF));
            }
            zip_put_u32(data, 0x02014b50);
            zip_put_u16(data, 20); // version made by
            zip_put_u16(data, 20); // version needed
            zip_put_u16(data, 0);  // flags
            zip_put_u16(data, r.method);
            zip_put_u16(data, 0); // mod time
            zip_put_u16(data, 0); // mod date
            zip_put_u32(data, r.crc);
            zip_put_u32(data, r.force_zip64 ? 0xFFFFFFFFu : static_cast<std::uint32_t>(r.comp_size));
            zip_put_u32(data, r.force_zip64 ? 0xFFFFFFFFu : static_cast<std::uint32_t>(r.uncomp_size));
            zip_put_u16(data, static_cast<std::uint16_t>(r.name.size()));
            zip_put_u16(data, static_cast<std::uint16_t>(zip64_extra.size()));
            zip_put_u16(data, 0); // comment length
            zip_put_u16(data, 0); // disk number start
            zip_put_u16(data, 0); // internal attrs
            zip_put_u32(data, 0); // external attrs
            zip_put_u32(data, r.force_zip64 ? 0xFFFFFFFFu : static_cast<std::uint32_t>(r.local_offset));
            zip_put_bytes(data, r.name);
            data.insert(data.end(), zip64_extra.begin(), zip64_extra.end());
        }
        const std::uint64_t cd_size = data.size() - cd_start;

        // EOCD (+ ZIP64 EOCD/locator when forced).
        const bool any_zip64 = std::any_of(items_.begin(), items_.end(), [](const Item& i) { return i.force_zip64; });
        if (any_zip64) {
            const std::uint64_t eocd64_off = data.size();
            zip_put_u32(data, 0x06064b50);
            for (int i = 0; i < 8; ++i) data.push_back(static_cast<std::byte>((std::uint64_t{44} >> (8 * i)) & 0xFF));
            zip_put_u16(data, 20);
            zip_put_u16(data, 20);
            zip_put_u32(data, 0);
            zip_put_u32(data, 0);
            for (int i = 0; i < 8; ++i) data.push_back(static_cast<std::byte>((cd.size() >> (8 * i)) & 0xFF));
            for (int i = 0; i < 8; ++i) data.push_back(static_cast<std::byte>((cd.size() >> (8 * i)) & 0xFF));
            for (int i = 0; i < 8; ++i) data.push_back(static_cast<std::byte>((cd_size >> (8 * i)) & 0xFF));
            for (int i = 0; i < 8; ++i) data.push_back(static_cast<std::byte>((cd_start >> (8 * i)) & 0xFF));

            zip_put_u32(data, 0x07064b50);
            zip_put_u32(data, 0);
            for (int i = 0; i < 8; ++i) data.push_back(static_cast<std::byte>((eocd64_off >> (8 * i)) & 0xFF));
            zip_put_u32(data, 1);
        }
        zip_put_u32(data, 0x06054b50);
        zip_put_u16(data, 0);
        zip_put_u16(data, 0);
        zip_put_u16(data, any_zip64 ? 0xFFFF : static_cast<std::uint16_t>(cd.size()));
        zip_put_u16(data, any_zip64 ? 0xFFFF : static_cast<std::uint16_t>(cd.size()));
        zip_put_u32(data, any_zip64 ? 0xFFFFFFFFu : static_cast<std::uint32_t>(cd_size));
        zip_put_u32(data, any_zip64 ? 0xFFFFFFFFu : static_cast<std::uint32_t>(cd_start));
        zip_put_u16(data, 0);
        return data;
    }

private:
    std::vector<Item> items_;
};

} // namespace g2m::test_support
