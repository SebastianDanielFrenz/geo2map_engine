// Tile packs: geometry, round trip, verification and corruption.

#include "g2m/layer/pack.h"

#include "splitmix64.h"
#include "synthetic.h"

#include <catch2/catch_test_macros.hpp>

using namespace g2m;

namespace {

struct Fixture {
    PackDirectory dir;
    std::array<std::vector<std::uint8_t>, kPackTiles> containers;
};

Fixture make_pack() {
    Fixture f;
    f.dir.layer = "g2m.src.landcover";
    f.dir.rid[0] = 0xab;
    f.dir.pack = pack_of(g2m_test::home_tile());
    for (int i = 0; i < kPackTiles; ++i) {
        PackEntry& e = f.dir.entries[static_cast<std::size_t>(i)];
        if (i % 37 == 0) {
            TileBody b;
            b.encoding = 2;
            b.encoding_version = 1;
            b.sections.push_back(Section{1, SectionCodec::Zstd, std::vector<std::uint8_t>(100, static_cast<std::uint8_t>(i))});
            auto body = encode_body(b);
            TileHeader h;
            h.layer = f.dir.layer;
            h.layer_version = 1;
            h.rid = f.dir.rid;
            h.key = pack_tile(f.dir.pack, i);
            auto c = assemble_container(h, *body);
            e.state = PackTileState::Present;
            e.body_hash = content_hash(*body);
            f.containers[static_cast<std::size_t>(i)] = *c;
        } else {
            e.state = i % 5 == 0 ? PackTileState::Unavailable : PackTileState::NotFound;
        }
    }
    return f;
}

} // namespace

TEST_CASE("pack: geometry, negative coordinates included", "[layer][pack]") {
    for (std::int32_t x : {-33, -17, -16, -1, 0, 15, 16, 350}) {
        for (std::int32_t y : {-16, -15, 0, 5450}) {
            TileKey t{g2m_test::kZone32N, 2, x, y};
            const TileKey p = pack_of(t);
            const int i = pack_entry_index(t);
            REQUIRE(i >= 0);
            REQUIRE(i < kPackTiles);
            CHECK(pack_tile(p, i) == t);
        }
    }
    CHECK(pack_of(TileKey{g2m_test::kZone32N, 2, -1, -1}).x == -1);
    CHECK(pack_entry_index(TileKey{g2m_test::kZone32N, 2, -1, -1}) == 255);
}

TEST_CASE("pack: round trip", "[layer][pack]") {
    const Fixture f = make_pack();
    auto bytes = encode_pack(f.dir, f.containers);
    REQUIRE(bytes);
    auto v = decode_pack(*bytes);
    REQUIRE(v);
    CHECK(v->dir == f.dir);
    for (std::size_t i = 0; i < kPackTiles; ++i) {
        CHECK(std::vector<std::uint8_t>(v->containers[i].begin(), v->containers[i].end()) == f.containers[i]);
    }
    auto again = encode_pack(v->dir, f.containers);
    REQUIRE(again);
    CHECK(*again == *bytes);
}

TEST_CASE("pack: encoder rejects inconsistent entries", "[layer][pack]") {
    Fixture f = make_pack();
    f.dir.entries[0].body_hash[0] ^= 1; // hash does not match the container
    CHECK_FALSE(encode_pack(f.dir, f.containers));
    f = make_pack();
    f.containers[1] = f.containers[0]; // data on a non-present entry
    CHECK_FALSE(encode_pack(f.dir, f.containers));
    f = make_pack();
    f.containers[37].swap(f.containers[74]); // container of another tile
    std::swap(f.dir.entries[37], f.dir.entries[74]);
    CHECK_FALSE(encode_pack(f.dir, f.containers));
}

TEST_CASE("pack: decoder rejects corruption and truncation", "[layer][pack]") {
    const Fixture f = make_pack();
    auto bytes = encode_pack(f.dir, f.containers);
    REQUIRE(bytes);
    g2m_test::SplitMix64 r(8);
    for (int iter = 0; iter < 400; ++iter) {
        auto m = *bytes;
        const std::size_t pos = static_cast<std::size_t>(r.next() % m.size());
        m[pos] ^= static_cast<std::uint8_t>(1 + r.next() % 255);
        auto v = decode_pack(m);
        // A flip in a non-present entry's state byte from NotFound to
        // Unavailable (or back) is the only undetectable change.
        if (v) CHECK(v->dir.entries != f.dir.entries);
    }
    for (std::size_t n = 0; n < bytes->size(); n += 13) {
        CHECK_FALSE(decode_pack(std::span<const std::uint8_t>(bytes->data(), n)));
    }
}
