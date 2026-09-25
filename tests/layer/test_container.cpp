// g2m.tile/1 container: body/header round trips, determinism, corruption
// and truncation handling (docs/formats/tile_container.md).

#include "g2m/layer/tile_container.h"

#include "splitmix64.h"
#include "synthetic.h"

#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <vector>

using namespace g2m;

namespace {

std::vector<std::uint8_t> random_bytes(g2m_test::SplitMix64& r, std::size_t n, int alphabet = 256) {
    std::vector<std::uint8_t> v(n);
    for (auto& b : v) b = static_cast<std::uint8_t>(r.next() % static_cast<std::uint64_t>(alphabet));
    return v;
}

TileBody sample_body() {
    g2m_test::SplitMix64 r(11);
    TileBody b;
    b.encoding = 7;
    b.encoding_version = 3;
    b.sections.push_back(Section{1, SectionCodec::None, {1, 2, 3}});
    b.sections.push_back(Section{2, SectionCodec::Zstd, random_bytes(r, 5000, 4)});
    b.sections.push_back(Section{5, SectionCodec::Zstd, {}});
    b.sections.push_back(Section{9, SectionCodec::None, random_bytes(r, 300)});
    return b;
}

TileHeader sample_header() {
    TileHeader h;
    h.layer = "g2m.src.elev.dgm1";
    h.layer_version = 1;
    for (std::size_t i = 0; i < h.rid.size(); ++i) h.rid[i] = static_cast<std::uint8_t>(i * 7);
    h.key = g2m_test::home_tile(3, -2);
    h.datum = VerticalDatum::DHHN2016;
    h.height_offset = -1234;
    ContentHash a{}, b{};
    a[0] = 1;
    b[0] = 2;
    h.inputs = {a, b};
    return h;
}

} // namespace

TEST_CASE("container: body round trip, both codecs", "[layer][container]") {
    const TileBody b = sample_body();
    auto enc = encode_body(b);
    REQUIRE(enc);
    auto dec = decode_body(*enc);
    REQUIRE(dec);
    CHECK(*dec == b);
    // The compressible section is actually compressed.
    CHECK(enc->size() < 5000);
    // Deterministic: a second encode gives the same bytes.
    auto enc2 = encode_body(*dec);
    REQUIRE(enc2);
    CHECK(*enc2 == *enc);
}

TEST_CASE("container: encode_body rejects non-canonical input", "[layer][container]") {
    TileBody b = sample_body();
    std::swap(b.sections[0], b.sections[1]);
    CHECK_FALSE(encode_body(b));
    b = sample_body();
    b.sections[1].type = b.sections[0].type; // duplicate type
    CHECK_FALSE(encode_body(b));
    b = sample_body();
    b.sections.clear();
    for (std::uint16_t t = 1; t <= kMaxSections + 1; ++t) b.sections.push_back(Section{t, SectionCodec::None, {}});
    auto e = encode_body(b);
    REQUIRE_FALSE(e);
    CHECK(e.error().code == ErrorCode::LimitExceeded);
}

TEST_CASE("container: header and full container round trip", "[layer][container]") {
    const auto body = encode_body(sample_body());
    REQUIRE(body);
    auto c = assemble_container(sample_header(), *body);
    REQUIRE(c);
    auto v = parse_container(*c);
    REQUIRE(v);
    TileHeader expect = sample_header();
    expect.body_hash = content_hash(*body);
    expect.body_size = body->size();
    CHECK(v->header == expect);
    CHECK(std::vector<std::uint8_t>(v->body.begin(), v->body.end()) == *body);

    std::size_t consumed = 0;
    auto h = decode_header(*c, &consumed);
    REQUIRE(h);
    CHECK(consumed + body->size() == c->size());
}

TEST_CASE("container: encode_header rejects bad input", "[layer][container]") {
    TileHeader h = sample_header();
    h.layer = "Bad.Name";
    CHECK_FALSE(encode_header(h));
    h = sample_header();
    std::swap(h.inputs[0], h.inputs[1]); // unsorted
    CHECK_FALSE(encode_header(h));
    h = sample_header();
    h.inputs.push_back(h.inputs.back()); // duplicate
    CHECK_FALSE(encode_header(h));
    h = sample_header();
    h.key.level = TileKey::kMaxLevel + 1;
    CHECK_FALSE(encode_header(h));
}

TEST_CASE("container: every single-byte flip is detected", "[layer][container]") {
    const auto body = encode_body(sample_body());
    REQUIRE(body);
    auto c = assemble_container(sample_header(), *body);
    REQUIRE(c);
    std::size_t header_size = 0;
    REQUIRE(decode_header(*c, &header_size));
    for (std::size_t i = 0; i < c->size(); ++i) {
        auto bad = *c;
        bad[i] ^= 0x20;
        auto v = parse_container(bad);
        INFO("byte " << i);
        REQUIRE_FALSE(v);
        if (i >= header_size) CHECK(v.error().code == ErrorCode::HashMismatch);
    }
}

TEST_CASE("container: every truncation fails cleanly", "[layer][container]") {
    const auto body = encode_body(sample_body());
    REQUIRE(body);
    auto c = assemble_container(sample_header(), *body);
    REQUIRE(c);
    for (std::size_t n = 0; n < c->size(); ++n) {
        std::span<const std::uint8_t> part(c->data(), n);
        CHECK_FALSE(parse_container(part));
        CHECK_FALSE(decode_body(std::span<const std::uint8_t>(body->data(), std::min(n, body->size() - 1))));
    }
    // Trailing garbage is rejected too.
    auto longer = *c;
    longer.push_back(0);
    CHECK_FALSE(parse_container(longer));
}

TEST_CASE("container: random bytes never crash the decoders", "[layer][container]") {
    g2m_test::SplitMix64 r(99);
    const auto body = encode_body(sample_body());
    REQUIRE(body);
    for (int iter = 0; iter < 2000; ++iter) {
        auto m = *body;
        const int flips = 1 + static_cast<int>(r.next() % 8);
        for (int k = 0; k < flips; ++k) {
            const std::size_t pos = static_cast<std::size_t>(r.next() % m.size());
            const std::uint8_t val = static_cast<std::uint8_t>(r.next());
            m[pos] = val;
        }
        auto d = decode_body(m); // may succeed (payload bytes of a raw section), must not crash
        if (d) CHECK(d->sections.size() <= kMaxSections);
    }
}
