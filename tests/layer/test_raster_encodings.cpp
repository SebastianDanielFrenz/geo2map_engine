// src.elev and src.landcover: exact round trips (NoData kept), every
// predictor, georef variants, and rejection of malformed bodies.

#include "g2m/layer/src_elev.h"
#include "g2m/layer/src_landcover.h"

#include "splitmix64.h"
#include "synthetic.h"

#include <catch2/catch_test_macros.hpp>

#include <climits>
#include <cstdio>
#include <vector>

using namespace g2m;

namespace {

Result<ElevTile> elev_round_trip(const ElevTile& t, ElevPredictor p, std::vector<std::uint8_t>* bytes = nullptr) {
    auto body = encode_src_elev(t, p);
    if (!body) return body.error();
    auto enc = encode_body(*body);
    if (!enc) return enc.error();
    if (bytes) *bytes = *enc;
    auto dec = decode_body(*enc);
    if (!dec) return dec.error();
    return decode_src_elev(*dec);
}

ElevTile small_tile(int w, int h, RowOrder order = RowOrder::SouthToNorth,
                    SampleRegistration reg = SampleRegistration::Node) {
    ElevTile t;
    t.georef.zone = g2m_test::kZone32N;
    t.georef.west_mm = 358400000;
    t.georef.south_mm = 5580800000;
    t.georef.spacing_mm = 250;
    t.georef.registration = reg;
    t.georef.row_order = order;
    t.georef.width = w;
    t.georef.height = h;
    t.heights = HeightRaster(w, h);
    return t;
}

} // namespace

TEST_CASE("src.elev: synthetic DGM1 tile round-trips exactly, both predictors", "[layer][elev]") {
    const ElevTile t = g2m_test::make_elev_tile(1, g2m_test::home_tile(), 256);
    std::size_t nodata = 0;
    for (std::int32_t v : t.heights.data()) nodata += v == kHeightNoData ? 1u : 0u;
    REQUIRE(nodata > 0);
    for (ElevPredictor p : {ElevPredictor::Left, ElevPredictor::Gradient}) {
        std::vector<std::uint8_t> a, b;
        auto d = elev_round_trip(t, p, &a);
        REQUIRE(d);
        CHECK(*d == t);
        REQUIRE(elev_round_trip(t, p, &b));
        CHECK(a == b);
    }
}

TEST_CASE("src.elev: extreme int32 values, NoData everywhere, 1x1", "[layer][elev]") {
    g2m_test::SplitMix64 r(5);
    for (RowOrder order : {RowOrder::SouthToNorth, RowOrder::NorthToSouth}) {
        for (SampleRegistration reg : {SampleRegistration::Node, SampleRegistration::Cell}) {
            ElevTile t = small_tile(37, 23, order, reg);
            for (auto& v : t.heights.data()) {
                const std::uint64_t kind = r.next() % 6;
                const std::uint32_t bits = static_cast<std::uint32_t>(r.next());
                v = kind == 0   ? kHeightNoData
                    : kind == 1 ? INT32_MAX
                    : kind == 2 ? INT32_MIN + 1
                                : static_cast<std::int32_t>(bits);
            }
            for (ElevPredictor p : {ElevPredictor::Left, ElevPredictor::Gradient}) {
                auto d = elev_round_trip(t, p);
                REQUIRE(d);
                CHECK(*d == t);
            }
        }
    }
    ElevTile all = small_tile(9, 4);
    for (auto& v : all.heights.data()) v = kHeightNoData;
    auto d = elev_round_trip(all, ElevPredictor::Gradient);
    REQUIRE(d);
    CHECK(*d == all);

    ElevTile one = small_tile(1, 1);
    one.heights.at(0, 0) = -77;
    d = elev_round_trip(one, ElevPredictor::Left);
    REQUIRE(d);
    CHECK(*d == one);
}

TEST_CASE("src.elev: NoData section only when needed", "[layer][elev]") {
    ElevTile t = small_tile(8, 8);
    for (auto& v : t.heights.data()) v = 100 * 256;
    auto body = encode_src_elev(t);
    REQUIRE(body);
    CHECK(body->find(src_elev::kNoData) == nullptr);
    t.heights.at(3, 4) = kHeightNoData;
    body = encode_src_elev(t);
    REQUIRE(body);
    CHECK(body->find(src_elev::kNoData) != nullptr);
}

TEST_CASE("src.elev: encoder rejects inconsistent input", "[layer][elev]") {
    ElevTile t = small_tile(8, 8);
    t.georef.width = 9; // does not match the raster
    CHECK_FALSE(encode_src_elev(t));
    t = small_tile(8, 8);
    t.georef.spacing_mm = 0;
    CHECK_FALSE(encode_src_elev(t));
    t = small_tile(8, 8);
    CHECK_FALSE(encode_src_elev(t, static_cast<ElevPredictor>(9)));
}

TEST_CASE("src.elev: decoder rejects malformed bodies", "[layer][elev]") {
    ElevTile t = small_tile(7, 9); // 63 samples: one padding bit in the NoData bitmap
    for (auto& v : t.heights.data()) v = 5;
    t.heights.at(2, 2) = kHeightNoData;
    auto good = encode_src_elev(t);
    REQUIRE(good);
    REQUIRE(decode_src_elev(*good));

    TileBody b = *good;
    b.encoding = 2;
    CHECK_FALSE(decode_src_elev(b));
    b = *good;
    b.encoding_version = 2;
    CHECK_FALSE(decode_src_elev(b));
    b = *good;
    b.sections.pop_back(); // drop the NoData bitmap: flag and sections disagree
    CHECK_FALSE(decode_src_elev(b));
    b = *good;
    b.sections[1].data.pop_back(); // sample section size
    CHECK_FALSE(decode_src_elev(b));
    b = *good;
    b.sections[2].data.assign(b.sections[2].data.size(), 0); // empty bitmap
    CHECK_FALSE(decode_src_elev(b));
    b = *good;
    b.sections[2].data.back() |= 0x80; // the padding bit
    CHECK_FALSE(decode_src_elev(b));
    b = *good;
    b.sections[0].data.push_back(0); // meta trailing byte
    CHECK_FALSE(decode_src_elev(b));
}

TEST_CASE("src.elev: predictor size comparison", "[.][measure][elev]") {
    for (int seed = 1; seed <= 3; ++seed) {
        const ElevTile t = g2m_test::make_elev_tile(static_cast<std::uint64_t>(seed), g2m_test::home_tile(), 1024);
        std::vector<std::uint8_t> left, grad;
        REQUIRE(elev_round_trip(t, ElevPredictor::Left, &left));
        REQUIRE(elev_round_trip(t, ElevPredictor::Gradient, &grad));
        std::printf("seed %d: raw %zu, Left %zu, Gradient %zu bytes\n", seed, t.heights.size() * 4, left.size(),
                    grad.size());
    }
}

TEST_CASE("src.landcover: round trip and rejection", "[layer][landcover]") {
    const LandcoverTile t = g2m_test::make_landcover_tile(3, g2m_test::home_tile());
    auto body = encode_src_landcover(t);
    REQUIRE(body);
    auto enc = encode_body(*body);
    REQUIRE(enc);
    CHECK(enc->size() < t.classes.size() / 2); // compressed
    auto dec = decode_body(*enc);
    REQUIRE(dec);
    auto back = decode_src_landcover(*dec);
    REQUIRE(back);
    CHECK(*back == t);

    LandcoverTile bad = t;
    bad.class_scheme = "ESA";
    CHECK_FALSE(encode_src_landcover(bad));
    bad = t;
    bad.class_scheme.clear();
    CHECK_FALSE(encode_src_landcover(bad));
    bad = t;
    bad.georef.width = 255;
    CHECK_FALSE(encode_src_landcover(bad));

    TileBody b = *body;
    b.sections[1].data.pop_back();
    CHECK_FALSE(decode_src_landcover(b));
    b = *body;
    b.encoding = 1;
    CHECK_FALSE(decode_src_landcover(b));
}
