// tests/unit/test_tile_raster_hash.cpp - tile keys, rasters and their
// georeference, hashes.

#include "g2m/core/hash.h"
#include "g2m/core/raster.h"
#include "g2m/core/tile_key.h"
#include "g2m/core/version.h"

#include "splitmix64.h"

#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <climits>
#include <cmath>
#include <string>
#include <vector>

using namespace g2m;
using geo::Hemisphere;
using geo::UtmZone;

namespace {
constexpr UtmZone k32N{32, Hemisphere::North};
}

TEST_CASE("version string", "[hash]") { CHECK(std::string(version_string()) == "0.1.0"); }

TEST_CASE("tile key from grid coordinates", "[tile]") {
    const auto k = tile_key_at(k32N, 0, 475999.99, 5550000.0);
    REQUIRE(k);
    CHECK(k->x == 1859); // floor(475999.99 / 256)
    CHECK(k->y == 21679);
    CHECK(k->size_m() == 256);
    CHECK(k->min_easting() == 1859 * 256);
    const auto k2 = *tile_key_at(k32N, 2, 476000.0, 5550000.0);
    CHECK(k2.size_m() == 1024);
    CHECK(k2.x == 464);
    // Negative eastings (extended zone far west of the central meridian).
    const auto neg = *tile_key_at(k32N, 0, -0.5, 100.0);
    CHECK(neg.x == -1);
    CHECK(neg.min_easting() == -256);
    CHECK(to_string(k2) == "utm32n/2/464/5419");
    CHECK_FALSE(tile_key_at(k32N, -1, 0, 0));
    CHECK_FALSE(tile_key_at(k32N, 21, 0, 0));
    CHECK_FALSE(tile_key_at({61, Hemisphere::North}, 0, 0, 0));
    CHECK_FALSE(tile_key_at(k32N, 0, INFINITY, 0));
    CHECK_FALSE(tile_key_at(k32N, 0, 256.0 * 8388608.0, 0)); // x = 2^23: out of the 24-bit range
}

TEST_CASE("tile key packing round-trips and orders", "[tile]") {
    g2m_test::SplitMix64 rng(31);
    std::vector<TileKey> keys;
    for (int i = 0; i < 5000; ++i) {
        TileKey k;
        k.zone = {1 + static_cast<int>(rng.next() % 60), (rng.next() & 1) ? Hemisphere::South : Hemisphere::North};
        k.level = static_cast<int>(rng.next() % (TileKey::kMaxLevel + 1));
        k.x = static_cast<std::int32_t>(static_cast<std::int64_t>(rng.next() % (1u << 24)) - (1 << 23));
        k.y = static_cast<std::int32_t>(static_cast<std::int64_t>(rng.next() % (1u << 24)) - (1 << 23));
        REQUIRE(k.valid());
        const auto back = TileKey::unpack(k.packed());
        REQUIRE(back);
        CHECK(*back == k);
        keys.push_back(k);
    }
    std::vector<TileKey> by_key = keys;
    std::sort(by_key.begin(), by_key.end());
    std::vector<TileKey> by_packed = keys;
    std::sort(by_packed.begin(), by_packed.end(),
              [](const TileKey& a, const TileKey& b) { return a.packed() < b.packed(); });
    CHECK(by_key == by_packed);
    // Fixed packing (a change here breaks every cache and URL).
    const TileKey fixed{k32N, 0, 1859, 21679};
    CHECK(fixed.packed() == ((32ull << 54) | (0ull << 53) | (0ull << 48) | ((21679ull + (1u << 23)) << 24) |
                             (1859ull + (1u << 23))));
    CHECK_FALSE(TileKey::unpack(1ull << 60));
    CHECK_FALSE(TileKey::unpack(0)); // zone 0
}

TEST_CASE("tile key parent/child", "[tile]") {
    const TileKey k{k32N, 0, -3, 5};
    const TileKey p = k.parent();
    CHECK(p.level == 1);
    CHECK(p.x == -2); // floor(-3 / 2)
    CHECK(p.y == 2);
    for (int dy = 0; dy < 2; ++dy) {
        for (int dx = 0; dx < 2; ++dx) {
            const TileKey c = p.child(dx, dy);
            CHECK(c.parent() == p);
            CHECK(c.min_easting() >= p.min_easting());
            CHECK(c.min_easting() + c.size_m() <= p.min_easting() + p.size_m());
        }
    }
    // A grid point's tile at level L+1 is the parent of its tile at level L.
    g2m_test::SplitMix64 rng(41);
    for (int i = 0; i < 2000; ++i) {
        const double e = rng.uniform(-600000.0, 1600000.0);
        const double n = rng.uniform(0.0, 9400000.0);
        for (int level = 0; level < 10; ++level) {
            CHECK(tile_key_at(k32N, level, e, n)->parent() == *tile_key_at(k32N, level + 1, e, n));
        }
    }
}

TEST_CASE("tile key path strings: parse_tile_path inverts to_string", "[tile]") {
    g2m_test::SplitMix64 rng(77);
    for (int i = 0; i < 2000; ++i) {
        TileKey k;
        const auto zone = static_cast<int>(1 + rng.next() % 60);
        const bool south = (rng.next() & 1) != 0;
        k.zone = UtmZone{zone, south ? Hemisphere::South : Hemisphere::North};
        k.level = static_cast<int>(rng.next() % (TileKey::kMaxLevel + 1));
        const auto x = static_cast<std::int64_t>(rng.next() % (std::uint64_t{1} << TileKey::kIndexBits));
        k.x = static_cast<std::int32_t>(x + TileKey::kIndexMin);
        const auto y = static_cast<std::int64_t>(rng.next() % (std::uint64_t{1} << TileKey::kIndexBits));
        k.y = static_cast<std::int32_t>(y + TileKey::kIndexMin);
        REQUIRE(k.valid());
        const auto back = parse_tile_path(to_string(k));
        REQUIRE(back);
        CHECK(*back == k);
    }
    CHECK(parse_tile_path("utm32n/2/350/5450") == TileKey{k32N, 2, 350, 5450});
    CHECK(parse_tile_path("utm1s/0/-8388608/8388607") ==
          TileKey{UtmZone{1, Hemisphere::South}, 0, TileKey::kIndexMin, TileKey::kIndexMax});
    for (const char* bad : {"", "utm32n", "utm32n/2/350", "utm32n/2/350/5450/", "/utm32n/2/350/5450",
                            "utm32n/2/350/+5450", "utm32n/02/350/5450", "utm32n/2/-0/5450", "utm32n/2/ 350/5450",
                            "utm32n/21/0/0", "utm32n/2/8388608/0", "utm32n/2/0/-8388609", "utm0n/2/0/0",
                            "utm61n/2/0/0", "utm032n/2/0/0", "utm32N/2/0/0", "UTM32n/2/0/0", "utm32/2/0/0",
                            "utm32n/2/0/99999999999999999999"}) {
        INFO(bad);
        CHECK_FALSE(parse_tile_path(bad));
    }
    CHECK(geo::parse_utm_scheme("utm60s") == UtmZone{60, Hemisphere::South});
    CHECK_FALSE(geo::parse_utm_scheme("utm60"));
}

TEST_CASE("height quantisation", "[raster]") {
    CHECK(*quantise_height(0.0) == 0);
    CHECK(*quantise_height(1.0) == 256);
    CHECK(*quantise_height(-1.5) == -384);
    CHECK(*quantise_height(1.0 / 512) == 1);    // tie: away from zero
    CHECK(*quantise_height(-1.0 / 512) == -1);
    CHECK(*quantise_height(123.456) == 31605);  // 31604.736
    CHECK(dequantise_height(31605) == 31605.0 / 256.0);
    CHECK_FALSE(quantise_height(NAN));
    CHECK_FALSE(quantise_height(1e7));
    CHECK_FALSE(quantise_height(static_cast<double>(INT32_MIN) / 256.0));
    for (std::int32_t u : {-1000000, -1, 0, 1, 77777, 2000000}) {
        CHECK(*quantise_height(dequantise_height(u)) == u);
    }
}

TEST_CASE("raster storage", "[raster]") {
    HeightRaster r(3, 2, 7);
    CHECK(r.size() == 6);
    r.at(2, 1) = -5;
    CHECK(r.data()[5] == -5);
    CHECK(r.at(0, 0) == 7);
    HeightRaster r2 = r;
    CHECK(r2 == r);
    r2.at(0, 1) = 0;
    CHECK_FALSE(r2 == r);
}

TEST_CASE("raster georeference: node vs cell registration", "[raster]") {
    const TileKey tile{k32N, 0, 1859, 21679};
    const auto node = *tile_georef(tile, 256, SampleRegistration::Node);
    CHECK(node.width == 257);
    CHECK(node.spacing_mm == 1000);
    CHECK(node.cells_x() == 256);
    CHECK(node.sample_easting_m(0) == 1859.0 * 256.0);
    CHECK(node.sample_easting_m(256) == 1860.0 * 256.0);
    const auto cell = *tile_georef(tile, 256, SampleRegistration::Cell);
    CHECK(cell.width == 256);
    CHECK(cell.sample_easting_m(0) == 1859.0 * 256.0 + 0.5);
    CHECK(cell.sample_northing_half_mm(0) == 2 * (21679ll * 256 * 1000) + 1000);
    CHECK_FALSE(tile_georef(tile, 3, SampleRegistration::Cell)); // 256000 mm / 3 is not whole

    // Hessen DGM1 tile dgm1_32_475_5550 as its .tfw describes it: 1 m cells,
    // first (top-left) pixel CENTRE at (475000.5, 5550999.5), rows N -> S.
    RasterGeoref dgm;
    dgm.zone = k32N;
    dgm.west_mm = 475000000;
    dgm.south_mm = 5550000000;
    dgm.spacing_mm = 1000;
    dgm.registration = SampleRegistration::Cell;
    dgm.row_order = RowOrder::NorthToSouth;
    dgm.width = 1000;
    dgm.height = 1000;
    REQUIRE(dgm.valid());
    CHECK(dgm.sample_easting_m(0) == 475000.5);
    CHECK(dgm.sample_northing_m(0) == 5550999.5);
    CHECK(dgm.sample_northing_m(999) == 5550000.5);

    // A g2m cell-registered 1 m tile inside it shares the lattice exactly...
    const TileKey inside = *tile_key_at(k32N, 0, 475100.0, 5550100.0);
    const auto g2m_cell = *tile_georef(inside, 256, SampleRegistration::Cell);
    const auto off = lattice_offset(dgm, g2m_cell);
    REQUIRE(off);
    CHECK(off->row_flip);
    const auto dcol = static_cast<int>(off->dcol);
    CHECK(dgm.sample_easting_half_mm(dcol) == g2m_cell.sample_easting_half_mm(0));
    const int arow = static_cast<int>(off->drow - 0); // b row 0 -> a row drow (flipped)
    CHECK(dgm.sample_northing_half_mm(arow) == g2m_cell.sample_northing_half_mm(0));
    CHECK(dgm.sample_northing_half_mm(arow - 255) == g2m_cell.sample_northing_half_mm(255));
    // ...a node-registered one is half a metre off: copying would resample.
    const auto g2m_node = *tile_georef(inside, 256, SampleRegistration::Node);
    CHECK_FALSE(lattice_offset(dgm, g2m_node));

    // Same row order, both directions.
    auto flipped = dgm;
    flipped.row_order = RowOrder::SouthToNorth;
    const auto same = lattice_offset(flipped, g2m_cell);
    REQUIRE(same);
    CHECK_FALSE(same->row_flip);
    CHECK(flipped.sample_northing_half_mm(static_cast<int>(same->drow) + 17) ==
          g2m_cell.sample_northing_half_mm(17));
    const auto both_ns = lattice_offset(dgm, *tile_georef(inside, 256, SampleRegistration::Cell, RowOrder::NorthToSouth));
    REQUIRE(both_ns);
    CHECK_FALSE(both_ns->row_flip);
}

TEST_CASE("FNV-1a 64 test vectors and serialisation", "[hash]") {
    CHECK(Fnv1a64().value() == 0xcbf29ce484222325ull);
    CHECK(Fnv1a64().bytes("a", 1).value() == 0xaf63dc4c8601ec8cull);
    CHECK(Fnv1a64().bytes("foobar", 6).value() == 0x85944171f73967e8ull);
    // Explicit little-endian: independent of host byte order.
    CHECK(Fnv1a64().u32(0x04030201u).value() == Fnv1a64().bytes("\x01\x02\x03\x04", 4).value());
    CHECK(Fnv1a64().f64(1.0).value() == Fnv1a64().u64(0x3ff0000000000000ull).value());
    CHECK(Fnv1a64().f64(0.0).value() != Fnv1a64().f64(-0.0).value());
    CHECK(Fnv1a64().str("ab").str("c").value() != Fnv1a64().str("a").str("bc").value());
    CHECK(to_hex(std::uint64_t{0xcbf29ce484222325ull}) == "cbf29ce484222325");
}

TEST_CASE("SHA-256 FIPS 180-4 test vectors", "[hash]") {
    CHECK(to_hex(Sha256::of("")) == "e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855");
    CHECK(to_hex(Sha256::of("abc")) == "ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f20015ad");
    CHECK(to_hex(Sha256::of("abcdbcdecdefdefgefghfghighijhijkijkljklmklmnlmnomnopnopq")) ==
          "248d6a61d20638b8e5c026930c3e6039a33ce45964ff2167f6ecedd419db06c1");
    // One million 'a', fed in uneven chunks (exercises the block buffer).
    Sha256 h;
    const std::string chunk(997, 'a');
    std::size_t left = 1000000;
    while (left > 0) {
        const std::size_t n = std::min(left, chunk.size());
        h.update(chunk.data(), n);
        left -= n;
    }
    CHECK(to_hex(h.finish()) == "cdc76e5c9914fb9281a1c7e284d73e67f1809a48a497200e046d39ccc7112cd0");
}
