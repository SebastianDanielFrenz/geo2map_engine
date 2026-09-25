// ClientCache (PLAN.md 7.2): containers keyed by (rid, layer, key) on a
// TileStore - round trip, pinning, LRU cap, integrity drop.

#include "g2m/client/cache.h"

#include "server_test_util.h"

#include <catch2/catch_test_macros.hpp>

using namespace g2m;
using g2m_test::TempDir;

namespace {

const std::string kLc(kSrcLandcoverLayer);

ReleaseId rid_of(std::uint8_t b) {
    ReleaseId r{};
    r[0] = b;
    return r;
}

std::vector<std::uint8_t> container(const ReleaseId& rid, std::int32_t dx, std::size_t size) {
    const TileKey k = g2m_test::home_tile(dx, 0);
    const auto body = g2m_test::make_body(500 + static_cast<std::uint64_t>(dx), size);
    TileHeader h = g2m_test::make_header(kLc, k, body);
    h.rid = rid;
    return assemble_container(h, body).value();
}

std::unique_ptr<ClientCache> open_cache(const TempDir& d, std::uint64_t cap) {
    ClientCacheConfig c;
    c.dir = d.path();
    c.size_cap_bytes = cap;
    c.inline_limit = 1024;
    auto cache = ClientCache::open(c);
    REQUIRE(cache);
    return std::move(*cache);
}

} // namespace

TEST_CASE("client cache: round trip, releases kept apart", "[client][store]") {
    TempDir d("client_rt");
    auto cache = open_cache(d, 0);
    const auto a = container(rid_of(1), 0, 300);
    const auto b = container(rid_of(2), 0, 3000);
    REQUIRE(cache->put(a));
    REQUIRE(cache->put(b, /*pin=*/true));
    auto ga = cache->get(rid_of(1), kLc, g2m_test::home_tile());
    REQUIRE(ga);
    REQUIRE(*ga);
    CHECK(**ga == a);
    auto gb = cache->get(rid_of(2), kLc, g2m_test::home_tile());
    REQUIRE(gb);
    REQUIRE(*gb);
    CHECK(**gb == b);
    auto miss = cache->get(rid_of(3), kLc, g2m_test::home_tile());
    REQUIRE(miss);
    CHECK_FALSE(*miss);
    CHECK(std::filesystem::exists(d.path() / "cache.sqlite3"));
    CHECK(cache->stats()->pinned_tiles == 1);
    CHECK(cache->check_consistency());

    // No release id, or a damaged container: refused.
    CHECK_FALSE(cache->put(container(ReleaseId{}, 1, 100)));
    auto bad = container(rid_of(1), 1, 100);
    bad[bad.size() - 3] ^= 1;
    CHECK_FALSE(cache->put(bad));
    CHECK(cache->stats()->tiles == 2);
    REQUIRE(cache->remove(rid_of(1), kLc, g2m_test::home_tile()));
    CHECK(cache->stats()->tiles == 1);
}

TEST_CASE("client cache: LRU cap, pinning and integrity drops", "[client][store]") {
    TempDir d("client_lru");
    auto cache = open_cache(d, 2500);
    const ReleaseId rid = rid_of(7);
    REQUIRE(cache->put(container(rid, 0, 1000)));
    REQUIRE(cache->put(container(rid, 1, 1000)));
    auto p = cache->set_pinned(rid, kLc, g2m_test::home_tile(0, 0), true);
    REQUIRE(p);
    CHECK(*p);
    REQUIRE(cache->put(container(rid, 2, 1000))); // evicts 1, not the pinned 0
    CHECK(*cache->get(rid, kLc, g2m_test::home_tile(0, 0)));
    CHECK_FALSE(*cache->get(rid, kLc, g2m_test::home_tile(1, 0)));
    CHECK(cache->stats()->evicted_tiles == 1);
    auto n = cache->set_pinned_range(rid, kLc, g2m_test::home_tile(0, 0), g2m_test::home_tile(5, 0), true);
    REQUIRE(n);
    CHECK(*n == 2);

    // Flip a byte of a blob file: a miss, the entry and file are gone.
    const auto big = container(rid, 3, 4000);
    REQUIRE(cache->put(big));
    const auto file = g2m_test::blob_file(d.path(), "cache", parse_container(big)->header.body_hash);
    REQUIRE(std::filesystem::exists(file));
    auto bytes = g2m_test::read_file(file);
    bytes[10] ^= 0x80;
    g2m_test::write_file(file, bytes);
    auto g = cache->get(rid, kLc, g2m_test::home_tile(3, 0));
    REQUIRE(g);
    CHECK_FALSE(*g);
    CHECK(cache->stats()->integrity_drops == 1);
    CHECK_FALSE(std::filesystem::exists(file));
    CHECK(cache->check_consistency());
}
