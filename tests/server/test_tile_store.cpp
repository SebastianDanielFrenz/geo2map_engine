// TileStore: round trips (inline and blob file), dedup, LRU under a cap,
// pinning, corruption (blob file and inline) -> miss + dropped entry,
// persistence, and the concurrency contract (tile_store.h).

#include "g2m/server/tile_store.h"

#include "server_test_util.h"

#include <catch2/catch_test_macros.hpp>
#include <sqlite3.h>

#include <atomic>
#include <mutex>
#include <thread>

using namespace g2m;
using g2m_test::TempDir;

namespace {

constexpr const char* kLc = "g2m.src.landcover";
constexpr std::size_t kInlineLimit = 1024;

std::unique_ptr<TileStore> open_store(const TempDir& d, std::uint64_t cap = 0, std::string name = "tiles") {
    TileStoreConfig c;
    c.dir = d.path();
    c.name = std::move(name);
    c.size_cap_bytes = cap;
    c.inline_limit = kInlineLimit;
    auto s = TileStore::open(c);
    REQUIRE(s);
    return std::move(*s);
}

TileAddress addr(std::int32_t dx, std::string scope = "s1") {
    return TileAddress{std::move(scope), kLc, g2m_test::home_tile(dx, 0)};
}

void put(TileStore& s, const TileAddress& a, const std::vector<std::uint8_t>& body, bool pin = false) {
    auto r = s.put(a, g2m_test::make_header(a.layer, a.key, body), body, pin);
    REQUIRE(r);
}

std::optional<StoredTile> get(TileStore& s, const TileAddress& a) {
    auto r = s.get(a);
    REQUIRE(r);
    return std::move(*r);
}

TileStoreStats stats(TileStore& s) {
    auto r = s.stats();
    REQUIRE(r);
    return *r;
}

void consistent(TileStore& s) {
    auto r = s.check_consistency();
    INFO((r ? std::string() : r.error().message));
    CHECK(r);
}

// Runs one SQL statement with one blob parameter on the store's database file.
void raw_sql(const TempDir& d, const char* sql, const std::vector<std::uint8_t>& blob) {
    sqlite3* db = nullptr;
    REQUIRE(sqlite3_open((d.path() / "tiles.sqlite3").string().c_str(), &db) == SQLITE_OK);
    sqlite3_stmt* st = nullptr;
    REQUIRE(sqlite3_prepare_v2(db, sql, -1, &st, nullptr) == SQLITE_OK);
    sqlite3_bind_blob(st, 1, blob.data(), static_cast<int>(blob.size()), SQLITE_TRANSIENT);
    CHECK(sqlite3_step(st) == SQLITE_DONE);
    sqlite3_finalize(st);
    sqlite3_close(db);
}

} // namespace

TEST_CASE("tile store: inline and blob-file round trips", "[server][store]") {
    TempDir d("store_rt");
    auto s = open_store(d);
    const auto small = g2m_test::make_body(1, 300);
    const auto large = g2m_test::make_body(2, 5000);
    REQUIRE(small.size() == 300);
    REQUIRE(large.size() == 5000);
    put(*s, addr(0), small);
    put(*s, addr(1), large, /*pin=*/true);

    auto a = get(*s, addr(0));
    REQUIRE(a);
    CHECK(a->body == small);
    CHECK(a->header.body_hash == content_hash(small));
    CHECK_FALSE(a->pinned);
    auto pc = parse_container(a->container());
    REQUIRE(pc);
    CHECK(pc->header == a->header);

    auto b = get(*s, addr(1));
    REQUIRE(b);
    CHECK(b->body == large);
    CHECK(b->pinned);
    CHECK(std::filesystem::exists(g2m_test::blob_file(d.path(), "tiles", content_hash(large))));
    CHECK_FALSE(std::filesystem::exists(g2m_test::blob_file(d.path(), "tiles", content_hash(small))));

    CHECK_FALSE(get(*s, addr(2)));
    CHECK_FALSE(get(*s, addr(0, "other")));
    auto ph = s->peek_hash(addr(1));
    REQUIRE(ph);
    CHECK(*ph == content_hash(large));

    const TileStoreStats st = stats(*s);
    CHECK(st.tiles == 2);
    CHECK(st.bodies == 2);
    CHECK(st.blob_files == 1);
    CHECK(st.body_bytes == 5300);
    CHECK(st.pinned_tiles == 1);
    consistent(*s);

    auto idx = s->index("s1");
    REQUIRE(idx);
    REQUIRE(idx->size() == 2);
    CHECK((*idx)[0].key == g2m_test::home_tile(0, 0));
    CHECK((*idx)[1].body_hash == content_hash(large));
}

TEST_CASE("tile store: argument checks", "[server][store]") {
    TempDir d("store_args");
    auto s = open_store(d);
    const auto body = g2m_test::make_body(3, 100);
    CHECK_FALSE(s->put(TileAddress{"", kLc, g2m_test::home_tile()}, g2m_test::make_header(kLc, g2m_test::home_tile(), body), body));
    CHECK_FALSE(s->put(TileAddress{std::string(129, 'a'), kLc, g2m_test::home_tile()},
                       g2m_test::make_header(kLc, g2m_test::home_tile(), body), body));
    CHECK_FALSE(s->put(TileAddress{"s", "Bad", g2m_test::home_tile()}, g2m_test::make_header("Bad", g2m_test::home_tile(), body), body));
    // header must match the address
    CHECK_FALSE(s->put(addr(0), g2m_test::make_header(kLc, g2m_test::home_tile(1, 0), body), body));
    CHECK_FALSE(s->put(addr(0), g2m_test::make_header("g2m.src.osm", g2m_test::home_tile(0, 0), body), body));
    // put_container verifies the container
    auto c = assemble_container(g2m_test::make_header(kLc, g2m_test::home_tile(), body), body).value();
    c.back() ^= 1;
    CHECK_FALSE(s->put_container(addr(0), c));
    CHECK(stats(*s).tiles == 0);

    TileStoreConfig bad;
    bad.dir = d.path();
    bad.name = "Bad-Name";
    CHECK_FALSE(TileStore::open(bad));
}

TEST_CASE("tile store: dedup, replace and reference counts", "[server][store]") {
    TempDir d("store_dedup");
    auto s = open_store(d);
    const auto big = g2m_test::make_body(4, 4000);
    const auto other = g2m_test::make_body(5, 200);
    put(*s, addr(0), big);
    put(*s, addr(1), big);
    put(*s, addr(0, "s2"), big);
    CHECK(stats(*s).tiles == 3);
    CHECK(stats(*s).bodies == 1);
    CHECK(stats(*s).body_bytes == 4000);
    const auto file = g2m_test::blob_file(d.path(), "tiles", content_hash(big));

    auto r = s->remove(addr(0));
    REQUIRE(r);
    CHECK(*r);
    CHECK(std::filesystem::exists(file));
    put(*s, addr(1), other); // replace: drops one more reference
    CHECK(stats(*s).bodies == 2);
    CHECK(std::filesystem::exists(file));
    r = s->remove(addr(0, "s2"));
    REQUIRE(r);
    CHECK(*r);
    CHECK_FALSE(std::filesystem::exists(file));
    r = s->remove(addr(0, "s2"));
    REQUIRE(r);
    CHECK_FALSE(*r);
    const TileStoreStats st = stats(*s);
    CHECK(st.tiles == 1);
    CHECK(st.bodies == 1);
    CHECK(st.body_bytes == 200);
    CHECK(st.blob_files == 0);
    consistent(*s);
}

TEST_CASE("tile store: LRU eviction under the size cap", "[server][store]") {
    TempDir d("store_lru");
    auto s = open_store(d, /*cap=*/3000);
    for (int i = 0; i < 3; ++i) put(*s, addr(i), g2m_test::make_body(10 + static_cast<std::uint64_t>(i), 1000));
    CHECK(stats(*s).tiles == 3);
    REQUIRE(get(*s, addr(0))); // 0 is now the most recently used; 1 the least
    put(*s, addr(3), g2m_test::make_body(13, 1000));
    CHECK(get(*s, addr(0)));
    CHECK_FALSE(get(*s, addr(1)));
    CHECK(get(*s, addr(2)));
    CHECK(get(*s, addr(3)));
    TileStoreStats st = stats(*s);
    CHECK(st.evicted_tiles == 1);
    CHECK(st.body_bytes == 3000);
    // One oversized put evicts everything else but never itself.
    put(*s, addr(4), g2m_test::make_body(14, 5000));
    CHECK(get(*s, addr(4)));
    st = stats(*s);
    CHECK(st.tiles == 1);
    CHECK(st.body_bytes == 5000);
    consistent(*s);
}

TEST_CASE("tile store: pinning survives eviction", "[server][store]") {
    TempDir d("store_pin");
    auto s = open_store(d, /*cap=*/2500);
    put(*s, addr(0), g2m_test::make_body(20, 1000), /*pin=*/true);
    put(*s, addr(1), g2m_test::make_body(21, 1000));
    put(*s, addr(2), g2m_test::make_body(22, 1000)); // evicts 1 (0 is pinned although older)
    CHECK(get(*s, addr(0)));
    CHECK_FALSE(get(*s, addr(1)));
    CHECK(get(*s, addr(2)));
    // A re-put without pin keeps the pin.
    put(*s, addr(0), g2m_test::make_body(20, 1000));
    CHECK(get(*s, addr(0))->pinned);

    // Range pinning over a rectangle.
    for (int i = 3; i < 6; ++i) put(*s, addr(i), g2m_test::make_body(20 + static_cast<std::uint64_t>(i), 10));
    auto n = s->set_pinned_range("s1", kLc, g2m_test::home_tile(2, 0), g2m_test::home_tile(4, 0), true);
    REQUIRE(n);
    CHECK(*n == 3); // 2, 3, 4
    CHECK(stats(*s).pinned_tiles == 4);
    auto e = s->evict_to(0);
    REQUIRE(e);
    CHECK(*e == 1); // only 5 was unpinned
    CHECK_FALSE(get(*s, addr(5)));
    auto u = s->set_pinned(addr(0), false);
    REQUIRE(u);
    CHECK(*u);
    e = s->evict_to(0);
    REQUIRE(e);
    CHECK(*e == 1);
    CHECK(stats(*s).tiles == 3);
    u = s->set_pinned(addr(9), true);
    REQUIRE(u);
    CHECK_FALSE(*u);
    consistent(*s);
}

TEST_CASE("tile store: a damaged blob file is a miss and drops the entry", "[server][store]") {
    TempDir d("store_corrupt_file");
    auto s = open_store(d);
    const auto body = g2m_test::make_body(30, 6000);
    const auto file = g2m_test::blob_file(d.path(), "tiles", content_hash(body));

    SECTION("one flipped byte") {
        put(*s, addr(0), body);
        put(*s, addr(1), body); // shares the body
        auto bytes = g2m_test::read_file(file);
        bytes[3000] ^= 0x10;
        g2m_test::write_file(file, bytes);
        CHECK_FALSE(get(*s, addr(0)));
        const TileStoreStats st = stats(*s);
        // The body is bad for every address: all its entries go, and the file with them.
        CHECK(st.integrity_drops == 2);
        CHECK(st.tiles == 0);
        CHECK(st.bodies == 0);
        CHECK_FALSE(std::filesystem::exists(file));
        CHECK_FALSE(get(*s, addr(1)));
        consistent(*s);
        // A later put stores the body again.
        put(*s, addr(0), body);
        CHECK(get(*s, addr(0))->body == body);
        consistent(*s);
    }
    SECTION("truncated") {
        put(*s, addr(0), body);
        auto bytes = g2m_test::read_file(file);
        bytes.resize(100);
        g2m_test::write_file(file, bytes);
        CHECK_FALSE(get(*s, addr(0)));
        CHECK(stats(*s).integrity_drops == 1);
        CHECK(stats(*s).tiles == 0);
        consistent(*s);
    }
    SECTION("missing") {
        put(*s, addr(0), body);
        std::filesystem::remove(file);
        CHECK_FALSE(get(*s, addr(0)));
        CHECK(stats(*s).integrity_drops == 1);
        CHECK(stats(*s).tiles == 0);
        consistent(*s);
    }
    SECTION("a put repairs a missing file of a body it shares") {
        put(*s, addr(0), body);
        std::filesystem::remove(file);
        put(*s, addr(1), body);
        CHECK(std::filesystem::exists(file));
        CHECK(get(*s, addr(0))->body == body);
        consistent(*s);
    }
}

TEST_CASE("tile store: damaged inline body or header is a miss and drops the entry", "[server][store]") {
    TempDir d("store_corrupt_inline");
    const auto body = g2m_test::make_body(31, 500);
    {
        auto s = open_store(d);
        put(*s, addr(0), body);
        put(*s, addr(1), g2m_test::make_body(32, 400));
    }
    auto flipped = body;
    flipped[250] ^= 0x01;
    raw_sql(d, "UPDATE bodies SET data = ?1 WHERE size = 500", flipped);
    auto hdr = g2m_test::make_header(kLc, g2m_test::home_tile(1, 0), g2m_test::make_body(32, 400));
    auto hb = encode_header(hdr).value();
    hb[hb.size() / 2] ^= 0x01;
    raw_sql(d, "UPDATE tiles SET header = ?1 WHERE hash IN (SELECT hash FROM bodies WHERE size = 400)", hb);

    auto s = open_store(d);
    CHECK_FALSE(get(*s, addr(0)));
    CHECK_FALSE(get(*s, addr(1)));
    const TileStoreStats st = stats(*s);
    CHECK(st.integrity_drops == 2);
    CHECK(st.tiles == 0);
    CHECK(st.bodies == 0);
    CHECK(st.body_bytes == 0);
    consistent(*s);
}

TEST_CASE("tile store: contents persist across reopen", "[server][store]") {
    TempDir d("store_reopen");
    const auto small = g2m_test::make_body(40, 100);
    const auto large = g2m_test::make_body(41, 3000);
    {
        auto s = open_store(d);
        put(*s, addr(0), small, true);
        put(*s, addr(1), large);
    }
    auto s = open_store(d);
    auto a = get(*s, addr(0));
    REQUIRE(a);
    CHECK(a->body == small);
    CHECK(a->pinned);
    CHECK(get(*s, addr(1))->body == large);
    CHECK(stats(*s).body_bytes == 3100);
    consistent(*s);
}

// PLAN.md 7.3: several g2m workers share one store object, and a second
// object (another process in real use) opens the same directory. Bodies are
// drawn from a small shared pool so dedup/refcount races are exercised.
TEST_CASE("tile store: concurrent use from threads and two objects", "[server][store]") {
    TempDir d("store_threads");
    auto s1 = open_store(d, /*cap=*/40000);
    auto s2 = open_store(d, /*cap=*/40000);
    std::vector<std::vector<std::uint8_t>> pool;
    for (int i = 0; i < 12; ++i) {
        pool.push_back(g2m_test::make_body(100 + static_cast<std::uint64_t>(i), i % 3 == 0 ? 3000 : 400));
    }
    std::atomic<int> failures{0}, bad_reads{0}, hits{0};
    std::mutex err_mu;
    std::vector<std::string> errors;
    const auto fail = [&](const Error& e) {
        failures++;
        std::lock_guard lock(err_mu);
        errors.push_back(e.message);
    };
    const auto worker = [&](TileStore& s, std::uint64_t seed) {
        g2m_test::SplitMix64 r(seed);
        for (int it = 0; it < 300; ++it) {
            const auto op = g2m_test::below(r, 10);
            const auto slot = static_cast<std::int32_t>(g2m_test::below(r, 16));
            const auto b = static_cast<std::size_t>(g2m_test::below(r, pool.size()));
            const TileAddress a = addr(slot);
            if (op < 4) {
                if (auto res = s.put(a, g2m_test::make_header(kLc, a.key, pool[b]), pool[b], op == 0); !res) fail(res.error());
            } else if (op < 8) {
                auto t = s.get(a);
                if (!t) {
                    fail(t.error());
                } else if (*t) {
                    hits++;
                    const auto& tile = **t;
                    bool known = false;
                    for (const auto& p : pool) known = known || p == tile.body;
                    if (!known || tile.header.body_hash != content_hash(tile.body) || tile.header.key != a.key) bad_reads++;
                }
            } else if (op == 8) {
                if (auto res = s.remove(a); !res) fail(res.error());
            } else {
                if (auto res = s.set_pinned(a, false); !res) fail(res.error());
            }
        }
    };
    std::vector<std::thread> threads;
    for (int t = 0; t < 4; ++t) threads.emplace_back(worker, std::ref(*s1), 1000 + static_cast<std::uint64_t>(t));
    for (int t = 0; t < 2; ++t) threads.emplace_back(worker, std::ref(*s2), 2000 + static_cast<std::uint64_t>(t));
    for (auto& t : threads) t.join();
    for (const auto& e : errors) UNSCOPED_INFO(e);
    CHECK(failures == 0);
    CHECK(bad_reads == 0);
    CHECK(hits > 0);
    // No read ever saw damage: nothing was dropped for integrity.
    CHECK(stats(*s1).integrity_drops == 0);
    CHECK(stats(*s2).integrity_drops == 0);
    consistent(*s1);
    consistent(*s2);
}
