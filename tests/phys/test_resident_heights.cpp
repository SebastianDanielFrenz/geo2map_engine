// tests/phys/test_resident_heights.cpp - ResidentHeightSet (PLAN.md R2.2
// section 1a/4a): sorted find/install/retain_only.
#include "g2m/phys/resident_heights.h"

#include "g2m/core/geo/utm.h"

#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <atomic>
#include <thread>
#include <memory>
#include <vector>

using g2m::TileKey;
using g2m::geo::Hemisphere;
using g2m::geo::UtmZone;
using g2m::phys::ResidentHeightSet;

namespace {
constexpr UtmZone k32N{32, Hemisphere::North};

TileKey key_at(std::int32_t x, std::int32_t y) {
    TileKey k;
    k.zone = k32N;
    k.level = 0;
    k.x = x;
    k.y = y;
    return k;
}

std::shared_ptr<const g2m::HeightTile> make_tile(const TileKey& key, std::int32_t fill_value) {
    auto tile = std::make_shared<g2m::HeightTile>();
    tile->key = key;
    tile->h.fill(fill_value);
    return tile;
}
} // namespace

TEST_CASE("ResidentHeightSet: find on an empty set is Missing", "[phys][resident_heights]") {
    ResidentHeightSet set;
    const g2m::HeightTile* out = nullptr;
    CHECK(set.find(key_at(1, 2), &out) == ResidentHeightSet::Lookup::Missing);
    CHECK(out == nullptr);
    CHECK(set.size() == 0);
    CHECK_FALSE(set.contains(key_at(1, 2)));
}

TEST_CASE("ResidentHeightSet: install a resident tile, find returns it", "[phys][resident_heights]") {
    ResidentHeightSet set;
    const TileKey key = key_at(3, -4);
    auto tile = make_tile(key, 12345);
    const auto* raw = tile.get();
    set.install(key, tile);

    const g2m::HeightTile* out = nullptr;
    CHECK(set.find(key, &out) == ResidentHeightSet::Lookup::Resident);
    CHECK(out == raw);
    CHECK(set.size() == 1);
    CHECK(set.contains(key));
}

TEST_CASE("ResidentHeightSet: install nullptr records a confirmed Absent", "[phys][resident_heights]") {
    ResidentHeightSet set;
    const TileKey key = key_at(0, 0);
    set.install(key, nullptr);

    const g2m::HeightTile* out = nullptr;
    CHECK(set.find(key, &out) == ResidentHeightSet::Lookup::Absent);
    CHECK(out == nullptr);
    CHECK(set.size() == 1);
    CHECK(set.contains(key)); // Absent still "contains" - only Missing does not
}

TEST_CASE("ResidentHeightSet: installing the same key twice replaces the entry, not duplicates it",
         "[phys][resident_heights]") {
    ResidentHeightSet set;
    const TileKey key = key_at(5, 5);
    set.install(key, make_tile(key, 1));
    CHECK(set.size() == 1);

    auto second = make_tile(key, 2);
    const auto* raw2 = second.get();
    set.install(key, second);
    CHECK(set.size() == 1);

    const g2m::HeightTile* out = nullptr;
    CHECK(set.find(key, &out) == ResidentHeightSet::Lookup::Resident);
    CHECK(out == raw2);

    // Absent can also replace a previously-Resident entry (a re-fetch that
    // now 404s) and vice versa.
    set.install(key, nullptr);
    CHECK(set.size() == 1);
    CHECK(set.find(key, &out) == ResidentHeightSet::Lookup::Absent);
}

TEST_CASE("ResidentHeightSet: find is correct regardless of install order (sorted binary search)",
         "[phys][resident_heights]") {
    ResidentHeightSet set;
    // Install out of packed() order on purpose.
    const TileKey keys[] = {key_at(10, 0), key_at(-5, 3), key_at(0, 0), key_at(2, -2), key_at(1000, -1000)};
    for (int i = 0; i < 5; ++i) {
        set.install(keys[i], make_tile(keys[i], i));
    }
    CHECK(set.size() == 5);

    for (int i = 0; i < 5; ++i) {
        const g2m::HeightTile* out = nullptr;
        CAPTURE(i);
        REQUIRE(set.find(keys[i], &out) == ResidentHeightSet::Lookup::Resident);
        REQUIRE(out != nullptr);
        CHECK(out->h[0] == i);
    }
    // A key never installed is still Missing.
    const g2m::HeightTile* out = nullptr;
    CHECK(set.find(key_at(999, 999), &out) == ResidentHeightSet::Lookup::Missing);
}

TEST_CASE("ResidentHeightSet::retain_only drops everything not in the keep list", "[phys][resident_heights]") {
    ResidentHeightSet set;
    const TileKey a = key_at(0, 0);
    const TileKey b = key_at(1, 0);
    const TileKey c = key_at(2, 0);
    const TileKey d = key_at(3, 0);
    set.install(a, make_tile(a, 1));
    set.install(b, nullptr); // Absent
    set.install(c, make_tile(c, 3));
    set.install(d, make_tile(d, 4));
    REQUIRE(set.size() == 4);

    // Keep b and d only (must be sorted ascending by packed()).
    std::vector<TileKey> keep = {b, d};
    std::sort(keep.begin(), keep.end(), [](const TileKey& x, const TileKey& y) { return x.packed() < y.packed(); });
    set.retain_only(keep);

    CHECK(set.size() == 2);
    const g2m::HeightTile* out = nullptr;
    CHECK(set.find(a, &out) == ResidentHeightSet::Lookup::Missing);
    CHECK(set.find(b, &out) == ResidentHeightSet::Lookup::Absent);
    CHECK(set.find(c, &out) == ResidentHeightSet::Lookup::Missing);
    CHECK(set.find(d, &out) == ResidentHeightSet::Lookup::Resident);
}

TEST_CASE("ResidentHeightSet::retain_only with an empty keep list evicts everything", "[phys][resident_heights]") {
    ResidentHeightSet set;
    const TileKey a = key_at(0, 0);
    set.install(a, make_tile(a, 1));
    REQUIRE(set.size() == 1);

    set.retain_only({});
    CHECK(set.size() == 0);
    const g2m::HeightTile* out = nullptr;
    CHECK(set.find(a, &out) == ResidentHeightSet::Lookup::Missing);
}

TEST_CASE("ResidentHeightSet::retain_only tolerates duplicate keys in the keep list", "[phys][resident_heights]") {
    ResidentHeightSet set;
    const TileKey a = key_at(7, 7);
    const TileKey b = key_at(8, 7);
    set.install(a, make_tile(a, 1));
    set.install(b, make_tile(b, 2));

    const std::vector<TileKey> keep = {a, a, b, b, b};
    set.retain_only(keep);
    CHECK(set.size() == 2);
    CHECK(set.contains(a));
    CHECK(set.contains(b));
}

TEST_CASE("ResidentHeightSet::retain_only keeps a key that never resided (harmless no-op for it)",
         "[phys][resident_heights]") {
    ResidentHeightSet set;
    const TileKey a = key_at(0, 0);
    set.install(a, make_tile(a, 1));

    const TileKey never_installed = key_at(99, 99);
    std::vector<TileKey> keep = {a, never_installed};
    std::sort(keep.begin(), keep.end(), [](const TileKey& x, const TileKey& y) { return x.packed() < y.packed(); });
    set.retain_only(keep);

    CHECK(set.size() == 1); // never_installed was never an entry, so nothing to keep for it
    CHECK(set.contains(a));
}


TEST_CASE("Resident snapshots survive replacement and eviction until owner purge", "[phys][snapshot]") {
    ResidentHeightSet set;
    const auto key = key_at(-1, -2);
    set.install(key, make_tile(key, 256));
    set.publish_snapshot();
    auto view = set.snapshot();
    std::weak_ptr<const ResidentHeightSet> weak = view;
    set.install(key, make_tile(key, 512));
    CHECK_FALSE(set.snapshot()); // cannot publish during tick acquisition
    const g2m::HeightTile* old = nullptr;
    CHECK(view->find(key, &old) == ResidentHeightSet::Lookup::Resident);
    CHECK(old->h[0] == 256);
    set.retain_only({});
    CHECK(set.size() == 0);
    set.publish_snapshot();
    CHECK(set.snapshot()->size() == 0);
    set.purge_retired_snapshots();
    CHECK_FALSE(weak.expired());
    view.reset(); // releasing a ticket must not destroy its snapshot
    CHECK_FALSE(weak.expired());
    set.purge_retired_snapshots();
    CHECK(weak.expired());
    CHECK(set.retired_snapshot_count() == 0);
}


TEST_CASE("Resident snapshot reads race neither owner replacement nor purge", "[phys][snapshot]") {
    ResidentHeightSet set;
    const auto key = key_at(-1, -2);
    set.install(key, make_tile(key, 256));
    set.publish_snapshot();
    auto view = set.snapshot();
    std::atomic<bool> start{false}, ok{true};
    std::thread reader([&] {
        while (!start.load(std::memory_order_acquire)) std::this_thread::yield();
        for (int i = 0; i < 10000; ++i) {
            const g2m::HeightTile* height = nullptr;
            if (view->find(key, &height) != ResidentHeightSet::Lookup::Resident || height->h[0] != 256) ok = false;
        }
    });
    start.store(true, std::memory_order_release);
    for (int i = 0; i < 32; ++i) {
        set.install(key, make_tile(key, 512 + i));
        set.publish_snapshot();
        set.purge_retired_snapshots();
    }
    reader.join();
    CHECK(ok.load());
}
