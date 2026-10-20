// Phase 9 (terrain migration plan 6.6): the deck installer and the deck selection moved from racing_game into
// g2m::ps_bridge. The install sequence of the recorded corridor deck set - (DeckKey, BodyId, debug_body_shape_digest)
// in install order - must be deterministic (equal for 1 and 10 world workers, any installer thread count, any
// builder completion order, the async and the legacy path) AND equal to what racing_game's own
// rail_bridges deck_installer.cpp produces over the same set: kSequencePin below was recorded by that game source
// (compiled unchanged in a throwaway harness against this repo's RoadDeck, see the commit message), so a drift of
// install order, body ids or shape bytes in either direction fails here.
#include "deck_scenario.h"

#include "g2m/ps_bridge/deck_installer.h"

#include "ps/world/world.h"

#include <catch2/catch_test_macros.hpp>

#include <chrono>
#include <cstdio>
#include <thread>

using namespace g2m_bridge_test;
using g2m::ps_bridge::DeckCandidate;
using g2m::ps_bridge::DeckInstaller;
using g2m::ps_bridge::DeckKey;
using g2m::ps_bridge::select_decks;

namespace {

// Recorded from racing_game's rg::DeckInstaller (rail_bridges 9506513, core/src/deck_installer.cpp unchanged) run over
// corridor_decks() with deck_scenario.h's scenario.
constexpr std::uint64_t kSequencePin = 0x5a679cd849b363f6ULL;

const std::vector<DeckPtr>& decks() {
    static const auto all = corridor_decks();
    return all;
}

std::vector<DeckRecord> run_bridge(int world_workers, unsigned installer_workers, bool shuffle, bool legacy) {
    ps::WorldConfig cfg;
    cfg.job_workers = world_workers;
    cfg.max_bodies = 16384;
    ps::World world(cfg);
    DeckInstaller::Options options;
    options.workers = installer_workers;
    if (shuffle) {
        // Different builders finish in a different order on every key.
        options.before_build = [](const DeckKey& k) {
            std::this_thread::sleep_for(std::chrono::microseconds(((k.way_id ^ (k.start * 31) ^ (k.end * 17)) & 7) * 300));
        };
    }
    DeckInstaller installer(world, std::move(options));
    const auto select = [&](double e, double n, double required_radius, double prefetch_radius, DeckInstaller& inst) {
        auto s = select_decks(decks(), e, n, kDeckE0, kDeckN0, required_radius, prefetch_radius, kDeckPaved, kDeckUnpaved,
                              /*legacy_radius=*/false, &inst);
        return std::make_pair(std::move(s.required), std::move(s.prefetch));
    };
    return run_deck_scenario(world, installer, decks(), select, legacy);
}

} // namespace

TEST_CASE("ps_bridge decks: the recorded corridor set installs in one deterministic order, equal to the game's",
          "[ps_bridge][decks][determinism]") {
    REQUIRE(decks().size() >= 30);
    const auto a = run_bridge(1, 1, /*shuffle=*/true, /*legacy=*/false);
    const auto b = run_bridge(10, 4, /*shuffle=*/true, /*legacy=*/false);
    const auto c = run_bridge(10, 2, /*shuffle=*/false, /*legacy=*/false);
    const auto legacy = run_bridge(1, 1, /*shuffle=*/false, /*legacy=*/true);
    REQUIRE(a.size() >= 20);
    std::printf("ps_bridge decks: %zu decks in the set, %zu installed, sequence hash 0x%016llx\n", decks().size(), a.size(),
                static_cast<unsigned long long>(hash_records(a)));
    CHECK(a == b);
    CHECK(a == c);
    CHECK(a == legacy); // the reference path builds each shape on the calling thread: same bodies, same bytes
    // Bodies are numbered in install order, which is the (way, start, end) order within each focus step.
    for (std::size_t i = 1; i < a.size(); ++i) CHECK(a[i].id.index != a[i - 1].id.index);
    CHECK(hash_records(a) == kSequencePin);
}

namespace {

// The game's selection loop (racing_game core/src/session.cpp, Session::sync_road_decks), transcribed over the set.
std::pair<std::vector<DeckCandidate>, std::vector<DeckCandidate>> game_selection(double e, double n, double radius,
                                                                                  double prefetch_radius, bool legacy,
                                                                                  const DeckInstaller* installer) {
    std::vector<DeckCandidate> required, prefetch;
    for (const auto& deck : decks()) {
        const DeckKey key = g2m::ps_bridge::deck_key(*deck);
        if (installer != nullptr && installer->installed(key)) continue;
        const double dx = std::max({deck->min_easting - e, e - deck->max_easting, 0.0});
        const double dy = std::max({deck->min_northing - n, n - deck->max_northing, 0.0});
        const double d2 = dx * dx + dy * dy;
        if (d2 > (legacy ? radius : prefetch_radius) * (legacy ? radius : prefetch_radius)) continue;
        DeckCandidate c;
        c.key = key;
        c.deck = deck;
        c.surface = deck->land_class == g2m::LandClass::PavedRoad ? kDeckPaved : kDeckUnpaved;
        c.x = deck->mesh.origin[0] - kDeckE0;
        c.y = deck->mesh.origin[1] - kDeckN0;
        (d2 <= radius * radius ? required : prefetch).push_back(std::move(c));
    }
    const auto by_key = [](const DeckCandidate& a, const DeckCandidate& b) { return a.key < b.key; };
    std::stable_sort(required.begin(), required.end(), by_key);
    std::stable_sort(prefetch.begin(), prefetch.end(), by_key);
    return {std::move(required), std::move(prefetch)};
}

bool same(const std::vector<DeckCandidate>& a, const std::vector<DeckCandidate>& b) {
    if (a.size() != b.size()) return false;
    for (std::size_t i = 0; i < a.size(); ++i) {
        if (!(a[i].key == b[i].key) || a[i].deck != b[i].deck || a[i].surface != b[i].surface || a[i].x != b[i].x ||
            a[i].y != b[i].y) {
            return false;
        }
    }
    return true;
}

} // namespace

TEST_CASE("ps_bridge decks: select_decks equals the game's selection loop", "[ps_bridge][decks]") {
    const auto [a, b] = scenario_foci(decks());
    int split_cases = 0;
    for (const auto& focus : {a, b}) {
        for (const double radius : {0.0, 150.0, 700.0, 3000.0}) {
            for (const bool legacy : {false, true}) {
                const double prefetch = radius + 400.0;
                const auto want = game_selection(focus.first, focus.second, radius, prefetch, legacy, nullptr);
                const auto got = select_decks(decks(), focus.first, focus.second, kDeckE0, kDeckN0, radius, prefetch, kDeckPaved,
                                              kDeckUnpaved, legacy);
                CHECK(same(got.required, want.first));
                CHECK(same(got.prefetch, want.second));
                if (!legacy && !got.required.empty() && !got.prefetch.empty()) ++split_cases;
                if (legacy) CHECK(got.prefetch.empty());
                for (std::size_t i = 1; i < got.required.size(); ++i) CHECK(got.required[i - 1].key < got.required[i].key);
            }
        }
    }
    CHECK(split_cases >= 2); // the set exercises both lists
}

TEST_CASE("ps_bridge decks: select_decks leaves out installed decks and uses the class surface", "[ps_bridge][decks]") {
    ps::WorldConfig cfg;
    cfg.job_workers = 1;
    cfg.max_bodies = 16384;
    ps::World world(cfg);
    DeckInstaller installer(world, DeckInstaller::Options{});
    const auto [a, b] = scenario_foci(decks());
    const auto first = select_decks(decks(), a.first, a.second, kDeckE0, kDeckN0, kDeckRequiredRadiusM,
                                    kDeckRequiredRadiusM + 100.0, kDeckPaved, kDeckUnpaved, false, &installer);
    REQUIRE(!first.required.empty());
    installer.wait_until_installed(world, first.required);
    const auto again = select_decks(decks(), a.first, a.second, kDeckE0, kDeckN0, kDeckRequiredRadiusM,
                                    kDeckRequiredRadiusM + 100.0, kDeckPaved, kDeckUnpaved, false, &installer);
    CHECK(again.required.empty()); // everything installed is skipped
    const auto without = select_decks(decks(), a.first, a.second, kDeckE0, kDeckN0, kDeckRequiredRadiusM,
                                      kDeckRequiredRadiusM + 100.0, kDeckPaved, kDeckUnpaved, false, nullptr);
    CHECK(without.required.size() == first.required.size()); // without the installer they are all listed again
    for (const auto& c : without.required) {
        CHECK(c.surface == (c.deck->land_class == g2m::LandClass::PavedRoad ? kDeckPaved : kDeckUnpaved));
    }
}
