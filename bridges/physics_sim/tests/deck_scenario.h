// bridges/physics_sim/tests/deck_scenario.h - the recorded deck set and the install scenario of the phase-9 parity
// tests (terrain migration plan 6.6 "Phase 9").
//
// The set: every deck of road_mesh_test_data.h's cases (the committed OSM corridor fixtures frankfurt_hoechst and
// hoechst_a66 as served roads.geom tiles, plus the synthetic structure / junction / overpass scenes), de-duplicated by
// DeckKey (the ".holed" / ".sep" variants of one tile build the same keys). The scenario installs it like the game's
// Session does: a start-up focus (prefetch built ahead, required decks waited for) and a second focus further along.
//
// run_deck_scenario is written against the installer's duck type (update / wait_until_installed / installed / bodies,
// candidates with .key.{way_id,start,end}), so the SAME text runs against g2m::ps_bridge::DeckInstaller (this repo)
// and against racing_game's rg::DeckInstaller (the local, uncommitted parity harness).
#pragma once

#include "road_mesh_test_data.h"

#include "g2m/core/hash.h"
#include "g2m/world/road_structures.h"

#include "ps/world/world.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <map>
#include <memory>
#include <tuple>
#include <utility>
#include <vector>

namespace g2m_bridge_test {

using DeckPtr = std::shared_ptr<const g2m::world::RoadDeck>;

// Session frame origin of the scenario (absolute UTM metres) and the grip ids of its two classes.
inline constexpr double kDeckE0 = 464000.0, kDeckN0 = 5559000.0;
inline constexpr ps::SurfaceId kDeckPaved = 3, kDeckUnpaved = 7;

inline std::vector<DeckPtr> corridor_decks() {
    std::vector<DeckPtr> out;
    std::map<std::tuple<std::int64_t, int, int>, bool> seen;
    for (const auto& c : g2m_mesh_test::mesh_cases()) {
        g2m::world::RoadSeparation sep;
        sep.enabled = c.separated;
        for (auto& deck : g2m::world::build_road_decks(c.tile, sep)) {
            const auto k = std::make_tuple(deck->way_id, static_cast<int>(std::round(deck->start_station)),
                                           static_cast<int>(std::round(deck->end_station)));
            if (seen.emplace(k, true).second) out.push_back(std::move(deck));
        }
    }
    return out;
}

struct DeckRecord {
    std::int64_t way_id = 0;
    int start = 0, end = 0;
    ps::BodyId id{};
    std::uint64_t digest = 0;
    friend bool operator==(const DeckRecord&, const DeckRecord&) = default;
};

inline std::uint64_t hash_records(const std::vector<DeckRecord>& records) {
    g2m::Fnv1a64 h;
    h.u64(records.size());
    for (const auto& r : records) h.i64(r.way_id).i32(r.start).i32(r.end).u32(r.id.index).u32(r.id.generation).u64(r.digest);
    return h.value();
}

// Absolute-UTM focus points of the scenario: the middle of the first deck's box and of the last deck's box (in key
// order), so the second focus pulls in decks the first only saw as prefetch or not at all.
inline std::pair<std::pair<double, double>, std::pair<double, double>> scenario_foci(const std::vector<DeckPtr>& decks) {
    const auto mid = [](const DeckPtr& d) {
        return std::make_pair(0.5 * (d->min_easting + d->max_easting), 0.5 * (d->min_northing + d->max_northing));
    };
    const auto key = [](const DeckPtr& d) {
        return std::make_tuple(d->way_id, static_cast<int>(std::round(d->start_station)), static_cast<int>(std::round(d->end_station)));
    };
    DeckPtr first = decks.front(), last = decks.front();
    for (const auto& d : decks) {
        if (key(d) < key(first)) first = d;
        if (key(last) < key(d)) last = d;
    }
    return {mid(first), mid(last)};
}

inline constexpr double kDeckRequiredRadiusM = 700.0;
inline constexpr double kDeckPrefetchMarginM = 700.0;

// select(e, n, required_radius, prefetch_radius, installer) -> {required, prefetch} (each ascending by key).
// The game's start-up path (Session::sync_road_decks with budget <= 0): prefetch is queued, then the required decks
// are waited for. Returns one record per deck installed by this run, in install order.
template <class Installer, class Select>
std::vector<DeckRecord> run_deck_scenario(ps::World& world, Installer& installer, const std::vector<DeckPtr>& decks,
                                          Select&& select, bool legacy = false) {
    std::vector<DeckRecord> records;
    const auto [a, b] = scenario_foci(decks);
    for (const auto& focus : {a, b}) {
        auto [required, prefetch] = select(focus.first, focus.second, kDeckRequiredRadiusM,
                                           kDeckRequiredRadiusM + kDeckPrefetchMarginM, installer);
        std::vector<std::tuple<std::int64_t, int, int>> expected;
        for (const auto& c : required) {
            if (!installer.installed(c.key)) expected.emplace_back(c.key.way_id, c.key.start, c.key.end);
        }
        const std::size_t before = installer.bodies().size();
        if (legacy) {
            while (!installer.update_legacy(world, required, 0)) {
            }
        } else {
            (void)installer.update(world, {}, prefetch, 0);
            installer.wait_until_installed(world, required);
        }
        const auto& bodies = installer.bodies();
        // A deck whose mesh Jolt rejects is skipped without a body; none of the recorded set may fail.
        if (bodies.size() - before != expected.size()) throw std::runtime_error("deck scenario: installed count differs from the plan");
        for (std::size_t i = 0; i < expected.size(); ++i) {
            DeckRecord r;
            std::tie(r.way_id, r.start, r.end) = expected[i];
            r.id = bodies[before + i];
            r.digest = world.backend().debug_body_shape_digest(r.id);
            records.push_back(r);
        }
    }
    return records;
}

} // namespace g2m_bridge_test
