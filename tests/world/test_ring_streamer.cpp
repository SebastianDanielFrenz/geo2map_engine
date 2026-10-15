// tests/world/test_ring_streamer.cpp - g2m::world::RingStreamer (terrain migration phase 3b) against racing_game
// 5a3baed's own road_stream.gd and building_stream.gd: ring_pins.inc was recorded by refgen/extract_ring_ref.sh,
// which runs those scripts headless under refgen/ring_harness.gd over the STEPS below (the same table) and prints
// the job list after each schedule, the jobs it uploaded, and the resident and known tiles after each step.
//
// The scripts compute in float32 (Vector2 at UTM magnitudes, 0.5 m steps); the port computes in double, so a job's
// order is compared within 1 m. Everything else (which tiles are wanted, at which tier, the serving order, what is
// resident, visible, downgraded, evicted and known) must match exactly.
#include "g2m/world/ring_streamer.h"

#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <cstdint>
#include <map>
#include <tuple>
#include <utility>
#include <vector>

namespace {
namespace gw = g2m::world;

struct Row {
    int product, step;
    char kind;
    int x, y, a, b, c;
    double v;
};
constexpr Row kRows[] = {
#include "ring_pins.inc"
};

constexpr double kE0 = 466000.0, kN0 = 5552000.0;
struct Step {
    double x, y, vx, vy;
    int uploads;
};
constexpr Step kSteps[] = {
    {137.3, 411.9, 0.0, 0.0, 9},       {301.7, 455.2, 12.0, 3.0, 4},       {1290.4, 610.8, 35.0, 8.0, 12},
    {2470.1, 1580.6, 30.0, 25.0, 12},  {2502.9, 1601.3, 30.0, 25.0, 0},    {-1820.6, -2290.3, -1.0, 0.5, 15},
    {6950.2, 4180.7, 50.0, -10.0, 6},  {7300.9, 4090.4, 50.0, -10.0, 3},
};

bool content(int product, int x, int y) {
    const auto pmod = [](int a, int m) { return ((a % m) + m) % m; };
    return product == 0 ? pmod(x + y, 5) != 0 : pmod(x * 7 + y, 3) != 0;
}

gw::RingStreamerConfig config(int product) {
    gw::RingStreamerConfig c;
    if (product == 0) {
        c.full_m = c.far_m = 2500.0; // road_stream.gd distance_m
        c.empty_keeps_resident = true;
    } else {
        c.full_m = 2200.0; // building_stream.gd ordinary_distance_m
        c.far_m = 6000.0;  // skyline_distance_m (the harness's)
    }
    return c;
}

struct TileState {
    bool full, near_visible, far_visible;
};

void run(int product) {
    gw::RingStreamer ring(config(product));
    std::map<std::pair<int, int>, TileState> tiles; // (y, x)
    std::size_t rows_seen = 0;
    const auto apply = [&](double e, double n) {
        std::vector<gw::RingChange> changes;
        ring.frame(e, n, changes);
        for (const auto& c : changes) {
            const auto key = std::make_pair(c.y, c.x);
            if (c.evicted) {
                CHECK(tiles.erase(key) == 1);
                continue;
            }
            auto it = tiles.find(key);
            REQUIRE(it != tiles.end());
            it->second.near_visible = c.near_visible;
            it->second.far_visible = c.far_visible;
            if (c.downgraded) it->second.full = false;
        }
    };
    for (int s = 0; s < static_cast<int>(std::size(kSteps)); ++s) {
        INFO("product " << product << " step " << s);
        const Step& st = kSteps[s];
        const double e = st.x + kE0, n = st.y + kN0;
        std::vector<Row> jobs, uploads, resident, known;
        for (const Row& r : kRows) {
            if (r.product != product || r.step != s) continue;
            ++rows_seen;
            (r.kind == 'J' ? jobs : r.kind == 'U' ? uploads : r.kind == 'T' ? resident : known).push_back(r);
        }
        ring.update(e, n, st.vx, st.vy);
        apply(e, n);
        const auto queue = ring.jobs();
        REQUIRE(queue.size() == jobs.size());
        for (std::size_t i = 0; i < queue.size(); ++i) {
            INFO("job " << i);
            CHECK(queue[i].x == jobs[i].x);
            CHECK(queue[i].y == jobs[i].y);
            CHECK(queue[i].full == (jobs[i].a != 0));
            CHECK(std::abs(queue[i].order - jobs[i].v) < 1.0);
        }
        for (const Row& u : uploads) {
            const auto job = ring.next_job();
            REQUIRE(job);
            CHECK(job->x == u.x);
            CHECK(job->y == u.y);
            CHECK(job->full == (u.a != 0));
            CHECK(content(product, u.x, u.y) == (u.b != 0));
            const auto r = ring.accept(job->x, job->y, job->full, u.b != 0);
            const auto key = std::make_pair(job->y, job->x);
            if (r == gw::RingAccept::Added) tiles[key] = TileState{job->full, false, false};
            if (r == gw::RingAccept::Empty && !ring.config().empty_keeps_resident) tiles.erase(key);
        }
        apply(e, n);
        REQUIRE(tiles.size() == resident.size());
        REQUIRE(ring.resident_count() == resident.size());
        std::size_t i = 0;
        for (const auto& [key, t] : tiles) {
            const Row& r = resident[i++];
            INFO("tile " << r.x << "/" << r.y);
            CHECK(key.second == r.x);
            CHECK(key.first == r.y);
            CHECK(t.full == (r.a != 0));
            CHECK(t.near_visible == (r.b != 0));
            CHECK(t.far_visible == (r.c != 0));
        }
        const auto k = ring.known();
        REQUIRE(k.size() == known.size());
        for (std::size_t j = 0; j < k.size(); ++j) {
            CHECK(k[j].x == known[j].x);
            CHECK(k[j].y == known[j].y);
            CHECK(k[j].full == (known[j].a != 0));
        }
    }
    std::size_t total = 0;
    for (const Row& r : kRows) total += r.product == product;
    CHECK(rows_seen == total);
}
} // namespace

TEST_CASE("RingStreamer reproduces road_stream.gd", "[world][ring_streamer]") { run(0); }

TEST_CASE("RingStreamer reproduces building_stream.gd", "[world][ring_streamer]") { run(1); }

TEST_CASE("RingStreamer rejects a partial result for a full tile and downgrades", "[world][ring_streamer]") {
    gw::RingStreamerConfig c;
    c.full_m = 1000.0;
    c.far_m = 5000.0;
    gw::RingStreamer ring(c);
    ring.update(512.0, 512.0);
    CHECK(ring.accept(0, 0, true, true) == gw::RingAccept::Added);
    CHECK(ring.accept(0, 0, false, true) == gw::RingAccept::Rejected);
    std::vector<gw::RingChange> ch;
    ring.frame(512.0, 512.0, ch);
    REQUIRE(ch.size() == 1);
    CHECK((ch[0].near_visible && ch[0].far_visible && !ch[0].downgraded && !ch[0].evicted));
    ch.clear();
    ring.frame(512.0, 512.0, ch);
    CHECK(ch.empty()); // nothing changed
    ring.frame(512.0 + 3100.0, 512.0, ch);
    REQUIRE(ch.size() == 1);
    CHECK((ch[0].downgraded && !ch[0].near_visible && ch[0].far_visible));
    CHECK(ring.known().front().full == false);
    CHECK(ring.accept(0, 0, false, true) == gw::RingAccept::Added); // the partial result is taken now
    ch.clear();
    ring.frame(512.0 + 7100.0, 512.0, ch);
    REQUIRE(ch.size() == 1);
    CHECK(ch[0].evicted);
    CHECK(ring.known().empty());
    CHECK(ring.resident_count() == 0);
}

TEST_CASE("RingStreamer orders tiles behind a moving focus last", "[world][ring_streamer]") {
    gw::RingStreamerConfig c;
    c.full_m = c.far_m = 3000.0;
    gw::RingStreamer ring(c);
    ring.update(512.0, 512.0, 30.0, 0.0);
    const auto jobs = ring.jobs();
    REQUIRE(!jobs.empty());
    bool behind_seen = false;
    for (const auto& j : jobs) {
        const bool behind = (j.x * 1024.0 + 512.0 - 512.0) < -1024.0;
        if (behind) behind_seen = true;
        else CHECK(!behind_seen); // every job ahead/beside comes before every job behind
    }
    CHECK(behind_seen);
    CHECK(ring.next_job()->x == 0);
    CHECK(ring.pending_jobs() == jobs.size() - 1);
}
