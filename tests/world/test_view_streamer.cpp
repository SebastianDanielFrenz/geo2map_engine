// tests/world/test_view_streamer.cpp - g2m::world view functions and ViewStreamer (terrain migration phase 3b) against
// racing_game 5a3baed's own world_terrain.cpp view functions, TerrainViewStreamer and diff-order helpers:
// view_pins.inc was recorded by refgen/refgen_view.cpp from the game's code over view_test_data.h; the port must
// reproduce every hash bit for bit (the one arithmetic change, std::hypot -> det::hypot, moves no pinned value).
#include "view_test_data.h"

#include "g2m/world/view_streamer.h"

#include <catch2/catch_test_macros.hpp>

#include <map>
#include <string>

namespace {
using namespace g2m_view_test;
namespace gw = g2m::world;

struct ViewPin {
    const char* name;
    std::uint64_t count;
    std::uint64_t hash;
};
constexpr ViewPin kPins[] = {
#include "view_pins.inc"
};

void check_pins(const std::vector<Pin>& got) {
    std::map<std::string, ViewPin> pins;
    for (const auto& p : kPins) pins.emplace(p.name, p);
    for (const auto& g : got) {
        INFO(g.name);
        const auto it = pins.find(g.name);
        REQUIRE(it != pins.end());
        CHECK(g.count == it->second.count);
        CHECK(g.hash == it->second.hash);
    }
}
} // namespace

TEST_CASE("view chunk builds reproduce the game's render chunks", "[world][view_streamer]") {
    check_pins(build_pins<gw::ViewSource, gw::ViewChunk>(
        [](double x, double y, const g2m::mesh::LodParams& p, double e0, double n0, std::vector<ChunkKey>& keys) {
            gw::select_view_keys(x, y, p, e0, n0, keys);
        },
        [](const std::vector<ChunkKey>& keys, const gw::ViewSource& s, unsigned threads, std::vector<gw::ViewChunk>& out) {
            REQUIRE(gw::build_view_chunks(keys, s, threads, out));
        }));
}

TEST_CASE("ViewStreamer reproduces the game's diff sequence at 1 and 3 build threads", "[world][view_streamer]") {
    for (unsigned threads : {1u, 3u}) {
        INFO("build threads " << threads);
        check_pins(streamer_pins<gw::ViewSource, gw::ViewStreamer, gw::ViewChunk, gw::ViewDiff>(threads));
    }
}

TEST_CASE("diff-order helpers reproduce the game's", "[world][view_streamer]") {
    check_pins(order_pins<gw::ViewChunk>(
        [](const ChunkKey& k, double e0, double n0, double px, double py, double vx, double vy) {
            return gw::chunk_time_to_reach_s(k, e0, n0, px, py, vx, vy);
        },
        [](double base, double speed) { return gw::reselect_distance_for_speed(base, speed); },
        [](std::vector<gw::ViewChunk>& c, double e0, double n0, double px, double py, double vx, double vy) {
            gw::sort_adds_by_urgency(c, e0, n0, px, py, vx, vy);
        },
        [](const g2m::RenderRoadLod* a, const g2m::RenderRoadLod* b, int max_level) {
            return gw::changed_road_lod_levels(a, b, max_level);
        }));
}

TEST_CASE("the colour hook sees the finished chunk and fills rgba", "[world][view_streamer]") {
    Store store;
    auto src = make_source<gw::ViewSource>(store, true, true);
    std::size_t calls = 0;
    std::mutex m;
    src.colour = [&](gw::ViewChunk& c) {
        std::lock_guard<std::mutex> lk(m);
        ++calls;
        // Class ids per vertex are the mesh's land_class; the hook paints roads white, the rest by carve state.
        c.rgba.assign(c.mesh.positions.size() / 3, 0);
        for (std::size_t i = 0; i < c.rgba.size(); ++i) {
            const auto cls = static_cast<g2m::LandClass>(c.mesh.land_class[i]);
            c.rgba[i] = (cls == g2m::LandClass::PavedRoad || cls == g2m::LandClass::UnpavedRoad) ? 0xffffffffu
                                                                                                : 0x00000000u + static_cast<std::uint32_t>(c.carve);
        }
    };
    std::vector<gw::ViewChunk> with, without;
    gw::build_static_view(900.0, 1300.0, src, 3, with);
    src.colour = nullptr;
    gw::build_static_view(900.0, 1300.0, src, 3, without);
    REQUIRE(with.size() == without.size());
    CHECK(calls == with.size());
    std::size_t road_vertices = 0;
    for (std::size_t i = 0; i < with.size(); ++i) {
        CHECK(without[i].rgba.empty());
        CHECK(with[i].rgba.size() == with[i].mesh.positions.size() / 3);
        for (auto px : with[i].rgba) road_vertices += px == 0xffffffffu;
    }
    CHECK(road_vertices > 0);
    CHECK(hash_chunks(with) == hash_chunks(without)); // the hook changes colours only
}

TEST_CASE("ViewStreamer cancels a build on destruction", "[world][view_streamer]") {
    Store store;
    gw::ViewStreamer::Options opt;
    opt.build_threads = 1;
    auto src = make_source<gw::ViewSource>(store, true, true, nullptr, 3000.0);
    {
        gw::ViewStreamer s(src, opt);
        s.update_focus(900.0, 1300.0);
        CHECK(s.phase() == gw::ViewStreamer::Phase::Building);
    } // joins without building the whole selection
    SUCCEED();
}
