// tests/world/view_test_data.h - inputs, scenarios and hashes for the phase-3b view streamer pins (terrain migration
// plan phase 3b): chunk builds (build_view_chunks / build_static_view) and ViewStreamer diff sequences over a
// synthetic tile store.
//
// The pins in view_pins.inc were produced by racing_game's own code (5a3baed: world_terrain.cpp's view functions,
// terrain_view_streamer.cpp, terrain_stream.cpp's diff-order functions, lod_settings.cpp's changed_road_lod_levels,
// extracted verbatim by refgen/extract_view_ref.sh) run over exactly these inputs by refgen/refgen_view.cpp;
// test_view_streamer.cpp runs the ported g2m::world code over the same inputs. The scenarios are templates over the
// source/streamer types, which have the same member names in both (rg::TerrainViewSource / g2m::world::ViewSource,
// rg::TerrainViewStreamer / g2m::world::ViewStreamer). Chunk colours (rgba) are not hashed: they are the game's.
// Exceptions instead of Catch2 assertions (the reference generator links no Catch2).
#pragma once

#include "world_test_data.h"

#include "g2m/core/hash.h"
#include "g2m/core/tile_key.h"
#include "g2m/layer/height_tile.h"
#include "g2m/layer/osm_roads.h"
#include "g2m/mesh/lod_select.h"
#include "g2m/mesh/terrain_chunk.h"
#include "g2m/world/chunk_road_classes.h"

#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <map>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

namespace g2m_view_test {

using g2m_world_test::kZone;
using g2m::mesh::ChunkKey;

// Session origin: the road scene of world_test_data.h (kOsmKey, E 358400..359424, N 5580800..5581824) lies at
// session (400..1424, 800..1824).
inline constexpr double kE0 = 358000.0;
inline constexpr double kN0 = 5580000.0;

// Height tiles of any level, synthesised on first request (integer arithmetic only), cached; thread-safe. Level-0
// tiles whose south-west corner is at E >= 359680 are absent (nullptr, the coverage edge); a 40 m square of level-0
// samples is NoData. carve() is the carve state the ViewSource's carve function reports for a tile.
class Store {
public:
    const g2m::HeightTile* get(const g2m::TileKey& key) {
        std::lock_guard<std::mutex> lk(mutex_);
        auto it = tiles_.find(key);
        if (it != tiles_.end()) return it->second.get();
        std::unique_ptr<g2m::HeightTile> tile;
        const std::int64_t step = std::int64_t{1} << key.level;
        const std::int64_t sw_e = std::int64_t{key.x} * 256 * step, sw_n = std::int64_t{key.y} * 256 * step;
        if (!(key.level == 0 && sw_e >= 359680)) {
            tile = std::make_unique<g2m::HeightTile>(); // heap: 256 KiB
            tile->key = key;
            for (int row = 0; row < 256; ++row)
                for (int col = 0; col < 256; ++col) {
                    const std::int64_t e = sw_e + col * step, n = sw_n + row * step;
                    const std::int64_t dx = e - 358000, dy = n - 5580000;
                    std::int64_t h = 120 * 256 + dx * 3 - dy * 2 + ((dx * dy) / 4096) % 700 + (e * 13 + n * 7) % 97 - 48;
                    if (key.level == 0 && e >= 359000 && e < 359040 && n >= 5581400 && n < 5581440) {
                        h = g2m::kHeightNoData;
                        tile->has_nodata = true;
                    }
                    tile->h[static_cast<std::size_t>(row) * 256 + static_cast<std::size_t>(col)] = static_cast<std::int32_t>(h);
                }
        }
        return tiles_.emplace(key, std::move(tile)).first->second.get();
    }
    static std::uint8_t carve(const g2m::TileKey& key) {
        if (key.level > 2) return 0;
        const int v = static_cast<int>((((std::int64_t{key.x} * 3 + std::int64_t{key.y} * 5 + key.level) % 11) + 11) % 11);
        return v == 0 ? 2 : (v <= 4 ? 1 : 0);
    }

private:
    std::mutex mutex_;
    std::map<g2m::TileKey, std::unique_ptr<g2m::HeightTile>> tiles_;
};

inline const g2m::HeightTile* store_lookup(void* ctx, const g2m::TileKey& key) { return static_cast<Store*>(ctx)->get(key); }
inline std::uint8_t store_carve(void*, const g2m::TileKey& key) { return Store::carve(key); }

// Road segments of world_test_data.h's road scene for kOsmKey, none elsewhere (thread-safe: read only).
inline std::shared_ptr<const std::vector<g2m::RoadSegment>> scene_segments() {
    static const auto segs = std::make_shared<const std::vector<g2m::RoadSegment>>(g2m_world_test::make_road_segments());
    return segs;
}
inline std::shared_ptr<const std::vector<g2m::RoadSegment>> scene_road_fn(void*, const g2m::TileKey& key) {
    return key == g2m_world_test::kOsmKey ? scene_segments() : nullptr;
}

// A second render road LOD policy: level 0 drops roads below rank 6, level 1 has none, the rest as the default.
inline std::shared_ptr<const g2m::RenderRoadLod> coarse_road_lod() {
    auto p = std::make_shared<g2m::RenderRoadLod>(g2m::default_render_road_lod());
    if (p->levels.size() < 2) p->levels.resize(2);
    p->levels[0].min_rank = 6;
    p->levels[1].enabled = false;
    return p;
}

inline g2m::mesh::LodParams lod_params(double max_distance_m = 700.0) {
    g2m::mesh::LodParams p;
    p.zone = kZone;
    p.range0_m = 64.0;
    p.max_level = 3; // 64 / 128 / 256 / 512 m chunks
    p.max_distance_m = max_distance_m;
    return p;
}

// Fills the fields both source types share. classes: with the road scene's class lookup; carve: with store_carve.
template <class Source>
Source make_source(Store& store, bool classes, bool carve, std::shared_ptr<const g2m::RenderRoadLod> road_lod = nullptr,
                   double max_distance_m = 700.0) {
    Source s;
    s.params = lod_params(max_distance_m);
    s.e0 = kE0;
    s.n0 = kN0;
    s.lookup = &store_lookup;
    s.ctx = &store;
    if (classes) {
        s.class_lookup.fn = &scene_road_fn;
        s.class_lookup.lod = road_lod.get();
    }
    if (carve) s.carve_state = &store_carve;
    s.road_lod = std::move(road_lod);
    return s;
}

inline std::uint32_t f32_bits(float f) {
    std::uint32_t u;
    std::memcpy(&u, &f, sizeof u);
    return u;
}

inline void hash_key(g2m::Fnv1a64& h, const ChunkKey& k) {
    h.u8(static_cast<std::uint8_t>(k.zone.zone)).u8(k.zone.hemisphere == g2m::geo::Hemisphere::South ? 1 : 0).u8(k.level).i32(k.cx).i32(k.cy);
}

// key, carve, mesh (every member), origin_session; not rgba.
template <class Chunk> void hash_chunk(g2m::Fnv1a64& h, const Chunk& c) {
    hash_key(h, c.key);
    h.u8(static_cast<std::uint8_t>(c.carve));
    hash_key(h, c.mesh.key);
    for (double v : c.mesh.origin) h.f64(v);
    h.u64(c.mesh.positions.size());
    for (float v : c.mesh.positions) h.u32(f32_bits(v));
    for (float v : c.mesh.normals) h.u32(f32_bits(v));
    h.u64(c.mesh.land_class.size());
    if (!c.mesh.land_class.empty()) h.bytes(c.mesh.land_class.data(), c.mesh.land_class.size());
    h.u64(c.mesh.indices.size());
    for (auto i : c.mesh.indices) h.u16(i);
    h.u32(c.mesh.skirt_first_vertex).u32(f32_bits(c.mesh.skirt_depth_m)).u64(c.mesh.content_hash);
    for (double v : c.origin_session) h.f64(v);
}

template <class Chunk> std::uint64_t hash_chunks(const std::vector<Chunk>& chunks) {
    g2m::Fnv1a64 h;
    h.u64(chunks.size());
    for (const auto& c : chunks) hash_chunk(h, c);
    return h.value();
}

// One pinned row: a name, a count and a hash.
struct Pin {
    std::string name;
    std::uint64_t count;
    std::uint64_t hash;
};

template <class Diff> Pin diff_pin(const std::string& name, const Diff& d) {
    g2m::Fnv1a64 h;
    h.u64(d.serial).f64(d.focus_x).f64(d.focus_y).f64(d.velocity_x).f64(d.velocity_y);
    h.u64(hash_chunks(d.added));
    h.u64(d.removed.size());
    for (const auto& k : d.removed) hash_key(h, k);
    h.u64(hash_chunks(d.replaced));
    return {name, d.added.size() * 1000000 + d.removed.size() * 1000 + d.replaced.size(), h.value()};
}

template <class Streamer, class Diff> bool wait_diff(Streamer& s, Diff& out) {
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(120);
    while (std::chrono::steady_clock::now() < deadline) {
        if (s.poll(out)) return true;
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    return false;
}

// The streamer scenario: build_initial, then focus moves (with and without velocity), a road LOD change and a view
// distance change through set_source, each diff awaited, pinned and committed. `Options` has the same members in
// both streamers (reselect_distance_m, build_threads, initial_build_threads).
template <class Source, class Streamer, class Chunk, class Diff> std::vector<Pin> streamer_pins(unsigned build_threads) {
    Store store;
    std::vector<Pin> out;
    typename Streamer::Options opt;
    opt.reselect_distance_m = 128.0;
    opt.build_threads = build_threads;
    opt.initial_build_threads = build_threads;
    Streamer s(make_source<Source>(store, true, true), opt);
    std::vector<Chunk> initial;
    if (!s.build_initial(900.0, 1300.0, initial)) throw std::runtime_error("build_initial");
    out.push_back({"stream.initial", initial.size(), hash_chunks(initial)});
    const auto step = [&](const std::string& name) {
        Diff d;
        if (!wait_diff(s, d)) throw std::runtime_error(name + ": no diff");
        out.push_back(diff_pin(name, d));
        if (!s.commit(d.serial)) throw std::runtime_error(name + ": commit");
    };
    const auto idle = [&](const std::string& name) {
        if (s.phase() != Streamer::Phase::Idle) throw std::runtime_error(name + ": not idle");
        out.push_back({name, 0, 0});
    };
    s.update_focus(1000.0, 1300.0); // 100 m: below the reselect distance
    idle("stream.short_move");
    s.update_focus(1100.0, 1350.0);
    step("stream.move");
    s.update_focus(1300.0, 1450.0, 30.0, 10.0, 1280.0, 1440.0); // urgency order around the car
    step("stream.velocity");
    s.update_focus(1390.0, 1520.0, 60.0, 40.0); // 72 m/s: reselect distance 0.8 x 128 = 102.4 m < 114 m
    step("stream.fast");
    s.update_focus(1395.0, 1525.0, 60.0, 40.0);
    idle("stream.fast_short");
    s.set_source(make_source<Source>(store, true, true, coarse_road_lod()));
    step("stream.road_lod");
    s.set_source(make_source<Source>(store, true, true, coarse_road_lod(), 500.0));
    step("stream.view_distance");
    s.update_focus(600.0, 900.0, -20.0, -30.0);
    step("stream.back");
    const auto st = s.stats();
    g2m::Fnv1a64 h;
    h.u64(st.selections_started).u64(st.diffs_committed).u64(st.chunks_built).u64(st.chunks_reused).u64(st.last_added);
    h.u64(st.last_removed).u64(st.chunks_rebuilt).u64(st.resident_count);
    for (const auto& k : s.resident_keys()) hash_key(h, k);
    out.push_back({"stream.stats", st.resident_count, h.value()});
    return out;
}

// Chunk builds: `select(x, y, params, e0, n0, keys)` and `build(keys, source, threads, out)` are the two halves of the
// static view in either code base.
template <class Source, class Chunk, class Select, class Build> std::vector<Pin> build_pins(Select select, Build build) {
    Store store;
    std::vector<Pin> out;
    const auto view = [&](const std::string& name, const Source& src, double x, double y, unsigned threads) {
        std::vector<ChunkKey> keys;
        select(x, y, src.params, src.e0, src.n0, keys);
        std::vector<Chunk> chunks;
        build(keys, src, threads, chunks);
        out.push_back({name, chunks.size(), hash_chunks(chunks)});
    };
    view("static.plain", make_source<Source>(store, false, false), 900.0, 1300.0, 1);
    view("static.classes", make_source<Source>(store, true, false), 900.0, 1300.0, 3);
    view("static.carve", make_source<Source>(store, true, true), 900.0, 1300.0, 2);
    view("static.coarse_lod", make_source<Source>(store, true, true, coarse_road_lod()), 900.0, 1300.0, 4);
    view("static.edge", make_source<Source>(store, true, true, nullptr, 1500.0), 1500.0, 1700.0, 3);
    {
        // The explicit chunk keys of the road class tests (levels 0-4, the owner rule), built with classes and carve.
        const Source src = make_source<Source>(store, true, true);
        std::vector<Chunk> chunks;
        build(g2m_world_test::class_chunk_keys(), src, 2, chunks);
        out.push_back({"keys.class_chunks", chunks.size(), hash_chunks(chunks)});
    }
    return out;
}

// The diff-order helpers over a grid of keys, cars and velocities. `reach(key, e0, n0, px, py, vx, vy)`,
// `reselect(base, speed)`, `sort(chunks, e0, n0, px, py, vx, vy)` and `changed(a, b, max_level)`.
template <class Chunk, class Reach, class Reselect, class Sort, class Changed>
std::vector<Pin> order_pins(Reach reach, Reselect reselect, Sort sort, Changed changed) {
    std::vector<Pin> out;
    std::vector<ChunkKey> keys;
    for (std::uint8_t level = 0; level <= 3; ++level)
        for (int cy = -2; cy <= 2; ++cy)
            for (int cx = -2; cx <= 2; ++cx) {
                const int f = 1 << level;
                keys.push_back(ChunkKey{kZone, level, static_cast<std::int32_t>(358900 / (64 * f) + cx),
                                        static_cast<std::int32_t>(5581300 / (64 * f) + cy)});
            }
    const double vel[][2] = {{0, 0}, {1.5, 0.5}, {30, 10}, {-40, 25}, {0, -90}, {120, 0}};
    {
        g2m::Fnv1a64 h;
        std::uint64_t n = 0;
        for (const auto& v : vel)
            for (const auto& k : keys) {
                // Quantised to the microsecond: std::hypot (game) and det::hypot (port) may differ in the last bits.
                h.i64(static_cast<std::int64_t>(std::llround(reach(k, kE0, kN0, 900.0, 1300.0, v[0], v[1]) * 1e6)));
                ++n;
            }
        out.push_back({"order.time_to_reach", n, h.value()});
    }
    {
        g2m::Fnv1a64 h;
        std::uint64_t n = 0;
        for (double speed = 0.0; speed <= 130.0; speed += 6.5) {
            h.f64(reselect(128.0, speed));
            ++n;
        }
        out.push_back({"order.reselect", n, h.value()});
    }
    for (std::size_t vi = 0; vi < 6; ++vi) {
        std::vector<Chunk> chunks(keys.size());
        for (std::size_t i = 0; i < keys.size(); ++i) chunks[i].key = keys[(i * 37) % keys.size()];
        sort(chunks, kE0, kN0, 890.0, 1310.0, vel[vi][0], vel[vi][1]);
        g2m::Fnv1a64 h;
        for (const auto& c : chunks) hash_key(h, c.key);
        out.push_back({"order.sort." + std::to_string(vi), chunks.size(), h.value()});
    }
    {
        const auto coarse = coarse_road_lod();
        g2m::Fnv1a64 h;
        std::uint64_t n = 0;
        for (const auto& [a, b] : {std::pair<const g2m::RenderRoadLod*, const g2m::RenderRoadLod*>{nullptr, nullptr},
                                   {nullptr, coarse.get()}, {coarse.get(), nullptr}, {coarse.get(), coarse.get()}}) {
            const auto levels = changed(a, b, 6);
            h.u64(levels.size());
            for (int l : levels) h.i32(l);
            n += levels.size();
        }
        out.push_back({"order.changed_levels", n, h.value()});
    }
    return out;
}

} // namespace g2m_view_test
