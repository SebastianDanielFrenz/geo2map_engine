// Reference generator for tests/world: runs racing_game a8a1b13's code (the
// functions extracted verbatim into game_ref.inc by extract_game_ref.sh, plus
// road_classes.cpp) over the shared inputs of world_test_data.h and prints the
// values pinned in test_*.cpp. Build/run: see tests/world/CMakeLists.txt.
#include "world_test_data.h"
#include "game_ref.inc"
#include "rg/road_classes.h"
#include <cstdio>
using namespace g2m_world_test;

static void print_hex(const char* name, std::uint64_t v) { std::printf("%-44s 0x%016llx\n", name, (unsigned long long)v); }

int main() {
    // ---- decode ----
    for (const DecodeCase& c : decode_cases()) {
        rg::HeightTileFetchResult r = rg::decode_height_tile_container(c.container, c.expect_layer, c.expect_key);
        std::printf("decode %-24s status=%d ", c.name.c_str(), (int)r.status);
        if (r.tile) print_hex("hash", hash_height_tile(*r.tile)); else std::printf("null\n");
    }
    // ---- road responses ----
    for (const RoadResponseCase& c : road_response_cases()) {
        rg::RoadSegmentsResult r = rg::decode_road_segments_response(c.response, c.expect_key, kZone);
        std::printf("road %-28s status=%d n=%lld ", c.name.c_str(), (int)r.status, r.segments ? (long long)r.segments->size() : -1ll);
        if (r.segments) print_hex("hash", hash_road_segments(*r.segments)); else std::printf("null\n");
    }
    // ---- bilinear ----
    for (int pass = 0; pass < 3; ++pass) {
        FieldSet set(pass == 0 ? 0 : 3);
        if (pass == 2) set.missing = {1001, 20000};
        rg::L0TileLookupFn fn = [&](const g2m::TileKey& k) { return set.lookup(k); };
        g2m_test::SplitMix64 r(0xB111EAF000ull + pass);
        g2m::Fnv1a64 h;
        int nulls = 0;
        const int N = 400;
        std::uint64_t first[4] = {};
        for (int i = 0; i < N; ++i) {
            const std::uint64_t a = r.next();
            const std::uint64_t b = r.next();
            const double x = static_cast<double>(a % 650000) / 1000.0 - 200.0;
            const double y = static_cast<double>(b % 650000) / 1000.0 - 200.0;
            const auto v = rg::sample_l0_height(fn, kZone, kE0, kN0, x, y);
            h.u8(v ? 1 : 0).u64(v ? bits_of(*v) : 0);
            if (!v) ++nulls;
            if (i < 4) first[i] = v ? bits_of(*v) : 0;
        }
        std::printf("bilinear pass %d nulls=%d\n", pass, nulls);
        print_hex("  hash", h.value());
        for (int i = 0; i < 4; ++i) print_hex("  first", first[i]);
        // fixed probe points
        const double pts[][2] = {{0, 0}, {0.5, 0.5}, {255.5, 255.5}, {255.99, 0.01}, {-0.001, -0.001}, {256.0, 100.25}, {100.25, 256.0}, {-255.5, -255.5}};
        for (auto& p : pts) {
            const auto v = rg::sample_l0_height(fn, kZone, kE0, kN0, p[0], p[1]);
            std::printf("  pt (%g,%g) -> %s ", p[0], p[1], v ? "ok" : "nullopt");
            print_hex("", v ? bits_of(*v) : 0);
        }
    }
    // ---- chunk road classes ----
    {
        RoadLookupCtx ctx;
        ctx.by_key[kOsmKey] = std::make_shared<const std::vector<g2m::RoadSegment>>(make_road_segments());
        std::printf("segments: %zu hash ", ctx.by_key[kOsmKey]->size());
        print_hex("", hash_road_segments(*ctx.by_key[kOsmKey]));
        g2m::Fnv1a64 all;
        for (const g2m::mesh::ChunkKey& key : class_chunk_keys()) {
            ctx.calls.clear();
            g2m::mesh::ClassWindow w;
            rg::rasterize_chunk_road_classes(key, rg::ClassLookup{&road_lookup_fn, &ctx}, w);
            int nz = 0, paved = 0, unpaved = 0;
            for (auto v : w.c) { nz += v != 0; paved += v == (int)g2m::LandClass::PavedRoad; unpaved += v == (int)g2m::LandClass::UnpavedRoad; }
            std::printf("chunk L%d (%d,%d) calls=%zu nonzero=%d paved=%d unpaved=%d ", key.level, key.cx, key.cy, ctx.calls.size(), nz, paved, unpaved);
            print_hex("hash", hash_class_window(w));
            all.u64(hash_class_window(w));
        }
        print_hex("chunk all", all.value());
        {
            // Owner rule: tile 349 (west neighbour) also known, vs. only 350.
            RoadLookupCtx both;
            both.by_key[kOsmKey] = ctx.by_key[kOsmKey];
            both.by_key[g2m_test::home_tile(-1, 0)] = ctx.by_key[kOsmKey];
            g2m::Fnv1a64 hb;
            for (const g2m::mesh::ChunkKey& key : class_chunk_keys()) {
                g2m::mesh::ClassWindow w;
                rg::rasterize_chunk_road_classes(key, rg::ClassLookup{&road_lookup_fn, &both}, w);
                hb.u64(hash_class_window(w));
                if (key.cx == 5599 || key.cx == 5600) {
                    int nz = 0;
                    for (auto v : w.c) nz += v != 0;
                    std::printf("both chunk L%d (%d,%d) nonzero=%d ", key.level, key.cx, key.cy, nz);
                    print_hex("hash", hash_class_window(w));
                }
            }
            print_hex("chunk all (349 and 350 known)", hb.value());
        }
        // null lookup
        g2m::mesh::ClassWindow w;
        rg::rasterize_chunk_road_classes(class_chunk_keys()[0], rg::ClassLookup{}, w);
        print_hex("null lookup", hash_class_window(w));
        // tile list + lattice
        for (const g2m::mesh::ChunkKey& key : class_chunk_keys()) {
            const auto tiles = rg::src_osm_tiles_for_chunk(key);
            const g2m::Lattice lat = rg::road_class_lattice(key);
            std::printf("L%d (%d,%d) tiles=%zu lat=%lld,%lld,%lld,%d,%d\n", key.level, key.cx, key.cy, tiles.size(), (long long)lat.origin_e_mm, (long long)lat.origin_n_mm, (long long)lat.spacing_mm, lat.nx, lat.ny);
        }
    }
    return 0;
}
