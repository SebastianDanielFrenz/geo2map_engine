// g2m::world chunk road classes (include/g2m/world/chunk_road_classes.h;
// terrain migration plan phase 1, vault G2M-020).
//
// Ported from racing_game's tests/unit/test_road_classes.cpp (a8a1b13), plus a
// parity block: the class windows below were rasterised by the game's own
// rasterize_chunk_road_classes (road_classes.cpp compiled verbatim) from the
// road network in world_test_data.h, and their FNV hashes pinned here.
#include "g2m/world/chunk_road_classes.h"

#include "world_test_data.h"

#include "g2m/layer/classifier.h"

#include <catch2/catch_test_macros.hpp>

#include <cstdint>
#include <memory>
#include <vector>

namespace {

using g2m::geo::Hemisphere;
using g2m::geo::UtmZone;

constexpr UtmZone kZone32{32, Hemisphere::North};

// Ignores `key` entirely and always returns the same segment list.
std::shared_ptr<const std::vector<g2m::RoadSegment>> synthetic_road_lookup(void* ctx, const g2m::TileKey&) {
    return *static_cast<const std::shared_ptr<const std::vector<g2m::RoadSegment>>*>(ctx);
}

g2m::world::ClassLookup make_lookup(const std::shared_ptr<const std::vector<g2m::RoadSegment>>& segments) {
    return g2m::world::ClassLookup{&synthetic_road_lookup, const_cast<void*>(static_cast<const void*>(&segments))};
}

std::uint8_t at(const g2m::mesh::ClassWindow& w, int i, int j) {
    return w.c[static_cast<std::size_t>(j) * static_cast<std::size_t>(g2m::mesh::kChunkVerts) +
                static_cast<std::size_t>(i)];
}

constexpr std::uint8_t kUnknown = static_cast<std::uint8_t>(g2m::LandClass::Unknown);
constexpr std::uint8_t kPaved = static_cast<std::uint8_t>(g2m::LandClass::PavedRoad);
constexpr std::uint8_t kUnpaved = static_cast<std::uint8_t>(g2m::LandClass::UnpavedRoad);

// Two-tile lookup for the owner-rule test: answers differently depending on
// which g2m.src.osm tile is asked for (key.x == 0 vs. key.x == 1).
struct TwoTileLookupCtx {
    std::shared_ptr<const std::vector<g2m::RoadSegment>> tile0;
    std::shared_ptr<const std::vector<g2m::RoadSegment>> tile1;
};

std::shared_ptr<const std::vector<g2m::RoadSegment>> two_tile_road_lookup(void* ctx, const g2m::TileKey& key) {
    const auto* c = static_cast<const TwoTileLookupCtx*>(ctx);
    return key.x == 0 ? c->tile0 : c->tile1;
}

} // namespace

TEST_CASE("road_class_lattice: matches terrain_chunk.h's vertex-placement formula", "[world][chunk_road_classes]") {
    {
        const g2m::mesh::ChunkKey key{kZone32, 0, 0, 0};
        const g2m::Lattice lattice = g2m::world::road_class_lattice(key);
        CHECK(lattice.origin_e_mm == 500);
        CHECK(lattice.origin_n_mm == 500);
        CHECK(lattice.spacing_mm == 1000);
        CHECK(lattice.nx == g2m::mesh::kChunkVerts);
        CHECK(lattice.ny == g2m::mesh::kChunkVerts);
    }
    {
        // Level 2, cx=3, cy=-1: spacing = 4000 mm; origin = (64*cx*spacing + 500, 64*cy*spacing + 500).
        const g2m::mesh::ChunkKey key{kZone32, 2, 3, -1};
        const g2m::Lattice lattice = g2m::world::road_class_lattice(key);
        CHECK(lattice.spacing_mm == 4000);
        CHECK(lattice.origin_e_mm == 64 * 3 * 4000 + 500);
        CHECK(lattice.origin_n_mm == 64 * (-1) * 4000 + 500);
    }
}

TEST_CASE("src_osm_tiles_for_chunk: one tile fully inside, several when straddling a boundary", "[world][chunk_road_classes]") {
    // Level 0 chunk (0,0): footprint [0,64) x [0,64) m, well inside g2m.src.osm
    // tile (level 2, x=0, y=0) = [0, 1024) x [0, 1024) m.
    {
        const g2m::mesh::ChunkKey key{kZone32, 0, 0, 0};
        const std::vector<g2m::TileKey> tiles = g2m::world::src_osm_tiles_for_chunk(key);
        REQUIRE(tiles.size() == 1);
        CHECK(tiles[0].level == 2);
        CHECK(tiles[0].x == 0);
        CHECK(tiles[0].y == 0);
    }
    // Level 3 chunk (cx=1, cy=0): 512 m side, footprint [512, 1024) x [0, 512),
    // whose east edge sits exactly on the osm tile x=1024 boundary - the
    // inclusive floor_div(e_max, 1024) range therefore covers both x=0 and
    // x=1, y stays a single row.
    {
        const g2m::mesh::ChunkKey key{kZone32, 3, 1, 0};
        const std::vector<g2m::TileKey> tiles = g2m::world::src_osm_tiles_for_chunk(key);
        CHECK(tiles.size() == 2);
        for (const g2m::TileKey& t : tiles) {
            CHECK(t.level == 2);
            CHECK(t.y == 0);
        }
    }
}

TEST_CASE("rasterize_chunk_road_classes: a road segment appears at level 0", "[world][chunk_road_classes]") {
    // A short paved road along N = 32.5 m, E in [10, 54] m - covers the
    // level-0 chunk (0,0)'s vertex (32, 32) (absolute (32.5, 32.5) m, this
    // chunk's own vertex-placement formula) and stays well clear of the
    // chunk's far corners.
    auto segments = std::make_shared<const std::vector<g2m::RoadSegment>>(std::vector<g2m::RoadSegment>{
        g2m::RoadSegment{{10000, 32500}, {54000, 32500}, /*half_width_mm=*/2000, g2m::LandClass::PavedRoad,
                         g2m::SurfaceKind::Asphalt, /*rank=*/9},
    });
    const g2m::world::ClassLookup lookup = make_lookup(segments);

    const g2m::mesh::ChunkKey key{kZone32, 0, 0, 0};
    g2m::mesh::ClassWindow classes;
    g2m::world::rasterize_chunk_road_classes(key, lookup, classes);

    CHECK(at(classes, 32, 32) == kPaved);
    CHECK(at(classes, 0, 0) == kUnknown);
    CHECK(at(classes, 64, 64) == kUnknown);
}

TEST_CASE("rasterize_chunk_road_classes: the level filter works", "[world][chunk_road_classes]") {
    const g2m::RoadStyle& style = g2m::RoadStyle::default_style();
    const int residential_rank = style.find_highway("residential")->rank;
    const int tertiary_rank = style.find_highway("tertiary")->rank;
    const int primary_rank = style.find_highway("primary")->rank;
    REQUIRE(residential_rank < tertiary_rank);
    REQUIRE(tertiary_rank < primary_rank);

    // Every chunk below is (cx=0, cy=0) at its own level, and every level's
    // own vertex-placement formula puts SOME integer vertex exactly at
    // absolute (32.5, 32.5) m (level 0 -> (32,32), level 1 -> (16,16),
    // level 2 -> (8,8), level 3 -> (4,4)) - the same physical point the
    // segment below sits on, so every level's "does this road show up
    // here" check lands on an exact vertex, not a tolerance.
    auto make_segments_at_rank = [](int rank) {
        return std::make_shared<const std::vector<g2m::RoadSegment>>(std::vector<g2m::RoadSegment>{
            g2m::RoadSegment{{10000, 32500}, {54000, 32500}, 2000, g2m::LandClass::PavedRoad, g2m::SurfaceKind::Asphalt,
                             rank},
        });
    };

    auto vertex_for_level = [](int level) -> int {
        switch (level) {
            case 0: return 32;
            case 1: return 16;
            case 2: return 8;
            case 3: return 4;
            default: return 0;
        }
    };

    // Residential-rank road: visible at L0/L1, filtered out at L2/L3/L4.
    {
        auto segments = make_segments_at_rank(residential_rank);
        const g2m::world::ClassLookup lookup = make_lookup(segments);
        for (int level : {0, 1}) {
            g2m::mesh::ClassWindow classes;
            g2m::world::rasterize_chunk_road_classes(g2m::mesh::ChunkKey{kZone32, static_cast<std::uint8_t>(level), 0, 0}, lookup,
                                             classes);
            const int v = vertex_for_level(level);
            CHECK(at(classes, v, v) == kPaved);
        }
        for (int level : {2, 3, 4}) {
            g2m::mesh::ClassWindow classes;
            g2m::world::rasterize_chunk_road_classes(g2m::mesh::ChunkKey{kZone32, static_cast<std::uint8_t>(level), 0, 0}, lookup,
                                             classes);
            const int v = vertex_for_level(level);
            CHECK(at(classes, v, v) == kUnknown);
        }
    }

    // Tertiary-rank road: visible at L0/L1/L2, filtered out at L3/L4 (below
    // L3's own "primary and above" floor).
    {
        auto segments = make_segments_at_rank(tertiary_rank);
        const g2m::world::ClassLookup lookup = make_lookup(segments);
        for (int level : {0, 1, 2}) {
            g2m::mesh::ClassWindow classes;
            g2m::world::rasterize_chunk_road_classes(g2m::mesh::ChunkKey{kZone32, static_cast<std::uint8_t>(level), 0, 0}, lookup,
                                             classes);
            const int v = vertex_for_level(level);
            CHECK(at(classes, v, v) == kPaved);
        }
        g2m::mesh::ClassWindow classes3;
        g2m::world::rasterize_chunk_road_classes(g2m::mesh::ChunkKey{kZone32, 3, 0, 0}, lookup, classes3);
        CHECK(at(classes3, vertex_for_level(3), vertex_for_level(3)) == kUnknown);
    }

    // Primary-rank road: visible everywhere including L3.
    {
        auto segments = make_segments_at_rank(primary_rank);
        const g2m::world::ClassLookup lookup = make_lookup(segments);
        for (int level : {0, 1, 2, 3}) {
            g2m::mesh::ClassWindow classes;
            g2m::world::rasterize_chunk_road_classes(g2m::mesh::ChunkKey{kZone32, static_cast<std::uint8_t>(level), 0, 0}, lookup,
                                             classes);
            const int v = vertex_for_level(level);
            CHECK(at(classes, v, v) == kPaved);
        }
        // L4 is disabled outright, whatever the rank.
        g2m::mesh::ClassWindow classes4;
        g2m::world::rasterize_chunk_road_classes(g2m::mesh::ChunkKey{kZone32, 4, 0, 0}, lookup, classes4);
        CHECK(at(classes4, 0, 0) == kUnknown);
    }
}

TEST_CASE("rasterize_chunk_road_classes: owner rule - an equal-rank overlap at a tile boundary is decided by "
          "ownership, not lookup call order",
          "[world][chunk_road_classes]") {
    // Level 3 chunk (cx=1, cy=0): same boundary-straddling geometry as
    // "src_osm_tiles_for_chunk: ... several when straddling a boundary"
    // above - footprint [512, 1024) x [0, 512) m, straddling g2m.src.osm
    // tiles x=0 and x=1 (both y=0). spacing_mm = 8000 (8 m); origin_e_mm =
    // 1*64*8000 + 500 = 512500 (512.5 m); origin_n_mm = 500 (0.5 m) - so
    // vertex (i, j) sits at absolute E = 512.5 + 8*i m, N = 0.5 + 8*j m.
    const g2m::mesh::ChunkKey key{kZone32, 3, 1, 0};

    // Vertex i=64 (E = 1024.5 m) is owned by tile x=1 (floor_div(1024500,
    // 1024000) == 1); vertex i=63 (E = 1016.5 m) is owned by tile x=0
    // (floor_div(1016500, 1024000) == 0). j=32 (N = 256.5 m) for both - well
    // inside both tiles' shared y=0 row.
    //
    // Both tiles' own lists carry a segment AT THE SAME RANK covering BOTH
    // points (a 70 m-long strip straddling the boundary, well within either
    // tile's 128 m halo) - tile 0's is PavedRoad, tile 1's is UnpavedRoad. If
    // ownership were not the sole decider, the old cross-tile merge +
    // stable_sort-by-rank-alone (equal rank -> whichever list was
    // concatenated first, i.e. src_osm_tiles_for_chunk's own x-ascending
    // order, always tile 0) would paint tile 0's PavedRoad at BOTH points.
    // The owner rule must instead paint each point from only its own owner
    // tile's list.
    constexpr int kRank = 9;
    auto tile0_segments = std::make_shared<const std::vector<g2m::RoadSegment>>(std::vector<g2m::RoadSegment>{
        g2m::RoadSegment{{990000, 256500}, {1060000, 256500}, /*half_width_mm=*/2000, g2m::LandClass::PavedRoad,
                         g2m::SurfaceKind::Asphalt, kRank},
    });
    auto tile1_segments = std::make_shared<const std::vector<g2m::RoadSegment>>(std::vector<g2m::RoadSegment>{
        g2m::RoadSegment{{990000, 256500}, {1060000, 256500}, /*half_width_mm=*/2000, g2m::LandClass::UnpavedRoad,
                         g2m::SurfaceKind::Gravel, kRank},
    });
    TwoTileLookupCtx ctx{tile0_segments, tile1_segments};
    const g2m::world::ClassLookup lookup{&two_tile_road_lookup, &ctx};

    g2m::mesh::ClassWindow classes;
    g2m::world::rasterize_chunk_road_classes(key, lookup, classes);

    // Owned by tile 1: tile 1's own UnpavedRoad wins, not tile 0's
    // equal-rank PavedRoad.
    CHECK(at(classes, 64, 32) == kUnpaved);
    // Owned by tile 0: tile 0's own PavedRoad wins, not tile 1's equal-rank
    // UnpavedRoad - no cross-tile contamination either way.
    CHECK(at(classes, 63, 32) == kPaved);
}

TEST_CASE("rasterize_chunk_road_classes: a null lookup is byte-identical to all-Unknown", "[world][chunk_road_classes]") {
    auto segments = std::make_shared<const std::vector<g2m::RoadSegment>>(std::vector<g2m::RoadSegment>{
        g2m::RoadSegment{{10000, 32500}, {54000, 32500}, 2000, g2m::LandClass::PavedRoad, g2m::SurfaceKind::Asphalt, 9},
    });
    (void)segments; // deliberately unused: lookup.fn is null below, so nothing ever calls it

    const g2m::world::ClassLookup null_lookup{}; // fn == nullptr
    const g2m::mesh::ChunkKey key{kZone32, 0, 0, 0};

    g2m::mesh::ClassWindow classes;
    // Pre-fill with a non-zero pattern so the test cannot pass by accident
    // (a default-constructed ClassWindow is already all-zero).
    classes.c.fill(static_cast<std::uint8_t>(g2m::LandClass::TreeCover));

    g2m::world::rasterize_chunk_road_classes(key, null_lookup, classes);

    CHECK(classes.c == g2m::mesh::ClassWindow{}.c);
}

// ---- parity with racing_game's rasterize_chunk_road_classes -------------------

namespace {

struct ChunkPin {
    std::uint8_t level;
    std::int32_t cx, cy;
    std::size_t lookup_calls;
    int paved;
    int unpaved;
    std::uint64_t hash;
    std::size_t src_tiles;
    std::int64_t lat_e_mm, lat_n_mm, lat_spacing_mm;
};

// Produced by racing_game a8a1b13: road_classes.cpp compiled verbatim, the
// segments of world_test_data.h make_road_osm_tile() answered for the one
// g2m.src.osm tile kOsmKey (nothing for any other tile).
constexpr ChunkPin kChunkPins[] = {
    {0, 5601, 87203, 1, 328, 0, 0xb3c353ffe34da4f3ull, 1, 358464500, 5580992500, 1000},
    {0, 5603, 87203, 1, 650, 0, 0xb0636ce676635db1ull, 1, 358592500, 5580992500, 1000},
    {0, 5604, 87203, 1, 629, 0, 0x5089e164b8e1b164ull, 1, 358656500, 5580992500, 1000},
    {0, 5607, 87204, 1, 656, 0, 0x272837c173da0eabull, 1, 358848500, 5581056500, 1000},
    {0, 5608, 87207, 1, 390, 0, 0xa926777295f32e1dull, 1, 358912500, 5581248500, 1000},
    {0, 5610, 87207, 1, 372, 0, 0x3e66e5fa7feb7fcbull, 1, 359040500, 5581248500, 1000},
    {0, 5603, 87211, 1, 0, 51, 0xfdf5adf28494a6fdull, 1, 358592500, 5581504500, 1000},
    {0, 5609, 87214, 1, 358, 0, 0x359bb8d2c9d69da5ull, 1, 358976500, 5581696500, 1000},
    {0, 5612, 87214, 1, 286, 0, 0xd51217dad9fedb39ull, 1, 359168500, 5581696500, 1000},
    {0, 5599, 87203, 2, 0, 0, 0x4ec4415870aeb5dfull, 2, 358336500, 5580992500, 1000},
    {0, 5615, 87210, 2, 0, 0, 0x4ec4415870aeb5dfull, 2, 359360500, 5581440500, 1000},
    {0, 5400, 87000, 1, 0, 0, 0x4ec4415870aeb5dfull, 1, 345600500, 5568000500, 1000},
    {0, 5599, 87201, 2, 6, 0, 0xa358aecb1459ad1dull, 2, 358336500, 5580864500, 1000},
    {0, 5600, 87201, 1, 390, 0, 0x149c80256f60ed9dull, 1, 358400500, 5580864500, 1000},
    {1, 2800, 43601, 1, 83, 0, 0x06118f3871c9e418ull, 1, 358400500, 5580928500, 2000},
    {1, 2803, 43602, 1, 422, 0, 0x84894390164ba9bbull, 1, 358784500, 5581056500, 2000},
    {2, 1400, 21800, 1, 127, 0, 0x378395ca369c28caull, 1, 358400500, 5580800500, 4000},
    {3, 700, 10900, 1, 77, 0, 0x05dc90488cf60890ull, 1, 358400500, 5580800500, 8000},
    {4, 350, 5450, 0, 0, 0, 0x4ec4415870aeb5dfull, 4, 358400500, 5580800500, 16000},
};
constexpr std::uint64_t kAllChunksHash = 0x39a893b49953f4f3ull;
constexpr std::uint64_t kAllChunksBothTilesHash = 0x5b4cafb72be708f2ull;
constexpr std::uint64_t kAllUnknownHash = 0x4ec4415870aeb5dfull;

} // namespace

TEST_CASE("rasterize_chunk_road_classes matches racing_game on the shared road network",
          "[world][chunk_road_classes]") {
    using namespace g2m_world_test;
    RoadLookupCtx ctx;
    ctx.by_key[kOsmKey] = std::make_shared<const std::vector<g2m::RoadSegment>>(make_road_segments());
    REQUIRE(ctx.by_key[kOsmKey]->size() == 7);
    CHECK(hash_road_segments(*ctx.by_key[kOsmKey]) == 0xc7434ae4a95a6a09ull);

    const std::vector<g2m::mesh::ChunkKey> keys = class_chunk_keys();
    REQUIRE(keys.size() == std::size(kChunkPins));
    g2m::Fnv1a64 all;
    int painted_chunks = 0;
    for (std::size_t i = 0; i < keys.size(); ++i) {
        const g2m::mesh::ChunkKey& key = keys[i];
        const ChunkPin& pin = kChunkPins[i];
        INFO("chunk L" << int(key.level) << " (" << key.cx << "," << key.cy << ")");
        REQUIRE(key.level == pin.level);
        REQUIRE(key.cx == pin.cx);
        REQUIRE(key.cy == pin.cy);

        ctx.calls.clear();
        g2m::mesh::ClassWindow w;
        g2m::world::rasterize_chunk_road_classes(key, g2m::world::ClassLookup{&road_lookup_fn, &ctx}, w);
        int paved = 0;
        int unpaved = 0;
        for (const std::uint8_t v : w.c) {
            paved += v == kPaved;
            unpaved += v == kUnpaved;
        }
        CHECK(ctx.calls.size() == pin.lookup_calls);
        CHECK(paved == pin.paved);
        CHECK(unpaved == pin.unpaved);
        CHECK(hash_class_window(w) == pin.hash);
        all.u64(hash_class_window(w));
        painted_chunks += (paved + unpaved) > 0;

        CHECK(g2m::world::src_osm_tiles_for_chunk(key).size() == pin.src_tiles);
        const g2m::Lattice lat = g2m::world::road_class_lattice(key);
        CHECK(lat.origin_e_mm == pin.lat_e_mm);
        CHECK(lat.origin_n_mm == pin.lat_n_mm);
        CHECK(lat.spacing_mm == pin.lat_spacing_mm);
        CHECK(lat.nx == g2m::mesh::kChunkVerts);
        CHECK(lat.ny == g2m::mesh::kChunkVerts);
    }
    CHECK(all.value() == kAllChunksHash);
    CHECK(painted_chunks >= 12); // the pins are not trivially all-Unknown
}

TEST_CASE("rasterize_chunk_road_classes: the owner rule against the shared road network",
          "[world][chunk_road_classes]") {
    using namespace g2m_world_test;
    // A road crossing the tile edge E 358400 (way 106). With only the east tile
    // (350) known, the chunk straddling the edge paints just the samples tile
    // 350 owns (6 of them: the lattice column at E >= 358400); with the west
    // tile (349) known too, the same list paints the western samples as well.
    // Pins from the game's rasteriser.
    const auto segments = std::make_shared<const std::vector<g2m::RoadSegment>>(make_road_segments());
    RoadLookupCtx only_east;
    only_east.by_key[kOsmKey] = segments;
    RoadLookupCtx both;
    both.by_key[kOsmKey] = segments;
    both.by_key[g2m_test::home_tile(-1, 0)] = segments;

    const g2m::mesh::ChunkKey straddle{kZone, 0, 5599, 87201};
    g2m::mesh::ClassWindow east_only_w;
    g2m::world::rasterize_chunk_road_classes(straddle, g2m::world::ClassLookup{&road_lookup_fn, &only_east},
                                             east_only_w);
    g2m::mesh::ClassWindow both_w;
    g2m::world::rasterize_chunk_road_classes(straddle, g2m::world::ClassLookup{&road_lookup_fn, &both}, both_w);
    int east_n = 0;
    int both_n = 0;
    for (const std::uint8_t v : east_only_w.c) east_n += v != kUnknown;
    for (const std::uint8_t v : both_w.c) both_n += v != kUnknown;
    CHECK(east_n == 6);
    CHECK(both_n == 390);
    CHECK(hash_class_window(east_only_w) == 0xa358aecb1459ad1dull);
    CHECK(hash_class_window(both_w) == 0x149c80256f60ed9dull);

    g2m::Fnv1a64 all;
    for (const g2m::mesh::ChunkKey& key : class_chunk_keys()) {
        g2m::mesh::ClassWindow w;
        g2m::world::rasterize_chunk_road_classes(key, g2m::world::ClassLookup{&road_lookup_fn, &both}, w);
        all.u64(hash_class_window(w));
    }
    CHECK(all.value() == kAllChunksBothTilesHash);

    // A null lookup is all-Unknown with no calls.
    g2m::mesh::ClassWindow w;
    w.c.fill(static_cast<std::uint8_t>(g2m::LandClass::TreeCover));
    g2m::world::rasterize_chunk_road_classes(class_chunk_keys()[0], g2m::world::ClassLookup{}, w);
    CHECK(hash_class_window(w) == kAllUnknownHash);
}

TEST_CASE("src_osm_tiles_for_chunk: order is y ascending, then x ascending", "[world][chunk_road_classes]") {
    // A level-4 chunk (1024 m) on a tile corner touches a 2x2 block (shared
    // edges are inclusive).
    const std::vector<g2m::TileKey> tiles =
        g2m::world::src_osm_tiles_for_chunk(g2m::mesh::ChunkKey{g2m_world_test::kZone, 4, 350, 5450});
    REQUIRE(tiles.size() == 4);
    const g2m::TileKey expect[] = {
        {g2m_world_test::kZone, 2, 350, 5450},
        {g2m_world_test::kZone, 2, 351, 5450},
        {g2m_world_test::kZone, 2, 350, 5451},
        {g2m_world_test::kZone, 2, 351, 5451},
    };
    for (std::size_t i = 0; i < 4; ++i) CHECK(tiles[i] == expect[i]);
}
