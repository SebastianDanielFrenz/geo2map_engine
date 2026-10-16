// g2m::world chunk terrain classes (include/g2m/world/chunk_terrain_classes.h; street fidelity P2 slice 11, design 10.1,
// decision D62): the render ClassWindow read from a g2m.terrain.class tile. The served path (TerrainService +
// build_view_chunks) is in test_terrain_service.cpp.
#include "g2m/world/chunk_terrain_classes.h"

#include "g2m/layer/classifier.h"
#include "g2m/world/view_streamer.h"

#include <catch2/catch_test_macros.hpp>

#include <atomic>
#include <cstdint>
#include <memory>

using namespace g2m;
using namespace g2m::world;

namespace {

const geo::UtmZone kZone{32, geo::Hemisphere::North};

std::uint8_t at(const mesh::ClassWindow& w, int i, int j) {
    return w.c[static_cast<std::size_t>(j) * static_cast<std::size_t>(mesh::kChunkVerts) + static_cast<std::size_t>(i)];
}

// A class tile whose land_class encodes the node index (values 0..13, a different value for every neighbour in a
// 3 x 3 block) and whose rank plane is 0 (no road) everywhere.
TerrainClassTile pattern_tile(const TileKey& key) {
    TerrainClassTile t;
    t.reset(key);
    for (int j = 0; j < kTerrainClassSamples; ++j)
        for (int i = 0; i < kTerrainClassSamples; ++i)
            t.land_class[TerrainClassTile::index(i, j)] = static_cast<std::uint8_t>((i + 3 * j) % 14);
    return t;
}

std::uint8_t pattern(int i, int j) { return static_cast<std::uint8_t>((i + 3 * j) % 14); }

struct CountingCtx {
    std::shared_ptr<const TerrainClassTile> tile;
    std::atomic<int> calls{0};
    TileKey last{};
};

std::shared_ptr<const TerrainClassTile> counting_lookup(void* ctx, const TileKey& key) {
    auto* c = static_cast<CountingCtx*>(ctx);
    c->calls.fetch_add(1);
    c->last = key;
    return c->tile;
}

constexpr std::uint8_t kUnknown = static_cast<std::uint8_t>(LandClass::Unknown);
constexpr std::uint8_t kPaved = static_cast<std::uint8_t>(LandClass::PavedRoad);
constexpr std::uint8_t kForest = static_cast<std::uint8_t>(LandClass::TreeCover);

} // namespace

TEST_CASE("terrain_class_tile_for_chunk: four chunks per tile, floor for negative chunks", "[world][terrain_class_window]") {
    CHECK(terrain_class_tile_for_chunk({kZone, 0, 0, 0}) == TileKey{kZone, 0, 0, 0});
    CHECK(terrain_class_tile_for_chunk({kZone, 0, 3, 3}) == TileKey{kZone, 0, 0, 0});
    CHECK(terrain_class_tile_for_chunk({kZone, 0, 4, 7}) == TileKey{kZone, 0, 1, 1});
    CHECK(terrain_class_tile_for_chunk({kZone, 0, -1, -4}) == TileKey{kZone, 0, -1, -1});
    CHECK(terrain_class_tile_for_chunk({kZone, 0, -5, 2}) == TileKey{kZone, 0, -2, 0});
    CHECK(terrain_class_tile_for_chunk({kZone, 5, 9, -9}) == TileKey{kZone, 5, 2, -3});
}

TEST_CASE("terrain_class_window: each vertex takes the node at floor of its position", "[world][terrain_class_window]") {
    for (const int level : {0, 1, 3, 6}) {
        for (const int cx : {-5, -1, 0, 2, 3}) {
            for (const int cy : {-4, 1, 3}) {
                const mesh::ChunkKey key{kZone, static_cast<std::uint8_t>(level), cx, cy};
                const TileKey tk = terrain_class_tile_for_chunk(key);
                const TerrainClassTile tile = pattern_tile(tk);
                mesh::ClassWindow w;
                terrain_class_window(key, &tile, default_render_road_lod(), w);
                const std::int64_t spacing = std::int64_t{1} << level;
                const std::int64_t tile_e = static_cast<std::int64_t>(tk.x) * 256 * spacing;
                const std::int64_t tile_n = static_cast<std::int64_t>(tk.y) * 256 * spacing;
                int mismatches = 0;
                for (int j = 0; j < mesh::kChunkVerts; ++j)
                    for (int i = 0; i < mesh::kChunkVerts; ++i) {
                        // terrain_chunk.h: vertex (i, j) at E = (64 cx + i) 2^L + 0.5. Its south-west node, in
                        // integer arithmetic (floor of the half-metre position on the 2^L lattice of the tile).
                        const std::int64_t e2 = 2 * ((64 * static_cast<std::int64_t>(cx) + i) * spacing) + 1; // 2 E
                        const std::int64_t n2 = 2 * ((64 * static_cast<std::int64_t>(cy) + j) * spacing) + 1;
                        const std::int64_t ni = floor_div(e2 - 2 * tile_e, 2 * spacing);
                        const std::int64_t nj = floor_div(n2 - 2 * tile_n, 2 * spacing);
                        REQUIRE(ni >= 0);
                        REQUIRE(ni <= 256);
                        REQUIRE(nj >= 0);
                        REQUIRE(nj <= 256);
                        if (at(w, i, j) != pattern(static_cast<int>(ni), static_cast<int>(nj))) ++mismatches;
                    }
                INFO("level " << level << " cx " << cx << " cy " << cy);
                CHECK(mismatches == 0);
            }
        }
    }
    // The east chunk of a tile reads the tile's own east edge nodes (index 256).
    const mesh::ChunkKey east{kZone, 0, 3, 0};
    const TerrainClassTile tile = pattern_tile(terrain_class_tile_for_chunk(east));
    mesh::ClassWindow w;
    terrain_class_window(east, &tile, default_render_road_lod(), w);
    CHECK(at(w, 64, 0) == pattern(256, 0));
    CHECK(at(w, 64, 64) == pattern(256, 64));
    CHECK(at(w, 0, 0) == pattern(192, 0));
}

TEST_CASE("terrain_class_window: the render road LOD filters road nodes by rank", "[world][terrain_class_window]") {
    // Rank of the level policy (default: L0-L1 every road, L2 tertiary and above, L3 primary and above, L4+ none).
    const auto l2 = raster_params_for_render_level(2, default_render_road_lod());
    const auto l3 = raster_params_for_render_level(3, default_render_road_lod());
    REQUIRE(l2.has_value());
    REQUIRE(l3.has_value());
    REQUIRE(l3->min_rank > l2->min_rank);
    REQUIRE(l2->min_rank > 1);
    const std::uint8_t low = 1;                                            // a track or service road
    const std::uint8_t mid = static_cast<std::uint8_t>(l2->min_rank);      // tertiary
    const std::uint8_t high = static_cast<std::uint8_t>(l3->min_rank);     // primary

    for (const int level : {0, 1, 2, 3, 4, 6}) {
        const mesh::ChunkKey key{kZone, static_cast<std::uint8_t>(level), 0, 0};
        TerrainClassTile tile;
        tile.reset(terrain_class_tile_for_chunk(key));
        // Columns 0..3 of the chunk: forest (no road), low, mid and high rank roads.
        const std::uint8_t ranks[4] = {0, low, mid, high};
        for (int j = 0; j < kTerrainClassSamples; ++j)
            for (int i = 0; i < 4; ++i) {
                tile.land_class[TerrainClassTile::index(i, j)] = i == 0 ? kForest : kPaved;
                tile.rank[TerrainClassTile::index(i, j)] = ranks[i];
            }
        mesh::ClassWindow w;
        terrain_class_window(key, &tile, default_render_road_lod(), w);
        const auto params = raster_params_for_render_level(level, default_render_road_lod());
        INFO("level " << level);
        CHECK(at(w, 0, 5) == kForest); // never filtered
        for (int i = 1; i < 4; ++i) {
            const bool shown = params.has_value() && ranks[i] >= params->min_rank;
            CHECK(at(w, i, 5) == (shown ? kPaved : kUnknown));
        }
        if (level <= 1) CHECK(at(w, 1, 5) == kPaved);
        if (level == 2) CHECK(at(w, 1, 5) == kUnknown);
        if (level >= 4) CHECK(at(w, 3, 5) == kUnknown);

        // A caller policy: disabling the level hides every road, keeps the forest.
        RenderRoadLod off = default_render_road_lod();
        if (static_cast<std::size_t>(level) < off.levels.size()) {
            off.levels[static_cast<std::size_t>(level)].enabled = false;
            terrain_class_window(key, &tile, off, w);
            CHECK(at(w, 0, 5) == kForest);
            CHECK(at(w, 3, 5) == kUnknown);
        }
    }
}

TEST_CASE("terrain_class_window: absent or mismatched tiles give an all-Unknown window", "[world][terrain_class_window]") {
    const mesh::ChunkKey key{kZone, 0, 1, 2};
    mesh::ClassWindow w;
    w.c.fill(7);
    terrain_class_window(key, nullptr, default_render_road_lod(), w);
    for (const auto v : w.c) REQUIRE(v == kUnknown);

    const TerrainClassTile other = pattern_tile(TileKey{kZone, 0, 1, 0}); // not the chunk's tile
    w.c.fill(7);
    terrain_class_window(key, &other, default_render_road_lod(), w);
    for (const auto v : w.c) REQUIRE(v == kUnknown);

    TerrainClassTile empty;
    empty.key = terrain_class_tile_for_chunk(key); // planes not sized
    terrain_class_window(key, &empty, default_render_road_lod(), w);
    for (const auto v : w.c) REQUIRE(v == kUnknown);
}

TEST_CASE("fill_chunk_terrain_classes: one lookup of the chunk's tile; a null fn calls nothing", "[world][terrain_class_window]") {
    const mesh::ChunkKey key{kZone, 2, -3, 6};
    CountingCtx ctx;
    ctx.tile = std::make_shared<const TerrainClassTile>(pattern_tile(terrain_class_tile_for_chunk(key)));
    mesh::ClassWindow w;
    fill_chunk_terrain_classes(key, TerrainClassLookup{&counting_lookup, &ctx, nullptr}, w);
    CHECK(ctx.calls.load() == 1);
    CHECK(ctx.last == terrain_class_tile_for_chunk(key));
    mesh::ClassWindow direct;
    terrain_class_window(key, ctx.tile.get(), default_render_road_lod(), direct);
    CHECK(w.c == direct.c);

    w.c.fill(9);
    fill_chunk_terrain_classes(key, TerrainClassLookup{}, w);
    CHECK(ctx.calls.load() == 1);
    for (const auto v : w.c) REQUIRE(v == kUnknown);
}

TEST_CASE("build_view_chunks: terrain_classes takes precedence over the road class lookup", "[world][terrain_class_window]") {
    // A flat height tile so every vertex is valid; class ids end up in mesh.land_class.
    static HeightTile flat = [] {
        HeightTile h;
        h.key = TileKey{kZone, 0, 0, 0};
        h.h.fill(256 * 100);
        return h;
    }();
    struct Ctx {
        static const HeightTile* lookup(void*, const TileKey& k) { return k == flat.key ? &flat : nullptr; }
    };
    CountingCtx ctx;
    ctx.tile = std::make_shared<const TerrainClassTile>(pattern_tile(TileKey{kZone, 0, 0, 0}));
    ViewSource src;
    src.lookup = &Ctx::lookup;
    src.terrain_classes = TerrainClassLookup{&counting_lookup, &ctx, nullptr};
    // A road class lookup that must not be called while terrain_classes is set.
    static std::atomic<int> road_calls{0};
    src.class_lookup.fn = [](void*, const TileKey&) -> std::shared_ptr<const std::vector<RoadSegment>> {
        road_calls.fetch_add(1);
        return nullptr;
    };
    const std::vector<mesh::ChunkKey> keys{{kZone, 0, 1, 1}, {kZone, 0, 2, 1}};
    std::vector<ViewChunk> one, four;
    REQUIRE(build_view_chunks(keys, src, 1, one));
    REQUIRE(build_view_chunks(keys, src, 4, four));
    CHECK(road_calls.load() == 0);
    REQUIRE(one.size() == 2);
    for (std::size_t c = 0; c < keys.size(); ++c) {
        CHECK(one[c].mesh.content_hash == four[c].mesh.content_hash);
        mesh::ClassWindow w;
        terrain_class_window(keys[c], ctx.tile.get(), default_render_road_lod(), w);
        for (std::size_t v = 0; v < w.c.size(); ++v) REQUIRE(one[c].mesh.land_class[v] == w.c[v]);
    }
}
