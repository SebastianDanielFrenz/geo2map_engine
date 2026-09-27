// Physics-side road painting and the owner-rule block helpers
// (RoadValueLut, paint_road_segments, paint_road_blocks/
// rasterize_road_blocks, raster_params_for_render_level) -
// g2m/layer/osm_roads.h, G2.5a-grip commit G-a.

#include "g2m/layer/osm_roads.h"

#include "alloc_counter.h"
#include "synthetic.h"

#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <span>
#include <vector>

using namespace g2m;

namespace {

// A tiny fixed-size (L2 key -> RoadBlockLookupResult) table for tests, linear
// scan (test sizes are a handful of tiles). Any key not added reports
// Missing (never None), so a test can tell "queried a key it should not
// have" apart from "correctly got no data" for a key that IS listed with
// RoadBlockStatus::None. Allocates (std::vector `queried`) - fine for every
// test here except the dedicated allocation-free one, which uses its own
// non-allocating context instead.
struct TestLookupCtx {
    static constexpr int kMaxEntries = 4;
    TileKey keys[kMaxEntries]{};
    RoadBlockLookupResult results[kMaxEntries]{};
    int count = 0;
    std::vector<TileKey> queried;

    void add(const TileKey& key, RoadBlockLookupResult result) {
        keys[count] = key;
        results[count] = result;
        ++count;
    }
};

RoadBlockLookupResult test_lookup(void* ctx_v, const TileKey& key) {
    auto* ctx = static_cast<TestLookupCtx*>(ctx_v);
    ctx->queried.push_back(key);
    for (int i = 0; i < ctx->count; ++i) {
        if (ctx->keys[i] == key) return ctx->results[i];
    }
    return RoadBlockLookupResult{RoadBlockStatus::Missing, {}};
}

std::int64_t llround_i64(double v) { return static_cast<std::int64_t>(std::llround(v)); }

} // namespace

TEST_CASE("osm paint: RoadValueLut index/at/by_land_class", "[layer][osm]") {
    CHECK(RoadValueLut::index(LandClass::Unknown, SurfaceKind::Unknown) == 0);
    CHECK(RoadValueLut::index(LandClass::PavedRoad, SurfaceKind::Unknown) == 16);
    CHECK(RoadValueLut::index(LandClass::PavedRoad, SurfaceKind::Asphalt) == 17);
    CHECK(RoadValueLut::index(LandClass::UnpavedRoad, SurfaceKind::Dirt) == 40); // 2*16 + 8

    const RoadValueLut lut =
        RoadValueLut::by_land_class(/*off_road=*/9, {{LandClass::PavedRoad, static_cast<std::uint16_t>(1)}, {LandClass::UnpavedRoad, static_cast<std::uint16_t>(2)}});
    // Every SurfaceKind slot of a listed LandClass is overwritten - the LUT
    // is keyed by LandClass alone (roads_plan.md's "unpaved by LandClass
    // only" default).
    for (int s = 0; s < 12; ++s) {
        CHECK(lut.at(LandClass::PavedRoad, static_cast<SurfaceKind>(s)) == 1);
        CHECK(lut.at(LandClass::UnpavedRoad, static_cast<SurfaceKind>(s)) == 2);
    }
    // Every other LandClass stays at off_road, whatever the surface.
    CHECK(lut.at(LandClass::Grassland, SurfaceKind::Unknown) == 9);
    CHECK(lut.at(LandClass::Water, SurfaceKind::Asphalt) == 9);
}

TEST_CASE("osm paint: paint_road_segments matches rasterize_road_segments through an identity LUT",
          "[layer][osm]") {
    // An identity LUT (v[i] == i) turns paint_road_segments' output back
    // into exactly the (land_class, surface) pair rasterize_road_segments
    // would have written - so on the SAME segments/lattice, the two must
    // agree sample for sample, proving the shared paint_loop template
    // behaves identically through both public entry points.
    RoadValueLut identity;
    for (std::size_t i = 0; i < identity.v.size(); ++i) identity.v[i] = static_cast<std::uint16_t>(i);

    g2m_test::SplitMix64 r(0xC0FFEE);
    std::vector<RoadSegment> segs;
    for (int n = 0; n < 40; ++n) {
        RoadSegment seg{};
        seg.a = {llround_i64(r.uniform(-20000, 20000)), llround_i64(r.uniform(-20000, 20000))};
        seg.b = {llround_i64(r.uniform(-20000, 20000)), llround_i64(r.uniform(-20000, 20000))};
        // Half-widths safely inside [710, 128000] mm: paint_road_segments'
        // own two clamps are then no-ops, so both functions make the exact
        // same coverage decision, not just the same LUT lookup.
        seg.half_width_mm = llround_i64(r.uniform(1000, 50000));
        seg.land_class = static_cast<LandClass>(g2m_test::below(r, 14));
        seg.surface = static_cast<SurfaceKind>(g2m_test::below(r, 12));
        seg.rank = static_cast<int>(g2m_test::below(r, 10));
        segs.push_back(seg);
    }

    const Lattice lattice{-25000, -25000, 1000, 51, 51};
    const std::size_t n = static_cast<std::size_t>(lattice.nx) * static_cast<std::size_t>(lattice.ny);
    std::vector<std::uint8_t> land_class(n, static_cast<std::uint8_t>(LandClass::Unknown));
    std::vector<std::uint8_t> surface(n, static_cast<std::uint8_t>(SurfaceKind::Unknown));
    rasterize_road_segments(segs, lattice, RasterParams{}, land_class, surface);

    // Pre-filled to match identity.at(Unknown, Unknown) == 0, so an
    // untouched sample already agrees with rasterize's own untouched value.
    std::vector<std::uint16_t> painted(n, 0);
    paint_road_segments(segs, lattice, RasterParams{}, identity, painted, /*row_stride=*/static_cast<std::size_t>(lattice.nx));

    for (std::size_t idx = 0; idx < n; ++idx) {
        const std::uint16_t expected =
            identity.at(static_cast<LandClass>(land_class[idx]), static_cast<SurfaceKind>(surface[idx]));
        CHECK(painted[idx] == expected);
    }
}

TEST_CASE("osm paint: row-stride sub-rectangle writes stay within their own region", "[layer][osm]") {
    RoadSegment seg{};
    seg.a = {0, 0};
    seg.b = {5000, 0};
    seg.half_width_mm = 2000;
    seg.land_class = LandClass::PavedRoad;
    seg.surface = SurfaceKind::Asphalt;
    seg.rank = 0;
    const std::vector<RoadSegment> segs{seg};

    const Lattice lattice{-3000, -3000, 1000, 11, 7};
    const RoadValueLut lut = RoadValueLut::by_land_class(0, {{LandClass::PavedRoad, static_cast<std::uint16_t>(42)}});

    // Reference: a contiguous buffer, row_stride == nx.
    std::vector<std::uint16_t> reference(static_cast<std::size_t>(lattice.nx) * static_cast<std::size_t>(lattice.ny), 0);
    paint_road_segments(segs, lattice, RasterParams{}, lut, reference, static_cast<std::size_t>(lattice.nx));

    // The same paint into one sub-rectangle of a larger, padded buffer, at a
    // nonzero row/column offset and a row_stride bigger than lattice.nx -
    // exactly the contract paint_road_blocks itself relies on.
    constexpr std::size_t kBigStride = 20;
    constexpr int kRowOffset = 3, kColOffset = 4;
    std::vector<std::uint16_t> big(kBigStride * static_cast<std::size_t>(kRowOffset + lattice.ny + 2), 0xBEEF);
    // Like rasterize_road_segments, paint only overwrites samples a segment
    // actually covers - an untouched sample keeps whatever the caller put
    // there. `reference` starts all-0, so the sub-rectangle of `big` must
    // too, or an untouched cell would mismatch (0xBEEF vs. 0) for a reason
    // that has nothing to do with row-stride/offset correctness.
    for (int j = 0; j < lattice.ny; ++j) {
        for (int i = 0; i < lattice.nx; ++i) {
            big[static_cast<std::size_t>(j + kRowOffset) * kBigStride + static_cast<std::size_t>(i + kColOffset)] = 0;
        }
    }
    paint_road_segments(segs, lattice, RasterParams{}, lut,
                         std::span<std::uint16_t>(big).subspan(static_cast<std::size_t>(kRowOffset) * kBigStride +
                                                                static_cast<std::size_t>(kColOffset)),
                         kBigStride);

    for (int j = 0; j < lattice.ny; ++j) {
        for (int i = 0; i < lattice.nx; ++i) {
            const std::uint16_t expected = reference[static_cast<std::size_t>(j) * static_cast<std::size_t>(lattice.nx) +
                                                      static_cast<std::size_t>(i)];
            const std::uint16_t got =
                big[static_cast<std::size_t>(j + kRowOffset) * kBigStride + static_cast<std::size_t>(i + kColOffset)];
            CHECK(got == expected);
        }
    }
    for (std::size_t idx = 0; idx < big.size(); ++idx) {
        const std::size_t row = idx / kBigStride, col = idx % kBigStride;
        const bool inside = row >= static_cast<std::size_t>(kRowOffset) &&
                            row < static_cast<std::size_t>(kRowOffset + lattice.ny) &&
                            col >= static_cast<std::size_t>(kColOffset) &&
                            col < static_cast<std::size_t>(kColOffset + lattice.nx);
        if (!inside) CHECK(big[idx] == 0xBEEF);
    }
}

TEST_CASE("osm paint: paint_road_blocks derives the owner L2 tile at block_level 0 via parent() twice",
          "[layer][osm]") {
    // Two adjacent level-0 (256 m) blocks straddling an L2 (1024 m)
    // boundary at E = 104 * 256 = 26624 m: x=103's owner is (103>>2)=25,
    // x=104's owner is (104>>2)=26 - genuinely different L2 tiles, so this
    // also exercises the "physics cell centre's L0 tile lies in exactly one
    // L2 core" derivation (roads_plan.md section 2), not just the partition
    // math. Level 2 itself (owner == block, 0 parent() calls) is exercised
    // by the tile-set-independence and negative-control tests below.
    const TileKey owner_103{g2m_test::kZone32N, 2, 25, 48};
    const TileKey owner_104{g2m_test::kZone32N, 2, 26, 48};
    RoadSegment seg_103{{26600000, 49999000}, {26650000, 49999000}, 20000, LandClass::PavedRoad, SurfaceKind::Asphalt, 5};
    RoadSegment seg_104{{26600000, 49999000}, {26650000, 49999000}, 20000, LandClass::UnpavedRoad, SurfaceKind::Dirt, 5};

    TestLookupCtx ctx;
    ctx.add(owner_103, {RoadBlockStatus::Segments, std::span<const RoadSegment>(&seg_103, 1)});
    ctx.add(owner_104, {RoadBlockStatus::Segments, std::span<const RoadSegment>(&seg_104, 1)});

    // E = 26623, 26624, 26625, 26626 m (single N row at 49999 m, well inside
    // both blocks' own y=195 span [49920, 50176) m): i=0 is block x=103,
    // i=1..3 are block x=104.
    const Lattice lattice{26623000, 49999000, 1000, 4, 1};
    const RoadValueLut lut = RoadValueLut::by_land_class(0, {{LandClass::PavedRoad, static_cast<std::uint16_t>(111)}, {LandClass::UnpavedRoad, static_cast<std::uint16_t>(222)}});
    std::vector<std::uint16_t> out(4, 0xFFFF);

    const RoadBlockStats stats =
        paint_road_blocks(lattice, /*block_level=*/0, g2m_test::kZone32N, RasterParams{}, lut, &test_lookup, &ctx, out, 4);

    CHECK(stats.segments_blocks == 2);
    CHECK(stats.none_blocks == 0);
    CHECK(stats.missing_blocks == 0);
    CHECK(out[0] == 111);
    CHECK(out[1] == 222);
    CHECK(out[2] == 222);
    CHECK(out[3] == 222);

    // lookup() is called exactly once per BLOCK, not once per sample.
    REQUIRE(ctx.queried.size() == 2);
    CHECK(ctx.queried[0] == owner_103);
    CHECK(ctx.queried[1] == owner_104);
}

TEST_CASE("osm paint: a block's own output does not depend on any other block's status or content",
          "[layer][osm]") {
    const TileKey owner_a{g2m_test::kZone32N, 2, 25, 48};
    const TileKey owner_b{g2m_test::kZone32N, 2, 26, 48};
    RoadSegment seg_a{{26600000, 50000000}, {26650000, 50000000}, 20000, LandClass::PavedRoad, SurfaceKind::Asphalt, 5};
    RoadSegment seg_b1{{26600000, 50000000}, {26650000, 50000000}, 20000, LandClass::UnpavedRoad, SurfaceKind::Dirt, 5};
    RoadSegment seg_b2{{26600000, 50000000}, {26650000, 50000000}, 20000, LandClass::UnpavedRoad, SurfaceKind::Gravel, 5};

    const Lattice lattice{26623000, 50000000, 1000, 4, 1}; // i=0 -> A (x=25), i=1..3 -> B (x=26)
    const RoadValueLut lut = RoadValueLut::by_land_class(0, {{LandClass::PavedRoad, static_cast<std::uint16_t>(111)}, {LandClass::UnpavedRoad, static_cast<std::uint16_t>(222)}});

    TestLookupCtx ctx1;
    ctx1.add(owner_a, {RoadBlockStatus::Segments, std::span<const RoadSegment>(&seg_a, 1)});
    ctx1.add(owner_b, {RoadBlockStatus::Segments, std::span<const RoadSegment>(&seg_b1, 1)});
    std::vector<std::uint16_t> out1(4, 0xFFFF);
    const RoadBlockStats stats1 =
        paint_road_blocks(lattice, 2, g2m_test::kZone32N, RasterParams{}, lut, &test_lookup, &ctx1, out1, 4);
    CHECK(stats1.segments_blocks == 2);

    // B's own segment CONTENT changes - A's own sample must not move.
    TestLookupCtx ctx2;
    ctx2.add(owner_a, {RoadBlockStatus::Segments, std::span<const RoadSegment>(&seg_a, 1)});
    ctx2.add(owner_b, {RoadBlockStatus::Segments, std::span<const RoadSegment>(&seg_b2, 1)});
    std::vector<std::uint16_t> out2(4, 0xFFFF);
    paint_road_blocks(lattice, 2, g2m_test::kZone32N, RasterParams{}, lut, &test_lookup, &ctx2, out2, 4);
    CHECK(out2[0] == out1[0]);

    // B reports None entirely - A still unaffected, B's own sub-rectangle
    // stays untouched (the caller's job to pre-fill).
    TestLookupCtx ctx3;
    ctx3.add(owner_a, {RoadBlockStatus::Segments, std::span<const RoadSegment>(&seg_a, 1)});
    ctx3.add(owner_b, {RoadBlockStatus::None, {}});
    std::vector<std::uint16_t> out3(4, 0xFFFF);
    const RoadBlockStats stats3 =
        paint_road_blocks(lattice, 2, g2m_test::kZone32N, RasterParams{}, lut, &test_lookup, &ctx3, out3, 4);
    CHECK(stats3.segments_blocks == 1);
    CHECK(stats3.none_blocks == 1);
    CHECK(out3[0] == out1[0]);
    CHECK(out3[1] == 0xFFFF);
    CHECK(out3[2] == 0xFFFF);
    CHECK(out3[3] == 0xFFFF);

    // A itself is Missing (not resident / load failed) - B is still painted
    // normally, A's own sub-rectangle is left untouched.
    TestLookupCtx ctx4;
    ctx4.add(owner_b, {RoadBlockStatus::Segments, std::span<const RoadSegment>(&seg_b1, 1)});
    std::vector<std::uint16_t> out4(4, 0xFFFF);
    const RoadBlockStats stats4 =
        paint_road_blocks(lattice, 2, g2m_test::kZone32N, RasterParams{}, lut, &test_lookup, &ctx4, out4, 4);
    CHECK(stats4.segments_blocks == 1);
    CHECK(stats4.missing_blocks == 1);
    CHECK(out4[0] == 0xFFFF);
    CHECK(out4[1] == 222);
}

TEST_CASE("osm paint: negative control - the owner rule removes the old cross-tile merge's order dependence",
          "[layer][osm]") {
    const TileKey owner_a{g2m_test::kZone32N, 2, 25, 48};
    const TileKey owner_b{g2m_test::kZone32N, 2, 26, 48};
    // Equal rank: rasterize_road_segments' own "later overwrites earlier"
    // rule means whichever segment paints LAST wins at any sample both
    // cover - exactly the tie a rank-only stable_sort cannot break.
    RoadSegment seg_a{{26600000, 50000000}, {26622000, 50000000}, 5000, LandClass::PavedRoad, SurfaceKind::Asphalt, 5};
    RoadSegment seg_b{{26630000, 50000000}, {26650000, 50000000}, 15000, LandClass::UnpavedRoad, SurfaceKind::Dirt, 5};

    const geom::PointMm p{26620000, 50000000};
    REQUIRE(floor_div(p.x / 1000, 1024) == owner_a.x); // P is inside A's own 1024 m core...
    // ...yet seg_b (owned by B, 10 m past the gap at its own half-width of
    // 15 m) geometrically reaches it too - a realistic "way close to the
    // border, kept by both tiles' own 128 m halo" case.
    const Lattice point_lattice{p.x, p.y, 1000, 1, 1};

    // --- OLD behaviour: concatenate every touched tile's own list, then one
    // rank-only stable_sort (racing_game's CURRENT rasterize_chunk_
    // road_classes, R/core/src/road_classes.cpp ~:93-114) - equal rank
    // means the sort is a no-op, so whichever tile happened to be
    // concatenated LAST wins, regardless of which tile actually OWNS P.
    const auto old_merge_winner = [&](std::vector<RoadSegment> merged) {
        std::stable_sort(merged.begin(), merged.end(),
                          [](const RoadSegment& x, const RoadSegment& y) { return x.rank < y.rank; });
        std::vector<std::uint8_t> lc(1, static_cast<std::uint8_t>(LandClass::Unknown));
        std::vector<std::uint8_t> sf(1, static_cast<std::uint8_t>(SurfaceKind::Unknown));
        rasterize_road_segments(merged, point_lattice, RasterParams{}, lc, sf);
        return static_cast<LandClass>(lc[0]);
    };
    const LandClass winner_ab = old_merge_winner({seg_a, seg_b}); // A's tile enumerated before B's
    const LandClass winner_ba = old_merge_winner({seg_b, seg_a}); // B's tile enumerated before A's
    CHECK(winner_ab == LandClass::UnpavedRoad); // B, concatenated last, wins
    CHECK(winner_ba == LandClass::PavedRoad);   // A, concatenated last, wins
    CHECK(winner_ab != winner_ba); // THE BUG: same data, different tile enumeration order, different answer

    // --- NEW behaviour: the owner rule. P is owned by A alone, so its value
    // is always A's own list - never a merge with B's.
    TestLookupCtx ctx;
    ctx.add(owner_a, {RoadBlockStatus::Segments, std::span<const RoadSegment>(&seg_a, 1)});
    ctx.add(owner_b, {RoadBlockStatus::Segments, std::span<const RoadSegment>(&seg_b, 1)});
    std::vector<std::uint8_t> lc(1, static_cast<std::uint8_t>(LandClass::Unknown));
    std::vector<std::uint8_t> sf(1, static_cast<std::uint8_t>(SurfaceKind::Unknown));
    rasterize_road_blocks(point_lattice, 2, g2m_test::kZone32N, RasterParams{}, &test_lookup, &ctx, lc, sf);
    CHECK(static_cast<LandClass>(lc[0]) == LandClass::PavedRoad);

    // Change B's list arbitrarily - including giving it a HIGHER rank, which
    // would dominate any rank-based merge - P's owner-rule answer still does
    // not move: it never even looks at B's list.
    RoadSegment seg_b_stronger = seg_b;
    seg_b_stronger.rank = 9;
    TestLookupCtx ctx2;
    ctx2.add(owner_a, {RoadBlockStatus::Segments, std::span<const RoadSegment>(&seg_a, 1)});
    ctx2.add(owner_b, {RoadBlockStatus::Segments, std::span<const RoadSegment>(&seg_b_stronger, 1)});
    std::vector<std::uint8_t> lc2(1, static_cast<std::uint8_t>(LandClass::Unknown));
    std::vector<std::uint8_t> sf2(1, static_cast<std::uint8_t>(SurfaceKind::Unknown));
    rasterize_road_blocks(point_lattice, 2, g2m_test::kZone32N, RasterParams{}, &test_lookup, &ctx2, lc2, sf2);
    CHECK(static_cast<LandClass>(lc2[0]) == LandClass::PavedRoad);
}

TEST_CASE("osm paint: clamps - 0.71 m minimum and the 128 m halo maximum", "[layer][osm]") {
    SECTION("minimum widens a thin segment to 710 mm") {
        RoadSegment seg{};
        seg.a = {0, 0};
        seg.b = {10000, 0};
        seg.half_width_mm = 50; // far thinner than 710 mm
        seg.land_class = LandClass::PavedRoad;
        seg.rank = 0;
        const std::vector<RoadSegment> segs{seg};

        // 3x3 grid: columns x in {4900, 5000, 5100} mm (all within the
        // segment's own [0, 10000] span, so every sample is a pure
        // perpendicular-distance case), rows y in {600, 700, 800} mm.
        const Lattice lattice{4900, 600, 100, 3, 3};
        std::vector<std::uint8_t> land_class(9, static_cast<std::uint8_t>(LandClass::Unknown));
        std::vector<std::uint8_t> surface(9, static_cast<std::uint8_t>(SurfaceKind::Unknown));
        rasterize_road_segments(segs, lattice, RasterParams{}, land_class, surface);
        // Unclamped: every row's own perpendicular distance (600/700/800)
        // exceeds the raw 50 mm half-width - nothing is covered at all.
        CHECK(std::all_of(land_class.begin(), land_class.end(),
                           [](std::uint8_t v) { return v == static_cast<std::uint8_t>(LandClass::Unknown); }));

        const RoadValueLut lut = RoadValueLut::by_land_class(0, {{LandClass::PavedRoad, static_cast<std::uint16_t>(7)}});
        std::vector<std::uint16_t> painted(9, 0);
        paint_road_segments(segs, lattice, RasterParams{}, lut, painted, 3);
        CHECK(painted[0 * 3 + 1] == 7); // y=600 <= 710: covered
        CHECK(painted[1 * 3 + 1] == 7); // y=700 <= 710: covered
        CHECK(painted[2 * 3 + 1] == 0); // y=800 >  710: still uncovered
    }

    SECTION("maximum caps half-width to the 128 m halo") {
        RoadSegment seg{};
        seg.a = {0, 0};
        seg.b = {10000, 0};
        seg.half_width_mm = 500000; // 500 m - far past the 128 m halo
        seg.land_class = LandClass::PavedRoad;
        seg.rank = 0;
        const std::vector<RoadSegment> segs{seg};

        // One sample, 200 m from the segment (x=5000 mm is within the
        // segment's own span, so this is the pure perpendicular case):
        // inside the raw 500 m half-width, outside the clamped 128 m one.
        const Lattice lattice{5000, 200000, 1000, 1, 1};

        std::vector<std::uint8_t> lc(1, static_cast<std::uint8_t>(LandClass::Unknown));
        std::vector<std::uint8_t> sf(1, static_cast<std::uint8_t>(SurfaceKind::Unknown));
        rasterize_road_segments(segs, lattice, RasterParams{}, lc, sf);
        CHECK(lc[0] == static_cast<std::uint8_t>(LandClass::PavedRoad)); // unclamped: 200 m <= 500 m

        const RoadValueLut lut = RoadValueLut::by_land_class(0, {{LandClass::PavedRoad, static_cast<std::uint16_t>(7)}});
        std::vector<std::uint16_t> painted(1, 0);
        paint_road_segments(segs, lattice, RasterParams{}, lut, painted, 1);
        CHECK(painted[0] == 0); // clamped to 128 m: 200 m is NOT covered
    }
}

TEST_CASE("osm paint: centre-inside-capsule implies at least one corner inside (the one-cell proof)",
          "[layer][osm]") {
    // A physics cell is exactly the render L0 quad (roads_plan.md section
    // 2): centre C and its 4 corners C + (+-500, +-500) mm sit on a common
    // 500 mm lattice. For half_width_mm >= 710 (paint_road_segments' own
    // minimum clamp), "C is inside the capsule" must imply "at least one of
    // the 4 corners is inside it too" - checked here directly against the
    // real sample_inside_segment decision (via rasterize_road_segments,
    // which shares paint_loop with paint_road_segments) over many random
    // capsules, rather than re-derived as a separate closed-form proof.
    constexpr std::int64_t kCx = 10000, kCy = 10000; // arbitrary origin, mm
    const Lattice lattice{kCx - 500, kCy - 500, 500, 3, 3};
    // Row-major index -> which of the 9 samples: 0,2,6,8 are corners; 4 is
    // the centre; 1,3,5,7 are edge midpoints (unused here).
    constexpr int kCentreIdx = 4;
    constexpr int kCornerIdx[4] = {0, 2, 6, 8};

    g2m_test::SplitMix64 r(0x0EEC0DE);
    int centre_inside_trials = 0;
    for (int trial = 0; trial < 2000; ++trial) {
        RoadSegment seg{};
        seg.a = {kCx + llround_i64(r.uniform(-4000, 4000)), kCy + llround_i64(r.uniform(-4000, 4000))};
        seg.b = {kCx + llround_i64(r.uniform(-4000, 4000)), kCy + llround_i64(r.uniform(-4000, 4000))};
        seg.half_width_mm = llround_i64(r.uniform(710, 5000)); // >= the paint minimum
        seg.land_class = LandClass::PavedRoad;
        seg.rank = 0;
        const std::vector<RoadSegment> segs{seg};

        std::vector<std::uint8_t> lc(9, static_cast<std::uint8_t>(LandClass::Unknown));
        std::vector<std::uint8_t> sf(9, static_cast<std::uint8_t>(SurfaceKind::Unknown));
        rasterize_road_segments(segs, lattice, RasterParams{}, lc, sf);

        if (lc[kCentreIdx] != static_cast<std::uint8_t>(LandClass::PavedRoad)) continue;
        ++centre_inside_trials;
        const bool any_corner_inside =
            std::any_of(std::begin(kCornerIdx), std::end(kCornerIdx),
                        [&](int idx) { return lc[idx] == static_cast<std::uint8_t>(LandClass::PavedRoad); });
        CHECK(any_corner_inside);
    }
    // The random generation must actually exercise the "centre inside" case
    // often enough for the check above to mean something.
    CHECK(centre_inside_trials > 200);
}

TEST_CASE("osm paint: paint_road_blocks/rasterize_road_blocks allocate nothing", "[layer][osm]") {
    // A non-allocating lookup context: plain members, no std::vector -
    // unlike TestLookupCtx above (used by every other test in this file,
    // where allocation is not being measured).
    struct AllocFreeCtx {
        TileKey key_a, key_b;
        RoadSegment seg_a, seg_b;
    };
    static const auto lookup = [](void* ctx_v, const TileKey& key) -> RoadBlockLookupResult {
        auto* ctx = static_cast<AllocFreeCtx*>(ctx_v);
        if (key == ctx->key_a) return {RoadBlockStatus::Segments, std::span<const RoadSegment>(&ctx->seg_a, 1)};
        if (key == ctx->key_b) return {RoadBlockStatus::Segments, std::span<const RoadSegment>(&ctx->seg_b, 1)};
        return {RoadBlockStatus::None, {}};
    };

    AllocFreeCtx ctx{};
    ctx.key_a = TileKey{g2m_test::kZone32N, 2, 25, 48};
    ctx.key_b = TileKey{g2m_test::kZone32N, 2, 26, 48};
    ctx.seg_a = RoadSegment{{26600000, 50000000}, {26650000, 50000000}, 20000,
                            LandClass::PavedRoad, SurfaceKind::Asphalt, 5};
    ctx.seg_b = RoadSegment{{26600000, 50000000}, {26650000, 50000000}, 20000,
                            LandClass::UnpavedRoad, SurfaceKind::Dirt, 5};

    const Lattice lattice{26623000, 50000000, 1000, 4, 1}; // i=0 -> A, i=1..3 -> B
    const RoadValueLut lut = RoadValueLut::by_land_class(0, {{LandClass::PavedRoad, static_cast<std::uint16_t>(1)}, {LandClass::UnpavedRoad, static_cast<std::uint16_t>(2)}});
    std::vector<std::uint16_t> out(4, 0);
    std::vector<std::uint8_t> land_class(4, 0), surface(4, 0);

    if (!g2m_test::kAllocCounterAvailable) {
        SKIP("allocation counting unavailable under a sanitizer build (G2M_SANITIZER_BUILD)");
    }
    {
        g2m_test::ScopedAllocCounter counter;
        const RoadBlockStats stats =
            paint_road_blocks(lattice, 2, g2m_test::kZone32N, RasterParams{}, lut, lookup, &ctx, out, 4);
        CHECK(stats.segments_blocks == 2);
        CHECK(counter.count() == 0);
    }
    {
        g2m_test::ScopedAllocCounter counter;
        const RoadBlockStats stats =
            rasterize_road_blocks(lattice, 2, g2m_test::kZone32N, RasterParams{}, lookup, &ctx, land_class, surface);
        CHECK(stats.segments_blocks == 2);
        CHECK(counter.count() == 0);
    }
}

TEST_CASE("osm paint: raster_params_for_render_level matches the moved racing_game policy", "[layer][osm]") {
    for (int level : {0, 1}) {
        const auto p = raster_params_for_render_level(level);
        REQUIRE(p);
        CHECK(p->min_rank == 0);
        CHECK(p->min_half_width_mm == 0);
    }
    const RoadStyle& style = RoadStyle::default_style();
    {
        const auto p = raster_params_for_render_level(2);
        REQUIRE(p);
        const HighwayStyleEntry* tertiary = style.find_highway("tertiary");
        REQUIRE(tertiary != nullptr);
        CHECK(p->min_rank == tertiary->rank);
        CHECK(p->min_half_width_mm == 0);
    }
    {
        const auto p = raster_params_for_render_level(3);
        REQUIRE(p);
        const HighwayStyleEntry* primary = style.find_highway("primary");
        REQUIRE(primary != nullptr);
        CHECK(p->min_rank == primary->rank);
        CHECK(p->min_half_width_mm == 6000); // 3/4 of (1 << 3) * 1000 = 8000 mm spacing
    }
    for (int level : {4, 5, 10}) {
        CHECK_FALSE(raster_params_for_render_level(level));
    }
}
