// tests/mesh/test_lod_select.cpp - g2m::mesh::select_chunks (PLAN.md G2.3):
// over 10,000 seeded (SplitMix64) camera positions, checks the output is
// sorted (level desc, cy, cx) with no duplicate keys, restricted-quadtree
// 2:1-balanced, and never exceeds 400 chunks; over a smaller subsample (see
// the comment on kAreaCheckStride below) additionally checks the leaves'
// footprints exactly tile the root candidates' footprints (no overlap, exact
// area conservation).

#include "g2m/mesh/lod_select.h"

#include "splitmix64.h"

#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <cstdint>
#include <unordered_set>
#include <vector>

using namespace g2m;
using namespace g2m::mesh;
using geo::Hemisphere;
using geo::UtmZone;

namespace {
constexpr UtmZone k32N{32, Hemisphere::North};

std::uint64_t pack(std::uint8_t level, std::int32_t cx, std::int32_t cy) {
    constexpr std::int64_t kBias = std::int64_t{1} << 27;
    constexpr std::uint64_t kMask28 = 0x0FFFFFFFull;
    const std::uint64_t ucx = static_cast<std::uint64_t>(static_cast<std::int64_t>(cx) + kBias) & kMask28;
    const std::uint64_t ucy = static_cast<std::uint64_t>(static_cast<std::int64_t>(cy) + kBias) & kMask28;
    return (static_cast<std::uint64_t>(level) << 56) | (ucx << 28) | ucy;
}

// Independent (test-only) re-derivation of a chunk's world footprint, from
// the SAME publicly documented formula build_chunk uses for
// TerrainChunkMesh::origin (terrain_chunk.h: "origin is a half-integer
// multiple of 2^k") - not a copy of lod_select.cpp's own (internal, static)
// chunk_rect.
struct Rect {
    double min_e, max_e, min_n, max_n;
};
double step_m_for(int level) { return static_cast<double>(std::int64_t{1} << level); }
Rect footprint(const ChunkKey& c) {
    const double step = step_m_for(c.level);
    const double origin_e = (static_cast<double>(kChunkQuads) * c.cx + 0.5) * step;
    const double origin_n = (static_cast<double>(kChunkQuads) * c.cy + 0.5) * step;
    const double size = static_cast<double>(kChunkQuads) * step;
    return {origin_e, origin_e + size, origin_n, origin_n + size};
}
double dist_to_footprint(double e, double n, const Rect& r) {
    const double dx = std::max({r.min_e - e, 0.0, e - r.max_e});
    const double dy = std::max({r.min_n - n, 0.0, n - r.max_n});
    return std::sqrt(dx * dx + dy * dy);
}

// A leaf's footprint in level-0-chunk-equivalent integer units: a level-l
// chunk (cx, cy) covers [cx*2^l, (cx+1)*2^l) x [cy*2^l, (cy+1)*2^l) - exact
// (PLAN.md lod_select.h: "child index = 2*cx + {0,1}, 2*cy + {0,1}", so going
// up one level always doubles the covered range).
struct IRect {
    std::int64_t x0, x1, y0, y1;
};
IRect integer_rect(const ChunkKey& c) {
    const std::int64_t s = std::int64_t{1} << c.level;
    return {static_cast<std::int64_t>(c.cx) * s, (static_cast<std::int64_t>(c.cx) + 1) * s,
            static_cast<std::int64_t>(c.cy) * s, (static_cast<std::int64_t>(c.cy) + 1) * s};
}
bool overlaps(const IRect& a, const IRect& b) {
    return !(a.x1 <= b.x0 || b.x1 <= a.x0 || a.y1 <= b.y0 || b.y1 <= a.y0);
}

// 2:1 balance check on the OUTPUT set, independent of lod_select.cpp's own
// (internal) balance pass - any correctness check of this invariant needs an
// adjacency/ancestor walk of this same shape; this one only ever READS
// `leaves`, never decides how to split it.
bool is_balanced(const std::vector<ChunkKey>& leaves, int max_level) {
    std::unordered_set<std::uint64_t> present;
    present.reserve(leaves.size() * 2);
    for (const ChunkKey& c : leaves) present.insert(pack(c.level, c.cx, c.cy));

    constexpr int kDirs[4][2] = {{1, 0}, {-1, 0}, {0, 1}, {0, -1}};
    for (const ChunkKey& leaf : leaves) {
        for (const auto& dir : kDirs) {
            const std::int32_t ncx = leaf.cx + dir[0];
            const std::int32_t ncy = leaf.cy + dir[1];
            if (present.contains(pack(leaf.level, ncx, ncy))) continue;
            for (int d = 1; leaf.level + d <= max_level; ++d) {
                const auto alevel = static_cast<std::uint8_t>(leaf.level + d);
                if (present.contains(pack(alevel, ncx >> d, ncy >> d))) {
                    if (d > 1) return false;
                    break;
                }
            }
        }
    }
    return true;
}

// The area check below is O(n^2) per camera; run it on a stride of the full
// 10,000-camera sweep (still a few hundred cameras) rather than all of them,
// to keep this test well under the CI's non-[long] budget. The cheap checks
// (count, sort order, 2:1 balance) run on the FULL sweep.
constexpr int kCameraCount = 10000;
constexpr int kAreaCheckStride = 31;

} // namespace

TEST_CASE("select_chunks: sorted, balanced, bounded, area-exact over 10000 seeded cameras", "[mesh]") {
    LodParams params;
    params.zone = k32N;
    params.range0_m = 250.0;
    params.max_level = 6;
    params.max_distance_m = 6000.0;

    g2m_test::SplitMix64 rng(0xC0FFEE'2026'0926ull);

    for (int cam = 0; cam < kCameraCount; ++cam) {
        const double cam_e = rng.uniform(-40000.0, 40000.0);
        const double cam_n = rng.uniform(-40000.0, 40000.0);

        std::vector<ChunkKey> out;
        select_chunks(cam_e, cam_n, params, out);

        INFO("camera #" << cam << " e=" << cam_e << " n=" << cam_n);
        REQUIRE(out.size() <= 400);

        // Strictly sorted (level desc, cy, cx), no duplicate keys.
        for (std::size_t i = 1; i < out.size(); ++i) {
            const ChunkKey& a = out[i - 1];
            const ChunkKey& b = out[i];
            const bool strictly_less =
                (a.level != b.level) ? (a.level > b.level) : (a.cy != b.cy) ? (a.cy < b.cy) : (a.cx < b.cx);
            CHECK(strictly_less);
        }

        CHECK(is_balanced(out, params.max_level));

        if (cam % kAreaCheckStride == 0) {
            // Root candidates: level = max_level chunks within max_distance_m
            // of the camera (lod_select.h's own documented root criterion),
            // enumerated independently here over a generously margined range.
            const double root_step = step_m_for(params.max_level);
            const double root_size = static_cast<double>(kChunkQuads) * root_step;
            const auto cx_min = static_cast<std::int32_t>(std::floor((cam_e - params.max_distance_m) / root_size)) - 3;
            const auto cx_max = static_cast<std::int32_t>(std::ceil((cam_e + params.max_distance_m) / root_size)) + 3;
            const auto cy_min = static_cast<std::int32_t>(std::floor((cam_n - params.max_distance_m) / root_size)) - 3;
            const auto cy_max = static_cast<std::int32_t>(std::ceil((cam_n + params.max_distance_m) / root_size)) + 3;

            std::int64_t root_units = 0;
            for (std::int32_t cy = cy_min; cy <= cy_max; ++cy) {
                for (std::int32_t cx = cx_min; cx <= cx_max; ++cx) {
                    const ChunkKey root{k32N, static_cast<std::uint8_t>(params.max_level), cx, cy};
                    if (dist_to_footprint(cam_e, cam_n, footprint(root)) <= params.max_distance_m) {
                        root_units += std::int64_t{1} << (2 * params.max_level);
                    }
                }
            }

            std::int64_t leaf_units = 0;
            std::vector<IRect> rects;
            rects.reserve(out.size());
            for (const ChunkKey& c : out) {
                leaf_units += std::int64_t{1} << (2 * c.level);
                rects.push_back(integer_rect(c));
            }
            for (std::size_t i = 0; i < rects.size(); ++i) {
                for (std::size_t j = i + 1; j < rects.size(); ++j) {
                    CHECK_FALSE(overlaps(rects[i], rects[j]));
                }
            }
            CHECK(leaf_units == root_units);
        }
    }
}
