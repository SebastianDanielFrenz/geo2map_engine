// src/mesh/lod_select.cpp - see g2m/mesh/lod_select.h.
#include "g2m/mesh/lod_select.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <unordered_set>

namespace g2m::mesh {
namespace {

struct Rect {
    double min_e, max_e, min_n, max_n;
};

// 2^level as an exact double (level is always small, well within double's
// exponent range) - no libm transcendental (pow/exp/log) anywhere here.
double step_m_for(int level) { return static_cast<double>(std::int64_t{1} << level); }

Rect chunk_rect(const ChunkKey& c) {
    const double step = step_m_for(c.level);
    const double origin_e = (static_cast<double>(kChunkQuads) * static_cast<double>(c.cx) + 0.5) * step;
    const double origin_n = (static_cast<double>(kChunkQuads) * static_cast<double>(c.cy) + 0.5) * step;
    const double size = static_cast<double>(kChunkQuads) * step;
    return {origin_e, origin_e + size, origin_n, origin_n + size};
}

// 2D distance from (e, n) to the closest point of rect (0 if inside).
double dist_to_rect(double e, double n, const Rect& r) {
    const double dx = std::max({r.min_e - e, 0.0, e - r.max_e});
    const double dy = std::max({r.min_n - n, 0.0, n - r.max_n});
    return std::sqrt(dx * dx + dy * dy);
}

// Packs (level, cx, cy) into a single key for O(1) set/map lookups - the
// packing itself never affects the OUTPUT order (select_chunks sorts
// explicitly at the end), only which chunk a hash-set lookup finds, so an
// unordered_set here carries none of the cross-platform iteration-order risk
// physics_sim's TileManager comment warns about.
std::uint64_t pack_key(std::uint8_t level, std::int32_t cx, std::int32_t cy) {
    constexpr std::int64_t kBias = std::int64_t{1} << 27;
    constexpr std::uint64_t kMask28 = 0x0FFFFFFFull;
    const std::uint64_t ucx = static_cast<std::uint64_t>(static_cast<std::int64_t>(cx) + kBias) & kMask28;
    const std::uint64_t ucy = static_cast<std::uint64_t>(static_cast<std::int64_t>(cy) + kBias) & kMask28;
    return (static_cast<std::uint64_t>(level) << 56) | (ucx << 28) | ucy;
}

// Recursively splits `chunk` toward the camera per LodParams::range0_m,
// appending final (unsplit) chunks to `leaves` - see lod_select.h's own
// algorithm comment.
void split_recursive(const ChunkKey& chunk, double cam_e, double cam_n, const LodParams& params,
                      std::vector<ChunkKey>& leaves) {
    if (chunk.level > 0) {
        const double d = dist_to_rect(cam_e, cam_n, chunk_rect(chunk));
        const double threshold = params.range0_m * step_m_for(chunk.level - 1);
        if (d < threshold) {
            const std::uint8_t child_level = static_cast<std::uint8_t>(chunk.level - 1);
            for (int dcy = 0; dcy < 2; ++dcy) {
                for (int dcx = 0; dcx < 2; ++dcx) {
                    split_recursive(ChunkKey{chunk.zone, child_level, 2 * chunk.cx + dcx, 2 * chunk.cy + dcy}, cam_e,
                                     cam_n, params, leaves);
                }
            }
            return;
        }
    }
    leaves.push_back(chunk);
}

} // namespace

void select_chunks(double cam_e, double cam_n, const LodParams& params, std::vector<ChunkKey>& out) {
    // Roots: level = params.max_level chunks within max_distance_m. The
    // candidate (cx, cy) range is computed approximately (chunk_size only,
    // ignoring the sub-chunk half-step origin offset) with a 2-chunk margin
    // on every side; every candidate is then re-checked against the real
    // rectangle distance, so the margin only has to be generous, not exact.
    const std::uint8_t root_level = static_cast<std::uint8_t>(params.max_level);
    const double root_step = step_m_for(root_level);
    const double root_chunk_size = static_cast<double>(kChunkQuads) * root_step;
    const auto cx_min =
        static_cast<std::int32_t>(std::floor((cam_e - params.max_distance_m) / root_chunk_size)) - 2;
    const auto cx_max =
        static_cast<std::int32_t>(std::ceil((cam_e + params.max_distance_m) / root_chunk_size)) + 2;
    const auto cy_min =
        static_cast<std::int32_t>(std::floor((cam_n - params.max_distance_m) / root_chunk_size)) - 2;
    const auto cy_max =
        static_cast<std::int32_t>(std::ceil((cam_n + params.max_distance_m) / root_chunk_size)) + 2;

    std::vector<ChunkKey> leaves;
    for (std::int32_t cy = cy_min; cy <= cy_max; ++cy) {
        for (std::int32_t cx = cx_min; cx <= cx_max; ++cx) {
            const ChunkKey root{params.zone, root_level, cx, cy};
            if (dist_to_rect(cam_e, cam_n, chunk_rect(root)) <= params.max_distance_m) {
                split_recursive(root, cam_e, cam_n, params, leaves);
            }
        }
    }

    // Restricted-quadtree 2:1 balance pass: repeat until no edge-adjacent
    // leaf pair differs by more than 1 level.
    constexpr int kDirs[4][2] = {{1, 0}, {-1, 0}, {0, 1}, {0, -1}};
    for (;;) {
        std::unordered_set<std::uint64_t> present;
        present.reserve(leaves.size() * 2);
        for (const ChunkKey& c : leaves) present.insert(pack_key(c.level, c.cx, c.cy));

        std::unordered_set<std::uint64_t> to_split_keys;
        std::vector<ChunkKey> to_split;
        for (const ChunkKey& leaf : leaves) {
            for (const auto& dir : kDirs) {
                const std::int32_t ncx = leaf.cx + dir[0];
                const std::int32_t ncy = leaf.cy + dir[1];
                if (present.contains(pack_key(leaf.level, ncx, ncy))) continue; // same-level neighbour present

                for (int d = 1; leaf.level + d <= params.max_level; ++d) {
                    const auto alevel = static_cast<std::uint8_t>(leaf.level + d);
                    const std::int32_t acx = ncx >> d;
                    const std::int32_t acy = ncy >> d;
                    const std::uint64_t key = pack_key(alevel, acx, acy);
                    if (present.contains(key)) {
                        if (d > 1 && to_split_keys.insert(key).second) {
                            to_split.push_back(ChunkKey{leaf.zone, alevel, acx, acy});
                        }
                        break; // found the (possibly coarser) ancestor covering this neighbour position
                    }
                }
                // No ancestor found at all: the neighbour position is covered
                // by finer leaves, which is this leaf's own would-be finer
                // neighbour - any imbalance there is caught from that side.
            }
        }

        if (to_split.empty()) break;

        for (const ChunkKey& coarse : to_split) {
            const std::uint64_t key = pack_key(coarse.level, coarse.cx, coarse.cy);
            leaves.erase(std::remove_if(leaves.begin(), leaves.end(),
                                         [&](const ChunkKey& c) { return pack_key(c.level, c.cx, c.cy) == key; }),
                         leaves.end());
            const auto child_level = static_cast<std::uint8_t>(coarse.level - 1);
            for (int dcy = 0; dcy < 2; ++dcy) {
                for (int dcx = 0; dcx < 2; ++dcx) {
                    leaves.push_back(ChunkKey{coarse.zone, child_level, 2 * coarse.cx + dcx, 2 * coarse.cy + dcy});
                }
            }
        }
    }

    std::sort(leaves.begin(), leaves.end(), [](const ChunkKey& a, const ChunkKey& b) {
        if (a.level != b.level) return a.level > b.level;
        if (a.cy != b.cy) return a.cy < b.cy;
        return a.cx < b.cx;
    });
    out = std::move(leaves);
}

} // namespace g2m::mesh
