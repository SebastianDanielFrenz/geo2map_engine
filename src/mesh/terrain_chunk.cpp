// src/mesh/terrain_chunk.cpp - see g2m/mesh/terrain_chunk.h.
#include "g2m/mesh/terrain_chunk.h"

#include "g2m/core/hash.h"

#include <bit>
#include <cmath>
#include <cstdlib>

namespace g2m::mesh {
namespace {

constexpr int kWin = kHaloWindow; // 67

// Window offset (0..66) for local chunk coordinate c (-1..65).
constexpr int woff(int c) { return c + 1; }

std::int32_t win_h(const HeightWindow& w, int i, int j) {
    return w.h[static_cast<std::size_t>(woff(j)) * kWin + static_cast<std::size_t>(woff(i))];
}

bool is_nodata(std::int32_t h) { return h == kHeightNoData; }

int base_index(int i, int j) { return j * kChunkVerts + i; }

// Skirt vertex order: west (i=0), east (i=64), south (j=0), north (j=64),
// each kChunkVerts entries in south-to-north / west-to-east order - see
// terrain_chunk.h's TerrainChunkMesh comment.
int skirt_index(int border, int k) { return kBaseVertexCount + border * kChunkVerts + k; }

struct Vec3f {
    float x, y, z;
};

// PLAN.md G2.3a: the fixed bilinear fraction, identical for EVERY vertex of
// a level-k (k >= 1) chunk - vertex i sits at E = (64*cx+i)*2^k + 0.5, i.e.
// at fraction t = 0.5 + 2^-(k+1) from level-k sample (64*cx+i-1) toward
// sample (64*cx+i) (same fraction for N from sample j-1 toward j) - see
// terrain_chunk.h's build_chunk comment. 2^-(k+1) is an exact dyadic double
// for every level this pyramid reaches (k <= kTerrainMaxLevel == 6).
double vertex_lattice_t(int level) {
    return 0.5 + 1.0 / static_cast<double>(std::int64_t{1} << (level + 1));
}

// The 2D bilinear height (in metres) and analytic gradient of the level-k
// (k >= 1) patch at fraction (t, t) between the 4 surrounding level-k
// samples h(i-1,j-1)/h(i,j-1)/h(i-1,j)/h(i,j) - both from the SAME 4 samples,
// so no halo beyond the existing 1-sample HeightWindow halo is needed (a
// vertex needing samples i-1/i is always within the window's local -1..65
// range for i in [0, kChunkQuads]). NoData (any of the 4 samples) is the
// caller's concern (nodata_out).
struct VertexHeightK1 {
    bool nodata;
    double z_m;
    double dzde_m;   // dz/dE, metres per metre
    double dzdn_m;   // dz/dN, metres per metre
};

VertexHeightK1 sample_vertex_k1(const HeightWindow& w, int i, int j, double t, double step_m) {
    const std::int32_t h00 = win_h(w, i - 1, j - 1);
    const std::int32_t h10 = win_h(w, i, j - 1);
    const std::int32_t h01 = win_h(w, i - 1, j);
    const std::int32_t h11 = win_h(w, i, j);
    if (is_nodata(h00) || is_nodata(h10) || is_nodata(h01) || is_nodata(h11)) {
        return {true, 0.0, 0.0, 0.0};
    }
    const double d00 = static_cast<double>(h00);
    const double d10 = static_cast<double>(h10);
    const double d01 = static_cast<double>(h01);
    const double d11 = static_cast<double>(h11);
    const double one_minus_t = 1.0 - t;
    // z(u, v) = (1-u)(1-v)*h00 + u(1-v)*h10 + (1-u)v*h01 + uv*h11, u=v=t here
    // (bilinear - exact for an affine height field). Units 1/256 m.
    const double z_units = one_minus_t * one_minus_t * d00 + t * one_minus_t * d10 + one_minus_t * t * d01 +
                            t * t * d11;
    // dz/du = (1-v)*(h10-h00) + v*(h11-h01); dz/dv = (1-u)*(h01-h00) + u*(h11-h10).
    // dz/dE = (dz/du) / step_m (u = (E-E0)/step_m); likewise dz/dN.
    const double dzdu_units = one_minus_t * (d10 - d00) + t * (d11 - d01);
    const double dzdv_units = one_minus_t * (d01 - d00) + t * (d11 - d10);
    return {false, z_units / 256.0, dzdu_units / (256.0 * step_m), dzdv_units / (256.0 * step_m)};
}

Vec3f compute_normal(const HeightWindow& w, int i, int j, double step_m) {
    const std::int32_t hc = win_h(w, i, j);
    const std::int32_t hw = win_h(w, i - 1, j);
    const std::int32_t he = win_h(w, i + 1, j);
    const std::int32_t hs = win_h(w, i, j - 1);
    const std::int32_t hn = win_h(w, i, j + 1);
    const bool c_ok = !is_nodata(hc);
    const bool w_ok = !is_nodata(hw);
    const bool e_ok = !is_nodata(he);
    const bool s_ok = !is_nodata(hs);
    const bool n_ok = !is_nodata(hn);

    const float s = static_cast<float>(step_m);
    float dzdx = 0.0f;
    if (w_ok && e_ok) {
        dzdx = (static_cast<float>(he) - static_cast<float>(hw)) / 256.0f / (2.0f * s);
    } else if (e_ok && c_ok) {
        dzdx = (static_cast<float>(he) - static_cast<float>(hc)) / 256.0f / s;
    } else if (w_ok && c_ok) {
        dzdx = (static_cast<float>(hc) - static_cast<float>(hw)) / 256.0f / s;
    }
    float dzdy = 0.0f;
    if (s_ok && n_ok) {
        dzdy = (static_cast<float>(hn) - static_cast<float>(hs)) / 256.0f / (2.0f * s);
    } else if (n_ok && c_ok) {
        dzdy = (static_cast<float>(hn) - static_cast<float>(hc)) / 256.0f / s;
    } else if (s_ok && c_ok) {
        dzdy = (static_cast<float>(hc) - static_cast<float>(hs)) / 256.0f / s;
    }

    Vec3f n{-dzdx, -dzdy, 1.0f};
    const float len = std::sqrt(n.x * n.x + n.y * n.y + n.z * n.z);
    n.x /= len;
    n.y /= len;
    n.z /= len;
    return n;
}

// Worst 2:1-neighbour crack (PLAN.md G2.3): for one border's 65 samples,
// max_i |h[2i+1] - (h[2i]+h[2i+2])/2| over i in [0,31], computed in int64 so
// the comparison is exact; a pair touching NoData does not contribute.
// Returns the running max of 2*|h[2i+1] - (h[2i]+h[2i+2])/2| (units 1/256 m),
// i.e. |2*h[2i+1] - h[2i] - h[2i+2]|, so the /2 never needs to round.
void accumulate_border_crack(const HeightWindow& w, bool along_i, int fixed, std::int64_t& worst2) {
    for (int k = 0; k < kChunkQuads / 2; ++k) {
        const int a = 2 * k, b = 2 * k + 1, c = 2 * k + 2;
        const std::int32_t ha = along_i ? win_h(w, a, fixed) : win_h(w, fixed, a);
        const std::int32_t hb = along_i ? win_h(w, b, fixed) : win_h(w, fixed, b);
        const std::int32_t hc = along_i ? win_h(w, c, fixed) : win_h(w, fixed, c);
        if (is_nodata(ha) || is_nodata(hb) || is_nodata(hc)) continue;
        const std::int64_t diff2 = 2 * static_cast<std::int64_t>(hb) - static_cast<std::int64_t>(ha) - static_cast<std::int64_t>(hc);
        const std::int64_t a2 = diff2 < 0 ? -diff2 : diff2;
        if (a2 > worst2) worst2 = a2;
    }
}

// Level >= 1 counterpart of accumulate_border_crack: same worst-case
// T-junction gap (a level-(k+1) neighbour's edge only has every OTHER of
// this border's vertices, so its edge is a straight line between them; the
// in-between vertex's vertical deviation from that line is the gap the
// skirt must hide), but measured on the ACTUAL rendered (bilinearly
// interpolated) vertex heights in metres, not on raw pyramid samples - see
// terrain_chunk.h's build_chunk comment and PLAN.md G2.3a's skirt-depth
// argument (this file's own header / the G2.3a commit message). A pair
// touching a NoData vertex does not contribute, same as the level-0 form.
void accumulate_border_crack_k1(const HeightWindow& w, bool along_i, int fixed, double t, double step_m,
                                 double& worst_gap_m) {
    for (int k = 0; k < kChunkQuads / 2; ++k) {
        const int a = 2 * k, b = 2 * k + 1, c = 2 * k + 2;
        const VertexHeightK1 va = along_i ? sample_vertex_k1(w, a, fixed, t, step_m) : sample_vertex_k1(w, fixed, a, t, step_m);
        const VertexHeightK1 vb = along_i ? sample_vertex_k1(w, b, fixed, t, step_m) : sample_vertex_k1(w, fixed, b, t, step_m);
        const VertexHeightK1 vc = along_i ? sample_vertex_k1(w, c, fixed, t, step_m) : sample_vertex_k1(w, fixed, c, t, step_m);
        if (va.nodata || vb.nodata || vc.nodata) continue;
        const double gap = std::fabs(vb.z_m - 0.5 * (va.z_m + vc.z_m));
        if (gap > worst_gap_m) worst_gap_m = gap;
    }
}

void hash_f32(Fnv1a64& h, float v) { h.u32(std::bit_cast<std::uint32_t>(v)); }

constexpr int kMaxBaseIndices = kChunkQuads * kChunkQuads * 2 * 3;      // 24576
constexpr int kMaxSkirtIndices = 4 * kChunkQuads * 2 * 3;               // 1536
constexpr int kMaxIndices = kMaxBaseIndices + kMaxSkirtIndices;         // 26112

} // namespace

void gather_window(const ChunkKey& key, TileLookup lookup, void* ctx, HeightWindow& out) {
    const std::int64_t base_i = static_cast<std::int64_t>(kChunkQuads) * key.cx - 1;
    const std::int64_t base_j = static_cast<std::int64_t>(kChunkQuads) * key.cy - 1;

    struct CacheEntry {
        std::int64_t tx = 0, ty = 0;
        const HeightTile* tile = nullptr;
        bool present = false;
    };
    std::array<CacheEntry, 9> cache{};
    int cache_n = 0;

    auto tile_for = [&](std::int64_t tx, std::int64_t ty) -> const HeightTile* {
        for (int c = 0; c < cache_n; ++c) {
            if (cache[c].tx == tx && cache[c].ty == ty) return cache[c].tile;
        }
        const TileKey tk{key.zone, key.level, static_cast<std::int32_t>(tx), static_cast<std::int32_t>(ty)};
        const HeightTile* tile = lookup(ctx, tk);
        if (cache_n < static_cast<int>(cache.size())) {
            cache[static_cast<std::size_t>(cache_n)] = CacheEntry{tx, ty, tile, true};
            ++cache_n;
        }
        return tile;
    };

    for (int oj = 0; oj < kWin; ++oj) {
        const std::int64_t aj = base_j + oj;
        const std::int64_t ty = floor_div(aj, kTerrainTileSamples);
        const std::int64_t lj = aj - kTerrainTileSamples * ty;
        for (int oi = 0; oi < kWin; ++oi) {
            const std::int64_t ai = base_i + oi;
            const std::int64_t tx = floor_div(ai, kTerrainTileSamples);
            const std::int64_t li = ai - kTerrainTileSamples * tx;
            const HeightTile* tile = tile_for(tx, ty);
            const std::size_t idx = static_cast<std::size_t>(oj) * kWin + static_cast<std::size_t>(oi);
            out.h[idx] = tile ? tile->h[static_cast<std::size_t>(lj) * kTerrainTileSamples + static_cast<std::size_t>(li)]
                               : kHeightNoData;
        }
    }
}

void build_chunk(const ChunkKey& key, const HeightWindow& window, const ClassWindow* classes, TerrainChunkMesh& out) {
    out.key = key;
    const double step_m = static_cast<double>(std::int64_t{1} << key.level);
    // PLAN.md G2.3a (nesting fix): origin is vertex (0,0)'s absolute position,
    // which for EVERY level is an L0 sample centre - a CONSTANT +0.5 (not
    // +0.5*step_m, the old formula), so a level-(k+1) chunk's own origin
    // exactly matches its SW child's origin. Level 0 is unchanged (step_m=1,
    // so +0.5 == the old +0.5*step_m).
    out.origin[0] = (static_cast<double>(kChunkQuads) * static_cast<double>(key.cx)) * step_m + 0.5;
    out.origin[1] = (static_cast<double>(kChunkQuads) * static_cast<double>(key.cy)) * step_m + 0.5;
    out.origin[2] = 0.0;

    // Constant-size arrays every call - resize to the same size never
    // reallocates once the vector has that capacity (PLAN.md G2.3:
    // "build_chunk does not allocate once out has capacity").
    out.positions.resize(static_cast<std::size_t>(kTotalVertexCount) * 3);
    out.normals.resize(static_cast<std::size_t>(kTotalVertexCount) * 3);
    out.land_class.resize(static_cast<std::size_t>(kTotalVertexCount));
    out.indices.reserve(static_cast<std::size_t>(kMaxIndices));
    out.indices.resize(static_cast<std::size_t>(kMaxIndices));

    std::array<bool, kBaseVertexCount> nodata{};
    // level >= 1 only: the fixed bilinear fraction (same for every vertex,
    // see vertex_lattice_t's comment) - computed once, not per vertex.
    const double t = key.level > 0 ? vertex_lattice_t(key.level) : 0.0;

    // Base 65x65 grid: positions, normals, land_class, NoData flags. Level 0
    // is untouched (vertex IS the sample, PLAN.md G2.3a: "L0 stays exactly
    // as it is" - every L0 content_hash is unchanged by this fix); level
    // k >= 1 bilinearly interpolates the level-k pyramid (see
    // terrain_chunk.h's build_chunk comment).
    for (int j = 0; j < kChunkVerts; ++j) {
        for (int i = 0; i < kChunkVerts; ++i) {
            const int vi = base_index(i, j);
            float z;
            Vec3f n;
            if (key.level == 0) {
                const std::int32_t h = win_h(window, i, j);
                nodata[static_cast<std::size_t>(vi)] = is_nodata(h);
                z = static_cast<float>(h) / 256.0f;
                n = compute_normal(window, i, j, step_m);
            } else {
                const VertexHeightK1 vh = sample_vertex_k1(window, i, j, t, step_m);
                nodata[static_cast<std::size_t>(vi)] = vh.nodata;
                if (vh.nodata) {
                    z = static_cast<float>(kHeightNoData) / 256.0f;
                    n = Vec3f{0.0f, 0.0f, 1.0f};
                } else {
                    z = static_cast<float>(vh.z_m);
                    n = Vec3f{static_cast<float>(-vh.dzde_m), static_cast<float>(-vh.dzdn_m), 1.0f};
                    const float len = std::sqrt(n.x * n.x + n.y * n.y + n.z * n.z);
                    n.x /= len;
                    n.y /= len;
                    n.z /= len;
                }
            }

            const float x = static_cast<float>(i) * static_cast<float>(step_m);
            const float y = static_cast<float>(j) * static_cast<float>(step_m);
            out.positions[static_cast<std::size_t>(vi) * 3 + 0] = x;
            out.positions[static_cast<std::size_t>(vi) * 3 + 1] = y;
            out.positions[static_cast<std::size_t>(vi) * 3 + 2] = z;

            out.normals[static_cast<std::size_t>(vi) * 3 + 0] = n.x;
            out.normals[static_cast<std::size_t>(vi) * 3 + 1] = n.y;
            out.normals[static_cast<std::size_t>(vi) * 3 + 2] = n.z;

            out.land_class[static_cast<std::size_t>(vi)] =
                classes ? classes->c[static_cast<std::size_t>(j) * kChunkVerts + static_cast<std::size_t>(i)] : 0;
        }
    }

    // Skirt depth (PLAN.md G2.3, extended by G2.3a for level >= 1): 1.0 m +
    // the worst 2:1-neighbour T-junction gap over all 4 borders. Level 0 is
    // the ORIGINAL formula, unchanged, in int64 (accumulate_border_crack) -
    // required for L0's content_hash to stay bit-identical. Level k >= 1
    // uses the actual bilinearly interpolated vertex heights in metres
    // (accumulate_border_crack_k1) - see that function's own comment for why
    // this is the right generalisation once the vertex itself is no longer
    // the raw sample.
    float skirt_depth;
    if (key.level == 0) {
        std::int64_t worst2 = 0;
        accumulate_border_crack(window, /*along_i=*/false, /*fixed=*/0, worst2);              // south (j=0), varying i
        accumulate_border_crack(window, /*along_i=*/false, /*fixed=*/kChunkQuads, worst2);     // north (j=64), varying i
        accumulate_border_crack(window, /*along_i=*/true, /*fixed=*/0, worst2);                // west (i=0), varying j
        accumulate_border_crack(window, /*along_i=*/true, /*fixed=*/kChunkQuads, worst2);      // east (i=64), varying j
        skirt_depth = 1.0f + static_cast<float>(worst2) / 512.0f; // /2 (undo the doubling) /256 (units -> m)
    } else {
        double worst_gap_m = 0.0;
        accumulate_border_crack_k1(window, /*along_i=*/false, /*fixed=*/0, t, step_m, worst_gap_m);
        accumulate_border_crack_k1(window, /*along_i=*/false, /*fixed=*/kChunkQuads, t, step_m, worst_gap_m);
        accumulate_border_crack_k1(window, /*along_i=*/true, /*fixed=*/0, t, step_m, worst_gap_m);
        accumulate_border_crack_k1(window, /*along_i=*/true, /*fixed=*/kChunkQuads, t, step_m, worst_gap_m);
        skirt_depth = 1.0f + static_cast<float>(worst_gap_m);
    }
    out.skirt_depth_m = skirt_depth;
    out.skirt_first_vertex = static_cast<std::uint32_t>(kBaseVertexCount);

    // Skirt vertices: border 0=west (i=0), 1=east (i=64), 2=south (j=0),
    // 3=north (j=64); each kChunkVerts vertices, same (x,y) as the border
    // vertex, z lowered by skirt_depth; NoData follows the border vertex.
    auto write_skirt = [&](int border, int k, int base_i, int base_j) {
        const int base_vi = base_index(base_i, base_j);
        const int svi = skirt_index(border, k);
        out.positions[static_cast<std::size_t>(svi) * 3 + 0] = out.positions[static_cast<std::size_t>(base_vi) * 3 + 0];
        out.positions[static_cast<std::size_t>(svi) * 3 + 1] = out.positions[static_cast<std::size_t>(base_vi) * 3 + 1];
        out.positions[static_cast<std::size_t>(svi) * 3 + 2] = out.positions[static_cast<std::size_t>(base_vi) * 3 + 2] - skirt_depth;
        out.normals[static_cast<std::size_t>(svi) * 3 + 0] = out.normals[static_cast<std::size_t>(base_vi) * 3 + 0];
        out.normals[static_cast<std::size_t>(svi) * 3 + 1] = out.normals[static_cast<std::size_t>(base_vi) * 3 + 1];
        out.normals[static_cast<std::size_t>(svi) * 3 + 2] = out.normals[static_cast<std::size_t>(base_vi) * 3 + 2];
        out.land_class[static_cast<std::size_t>(svi)] = out.land_class[static_cast<std::size_t>(base_vi)];
    };
    for (int k = 0; k < kChunkVerts; ++k) {
        write_skirt(0, k, 0, k);
        write_skirt(1, k, kChunkQuads, k);
        write_skirt(2, k, k, 0);
        write_skirt(3, k, k, kChunkQuads);
    }

    // Indices: base grid, dropping any triangle touching a NoData vertex.
    std::size_t n = 0;
    auto emit = [&](int a, int b, int c) {
        if (nodata[static_cast<std::size_t>(a)] || nodata[static_cast<std::size_t>(b)] || nodata[static_cast<std::size_t>(c)]) return;
        out.indices[n++] = static_cast<std::uint16_t>(a);
        out.indices[n++] = static_cast<std::uint16_t>(b);
        out.indices[n++] = static_cast<std::uint16_t>(c);
    };
    for (int j = 0; j < kChunkQuads; ++j) {
        for (int i = 0; i < kChunkQuads; ++i) {
            // Jolt's own diagonal in our (i=east, j=north) convention: the
            // (i,j+1)-(i+1,j) diagonal - see terrain_chunk.h's build_chunk
            // comment for the full derivation.
            const int v00 = base_index(i, j);
            const int v10 = base_index(i + 1, j);
            const int v01 = base_index(i, j + 1);
            const int v11 = base_index(i + 1, j + 1);
            emit(v00, v10, v01);
            emit(v10, v11, v01);
        }
    }
    // Skirt triangles, one border segment at a time; dropped if either
    // border vertex is NoData.
    auto emit_skirt_quad = [&](int border, int k, int base_i0, int base_j0, int base_i1, int base_j1) {
        const int b0 = base_index(base_i0, base_j0);
        const int b1 = base_index(base_i1, base_j1);
        if (nodata[static_cast<std::size_t>(b0)] || nodata[static_cast<std::size_t>(b1)]) return;
        const int s0 = skirt_index(border, k);
        const int s1 = skirt_index(border, k + 1);
        out.indices[n++] = static_cast<std::uint16_t>(b0);
        out.indices[n++] = static_cast<std::uint16_t>(b1);
        out.indices[n++] = static_cast<std::uint16_t>(s0);
        out.indices[n++] = static_cast<std::uint16_t>(b1);
        out.indices[n++] = static_cast<std::uint16_t>(s1);
        out.indices[n++] = static_cast<std::uint16_t>(s0);
    };
    for (int k = 0; k < kChunkQuads; ++k) {
        emit_skirt_quad(0, k, 0, k, 0, k + 1);
        emit_skirt_quad(1, k, kChunkQuads, k, kChunkQuads, k + 1);
        emit_skirt_quad(2, k, k, 0, k + 1, 0);
        emit_skirt_quad(3, k, k, kChunkQuads, k + 1, kChunkQuads);
    }
    out.indices.resize(n); // shrink to actual count; capacity (kMaxIndices) is unchanged

    // content_hash: FNV-1a over every exact byte that defines the chunk.
    Fnv1a64 h;
    h.u32(static_cast<std::uint32_t>(key.zone.zone));
    h.u8(static_cast<std::uint8_t>(key.zone.hemisphere));
    h.u8(key.level);
    h.i32(key.cx);
    h.i32(key.cy);
    h.f64(out.origin[0]);
    h.f64(out.origin[1]);
    h.f64(out.origin[2]);
    for (float v : out.positions) hash_f32(h, v);
    for (float v : out.normals) hash_f32(h, v);
    h.bytes(out.land_class.data(), out.land_class.size());
    for (std::uint16_t v : out.indices) h.u16(v);
    h.u32(out.skirt_first_vertex);
    hash_f32(h, out.skirt_depth_m);
    out.content_hash = h.value();
}

} // namespace g2m::mesh
