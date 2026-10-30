// g2m/world/lattice_drape.h (internal) - drapes ribbon triangles on the terrain mesh's own triangulation (vault G2M-025).
//
// The terrain mesh, the physics ground (Jolt's heightfield) and sample_height_exact share ONE surface: the 1 m lattice
// whose nodes are the DGM1 cell centres (absolute x.5 m) with every quad split along its NW-SE diagonal, the one joining
// (c+1, r) and (c, r+1). A ribbon vertex placed by bilinear sampling is not on that surface, and a ribbon triangle is a
// plane that the surface does not follow between its vertices: on a steep bank the terrain stood up to 0.38 m above the
// road. LatticeDrape reads the caller's ground function at lattice NODES only, builds the exact triangle surface from
// them, and cuts every ribbon triangle where it crosses a lattice line or a diagonal. Each piece then lies in one terrain
// triangle, its vertices (inserted at the cuts, shared with the neighbours) carry the terrain height, and so the road IS
// the terrain surface (plus the lift) wherever the terrain has ground.
//
// Coordinates are tile-local metres (x east, y north from the tile's south-west corner, whose absolute easting and
// northing are whole metres: the lattice lines are x = k + 0.5 there too).
#pragma once

#include "g2m/world/road_ribbons.h"

#include <array>
#include <cstdint>
#include <limits>
#include <map>
#include <optional>
#include <tuple>
#include <unordered_map>
#include <vector>

namespace g2m::world::detail {

// A ribbon vertex before it has a height: the position and the RoadRibbonVertex attributes u .. verge, linear inside a
// triangle (flags, lanes, forward and verge are constant over one triangle, so the interpolation reproduces them).
struct DrapeVertex {
    double x = 0, y = 0;
    std::array<double, 8> a{}; // u, v, width, fade, lanes, forward, flags, verge
    double pin = std::numeric_limits<double>::quiet_NaN(); // add_ramp only: this vertex's own height (a deck row); NaN = on the terrain
};

struct DrapeStats {
    std::size_t triangles_in = 0, pieces = 0, uncut = 0, dropped = 0, node_reads = 0; // uncut: emitted whole; dropped: pieces over a terrain triangle without ground
};

class LatticeDrape {
public:
    // `ground`: absolute (easting, northing) -> height, read at lattice nodes. (ox, oy): absolute coordinates of the tile's
    // south-west corner, whole metres.
    // `tolerance_m`: a triangle whose edges and cut vertices all stay within it of the terrain is emitted uncut (0: cut all).
    LatticeDrape(const RoadRibbonGround& ground, double ox, double oy, double tolerance_m = 0.005);

    // Height of the terrain triangle surface at a tile-local point (nullopt: that triangle has a corner without ground).
    std::optional<double> height(double x, double y);
    // The same at an absolute point.
    std::optional<double> height_abs(double e, double n) { return height(e - ox_, n - oy_); }

    // Cuts the triangle a, b, c on the terrain triangles and appends the pieces to `mesh` (same winding as a, b, c),
    // `raise` above the terrain surface. Vertices at the same place with the same attributes are shared.
    // `tolerance_m` < 0: the constructor's. Marking layers pass a tighter one: they sit a few mm above the road bed and must not
    // fall under it.
    void add(RoadRibbonTile& mesh, const DrapeVertex& a, const DrapeVertex& b, const DrapeVertex& c, double raise, double tolerance_m = -1);

    // A triangle with deck-row corners (pin set) and terrain corners: the ramp from the deck end down (or up) to the ground. Its pieces
    // follow the plane through the corner heights but never fall below the terrain: a piece vertex takes max(plane, terrain), except
    // on a deck-deck edge (the plane) and a ground-ground edge (the terrain), which the neighbouring quads share.
    void add_ramp(RoadRibbonTile& mesh, const DrapeVertex& a, const DrapeVertex& b, const DrapeVertex& c, double raise);

    const DrapeStats& stats() const { return stats_; }

private:
    std::optional<double> node(std::int64_t c, std::int64_t r);
    template <class ZFn>
    void cut_emit(RoadRibbonTile& mesh, const DrapeVertex& a, const DrapeVertex& b, const DrapeVertex& c, double raise, ZFn&& zfn);
    bool straight_edge(double px, double py, double pz, double qx, double qy, double qz, double tol);
    std::int32_t vertex_for(RoadRibbonTile& mesh, const DrapeVertex& v, double z, double raise);

    RoadRibbonGround ground_;
    double ox_, oy_, tol_;
    // Lattice node heights: a dense window around the tile (NaN: not read yet, +inf: no ground), a map beyond it.
    static constexpr std::int64_t kWindowMargin = 64, kWindowSize = 1024 + 2 * kWindowMargin;
    std::vector<double> window_;
    std::unordered_map<std::uint64_t, std::optional<double>> far_nodes_;
    struct SharedKey {
        std::int64_t x, y;
        std::int32_t lanes, forward, flags, verge, raise;
        bool operator==(const SharedKey&) const = default;
    };
    struct SharedHash {
        std::size_t operator()(const SharedKey& k) const noexcept {
            std::uint64_t h = 1469598103934665603ull;
            for (const std::uint64_t v : {static_cast<std::uint64_t>(k.x), static_cast<std::uint64_t>(k.y), static_cast<std::uint64_t>(static_cast<std::uint32_t>(k.lanes)),
                                          static_cast<std::uint64_t>(static_cast<std::uint32_t>(k.forward)), static_cast<std::uint64_t>(static_cast<std::uint32_t>(k.flags)),
                                          static_cast<std::uint64_t>(static_cast<std::uint32_t>(k.verge)), static_cast<std::uint64_t>(static_cast<std::uint32_t>(k.raise))}) {
                h ^= v;
                h *= 1099511628211ull;
                h ^= h >> 29;
            }
            return static_cast<std::size_t>(h);
        }
    };
    std::unordered_map<SharedKey, std::int32_t, SharedHash> shared_;
    std::vector<double> scratch_t_;
    DrapeStats stats_;
};

} // namespace g2m::world::detail
