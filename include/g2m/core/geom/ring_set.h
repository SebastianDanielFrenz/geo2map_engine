// g2m/core/geom/ring_set.h - a set of closed polygon rings and the exact
// tests over them: point-in-polygon and "how does this UTM tile square
// relate to the boundary" (G1c plan I1; used by region resolution, I2's
// OsmAdminPart).
#pragma once

#include "g2m/core/geo/utm.h"
#include "g2m/core/geom/exact.h"
#include "g2m/core/tile_key.h"

#include <cstdint>
#include <utility>
#include <vector>

namespace g2m::geom {

// A closed polygon ring: vertices in order, IMPLICITLY closed (the last
// vertex connects back to the first - do not repeat it).
using Ring = std::vector<PointMm>;

// A polygon assembled from outer and inner (hole) rings. `contains()`
// tests membership with the even-odd rule over EVERY ring together - outer
// and inner are not distinguished at query time (assemble_rings already
// used even-odd nesting depth to tell them apart when building this set,
// so a plain even-odd union over all of them reproduces the same "outer
// minus holes" membership).
struct RingSet {
    std::vector<Ring> outers;
    std::vector<Ring> inners;
    // Bounding box of every vertex in `outers` + `inners`; see
    // bounding_box(). Invalid (w > e, BoxMm::valid() false) for an empty
    // set. tiles_near() trusts this field - keep it in sync.
    BoxMm bbox{};

    friend bool operator==(const RingSet&, const RingSet&) = default;
};

// The bounding box of every ring's vertices (outers + inners). Invalid
// (w > e) if `rings` has not a single vertex.
BoxMm bounding_box(const RingSet& rings);

// Exact even-odd point-in-polygon test. A point exactly ON any ring edge
// counts as inside.
bool contains(const RingSet& rings, PointMm p);

enum class TileRelation : std::uint8_t {
    Outside = 0,
    Boundary = 1, // a ring segment touches the square, or comes within halo_m of it
    Interior = 2, // fully inside `rings`, no ring segment touches or is within halo_m
};

// How square `sq` relates to `rings`: Boundary if any ring segment touches
// it (exact); else Interior if `rings` contains sq's SW corner; else
// Boundary if the minimum distance from sq to any ring segment is at most
// halo_m metres (computed in double, only + - * /, no sqrt - IEEE-exact
// and so identical on every platform); else Outside.
TileRelation classify_square(const RingSet& rings, BoxMm sq, double halo_m);

// Every tile at `level` that is Boundary or Interior for `rings`, scanning
// `rings.bbox` dilated by halo_m on every side, row-major (y then x, both
// ascending). Sorted by TileKey::packed(). Empty if `rings.bbox` is
// invalid (an empty ring set).
std::vector<std::pair<TileKey, TileRelation>> tiles_near(const RingSet& rings, geo::UtmZone zone, std::uint8_t level,
                                                          std::uint32_t halo_m);

} // namespace g2m::geom
