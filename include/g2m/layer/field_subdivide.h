// Field subdivision kernel (docs/pipeline/fields.md): cut one OSM polygon along linear features and around area
// features. Pure and deterministic, integer arithmetic only.
//
// Method: a raster of rules.cell_mm cells on the absolute millimetre grid. A cell belongs to the polygon when its
// centre is inside (exact scanline, even-odd over the outer ring and the holes). A line feature takes out every
// cell whose centre is within its half width of the segment (exact squared-distance comparison), an obstacle area
// every cell whose centre is inside it. The remaining cells are labelled by 4-connectivity in row-major order from
// the south-west, each label's boundary is traced as unit edges, merged into straight runs and simplified with
// Douglas-Peucker (rules.simplify_tolerance_mm, exact comparisons). A polygon nothing cuts is returned untouched
// ("verbatim"), so an uncut field keeps its exact OSM outline.
#pragma once
#include "g2m/core/result.h"
#include "g2m/layer/field_rules.h"
#include "g2m/layer/field_tile.h"
#include <cstdint>
#include <span>
#include <vector>

namespace g2m {
struct FieldSplitSegment {
    geom::PointMm a, b;
    std::int32_t half_width_mm = 0;
};
// An area to cut out; all rings together are filled with the even-odd rule (outer and inner rings of a multipolygon).
struct FieldObstacle {
    std::vector<geom::Ring> rings;
    geom::BoxMm bbox;
};
struct SubdivideResult {
    bool verbatim = true;                  // the polygon is kept as it is (pieces is empty)
    std::vector<FieldPolygon> pieces;      // in label order: lowest row first, then lowest column of the piece's first cell
    std::uint32_t slivers_dropped = 0;
    bool sliver_block = false;             // every piece was a sliver; verbatim
    bool obstacles_ignored = false;
    bool simplify_fallback = false;
    std::optional<FieldDeclineReason> declined;  // set instead of a result when the raster budget is exceeded
};
Result<SubdivideResult> subdivide_polygon(const FieldRules& rules, const FieldPolygon& polygon,
    std::span<const FieldSplitSegment> lines, std::span<const FieldObstacle* const> obstacles);

// Exposed for tests.
geom::Ring simplify_field_ring(const geom::Ring& ring, std::int64_t tolerance_mm);
// True when no two non-adjacent segments of the polygon's rings cross at a single interior point.
bool field_polygon_simple(const FieldPolygon& polygon);
} // namespace g2m
