// src/layer/osm_roads.cpp - see g2m/layer/osm_roads.h.

#include "g2m/layer/osm_roads.h"

#include "g2m/core/geom/project.h" // project_e7_to_mm
#include "g2m/core/tile_key.h"     // floor_div

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdlib>
#include <optional>

namespace g2m {
namespace {

// Parses `text` as one clean decimal number (nothing before or after it,
// modulo surrounding whitespace strtod itself skips) - matches
// src/decode/tiff/tfw.cpp's own "not a clean number" rejection rule. A
// unit-suffixed OSM tag (e.g. "3.5 m") or anything else that does not parse
// cleanly is treated as absent, not an error.
std::optional<double> parse_clean_double(std::string_view text) {
    const std::string s(text);
    char* end = nullptr;
    const double v = std::strtod(s.c_str(), &end);
    if (end == s.c_str() || *end != '\0') return std::nullopt;
    return v;
}

constexpr double kMetresPerLane = 3.25;

// PLAN.md 2.5's own IClassifier philosophy: a data table, not code
// branches. Rank ascending by importance (see osm_roads.h's own comment on
// RoadStyle::highways); a `*_link` entry shares its parent tier's rank.
constexpr std::array<HighwayStyleEntry, 15> kDefaultHighways{{
    {"track", LandClass::UnpavedRoad, 0, 3.0},
    {"service", LandClass::PavedRoad, 1, 4.0},
    {"living_street", LandClass::PavedRoad, 2, 5.0},
    {"residential", LandClass::PavedRoad, 3, 5.5},
    {"unclassified", LandClass::PavedRoad, 4, 5.5},
    {"tertiary_link", LandClass::PavedRoad, 5, 5.0},
    {"tertiary", LandClass::PavedRoad, 5, 6.0},
    {"secondary_link", LandClass::PavedRoad, 6, 5.0},
    {"secondary", LandClass::PavedRoad, 6, 6.5},
    {"primary_link", LandClass::PavedRoad, 7, 5.5},
    {"primary", LandClass::PavedRoad, 7, 7.0},
    {"trunk_link", LandClass::PavedRoad, 8, 6.0},
    {"trunk", LandClass::PavedRoad, 8, 7.5},
    {"motorway_link", LandClass::PavedRoad, 9, 6.0},
    {"motorway", LandClass::PavedRoad, 9, 12.0},
}};

// 13 entries: one per non-Unknown SurfaceKind plus two common OSM synonyms
// ("fine_gravel" -> Gravel, "ground" -> Dirt) and the generic "unpaved" tag
// (-> Compacted, the usual meaning when the specific material is unknown).
// SurfaceKind::Water has no entry here: a road's own driving surface is
// never water (that value exists for terrain/landcover classification, not
// this table).
constexpr std::array<SurfaceStyleEntry, 13> kDefaultSurfaces{{
    {"asphalt", SurfaceKind::Asphalt, LandClass::PavedRoad},
    {"concrete", SurfaceKind::Concrete, LandClass::PavedRoad},
    {"paving_stones", SurfaceKind::PavingStones, LandClass::PavedRoad},
    {"sett", SurfaceKind::Sett, LandClass::PavedRoad},
    {"cobblestone", SurfaceKind::Cobblestone, LandClass::PavedRoad},
    {"gravel", SurfaceKind::Gravel, LandClass::UnpavedRoad},
    {"fine_gravel", SurfaceKind::Gravel, LandClass::UnpavedRoad},
    {"compacted", SurfaceKind::Compacted, LandClass::UnpavedRoad},
    {"unpaved", SurfaceKind::Compacted, LandClass::UnpavedRoad},
    {"dirt", SurfaceKind::Dirt, LandClass::UnpavedRoad},
    {"ground", SurfaceKind::Dirt, LandClass::UnpavedRoad},
    {"grass", SurfaceKind::Grass, LandClass::UnpavedRoad},
    {"sand", SurfaceKind::Sand, LandClass::UnpavedRoad},
}};

} // namespace

const HighwayStyleEntry* RoadStyle::find_highway(std::string_view highway) const {
    for (const HighwayStyleEntry& e : highways) {
        if (e.highway == highway) return &e;
    }
    return nullptr;
}

const SurfaceStyleEntry* RoadStyle::find_surface(std::string_view surface) const {
    for (const SurfaceStyleEntry& e : surfaces) {
        if (e.surface == surface) return &e;
    }
    return nullptr;
}

const RoadStyle& RoadStyle::default_style() {
    static const RoadStyle style{kDefaultHighways, kDefaultSurfaces};
    return style;
}

namespace {

struct WayTags {
    std::optional<std::string_view> highway;
    std::optional<std::string_view> surface;
    std::optional<std::string_view> width;
    std::optional<std::string_view> lanes;
    bool area = false;
    bool tunnel = false;
};

WayTags read_way_tags(const osm::OsmData& set, const osm::Way& way) {
    WayTags t;
    for (const osm::Tag& tag : set.tags_of(way)) {
        const std::string_view key = set.str(tag.key);
        const std::string_view value = set.str(tag.value);
        if (key == "highway") t.highway = value;
        else if (key == "surface") t.surface = value;
        else if (key == "width") t.width = value;
        else if (key == "lanes") t.lanes = value;
        else if (key == "area") t.area = (value == "yes");
        else if (key == "tunnel") t.tunnel = (value == "yes");
    }
    return t;
}

// Resolved per-way style, before per-segment projection.
struct WayStyle {
    double width_m;
    LandClass land_class;
    SurfaceKind surface;
    int rank;
};

std::optional<WayStyle> resolve_way_style(const WayTags& tags, const RoadStyle& style) {
    if (tags.area || tags.tunnel || !tags.highway) return std::nullopt;
    const HighwayStyleEntry* hw = style.find_highway(*tags.highway);
    if (!hw) return std::nullopt; // not drivable / not in the table

    WayStyle out{hw->default_width_m, hw->land_class, SurfaceKind::Unknown, hw->rank};

    // A `width` tag that FAILS to parse cleanly is treated as absent
    // (parse_clean_double's own contract) - it must fall through to
    // `lanes`, not merely leave the highway's default width in place while
    // also blocking the `lanes` fallback (a width tag being PRESENT is not
    // the same as it being USABLE).
    bool have_width = false;
    if (tags.width) {
        if (const std::optional<double> w = parse_clean_double(*tags.width); w && *w > 0.0) {
            out.width_m = *w;
            have_width = true;
        }
    }
    if (!have_width && tags.lanes) {
        if (const std::optional<double> n = parse_clean_double(*tags.lanes); n && *n > 0.0) {
            out.width_m = *n * kMetresPerLane;
        }
    }

    if (tags.surface) {
        if (const SurfaceStyleEntry* sf = style.find_surface(*tags.surface)) {
            out.surface = sf->surface_kind;
            out.land_class = sf->land_class;
        }
    }
    return out;
}

} // namespace

std::vector<RoadSegment> extract_road_segments(const osm::OsmData& set, geo::UtmZone zone, const RoadStyle& style) {
    std::vector<RoadSegment> out;
    const geo::Utm utm(geo::kGRS80);

    for (std::size_t way_index = 0; way_index < set.ways.size(); ++way_index) {
        const osm::Way& way = set.ways[way_index];
        const WayTags tags = read_way_tags(set, way);
        const std::optional<WayStyle> ws = resolve_way_style(tags, style);
        if (!ws) continue;

        const std::int64_t half_width_mm = std::llround(ws->width_m * 500.0); // metres -> mm, halved
        const std::span<const std::int64_t> refs = set.refs_of(way);
        for (std::size_t i = 0; i + 1 < refs.size(); ++i) {
            const osm::Node* na = set.find_node(refs[i]);
            const osm::Node* nb = set.find_node(refs[i + 1]);
            if (!na || !nb) continue; // set not validated / incomplete way: skip, do not fail
            const std::optional<geom::PointMm> a = geom::project_e7_to_mm(utm, zone, na->lat, na->lon);
            const std::optional<geom::PointMm> b = geom::project_e7_to_mm(utm, zone, nb->lat, nb->lon);
            if (!a || !b) continue; // "far" from the zone

            RoadSegment seg;
            seg.a = *a;
            seg.b = *b;
            seg.half_width_mm = half_width_mm;
            seg.land_class = ws->land_class;
            seg.surface = ws->surface;
            seg.rank = ws->rank;
            out.push_back(seg);
        }
    }

    std::stable_sort(out.begin(), out.end(), [&](const RoadSegment& x, const RoadSegment& y) {
        return x.rank < y.rank;
    });
    // stable_sort by rank alone already preserves (way id, segment index)
    // order among equal ranks, since `out` was built by walking set.ways
    // (strictly ascending id, docs/formats/src_osm.md's own canonical-form
    // rule) and each way's own refs in order - so this single stable sort
    // realises the documented (rank, way id, segment index) ordering
    // without tracking those keys separately.
    return out;
}

namespace {

// See osm_roads.h's own derivation of this test.
bool sample_inside_segment(geom::PointMm sample, const RoadSegment& seg, std::int64_t hw) {
    const std::int64_t wx = sample.x - seg.a.x, wy = sample.y - seg.a.y;
    const std::int64_t dx = seg.b.x - seg.a.x, dy = seg.b.y - seg.a.y;
    const std::int64_t dot_wd = wx * dx + wy * dy;
    const std::int64_t dot_dd = dx * dx + dy * dy;
    const std::int64_t hw2 = hw * hw;

    if (dot_dd == 0) { // degenerate (a == b): point-in-circle
        return wx * wx + wy * wy <= hw2;
    }
    if (dot_wd <= 0) {
        return wx * wx + wy * wy <= hw2;
    }
    if (dot_wd >= dot_dd) {
        const std::int64_t ux = sample.x - seg.b.x, uy = sample.y - seg.b.y;
        return ux * ux + uy * uy <= hw2;
    }
    const std::int64_t cross = wx * dy - wy * dx;
    // cross*cross <= hw2*dot_dd, exact via a 128-bit intermediate product
    // on each side (g2m::geom::sign_diff_products) - see osm_roads.h.
    return geom::sign_diff_products(cross, cross, hw2, dot_dd) <= 0;
}

} // namespace

void rasterize_road_segments(std::span<const RoadSegment> segs, const Lattice& lattice, const RasterParams& params,
                              std::span<std::uint8_t> land_class, std::span<std::uint8_t> surface) {
    if (lattice.nx <= 0 || lattice.ny <= 0 || lattice.spacing_mm <= 0) return;

    for (const RoadSegment& seg : segs) {
        if (seg.rank < params.min_rank) continue;
        const std::int64_t hw = std::max(seg.half_width_mm, params.min_half_width_mm);
        if (hw < 0) continue;

        // Integer bounding box of the segment widened by hw on every side,
        // converted to a lattice index range via floor_div (exact: the box
        // edges and spacing are all integers).
        const std::int64_t box_w = std::min(seg.a.x, seg.b.x) - hw;
        const std::int64_t box_e = std::max(seg.a.x, seg.b.x) + hw;
        const std::int64_t box_s = std::min(seg.a.y, seg.b.y) - hw;
        const std::int64_t box_n = std::max(seg.a.y, seg.b.y) + hw;

        std::int64_t i0 = floor_div(box_w - lattice.origin_e_mm, lattice.spacing_mm);
        std::int64_t i1 = floor_div(box_e - lattice.origin_e_mm, lattice.spacing_mm) + 1;
        std::int64_t j0 = floor_div(box_s - lattice.origin_n_mm, lattice.spacing_mm);
        std::int64_t j1 = floor_div(box_n - lattice.origin_n_mm, lattice.spacing_mm) + 1;
        i0 = std::max<std::int64_t>(i0, 0);
        j0 = std::max<std::int64_t>(j0, 0);
        i1 = std::min<std::int64_t>(i1, lattice.nx - 1);
        j1 = std::min<std::int64_t>(j1, lattice.ny - 1);

        for (std::int64_t j = j0; j <= j1; ++j) {
            const std::int64_t sy = lattice.origin_n_mm + j * lattice.spacing_mm;
            for (std::int64_t i = i0; i <= i1; ++i) {
                const std::int64_t sx = lattice.origin_e_mm + i * lattice.spacing_mm;
                if (!sample_inside_segment(geom::PointMm{sx, sy}, seg, hw)) continue;
                const std::size_t idx = static_cast<std::size_t>(j) * static_cast<std::size_t>(lattice.nx) +
                                        static_cast<std::size_t>(i);
                land_class[idx] = static_cast<std::uint8_t>(seg.land_class);
                surface[idx] = static_cast<std::uint8_t>(seg.surface);
            }
        }
    }
}

} // namespace g2m
