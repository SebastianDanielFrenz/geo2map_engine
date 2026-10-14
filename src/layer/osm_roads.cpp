// src/layer/osm_roads.cpp - see g2m/layer/osm_roads.h.

#include "g2m/layer/osm_roads.h"

#include "g2m/core/hash.h"
#include "g2m/core/det_math.h"

#include "g2m/core/geom/project.h" // project_e7_to_mm
#include "g2m/core/tile_key.h"     // floor_div
#include "g2m/layer/osm_clip.h"    // kDefaultOsmHaloM

#include <algorithm>
#include <array>
#include <cmath>
#include <charconv>
#include <cstdlib>
#include <limits>
#include <optional>

namespace g2m {
RoadSpeedLimit parse_road_speed_limit(std::string_view text) noexcept {
    while (!text.empty() && text.front() == ' ') text.remove_prefix(1);
    while (!text.empty() && text.back() == ' ') text.remove_suffix(1);
    if (text == "none") return {SpeedLimitKind::Unrestricted, 0.0, false};
    double factor = 1.0;
    for (const auto unit : {std::string_view("km/h"), std::string_view("kmh"), std::string_view("kph"), std::string_view("mph"), std::string_view("knots")}) {
        if (text.ends_with(unit)) {
            text.remove_suffix(unit.size());
            factor = unit == "mph" ? 1.609344 : unit == "knots" ? 1.852 : 1.0;
            break;
        }
    }
    while (!text.empty() && text.back() == ' ') text.remove_suffix(1);
    if (text.empty() || text.front() < '0' || text.front() > '9') return {};
    double value = 0.0;
    const auto parsed = std::from_chars(text.data(), text.data() + text.size(), value);
    if (parsed.ec != std::errc{} || parsed.ptr != text.data() + text.size() || !std::isfinite(value) || value < 0.0 || value * factor > 1000.0) return {};
    return {SpeedLimitKind::Numeric, value * factor, false};
}

RoadSpeedMatch match_road_speed_limit(std::span<const RoadSegment> roads,
    geom::PointMm point, double heading_x, double heading_y) noexcept {
    RoadSpeedMatch best;
    for (const auto& road : roads) {
        if (road.bridge) continue;
        const double dx = static_cast<double>(road.b.x - road.a.x);
        const double dy = static_cast<double>(road.b.y - road.a.y);
        const double length2 = dx * dx + dy * dy;
        if (length2 == 0.0) continue;
        const double px = static_cast<double>(point.x - road.a.x);
        const double py = static_cast<double>(point.y - road.a.y);
        const double t = std::clamp((px * dx + py * dy) / length2, 0.0, 1.0);
        const double ex = px - t * dx, ey = py - t * dy;
        const double distance = det::sqrt(ex * ex + ey * ey);
        if (distance > static_cast<double>(road.half_width_mm)) continue;
        if (distance < best.distance_mm || (distance == best.distance_mm && road.way_id < best.way_id)) {
            best.distance_mm = distance;
            best.way_id = road.way_id;
            best.limit = dx * heading_x + dy * heading_y >= 0.0 ? road.forward_limit : road.backward_limit;
        }
    }
    return best;
}
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
    std::optional<std::string_view> maxspeed, forward, backward;
    bool conditional = false;
    bool bridge = false;
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
        if (key == "maxspeed") t.maxspeed = value;
        else if (key == "maxspeed:forward") t.forward = value;
        else if (key == "maxspeed:backward") t.backward = value;
        else if (key.starts_with("maxspeed:") && (key.find("conditional") != key.npos || key.find("lanes") != key.npos || key.find("variable") != key.npos)) t.conditional = true;
        else if (key == "bridge") t.bridge = value != "no";
        else if (key == "highway") t.highway = value;
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
            seg.way_id = way.id;
            seg.bridge = tags.bridge;
            seg.forward_limit = parse_road_speed_limit(tags.forward.value_or(tags.maxspeed.value_or("")));
            seg.backward_limit = parse_road_speed_limit(tags.backward.value_or(tags.maxspeed.value_or("")));
            seg.forward_limit.conditional = seg.backward_limit.conditional = tags.conditional;
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

// Sentinels for paint_loop's extra_min_half_width_mm/max_half_width_mm
// meaning "rasterize_road_segments' own unclamped behaviour" - real limits
// (kPaintMinHalfWidthMm, kOwnerHaloClampMm below) are always well inside
// these, so std::max(..., kNoExtraMin) / std::min(..., kNoExtraMax) never
// change a value that would not otherwise have hit std::max(seg.half_width_mm,
// params.min_half_width_mm) - including when both of THOSE are negative,
// where inserting a literal 0 into the max() instead of this sentinel would
// have silently changed rasterize_road_segments' own result.
constexpr std::int64_t kNoExtraMinHalfWidthMm = std::numeric_limits<std::int64_t>::min();
constexpr std::int64_t kNoExtraMaxHalfWidthMm = std::numeric_limits<std::int64_t>::max();

// Physics one-cell-proof minimum (osm_roads.h's own derivation) and the
// owner-rule halo maximum (osm_clip.h's kDefaultOsmHaloM) - see
// paint_road_segments' own comment.
constexpr std::int64_t kPaintMinHalfWidthMm = 710;
constexpr std::int64_t kOwnerHaloClampMm = static_cast<std::int64_t>(kDefaultOsmHaloM) * 1000;

// Shared inner loop for rasterize_road_segments and paint_road_segments (and
// the block helpers' own per-block calls below) - the ONE place the exact
// integer inside test, round end caps and rank order live. `sink(j, i, seg)`
// is called once per covered sample, in `segs`' own order (a later segment's
// sink call for the same (j, i) is expected to overwrite an earlier one -
// callers rely on this, not this loop). `extra_min_half_width_mm`/
// `max_half_width_mm` let a caller widen (paint's 0.71 m minimum) or cap
// (the block helpers' 128 m halo maximum) every segment's half width without
// a second copy of this loop.
template <typename Sink>
void paint_loop(std::span<const RoadSegment> segs, const Lattice& lattice, const RasterParams& params,
                 std::int64_t extra_min_half_width_mm, std::int64_t max_half_width_mm, Sink&& sink) {
    if (lattice.nx <= 0 || lattice.ny <= 0 || lattice.spacing_mm <= 0) return;

    for (const RoadSegment& seg : segs) {
        if (seg.rank < params.min_rank) continue;
        std::int64_t hw = std::max({seg.half_width_mm, params.min_half_width_mm, extra_min_half_width_mm});
        hw = std::min(hw, max_half_width_mm);
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
                sink(static_cast<std::size_t>(j), static_cast<std::size_t>(i), seg);
            }
        }
    }
}

// Derives a block key's OWNER L2 tile: itself at block_level == 2, else its
// TileKey::parent() ancestor two levels up (block_level == 0) - the OWNER
// RULE (osm_roads.h's own comment on paint_road_blocks).
TileKey owner_l2_key(TileKey key) {
    while (key.level < 2) key = key.parent();
    return key;
}

// Ceiling division for b > 0, exact for any integer a (the floor_div
// identity ceil_div(a, b) == -floor_div(-a, b)).
constexpr std::int64_t ceil_div(std::int64_t a, std::int64_t b) { return -floor_div(-a, b); }

std::int64_t block_index_of(std::int64_t coord_mm, std::int64_t block_size_m) {
    return floor_div(floor_div(coord_mm, 1000), block_size_m);
}

// Calls `per_block(owner_key, i0, i1, j0, j1)` once for every block of
// `block_level`-sized TileKey tiles that `lattice` touches - i0..i1/j0..j1
// are lattice SAMPLE index ranges, inclusive, already clamped to
// [0, nx)/[0, ny). Never called for an empty range. Shared partition math
// for paint_road_blocks/rasterize_road_blocks (see their own comments).
template <typename PerBlock>
void for_each_block(const Lattice& lattice, int block_level, geo::UtmZone zone, PerBlock&& per_block) {
    if (lattice.nx <= 0 || lattice.ny <= 0 || lattice.spacing_mm <= 0) return;
    const std::int64_t block_size_m = TileKey::kLevel0SizeM << block_level;
    const std::int64_t block_size_mm = block_size_m * 1000;

    const std::int64_t e_last_mm = lattice.origin_e_mm + (lattice.nx - 1) * lattice.spacing_mm;
    const std::int64_t n_last_mm = lattice.origin_n_mm + (lattice.ny - 1) * lattice.spacing_mm;
    const std::int64_t bx_lo = block_index_of(lattice.origin_e_mm, block_size_m);
    const std::int64_t bx_hi = block_index_of(e_last_mm, block_size_m);
    const std::int64_t by_lo = block_index_of(lattice.origin_n_mm, block_size_m);
    const std::int64_t by_hi = block_index_of(n_last_mm, block_size_m);

    for (std::int64_t by = by_lo; by <= by_hi; ++by) {
        const std::int64_t j0 = std::max<std::int64_t>(
            0, ceil_div(by * block_size_mm - lattice.origin_n_mm, lattice.spacing_mm));
        const std::int64_t j1 = std::min<std::int64_t>(
            lattice.ny - 1, ceil_div((by + 1) * block_size_mm - lattice.origin_n_mm, lattice.spacing_mm) - 1);
        if (j0 > j1) continue;

        for (std::int64_t bx = bx_lo; bx <= bx_hi; ++bx) {
            const std::int64_t i0 = std::max<std::int64_t>(
                0, ceil_div(bx * block_size_mm - lattice.origin_e_mm, lattice.spacing_mm));
            const std::int64_t i1 = std::min<std::int64_t>(
                lattice.nx - 1, ceil_div((bx + 1) * block_size_mm - lattice.origin_e_mm, lattice.spacing_mm) - 1);
            if (i0 > i1) continue;

            const TileKey block_key{zone, block_level, static_cast<std::int32_t>(bx), static_cast<std::int32_t>(by)};
            per_block(owner_l2_key(block_key), i0, i1, j0, j1);
        }
    }
}

} // namespace

void rasterize_road_segments(std::span<const RoadSegment> segs, const Lattice& lattice, const RasterParams& params,
                              std::span<std::uint8_t> land_class, std::span<std::uint8_t> surface) {
    paint_loop(segs, lattice, params, kNoExtraMinHalfWidthMm, kNoExtraMaxHalfWidthMm,
               [&](std::size_t j, std::size_t i, const RoadSegment& seg) {
                   const std::size_t idx = j * static_cast<std::size_t>(lattice.nx) + i;
                   land_class[idx] = static_cast<std::uint8_t>(seg.land_class);
                   surface[idx] = static_cast<std::uint8_t>(seg.surface);
               });
}

void paint_road_segments(std::span<const RoadSegment> segs, const Lattice& lattice, const RasterParams& params,
                          const RoadValueLut& lut, std::span<std::uint16_t> out, std::size_t row_stride) {
    paint_loop(segs, lattice, params, kPaintMinHalfWidthMm, kOwnerHaloClampMm,
               [&](std::size_t j, std::size_t i, const RoadSegment& seg) {
                   out[j * row_stride + i] = lut.at(seg.land_class, seg.surface);
               });
}

RoadValueLut RoadValueLut::by_land_class(std::uint16_t off_road,
                                          std::initializer_list<std::pair<LandClass, std::uint16_t>> class_values) {
    RoadValueLut lut;
    lut.v.fill(off_road);
    for (const auto& [land_class, value] : class_values) {
        const std::size_t base = static_cast<std::size_t>(land_class) * 16;
        for (std::size_t s = 0; s < 16; ++s) lut.v[base + s] = value;
    }
    return lut;
}

RoadBlockStats paint_road_blocks(const Lattice& lattice, int block_level, geo::UtmZone zone, const RasterParams& params,
                                  const RoadValueLut& lut, RoadBlockLookupFn lookup, void* ctx,
                                  std::span<std::uint16_t> out, std::size_t row_stride) {
    RoadBlockStats stats;
    for_each_block(lattice, block_level, zone,
                   [&](const TileKey& owner, std::int64_t i0, std::int64_t i1, std::int64_t j0, std::int64_t j1) {
                       const RoadBlockLookupResult result = lookup(ctx, owner);
                       if (result.status == RoadBlockStatus::None) {
                           ++stats.none_blocks;
                           return;
                       }
                       if (result.status == RoadBlockStatus::Missing) {
                           ++stats.missing_blocks;
                           return;
                       }
                       ++stats.segments_blocks;
                       const Lattice sub{lattice.origin_e_mm + i0 * lattice.spacing_mm,
                                          lattice.origin_n_mm + j0 * lattice.spacing_mm, lattice.spacing_mm,
                                          static_cast<int>(i1 - i0 + 1), static_cast<int>(j1 - j0 + 1)};
                       paint_road_segments(result.segments, sub, params, lut,
                                            out.subspan(static_cast<std::size_t>(j0) * row_stride +
                                                        static_cast<std::size_t>(i0)),
                                            row_stride);
                   });
    return stats;
}

RoadBlockStats rasterize_road_blocks(const Lattice& lattice, int block_level, geo::UtmZone zone,
                                      const RasterParams& params, RoadBlockLookupFn lookup, void* ctx,
                                      std::span<std::uint8_t> land_class, std::span<std::uint8_t> surface) {
    RoadBlockStats stats;
    for_each_block(lattice, block_level, zone,
                   [&](const TileKey& owner, std::int64_t i0, std::int64_t i1, std::int64_t j0, std::int64_t j1) {
                       const RoadBlockLookupResult result = lookup(ctx, owner);
                       if (result.status == RoadBlockStatus::None) {
                           ++stats.none_blocks;
                           return;
                       }
                       if (result.status == RoadBlockStatus::Missing) {
                           ++stats.missing_blocks;
                           return;
                       }
                       ++stats.segments_blocks;
                       const Lattice sub{lattice.origin_e_mm + i0 * lattice.spacing_mm,
                                          lattice.origin_n_mm + j0 * lattice.spacing_mm, lattice.spacing_mm,
                                          static_cast<int>(i1 - i0 + 1), static_cast<int>(j1 - j0 + 1)};
                       paint_loop(result.segments, sub, params, kNoExtraMinHalfWidthMm, kOwnerHaloClampMm,
                                  [&](std::size_t jj, std::size_t ii, const RoadSegment& seg) {
                                      const std::size_t idx =
                                          (static_cast<std::size_t>(j0) + jj) * static_cast<std::size_t>(lattice.nx) +
                                          (static_cast<std::size_t>(i0) + ii);
                                      land_class[idx] = static_cast<std::uint8_t>(seg.land_class);
                                      surface[idx] = static_cast<std::uint8_t>(seg.surface);
                                  });
                   });
    return stats;
}

namespace {

// osm_roads.cpp's own kDefaultHighways rank table, current values (track=0
// .. motorway=9): fallback only, used if a style is ever shipped without
// "tertiary"/"primary" at all (matches racing_game's own pre-move fallback).
constexpr int kFallbackTertiaryRank = 5;
constexpr int kFallbackPrimaryRank = 7;

} // namespace

std::optional<RasterParams> raster_params_for_render_level(int level) {
    if (level <= 1) {
        return RasterParams{/*min_rank=*/0, /*min_half_width_mm=*/0};
    }

    const RoadStyle& style = RoadStyle::default_style();

    if (level == 2) {
        const HighwayStyleEntry* tertiary = style.find_highway("tertiary");
        const int min_rank = tertiary != nullptr ? tertiary->rank : kFallbackTertiaryRank;
        return RasterParams{min_rank, /*min_half_width_mm=*/0};
    }

    if (level == 3) {
        const HighwayStyleEntry* primary = style.find_highway("primary");
        const int min_rank = primary != nullptr ? primary->rank : kFallbackPrimaryRank;
        // spacing_mm = (1 << level) * 1000 is exactly divisible by 4 for
        // level >= 2, so this integer arithmetic is exact (0.75 * spacing).
        const std::int64_t spacing_mm = (std::int64_t{1} << level) * 1000;
        const std::int64_t min_half_width_mm = (spacing_mm * 3) / 4;
        return RasterParams{min_rank, min_half_width_mm};
    }

    return std::nullopt; // L>=4: no road classes
}

const RenderRoadLod& default_render_road_lod() {
    static const RenderRoadLod lod = [] {
        RenderRoadLod d;
        for (int level = 0; level <= 3; ++level) {
            const auto p = raster_params_for_render_level(level);
            d.levels.push_back({true, p->min_rank, p->min_half_width_mm});
        }
        return d;
    }();
    return lod;
}

std::optional<RasterParams> raster_params_for_render_level(int level, const RenderRoadLod& lod) {
    // Negative levels behave like level 0, as in the compiled policy.
    const std::size_t index = level < 0 ? 0 : static_cast<std::size_t>(level);
    if (index >= lod.levels.size()) return std::nullopt;
    const RenderRoadLodLevel& l = lod.levels[index];
    if (!l.enabled) return std::nullopt;
    return RasterParams{l.min_rank, l.min_half_width_mm};
}

std::uint64_t render_road_lod_key(const RenderRoadLod& lod, int level) {
    Fnv1a64 h;
    h.str("g2m.render_road_lod/1").i32(level);
    const auto p = raster_params_for_render_level(level, lod);
    if (!p) return h.u8(0).value();
    return h.u8(1).i32(p->min_rank).i64(p->min_half_width_mm).value();
}

} // namespace g2m
