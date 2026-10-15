// src/layer/terrain_carve.cpp - see g2m/layer/terrain_carve.h (street fidelity P2 slice 9, design 5.3 and 6.1-6.4).
#include "g2m/layer/terrain_carve.h"
#include "g2m/core/det_math.h"
#include "g2m/core/geom/exact.h"
#include "g2m/core/hash.h"
#include "g2m/core/raster.h"
#include "g2m/layer/road_cross_section.h"
#include "g2m/layer/road_surface.h"
#include "g2m/layer/terrain_layers.h"
#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <thread>
#include <tuple>
#include <utility>

namespace g2m {
namespace {
constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();
constexpr std::size_t kNone = std::numeric_limits<std::size_t>::max();
// Stations of the approach polylines are generated only over the reference segments within this distance of the
// covered region; every core, blend or deck footprint that can reach the region lies well inside it (outer half width
// + 15 m blend < 64 m).
constexpr double kCullM = 64.0;

// round(num / den) with ties to even, den > 0, exact on int64.
std::int64_t div_half_even(std::int64_t num, std::int64_t den) {
    std::int64_t q = floor_div(num, den);
    const std::int64_t r = num - q * den;   // 0 <= r < den
    if (2 * r > den || (2 * r == den && (q & 1) != 0)) ++q;
    return q;
}
} // namespace

// ---- smoothing (design 6.2 step 1) ----

Result<CarveTerrain> CarveTerrain::make(const TileKey& centre, std::span<const HeightTile* const> window,
    const CarveSmoothing& smoothing, std::int64_t margin) {
    if (!centre.valid() || centre.level != 0) return make_error(ErrorCode::InvalidArgument, "carve: level-0 centre required");
    if (window.size() != 9) return make_error(ErrorCode::InvalidArgument, "carve: 3 x 3 DEM window required");
    if (margin < 0 || margin > 200) return make_error(ErrorCode::InvalidArgument, "carve: margin out of range");
    if (smoothing.enabled && (smoothing.radius_m < 1 || smoothing.radius_m > 16 ||
            smoothing.kernel.size() != static_cast<std::size_t>(2 * smoothing.radius_m + 1)))
        return make_error(ErrorCode::InvalidArgument, "carve: smoothing kernel does not match its radius");
    for (std::size_t i = 0; i < 9; ++i) {
        if (!window[i]) continue;
        const TileKey expect{centre.zone, 0, centre.x + static_cast<std::int32_t>(i % 3) - 1, centre.y + static_cast<std::int32_t>(i / 3) - 1};
        if (window[i]->key != expect) return make_error(ErrorCode::InvalidArgument, "carve: DEM window tile out of place");
    }
    CarveTerrain out;
    out.centre_ = centre;
    out.smoothing_ = smoothing;
    out.e0_ = centre.min_easting() - margin; out.n0_ = centre.min_northing() - margin; out.size_ = 256 + 2 * margin;
    out.we0_ = centre.min_easting() - 256; out.wn0_ = centre.min_northing() - 256;
    out.raw_.assign(static_cast<std::size_t>(768 * 768), kHeightNoData);
    for (std::size_t i = 0; i < 9; ++i) {
        if (!window[i]) continue;
        const std::size_t ox = (i % 3) * 256, oy = (i / 3) * 256;
        for (std::size_t j = 0; j < 256; ++j)
            for (std::size_t c = 0; c < 256; ++c) out.raw_[(oy + j) * 768 + ox + c] = window[i]->h[j * 256 + c];
    }
    out.smoothed_.resize(static_cast<std::size_t>(out.size_ * out.size_));
    for (std::int64_t j = 0; j < out.size_; ++j)
        for (std::int64_t i = 0; i < out.size_; ++i)
            out.smoothed_[static_cast<std::size_t>(j * out.size_ + i)] = out.smooth_cell(out.e0_ + i, out.n0_ + j);
    return out;
}

std::int32_t CarveTerrain::raw(std::int64_t e, std::int64_t n) const noexcept {
    const std::int64_t i = e - we0_, j = n - wn0_;
    if (i < 0 || j < 0 || i >= 768 || j >= 768) return kHeightNoData;
    return raw_[static_cast<std::size_t>(j * 768 + i)];
}

// One cell of the smoothing (design 6.2 step 1): the x pass at each tap row (int64 sum over the taps that are not
// NoData divided by their weight sum, half-even), the y pass over those values likewise, then
// h + round_half_even(strength * (s - h) / 100). A NoData cell stays NoData; a row whose own centre is NoData has no x
// value (it is skipped by the y pass, as the game skips the NaN of its first pass).
std::int32_t CarveTerrain::smooth_cell(std::int64_t e, std::int64_t n) const noexcept {
    const std::int32_t h = raw(e, n);
    if (h == kHeightNoData || !smoothing_.enabled || smoothing_.strength_percent == 0) return h;
    const std::int64_t r = smoothing_.radius_m;
    std::int64_t ysum = 0, yweights = 0;
    for (std::int64_t dy = -r; dy <= r; ++dy) {
        if (raw(e, n + dy) == kHeightNoData) continue;
        std::int64_t sum = 0, weights = 0;
        for (std::int64_t dx = -r; dx <= r; ++dx) {
            const std::int32_t v = raw(e + dx, n + dy);
            if (v == kHeightNoData) continue;
            const std::int64_t k = smoothing_.kernel[static_cast<std::size_t>(dx + r)];
            sum += k * v; weights += k;
        }
        const std::int64_t k = smoothing_.kernel[static_cast<std::size_t>(dy + r)];
        ysum += k * div_half_even(sum, weights); yweights += k;
    }
    const std::int64_t s = div_half_even(ysum, yweights);
    return static_cast<std::int32_t>(h + div_half_even(static_cast<std::int64_t>(smoothing_.strength_percent) * (s - h), 100));
}

std::int32_t CarveTerrain::smoothed(std::int64_t e, std::int64_t n) const noexcept {
    const std::int64_t i = e - e0_, j = n - n0_;
    if (i >= 0 && j >= 0 && i < size_ && j < size_) return smoothed_[static_cast<std::size_t>(j * size_ + i)];
    return smooth_cell(e, n);   // outside the table (blend widths of far road-side points): the same function, on demand
}

bool CarveTerrain::in_window(std::int64_t e, std::int64_t n) const noexcept {
    const std::int64_t r = smoothing_.enabled ? smoothing_.radius_m : 0;
    return e - we0_ >= r && n - wn0_ >= r && e - we0_ < 768 - r && n - wn0_ < 768 - r;
}

void CarveStats::add_cells(const CarveStats& o) {
    cells_terrain += o.cells_terrain; cells_nodata += o.cells_nodata; cells_junction_core += o.cells_junction_core;
    cells_approach_core += o.cells_approach_core; cells_junction_blend += o.cells_junction_blend;
    cells_approach_blend += o.cells_approach_blend; cells_deck_clamped += o.cells_deck_clamped;
    deck_clearance_violations += o.deck_clearance_violations;
}

// ---- the model (design 6.2 step 2, 6.3, 6.4) ----

namespace {
struct Station {
    double s = 0, x = 0, y = 0;
    double outer_left = 0, outer_right = 0;
    double b_left = kNaN, b_right = kNaN;
    bool connects_next = false, global_first = false, global_last = false;
};
// One carved ground entry or one bridge entry (the deck clamp).
struct Line {
    std::size_t entry = 0;
    bool bridge = false;
    double length = 0;
    double trim_start = 0, trim_end = 0;
    double b_class = 0;
    // Bridge lines: the deck's stations; the clearance ramp runs from both (0 and length for a bridge entry, an
    // underpass deck's own span for a ground entry, D58).
    double deck_from = 0, deck_to = 0;
    std::vector<Station> stations;
};
struct CarvedJunction {
    std::size_t record = 0;                 // into Impl::junctions
    std::vector<geom::PointMm> ring;              // outer ring, mm
    std::vector<double> rx, ry, arc;        // metres; arc[k] = arc length to vertex k
    double perimeter = 0;
    std::int64_t min_x = 0, min_y = 0, max_x = 0, max_y = 0;   // mm
    std::vector<std::pair<double, double>> knots;               // (arc, blend width), ascending arc
    JunctionSurface surface;
};
enum ElemKind : std::uint8_t { kJunction = 0, kGround = 1, kBridge = 2 };
struct BlockElem { std::uint8_t kind = 0; std::uint32_t index = 0; std::uint32_t seg_begin = 0, seg_end = 0; };
struct Block { std::uint32_t elem_begin = 0, elem_end = 0; };

std::uint8_t way_rank(const RoadGraph& graph, std::int64_t way_id) {
    auto it = std::lower_bound(graph.ways.begin(), graph.ways.end(), way_id, [](const RoadGraphWay& w, std::int64_t id) { return w.osm_id < id; });
    const RoadGraphWay* way = (it != graph.ways.end() && it->osm_id == way_id) ? &*it : nullptr;
    if (!way) for (const auto& w : graph.ways) if (w.osm_id == way_id) { way = &w; break; }
    const int r = way ? way->rank : 0;
    return static_cast<std::uint8_t>(std::clamp(r, 1, 255));
}

std::string_view way_highway(const RoadGraph& graph, std::int64_t way_id) {
    auto it = std::lower_bound(graph.ways.begin(), graph.ways.end(), way_id, [](const RoadGraphWay& w, std::int64_t id) { return w.osm_id < id; });
    const RoadGraphWay* way = (it != graph.ways.end() && it->osm_id == way_id) ? &*it : nullptr;
    if (!way) for (const auto& w : graph.ways) if (w.osm_id == way_id) { way = &w; break; }
    if (!way) return {};
    for (const auto& [k, v] : way->tags) if (k == "highway") return v;
    return {};
}


// The approach station set of an entry (design 6.3): every whole metre in [0, L], the half metres within 2 m of each
// joint between reference segments (inside (0, L)), and L. `half_station` says whether half metre h / 2 is one.
bool half_station(const RoadReferenceLine& ref, std::int64_t h) noexcept {
    const double s = static_cast<double>(h) * 0.5;
    if (!(s > 0.0 && s < ref.length_m)) return false;
    for (std::size_t i = 1; i < ref.segments.size(); ++i) {
        const double joint = ref.segments[i].station_m;
        if (static_cast<double>(h) >= std::ceil((joint - 2.0) * 2.0) && s <= joint + 2.0) return true;
    }
    return false;
}
// The stations bracketing s in that set (equal when s is one).
std::pair<double, double> station_bracket(const RoadReferenceLine& ref, double s) noexcept {
    const double L = ref.length_m;
    if (s >= L) return {L, L};
    if (s <= 0.0) return {0.0, 0.0};
    double lo = std::floor(s), hi = std::ceil(s);
    if (hi > L) hi = L;
    const double h_lo = std::floor(s * 2.0), h_hi = std::ceil(s * 2.0);
    if (half_station(ref, static_cast<std::int64_t>(h_lo))) lo = std::max(lo, h_lo * 0.5);
    if (half_station(ref, static_cast<std::int64_t>(h_hi))) hi = std::min(hi, h_hi * 0.5);
    return {lo, hi};
}
// Winding-number point-in-ring, exact on mm; a point on the boundary is inside.
bool inside_ring(const std::vector<geom::PointMm>& ring, geom::PointMm p) {
    int wn = 0;
    const std::size_t n = ring.size();
    for (std::size_t k = 0; k < n; ++k) {
        const geom::PointMm a = ring[k], b = ring[(k + 1) % n];
        const int o = geom::orient(a, b, p);
        if (o == 0 && p.x >= std::min(a.x, b.x) && p.x <= std::max(a.x, b.x) && p.y >= std::min(a.y, b.y) && p.y <= std::max(a.y, b.y)) return true;
        if (a.y <= p.y) { if (b.y > p.y && o > 0) ++wn; }
        else if (b.y <= p.y && o < 0) --wn;
    }
    return wn != 0;
}

// Nearest point of the ring boundary to (x, y): squared distance, its arc position and the point. Ties keep the lowest edge.
struct RingNearest { double d2 = 0, arc = 0, px = 0, py = 0; };
RingNearest ring_nearest(const CarvedJunction& j, double x, double y) noexcept {
    RingNearest best; best.d2 = std::numeric_limits<double>::infinity();
    const std::size_t n = j.rx.size();
    for (std::size_t k = 0; k < n; ++k) {
        const std::size_t k1 = (k + 1) % n;
        const double ax = j.rx[k], ay = j.ry[k], dx = j.rx[k1] - ax, dy = j.ry[k1] - ay;
        const double len2 = dx * dx + dy * dy;
        double u = len2 > 0.0 ? ((x - ax) * dx + (y - ay) * dy) / len2 : 0.0;
        u = std::clamp(u, 0.0, 1.0);
        const double qx = ax + u * dx, qy = ay + u * dy, ex = qx - x, ey = qy - y;
        const double d2 = ex * ex + ey * ey;
        if (d2 < best.d2) {
            best.d2 = d2; best.px = qx; best.py = qy;
            best.arc = j.arc[k] + u * (j.arc[k + 1] - j.arc[k]);
        }
    }
    return best;
}

// The blend width at arc position `a`: linear between the knots along the boundary, cyclic.
double ring_blend_width(const CarvedJunction& j, double a) noexcept {
    const auto& kn = j.knots;
    const std::size_t n = kn.size();
    if (n == 0) return kNaN;
    if (n == 1) return kn.front().second;
    // The last knot at or before a; before the first knot the segment is the wrap-around one from the last knot.
    std::size_t i = n - 1;
    for (std::size_t k = 0; k < n && kn[k].first <= a; ++k) i = k;
    const std::size_t next = i + 1 < n ? i + 1 : 0;
    double span = kn[next].first - kn[i].first;
    double off = a - kn[i].first;
    if (next == 0) span += j.perimeter;   // wrap-around segment
    if (off < 0.0) off += j.perimeter;
    if (!(span > 0.0)) return kn[i].second;
    return kn[i].second + (kn[next].second - kn[i].second) * (off / span);
}

struct Projection {
    bool valid = false, outside = false;
    bool clamped = false;   // the foot is a vertex (the point lies in the wedge beyond a segment end)
    std::size_t k = 0;
    double s = 0, t = 0, u = 0;
    // Beyond a global end (outside): the distance past the end along the end segment and the signed offset from its
    // line (positive left), for the end caps (D38).
    bool at_start = false;
    double along = 0, lat = 0;
};
// Nearest segment of the line among `segs` (ascending station indices), ties to the lowest segment.
Projection project(const Line& line, const std::uint32_t* segs, std::size_t count, double x, double y) noexcept {
    Projection p;
    double best = std::numeric_limits<double>::infinity(), best_u = 0, best_uc = 0, best_cross = 0, best_len2 = 0;
    std::size_t best_k = 0;
    for (std::size_t i = 0; i < count; ++i) {
        const std::size_t k = segs[i];
        const Station& a = line.stations[k];
        const Station& b = line.stations[k + 1];
        const double dx = b.x - a.x, dy = b.y - a.y, len2 = dx * dx + dy * dy;
        const double vx = x - a.x, vy = y - a.y;
        const double u = len2 > 0.0 ? (vx * dx + vy * dy) / len2 : 0.0;
        const double uc = std::clamp(u, 0.0, 1.0);
        const double qx = a.x + uc * dx - x, qy = a.y + uc * dy - y;
        const double d2 = qx * qx + qy * qy;
        if (d2 < best) { best = d2; best_k = k; best_u = u; best_uc = uc; best_cross = dx * vy - dy * vx; best_len2 = len2; }
    }
    if (!(best < std::numeric_limits<double>::infinity())) return p;
    const Station& a = line.stations[best_k];
    const Station& b = line.stations[best_k + 1];
    p.valid = true; p.k = best_k; p.u = best_uc; p.clamped = best_u != best_uc;
    p.s = a.s + best_uc * (b.s - a.s);
    const double dist = best > 0.0 ? det::sqrt(best) : 0.0;
    p.t = best_cross >= 0.0 ? dist : -dist;
    p.outside = (a.global_first && best_u < 0.0) || (b.global_last && best_u > 1.0);
    if (p.outside && best_len2 > 0.0) {
        const double len = det::sqrt(best_len2);
        p.at_start = a.global_first && best_u < 0.0;
        p.along = p.at_start ? -best_u * len : (best_u - 1.0) * len;
        p.lat = best_cross / len;
    }
    return p;
}
} // namespace

struct CarveModel::Impl {
    const CarveTerrain* terrain = nullptr;
    CarveOptions options;
    RoadSurfaceParams surface;
    double blend_min = 0, blend_max = 0, blend_factor = 0;
    std::int64_t blend_reach_mm = 0;   // blend_max in mm, rounded up
    double deck_clearance = 0, deck_ramp = 0, deck_margin = 0;
    RoadGeomTile core, kept;
    std::vector<RoadJunction> junctions;        // every loaded record, merged entry indices
    std::vector<CarvedJunction> carved;
    std::vector<double> trim_start, trim_end;   // per merged entry
    std::vector<std::uint8_t> rank;             // per merged entry: the way's rank clamped to [1, 255] (terrain.class)
    std::vector<std::size_t> line_of;           // per merged entry: index into lines or kNone
    std::vector<Line> lines;
    std::int64_t bx0 = 0, by0 = 0, nbx = 0, nby = 0;
    std::vector<Block> blocks;
    std::vector<BlockElem> elems;
    std::vector<std::uint32_t> segs;

    double terrain_m(std::int64_t e, std::int64_t n, bool raw_dem) const noexcept {
        const std::int32_t v = raw_dem ? terrain->raw(e, n) : terrain->smoothed(e, n);
        return v == kHeightNoData ? kNaN : static_cast<double>(v) / 256.0;
    }
    // The blend width of design 6.3 at the outer edge (s, t_edge) of entry e.
    double blend_width(std::size_t e, const RoadReferenceLine& ref, double b_class, double s, double t_edge, const TileKey* count_in,
        CarveStats* stats, std::size_t* outside = nullptr) const {
        const double z_edge = road_surface_height_m(core, e, s, t_edge, surface);
        double delta = 0;
        if (const auto pt = lateral_point(ref, s, t_edge)) {
            const double ex = std::floor(pt->x), ey = std::floor(pt->y);
            const double te = terrain_m(static_cast<std::int64_t>(ex), static_cast<std::int64_t>(ey), options.blend_from_raw);
            const bool inside = terrain->in_window(static_cast<std::int64_t>(ex), static_cast<std::int64_t>(ey));
            if (!inside && outside) ++*outside;
            if (inside && te == te && z_edge == z_edge) delta = te - z_edge;
            if (stats && count_in && ex >= static_cast<double>(count_in->min_easting()) && ex < static_cast<double>(count_in->min_easting() + 256)
                && ey >= static_cast<double>(count_in->min_northing()) && ey < static_cast<double>(count_in->min_northing() + 256)) {
                ++stats->blend_stations;
                if (blend_factor * (delta < 0 ? -delta : delta) > blend_max) ++stats->steep_blend_stations;
            }
        }
        const double need = blend_factor * (delta < 0 ? -delta : delta);
        const double b = std::clamp(std::max(b_class, need), blend_min, blend_max);
        if (stats && b > stats->max_blend_width_m) stats->max_blend_width_m = b;
        return b;
    }
    // The approach's blend width at station s on one side: linear between its stations (NaN without a bracket).
    double line_blend(const Line& line, double s, bool left) const noexcept {
        const auto& st = line.stations;
        auto it = std::upper_bound(st.begin(), st.end(), s, [](double v, const Station& x) { return v < x.s; });
        if (it == st.begin()) return kNaN;
        const std::size_t k = static_cast<std::size_t>(it - st.begin()) - 1;
        const double b0 = left ? st[k].b_left : st[k].b_right;
        if (st[k].s == s || k + 1 >= st.size() || !st[k].connects_next) return st[k].s == s ? b0 : kNaN;
        const double b1 = left ? st[k + 1].b_left : st[k + 1].b_right;
        const double u = (s - st[k].s) / (st[k + 1].s - st[k].s);
        return b0 + (b1 - b0) * u;
    }
};

CarveModel::CarveModel(std::unique_ptr<Impl> impl) : impl_(std::move(impl)) {}
CarveModel::CarveModel(CarveModel&&) noexcept = default;
CarveModel& CarveModel::operator=(CarveModel&&) noexcept = default;
CarveModel::~CarveModel() = default;
const RoadGeomTile& CarveModel::core_tile() const noexcept { return impl_->core; }
const RoadGeomTile& CarveModel::kept_tile() const noexcept { return impl_->kept; }
std::size_t CarveModel::junction_count() const noexcept { return impl_->carved.size(); }
const RoadJunction& CarveModel::junction(std::size_t i) const noexcept { return impl_->junctions[impl_->carved[i].record]; }
const JunctionSurface& CarveModel::junction_surface(std::size_t i) const noexcept { return impl_->carved[i].surface; }
double CarveModel::trim_start(std::size_t entry) const noexcept { return entry < impl_->trim_start.size() ? impl_->trim_start[entry] : 0.0; }
double CarveModel::trim_end(std::size_t entry) const noexcept { return entry < impl_->trim_end.size() ? impl_->trim_end[entry] : 0.0; }
double CarveModel::blend_width_m(std::size_t entry, double s, bool left) const noexcept {
    if (entry >= impl_->line_of.size() || impl_->line_of[entry] == kNone) return kNaN;
    const Line& line = impl_->lines[impl_->line_of[entry]];
    return line.bridge ? kNaN : impl_->line_blend(line, s, left);
}
bool CarveModel::station_of(std::size_t entry, double x, double y, double& s, double& t) const {
    if (entry >= impl_->line_of.size() || impl_->line_of[entry] == kNone) return false;
    const Line& line = impl_->lines[impl_->line_of[entry]];
    std::vector<std::uint32_t> segs;
    for (std::size_t k = 0; k + 1 < line.stations.size(); ++k) if (line.stations[k].connects_next) segs.push_back(static_cast<std::uint32_t>(k));
    const Projection p = project(line, segs.data(), segs.size(), x, y);
    if (!p.valid || p.outside) return false;
    s = p.s; t = p.t;
    return true;
}

Result<CarveModel> CarveModel::build(std::span<const CarveGeomInput> inputs_in, const CarveParams& params,
    const CarveTerrain& terrain, const CarveOptions& options, const RoadRules& rules, CarveStats* stats) {
    auto impl = std::make_unique<Impl>();
    Impl& m = *impl;
    m.terrain = &terrain; m.options = options; m.surface = params.surface_params();
    m.blend_min = static_cast<double>(params.blend_min_mm) / 1000.0; m.blend_max = static_cast<double>(params.blend_max_mm) / 1000.0;
    m.blend_reach_mm = static_cast<std::int64_t>(params.blend_max_mm) + 1;
    m.blend_factor = static_cast<double>(params.blend_slope_factor);
    m.deck_clearance = static_cast<double>(params.deck.clearance_mm) / 1000.0;
    m.deck_ramp = static_cast<double>(params.deck.ramp_mm) / 1000.0;
    m.deck_margin = static_cast<double>(params.deck.margin_mm) / 1000.0;
    if (!(m.blend_min > 0 && m.blend_max >= m.blend_min && m.deck_ramp > 0)) return make_error(ErrorCode::InvalidArgument, "carve: invalid parameters");

    std::vector<CarveGeomInput> inputs(inputs_in.begin(), inputs_in.end());
    for (const auto& in : inputs) if (!in.tile) return make_error(ErrorCode::InvalidArgument, "carve: null roads.geom tile");
    std::sort(inputs.begin(), inputs.end(), [](const CarveGeomInput& a, const CarveGeomInput& b) { return a.key < b.key; });
    for (std::size_t i = 1; i < inputs.size(); ++i) if (inputs[i].key == inputs[i - 1].key) return make_error(ErrorCode::InvalidArgument, "carve: duplicate roads.geom tile");

    // Entries: unique by (way id, stretch); the copy of the lowest tile key wins. Merged order = that key's order.
    using EntryKey = std::tuple<std::int64_t, std::uint32_t, std::uint32_t>;
    std::map<EntryKey, std::pair<std::size_t, std::size_t>> chosen;
    for (std::size_t ti = 0; ti < inputs.size(); ++ti)
        for (std::size_t ei = 0; ei < inputs[ti].tile->entries.size(); ++ei) {
            const auto& e = inputs[ti].tile->entries[ei];
            chosen.try_emplace(EntryKey{e.way_id, e.stretch.start_ref, e.stretch.end_ref}, ti, ei);
        }
    std::map<EntryKey, std::size_t> merged_index;
    std::vector<std::vector<std::pair<double, double>>> deck_spans;   // per merged entry: its underpass decks (D58)
    std::vector<std::string_view> highway;
    for (const auto& [key, src] : chosen) {
        merged_index.emplace(key, m.kept.entries.size());
        const RoadGeomTile& t = *inputs[src.first].tile;
        m.kept.entries.push_back(t.entries[src.second]);
        highway.push_back(way_highway(t.source.graph, t.entries[src.second].way_id));
        m.rank.push_back(way_rank(t.source.graph, t.entries[src.second].way_id));
        deck_spans.emplace_back();
        if (options.underpass_decks)
            for (const auto& u : t.underpasses) if (u.entry_index == src.second) deck_spans.back().emplace_back(u.start_m, u.end_m);
    }
    const std::size_t n_entries = m.kept.entries.size();
    std::vector<std::vector<std::size_t>> remap(inputs.size());
    for (std::size_t ti = 0; ti < inputs.size(); ++ti) {
        const auto& entries = inputs[ti].tile->entries;
        remap[ti].resize(entries.size());
        for (std::size_t ei = 0; ei < entries.size(); ++ei)
            remap[ti][ei] = merged_index.at(EntryKey{entries[ei].way_id, entries[ei].stretch.start_ref, entries[ei].stretch.end_ref});
    }
    {   // Features of the chosen copies, re-indexed.
        std::size_t mi = 0;
        for (const auto& [key, src] : chosen) {
            (void)key;
            for (const auto& f : inputs[src.first].tile->features)
                if (f.entry_index == src.second) { ProfileFeature g = f; g.entry_index = mi; m.kept.features.push_back(g); }
            ++mi;
        }
        std::sort(m.kept.features.begin(), m.kept.features.end(), [](const ProfileFeature& a, const ProfileFeature& b) {
            return std::tie(a.entry_index, a.station_m, a.node_id) < std::tie(b.entry_index, b.station_m, b.node_id);
        });
    }
    // Junctions: unique by (owner node, ClusterTooLarge), the lowest tile key's record; entry indices re-mapped.
    std::map<std::pair<std::int64_t, bool>, RoadJunction> junction_by_owner;
    for (std::size_t ti = 0; ti < inputs.size(); ++ti)
        for (const auto& j : inputs[ti].tile->junctions) {
            if (j.nodes.empty()) return make_error(ErrorCode::Malformed, "carve: junction without nodes");
            const auto key = std::pair{j.nodes.front(), j.decline == JunctionDecline::ClusterTooLarge};
            if (junction_by_owner.count(key)) continue;
            RoadJunction r = j;
            for (auto& a : r.approaches) {
                if (a.entry_index >= remap[ti].size()) return make_error(ErrorCode::Malformed, "carve: junction approach index");
                a.entry_index = remap[ti][a.entry_index];
            }
            for (auto& x : r.absorbed) {
                if (x >= remap[ti].size()) return make_error(ErrorCode::Malformed, "carve: junction absorbed index");
                x = remap[ti][x];
            }
            junction_by_owner.emplace(key, std::move(r));
        }
    for (auto& [key, j] : junction_by_owner) { (void)key; m.junctions.push_back(std::move(j)); }
    // Features adjusted for the loaded junctions (D23, D34): the junction surfaces read the kept features, the approach
    // cores the kept features plus the raised-junction ramps.
    const JunctionFeatureAdjustment adjusted = adjust_features_for_junctions(m.kept, m.junctions);
    m.kept.features = adjusted.features;
    m.core = m.kept;
    m.core.features = adjusted.core_features();

    // Trims of every loaded, non-declined junction (OnStructure ones included: their approaches are bridges anyway).
    m.trim_start.assign(n_entries, 0.0); m.trim_end.assign(n_entries, 0.0);
    for (const auto& j : m.junctions) {
        if (j.decline != JunctionDecline::None) continue;
        for (const auto& a : j.approaches) {
            if (a.status != JunctionApproachStatus::Used) continue;
            double& trim = a.at_end ? m.trim_end[a.entry_index] : m.trim_start[a.entry_index];
            trim = std::max(trim, a.trim_m);
        }
    }

    // Lines: ground entries (carved) and bridge entries (deck clamp), stations over the culled region.
    const double rx0 = static_cast<double>(terrain.e0()) - kCullM, ry0 = static_cast<double>(terrain.n0()) - kCullM;
    const double rx1 = static_cast<double>(terrain.e0() + terrain.size()) + kCullM, ry1 = static_cast<double>(terrain.n0() + terrain.size()) + kCullM;
    const TileKey& centre = terrain.centre();
    m.line_of.assign(n_entries, kNone);
    if (stats) { stats->entries = n_entries; stats->junctions = m.junctions.size(); }
    for (std::size_t e = 0; e < n_entries; ++e) {
        const RoadGeomEntry& entry = m.kept.entries[e];
        if (!entry.profile || !entry.cross_section) continue;
        const RoadProfile& p = *entry.profile;
        if (p.attributes.tunnel) { if (stats) ++stats->tunnel_entries; continue; }
        const bool bridge = p.attributes.bridge;
        if (stats) ++(bridge ? stats->bridge_entries : stats->ground_entries);
        const RoadReferenceLine& ref = p.reference;
        const double L = ref.length_m;
        // Active station intervals: reference segments within reach of the region.
        std::vector<std::pair<double, double>> active;
        for (const auto& seg : ref.segments) {
            const double r = seg.curve.length;
            if (seg.curve.x0 + r < rx0 || seg.curve.x0 - r > rx1 || seg.curve.y0 + r < ry0 || seg.curve.y0 - r > ry1) continue;
            active.emplace_back(seg.station_m - 1.0, seg.station_m + seg.curve.length + 1.0);
        }
        if (active.empty()) continue;
        // Stations: every metre, 0.5 m within 2 m of a joint between reference segments, and L.
        std::vector<double> all;
        for (std::int64_t k = 0; static_cast<double>(k) <= L; ++k) all.push_back(static_cast<double>(k));
        for (std::size_t i = 1; i < ref.segments.size(); ++i) {
            const double joint = ref.segments[i].station_m;
            for (std::int64_t h = static_cast<std::int64_t>(std::ceil((joint - 2.0) * 2.0)); static_cast<double>(h) * 0.5 <= joint + 2.0; ++h)
                if (half_station(ref, h)) all.push_back(static_cast<double>(h) * 0.5);
        }
        all.push_back(L);
        // D58: a ground stretch is carved only outside its underpass decks; each deck is a bridge line of its own.
        const auto& decks = bridge ? std::vector<std::pair<double, double>>{} : deck_spans[e];
        for (const auto& [d0, d1] : decks) { all.push_back(d0); all.push_back(d1); }
        std::sort(all.begin(), all.end());
        all.erase(std::unique(all.begin(), all.end()), all.end());
        const auto in_deck = [&](double s) { for (const auto& [d0, d1] : decks) if (s > d0 && s < d1) return true; return false; };
        Line line; line.entry = e; line.bridge = bridge; line.length = L; line.deck_from = 0; line.deck_to = L;
        line.trim_start = m.trim_start[e]; line.trim_end = m.trim_end[e];
        line.b_class = static_cast<double>(params.blend_for_class_mm(highway[e])) / 1000.0;
        std::size_t prev_global = kNone;
        for (std::size_t g = 0; g < all.size(); ++g) {
            const double s = all[g];
            bool on = false;
            for (const auto& [a, b] : active) if (s >= a && s <= b) { on = true; break; }
            if (!on || in_deck(s)) continue;
            const auto pose = ref.at(s);
            if (!pose) return make_error(ErrorCode::Malformed, "carve: reference station " + pose.error().message);
            Station st; st.s = s; st.x = pose->x; st.y = pose->y;
            st.global_first = g == 0; st.global_last = g + 1 == all.size();
            const CarveFootprint f = carve_footprint(*entry.cross_section, s);
            st.outer_left = f.outer_left; st.outer_right = f.outer_right;
            if (!line.stations.empty() && prev_global + 1 == g) line.stations.back().connects_next = true;
            line.stations.push_back(st);
            prev_global = g;
        }
        // The deck lines (D58): the stations of each deck, its ends as the line's own ends.
        std::vector<Line> deck_lines;
        for (const auto& [d0, d1] : decks) {
            Line deck; deck.entry = e; deck.bridge = true; deck.length = L; deck.deck_from = d0; deck.deck_to = d1;
            std::size_t prev = kNone;
            for (std::size_t g = 0; g < all.size(); ++g) {
                const double s = all[g];
                if (s < d0 || s > d1) continue;
                bool on = false;
                for (const auto& [a, b] : active) if (s >= a && s <= b) { on = true; break; }
                if (!on) continue;
                const auto pose = ref.at(s);
                if (!pose) return make_error(ErrorCode::Malformed, "carve: reference station " + pose.error().message);
                Station st; st.s = s; st.x = pose->x; st.y = pose->y;
                st.global_first = s == d0; st.global_last = s == d1;
                const CarveFootprint f = carve_footprint(*entry.cross_section, s);
                st.outer_left = f.outer_left; st.outer_right = f.outer_right;
                if (!deck.stations.empty() && prev + 1 == g) deck.stations.back().connects_next = true;
                deck.stations.push_back(st);
                prev = g;
            }
            if (deck.stations.size() >= 2) deck_lines.push_back(std::move(deck));
        }
        if (stats) stats->underpass_decks += decks.size();
        if (line.stations.size() >= 2) {
            if (!bridge)
                for (auto& st : line.stations) {
                    st.b_left = m.blend_width(e, ref, line.b_class, st.s, st.outer_left, &centre, stats);
                    st.b_right = m.blend_width(e, ref, line.b_class, st.s, st.outer_right, &centre, stats);
                }
            m.line_of[e] = m.lines.size();
            m.lines.push_back(std::move(line));
        }
        for (auto& deck : deck_lines) m.lines.push_back(std::move(deck));
    }

    // Carved junctions: not declined, not OnStructure, with a surface (design 6.3, 6.4).
    for (std::size_t r = 0; r < m.junctions.size(); ++r) {
        const RoadJunction& j = m.junctions[r];
        if (j.decline != JunctionDecline::None) { if (stats) ++stats->junctions_declined; continue; }
        if (j.flags & static_cast<std::uint16_t>(JunctionFlags::OnStructure)) { if (stats) ++stats->junctions_on_structure; continue; }
        const auto& ring = j.rings[static_cast<std::size_t>(JunctionRingLayer::Outer)].vertices;
        if (ring.size() < 3) { if (stats) ++stats->junctions_no_surface; continue; }
        {   // Only junctions whose ring plus the widest blend can reach the covered region (the rest cannot touch a cell).
            std::int64_t x0 = ring.front().x, x1 = x0, y0 = ring.front().y, y1 = y0;
            for (const auto& v : ring) { x0 = std::min(x0, v.x); x1 = std::max(x1, v.x); y0 = std::min(y0, v.y); y1 = std::max(y1, v.y); }
            const std::int64_t reach = static_cast<std::int64_t>(params.blend_max_mm) + 1000;
            if (x1 + reach < terrain.e0() * 1000 || x0 - reach > (terrain.e0() + terrain.size()) * 1000 ||
                y1 + reach < terrain.n0() * 1000 || y0 - reach > (terrain.n0() + terrain.size()) * 1000) continue;
        }
        auto surface = make_junction_surface(m.kept, j, rules, m.surface, params.junction_fade_params());
        if (surface && !options.junction_relief_fade)   // sabotage: the full relief everywhere (the D22-D24 surface)
            for (auto& a : surface->approaches) a.fade_m = a.verge_fade_m = std::numeric_limits<double>::infinity();
        if (!surface) { if (stats) ++stats->junctions_no_surface; continue; }
        CarvedJunction c; c.record = r; c.ring = ring; c.surface = std::move(*surface);
        c.min_x = c.max_x = ring.front().x; c.min_y = c.max_y = ring.front().y;
        for (const auto& v : ring) {
            c.min_x = std::min(c.min_x, v.x); c.max_x = std::max(c.max_x, v.x); c.min_y = std::min(c.min_y, v.y); c.max_y = std::max(c.max_y, v.y);
            c.rx.push_back(static_cast<double>(v.x) / 1000.0); c.ry.push_back(static_cast<double>(v.y) / 1000.0);
        }
        c.arc.push_back(0.0);
        for (std::size_t k = 0; k < ring.size(); ++k) {
            const std::size_t k1 = (k + 1) % ring.size();
            const double dx = c.rx[k1] - c.rx[k], dy = c.ry[k1] - c.ry[k];
            c.arc.push_back(c.arc.back() + det::sqrt(dx * dx + dy * dy));
        }
        c.perimeter = c.arc.back();
        // Blend width knots at the outer trim points of the used approaches: the approach's own width there, i.e. the
        // linear interpolation between the two approach stations bracketing the trim station (the values the approach
        // line itself interpolates, so the seam on the extension of a trim line is continuous; computed from the
        // station set, not from this tile's culled line, so every tile gets the same knot), interpolated along the
        // boundary in between (D36).
        for (const auto& a : c.surface.approaches) {
            const RoadGeomEntry& entry = m.kept.entries[a.entry_index];
            const RoadReferenceLine& ref = entry.profile->reference;
            const double L = ref.length_m;
            const double s_trim = std::clamp(a.at_end ? a.length_m - a.trim_m : a.trim_m, 0.0, L);
            const auto [s_lo, s_hi] = station_bracket(ref, s_trim);
            const double b_class = static_cast<double>(params.blend_for_class_mm(highway[a.entry_index])) / 1000.0;
            const CarveFootprint f = carve_footprint(*entry.cross_section, s_trim);
            const CarveFootprint f_lo = carve_footprint(*entry.cross_section, s_lo);
            const CarveFootprint f_hi = carve_footprint(*entry.cross_section, s_hi);
            for (const bool left : {false, true}) {
                const auto pt = lateral_point(ref, s_trim, left ? f.outer_left : f.outer_right);
                if (!pt) continue;
                std::size_t outside = 0;
                const double b_lo = m.blend_width(a.entry_index, ref, b_class, s_lo, left ? f_lo.outer_left : f_lo.outer_right, nullptr, nullptr, &outside);
                double b = b_lo;
                if (s_hi > s_lo) {
                    const double b_hi = m.blend_width(a.entry_index, ref, b_class, s_hi, left ? f_hi.outer_left : f_hi.outer_right, nullptr, nullptr, &outside);
                    b = b_lo + (b_hi - b_lo) * ((s_trim - s_lo) / (s_hi - s_lo));
                }
                if (stats && outside) ++stats->junction_knots_outside_window;
                c.knots.emplace_back(ring_nearest(c, pt->x, pt->y).arc, b);
            }
        }
        std::sort(c.knots.begin(), c.knots.end());
        if (c.knots.empty()) { if (stats) ++stats->junctions_no_surface; continue; }
        m.carved.push_back(std::move(c));
    }
    if (stats) stats->junctions_carved = m.carved.size();

    // Buckets over the covered region.
    m.bx0 = floor_div(terrain.e0(), kCarveBucketM); m.by0 = floor_div(terrain.n0(), kCarveBucketM);
    m.nbx = floor_div(terrain.e0() + terrain.size() - 1, kCarveBucketM) - m.bx0 + 1;
    m.nby = floor_div(terrain.n0() + terrain.size() - 1, kCarveBucketM) - m.by0 + 1;
    struct Item { std::uint8_t kind; std::uint32_t index; std::uint32_t seg; };
    std::vector<std::vector<Item>> items(static_cast<std::size_t>(m.nbx * m.nby));
    const auto add_box = [&](double x0, double y0, double x1, double y1, Item it) {
        const std::int64_t b0x = std::max<std::int64_t>(floor_div(static_cast<std::int64_t>(std::floor(x0)), kCarveBucketM) - m.bx0, 0);
        const std::int64_t b1x = std::min<std::int64_t>(floor_div(static_cast<std::int64_t>(std::floor(x1)), kCarveBucketM) - m.bx0, m.nbx - 1);
        const std::int64_t b0y = std::max<std::int64_t>(floor_div(static_cast<std::int64_t>(std::floor(y0)), kCarveBucketM) - m.by0, 0);
        const std::int64_t b1y = std::min<std::int64_t>(floor_div(static_cast<std::int64_t>(std::floor(y1)), kCarveBucketM) - m.by0, m.nby - 1);
        for (std::int64_t by = b0y; by <= b1y; ++by) for (std::int64_t bx = b0x; bx <= b1x; ++bx)
            items[static_cast<std::size_t>(by * m.nbx + bx)].push_back(it);
    };
    for (std::size_t c = 0; c < m.carved.size(); ++c) {
        const auto& j = m.carved[c];
        const double r = m.blend_max + 1.0;
        add_box(static_cast<double>(j.min_x) / 1000.0 - r, static_cast<double>(j.min_y) / 1000.0 - r,
            static_cast<double>(j.max_x) / 1000.0 + r, static_cast<double>(j.max_y) / 1000.0 + r, Item{kJunction, static_cast<std::uint32_t>(c), 0});
    }
    for (std::size_t li = 0; li < m.lines.size(); ++li) {
        const Line& line = m.lines[li];
        for (std::size_t k = 0; k + 1 < line.stations.size(); ++k) {
            const Station& a = line.stations[k];
            if (!a.connects_next) continue;
            const Station& b = line.stations[k + 1];
            const double half = std::max({-a.outer_right, a.outer_left, -b.outer_right, b.outer_left, 0.0});
            const double r = half + (line.bridge ? m.deck_margin : m.blend_max) + 1.0;
            add_box(std::min(a.x, b.x) - r, std::min(a.y, b.y) - r, std::max(a.x, b.x) + r, std::max(a.y, b.y) + r,
                Item{line.bridge ? kBridge : kGround, static_cast<std::uint32_t>(li), static_cast<std::uint32_t>(k)});
        }
    }
    m.blocks.resize(items.size());
    for (std::size_t b = 0; b < items.size(); ++b) {
        auto& list = items[b];
        std::sort(list.begin(), list.end(), [](const Item& x, const Item& y) { return std::tie(x.kind, x.index, x.seg) < std::tie(y.kind, y.index, y.seg); });
        m.blocks[b].elem_begin = static_cast<std::uint32_t>(m.elems.size());
        for (std::size_t i = 0; i < list.size();) {
            BlockElem el; el.kind = list[i].kind; el.index = list[i].index;
            el.seg_begin = static_cast<std::uint32_t>(m.segs.size());
            std::size_t k = i;
            for (; k < list.size() && list[k].kind == el.kind && list[k].index == el.index; ++k) m.segs.push_back(list[k].seg);
            el.seg_end = static_cast<std::uint32_t>(m.segs.size());
            m.elems.push_back(el);
            i = k;
        }
        m.blocks[b].elem_end = static_cast<std::uint32_t>(m.elems.size());
    }
    return CarveModel(std::move(impl));
}

namespace {
// A permutation of 0..n-1 by a seeded stride (T14), the identity for seed 0.
struct Order {
    std::size_t n = 0, offset = 0, step = 1;
    Order(std::size_t count, std::uint64_t seed) : n(count) {
        if (seed == 0 || n < 2) return;
        offset = static_cast<std::size_t>(seed % n);
        step = 1 + static_cast<std::size_t>((seed / n) % (n - 1));
        const auto gcd = [](std::size_t a, std::size_t b) { while (b) { const std::size_t t = a % b; a = b; b = t; } return a; };
        while (gcd(step, n) != 1) step = step % (n - 1) + 1;
    }
    std::size_t operator[](std::size_t i) const noexcept { return (offset + i * step) % n; }
};
} // namespace

CarveSample CarveModel::evaluate(double x, double y) const noexcept {
    const Impl& m = *impl_;
    CarveSample out;
    const std::int64_t ce = static_cast<std::int64_t>(std::floor(x)), cn = static_cast<std::int64_t>(std::floor(y));
    const double T = m.terrain_m(ce, cn, false);
    out.terrain = T;
    if (!(T == T)) { out.z = kNaN; out.owner = CarveOwner::NoData; return out; }
    const double T_blend = m.options.blend_from_raw ? m.terrain_m(ce, cn, true) : T;
    out.z = T; out.owner = CarveOwner::Terrain; out.weight = 0;
    const std::int64_t bx = floor_div(ce, kCarveBucketM) - m.bx0, by = floor_div(cn, kCarveBucketM) - m.by0;
    if (bx < 0 || by < 0 || bx >= m.nbx || by >= m.nby) return out;
    const Block& block = m.blocks[static_cast<std::size_t>(by * m.nbx + bx)];
    const std::size_t count = block.elem_end - block.elem_begin;
    const Order order(count, m.options.candidate_order_seed);
    const bool first = m.options.first_come_ownership;
    const geom::PointMm pmm{static_cast<std::int64_t>(std::llround(x * 1000.0)), static_cast<std::int64_t>(std::llround(y * 1000.0))};

    // 1. Junction cores: the lowest z among them.
    bool have = false; double best_z = 0; std::size_t best_el = 0;
    for (std::size_t i = 0; i < count; ++i) {
        const BlockElem& el = m.elems[block.elem_begin + order[i]];
        if (el.kind != kJunction) continue;
        const CarvedJunction& j = m.carved[el.index];
        if (pmm.x < j.min_x || pmm.x > j.max_x || pmm.y < j.min_y || pmm.y > j.max_y || !inside_ring(j.ring, pmm)) continue;
        const double z = junction_height_m(j.surface, m.kept, x, y, m.surface);
        if (!have || (!first && z < best_z)) { have = true; best_z = z; best_el = el.index; }
    }
    if (have) { out.z = best_z; out.owner = CarveOwner::JunctionCore; out.element = best_el; out.weight = 1; }
    // 2. Approach cores (lowest z) and approach blends.
    bool have_core = false; double core_z = 0; std::size_t core_el = 0;
    bool have_blend = false; double blend_z = 0, blend_w = 0; std::size_t blend_el = 0; CarveOwner blend_owner = CarveOwner::ApproachBlend;
    const auto offer_blend = [&](double z, double w, std::size_t el, CarveOwner owner) {
        if (!have_blend || (!first && (w > blend_w || (w == blend_w && z < blend_z)))) { have_blend = true; blend_z = z; blend_w = w; blend_el = el; blend_owner = owner; }
    };
    if (!have) {
        for (std::size_t i = 0; i < count; ++i) {
            const BlockElem& el = m.elems[block.elem_begin + order[i]];
            if (el.kind != kGround) continue;
            const Line& line = m.lines[el.index];
            const Projection p = project(line, m.segs.data() + el.seg_begin, el.seg_end - el.seg_begin, x, y);
            if (!p.valid) continue;
            // A core is the band of the nearest segment at an unclamped foot; the wedges beyond the vertices on the outer
            // side of a bend belong to the blends, whose band rectangles meet the core there exactly (D40).
            if (!p.outside && !p.clamped && p.s >= line.trim_start && p.s <= line.length - line.trim_end) {
                const CarveFootprint f = carve_footprint(*m.core.entries[line.entry].cross_section, p.s);
                if (p.t >= f.outer_right && p.t <= f.outer_left) {
                    const double z = road_surface_height_m(m.core, line.entry, p.s, p.t, m.surface);
                    if (!have_core || (!first && z < core_z)) { have_core = true; core_z = z; core_el = line.entry; }
                    continue;
                }
            }
            if (have_core) continue;   // a core wins over every blend
            // Blends (D40): every segment of the line in the bucket offers the blend of its own band rectangle, the
            // largest weight wins. The rectangle is the segment's part of the carved line (between the trims, the
            // global ends where there are none) times [outer_right, outer_left]; d is the distance to it, so beyond an
            // end the blend wraps round the end line as a cap (D38) and a trimmed end overlaps the junction blend
            // beyond it, never a cut. The width is interpolated along the segment and, round a cap, across the road
            // between its two sides. Each candidate is continuous in the position, and so is their maximum: the
            // nearest segment alone would switch stations on the medial axis inside a bend, where the blend widths
            // and edge heights of the two stations differ (a step at w > 0).
            const std::uint32_t* sg = m.segs.data() + el.seg_begin;
            const RoadCrossSection& xs = *m.core.entries[line.entry].cross_section;
            const double s_lo = line.trim_start, s_hi = line.length - line.trim_end;
            for (std::size_t q = 0; q < el.seg_end - el.seg_begin; ++q) {
                const std::size_t k = sg[q];
                const Station& a = line.stations[k];
                const Station& b = line.stations[k + 1];
                if (!(s_lo <= s_hi) || b.s < s_lo || a.s > s_hi) continue;
                const double dx = b.x - a.x, dy = b.y - a.y, len2 = dx * dx + dy * dy;
                if (!(len2 > 0.0)) continue;
                const double ds = b.s - a.s;
                const double u_lo = s_lo > a.s ? (s_lo - a.s) / ds : 0.0;
                const double u_hi = s_hi < b.s ? (s_hi - a.s) / ds : 1.0;
                const double vx = x - a.x, vy = y - a.y;
                const double u = (vx * dx + vy * dy) / len2;
                const double uc = std::clamp(u, u_lo, u_hi);
                const double fx = a.x + uc * dx - x, fy = a.y + uc * dy - y;
                const double reach = std::max({-a.outer_right, a.outer_left, -b.outer_right, b.outer_left}) + m.blend_max;
                if (!(fx * fx + fy * fy < reach * reach)) continue;
                const double len = det::sqrt(len2);
                const double along = (u - uc) * len, lat = (dx * vy - dy * vx) / len;
                const double s_foot = a.s + uc * ds;
                const CarveFootprint f = carve_footprint(xs, s_foot);
                const double tl = std::clamp(lat, f.outer_right, f.outer_left);
                const double dl = lat - tl;
                const double d2 = along * along + dl * dl;
                const double bl = a.b_left + (b.b_left - a.b_left) * uc, br = a.b_right + (b.b_right - a.b_right) * uc;
                double width = 0;
                if (lat >= f.outer_left) width = bl;
                else if (lat <= f.outer_right) width = br;
                else { const double span = f.outer_left - f.outer_right; width = br + (bl - br) * ((tl - f.outer_right) / span); }
                if (!(d2 < width * width)) continue;
                const double d = d2 > 0.0 ? det::sqrt(d2) : 0.0;
                const double S = road_smooth_step(d / width);
                if (have_blend && !first && 1.0 - S < blend_w) continue;
                const double z_b = road_surface_height_m(m.core, line.entry, s_foot, tl, m.surface);
                offer_blend(z_b + (T_blend - z_b) * S, 1.0 - S, line.entry, CarveOwner::ApproachBlend);
            }
        }
        if (have_core) { out.z = core_z; out.owner = CarveOwner::ApproachCore; out.element = core_el; out.weight = 1; }
    }
    // 3. Junction blends.
    if (!have && !have_core) {
        for (std::size_t i = 0; i < count; ++i) {
            const BlockElem& el = m.elems[block.elem_begin + order[i]];
            if (el.kind != kJunction) continue;
            const CarvedJunction& j = m.carved[el.index];
            const std::int64_t reach = m.blend_reach_mm;
            if (pmm.x < j.min_x - reach || pmm.x > j.max_x + reach || pmm.y < j.min_y - reach || pmm.y > j.max_y + reach) continue;
            // Every boundary edge offers its own blend from its clamped nearest point (D40, as the approach segments).
            const std::size_t n = j.rx.size();
            for (std::size_t k = 0; k < n; ++k) {
                const std::size_t k1 = (k + 1) % n;
                const double ax = j.rx[k], ay = j.ry[k], dx = j.rx[k1] - ax, dy = j.ry[k1] - ay;
                const double len2 = dx * dx + dy * dy;
                const double u = len2 > 0.0 ? std::clamp(((x - ax) * dx + (y - ay) * dy) / len2, 0.0, 1.0) : 0.0;
                const double px = ax + u * dx, py = ay + u * dy, ex = px - x, ey = py - y;
                const double d2 = ex * ex + ey * ey;
                if (!(d2 < m.blend_max * m.blend_max)) continue;
                const double d = det::sqrt(d2);
                const double width = ring_blend_width(j, j.arc[k] + u * (j.arc[k + 1] - j.arc[k]));
                if (!(d < width)) continue;
                const double S = road_smooth_step(d / width);
                if (have_blend && !first && 1.0 - S < blend_w) continue;
                const double z_b = junction_height_m(j.surface, m.kept, px, py, m.surface);
                offer_blend(z_b + (T_blend - z_b) * S, 1.0 - S, el.index, CarveOwner::JunctionBlend);
            }
        }
        if (have_blend) { out.z = blend_z; out.owner = blend_owner; out.element = blend_el; out.weight = blend_w; }
    }
    // 4. Deck clearance clamp (design 6.4): lowers only.
    if (m.options.deck_clamp) {
        for (std::size_t i = 0; i < count; ++i) {
            const BlockElem& el = m.elems[block.elem_begin + order[i]];
            if (el.kind != kBridge) continue;
            const Line& line = m.lines[el.index];
            const Projection p = project(line, m.segs.data() + el.seg_begin, el.seg_end - el.seg_begin, x, y);
            if (!p.valid || p.outside) continue;
            const CarveFootprint f = carve_footprint(*m.core.entries[line.entry].cross_section, p.s);
            if (p.t < f.paved_right - m.deck_margin || p.t > f.paved_left + m.deck_margin) continue;
            const double top = road_surface_height_m(m.core, line.entry, p.s, std::clamp(p.t, f.paved_right, f.paved_left), m.surface);
            const double edge = std::min(p.s - line.deck_from, line.deck_to - p.s);
            const double limit = top - m.deck_clearance * road_smooth_step(edge / m.deck_ramp);
            if (limit < out.z) { out.z = limit; out.deck_clamped = true; }
        }
    }
    return out;
}

CarveClass CarveModel::classify(double x, double y, const CarveSample& c) const noexcept {
    const Impl& m = *impl_;
    CarveClass out;
    const geom::PointMm pmm{static_cast<std::int64_t>(std::llround(x * 1000.0)), static_cast<std::int64_t>(std::llround(y * 1000.0))};
    const auto road = [&](std::size_t entry, TerrainRole role, bool sidewalk) {
        out.role = role; out.base = false; out.rank = entry < m.rank.size() ? m.rank[entry] : 1;
        if (sidewalk) { out.land_class = LandClass::PavedRoad; out.surface = SurfaceKind::PavingStones; return; }
        const auto& profile = m.core.entries[entry].profile;
        out.land_class = profile ? profile->attributes.land_class : LandClass::PavedRoad;
        out.surface = profile ? profile->attributes.surface : SurfaceKind::Unknown;
    };
    const std::int64_t ce = static_cast<std::int64_t>(std::floor(x)), cn = static_cast<std::int64_t>(std::floor(y));
    const std::int64_t bx = floor_div(ce, kCarveBucketM) - m.bx0, by = floor_div(cn, kCarveBucketM) - m.by0;
    const bool in_blocks = bx >= 0 && by >= 0 && bx < m.nbx && by < m.nby;
    const Block* block = in_blocks ? &m.blocks[static_cast<std::size_t>(by * m.nbx + bx)] : nullptr;
    switch (c.owner) {
    case CarveOwner::JunctionCore: {
        const CarvedJunction& j = m.carved[c.element];
        const RoadJunction& r = m.junctions[j.record];
        // The approach that names the surface: highest rank, ties to the smallest way id (then the ring order).
        std::size_t best = kNone;
        for (const auto& a : r.approaches) {
            if (a.status != JunctionApproachStatus::Used || a.entry_index >= m.core.entries.size()) continue;
            if (best == kNone) { best = a.entry_index; continue; }
            const std::uint8_t ra = m.rank[a.entry_index], rb = m.rank[best];
            if (ra > rb || (ra == rb && m.core.entries[a.entry_index].way_id < m.core.entries[best].way_id)) best = a.entry_index;
        }
        const auto& bed = r.rings[static_cast<std::size_t>(JunctionRingLayer::RoadBed)].vertices;
        const auto& paved = r.rings[static_cast<std::size_t>(JunctionRingLayer::Paved)].vertices;
        if (best != kNone && bed.size() >= 3 && inside_ring(bed, pmm)) road(best, TerrainRole::JunctionRoadBed, false);
        else if (best != kNone && paved.size() >= 3 && inside_ring(paved, pmm)) road(best, TerrainRole::JunctionSidewalk, true);
        else out.role = TerrainRole::Verge;
        return out;
    }
    case CarveOwner::ApproachCore: {
        out.role = TerrainRole::Verge;
        if (!block) return out;
        // The core candidate of evaluate() for this entry (the same projection and core test).
        for (std::size_t i = block->elem_begin; i < block->elem_end; ++i) {
            const BlockElem& el = m.elems[i];
            if (el.kind != kGround) continue;
            const Line& line = m.lines[el.index];
            if (line.entry != c.element) continue;
            const Projection p = project(line, m.segs.data() + el.seg_begin, el.seg_end - el.seg_begin, x, y);
            if (!p.valid || p.outside || p.clamped || p.s < line.trim_start || p.s > line.length - line.trim_end) continue;
            const CarveFootprint f = carve_footprint(*m.core.entries[line.entry].cross_section, p.s);
            if (p.t < f.outer_right || p.t > f.outer_left) continue;
            if (p.t >= f.roadbed_right && p.t <= f.roadbed_left) road(line.entry, TerrainRole::RoadBed, false);
            else if (p.t >= f.paved_right && p.t <= f.paved_left) road(line.entry, TerrainRole::Sidewalk, true);
            return out;
        }
        return out;
    }
    case CarveOwner::ApproachBlend:
    case CarveOwner::JunctionBlend: out.role = TerrainRole::Blend; break;
    case CarveOwner::Terrain:
    case CarveOwner::NoData: break;
    }
    if (!block || c.owner == CarveOwner::NoData) return out;
    for (std::size_t i = block->elem_begin; i < block->elem_end; ++i) {
        const BlockElem& el = m.elems[i];
        if (el.kind != kBridge) continue;
        const Line& line = m.lines[el.index];
        const Projection p = project(line, m.segs.data() + el.seg_begin, el.seg_end - el.seg_begin, x, y);
        if (!p.valid || p.outside) continue;
        const CarveFootprint f = carve_footprint(*m.core.entries[line.entry].cross_section, p.s);
        if (p.t < f.paved_right - m.deck_margin || p.t > f.paved_left + m.deck_margin) continue;
        out.role = TerrainRole::BridgeDeck;
        break;
    }
    return out;
}

// ---- the tile ----

namespace {
// Carves into the caller's tile (a HeightTile is 256 KiB: callers keep it on the heap, never as a stack local or a
// by-value Result, or several of them overflow a 1 MiB stack under MSVC).
Result<void> carve_terrain_tile_into(HeightTile& out, const TileKey& key, std::span<const HeightTile* const> dem,
    std::span<const CarveGeomInput> roads, const CarveParams& params, const CarveOptions& options, unsigned workers, CarveStats* stats) {
    auto terrain = CarveTerrain::make(key, dem, params.smoothing);
    if (!terrain) return terrain.error();
    CarveStats local;
    auto model = CarveModel::build(roads, params, *terrain, options, builtin_road_rules_de(), &local);
    if (!model) return model.error();
    out.key = key;
    if (workers < 1) workers = 1;
    if (workers > 64) workers = 64;
    std::vector<CarveStats> part(workers);
    std::vector<int> failed(workers, 0);
    const std::int64_t e0 = key.min_easting(), n0 = key.min_northing();
    const auto run = [&](unsigned w) {
        CarveStats& s = part[w];
        for (std::int64_t j = w; j < 256; j += workers) for (std::int64_t i = 0; i < 256; ++i) {
            const CarveSample c = model->evaluate(static_cast<double>(e0 + i) + 0.5, static_cast<double>(n0 + j) + 0.5);
            std::int32_t& h = out.h[static_cast<std::size_t>(j * 256 + i)];
            switch (c.owner) {
            case CarveOwner::NoData: ++s.cells_nodata; break;
            case CarveOwner::Terrain: ++s.cells_terrain; break;
            case CarveOwner::JunctionCore: ++s.cells_junction_core; break;
            case CarveOwner::ApproachCore: ++s.cells_approach_core; break;
            case CarveOwner::JunctionBlend: ++s.cells_junction_blend; break;
            case CarveOwner::ApproachBlend: ++s.cells_approach_blend; break;
            }
            if (c.deck_clamped) {
                ++s.cells_deck_clamped;
                if (c.owner == CarveOwner::JunctionCore || c.owner == CarveOwner::ApproachCore) ++s.deck_clearance_violations;
            }
            if (c.owner == CarveOwner::NoData) { h = kHeightNoData; continue; }
            const auto q = quantise_height_half_even(c.z);
            if (!q) { failed[w] = 1; h = kHeightNoData; continue; }
            h = *q;
        }
    };
    if (workers == 1) run(0);
    else {
        std::vector<std::thread> threads;
        for (unsigned w = 0; w < workers; ++w) threads.emplace_back(run, w);
        for (auto& t : threads) t.join();
    }
    for (unsigned w = 0; w < workers; ++w) {
        if (failed[w]) return make_error(ErrorCode::LimitExceeded, "carve: height out of range");
        local.add_cells(part[w]);
    }
    out.has_nodata = local.cells_nodata > 0;
    if (stats) *stats = local;
    return {};
}
} // namespace

Result<HeightTile> carve_terrain_tile(const TileKey& key, std::span<const HeightTile* const> dem,
    std::span<const CarveGeomInput> roads, const CarveParams& params, const CarveOptions& options, unsigned workers, CarveStats* stats) {
    auto out = std::make_unique<HeightTile>();
    if (auto ok = carve_terrain_tile_into(*out, key, dem, roads, params, options, workers, stats); !ok) return ok.error();
    return std::move(*out);
}

// ---- layer and deriver ----

std::vector<TileKey> carve_road_geom_keys(const TileKey& level0) {
    const std::int64_t size2 = TileKey::kLevel0SizeM << 2;
    const std::int64_t e0 = level0.min_easting() - kCarveRoadRadiusM, e1 = level0.min_easting() + level0.size_m() + kCarveRoadRadiusM;
    const std::int64_t n0 = level0.min_northing() - kCarveRoadRadiusM, n1 = level0.min_northing() + level0.size_m() + kCarveRoadRadiusM;
    std::vector<TileKey> keys;
    for (std::int64_t y = floor_div(n0, size2); y <= floor_div(n1 - 1, size2); ++y)
        for (std::int64_t x = floor_div(e0, size2); x <= floor_div(e1 - 1, size2); ++x)
            keys.push_back(TileKey{level0.zone, 2, static_cast<std::int32_t>(x), static_cast<std::int32_t>(y)});
    std::sort(keys.begin(), keys.end());
    return keys;
}

LayerDesc terrain_carved_desc() {
    LayerDesc d;
    d.id = LayerId{std::string(kTerrainCarvedLayer), 1};
    d.kind = LayerKind::Baked;
    d.encoding = static_cast<std::uint16_t>(LayerEncoding::SrcElev);
    d.encoding_version = 1;
    LevelMask mask = 0;
    for (int l = 0; l <= kTerrainMaxLevel; ++l) mask |= level_bit(l);
    d.levels = mask;
    d.inputs = {std::string(kElevBaseLayer), std::string(kRoadGeomLayer)};
    d.content = "carved terrain height pyramid, levels 0-6 (int32 1/256 m): L0 smoothed elev.base carved by roads.geom "
                "(junction and approach cores, blends, deck clearance), L1-L6 mean-of-4 half-even";
    return d;
}

std::string terrain_carved_config_text() {
    const auto& sha = builtin_carve_default().content_sha256;
    return "g2m.terrain.carved/1 smooth+carve L0 roads.geom enc=9 radius=192 junction=D42 underpass-deck=1 carve=" +
        to_hex(std::span<const std::uint8_t>(sha.data(), sha.size())) + "; L1-L6 mean-of-4 half-even";
}

TerrainCarvedDeriver::TerrainCarvedDeriver() : desc_(terrain_carved_desc()) {}
ContentHash TerrainCarvedDeriver::config_hash() const { return Sha256::of(terrain_carved_config_text()); }

std::vector<InputTileRef> TerrainCarvedDeriver::inputs_for(const TileKey& target) const {
    if (!target.valid() || target.level < 0 || target.level > kTerrainMaxLevel) return {};
    std::vector<InputTileRef> refs;
    if (target.level == 0) {
        for (int dy = -1; dy <= 1; ++dy) for (int dx = -1; dx <= 1; ++dx)
            refs.push_back({std::string(kElevBaseLayer), TileKey{target.zone, 0, target.x + dx, target.y + dy}});
        for (const auto& k : carve_road_geom_keys(target)) refs.push_back({std::string(kRoadGeomLayer), k});
    } else {
        for (int dy = 0; dy <= 1; ++dy) for (int dx = 0; dx <= 1; ++dx)
            refs.push_back({std::string(kTerrainCarvedLayer), target.child(dx, dy)});
    }
    std::sort(refs.begin(), refs.end());
    return refs;
}

namespace {
Result<void> check_header(const InputTile& in) {
    if (in.header->key != in.ref.key || in.header->layer != in.ref.layer || in.header->layer_version != 1 ||
        in.header->body_size != in.body.size() || in.header->body_hash != content_hash(in.body))
        return make_error(ErrorCode::Malformed, "terrain.carved: dependency header mismatch");
    return {};
}
} // namespace

std::vector<CarveGeomInput> CarveL0Inputs::geom() const {
    std::vector<CarveGeomInput> out;
    for (std::size_t i = 0; i < roads.size(); ++i) out.push_back({road_keys[i], &roads[i]});
    return out;
}

Result<void> decode_carve_l0_inputs(const TileKey& target, std::span<const InputTile> inputs, CarveL0Inputs& out,
    std::string_view what_view) {
    const std::string what(what_view);
    std::optional<VerticalDatum> datum;
    bool centre = false;
    std::size_t bytes = 0;
    for (const auto& in : inputs) {
        if (in.ref.layer != kElevBaseLayer && in.ref.layer != kRoadGeomLayer) continue;   // the caller's own layers
        if (in.body.size() > kMaxBodyRawSize - bytes) return make_error(ErrorCode::LimitExceeded, what + ": input byte budget");
        bytes += in.body.size();
        if (!in.header) continue;   // absent (or rejected, G-1): NoData / no roads
        if (in.header->key != in.ref.key || in.header->layer != in.ref.layer || in.header->layer_version != 1 ||
            in.header->body_size != in.body.size() || in.header->body_hash != content_hash(in.body))
            return make_error(ErrorCode::Malformed, what + ": dependency header mismatch");
        const auto body = decode_body(in.body); if (!body) return body.error();
        if (in.ref.layer == kElevBaseLayer) {
            const std::int64_t dx = static_cast<std::int64_t>(in.ref.key.x) - target.x, dy = static_cast<std::int64_t>(in.ref.key.y) - target.y;
            if (dx < -1 || dx > 1 || dy < -1 || dy > 1 || in.ref.key.level != 0) return make_error(ErrorCode::Malformed, what + ": elev.base outside the window");
            const std::size_t slot = static_cast<std::size_t>((dy + 1) * 3 + (dx + 1));
            if (in.header->datum != VerticalDatum::DHHN2016 && in.header->datum != VerticalDatum::EGM2008)
                return make_error(ErrorCode::Unsupported, what + ": unknown elevation datum");
            if (datum && *datum != in.header->datum) return make_error(ErrorCode::Malformed, what + ": mixed elevation datums");
            datum = in.header->datum;
            if (auto ok = decode_height_tile(*body, in.ref.key, out.dem[slot]); !ok) return ok.error();
            for (auto& h : out.dem[slot].h) if (h != kHeightNoData) {
                const auto adjusted = static_cast<std::int64_t>(h) + in.header->height_offset;
                if (adjusted <= std::numeric_limits<std::int32_t>::min() || adjusted > std::numeric_limits<std::int32_t>::max())
                    return make_error(ErrorCode::LimitExceeded, what + ": elevation offset overflow");
                h = static_cast<std::int32_t>(adjusted);
            }
            out.window[slot] = &out.dem[slot];
            if (slot == 4) centre = true;
        } else {
            auto tile = decode_road_geom(*body); if (!tile) return tile.error();
            if (tile->source.key != in.ref.key) return make_error(ErrorCode::Malformed, what + ": roads.geom key mismatch");
            out.roads.push_back(std::move(*tile)); out.road_keys.push_back(in.ref.key);
        }
    }
    if (!centre) return make_error(ErrorCode::NotFound, what + ": g2m.elev.base input missing");
    // roads.geom heights are in the DEM's datum (or None for a tile without profiles).
    for (const auto& in : inputs)
        if (in.header && in.ref.layer == kRoadGeomLayer && in.header->datum != VerticalDatum::None && in.header->datum != *datum)
            return make_error(ErrorCode::Malformed, what + ": roads.geom datum differs from the DEM");
    out.datum = *datum;
    return {};
}

bool TerrainCarvedDeriver::rejected_input_is_absent(const InputTileRef& ref) const {
    return ref.layer == kRoadGeomLayer;
}

Result<DeriveResult> TerrainCarvedDeriver::derive(const DeriveRequest& request) const {
    const TileKey& target = request.target;
    if (!target.valid() || target.level < 0 || target.level > kTerrainMaxLevel)
        return make_error(ErrorCode::InvalidArgument, "terrain.carved: level out of range [0, kTerrainMaxLevel]");
    const auto expected = inputs_for(target);
    if (request.inputs.size() != expected.size()) return make_error(ErrorCode::InvalidArgument, "terrain.carved: unexpected inputs");
    for (std::size_t i = 0; i < expected.size(); ++i)
        if (!(request.inputs[i].ref == expected[i])) return make_error(ErrorCode::InvalidArgument, "terrain.carved: unexpected inputs");
    if (target.level > 0) {
        // Same layer: SW, SE, NW, NE in inputs_for order (ascending: y before x).
        std::vector<HeightTile> children(4);
        std::array<const HeightTile*, 4> ptrs{nullptr, nullptr, nullptr, nullptr};
        for (std::size_t i = 0; i < 4; ++i) {
            const InputTile& in = request.inputs[i];
            if (!in.header) continue;
            if (auto ok = check_header(in); !ok) return ok.error();
            const auto body = decode_body(in.body); if (!body) return body.error();
            if (auto ok = decode_height_tile(*body, in.ref.key, children[i]); !ok) return ok.error();
            ptrs[i] = &children[i];
        }
        auto tile_p = std::make_unique<HeightTile>(); HeightTile& tile = *tile_p; tile.key = target;
        downsample_2x2(ptrs[0], ptrs[1], ptrs[2], ptrs[3], tile);
        auto encoded = encode_height_tile(tile); if (!encoded) return encoded.error();
        auto bytes = encode_body(*encoded); if (!bytes) return bytes.error();
        return DeriveResult{std::move(*bytes), VerticalDatum::DHHN2016, 0};
    }
    CarveL0Inputs in;
    if (auto ok = decode_carve_l0_inputs(target, request.inputs, in, "terrain.carved"); !ok) return ok.error();
    const auto geom = in.geom();
    auto tile = std::make_unique<HeightTile>();
    if (auto ok = carve_terrain_tile_into(*tile, target, in.window, geom, builtin_carve_default(), {}, 1, nullptr); !ok) return ok.error();
    auto encoded = encode_height_tile(*tile); if (!encoded) return encoded.error();
    auto body = encode_body(*encoded); if (!body) return body.error();
    return DeriveResult{std::move(*body), in.datum, 0};
}
} // namespace g2m
