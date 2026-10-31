// Railways R1/R2: building g2m.rail.geom pieces from OSM railway ways and the DEM. See g2m/layer/rail_geom_tile.h.
#include "g2m/layer/rail_geom_tile.h"
#include "g2m/core/geom/project.h"
#include "g2m/layer/terrain_layers.h"
#include "g2m/layer/portal_approach.h"
#include <algorithm>
#include <cmath>
#include <set>

namespace g2m {
namespace {
using Tags = std::span<const osm::Tag>;

bool contains(const std::vector<std::string>& list, std::string_view value) {
    return std::binary_search(list.begin(), list.end(), value, [](std::string_view a, std::string_view b) { return a < b; });
}
// A small signed integer at the start of `text` (no sign other than '-', no leading space); nullopt if there is none.
std::optional<std::int64_t> leading_int(std::string_view text) {
    std::size_t i = 0; bool neg = false;
    if (i < text.size() && text[i] == '-') { neg = true; ++i; }
    if (i >= text.size() || text[i] < '0' || text[i] > '9') return std::nullopt;
    std::int64_t v = 0;
    for (; i < text.size() && text[i] >= '0' && text[i] <= '9'; ++i) {
        v = v * 10 + (text[i] - '0');
        if (v > 1000000000) return std::nullopt;
    }
    return neg ? -v : v;
}
double dist_mm(const geom::PointMm& a, const geom::PointMm& b) {
    const double dx = static_cast<double>(b.x - a.x), dy = static_cast<double>(b.y - a.y);
    return std::sqrt(dx * dx + dy * dy);
}
std::vector<double> arcs_mm(const RailWay& way) {
    std::vector<double> s(way.pts.size(), 0.0);
    for (std::size_t i = 1; i < way.pts.size(); ++i) s[i] = s[i - 1] + dist_mm(way.pts[i - 1], way.pts[i]);
    return s;
}
double length_mm(const RailWay& way) { return arcs_mm(way).back(); }
bool owned_by(const TileKey& key, const geom::PointMm& p) {
    const auto size = static_cast<std::int64_t>(key.size_m()) * 1000;
    return floor_div(p.x, size) == key.x && floor_div(p.y, size) == key.y;
}
constexpr std::uint16_t kStructure = kRailBridge | kRailTunnel;
}

std::optional<RailAttrs> resolve_rail_attrs(const osm::OsmData& data, const osm::Way& way, const RailRules& rules) {
    const Tags tags = data.tags_of(way);
    const auto railway = data.tag(tags, "railway");
    if (!railway) return std::nullopt;
    if (const auto area = data.tag(tags, "area"); area && *area == "yes") return std::nullopt;
    RailAttrs a;
    std::string_view name = *railway;
    if (name == "construction") {
        if (!rules.construction_enabled) return std::nullopt;
        const auto kind = data.tag(tags, "construction");
        if (!kind) return std::nullopt;
        name = *kind; a.condition = RailCondition::Construction;
    }
    const RailClassRules* cls = rules.find_class(name);
    if (!cls || !cls->enabled) return std::nullopt;
    a.cls = cls->id;
    const auto bridge = data.tag(tags, "bridge"), tunnel = data.tag(tags, "tunnel");
    if (tunnel && !contains(rules.tunnel_not, *tunnel)) a.flags |= kRailTunnel;
    else if (bridge && !contains(rules.bridge_not, *bridge)) a.flags |= kRailBridge;
    if (const auto v = data.tag(tags, "embankment"); v && *v == "yes") a.flags |= kRailEmbankment;
    if (const auto v = data.tag(tags, "cutting"); v && *v == "yes") a.flags |= kRailCutting;
    if (const auto v = data.tag(tags, "electrified"); v && *v != "no") a.flags |= kRailElectrified;
    if (const auto v = data.tag(tags, "layer")) {
        const auto n = leading_int(*v);
        a.layer = static_cast<std::int8_t>(n ? std::clamp<std::int64_t>(*n, -5, 5) : 0);
    }
    a.tracks = static_cast<std::uint8_t>(cls->tracks);
    if (const auto v = data.tag(tags, "tracks")) { const auto n = leading_int(*v); if (n && *n >= 1 && *n <= 8) a.tracks = static_cast<std::uint8_t>(*n); }
    a.gauge_mm = static_cast<std::uint16_t>(cls->gauge_mm);
    if (const auto v = data.tag(tags, "gauge")) { const auto n = leading_int(*v); if (n && *n >= 300 && *n <= 3000) a.gauge_mm = static_cast<std::uint16_t>(*n); }
    if (const auto v = data.tag(tags, "service")) {
        if (*v == "siding") a.service = RailService::Siding; else if (*v == "yard") a.service = RailService::Yard;
        else if (*v == "spur") a.service = RailService::Spur; else if (*v == "crossover") a.service = RailService::Crossover;
    }
    if (const auto v = data.tag(tags, "usage")) {
        if (*v == "main") a.usage = RailUsage::Main; else if (*v == "branch") a.usage = RailUsage::Branch;
        else if (*v == "industrial") a.usage = RailUsage::Industrial; else if (*v == "military") a.usage = RailUsage::Military;
        else if (*v == "tourism") a.usage = RailUsage::Tourism; else if (*v == "test") a.usage = RailUsage::Test;
    }
    return a;
}

Result<RailWorld> collect_rail_world(std::span<const osm::OsmData* const> tiles, geo::UtmZone zone, const RailRules& rules) {
    if (zone.zone < 1 || zone.zone > 60) return make_error(ErrorCode::InvalidArgument, "rail.geom: invalid UTM zone");
    const geo::Utm utm(geo::kGRS80);
    RailWorld world;
    for (const osm::OsmData* data : tiles) {
        if (!data) return make_error(ErrorCode::InvalidArgument, "rail.geom: null OSM tile");
        for (const auto& way : data->ways) {
            const auto attrs = resolve_rail_attrs(*data, way, rules);
            if (!attrs) continue;
            RailWay out; out.id = way.id; out.attrs = *attrs;
            bool far = false;
            for (const auto ref : data->refs_of(way)) {
                if (!out.refs.empty() && out.refs.back() == ref) continue;
                const osm::Node* node = data->find_node(ref);
                if (!node) return make_error(ErrorCode::Malformed, "rail.geom: unresolved way ref");
                const auto position = geom::project_e7_to_mm(utm, zone, node->lat, node->lon);
                if (!position) { far = true; break; }
                if (!out.pts.empty() && out.pts.back() == *position) continue;   // zero-length segment: keep the first node
                out.refs.push_back(ref); out.pts.push_back(*position);
            }
            if (far || out.refs.size() < 2) continue;
            const auto [it, inserted] = world.ways.emplace(out.id, out);
            if (!inserted && !(it->second == out)) return make_error(ErrorCode::Malformed, "rail.geom: conflicting way copies");
        }
    }
    for (const auto& [id, way] : world.ways) for (const auto ref : way.refs) {
        auto& list = world.node_ways[ref];
        if (list.empty() || list.back() != id) list.push_back(id);
    }
    return world;
}

std::vector<RailPiecePlan> plan_way_pieces(const RailWay& way, const RailRules& rules) {
    std::vector<RailPiecePlan> out;
    if (way.pts.size() < 2) return out;
    const auto s = arcs_mm(way);
    const double limit = static_cast<double>(rules.piece_max_length_m) * 1000.0;
    std::size_t a = 0; std::uint32_t index = 0;
    while (a + 1 < way.pts.size()) {
        std::size_t b = a + 1;
        while (b + 1 < way.pts.size() && s[b] - s[a] < limit) ++b;
        out.push_back({&way, index++, a, b, s[a]});
        a = b;
    }
    return out;
}

std::vector<RailPiecePlan> plan_owned_pieces(const RailWorld& world, const TileKey& key, const RailRules& rules) {
    std::vector<RailPiecePlan> out;
    for (const auto& [id, way] : world.ways) {
        // A way far from the tile has no vertex in it: skip the arc computation.
        bool touches = false;
        for (const auto& p : way.pts) if (owned_by(key, p)) { touches = true; break; }
        if (!touches) continue;
        for (const auto& plan : plan_way_pieces(way, rules)) if (owned_by(key, way.pts[plan.first])) out.push_back(plan);
    }
    return out;
}

RailChain rail_chain(const RailWorld& world, const RailWay& way, const RailRules& rules) {
    RailChain c;
    c.node_a = way.refs.front(); c.node_b = way.refs.back(); c.pos_a = way.pts.front(); c.pos_b = way.pts.back();
    const std::uint16_t kind = way.attrs.flags & kStructure;
    const double own = length_mm(way);
    c.chain_mm = own;
    if (kind == 0) return c;
    const double budget = static_cast<double>(rules.chain_walk_budget_m) * 1000.0;
    const auto walk = [&](bool from_start, std::int64_t& node, geom::PointMm& pos, double& ext) {
        const RailWay* cur = &way;
        node = from_start ? way.refs.front() : way.refs.back(); pos = from_start ? way.pts.front() : way.pts.back(); ext = 0;
        std::set<std::int64_t> seen{way.id};
        for (int ways = 0; ways < rules.chain_max_ways && ext < budget; ++ways) {
            const auto it = world.node_ways.find(node);
            if (it == world.node_ways.end()) break;
            std::int64_t next_id = 0; int others = 0;
            for (const auto id : it->second) if (id != cur->id) { next_id = id; ++others; }
            if (others != 1) break;
            const RailWay& next = world.ways.at(next_id);
            if ((next.attrs.flags & kStructure) != kind || seen.count(next.id)) break;
            if (next.refs.front() == node) { node = next.refs.back(); pos = next.pts.back(); }
            else if (next.refs.back() == node) { node = next.refs.front(); pos = next.pts.front(); }
            else break;   // the other way passes through the node: a junction, not a continuation
            ext += length_mm(next); seen.insert(next.id); cur = &next;
        }
    };
    walk(true, c.node_a, c.pos_a, c.ext_a_mm);
    walk(false, c.node_b, c.pos_b, c.ext_b_mm);
    c.chain_mm = c.ext_a_mm + own + c.ext_b_mm;
    return c;
}

// 7a-1: the polyline of the lowest-id non-structure ("joining ground") way at `node`, running outward from the node (first
// point = the node); empty where no ground way joins. Shared by the anchor and by the DEM dependency set.
static std::vector<geom::PointMm> rail_approach_polyline(const RailWorld& world, std::int64_t node) {
    std::vector<geom::PointMm> line;
    const auto it = world.node_ways.find(node);
    if (it == world.node_ways.end()) return line;
    for (const auto id : it->second) {
        const RailWay& w = world.ways.at(id);
        if (w.attrs.flags & kStructure) continue;
        const auto at = std::find(w.refs.begin(), w.refs.end(), node);
        if (at == w.refs.end()) continue;
        const auto index = static_cast<std::size_t>(at - w.refs.begin());
        if (index + 1 < w.pts.size()) line.assign(w.pts.begin() + static_cast<std::ptrdiff_t>(index), w.pts.end());
        else line.assign(w.pts.rbegin(), w.pts.rend());
        break;
    }
    return line;
}

namespace {
// Adds the level-0 tiles within `margin_mm` of segment (a, b) (a == b: a point) to `keys`.
Result<void> add_segment_tiles(std::set<TileKey>& keys, geo::UtmZone zone, const geom::PointMm& a, const geom::PointMm& b,
    std::int64_t margin_mm, std::size_t max_tiles) {
    constexpr std::int64_t size = 256000;
    const auto x0 = floor_div(std::min(a.x, b.x) - margin_mm - 1, size), x1 = floor_div(std::max(a.x, b.x) + margin_mm, size);
    const auto y0 = floor_div(std::min(a.y, b.y) - margin_mm - 1, size), y1 = floor_div(std::max(a.y, b.y) + margin_mm, size);
    if (x0 < TileKey::kIndexMin || x1 > TileKey::kIndexMax || y0 < TileKey::kIndexMin || y1 > TileKey::kIndexMax)
        return make_error(ErrorCode::LimitExceeded, "rail.geom: coordinates outside tile grid");
    if (x1 - x0 + 1 > 65536 || y1 - y0 + 1 > 65536 || (x1 - x0 + 1) * (y1 - y0 + 1) > 65536)
        return make_error(ErrorCode::LimitExceeded, "rail.geom: segment search budget");
    TileKey key{zone, 0, 0, 0};
    for (auto y = y0; y <= y1; ++y) for (auto x = x0; x <= x1; ++x) {
        if (!geom::segment_touches_box(a.x, a.y, b.x, b.y, x * size - margin_mm, y * size - margin_mm, (x + 1) * size + margin_mm, (y + 1) * size + margin_mm)) continue;
        key.x = static_cast<std::int32_t>(x); key.y = static_cast<std::int32_t>(y); keys.insert(key);
        if (keys.size() > max_tiles) return make_error(ErrorCode::LimitExceeded, "rail.geom: DEM working-set budget");
    }
    return {};
}
}

Result<std::vector<InputTileRef>> rail_dem_dependencies(const RailWorld& world, const TileKey& key, const RailRules& rules, std::size_t max_tiles,
    bool portal_anchor) {
    if (!key.valid() || key.level != 2) return make_error(ErrorCode::InvalidArgument, "rail.geom: level-2 target required");
    const std::int64_t margin = static_cast<std::int64_t>(rules.dependency_margin_m) * 1000;
    std::set<TileKey> keys;
    std::set<std::int64_t> chained;
    // 7a-1: the segments of the joining ground way within kPortalApproachFarM of `node` (the approach samples are bilinear, so
    // the margin applies): what rail_node_anchor reads. The whole segments are listed - over-inclusive, deterministic.
    const auto add_approach = [&](std::int64_t node) -> Result<void> {
        const auto line = rail_approach_polyline(world, node);
        double walked = 0;
        for (std::size_t i = 0; i + 1 < line.size() && walked <= kPortalApproachFarM + 1.0; ++i) {
            const auto r = add_segment_tiles(keys, key.zone, line[i], line[i + 1], margin, max_tiles); if (!r) return r;
            const double dx = static_cast<double>(line[i + 1].x - line[i].x) / 1000.0, dy = static_cast<double>(line[i + 1].y - line[i].y) / 1000.0;
            walked += std::sqrt(dx * dx + dy * dy);
        }
        return {};
    };
    const auto touches_structure = [&](std::int64_t node) {
        const auto it = world.node_ways.find(node);
        if (it == world.node_ways.end()) return false;
        return std::any_of(it->second.begin(), it->second.end(), [&](auto id) { return (world.ways.at(id).attrs.flags & kStructure) != 0; });
    };
    for (const auto& plan : plan_owned_pieces(world, key, rules)) {
        const RailWay& way = *plan.way;
        for (std::size_t i = plan.first; i < plan.last; ++i) {
            const auto r = add_segment_tiles(keys, key.zone, way.pts[i], way.pts[i + 1], margin, max_tiles); if (!r) return r.error();
        }
        if ((way.attrs.flags & kStructure) && chained.insert(way.id).second) {
            const RailChain c = rail_chain(world, way, rules);
            for (const auto& p : {c.pos_a, c.pos_b}) { const auto r = add_segment_tiles(keys, key.zone, p, p, margin, max_tiles); if (!r) return r.error(); }
            if (portal_anchor) for (const auto node : {c.node_a, c.node_b}) { const auto r = add_approach(node); if (!r) return r.error(); }
        } else if (portal_anchor && !(way.attrs.flags & kStructure)) {
            for (const auto vertex : {plan.first, plan.last})
                if (touches_structure(way.refs[vertex])) { const auto r = add_approach(way.refs[vertex]); if (!r) return r.error(); }
        }
    }
    std::vector<InputTileRef> out; out.reserve(keys.size());
    for (const auto& k : keys) out.push_back({std::string(kElevBaseLayer), k});
    return out;
}

std::vector<std::int32_t> rail_ground_profile(std::span<const double> s_m, std::span<const std::int32_t> raw,
    std::int32_t smoothing_m, std::int32_t max_grade_permille) {
    const std::size_t n = raw.size();
    std::vector<std::int32_t> result(raw.begin(), raw.end());
    if (n < 3 || s_m.size() != n) return result;
    std::vector<double> z(n), out(n);
    for (std::size_t i = 0; i < n; ++i) z[i] = out[i] = static_cast<double>(raw[i]);
    const double width = static_cast<double>(smoothing_m);
    if (width > 0) for (std::size_t i = 1; i + 1 < n; ++i) {
        const double e = std::min(s_m[i] - s_m[0], s_m[n - 1] - s_m[i]);
        const double t = e >= width ? 1.0 : e / width;
        double sum = 0, wsum = 0;
        for (std::size_t j = i + 1; j-- > 0;) {
            const double d = s_m[i] - s_m[j]; if (d >= width) break;
            sum += (width - d) * z[j]; wsum += width - d;
        }
        for (std::size_t j = i + 1; j < n; ++j) {
            const double d = s_m[j] - s_m[i]; if (d >= width) break;
            sum += (width - d) * z[j]; wsum += width - d;
        }
        out[i] = z[i] + t * (sum / wsum - z[i]);
    }
    const double grade = static_cast<double>(max_grade_permille) / 1000.0 * 256.0;   // 1/256 m per metre
    for (std::size_t i = 1; i < n; ++i) {
        const double lim = grade * (s_m[i] - s_m[i - 1]);
        out[i] = std::clamp(out[i], out[i - 1] - lim, out[i - 1] + lim);
    }
    out[n - 1] = z[n - 1];
    for (std::size_t i = n - 1; i-- > 1;) {
        const double lim = grade * (s_m[i + 1] - s_m[i]);
        out[i] = std::clamp(out[i], out[i + 1] - lim, out[i + 1] + lim);
    }
    for (std::size_t i = 0; i < n; ++i) result[i] = static_cast<std::int32_t>(std::floor(out[i] + 0.5));
    return result;
}

// 7a-1: the ground height a bridge/tunnel chain end and a ground piece ending at the same node both take: the approach
// level of the lowest-id joining ground way (median DEM 2-8 m out along it, carried back along its grade); the DEM at
// the node where no ground way joins or the approach is missing. A pure function of (world, node, DEM near the node).
static std::optional<std::int32_t> rail_node_anchor(const RailWorld& world, const RoadDemSampler& dem, geo::UtmZone zone,
    std::int64_t node, const geom::PointMm& pos, bool portal_anchor) {
    if (portal_anchor) {
        const auto line = rail_approach_polyline(world, node);
        if (!line.empty()) if (const auto approach = portal_approach(dem, zone, line)) return approach->height;
    }
    const auto v = dem.sample(zone, pos); if (!v) return std::nullopt; return static_cast<std::int32_t>(*v);
}

Result<RailGeomTile> build_rail_geom_tile(const RailWorld& world, const TileKey& key, const RailRules& rules, const RoadDemSampler& dem, bool portal_anchor) {
    if (!key.valid() || key.level != 2) return make_error(ErrorCode::InvalidArgument, "rail.geom: level-2 target required");
    RailGeomTile tile; tile.key = key;
    std::map<std::int64_t, std::vector<double>> arc_cache;
    std::map<std::int64_t, RailChain> chain_cache;
    // A DEM miss declines the piece; any other error is the caller's problem.
    const auto sample = [&](const geom::PointMm& p, bool& declined) -> std::int32_t {
        const auto v = dem.sample(key.zone, p);
        if (!v) { declined = true; return 0; }
        return *v;
    };
    for (const auto& plan : plan_owned_pieces(world, key, rules)) {
        const RailWay& way = *plan.way;
        RailPiece piece;
        piece.way_id = way.id; piece.piece = plan.index; piece.cls = way.attrs.cls; piece.condition = way.attrs.condition;
        piece.flags = way.attrs.flags; piece.layer = way.attrs.layer; piece.tracks = way.attrs.tracks;
        piece.service = way.attrs.service; piece.usage = way.attrs.usage; piece.gauge_mm = way.attrs.gauge_mm;
        const bool structure = (way.attrs.flags & kStructure) != 0;
        bool declined = false;
        if (structure) {
            for (std::size_t i = plan.first; i <= plan.last; ++i) piece.xy.push_back(way.pts[i]);
            auto arcs = arc_cache.find(way.id);
            if (arcs == arc_cache.end()) arcs = arc_cache.emplace(way.id, arcs_mm(way)).first;
            auto chain = chain_cache.find(way.id);
            if (chain == chain_cache.end()) chain = chain_cache.emplace(way.id, rail_chain(world, way, rules)).first;
            const RailChain& c = chain->second;
            const auto anchor = [&](std::int64_t node, const geom::PointMm& pos) -> std::int32_t {
                const auto v = rail_node_anchor(world, dem, key.zone, node, pos, portal_anchor);
                if (!v) { declined = true; return 0; }
                return *v;
            };
            const std::int32_t za = anchor(c.node_a, c.pos_a), zb = anchor(c.node_b, c.pos_b);
            piece.z.resize(piece.xy.size(), 0);
            if (!declined) for (std::size_t i = plan.first; i <= plan.last; ++i) {
                const double f = c.chain_mm > 0 ? (c.ext_a_mm + arcs->second[i]) / c.chain_mm : 0.0;
                piece.z[i - plan.first] = static_cast<std::int32_t>(std::floor(static_cast<double>(za) + (static_cast<double>(zb) - static_cast<double>(za)) * f + 0.5));
            }
        } else {
            // Resample to at most ground_spacing_m (widened until the piece fits piece_max_points).
            double spacing = static_cast<double>(rules.ground_spacing_m) * 1000.0;
            const auto count_for = [&](double sp) {
                std::size_t total = 1;
                for (std::size_t i = plan.first; i < plan.last; ++i) total += static_cast<std::size_t>(std::max(1.0, std::ceil(dist_mm(way.pts[i], way.pts[i + 1]) / sp)));
                return total;
            };
            while (count_for(spacing) > static_cast<std::size_t>(rules.piece_max_points)) spacing *= 2.0;
            piece.xy.push_back(way.pts[plan.first]);
            for (std::size_t i = plan.first; i < plan.last; ++i) {
                const auto& a = way.pts[i]; const auto& b = way.pts[i + 1];
                const std::int64_t sub = static_cast<std::int64_t>(std::max(1.0, std::ceil(dist_mm(a, b) / spacing)));
                for (std::int64_t k = 1; k < sub; ++k)
                    piece.xy.push_back({a.x + floor_div((b.x - a.x) * k, sub), a.y + floor_div((b.y - a.y) * k, sub)});
                piece.xy.push_back(b);
            }
            std::vector<std::int32_t> raw(piece.xy.size());
            std::vector<double> s(piece.xy.size(), 0.0);
            // A ground piece ending at a node a bridge/tunnel also touches takes that node's anchor (the chain's own).
            const auto touches_structure = [&](std::size_t vertex) {
                const auto it = world.node_ways.find(way.refs[vertex]);
                if (it == world.node_ways.end()) return false;
                return std::any_of(it->second.begin(), it->second.end(), [&](auto id) { return (world.ways.at(id).attrs.flags & kStructure) != 0; });
            };
            const bool anchor_first = portal_anchor && touches_structure(plan.first), anchor_last = portal_anchor && touches_structure(plan.last);
            for (std::size_t i = 0; i < piece.xy.size() && !declined; ++i) {
                if ((i == 0 && anchor_first) || (i + 1 == piece.xy.size() && anchor_last)) {
                    const std::size_t vertex = i == 0 ? plan.first : plan.last;
                    const auto v = rail_node_anchor(world, dem, key.zone, way.refs[vertex], way.pts[vertex], true);
                    if (!v) declined = true; else raw[i] = *v;
                } else raw[i] = sample(piece.xy[i], declined);
                if (i > 0) s[i] = s[i - 1] + dist_mm(piece.xy[i - 1], piece.xy[i]) / 1000.0;
            }
            if (!declined) {
                const auto* cls = rules.find_class_id(way.attrs.cls);
                piece.z = rail_ground_profile(s, raw, rules.smoothing_m, cls ? cls->max_grade_permille : 1000);
            } else piece.z.assign(piece.xy.size(), 0);
        }
        if (declined) { piece.flags |= kRailDeclined; piece.z.assign(piece.xy.size(), 0); }
        tile.pieces.push_back(std::move(piece));
    }
    const auto valid = validate_rail_geom_tile(tile); if (!valid) return valid.error();
    return tile;
}
}
