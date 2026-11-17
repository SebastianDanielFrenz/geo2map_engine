// Gore shoulders in the drawn ribbons (ROADS stage 1): a link landed beside a through carriageway and the carriageway are one
// pavement from the node to the gore nose; their road beds (lanes and shoulders, as build_road_ribbons draws them) must not overlap
// more than 0.05 m there. Each way's ribbon is built on its own; every bed vertex of one way is tested against the other way's bed
// rows (lateral extent per 1 m row, the row nearest the vertex's foot on the other way's reference).
//
// Recorded sabotages (2026-11-17; each makes the named tests fail, then reverted):
//   S1 road_cross_section_join.cpp resolve_cross_section: gore disabled (resolve_impl(..., false) on the served path)      -> GR1, GR2 fail (0.675 m)
//   S2 apply_gore_shoulders: ramp the shoulder on the side away from the other way                                        -> GR1, GR2 fail (0.675 m)
//   S3 apply_gore_shoulders: nose where the gap reaches the larger shoulder only (need = max, not sum)                     -> GR1, GR2 fail (0.425 m)
//   S13 road_ribbons.cpp: the 3-11 m junction fade kept at a START end that carries a join (ROADS stage 3b)             -> GR3 fails (fade 0
//       at the node for every arm that starts there: the single arm of the merge, the through arm and the link of the diverge)
#include "road_fixture.h"

#include "g2m/layer/road_rules.h"
#include "g2m/world/road_ribbons.h"

#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <cmath>
#include <map>
#include <string>
#include <vector>

namespace {
using namespace g2m;
using Tags = std::vector<std::pair<std::string, std::string>>;

const TileKey kKey{geo::UtmZone{32, geo::Hemisphere::North}, 2, 455, 5422};
constexpr double kE0 = 455.0 * 1024.0 + 512.0, kN0 = 5422.0 * 1024.0 + 512.0;   // the tile's middle: node 2 sits here

RoadGraphWay way(std::int64_t id, std::vector<std::int64_t> nodes, Tags tags) {
    std::sort(tags.begin(), tags.end());
    return RoadGraphWay{id, 3, std::move(nodes), std::move(tags)};
}
// A (2 lanes) and B (3 lanes) on +x, the link R (15 degrees) landing on B's added lane at node 2; `diverge` reverses travel.
RoadGraph scene(bool diverge) {
    RoadGraph g;
    const auto node = [&](std::int64_t id, double x, double y) { g.nodes.push_back({id, {std::llround((kE0 + x) * 1000.0), std::llround((kN0 + y) * 1000.0)}, {}}); };
    node(1, -400, 0); node(2, 0, 0); node(3, 200, 0); node(4, diverge ? 150 : -150, -40);
    const Tags a = {{"highway", "trunk"}, {"oneway", "yes"}, {"lanes", "2"}};
    const Tags b = {{"highway", "trunk"}, {"oneway", "yes"}, {"lanes", "3"}, {"placement", "right_of:1"},
                    {"turn:lanes", diverge ? "none|none|slight_right" : "none|none|merge_to_left"}};
    const Tags r = {{"highway", "trunk_link"}, {"oneway", "yes"}, {"lanes", "1"}};
    if (!diverge) { g.ways.push_back(way(10, {1, 2}, a)); g.ways.push_back(way(11, {2, 3}, b)); g.ways.push_back(way(12, {4, 2}, r)); }
    else { g.ways.push_back(way(10, {2, 3}, a)); g.ways.push_back(way(11, {1, 2}, b)); g.ways.push_back(way(12, {2, 4}, r)); }
    for (const auto& w : g.ways) for (std::size_t i = 1; i < w.node_ids.size(); ++i)
        g.edges.push_back({w.osm_id, static_cast<std::uint32_t>(i - 1), w.node_ids[i - 1], w.node_ids[i]});
    return g;
}

struct Bed {
    const RoadReferenceLine* ref = nullptr;
    std::vector<std::pair<double, double>> points;   // bed vertices, absolute UTM
    std::map<long long, std::pair<double, double>> rows;   // 1 m row (station mm) -> lateral [lo, hi]
};
double lateral(const geom::CurvePose& p, double x, double y) { return -(x - p.x) * std::sin(p.heading) + (y - p.y) * std::cos(p.heading); }
Bed bed_of(const RoadGeomTile& whole, std::int64_t way_id) {
    Bed bed;
    for (const auto& entry : whole.entries) {
        if (entry.way_id != way_id || !entry.cross_section || !entry.profile) continue;
        RoadGeomTile one = whole; one.entries = {entry}; one.features.clear();
        world::RoadRibbonOptions options; options.drape = world::RibbonDrape::PointSampled;
        const auto mesh = world::build_road_ribbons(one, kKey, [](double, double) -> std::optional<double> { return 100.0; }, 0.0, {}, options);
        const double ox = static_cast<double>(kKey.min_easting()), oy = static_cast<double>(kKey.min_northing());
        bed.ref = &entry.profile->reference;
        for (const auto& v : mesh.vertices) {
            if ((static_cast<int>(v.flags) & (16 | 32)) || v.verge > 0.5f || v.lanes == 0) continue;   // markings, overlays, verges
            const double x = ox + v.x, y = oy + v.y, s = std::clamp(static_cast<double>(v.v), 0.0, bed.ref->length_m);
            const auto p = bed.ref->at(s); REQUIRE(p);
            const double t = lateral(*p, x, y);
            auto [it, fresh] = bed.rows.try_emplace(std::llround(s * 1000.0), t, t);
            if (!fresh) { it->second.first = std::min(it->second.first, t); it->second.second = std::max(it->second.second, t); }
            bed.points.emplace_back(x, y);
        }
    }
    REQUIRE(bed.ref);
    return bed;
}
// How deep (metres) the point lies inside `bed` (0 outside or beyond its ends).
double depth_in(const Bed& bed, double x, double y) {
    double u = 0;
    for (int it = 0; it < 30; ++it) {   // the foot of the perpendicular on bed's reference
        const auto p = bed.ref->at(u); if (!p) return 0;
        const double along = (x - p->x) * std::cos(p->heading) + (y - p->y) * std::sin(p->heading);
        const double nu = std::clamp(u + along, 0.0, bed.ref->length_m);
        if (std::abs(nu - u) < 1e-7) break;
        u = nu;
    }
    const auto p = bed.ref->at(u); if (!p) return 0;
    if (std::abs((x - p->x) * std::cos(p->heading) + (y - p->y) * std::sin(p->heading)) > 0.5) return 0;
    auto it = bed.rows.lower_bound(std::llround(u * 1000.0));
    if (it == bed.rows.end()) --it;
    else if (it != bed.rows.begin() && std::llround(u * 1000.0) - std::prev(it)->first < it->first - std::llround(u * 1000.0)) --it;
    if (std::abs(it->first / 1000.0 - u) > 1.0) return 0;
    const double t = lateral(*p, x, y);
    return std::max(0.0, std::min(t - it->second.first, it->second.second - t));
}
double worst_overlap(bool diverge) {
    const auto graph = scene(diverge);
    g2m_test::GeomOptions o;
    o.height_m = [](double, double) { return 100.0; };
    o.completion = served_road_completion_options();
    auto tile = g2m_test::geometry_in_tile_with(graph, kKey, o);
    REQUIRE(tile);
    const auto a = bed_of(*tile, 10), r = bed_of(*tile, 12);
    REQUIRE(a.points.size() > 100); REQUIRE(r.points.size() > 100);
    double worst = 0;
    for (const auto& [x, y] : r.points) worst = std::max(worst, depth_in(a, x, y));
    for (const auto& [x, y] : a.points) worst = std::max(worst, depth_in(r, x, y));
    return worst;
}
} // namespace

TEST_CASE("GR1 a merge: the link's and the through way's beds do not overlap between node and gore nose", "[world][road_ribbons][road_gore]") {
    const double w = worst_overlap(false);
    INFO("worst overlap " << w << " m");
    CHECK(w <= 0.05);
}
TEST_CASE("GR2 a diverge: the same", "[world][road_ribbons][road_gore]") {
    const double w = worst_overlap(true);
    INFO("worst overlap " << w << " m");
    CHECK(w <= 0.05);
}

// ROADS stage 3b: at a landed merge node the three arms continue each other; no arm's markings fade toward the node (the 3-11 m
// junction fade stays for other nodes of degree > 2).
double min_fade_near_node(bool diverge, std::int64_t way_id) {
    const auto graph = scene(diverge);
    g2m_test::GeomOptions o;
    o.height_m = [](double, double) { return 100.0; };
    o.completion = served_road_completion_options();
    auto tile = g2m_test::geometry_in_tile_with(graph, kKey, o);
    REQUIRE(tile);
    double worst = 2;
    for (const auto& entry : tile->entries) {
        if (entry.way_id != way_id || !entry.cross_section || !entry.profile) continue;
        RoadGeomTile one = *tile; one.entries = {entry}; one.features.clear();
        world::RoadRibbonOptions options; options.drape = world::RibbonDrape::PointSampled;
        const auto mesh = world::build_road_ribbons(one, kKey, [](double, double) -> std::optional<double> { return 100.0; }, 0.0, {}, options);
        const double ox = static_cast<double>(kKey.min_easting()), oy = static_cast<double>(kKey.min_northing());
        for (const auto& v : mesh.vertices) {
            const double x = ox + v.x - kE0, y = oy + v.y - kN0;
            if (std::hypot(x, y) < 11.0) worst = std::min(worst, static_cast<double>(v.fade));
        }
    }
    return worst;
}
TEST_CASE("GR3 a landed merge node: the markings of all three arms run to the node unfaded", "[world][road_ribbons][road_gore]") {
    for (const bool diverge : {false, true}) for (const std::int64_t id : {10, 11, 12}) {
        INFO("diverge " << diverge << " way " << id);
        CHECK(min_fade_near_node(diverge, id) == 1.0);
    }
}
