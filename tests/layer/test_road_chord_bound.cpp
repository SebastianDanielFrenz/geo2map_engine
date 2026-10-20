// G2M-024: the reference line of a stretch may not leave the chord of its OSM node pairs by more than a bound
// (chord_half_width_factor * half the way's width + chord_margin_m, perpendicular distance). A two-node track beside
// a trunk road used to bow 15 m across it, because its clothoid takes its end headings from the neighbouring ways.
#include "road_fixture.h"
#include "g2m/layer/road_geom_tile.h"
#include "g2m/layer/road_reference.h"
#include <catch2/catch_test_macros.hpp>
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <string>
#include <utility>
#include <vector>
using namespace g2m;
namespace {
using Tags = std::vector<std::pair<std::string, std::string>>;
using Polyline = std::vector<std::pair<double, double>>;
constexpr std::int64_t m = 1000;

RoadGraph graph_of(std::vector<std::pair<std::int64_t, std::pair<double, double>>> pts, std::vector<RoadGraphWay> ways) {
    RoadGraph g;
    for (const auto& [id, p] : pts) g.nodes.push_back({id, {std::llround(p.first * m), std::llround(p.second * m)}});
    std::sort(g.nodes.begin(), g.nodes.end(), [](const auto& a, const auto& b) { return a.osm_id < b.osm_id; });
    std::sort(ways.begin(), ways.end(), [](const auto& a, const auto& b) { return a.osm_id < b.osm_id; });
    g.ways = std::move(ways);
    for (const auto& w : g.ways) for (std::size_t i = 1; i < w.node_ids.size(); ++i)
        g.edges.push_back({w.osm_id, static_cast<std::uint32_t>(i - 1), w.node_ids[i - 1], w.node_ids[i]});
    return g;
}
// Largest distance of the fitted line (sampled every 0.5 m) from the OSM polyline, perpendicular to its segments.
double deviation(const RoadReferenceLine& line, const Polyline& polyline) {
    double worst = 0;
    for (double s = 0;; s += 0.5) {
        const auto p = line.at(std::min(s, line.length_m));
        if (p) {
            double best = 1e18;
            for (std::size_t i = 0; i + 1 < polyline.size(); ++i) {
                const double dx = polyline[i + 1].first - polyline[i].first, dy = polyline[i + 1].second - polyline[i].second;
                const double t = std::clamp(((p->x - polyline[i].first) * dx + (p->y - polyline[i].second) * dy) / (dx * dx + dy * dy), 0.0, 1.0);
                best = std::min(best, std::hypot(p->x - polyline[i].first - t * dx, p->y - polyline[i].second - t * dy));
            }
            worst = std::max(worst, best);
        }
        if (s >= line.length_m) break;
    }
    return worst;
}
ReferenceFitLimits limits_with(double factor, double margin) {
    ReferenceFitLimits l; l.allow_curvature_exceptions = true;
    l.chord_half_width_factor = factor; l.chord_margin_m = margin;
    return l;
}
// The shipped bound as the library sees it.
ReferenceFitLimits shipped() {
    const auto& r = builtin_road_rules_de().reference;
    return limits_with(r.chord_half_width_percent / 100.0, r.chord_margin_mm / 1000.0);
}
RoadReferenceLine fit_first(const RoadGraph& g, std::int64_t way, const ReferenceFitLimits& limits) {
    const auto builder = RoadReferenceBuilder::make(g); REQUIRE(builder);
    const auto stretches = builder->stretches(way); REQUIRE(stretches); REQUIRE_FALSE(stretches->empty());
    auto fit = builder->fit_stretch(way, stretches->front(), limits); REQUIRE(fit);
    return std::move(*fit);
}
bool same_line(const RoadReferenceLine& a, const RoadReferenceLine& b) {
    if (a.segments.size() != b.segments.size() || a.length_m != b.length_m) return false;
    for (double s = 0; s <= a.length_m; s += 1.0) {
        const auto p = a.at(s), q = b.at(s);
        if (!p || !q || p->x != q->x || p->y != q->y || p->heading != q->heading) return false;
    }
    return true;
}
// The track 586697591 of the home region in miniature: a 200 m two-node track whose end nodes are the ends of two
// other ways that run away at right angles, so the join bisectors bend its clothoid into an arch.
RoadGraph track_between_bending_ways(int far_m) {
    const Tags track{{"highway", "track"}};
    return graph_of({{1, {0, 0}}, {2, {200, 0}}, {3, {0, -double(far_m)}}, {4, {200, -double(far_m)}}},
        {RoadGraphWay{10, 3, {1, 2}, track}, RoadGraphWay{11, 3, {3, 1}, track}, RoadGraphWay{12, 3, {2, 4}, track}});
}
}

TEST_CASE("A two-node track between two bending ways stays on its chord", "[layer][road_reference][chord]") {
    for (const int far_m : {400, 600, 800}) {
        INFO("far end of the neighbours " << far_m << " m");
        const auto g = track_between_bending_ways(far_m);
        const Polyline chord{{0, 0}, {200, 0}};
        const auto before = fit_first(g, 10, limits_with(0, 0));
        const auto after = fit_first(g, 10, shipped());
        const double dev_before = deviation(before, chord), dev_after = deviation(after, chord);
        std::printf("[G2M-024] track between bending ways (neighbours %d m): bow %.2f m -> %.2f m, %zu -> %zu segments\n",
            far_m, dev_before, dev_after, before.segments.size(), after.segments.size());
        CHECK(dev_before > 10);                    // the failure the rule exists for
        CHECK(dev_after < 0.01);                   // the rounded fallback of two nodes is the chord itself
        CHECK_FALSE(after.chord_bound_relaxed);
        CHECK(std::fabs(after.length_m - 200) < 1e-9);
    }
    // Neighbours that bend the clothoid less than the bound allows leave the fit alone.
    const auto mild = track_between_bending_ways(6000);
    const auto a = fit_first(mild, 10, limits_with(0, 0)), b = fit_first(mild, 10, shipped());
    CHECK(deviation(a, {{0, 0}, {200, 0}}) > 1);   // it does bow, but inside the bound
    CHECK(same_line(a, b));
}

TEST_CASE("A hairpin vertex is rounded inside the corridor instead of looping out of it", "[layer][road_reference][chord]") {
    const Tags local{{"highway", "track"}};
    const double r = 150 * 3.14159265358979 / 180, leg = 60;
    const std::pair<double, double> p2{std::round(leg + leg * std::cos(r)), std::round(leg * std::sin(r))};
    const auto g = graph_of({{1, {0, 0}}, {2, {leg, 0}}, {3, p2}}, {RoadGraphWay{10, 3, {1, 2, 3}, local}});
    const Polyline polyline{{0, 0}, {leg, 0}, p2};
    const auto before = fit_first(g, 10, limits_with(0, 0)), after = fit_first(g, 10, shipped());
    std::printf("[G2M-024] 150 degree hairpin with 60 m legs: deviation %.2f m -> %.2f m\n", deviation(before, polyline), deviation(after, polyline));
    CHECK(deviation(before, polyline) > 20);
    CHECK(deviation(after, polyline) < 3.0);       // inside the 3 m corridor of a local road
    CHECK(after.segments.size() == 3);             // straight, corner arc, straight
    CHECK(after.segments[1].piece_index == 1);
    CHECK_FALSE(after.chord_bound_relaxed);
}

TEST_CASE("A legitimate motorway curve keeps its reference line bit for bit", "[layer][road_reference][chord]") {
    // R 400 m, a node every 100 m: the clothoids bow ~3 m from their chords, well inside half a motorway's width + 3 m.
    const Tags motorway{{"highway", "motorway"}, {"oneway", "yes"}};
    const double radius = 400, step = 100.0 / radius;
    std::vector<std::pair<std::int64_t, std::pair<double, double>>> pts;
    std::vector<std::int64_t> ids;
    for (int i = 0; i <= 8; ++i) {
        pts.push_back({1 + i, {std::round(radius * std::sin(i * step)), std::round(radius * (1 - std::cos(i * step)))}});
        ids.push_back(1 + i);
    }
    const auto g = graph_of(pts, {RoadGraphWay{10, 6, ids, motorway}});
    Polyline polyline;
    for (const auto& [id, p] : pts) polyline.push_back(p);
    const auto before = fit_first(g, 10, limits_with(0, 0)), after = fit_first(g, 10, shipped());
    std::printf("[G2M-024] motorway R 400 m, 100 m nodes: %zu segments, deviation %.2f m, line identical %d\n",
        after.segments.size(), deviation(after, polyline), same_line(before, after));
    CHECK(after.segments.size() == 8);
    CHECK(deviation(after, polyline) < 4);          // the clothoids' own bow, ~3.2 m
    CHECK(same_line(before, after));
    CHECK_FALSE(after.chord_bound_relaxed);
    // A much tighter bound would have rounded this curve: the check above is not vacuous.
    const auto tight = fit_first(g, 10, limits_with(0, 0.5));
    CHECK_FALSE(same_line(before, tight));
}

TEST_CASE("A stretch the bound makes unfittable keeps its unbounded fit and says so", "[layer][road_reference][chord]") {
    // A 150 degree hairpin on a primary road: the corridor is 0.5 m, a corner arc of radius >= 1 m cannot stay in it,
    // so the rounded fallback is infeasible. The clothoid chain bows 23 m, which the bound refuses, so the fit that
    // existed before the bound is kept (a bowed road is better than none).
    const Tags primary{{"highway", "primary"}};
    const double r = 150 * 3.14159265358979 / 180, leg = 60;
    const std::pair<double, double> p2{std::round(leg + leg * std::cos(r)), std::round(leg * std::sin(r))};
    const auto g = graph_of({{1, {0, 0}}, {2, {leg, 0}}, {3, p2}}, {RoadGraphWay{10, 5, {1, 2, 3}, primary}});
    const Polyline polyline{{0, 0}, {leg, 0}, p2};
    const auto before = fit_first(g, 10, limits_with(0, 0)), after = fit_first(g, 10, shipped());
    CHECK(deviation(before, polyline) > 20);
    CHECK(after.chord_bound_relaxed);
    CHECK(same_line(before, after));
    CHECK_FALSE(before.chord_bound_relaxed);
    const auto builder = RoadReferenceBuilder::make(g); REQUIRE(builder);
    // fit() reports the flag of any of its stretches.
    const auto whole = builder->fit(10, shipped());
    REQUIRE(whole); CHECK(whole->chord_bound_relaxed);
}

TEST_CASE("The chord bound limits are validated", "[layer][road_reference][chord]") {
    const auto g = track_between_bending_ways(400);
    const auto builder = RoadReferenceBuilder::make(g); REQUIRE(builder);
    const auto stretches = builder->stretches(10); REQUIRE(stretches);
    for (const auto& [factor, margin] : std::vector<std::pair<double, double>>{{-1, 3}, {1, -1}, {11, 3}, {1, 101}, {std::nan(""), 3}, {1, std::nan("")}})
        CHECK_FALSE(builder->fit_stretch(10, stretches->front(), limits_with(factor, margin)));
    CHECK(builder->fit_stretch(10, stretches->front(), limits_with(10, 100)));
}

namespace {
const TileKey kTile{geo::UtmZone{32, geo::Hemisphere::North}, 2, 455, 5421};
constexpr double kE0 = 455.0 * 1024.0 + 512.0, kN0 = 5421.0 * 1024.0 + 512.0;
// The scene of the first test, inside tile kTile (a flat 100 m DEM, no completion, no underpass).
RoadGraph tile_scene() {
    RoadGraph g;
    const auto node = [&](std::int64_t id, double x, double y) {
        g.nodes.push_back({id, {std::llround((kE0 + x) * 1000.0), std::llround((kN0 + y) * 1000.0)}, {}});
    };
    node(1, -100, 0); node(2, 100, 0); node(3, -100, -450); node(4, 100, -450);
    const std::vector<std::pair<std::string, std::string>> track{{"highway", "track"}};
    g.ways.push_back({10, 3, {1, 2}, track});
    g.ways.push_back({11, 3, {3, 1}, track});
    g.ways.push_back({12, 3, {2, 4}, track});
    for (const auto& w : g.ways)
        for (std::size_t i = 1; i < w.node_ids.size(); ++i)
            g.edges.push_back({w.osm_id, static_cast<std::uint32_t>(i - 1), w.node_ids[i - 1], w.node_ids[i]});
    return g;
}
double built_bow(const RoadRules& rules) {
    g2m_test::GeomOptions o; o.rules = &rules;
    const auto world = tile_scene();
    const auto tile = g2m_test::geometry_in_tile_with(world, kTile, o); REQUIRE(tile);
    for (const auto& e : tile->entries) {
        if (e.way_id != 10) continue;
        REQUIRE(e.profile);
        return deviation(e.profile->reference, {{kE0 - 100, kN0}, {kE0 + 100, kN0}});
    }
    FAIL("way 10 has no entry");
    return 0;
}
}

TEST_CASE("build_road_geom_tile hands the rules' chord bound to the fit", "[layer][road_reference][chord][road_rules]") {
    RoadRules rules = builtin_road_rules_de();
    const double shipped_bow = built_bow(rules);
    rules.reference.chord_half_width_percent = 0; rules.reference.chord_margin_mm = 0;
    const double off_bow = built_bow(rules);
    // Each field alone carries the bound: percent 0 + 3 m, and 100 % + a margin wider than the bow.
    rules.reference.chord_half_width_percent = 0; rules.reference.chord_margin_mm = 3000;
    const double margin_only = built_bow(rules);
    rules.reference.chord_half_width_percent = 100; rules.reference.chord_margin_mm = 0;
    const double width_only = built_bow(rules);
    rules.reference.chord_half_width_percent = 100; rules.reference.chord_margin_mm = 60000;
    const double wide = built_bow(rules);
    std::printf("[G2M-024] built tile, bow of the two-node track: shipped %.2f m, off %.2f m, margin only %.2f m, width only %.2f m, 60 m margin %.2f m\n",
        shipped_bow, off_bow, margin_only, width_only, wide);
    CHECK(off_bow > 10);
    CHECK(shipped_bow < 0.05);
    CHECK(margin_only < 0.05);
    CHECK(width_only < 0.05);      // half the track's width alone is below the bow
    CHECK(wide == off_bow);        // a bound wider than the bow changes nothing
}
