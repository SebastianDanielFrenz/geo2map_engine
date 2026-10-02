#include "g2m/layer/road_reference.h"
#include "g2m/core/hash.h"
#include <catch2/catch_test_macros.hpp>
#include <cmath>
using namespace g2m;
namespace {
RoadGraph graph(std::vector<geom::PointMm> points, bool closed = false) {
    RoadGraph g; RoadGraphWay w{10, 1, {}, {{"highway", "service"}}};
    for (std::size_t i = 0; i < points.size(); ++i) {
        const auto id = static_cast<std::int64_t>(i + 1); g.nodes.push_back({id, points[i]}); w.node_ids.push_back(id);
    }
    if (closed) w.node_ids.push_back(1);
    for (std::size_t i = 0; i + 1 < w.node_ids.size(); ++i)
        g.edges.push_back({10, static_cast<std::uint32_t>(i), w.node_ids[i], w.node_ids[i + 1]});
    g.ways.push_back(w); return g;
}
}
TEST_CASE("Complete road reference lines retain stable pair ids across coincident coordinates", "[layer][road_reference]") {
    const auto g = graph({{0, 0}, {0, 0}, {100000, 0}, {200000, 0}});
    const auto line = fit_road_reference(g, 10); REQUIRE(line); CHECK(line->coincident_pairs == 1);
    REQUIRE(line->segments.size() == 2); CHECK(line->segments[0].original_pair_index == 1);
    CHECK(line->segments[1].original_pair_index == 2); CHECK(std::fabs(line->length_m - 200) < 1e-8);
    const auto middle = line->at(150); REQUIRE(middle); CHECK(std::fabs(middle->x - 150) < 1e-8);
    CHECK_FALSE(line->at(-1)); CHECK_FALSE(line->at(201)); CHECK_FALSE(fit_road_reference(g, 99));
}
TEST_CASE("Road reference curves are G1 at shape points and respect class bounds", "[layer][road_reference]") {
    const auto g = graph({{0, 0}, {100000, 0}, {100000, 100000}});
    ReferenceFitLimits limits; limits.max_deviation_m = 30;
    const auto line = fit_road_reference(g, 10, limits); REQUIRE(line); REQUIRE(line->segments.size() == 2);
    const auto& a = line->segments[0].curve; const auto& b = line->segments[1].curve;
    const auto end = a.at(a.length), start = b.at(0); REQUIRE(end); REQUIRE(start);
    CHECK(std::fabs(end->x - start->x) < 1e-7); CHECK(std::fabs(end->y - start->y) < 1e-7);
    CHECK(std::fabs(end->heading - start->heading) < 1e-12);
    limits.min_radius_m = 500; CHECK_FALSE(fit_road_reference(g, 10, limits));
    limits.min_radius_m = 6; limits.max_deviation_m = 0.01; limits.max_bow_m = 0.01;
    CHECK_FALSE(fit_road_reference(g, 10, limits));
    limits.max_deviation_m = 20; limits.max_length_m = 1; CHECK_FALSE(fit_road_reference(g, 10, limits));
}
TEST_CASE("Rounded road corners retain class radius and G1 joins inside the original corridor", "[layer][road_reference]") {
    ReferenceFitLimits limits; limits.max_bow_m = 3;
    const auto line = fit_road_reference(graph({{0, 0}, {100000, 0}, {100000, 100000}}), 10, limits);
    REQUIRE(line); REQUIRE(line->segments.size() == 3);
    CHECK(line->segments[1].piece_index == 1); CHECK(line->segments[1].original_pair_index == 0);
    const auto& arc = line->segments[1].curve;
    CHECK(std::fabs(arc.curvature0 - 1.0 / 6) < 1e-12); CHECK(arc.curvature_rate == 0);
    for (std::size_t i = 1; i < line->segments.size(); ++i) {
        const auto& previous = line->segments[i - 1].curve;
        const auto end = previous.at(previous.length), start = line->segments[i].curve.at(0);
        REQUIRE(end); REQUIRE(start);
        CHECK(std::fabs(end->x - start->x) < 1e-8); CHECK(std::fabs(end->y - start->y) < 1e-8);
        CHECK(std::fabs(end->heading - start->heading) < 1e-12);
    }
    const auto middle = arc.at(arc.length / 2); REQUIRE(middle);
    CHECK(std::fabs(std::sqrt((middle->x - 100) * (middle->x - 100) + middle->y * middle->y) - 6 * (std::sqrt(2) - 1)) < 1e-8);
}
TEST_CASE("Closed road reference lines join tangents without inventing topology", "[layer][road_reference]") {
    const auto g = graph({{0, 0}, {100000, 0}, {100000, 100000}, {0, 100000}}, true);
    ReferenceFitLimits limits; limits.max_deviation_m = 50;
    const auto line = fit_road_reference(g, 10, limits); REQUIRE(line);
    const auto start = line->at(0), end = line->at(line->length_m); REQUIRE(start); REQUIRE(end);
    CHECK(std::fabs(start->x - end->x) < 1e-7); CHECK(std::fabs(start->y - end->y) < 1e-7);
    CHECK(std::fabs(start->heading - end->heading) < 1e-12);
    Fnv1a64 golden;
    for (const auto& s : line->segments)
        golden.u32(s.original_pair_index).f64(s.station_m).f64(s.curve.length).f64(s.curve.heading0)
            .f64(s.curve.curvature0).f64(s.curve.curvature_rate);
    CHECK(to_hex(golden.value()) == "a0edfbfd50facd4c");
    CHECK_FALSE(fit_road_reference(graph({{0, 0}, {0, 0}}), 10));
    CHECK_FALSE(fit_road_reference(graph({{0, 0}, {100000, 0}, {0, 0}}), 10));
}
TEST_CASE("Road references split at shared junction nodes instead of forcing a smooth turn", "[layer][road_reference]") {
    auto g = graph({{0, 0}, {100000, 0}, {100000, 100000}});
    g.nodes.push_back({4, {0, 100000}}); g.ways.push_back({20, 1, {2, 4}, {{"highway", "service"}}});
    g.edges.push_back({20, 0, 2, 4});
    const auto builder = RoadReferenceBuilder::make(g); REQUIRE(builder);
    const auto line = builder->fit(10); REQUIRE(line); REQUIRE(line->segments.size() == 2);
    CHECK_FALSE(line->segments[0].break_before); CHECK(line->segments[1].break_before);
    REQUIRE(line->anchors.size() == 3); CHECK(line->anchors[1].node_id == 2); CHECK(std::fabs(line->anchors[1].station_m - 100) < 1e-8);
    const auto a = line->segments[0].curve.at(100), b = line->segments[1].curve.at(0); REQUIRE(a); REQUIRE(b);
    CHECK(std::fabs(a->x - b->x) < 1e-8); CHECK(std::fabs(a->y - b->y) < 1e-8);
    CHECK(std::fabs(a->heading - b->heading) > 1); // junction surfaces own this transition
    CHECK(builder->fit(20));
    const auto stretches = builder->stretches(10); REQUIRE(stretches); REQUIRE(stretches->size() == 2);
    const auto fitted = builder->fit_stretches(10); REQUIRE(fitted); REQUIRE(fitted->size() == 2);
    CHECK((*fitted)[0].reference); CHECK((*fitted)[1].reference);
    CHECK_FALSE(builder->fit_stretch(10, {0, 2})); CHECK_FALSE(builder->fit_stretch(10, {0, 0}));
    auto bad = g; bad.ways[0].node_ids[0] = 999; CHECK_FALSE(RoadReferenceBuilder::make(bad));
}
TEST_CASE("A declined stretch does not hide independent valid geometry", "[layer][road_reference]") {
    auto g = graph({{0, 0}, {100000, 0}, {100000, 100000}, {200000, 100000}, {300000, 100000}});
    g.nodes.push_back({6, {100000, 200000}}); g.ways.push_back({20, 1, {3, 6}, {{"highway", "service"}}});
    g.edges.push_back({20, 0, 3, 6}); const auto builder = RoadReferenceBuilder::make(g); REQUIRE(builder);
    ReferenceFitLimits limits; limits.min_radius_m = 100;
    const auto parts = builder->fit_stretches(10, limits); REQUIRE(parts); REQUIRE(parts->size() == 2);
    CHECK_FALSE((*parts)[0].reference); CHECK((*parts)[1].reference); CHECK((*parts)[1].stretch.start_ref == 2);
    CHECK_FALSE(builder->fit(10, limits));
}
