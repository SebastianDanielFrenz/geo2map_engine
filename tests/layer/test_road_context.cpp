// Street fidelity P1 slice 1: the stretch context (world node degrees instead of
// tile-halo degrees) and the shared-bisector heading at degree-two way joins.
#include "road_fixture.h"
#include "g2m/layer/road_reference.h"
#include <catch2/catch_test_macros.hpp>
#include <algorithm>
#include <cmath>
#include <cstdio>
using namespace g2m;
namespace {
const geo::UtmZone zone = g2m_test::kZone32N;
constexpr double kPi = 3.14159265358979323846;

// A synthetic world: a long way W through three level-2 tiles with a junction in
// tile 0 (a short side way U) and one far away in tile 2 (a side way V).
RoadGraph long_world() {
    RoadGraph g; constexpr std::int64_t m = 1000;
    for (int i = 0; i < 15; ++i) g.nodes.push_back({i + 1, {(100 + 200 * i) * m, (500 + (i % 3) * 15) * m}});
    RoadGraphWay w{10, 3, {}, {{"highway", "residential"}}};
    for (int i = 0; i < 15; ++i) w.node_ids.push_back(i + 1);
    g.nodes.push_back({101, {(500 + 2000) * m, 900 * m}});
    g.nodes.push_back({102, {900 * m, 150 * m}});
    RoadGraphWay v{20, 3, {13, 101}, {{"highway", "residential"}}}, u{30, 3, {5, 102}, {{"highway", "residential"}}};
    g.ways = {w, v, u};
    std::sort(g.nodes.begin(), g.nodes.end(), [](const auto& a, const auto& b) { return a.osm_id < b.osm_id; });
    for (const auto& way : g.ways) for (std::size_t i = 1; i < way.node_ids.size(); ++i)
        g.edges.push_back({way.osm_id, static_cast<std::uint32_t>(i - 1), way.node_ids[i - 1], way.node_ids[i]});
    return g;
}
bool same_geometry(const RoadGeomEntry& a, const RoadGeomEntry& b) {
    if (a.way_id != b.way_id || a.stretch.start_ref != b.stretch.start_ref || a.stretch.end_ref != b.stretch.end_ref ||
        a.profile.has_value() != b.profile.has_value() || a.decline.has_value() != b.decline.has_value()) return false;
    if (!a.profile) return a.decline->code == b.decline->code && a.decline->message == b.decline->message;
    const auto& ra = a.profile->reference; const auto& rb = b.profile->reference;
    if (ra.length_m != rb.length_m || ra.segments.size() != rb.segments.size() ||
        a.profile->vertical.segments.size() != b.profile->vertical.segments.size()) return false;
    for (std::size_t i = 0; i < ra.segments.size(); ++i) {
        const auto &x = ra.segments[i], &y = rb.segments[i];
        if (x.original_pair_index != y.original_pair_index || x.station_m != y.station_m || x.piece_index != y.piece_index ||
            x.curve.x0 != y.curve.x0 || x.curve.y0 != y.curve.y0 || x.curve.heading0 != y.curve.heading0 ||
            x.curve.length != y.curve.length || x.curve.curvature0 != y.curve.curvature0 ||
            x.curve.curvature_rate != y.curve.curvature_rate) return false;
    }
    for (std::size_t i = 0; i < a.profile->vertical.segments.size(); ++i) {
        const auto &x = a.profile->vertical.segments[i], &y = b.profile->vertical.segments[i];
        if (x.station_m != y.station_m || x.length_m != y.length_m || x.height0_m != y.height0_m || x.height1_m != y.height1_m ||
            x.grade0 != y.grade0 || x.grade1 != y.grade1) return false;
    }
    return true;
}
std::vector<const RoadGeomEntry*> entries_of(const RoadGeomTile& tile, std::int64_t way) {
    std::vector<const RoadGeomEntry*> out;
    for (const auto& e : tile.entries) if (e.way_id == way) out.push_back(&e);
    return out;
}
double angle_diff(double a, double b) { return std::fabs(std::remainder(a - b, 2 * kPi)); }

struct JoinStats { std::size_t checked = 0, skipped = 0, rounded = 0; double worst = 0, worst_rounded = 0; };
// Fits both stretches at every degree-two way join and compares their end headings.
JoinStats check_joins(const RoadGraph& graph) {
    const auto builder = RoadReferenceBuilder::make(graph); REQUIRE(builder);
    ReferenceFitLimits limits; limits.allow_curvature_exceptions = true;
    JoinStats stats;
    for (const auto& way : graph.ways) {
        const auto stretches = builder->stretches(way.osm_id); REQUIRE(stretches);
        for (const auto& stretch : *stretches) for (const bool end : {false, true}) {
            const auto join = builder->join(way.osm_id, stretch, end); REQUIRE(join);
            if (!*join) continue;
            const auto theirs = builder->stretches((*join)->way_id); REQUIRE(theirs);
            const RoadStretch* other = nullptr;
            for (const auto& s : *theirs) if ((*join)->leaves_node ? s.start_ref == (*join)->ref : s.end_ref == (*join)->ref) other = &s;
            REQUIRE(other);
            if ((*join)->way_id == way.osm_id && other->start_ref == stretch.start_ref) continue; // a ring joined to itself
            const auto a = builder->fit_stretch(way.osm_id, stretch, limits), b = builder->fit_stretch((*join)->way_id, *other, limits);
            if (!a || !b) { ++stats.skipped; continue; }
            const auto pa = a->at(end ? a->length_m : 0), pb = b->at((*join)->leaves_node ? 0 : b->length_m);
            REQUIRE(pa); REQUIRE(pb);
            const bool reversed = end != (*join)->leaves_node;
            const auto diff = angle_diff(pa->heading, pb->heading + (reversed ? kPi : 0));
            const auto is_rounded = [](const RoadReferenceLine& line) {
                // The rounded() fallback (corner arcs, or a plain chord when the end headings would need more
                // curvature than the minimum radius allows) ignores the fitted end headings. Its straight pieces
                // are exactly curvature 0 / rate 0; a clothoid fit leaves rounding residue or real curvature.
                return std::any_of(line.segments.begin(), line.segments.end(), [](const auto& s) {
                    return s.piece_index == 1 || (s.curve.curvature0 == 0 && s.curve.curvature_rate == 0); });
            };
            ++stats.checked;
            if (is_rounded(*a) || is_rounded(*b)) { ++stats.rounded; stats.worst_rounded = std::max(stats.worst_rounded, diff); continue; }
            stats.worst = std::max(stats.worst, diff);
            if (diff >= 1e-9) std::printf("  join way %lld [%u,%u] end=%d <-> way %lld ref %u leaves=%d diff=%g\n", static_cast<long long>(way.osm_id),
                stretch.start_ref, stretch.end_ref, end, static_cast<long long>((*join)->way_id), (*join)->ref, (*join)->leaves_node, diff);
        }
    }
    return stats;
}
}

TEST_CASE("Stretches and profiles of a long way are identical in every tile that owns part of it", "[layer][road_context][roads]") {
    const auto world = long_world();
    const TileKey k0{zone, 2, 0, 0}, k1{zone, 2, 1, 0}, k2{zone, 2, 2, 0};
    const auto g0 = g2m_test::geometry_in_tile(world, k0), g1 = g2m_test::geometry_in_tile(world, k1), g2 = g2m_test::geometry_in_tile(world, k2);
    REQUIRE(g0); REQUIRE(g1); REQUIRE(g2);
    const auto w0 = entries_of(*g0, 10), w1 = entries_of(*g1, 10), w2 = entries_of(*g2, 10);
    // The junction with V (tile 2) and with U (tile 0) split W in all three tiles.
    REQUIRE(w0.size() == 3); REQUIRE(w1.size() == 3); REQUIRE(w2.size() == 3);
    for (std::size_t i = 0; i < 3; ++i) {
        CHECK(same_geometry(*w0[i], *w1[i])); CHECK(same_geometry(*w1[i], *w2[i]));
        CHECK(w0[i]->profile.has_value());
    }
    // Negative control: tile 1 alone does not contain V or U's far end, so without the context W is one stretch there.
    const auto lone = g2m_test::geometry_in_tile(world, k1, false); REQUIRE(lone);
    CHECK(entries_of(*lone, 10).size() < 3);
}

TEST_CASE("Context dependencies name the tiles holding the nodes and are bounded", "[layer][road_context][roads]") {
    const auto world = long_world();
    const TileKey k1{zone, 2, 1, 0};
    const auto tile = select_road_graph_tile(world, k1); REQUIRE(tile);
    const auto deps = road_context_dependencies(tile->graph, zone, &k1); REQUIRE(deps);
    REQUIRE(deps->size() == 2);
    CHECK((*deps)[0].key == TileKey{zone, 2, 0, 0}); CHECK((*deps)[1].key == TileKey{zone, 2, 2, 0});
    CHECK((*deps)[0].layer == kRoadGraphLayer);
    const auto all = road_context_dependencies(tile->graph, zone, nullptr); REQUIRE(all); CHECK(all->size() == 3);
    const auto limited = road_context_dependencies(tile->graph, zone, &k1, 1);
    REQUIRE_FALSE(limited); CHECK(limited.error().code == ErrorCode::LimitExceeded);
    // Conflicting copies of one way are an error, identical copies merge.
    auto changed = world; changed.ways[1].tags.emplace_back("lanes", "2");
    const RoadGraph* pair[] = {&world, &changed}; CHECK_FALSE(merge_road_graphs(pair));
    const RoadGraph* twice[] = {&world, &world}; const auto merged = merge_road_graphs(twice); REQUIRE(merged); CHECK(*merged == world);
}

TEST_CASE("Both stretches of a degree-two way join end on one shared bisector heading", "[layer][road_context][roads]") {
    // A -> B -> C continue in one direction, D meets A head-to-head, E meets C tail-to-tail.
    RoadGraph g; constexpr std::int64_t m = 1000;
    const std::pair<std::int64_t, std::pair<int, int>> pts[] = {
        {1, {0, 0}}, {2, {100, 0}}, {3, {200, 25}}, {4, {300, 80}}, {5, {400, 90}}, {6, {500, 60}},
        {7, {600, 100}}, {8, {-100, -30}}, {9, {-200, -10}}, {10, {700, 40}}, {11, {800, 90}}};
    for (const auto& [id, p] : pts) g.nodes.push_back({id, {p.first * m, p.second * m}});
    g.ways = {{1, 3, {1, 2, 3}, {{"highway", "tertiary"}}}, {2, 3, {3, 4, 5}, {{"highway", "tertiary"}}},
        {3, 3, {5, 6, 7}, {{"highway", "tertiary"}}}, {4, 3, {1, 8, 9}, {{"highway", "tertiary"}}},
        {5, 3, {11, 10, 7}, {{"highway", "tertiary"}}}};
    for (const auto& way : g.ways) for (std::size_t i = 1; i < way.node_ids.size(); ++i)
        g.edges.push_back({way.osm_id, static_cast<std::uint32_t>(i - 1), way.node_ids[i - 1], way.node_ids[i]});
    const auto stats = check_joins(g);
    CHECK(stats.checked == 8); // 4 joins, each seen from both sides
    CHECK(stats.rounded == 0); CHECK(stats.skipped == 0);
    CHECK(stats.worst < 1e-9);
    // The old extrapolated tangents differ here: the same-direction join A -> B kinks by degrees.
    const auto builder = RoadReferenceBuilder::make(g); REQUIRE(builder);
    const auto a = builder->fit_stretch(1, {0, 2}), b = builder->fit_stretch(2, {0, 2}); REQUIRE(a); REQUIRE(b);
    CHECK(a->at(a->length_m)->heading == b->at(0)->heading); // bit-identical for a continuing neighbour
}

TEST_CASE("Real fixture: shared-bisector headings at every degree-two way join", "[layer][road_context][roads][real-data]") {
    const auto data = g2m_test::read_osm_fixture(std::string(G2M_SOURCE_DIR) + "/testdata/fixtures/osm/frankfurt_hoechst.osm.pbf");
    const auto graph = build_road_graph(data, zone); REQUIRE(graph);
    const auto stats = check_joins(*graph);
    std::printf("slice1 fixture joins: checked=%zu skipped=%zu rounded=%zu worst=%g worst_rounded=%g\n",
        stats.checked, stats.skipped, stats.rounded, stats.worst, stats.worst_rounded);
    CHECK(stats.checked - stats.rounded > 100);
    CHECK(stats.worst < 1e-9);
}
