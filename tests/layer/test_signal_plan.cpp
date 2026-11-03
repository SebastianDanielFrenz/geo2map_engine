// Traffic signals subsystem (docs/traffic_signals.md): the timing data file, plans derived from furniture, the plan clock,
// the spatial index and the stop-or-go rule. Synthetic scenes: an X junction (two axes, four approaches) and a mid-block
// signal on a straight road (one axis).
#include "road_fixture.h"
#include "g2m/layer/road_furniture.h"
#include "g2m/layer/signal_plan.h"
#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <algorithm>
#include <cmath>
#include <functional>
#include <map>
#include <memory>
#include <set>
#include <string>
using namespace g2m;
using Catch::Approx;
namespace {
const geo::UtmZone zone{32, geo::Hemisphere::North};
const TileKey kTile{zone, 2, 455, 5421};
constexpr double kOriginE = 455.0 * 1024.0 + 512.0, kOriginN = 5421.0 * 1024.0 + 512.0;
using Tags = std::vector<std::pair<std::string, std::string>>;
struct N { std::int64_t id; double x, y; Tags tags = {}; };
struct W { std::int64_t id; std::vector<std::int64_t> nodes; Tags tags = {{"highway", "residential"}}; };
int rank_of(const Tags& tags) {
    for (const auto& [k, v] : tags) if (k == "highway") return v == "residential" ? 3 : v == "tertiary" ? 5 : v == "secondary" ? 6 : v == "primary" ? 7 : 3;
    return 3;
}
struct Scene { RoadGraph graph; RoadGeomTile tile; std::optional<RoadReferenceBuilder> builder; std::vector<RoadJunction> junctions; };
std::unique_ptr<Scene> make_scene(const std::vector<N>& nodes, std::vector<W> ways) {
    auto scene = std::make_unique<Scene>();
    for (const auto& n : nodes) {
        Tags tags = n.tags; std::sort(tags.begin(), tags.end());
        scene->graph.nodes.push_back({n.id, {std::llround((kOriginE + n.x) * 1000.0), std::llround((kOriginN + n.y) * 1000.0)}, tags});
    }
    std::sort(scene->graph.nodes.begin(), scene->graph.nodes.end(), [](const auto& a, const auto& b) { return a.osm_id < b.osm_id; });
    std::sort(ways.begin(), ways.end(), [](const auto& a, const auto& b) { return a.id < b.id; });
    for (auto& w : ways) {
        Tags tags = w.tags; std::sort(tags.begin(), tags.end());
        scene->graph.ways.push_back({w.id, rank_of(w.tags), w.nodes, tags});
        for (std::size_t i = 1; i < w.nodes.size(); ++i) scene->graph.edges.push_back({w.id, static_cast<std::uint32_t>(i - 1), w.nodes[i - 1], w.nodes[i]});
    }
    auto tile = g2m_test::geometry_in_tile(scene->graph, kTile, true, {});
    REQUIRE(tile);
    scene->tile = std::move(*tile);
    auto builder = RoadReferenceBuilder::make(scene->graph);
    REQUIRE(builder);
    scene->builder = std::move(*builder);
    auto js = build_road_junctions(scene->tile, *scene->builder);
    REQUIRE(js);
    scene->junctions = std::move(*js);
    auto furniture = build_road_furniture(scene->tile, *scene->builder, scene->junctions);
    REQUIRE(furniture);
    scene->tile.furniture = std::move(*furniture);
    return scene;
}
// An X with arms of 300 m: way 10 east-west {2,1,3}, way 11 north-south {4,1,5}; node 1 is the signal.
std::unique_ptr<Scene> x_scene(const std::string& lanes = "") {
    Tags main = {{"highway", "primary"}};
    if (!lanes.empty()) main.push_back({"lanes", lanes});
    std::vector<N> nodes = {{1, 0, 0, {{"highway", "traffic_signals"}}}, {2, -300, 0}, {3, 300, 0}, {4, 0, -300}, {5, 0, 300}};
    return make_scene(nodes, {{10, {2, 1, 3}, main}, {11, {4, 1, 5}}});
}
// A straight east-west road with a signal at node 3 (x = 0).
std::unique_ptr<Scene> line_scene() {
    std::vector<N> nodes;
    for (int k = 1; k <= 5; ++k) nodes.push_back({k, -200.0 + 100.0 * (k - 1), 0});
    nodes[2].tags = {{"highway", "traffic_signals"}};
    return make_scene(nodes, {{10, {1, 2, 3, 4, 5}}});
}
std::vector<SignalPlan> plans_of(const Scene& s, const SignalTiming& t = builtin_signal_timing(), SignalPlanStats* stats = nullptr) {
    auto p = build_signal_plans(s.tile, t, stats);
    REQUIRE(p);
    return std::move(*p);
}
// How many 50 ms instants over `cycles` cycles have two approaches on different phases both lit green or red-amber.
std::size_t conflicts(const SignalPlan& plan, int cycles = 2) {
    std::size_t bad = 0;
    for (std::int64_t t = 0; t < static_cast<std::int64_t>(cycles) * plan.cycle_ms; t += 50) {
        std::set<std::uint32_t> lit;
        for (const auto& a : plan.approaches) {
            const auto s = plan.state_of(a, t / 1000.0 + 0.0001);
            if (s.aspect == SignalAspect::Green || s.aspect == SignalAspect::RedAmber) lit.insert(a.phase);
        }
        if (lit.size() > 1) ++bad;
    }
    return bad;
}
std::vector<std::byte> bytes_of(const std::string& s) {
    std::vector<std::byte> out;
    for (const char c : s) out.push_back(static_cast<std::byte>(c));
    return out;
}
std::string builtin_text() {
    const auto b = builtin_signal_timing_bytes();
    return std::string(reinterpret_cast<const char*>(b.data()), b.size());
}
std::string replaced(std::string text, const std::string& from, const std::string& to) {
    const auto pos = text.find(from);
    REQUIRE(pos != std::string::npos);
    return text.replace(pos, from.size(), to);
}
}

TEST_CASE("S1: the embedded signal timing loads and rejects malformed files", "[layer][roads][signals][S1]") {
    const auto& t = builtin_signal_timing();
    CHECK(t.amber_ms == 3000); CHECK(t.red_amber_ms == 1000); CHECK(t.clearance_ms == 2000);
    CHECK(t.driver.comfortable_decel_mps2 == Approx(3.0)); CHECK(t.driver.heading_cos_min == Approx(0.7));
    const auto text = builtin_text();
    CHECK(load_signal_timing(bytes_of(text)));
    CHECK_FALSE(load_signal_timing(bytes_of("{")));
    CHECK_FALSE(load_signal_timing(bytes_of(replaced(text, "g2m.signal_timing/1", "g2m.signal_timing/2"))));
    CHECK_FALSE(load_signal_timing(bytes_of(replaced(text, "\"amber_ms\"", "\"amberx_ms\""))));
    CHECK_FALSE(load_signal_timing(bytes_of(replaced(text, "\"green_min_ms\": 8000", "\"green_min_ms\": 50000"))));
    CHECK_FALSE(load_signal_timing(bytes_of(replaced(text, "\"single_axis_red_ms\": 15000", "\"single_axis_red_ms\": 4000"))));
    CHECK_FALSE(load_signal_timing(bytes_of(replaced(text, "\"comfortable_decel_mmps2\": 3000", "\"comfortable_decel_mmps2\": 9000"))));
    CHECK_FALSE(load_signal_timing(bytes_of(replaced(text, "\"amber_ms\": 3000", "\"amber_ms\": 3.5"))));
}

TEST_CASE("S2: a signalised X gets one plan with a phase per axis and never two conflicting greens", "[layer][roads][signals][S2]") {
    const auto scene = x_scene();
    SignalPlanStats stats;
    const auto plans = plans_of(*scene, builtin_signal_timing(), &stats);
    REQUIRE(plans.size() == 1);
    const auto& plan = plans[0];
    CHECK(plan.node_id == 1);
    REQUIRE(plan.approaches.size() == 4);
    CHECK(stats.plans == 1); CHECK(stats.approaches == 4); CHECK(stats.heads == 4); CHECK(stats.single_axis_plans == 0);
    REQUIRE(plan.phases.size() == 2);
    // The two east-west approaches (way 10) share a phase, the two north-south ones (way 11) the other.
    std::set<std::uint32_t> ew, ns;
    for (const auto& a : plan.approaches) {
        CHECK(std::hypot(a.dir_x, a.dir_y) == Approx(1.0).margin(1e-9));
        CHECK(a.heads.size() == 1);
        CHECK(a.lanes >= 1);
        (a.way_id == 10 ? ew : ns).insert(a.phase);
        // The stop line lies on the approach side: the junction node is ahead of the line along the travel direction.
        const double ahead = (0.0 - (a.x - kOriginE)) * a.dir_x + (0.0 - (a.y - kOriginN)) * a.dir_y;
        CHECK(ahead > 0.5); CHECK(ahead < 40.0);
        // The head stands beside the line and faces the approaching traffic.
        CHECK(a.heads[0].face_x * a.dir_x + a.heads[0].face_y * a.dir_y == Approx(-1.0).margin(1e-9));
        CHECK(std::hypot(a.ax - a.bx, a.ay - a.by) > 1.0);
    }
    CHECK(ew.size() == 1); CHECK(ns.size() == 1); CHECK(*ew.begin() != *ns.begin());
    CHECK(conflicts(plan) == 0);
    // Each phase runs red -> red-amber -> green -> amber -> red, in that order, with the data durations.
    for (std::uint32_t phase = 0; phase < 2; ++phase) {
        std::vector<SignalAspect> seq;
        for (std::int64_t t = 0; t < plan.cycle_ms; t += 10) {
            const auto s = plan.state(phase, (t - plan.offset_ms) / 1000.0 + 0.0005);
            if (seq.empty() || seq.back() != s.aspect) seq.push_back(s.aspect);
        }
        // Rotate so the sequence starts at the first red-amber (the cycle may begin inside any aspect).
        std::vector<SignalAspect> cyc = seq;
        while (cyc.size() > 1 && cyc.front() == cyc.back()) cyc.pop_back();
        CHECK(cyc.size() == 4);
        const auto at = std::find(cyc.begin(), cyc.end(), SignalAspect::RedAmber);
        REQUIRE(at != cyc.end());
        std::rotate(cyc.begin(), at, cyc.end());
        CHECK(cyc == std::vector<SignalAspect>{SignalAspect::RedAmber, SignalAspect::Green, SignalAspect::Amber, SignalAspect::Red});
    }
    // The cycle is the sum of the phase lengths, within the maximum, and the clock is periodic.
    CHECK(plan.cycle_ms <= builtin_signal_timing().max_cycle_ms);
    for (const double t : {0.0, 7.3, 31.9, 1234.5, -50.0})
        for (std::uint32_t phase = 0; phase < 2; ++phase) {
            const auto a = plan.state(phase, t), b = plan.state(phase, t + plan.cycle_ms / 1000.0);
            CHECK(a.aspect == b.aspect); CHECK(a.seconds_to_change == Approx(b.seconds_to_change).margin(1e-9));
        }
}

TEST_CASE("S2b: negative control, a timing that puts every approach on one axis makes the conflict check fail", "[layer][roads][signals][S2]") {
    const auto scene = x_scene();
    // axis_cross_max_permille 1000 = every pair of approaches is on one axis: one phase for crossing traffic.
    const auto loose = load_signal_timing(bytes_of(replaced(builtin_text(), "\"axis_cross_max_permille\": 500", "\"axis_cross_max_permille\": 1000")));
    REQUIRE(loose);
    const auto plans = plans_of(*scene, *loose);
    REQUIRE(plans.size() == 1);
    // One real phase plus the virtual cross phase: the crossing approaches are green together, which the plan clock cannot tell,
    // but a geometric check can: two approaches with crossing directions share a phase.
    std::size_t crossing_pairs_in_one_phase = 0;
    for (const auto& a : plans[0].approaches)
        for (const auto& b : plans[0].approaches)
            if (a.phase == b.phase && std::abs(a.dir_x * b.dir_y - a.dir_y * b.dir_x) > 0.9) ++crossing_pairs_in_one_phase;
    CHECK(crossing_pairs_in_one_phase > 0);
    const auto good = plans_of(*scene);
    std::size_t good_pairs = 0;
    for (const auto& a : good[0].approaches)
        for (const auto& b : good[0].approaches)
            if (a.phase == b.phase && std::abs(a.dir_x * b.dir_y - a.dir_y * b.dir_x) > 0.9) ++good_pairs;
    CHECK(good_pairs == 0);
}

TEST_CASE("S3: a signal on a straight road has one axis, so a virtual cross phase stops traffic once per cycle", "[layer][roads][signals][S3]") {
    const auto scene = line_scene();
    SignalPlanStats stats;
    const auto plans = plans_of(*scene, builtin_signal_timing(), &stats);
    REQUIRE(plans.size() == 1);
    const auto& plan = plans[0];
    REQUIRE(plan.approaches.size() == 2);
    CHECK(stats.single_axis_plans == 1);
    REQUIRE(plan.phases.size() == 2);
    CHECK(plan.approaches[0].phase == 0); CHECK(plan.approaches[1].phase == 0);
    CHECK(plan.approaches[0].dir_x * plan.approaches[1].dir_x + plan.approaches[0].dir_y * plan.approaches[1].dir_y == Approx(-1.0).margin(1e-6));
    // Both directions share the lamps' state; red for at least the data red time per cycle, green for the rest of phase 0.
    std::int64_t red = 0, green = 0;
    for (std::int64_t t = 0; t < plan.cycle_ms; t += 10) {
        const auto s = plan.state_of(plan.approaches[0], t / 1000.0);
        if (s.aspect == SignalAspect::Red) ++red; else if (s.aspect == SignalAspect::Green) ++green;
    }
    CHECK(red * 10 >= builtin_signal_timing().single_axis_red_ms);
    CHECK(green > 0);
}

TEST_CASE("S4: plans are a pure function of the furniture and the timing, offsets differ per node", "[layer][roads][signals][S4]") {
    const auto a = plans_of(*x_scene()), b = plans_of(*x_scene());
    CHECK(a == b);
    // The cycle offset is the node-id hash modulo the cycle: two plans with different nodes are not in step.
    std::vector<N> nodes = {{7, 0, 0, {{"highway", "traffic_signals"}}}, {2, -300, 0}, {3, 300, 0}, {4, 0, -300}, {5, 0, 300}};
    const auto other = make_scene(nodes, {{10, {2, 7, 3}, {{"highway", "primary"}}}, {11, {4, 7, 5}}});
    const auto c = plans_of(*other);
    REQUIRE(c.size() == 1);
    CHECK(c[0].node_id == 7);
    CHECK(c[0].offset_ms != a[0].offset_ms);
    CHECK(a[0].offset_ms >= 0); CHECK(a[0].offset_ms < a[0].cycle_ms);
}

TEST_CASE("S5: the index finds the stop line ahead of an agent and only that one", "[layer][roads][signals][S5]") {
    const auto scene = x_scene();
    SignalIndex index;
    CHECK(index.add(plans_of(*scene)) == 1);
    CHECK(index.add(plans_of(*scene)) == 0);   // a plan of a node already present is skipped
    CHECK(index.plan_count() == 1);
    REQUIRE(index.plan_of_node(1) != nullptr);
    const auto& plan = *index.plan_of_node(1);
    for (const auto& ap : plan.approaches) {
        // 25 m behind the line on the line's own centre, heading along the travel direction.
        const double x = ap.x - 25.0 * ap.dir_x, y = ap.y - 25.0 * ap.dir_y;
        const auto hit = index.next_ahead(x, y, ap.dir_x, ap.dir_y, 60.0, 100.0);
        REQUIRE(hit);
        CHECK(hit->approach->id() == ap.id());
        CHECK(hit->distance_m == Approx(25.0).margin(1e-6));
        CHECK(std::abs(hit->lateral_m) < 1e-6);
        const auto s = plan.state_of(ap, 100.0);
        CHECK(hit->aspect == s.aspect);
        CHECK(hit->seconds_to_change == Approx(s.seconds_to_change).margin(1e-9));
        // Out of range, too far to the side, wrong way, already past the line.
        CHECK_FALSE(index.next_ahead(x, y, ap.dir_x, ap.dir_y, 20.0, 100.0));
        CHECK_FALSE(index.next_ahead(x - 12.0 * ap.dir_y, y + 12.0 * ap.dir_x, ap.dir_x, ap.dir_y, 60.0, 100.0));
        const auto back = index.next_ahead(x, y, -ap.dir_x, -ap.dir_y, 60.0, 100.0);
        if (back) CHECK(back->approach->id() != ap.id());
        CHECK_FALSE(index.next_ahead(ap.x + 5.0 * ap.dir_x, ap.y + 5.0 * ap.dir_y, ap.dir_x, ap.dir_y, 60.0, 100.0));
    }
    std::vector<std::uint64_t> ids;
    index.for_each_near(kOriginE, kOriginN, 80.0, [&](const SignalPlan&, const SignalApproach& a) { ids.push_back(a.id()); });
    CHECK(ids.size() == 4);
    CHECK(std::is_sorted(ids.begin(), ids.end()));
    std::size_t far = 0;
    index.for_each_near(kOriginE + 5000.0, kOriginN, 80.0, [&](const SignalPlan&, const SignalApproach&) { ++far; });
    CHECK(far == 0);
}

TEST_CASE("S6: the stop-or-go rule", "[layer][roads][signals][S6]") {
    const auto scene = x_scene();
    const auto plans = plans_of(*scene);
    SignalHit hit;
    hit.plan = &plans[0]; hit.approach = &plans[0].approaches[0];
    const SignalDriverRules rules;   // 3 m/s^2 comfortable, 0.5 s reaction
    const double v = 14.0;           // 50 km/h
    const double stopping = v * 0.5 + v * v / 6.0;   // 39.7 m
    hit.aspect = SignalAspect::Green; hit.distance_m = 5.0;
    CHECK(signal_advice(hit, v, rules) == SignalAdvice::Go);
    hit.aspect = SignalAspect::Red; hit.distance_m = 100.0;
    CHECK(signal_advice(hit, v, rules) == SignalAdvice::Stop);
    hit.aspect = SignalAspect::RedAmber;
    CHECK(signal_advice(hit, v, rules) == SignalAdvice::Stop);
    hit.aspect = SignalAspect::Amber;
    hit.distance_m = stopping + 1.0; CHECK(signal_advice(hit, v, rules) == SignalAdvice::Stop);
    hit.distance_m = stopping - 1.0; CHECK(signal_advice(hit, v, rules) == SignalAdvice::Go);
    hit.distance_m = 5.0; CHECK(signal_advice(hit, 0.0, rules) == SignalAdvice::Stop);   // a stopped agent stays
    hit.aspect = SignalAspect::Red; hit.distance_m = -1.0;
    CHECK(signal_advice(hit, v, rules) == SignalAdvice::Go);                              // already over the line
    CHECK(signal_stop_speed_cap(0.0, rules) == 0.0);
    CHECK(signal_stop_speed_cap(-3.0, rules) == 0.0);
    CHECK(signal_stop_speed_cap(25.0, rules) == Approx(std::sqrt(2.0 * 3.0 * 25.0)));
    // A heavy vehicle (tractor with a trailer: 1.5 m/s^2) needs a longer distance to be able to stop on amber.
    SignalDriverRules heavy = rules; heavy.comfortable_decel_mps2 = 1.5;
    hit.aspect = SignalAspect::Amber; hit.distance_m = stopping + 1.0;
    CHECK(signal_advice(hit, v, heavy) == SignalAdvice::Go);
}

// ---- real OSM fixtures ----

namespace {
struct FixturePlans {
    std::map<std::int64_t, SignalPlan> plans;     // first tile that produced the node
    std::size_t repeated = 0, mismatches = 0, conflicts = 0, far_heads = 0, bad_dirs = 0, no_heads = 0, signal_nodes_world = 0, single_axis = 0;
    std::size_t approaches = 0, heads = 0, max_phases = 0, unmatched = 0;
};
FixturePlans run_plans(const std::string& pbf) {
    const auto data = g2m_test::read_osm_fixture(std::string(G2M_SOURCE_DIR) + "/testdata/fixtures/osm/" + pbf);
    const auto world = build_road_graph(data, g2m_test::kZone32N); REQUIRE(world);
    const auto builder = RoadReferenceBuilder::make(*world); REQUIRE(builder);
    FixturePlans out;
    for (const auto& n : world->nodes) {   // signal nodes: highway=traffic_signals or a crossing with crossing=traffic_signals
        bool signal = false, crossing = false, signalised_crossing = false;
        for (const auto& [k, v] : n.tags) {
            if (k == "highway" && v == "traffic_signals") signal = true;
            if (k == "highway" && v == "crossing") crossing = true;
            if (k == "crossing" && v == "traffic_signals") signalised_crossing = true;
        }
        if (signal || (crossing && signalised_crossing)) ++out.signal_nodes_world;
    }
    std::set<TileKey> keys;
    for (const auto& n : world->nodes) {
        const auto k = tile_key_at(g2m_test::kZone32N, 2, static_cast<double>(n.position.x) / 1000.0, static_cast<double>(n.position.y) / 1000.0);
        if (k) keys.insert(*k);
    }
    for (const auto& key : keys) {
        const auto probe = select_road_graph_tile(*world, key);
        if (!probe || probe->owned_edges.empty()) continue;
        auto g = g2m_test::geometry_in_tile(*world, key); REQUIRE(g);
        auto js = build_road_junctions(*g, *builder); REQUIRE(js);
        auto fur = build_road_furniture(*g, *builder, *js); REQUIRE(fur);
        g->furniture = std::move(*fur);
        SignalPlanStats stats;
        auto plans = build_signal_plans(*g, builtin_signal_timing(), &stats); REQUIRE(plans);
        out.unmatched += stats.unmatched_signals;
        for (auto& p : *plans) {
            for (const auto& a : p.approaches) {
                ++out.approaches; out.heads += a.heads.size();
                if (a.heads.empty()) ++out.no_heads;
                if (std::abs(std::hypot(a.dir_x, a.dir_y) - 1.0) > 1e-9) ++out.bad_dirs;
                for (const auto& h : a.heads) if (std::hypot(h.x - a.x, h.y - a.y) > 12.0) ++out.far_heads;
            }
            if (conflicts(p, 1) != 0) ++out.conflicts;
            if (p.phases.size() == 2 && p.approaches.size() && std::all_of(p.approaches.begin(), p.approaches.end(), [](const auto& a) { return a.phase == 0; })) ++out.single_axis;
            out.max_phases = std::max(out.max_phases, p.phases.size());
            const auto it = out.plans.find(p.node_id);
            if (it == out.plans.end()) out.plans.emplace(p.node_id, std::move(p));
            else { ++out.repeated; if (!(it->second == p)) ++out.mismatches; }
        }
    }
    return out;
}
}

TEST_CASE("S7 fixtures: plans of real signalised nodes are conflict-free, well formed and identical across tiles", "[layer][roads][signals][fixtures][S7]") {
    for (const char* pbf : {"frankfurt_hoechst.osm.pbf", "hoechst_a66.osm.pbf"}) {
        const auto r = run_plans(pbf);
        std::printf("[signals] %s: signal nodes %zu, plans %zu, approaches %zu, heads %zu, single-axis plans %zu, max phases %zu, repeated plans %zu (differ %zu), unmatched signals %zu\n",
            pbf, r.signal_nodes_world, r.plans.size(), r.approaches, r.heads, r.single_axis, r.max_phases, r.repeated, r.mismatches, r.unmatched);
        CHECK(r.plans.size() > 0);
        CHECK(r.plans.size() <= r.signal_nodes_world);
        CHECK(r.conflicts == 0); CHECK(r.bad_dirs == 0); CHECK(r.no_heads == 0); CHECK(r.far_heads == 0);
        CHECK(r.mismatches == 0);
        CHECK(r.unmatched == 0);
    }
}
