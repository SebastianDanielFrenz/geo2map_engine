// Street fidelity P2 slice 6: the profile completion tier (design 6.6, tests T6 completion variant, T18 and the tile
// invariance argument of decision D25). Synthetic worlds for the three paths (retry, bridge, ground) compared with a
// frozen copy of racing_game's own code (tests/support/road_completion_reference.h), the tier on the two committed
// OSM fixtures, and the tile-invariance tests with their negative controls.
#include "road_fixture.h"
#include "road_completion_reference.h"
#include <catch2/catch_test_macros.hpp>
#include <algorithm>
#include <bit>
#include <cmath>
#include <cstdio>
#include <functional>
#include <limits>
#include <map>
#include <memory>
#include <set>
#include <string>
using namespace g2m;
namespace {
const geo::UtmZone zone{32, geo::Hemisphere::North};
constexpr double kOriginE = 455.0 * 1024.0 + 512.0, kOriginN = 5421.0 * 1024.0 + 512.0;   // centre of tile 455/5421
const TileKey kTile{zone, 2, 455, 5421};
const TileKey kTileEast{zone, 2, 456, 5421};
using Tags = std::vector<std::pair<std::string, std::string>>;
using Fn = std::function<double(double, double)>;
struct N { std::int64_t id; double x, y; };
struct W { std::int64_t id; std::vector<std::int64_t> nodes; Tags tags = {{"highway", "tertiary"}}; };
const Tags kBridge = {{"highway", "primary"}, {"bridge", "yes"}};

RoadGraph world_of(const std::vector<N>& nodes, std::vector<W> ways) {
    RoadGraph g;
    for (const auto& n : nodes) g.nodes.push_back({n.id, {std::llround((kOriginE + n.x) * 1000.0), std::llround((kOriginN + n.y) * 1000.0)}, {}});
    std::sort(g.nodes.begin(), g.nodes.end(), [](const auto& a, const auto& b) { return a.osm_id < b.osm_id; });
    std::sort(ways.begin(), ways.end(), [](const auto& a, const auto& b) { return a.id < b.id; });
    for (auto& w : ways) {
        Tags tags = w.tags; std::sort(tags.begin(), tags.end());
        g.ways.push_back({w.id, 3, w.nodes, tags});
        for (std::size_t i = 1; i < w.nodes.size(); ++i) g.edges.push_back({w.id, static_cast<std::uint32_t>(i - 1), w.nodes[i - 1], w.nodes[i]});
    }
    return g;
}
// A gently rolling terrain: every approach has its own grade, but the curvature stays far below any class limit.
double rolling(double e, double n) {
    const double x = e - kOriginE, y = n - kOriginN;
    return 100.0 + 0.03 * x + 0.02 * y + 3.0 * std::sin(x / 160.0) * std::cos(y / 220.0);
}

// The inputs of one roads.geom tile, kept alive for direct calls into the tier.
struct Env {
    RoadGraphTile tile;
    RoadGraph context;
    std::vector<HeightTile> heights;
    std::optional<RoadDemSampler> dem;
    RoadProfileLimits limits;
};
std::unique_ptr<Env> make_env(const RoadGraph& world, const TileKey& key, const Fn& height) {
    auto env = std::make_unique<Env>();
    auto tile = select_road_graph_tile(world, key); REQUIRE(tile); env->tile = std::move(*tile);
    const auto deps = road_dem_dependencies(env->tile.graph, key.zone); REQUIRE(deps);
    env->heights.resize(deps->size());
    std::vector<const HeightTile*> views;
    for (std::size_t i = 0; i < env->heights.size(); ++i) { env->heights[i].key = (*deps)[i].key; g2m_test::fill_dem(env->heights[i], height); views.push_back(&env->heights[i]); }
    auto dem = RoadDemSampler::make(views); REQUIRE(dem); env->dem = std::move(*dem);
    const auto cdeps = road_context_dependencies(env->tile.graph, key.zone, &key); REQUIRE(cdeps);
    std::vector<RoadGraphTile> others;
    for (const auto& d : *cdeps) { auto t = select_road_graph_tile(world, d.key); REQUIRE(t); others.push_back(std::move(*t)); }
    std::vector<const RoadGraph*> graphs{&env->tile.graph};
    for (const auto& t : others) graphs.push_back(&t.graph);
    auto context = merge_road_graphs(graphs); REQUIRE(context); env->context = std::move(*context);
    env->limits.reference.allow_curvature_exceptions = true;
    return env;
}
RoadGeomTile build(const Env& env, bool completion, RoadCompletionStats* stats = nullptr) {
    RoadCompletionOptions o; o.enabled = completion;
    auto g = build_road_geom_tile(env.tile, *env.dem, env.limits, &env.context, builtin_road_rules_de(), o, stats); REQUIRE(g);
    return std::move(*g);
}
std::size_t index_of(const RoadGeomTile& t, std::int64_t way, std::uint32_t start) {
    for (std::size_t i = 0; i < t.entries.size(); ++i) if (t.entries[i].way_id == way && t.entries[i].stretch.start_ref == start) return i;
    FAIL("no such entry"); return 0;
}
std::size_t last_of(const RoadGeomTile& t, std::int64_t way) {
    std::size_t best = t.entries.size();
    for (std::size_t i = 0; i < t.entries.size(); ++i)
        if (t.entries[i].way_id == way && (best == t.entries.size() || t.entries[i].stretch.start_ref > t.entries[best].stretch.start_ref)) best = i;
    REQUIRE(best < t.entries.size());
    return best;
}
bool same_vertical(const VerticalProfile& a, const VerticalProfile& b) {
    if (a.segments.size() != b.segments.size()) return false;
    for (std::size_t i = 0; i < a.segments.size(); ++i) {
        const auto& x = a.segments[i]; const auto& y = b.segments[i];
        if (x.station_m != y.station_m || x.length_m != y.length_m || x.height0_m != y.height0_m || x.height1_m != y.height1_m ||
            x.grade0 != y.grade0 || x.grade1 != y.grade1) return false;
    }
    return true;
}
bool same_outcome(const RoadGeomEntry& a, const RoadProfileOrigin& oa, const RoadGeomEntry& b, const RoadProfileOrigin& ob) {
    return oa == ob && a.profile.has_value() == b.profile.has_value() && (!a.profile || same_vertical(a.profile->vertical, b.profile->vertical));
}
struct Delta { double height = 0, grade = 0; };
Delta max_delta(const RoadProfile& a, const RoadProfile& b) {
    Delta d; REQUIRE(std::fabs(a.reference.length_m - b.reference.length_m) < 1e-9);
    for (double s = 0; s <= a.reference.length_m; s += 0.25) {
        const auto x = a.at(s); const auto y = b.at(s); REQUIRE(x); REQUIRE(y);
        d.height = std::max(d.height, std::fabs(x->height_m - y->height_m)); d.grade = std::max(d.grade, std::fabs(x->grade - y->grade));
    }
    return d;
}
double end_height(const RoadGeomTile& t, std::size_t i, bool at_end) { const auto& p = *t.entries[i].profile; return p.at(at_end ? p.reference.length_m : 0)->height_m; }
const char* name_of(ProfileProvenance p) {
    return p == ProfileProvenance::Strict ? "strict" : p == ProfileProvenance::Retry ? "retry" : p == ProfileProvenance::BridgeCompleted ? "bridge-completed" : "ground-completed";
}

// A bridge whose two ends each have a junction 15 m beyond them: the strict anchor walk reaches the junction before
// 32 m of support and declines, so the span and the two short ground stretches next to it are completion cases.
RoadGraph bridge_world() {
    return world_of({{1, -300, 0}, {2, -250, 0}, {3, -115, 0}, {4, -100, 0}, {5, 0, 0}, {6, -115, 80}, {7, 15, 0}, {8, 15, 80}, {9, 150, 0}, {10, 300, 0}},
        {{10, {1, 2, 3, 4}}, {11, {3, 6}}, {20, {4, 5}, kBridge}, {12, {5, 7, 9, 10}}, {13, {7, 8}}});
}
}

TEST_CASE("completion: off by default leaves a tile byte for byte what the strict pass stores; on without work changes nothing", "[layer][roads][completion]") {
    const auto world = bridge_world();
    const auto env = make_env(world, kTile, rolling);
    const auto plain = build(*env, false);
    REQUIRE(plain.origins.size() == plain.entries.size());   // slice 8: always filled, all Strict without the tier
    for (const auto& o : plain.origins) CHECK(o == RoadProfileOrigin{});
    // A tile that holds nothing to complete: enabled and disabled encode to the same bytes.
    const auto calm = world_of({{1, -200, 0}, {2, 0, 0}, {3, 200, 0}}, {{10, {1, 2, 3}}});
    const auto calm_env = make_env(calm, kTile, rolling);
    const auto off = build(*calm_env, false); RoadCompletionStats stats; const auto on = build(*calm_env, true, &stats);
    REQUIRE(on.origins.size() == on.entries.size());
    for (const auto& o : on.origins) CHECK(o == RoadProfileOrigin{});
    const auto a = encode_road_geom(off), b = encode_road_geom(on); REQUIRE(a); REQUIRE(b);
    CHECK(*a == *b);
    CHECK(stats.retry_candidates + stats.bridge_candidates + stats.ground_candidates == 0);
}

TEST_CASE("completion: the three paths on a bridge with junctions next to its ends, against the game's own code", "[layer][roads][completion][T18]") {
    const auto world = bridge_world();
    const auto env = make_env(world, kTile, rolling);
    const auto strict = build(*env, false);
    RoadCompletionStats stats; const auto lib = build(*env, true, &stats);
    const auto span = index_of(strict, 20, 0), west = index_of(strict, 10, 2), east = index_of(strict, 12, 0);
    for (const auto i : {span, west, east}) CHECK_FALSE(strict.entries[i].profile);
    CHECK(lib.origins[span].provenance == ProfileProvenance::BridgeCompleted);
    CHECK(lib.origins[west].provenance == ProfileProvenance::GroundCompleted);
    CHECK(lib.origins[east].provenance == ProfileProvenance::GroundCompleted);
    for (const auto i : {span, west, east}) { REQUIRE(lib.entries[i].profile); CHECK(lib.origins[i].strict_decline_code == static_cast<std::uint16_t>(strict.entries[i].decline->code)); }
    // Everything else keeps its strict profile and provenance.
    for (std::size_t i = 0; i < lib.entries.size(); ++i) if (i != span && i != west && i != east) {
        CHECK(lib.origins[i] == RoadProfileOrigin{});
        REQUIRE(lib.entries[i].profile.has_value() == strict.entries[i].profile.has_value());
        if (lib.entries[i].profile) CHECK(same_vertical(lib.entries[i].profile->vertical, strict.entries[i].profile->vertical));
    }
    // The game's code on the same strict tile (its builder is the tile graph, which is the whole world here).
    auto game = strict; g2m_test::game_reference::complete_road_profiles(game, *env->dem);
    double worst_h = 0, worst_g = 0;
    for (const auto i : {span, west, east}) {
        REQUIRE(game.entries[i].profile);
        const auto d = max_delta(*lib.entries[i].profile, *game.entries[i].profile);
        worst_h = std::max(worst_h, d.height); worst_g = std::max(worst_g, d.grade);
    }
    std::printf("T18 bridge scene: span, west and east ground stretch vs the game: max |dz| %.3e m, max |dgrade| %.3e (limit 1 mm)\n", worst_h, worst_g);
    CHECK(worst_h <= 1e-3); CHECK(worst_g <= 1e-3);
    // Flags: the span's anchors are inferred, the ground stretches' bridge-side end too.
    CHECK(lib.entries[span].profile->start_anchor_inferred); CHECK(lib.entries[span].profile->end_anchor_inferred);
    CHECK_FALSE(lib.entries[west].profile->start_anchor_inferred); CHECK(lib.entries[west].profile->end_anchor_inferred);
    CHECK(lib.entries[east].profile->start_anchor_inferred); CHECK_FALSE(lib.entries[east].profile->end_anchor_inferred);
    CHECK(stats.bridge_completed == 1); CHECK(stats.ground_completed == 2); CHECK(stats.ground_bridge_anchored == 2);
    // T6 completion variant: every stretch that ends at a bridge node reads the same height there (1/256 m).
    CHECK(std::fabs(end_height(lib, west, true) - end_height(lib, span, false)) <= 1.0 / 256.0);
    CHECK(std::fabs(end_height(lib, span, true) - end_height(lib, east, false)) <= 1.0 / 256.0);
    // Encoding 7 stores the provenance (section 7) and the decode returns it.
    const auto bytes = encode_road_geom(lib); REQUIRE(bytes); CHECK(bytes->encoding == 7);
    const auto back = decode_road_geom(*bytes); REQUIRE(back); CHECK(back->origins == lib.origins);
    // Encoding 6 cannot say "completed": it refuses the tile rather than store it as strict.
    auto six = lib; six.encoding = kRoadGeomEncodingPrevious; six.junctions.clear(); six.furniture.clear();
    const auto refused = encode_road_geom(six); REQUIRE_FALSE(refused);
    CHECK(refused.error().message.find("cannot carry a completed profile") != std::string::npos);
}

TEST_CASE("completion: T6 check detects one stretch left at another height; the validator knows the origins", "[layer][roads][completion][T6]") {
    const auto world = bridge_world();
    const auto env = make_env(world, kTile, rolling);
    auto lib = build(*env, true);
    const auto span = index_of(lib, 20, 0), west = index_of(lib, 10, 2);
    const double gap = std::fabs(end_height(lib, west, true) - end_height(lib, span, false));
    CHECK(gap <= 1.0 / 256.0);
    // Sabotage: only the bridge was refitted, the ground stretch keeps a height 2 cm off at the node.
    auto lifted = lib; lifted.entries[west].profile->vertical.segments.back().height1_m += 0.02;
    const double lifted_gap = std::fabs(end_height(lifted, west, true) - end_height(lifted, span, false));
    std::printf("T6 completion sabotage (one stretch 2 cm off): node gap %.4f m vs the 1/256 m = %.4f m check\n", lifted_gap, 1.0 / 256.0);
    CHECK(lifted_gap > 1.0 / 256.0);
    // Validation: origins are checked.
    REQUIRE(validate_road_geom_tile(lib));
    auto bad = lib; bad.origins.pop_back(); CHECK_FALSE(validate_road_geom_tile(bad));
    bad = lib; bad.origins[0].strict_decline_code = 4; CHECK_FALSE(validate_road_geom_tile(bad));                      // a Strict entry names no decline
    bad = lib; bad.origins[span].strict_decline_code = 0; CHECK_FALSE(validate_road_geom_tile(bad));                    // a completed one must
    bad = lib; bad.origins[span].provenance = static_cast<ProfileProvenance>(4); CHECK_FALSE(validate_road_geom_tile(bad));
    bad = lib; bad.entries[span].profile.reset(); CHECK_FALSE(validate_road_geom_tile(bad));                           // a completed origin needs its profile
}

TEST_CASE("completion: a motorway steeper than its class is a flagged ground completion with end grades clamped to 10 %", "[layer][roads][completion][T18]") {
    // 11 % over 300 m of motorway (class 6 %): the strict endpoint-slope check declines; the ground fit allows 12 %
    // and clamps the end grades to 10 %.
    std::vector<N> nodes; std::vector<std::int64_t> ids;
    for (std::int64_t i = 0; i <= 3; ++i) { nodes.push_back({i + 1, -150.0 + 100.0 * static_cast<double>(i), 0}); ids.push_back(i + 1); }
    const auto world = world_of(nodes, {{10, ids, {{"highway", "motorway"}}}});
    const Fn steep = [](double e, double) { return 100.0 + 0.11 * (e - kOriginE); };
    const auto env = make_env(world, kTile, steep);
    const auto strict = build(*env, false);
    REQUIRE(strict.entries.size() == 1); REQUIRE_FALSE(strict.entries[0].profile);
    std::printf("T18 clamp scene strict decline: %s\n", strict.entries[0].decline->message.c_str());
    const auto lib = build(*env, true);
    REQUIRE(lib.entries[0].profile);
    CHECK(lib.origins[0].provenance == ProfileProvenance::GroundCompleted);
    const auto& first = lib.entries[0].profile->vertical.segments.front(); const auto& last = lib.entries[0].profile->vertical.segments.back();
    std::printf("T18 clamp scene: start grade %.6f, end grade %.6f (plane slope 0.11)\n", first.grade0, last.grade1);
    CHECK(first.grade0 == 0.10); CHECK(last.grade1 == 0.10);
    auto game = strict; g2m_test::game_reference::complete_road_profiles(game, *env->dem);
    REQUIRE(game.entries[0].profile);
    const auto d = max_delta(*lib.entries[0].profile, *game.entries[0].profile);
    std::printf("T18 clamp scene vs the game: max |dz| %.3e m\n", d.height);
    CHECK(d.height <= 1e-3);
    // Sabotage "no 10 % clamp": the same samples fitted with the plane's own 0.11 end grades give another profile.
    const auto& ref = lib.entries[0].profile->reference;
    std::vector<VerticalSample> samples; const auto count = static_cast<std::size_t>(std::ceil(ref.length_m / 40));
    for (std::size_t i = 0; i <= count; ++i) {
        const double s = ref.length_m * static_cast<double>(i) / static_cast<double>(count);
        samples.push_back({s, lib.entries[0].profile->at(s)->height_m});
    }
    VerticalFitLimits fit; fit.max_grade = 0.12; fit.min_radius_m = 500; fit.max_iterations = 16384; fit.start_grade = fit.end_grade = 0.11;
    const auto unclamped = fit_vertical_profile(samples, fit); REQUIRE(unclamped);
    CHECK(unclamped->segments.front().grade0 != first.grade0);
    // The validator accepts the exception under its provenance and refuses it as a strict profile.
    REQUIRE(validate_road_geom_tile(lib));
    auto strict_claim = lib; strict_claim.origins[0] = RoadProfileOrigin{};
    CHECK_FALSE(validate_road_geom_tile(strict_claim));
}

TEST_CASE("completion: the retry and the ground path on noisy long stretches, against the game's own code", "[layer][roads][completion][T18]") {
    const auto road = [](double length) {
        std::vector<N> nodes; std::vector<std::int64_t> ids; std::int64_t id = 1;
        for (double x = -length / 2; x <= length / 2 + 1e-9; x += 100) { nodes.push_back({id, x, 0}); ids.push_back(id++); }
        return world_of(nodes, {{10, ids}});
    };
    const auto noise = [](double amp, double scale) { return Fn([=](double e, double) { return 100.0 + 0.02 * (e - kOriginE) + amp * std::sin((e - kOriginE) / scale); }); };
    {   // Retry: 1.5 km, 5 m samples exhaust the solver, 20 m samples at the same limits do not.
        const auto env = make_env(road(1500), kTile, noise(0.8, 2.2));
        const auto strict = build(*env, false); REQUIRE_FALSE(strict.entries[0].profile);
        CHECK(strict.entries[0].decline->message.find("bounded solver") != std::string::npos);
        RoadCompletionStats stats; const auto lib = build(*env, true, &stats);
        REQUIRE(lib.entries[0].profile); CHECK(lib.origins[0].provenance == ProfileProvenance::Retry);
        CHECK(stats.retry_accepted == 1);
        auto game = strict; g2m_test::game_reference::retry_declines(game, *env->dem);
        REQUIRE(game.entries[0].profile);
        const auto d = max_delta(*lib.entries[0].profile, *game.entries[0].profile);
        std::printf("T18 retry path vs the game: max |dz| %.3e m, max |dgrade| %.3e\n", d.height, d.grade);
        CHECK(d.height <= 1e-3); CHECK(same_vertical(lib.entries[0].profile->vertical, game.entries[0].profile->vertical));
    }
    {   // Ground: 3 km, the retry fails too, the 40 m ground fit accepts.
        const auto env = make_env(road(3000), kTile, noise(0.8, 2.2));
        const auto strict = build(*env, false); REQUIRE_FALSE(strict.entries[0].profile);
        RoadCompletionStats stats; const auto lib = build(*env, true, &stats);
        REQUIRE(lib.entries[0].profile); CHECK(lib.origins[0].provenance == ProfileProvenance::GroundCompleted);
        CHECK(stats.retry_accepted == 0);   // the 3 km decline is not a solver-budget one: it goes straight to the ground fit
        auto game = strict; g2m_test::game_reference::complete_road_profiles(game, *env->dem);
        REQUIRE(game.entries[0].profile);
        const auto d = max_delta(*lib.entries[0].profile, *game.entries[0].profile);
        std::printf("T18 ground path vs the game: max |dz| %.3e m, max |dgrade| %.3e\n", d.height, d.grade);
        CHECK(d.height <= 1e-3);
    }
}

TEST_CASE("completion: bridge anchors are a function of the node, so a split bridge is order independent (the game's anchor map is not)", "[layer][roads][completion][order]") {
    // Two bridge ways meet at node 5 (10 m each), ground roads with a junction 15 m beyond the outer ends.
    const auto world = world_of({{1, -300, 0}, {2, -250, 0}, {3, -135, 0}, {4, -120, 0}, {5, -110, 0}, {6, -100, 0}, {7, -85, 0}, {8, 50, 0}, {9, 200, 0}, {12, -135, 80}, {13, -85, 80}},
        {{10, {1, 2, 3, 4}}, {11, {3, 12}}, {20, {4, 5}, kBridge}, {21, {5, 6}, kBridge}, {22, {6, 7, 8, 9}}, {23, {7, 13}}});
    const auto env = make_env(world, kTile, rolling);
    const auto strict = build(*env, false);
    const auto b1 = index_of(strict, 20, 0), b2 = index_of(strict, 21, 0);
    REQUIRE_FALSE(strict.entries[b1].profile); REQUIRE_FALSE(strict.entries[b2].profile);
    const auto lib = build(*env, true);
    REQUIRE(lib.entries[b1].profile); REQUIRE(lib.entries[b2].profile);
    CHECK(end_height(lib, b1, true) == end_height(lib, b2, false));      // the node they share
    // Reversing the entries gives the same result entry by entry.
    RoadCompletionOptions opts; opts.enabled = true;
    const auto builder = RoadReferenceBuilder::make(env->context); REQUIRE(builder);
    auto entries = strict.entries; std::vector<RoadProfileOrigin> origins;
    std::reverse(entries.begin(), entries.end());
    REQUIRE(complete_road_profiles(entries, origins, *builder, *env->dem, zone, env->limits, opts));
    std::reverse(entries.begin(), entries.end()); std::reverse(origins.begin(), origins.end());
    for (std::size_t i = 0; i < entries.size(); ++i) CHECK(same_outcome(entries[i], origins[i], lib.entries[i], lib.origins[i]));
    // Negative control: the game's anchors map, filled in entry order, hands the shared node to whichever bridge comes first.
    auto forward = strict; g2m_test::game_reference::complete_road_profiles(forward, *env->dem);
    auto backward = strict; std::reverse(backward.entries.begin(), backward.entries.end()); g2m_test::game_reference::complete_road_profiles(backward, *env->dem);
    std::reverse(backward.entries.begin(), backward.entries.end());
    REQUIRE(forward.entries[b1].profile); REQUIRE(backward.entries[b1].profile);
    const double game_shift = std::fabs(end_height(forward, b1, true) - end_height(backward, b1, true));
    std::printf("order control: the game's height at the shared bridge node moves %.4f m when the entries are reversed; the tier's moves 0\n", game_shift);
    CHECK(game_shift > 1.0 / 256.0);
}

TEST_CASE("completion: a completed bridge stretch meets the strict profiles at its bridge nodes at their height (D30)", "[layer][roads][completion][T6]") {
    // A 30 m bridge (way 20: 4 -> 5 -> 6) with a side road joining at its middle node 5, between strict ground roads (way 10
    // ends at 4, way 12 starts at 6). The ground steps 0.5 m under the bridge, flat on both sides: the strict anchors at 4
    // and 6 carry the feasibility correction of the span (a 0.5 m rise between level ends is beyond the 30 m span's
    // curvature), so the strict ground profiles end about 0.15 m off the raw approach planes. Node 5 has two bridge edges:
    // both bridge stretches and the side road are declined by the strict pass and completed.
    const auto world = world_of({{1, -300, 0}, {2, -200, 0}, {3, -100, 0}, {4, 0, 0}, {5, 15, 0}, {6, 30, 0}, {7, 15, 80}, {8, 130, 0}, {9, 330, 0}},
        {{10, {1, 2, 3, 4}}, {20, {4, 5, 6}, kBridge}, {11, {5, 7}}, {12, {6, 8, 9}}});
    const Fn step = [](double e, double) { const double x = e - kOriginE; return 100.0 + 0.5 * std::clamp((x - 2.0) / 26.0, 0.0, 1.0); };
    const auto env = make_env(world, kTile, step);
    const auto strict = build(*env, false);
    const auto west = last_of(strict, 10), east = index_of(strict, 12, 0), b1 = index_of(strict, 20, 0), b2 = last_of(strict, 20);
    REQUIRE(b1 != b2);
    REQUIRE(strict.entries[west].profile); REQUIRE(strict.entries[east].profile);
    REQUIRE_FALSE(strict.entries[b1].profile); REQUIRE_FALSE(strict.entries[b2].profile);
    REQUIRE(strict.entries[west].profile->end_anchor_adjusted); REQUIRE(strict.entries[east].profile->start_anchor_adjusted);
    RoadCompletionStats stats; const auto lib = build(*env, true, &stats);
    REQUIRE(lib.entries[b1].profile); REQUIRE(lib.entries[b2].profile);
    CHECK(lib.origins[b1].provenance == ProfileProvenance::BridgeCompleted); CHECK(lib.origins[b2].provenance == ProfileProvenance::BridgeCompleted);
    // The strict profiles are untouched (D26).
    CHECK(lib.origins[west] == RoadProfileOrigin{}); CHECK(same_vertical(lib.entries[west].profile->vertical, strict.entries[west].profile->vertical));
    const double gap_west = std::fabs(end_height(lib, west, true) - end_height(lib, b1, false));
    const double gap_east = std::fabs(end_height(lib, b2, true) - end_height(lib, east, false));
    // The walk's abutment plane at node 4 is the uncorrected approach plane: what slice 6 used.
    const auto walk = road_bridge_anchor(env->context, *env->dem, zone, 4); REQUIRE(walk.plane);
    const double walk_gap = std::fabs(end_height(lib, west, true) - walk.plane->height / 256.0);
    std::printf("D30 gap scene: strict ground vs completed bridge at node 4 %.4f m, at node 6 %.4f m (slice 6's walk plane at node 4 sat %.4f m off); %zu strict anchors taken\n",
        gap_west, gap_east, walk_gap, stats.strict_anchors);
    CHECK(gap_west <= 1.0 / 256.0); CHECK(gap_east <= 1.0 / 256.0);
    CHECK(walk_gap > 0.1);   // the scene does exercise the correction
    CHECK(stats.strict_anchors == 2);
    // The completed stretches still agree among themselves at the node they share (T6 completion variant).
    CHECK(std::fabs(end_height(lib, b1, true) - end_height(lib, b2, false)) <= 1.0 / 256.0);
    // Direction 2: a completed GROUND stretch next to a strict bridge anchor. Way 10 forced to decline (a decline that is
    // not a retry candidate): its completion must end at the strict anchor of node 4, where a strict bridge would start.
    {
        auto entries = strict.entries; std::vector<RoadProfileOrigin> origins;
        entries[west].profile.reset(); entries[west].decline = make_error(ErrorCode::LimitExceeded, "test: forced decline");
        RoadCompletionOptions opts; opts.enabled = true;
        const auto builder = RoadReferenceBuilder::make(env->context); REQUIRE(builder);
        REQUIRE(complete_road_profiles(entries, origins, *builder, *env->dem, zone, env->limits, opts));
        REQUIRE(entries[west].profile); CHECK(origins[west].provenance == ProfileProvenance::GroundCompleted);
        const auto anchor = road_strict_anchor(*builder, *env->dem, zone, 4, env->limits.reference); REQUIRE(anchor);
        const double h = entries[west].profile->at(entries[west].profile->reference.length_m)->height_m;
        std::printf("D30 direction 2: completed ground at node 4 %.4f m, strict anchor %.4f m\n", h, anchor->plane.height / 256.0);
        CHECK(std::fabs(h - anchor->plane.height / 256.0) <= 1.0 / 256.0);
    }
}

// ---- tile invariance (D25) ----

namespace {
// Way 10 runs from the west of tile 455 into tile 456 and ends in a short stub j -> n (15 m) whose far end n carries a
// 300 m bridge that starts 130 m inside tile 456: the west tile holds way 10 (complete ways touching its halo) but not
// the bridge, the east tile holds both. A junction at j makes the strict anchor walk decline, so the stub is a
// completion case whose end anchor is the bridge's abutment plane. A ditch under the bridge makes the DEM plane AT n
// differ from the supported plane 32 m away.
RoadGraph two_tile_world() {
    std::vector<N> nodes; std::vector<std::int64_t> ids; std::int64_t id = 1;
    for (double x = -400; x <= 600; x += 100) { nodes.push_back({id, x, 0}); ids.push_back(id++); }
    const std::int64_t j = 100, n = 101, side = 102, m = 103, g1 = 104, g2 = 105;
    nodes.push_back({j, 650, 0}); nodes.push_back({n, 665, 0}); nodes.push_back({side, 650, 80});
    nodes.push_back({m, 965, 0}); nodes.push_back({g1, 1065, 0}); nodes.push_back({g2, 1250, 0});
    ids.push_back(j); ids.push_back(n);
    return world_of(nodes, {{10, ids}, {11, {j, side}}, {20, {n, m}, kBridge}, {12, {m, g1, g2}}});
}
double valley(double e, double n) {
    const double x = e - kOriginE;
    return rolling(e, n) - (x > 668.0 && x < 962.0 ? 6.0 : 0.0);
}
}

TEST_CASE("completion: tile invariance on a two-tile world, with the tile graph and the game's tier as negative controls", "[layer][roads][completion][invariance]") {
    const auto world = two_tile_world();
    const auto west = make_env(world, kTile, valley), east = make_env(world, kTileEast, valley);
    const auto holds = [](const RoadGeomTile& t, std::int64_t way) { return std::any_of(t.entries.begin(), t.entries.end(), [&](const auto& e) { return e.way_id == way; }); };
    const auto w_west = build(*west, true), w_east = build(*east, true);
    REQUIRE(holds(w_west, 10)); REQUIRE(holds(w_east, 10)); REQUIRE_FALSE(holds(w_west, 20)); REQUIRE(holds(w_east, 20));
    const auto strict_west = build(*west, false), strict_east = build(*east, false);
    std::size_t completed = 0, differing = 0, compared = 0;
    for (std::size_t i = 0; i < w_west.entries.size(); ++i) {
        const auto j = index_of(w_east, w_west.entries[i].way_id, w_west.entries[i].stretch.start_ref);
        ++compared;
        if (!same_outcome(w_west.entries[i], w_west.origins[i], w_east.entries[j], w_east.origins[j])) ++differing;
        if (w_west.origins[i].provenance != ProfileProvenance::Strict) ++completed;
        // The strict outcome of the shared entry is the same in both tiles (the premise of the tier's own claim).
        CHECK(strict_west.entries[i].profile.has_value() == strict_east.entries[j].profile.has_value());
    }
    std::printf("D25 two-tile world: %zu entries of the west tile compared with the east tile, %zu completed, %zu differ\n", compared, completed, differing);
    CHECK(completed == 1); CHECK(differing == 0);
    const auto stub = last_of(w_west, 10);
    CHECK(w_west.origins[stub].provenance == ProfileProvenance::GroundCompleted); CHECK(w_west.entries[stub].profile->end_anchor_inferred);
    // Control 1: hand the tier the TILE graph as its context in the west tile. The bridge is not in it: no abutment,
    // the end falls back to the DEM plane under the ditch and the stretch differs from what the east tile computes.
    {
        auto entries = strict_west.entries; std::vector<RoadProfileOrigin> origins;
        const auto narrow = RoadReferenceBuilder::make(west->tile.graph); REQUIRE(narrow);
        RoadCompletionOptions opts; opts.enabled = true;
        REQUIRE(complete_road_profiles(entries, origins, *narrow, *west->dem, zone, west->limits, opts));
        std::size_t differs = 0;
        for (std::size_t i = 0; i < entries.size(); ++i) if (!same_outcome(entries[i], origins[i], w_west.entries[i], w_west.origins[i])) ++differs;
        std::printf("control (tile graph instead of the context): %zu of %zu entries differ\n", differs, entries.size());
        CHECK(differs > 0);
    }
    // Control 2: the game's tier, which reads the bridges of ITS tile, completes the stub differently in the two tiles.
    {
        auto game_west = strict_west, game_east = strict_east;
        g2m_test::game_reference::complete_road_profiles(game_west, *west->dem); g2m_test::game_reference::complete_road_profiles(game_east, *east->dem);
        const auto jw = last_of(game_west, 10), je = last_of(game_east, 10);
        const bool west_done = game_west.entries[jw].profile.has_value(), east_done = game_east.entries[je].profile.has_value();
        double shift = -1;
        if (west_done && east_done) shift = max_delta(*game_west.entries[jw].profile, *game_east.entries[je].profile).height;
        std::printf("control (the game's tier): stub completed west %d east %d, max |dz| between the tiles %.4f m\n", west_done, east_done, shift);
        CHECK((west_done != east_done || shift > 1e-3));
    }
}

TEST_CASE("completion: one bridge anchor is the same from every tile; the tile graph alone has none", "[layer][roads][completion][invariance]") {
    const auto world = two_tile_world();
    const auto west = make_env(world, kTile, valley), east = make_env(world, kTileEast, valley);
    RoadCompletionOptions opts;
    const auto a_west = road_bridge_anchor(west->context, *west->dem, zone, 101, opts), a_east = road_bridge_anchor(east->context, *east->dem, zone, 101, opts);
    REQUIRE(a_west.bridging); REQUIRE(a_east.bridging); REQUIRE(a_west.plane); REQUIRE(a_east.plane);
    CHECK(a_west.plane->height == a_east.plane->height); CHECK(a_west.plane->grade_x == a_east.plane->grade_x); CHECK(a_west.plane->grade_y == a_east.plane->grade_y);
    // A node away from every bridge is not bridging.
    CHECK_FALSE(road_bridge_anchor(east->context, *east->dem, zone, 5, opts).bridging);
    // The walk budget bounds the read: 20 m of path cannot reach the support 32 m of ground away.
    opts.abutment_path_m = 20; CHECK_FALSE(road_bridge_anchor(east->context, *east->dem, zone, 101, opts).plane.has_value());
    // The tile graph of the west tile lacks the bridge: no bridging node at all there (the hazard the context removes).
    CHECK_FALSE(road_bridge_anchor(west->tile.graph, *west->dem, zone, 101, RoadCompletionOptions{}).bridging);
}

namespace {
// Every level-0 DEM tile under the world's nodes widened by 512 m: a DEM no tile can miss a cell of.
struct FullDem { std::vector<HeightTile> heights; std::optional<RoadDemSampler> dem; };
std::unique_ptr<FullDem> full_dem(const RoadGraph& world, const Fn& height) {
    auto out = std::make_unique<FullDem>();
    std::int64_t x0 = std::numeric_limits<std::int64_t>::max(), x1 = std::numeric_limits<std::int64_t>::min(), y0 = x0, y1 = x1;
    for (const auto& n : world.nodes) { x0 = std::min(x0, n.position.x); x1 = std::max(x1, n.position.x); y0 = std::min(y0, n.position.y); y1 = std::max(y1, n.position.y); }
    const auto lo = [](std::int64_t v) { return floor_div(v - 512000, 256000); };
    const auto hi = [](std::int64_t v) { return floor_div(v + 512000, 256000); };
    for (auto y = lo(y0); y <= hi(y1); ++y) for (auto x = lo(x0); x <= hi(x1); ++x) {
        HeightTile t; t.key = TileKey{zone, 0, static_cast<std::int32_t>(x), static_cast<std::int32_t>(y)};
        out->heights.push_back(std::move(t));
    }
    std::vector<const HeightTile*> views;
    for (auto& t : out->heights) { g2m_test::fill_dem(t, height); views.push_back(&t); }
    auto dem = RoadDemSampler::make(views); REQUIRE(dem); out->dem = std::move(*dem);
    return out;
}
// A bridge B (way 40) from the west tile 455 to node 5 at x = 702.7, 65.3 m west of the DEM tile edge at x = 768, outside
// the 128 m halo of tile 455. From node 5 a second bridge (way 41, `span` m) and a ground road (way 31) run east, so the
// walk from node 5 crosses `span` m of bridge and finds its support 32 m further: path span + 32. Tile 455's DEM
// dependencies end at x = 768 (65 m beyond B's last segment); tile 455 does not hold ways 41 and 31 (its context does).
RoadGraph fence_world(double span) {
    return world_of({{1, 300, 0}, {2, 400, 0}, {3, 500, 0}, {4, 600, 0}, {5, 702.7, 0}, {6, 702.7 + span, 0}, {7, 802.7 + span, 0}},
        {{30, {1, 2, 3}}, {40, {3, 4, 5}, kBridge}, {41, {5, 6}, kBridge}, {31, {6, 7}}});
}
}

TEST_CASE("completion: the walk budget bound keeps every DEM read inside the entry's DEM dependencies (D29)", "[layer][roads][completion][invariance]") {
    RoadCompletionOptions opts; opts.enabled = true;
    // The bound itself: 61.5 m accepted, anything beyond refused before any work.
    {
        const auto env = make_env(fence_world(29.4), kTile, rolling);
        const auto builder = RoadReferenceBuilder::make(env->context); REQUIRE(builder);
        auto entries = build(*env, false).entries; std::vector<RoadProfileOrigin> origins;
        opts.abutment_path_m = kRoadCompletionMaxAbutmentPathM;
        CHECK(complete_road_profiles(entries, origins, *builder, *env->dem, zone, env->limits, opts));
        opts.abutment_path_m = kRoadCompletionMaxAbutmentPathM + 0.1;
        const auto refused = complete_road_profiles(entries, origins, *builder, *env->dem, zone, env->limits, opts);
        CHECK_FALSE(refused);
        opts.abutment_path_m = 62;   // slice 6's bound
        CHECK_FALSE(complete_road_profiles(entries, origins, *builder, *env->dem, zone, env->limits, opts));
    }
    opts.abutment_path_m = kRoadCompletionMaxAbutmentPathM;
    // Paths 61.4 m (inside the bound) and 61.9 m (beyond it): the tile's own DEM dependencies and a DEM without holes give
    // the same tier result for every entry.
    for (const double span : {29.4, 29.9}) {
        const auto world = fence_world(span);
        const auto env = make_env(world, kTile, rolling); const auto full = full_dem(world, rolling);
        const auto builder = RoadReferenceBuilder::make(env->context); REQUIRE(builder);
        const auto strict = build(*env, false);
        const auto bridge = index_of(strict, 40, 0); REQUIRE_FALSE(strict.entries[bridge].profile);   // two bridges meet at node 5
        auto fenced = strict.entries, open = strict.entries; std::vector<RoadProfileOrigin> fo, oo;
        REQUIRE(complete_road_profiles(fenced, fo, *builder, *env->dem, zone, env->limits, opts));
        REQUIRE(complete_road_profiles(open, oo, *builder, *full->dem, zone, env->limits, opts));
        for (std::size_t i = 0; i < fenced.size(); ++i) CHECK(same_outcome(fenced[i], fo[i], open[i], oo[i]));
        const bool completed = oo[bridge].provenance == ProfileProvenance::BridgeCompleted;
        std::printf("D29 fence, walk path %.1f m at budget %.1f m: bridge completed %d (fenced DEM and full DEM agree)\n", span + 32, opts.abutment_path_m, completed);
        CHECK(completed == (span + 32 <= opts.abutment_path_m));
    }
    // Control: slice 6's 62 m budget reads beyond the DEM dependencies at path 61.9 m: the tile's DEM misses the cell the
    // full DEM has, so the abutment exists in one and not the other.
    {
        const auto world = fence_world(29.9);
        const auto env = make_env(world, kTile, rolling); const auto full = full_dem(world, rolling);
        RoadCompletionOptions wide; wide.abutment_path_m = 62;
        const auto fenced = road_bridge_anchor(env->context, *env->dem, zone, 5, wide), open = road_bridge_anchor(env->context, *full->dem, zone, 5, wide);
        std::printf("D29 control, 62 m budget at path 61.9 m: abutment with the tile's DEM %d, with the full DEM %d\n", fenced.plane.has_value(), open.plane.has_value());
        CHECK_FALSE(fenced.plane.has_value()); CHECK(open.plane.has_value());
    }
}

// ---- the tier on the two committed fixtures ----

namespace {
double tilted_fixture_dem(double e, double n) { return 100.0 + 0.03 * (e - 455000.0) + 2.0 * std::sin(e / 23.0) * std::cos(n / 31.0); }
std::string kind_of(const RoadGeomTile& t, const RoadGeomEntry& e) {
    const auto it = std::lower_bound(t.source.graph.ways.begin(), t.source.graph.ways.end(), e.way_id, [](const auto& w, auto id) { return w.osm_id < id; });
    const auto a = resolve_road_attributes(*it);
    return !a ? "other" : a->tunnel ? "tunnel" : a->bridge ? "bridge" : "ground";
}
std::string outcome_key(const RoadGeomEntry& e, const RoadProfileOrigin& o) {
    std::string s = std::to_string(static_cast<int>(o.provenance)) + ":" + std::to_string(o.strict_decline_code) + ":";
    if (!e.profile) return s + "declined";
    for (const auto& seg : e.profile->vertical.segments)
        for (const double d : {seg.station_m, seg.length_m, seg.height0_m, seg.height1_m, seg.grade0, seg.grade1}) s += std::to_string(std::bit_cast<std::uint64_t>(d)) + ",";
    return s;
}
}

TEST_CASE("completion: fixtures, completed stretches by kind before and after, tile invariance and bridge-node heights", "[layer][roads][completion][fixtures][invariance]") {
    struct Mode { const char* name; Fn dem; };
    const Mode modes[] = {{"flat DEM", Fn{}}, {"tilted bumpy DEM", Fn(tilted_fixture_dem)}};
    for (const char* pbf : {"frankfurt_hoechst.osm.pbf", "hoechst_a66.osm.pbf"}) {
        const auto data = g2m_test::read_osm_fixture(std::string(G2M_SOURCE_DIR) + "/testdata/fixtures/osm/" + pbf);
        const auto world = build_road_graph(data, g2m_test::kZone32N); REQUIRE(world);
        std::set<TileKey> keys;
        for (const auto& n : world->nodes) {
            const auto k = tile_key_at(g2m_test::kZone32N, 2, static_cast<double>(n.position.x) / 1000.0, static_cast<double>(n.position.y) / 1000.0);
            if (k) keys.insert(*k);
        }
        for (const auto& mode : modes) {
            std::map<std::pair<std::int64_t, std::uint32_t>, std::string> before, after, first_blob;
            std::size_t mismatches = 0, compared = 0;
            RoadCompletionStats stats;
            // Heights at the nodes where a completed or bridge profile ends.
            std::map<std::int64_t, std::vector<std::pair<double, ProfileProvenance>>> node_heights; std::set<std::int64_t> bridge_nodes;
            for (const auto& key : keys) {
                g2m_test::GeomOptions o; o.height_m = mode.dem; o.completion.enabled = true; o.stats = &stats;
                auto g = g2m_test::geometry_in_tile_with(*world, key, o); REQUIRE(g);
                for (std::size_t i = 0; i < g->entries.size(); ++i) {
                    const auto& e = g->entries[i]; const auto& origin = g->origins[i];
                    const auto id = std::make_pair(e.way_id, e.stretch.start_ref);
                    const std::string kind = kind_of(*g, e);
                    const auto blob = outcome_key(e, origin);
                    const auto [it, inserted] = first_blob.emplace(id, blob);
                    if (!inserted) { ++compared; if (it->second != blob) ++mismatches; }
                    before[id] = kind + ((e.profile && origin.provenance == ProfileProvenance::Strict) ? " ok" : " declined");
                    after[id] = kind + (e.profile ? std::string(" ") + name_of(origin.provenance) : std::string(" declined"));
                    if (!e.profile) continue;
                    const auto* way = &*std::lower_bound(g->source.graph.ways.begin(), g->source.graph.ways.end(), e.way_id, [](const auto& w, auto v) { return w.osm_id < v; });
                    for (const bool end : {false, true}) {
                        const auto node = way->node_ids[end ? e.stretch.end_ref : e.stretch.start_ref];
                        const auto z = e.profile->at(end ? e.profile->reference.length_m : 0);
                        node_heights[node].push_back({z->height_m, origin.provenance});
                        if (kind == "bridge") bridge_nodes.insert(node);
                    }
                }
            }
            std::map<std::string, int> b, a;
            for (const auto& [id, s] : before) ++b[s];
            for (const auto& [id, s] : after) ++a[s];
            std::printf("[completion] %s (%s): %zu stretches\n  before:", pbf, mode.name, before.size());
            for (const auto& [s, c] : b) std::printf(" %s=%d;", s.c_str(), c);
            std::printf("\n  after: ");
            for (const auto& [s, c] : a) std::printf(" %s=%d;", s.c_str(), c);
            std::printf("\n  tier counters over all tiles (an entry counts once per tile that holds it): retry %zu/%zu accepted; bridge %zu candidates, %zu completed, %zu without support, %zu fit failed; ground %zu candidates, %zu completed (%zu bridge-anchored), %zu failed; tunnels skipped %zu; strict anchors taken %zu (D30)\n",
                stats.retry_accepted, stats.retry_candidates, stats.bridge_candidates, stats.bridge_completed, stats.bridge_no_support, stats.bridge_fit_failed,
                stats.ground_candidates, stats.ground_completed, stats.ground_bridge_anchored, stats.ground_failed, stats.tunnel_skipped, stats.strict_anchors);
            std::printf("  tile invariance: %zu repeated entries compared across tiles, %zu differ\n", compared, mismatches);
            CHECK(mismatches == 0);
            CHECK(compared > 0);
            // T6, completion variant: the profile ends at a bridge node agree wherever a completed profile ends there.
            double worst_completed = 0, worst_mixed = 0; std::size_t nodes_completed = 0, nodes_mixed = 0;
            for (const auto node : bridge_nodes) {
                const auto& hs = node_heights[node];
                bool any_completed = false, any_strict = false;
                for (const auto& [h, p] : hs) { (void)h; if (p == ProfileProvenance::Strict) any_strict = true; else any_completed = true; }
                if (!any_completed) continue;
                const auto [lo, hi] = std::minmax_element(hs.begin(), hs.end(), [](const auto& x, const auto& y) { return x.first < y.first; });
                if (any_strict) { ++nodes_mixed; worst_mixed = std::max(worst_mixed, hi->first - lo->first); }
                else { ++nodes_completed; worst_completed = std::max(worst_completed, hi->first - lo->first); }
            }
            std::printf("  T6 completion variant: %zu bridge nodes with only completed ends, worst spread %.3e m; %zu nodes where a strict profile meets a completed one, worst spread %.4f m (D26, D30)\n",
                nodes_completed, worst_completed, nodes_mixed, worst_mixed);
            CHECK(worst_completed <= 1.0 / 256.0);
            CHECK(worst_mixed <= 1.0 / 256.0);   // D30: a completed end takes the strict anchor where strict profiles end
        }
    }
}

TEST_CASE("strict pass: the latent far-end read of a bridge anchor makes one entry tile-dependent (P1, measured in D32)", "[layer][roads][completion][invariance]") {
    // Way 10 runs from tile 455 to node 4 at x = 700 (tile 456, outside tile 455's 128 m halo), where a 600 m bridge (way 20)
    // starts; a ground road (way 12) continues from its far end at x = 1300. The strict anchor of node 4 (surface_anchor)
    // projects onto the span's feasible cubic rise, which needs the raw anchor of the FAR end: the DEM 32 m beyond x = 1300.
    // Tile 455 loads the DEM only 65 m around its own ways (to x = 765): its strict pass declines way 10 for a missing DEM
    // tile, tile 456 (which holds the bridge) profiles it. This is P1 behaviour, not the completion tier's.
    const auto world = world_of({{1, -300, 0}, {2, -100, 0}, {3, 300, 0}, {4, 700, 0}, {5, 1000, 0}, {6, 1300, 0}, {7, 1600, 0}},
        {{10, {1, 2, 3, 4}}, {20, {4, 5, 6}, kBridge}, {12, {6, 7}}});
    const auto west = make_env(world, kTile, rolling), east = make_env(world, kTileEast, rolling);
    const auto sw = build(*west, false), se = build(*east, false);
    const auto& a = sw.entries[index_of(sw, 10, 0)]; const auto& b = se.entries[index_of(se, 10, 0)];
    std::printf("D32 latent far end: way 10 in tile 455 %s, in tile 456 %s\n", a.profile ? "profiled" : a.decline->message.c_str(), b.profile ? "profiled" : b.decline->message.c_str());
    REQUIRE_FALSE(a.profile); CHECK(a.decline->message.find("road DEM") != std::string::npos);
    CHECK(b.profile.has_value());
    // The tier completes the west copy (a ground completion ending at the walk's abutment plane) and leaves the east one strict.
    const auto cw = build(*west, true);
    CHECK(cw.origins[index_of(cw, 10, 0)].provenance == ProfileProvenance::GroundCompleted);
}

TEST_CASE("completion: fixtures against a world reference (whole world as context, a DEM without holes); strict far-end dependence measured (D32)", "[layer][roads][completion][fixtures][invariance]") {
    // Every entry of every tile is compared with the same tile built from the WHOLE world graph as its context and a DEM
    // covering the whole world: the reads a tile cannot have are then all available. This covers every entry (not only the
    // ones two tiles both hold) and reads up to the bounds. The strict pass keeps P1's latent far-end dependence
    // (surface_anchor reads the far end of a bridge span), which this counts; the completion tier must add no difference of
    // its own: wherever the strict outcome agrees with the reference, the final outcome agrees too.
    struct Mode { const char* name; Fn dem; };
    const Mode modes[] = {{"flat DEM", Fn{}}, {"tilted bumpy DEM", Fn(tilted_fixture_dem)}};
    const auto strict_key = [](const RoadGeomEntry& e) {
        if (!e.profile) return std::string("declined:") + std::to_string(static_cast<int>(e.decline->code)) + ":" + e.decline->message;
        std::string s;
        for (const auto& seg : e.profile->vertical.segments)
            for (const double d : {seg.station_m, seg.length_m, seg.height0_m, seg.height1_m, seg.grade0, seg.grade1}) s += std::to_string(std::bit_cast<std::uint64_t>(d)) + ",";
        return s;
    };
    for (const char* pbf : {"frankfurt_hoechst.osm.pbf", "hoechst_a66.osm.pbf"}) {
        const auto data = g2m_test::read_osm_fixture(std::string(G2M_SOURCE_DIR) + "/testdata/fixtures/osm/" + pbf);
        const auto world = build_road_graph(data, g2m_test::kZone32N); REQUIRE(world);
        std::set<TileKey> keys;
        for (const auto& n : world->nodes) {
            const auto k = tile_key_at(g2m_test::kZone32N, 2, static_cast<double>(n.position.x) / 1000.0, static_cast<double>(n.position.y) / 1000.0);
            if (k) keys.insert(*k);
        }
        for (const auto& mode : modes) {
            const auto full = full_dem(*world, mode.dem);
            std::size_t entries = 0, strict_differ = 0, strict_differ_bridge_end = 0, tier_differ = 0, completed = 0, control_differ = 0;
            std::vector<std::string> examples;
            for (const auto& key : keys) {
                g2m_test::GeomOptions strict_o; strict_o.height_m = mode.dem;
                g2m_test::GeomOptions tier_o = strict_o; tier_o.completion.enabled = true;
                auto strict = g2m_test::geometry_in_tile_with(*world, key, strict_o); REQUIRE(strict);
                auto tier = g2m_test::geometry_in_tile_with(*world, key, tier_o); REQUIRE(tier);
                auto graph = select_road_graph_tile(*world, key); REQUIRE(graph);
                RoadProfileLimits limits; limits.reference.allow_curvature_exceptions = true;
                RoadCompletionOptions on; on.enabled = true;
                auto ref_strict = build_road_geom_tile(*graph, *full->dem, limits, &*world); REQUIRE(ref_strict);
                auto ref_tier = build_road_geom_tile(*graph, *full->dem, limits, &*world, builtin_road_rules_de(), on); REQUIRE(ref_tier);
                REQUIRE(strict->entries.size() == ref_strict->entries.size()); REQUIRE(tier->entries.size() == ref_tier->entries.size());
                // Control: the tier run on the tile's strict entries with the TILE graph as its context (and the full DEM).
                auto narrow_entries = strict->entries; std::vector<RoadProfileOrigin> narrow_origins;
                const auto narrow = RoadReferenceBuilder::make(graph->graph); REQUIRE(narrow);
                REQUIRE(complete_road_profiles(narrow_entries, narrow_origins, *narrow, *full->dem, key.zone, limits, on));
                for (std::size_t i = 0; i < tier->entries.size(); ++i) {
                    ++entries;
                    if (tier->origins[i].provenance != ProfileProvenance::Strict) ++completed;
                    const bool same_strict = strict_key(strict->entries[i]) == strict_key(ref_strict->entries[i]);
                    if (!same_strict) {
                        ++strict_differ;
                        const auto& e = strict->entries[i];
                        const bool far_end = e.decline && e.decline->message.find("road DEM") != std::string::npos;
                        if (far_end) ++strict_differ_bridge_end;
                        if (examples.size() < 4) {
                            char buf[512];
                            std::snprintf(buf, sizeof buf, "way %lld ref %u: tile '%s' / reference '%s'", static_cast<long long>(e.way_id), e.stretch.start_ref,
                                e.profile ? "profile" : e.decline->message.c_str(), ref_strict->entries[i].profile ? "profile" : ref_strict->entries[i].decline->message.c_str());
                            examples.push_back(buf);
                        }
                        continue;
                    }
                    const auto reference = outcome_key(ref_tier->entries[i], ref_tier->origins[i]);
                    if (outcome_key(tier->entries[i], tier->origins[i]) != reference) ++tier_differ;
                    if (outcome_key(narrow_entries[i], narrow_origins[i]) != reference) ++control_differ;
                }
            }
            std::printf("[D32] %s (%s): %zu entries over %zu tiles, %zu completed; strict outcome differs from the world reference for %zu (%zu of them a missing DEM tile); completion differs where strict agrees: %zu (control, tile graph as the tier's context: %zu)\n",
                pbf, mode.name, entries, keys.size(), completed, strict_differ, strict_differ_bridge_end, tier_differ, control_differ);
            for (const auto& ex : examples) std::printf("    %s\n", ex.c_str());
            CHECK(tier_differ == 0);
            CHECK(control_differ > 0);
        }
    }
}
