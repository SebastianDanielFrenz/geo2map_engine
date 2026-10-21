// Street fidelity P2 slice 6: the profile completion tier (design 6.6, tests T6 completion variant, T18 and the tile
// invariance argument of decision D25). Synthetic worlds for the three paths (retry, bridge, ground) compared with a
// frozen copy of racing_game's own code (tests/support/road_completion_reference.h), the tier on the two committed
// OSM fixtures, and the tile-invariance tests with their negative controls.
#include "road_fixture.h"
#include "road_completion_reference.h"
#include <catch2/catch_test_macros.hpp>
#include <algorithm>
#include <bit>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <functional>
#include <limits>
#include <map>
#include <memory>
#include <set>
#include <string>
#include <atomic>
#include <thread>
#include "g2m/layer/road_envelope.h"
#include "g2m/core/hash.h"
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
// The deriver's inputs (both discovery stages); `bridge_far_end` = false stops after stage 1 (the pre-D35 set).
std::unique_ptr<Env> make_env(const RoadGraph& world, const TileKey& key, const Fn& height, bool bridge_far_end = true,
        std::int64_t walk_margin_mm = kRoadBridgeWalkMarginMm) {
    auto env = std::make_unique<Env>();
    auto in = g2m_test::road_geom_inputs(world, key, bridge_far_end, walk_margin_mm); REQUIRE(in);
    env->tile = std::move(in->tile); env->context = std::move(in->context);
    env->heights.resize(in->dem_keys.size());
    std::vector<const HeightTile*> views;
    for (std::size_t i = 0; i < env->heights.size(); ++i) { env->heights[i].key = in->dem_keys[i]; g2m_test::fill_dem(env->heights[i], height); views.push_back(&env->heights[i]); }
    auto dem = RoadDemSampler::make(views); REQUIRE(dem); env->dem = std::move(*dem);
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
    return p == ProfileProvenance::Strict ? "strict" : p == ProfileProvenance::Retry ? "retry" : p == ProfileProvenance::BridgeCompleted ? "bridge-completed" : p == ProfileProvenance::TunnelFloor ? "tunnel-floor" : "ground-completed";
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
    const auto bytes = encode_road_geom(lib); REQUIRE(bytes); CHECK(bytes->encoding == 8);
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
    bad = lib; bad.origins[span].provenance = static_cast<ProfileProvenance>(5); CHECK_FALSE(validate_road_geom_tile(bad));
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

TEST_CASE("completion: a short ground stretch whose end planes contradict its measured rise is completed with end grades eased toward the chord (D43)", "[layer][roads][completion][D43]") {
    // racing_game report 2026-10-07 (rhein_main_hessen, A 648 under the Westerbachstrasse bridge, UTM32 E 469992 N 5552459):
    // way 608415202 is a 27.6 m motorway split (maxheight under the deck) between two degree-2 nodes. The bare-earth DEM is
    // interpolated under the deck and catches up 0.5 m right at its edge, so the 3 m DEM plane at the far node reads -5.4 %
    // while the stretch descends -2.4 % end to end. With those end grades no |z''| <= 1/500 m cubic can descend only
    // 0.67 m: strict, retry and the ground fit all declined, the lower road was neither carved nor drawn. Scene: a
    // motorway 1 -> 2 -> 3 -> 4 split at 2 and 3 (way 11 = the 27.6 m stretch), -2.5 % with a 0.12 m tanh catch-up at 3.
    const auto world = world_of({{1, -300, 0}, {2, 0, 0}, {3, 27.6, 0}, {4, 300, 0}},
        {{10, {1, 2}, {{"highway", "motorway"}}}, {11, {2, 3}, {{"highway", "motorway"}}}, {12, {3, 4}, {{"highway", "motorway"}}}});
    const Fn catch_up = [](double e, double) { const double x = e - kOriginE; return 100.0 - 0.025 * x - 0.06 * std::tanh((x - 27.6) / 2.0); };
    const auto env = make_env(world, kTile, catch_up);
    const auto strict = build(*env, false);
    const auto mid = index_of(strict, 11, 0);
    REQUIRE_FALSE(strict.entries[mid].profile);
    std::printf("D43 scene strict decline: %s\n", strict.entries[mid].decline->message.c_str());
    // The end planes and the chord the tier sees (completion anchors: no bridge here, so the DEM plane at the node).
    const auto node_at = [&](std::int64_t id) {
        return std::lower_bound(env->context.nodes.begin(), env->context.nodes.end(), id, [](const auto& n, auto v) { return n.osm_id < v; })->position;
    };
    const auto p2 = env->dem->junction_plane(zone, node_at(2)); REQUIRE(p2);
    const auto p3 = env->dem->junction_plane(zone, node_at(3)); REQUIRE(p3);
    // Way 11 runs due east: the plane grade along it is grade_x.
    const double length = 27.6, chord = (p3->height - p2->height) / 256.0 / length;
    std::printf("D43 scene: plane grades %.4f / %.4f, chord %.4f\n", p2->grade_x, p3->grade_x, chord);
    REQUIRE(p3->grade_x < chord - 0.02);   // the scene does contradict its rise
    // The game's own completion (frozen reference) declines it too: the lower road stays uncarved in racing_game.
    auto game = strict; g2m_test::game_reference::complete_road_profiles(game, *env->dem);
    CHECK_FALSE(game.entries[mid].profile);
    RoadCompletionStats stats; const auto lib = build(*env, true, &stats);
    REQUIRE(lib.entries[mid].profile);
    CHECK(lib.origins[mid].provenance == ProfileProvenance::GroundCompleted);
    CHECK(stats.ground_grades_eased >= 1);
    const auto& p = *lib.entries[mid].profile;
    REQUIRE(std::fabs(p.reference.length_m - length) < 1e-6);
    const auto& first = p.vertical.segments.front(); const auto& last = p.vertical.segments.back();
    // Endpoint heights win: the measured plane heights at both nodes, untouched.
    CHECK(first.height0_m == p2->height / 256.0); CHECK(last.height1_m == p3->height / 256.0);
    // End grades moved toward the chord by the same fraction t = k/8 at both ends, the smallest k that fits.
    const double chord_ref = (last.height1_m - first.height0_m) / p.reference.length_m;
    const double t = (last.grade1 - p3->grade_x) / (chord_ref - p3->grade_x);
    std::printf("D43 completed: end grades %.4f / %.4f (eased by t = %.3f), kink at node 3 %.4f instead of a decline\n",
        first.grade0, last.grade1, t, std::fabs(last.grade1 - p3->grade_x));
    CHECK(t > 0); CHECK(t <= 1);
    CHECK(std::fabs(t * 8 - std::round(t * 8)) < 1e-9);
    CHECK(std::fabs(first.grade0 - (p2->grade_x + t * (chord_ref - p2->grade_x))) < 1e-12);
    {   // The previous step (k - 1)/8 does not fit: the ease is minimal on the ladder.
        const double tp = t - 0.125;
        const std::array<VerticalSample, 2> samples{{{0, first.height0_m}, {p.reference.length_m, last.height1_m}}};
        VerticalFitLimits fit; fit.max_grade = 0.12; fit.min_radius_m = 500; fit.max_iterations = 16384;
        fit.start_grade = p2->grade_x + tp * (chord_ref - p2->grade_x); fit.end_grade = p3->grade_x + tp * (chord_ref - p3->grade_x);
        CHECK_FALSE(fit_vertical_profile(samples, fit));
    }
    REQUIRE(validate_road_geom_tile(lib));
}

TEST_CASE("completion: an end plane far from the chord is terrain, not a misread: the ground stretch stays declined (D43)", "[layer][roads][completion][D43]") {
    // The D43 scene with a 0.34 m catch-up at node 3: the plane there departs from the chord by more than
    // kGroundEaseMismatch (0.05), so the ease does not run. On the bumpy G3-D terrain such stretches eased to the chord
    // rode off the ground (junction gradient 3.08 between an eased approach and its climbing neighbour).
    const auto world = world_of({{1, -300, 0}, {2, 0, 0}, {3, 27.6, 0}, {4, 300, 0}},
        {{10, {1, 2}, {{"highway", "motorway"}}}, {11, {2, 3}, {{"highway", "motorway"}}}, {12, {3, 4}, {{"highway", "motorway"}}}});
    const Fn catch_up = [](double e, double) { const double x = e - kOriginE; return 100.0 - 0.025 * x - 0.17 * std::tanh((x - 27.6) / 2.0); };
    const auto env = make_env(world, kTile, catch_up);
    const auto node_at = [&](std::int64_t id) {
        return std::lower_bound(env->context.nodes.begin(), env->context.nodes.end(), id, [](const auto& n, auto v) { return n.osm_id < v; })->position;
    };
    const auto p2 = env->dem->junction_plane(zone, node_at(2)); REQUIRE(p2);
    const auto p3 = env->dem->junction_plane(zone, node_at(3)); REQUIRE(p3);
    const double chord = (p3->height - p2->height) / 256.0 / 27.6;
    std::printf("D43 far scene: plane grades %.4f / %.4f, chord %.4f\n", p2->grade_x, p3->grade_x, chord);
    REQUIRE(std::fabs(p3->grade_x) < 0.10);            // in range: the ground step's clamp does not hide the mismatch
    REQUIRE(std::fabs(p3->grade_x - chord) > 0.05);
    RoadCompletionStats stats; const auto lib = build(*env, true, &stats);
    const auto mid = index_of(lib, 11, 0);
    CHECK_FALSE(lib.entries[mid].profile);
    CHECK(stats.ground_grades_eased == 0);
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
// D48 fence (terrain migration phase 5). A bridge B (way 40) from the west tile 455 to node 5 at x = 817.9, outside the
// 128 m halo of tile 455; a 1 m bridge (way 41) to node 6 at 818.9, a second bridge (way 42, `span` m) and a ground road
// (way 31) run east, so the walk from node 5 crosses 1 + `span` m of bridge and finds its support 32 m further: path
// span + 33. Ways 40 and 41 are the bridges incident to tile nodes (stage 2, D35); their nodes' 205 m squares end at
// x = 1023.9, just short of the DEM tile edge at x = 1024. Way 42 is not incident to a tile node: only the squares
// cover the walk beyond way 41's own 65 m margin.
RoadGraph fence_world(double span, double x5 = 817.9) {
    return world_of({{1, 300, 0}, {2, 400, 0}, {3, 500, 0}, {4, 600, 0}, {5, x5, 0}, {6, x5 + 1, 0}, {7, x5 + 1 + span, 0}, {8, x5 + 101 + span, 0}},
        {{30, {1, 2, 3}}, {40, {3, 4, 5}, kBridge}, {41, {5, 6}, kBridge}, {42, {6, 7}, kBridge}, {31, {7, 8}}});
}
}

TEST_CASE("completion: the walk budget bound keeps every DEM read inside the stage-2 walk squares (D29, D48)", "[layer][roads][completion][invariance][D48]") {
    RoadCompletionOptions opts; opts.enabled = true;
    // The bound itself: 201.5 m accepted, anything beyond refused before any work.
    {
        const auto env = make_env(fence_world(168.4), kTile, rolling);
        const auto builder = RoadReferenceBuilder::make(env->context); REQUIRE(builder);
        auto entries = build(*env, false).entries; std::vector<RoadProfileOrigin> origins;
        opts.abutment_path_m = kRoadCompletionMaxAbutmentPathM;
        CHECK(complete_road_profiles(entries, origins, *builder, *env->dem, zone, env->limits, opts));
        opts.abutment_path_m = kRoadCompletionMaxAbutmentPathM + 0.1;
        CHECK_FALSE(complete_road_profiles(entries, origins, *builder, *env->dem, zone, env->limits, opts));
    }
    opts.abutment_path_m = kRoadCompletionMaxAbutmentPathM;
    // Paths 201.4 m (inside the bound) and 201.9 m (beyond it): the tile's own dependencies and a DEM without holes give
    // the same tier result for every entry.
    for (const double span : {168.4, 168.9}) {
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
        const auto a = road_bridge_anchor(env->context, *env->dem, zone, 5, opts);
        std::printf("D48 fence, walk path %.1f m at budget %.1f m: abutment at node 5 %d, bridge completed %d (fenced DEM and full DEM agree)\n",
            span + 33, opts.abutment_path_m, a.plane.has_value(), completed);
        CHECK(a.plane.has_value() == (span + 33 <= opts.abutment_path_m));
    }
    // Control 1: a 203 m budget at path 202.9 m reads beyond the squares: the tile's DEM misses the cell the full DEM has.
    {
        const auto world = fence_world(169.9);
        const auto env = make_env(world, kTile, rolling); const auto full = full_dem(world, rolling);
        RoadCompletionOptions wide; wide.abutment_path_m = 203;
        const auto fenced = road_bridge_anchor(env->context, *env->dem, zone, 5, wide), open = road_bridge_anchor(env->context, *full->dem, zone, 5, wide);
        std::printf("D48 control, 203 m budget at path 202.9 m: abutment with the tile's DEM %d, with the full DEM %d\n", fenced.plane.has_value(), open.plane.has_value());
        CHECK_FALSE(fenced.plane.has_value()); CHECK(open.plane.has_value());
    }
    // Control 2: the pre-D48 stage 2 (no squares) misses way 42's ground support at a served 200 m walk. Node 5 at
    // x = 830: the support at x = 1029 lies in the DEM tile beyond x = 1024, which only the squares (to 1036) list.
    {
        const auto world = fence_world(166, 830);
        const auto env = make_env(world, kTile, rolling, true, 0); const auto full = full_dem(world, rolling);
        const auto served = served_road_completion_options();
        const auto fenced = road_bridge_anchor(env->context, *env->dem, zone, 5, served), open = road_bridge_anchor(env->context, *full->dem, zone, 5, served);
        std::printf("D48 control, no walk squares at path 199 m: abutment with the tile's DEM %d, with the full DEM %d\n", fenced.plane.has_value(), open.plane.has_value());
        CHECK_FALSE(fenced.plane.has_value()); CHECK(open.plane.has_value());
        const auto squared = make_env(world, kTile, rolling);
        CHECK(road_bridge_anchor(squared->context, *squared->dem, zone, 5, served).plane.has_value());
    }
}

TEST_CASE("completion: walks of 60, 200 and 500 m at the 60 m and the served 200 m budget (D48)", "[layer][roads][completion][D48]") {
    // fence_world(span) needs a walk of span + 33 m from node 5.
    for (const double path : {58.0, 190.0, 480.0}) {
        const auto world = fence_world(path - 33);
        const auto env = make_env(world, kTile, rolling); const auto full = full_dem(world, rolling);
        for (const double budget : {60.0, 200.0}) {
            RoadCompletionOptions o = served_road_completion_options(); o.abutment_path_m = budget;
            const auto fenced = road_bridge_anchor(env->context, *env->dem, zone, 5, o), open = road_bridge_anchor(env->context, *full->dem, zone, 5, o);
            REQUIRE(fenced.bridging);
            CHECK(fenced.plane.has_value() == open.plane.has_value());
            if (fenced.plane && open.plane) CHECK(fenced.plane->height == open.plane->height);
            CHECK(fenced.plane.has_value() == (path <= budget));
            RoadCompletionStats stats; auto g = build_road_geom_tile(env->tile, *env->dem, env->limits, &env->context, builtin_road_rules_de(), o, &stats); REQUIRE(g);
            const auto i = index_of(*g, 40, 0);
            std::printf("D48 walk %.0f m at budget %.0f m: abutment %d, bridge 40 %s\n", path, budget, fenced.plane.has_value(),
                g->entries[i].profile ? (g->origins[i].provenance == ProfileProvenance::BridgeCompleted ? "completed" : "profiled") : "declined");
            // Without support at node 5 the span has no second abutment: it stays declined (the relaxed secant needs both).
            CHECK(g->entries[i].profile.has_value() == (path <= budget));
        }
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

TEST_CASE("completion: fixtures, completed stretches by kind before and after, tile invariance and bridge-node heights", "[layer][roads][completion][fixtures][invariance][long]") {
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
                auto g = g2m_test::geometry_in_tile_with(*world, key, o); if (!g) FAIL(g.error().message);
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
            std::printf("\n  tier counters over all tiles (an entry counts once per tile that holds it): retry %zu/%zu accepted; bridge %zu candidates, %zu completed, %zu without support, %zu fit failed; ground %zu candidates, %zu completed (%zu bridge-anchored, %zu with eased end grades, D43), %zu failed; tunnels %zu candidates, %zu floors (%zu chords), %zu failed; strict anchors taken %zu (D30); accepted bridge planes %zu, walks over a later bridge edge %zu (D47)\n",
                stats.retry_accepted, stats.retry_candidates, stats.bridge_candidates, stats.bridge_completed, stats.bridge_no_support, stats.bridge_fit_failed,
                stats.ground_candidates, stats.ground_completed, stats.ground_bridge_anchored, stats.ground_grades_eased, stats.ground_failed,
                stats.tunnel_candidates, stats.tunnel_completed, stats.tunnel_chord, stats.tunnel_failed, stats.strict_anchors, stats.bridge_anchor_reused, stats.walk_other_side);
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

TEST_CASE("strict pass: the far-end read of a bridge anchor is tile-invariant with the bridge dependencies (D32, fixed by D35)", "[layer][roads][completion][invariance]") {
    // Way 10 runs from tile 455 to node 4 at x = 700 (tile 456, outside tile 455's 128 m halo), where a bridge (way 20)
    // starts; a ground road (way 12) continues from its far end. The strict anchor of node 4 (surface_anchor) projects onto
    // the span's feasible cubic rise, which needs the raw anchor of the FAR end: the 32 m walk along way 12 and the DEM
    // around it. Tile 455 does not hold the bridge. Two spans:
    //   600 m: the far end (x = 1300) lies in tile 456, whose graph tile is already in tile 455's stretch context, so only
    //          the DEM beyond x = 765 is missing from the stage-1 set;
    //   1000 m: the far end (x = 1700) lies in tile 457, 164 m east of tile 456: way 12 is in neither stage-1 context tile,
    //          so the far end's ground approach is missing as well.
    // D35: the second discovery stage adds every incident bridge's DEM (65 m margin) and the graph tiles of its nodes, so
    // both tiles derive way 10 alike. Negative control: the stage-1 set (the pre-D35 deriver) - tile 455 declines way 10,
    // tile 456 profiles it (the D32 measurement).
    for (const double span : {600.0, 1000.0}) {
        const double far = 700 + span;
        const auto world = world_of({{1, -300, 0}, {2, -100, 0}, {3, 300, 0}, {4, 700, 0}, {5, 700 + span / 2, 0}, {6, far, 0}, {7, far + 300, 0}},
            {{10, {1, 2, 3, 4}}, {20, {4, 5, 6}, kBridge}, {12, {6, 7}}});
        const auto west = make_env(world, kTile, rolling), east = make_env(world, kTileEast, rolling);
        const auto sw = build(*west, false), se = build(*east, false);
        const auto ia = index_of(sw, 10, 0), ib = index_of(se, 10, 0);
        const auto& a = sw.entries[ia]; const auto& b = se.entries[ib];
        std::printf("D35 far end, span %.0f m: way 10 in tile 455 %s, in tile 456 %s\n", span, a.profile ? "profiled" : a.decline->message.c_str(),
            b.profile ? "profiled" : b.decline->message.c_str());
        REQUIRE(b.profile.has_value());   // a real profile, not two equal declines
        CHECK(same_outcome(a, sw.origins[ia], b, se.origins[ib]));
        // With the tier the two copies stay strict and equal too.
        const auto cw = build(*west, true), ce = build(*east, true);
        CHECK(cw.origins[index_of(cw, 10, 0)].provenance == ProfileProvenance::Strict);
        CHECK(same_outcome(cw.entries[index_of(cw, 10, 0)], cw.origins[index_of(cw, 10, 0)], ce.entries[index_of(ce, 10, 0)], ce.origins[index_of(ce, 10, 0)]));
        // Negative control: without the second stage tile 455 declines (missing DEM or missing ground-approach context).
        const auto stage1 = make_env(world, kTile, rolling, false);
        const auto s1 = build(*stage1, false); const auto& c = s1.entries[index_of(s1, 10, 0)];
        std::printf("    stage-1 dependencies only (pre-D35): way 10 in tile 455 %s\n", c.profile ? "profiled" : c.decline->message.c_str());
        CHECK_FALSE(c.profile.has_value());
        CHECK_FALSE(same_outcome(c, s1.origins[index_of(s1, 10, 0)], b, se.origins[ib]));
    }
}

TEST_CASE("completion: fixtures against a world reference (whole world as context, a DEM without holes); strict far-end reads tile-invariant (D32, D35)", "[layer][roads][completion][fixtures][invariance][long]") {
    // Every entry of every tile is compared with the same tile built from the WHOLE world graph as its context and a DEM
    // covering the whole world: the reads a tile cannot have are then all available. This covers every entry (not only the
    // ones two tiles both hold) and reads up to the bounds. P1's latent far-end dependence (surface_anchor reads
    // the far end of a bridge span, D32) is closed by D35's second discovery stage: the strict outcome must agree everywhere,
    // and the completion tier must add no difference of its own.
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
        {   // D35: how much the second discovery stage adds on the fixture (graph and DEM tiles beyond stage 1).
            std::size_t tiles_added = 0, graph_added = 0, dem_added = 0;
            for (const auto& key : keys) {
                const auto s1 = g2m_test::road_geom_inputs(*world, key, false), s2 = g2m_test::road_geom_inputs(*world, key, true);
                REQUIRE(s1); REQUIRE(s2);
                const auto ways_of = [](const RoadGraph& g) { std::set<std::int64_t> w; for (const auto& x : g.ways) w.insert(x.osm_id); return w; };
                const std::size_t g = ways_of(s2->context).size() - ways_of(s1->context).size(), d = s2->dem_keys.size() - s1->dem_keys.size();
                if (g || d) ++tiles_added;
                graph_added += g; dem_added += d;
            }
            std::printf("[D35] %s: %zu tiles, stage 2 adds dependencies in %zu (context ways +%zu, DEM tiles +%zu)\n", pbf, keys.size(), tiles_added, graph_added, dem_added);
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
            CHECK(strict_differ == 0);   // D35: no far-end read left outside the dependency set
            CHECK(tier_differ == 0);
            CHECK(control_differ > 0);
        }
    }
}

// ---------------------------------------------------------------------------------------------------------------
// Terrain migration phase 4 (decision D47): tunnel floors and the game's accepted-bridge anchors, against the game's
// live code (game_reference::complete_road_profiles_live, racing_game ef4ce25) run on what the previous release served.
namespace {
const Tags kTunnel = {{"highway", "primary"}, {"tunnel", "yes"}, {"layer", "-1"}};
// What the release before phase 4 served: the tier without tunnel floors and without D47's anchors.
// The completion of the release before terrain migration phase 4: no tunnel floors, no accepted-bridge anchors, 60 m walk.
RoadCompletionOptions previous_served() { auto o = served_road_completion_options(); o.tunnel_floors = false; o.bridge_anchor_reuse = false; o.abutment_path_m = 60; o.tunnel_chord_min_length_m = 0; return o; }
// Phase 4's release (D47 at the 60 m walk), before D48 and before the straight-grade ruling.
RoadCompletionOptions phase4_served() { auto o = served_road_completion_options(); o.abutment_path_m = 60; o.tunnel_chord_min_length_m = 0; return o; }
// Phase 5's release (D48), before the owner's straight-grade ruling: every tunnel follows the DEM.
RoadCompletionOptions phase5_served() { auto o = served_road_completion_options(); o.tunnel_chord_min_length_m = 0; return o; }
RoadGeomTile build_with(const Env& env, const RoadCompletionOptions& o, RoadCompletionStats* stats = nullptr) {
    auto g = build_road_geom_tile(env.tile, *env.dem, env.limits, &env.context, builtin_road_rules_de(), o, stats); REQUIRE(g);
    return std::move(*g);
}
// Nodes along an arc from (x0, y0) with initial heading h0 (rad), radius r (0 = straight), length len, every `step` metres.
std::vector<N> arc(std::int64_t first_id, double x0, double y0, double h0, double r, double len, double step) {
    std::vector<N> out;
    const int n = static_cast<int>(std::ceil(len / step));
    for (int i = 0; i <= n; ++i) {
        const double s = len * i / n;
        if (r == 0) { out.push_back({first_id + i, x0 + s * std::cos(h0), y0 + s * std::sin(h0)}); continue; }
        const double a = s / r;
        out.push_back({first_id + i, x0 + r * (std::sin(h0 + a) - std::sin(h0)), y0 - r * (std::cos(h0 + a) - std::cos(h0))});
    }
    return out;
}
// A ground road, a tunnel along `tunnel` (ids from 100) and a ground road after it.
RoadGraph tunnel_world(const std::vector<N>& tunnel) {
    std::vector<N> nodes = tunnel;
    const auto& a = tunnel.front(); const auto& b = tunnel.back();
    nodes.push_back({1, a.x - 300, a.y}); nodes.push_back({2, a.x - 150, a.y});
    nodes.push_back({3, b.x + 150, b.y}); nodes.push_back({4, b.x + 300, b.y});
    std::vector<std::int64_t> ids; for (const auto& n : tunnel) ids.push_back(n.id);
    return world_of(nodes, {{10, {1, 2, a.id}}, {30, ids, kTunnel}, {12, {b.id, 3, 4}}});
}
double chord_departure(const RoadProfile& p) {
    const double L = p.reference.length_m, h0 = p.at(0)->height_m, h1 = p.at(L)->height_m;
    double worst = 0;
    for (double s = 0; s <= L; s += 1) worst = std::max(worst, std::fabs(p.at(s)->height_m - (h0 + (h1 - h0) * s / L)));
    return worst;
}
}

TEST_CASE("tunnel floors: the game's tunnel step byte for byte on a straight and a curved, sloped tunnel (D47)", "[layer][roads][completion][tunnel][D47]") {
    // A hill over the tunnel and a 4 % slope along it: the floor follows the DEM at <= 30 % and a 20 m vertical radius.
    const Fn hill = [](double e, double n) {
        const double x = e - kOriginE, y = n - kOriginN;
        return 100.0 + 0.04 * x + 0.01 * y + 6.0 * std::exp(-(x * x + y * y) / (2 * 60.0 * 60.0));
    };
    struct Case { const char* name; double radius; };
    for (const Case c : {Case{"straight", 0}, Case{"curved r=120 m", 120}}) {
        const auto world = tunnel_world(arc(100, -90, 0, 0.2, c.radius, 180, 12));
        const auto env = make_env(world, kTile, hill);
        const auto strict = build(*env, false);
        const auto t = index_of(strict, 30, 0);
        REQUIRE_FALSE(strict.entries[t].profile);
        CHECK(strict.entries[t].decline->message.find("tunnel") != std::string::npos);
        RoadCompletionStats stats; const auto lib = build(*env, true, &stats);
        REQUIRE(lib.entries[t].profile);
        CHECK(lib.origins[t].provenance == ProfileProvenance::TunnelFloor);
        CHECK(lib.origins[t].strict_decline_code == static_cast<std::uint16_t>(strict.entries[t].decline->code));
        CHECK(stats.tunnel_completed == 1); CHECK(stats.tunnel_chord == 0);
        // The game's live code on what the previous release served: identical vertical profile, bit for bit.
        auto game = build_with(*env, previous_served());
        REQUIRE_FALSE(game.entries[t].profile);
        g2m_test::game_reference::complete_road_profiles_live(game, *env->dem);
        REQUIRE(game.entries[t].profile);
        const bool identical = same_vertical(lib.entries[t].profile->vertical, game.entries[t].profile->vertical);
        const auto d = max_delta(*lib.entries[t].profile, *game.entries[t].profile);
        std::printf("D47 tunnel %s: %zu vertical pieces, chord departure %.2f m, vs the game: identical=%d max |dz| %.3e m\n", c.name,
            lib.entries[t].profile->vertical.segments.size(), chord_departure(*lib.entries[t].profile), identical ? 1 : 0, d.height);
        CHECK(identical);
        CHECK(lib.entries[t].profile->reference.length_m == game.entries[t].profile->reference.length_m);
        // Sabotages: no tunnel step leaves it declined; a 10 m sample spacing is another profile than the game's.
        // Below the served straight-grade threshold (200 m) the served tier gives this 180 m tunnel the same DEM floor.
        { const auto served = build_with(*env, served_road_completion_options()); REQUIRE(served.entries[t].profile);
          CHECK(same_vertical(served.entries[t].profile->vertical, lib.entries[t].profile->vertical)); }
        auto off = served_road_completion_options(); off.tunnel_floors = false;
        CHECK_FALSE(build_with(*env, off).entries[t].profile);
        auto coarse = served_road_completion_options(); coarse.tunnel_spacing_m = 10;
        const auto other = build_with(*env, coarse); REQUIRE(other.entries[t].profile);
        CHECK_FALSE(same_vertical(other.entries[t].profile->vertical, game.entries[t].profile->vertical));
        // Downstream: still a Tunnel approach at its junctions, no grip segment painted from it, validated and stored as 8.
        for (const auto& j : lib.junctions) for (const auto& a : j.approaches) if (a.entry_index == t) CHECK(a.status == JunctionApproachStatus::Tunnel);
        for (int i = 0; i < 4; ++i) for (int k = 0; k < 4; ++k) {
            const TileKey l0{zone, 0, kTile.x * 4 + i, kTile.y * 4 + k};
            for (const auto& seg : envelope_road_segments(lib, l0, 1.0)) CHECK(seg.way_id != 30);
        }
        REQUIRE(validate_road_geom_tile(lib));
        auto claimed = lib; claimed.origins[t] = RoadProfileOrigin{}; CHECK_FALSE(validate_road_geom_tile(claimed));   // a tunnel profile is never Strict
        auto as_ground = lib; as_ground.origins[t].provenance = ProfileProvenance::GroundCompleted; CHECK_FALSE(validate_road_geom_tile(as_ground));
        const auto bytes = encode_road_geom(lib); REQUIRE(bytes); CHECK(bytes->encoding == 8);
        const auto back = decode_road_geom(*bytes); REQUIRE(back); CHECK(back->origins == lib.origins);
        REQUIRE(back->entries[t].profile); CHECK(same_vertical(back->entries[t].profile->vertical, lib.entries[t].profile->vertical));
        auto seven = lib; seven.encoding = kRoadGeomEncodingJunctions;
        const auto refused = encode_road_geom(seven); REQUIRE_FALSE(refused);
        CHECK(refused.error().message.find("tunnel floor") != std::string::npos);
        auto relabelled = *bytes; relabelled.encoding = kRoadGeomEncodingJunctions; CHECK_FALSE(decode_road_geom(relabelled));   // 7 knows no provenance 4
    }
}

TEST_CASE("tunnel floors: three long tunnels (539, 499, 498 m) follow the straight portal grade as served; the DEM floor below the threshold (owner ruling 2026-10-15)", "[layer][roads][completion][tunnel][D47]") {
    // A 45 m high ridge over every tunnel and portals 12 m apart in height: the DEM floor (threshold 0, the release
    // before the ruling) rides over the ridge (the owner's deep-tunnel concern); the served threshold (de.json, 200 m)
    // gives the straight portal chord, along the curve too. Straight, one arc and an S of two arcs.
    const Fn ridge = [](double e, double n) {
        const double x = e - kOriginE, y = n - kOriginN;
        return 100.0 + 0.012 * x + 0.004 * y + 45.0 * std::exp(-(x * x + y * y) / (2 * 120.0 * 120.0));
    };
    struct Case { const char* name; double length; std::vector<N> nodes; };
    std::vector<Case> cases;
    cases.push_back({"straight 539 m", 539, arc(100, -270, 0, 0.0, 0, 539, 15)});
    cases.push_back({"curved 499 m (r 300 m)", 499, arc(100, -240, -60, 0.4, 300, 499, 15)});
    {   // S curve: 249.5 m left at r 250 m, then 248.5 m right at r 250 m
        auto first = arc(100, -230, -40, 0.3, 250, 249.5, 15); const auto& e = first.back();
        const double h = 0.3 + 249.5 / 250.0;
        auto second = arc(100 + static_cast<std::int64_t>(first.size()) - 1, e.x, e.y, h, -250, 248.5, 15);
        first.insert(first.end(), second.begin() + 1, second.end());
        cases.push_back({"S-curved 498 m", 498, first});
    }
    for (const auto& c : cases) {
        const auto world = tunnel_world(c.nodes);
        const auto env = make_env(world, kTile, ridge);
        RoadCompletionStats floor_stats; const auto floor = build(*env, true, &floor_stats);
        const auto t = index_of(floor, 30, 0);
        REQUIRE(floor.entries[t].profile); CHECK(floor.origins[t].provenance == ProfileProvenance::TunnelFloor);
        const double L = floor.entries[t].profile->reference.length_m;
        const auto chord_opts = served_road_completion_options();
        REQUIRE(chord_opts.tunnel_chord_min_length_m == 200);
        RoadCompletionStats chord_stats; const auto chord = build_with(*env, chord_opts, &chord_stats);
        { RoadCompletionStats s5; const auto before = build_with(*env, phase5_served(), &s5);   // the previous release: the DEM floor
          REQUIRE(before.entries[t].profile); CHECK(same_vertical(before.entries[t].profile->vertical, floor.entries[t].profile->vertical)); CHECK(s5.tunnel_chord == 0); }
        REQUIRE(chord.entries[t].profile); CHECK(chord.origins[t].provenance == ProfileProvenance::TunnelFloor);
        const auto& fp = *floor.entries[t].profile; const auto& cp = *chord.entries[t].profile;
        double max_grade = 0; for (const auto& s : fp.vertical.segments) max_grade = std::max({max_grade, std::fabs(s.grade0), std::fabs(s.grade1)});
        std::printf("D47 long tunnel %s: reference %.1f m; DEM floor departs %.2f m from the portal chord (max grade %.3f); chord mode departs %.2e m, portals %.2f / %.2f m\n",
            c.name, L, chord_departure(fp), max_grade, chord_departure(cp), cp.at(0)->height_m, cp.at(L)->height_m);
        CHECK(std::fabs(L - c.length) < 2.0);
        CHECK(chord_departure(fp) > 20.0);      // the DEM floor follows the ridge
        CHECK(max_grade <= 0.30 + 1e-9);
        CHECK(chord_departure(cp) < 1e-6);      // the chord: a straight grade line along the curved reference
        CHECK(cp.at(0)->height_m == fp.at(0)->height_m); CHECK(cp.at(L)->height_m == fp.at(L)->height_m);   // same portal samples
        CHECK(chord_stats.tunnel_chord == 1); CHECK(floor_stats.tunnel_chord == 0);
        // Below the threshold the floor rule stays: a threshold longer than the tunnel gives the DEM floor bit for bit.
        auto above = served_road_completion_options(); above.tunnel_chord_min_length_m = 600;
        const auto same = build_with(*env, above); REQUIRE(same.entries[t].profile);
        CHECK(same_vertical(same.entries[t].profile->vertical, fp.vertical));
    }
    {   // A chord steeper than tunnel_max_grade is not taken: the outcome is the DEM-floor outcome.
        const auto world = tunnel_world(arc(100, -100, 0, 0, 0, 200, 15));
        const Fn steep = [](double e, double) { return 100.0 + 0.33 * (e - kOriginE); };
        const auto env = make_env(world, kTile, steep);
        auto chord_opts = served_road_completion_options(); chord_opts.tunnel_chord_min_length_m = 150;
        RoadCompletionStats s; const auto a = build_with(*env, chord_opts, &s); const auto b = build(*env, true);
        const auto t = index_of(a, 30, 0);
        CHECK(s.tunnel_chord == 0);
        CHECK(same_outcome(a.entries[t], a.origins[t], b.entries[t], b.origins[t]));
    }
    // Option bounds: the validator's certificate (30 %, 20 m) is the outer limit of the options.
    {
        const auto world = tunnel_world(arc(100, -90, 0, 0, 0, 180, 12));
        const auto env = make_env(world, kTile, rolling);
        const auto builder = RoadReferenceBuilder::make(env->context); REQUIRE(builder);
        const auto strict = build(*env, false);
        for (int bad = 0; bad < 4; ++bad) {
            auto o = served_road_completion_options();
            if (bad == 0) o.tunnel_max_grade = 0.31;
            if (bad == 1) o.tunnel_min_radius_m = 19;
            if (bad == 2) o.tunnel_spacing_m = 0.25;
            if (bad == 3) o.tunnel_chord_min_length_m = -1;
            auto entries = strict.entries; std::vector<RoadProfileOrigin> origins;
            CHECK_FALSE(complete_road_profiles(entries, origins, *builder, *env->dem, zone, env->limits, o));
        }
    }
}

TEST_CASE("tunnel floors: a long curved, sloped tunnel across a tile edge takes one straight grade, the same from both tiles; the tile graph and a clipped-length rule are the controls", "[layer][roads][completion][tunnel][invariance]") {
    // A 450 m tunnel on a 400 m arc from x = 360 (west tile 455) to (683, 279) (east tile 456), under a 40 m ridge on a
    // 1.5 % slope. Only ~150 m of it lie in the west tile: the threshold applies to the stretch, never to a tile's part.
    const Fn ridge = [](double e, double n) {
        const double x = e - kOriginE, y = n - kOriginN;
        return 100.0 + 0.015 * x - 0.005 * y + 40.0 * std::exp(-((x - 560) * (x - 560) + (y - 90) * (y - 90)) / (2 * 110.0 * 110.0));
    };
    const auto nodes = arc(100, 360, 0, 0.15, 400, 450, 15);
    const auto world = tunnel_world(nodes);
    const auto west = make_env(world, kTile, ridge), east = make_env(world, kTileEast, ridge);
    const auto served = served_road_completion_options();
    REQUIRE(served.tunnel_chord_min_length_m == 200);
    RoadCompletionStats sw, se;
    const auto a = build_with(*west, served, &sw), b = build_with(*east, served, &se);
    const auto tw = index_of(a, 30, 0), te = index_of(b, 30, 0);
    REQUIRE(a.entries[tw].profile); REQUIRE(b.entries[te].profile);
    CHECK(a.origins[tw].provenance == ProfileProvenance::TunnelFloor); CHECK(b.origins[te].provenance == ProfileProvenance::TunnelFloor);
    CHECK(sw.tunnel_chord == 1); CHECK(se.tunnel_chord == 1);
    CHECK(same_outcome(a.entries[tw], a.origins[tw], b.entries[te], b.origins[te]));
    const auto& p = *a.entries[tw].profile;
    const double L = p.reference.length_m;
    CHECK(L == b.entries[te].profile->reference.length_m);
    CHECK(std::fabs(L - 450) < 2.0);
    CHECK(chord_departure(p) < 1e-6);   // one constant grade along the curved reference
    double west_part = 0;               // the tunnel's length inside the west tile (x < 512)
    for (std::size_t i = 1; i < nodes.size(); ++i) if (nodes[i].x < 512) west_part += std::hypot(nodes[i].x - nodes[i - 1].x, nodes[i].y - nodes[i - 1].y);
    CHECK(west_part < 200);
    // The portal heights are the DEM-floor profile's own end samples (the phase-4 anchors).
    const auto floor_west = build_with(*west, phase5_served()); REQUIRE(floor_west.entries[tw].profile);
    const auto& fp = *floor_west.entries[tw].profile;
    CHECK(p.at(0)->height_m == fp.at(0)->height_m); CHECK(p.at(L)->height_m == fp.at(L)->height_m);
    const double grade = (p.at(L)->height_m - p.at(0)->height_m) / L;
    std::printf("straight-grade tunnel across a tile edge: %.1f m (%.1f m in the west tile), grade %.4f, DEM floor departs %.2f m from it, served departs %.2e m\n",
        L, west_part, grade, chord_departure(fp), chord_departure(p));
    CHECK(chord_departure(fp) > 15.0);
    // Control 1: a rule measured on the tile's own part (west: ~150 m < 200 m, so the DEM floor) would disagree with the
    // east tile's straight grade by metres at the same stations.
    {
        const auto d = max_delta(fp, *b.entries[te].profile);
        std::printf("control (clipped-length rule): west DEM floor vs east straight grade, max |dz| %.2f m\n", d.height);
        CHECK(d.height > 15.0);
    }
    // Control 2: hand the tier the TILE graph of the west tile as its context. The ground way past the east portal is
    // not in it, so the stretch's reference (and the profile along it) is not the one the east tile computes.
    {
        const auto strict = build(*west, false);
        auto entries = strict.entries; std::vector<RoadProfileOrigin> origins;
        const auto narrow = RoadReferenceBuilder::make(west->tile.graph); REQUIRE(narrow);
        REQUIRE(complete_road_profiles(entries, origins, *narrow, *west->dem, zone, west->limits, served));
        const bool same = same_outcome(entries[tw], origins[tw], b.entries[te], b.origins[te]);
        std::printf("control (tile graph instead of the context): tunnel profiled %d, same as the east tile %d\n", entries[tw].profile.has_value() ? 1 : 0, same ? 1 : 0);
        CHECK_FALSE(same);
    }
}

TEST_CASE("bridge anchors: a short bridge way next to a long one is supported over the short one; D25's first-edge walk is the control (D47)", "[layer][roads][completion][bridge][D47]") {
    // Ground 10 | bridge 20 (15 m) | bridge 21 (120 m) | ground 12. At node 5, between the two bridges, the walk away from
    // the first incident bridge edge (way 20) has to cross the 120 m span: beyond the 60 m budget. Walking away from
    // way 21 instead crosses 15 m of bridge and reaches 32 m of ground on way 10 within 47 m.
    const auto world = world_of({{1, -330, 0}, {2, -180, 0}, {3, -30, 0}, {5, -15, 0}, {6, 105, 0}, {7, 255, 0}, {8, 405, 0}},
        {{10, {1, 2, 3}}, {20, {3, 5}, kBridge}, {21, {5, 6}, kBridge}, {12, {6, 7, 8}}});
    const auto env = make_env(world, kTile, rolling);
    const auto strict = build(*env, false);
    const auto a = index_of(strict, 20, 0), b = index_of(strict, 21, 0);
    REQUIRE_FALSE(strict.entries[a].profile); REQUIRE_FALSE(strict.entries[b].profile);
    // At phase 4's 60 m walk (D48 later raised it to 200 m, see below).
    RoadCompletionStats stats; const auto lib = build_with(*env, phase4_served(), &stats);
    REQUIRE(lib.entries[a].profile); REQUIRE(lib.entries[b].profile);
    CHECK(lib.origins[a].provenance == ProfileProvenance::BridgeCompleted); CHECK(lib.origins[b].provenance == ProfileProvenance::BridgeCompleted);
    CHECK(std::fabs(end_height(lib, a, true) - end_height(lib, b, false)) <= 1.0 / 256.0);
    CHECK(stats.walk_other_side >= 1);
    const auto walk = road_bridge_anchor(env->context, *env->dem, zone, 5, phase4_served()); CHECK(walk.plane.has_value());
    // Control: D25 (bridge_anchor_reuse off) finds no support at node 5 for either span.
    RoadCompletionStats old_stats; const auto old = build_with(*env, previous_served(), &old_stats);
    CHECK_FALSE(old.entries[a].profile); CHECK_FALSE(old.entries[b].profile);
    RoadCompletionOptions d25 = previous_served();
    CHECK_FALSE(road_bridge_anchor(env->context, *env->dem, zone, 5, d25).plane.has_value());
    // D48: the served 200 m walk crosses the 120 m span as the game does; the first-edge walk now finds support too.
    d25.abutment_path_m = 200; CHECK(road_bridge_anchor(env->context, *env->dem, zone, 5, d25).plane.has_value());
    const auto served = build(*env, true);
    REQUIRE(served.entries[a].profile); REQUIRE(served.entries[b].profile);
    // The game (unbounded walk, anchor map) completes both on what the previous release served.
    auto game = old; g2m_test::game_reference::complete_road_profiles_live(game, *env->dem);
    REQUIRE(game.entries[a].profile); REQUIRE(game.entries[b].profile);
    std::printf("D47/D48 bridge chain: node 5 at %.3f m (60 m walk over the 15 m span), %.3f m (served 200 m walk) vs %.3f m (game, unbounded walk over the 120 m span)\n",
        end_height(lib, b, false), end_height(served, b, false), end_height(game, b, false));
    CHECK(std::fabs(end_height(served, b, false) - end_height(game, b, false)) <= 1.0 / 256.0);
    // The game-only completions of this scene are gone (GAP 2a/2b of the migration plan).
    std::size_t game_only = 0;
    for (std::size_t i = 0; i < game.entries.size(); ++i) if (game.entries[i].profile && !lib.entries[i].profile) ++game_only;
    CHECK(game_only == 0);
}

TEST_CASE("bridge anchors: a ground stretch next to a secant-fallback span takes the span's end grade, as the game's anchor map (D47)", "[layer][roads][completion][bridge][D47]") {
    // A 20 m bridge over a gully between two 10 % slopes falling toward it: the span's cubic between the two approach
    // planes (-10 %, +10 %) is beyond its curvature, so it takes the common secant (level). The ground stretches beside it
    // are declined by the strict pass (the approach slope exceeds the class) and completed; their bridge-side end grade
    // is the span's end grade (0) with D47, the approach plane's (clamped to 10 %) without.
    const auto world = world_of({{1, -310, 0}, {2, -160, 0}, {3, -10, 0}, {4, 10, 0}, {5, 160, 0}, {6, 310, 0}},
        {{10, {1, 2, 3}}, {20, {3, 4}, kBridge}, {12, {4, 5, 6}}});
    const Fn valley = [](double e, double) { const double x = std::fabs(e - kOriginE); return x < 10 ? 95.0 : 100.0 + 0.10 * (x - 10); };
    const auto env = make_env(world, kTile, valley);
    const auto strict = build(*env, false);
    const auto west = index_of(strict, 10, 0), span = index_of(strict, 20, 0), east = index_of(strict, 12, 0);
    for (const auto i : {west, span, east}) REQUIRE_FALSE(strict.entries[i].profile);
    RoadCompletionStats stats; const auto lib = build(*env, true, &stats);
    for (const auto i : {west, span, east}) REQUIRE(lib.entries[i].profile);
    const auto& sp = *lib.entries[span].profile;
    CHECK(std::fabs(sp.vertical.segments.front().grade0) < 1e-12);   // the secant fallback
    const double west_end = lib.entries[west].profile->vertical.segments.back().grade1, east_start = lib.entries[east].profile->vertical.segments.front().grade0;
    CHECK(std::fabs(west_end) < 1e-9); CHECK(std::fabs(east_start) < 1e-9);
    CHECK(stats.bridge_anchor_reused >= 2);
    // Control: without D47 the ground ends at the approach plane's 10 %: a grade step of 0.1 at the bridge ends.
    const auto old = build_with(*env, previous_served()); REQUIRE(old.entries[west].profile);
    const double old_step = std::fabs(old.entries[west].profile->vertical.segments.back().grade1 - old.entries[span].profile->vertical.segments.back().grade1);
    std::printf("D47 secant span: grade step at the bridge end %.4f with D47, %.4f without\n", std::fabs(west_end), old_step);
    CHECK(old_step > 0.05);
    // The game on the strict tile completes the same three stretches, within 1 mm.
    auto game = strict; g2m_test::game_reference::complete_road_profiles_live(game, *env->dem);
    double worst = 0;
    for (const auto i : {west, span, east}) { REQUIRE(game.entries[i].profile); worst = std::max(worst, max_delta(*lib.entries[i].profile, *game.entries[i].profile).height); }
    std::printf("D47 secant span vs the game: max |dz| %.3e m\n", worst);
    CHECK(worst <= 1e-3);
}

TEST_CASE("completion: fixture tiles derive the same bytes on 1 and 4 threads; game-only completions per fixture (roadcmp, D47)", "[layer][roads][completion][fixtures][D47][long]") {
    for (const char* pbf : {"frankfurt_hoechst.osm.pbf", "hoechst_a66.osm.pbf"}) {
        const auto data = g2m_test::read_osm_fixture(std::string(G2M_SOURCE_DIR) + "/testdata/fixtures/osm/" + pbf);
        const auto world = build_road_graph(data, g2m_test::kZone32N); REQUIRE(world);
        std::vector<TileKey> keys;
        { std::set<TileKey> s;
          for (const auto& n : world->nodes) if (const auto k = tile_key_at(g2m_test::kZone32N, 2, static_cast<double>(n.position.x) / 1000.0, static_cast<double>(n.position.y) / 1000.0)) s.insert(*k);
          keys.assign(s.begin(), s.end()); }
        const auto encode = [&](const TileKey& key, const RoadCompletionOptions& o, bool game_after = false) {
            g2m_test::GeomOptions go; go.height_m = Fn(tilted_fixture_dem); go.completion = o;
            if (game_after) go.post = [](RoadGeomTile& t, const RoadDemSampler& dem) { g2m_test::game_reference::complete_road_profiles_live(t, dem); };
            auto g = g2m_test::geometry_in_tile_with(*world, key, go); REQUIRE(g);
            return *g;
        };
        std::vector<std::vector<std::uint8_t>> serial(keys.size()), parallel(keys.size());
        for (std::size_t i = 0; i < keys.size(); ++i) { auto b = encode_road_geom(encode(keys[i], served_road_completion_options())); REQUIRE(b); serial[i] = encode_body(*b).value(); }
        {
            std::vector<std::thread> threads; std::atomic<std::size_t> next{0};
            for (int t = 0; t < 4; ++t) threads.emplace_back([&] {
                for (std::size_t i; (i = next.fetch_add(1)) < keys.size();) {
                    g2m_test::GeomOptions go; go.height_m = Fn(tilted_fixture_dem); go.completion = served_road_completion_options();
                    auto g = g2m_test::geometry_in_tile_with(*world, keys[i], go);
                    if (!g) continue;
                    auto b = encode_road_geom(*g); if (b) { auto e = encode_body(*b); if (e) parallel[i] = std::move(*e); }
                }
            });
            for (auto& t : threads) t.join();
        }
        Fnv1a64 hash; for (const auto& s : serial) { hash.u64(s.size()); hash.bytes(s.data(), s.size()); }
        std::size_t differ = 0; for (std::size_t i = 0; i < keys.size(); ++i) if (serial[i] != parallel[i]) ++differ;
        // roadcmp: entries the game's live code profiles on the previous release's tile that the new tier leaves declined.
        std::size_t game_only = 0, game_tunnels = 0, new_tunnels = 0, game_bridges_old = 0, game_bridges_new = 0;
        for (const auto& key : keys) {
            const auto prev = encode(key, previous_served()), game = encode(key, previous_served(), true), now = encode(key, served_road_completion_options());
            for (std::size_t i = 0; i < game.entries.size(); ++i) {
                const bool tunnel = game.entries[i].profile && game.entries[i].profile->attributes.tunnel;
                const bool bridge = game.entries[i].profile && game.entries[i].profile->attributes.bridge;
                if (!prev.entries[i].profile && game.entries[i].profile) { if (tunnel) ++game_tunnels; if (bridge) ++game_bridges_old; }
                if (now.entries[i].profile && now.origins[i].provenance == ProfileProvenance::TunnelFloor) ++new_tunnels;
                if (game.entries[i].profile && !now.entries[i].profile) {
                    ++game_only; if (bridge) ++game_bridges_new;
                    if (game_only <= 40) std::printf("    game-only: way %lld ref %u (%s) served decline '%s'\n", static_cast<long long>(now.entries[i].way_id), now.entries[i].stretch.start_ref,
                        kind_of(now, now.entries[i]).c_str(), now.entries[i].decline->message.c_str());
                }
            }
        }
        std::printf("[D47] %s: %zu tiles, 1 vs 4 threads %zu differ, bytes fnv %016llx; game completes %zu tunnels and %zu bridges the previous release declined; new tier: %zu tunnel floors; game-only entries left: %zu (%zu bridges)\n",
            pbf, keys.size(), differ, static_cast<unsigned long long>(hash.value()), game_tunnels, game_bridges_old, new_tunnels, game_only, game_bridges_new);
        CHECK(differ == 0);
        CHECK(game_only == 0);  // owner rule: no game-only profile before the game's completion code is deleted
    }
}

TEST_CASE("completion: release cost of the 200 m walk on the fixtures (D48)", "[layer][roads][completion][D48][cost][long]") {
    // The deriver's input set and the tier's outcome per fixture tile, pre-D48 (no walk squares, 60 m) against D48. A
    // measurement: only the outcome direction is checked (the longer walk never loses a completed bridge).
    for (const char* pbf : {"frankfurt_hoechst.osm.pbf", "hoechst_a66.osm.pbf"}) {
        const auto data = g2m_test::read_osm_fixture(std::string(G2M_SOURCE_DIR) + "/testdata/fixtures/osm/" + pbf);
        const auto world = build_road_graph(data, g2m_test::kZone32N); REQUIRE(world);
        std::set<TileKey> keys;
        for (const auto& n : world->nodes) if (const auto k = tile_key_at(g2m_test::kZone32N, 2, static_cast<double>(n.position.x) / 1000.0, static_cast<double>(n.position.y) / 1000.0)) keys.insert(*k);
        struct Mode { const char* name; std::int64_t margin; double path; std::size_t dem = 0, graph = 0, bridges = 0, ground = 0; double ms = 0; };
        Mode modes[] = {{"pre-D48 (60 m, no squares)", 0, 60}, {"D48 (200 m, 205 m squares)", kRoadBridgeWalkMarginMm, 200}};
        for (auto& m : modes) {
            for (const auto& key : keys) {
                const auto in = g2m_test::road_geom_inputs(*world, key, true, m.margin); REQUIRE(in);
                m.dem += in->dem_keys.size();
                const auto graph_deps = road_context_dependencies(in->context, key.zone, &key); REQUIRE(graph_deps);
                m.graph += graph_deps->size();
                std::vector<HeightTile> heights(in->dem_keys.size()); std::vector<const HeightTile*> views;
                for (std::size_t i = 0; i < heights.size(); ++i) { heights[i].key = in->dem_keys[i]; g2m_test::fill_dem(heights[i], tilted_fixture_dem); views.push_back(&heights[i]); }
                const auto dem = RoadDemSampler::make(views); REQUIRE(dem);
                RoadProfileLimits limits; limits.reference.allow_curvature_exceptions = true;
                auto o = served_road_completion_options(); o.abutment_path_m = m.path;
                RoadCompletionStats stats;
                const auto t0 = std::chrono::steady_clock::now();
                const auto g = build_road_geom_tile(in->tile, *dem, limits, &in->context, builtin_road_rules_de(), o, &stats); REQUIRE(g);
                m.ms += std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
                m.bridges += stats.bridge_completed; m.ground += stats.ground_completed;
            }
            std::printf("[D48 cost] %s %s: %zu tiles, DEM inputs %zu (%.1f MiB raw), context graph tiles %zu, bridges completed %zu, ground completed %zu, build %.0f ms\n",
                pbf, m.name, keys.size(), m.dem, static_cast<double>(m.dem) * 65536.0 * 4.0 / 1048576.0, m.graph, m.bridges, m.ground, m.ms);
        }
        CHECK(modes[1].bridges >= modes[0].bridges);
        CHECK(modes[1].dem >= modes[0].dem);
    }
}

TEST_CASE("tunnel floors: the straight-grade ruling on the fixture tunnels (measurement, owner ruling 2026-10-15)", "[layer][roads][completion][tunnel][fixtures][long]") {
    // Every tunnel stretch of both fixtures on the tilted bumpy DEM, the release before the ruling (DEM floor for every
    // length) against the served tier. Also counts tunnel chains: tunnel stretches joined end to end at a node no other
    // road meets, whose own stretches are each judged by their own length.
    for (const char* pbf : {"frankfurt_hoechst.osm.pbf", "hoechst_a66.osm.pbf"}) {
        const auto data = g2m_test::read_osm_fixture(std::string(G2M_SOURCE_DIR) + "/testdata/fixtures/osm/" + pbf);
        const auto world = build_road_graph(data, g2m_test::kZone32N); REQUIRE(world);
        std::set<TileKey> keys;
        for (const auto& n : world->nodes) if (const auto k = tile_key_at(g2m_test::kZone32N, 2, static_cast<double>(n.position.x) / 1000.0, static_cast<double>(n.position.y) / 1000.0)) keys.insert(*k);
        const auto encode = [&](const TileKey& key, const RoadCompletionOptions& o) {
            g2m_test::GeomOptions go; go.height_m = Fn(tilted_fixture_dem); go.completion = o;
            auto g = g2m_test::geometry_in_tile_with(*world, key, go); REQUIRE(g); return std::move(*g);
        };
        struct Seen { double length = 0, delta = 0, grade = 0; bool straight = false; std::int64_t a = 0, b = 0; };
        std::map<std::pair<std::int64_t, std::uint32_t>, Seen> tunnels;
        std::size_t inconsistent = 0;
        for (const auto& key : keys) {
            const auto before = encode(key, phase5_served()), now = encode(key, served_road_completion_options());
            REQUIRE(before.entries.size() == now.entries.size());
            for (std::size_t i = 0; i < now.entries.size(); ++i) {
                const auto& e = now.entries[i];
                if (kind_of(now, e) != "tunnel") continue;
                // Everything that is not a tunnel profile is unchanged by the ruling.
                Seen s;
                const auto* w = &*std::lower_bound(world->ways.begin(), world->ways.end(), e.way_id, [](const auto& x, auto id) { return x.osm_id < id; });
                s.a = w->node_ids[e.stretch.start_ref]; s.b = w->node_ids[e.stretch.end_ref];
                if (e.profile) {
                    REQUIRE(before.entries[i].profile);
                    s.length = e.profile->reference.length_m;
                    s.straight = chord_departure(*e.profile) < 1e-6 && s.length >= 200;
                    s.delta = max_delta(*before.entries[i].profile, *e.profile).height;
                    s.grade = (e.profile->at(s.length)->height_m - e.profile->at(0)->height_m) / s.length;
                    if (s.length < 200) CHECK(same_vertical(before.entries[i].profile->vertical, e.profile->vertical));
                }
                const auto [it, fresh] = tunnels.emplace(std::make_pair(e.way_id, e.stretch.start_ref), s);
                if (!fresh && (it->second.length != s.length || it->second.delta != s.delta)) ++inconsistent;
            }
            for (std::size_t i = 0; i < now.entries.size(); ++i) if (kind_of(now, now.entries[i]) != "tunnel")
                CHECK(same_outcome(before.entries[i], before.origins[i], now.entries[i], now.origins[i]));
        }
        CHECK(inconsistent == 0);
        // Tunnel chains through nodes that join exactly two tunnel stretches and nothing else.
        std::map<std::int64_t, int> degree; for (const auto& e : world->edges) { ++degree[e.from_node]; ++degree[e.to_node]; }
        std::map<std::int64_t, std::vector<std::pair<std::int64_t, std::uint32_t>>> at;
        for (const auto& [id, s] : tunnels) { at[s.a].push_back(id); at[s.b].push_back(id); }
        std::map<std::pair<std::int64_t, std::uint32_t>, std::pair<std::int64_t, std::uint32_t>> parent;
        for (const auto& [id, s] : tunnels) parent[id] = id;
        const std::function<std::pair<std::int64_t, std::uint32_t>(std::pair<std::int64_t, std::uint32_t>)> root = [&](auto x) { return parent[x] == x ? x : parent[x] = root(parent[x]); };
        for (const auto& [node, list] : at) if (list.size() == 2 && degree[node] == 2) parent[root(list[0])] = root(list[1]);
        std::map<std::pair<std::int64_t, std::uint32_t>, std::pair<double, int>> chains;
        for (const auto& [id, s] : tunnels) { auto& c = chains[root(id)]; c.first += s.length; ++c.second; }
        std::size_t profiled = 0, straight = 0, multi = 0, missed = 0; double max_delta_m = 0, max_len = 0, max_grade = 0;
        for (const auto& [id, s] : tunnels) {
            if (s.length > 0) ++profiled;
            if (s.straight) { ++straight; max_grade = std::max(max_grade, std::fabs(s.grade)); }
            max_delta_m = std::max(max_delta_m, s.delta); max_len = std::max(max_len, s.length);
        }
        for (const auto& [r, c] : chains) {
            if (c.second < 2) continue;
            ++multi;
            bool any_long = false;
            for (const auto& [id, s] : tunnels) if (root(id) == r && s.length >= 200) any_long = true;
            if (c.first >= 200 && !any_long) ++missed;
        }
        std::printf("[tunnel straight grade] %s: %zu tunnel stretches (%zu profiled, longest %.1f m), %zu now on one straight grade (max |grade| %.4f), max |dz| vs the DEM floor %.2f m; %zu chains of several tunnel stretches, %zu of them >= 200 m with no stretch >= 200 m\n",
            pbf, tunnels.size(), profiled, max_len, straight, max_grade, max_delta_m, multi, missed);
        for (const auto& [id, s] : tunnels) if (s.length >= 100)
            std::printf("    way %lld ref %u: %.1f m, %s, max |dz| %.2f m\n", static_cast<long long>(id.first), id.second, s.length, s.straight ? "straight grade" : "DEM floor", s.delta);
    }
}
