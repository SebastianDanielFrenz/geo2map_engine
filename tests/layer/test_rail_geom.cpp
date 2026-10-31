// tests/layer/test_rail_geom.cpp - railways R1/R2: the g2m.rail.geom layer (rail_geom_tile.h, rail_rules.h). Attribute
// resolution, the rules file, the tile codec and validator, the height profile (a bridge from the line itself, never
// from the bare-earth DEM under the span; chains across split ways; the grade-limited ground profile with pinned ends),
// piece ownership and the DEM dependency set. Every property has a recorded sabotage: the naive variant is computed in
// the test and must FAIL the same check (its numbers are printed).
#include "synthetic.h"

#include "g2m/core/hash.h"
#include "g2m/layer/rail_geom_tile.h"
#include "g2m/layer/src_osm.h"
#include "g2m/layer/terrain_layers.h"

#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <functional>
#include <map>
#include <memory>
#include <set>
#include <string>
#include <vector>

using namespace g2m;

namespace {
const geo::UtmZone kZone = g2m_test::kZone32N;
const TileKey kTile{kZone, 2, 456, 5420};
constexpr double kE0 = 456.0 * 1024.0 + 512.0, kN0 = 5420.0 * 1024.0 + 512.0;   // the tile centre, metres

using Tags = osm::OsmBuilder::TagList;

// A scene of railway ways laid out in metres relative to the tile centre. Nodes are shared by position.
class Scene {
public:
    // A polyline through (x, y) points; `tags` are the way tags. Returns the way id.
    std::int64_t way(std::int64_t id, const std::vector<std::pair<double, double>>& points, Tags tags) {
        std::vector<std::int64_t> refs;
        for (const auto& p : points) refs.push_back(node(p.first, p.second));
        builder_.add_way(id, refs, std::move(tags));
        return id;
    }
    // A straight east-west way at northing `y` from x0 to x1 with a node every `step` metres.
    std::int64_t line(std::int64_t id, double x0, double x1, double step, double y, Tags tags) {
        std::vector<std::pair<double, double>> points;
        for (double x = x0; x <= x1 + 1e-9; x += step) points.emplace_back(x, y);
        return way(id, points, std::move(tags));
    }
    osm::OsmData data() const { auto d = builder_.build(); REQUIRE(d); return std::move(*d); }
private:
    std::int64_t node(double x, double y) {
        const auto key = std::make_pair(std::llround(x * 1000.0), std::llround(y * 1000.0));
        const auto it = nodes_.find(key);
        if (it != nodes_.end()) return it->second;
        const geo::Utm utm(geo::kGRS80);
        const geo::TmReverse g = utm.reverse(kZone, kE0 + x, kN0 + y);
        const std::int64_t id = 1000 + static_cast<std::int64_t>(nodes_.size());
        builder_.add_node(id, g2m_test::to_e7(g.lat), g2m_test::to_e7(g.lon));
        nodes_[key] = id;
        return id;
    }
    osm::OsmBuilder builder_;
    std::map<std::pair<std::int64_t, std::int64_t>, std::int64_t> nodes_;
};

RailWorld world_of(const osm::OsmData& data, const RailRules& rules = builtin_rail_rules_de()) {
    const osm::OsmData* tiles[] = {&data};
    auto world = collect_rail_world(tiles, kZone, rules);
    REQUIRE(world);
    return std::move(*world);
}

// Level-0 elev.base tiles with `height_m(easting, northing)` at the cell centres (1/256 m).
struct Dem {
    std::vector<std::unique_ptr<HeightTile>> tiles;
    void add(const TileKey& key, const std::function<double(double, double)>& height_m) {
        auto t = std::make_unique<HeightTile>(); t->key = key;
        const double e0 = static_cast<double>(key.min_easting()), n0 = static_cast<double>(key.min_northing());
        for (std::size_t j = 0; j < 256; ++j) for (std::size_t i = 0; i < 256; ++i)
            t->h[j * 256 + i] = static_cast<std::int32_t>(std::llround(height_m(e0 + (static_cast<double>(i) + 0.5), n0 + (static_cast<double>(j) + 0.5)) * 256.0));
        tiles.push_back(std::move(t));
    }
    // Every tile of the rectangle [x0, x1] x [y0, y1] metres relative to the tile centre.
    void add_rect(double x0, double x1, double y0, double y1, const std::function<double(double, double)>& height_m) {
        const auto floor256 = [](double v) { return static_cast<std::int32_t>(std::floor(v / 256.0)); };
        for (auto ty = floor256(kN0 + y0); ty <= floor256(kN0 + y1); ++ty)
            for (auto tx = floor256(kE0 + x0); tx <= floor256(kE0 + x1); ++tx) add(TileKey{kZone, 0, tx, ty}, height_m);
    }
    RoadDemSampler sampler() const {
        std::vector<const HeightTile*> views; for (const auto& t : tiles) views.push_back(t.get());
        auto s = RoadDemSampler::make(views); REQUIRE(s); return std::move(*s);
    }
};

// 3 % east-rising terrain with a 10 m deep, 40 m wide cutting along the north-south line x = 0 (a road under the bridge).
double cutting_dem(double e, double /*n*/) {
    const double x = e - kE0;
    return 100.0 + 0.03 * x - (std::fabs(x) < 20.0 ? 10.0 : 0.0);
}
double ramp_dem(double e, double /*n*/) { return 100.0 + 0.03 * (e - kE0); }

const RailPiece* find_piece(const RailGeomTile& tile, std::int64_t way, std::uint32_t index = 0) {
    for (const auto& p : tile.pieces) if (p.way_id == way && p.piece == index) return &p;
    return nullptr;
}
// z (1/256 m) of the vertex of `p` nearest to x metres relative to the tile centre.
std::int32_t z_at(const RailPiece& p, double x) {
    std::size_t best = 0; double d = 1e18;
    for (std::size_t i = 0; i < p.xy.size(); ++i) {
        const double v = std::fabs(static_cast<double>(p.xy[i].x) / 1000.0 - (kE0 + x));
        if (v < d) { d = v; best = i; }
    }
    return p.z[best];
}
double max_grade(const RailPiece& p) {
    double worst = 0;
    for (std::size_t i = 1; i < p.xy.size(); ++i) {
        const double dx = static_cast<double>(p.xy[i].x - p.xy[i - 1].x) / 1000.0, dy = static_cast<double>(p.xy[i].y - p.xy[i - 1].y) / 1000.0;
        const double ds = std::sqrt(dx * dx + dy * dy);
        worst = std::max(worst, std::fabs(static_cast<double>(p.z[i] - p.z[i - 1]) / 256.0) / ds);
    }
    return worst;
}
} // namespace

TEST_CASE("rail attributes: classes, structure flags, construction, layer, tracks, gauge", "[layer][rail]") {
    const auto& rules = builtin_rail_rules_de();
    osm::OsmBuilder b;
    b.add_node(1, 500000000, 80000000); b.add_node(2, 500010000, 80010000);
    const auto add = [&](std::int64_t id, Tags tags) { b.add_way(id, {1, 2}, std::move(tags)); };
    add(10, {{"railway", "rail"}});
    add(11, {{"bridge", "yes"}, {"layer", "1"}, {"railway", "rail"}});
    add(12, {{"bridge", "no"}, {"railway", "rail"}});
    add(13, {{"railway", "rail"}, {"tunnel", "culvert"}});
    add(14, {{"bridge", "viaduct"}, {"railway", "rail"}, {"tunnel", "yes"}});
    add(15, {{"construction", "rail"}, {"bridge", "yes"}, {"railway", "construction"}});
    add(16, {{"railway", "tram"}});
    add(17, {{"railway", "abandoned"}});
    add(18, {{"area", "yes"}, {"railway", "rail"}});
    add(19, {{"gauge", "1000"}, {"railway", "light_rail"}, {"tracks", "2"}});
    add(20, {{"gauge", "1435;1000"}, {"railway", "rail"}, {"tracks", "99"}});
    add(21, {{"electrified", "contact_line"}, {"embankment", "yes"}, {"layer", "-1"}, {"railway", "subway"}, {"service", "siding"}, {"usage", "branch"}});
    add(22, {{"construction", "tram"}, {"railway", "construction"}});
    add(23, {{"railway", "construction"}});
    add(24, {{"cutting", "yes"}, {"layer", "9"}, {"railway", "rail"}});
    const auto data = [&] { auto d = b.build(); REQUIRE(d); return std::move(*d); }();
    const auto attrs = [&](std::int64_t id) { return resolve_rail_attrs(data, *data.find_way(id), rules); };
    REQUIRE(attrs(10)); CHECK(attrs(10)->cls == 0); CHECK(attrs(10)->flags == 0); CHECK(attrs(10)->tracks == 1); CHECK(attrs(10)->gauge_mm == 1435);
    REQUIRE(attrs(11)); CHECK(attrs(11)->flags == kRailBridge); CHECK(attrs(11)->layer == 1);
    REQUIRE(attrs(12)); CHECK(attrs(12)->flags == 0);                      // bridge=no is not a bridge
    REQUIRE(attrs(13)); CHECK(attrs(13)->flags == 0);                      // tunnel=culvert is not a tunnel
    REQUIRE(attrs(14)); CHECK(attrs(14)->flags == kRailTunnel);            // tunnel wins over bridge
    REQUIRE(attrs(15)); CHECK(attrs(15)->condition == RailCondition::Construction); CHECK(attrs(15)->flags == kRailBridge);
    CHECK_FALSE(attrs(16));                                                // tram is off in v1
    CHECK_FALSE(attrs(17));
    CHECK_FALSE(attrs(18));
    REQUIRE(attrs(19)); CHECK(attrs(19)->cls == 1); CHECK(attrs(19)->gauge_mm == 1000); CHECK(attrs(19)->tracks == 2);
    REQUIRE(attrs(20)); CHECK(attrs(20)->gauge_mm == 1435); CHECK(attrs(20)->tracks == 1);   // unusable tags fall back to the class defaults
    REQUIRE(attrs(21)); CHECK(attrs(21)->cls == 2); CHECK(attrs(21)->flags == (kRailElectrified | kRailEmbankment)); CHECK(attrs(21)->layer == -1);
    CHECK(attrs(21)->service == RailService::Siding); CHECK(attrs(21)->usage == RailUsage::Branch);
    CHECK_FALSE(attrs(22));                                                // a tram under construction is still tram
    CHECK_FALSE(attrs(23));                                                // construction without a class
    REQUIRE(attrs(24)); CHECK(attrs(24)->flags == kRailCutting); CHECK(attrs(24)->layer == 5);   // layer clamped

    // Sabotage: the same ways with tram enabled DO resolve - the rules, not the code, keep tram out of v1.
    RailRules tram = rules;
    for (auto& c : tram.classes) if (c.railway == "tram") c.enabled = true;
    CHECK(resolve_rail_attrs(data, *data.find_way(16), tram));
    CHECK(resolve_rail_attrs(data, *data.find_way(22), tram));
    RailRules no_construction = rules; no_construction.construction_enabled = false;
    CHECK_FALSE(resolve_rail_attrs(data, *data.find_way(15), no_construction));
}

TEST_CASE("rail rules file: shipped values, strict loader", "[layer][rail]") {
    const auto& rules = builtin_rail_rules_de();
    REQUIRE(rules.classes.size() == 4);
    CHECK(rules.find_class("rail")->id == 0);
    CHECK(rules.find_class("tram")->id == 3);
    CHECK_FALSE(rules.find_class("tram")->enabled);
    CHECK(rules.find_class_id(2)->railway == "subway");
    CHECK(rules.find_class("rail")->max_grade_permille == 40);
    CHECK(rules.piece_max_length_m == 512);
    CHECK(rules.content_sha256 == Sha256::of(builtin_rail_rules_de_bytes()));

    const auto bytes = builtin_rail_rules_de_bytes();
    std::string text(reinterpret_cast<const char*>(bytes.data()), bytes.size());
    const auto load = [](const std::string& s) {
        return load_rail_rules(std::span<const std::byte>(reinterpret_cast<const std::byte*>(s.data()), s.size()));
    };
    CHECK(load(text));
    // Sabotages: an unknown key, a wrong format string and a truncated file are rejected.
    auto extra = text; extra.replace(extra.find('{'), 1, "{\"surprise\": 1,");
    CHECK_FALSE(load(extra));
    auto format = text; format.replace(format.find("rail_rules/1"), 12, "rail_rules/2");
    CHECK_FALSE(load(format));
    CHECK_FALSE(load(text.substr(0, text.size() / 2)));
    // A different number changes the content hash (and with it the layer's config hash).
    auto changed = text; const auto at = changed.find("512"); REQUIRE(at != std::string::npos); changed.replace(at, 3, "256");
    const auto other = load(changed); REQUIRE(other);
    CHECK(other->content_sha256 != rules.content_sha256);
    CHECK(other->piece_max_length_m == 256);
}

TEST_CASE("rail.geom codec: round trip, validator, corrupted bodies", "[layer][rail][codec]") {
    RailGeomTile tile; tile.key = kTile;
    const auto mm = [](double x, double y) { return geom::PointMm{std::llround((kE0 + x) * 1000.0), std::llround((kN0 + y) * 1000.0)}; };
    RailPiece a; a.way_id = -5; a.piece = 0; a.cls = 0; a.flags = kRailBridge | kRailElectrified; a.layer = 1; a.tracks = 2;
    a.xy = {mm(0, 0), mm(40, 3), mm(80, 10)}; a.z = {25600, 25650, 25520};
    RailPiece b; b.way_id = 7; b.piece = 0; b.cls = 2; b.condition = RailCondition::Construction; b.flags = kRailTunnel; b.layer = -2;
    b.service = RailService::Yard; b.usage = RailUsage::Industrial; b.gauge_mm = 1000; b.xy = {mm(-10, 5), mm(20, 5)}; b.z = {-3000, -3100};
    RailPiece c = b; c.piece = 1; c.flags = kRailDeclined; c.z = {0, 0};
    tile.pieces = {a, b, c};
    REQUIRE(validate_rail_geom_tile(tile));
    const auto body = encode_rail_geom(tile); REQUIRE(body);
    const auto back = decode_rail_geom(*body, kTile); REQUIRE(back);
    CHECK(*back == tile);
    CHECK(body->sections.size() == 6);
    const auto again = encode_rail_geom(*back); REQUIRE(again);
    CHECK(encode_body(*again).value() == encode_body(*body).value());   // canonical bytes

    // Validator sabotages: each must be rejected, each on its own.
    const auto rejects = [&](const std::function<void(RailGeomTile&)>& mutate) {
        auto t = tile; mutate(t); return !validate_rail_geom_tile(t) && !encode_rail_geom(t);
    };
    CHECK(rejects([](RailGeomTile& t) { std::swap(t.pieces[0], t.pieces[1]); }));            // not ascending
    CHECK(rejects([](RailGeomTile& t) { t.pieces[1].piece = 0; t.pieces[2].piece = 0; }));   // duplicate (way, piece)
    CHECK(rejects([](RailGeomTile& t) { t.pieces[0].flags = kRailBridge | kRailTunnel; }));
    CHECK(rejects([](RailGeomTile& t) { t.pieces[0].flags |= 64; }));
    CHECK(rejects([](RailGeomTile& t) { t.pieces[0].cls = 4; }));
    CHECK(rejects([](RailGeomTile& t) { t.pieces[0].layer = 6; }));
    CHECK(rejects([](RailGeomTile& t) { t.pieces[0].tracks = 0; }));
    CHECK(rejects([](RailGeomTile& t) { t.pieces[0].gauge_mm = 3001; }));
    CHECK(rejects([](RailGeomTile& t) { t.pieces[0].z.pop_back(); }));
    CHECK(rejects([](RailGeomTile& t) { t.pieces[0].xy.resize(1); t.pieces[0].z.resize(1); }));
    CHECK(rejects([](RailGeomTile& t) { t.pieces[2].z[1] = 5; }));                           // declined with a height
    CHECK(rejects([&](RailGeomTile& t) { t.pieces[0].xy[0] = mm(2000, 0); }));               // first vertex outside the tile
    CHECK(rejects([](RailGeomTile& t) { t.pieces[0].z[0] = 1 << 27; }));
    CHECK(rejects([](RailGeomTile& t) { t.key.level = 1; }));
    // A later vertex outside the tile is fine (a piece reaches past its tile).
    { auto t = tile; t.pieces[0].xy[2] = mm(2000, 0); CHECK(validate_rail_geom_tile(t)); }

    // Decoder: truncations and byte flips never crash and never produce an invalid tile.
    std::size_t flips = 0, accepted = 0;
    for (std::size_t s = 0; s < body->sections.size(); ++s) {
        for (std::size_t i = 0; i < body->sections[s].data.size(); ++i) for (const std::uint8_t mask : {0x01, 0x80, 0xff}) {
            auto bad = *body; bad.sections[s].data[i] ^= mask; ++flips;
            const auto r = decode_rail_geom(bad, kTile);
            if (r) { ++accepted; CHECK(validate_rail_geom_tile(*r)); }
        }
        auto cut = *body; if (!cut.sections[s].data.empty()) { cut.sections[s].data.pop_back(); CHECK_FALSE(decode_rail_geom(cut, kTile)); }
    }
    std::printf("rail.geom decode: %zu byte flips, %zu still decode to a valid tile\n", flips, accepted);
    auto wrong = *body; wrong.encoding_version = 2; CHECK_FALSE(decode_rail_geom(wrong, kTile));
    auto trailing = *body; trailing.sections[2].data.push_back(0); CHECK_FALSE(decode_rail_geom(trailing, kTile));
    CHECK_FALSE(decode_rail_geom(*body, TileKey{kZone, 2, 400, 5420}));    // owner check against another key
}

TEST_CASE("rail bridge profile comes from the line, not the DEM under the span", "[layer][rail][profile]") {
    Scene scene;
    scene.line(10, -400, -100, 50, 0, {{"railway", "rail"}});
    scene.line(11, -100, 100, 50, 0, {{"bridge", "yes"}, {"layer", "1"}, {"railway", "rail"}});
    scene.line(12, 100, 400, 50, 0, {{"railway", "rail"}});
    const auto data = scene.data();
    const auto world = world_of(data);
    Dem dem; dem.add_rect(-420, 420, -20, 20, cutting_dem);
    const auto tile = build_rail_geom_tile(world, kTile, builtin_rail_rules_de(), dem.sampler()); REQUIRE(tile);
    const auto* bridge = find_piece(*tile, 11); REQUIRE(bridge);
    REQUIRE((bridge->flags & kRailBridge));
    CHECK_FALSE((bridge->flags & kRailDeclined));
    // Linear between the ground at x = -100 (97 m) and x = +100 (103 m): 100 m at the cutting.
    CHECK(std::abs(z_at(*bridge, -100) - 97 * 256) <= 2);
    CHECK(std::abs(z_at(*bridge, 100) - 103 * 256) <= 2);
    CHECK(std::abs(z_at(*bridge, 0) - 100 * 256) <= 2);
    for (std::size_t i = 0; i < bridge->xy.size(); ++i) {
        const double x = static_cast<double>(bridge->xy[i].x) / 1000.0 - kE0;
        CHECK(std::abs(bridge->z[i] / 256.0 - (100.0 + 0.03 * x)) < 0.02);
    }
    // The bare-earth DEM at the span centre is 10 m lower: that is what the neighbouring road's cutting looks like.
    const auto s = dem.sampler();
    const auto under = s.sample(kZone, {std::llround(kE0 * 1000.0), std::llround(kN0 * 1000.0)}); REQUIRE(under);
    CHECK(*under < 91 * 256);
    // Sabotage: profiling the bridge like ground (the DEM under every vertex) dips 10 m into the cutting - the check above fails.
    double worst = 0;
    for (std::size_t i = 0; i < bridge->xy.size(); ++i) {
        const auto v = s.sample(kZone, bridge->xy[i]); REQUIRE(v);
        const double x = static_cast<double>(bridge->xy[i].x) / 1000.0 - kE0;
        worst = std::max(worst, std::fabs(*v / 256.0 - (100.0 + 0.03 * x)));
    }
    std::printf("rail bridge sabotage (DEM under the span): worst deviation from the line %.2f m\n", worst);
    CHECK(worst > 5.0);
    // The flat deck meets the ground pieces at the abutments (continuity at the shared node).
    const auto* west = find_piece(*tile, 10); const auto* east = find_piece(*tile, 12); REQUIRE(west); REQUIRE(east);
    CHECK(std::abs(z_at(*west, -100) - z_at(*bridge, -100)) <= 2);
    CHECK(std::abs(z_at(*east, 100) - z_at(*bridge, 100)) <= 2);
}

TEST_CASE("rail chain: a bridge split into several ways keeps one profile", "[layer][rail][profile]") {
    const auto build = [&](bool split, const RailRules& rules) {
        Scene scene;
        scene.line(10, -400, -100, 50, 0, {{"railway", "rail"}});
        if (split) {
            scene.line(11, -100, 0, 50, 0, {{"bridge", "yes"}, {"railway", "rail"}});
            scene.line(13, 0, 100, 50, 0, {{"bridge", "yes"}, {"railway", "rail"}});
        } else scene.line(11, -100, 100, 50, 0, {{"bridge", "yes"}, {"railway", "rail"}});
        scene.line(12, 100, 400, 50, 0, {{"railway", "rail"}});
        const auto data = scene.data();
        const auto world = world_of(data, rules);
        Dem dem; dem.add_rect(-420, 420, -20, 20, cutting_dem);
        auto tile = build_rail_geom_tile(world, kTile, rules, dem.sampler()); REQUIRE(tile);
        return std::move(*tile);
    };
    const auto whole = build(false, builtin_rail_rules_de());
    const auto split = build(true, builtin_rail_rules_de());
    const auto* a = find_piece(split, 11); const auto* b = find_piece(split, 13); const auto* w = find_piece(whole, 11);
    REQUIRE(a); REQUIRE(b); REQUIRE(w);
    for (const auto* half : {a, b}) for (std::size_t i = 0; i < half->xy.size(); ++i) {
        const auto x = static_cast<double>(half->xy[i].x) / 1000.0 - kE0;
        CHECK(std::abs(half->z[i] / 256.0 - (100.0 + 0.03 * x)) < 0.02);
    }
    CHECK(std::abs(a->z.back() - b->z.front()) <= 1);                 // the two halves meet at the shared node
    CHECK(std::abs(z_at(*a, 0) - z_at(*w, 0)) <= 1);                  // and equal the unsplit bridge
    // Sabotage: no chain walk (every way anchored at its own end nodes) puts the shared node onto the cutting floor.
    auto rules = builtin_rail_rules_de(); rules.chain_max_ways = 0;
    const auto naive = build(true, rules);
    const auto* na = find_piece(naive, 11); REQUIRE(na);
    const double drop = (z_at(*a, 0) - na->z.back()) / 256.0;
    std::printf("rail chain sabotage (no walk): shared node %.2f m below the chain profile\n", drop);
    CHECK(drop > 5.0);
}

TEST_CASE("rail ground profile: pinned ends, grade limit, resampling, cut", "[layer][rail][profile]") {
    // Pure function first: a 2 m ramp over 20 m, 5 m smoothing off. The limiter must bend it to 40 permille.
    std::vector<double> s; std::vector<std::int32_t> raw;
    for (int i = 0; i <= 100; ++i) { s.push_back(i * 5.0); raw.push_back(i < 50 ? 0 : (i < 54 ? (i - 49) * 128 : 512)); }   // 0 -> 2 m in 20 m
    const auto limited = rail_ground_profile(s, raw, 0, 40);
    REQUIRE(limited.size() == raw.size());
    CHECK(limited.front() == raw.front());
    CHECK(limited.back() == raw.back());
    double worst = 0;
    for (std::size_t i = 1; i < limited.size(); ++i) worst = std::max(worst, std::fabs((limited[i] - limited[i - 1]) / 256.0) / (s[i] - s[i - 1]));
    CHECK(worst <= 0.0410);   // 40 permille plus the 1/256 m rounding over 5 m
    const auto free = rail_ground_profile(s, raw, 0, 1000);
    double free_worst = 0;
    for (std::size_t i = 1; i < free.size(); ++i) free_worst = std::max(free_worst, std::fabs((free[i] - free[i - 1]) / 256.0) / (s[i] - s[i - 1]));
    std::printf("rail grade sabotage (limit 1000 permille): %.1f permille vs %.1f limited\n", free_worst * 1000, worst * 1000);
    CHECK(free_worst > 0.08);
    // A constant profile is a fixed point and the ends never move even with heavy smoothing.
    std::vector<std::int32_t> flat(s.size(), 777);
    CHECK(rail_ground_profile(s, flat, 40, 40) == flat);
    const auto smooth = rail_ground_profile(s, raw, 40, 1000);
    CHECK(smooth.front() == raw.front()); CHECK(smooth.back() == raw.back());
    CHECK(smooth[50] > raw[50]); CHECK(smooth[50] < raw[54]);      // the step is rounded

    // In the tile: a long ground way over terrain with a 2 m step at x = -250 is cut into pieces that meet exactly.
    Scene scene;
    scene.line(10, -500, 500, 100, 0, {{"railway", "rail"}});     // 10 vertices 100 m apart, 1000 m
    const auto data = scene.data();
    const auto world = world_of(data);
    const auto step_dem = [](double e, double) {
        const double x = e - kE0;
        const double t = std::clamp((x + 260.0) / 20.0, 0.0, 1.0);
        return 100.0 + 2.0 * t * t * (3 - 2 * t);
    };
    Dem dem; dem.add_rect(-520, 520, -20, 20, step_dem);
    const auto tile = build_rail_geom_tile(world, kTile, builtin_rail_rules_de(), dem.sampler()); REQUIRE(tile);
    REQUIRE(tile->pieces.size() == 2);                              // cut after >= 512 m: vertices 0..6 and 6..10
    const auto& p0 = tile->pieces[0]; const auto& p1 = tile->pieces[1];
    CHECK(p0.piece == 0); CHECK(p1.piece == 1);
    CHECK(p0.xy.back() == p1.xy.front());                           // pieces share the cut vertex
    CHECK(p0.z.back() == p1.z.front());                             // and its height exactly (raw DEM pins)
    CHECK(max_grade(p0) <= 0.0412); CHECK(max_grade(p1) <= 0.0412);
    for (const auto* p : {&p0, &p1}) for (std::size_t i = 1; i < p->xy.size(); ++i) {
        const double dx = static_cast<double>(p->xy[i].x - p->xy[i - 1].x) / 1000.0;
        CHECK(dx <= 20.0 + 1e-6);                                   // ground spacing
    }
    // Only the piece whose first vertex lies in the tile is owned: the neighbour east of the tile owns nothing of it.
    const TileKey east{kZone, 2, 457, 5420};
    CHECK(build_rail_geom_tile(world, east, builtin_rail_rules_de(), dem.sampler()).value().pieces.empty());
}

TEST_CASE("rail piece ownership: every piece lives in exactly one tile", "[layer][rail][owner]") {
    // A way that crosses the tile edge x = +512 m: owned by the tile of its first vertex only.
    Scene scene;
    scene.line(10, 400, 700, 50, 0, {{"railway", "rail"}});     // first vertex in tile 456, last in tile 457
    scene.line(11, 600, 900, 50, 100, {{"railway", "rail"}});   // wholly in tile 457
    scene.line(12, 300, 520, 20, -50, {{"railway", "rail"}});   // first vertex in 456, last in 457
    const auto data = scene.data();
    const auto world = world_of(data);
    Dem dem; dem.add_rect(-20, 920, -80, 120, ramp_dem);
    const TileKey east{kZone, 2, 457, 5420};
    const auto t0 = build_rail_geom_tile(world, kTile, builtin_rail_rules_de(), dem.sampler()); REQUIRE(t0);
    const auto t1 = build_rail_geom_tile(world, east, builtin_rail_rules_de(), dem.sampler()); REQUIRE(t1);
    CHECK(find_piece(*t0, 10)); CHECK(find_piece(*t0, 12)); CHECK_FALSE(find_piece(*t0, 11));
    CHECK(find_piece(*t1, 11)); CHECK_FALSE(find_piece(*t1, 10)); CHECK_FALSE(find_piece(*t1, 12));
    CHECK(t0->pieces.size() + t1->pieces.size() == 3);
    // The piece reaches past its tile (last vertex east of the edge) - the consumer needs the tile ring.
    const auto* crossing = find_piece(*t0, 10); REQUIRE(crossing);
    CHECK(crossing->xy.back().x / 1000.0 > kE0 + 512.0);
    // Sabotage: "every tile the piece touches" ownership would list way 10 and 12 twice.
    std::size_t touched = 0;
    for (const auto& [id, way] : world.ways) {
        std::set<std::int64_t> tiles;
        for (const auto& p : way.pts) tiles.insert(floor_div(p.x, 1024000));
        touched += tiles.size();
    }
    std::printf("rail ownership sabotage: %zu tile memberships for 3 ways (owned: %zu)\n", touched, t0->pieces.size() + t1->pieces.size());
    CHECK(touched > 3);
    // The tile does not depend on ways that own nothing in it (the 3 x 3 window may carry more of them).
    Scene other;
    other.line(10, 400, 700, 50, 0, {{"railway", "rail"}});
    other.line(11, 600, 900, 50, 100, {{"railway", "rail"}});
    other.line(12, 300, 520, 20, -50, {{"railway", "rail"}});
    other.line(99, -900, -700, 50, 300, {{"railway", "rail"}, {"bridge", "yes"}});
    const auto data2 = other.data();
    const auto t0b = build_rail_geom_tile(world_of(data2), kTile, builtin_rail_rules_de(), dem.sampler()); REQUIRE(t0b);
    CHECK(*t0b == *t0);
    // Complete ways are merged: the same way in two src.osm tiles (the 128 m halo) is one way.
    const osm::OsmData* both[] = {&data, &data};
    const auto merged = collect_rail_world(both, kZone, builtin_rail_rules_de()); REQUIRE(merged);
    CHECK(merged->ways == world.ways);
    const osm::OsmData* swapped[] = {&data2, &data};
    const auto cross = collect_rail_world(swapped, kZone, builtin_rail_rules_de()); REQUIRE(cross);
    CHECK(cross->ways.size() == 4);
}

TEST_CASE("rail DEM dependencies cover the anchors; a missing tile declines the piece", "[layer][rail][deps]") {
    // Way 11 (bridge) starts in the tile and ends in the tile east of it, where way 13 continues the bridge (owned by
    // the east tile). Its chain anchor at x = 1000 m lies in level-0 tiles no segment of way 11 comes near.
    Scene scene;
    scene.line(10, 300, 400, 50, 0, {{"railway", "rail"}});
    scene.line(11, 400, 520, 40, 0, {{"bridge", "yes"}, {"railway", "rail"}});
    scene.line(13, 520, 1000, 40, 0, {{"bridge", "yes"}, {"railway", "rail"}});
    scene.line(12, 1000, 1200, 50, 0, {{"railway", "rail"}});
    const auto data = scene.data();
    const auto world = world_of(data);
    const auto& rules = builtin_rail_rules_de();
    const auto chain = rail_chain(world, world.ways.at(11), rules);
    CHECK(std::abs(static_cast<double>(chain.pos_b.x) / 1000.0 - (kE0 + 1000.0)) < 0.1);
    CHECK(chain.ext_a_mm == 0);                                                          // the ground way before it is not part of the chain
    CHECK(chain.ext_b_mm / 1000.0 > 470.0);
    const auto deps = rail_dem_dependencies(world, kTile, rules); REQUIRE(deps);
    REQUIRE(!deps->empty());
    CHECK(std::is_sorted(deps->begin(), deps->end()));
    // The deck is profiled from a DEM made of exactly these tiles: nothing is declined, the profile is the chain's line.
    Dem exact;
    for (const auto& ref : *deps) { REQUIRE(ref.layer == kElevBaseLayer); REQUIRE(ref.key.level == 0); exact.add(ref.key, ramp_dem); }
    const auto tile = build_rail_geom_tile(world, kTile, rules, exact.sampler()); REQUIRE(tile);
    const auto* deck = find_piece(*tile, 11); REQUIRE(deck);
    CHECK_FALSE((deck->flags & kRailDeclined));
    CHECK(std::abs(z_at(*deck, 520) / 256.0 - (100.0 + 0.03 * 520)) < 0.05);
    // Sabotage: a dependency set without the anchors' tiles (segments only) leaves the anchor unsampled.
    std::set<TileKey> segments_only;
    for (const auto& plan : plan_owned_pieces(world, kTile, rules)) {
        const auto& w = *plan.way;
        for (std::size_t i = plan.first; i <= plan.last; ++i) {
            const auto x = floor_div(w.pts[i].x, 256000), y = floor_div(w.pts[i].y, 256000);
            segments_only.insert(TileKey{kZone, 0, static_cast<std::int32_t>(x), static_cast<std::int32_t>(y)});
        }
    }
    Dem partial;
    for (const auto& k : segments_only) partial.add(k, ramp_dem);
    const auto declined = build_rail_geom_tile(world, kTile, rules, partial.sampler()); REQUIRE(declined);
    const auto* bad = find_piece(*declined, 11); REQUIRE(bad);
    std::printf("rail dependency sabotage: %zu tiles with anchors, %zu without; bridge declined = %d\n", deps->size(), segments_only.size(), (bad->flags & kRailDeclined) != 0);
    CHECK((bad->flags & kRailDeclined));
    for (const auto h : bad->z) CHECK(h == 0);
    CHECK(validate_rail_geom_tile(*declined));
    // Too many tiles is a limit error, not a truncated list.
    CHECK_FALSE(rail_dem_dependencies(world, kTile, rules, 2));
}

TEST_CASE("rail.geom build is deterministic and tile-set independent", "[layer][rail][determinism]") {
    Scene scene;
    scene.line(10, -500, 500, 25, 0, {{"railway", "rail"}, {"electrified", "contact_line"}, {"tracks", "2"}});
    scene.line(11, -300, 300, 60, 40, {{"bridge", "yes"}, {"layer", "1"}, {"railway", "light_rail"}});
    scene.line(12, -200, 200, 50, -40, {{"railway", "subway"}, {"tunnel", "yes"}, {"layer", "-2"}});
    const auto data = scene.data();
    Dem dem; dem.add_rect(-520, 520, -60, 60, cutting_dem);
    const auto build = [&] { auto t = build_rail_geom_tile(world_of(data), kTile, builtin_rail_rules_de(), dem.sampler()); REQUIRE(t); return std::move(*t); };
    const auto a = build(), b = build();
    CHECK(a == b);
    REQUIRE(a.pieces.size() == 5);   // the 1000 m ground way and the 600 m bridge are each cut after >= 512 m
    const auto ea = encode_rail_geom(a), eb = encode_rail_geom(b); REQUIRE(ea); REQUIRE(eb);
    CHECK(encode_body(*ea).value() == encode_body(*eb).value());
    const auto* tunnel = find_piece(a, 12); REQUIRE(tunnel);
    CHECK((tunnel->flags & kRailTunnel)); CHECK(tunnel->layer == -2); CHECK(tunnel->cls == 2);
    // A tunnel profile is the chain line between the portals, like a bridge: it ignores the cutting at x = 0.
    CHECK(std::abs(z_at(*tunnel, 0) / 256.0 - 100.0) < 0.05);
    // The DEM tiles may arrive in any order and be any superset: the tile is the same.
    Dem more; more.add_rect(-780, 780, -300, 300, cutting_dem);
    std::reverse(more.tiles.begin(), more.tiles.end());
    const auto c = build_rail_geom_tile(world_of(data), kTile, builtin_rail_rules_de(), more.sampler()); REQUIRE(c);
    CHECK(*c == a);
    // Sabotage: a different rules file (the piece length) is a different tile - the content hash must reach the config hash.
    CHECK(RailGeomDeriver().config_hash() == RailGeomDeriver().config_hash());
    CHECK(rail_geom_config_text().find("rules=de:") != std::string::npos);
    auto shorter = builtin_rail_rules_de(); shorter.piece_max_length_m = 100;
    const auto d = build_rail_geom_tile(world_of(data, shorter), kTile, shorter, dem.sampler()); REQUIRE(d);
    CHECK(d->pieces.size() > a.pieces.size());
}

TEST_CASE("rail.geom deriver accepts unspecified height reference and rejects None", "[layer][rail]") {
    Scene scene;
    scene.line(10, -500, 500, 25, 0, {{"railway", "rail"}});
    const auto data = scene.data();
    const RailGeomDeriver deriver;

    std::vector<InputTile> inputs;
    for (const auto& ref : deriver.inputs_for(kTile)) {
        InputTile in;
        in.ref = ref;
        if (ref.key == kTile) {
            const auto raw = encode_src_osm(OsmTile{64, data}); REQUIRE(raw);
            const auto bytes = encode_body(*raw); REQUIRE(bytes);
            in.body = *bytes;
            TileHeader header;
            header.layer = ref.layer; header.layer_version = 1; header.key = ref.key;
            header.datum = VerticalDatum::None; header.body_size = in.body.size();
            header.body_hash = content_hash(in.body); in.header = header;
        }
        inputs.push_back(std::move(in));
    }
    std::sort(inputs.begin(), inputs.end(), [](const auto& a, const auto& b) { return a.ref < b.ref; });
    const auto dem_refs = deriver.additional_inputs_for({kTile, 0, inputs}); REQUIRE(dem_refs);
    REQUIRE(dem_refs->size() > 1);
    const auto add_dem = [&](VerticalDatum datum) {
        auto result = inputs;
        for (const auto& ref : *dem_refs) {
            HeightTile height; height.key = ref.key; height.h.fill(25600);
            const auto raw = encode_height_tile(height); REQUIRE(raw);
            const auto bytes = encode_body(*raw); REQUIRE(bytes);
            InputTile in;
            in.ref = ref; in.body = *bytes;
            TileHeader header;
            header.layer = ref.layer; header.layer_version = 1; header.key = ref.key;
            header.datum = datum; header.body_size = in.body.size();
            header.body_hash = content_hash(in.body); in.header = header;
            result.push_back(std::move(in));
        }
        std::sort(result.begin(), result.end(), [](const auto& a, const auto& b) { return a.ref < b.ref; });
        return result;
    };

    auto unspecified = add_dem(VerticalDatum::Unspecified);
    const auto result = deriver.derive({kTile, 0, unspecified}); REQUIRE(result);
    CHECK(result->datum == VerticalDatum::Unspecified);

    auto none = unspecified;
    for (auto& in : none) if (in.ref.layer == kElevBaseLayer) { in.header->datum = VerticalDatum::None; break; }
    CHECK_FALSE(deriver.derive({kTile, 0, none}));

    auto mixed_known = add_dem(VerticalDatum::DHHN2016);
    std::vector<std::size_t> elevation_indices;
    for (std::size_t i = 0; i < mixed_known.size(); ++i)
        if (mixed_known[i].ref.layer == kElevBaseLayer) elevation_indices.push_back(i);
    REQUIRE(elevation_indices.size() >= 3);
    for (const auto i : elevation_indices) mixed_known[i].header->datum = VerticalDatum::Unspecified;
    mixed_known[elevation_indices.front()].header->datum = VerticalDatum::DHHN2016;
    mixed_known[elevation_indices.back()].header->datum = VerticalDatum::EGM2008;
    CHECK_FALSE(deriver.derive({kTile, 0, mixed_known}));
    std::reverse(mixed_known.begin(), mixed_known.end());
    CHECK_FALSE(deriver.derive({kTile, 0, mixed_known}));
}
TEST_CASE("rail.geom layer descriptor", "[layer][rail]") {
    const auto desc = rail_geom_desc();
    CHECK(desc.id.name == "g2m.rail.geom");
    CHECK(desc.id.version == 1);
    CHECK(desc.encoding == static_cast<std::uint16_t>(LayerEncoding::RailGeom));
    CHECK(desc.encoding_version == kRailGeomEncodingVersion);
    CHECK(desc.levels == level_bit(2));
    REQUIRE(desc.inputs.size() == 2);
    CHECK(desc.inputs[0] == kElevBaseLayer);
    CHECK(desc.inputs[1] == kSrcOsmLayer);
    const RailGeomDeriver d;
    CHECK(d.coverage_inputs().size() == 1);
    CHECK(d.coverage_inputs()[0] == kSrcOsmLayer);
    const auto inputs = d.inputs_for(kTile);
    CHECK(inputs.size() == 9);
    CHECK(std::is_sorted(inputs.begin(), inputs.end()));
    CHECK(d.inputs_for(TileKey{kZone, 1, 3, 3}).empty());
}
