// Road clearing tiers (road_tier.h): per-way tier from the caller's ordered rule list, carried on RoadSegment::tier and read by
// RoadValueLut. The tier set is caller data; these tests use a six-tier map shaped like racing_game's (T1 motorway / motorroad,
// T2 trunk, T3 primary, T4 secondary, T5 tertiary, T6 every other paved road).
//
// Recorded sabotages (2026-11-02; each makes the named tests fail, then reverted):
//   S1 road_tier.h tier_for: ignore require_motorroad_yes                                    -> RT1, RT4 fail
//   S2 osm_roads.cpp extract_road_segments: swap tiers 1 and 2 on assignment                 -> RT1 fails
//   S3 osm_roads.cpp paint_road_segments: look the LUT up with tier 0 (ignore seg.tier)      -> RT3 fails
//   S4 osm_roads.cpp by_tier: write tier 0 only (no per-tier fill)                           -> RT2, RT3 fail
#include "g2m/layer/osm_roads.h"
#include "g2m/layer/road_graph.h"
#include "g2m/layer/road_tier.h"

#include "synthetic.h"

#include <catch2/catch_test_macros.hpp>

#include <array>
#include <optional>
#include <string>
#include <vector>

using namespace g2m;
using osm::OsmBuilder;

namespace {

void add_utm_node(OsmBuilder& b, std::int64_t id, double e, double n) {
    const geo::Utm utm(geo::kGRS80);
    const geo::TmReverse g = utm.reverse(g2m_test::kZone32N, e, n);
    b.add_node(id, g2m_test::to_e7(g.lat), g2m_test::to_e7(g.lon));
}

RoadTierRule rule(std::uint8_t tier, std::vector<std::string> highways, bool require = false, bool exclude = false) {
    RoadTierRule r;
    r.tier = tier; r.highways = std::move(highways); r.require_motorroad_yes = require; r.exclude_motorroad_yes = exclude;
    return r;
}

// T1 motorway(+link) or motorroad=yes; T2 trunk(+link); T3 primary; T4 secondary; T5 tertiary; default T6 (tier 5).
RoadTierMap six_tier_map() {
    RoadTierMap m;
    m.rules = {rule(0, {"motorway", "motorway_link"}),
               rule(0, {}, /*require*/ true),
               rule(1, {"trunk", "trunk_link"}, false, /*exclude*/ true),
               rule(2, {"primary", "primary_link"}),
               rule(3, {"secondary", "secondary_link"}),
               rule(4, {"tertiary", "tertiary_link"})};
    m.default_tier = 5;
    return m;
}

// One two-node way tagged as given; the single resulting RoadSegment, or nullopt (not drivable).
std::optional<RoadSegment> extract_one(OsmBuilder::TagList tags, const RoadTierMap* tiers) {
    OsmBuilder b;
    add_utm_node(b, 1, 358900.0, 5581300.0);
    add_utm_node(b, 2, 358950.0, 5581300.0);
    b.add_way(7, {1, 2}, std::move(tags));
    auto set = b.build();
    REQUIRE(set);
    auto segs = extract_road_segments(*set, g2m_test::kZone32N, RoadStyle::default_style(), tiers);
    if (segs.empty()) return std::nullopt;
    REQUIRE(segs.size() == 1);
    return segs.front();
}

} // namespace

TEST_CASE("RT1 road tiers: highway and motorroad pick the tier, first rule wins, an unpaved road has none", "[layer][osm][road_tier]") {
    const RoadTierMap map = six_tier_map();
    struct Case { const char* highway; const char* motorroad; int tier; };
    const Case cases[] = {
        {"motorway", nullptr, 0},       {"motorway_link", nullptr, 0},  {"trunk", "yes", 0},          {"trunk_link", "yes", 0},
        {"trunk", nullptr, 1},          {"trunk", "no", 1},             {"trunk_link", nullptr, 1},   {"primary", nullptr, 2},
        {"primary_link", nullptr, 2},   {"secondary", nullptr, 3},      {"secondary_link", nullptr, 3}, {"tertiary", nullptr, 4},
        {"tertiary_link", nullptr, 4},  {"residential", nullptr, 5},    {"unclassified", nullptr, 5}, {"service", nullptr, 5},
        {"living_street", nullptr, 5},  {"primary", "yes", 0},          {"residential", "yes", 0},
    };
    for (const Case& c : cases) {
        INFO(c.highway << " motorroad=" << (c.motorroad ? c.motorroad : "(absent)"));
        OsmBuilder::TagList tags{{"highway", c.highway}};
        if (c.motorroad) tags.emplace_back("motorroad", c.motorroad);
        const auto seg = extract_one(tags, &map);
        REQUIRE(seg);
        CHECK(seg->land_class == LandClass::PavedRoad);
        CHECK(seg->tier == c.tier);
        // Without a map the old behaviour: tier 0 everywhere.
        CHECK(extract_one(tags, nullptr)->tier == 0);
    }
    // An unpaved track has no tier; the segment carries 0 (ignored by the LUT) and the map says "none".
    const auto track = extract_one({{"highway", "track"}}, &map);
    REQUIRE(track);
    CHECK(track->land_class == LandClass::UnpavedRoad);
    CHECK(track->tier == 0);
    CHECK(map.tier_of("track", false, LandClass::UnpavedRoad) == kNoRoadTier);
    CHECK(map.tier_of("residential", false, LandClass::PavedRoad) == 5);
    // A paved override on a track: paved, no rule matches, the default tier.
    const auto paved_track = extract_one({{"highway", "track"}, {"surface", "asphalt"}}, &map);
    REQUIRE(paved_track);
    CHECK(paved_track->land_class == LandClass::PavedRoad);
    CHECK(paved_track->tier == 5);
    // An unpaved override on a primary road: no tier either.
    const auto gravel_primary = extract_one({{"highway", "primary"}, {"surface", "gravel"}}, &map);
    REQUIRE(gravel_primary);
    CHECK(gravel_primary->land_class == LandClass::UnpavedRoad);
    CHECK(gravel_primary->tier == 0);
}

TEST_CASE("RT2 RoadValueLut by_tier: one value for all tiers equals the old table; a tier value shows only at its tier", "[layer][osm][road_tier]") {
    const auto old_lut = RoadValueLut::by_land_class(3, {{LandClass::PavedRoad, 100}, {LandClass::UnpavedRoad, 50}});
    const std::array<std::uint16_t, 1> one{100};
    const auto flat = RoadValueLut::by_tier(3, 50, one);
    const std::array<std::uint16_t, 8> all_same{100, 100, 100, 100, 100, 100, 100, 100};
    const auto flat8 = RoadValueLut::by_tier(3, 50, all_same);
    CHECK(RoadValueLut::kTiers == kRoadTierSlots);
    CHECK(old_lut.v.size() == 16u * 16u * 8u);
    for (std::size_t lc = 0; lc < 16; ++lc)
        for (std::size_t sk = 0; sk < 16; ++sk)
            for (std::size_t t = 0; t < 8; ++t) {
                const auto land = static_cast<LandClass>(lc);
                const auto kind = static_cast<SurfaceKind>(sk);
                INFO(lc << " " << sk << " tier " << t);
                CHECK(flat.at(land, kind, static_cast<std::uint8_t>(t)) == old_lut.at(land, kind));
                CHECK(flat8.at(land, kind, static_cast<std::uint8_t>(t)) == old_lut.at(land, kind));
                CHECK(old_lut.at(land, kind, static_cast<std::uint8_t>(t)) == old_lut.at(land, kind));
            }
    CHECK(flat.v == flat8.v);

    // Tier-specific values (200 + tier) for paved roads: unpaved and off-road stay tier-independent.
    const std::array<std::uint16_t, 6> six{200, 201, 202, 203, 204, 205};
    const auto lut = RoadValueLut::by_tier(3, 50, six);
    for (std::uint8_t t = 0; t < 8; ++t) {
        const std::uint16_t want = static_cast<std::uint16_t>(200 + (t < 6 ? t : 5)); // tiers past the list repeat its last value
        for (std::size_t sk = 0; sk < 16; ++sk) {
            CHECK(lut.at(LandClass::PavedRoad, static_cast<SurfaceKind>(sk), t) == want);
            CHECK(lut.at(LandClass::UnpavedRoad, static_cast<SurfaceKind>(sk), t) == 50);
            CHECK(lut.at(LandClass::Unknown, static_cast<SurfaceKind>(sk), t) == 3);
        }
    }
    // The tier-0 accessors still address the T1 slice, a tier past the last slot is clamped to it.
    CHECK(lut.at(LandClass::PavedRoad, SurfaceKind::Asphalt) == 200);
    CHECK(lut.v[RoadValueLut::index(LandClass::PavedRoad, SurfaceKind::Asphalt)] == 200);
    CHECK(lut.at(LandClass::PavedRoad, SurfaceKind::Asphalt, 200) == lut.at(LandClass::PavedRoad, SurfaceKind::Asphalt, 7));
    // No paved values: paved roads stay off-road in every tier.
    const auto none = RoadValueLut::by_tier(3, 50, {});
    CHECK(none.at(LandClass::PavedRoad, SurfaceKind::Asphalt, 4) == 3);
}

TEST_CASE("RT3 painting reads the segment's tier", "[layer][osm][road_tier]") {
    // Two parallel segments 100 m apart, tier 0 and tier 3; same land class and surface.
    RoadSegment a, b;
    a.a = {0, 0}; a.b = {100000, 0};
    b.a = {0, 100000}; b.b = {100000, 100000};
    a.half_width_mm = b.half_width_mm = 3000;
    a.land_class = b.land_class = LandClass::PavedRoad;
    a.surface = b.surface = SurfaceKind::Asphalt;
    a.tier = 0; b.tier = 3;
    const std::array<std::uint16_t, 6> six{200, 201, 202, 203, 204, 205};
    const auto lut = RoadValueLut::by_tier(0, 50, six);
    Lattice lat;
    lat.origin_e_mm = 0; lat.origin_n_mm = 0; lat.spacing_mm = 20000; lat.nx = 6; lat.ny = 6; // 100 m x 100 m, 20 m spacing
    std::vector<std::uint16_t> out(36, 0);
    const std::array<RoadSegment, 2> segs{a, b};
    paint_road_segments(segs, lat, RasterParams{}, lut, out, 6);
    for (int i = 0; i < 6; ++i) {
        CHECK(out[static_cast<std::size_t>(0 * 6 + i)] == 200);   // row 0: northing 0 -> tier 0
        CHECK(out[static_cast<std::size_t>(5 * 6 + i)] == 203);   // row 5: northing 100 m -> tier 3
        CHECK(out[static_cast<std::size_t>(2 * 6 + i)] == 0);     // row 2: untouched
    }
    // The flat LUT paints both rows alike (the pre-tier behaviour).
    const auto flat = RoadValueLut::by_land_class(0, {{LandClass::PavedRoad, 100}});
    std::vector<std::uint16_t> out2(36, 0);
    paint_road_segments(segs, lat, RasterParams{}, flat, out2, 6);
    CHECK(out2[0] == 100);
    CHECK(out2[5 * 6] == 100);
}

TEST_CASE("RT4 the ingest keeps motorroad: it reaches the graph way and decides the tier there too", "[layer][osm][road_tier]") {
    OsmBuilder b;
    add_utm_node(b, 1, 358900.0, 5581300.0);
    add_utm_node(b, 2, 358950.0, 5581300.0);
    b.add_way(7, {1, 2}, {{"highway", "trunk"}, {"motorroad", "yes"}});
    b.add_way(8, {1, 2}, {{"highway", "trunk"}});
    b.add_way(9, {1, 2}, {{"highway", "track"}});
    auto set = b.build();
    REQUIRE(set);
    const auto graph = build_road_graph(*set, g2m_test::kZone32N);
    REQUIRE(graph);
    REQUIRE(graph->ways.size() == 3);
    const RoadTierMap map = six_tier_map();
    bool saw_tag = false;
    for (const auto& [k, v] : graph->ways[0].tags) saw_tag = saw_tag || (k == "motorroad" && v == "yes");
    CHECK(saw_tag);
    CHECK(road_tier_of(map, graph->ways[0], LandClass::PavedRoad) == 0);   // trunk + motorroad=yes
    CHECK(road_tier_of(map, graph->ways[1], LandClass::PavedRoad) == 1);   // plain trunk
    CHECK(road_tier_of(map, graph->ways[2], LandClass::UnpavedRoad) == kNoRoadTier);
}
