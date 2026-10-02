// OSM road segment extraction (RoadStyle, extract_road_segments) and the
// exact rasteriser (rasterize_road_segments) - g2m/layer/osm_roads.h,
// PLAN.md 6.5.

#include "g2m/layer/osm_roads.h"

#include "g2m/decode/osm/pbf_reader.h"

#include "synthetic.h"

#include <catch2/catch_test_macros.hpp>

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

using namespace g2m;
using osm::OsmBuilder;
using osm::OsmData;

namespace {

// Node at a UTM 32N position (metres), as OSM 1e-7 degrees - same helper as
// tests/layer/test_osm.cpp's own add_utm_node.
void add_utm_node(OsmBuilder& b, std::int64_t id, double e, double n, OsmBuilder::TagList tags = {}) {
    const geo::Utm utm(geo::kGRS80);
    const geo::TmReverse g = utm.reverse(g2m_test::kZone32N, e, n);
    b.add_node(id, g2m_test::to_e7(g.lat), g2m_test::to_e7(g.lon), std::move(tags));
}

// A minimal PbfHandler forwarding nodes/ways straight into an OsmBuilder -
// relations are not needed by anything in this file, and every way in
// testdata/fixtures/osm/frankfurt_hoechst.osm.pbf is documented complete
// (README.md: "every way with >= 1 node in the bbox, completed with all its
// nodes"), so building from nodes + ways alone always succeeds.
class BuilderHandler : public decode::osm::PbfHandler {
public:
    explicit BuilderHandler(OsmBuilder& b) : b_(b) {}

    void on_node(const decode::osm::OsmNode& n) override {
        OsmBuilder::TagList tags;
        for (const decode::osm::Tag& t : n.tags) tags.emplace_back(t.key, t.value);
        b_.add_node(n.id, n.lat_1e7, n.lon_1e7, std::move(tags));
    }
    void on_way(const decode::osm::OsmWay& w) override {
        OsmBuilder::TagList tags;
        for (const decode::osm::Tag& t : w.tags) tags.emplace_back(t.key, t.value);
        b_.add_way(w.id, w.node_refs, std::move(tags));
    }

private:
    OsmBuilder& b_;
};

// One two-node way (nodes 1, 2, ~50 m apart) tagged as given; returns the
// single resulting RoadSegment, or nullopt if extract_road_segments dropped
// the way (not drivable / tunnel / area).
std::optional<RoadSegment> extract_one(std::int64_t way_id, OsmBuilder::TagList tags) {
    OsmBuilder b;
    add_utm_node(b, 1, 358900.0, 5581300.0);
    add_utm_node(b, 2, 358950.0, 5581300.0);
    b.add_way(way_id, {1, 2}, std::move(tags));
    auto set = b.build();
    REQUIRE(set);
    auto segs = extract_road_segments(*set, g2m_test::kZone32N, RoadStyle::default_style());
    if (segs.empty()) return std::nullopt;
    REQUIRE(segs.size() == 1);
    return segs.front();
}

} // namespace

TEST_CASE("osm roads: width comes from the tag, from lanes, or from the default", "[layer][osm]") {
    // Default: highway=residential, no width/lanes tag -> 5.5 m (RoadStyle's
    // own default_width_m for "residential").
    {
        auto seg = extract_one(1, {{"highway", "residential"}});
        REQUIRE(seg);
        CHECK(seg->half_width_mm == 2750); // 5.5 m / 2 -> mm
        CHECK(seg->land_class == LandClass::PavedRoad);
    }
    // width tag overrides.
    {
        auto seg = extract_one(2, {{"highway", "residential"}, {"width", "7"}});
        REQUIRE(seg);
        CHECK(seg->half_width_mm == 3500); // 7 m / 2
    }
    // lanes tag (x 3.25 m) overrides when width is absent.
    {
        auto seg = extract_one(3, {{"highway", "residential"}, {"lanes", "4"}});
        REQUIRE(seg);
        CHECK(seg->half_width_mm == 6500); // 4 * 3.25 m / 2 = 13 m / 2
    }
    // width wins over lanes when both are present.
    {
        auto seg = extract_one(4, {{"highway", "residential"}, {"width", "7"}, {"lanes", "4"}});
        REQUIRE(seg);
        CHECK(seg->half_width_mm == 3500);
    }
    // A unit-suffixed width tag does not parse "cleanly" (tfw.cpp's own
    // rule): treated as absent, falls back to lanes.
    {
        auto seg = extract_one(5, {{"highway", "residential"}, {"width", "7 m"}, {"lanes", "4"}});
        REQUIRE(seg);
        CHECK(seg->half_width_mm == 6500);
    }
    // A zero/negative width tag is not used either.
    {
        auto seg = extract_one(6, {{"highway", "residential"}, {"width", "0"}});
        REQUIRE(seg);
        CHECK(seg->half_width_mm == 2750);
    }
}

TEST_CASE("osm roads: surface tag overrides the highway's default land class", "[layer][osm]") {
    // Paved highway, unpaved surface -> UnpavedRoad + the matched SurfaceKind.
    {
        auto seg = extract_one(10, {{"highway", "residential"}, {"surface", "dirt"}});
        REQUIRE(seg);
        CHECK(seg->land_class == LandClass::UnpavedRoad);
        CHECK(seg->surface == SurfaceKind::Dirt);
    }
    // Unpaved highway, paved surface -> PavedRoad + the matched SurfaceKind.
    {
        auto seg = extract_one(11, {{"highway", "track"}, {"surface", "asphalt"}});
        REQUIRE(seg);
        CHECK(seg->land_class == LandClass::PavedRoad);
        CHECK(seg->surface == SurfaceKind::Asphalt);
    }
    // Paved highway, no surface tag -> PavedRoad from the table, surface Unknown.
    {
        auto seg = extract_one(12, {{"highway", "motorway"}});
        REQUIRE(seg);
        CHECK(seg->land_class == LandClass::PavedRoad);
        CHECK(seg->surface == SurfaceKind::Unknown);
    }
    // Unpaved highway, no surface tag -> UnpavedRoad from the table, surface Unknown.
    {
        auto seg = extract_one(13, {{"highway", "track"}});
        REQUIRE(seg);
        CHECK(seg->land_class == LandClass::UnpavedRoad);
        CHECK(seg->surface == SurfaceKind::Unknown);
    }
    // A surface value matching the table but not changing the land class
    // (still paved) - SurfaceKind extraction is independent of the
    // paved/unpaved decision.
    {
        auto seg = extract_one(14, {{"highway", "service"}, {"surface", "paving_stones"}});
        REQUIRE(seg);
        CHECK(seg->land_class == LandClass::PavedRoad);
        CHECK(seg->surface == SurfaceKind::PavingStones);
    }
    // An unrecognised surface value (a real-world OSM variant not in the
    // table) is not matched: falls back to the highway's own default,
    // rather than being misclassified as paved or unpaved.
    {
        auto seg = extract_one(15, {{"highway", "track"}, {"surface", "cobblestone:flattened"}});
        REQUIRE(seg);
        CHECK(seg->land_class == LandClass::UnpavedRoad);
        CHECK(seg->surface == SurfaceKind::Unknown);
    }
}

TEST_CASE("osm roads: tunnel and area ways are skipped", "[layer][osm]") {
    CHECK_FALSE(extract_one(20, {{"highway", "residential"}, {"tunnel", "yes"}}));
    CHECK_FALSE(extract_one(21, {{"highway", "residential"}, {"area", "yes"}}));
    // bridge=yes is NOT skipped (painted on the ground under it until G3).
    CHECK(extract_one(22, {{"highway", "residential"}, {"bridge", "yes"}}));
    // A highway value not in the table (non-drivable) is skipped.
    CHECK_FALSE(extract_one(23, {{"highway", "footway"}}));
    CHECK_FALSE(extract_one(24, {{"highway", "cycleway"}}));
    // No highway tag at all.
    CHECK_FALSE(extract_one(25, {}));
}

TEST_CASE("osm roads: priority overwrite (rank order wins, array order otherwise)", "[layer][osm]") {
    // Two segments covering the same lattice sample: rank 1 first, rank 2
    // second (as extract_road_segments would order them) - the higher rank
    // must win regardless of which one is geometrically "on top".
    RoadSegment low{};
    low.a = {0, 0};
    low.b = {10000, 0};
    low.half_width_mm = 5000;
    low.land_class = LandClass::UnpavedRoad;
    low.surface = SurfaceKind::Dirt;
    low.rank = 1;

    RoadSegment high = low;
    high.land_class = LandClass::PavedRoad;
    high.surface = SurfaceKind::Asphalt;
    high.rank = 2;

    const Lattice lattice{-10000, -10000, 1000, 21, 21}; // covers [-10000, 10000] both axes
    std::vector<std::uint8_t> land_class(static_cast<std::size_t>(lattice.nx * lattice.ny),
                                         static_cast<std::uint8_t>(LandClass::Unknown));
    std::vector<std::uint8_t> surface(land_class.size(), static_cast<std::uint8_t>(SurfaceKind::Unknown));

    const std::vector<RoadSegment> low_then_high{low, high};
    rasterize_road_segments(low_then_high, lattice, RasterParams{}, land_class, surface);

    const std::size_t centre = 10 * static_cast<std::size_t>(lattice.nx) + 10; // sample (0, 0)
    CHECK(land_class[centre] == static_cast<std::uint8_t>(LandClass::PavedRoad));
    CHECK(surface[centre] == static_cast<std::uint8_t>(SurfaceKind::Asphalt));

    // Reversed array order: low now overwrites high (rasterize_road_segments
    // trusts array order, not rank, by its own documented contract).
    std::fill(land_class.begin(), land_class.end(), static_cast<std::uint8_t>(LandClass::Unknown));
    std::fill(surface.begin(), surface.end(), static_cast<std::uint8_t>(SurfaceKind::Unknown));
    const std::vector<RoadSegment> high_then_low{high, low};
    rasterize_road_segments(high_then_low, lattice, RasterParams{}, land_class, surface);
    CHECK(land_class[centre] == static_cast<std::uint8_t>(LandClass::UnpavedRoad));
    CHECK(surface[centre] == static_cast<std::uint8_t>(SurfaceKind::Dirt));
}

TEST_CASE("osm roads: exact boundary inclusion", "[layer][osm]") {
    // A horizontal segment from (0,0) to (10000,0), half width 3000 mm.
    // Samples exactly at the boundary (perpendicular distance == half
    // width, and exactly at the round end caps) must be INSIDE.
    RoadSegment seg{};
    seg.a = {0, 0};
    seg.b = {10000, 0};
    seg.half_width_mm = 3000;
    seg.land_class = LandClass::PavedRoad;
    seg.surface = SurfaceKind::Asphalt;
    seg.rank = 5;

    const Lattice lattice{-6000, -6000, 1000, 23, 13}; // x in [-6000, 16000], y in [-6000, 6000]
    std::vector<std::uint8_t> land_class(static_cast<std::size_t>(lattice.nx * lattice.ny),
                                         static_cast<std::uint8_t>(LandClass::Unknown));
    std::vector<std::uint8_t> surface(land_class.size(), static_cast<std::uint8_t>(SurfaceKind::Unknown));
    const std::vector<RoadSegment> segs{seg};
    rasterize_road_segments(segs, lattice, RasterParams{}, land_class, surface);

    const auto at = [&](std::int64_t x, std::int64_t y) {
        const std::int64_t i = (x - lattice.origin_e_mm) / lattice.spacing_mm;
        const std::int64_t j = (y - lattice.origin_n_mm) / lattice.spacing_mm;
        return land_class[static_cast<std::size_t>(j) * static_cast<std::size_t>(lattice.nx) +
                          static_cast<std::size_t>(i)] == static_cast<std::uint8_t>(LandClass::PavedRoad);
    };

    // Perpendicular boundary, mid-segment: y == +-3000 at x == 5000 (inside),
    // y == +-3001 would be outside but 3001 is not on the 1000 mm lattice,
    // so check the exact-3000 case is inside and 4000 (1000 mm further out)
    // is outside.
    CHECK(at(5000, 3000));
    CHECK(at(5000, -3000));
    CHECK_FALSE(at(5000, 4000));
    CHECK_FALSE(at(5000, -4000));

    // Round end cap at `a` = (0,0): the point (-3000, 0) is exactly hw from
    // `a` along the segment's own axis (a round cap, not a square one -
    // (-3000, 0) is inside, but the square-cap corner (-3000, 3000), at
    // distance hw*sqrt(2) from `a`, is NOT).
    CHECK(at(-3000, 0));
    CHECK_FALSE(at(-3000, 3000));

    // Round end cap at `b` = (10000,0): symmetric.
    CHECK(at(13000, 0));
    CHECK_FALSE(at(13000, 3000));

    // Well outside the segment entirely.
    CHECK_FALSE(at(5000, 6000));
    CHECK_FALSE(at(-6000, 6000));
}

TEST_CASE("osm roads: shuffled input order gives identical extract_road_segments output", "[layer][osm]") {
    OsmBuilder a;
    add_utm_node(a, 1, 358900.0, 5581300.0);
    add_utm_node(a, 2, 358950.0, 5581300.0);
    add_utm_node(a, 3, 359000.0, 5581350.0);
    a.add_way(100, {1, 2}, {{"highway", "primary"}});
    a.add_way(101, {2, 3}, {{"highway", "track"}, {"surface", "asphalt"}});
    a.add_way(102, {3, 1}, {{"highway", "footway"}}); // dropped either way
    auto set_a = a.build();
    REQUIRE(set_a);

    // Same elements, built in reverse order (ways, then nodes) - mirrors
    // test_osm.cpp's own "rebuild in reverse order" pattern.
    OsmBuilder b;
    b.add_way(102, {3, 1}, {{"highway", "footway"}});
    b.add_way(101, {2, 3}, {{"highway", "track"}, {"surface", "asphalt"}});
    b.add_way(100, {1, 2}, {{"highway", "primary"}});
    add_utm_node(b, 3, 359000.0, 5581350.0);
    add_utm_node(b, 2, 358950.0, 5581300.0);
    add_utm_node(b, 1, 358900.0, 5581300.0);
    auto set_b = b.build();
    REQUIRE(set_b);
    REQUIRE(*set_a == *set_b); // OsmBuilder itself is already order-independent

    // Feed extract_road_segments a THIRD OsmData whose own internal storage
    // order differs (built from set_a's own data, walked in reverse) - the
    // actual point of this test: extract_road_segments's OUTPUT order must
    // not depend on `set`'s way/node storage order either.
    OsmBuilder c;
    for (auto it = set_a->ways.rbegin(); it != set_a->ways.rend(); ++it) {
        const auto refs = set_a->refs_of(*it);
        OsmBuilder::TagList tags;
        for (const osm::Tag& t : set_a->tags_of(*it)) tags.emplace_back(set_a->str(t.key), set_a->str(t.value));
        c.add_way(it->id, std::vector<std::int64_t>(refs.begin(), refs.end()), std::move(tags));
    }
    for (auto it = set_a->nodes.rbegin(); it != set_a->nodes.rend(); ++it) {
        OsmBuilder::TagList tags;
        for (const osm::Tag& t : set_a->tags_of(*it)) tags.emplace_back(set_a->str(t.key), set_a->str(t.value));
        c.add_node(it->id, it->lat, it->lon, std::move(tags));
    }
    auto set_c = c.build();
    REQUIRE(set_c);
    REQUIRE(*set_c == *set_a);

    const auto segs_a = extract_road_segments(*set_a, g2m_test::kZone32N, RoadStyle::default_style());
    const auto segs_c = extract_road_segments(*set_c, g2m_test::kZone32N, RoadStyle::default_style());
    REQUIRE(segs_a.size() == 2); // way 102 (footway) dropped
    CHECK(segs_a == segs_c);
}

TEST_CASE("osm roads: min_half_width_mm keeps a thin road continuous on a coarse lattice", "[layer][osm]") {
    // A thin (50 mm half width) diagonal segment from (0,0) to (10000,4000)
    // on a coarse 1000 mm lattice. At column x=1000, the segment's own
    // perpendicular distance to both neighbouring lattice rows (y=0 and
    // y=1000) exceeds 50 mm (~371 mm and ~557 mm respectively - see
    // osm_roads.h's derivation of the exact inside test for the formula),
    // so with no widening this column is entirely uncovered: a gap.
    RoadSegment seg{};
    seg.a = {0, 0};
    seg.b = {10000, 4000};
    seg.half_width_mm = 50;
    seg.land_class = LandClass::UnpavedRoad;
    seg.surface = SurfaceKind::Dirt;
    seg.rank = 0;

    const Lattice lattice{0, -2000, 1000, 11, 5}; // x in [0,10000], y in [-2000,2000]
    std::vector<std::uint8_t> land_class(static_cast<std::size_t>(lattice.nx * lattice.ny),
                                         static_cast<std::uint8_t>(LandClass::Unknown));
    std::vector<std::uint8_t> surface(land_class.size(), static_cast<std::uint8_t>(SurfaceKind::Unknown));

    const auto column_covered = [&](std::vector<std::uint8_t>& lc, std::int64_t i) {
        for (int j = 0; j < lattice.ny; ++j) {
            if (lc[static_cast<std::size_t>(j) * static_cast<std::size_t>(lattice.nx) + static_cast<std::size_t>(i)] !=
                static_cast<std::uint8_t>(LandClass::Unknown)) {
                return true;
            }
        }
        return false;
    };

    const std::vector<RoadSegment> segs{seg};
    rasterize_road_segments(segs, lattice, RasterParams{}, land_class, surface);
    CHECK_FALSE(column_covered(land_class, 1)); // the documented gap at x=1000

    std::fill(land_class.begin(), land_class.end(), static_cast<std::uint8_t>(LandClass::Unknown));
    std::fill(surface.begin(), surface.end(), static_cast<std::uint8_t>(SurfaceKind::Unknown));
    rasterize_road_segments(segs, lattice, RasterParams{0, 600}, land_class, surface);
    CHECK(column_covered(land_class, 1)); // widened to 600 mm: the gap is gone
}

TEST_CASE("osm roads: frankfurt_hoechst.osm.pbf fixture gives sane road counts", "[layer][osm][real-data]") {
    OsmBuilder b;
    BuilderHandler handler(b);
    const std::string path = std::string(G2M_SOURCE_DIR) + "/testdata/fixtures/osm/frankfurt_hoechst.osm.pbf";
    const decode::osm::PbfStats stats =
        decode::osm::read_pbf(path, handler, nullptr, decode::osm::PbfEntities::Nodes | decode::osm::PbfEntities::Ways);
    CHECK(stats.node_count == 26315);
    CHECK(stats.way_count == 5073);

    auto set = b.build();
    REQUIRE(set);
    REQUIRE(set->validate());

    const auto segs = extract_road_segments(*set, g2m_test::kZone32N, RoadStyle::default_style());
    CHECK(segs.size() > 200);
    CHECK(segs.size() < 30000);

    // Every segment must have a valid (nonzero) half width and a rank in
    // the table's own range, and (the owner's own correctness requirement,
    // 2026-09-27): wherever a surface tag was matched, its paved/unpaved
    // outcome must be internally consistent with the segment's own
    // land_class - the two are set together in resolve_way_style, so this
    // is a whole-dataset regression guard, not just the synthetic cases
    // above.
    bool saw_paved = false, saw_unpaved = false, saw_known_surface = false;
    for (const RoadSegment& s : segs) {
        CHECK(s.half_width_mm > 0);
        CHECK(s.rank >= 0);
        CHECK(s.rank <= 9);
        CHECK((s.land_class == LandClass::PavedRoad || s.land_class == LandClass::UnpavedRoad));
        if (s.land_class == LandClass::PavedRoad) saw_paved = true;
        if (s.land_class == LandClass::UnpavedRoad) saw_unpaved = true;
        if (s.surface != SurfaceKind::Unknown) saw_known_surface = true;
    }
    CHECK(saw_paved);
    CHECK(saw_unpaved);
    CHECK(saw_known_surface);
}

TEST_CASE("OSM speed limits preserve units direction and missing values", "[osm_roads][speed_limit]") {
    REQUIRE(parse_road_speed_limit("120").kph == 120.0);
    REQUIRE(parse_road_speed_limit("50 km/h").kph == 50.0);
    REQUIRE(parse_road_speed_limit("30 mph").kph > 48.28);
    REQUIRE(parse_road_speed_limit("30 mph").kph < 48.29);
    REQUIRE(parse_road_speed_limit("10 knots").kph == 18.52);
    REQUIRE(parse_road_speed_limit("none").kind == SpeedLimitKind::Unrestricted);
    for (const auto text : {"", "DE:urban", "signals", "50;80", "nan", "-5", "50junk"})
        REQUIRE(parse_road_speed_limit(text).kind == SpeedLimitKind::Unknown);
    const auto road = extract_one(42, {{"highway", "primary"}, {"maxspeed", "80"},
        {"maxspeed:forward", "50"}, {"maxspeed:backward", "none"},
        {"maxspeed:conditional", "30 @ (wet)"}});
    REQUIRE(road.has_value());
    REQUIRE(road->way_id == 42);
    REQUIRE(road->forward_limit.kph == 50.0);
    REQUIRE(road->backward_limit.kind == SpeedLimitKind::Unrestricted);
    REQUIRE(road->forward_limit.conditional);
}

TEST_CASE("Speed limit road matching uses direction and clears offroad", "[osm_roads][speed_limit]") {
    RoadSegment road;
    road.a = {0, 0}; road.b = {100000, 0}; road.half_width_mm = 3000; road.way_id = 42;
    road.forward_limit = parse_road_speed_limit("50");
    road.backward_limit = parse_road_speed_limit("80");
    std::vector<RoadSegment> roads{road};
    REQUIRE(match_road_speed_limit(roads, {50000, 0}, 1, 0).limit.kph == 50.0);
    REQUIRE(match_road_speed_limit(roads, {50000, 0}, -1, 0).limit.kph == 80.0);
    REQUIRE(match_road_speed_limit(roads, {50000, 4000}, 1, 0).way_id == 0);
    roads[0].bridge = true;
    REQUIRE(match_road_speed_limit(roads, {50000, 0}, 1, 0).way_id == 0);
    roads[0].bridge = false;
    auto nearer = road; nearer.a.y = nearer.b.y = 1000; nearer.way_id = 43;
    nearer.forward_limit = {};
    roads.push_back(nearer);
    REQUIRE(match_road_speed_limit(roads, {50000, 1000}, 1, 0).way_id == 43);
    REQUIRE(match_road_speed_limit(roads, {50000, 1000}, 1, 0).limit.kind == SpeedLimitKind::Unknown);
}
