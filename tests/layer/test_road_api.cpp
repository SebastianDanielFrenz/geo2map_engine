// Street fidelity P1 slice 6: the consumer API of a stored cross-section (design 8.1, 9.1) - carriageway_edges,
// carve_footprint, driving_lanes_at and envelope_road_segments (the grip-raster input), against hand-built
// sections, the committed OSM fixtures and the rasteriser itself.
#include "road_fixture.h"
#include "alloc_counter.h"
#include "g2m/layer/road_cross_section.h"
#include "g2m/layer/road_envelope.h"
#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <algorithm>
#include <array>
#include <cmath>
#include <string>
#include <tuple>
using namespace g2m;
using Catch::Approx;
namespace {
const geo::UtmZone zone{32, geo::Hemisphere::North};

CrossSectionBand band(LaneKind kind, LaneDirection dir, bool counted, int width_mm) {
    CrossSectionBand b; b.kind = kind; b.direction = dir; b.counted = counted; b.width_mm = width_mm;
    b.paved = kind != LaneKind::Verge; return b;
}
// Left to right: Sidewalk 2.0 | Shoulder 0.5 | Backward 3.0 | Forward 3.25 | Shoulder 0.5 | Verge 1.0; the anchor is the
// boundary between the two lanes.
RoadCrossSection two_way(double length_m = 100.0) {
    RoadCrossSection xs;
    xs.bands = {band(LaneKind::Sidewalk, LaneDirection::None, false, 2000), band(LaneKind::Shoulder, LaneDirection::None, false, 500),
        band(LaneKind::Driving, LaneDirection::Backward, true, 3000), band(LaneKind::Driving, LaneDirection::Forward, true, 3250),
        band(LaneKind::Shoulder, LaneDirection::None, false, 500), band(LaneKind::Verge, LaneDirection::None, false, 1000)};
    xs.bands[2].turn = kTurnLeft; xs.bands[3].turn = kTurnThrough | kTurnRight;
    xs.anchor = {2, AnchorEdge::Right, PlacementSource::Explicit};
    xs.length_m = length_m;
    return xs;
}
RoadCrossSection one_way(bool reversed) {
    RoadCrossSection xs;
    const auto dir = reversed ? LaneDirection::Backward : LaneDirection::Forward;
    xs.bands = {band(LaneKind::Shoulder, LaneDirection::None, false, 500), band(LaneKind::Driving, dir, true, 3500),
        band(LaneKind::Driving, dir, true, 3500), band(LaneKind::Shoulder, LaneDirection::None, false, 2500), band(LaneKind::Verge, LaneDirection::None, false, 1500)};
    xs.anchor = {1, AnchorEdge::Left, PlacementSource::Absent};
    xs.flags = static_cast<std::uint16_t>(kXsOneway | (reversed ? kXsReversedOneway : 0));
    xs.length_m = 100;
    return xs;
}
} // namespace

TEST_CASE("carriageway_edges and carve_footprint of a two-way section with a sidewalk", "[layer][road_api]") {
    const auto xs = two_way();
    const auto edges = carriageway_edges(xs, 50.0);
    CHECK(edges.left == Approx(3.0)); CHECK(edges.right == Approx(-3.25));
    const auto f = carve_footprint(xs, 50.0);
    CHECK(f.carriageway_left == Approx(3.0)); CHECK(f.carriageway_right == Approx(-3.25));
    CHECK(f.roadbed_left == Approx(3.5)); CHECK(f.roadbed_right == Approx(-3.75));
    CHECK(f.paved_left == Approx(5.5)); CHECK(f.paved_right == Approx(-3.75));
    CHECK(f.outer_left == Approx(5.5)); CHECK(f.outer_right == Approx(-4.75));
    CHECK(f.crown == CrownKind::Roof); CHECK(f.crown_pivot == Approx(0.0).margin(1e-12));
    CHECK(f.has_kerb_left); CHECK(f.kerb_left == Approx(3.5));
    CHECK_FALSE(f.has_kerb_right); CHECK(kRoadKerbHeightM == Approx(0.12));
    // Without a counted lane boundary the roof sits at the middle of the counted lanes.
    auto both = two_way(); both.bands[2].direction = both.bands[3].direction = LaneDirection::Both;
    CHECK(carve_footprint(both, 10).crown_pivot == Approx((3.0 - 3.25) / 2.0));
}

TEST_CASE("carve_footprint: one-way carriageways fall to the right of the travel direction", "[layer][road_api]") {
    const auto fwd = carve_footprint(one_way(false), 20);
    CHECK(fwd.crown == CrownKind::OneSidedRight); CHECK(fwd.crown_pivot == Approx(fwd.carriageway_left));
    CHECK(fwd.carriageway_left == Approx(0.0).margin(1e-12)); CHECK(fwd.carriageway_right == Approx(-7.0));
    CHECK(fwd.paved_right == Approx(-9.5)); CHECK(fwd.outer_right == Approx(-11.0)); CHECK(fwd.paved_left == Approx(0.5));
    const auto rev = carve_footprint(one_way(true), 20);
    CHECK(rev.crown == CrownKind::OneSidedLeft); CHECK(rev.crown_pivot == Approx(rev.carriageway_right));
    CHECK_FALSE(rev.has_kerb_left); CHECK_FALSE(rev.has_kerb_right);
}

TEST_CASE("driving_lanes_at: counted lanes of a direction, left to right in the travel direction", "[layer][road_api]") {
    auto xs = two_way();
    // Add a counted forward turn lane on the right so order and filtering are visible.
    xs.bands.insert(xs.bands.begin() + 4, band(LaneKind::Turn, LaneDirection::Forward, true, 3000));
    xs.bands[4].turn = kTurnRight;
    std::array<LaneQuery, 8> q{};
    const auto nf = driving_lanes_at(xs, 40, LaneDirection::Forward, q);
    REQUIRE(nf == 2);
    CHECK(q[0].band == 3); CHECK(q[1].band == 4);
    CHECK(q[0].centre_t == Approx(-1.625)); CHECK(q[1].centre_t == Approx(-3.25 - 1.5));
    CHECK(q[0].centre_t > q[1].centre_t); // left to right = decreasing t
    CHECK(q[0].turn == (kTurnThrough | kTurnRight)); CHECK(q[1].kind == LaneKind::Turn); CHECK(q[0].width_m == Approx(3.25));
    const auto nb = driving_lanes_at(xs, 40, LaneDirection::Backward, q);
    REQUIRE(nb == 1); CHECK(q[0].band == 2); CHECK(q[0].centre_t == Approx(1.5)); CHECK(q[0].direction == LaneDirection::Backward);
    CHECK(driving_lanes_at(xs, 40, LaneDirection::Both, q) == 0);
    // out smaller than the answer truncates; a Both lane serves either direction, in reverse band order for Backward.
    CHECK(driving_lanes_at(xs, 40, LaneDirection::Forward, std::span<LaneQuery>(q.data(), 1)) == 1);
    auto bidir = two_way(); bidir.bands[2].direction = bidir.bands[3].direction = LaneDirection::Both;
    REQUIRE(driving_lanes_at(bidir, 10, LaneDirection::Backward, q) == 2);
    CHECK(q[0].band == 3); CHECK(q[1].band == 2); CHECK(q[0].centre_t < q[1].centre_t);
    // A lane narrower than 0.5 m (mid-taper) is excluded.
    auto taper = two_way(); taper.bands[3].start = {20000, 0};
    CHECK(driving_lanes_at(taper, 0.0, LaneDirection::Forward, q) == 0);
    CHECK(driving_lanes_at(taper, 20.0, LaneDirection::Forward, q) == 1);
    // A reversed one-way serves Backward only.
    CHECK(driving_lanes_at(one_way(true), 5, LaneDirection::Forward, q) == 0);
    CHECK(driving_lanes_at(one_way(true), 5, LaneDirection::Backward, q) == 2);
}

TEST_CASE("consumer queries allocate nothing", "[layer][road_api][alloc]") {
    const auto xs = two_way();
    if (!g2m_test::kAllocCounterAvailable) SKIP("allocation counting unavailable under a sanitizer build (G2M_SANITIZER_BUILD)");
    std::array<LaneQuery, 8> q{}; std::array<double, 7> t{};
    double sink = 0; std::size_t lanes = 0;
    g2m_test::ScopedAllocCounter counter;
    for (int i = 0; i < 2000; ++i) {
        const double s = 0.05 * i;
        sink += carve_footprint(xs, s).crown_pivot + carriageway_edges(xs, s).left;
        sink += xs.boundaries_at(s, t) ? t[0] : 0.0;
        lanes += driving_lanes_at(xs, s, i % 2 ? LaneDirection::Forward : LaneDirection::Backward, q);
    }
    CHECK(counter.count() == 0);
    CHECK(lanes == 2000); CHECK(std::isfinite(sink));
}

namespace {
// A straight 100 m stretch (way 10, x = 10 m .. 110 m, y = 10 m) with a hand-made section.
RoadGeomTile straight_tile(const RoadCrossSection& xs) {
    RoadGraph graph;
    graph.nodes = {{1, {10000, 10000}}, {2, {110000, 10000}}};
    graph.ways = {{10, 4, {1, 2}, {{"highway", "residential"}, {"maxspeed", "30"}}}};
    graph.edges = {{10, 0, 1, 2}};
    const auto source = *select_road_graph_tile(graph, {zone, 2, 0, 0});
    HeightTile heights; heights.key = {zone, 0, 0, 0}; heights.h.fill(25600);
    const HeightTile* views[] = {&heights}; const auto dem = RoadDemSampler::make(views); REQUIRE(dem);
    auto tile = build_road_geom_tile(source, *dem); REQUIRE(tile);
    REQUIRE(tile->entries.size() == 1); REQUIRE(tile->entries[0].cross_section);
    auto section = xs; section.length_m = tile->entries[0].cross_section->length_m;
    tile->entries[0].cross_section = section;
    return std::move(*tile);
}
bool covered(const std::vector<RoadSegment>& segs, geom::PointMm p, std::uint8_t& land, std::uint8_t& surface) {
    std::uint8_t l = 0, s = 0;
    rasterize_road_segments(segs, Lattice{p.x, p.y, 250, 1, 1}, RasterParams{}, std::span<std::uint8_t>(&l, 1), std::span<std::uint8_t>(&s, 1));
    land = l; surface = s; return l != 0;
}
}

TEST_CASE("envelope_road_segments paints the carve footprint on a 0.25 m lattice (tapers, sidewalk, offset)", "[layer][road_api]") {
    auto xs = two_way();
    xs.bands[3].start = {60000, 0};      // a lane growing in over 60 m at the start (the steepest shipped taper)
    xs.bands[5].end = {20000, 0};        // the verge fades out at the end (never painted either way)
    xs.bands[0].end = {12000, 500};      // the sidewalk narrows at the end
    xs.offset = {OffsetMode::Ramps, 400, 15000, -250, 10000};
    const auto tile = straight_tile(xs);
    const auto& section = *tile.entries[0].cross_section;
    const double length = section.length_m;
    const auto segs = envelope_road_segments(tile, {zone, 0, 0, 0});
    REQUIRE(segs.size() > 100);
    const Lattice lattice{0, 0, 250, 560, 80};
    std::vector<std::uint8_t> land(560 * 80, 0), surface(560 * 80, 0);
    rasterize_road_segments(segs, lattice, RasterParams{}, land, surface);
    std::size_t checked = 0, painted = 0, sidewalk = 0;
    for (int j = 0; j < 80; ++j) for (int i = 0; i < 560; ++i) {
        const double x = i * 0.25, y = j * 0.25;
        const double s = x - 10.0, t = y - 10.0;
        if (s < 0.5 || s > length - 0.5) continue; // round end caps
        const auto f = carve_footprint(section, s);
        const double lo = f.paved_right, hi = f.paved_left;
        if (std::abs(t - lo) < 0.05 || std::abs(t - hi) < 0.05) continue;
        const bool expect = t > lo && t < hi;
        const bool is_painted = land[j * 560 + i] != 0;
        INFO("s " << s << " t " << t << " paved [" << lo << ", " << hi << "]");
        REQUIRE(is_painted == expect);
        ++checked; painted += expect;
        if (expect && t > f.roadbed_left + 0.05) { CHECK(surface[j * 560 + i] == static_cast<std::uint8_t>(SurfaceKind::PavingStones)); ++sidewalk; }
        if (expect && t < f.roadbed_left - 0.05) CHECK(land[j * 560 + i] == static_cast<std::uint8_t>(LandClass::PavedRoad));
    }
    CHECK(checked > 20000); CHECK(painted > 8000); CHECK(sidewalk > 1500);
    // Speed limit, rank and way id come with the segment.
    CHECK(segs.front().way_id == 10); CHECK(segs.front().rank == 4);
    CHECK(segs.front().forward_limit.kind == SpeedLimitKind::Numeric); CHECK(segs.front().forward_limit.kph == Approx(30.0));
}

TEST_CASE("envelope_road_segments: edge error at tapers stays within 3 cm", "[layer][road_api]") {
    // A peak slope of 1:20 (ramp 40 m for 1 m of width, bound 3 cm) and the steepest shipped taper (3.5 m lane
    // over 60 m, peak slope 1:8.6: measured 4.8 cm, bound 5 cm; the round caps of the capsule chain add omega^2 h / 2).
    for (const auto& [ramp_mm, grow_mm, bound] : {std::tuple{40000, 1000, 0.03}, std::tuple{60000, 3500, 0.05}}) {
        auto xs = two_way(); xs.bands[3].width_mm = grow_mm; xs.bands[3].start = {ramp_mm, 0};
        const auto tile = straight_tile(xs);
        const auto& section = *tile.entries[0].cross_section;
        const auto segs = envelope_road_segments(tile, {zone, 0, 0, 0});
        double worst = 0;
        for (double s = 3.0; s < std::min(section.length_m - 3.0, ramp_mm / 1000.0 + 3.0); s += 0.0377) {
            const auto f = carve_footprint(section, s);
            const double x = 10.0 + s;
            for (const auto& [edge, up] : {std::pair{f.paved_left, true}, std::pair{f.paved_right, false}}) {
                // Painted edge: bisect on the exact capsule test in doubles over (edge -/+ 0.2 m).
                const auto inside = [&](double y) {
                    for (const auto& seg : segs) {
                        const double ax = seg.a.x / 1000.0, ay = seg.a.y / 1000.0, bx = seg.b.x / 1000.0, by = seg.b.y / 1000.0;
                        const double dx = bx - ax, dy = by - ay, hw = seg.half_width_mm / 1000.0;
                        const double u = std::clamp(((x - ax) * dx + (y - ay) * dy) / (dx * dx + dy * dy), 0.0, 1.0);
                        const double ex = x - (ax + u * dx), ey = y - (ay + u * dy);
                        if (ex * ex + ey * ey <= hw * hw) return true;
                    }
                    return false;
                };
                double in = 10.0 + edge + (up ? -0.2 : 0.2), out = 10.0 + edge + (up ? 0.2 : -0.2);
                REQUIRE(inside(in)); REQUIRE(!inside(out));
                for (int k = 0; k < 40; ++k) { const double mid = (in + out) / 2; (inside(mid) ? in : out) = mid; }
                worst = std::max(worst, std::abs((in + out) / 2 - (10.0 + edge)));
            }
        }
        INFO("ramp " << ramp_mm << " mm, width " << grow_mm << " mm: worst edge error " << worst);
        CHECK(worst <= bound);
    }
}

TEST_CASE("envelope_road_segments: clipping, order, declines and legacy tiles", "[layer][road_api]") {
    const auto tile = straight_tile(two_way());
    const auto all = envelope_road_segments(tile, {zone, 0, 0, 0});
    CHECK(all.size() >= 200); // 100 stations of the road bed plus the sidewalk
    CHECK(envelope_road_segments(tile, {zone, 0, 1, 0}).empty());                 // another 256 m tile (x >= 256 m)
    CHECK(envelope_road_segments(tile, {{33, geo::Hemisphere::North}, 0, 0, 0}).empty()); // another zone
    CHECK(envelope_road_segments(tile, {zone, 0, 0, 0}, 0.0).empty());
    // A coarser spacing emits fewer capsules and the same road.
    CHECK(envelope_road_segments(tile, {zone, 0, 0, 0}, 5.0).size() * 4 < all.size());
    // Sidewalk capsules come before the road bed of the same way, so the asphalt wins where they touch.
    std::size_t first_bed = all.size(), last_sidewalk = 0;
    for (std::size_t i = 0; i < all.size(); ++i) {
        if (all[i].surface == SurfaceKind::PavingStones) last_sidewalk = i; else first_bed = std::min(first_bed, i);
    }
    CHECK(last_sidewalk < first_bed);
    // A second, more important way is painted later; declined entries are absent.
    RoadGeomTile two = tile;
    two.entries.push_back(two.entries[0]);
    two.entries[1].way_id = 11;
    two.source.graph.ways.push_back({11, 7, {1, 2}, {{"highway", "primary"}}});
    const auto ordered = envelope_road_segments(two, {zone, 0, 0, 0});
    REQUIRE(ordered.size() > all.size());
    CHECK(std::is_sorted(ordered.begin(), ordered.end(), [](const RoadSegment& a, const RoadSegment& b) { return std::pair{a.rank, a.way_id} < std::pair{b.rank, b.way_id}; }));
    CHECK(ordered.back().rank == 7);
    two.entries[1].cross_section.reset(); two.entries[1].cross_section_decline = CrossSectionDecline{};
    CHECK(envelope_road_segments(two, {zone, 0, 0, 0}).size() == all.size());
    two.entries[0].cross_section.reset(); two.encoding = kRoadGeomEncodingLegacy;
    CHECK(envelope_road_segments(two, {zone, 0, 0, 0}).empty());
}

TEST_CASE("Hoechst fixture: driving_lanes_at on the F8 asymmetric two-way join", "[layer][road_api][roads]") {
    const auto data = g2m_test::read_osm_fixture(std::string(G2M_SOURCE_DIR) + "/testdata/fixtures/osm/frankfurt_hoechst.osm.pbf");
    const auto world = build_road_graph(data, g2m_test::kZone32N); REQUIRE(world);
    constexpr std::int64_t node_id = 2648065583, way_a = 37804346, way_b = 259399543;
    const auto node = std::find_if(world->nodes.begin(), world->nodes.end(), [&](const auto& n) { return n.osm_id == node_id; });
    REQUIRE(node != world->nodes.end());
    const auto home = tile_key_at(g2m_test::kZone32N, 2, static_cast<double>(node->position.x) / 1000.0, static_cast<double>(node->position.y) / 1000.0); REQUIRE(home);
    std::vector<RoadGeomTile> tiles;
    for (int dy = -1; dy <= 1; ++dy) for (int dx = -1; dx <= 1; ++dx) {
        const TileKey key{home->zone, 2, home->x + dx, home->y + dy};
        const auto probe = select_road_graph_tile(*world, key);
        if (!probe || probe->owned_edges.empty()) continue;
        auto g = g2m_test::geometry_in_tile(*world, key); REQUIRE(g);
        tiles.push_back(std::move(*g));
    }
    const RoadGeomEntry* wider = nullptr; const RoadGraphWay* wider_way = nullptr;
    for (const auto& tile : tiles) for (const auto& entry : tile.entries) {
        if ((entry.way_id != way_a && entry.way_id != way_b) || !entry.cross_section) continue;
        const auto it = std::find_if(tile.source.graph.ways.begin(), tile.source.graph.ways.end(), [&](const auto& w) { return w.osm_id == entry.way_id; });
        REQUIRE(it != tile.source.graph.ways.end());
        const bool at_start = it->node_ids[entry.stretch.start_ref] == node_id, at_end = it->node_ids[entry.stretch.end_ref] == node_id;
        if (!at_start && !at_end) continue;
        std::size_t counted = 0; for (const auto& b : entry.cross_section->bands) counted += b.counted;
        if (counted == 4) { wider = &entry; wider_way = &*it; }
    }
    REQUIRE(wider); REQUIRE(wider_way);
    const auto& xs = *wider->cross_section;
    const bool at_end = wider_way->node_ids[wider->stretch.end_ref] == node_id;
    // The added forward lane grows from zero width at the join.
    const auto ramp = [&](const CrossSectionBand& b) { return at_end ? b.end : b.start; };
    std::size_t added = xs.bands.size();
    for (std::size_t i = 0; i < xs.bands.size(); ++i) if (xs.bands[i].counted && ramp(xs.bands[i]).length_mm > 0 && ramp(xs.bands[i]).from_width_mm == 0) added = i;
    REQUIRE(added < xs.bands.size());
    const double ramp_m = ramp(xs.bands[added]).length_mm / 1000.0;
    const double at_join = at_end ? xs.length_m : 0.0, past_ramp = at_end ? xs.length_m - ramp_m : ramp_m;
    std::array<LaneQuery, 8> q{};
    const std::size_t forward_full = driving_lanes_at(xs, past_ramp, LaneDirection::Forward, q);
    CHECK(forward_full == 3);
    for (std::size_t k = 0; k + 1 < forward_full; ++k) CHECK(q[k].centre_t > q[k + 1].centre_t);
    CHECK(q[forward_full - 1].band == added); // the added lane is the rightmost forward lane
    CHECK(driving_lanes_at(xs, at_join, LaneDirection::Forward, q) == 2); // zero width at the join: excluded
    CHECK(driving_lanes_at(xs, past_ramp, LaneDirection::Backward, q) == 1);
    CHECK(driving_lanes_at(xs, at_join, LaneDirection::Backward, q) == 1);
}

TEST_CASE("Hoechst A 66 tile: painted envelope equals the carve footprint along real stretches", "[layer][road_api][roads]") {
    const auto data = g2m_test::read_osm_fixture(std::string(G2M_SOURCE_DIR) + "/testdata/fixtures/osm/hoechst_a66.osm.pbf");
    const auto world = build_road_graph(data, g2m_test::kZone32N); REQUIRE(world);
    const TileKey key{g2m_test::kZone32N, 2, 455, 5421};
    auto geometry = g2m_test::geometry_in_tile(*world, key); REQUIRE(geometry);
    const auto whole = *geometry;
    std::size_t entries_checked = 0, probes = 0;
    for (std::size_t ei = 0; ei < whole.entries.size() && entries_checked < 14; ++ei) {
        const auto& entry = whole.entries[ei];
        if (!entry.cross_section || !entry.profile) continue;
        const auto& ref = entry.profile->reference; const auto& xs = *entry.cross_section;
        if (ref.length_m < 30 || ref.length_m > 250 || xs.bands.size() < 3) continue;
        bool gentle = true;
        for (double s = 0; s <= ref.length_m; s += 1.0) { const auto p = ref.at(s); if (!p || std::abs(p->curvature) > 0.02) { gentle = false; break; } }
        if (!gentle) continue;
        ++entries_checked;
        RoadGeomTile one = whole; one.entries = {entry}; one.features.clear();
        const auto mid = ref.at(ref.length_m / 2); REQUIRE(mid);
        const auto l0 = tile_key_at(g2m_test::kZone32N, 0, mid->x, mid->y); REQUIRE(l0);
        // A long road crosses several 256 m tiles: gather the 3 x 3 block around the middle.
        std::vector<RoadSegment> segs;
        for (int dy = -1; dy <= 1; ++dy) for (int dx = -1; dx <= 1; ++dx) {
            const auto part = envelope_road_segments(one, {l0->zone, 0, l0->x + dx, l0->y + dy});
            for (const auto& seg : part) if (std::find(segs.begin(), segs.end(), seg) == segs.end()) segs.push_back(seg);
        }
        for (double s = 2.0; s <= ref.length_m - 2.0; s += 0.61) {
            const auto f = carve_footprint(xs, s);
            for (double t = f.outer_right - 1.5; t <= f.outer_left + 1.5; t += 0.23) {
                if (std::abs(t - f.paved_left) < 0.06 || std::abs(t - f.paved_right) < 0.06) continue;   // 0.06: a landed merge ramp (G2M-028) puts a probe 0.0500 m off an edge
                const auto p = lateral_point(ref, s, t); REQUIRE(p);
                const geom::PointMm pm{static_cast<std::int64_t>(std::llround(p->x * 1000.0)), static_cast<std::int64_t>(std::llround(p->y * 1000.0))};
                std::uint8_t land = 0, surface = 0;
                const bool is_painted = covered(segs, pm, land, surface);
                const bool expect = t > f.paved_right && t < f.paved_left;
                INFO("way " << entry.way_id << " s " << s << " t " << t << " paved [" << f.paved_right << ", " << f.paved_left << "]");
                REQUIRE(is_painted == expect);
                ++probes;
                if (expect && f.has_kerb_left && t > f.kerb_left + 0.05) CHECK(surface == static_cast<std::uint8_t>(SurfaceKind::PavingStones));
            }
        }
    }
    CHECK(entries_checked >= 6); CHECK(probes > 3000);
}
