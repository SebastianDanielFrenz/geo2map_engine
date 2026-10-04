// tests/unit/test_osm_tag_probe.cpp - tools/osm_tag_probe (street-level fidelity
// plan P0): the aggregation and report logic on hand-built tags, and the PBF
// scan against the committed Frankfurt-Hoechst fixture (testdata/fixtures/):
// counts that must hold by construction, and two runs that must produce the
// same bytes.

#include "g2m/decode/osm/pbf_reader.h"
#include "g2m/import/region_resolve.h"
#include "g2m/layer/region.h"
#include "scan.h"
#include "tag_probe.h"

#include <catch2/catch_test_macros.hpp>

#include <fstream>
#include <sstream>
#include <string>
#include <vector>

using namespace g2m;
using namespace g2m::tag_probe;

namespace {

std::vector<Tag> tags(std::initializer_list<std::pair<const char*, const char*>> kv) {
    std::vector<Tag> t;
    for (const auto& p : kv) t.push_back({p.first, p.second});
    return t;
}

std::string fixtures_dir() { return std::string(G2M_SOURCE_DIR) + "/testdata/fixtures"; }

std::string read_file(const std::string& path) {
    std::ifstream in(path, std::ios::binary);
    REQUIRE(in.is_open());
    std::ostringstream ss;
    ss << in.rdbuf();
    return ss.str();
}

} // namespace

TEST_CASE("tag_probe: percent and unit formatting is exact integer rounding", "[osm][probe]") {
    CHECK(format_pct(1, 4, 1) == "25.0");
    CHECK(format_pct(1, 3, 2) == "33.33");
    CHECK(format_pct(2, 3, 2) == "66.67");
    CHECK(format_pct(1, 3, 0) == "33");
    CHECK(format_pct(1, 8, 1) == "12.5");
    CHECK(format_pct(1, 16, 1) == "6.3"); // 6.25 rounds half up
    CHECK(format_pct(5, 5, 2) == "100.00");
    CHECK(format_pct(0, 0, 2) == "0.00");
    CHECK(format_pct(7, 1000, 2) == "0.70");
    CHECK(format_km(1234567) == "1.235");
    CHECK(format_km(0) == "0.000");
    CHECK(format_km(999) == "0.001");
    CHECK(format_ha(25000) == "2.5");
    CHECK(format_ha(0) == "0.0");
}

TEST_CASE("tag_probe: road classes count ways, km and key presence", "[osm][probe]") {
    Accumulator acc;
    const auto a = tags({{"highway", "primary"}, {"lanes", "2"}, {"turn:lanes:forward", "left|through"}, {"sidewalk", "no"}});
    const auto b = tags({{"highway", "primary"}, {"junction", "roundabout"}, {"width", "7"}});
    const auto c = tags({{"highway", "primary_link"}, {"lanes:forward", "1"}, {"placement", "right_of:1"}});
    const auto d = tags({{"highway", "footway"}, {"lanes", "9"}}); // not a reported class
    const auto e = tags({{"highway", "residential"}, {"parking:lane:both", "parallel"}, {"sidewalk:left", "yes"}});
    acc.add_way(a, 1'000'000, 0, false);
    acc.add_way(b, 3'000'000, 0, true);
    acc.add_way(c, 500'000, 0, false);
    acc.add_way(d, 100'000, 0, false);
    acc.add_way(e, 200'000, 0, false);

    const auto& p = acc.road("primary");
    CHECK(p.all.n == 2);
    CHECK(p.all.length_mm == 4'000'000);
    CHECK(p.with.at("lanes").n == 1);
    CHECK(p.with.at("lanes").length_mm == 1'000'000);
    CHECK(p.with.at("turn_lanes").n == 1);
    CHECK(p.with.at("sidewalk").n == 1);
    CHECK(p.with.at("junction_roundabout").n == 1);
    CHECK(p.with.at("junction_roundabout").length_mm == 3'000'000);
    CHECK(p.with.at("width").n == 1);
    CHECK(p.with.count("placement") == 0);

    const auto& l = acc.road("primary_link");
    CHECK(l.all.n == 1);
    CHECK(l.with.at("lanes_fwd_bwd").n == 1);
    CHECK(l.with.at("placement").n == 1);
    CHECK(l.with.count("lanes") == 0); // lanes:forward is not lanes

    const auto& r = acc.road("residential");
    CHECK(r.with.at("parking_lane").n == 1);
    CHECK(r.with.at("parking_any").n == 1);
    CHECK(r.with.at("sidewalk").n == 1); // sidewalk:left counts

    CHECK(acc.road("footway").all.n == 0);
    CHECK(acc.road("motorway").all.n == 0);
}

TEST_CASE("tag_probe: junction=roundabout needs the exact value, empty values are absent", "[osm][probe]") {
    Accumulator acc;
    acc.add_way(tags({{"highway", "tertiary"}, {"junction", "circular"}, {"lanes", ""}}), 10, 0, false);
    const auto& t = acc.road("tertiary");
    CHECK(t.all.n == 1);
    CHECK(t.with.count("junction_roundabout") == 0);
    CHECK(t.with.count("lanes") == 0);
}

TEST_CASE("tag_probe: turn:lanes values are tallied and ranked by count then value", "[osm][probe]") {
    Accumulator acc;
    for (int i = 0; i < 3; ++i) acc.add_way(tags({{"highway", "primary"}, {"turn:lanes", "left|through"}}), 1, 0, false);
    for (int i = 0; i < 3; ++i) acc.add_way(tags({{"highway", "primary"}, {"turn:lanes:forward", "through|right"}}), 1, 0, false);
    acc.add_way(tags({{"highway", "primary"}, {"turn:lanes:backward", "through"}}), 1, 0, false);
    acc.add_way(tags({{"highway", "motorway"}, {"turn:lanes", "left|through"}}), 1, 0, false);
    const auto top = acc.top_values("road.turn:lanes", 15);
    REQUIRE(top.size() == 3);
    CHECK(top[0] == std::pair<std::string, std::uint64_t>{"left|through", 4});
    CHECK(top[1] == std::pair<std::string, std::uint64_t>{"through|right", 3});
    CHECK(top[2] == std::pair<std::string, std::uint64_t>{"through", 1});
    CHECK(acc.top_values("road.turn:lanes", 2).size() == 2);

    // Equal counts order by value, ascending.
    Accumulator tie;
    tie.add_way(tags({{"highway", "primary"}, {"turn:lanes", "b"}}), 1, 0, false);
    tie.add_way(tags({{"highway", "primary"}, {"turn:lanes", "a"}}), 1, 0, false);
    const auto tt = tie.top_values("road.turn:lanes", 0);
    REQUIRE(tt.size() == 2);
    CHECK(tt[0].first == "a");
}

TEST_CASE("tag_probe: node kinds, tallies and degree placement", "[osm][probe]") {
    CHECK(Accumulator::node_kind_mask(tags({{"highway", "crossing"}})) == kNodeCrossing);
    CHECK(Accumulator::node_kind_mask(tags({{"highway", "crossing"}, {"kerb", "lowered"}})) == (kNodeCrossing | kNodeKerb));
    CHECK(Accumulator::node_kind_mask(tags({{"highway", "traffic_signals"}, {"traffic_calming", "bump"}})) ==
          (kNodeSignals | kNodeCalming));
    CHECK(Accumulator::node_kind_mask(tags({{"natural", "tree"}})) == kNodeTree);
    CHECK(Accumulator::node_kind_mask(tags({{"highway", "bus_stop"}})) == 0);
    CHECK(Accumulator::node_kind_mask(tags({{"natural", "peak"}})) == 0);

    Accumulator acc;
    const auto c1 = tags({{"highway", "crossing"}, {"crossing", "traffic_signals"}, {"crossing:markings", "zebra"}});
    const auto c2 = tags({{"highway", "crossing"}, {"crossing", "uncontrolled"}, {"crossing_ref", "zebra"}});
    const auto c3 = tags({{"highway", "crossing"}});
    acc.add_node(Accumulator::node_kind_mask(c1), c1);
    acc.add_node(Accumulator::node_kind_mask(c2), c2);
    acc.add_node(Accumulator::node_kind_mask(c3), c3);
    const Group& g = acc.node_group("crossing");
    CHECK(g.all.n == 3);
    CHECK(g.with.at("crossing").n == 2);
    CHECK(g.with.at("crossing:markings").n == 1);
    CHECK(g.with.at("crossing_ref").n == 1);
    CHECK(acc.top_values("crossing.crossing:markings", 15).front().first == "zebra");

    acc.add_node_degree(kNodeCrossing, 0);
    acc.add_node_degree(kNodeCrossing, 2);
    acc.add_node_degree(kNodeCrossing, 2);
    acc.add_node_degree(kNodeCrossing, 4);
    acc.add_node_degree(kNodeCrossing | kNodeSignals, 1);
    const auto pc = acc.placement("crossing");
    CHECK(pc[kPlaceOffRoad] == 1);
    CHECK(pc[kPlaceDeadEnd] == 1);
    CHECK(pc[kPlaceMidWay] == 2);
    CHECK(pc[kPlaceJunction] == 1);
    CHECK(acc.placement("traffic_signals")[kPlaceDeadEnd] == 1);
    CHECK(acc.placement("tree")[kPlaceOffRoad] == 0); // trees carry no placement
}

TEST_CASE("tag_probe: buildings and forest polygons, closed ways vs relations vs open ways", "[osm][probe]") {
    Accumulator acc;
    const auto b1 = tags({{"building", "house"}, {"roof:shape", "gabled"}, {"building:levels", "2"}, {"height", "8"}});
    const auto b2 = tags({{"building", "yes"}, {"roof:shape", "flat"}, {"building:colour", "red"}});
    const auto b3 = tags({{"building", "no"}});
    acc.add_way(b1, 40'000, 120, true);
    acc.add_way(b2, 40'000, 80, true);
    acc.add_way(b3, 40'000, 50, true);                                    // building=no: ignored
    acc.add_way(tags({{"building", "yes"}}), 10'000, 0, false);           // open: counted aside
    const auto rel = tags({{"type", "multipolygon"}, {"building", "apartments"}, {"roof:shape", "gabled"}});
    CHECK(Accumulator::relation_is_relevant(rel));
    CHECK_FALSE(Accumulator::relation_is_relevant(tags({{"type", "boundary"}, {"building", "yes"}})));
    acc.add_relation_polygon(rel, 500);
    acc.add_relation_polygon(tags({{"type", "multipolygon"}, {"building", "yes"}}), 0); // no closed ring: area 0

    const Group& g = acc.way_group("building");
    CHECK(g.all.n == 4);
    CHECK(g.all.area_m2 == 120 + 80 + 500);
    CHECK(g.with.at("roof:shape").n == 3);
    CHECK(g.with.at("height").n == 1);
    CHECK(g.with.at("building:levels").n == 1);
    CHECK(g.with.at("building:colour").n == 1);
    CHECK(acc.building_relations().n == 2);
    CHECK(acc.building_open_ways().n == 1);
    const auto shapes = acc.top_values("building.roof:shape", 15);
    REQUIRE(shapes.size() == 2);
    CHECK(shapes[0] == std::pair<std::string, std::uint64_t>{"gabled", 2});

    acc.add_way(tags({{"natural", "wood"}, {"leaf_type", "broadleaved"}}), 0, 1000, true);
    acc.add_way(tags({{"landuse", "forest"}, {"natural", "wood"}, {"leaf_cycle", "deciduous"}}), 0, 3000, true);
    acc.add_way(tags({{"landuse", "forest"}}), 0, 0, false);
    const Group& f = acc.way_group("forest");
    CHECK(f.all.n == 2);
    CHECK(f.all.area_m2 == 4000);
    CHECK(f.with.at("leaf_type").area_m2 == 1000);
    CHECK(acc.forest_by_tag("natural=wood").n == 2);
    CHECK(acc.forest_by_tag("landuse=forest").n == 1);
    CHECK(acc.forest_open_ways().n == 1);

    acc.add_way(tags({{"natural", "tree_row"}, {"leaf_type", "broadleaved"}}), 25'000, 0, false);
    CHECK(acc.way_group("tree_row").all.length_mm == 25'000);
    CHECK(acc.way_group("tree_row").with.at("leaf_type").n == 1);
}

TEST_CASE("tag_probe: reports are deterministic and independent of insertion order", "[osm][probe]") {
    const auto w1 = tags({{"highway", "primary"}, {"lanes", "2"}});
    const auto w2 = tags({{"highway", "residential"}, {"sidewalk", "both"}});
    const auto w3 = tags({{"highway", "motorway"}, {"lanes", "3"}, {"turn:lanes", "none|none|none"}});
    Accumulator a, b;
    a.add_way(w1, 100, 0, false);
    a.add_way(w2, 200, 0, false);
    a.add_way(w3, 300, 0, false);
    b.add_way(w3, 300, 0, false);
    b.add_way(w2, 200, 0, false);
    b.add_way(w1, 100, 0, false);
    ReportMeta meta;
    meta.region_name = "t";
    CHECK(a.to_markdown(meta) == b.to_markdown(meta));
    CHECK(a.to_json(meta) == b.to_json(meta));
    CHECK(a.to_json(meta).front() == '{');
    CHECK(a.to_markdown(meta).find("## 1. Road classes") != std::string::npos);
}

TEST_CASE("tag_probe: fixture PBF scan is consistent and reproducible", "[osm][probe][io]") {
    const std::string pbf = fixtures_dir() + "/osm/frankfurt_hoechst.osm.pbf";
    const auto specs = parse_regions(read_file(fixtures_dir() + "/regions.json"));
    REQUIRE(specs.ok());
    const RegionSpec* spec = nullptr;
    for (const auto& s : *specs)
        if (s.name == "fixture") spec = &s;
    REQUIRE(spec != nullptr);
    const auto resolved = import::resolve_region(*spec, pbf);
    REQUIRE(resolved.ok());

    Accumulator acc1, acc2;
    const auto s1 = scan_pbf(pbf, *resolved, acc1);
    const auto s2 = scan_pbf(pbf, *resolved, acc2);
    REQUIRE(s1.ok());
    REQUIRE(s2.ok());
    CHECK(*s1 == *s2);
    const ReportMeta m1 = make_meta(*resolved, *s1);
    CHECK(acc1.to_markdown(m1) == acc2.to_markdown(make_meta(*resolved, *s2)));
    CHECK(acc1.to_json(m1) == acc2.to_json(make_meta(*resolved, *s2)));

    // Cross-check against an independent unfiltered count over the same PBF.
    struct Counter final : decode::osm::PbfHandler {
        std::uint64_t ways = 0, nodes = 0, relations = 0, road_ways = 0, crossings = 0, buildings = 0;
        void on_node(const decode::osm::OsmNode& n) override {
            ++nodes;
            if (Accumulator::node_kind_mask(n.tags) & kNodeCrossing) ++crossings;
        }
        void on_way(const decode::osm::OsmWay& w) override {
            ++ways;
            if (Accumulator::road_class_index(w.tags) >= 0) ++road_ways;
            if (const auto* b = find_tag(w.tags, "building"); b != nullptr && *b != "no") ++buildings;
        }
        void on_relation(const decode::osm::OsmRelation&) override { ++relations; }
    } all;
    decode::osm::read_pbf(pbf, all);

    CHECK(s1->nodes_total == all.nodes);
    CHECK(s1->ways_total == all.ways);
    CHECK(s1->relations_total == all.relations);
    CHECK(s1->ways_in_region <= s1->ways_relevant);
    CHECK(s1->ways_relevant <= s1->ways_total);
    CHECK(s1->ways_in_region > 0);
    CHECK(s1->ways_in_region < s1->ways_relevant); // the region cut is real: the fixture PBF is larger than the region
    CHECK(s1->relations_in_region <= s1->relations_relevant);
    std::uint64_t region_road_ways = 0;
    std::int64_t region_road_mm = 0;
    for (const auto& cls : Accumulator::road_class_names()) {
        region_road_ways += acc1.road(cls).all.n;
        region_road_mm += acc1.road(cls).all.length_mm;
    }
    CHECK(region_road_ways > 0);
    CHECK(region_road_ways <= all.road_ways);
    CHECK(region_road_mm > 0);
    CHECK(acc1.node_group("crossing").all.n <= all.crossings);
    // Closed building ways plus relations in the region never exceed all building ways plus relations.
    CHECK(acc1.way_group("building").all.n <= all.buildings + s1->relations_relevant);
    // Every placement-tracked node got exactly one placement class.
    const auto pc = acc1.placement("crossing");
    CHECK(pc[0] + pc[1] + pc[2] + pc[3] == acc1.node_group("crossing").all.n);
    // The percentage columns can never exceed the totals they are shares of.
    for (const auto& cls : Accumulator::road_class_names()) {
        const auto& rc = acc1.road(cls);
        for (const auto& kv : rc.with) {
            CHECK(kv.second.n <= rc.all.n);
            CHECK(kv.second.length_mm <= rc.all.length_mm);
        }
    }
}
