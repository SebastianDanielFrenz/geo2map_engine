// Street fidelity P1 slice 2: the country road rules data file and its loader.
#include "g2m/layer/road_rules.h"
#include "g2m/layer/osm_roads.h"
#include "g2m/core/hash.h"
#include <catch2/catch_test_macros.hpp>
#include <cstring>
#include <fstream>
#include <iterator>
#include <sstream>
#include <string>

using namespace g2m;
namespace {
std::string embedded_text() {
    const auto bytes = builtin_road_rules_de_bytes();
    return std::string(reinterpret_cast<const char*>(bytes.data()), bytes.size());
}
Result<RoadRules> load_text(const std::string& text) {
    return load_road_rules({reinterpret_cast<const std::byte*>(text.data()), text.size()});
}
// Replaces the first occurrence of `from`; the test fails if the file no longer contains it.
std::string mutated(const std::string& from, const std::string& to) {
    auto text = embedded_text();
    const auto at = text.find(from);
    REQUIRE(at != std::string::npos);
    text.replace(at, from.size(), to);
    return text;
}
}

TEST_CASE("The embedded rules file equals data/road_rules/de.json and its hash is pinned", "[layer][road_rules]") {
    std::ifstream in(std::string(G2M_SOURCE_DIR) + "/data/road_rules/de.json", std::ios::binary);
    REQUIRE(in);
    const std::string file((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
    CHECK(file == embedded_text());
    // A data edit must come with a conscious edit of this value (it enters the roads.geom config hash in slice 5).
    const auto bytes = builtin_road_rules_de_bytes();
    const auto digest = Sha256::of(bytes);
    CHECK(to_hex(digest) == "4db794471a35aa96650c058a78d7e8de7ecab724fc34215a878886ea6d10bc54");
    CHECK(builtin_road_rules_de().content_sha256 == digest);
}

TEST_CASE("The built-in German rules cover every road class and reproduce today's untagged widths", "[layer][road_rules]") {
    const auto& rules = builtin_road_rules_de();
    CHECK(rules.country == "DE");
    CHECK(rules.min_lane_mm == 2500); CHECK(rules.added_lane_side == AddedLaneSide::Right);
    CHECK(rules.cycle_lane_mm == 1850); CHECK(rules.sidewalk_mm == 2500); CHECK(rules.tagged_shoulder_mm == 1500);
    CHECK(rules.parking_width_mm(ParkingOrientation::Parallel) == 2000);
    CHECK(rules.parking_width_mm(ParkingOrientation::Diagonal) == 4500);
    CHECK(rules.parking_width_mm(ParkingOrientation::Perpendicular) == 5000);
    const auto& style = RoadStyle::default_style();
    REQUIRE(rules.classes.size() == style.highways.size());
    for (std::size_t i = 1; i < rules.classes.size(); ++i) CHECK(rules.classes[i - 1].highway < rules.classes[i].highway);
    for (const auto& entry : style.highways) CHECK(rules.find_class(entry.highway) != nullptr);
    CHECK(rules.find_class("footway") == nullptr);
    // D8: an untagged two-way road keeps today's paved width (design table in section 5).
    for (const char* name : {"primary", "secondary", "tertiary", "unclassified", "residential", "living_street", "service", "track"}) {
        const auto* c = rules.find_class(name); REQUIRE(c);
        const auto* s = style.find_highway(name); REQUIRE(s);
        CHECK(c->twoway_lanes * c->lane_mm == static_cast<std::int32_t>(s->default_width_m * 1000.0 + 0.5));
    }
    const auto* link = rules.find_class("motorway_link"); REQUIRE(link);
    CHECK(link->lane_mm + link->shoulder_inner_mm + link->shoulder_outer_mm == 6000);
    const auto* motorway = rules.find_class("motorway"); REQUIRE(motorway);
    CHECK(motorway->outer_lane_mm == 3750); CHECK(motorway->lane_mm == 3500);
    CHECK(motorway->shoulder_outer_mm == 2500); CHECK(motorway->shoulder_inner_mm == 750); CHECK(motorway->verge_mm == 1500);
    CHECK(motorway->oneway_lanes == 2); CHECK(motorway->twoway_lanes == 4);
    CHECK(motorway->min_edge_radius_m == 200);
    const auto* living = rules.find_class("living_street"); REQUIRE(living);
    CHECK(living->single_lane_both); CHECK(living->single_lane_mm == 5000); CHECK(living->lane_mm == 5000);
    CHECK(living->min_edge_radius_m == 10);
    CHECK_FALSE(rules.find_class("residential")->markings_default);
}

TEST_CASE("The traffic-calming defaults of the German rules", "[layer][road_rules]") {
    const auto& rules = builtin_road_rules_de();
    CHECK(rules.calming(ProfileFeatureKind::Hump) == TrafficCalmingRules{3700, 70, 0});
    CHECK(rules.calming(ProfileFeatureKind::Table) == TrafficCalmingRules{6000, 80, 0});
    for (const auto kind : {ProfileFeatureKind::Chicane, ProfileFeatureKind::Choker, ProfileFeatureKind::Island})
        CHECK(rules.calming(kind).height_mm == 0); // lateral kinds: render only
    for (const auto kind : {ProfileFeatureKind::Bump, ProfileFeatureKind::MiniBumps, ProfileFeatureKind::RumbleStrip, ProfileFeatureKind::Cushion,
                            ProfileFeatureKind::Hump, ProfileFeatureKind::Table, ProfileFeatureKind::Dip})
        CHECK(rules.calming(kind).height_mm > 0);
}

TEST_CASE("The rules loader is strict", "[layer][road_rules]") {
    const auto good = load_text(embedded_text()); REQUIRE(good);
    CHECK(good->classes == builtin_road_rules_de().classes);
    CHECK_FALSE(load_text("not json"));
    CHECK_FALSE(load_text(""));
    CHECK_FALSE(load_text(mutated("g2m.road_rules/1", "g2m.road_rules/2")));
    CHECK_FALSE(load_text(mutated("\"country\": \"DE\",", "\"country\": \"DE\", \"bogus\": 1,")));          // unknown key
    CHECK_FALSE(load_text(mutated("\"primary_link\": {", "\"primary_link\": { \"...\": \"as primary\",")));  // abbreviation marker
    CHECK_FALSE(load_text(mutated("\"track\": {", "\"trackx\": {")));                                     // missing + unknown class
    CHECK_FALSE(load_text(mutated("\"track\": {", "\"footway\": {")));
    CHECK_FALSE(load_text(mutated("\"verge_mm\": 1500", "\"verge_mm\": 20001")));                          // width range
    CHECK_FALSE(load_text(mutated("\"verge_mm\": 1500", "\"verge_mm\": -1")));
    CHECK_FALSE(load_text(mutated("\"cycle_lane_mm\": 1850", "\"cycle_lane_mm\": 1850.5")));              // integers only
    CHECK_FALSE(load_text(mutated("\"cycle_lane_mm\": 1850", "\"cycle_lane_mm\": \"1850\"")));
    CHECK_FALSE(load_text(mutated("\"added_lane_side\": \"right\"", "\"added_lane_side\": \"middle\"")));
    CHECK_FALSE(load_text(mutated("\"shift_min_m\": 60, \"shift_max_m\": 250", "\"shift_min_m\": 300, \"shift_max_m\": 250")));
    CHECK_FALSE(load_text(mutated("\"lane_mm\": 2750", "\"lane_mm\": 2000")));                             // below min_lane_mm
    CHECK_FALSE(load_text(mutated("\"single_lane_both\": true, \"single_lane_mm\": 5000", "\"single_lane_both\": true, \"lane_mm\": 3000, \"single_lane_mm\": 5000")));
    CHECK_FALSE(load_text(mutated("\"min_edge_radius_m\": {", "\"min_edge_radius_m\": { \"footway\": 5,")));
    CHECK_FALSE(load_text(mutated("\"default\": 10,", "\"default\": 0,")));
    // traffic_calming: every kind present, integers in range, no unknown key.
    CHECK_FALSE(load_text(mutated("\"island\":       { \"length_mm\": 4000,", "\"islandx\":      { \"length_mm\": 4000,")));
    CHECK_FALSE(load_text(mutated("\"hump\":         { \"length_mm\": 3700, \"height_mm\": 70,", "\"hump\":         { \"length_mm\": 3700, \"height_mm\": 301,")));
    CHECK_FALSE(load_text(mutated("\"length_mm\": 3700", "\"length_mm\": 0")));
    CHECK_FALSE(load_text(mutated("\"length_mm\": 3700", "\"length_mm\": 3700.5")));
    CHECK_FALSE(load_text(mutated("\"length_mm\": 3700, \"height_mm\": 70, \"width_mm\": 0", "\"length_mm\": 3700, \"height_mm\": 70")));
    CHECK_FALSE(load_text(mutated("\"length_mm\": 3700, \"height_mm\": 70, \"width_mm\": 0", "\"length_mm\": 3700, \"height_mm\": 70, \"width_mm\": 0, \"carve\": true")));
    // A taper too short for the radius: motorway 3750 mm over 20 m needs R <= 20^2 / (4 * 3.75) = 26.7 m, not 200 m.
    CHECK_FALSE(load_text(mutated("\"lane_add_m\": 60, \"width_change_m\": 60", "\"lane_add_m\": 20, \"width_change_m\": 60")));
    // ... but the same taper is fine once the stated radius allows it.
    auto relaxed = mutated("\"lane_add_m\": 60, \"width_change_m\": 60", "\"lane_add_m\": 20, \"width_change_m\": 60");
    const auto at = relaxed.find("\"motorway\": 200"); REQUIRE(at != std::string::npos);
    relaxed.replace(at, std::strlen("\"motorway\": 200"), "\"motorway\": 25");
    CHECK(load_text(relaxed));
}

TEST_CASE("The junction and furniture rules of the German file", "[layer][road_rules][junction]") {
    const auto& rules = builtin_road_rules_de();
    const auto& j = rules.junction;
    CHECK(j.trim_min_mm == 1000); CHECK(j.trim_margin_mm == 500); CHECK(j.edge_station_spacing_mm == 500);
    CHECK(j.link_trim_cap_m == 30); CHECK(j.cluster_max_extent_m == 48); CHECK(j.cluster_max_nodes == 12);
    CHECK(j.reach_max_m == 128); CHECK(j.bowtie_rounds == 3); CHECK(j.bowtie_step_mm == 1000);
    // The tile-invariance proof of design 5.2 and 5.3, restated from the loaded values.
    CHECK(j.cluster_max_extent_m + 2 * j.link_trim_cap_m <= j.reach_max_m);
    CHECK(j.reach_max_m + kCarveBlendMaxM + j.cluster_max_extent_m <= kCarverRadiusM);

    const auto& f = rules.furniture;
    CHECK(f.crossing_zebra_mm == 4000); CHECK(f.crossing_signalised_mm == 4000); CHECK(f.crossing_other_mm == 3000);
    CHECK(f.stop_line_before_crossing_mm == 1000); CHECK(f.junction_stop_setback_mm == 1000); CHECK(f.sign_line_setback_mm == 500);
    CHECK(f.line_stop_mm == 500); CHECK(f.line_give_way_mm == 500); CHECK(f.line_signal_mm == 500);
    CHECK(f.signal_assoc_max_m == 60); CHECK(f.sign_assoc_max_m == 40);
    CHECK(f.signal_lateral_offset_mm == 500); CHECK(f.signal_left_min_lanes == 3);
    // crossing:markings and crossing=* tables (design 7.1 pattern rules).
    REQUIRE(f.find_marking("zebra")); CHECK(*f.find_marking("zebra") == CrossingPattern::Zebra);
    REQUIRE(f.find_marking("ladder:skewed")); CHECK(*f.find_marking("ladder:skewed") == CrossingPattern::Ladder);
    REQUIRE(f.find_marking("dots")); CHECK(*f.find_marking("dots") == CrossingPattern::Dashes);
    REQUIRE(f.find_marking("no")); CHECK(*f.find_marking("no") == CrossingPattern::None);
    CHECK(f.find_marking("yes") == nullptr);   // yes falls through to crossing=*
    CHECK(f.find_marking("bogus") == nullptr);
    REQUIRE(f.find_kind("traffic_signals")); CHECK(*f.find_kind("traffic_signals") == CrossingPattern::Dashes);
    REQUIRE(f.find_kind("uncontrolled")); CHECK(*f.find_kind("uncontrolled") == CrossingPattern::Zebra);
    REQUIRE(f.find_kind("informal")); CHECK(*f.find_kind("informal") == CrossingPattern::None);
    CHECK(f.find_kind("") == nullptr);
    for (std::size_t i = 1; i < f.crossing_markings.size(); ++i) CHECK(f.crossing_markings[i - 1].value < f.crossing_markings[i].value);
    for (std::size_t i = 1; i < f.crossing_kind.size(); ++i) CHECK(f.crossing_kind[i - 1].value < f.crossing_kind[i].value);
}

TEST_CASE("The junction and furniture blocks are strict", "[layer][road_rules][junction]") {
    // Missing block, unknown key, wrong type, range: each rejection has its own mutation (the sabotage of the loader).
    CHECK_FALSE(load_text(mutated("\"junction\": {", "\"junctionx\": {")));
    CHECK_FALSE(load_text(mutated("\"furniture\": {", "\"furniturex\": {")));
    CHECK_FALSE(load_text(mutated("\"trim_min_mm\": 1000,", "\"trim_min_mm\": 1000, \"bogus\": 1,")));
    CHECK_FALSE(load_text(mutated("\"trim_min_mm\": 1000", "\"trim_min_mm\": 0")));              // (0, 200] m
    CHECK_FALSE(load_text(mutated("\"trim_min_mm\": 1000", "\"trim_min_mm\": 200001")));
    CHECK_FALSE(load_text(mutated("\"trim_margin_mm\": 500", "\"trim_margin_mm\": 0.5")));        // integers only (mm, not decimal metres)
    CHECK_FALSE(load_text(mutated("\"edge_station_spacing_mm\": 500", "\"edge_station_spacing_mm\": \"500\"")));
    CHECK_FALSE(load_text(mutated("\"cluster_max_nodes\": 12", "\"cluster_max_nodes\": 13")));    // the encoding stores 1..12 nodes
    CHECK_FALSE(load_text(mutated("\"cluster_max_nodes\": 12", "\"cluster_max_nodes\": 0")));
    CHECK_FALSE(load_text(mutated("\"bowtie_rounds\": 3", "\"bowtie_rounds\": -1")));
    // Cap proof 48 + 2 * 30 <= 128: raising the link cap or the extent breaks it, raising the reach fixes it again.
    CHECK_FALSE(load_text(mutated("\"link_trim_cap_m\": 30", "\"link_trim_cap_m\": 41")));
    CHECK_FALSE(load_text(mutated("\"cluster_max_extent_m\": 48", "\"cluster_max_extent_m\": 69")));
    // Carver radius 128 + 15 + 48 <= 192: a reach of 130 with the same extent exceeds it.
    {
        auto text = mutated("\"reach_max_m\": 128", "\"reach_max_m\": 130");
        CHECK_FALSE(load_text(text));
        const auto reduced = [&](const std::string& from, const std::string& to) {
            auto t = text; const auto at = t.find(from); REQUIRE(at != std::string::npos); t.replace(at, from.size(), to); return t;
        };
        CHECK(load_text(reduced("\"cluster_max_extent_m\": 48", "\"cluster_max_extent_m\": 46"))); // 130 + 15 + 46 = 191 <= 192
        CHECK_FALSE(load_text(reduced("\"cluster_max_extent_m\": 48", "\"cluster_max_extent_m\": 48")));
    }
    CHECK_FALSE(load_text(mutated("\"crossing_length_mm\": { \"zebra\": 4000,", "\"crossing_length_mm\": { \"zebra\": 4000, \"bogus\": 1,")));
    CHECK_FALSE(load_text(mutated("\"zebra\": 4000", "\"zebra\": 0")));
    CHECK_FALSE(load_text(mutated("\"other\": 3000", "\"other\": 200001")));
    CHECK_FALSE(load_text(mutated("\"signal_left_min_lanes\": 3", "\"signal_left_min_lanes\": 0")));
    CHECK_FALSE(load_text(mutated("\"signal_assoc_max_m\": 60", "\"signal_assoc_max_m\": 201")));
    CHECK_FALSE(load_text(mutated("\"line_width_mm\": { \"stop\": 500,", "\"line_width_mm\": { \"stopx\": 500,")));
    CHECK_FALSE(load_text(mutated("\"sign_line_setback_mm\": 500,", "")));                          // missing key
    CHECK_FALSE(load_text(mutated("\"dots\": \"dashes\"", "\"dots\": \"stripes\"")));              // unknown pattern name
    CHECK_FALSE(load_text(mutated("\"dots\": \"dashes\"", "\"dots\": 2")));
    CHECK_FALSE(load_text(mutated("\"dots\": \"dashes\"", "\"yes\": \"dashes\"")));                // yes is not a key
    CHECK_FALSE(load_text(mutated("\"unmarked\": \"none\", \"no\": \"none\", \"informal\": \"none\"", "")));  // trailing comma left: invalid JSON
    // A table must not be empty.
    {
        auto text = embedded_text();
        const auto from = text.find("\"crossing_kind\": {"); REQUIRE(from != std::string::npos);
        const auto to = text.find("}", from); REQUIRE(to != std::string::npos);
        text.replace(from, to - from + 1, "\"crossing_kind\": {}");
        CHECK_FALSE(load_text(text));
    }
    // The shipped blocks load and the rest of the file is unchanged by them.
    const auto good = load_text(embedded_text()); REQUIRE(good);
    CHECK(good->junction == builtin_road_rules_de().junction);
    CHECK(good->furniture == builtin_road_rules_de().furniture);
}

TEST_CASE("The tunnel block: the straight-grade threshold of long tunnels (owner ruling 2026-10-15)", "[layer][road_rules][tunnel]") {
    CHECK(builtin_road_rules_de().tunnel.straight_grade_min_length_m == 200);
    // Only the key/value pair is mutated, so the block's other keys (cut_*) may change without touching this test.
    const std::string key = "\"straight_grade_min_length_m\": 200";
    CHECK_FALSE(load_text(mutated("\"tunnel\": {", "\"tunnelx\": {")));                                       // the block is required
    CHECK_FALSE(load_text(mutated(key + ", ", "")));                                                         // and its key
    CHECK_FALSE(load_text(mutated(key, key + ", \"bogus\": 1")));
    CHECK_FALSE(load_text(mutated(key, "\"straight_grade_min_length_m\": -1")));
    CHECK_FALSE(load_text(mutated(key, "\"straight_grade_min_length_m\": 100001")));
    CHECK_FALSE(load_text(mutated(key, "\"straight_grade_min_length_m\": 200.5")));                          // integer metres
    const auto never = load_text(mutated(key, "\"straight_grade_min_length_m\": 0"));                        // 0 = never
    REQUIRE(never); CHECK(never->tunnel.straight_grade_min_length_m == 0);
    const auto other = load_text(mutated(key, "\"straight_grade_min_length_m\": 350"));
    REQUIRE(other); CHECK(other->tunnel.straight_grade_min_length_m == 350);
    CHECK(other->junction == builtin_road_rules_de().junction);
}

TEST_CASE("The underpass block: the deck rules of untagged grade-separated crossings (D53-D58)", "[layer][road_rules][underpass]") {
    const auto& u = builtin_road_rules_de().underpass;
    CHECK(u.enabled);
    CHECK(u.min_depth_mm == 1000); CHECK(u.window_m == 50); CHECK(u.sag_radius_m == 300); CHECK(u.max_span_m == 80);
    CHECK(u.abutment_mm == 2000); CHECK(u.depth_tol_mm == 100); CHECK(u.lower_margin_mm == 1000); CHECK(u.end_clearance_mm == 1000);
    const std::string from = "\"underpass\": { \"enabled\": true,";
    CHECK_FALSE(load_text(mutated(from, "\"underpassx\": { \"enabled\": true,")));     // the block is required
    CHECK_FALSE(load_text(mutated(from, "\"underpass\": { \"enabled\": 1,")));         // a boolean
    CHECK_FALSE(load_text(mutated(from, "\"underpass\": { \"bogus\": 1, \"enabled\": true,")));
    CHECK_FALSE(load_text(mutated("\"window_m\": 50", "\"window_m\": 61")));           // the 65 m DEM margin
    CHECK_FALSE(load_text(mutated("\"window_m\": 50", "\"window_m\": 50.5")));
    CHECK_FALSE(load_text(mutated("\"max_span_m\": 80", "\"max_span_m\": 0")));
    CHECK_FALSE(load_text(mutated("\"depth_tol_mm\": 100", "\"depth_tol_mm\": 1000")));   // below min_depth_mm
    const auto off = load_text(mutated(from, "\"underpass\": { \"enabled\": false,"));
    REQUIRE(off); CHECK_FALSE(off->underpass.enabled);
    CHECK(off->tunnel == builtin_road_rules_de().tunnel);
}

TEST_CASE("The reference block: the chord bound of the reference lines (G2M-024)", "[layer][road_rules][reference]") {
    const auto& r = builtin_road_rules_de().reference;
    CHECK(r.chord_half_width_percent == 100); CHECK(r.chord_margin_mm == 3000);
    CHECK(embedded_text().find("\"reference\": \"Vault G2M-024") != std::string::npos);   // the source tag
    const std::string from = "\"reference\": { \"chord_half_width_percent\": 100, \"chord_margin_mm\": 3000 }";
    CHECK_FALSE(load_text(mutated(from, "\"referencex\": { \"chord_half_width_percent\": 100, \"chord_margin_mm\": 3000 }")));  // required
    CHECK_FALSE(load_text(mutated(from, "\"reference\": { \"chord_half_width_percent\": 100 }")));                         // both keys
    CHECK_FALSE(load_text(mutated(from, "\"reference\": { \"chord_half_width_percent\": 100, \"chord_margin_mm\": 3000, \"bogus\": 1 }")));
    CHECK_FALSE(load_text(mutated(from, "\"reference\": { \"chord_half_width_percent\": -1, \"chord_margin_mm\": 3000 }")));
    CHECK_FALSE(load_text(mutated(from, "\"reference\": { \"chord_half_width_percent\": 1001, \"chord_margin_mm\": 3000 }")));
    CHECK_FALSE(load_text(mutated(from, "\"reference\": { \"chord_half_width_percent\": 100, \"chord_margin_mm\": 100001 }")));
    CHECK_FALSE(load_text(mutated(from, "\"reference\": { \"chord_half_width_percent\": 100, \"chord_margin_mm\": 3000.5 }")));  // integers only
    const auto off = load_text(mutated(from, "\"reference\": { \"chord_half_width_percent\": 0, \"chord_margin_mm\": 0 }"));
    REQUIRE(off); CHECK(off->reference.chord_half_width_percent == 0); CHECK(off->reference.chord_margin_mm == 0);   // 0 and 0 = off
    CHECK(off->underpass == builtin_road_rules_de().underpass);
}

TEST_CASE("The merge_lane block: angles, landing rate and bounds of the lane-add landing (G2M-028)", "[layer][road_rules][road_merge_lane]") {
    const auto& ml = builtin_road_rules_de().merge_lane;
    CHECK(ml.enabled);
    CHECK(ml.max_through_deflection_deg == 35); CHECK(ml.max_ramp_angle_deg == 60); CHECK(ml.direction_probe_m == 10);
    CHECK(ml.landing_m_per_m == 12); CHECK(ml.landing_min_m == 30); CHECK(ml.landing_max_m == 80); CHECK(ml.max_landing_offset_mm == 8000);
    const std::string from = "\"merge_lane\": { \"enabled\": true,";
    CHECK_FALSE(load_text(mutated(from, "\"merge_lanex\": { \"enabled\": true,")));        // the block is required
    CHECK_FALSE(load_text(mutated(from, "\"merge_lane\": { \"enabled\": 1,")));            // a boolean
    CHECK_FALSE(load_text(mutated(from, "\"merge_lane\": { \"bogus\": 1, \"enabled\": true,")));
    CHECK_FALSE(load_text(mutated("\"max_through_deflection_deg\": 35", "\"max_through_deflection_deg\": 0")));
    CHECK_FALSE(load_text(mutated("\"max_through_deflection_deg\": 35", "\"max_through_deflection_deg\": 90")));
    CHECK_FALSE(load_text(mutated("\"max_ramp_angle_deg\": 60", "\"max_ramp_angle_deg\": 90")));
    CHECK_FALSE(load_text(mutated("\"max_ramp_angle_deg\": 60", "\"max_ramp_angle_deg\": 60.5")));   // integer degrees
    CHECK_FALSE(load_text(mutated("\"direction_probe_m\": 10", "\"direction_probe_m\": 0")));
    CHECK_FALSE(load_text(mutated("\"landing_m_per_m\": 12", "\"landing_m_per_m\": 1001")));
    CHECK_FALSE(load_text(mutated("\"landing_min_m\": 30", "\"landing_min_m\": 81")));             // above landing_max_m
    CHECK_FALSE(load_text(mutated("\"max_landing_offset_mm\": 8000", "\"max_landing_offset_mm\": 20001")));
    // The landing S-ramp must not be sharper than min_edge_radius_m of the link classes: 4 W / a^2 <= 1 / R.
    CHECK_FALSE(load_text(mutated("\"landing_m_per_m\": 12", "\"landing_m_per_m\": 1")));          // 8 m over 30 m: radius far under 60 m
    const auto off = load_text(mutated(from, "\"merge_lane\": { \"enabled\": false,"));
    REQUIRE(off); CHECK_FALSE(off->merge_lane.enabled);
    CHECK(off->underpass == builtin_road_rules_de().underpass);
    // A disabled block is not checked against the curvature bound (nothing lands).
    auto sharp = mutated(from, "\"merge_lane\": { \"enabled\": false,");
    const std::string rate = "\"landing_m_per_m\": 12";
    const auto at = sharp.find(rate);
    REQUIRE(at != std::string::npos);
    sharp.replace(at, rate.size(), "\"landing_m_per_m\": 1");
    CHECK(load_text(sharp));
}

TEST_CASE("The structure block of the German rules (terrain migration phase 7a-2)", "[layer][road_rules][structure]") {
    const auto& s = builtin_road_rules_de().structure;
    CHECK(s.clearance_mm[static_cast<std::size_t>(StructureClass::Road)] == 4700);
    CHECK(s.clearance_mm[static_cast<std::size_t>(StructureClass::Rail)] == 6000);
    CHECK(s.clearance_mm[static_cast<std::size_t>(StructureClass::Subway)] == 4500);
    CHECK(s.max_grade_per_mille[static_cast<std::size_t>(StructureClass::Road)] == 50);
    CHECK(s.max_grade_per_mille[static_cast<std::size_t>(StructureClass::Rail)] == 40);
    CHECK(s.max_grade_per_mille[static_cast<std::size_t>(StructureClass::Tram)] == 50);
    CHECK(s.roof_mm == 1000); CHECK(s.min_tube_m == 10); CHECK(s.min_gap_m == 10);
    CHECK(s.bored_min_length_m == 300); CHECK(s.bored_min_cover_m == 15);
    CHECK(s.box_wall_mm == 800); CHECK(s.bored_lining_mm == 500); CHECK(s.box_side_margin_mm == 1000); CHECK(s.bored_segments == 32);
    // The classification thresholds are the library defaults for a road (the unit tests' Thresholds).
    const auto t = s.thresholds(StructureClass::Road);
    CHECK(t.clearance_m == 4.7); CHECK(t.roof_m == 1.0); CHECK(t.max_grade == 0.05); CHECK(t.cover_needed() == 5.7);
    CHECK(s.thresholds(StructureClass::Rail).max_grade == 0.04);
    // Strict loader: unknown/missing keys, a missing class, non-integers and out-of-range values are rejected.
    CHECK_FALSE(load_text(mutated("\"roof_mm\": 1000,", "\"roof_mm\": 1000, \"bogus\": 1,")));
    CHECK_FALSE(load_text(mutated("\"roof_mm\": 1000,", "")));
    CHECK_FALSE(load_text(mutated("\"narrow_gauge\": 4500 },\n    \"roof_mm\"", "\"narrow_gaugex\": 4500 },\n    \"roof_mm\"")));
    CHECK_FALSE(load_text(mutated("\"roof_mm\": 1000", "\"roof_mm\": 1000.5")));
    CHECK_FALSE(load_text(mutated("\"roof_mm\": 1000", "\"roof_mm\": 0")));
    CHECK_FALSE(load_text(mutated("\"bored_segments\": 32", "\"bored_segments\": 4")));
    CHECK_FALSE(load_text(mutated("\"road\": 50, \"rail\": 40", "\"road\": 0, \"rail\": 40")));
}
