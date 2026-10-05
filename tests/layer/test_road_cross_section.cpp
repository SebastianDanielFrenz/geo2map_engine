// Street fidelity P1 slice 3: resolve_cross_section_base (tags -> bands, design section 3) and the
// evaluation of a stored cross-section (design section 2.1).
#include "road_fixture.h"
#include "g2m/layer/road_cross_section.h"
#include <catch2/catch_test_macros.hpp>
#include <algorithm>
#include <cmath>
#include <initializer_list>
#include <string>
#include <utility>
using namespace g2m;
namespace {
const RoadRules& rules() { return builtin_road_rules_de(); }

RoadGraphWay make_way(std::initializer_list<std::pair<const char*, const char*>> tags) {
    RoadGraphWay w; w.osm_id = 1; w.rank = 3; w.node_ids = {1, 2};
    for (const auto& [k, v] : tags) w.tags.emplace_back(k, v);
    std::sort(w.tags.begin(), w.tags.end());
    return w;
}
RoadCrossSection resolve(std::initializer_list<std::pair<const char*, const char*>> tags) {
    const auto base = resolve_cross_section_base(make_way(tags), rules());
    REQUIRE(base);
    REQUIRE(base->xs.has_value());
    return *base->xs;
}
std::vector<LaneKind> kinds(const RoadCrossSection& xs) {
    std::vector<LaneKind> k; for (const auto& b : xs.bands) k.push_back(b.kind); return k;
}
std::vector<std::int32_t> widths(const RoadCrossSection& xs) {
    std::vector<std::int32_t> k; for (const auto& b : xs.bands) k.push_back(b.width_mm); return k;
}
std::size_t counted(const RoadCrossSection& xs) {
    return static_cast<std::size_t>(std::count_if(xs.bands.begin(), xs.bands.end(), [](const auto& b) { return b.counted; }));
}
std::size_t first_counted(const RoadCrossSection& xs) {
    for (std::size_t i = 0; i < xs.bands.size(); ++i) if (xs.bands[i].counted) return i;
    return xs.bands.size();
}
using LK = LaneKind;
std::ptrdiff_t count_kind(const RoadCrossSection& xs, LaneKind kind) {
    return std::count_if(xs.bands.begin(), xs.bands.end(), [&](const auto& b) { return b.kind == kind; });
}
} // namespace

TEST_CASE("The smooth step is exact at 0, 1/2 and 1 and C1", "[layer][road_cross_section][roads]") {
    CHECK(road_smooth_step(-1.0) == 0.0); CHECK(road_smooth_step(0.0) == 0.0);
    CHECK(road_smooth_step(0.5) == 0.5);
    CHECK(road_smooth_step(1.0) == 1.0); CHECK(road_smooth_step(7.0) == 1.0);
    CHECK(road_smooth_step(0.25) == 0.125); CHECK(road_smooth_step(0.75) == 0.875);
    // Both pieces meet with the same slope (2u * 2 = 2 at 1/2 from the left, 4(1-u) = 2 from the right).
    const double e = 1e-6;
    CHECK(std::fabs((road_smooth_step(0.5 + e) - road_smooth_step(0.5 - e)) / (2 * e) - 2.0) < 1e-4);
    double prev = 0.0;
    for (int i = 0; i <= 1000; ++i) { const double v = road_smooth_step(i / 1000.0); CHECK(v >= prev); prev = v; }
}

TEST_CASE("Untagged roads reproduce the class rules", "[layer][road_cross_section][roads]") {
    {   // residential: verge | 2.75 | 2.75 | verge, centre line between the lanes, anchor at its right edge
        const auto xs = resolve({{"highway", "residential"}});
        CHECK(kinds(xs) == std::vector<LK>{LK::Verge, LK::Driving, LK::Driving, LK::Verge});
        CHECK(widths(xs) == std::vector<std::int32_t>{500, 2750, 2750, 500});
        CHECK(xs.bands[1].direction == LaneDirection::Backward); CHECK(xs.bands[2].direction == LaneDirection::Forward);
        CHECK(xs.anchor.band == 1); CHECK(xs.anchor.edge == AnchorEdge::Right); CHECK(xs.anchor.source == PlacementSource::Absent);
        CHECK(xs.count_provenance == Provenance::Inferred); CHECK(xs.placement_provenance == Provenance::Inferred);
        CHECK(xs.bands[1].provenance.width == Provenance::Inferred);
        CHECK(xs.bands[1].provenance.direction == Provenance::Inferred);
        CHECK_FALSE(xs.bands[1].marked_right);          // markings_default false
        CHECK_FALSE(xs.bands[0].paved); CHECK(xs.bands[1].paved);
        CHECK((xs.flags & kXsOneway) == 0); CHECK(xs.field_declines == 0);
        CHECK(xs.bands[1].width_mm + xs.bands[2].width_mm == 5500); // today's default width
    }
    {   // primary: two 3.5 m lanes, no shoulder, 1.5 m verges, marked
        const auto xs = resolve({{"highway", "primary"}});
        CHECK(widths(xs) == std::vector<std::int32_t>{1500, 3500, 3500, 1500});
        CHECK(xs.bands[1].marked_right); CHECK_FALSE(xs.bands[2].marked_right);
    }
    {   // living street: one unmarked Both lane of 5 m
        const auto xs = resolve({{"highway", "living_street"}});
        CHECK(counted(xs) == 1);
        const auto& lane = xs.bands[first_counted(xs)];
        CHECK(lane.direction == LaneDirection::Both); CHECK(lane.width_mm == 5000);
        CHECK(lane.provenance.direction == Provenance::Derived);
        CHECK(xs.anchor.band == first_counted(xs)); CHECK(xs.anchor.edge == AnchorEdge::Centre);
    }
    {   // motorway: implicitly oneway, 3.5 + 3.75 outer, inner and outer shoulders, verges
        const auto xs = resolve({{"highway", "motorway"}});
        CHECK(kinds(xs) == std::vector<LK>{LK::Verge, LK::Shoulder, LK::Driving, LK::Driving, LK::Shoulder, LK::Verge});
        CHECK(widths(xs) == std::vector<std::int32_t>{1500, 750, 3500, 3750, 2500, 1500});
        CHECK((xs.flags & kXsOneway) != 0);
        CHECK(xs.bands[2].direction == LaneDirection::Forward);
        CHECK(xs.anchor.band == 2); CHECK(xs.anchor.edge == AnchorEdge::Right);
        CHECK(xs.bands[1].paved); CHECK_FALSE(xs.bands[0].paved);
    }
    {   // motorway_link: lane 3.5, inner 1.0 and outer 1.5 paved shoulder = 6 m
        const auto xs = resolve({{"highway", "motorway_link"}});
        std::int32_t paved = 0; for (const auto& b : xs.bands) if (b.paved) paved += b.width_mm;
        CHECK(paved == 6000);
    }
    {   // a track is unpaved throughout, and a surface tag turns it paved
        const auto xs = resolve({{"highway", "track"}});
        for (const auto& b : xs.bands) CHECK_FALSE(b.paved);
        const auto paved = resolve({{"highway", "track"}, {"surface", "asphalt"}});
        CHECK(paved.bands[first_counted(paved)].paved);
    }
}

TEST_CASE("Direction, oneway and the lane count", "[layer][road_cross_section][roads]") {
    {   const auto xs = resolve({{"highway", "residential"}, {"oneway", "yes"}});
        CHECK(counted(xs) == 1); CHECK((xs.flags & kXsOneway) != 0);
        CHECK(xs.bands[first_counted(xs)].direction == LaneDirection::Forward);
        CHECK(xs.bands[first_counted(xs)].provenance.direction == Provenance::Tagged);
    }
    {   const auto xs = resolve({{"highway", "residential"}, {"oneway", "-1"}, {"lanes", "2"}});
        CHECK((xs.flags & (kXsOneway | kXsReversedOneway)) == (kXsOneway | kXsReversedOneway));
        CHECK(counted(xs) == 2);
        for (const auto& b : xs.bands) if (b.counted) CHECK(b.direction == LaneDirection::Backward);
        CHECK(xs.count_provenance == Provenance::Tagged);
    }
    {   // oneway=no overrides the implicit oneway of a motorway and of a roundabout
        const auto xs = resolve({{"highway", "motorway"}, {"oneway", "no"}});
        CHECK((xs.flags & kXsOneway) == 0); CHECK(counted(xs) == 4);
        const auto ring = resolve({{"highway", "residential"}, {"junction", "roundabout"}});
        CHECK((ring.flags & kXsOneway) != 0);
    }
    {   // lanes + forward + backward agree: all Tagged
        const auto xs = resolve({{"highway", "tertiary"}, {"lanes", "3"}, {"lanes:forward", "2"}, {"lanes:backward", "1"}});
        const auto f = first_counted(xs);
        CHECK(counted(xs) == 3);
        CHECK(xs.bands[f].direction == LaneDirection::Backward);
        CHECK(xs.bands[f + 1].direction == LaneDirection::Forward); CHECK(xs.bands[f + 2].direction == LaneDirection::Forward);
        CHECK(xs.count_provenance == Provenance::Tagged); CHECK(xs.bands[f].provenance.direction == Provenance::Tagged);
        CHECK(xs.field_declines == 0);
        CHECK(xs.anchor.band == f + 1); CHECK(xs.anchor.edge == AnchorEdge::Centre); // odd count: middle lane
    }
    {   // lanes only: backward gets the floor
        const auto xs = resolve({{"highway", "tertiary"}, {"lanes", "3"}});
        const auto f = first_counted(xs);
        CHECK(xs.bands[f].direction == LaneDirection::Backward); CHECK(xs.bands[f + 1].direction == LaneDirection::Forward);
        CHECK(xs.bands[f].provenance.direction == Provenance::Inferred);
    }
    {   // lanes + one direction: the other is derived
        const auto xs = resolve({{"highway", "tertiary"}, {"lanes", "3"}, {"lanes:forward", "1"}});
        const auto f = first_counted(xs);
        CHECK(xs.bands[f].direction == LaneDirection::Backward); CHECK(xs.bands[f + 1].direction == LaneDirection::Backward);
        CHECK(xs.bands[f + 2].direction == LaneDirection::Forward);
        CHECK(xs.bands[f].provenance.direction == Provenance::Derived);
    }
    {   // no lanes, only the direction counts: their sum, Derived
        const auto xs = resolve({{"highway", "tertiary"}, {"lanes:forward", "2"}, {"lanes:backward", "1"}});
        CHECK(counted(xs) == 3); CHECK(xs.count_provenance == Provenance::Derived);
    }
    {   // a both_ways centre lane sits between the groups
        const auto xs = resolve({{"highway", "tertiary"}, {"lanes", "3"}, {"lanes:forward", "1"}, {"lanes:backward", "1"}, {"lanes:both_ways", "1"}});
        const auto f = first_counted(xs);
        CHECK(xs.bands[f].direction == LaneDirection::Backward); CHECK(xs.bands[f + 1].direction == LaneDirection::Both);
        CHECK(xs.bands[f + 2].direction == LaneDirection::Forward);
    }
    {   // a mismatch declines the field and falls back to the floor split
        const auto xs = resolve({{"highway", "tertiary"}, {"lanes", "4"}, {"lanes:forward", "1"}, {"lanes:backward", "1"}});
        CHECK((xs.field_declines & kDeclineDirectionCounts) != 0);
        const auto f = first_counted(xs);
        CHECK(counted(xs) == 4);
        CHECK(xs.bands[f + 1].direction == LaneDirection::Backward); CHECK(xs.bands[f + 2].direction == LaneDirection::Forward);
        CHECK(xs.bands[f].provenance.direction == Provenance::Inferred);
    }
    {   // two-way lanes=1 is a single Both lane; lanes:forward=1 alone is a oneway-like Forward lane
        const auto xs = resolve({{"highway", "primary"}, {"lanes", "1"}});
        CHECK(counted(xs) == 1); CHECK(xs.bands[first_counted(xs)].direction == LaneDirection::Both);
    }
    {   // a bad lanes value declines and the class default applies
        const auto xs = resolve({{"highway", "primary"}, {"lanes", "many"}});
        CHECK((xs.field_declines & kDeclineLanesValue) != 0); CHECK(counted(xs) == 2);
        const auto many = resolve({{"highway", "primary"}, {"lanes", "17"}});
        CHECK((many.field_declines & kDeclineLanesValue) != 0);
        const auto dir = resolve({{"highway", "primary"}, {"lanes:forward", "x"}});
        CHECK((dir.field_declines & kDeclineDirectionCounts) != 0); CHECK(counted(dir) == 2);
    }
    {   // placement default: even counts anchor at the right edge of lane n/2
        const auto four = resolve({{"highway", "motorway"}, {"oneway", "yes"}, {"lanes", "4"}});
        const auto f = first_counted(four);
        CHECK(four.anchor.band == f + 1); CHECK(four.anchor.edge == AnchorEdge::Right);
        const auto three = resolve({{"highway", "motorway"}, {"lanes", "3"}});
        CHECK(three.anchor.band == first_counted(three) + 1); CHECK(three.anchor.edge == AnchorEdge::Centre);
    }
}

TEST_CASE("Turn, change and bus tokens", "[layer][road_cross_section][roads]") {
    {   // forward group in way order, backward group REVERSED into way order
        const auto xs = resolve({{"highway", "tertiary"}, {"lanes", "4"}, {"lanes:forward", "2"}, {"lanes:backward", "2"},
            {"turn:lanes:forward", "left|through;right"}, {"turn:lanes:backward", "left|through"}});
        const auto f = first_counted(xs);
        // travel order backward: "left" is the travel-left lane = way-right = band f+1.
        CHECK(xs.bands[f].turn == kTurnThrough); CHECK(xs.bands[f + 1].turn == kTurnLeft);
        CHECK(xs.bands[f + 2].turn == kTurnLeft); CHECK(xs.bands[f + 3].turn == (kTurnThrough | kTurnRight));
        CHECK(xs.bands[f + 1].kind == LaneKind::Turn); CHECK(xs.bands[f].kind == LaneKind::Driving);
        CHECK(xs.bands[f + 3].kind == LaneKind::Driving); // through is among its tokens
        CHECK(xs.bands[f + 2].provenance.kind == Provenance::Tagged);
        CHECK(xs.bands[f].provenance.turn == Provenance::Tagged);
        CHECK(xs.field_declines == 0);
    }
    {   // oneway: turn:lanes in travel order; empty and "none" mean none
        const auto xs = resolve({{"highway", "primary"}, {"oneway", "yes"}, {"lanes", "3"}, {"turn:lanes", "left||slight_right"}});
        const auto f = first_counted(xs);
        CHECK(xs.bands[f].turn == kTurnLeft); CHECK(xs.bands[f + 1].turn == kTurnNone); CHECK(xs.bands[f + 2].turn == kTurnSlightRight);
        CHECK(xs.bands[f + 1].kind == LaneKind::Driving); CHECK(xs.bands[f + 2].kind == LaneKind::Turn);
    }
    {   // a reversed oneway lists the lanes in travel order, so against way order
        const auto xs = resolve({{"highway", "primary"}, {"oneway", "-1"}, {"lanes", "2"}, {"turn:lanes", "left|right"}});
        const auto f = first_counted(xs);
        CHECK(xs.bands[f].turn == kTurnRight); CHECK(xs.bands[f + 1].turn == kTurnLeft);
    }
    {   // merge-only lanes stay driving lanes
        const auto xs = resolve({{"highway", "primary"}, {"oneway", "yes"}, {"lanes", "3"}, {"turn:lanes", "none|none|merge_to_left"}});
        const auto f = first_counted(xs);
        CHECK(xs.bands[f + 2].turn == kTurnMergeToLeft); CHECK(xs.bands[f + 2].kind == LaneKind::Driving);
    }
    {   // count mismatch and unknown token decline their field
        const auto count = resolve({{"highway", "primary"}, {"oneway", "yes"}, {"lanes", "3"}, {"turn:lanes", "left|right"}});
        CHECK((count.field_declines & kDeclineTurnCount) != 0);
        for (const auto& b : count.bands) CHECK(b.turn == 0);
        const auto token = resolve({{"highway", "primary"}, {"oneway", "yes"}, {"lanes", "2"}, {"turn:lanes", "left|wiggle"}});
        CHECK((token.field_declines & kDeclineTurnToken) != 0);
        const auto f = first_counted(token);
        CHECK(token.bands[f].turn == kTurnLeft); CHECK(token.bands[f + 1].turn == 0);
    }
    {   // change tokens
        const auto xs = resolve({{"highway", "primary"}, {"oneway", "yes"}, {"lanes", "3"}, {"change:lanes", "not_right|yes|not_left"}});
        const auto f = first_counted(xs);
        CHECK(xs.bands[f].change == LaneChange::NotRight); CHECK(xs.bands[f + 1].change == LaneChange::Yes);
        CHECK(xs.bands[f + 2].change == LaneChange::NotLeft); CHECK(xs.bands[f].provenance.change == Provenance::Tagged);
        const auto bad = resolve({{"highway", "primary"}, {"oneway", "yes"}, {"lanes", "2"}, {"change:lanes", "yes|maybe"}});
        CHECK((bad.field_declines & kDeclineChangeCount) != 0);
        const auto count = resolve({{"highway", "primary"}, {"oneway", "yes"}, {"lanes", "2"}, {"change:lanes", "yes"}});
        CHECK((count.field_declines & kDeclineChangeCount) != 0);
    }
    {   // bus lanes: per-lane list, and the outermost-lane count form
        const auto list = resolve({{"highway", "primary"}, {"oneway", "yes"}, {"lanes", "2"}, {"bus:lanes", "|designated"}});
        const auto f = first_counted(list);
        CHECK(list.bands[f].kind == LaneKind::Driving); CHECK(list.bands[f + 1].kind == LaneKind::Bus);
        CHECK(list.bands[f + 1].provenance.kind == Provenance::Tagged);
        const auto count = resolve({{"highway", "primary"}, {"lanes", "4"}, {"lanes:forward", "2"}, {"lanes:backward", "2"}, {"lanes:bus:forward", "1"}});
        const auto g = first_counted(count);
        CHECK(count.bands[g + 3].kind == LaneKind::Bus); CHECK(count.bands[g + 2].kind == LaneKind::Driving);
        CHECK(count.bands[g + 3].provenance.kind == Provenance::Derived);
        const auto back = resolve({{"highway", "primary"}, {"lanes", "4"}, {"lanes:forward", "2"}, {"lanes:backward", "2"}, {"psv:lanes:backward", "designated|"}});
        CHECK(back.bands[g + 1].kind == LaneKind::Bus); CHECK(back.bands[g].kind == LaneKind::Driving); // travel-first = way-last of the group
        const auto bad = resolve({{"highway", "primary"}, {"oneway", "yes"}, {"lanes", "2"}, {"bus:lanes", "designated"}});
        CHECK((bad.field_declines & kDeclineBusLanesCount) != 0);
        const auto bad_count = resolve({{"highway", "primary"}, {"oneway", "yes"}, {"lanes", "2"}, {"lanes:bus", "5"}});
        CHECK((bad_count.field_declines & kDeclineBusLanesCount) != 0);
    }
}

TEST_CASE("Widths of counted lanes", "[layer][road_cross_section][roads]") {
    {   // width is the carriageway, split equally
        const auto xs = resolve({{"highway", "residential"}, {"width", "7"}});
        const auto f = first_counted(xs);
        CHECK(xs.bands[f].width_mm == 3500); CHECK(xs.bands[f + 1].width_mm == 3500);
        CHECK(xs.bands[f].provenance.width == Provenance::Derived);
    }
    {   // the remainder goes to the rightmost lanes, 1 mm each
        const auto xs = resolve({{"highway", "residential"}, {"width", "7.001"}});
        const auto f = first_counted(xs);
        CHECK(xs.bands[f].width_mm == 3500); CHECK(xs.bands[f + 1].width_mm == 3501);
        const auto three = resolve({{"highway", "residential"}, {"lanes", "3"}, {"width", "9.002"}});
        const auto g = first_counted(three);
        CHECK(three.bands[g].width_mm == 3000); CHECK(three.bands[g + 1].width_mm == 3001); CHECK(three.bands[g + 2].width_mm == 3001);
    }
    {   // on-carriageway cycle and parking bands come off the carriageway first
        const auto xs = resolve({{"highway", "residential"}, {"width", "9"}, {"cycleway", "lane"}});
        const auto f = first_counted(xs);
        CHECK(xs.bands[f].width_mm == 2650); CHECK(xs.bands[f + 1].width_mm == 2650);
    }
    {   // a width too small for the lanes declines as inconsistent and falls back to the rules
        const auto xs = resolve({{"highway", "residential"}, {"width", "4"}});
        CHECK((xs.field_declines & kDeclineWidthInconsistent) != 0);
        CHECK(xs.bands[first_counted(xs)].width_mm == 2750);
        CHECK(xs.bands[first_counted(xs)].provenance.width == Provenance::Inferred);
        const auto bad = resolve({{"highway", "residential"}, {"width", "wide"}});
        CHECK((bad.field_declines & kDeclineWidthValue) != 0);
    }
    {   // width:lanes is per lane, in travel order for the backward group
        const auto xs = resolve({{"highway", "tertiary"}, {"lanes", "3"}, {"lanes:forward", "2"}, {"lanes:backward", "1"},
            {"width:lanes:forward", "3.0|3.5"}, {"width:lanes:backward", "3.25"}});
        const auto f = first_counted(xs);
        CHECK(xs.bands[f].width_mm == 3250); CHECK(xs.bands[f + 1].width_mm == 3000); CHECK(xs.bands[f + 2].width_mm == 3500);
        CHECK(xs.bands[f].provenance.width == Provenance::Tagged);
        const auto rev = resolve({{"highway", "tertiary"}, {"lanes", "4"}, {"lanes:forward", "2"}, {"lanes:backward", "2"},
            {"width:lanes:backward", "3.0|3.5"}});
        const auto g = first_counted(rev);
        CHECK(rev.bands[g].width_mm == 3500); CHECK(rev.bands[g + 1].width_mm == 3000); // way order = reversed travel order
        CHECK(rev.bands[g + 2].width_mm == 3000);                                          // forward: rules
    }
    {   // a mixed tagged/untagged case: width fills only the lanes without width:lanes
        const auto xs = resolve({{"highway", "residential"}, {"oneway", "yes"}, {"lanes", "2"}, {"width:lanes", "3.0|"}, {"width", "8"}});
        const auto f = first_counted(xs);
        CHECK(xs.bands[f].width_mm == 3000); CHECK(xs.bands[f + 1].width_mm == 5000);
    }
    {   // mismatched or bad lists decline
        const auto count = resolve({{"highway", "residential"}, {"oneway", "yes"}, {"lanes", "2"}, {"width:lanes", "3"}});
        CHECK((count.field_declines & kDeclineWidthLanesCount) != 0);
        const auto bad = resolve({{"highway", "residential"}, {"oneway", "yes"}, {"lanes", "2"}, {"width:lanes", "3|x"}});
        CHECK((bad.field_declines & kDeclineWidthValue) != 0);
        CHECK(bad.bands[first_counted(bad)].width_mm == 2750);
    }
    {   // the outer lane of a oneway motorway is wider; with lanes=1 it is the only lane
        const auto xs = resolve({{"highway", "motorway"}, {"lanes", "3"}});
        const auto f = first_counted(xs);
        CHECK(xs.bands[f].width_mm == 3500); CHECK(xs.bands[f + 1].width_mm == 3500); CHECK(xs.bands[f + 2].width_mm == 3750);
        const auto rev = resolve({{"highway", "motorway"}, {"oneway", "-1"}, {"lanes", "2"}});
        CHECK(rev.bands[first_counted(rev)].width_mm == 3750); // travel-right is way-left
    }
    {   // single-lane classes spread their width over tagged lanes, never below the minimum lane width
        const auto xs = resolve({{"highway", "living_street"}, {"oneway", "yes"}, {"lanes", "2"}});
        CHECK(xs.bands[first_counted(xs)].width_mm == 2500);
    }
}

TEST_CASE("Non-counted bands: cycle, parking, shoulder, sidewalk, verge", "[layer][road_cross_section][roads]") {
    {   // cycle lanes on both sides of a two-way road; sidewalks replace the verge
        const auto xs = resolve({{"highway", "residential"}, {"cycleway", "lane"}, {"sidewalk", "both"}});
        CHECK(kinds(xs) == std::vector<LK>{LK::Sidewalk, LK::Cycle, LK::Driving, LK::Driving, LK::Cycle, LK::Sidewalk});
        CHECK(widths(xs) == std::vector<std::int32_t>{2500, 1850, 2750, 2750, 1850, 2500});
        CHECK(xs.bands[1].direction == LaneDirection::Backward); CHECK(xs.bands[4].direction == LaneDirection::Forward);
        CHECK(xs.bands[0].provenance.kind == Provenance::Tagged); CHECK(xs.bands[0].direction == LaneDirection::None);
        CHECK(xs.anchor.band == 2);
    }
    {   // on a oneway, a bare cycleway=lane is on the travel-right side only; sides are explicit otherwise
        const auto xs = resolve({{"highway", "residential"}, {"oneway", "yes"}, {"cycleway", "lane"}});
        CHECK(count_kind(xs, LK::Cycle) == 1);
        CHECK(xs.bands[xs.bands.size() - 2].kind == LK::Cycle);
        const auto rev = resolve({{"highway", "residential"}, {"oneway", "-1"}, {"cycleway", "lane"}});
        CHECK(rev.bands[1].kind == LK::Cycle);
        const auto left = resolve({{"highway", "residential"}, {"oneway", "yes"}, {"cycleway:left", "lane"}});
        CHECK(left.bands[1].kind == LK::Cycle);
        const auto both = resolve({{"highway", "residential"}, {"cycleway:both", "lane"}, {"cycleway:left", "no"}});
        CHECK(count_kind(both, LK::Cycle) == 1);
    }
    {   // tracks and separate ways get no band, unknown values decline
        const auto track = resolve({{"highway", "residential"}, {"cycleway", "track"}});
        CHECK(count_kind(track, LK::Cycle) == 0); CHECK(track.field_declines == 0);
        const auto bad = resolve({{"highway", "residential"}, {"cycleway", "yes"}});
        CHECK((bad.field_declines & kDeclineCyclewayValue) != 0);
    }
    {   // parking: new scheme with orientation; the old scheme declines
        const auto xs = resolve({{"highway", "residential"}, {"parking:right", "lane"}, {"parking:right:orientation", "diagonal"}});
        CHECK(kinds(xs) == std::vector<LK>{LK::Verge, LK::Driving, LK::Driving, LK::Parking, LK::Verge});
        CHECK(xs.bands[3].width_mm == 4500); CHECK(xs.bands[3].direction == LaneDirection::Forward);
        const auto both = resolve({{"highway", "residential"}, {"parking:both", "lane"}});
        CHECK(both.bands[1].kind == LK::Parking); CHECK(both.bands[1].width_mm == 2000);
        const auto old = resolve({{"highway", "residential"}, {"parking:lane:right", "parallel"}});
        CHECK((old.field_declines & kDeclineParkingScheme) != 0);
        CHECK(count_kind(old, LK::Parking) == 0);
        const auto none = resolve({{"highway", "residential"}, {"parking:right", "street_side"}});
        CHECK(none.field_declines == 0);
        const auto bad = resolve({{"highway", "residential"}, {"parking:right", "yes"}});
        CHECK((bad.field_declines & kDeclineParkingScheme) != 0);
        const auto orientation = resolve({{"highway", "residential"}, {"parking:left", "lane"}, {"parking:left:orientation", "slanted"}});
        CHECK((orientation.field_declines & kDeclineParkingScheme) != 0); CHECK(orientation.bands[1].width_mm == 2000);
    }
    {   // cycle then parking, outward from the carriageway
        const auto xs = resolve({{"highway", "residential"}, {"cycleway:right", "lane"}, {"parking:right", "lane"}});
        CHECK(kinds(xs) == std::vector<LK>{LK::Verge, LK::Driving, LK::Driving, LK::Cycle, LK::Parking, LK::Verge});
    }
    {   // shoulder tag: adds one where the class has none (tagged width), removes the class one
        const auto yes = resolve({{"highway", "residential"}, {"shoulder", "yes"}});
        CHECK(kinds(yes) == std::vector<LK>{LK::Verge, LK::Shoulder, LK::Driving, LK::Driving, LK::Shoulder, LK::Verge});
        CHECK(yes.bands[1].width_mm == 1500); CHECK(yes.bands[1].provenance.kind == Provenance::Tagged);
        const auto right = resolve({{"highway", "residential"}, {"shoulder", "right"}});
        CHECK(count_kind(right, LK::Shoulder) == 1);
        CHECK(right.bands[right.bands.size() - 2].kind == LK::Shoulder);
        const auto no = resolve({{"highway", "motorway"}, {"shoulder", "no"}});
        CHECK(count_kind(no, LK::Shoulder) == 0);
        const auto rev = resolve({{"highway", "motorway"}, {"oneway", "-1"}});
        // the median is the travel-left side, which is the way's right side
        CHECK(rev.bands[1].width_mm == 2500); CHECK(rev.bands[rev.bands.size() - 2].width_mm == 750);
    }
    {   // sidewalks: left only, side-specific tags, bad values
        const auto left = resolve({{"highway", "residential"}, {"sidewalk", "left"}});
        CHECK(left.bands.front().kind == LK::Sidewalk); CHECK(left.bands.back().kind == LK::Verge);
        const auto side = resolve({{"highway", "residential"}, {"sidewalk:right", "yes"}, {"sidewalk:left", "no"}});
        CHECK(side.bands.front().kind == LK::Verge); CHECK(side.bands.back().kind == LK::Sidewalk);
        const auto bothtag = resolve({{"highway", "residential"}, {"sidewalk:both", "yes"}});
        CHECK(bothtag.bands.front().kind == LK::Sidewalk); CHECK(bothtag.bands.back().kind == LK::Sidewalk);
        const auto separate = resolve({{"highway", "residential"}, {"sidewalk", "separate"}});
        CHECK(separate.field_declines == 0); CHECK(separate.bands.front().kind == LK::Verge);
        const auto bad = resolve({{"highway", "residential"}, {"sidewalk", "yes"}});
        CHECK((bad.field_declines & kDeclineSidewalkValue) != 0); CHECK(bad.bands.front().kind == LK::Verge);
    }
    {   // markings
        const auto no = resolve({{"highway", "primary"}, {"lane_markings", "no"}});
        CHECK_FALSE(no.bands[1].marked_right); CHECK((no.flags & kXsMarkedTagged) != 0);
        const auto yes = resolve({{"highway", "residential"}, {"lane_markings", "yes"}});
        CHECK(yes.bands[1].marked_right); CHECK((yes.flags & kXsMarkedTagged) != 0);
        const auto lanes = resolve({{"highway", "primary"}, {"oneway", "yes"}, {"lanes", "3"}});
        const auto f = first_counted(lanes);
        CHECK(lanes.bands[f].marked_right); CHECK(lanes.bands[f + 1].marked_right); CHECK_FALSE(lanes.bands[f + 2].marked_right);
        CHECK((lanes.flags & kXsMarkedTagged) == 0);
    }
}

TEST_CASE("Placement anchors", "[layer][road_cross_section][roads]") {
    {   // oneway: lane numbers count from the left in travel order
        const auto xs = resolve({{"highway", "primary"}, {"oneway", "yes"}, {"lanes", "3"}, {"placement", "right_of:1"}});
        const auto f = first_counted(xs);
        CHECK(xs.anchor.band == f); CHECK(xs.anchor.edge == AnchorEdge::Right);
        CHECK(xs.anchor.source == PlacementSource::Explicit); CHECK(xs.placement_provenance == Provenance::Tagged);
        const auto left = resolve({{"highway", "primary"}, {"oneway", "yes"}, {"lanes", "3"}, {"placement", "left_of:1"}});
        CHECK(left.anchor.band == f); CHECK(left.anchor.edge == AnchorEdge::Left);
        const auto mid = resolve({{"highway", "primary"}, {"oneway", "yes"}, {"lanes", "3"}, {"placement", "middle_of:3"}});
        CHECK(mid.anchor.band == f + 2); CHECK(mid.anchor.edge == AnchorEdge::Centre);
    }
    {   // the backward group is mirrored: travel-left is way-right
        const auto xs = resolve({{"highway", "tertiary"}, {"lanes", "4"}, {"lanes:forward", "2"}, {"lanes:backward", "2"},
            {"placement:backward", "left_of:1"}});
        const auto f = first_counted(xs);
        CHECK(xs.anchor.band == f + 1); CHECK(xs.anchor.edge == AnchorEdge::Right);
        const auto fwd = resolve({{"highway", "tertiary"}, {"lanes", "4"}, {"lanes:forward", "2"}, {"lanes:backward", "2"},
            {"placement:forward", "right_of:2"}});
        CHECK(fwd.anchor.band == f + 3); CHECK(fwd.anchor.edge == AnchorEdge::Right);
        const auto rev = resolve({{"highway", "primary"}, {"oneway", "-1"}, {"lanes", "2"}, {"placement", "left_of:1"}});
        CHECK(rev.anchor.band == first_counted(rev) + 1); CHECK(rev.anchor.edge == AnchorEdge::Right);
    }
    {   // both given: forward wins, a conflict declines the value
        const auto same = resolve({{"highway", "tertiary"}, {"lanes", "4"}, {"lanes:forward", "2"}, {"lanes:backward", "2"},
            {"placement:forward", "left_of:1"}, {"placement:backward", "left_of:1"}});
        CHECK(same.anchor.band == first_counted(same) + 2); CHECK(same.anchor.edge == AnchorEdge::Left);
        CHECK(same.field_declines == 0); // the same anchor seen from both groups
        const auto conflict = resolve({{"highway", "tertiary"}, {"lanes", "4"}, {"lanes:forward", "2"}, {"lanes:backward", "2"},
            {"placement:forward", "left_of:1"}, {"placement:backward", "right_of:2"}});
        CHECK((conflict.field_declines & kDeclinePlacementValue) != 0);
        CHECK(conflict.anchor.band == first_counted(conflict) + 2);
    }
    {   // transition, unparsable, out of range
        const auto t = resolve({{"highway", "primary"}, {"oneway", "yes"}, {"lanes", "3"}, {"placement", "transition"}});
        CHECK(t.anchor.source == PlacementSource::Transition); CHECK(t.anchor.edge == AnchorEdge::Centre);
        CHECK(t.anchor.band == first_counted(t) + 1); CHECK(t.placement_provenance == Provenance::Inferred);
        const auto two = resolve({{"highway", "primary"}, {"placement", "transition"}});
        CHECK(two.anchor.source == PlacementSource::Transition);
        const auto bare = resolve({{"highway", "primary"}, {"placement", "right_of:1"}});
        CHECK((bare.field_declines & kDeclinePlacementValue) != 0); CHECK(bare.anchor.source == PlacementSource::Absent);
        const auto junk = resolve({{"highway", "primary"}, {"oneway", "yes"}, {"placement", "sideways"}});
        CHECK((junk.field_declines & kDeclinePlacementValue) != 0); CHECK(junk.anchor.source == PlacementSource::Absent);
        const auto range = resolve({{"highway", "primary"}, {"oneway", "yes"}, {"lanes", "2"}, {"placement", "right_of:3"}});
        CHECK((range.field_declines & kDeclinePlacementRange) != 0); CHECK(range.anchor.source == PlacementSource::Absent);
        const auto zero = resolve({{"highway", "primary"}, {"oneway", "yes"}, {"lanes", "2"}, {"placement", "right_of:0"}});
        CHECK((zero.field_declines & kDeclinePlacementRange) != 0);
    }
    {   // the anchor band index includes the non-counted bands to its left
        const auto xs = resolve({{"highway", "primary"}, {"oneway", "yes"}, {"lanes", "2"}, {"sidewalk", "left"}, {"cycleway:left", "lane"}, {"placement", "left_of:1"}});
        CHECK(xs.anchor.band == 2); CHECK(xs.bands[2].counted); CHECK(xs.bands[0].kind == LK::Sidewalk);
    }
}

TEST_CASE("Whole-section declines and unsupported ways", "[layer][road_cross_section][roads]") {
    {   // 16 + 16 counted lanes plus two verges is 34 bands
        const auto base = resolve_cross_section_base(make_way({{"highway", "primary"}, {"lanes:forward", "16"}, {"lanes:backward", "16"}}), rules());
        REQUIRE(base); CHECK_FALSE(base->xs); REQUIRE(base->decline);
        CHECK(base->decline->code == CrossSectionDeclineCode::TooManyBands); CHECK(base->decline->detail == 34);
    }
    {   // 16 lanes of 6.5 m are 104 m
        const auto base = resolve_cross_section_base(make_way({{"highway", "primary"}, {"oneway", "yes"}, {"lanes", "16"},
            {"width:lanes", "6.5|6.5|6.5|6.5|6.5|6.5|6.5|6.5|6.5|6.5|6.5|6.5|6.5|6.5|6.5|6.5"}}), rules());
        REQUIRE(base); CHECK_FALSE(base->xs); REQUIRE(base->decline);
        CHECK(base->decline->code == CrossSectionDeclineCode::TotalWidthLimit);
    }
    CHECK_FALSE(resolve_cross_section_base(make_way({{"highway", "footway"}}), rules()));
    CHECK_FALSE(resolve_cross_section_base(make_way({{"highway", "residential"}, {"area", "yes"}}), rules()));
    auto unsorted = make_way({{"highway", "residential"}, {"lanes", "2"}});
    std::swap(unsorted.tags[0], unsorted.tags[1]);
    CHECK_FALSE(resolve_cross_section_base(unsorted, rules()));
}

TEST_CASE("Cross-section evaluation: ramps, offsets and boundaries", "[layer][road_cross_section][roads]") {
    RoadCrossSection xs;
    xs.length_m = 100.0;
    CrossSectionBand lane; lane.kind = LK::Driving; lane.counted = true; lane.width_mm = 3000;
    xs.bands = {lane, lane};
    xs.anchor = {1, AnchorEdge::Left, PlacementSource::Explicit};
    SECTION("no ramps: widths are constant and the boundaries monotone") {
        double b[3];
        for (double s : {0.0, 33.3, 100.0}) {
            REQUIRE(xs.boundaries_at(s, b));
            CHECK(b[0] == 3.0); CHECK(b[1] == 0.0); CHECK(b[2] == -3.0); // anchor = left edge of band 1 on the way line
        }
        double wrong[2];
        CHECK_FALSE(xs.boundaries_at(0.0, wrong));
    }
    SECTION("a start ramp grows the band from its join width, exactly at both ends") {
        xs.bands[0].start = {20000, 0};
        CHECK(xs.band_width_m(0, 0.0) == 0.0);
        CHECK(xs.band_width_m(0, 10.0) == 1.5);       // S(1/2) = 1/2
        CHECK(xs.band_width_m(0, 20.0) == 3.0);
        CHECK(xs.band_width_m(0, 100.0) == 3.0);
        xs.bands[0].end = {10000, 6000};
        CHECK(xs.band_width_m(0, 100.0) == 6.0);
        CHECK(xs.band_width_m(0, 95.0) == 4.5);
        CHECK(xs.band_width_m(0, 50.0) == 3.0);
        double b[3], prev = 1e9;
        for (double s = 0; s <= 100.0; s += 0.5) {
            REQUIRE(xs.boundaries_at(s, b));
            CHECK(b[0] >= b[1]); CHECK(b[1] > b[2]);
            CHECK(b[1] == 0.0);                       // the anchor boundary is the way line
            prev = b[0];
        }
        (void)prev;
    }
    SECTION("anchor edges") {
        double b[3];
        xs.anchor.edge = AnchorEdge::Centre; REQUIRE(xs.boundaries_at(10.0, b));
        CHECK(b[1] == 1.5); CHECK(b[2] == -1.5); CHECK(b[0] == 4.5);
        xs.anchor.edge = AnchorEdge::Right; REQUIRE(xs.boundaries_at(10.0, b));
        CHECK(b[1] == 3.0); CHECK(b[2] == 0.0);
        xs.anchor = {0, AnchorEdge::Left, PlacementSource::Absent}; REQUIRE(xs.boundaries_at(10.0, b));
        CHECK(b[0] == 0.0); CHECK(b[1] == -3.0); CHECK(b[2] == -6.0);
    }
    SECTION("offset modes") {
        xs.offset = {OffsetMode::Hold, 1500, 0, 0, 0};
        CHECK(xs.offset_m(0.0) == 1.5); CHECK(xs.offset_m(70.0) == 1.5);
        xs.offset = {OffsetMode::Ramps, 2000, 20000, -1000, 10000};
        CHECK(xs.offset_m(0.0) == 2.0); CHECK(xs.offset_m(10.0) == 1.0); CHECK(xs.offset_m(20.0) == 0.0);
        CHECK(xs.offset_m(50.0) == 0.0); CHECK(xs.offset_m(100.0) == -1.0); CHECK(xs.offset_m(95.0) == -0.5);
        xs.offset = {OffsetMode::Transition, 1500, 0, -1500, 0};
        CHECK(xs.offset_m(0.0) == 1.5); CHECK(xs.offset_m(50.0) == 0.0); CHECK(xs.offset_m(100.0) == -1.5);
        xs.offset = {};
        CHECK(xs.offset_m(33.0) == 0.0);
        xs.offset = {OffsetMode::Hold, 1000, 0, 0, 0};
        double b[3]; REQUIRE(xs.boundaries_at(10.0, b));
        CHECK(b[1] == 1.0);                           // the whole section shifted to the left
    }
}

TEST_CASE("Every road way of the Hoechst fixture resolves, and the A 661 link is placed left_of:1", "[layer][road_cross_section][roads][fixture]") {
    const auto data = g2m_test::read_osm_fixture(std::string(G2M_SOURCE_DIR) + "/testdata/fixtures/osm/frankfurt_hoechst.osm.pbf");
    const auto graph = build_road_graph(data, g2m_test::kZone32N); REQUIRE(graph);
    REQUIRE(graph->ways.size() > 100);
    std::size_t resolved = 0, declined = 0, field_declines = 0;
    for (const auto& way : graph->ways) {
        const auto base = resolve_cross_section_base(way, rules());
        REQUIRE(base);
        if (!base->xs) { ++declined; continue; }
        ++resolved;
        auto xs = *base->xs; xs.length_m = 100.0;
        if (xs.field_declines) ++field_declines;
        REQUIRE(xs.bands.size() >= 1); REQUIRE(xs.bands.size() <= 32);
        REQUIRE(xs.anchor.band < xs.bands.size());
        CHECK(xs.bands[xs.anchor.band].counted);
        std::vector<double> b(xs.bands.size() + 1);
        REQUIRE(xs.boundaries_at(50.0, b));
        for (std::size_t i = 0; i < xs.bands.size(); ++i) { CHECK(xs.bands[i].width_mm > 0); CHECK(b[i] > b[i + 1]); }
        // placement is a lateral position only: the way line lies inside the counted lanes
        std::size_t lo = xs.bands.size(), hi = 0;
        for (std::size_t i = 0; i < xs.bands.size(); ++i) if (xs.bands[i].counted) { lo = std::min(lo, i); hi = std::max(hi, i); }
        CHECK(b[lo] >= 0.0); CHECK(b[hi + 1] <= 0.0);
    }
    CHECK(resolved + declined == graph->ways.size());
    CHECK(declined == 0);
    std::printf("[road_cross_section] fixture ways %zu resolved %zu with field declines %zu\n", graph->ways.size(), resolved, field_declines);

    // F5: motorway_link 50876963, one lane, placement=left_of:1.
    const RoadGraphWay* link = nullptr;
    for (const auto& way : graph->ways) if (way.osm_id == 50876963) link = &way;
    REQUIRE(link);
    CHECK(g2m_test::way_tag(*link, "placement") == "left_of:1");
    const auto base = resolve_cross_section_base(*link, rules()); REQUIRE(base); REQUIRE(base->xs);
    auto xs = *base->xs; xs.length_m = 100.0;
    CHECK(xs.anchor.edge == AnchorEdge::Left); CHECK(xs.anchor.source == PlacementSource::Explicit);
    CHECK(xs.placement_provenance == Provenance::Tagged);
    CHECK(xs.bands[xs.anchor.band].counted); CHECK(xs.bands[xs.anchor.band].direction == LaneDirection::Forward);
    CHECK(xs.anchor.band == first_counted(xs)); CHECK((xs.flags & kXsOneway) != 0);
    std::vector<double> b(xs.bands.size() + 1);
    REQUIRE(xs.boundaries_at(50.0, b));
    CHECK(std::fabs(b[xs.anchor.band]) < 0.001); // the lane's left edge lies on the way line
}
