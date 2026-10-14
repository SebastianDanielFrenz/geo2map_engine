// Turn restriction extraction from canonical OSM relations (G4 slice 1).
#include "g2m/route/turn_restriction.h"

#include <catch2/catch_test_macros.hpp>

using namespace g2m;
using namespace g2m::route;

namespace {
using M = osm::OsmBuilder::MemberIn;
M way(std::int64_t id, const char* role) { return {osm::MemberType::Way, id, role}; }
M node(std::int64_t id, const char* role) { return {osm::MemberType::Node, id, role}; }
} // namespace

TEST_CASE("Turn restrictions: values, vehicle filters and malformed relations", "[route][restriction]") {
    osm::OsmBuilder b;
    b.add_relation(1, {way(10, "from"), node(5, "via"), way(20, "to")}, {{"type", "restriction"}, {"restriction", "no_left_turn"}});
    b.add_relation(2, {way(10, "from"), node(5, "via"), way(30, "to")}, {{"type", "restriction"}, {"restriction", "only_straight_on"}});
    // no_entry: two from ways -> two rules
    b.add_relation(3, {way(11, "from"), way(12, "from"), node(6, "via"), way(13, "to")}, {{"type", "restriction"}, {"restriction", "no_entry"}});
    // motorcar-specific value wins over the generic one
    b.add_relation(4, {way(10, "from"), node(5, "via"), way(40, "to")},
                   {{"type", "restriction"}, {"restriction", "only_right_turn"}, {"restriction:motorcar", "no_right_turn"}});
    b.add_relation(5, {way(10, "from"), node(5, "via"), way(20, "to")},
                   {{"type", "restriction"}, {"restriction", "no_left_turn"}, {"except", "bicycle;motorcar"}});
    b.add_relation(6, {way(10, "from"), node(5, "via"), way(20, "to")}, {{"type", "restriction"}, {"restriction:hgv", "no_left_turn"}});
    b.add_relation(7, {way(10, "from"), node(5, "via"), way(20, "to")},
                   {{"type", "restriction"}, {"restriction:conditional", "no_left_turn @ (Mo-Fr 07:00-09:00)"}});
    b.add_relation(8, {way(10, "from"), way(20, "to")}, {{"type", "restriction"}, {"restriction", "no_left_turn"}}); // no via
    b.add_relation(9, {way(10, "from"), node(5, "via"), way(15, "via"), way(20, "to")}, {{"type", "restriction"}, {"restriction", "no_left_turn"}});
    b.add_relation(10, {way(10, "from"), node(5, "via"), way(20, "to")}, {{"type", "restriction"}, {"restriction", "no_parking"}});
    b.add_relation(11, {way(10, "from"), way(15, "via"), way(16, "via"), way(20, "to")}, {{"type", "restriction"}, {"restriction", "no_u_turn"}});
    b.add_relation(12, {way(10, "from"), way(11, "from"), node(5, "via"), way(20, "to")}, {{"type", "restriction"}, {"restriction", "only_left_turn"}});
    b.add_relation(13, {way(10, "outer")}, {{"type", "multipolygon"}});
    const auto data = b.build();
    REQUIRE(data);
    const auto set = extract_turn_restrictions(*data);
    REQUIRE(set);
    CHECK(set->relations_seen == 12);
    CHECK(set->not_for_cars == 2);
    CHECK(set->conditional_only == 1);
    CHECK(set->malformed == 3); // 8 no via, 9 node and way via, 12 two from on an only_ rule
    CHECK(set->unknown_value == 1);
    REQUIRE(set->restrictions.size() == 6);
    const auto& r = set->restrictions;
    CHECK(r[0] == TurnRestriction{1, RestrictionKind::No, "no_left_turn", 10, 5, {}, 20});
    CHECK(r[1] == TurnRestriction{2, RestrictionKind::Only, "only_straight_on", 10, 5, {}, 30});
    CHECK(r[2] == TurnRestriction{3, RestrictionKind::No, "no_entry", 11, 6, {}, 13});
    CHECK(r[3] == TurnRestriction{3, RestrictionKind::No, "no_entry", 12, 6, {}, 13});
    CHECK(r[4] == TurnRestriction{4, RestrictionKind::No, "no_right_turn", 10, 5, {}, 40});
    CHECK(r[5] == TurnRestriction{11, RestrictionKind::No, "no_u_turn", 10, 0, {15, 16}, 20});

    const auto capped = extract_turn_restrictions(*data, 3);
    REQUIRE_FALSE(capped);
    CHECK(capped.error().code == ErrorCode::LimitExceeded);
}
