// Street fidelity P1 slice 5: roads.geom encoding 6 (design section 7) - cross-section and feature sections,
// every validation rule of 7.2 on a crafted tile or body, encoding-5 read compatibility (7.3), the config hash
// and release id (7.4).
#include "road_fixture.h"
#include "g2m/core/bytes.h"
#include "g2m/core/hash.h"
#include "g2m/import/bake.h"
#include "g2m/server/upstream.h"
#include "g2m/layer/road_cross_section.h"
#include "g2m/layer/road_geom_tile.h"
#include "g2m/server/builtin_release.h"
#include <catch2/catch_test_macros.hpp>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <functional>
#include <limits>
#include <string>
using namespace g2m;
namespace {
const geo::UtmZone zone{32, geo::Hemisphere::North};

RoadGraphTile source() {
    RoadGraph graph;
    graph.nodes = {{1, {10000, 10000}}, {2, {110000, 10000}}, {3, {10000, 100000}}, {4, {110000, 100000}}};
    graph.ways = {{10, 1, {1, 2}, {{"highway", "service"}, {"maxspeed", "30"}}},
        {20, 1, {3, 4}, {{"highway", "service"}, {"tunnel", "yes"}}}};
    graph.edges = {{10, 0, 1, 2}, {20, 0, 3, 4}};
    return *select_road_graph_tile(graph, {zone, 2, 0, 0});
}
// Entry 0 has a profile and a cross-section (100 m long), entry 1 is a declined tunnel (record kind Absent).
RoadGeomTile base_tile() {
    const auto graph = source(); HeightTile heights; heights.key = {zone, 0, 0, 0}; heights.h.fill(25600);
    const HeightTile* views[] = {&heights}; const auto dem = RoadDemSampler::make(views); REQUIRE(dem);
    auto tile = build_road_geom_tile(graph, *dem); REQUIRE(tile);
    REQUIRE(tile->entries.size() == 2); REQUIRE(tile->entries[0].cross_section); REQUIRE(tile->entries[1].decline);
    return std::move(*tile);
}
std::int32_t length_mm(const RoadGeomTile& t) { return static_cast<std::int32_t>(std::llround(t.entries[0].profile->reference.length_m * 1000.0)); }
ProfileFeature feature(std::int64_t node, double station, ProfileFeatureKind kind = ProfileFeatureKind::Table) {
    return {0, node, station, kind, true, 6000, 80, 0, Provenance::Tagged};
}
Result<TileBody> encoded(const RoadGeomTile& t) { return encode_road_geom(t); }
void rejects(const char* name, const std::function<void(RoadGeomTile&)>& mutate) {
    INFO(name);
    auto tile = base_tile(); mutate(tile);
    CHECK_FALSE(validate_road_geom_tile(tile));
    CHECK_FALSE(encoded(tile));
}
std::string hex_of(const TileBody& body) { return to_hex(content_hash(*encode_body(body))); }
}

TEST_CASE("roads.geom encodings 6 to 9 round trip cross-sections, declines and features", "[layer][road_geom_xs]") {
    auto tile = base_tile();
    CHECK(tile.encoding == 8); tile.encoding = kRoadGeomEncodingPrevious;
    // A non-trivial section: ramps, an offset with ramps, a join record, turn bits, a feature at each end.
    auto& xs = *tile.entries[0].cross_section;
    xs.bands.front().turn = kTurnLeft | kTurnThrough; xs.bands.front().marked_right = true;
    xs.bands.front().start = {12000, 0}; xs.bands.front().end = {20000, 1500};
    xs.offset = {OffsetMode::Ramps, 300, 10000, -200, 20000};
    xs.start_join = {JoinKind::LaneChange, 11, -3500}; xs.end_join = {JoinKind::PlacementConflict, 12, 1750};
    xs.flags |= kXsTaperShortenedStart; xs.field_declines = kDeclineTurnToken | kDeclineWidthValue;
    xs.count_provenance = Provenance::Derived; xs.placement_provenance = Provenance::Tagged;
    xs.bands.front().provenance = {Provenance::Tagged, Provenance::Derived, Provenance::Inferred, Provenance::Official, Provenance::Tagged};
    tile.features = {feature(1, 0.0), feature(2, tile.entries[0].profile->reference.length_m, ProfileFeatureKind::Chicane)};
    tile.features[1].carve = false; tile.features[1].length_mm = 0; tile.features[1].height_mm = 0;
    const auto body = encoded(tile); REQUIRE(body);
    CHECK(body->encoding == 6); CHECK(body->encoding_version == 1); REQUIRE(body->sections.size() == 4);
    for (std::size_t i = 0; i < 4; ++i) CHECK(body->sections[i].type == i + 1);
    const auto back = decode_road_geom(*body); REQUIRE(back);
    CHECK(back->encoding == 6); CHECK(back->features == tile.features);
    REQUIRE(back->entries.size() == 2);
    CHECK(back->entries[0].cross_section == tile.entries[0].cross_section);
    CHECK_FALSE(back->entries[0].cross_section_decline); CHECK_FALSE(back->entries[1].cross_section); CHECK_FALSE(back->entries[1].cross_section_decline);
    CHECK(encoded(*back)->sections[2].data == body->sections[2].data);
    CHECK(*encode_body(*encoded(*back)) == *encode_body(*body));
    // Encoding 7 of the same tile: the four P1 sections byte for byte, then junctions, furniture, provenance.
    auto seven = tile; seven.encoding = kRoadGeomEncodingJunctions;
    const auto body7 = encoded(seven); REQUIRE(body7);
    CHECK(body7->encoding == 7); REQUIRE(body7->sections.size() == 7);
    for (std::size_t i = 0; i < 7; ++i) CHECK(body7->sections[i].type == i + 1);
    for (std::size_t i = 0; i < 4; ++i) CHECK(body7->sections[i].data == body->sections[i].data);
    const auto back7 = decode_road_geom(*body7); REQUIRE(back7);
    CHECK(back7->encoding == 7); CHECK(back7->features == tile.features); CHECK(back7->origins == seven.origins);
    CHECK(*encode_body(*encoded(*back7)) == *encode_body(*body7));
    // Encoding 8 (D47): the byte layout of 7; only the encoding number differs for a tile without tunnel floors.
    auto eight = tile; eight.encoding = kRoadGeomEncodingTunnels;
    const auto body8 = encoded(eight); REQUIRE(body8);
    CHECK(body8->encoding == 8); REQUIRE(body8->sections.size() == 7);
    for (std::size_t i = 0; i < 7; ++i) CHECK(body8->sections[i].data == body7->sections[i].data);
    const auto back8 = decode_road_geom(*body8); REQUIRE(back8); CHECK(back8->encoding == 8); CHECK(back8->origins == eight.origins);
    // Encoding 9 (D56): sections 1-7 of encoding 8 byte for byte, then section 8 (no decks here: a zero count).
    auto nine = tile; nine.encoding = kRoadGeomEncoding;
    const auto body9 = encoded(nine); REQUIRE(body9);
    CHECK(body9->encoding == 9); REQUIRE(body9->sections.size() == 8);
    for (std::size_t i = 0; i < 7; ++i) CHECK(body9->sections[i].data == body8->sections[i].data);
    CHECK(body9->sections[7].type == 8);
    const auto back9 = decode_road_geom(*body9); REQUIRE(back9); CHECK(back9->encoding == 9); CHECK(back9->underpasses.empty());
    CHECK(*encode_body(*encoded(*back9)) == *encode_body(*body9));
    // A whole-section decline is a record of its own.
    tile.features.clear(); tile.entries[0].cross_section.reset();
    tile.entries[0].cross_section_decline = CrossSectionDecline{CrossSectionDeclineCode::TotalWidthLimit, -7};
    const auto declined = encoded(tile); REQUIRE(declined);
    const auto declined_back = decode_road_geom(*declined); REQUIRE(declined_back);
    CHECK(declined_back->entries[0].cross_section_decline == tile.entries[0].cross_section_decline);
    CHECK_FALSE(declined_back->entries[0].cross_section);
}

TEST_CASE("roads.geom encoding 6 keeps sections 1 to 2 as before and decodes encoding-5 bodies", "[layer][road_geom_xs]") {
    auto tile = base_tile(); tile.encoding = kRoadGeomEncodingPrevious; const auto body = encoded(tile); REQUIRE(body);
    REQUIRE(body->sections.size() == 4);
    // The embedded graph is the encoding-5 payload, byte for byte.
    CHECK(body->sections[0].data == encode_road_graph(tile.source, kRoadGraphEncoding)->sections[0].data);
    // Section 2 is unchanged: with the legacy graph it reproduces the encoding-5 golden body of the synthetic fixture.
    const auto legacy_graph = encode_road_graph(tile.source, kRoadGraphEncodingLegacy); REQUIRE(legacy_graph);
    const TileBody legacy{kRoadGeomEncodingLegacy, 1, {legacy_graph->sections[0], body->sections[1]}};
    CHECK(hex_of(legacy) == "4dcfcbca39838a8922c2749a9513eae7eecdbb398dec9a81b724d2970ff445f8");
    const auto decoded = decode_road_geom(legacy); REQUIRE(decoded);
    CHECK(decoded->encoding == 5);
    for (const auto& entry : decoded->entries) { CHECK_FALSE(entry.cross_section); CHECK_FALSE(entry.cross_section_decline); }
    CHECK(decoded->features.empty());
    CHECK(decoded->entries[0].profile.has_value()); CHECK(decoded->entries[1].decline.has_value());
    CHECK(decoded->source == tile.source);
    CHECK(validate_road_geom_tile(*decoded));
    // An encoding-5 tile is never written.
    const auto refused = encode_road_geom(*decoded); REQUIRE_FALSE(refused);
    CHECK(refused.error().code == ErrorCode::InvalidArgument);
    // The section count must match the encoding.
    auto five_with_four = *body; five_with_four.encoding = kRoadGeomEncodingLegacy; CHECK_FALSE(decode_road_geom(five_with_four));
    auto six_with_two = legacy; six_with_two.encoding = kRoadGeomEncodingPrevious; CHECK_FALSE(decode_road_geom(six_with_two));
    auto swapped = *body; std::swap(swapped.sections[2], swapped.sections[3]); CHECK_FALSE(decode_road_geom(swapped));
    auto wrong_version = *body; wrong_version.encoding_version = 2; CHECK_FALSE(decode_road_geom(wrong_version));
    auto seven_with_four = *body; seven_with_four.encoding = kRoadGeomEncoding; CHECK_FALSE(decode_road_geom(seven_with_four));
    auto eight_with_four = *body; eight_with_four.encoding = kRoadGeomEncodingJunctions; CHECK_FALSE(decode_road_geom(eight_with_four));
    auto nine = *body; nine.encoding = 9; CHECK_FALSE(decode_road_geom(nine));
    const auto body7 = encoded(base_tile()); REQUIRE(body7); REQUIRE(body7->sections.size() == 7);
    auto six_with_seven = *body7; six_with_seven.encoding = kRoadGeomEncodingPrevious; CHECK_FALSE(decode_road_geom(six_with_seven));
    auto swapped7 = *body7; std::swap(swapped7.sections[4], swapped7.sections[5]); CHECK_FALSE(decode_road_geom(swapped7));
}

TEST_CASE("roads.geom validation rejects every rule of design 7.2", "[layer][road_geom_xs]") {
    // Outcome shape.
    rejects("profile entry without an outcome", [](auto& t) { t.entries[0].cross_section.reset(); });
    rejects("profile entry with both outcomes", [](auto& t) { t.entries[0].cross_section_decline = CrossSectionDecline{}; });
    rejects("declined entry with a cross-section", [](auto& t) { t.entries[1].cross_section = t.entries[0].cross_section; });
    rejects("declined entry with a cross-section decline", [](auto& t) { t.entries[1].cross_section_decline = CrossSectionDecline{}; });
    rejects("cross-section length differs from the reference", [](auto& t) { t.entries[0].cross_section->length_m += 1; });
    rejects("unknown tile encoding", [](auto& t) { t.encoding = 10; });
    rejects("legacy tile with a cross-section", [](auto& t) { t.encoding = 5; });
    rejects("legacy tile with features", [](auto& t) { t.encoding = 5; t.entries[0].cross_section.reset(); t.features = {feature(1, 0)}; });
    // Enums, flags and counts.
    rejects("flag bit outside the defined mask", [](auto& t) { t.entries[0].cross_section->flags |= 1u << 6; });
    rejects("reserved licence bit 15", [](auto& t) { t.entries[0].cross_section->flags |= 1u << 15; });
    rejects("field decline bit outside the mask", [](auto& t) { t.entries[0].cross_section->field_declines |= 1u << 14; });
    rejects("lane kind", [](auto& t) { t.entries[0].cross_section->bands[0].kind = static_cast<LaneKind>(8); });
    rejects("lane direction", [](auto& t) { t.entries[0].cross_section->bands[0].direction = static_cast<LaneDirection>(4); });
    rejects("lane change", [](auto& t) { t.entries[0].cross_section->bands[0].change = static_cast<LaneChange>(5); });
    rejects("turn bits outside the defined mask", [](auto& t) { t.entries[0].cross_section->bands[0].turn = 1u << 11; });
    rejects("provenance", [](auto& t) { t.entries[0].cross_section->bands[0].provenance.width = static_cast<Provenance>(4); });
    rejects("section provenance", [](auto& t) { t.entries[0].cross_section->count_provenance = static_cast<Provenance>(4); });
    rejects("anchor edge", [](auto& t) { t.entries[0].cross_section->anchor.edge = static_cast<AnchorEdge>(3); });
    rejects("anchor source", [](auto& t) { t.entries[0].cross_section->anchor.source = static_cast<PlacementSource>(3); });
    rejects("offset mode", [](auto& t) { t.entries[0].cross_section->offset.mode = static_cast<OffsetMode>(4); });
    rejects("join kind", [](auto& t) { t.entries[0].cross_section->start_join.kind = static_cast<JoinKind>(5); });
    rejects("zero bands", [](auto& t) { t.entries[0].cross_section->bands.clear(); });
    rejects("33 bands", [](auto& t) { auto& x = *t.entries[0].cross_section; x.bands.resize(33, x.bands.back()); });
    // Anchor.
    rejects("anchor band out of range", [](auto& t) { t.entries[0].cross_section->anchor.band = 200; });
    rejects("anchor band not counted", [](auto& t) { auto& x = *t.entries[0].cross_section; x.bands[x.anchor.band].counted = false; });
    // Lengths.
    rejects("ramp longer than the stretch", [](auto& t) { t.entries[0].cross_section->bands[0].start.length_mm = length_mm(t) + 5; });
    rejects("negative ramp length", [](auto& t) { t.entries[0].cross_section->bands[0].end.length_mm = -1; });
    rejects("start plus end ramps exceed the stretch", [](auto& t) {
        auto& x = *t.entries[0].cross_section; x.bands[0].start.length_mm = length_mm(t) / 2 + 10; x.bands[0].end.length_mm = length_mm(t) / 2 + 10; });
    rejects("start ramp of one band plus end ramp of another exceed the stretch", [](auto& t) {
        auto& x = *t.entries[0].cross_section; x.bands.push_back(x.bands[0]);
        x.bands[0].start.length_mm = length_mm(t) / 2 + 10; x.bands[1].end.length_mm = length_mm(t) / 2 + 10; });
    rejects("offset ramp plus band ramp exceed the stretch", [](auto& t) {
        auto& x = *t.entries[0].cross_section; x.offset = {OffsetMode::Ramps, 10, length_mm(t) / 2 + 10, 0, 0};
        x.bands[0].end.length_mm = length_mm(t) / 2 + 10; });
    rejects("offset length longer than the stretch", [](auto& t) { t.entries[0].cross_section->offset = {OffsetMode::Ramps, 10, length_mm(t) + 5, 0, 0}; });
    rejects("hold with a start length", [](auto& t) { t.entries[0].cross_section->offset = {OffsetMode::Hold, 10, 100, 0, 0}; });
    rejects("hold with an end length", [](auto& t) { t.entries[0].cross_section->offset = {OffsetMode::Hold, 10, 0, 0, 100}; });
    rejects("hold with an end offset", [](auto& t) { t.entries[0].cross_section->offset = {OffsetMode::Hold, 10, 0, 5, 0}; });
    rejects("transition with a start length", [](auto& t) { t.entries[0].cross_section->offset = {OffsetMode::Transition, 10, 100, 20, 0}; });
    rejects("transition with an end length", [](auto& t) { t.entries[0].cross_section->offset = {OffsetMode::Transition, 10, 0, 20, 100}; });
    // Widths.
    rejects("band width above 20 m", [](auto& t) { t.entries[0].cross_section->bands[0].width_mm = 20001; });
    rejects("ramp start width above 20 m", [](auto& t) { t.entries[0].cross_section->bands[0].start.from_width_mm = 20001; });
    rejects("negative width", [](auto& t) { t.entries[0].cross_section->bands[0].width_mm = -1; });
    rejects("nominal widths above 100 m", [](auto& t) {
        auto& x = *t.entries[0].cross_section; for (int i = 0; i < 6; ++i) { x.bands.push_back(x.bands[0]); x.bands.back().width_mm = 20000; } });
    // Joins.
    rejects("join None with a neighbour", [](auto& t) { t.entries[0].cross_section->start_join = {JoinKind::None, 5, 0}; });
    rejects("join None with a delta", [](auto& t) { t.entries[0].cross_section->end_join = {JoinKind::None, 0, 7}; });
    rejects("join with a kind but no neighbour", [](auto& t) { t.entries[0].cross_section->end_join = {JoinKind::Consistent, 0, 0}; });
    // Features.
    const auto f = [](std::int64_t node, double s) { return feature(node, s); };
    rejects("feature on an entry without a cross-section", [&](auto& t) { auto x = f(1, 0); x.entry_index = 1; t.features = {x}; });
    rejects("feature entry out of range", [&](auto& t) { auto x = f(1, 0); x.entry_index = 9; t.features = {x}; });
    rejects("feature kind", [&](auto& t) { auto x = f(1, 0); x.kind = static_cast<ProfileFeatureKind>(10); t.features = {x}; });
    rejects("feature provenance", [&](auto& t) { auto x = f(1, 0); x.provenance = static_cast<Provenance>(4); t.features = {x}; });
    rejects("feature station beyond the stretch", [&](auto& t) { t.features = {f(2, t.entries[0].profile->reference.length_m + 0.001)}; });
    rejects("negative feature station", [&](auto& t) { t.features = {f(1, -1)}; });
    rejects("negative zero station", [&](auto& t) { t.features = {f(1, -0.0)}; });
    rejects("non-finite feature station", [&](auto& t) { t.features = {f(1, std::numeric_limits<double>::quiet_NaN())}; });
    rejects("feature node not on the stretch", [&](auto& t) { t.features = {f(3, 0)}; });
    rejects("feature length above 50 m", [&](auto& t) { auto x = f(1, 0); x.length_mm = 50001; t.features = {x}; });
    rejects("feature height above 300 mm", [&](auto& t) { auto x = f(1, 0); x.height_mm = 301; t.features = {x}; });
    rejects("negative feature height", [&](auto& t) { auto x = f(1, 0); x.height_mm = -1; t.features = {x}; });
    rejects("features out of order", [&](auto& t) { t.features = {f(2, t.entries[0].profile->reference.length_m), f(1, 0)}; });
    rejects("duplicate feature", [&](auto& t) { t.features = {f(1, 0), f(1, 0)}; });
    rejects("same station, descending node", [&](auto& t) { t.features = {f(2, 0), f(1, 0)}; });
    // The valid neighbours of the rejected values pass.
    auto tile = base_tile(); auto& xs = *tile.entries[0].cross_section; const auto len = length_mm(tile);
    xs.bands[0].start.length_mm = len / 2; xs.bands[0].end.length_mm = len / 2; xs.offset = {OffsetMode::Hold, 10, 0, 0, 0};
    tile.features = {f(1, 0), f(2, 0), f(2, tile.entries[0].profile->reference.length_m)};
    tile.features[0].height_mm = 300; tile.features[0].length_mm = 50000;
    CHECK(validate_road_geom_tile(tile)); CHECK(encoded(tile));
    xs.offset = {OffsetMode::Transition, 10, 0, 20, 0}; CHECK(validate_road_geom_tile(tile));
}

TEST_CASE("roads.geom decoding rejects crafted cross-section and feature bytes", "[layer][road_geom_xs]") {
    const auto tile = base_tile(); const auto body = *encoded(tile);
    // Section 3 layout of this tile: u8 count, record 0: u8 kind, u16 flags, u32 field_declines, u8 provenance, u8 band_count ...
    const auto patched = [&](std::size_t section, std::size_t at, std::uint8_t value) {
        auto b = body; b.sections[section].data[at] = value; return decode_road_geom(b);
    };
    CHECK(body.sections[2].data[0] == 2); CHECK(body.sections[2].data[1] == 1);
    CHECK_FALSE(patched(2, 0, 3));                 // record count differs from the entry count
    CHECK_FALSE(patched(2, 0, 1));
    CHECK_FALSE(patched(2, 1, 0));                 // Absent for an entry that has a profile
    CHECK_FALSE(patched(2, 1, 3));                 // record kind out of range
    CHECK_FALSE(patched(2, 3, 0x80));              // flag bit 15 (licence bit)
    CHECK_FALSE(patched(2, 2, 0xff));              // flag bits outside the mask
    CHECK_FALSE(patched(2, 7, 0x80));              // field-decline bits outside the mask
    CHECK_FALSE(patched(2, 8, 0xf0));              // section provenance reserved bits
    CHECK_FALSE(patched(2, 9, 0));                 // band count 0
    CHECK_FALSE(patched(2, 9, 33));                // band count 33
    CHECK_FALSE(patched(2, 10, 8));                // lane kind out of range
    CHECK_FALSE(patched(2, 11, 4));                // lane direction
    CHECK_FALSE(patched(2, 12, 5));                // lane change
    CHECK_FALSE(patched(2, 13, 8));                // band flag reserved bit
    // The last record (the declined tunnel entry) must be Absent.
    CHECK_FALSE(patched(2, body.sections[2].data.size() - 1, 1));
    CHECK_FALSE(patched(2, body.sections[2].data.size() - 1, 2));
    // Section 4: count and a forged huge count.
    CHECK_FALSE(patched(3, 0, 1));
    CHECK_FALSE(patched(3, 0, 0x7f));
    auto huge = body; ByteWriter big; big.varint(1ull << 40); huge.sections[3].data = big.take(); CHECK_FALSE(decode_road_geom(huge));
    // A feature body with forged fields is rejected; the unmodified one decodes.
    auto featured = tile; featured.features = {feature(1, 0.0)};
    const auto fbody = *encoded(featured); REQUIRE(decode_road_geom(fbody));
    const auto& fd = fbody.sections[3].data;
    for (std::size_t i = 0; i < fd.size(); ++i) {
        auto b = fbody; b.sections[3].data[i] = 0xff; const auto r = decode_road_geom(b);
        if (r) CHECK(encoded(*r));  // anything accepted is a valid tile
    }
    auto bad_kind = fbody; bad_kind.sections[3].data[fd.size() - 7] = 10; CHECK_FALSE(decode_road_geom(bad_kind));
    auto bad_carve = fbody; bad_carve.sections[3].data[fd.size() - 6] = 2; CHECK_FALSE(decode_road_geom(bad_carve));
    auto bad_prov = fbody; bad_prov.sections[3].data[fd.size() - 1] = 4; CHECK_FALSE(decode_road_geom(bad_prov));
    // Hostile bytes: every single-byte change of section 3 either fails or re-encodes to a canonical valid body.
    for (std::size_t i = 0; i < body.sections[2].data.size(); ++i) for (const std::uint8_t v : {std::uint8_t{0}, std::uint8_t{0xff}, std::uint8_t{0x7f}}) {
        auto b = body; b.sections[2].data[i] = v; const auto r = decode_road_geom(b);
        if (!r) continue;
        CHECK(validate_road_geom_tile(*r)); const auto again = encoded(*r); REQUIRE(again);
        CHECK(again->sections[2].data == b.sections[2].data);
    }
}

TEST_CASE("roads.geom encoding 6 over a real tile of the Hoechst A 66 fixture round trips", "[layer][road_geom_xs][roads]") {
    const auto data = g2m_test::read_osm_fixture(std::string(G2M_SOURCE_DIR) + "/testdata/fixtures/osm/hoechst_a66.osm.pbf");
    const auto world = build_road_graph(data, g2m_test::kZone32N); REQUIRE(world);
    const TileKey key{g2m_test::kZone32N, 2, 455, 5421};
    const auto geometry = g2m_test::geometry_in_tile(*world, key); REQUIRE(geometry);
    std::size_t with_xs = 0, joins = 0;
    for (const auto& entry : geometry->entries) if (entry.cross_section) {
        ++with_xs; joins += entry.cross_section->start_join.kind != JoinKind::None;
        CHECK(entry.cross_section->length_m == entry.profile->reference.length_m);
    }
    CHECK(with_xs > 400); CHECK(joins > 50);
    const auto body = encoded(*geometry); REQUIRE(body);
    const auto back = decode_road_geom(*body); REQUIRE(back);
    REQUIRE(back->entries.size() == geometry->entries.size());
    for (std::size_t i = 0; i < back->entries.size(); ++i) {
        CHECK(back->entries[i].cross_section == geometry->entries[i].cross_section);
        CHECK(back->entries[i].cross_section_decline == geometry->entries[i].cross_section_decline);
    }
    CHECK(*encode_body(*encoded(*back)) == *encode_body(*body));
    // Evaluating every decoded section is finite and ordered, at 17 stations.
    for (const auto& entry : back->entries) if (entry.cross_section) {
        const auto& xs = *entry.cross_section; std::vector<double> t(xs.bands.size() + 1);
        for (int i = 0; i <= 16; ++i) {
            REQUIRE(xs.boundaries_at(xs.length_m * i / 16.0, t));
            for (std::size_t j = 0; j < t.size(); ++j) { REQUIRE(std::isfinite(t[j])); if (j) REQUIRE(t[j] <= t[j - 1]); }
        }
    }
}

TEST_CASE("roads.geom config hash and release id carry the cross-section encoding and the rules", "[layer][road_geom_xs][release]") {
    const RoadGeomDeriver deriver;
    const std::string before = "g2m.roads.geom/1 class-policy-2026-10-02 spacing=5 max-samples=2048 iterations=4096 curvature-exceptions=true "
        "sloping-plane-r3 approach=32 height-correction=.25 merge-agreement=.5 incline-soft=5 dependency-margin=65 coverage=graph join-bisector=1 context=nodes";
    CHECK(deriver.config_hash() != Sha256::of(before));
    const auto& rules = builtin_road_rules_de().content_sha256;
    const std::string rules_hex = to_hex(std::span<const std::uint8_t>(rules.data(), rules.size()));
    const std::string p1 = "g2m.roads.geom/1 class-policy-2026-10-02 spacing=5 max-samples=2048 iterations=4096 curvature-exceptions=true "
        "sloping-plane-r3 approach=32 height-correction=.25 merge-agreement=.5 incline-soft=5 dependency-margin=65 coverage=graph xs=1 enc=6 rules=de:" +
        rules_hex + " join-bisector=1 context=nodes";
    CHECK(deriver.config_hash() != Sha256::of(p1));
    // Slice 8: enc=7 and every served completion option (D29's 60 m walk among them) and the D30/D34 switches;
    // terrain migration phase 4 (D47): enc=8, the tunnel-floor options and the accepted-bridge anchors.
    const std::string slice8 = "g2m.roads.geom/1 class-policy-2026-10-02 spacing=5 max-samples=2048 iterations=4096 curvature-exceptions=true "
        "sloping-plane-r3 approach=32 height-correction=.25 merge-agreement=.5 incline-soft=5 dependency-margin=65 coverage=graph xs=1 enc=7 rules=de:" +
        rules_hex + " join-bisector=1 context=nodes junction=1 furniture=1 completion=1 completion-retry=20/16384 completion-abutment=32/60 "
        "completion-ground=40/32 strict-anchor=1 ring-nesting=2mm bridge-far-end=1 ground-ease=8/0.05 corner-fillet=1 width-taper=5";
    CHECK(deriver.config_hash() != Sha256::of(slice8));
    const std::string phase4 = "g2m.roads.geom/1 class-policy-2026-10-02 spacing=5 max-samples=2048 iterations=4096 curvature-exceptions=true "
        "sloping-plane-r3 approach=32 height-correction=.25 merge-agreement=.5 incline-soft=5 dependency-margin=65 coverage=graph xs=1 enc=8 rules=de:" +
        rules_hex + " join-bisector=1 context=nodes junction=1 furniture=1 completion=1 completion-retry=20/16384 completion-abutment=32/60 "
        "completion-ground=40/32 strict-anchor=1 ring-nesting=2mm bridge-far-end=1 ground-ease=8/0.05 corner-fillet=1 width-taper=5"
        " tunnel-floor=5/0.29999999999999999/20/chord:0 bridge-anchor-reuse=1";
    CHECK(deriver.config_hash() != Sha256::of(phase4));
    // Terrain migration phase 5 (D48): the 200 m walk and its 205 m stage-2 squares.
    const std::string phase5 = "g2m.roads.geom/1 class-policy-2026-10-02 spacing=5 max-samples=2048 iterations=4096 curvature-exceptions=true "
        "sloping-plane-r3 approach=32 height-correction=.25 merge-agreement=.5 incline-soft=5 dependency-margin=65 coverage=graph xs=1 enc=8 rules=de:" +
        rules_hex + " join-bisector=1 context=nodes junction=1 furniture=1 completion=1 completion-retry=20/16384 completion-abutment=32/200 "
        "completion-ground=40/32 strict-anchor=1 ring-nesting=2mm bridge-far-end=1 ground-ease=8/0.05 corner-fillet=1 width-taper=5"
        " tunnel-floor=5/0.29999999999999999/20/chord:0 bridge-anchor-reuse=1 bridge-walk-margin=205";
    CHECK(deriver.config_hash() != Sha256::of(phase5));
    // Owner ruling 2026-10-15: tunnels of 200 m and more follow the straight portal grade (de.json tunnel block).
    const std::string d52 = "g2m.roads.geom/1 class-policy-2026-10-02 spacing=5 max-samples=2048 iterations=4096 curvature-exceptions=true "
        "sloping-plane-r3 approach=32 height-correction=.25 merge-agreement=.5 incline-soft=5 dependency-margin=65 coverage=graph xs=1 enc=8 rules=de:" +
        rules_hex + " join-bisector=1 context=nodes junction=1 furniture=1 completion=1 completion-retry=20/16384 completion-abutment=32/200 "
        "completion-ground=40/32 strict-anchor=1 ring-nesting=2mm bridge-far-end=1 ground-ease=8/0.05 corner-fillet=1 width-taper=5"
        " tunnel-floor=5/0.29999999999999999/20/chord:200 bridge-anchor-reuse=1 bridge-walk-margin=205";
    CHECK(deriver.config_hash() != Sha256::of(d52));
    // Owner decision 2026-10-15 (D53-D58): underpass decks, enc=9, and the crossing context.
    const std::string after = "g2m.roads.geom/1 class-policy-2026-10-02 spacing=5 max-samples=2048 iterations=4096 curvature-exceptions=true "
        "sloping-plane-r3 approach=32 height-correction=.25 merge-agreement=.5 incline-soft=5 dependency-margin=65 coverage=graph xs=1 enc=9 rules=de:" +
        rules_hex + " join-bisector=1 context=nodes junction=1 furniture=1 completion=1 completion-retry=20/16384 completion-abutment=32/200 "
        "completion-ground=40/32 strict-anchor=1 ring-nesting=2mm bridge-far-end=1 ground-ease=8/0.05 corner-fillet=1 width-taper=5"
        " tunnel-floor=5/0.29999999999999999/20/chord:200 bridge-anchor-reuse=1 bridge-walk-margin=205 underpass=1 crossing-context=1";
    CHECK(road_geom_config_text() == after);
    CHECK(deriver.config_hash() == Sha256::of(after));
    CHECK(road_geom_desc().encoding == 9);
    // The text is built from served_road_completion_options(): pin those values, so a changed default fails here and
    // in the literal text above together.
    {
        const auto served = served_road_completion_options();
        CHECK(served.enabled); CHECK(served.retry_spacing_m == 20); CHECK(served.retry_iterations == 16384);
        CHECK(served.abutment_ground_m == 32); CHECK(served.abutment_path_m == 200); CHECK(served.ground_spacing_m == 40); CHECK(served.ground_ramp_m == 32);
        CHECK(served.tunnel_floors); CHECK(served.tunnel_spacing_m == 5); CHECK(served.tunnel_max_grade == 0.30); CHECK(served.tunnel_min_radius_m == 20);
        CHECK(served.tunnel_chord_min_length_m == 200); CHECK(served.tunnel_chord_min_length_m == builtin_road_rules_de().tunnel.straight_grade_min_length_m); CHECK(served.bridge_anchor_reuse);
    }

    struct Temp {
        std::filesystem::path path = std::filesystem::temp_directory_path() /
            ("g2m_geom_rid_" + std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()));
        ~Temp() { std::error_code ec; std::filesystem::remove_all(path, ec); }
    } temp;
    auto sources = TileStore::open({temp.path, "sources"}); REQUIRE(sources);
    BuiltinTerrainDerivers derivers;
    const auto rid = [&](bool roads, bool geometry) {
        const auto manifest = make_local_release(**sources, builtin_local_release_params("geom_rid", derivers, {}, roads, geometry));
        REQUIRE(manifest); return manifest->rid;
    };
    const auto terrain = rid(false, false), graph = rid(true, false), geometry = rid(true, true);
    CHECK(terrain != graph); CHECK(graph != geometry); CHECK(terrain != geometry);
    std::printf("release ids (empty source store): terrain %s, roads.graph %s, roads.geom %s\n", to_hex(terrain).c_str(),
        to_hex(graph).c_str(), to_hex(geometry).c_str());
    // A terrain-only release is a function of the terrain layers alone: it ignores the road derivers.
    BuiltinTerrainDerivers fresh;
    const auto again = make_local_release(**sources, builtin_local_release_params("geom_rid", fresh, {}, false, false));
    REQUIRE(again); CHECK(again->rid == terrain);
    CHECK(derivers.ordered().size() == 2);
}
