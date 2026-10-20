// Street fidelity P2 slice 8: roads.geom encoding 7 (design 9.1-9.3) - the junction, furniture and provenance sections,
// every rejection of the 9.2 validation with its own malformed body (T8), and the ownership filter's tile invariance on
// the two committed OSM fixtures with its negative control (T7).
#include "road_fixture.h"
#include "road_junction_scene.h"
#include "g2m/core/bytes.h"
#include "g2m/layer/road_furniture.h"
#include "g2m/layer/road_junction.h"
#include <catch2/catch_test_macros.hpp>
#include <algorithm>
#include <bit>
#include <cmath>
#include <cstdio>
#include <functional>
#include <map>
#include <optional>
#include <set>
#include <string>
#include <utility>
using namespace g2m;
namespace {
const RoadGeomTile& scene() {
    static const RoadGeomTile tile = [] { auto t = g2m_test::junction_scene_tile(); if (!t) std::abort(); return std::move(*t); }();
    return tile;
}
// Sections 5-7 written from the format description of design 9.1 / docs/formats/road_geom.md, independently of the
// library's writer: equal bytes on a valid tile pin the format, and on a mutated tile they are the malformed body
// the decoder must reject.
std::vector<std::vector<std::uint8_t>> sections_5_to_7(const RoadGeomTile& t) {
    const auto f64 = [](ByteWriter& w, double d) { w.u64(std::bit_cast<std::uint64_t>(d)); };
    ByteWriter j; j.varint(t.junctions.size());
    for (const auto& r : t.junctions) {
        j.svarint(r.nodes.empty() ? 0 : r.nodes.front()); j.u16(r.flags); j.u16(static_cast<std::uint16_t>(r.decline));
        j.varint(r.nodes.size()); for (const auto n : r.nodes) j.svarint(n);
        j.varint(r.approaches.size());
        for (const auto& a : r.approaches) { j.varint(a.entry_index); j.u8(a.at_end ? 1 : 0); j.u8(static_cast<std::uint8_t>(a.status)); f64(j, a.trim_m); }
        j.varint(r.absorbed.size()); for (const auto e : r.absorbed) j.varint(e);
        for (const auto& ring : r.rings) {
            j.varint(ring.vertices.size());
            for (std::size_t i = 0; i < ring.vertices.size(); ++i) {
                const auto p = ring.vertices[i]; const auto q = i ? ring.vertices[i - 1] : geom::PointMm{0, 0};
                j.svarint(p.x - q.x); j.svarint(p.y - q.y);
            }
        }
        f64(j, r.raised_height_m); j.svarint(r.raised_node);
    }
    ByteWriter u; u.varint(t.furniture.size());
    for (const auto& f : t.furniture) {
        u.u8(static_cast<std::uint8_t>(f.kind)); u.u16(static_cast<std::uint16_t>(f.decline));
        u.svarint(f.node_id); u.varint(f.entry_index); u.varint(f.junction_index); f64(u, f.station_m); u.u8(static_cast<std::uint8_t>(f.travel));
        u.svarint(std::llround(f.t_from * 1000)); u.svarint(std::llround(f.t_to * 1000)); u.varint(static_cast<std::uint64_t>(std::llround(f.length_m * 1000)));
        u.u8(static_cast<std::uint8_t>(f.pattern)); u.u8(f.flags);
    }
    ByteWriter p; p.varint(t.origins.size());
    for (const auto& o : t.origins) { p.u8(static_cast<std::uint8_t>(o.provenance)); p.u16(o.strict_decline_code); }
    return {j.take(), u.take(), p.take()};
}
TileBody forged(const RoadGeomTile& t) {
    auto body = *encode_road_geom(scene());
    auto s = sections_5_to_7(t);
    for (std::size_t i = 0; i < 3; ++i) body.sections[4 + i].data = std::move(s[i]);
    return body;
}
std::size_t junction_at(const RoadGeomTile& t, std::int64_t owner) {
    for (std::size_t i = 0; i < t.junctions.size(); ++i) if (t.junctions[i].nodes.front() == owner) return i;
    FAIL("no junction at " << owner); return 0;
}
std::size_t furniture_of(const RoadGeomTile& t, FurnitureKind kind, FurnitureDecline decline = FurnitureDecline::None) {
    for (std::size_t i = 0; i < t.furniture.size(); ++i) if (t.furniture[i].kind == kind && t.furniture[i].decline == decline) return i;
    FAIL("no such furniture"); return 0;
}
// One 9.2 rule: the mutated tile fails validation with `message`, the encoder refuses it, and (when the format can
// express the defect at all) the body carrying the mutated records in sections 5-7 fails to decode.
void rejects(const char* name, const char* message, const std::function<void(RoadGeomTile&)>& mutate, bool body_can_carry = true) {
    INFO(name);
    auto t = scene(); mutate(t);
    const auto valid = validate_road_geom_tile(t);
    CHECK_FALSE(valid);
    if (!valid) { INFO("validation says: " << valid.error().message); CHECK(valid.error().message.find(message) != std::string::npos); }
    CHECK_FALSE(encode_road_geom(t));
    if (body_can_carry) CHECK_FALSE(decode_road_geom(forged(t)));
}
}

TEST_CASE("T8: the scene tile carries junctions and furniture of every kind and round trips", "[layer][road_geom][enc7][T8]") {
    const auto& t = scene();
    CHECK(t.encoding == 8);
    REQUIRE(t.junctions.size() == 3);
    const auto& j3 = t.junctions[junction_at(t, 3)]; const auto& j5 = t.junctions[junction_at(t, 5)]; const auto& j7 = t.junctions[junction_at(t, 7)];
    CHECK(j3.decline == JunctionDecline::None); CHECK(j3.approaches.size() == 4);
    CHECK((j5.flags & static_cast<std::uint16_t>(JunctionFlags::RaisedTable)) != 0); CHECK(j5.raised_node == 5); CHECK(j5.raised_height_m > 0);
    CHECK(j7.decline == JunctionDecline::None);
    std::size_t crossings = 0, crossings_declined = 0, stops = 0, signals = 0;
    for (const auto& f : t.furniture) {
        if (f.kind == FurnitureKind::Crossing) { if (f.decline == FurnitureDecline::None) ++crossings; else ++crossings_declined; }
        if (f.kind == FurnitureKind::StopLine && f.decline == FurnitureDecline::None) ++stops;
        if (f.kind == FurnitureKind::Signal && f.decline == FurnitureDecline::None) ++signals;
    }
    std::printf("[enc7 scene] junctions %zu, furniture %zu (crossings placed %zu, declined %zu, stop lines %zu, signals %zu), entries %zu\n",
        t.junctions.size(), t.furniture.size(), crossings, crossings_declined, stops, signals, t.entries.size());
    CHECK(crossings == 1); CHECK(crossings_declined == 1); CHECK(stops == 6); CHECK(signals == 4);
    // The node outside the tile owns nothing here (ownership filter).
    for (const auto& j : t.junctions) CHECK(road_tile_contains(t.source.key, std::lower_bound(t.source.graph.nodes.begin(), t.source.graph.nodes.end(), j.nodes.front(),
        [](const auto& n, std::int64_t v) { return n.osm_id < v; })->position));
    // Round trip, stable bytes, and the independent writer agrees with the library on sections 5-7.
    const auto body = encode_road_geom(t); REQUIRE(body);
    REQUIRE(body->sections.size() == 7);
    const auto mine = sections_5_to_7(t);
    for (std::size_t i = 0; i < 3; ++i) CHECK(body->sections[4 + i].data == mine[i]);
    const auto back = decode_road_geom(*body); REQUIRE(back);
    CHECK(back->junctions == t.junctions); CHECK(back->furniture == t.furniture); CHECK(back->origins == t.origins);
    CHECK(back->features == t.features); CHECK(back->entries.size() == t.entries.size());
    CHECK(*encode_body(*encode_road_geom(*back)) == *encode_body(*body));
    // Every truncation and a trailing byte of sections 5-7 fails.
    for (std::size_t s = 4; s < 7; ++s) {
        for (std::size_t i = 0; i < body->sections[s].data.size(); ++i) { auto cut = *body; cut.sections[s].data.resize(i); CHECK_FALSE(decode_road_geom(cut)); }
        auto trailing = *body; trailing.sections[s].data.push_back(0); CHECK_FALSE(decode_road_geom(trailing));
    }
    // Encoding 6 cannot carry the records; the same tile without them still writes and reads as encoding 6.
    auto six = t; six.encoding = kRoadGeomEncodingPrevious; CHECK_FALSE(encode_road_geom(six));
    six.junctions.clear(); six.furniture.clear();
    const auto body6 = encode_road_geom(six); REQUIRE(body6); CHECK(body6->sections.size() == 4);
    for (std::size_t i = 0; i < 4; ++i) CHECK(body6->sections[i].data == body->sections[i].data);
    const auto back6 = decode_road_geom(*body6); REQUIRE(back6);
    CHECK(back6->encoding == 6); CHECK(back6->junctions.empty()); CHECK(back6->furniture.empty()); CHECK(back6->origins.empty());
}

TEST_CASE("T8: every rule of the 9.2 validation rejects its own malformed body", "[layer][road_geom][enc7][T8]") {
    REQUIRE(validate_road_geom_tile(scene()));
    CHECK(decode_road_geom(forged(scene())));   // the forging itself keeps a valid tile valid
    // Ownership and node lists.
    rejects("owner outside the tile", "owner outside the tile", [](auto& t) { auto& j = t.junctions[junction_at(t, 7)]; j.nodes = {20}; });
    rejects("nodes not ascending", "not ascending", [](auto& t) { auto& j = t.junctions[junction_at(t, 3)]; j.nodes = {3, 2}; });
    rejects("node not in the graph", "not in the graph", [](auto& t) { t.junctions[junction_at(t, 3)].nodes.push_back(999999); });
    rejects("junction order", "junction order", [](auto& t) { std::swap(t.junctions[0], t.junctions[1]); });
    rejects("flags outside the mask", "junction flags", [](auto& t) { t.junctions[0].flags |= 32; });
    rejects("decline out of range", "junction decline", [](auto& t) { t.junctions[0].decline = static_cast<JunctionDecline>(7); });
    rejects("cluster flag on one node", "cluster flag", [](auto& t) { t.junctions[0].flags |= static_cast<std::uint16_t>(JunctionFlags::Cluster); });
    // Indices, status, ends, trims.
    rejects("approach entry index", "approach entry", [](auto& t) { t.junctions[0].approaches[0].entry_index = t.entries.size(); });
    rejects("approach status", "approach status", [](auto& t) { t.junctions[0].approaches[0].status = JunctionApproachStatus::NoProfile; });
    rejects("approach at the wrong end", "approach end", [](auto& t) { t.junctions[0].approaches[0].at_end = !t.junctions[0].approaches[0].at_end; });
    rejects("negative trim", "junction trim", [](auto& t) { t.junctions[0].approaches[0].trim_m = -0.5; });
    rejects("trim beyond L - 0.5", "junction trim", [](auto& t) {
        auto& a = t.junctions[0].approaches[0]; a.trim_m = t.entries[a.entry_index].profile->reference.length_m; });
    rejects("approach repeated", "approach repeated", [](auto& t) { t.junctions[0].approaches.push_back(t.junctions[0].approaches[0]); });
    // Rings (the design's sabotage: one flipped vertex is non-simple).
    rejects("flipped ring vertex", "not simple", [](auto& t) {
        auto& v = t.junctions[junction_at(t, 3)].rings[2].vertices; std::swap(v[0], v[1]); });
    rejects("clockwise ring", "not simple", [](auto& t) { auto& v = t.junctions[junction_at(t, 3)].rings[1].vertices; std::reverse(v.begin(), v.end()); });
    rejects("rings not nested", "not nested", [](auto& t) {
        auto& ring = t.junctions[junction_at(t, 3)].rings[2].vertices;   // the outer ring shrunk to a quarter around its first vertex
        const auto c = ring[0]; for (auto& p : ring) p = {c.x + (p.x - c.x) / 4, c.y + (p.y - c.y) / 4}; });
    rejects("ring missing on a built junction", "ring count", [](auto& t) { t.junctions[0].rings[1].vertices.clear(); });
    rejects("ring above the vertex cap", "ring size", [](auto& t) { t.junctions[0].rings[0].vertices.resize(kJunctionMaxRingVertices + 1, {0, 0}); });
    rejects("ring beyond 1 km of the owner", "ring reach", [](auto& t) {
        for (auto& ring : t.junctions[0].rings) for (auto& p : ring.vertices) p.x += 2000000; });
    rejects("raised height above 0.3 m", "raised table", [](auto& t) { t.junctions[junction_at(t, 5)].raised_height_m = 0.5; });
    rejects("raised height without the flag", "raised table", [](auto& t) { t.junctions[junction_at(t, 3)].raised_height_m = 0.1; });
    rejects("raised node not held", "raised table", [](auto& t) { t.junctions[junction_at(t, 5)].raised_node = 3; });
    // Furniture.
    rejects("furniture kind", "furniture enum", [](auto& t) { t.furniture[0].kind = static_cast<FurnitureKind>(4); });
    rejects("furniture decline", "furniture enum", [](auto& t) { t.furniture[0].decline = static_cast<FurnitureDecline>(kFurnitureDeclineCount); });
    rejects("reserved flag bit 3", "furniture flags", [](auto& t) { t.furniture[furniture_of(t, FurnitureKind::Crossing)].flags |= kFurnitureInsideJunction; });
    rejects("flag outside the mask", "furniture flags", [](auto& t) { t.furniture[0].flags |= 32; });
    rejects("furniture entry index", "furniture index", [](auto& t) { t.furniture[0].entry_index = t.entries.size(); });
    rejects("furniture junction index", "furniture index", [](auto& t) { t.furniture[0].junction_index = static_cast<std::uint32_t>(t.junctions.size() + 1); });
    rejects("furniture order", "furniture order", [](auto& t) { std::swap(t.furniture[0], t.furniture.back()); });
    rejects("furniture node", "furniture node", [](auto& t) { t.furniture[furniture_of(t, FurnitureKind::Crossing)].node_id = 999999; });
    // The body stores integer mm, so this one is an encoder-side rule only.
    rejects("t not mm-exact", "furniture value", [](auto& t) { t.furniture[furniture_of(t, FurnitureKind::Crossing)].t_to += 0.0004; }, false);
    rejects("crossing station beyond the entry", "furniture station", [](auto& t) {
        auto& f = t.furniture[furniture_of(t, FurnitureKind::Crossing)]; f.station_m = t.entries[f.entry_index].profile->reference.length_m + 50; });
    rejects("crossing with t_from >= t_to", "crossing record", [](auto& t) {
        auto& f = t.furniture[furniture_of(t, FurnitureKind::Crossing)]; std::swap(f.t_from, f.t_to); });
    // The design's sabotage: one line's t_to moved, the lane span recomputed from the entry rejects it.
    rejects("line t_to off its lanes", "lane span", [](auto& t) { t.furniture[furniture_of(t, FurnitureKind::StopLine)].t_to += 0.25; });
    rejects("signal with a length", "signal record", [](auto& t) { t.furniture[furniture_of(t, FurnitureKind::Signal)].length_m = 1; });
    rejects("declined crossing with an extent", "declined furniture extent", [](auto& t) {
        t.furniture[furniture_of(t, FurnitureKind::Crossing, FurnitureDecline::JunctionNode)].length_m = 4; });
    // Provenance.
    rejects("provenance count", "origins count", [](auto& t) { t.origins.pop_back(); });
    rejects("strict provenance naming a decline", "provenance record", [](auto& t) { t.origins[0].strict_decline_code = 4; });
    rejects("provenance out of range", "provenance", [](auto& t) { t.origins[0].provenance = static_cast<ProfileProvenance>(4); });
}

TEST_CASE("T8: encoding-6 and encoding-5 bodies still decode, without junctions, furniture or origins", "[layer][road_geom][enc7][T8]") {
    auto six = scene(); six.encoding = kRoadGeomEncodingPrevious; six.junctions.clear(); six.furniture.clear();
    const auto body6 = encode_road_geom(six); REQUIRE(body6);
    const auto back6 = decode_road_geom(*body6); REQUIRE(back6);
    CHECK(back6->encoding == 6); CHECK(back6->junctions.empty()); CHECK(back6->furniture.empty()); CHECK(back6->origins.empty());
    // An encoding-6 decode written back is encoding 6, byte for byte.
    CHECK(*encode_body(*encode_road_geom(*back6)) == *encode_body(*body6));
    const auto legacy_graph = encode_road_graph(six.source, kRoadGraphEncodingLegacy); REQUIRE(legacy_graph);
    const TileBody five{kRoadGeomEncodingLegacy, 1, {legacy_graph->sections[0], body6->sections[1]}};
    const auto back5 = decode_road_geom(five); REQUIRE(back5);
    CHECK(back5->encoding == 5); CHECK(back5->junctions.empty()); CHECK(back5->furniture.empty()); CHECK(back5->origins.empty());
}

namespace {
// T7 on a committed fixture: every served tile, each junction stored by exactly the tile of its owner node and equal to
// what any other tile builds for that owner; junction furniture stored once with its junction; entry furniture equal in
// every tile that holds the entry. Records are compared in a tile-independent form (entries as way:start_ref).
struct Invariance {
    std::size_t tiles = 0, junctions_stored = 0, owners_stored_twice = 0, owners_lost = 0, owners_built_elsewhere = 0;
    std::size_t junction_mismatch = 0, junction_furniture = 0, entry_furniture_repeated = 0, entry_furniture_mismatch = 0;
};
std::string entry_key(const RoadGeomTile& t, std::size_t i) { return std::to_string(t.entries[i].way_id) + ":" + std::to_string(t.entries[i].stretch.start_ref); }
std::string junction_key(const RoadGeomTile& t, const RoadJunction& j) {
    std::string s = "n";
    for (const auto n : j.nodes) s += " " + std::to_string(n);
    s += " f" + std::to_string(j.flags) + " d" + std::to_string(static_cast<int>(j.decline)) + " a";
    for (const auto& a : j.approaches) s += " " + entry_key(t, a.entry_index) + (a.at_end ? "e" : "s") + std::to_string(static_cast<int>(a.status)) + "/" +
        std::to_string(std::bit_cast<std::uint64_t>(a.trim_m));
    std::vector<std::string> absorbed; for (const auto e : j.absorbed) absorbed.push_back(entry_key(t, e));
    std::sort(absorbed.begin(), absorbed.end()); s += " b"; for (const auto& e : absorbed) s += " " + e;
    for (const auto& ring : j.rings) { s += " r"; for (const auto& p : ring.vertices) s += " " + std::to_string(p.x) + "," + std::to_string(p.y); }
    return s + " h" + std::to_string(std::bit_cast<std::uint64_t>(j.raised_height_m)) + " " + std::to_string(j.raised_node);
}
std::string furniture_key(const RoadGeomTile& t, const RoadFurniture& f) {
    char buf[256];
    std::snprintf(buf, sizeof buf, "%d %d %lld %llx %d %lld %lld %lld %d %u", static_cast<int>(f.kind), static_cast<int>(f.decline), static_cast<long long>(f.node_id),
        static_cast<unsigned long long>(std::bit_cast<std::uint64_t>(f.station_m)), static_cast<int>(f.travel), std::llround(f.t_from * 1000), std::llround(f.t_to * 1000),
        std::llround(f.length_m * 1000), static_cast<int>(f.pattern), static_cast<unsigned>(f.flags));
    const bool no_entry = f.decline == FurnitureDecline::JunctionDeclined;   // entry 0 is a placeholder for these
    return (no_entry ? std::string("-") : entry_key(t, f.entry_index)) + " " + buf;
}
Invariance invariance_of(const std::string& pbf, bool with_context) {
    const auto data = g2m_test::read_osm_fixture(std::string(G2M_SOURCE_DIR) + "/testdata/fixtures/osm/" + pbf);
    const auto world = build_road_graph(data, g2m_test::kZone32N); REQUIRE(world);
    const auto world_builder = RoadReferenceBuilder::make(*world); REQUIRE(world_builder);
    std::set<TileKey> keys;
    for (const auto& n : world->nodes) {
        const auto k = tile_key_at(g2m_test::kZone32N, 2, static_cast<double>(n.position.x) / 1000.0, static_cast<double>(n.position.y) / 1000.0);
        if (k) keys.insert(*k);
    }
    Invariance v;
    // (owner, ClusterTooLarge): a ClusterTooLarge record shares its owner with that node's single-node record.
    using Owner = std::pair<std::int64_t, bool>;
    const auto owner_of = [](const RoadJunction& j) { return Owner{j.nodes.front(), j.decline == JunctionDecline::ClusterTooLarge}; };
    std::map<Owner, std::string> stored;                 // the stored record
    std::map<Owner, std::set<std::string>> built;        // every record any tile builds for it (world context)
    std::map<std::string, std::vector<std::string>> entry_furniture;
    for (const auto& key : keys) {
        const auto probe = select_road_graph_tile(*world, key);
        if (!probe || probe->owned_edges.empty()) continue;
        g2m_test::GeomOptions o; o.with_context = with_context; o.completion = served_road_completion_options();
        const auto tile = g2m_test::geometry_in_tile_with(*world, key, o); REQUIRE(tile);
        ++v.tiles;
        CHECK(validate_road_geom_tile(*tile));
        for (const auto& j : tile->junctions) {
            ++v.junctions_stored;
            if (!stored.emplace(owner_of(j), junction_key(*tile, j)).second) ++v.owners_stored_twice;
        }
        // What this tile builds for every owner it can see, before the ownership filter (the world as context).
        if (const auto all = build_road_junctions(*tile, *world_builder)) for (const auto& j : *all) built[owner_of(j)].insert(junction_key(*tile, j));
        std::map<std::string, std::vector<std::string>> mine;
        for (const auto& f : tile->furniture) {
            if (f.junction_index) { ++v.junction_furniture; continue; }
            mine[entry_key(*tile, f.entry_index)].push_back(furniture_key(*tile, f));
        }
        for (std::size_t i = 0; i < tile->entries.size(); ++i) {
            auto& list = mine[entry_key(*tile, i)]; std::sort(list.begin(), list.end());
            const auto [it, inserted] = entry_furniture.emplace(entry_key(*tile, i), list);
            if (!inserted && !list.empty()) { ++v.entry_furniture_repeated; if (it->second != list) ++v.entry_furniture_mismatch; }
        }
    }
    // Every owner some tile builds is stored, and every tile that builds it builds exactly the stored record.
    for (const auto& [owner, records] : built) {
        const auto it = stored.find(owner);
        if (it == stored.end()) { ++v.owners_lost; continue; }
        if (records.size() != 1 || !records.count(it->second)) ++v.junction_mismatch;
        ++v.owners_built_elsewhere;
    }
    for (const auto& [owner, record] : stored) if (!built.count(owner)) ++v.junction_mismatch;
    return v;
}
}

TEST_CASE("T7: every junction is owned exactly once and equals what every tile builds; furniture follows its owner", "[layer][road_geom][enc7][T7][roads]") {
    std::size_t junction_furniture = 0;   // frankfurt_hoechst has no signalised or signed junction
    for (const char* pbf : {"frankfurt_hoechst.osm.pbf", "hoechst_a66.osm.pbf"}) {
        INFO(pbf);
        const auto v = invariance_of(pbf, true);
        std::printf("[T7] %s: tiles %zu, junction records stored %zu (owners stored twice %zu, lost %zu, differing from another tile's build %zu), "
                    "junction furniture %zu, entry furniture repeated %zu (differing %zu)\n",
            pbf, v.tiles, v.junctions_stored, v.owners_stored_twice, v.owners_lost, v.junction_mismatch, v.junction_furniture, v.entry_furniture_repeated, v.entry_furniture_mismatch);
        CHECK(v.junctions_stored > 50); junction_furniture += v.junction_furniture; CHECK(v.entry_furniture_repeated > 0);
        CHECK(v.owners_stored_twice == 0); CHECK(v.owners_lost == 0); CHECK(v.junction_mismatch == 0); CHECK(v.entry_furniture_mismatch == 0);
        // Negative control: degrees from the tile graph alone (P1 slice 1's control).
        const auto c = invariance_of(pbf, false);
        std::printf("[T7] %s control (tile graph as context): owners stored twice %zu, lost %zu, differing %zu, entry furniture differing %zu\n",
            pbf, c.owners_stored_twice, c.owners_lost, c.junction_mismatch, c.entry_furniture_mismatch);
        CHECK(c.owners_stored_twice + c.owners_lost + c.junction_mismatch + c.entry_furniture_mismatch > 0);
    }
    CHECK(junction_furniture > 0);
}
