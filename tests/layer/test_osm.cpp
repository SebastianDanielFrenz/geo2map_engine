// OSM: canonical builder, src.osm round trip, clip selection contract
// (docs/formats/src_osm.md) and the exact segment/box test.

#include "g2m/core/geo/utm.h"
#include "g2m/layer/osm_clip.h"
#include "g2m/layer/src_osm.h"

#include "splitmix64.h"
#include "synthetic.h"

#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <cstdint>
#include <limits>
#include <set>
#include <vector>

using namespace g2m;
using osm::MemberType;
using osm::OsmBuilder;
using osm::OsmData;

namespace {

Result<OsmTile> osm_round_trip(const OsmTile& t) {
    auto body = encode_src_osm(t);
    if (!body) return body.error();
    auto enc = encode_body(*body);
    if (!enc) return enc.error();
    auto dec = decode_body(*enc);
    if (!dec) return dec.error();
    return decode_src_osm(*dec);
}

template <class T>
std::set<std::int64_t> ids_of(const std::vector<T>& v) {
    std::set<std::int64_t> s;
    for (const T& e : v) s.insert(e.id);
    return s;
}

// Node at a UTM 32N position (metres), as OSM 1e-7 degrees.
void add_utm_node(OsmBuilder& b, std::int64_t id, double e, double n, OsmBuilder::TagList tags = {}) {
    const geo::Utm utm(geo::kGRS80);
    const geo::TmReverse g = utm.reverse(g2m_test::kZone32N, e, n);
    b.add_node(id, g2m_test::to_e7(g.lat), g2m_test::to_e7(g.lon), std::move(tags));
}

// Reference for segment_touches_box: Liang-Barsky with exact rational
// parameters, for small coordinates (|v| <= 64, so every product fits).
bool reference_touches(std::int64_t ax, std::int64_t ay, std::int64_t bx, std::int64_t by, std::int64_t x0,
                       std::int64_t y0, std::int64_t x1, std::int64_t y1) {
    // Parameter interval [lo_p/lo_q, hi_p/hi_q], denominators > 0.
    std::int64_t lo_p = 0, lo_q = 1, hi_p = 1, hi_q = 1;
    const auto axis = [&](std::int64_t a, std::int64_t d, std::int64_t lo, std::int64_t hi) {
        if (d == 0) return a >= lo && a <= hi;
        std::int64_t t0p = lo - a, t1p = hi - a, q = d;
        if (q < 0) {
            q = -q;
            t0p = -t0p;
            t1p = -t1p;
            std::swap(t0p, t1p);
        }
        if (t0p * lo_q > lo_p * q) {
            lo_p = t0p;
            lo_q = q;
        }
        if (t1p * hi_q < hi_p * q) {
            hi_p = t1p;
            hi_q = q;
        }
        return true;
    };
    if (!axis(ax, bx - ax, x0, x1)) return false;
    if (!axis(ay, by - ay, y0, y1)) return false;
    return lo_p * hi_q <= hi_p * lo_q;
}

} // namespace

TEST_CASE("osm: builder output is canonical and order independent", "[layer][osm]") {
    const OsmData a = g2m_test::make_osm_set(1, g2m_test::home_tile());
    REQUIRE(a.validate());
    REQUIRE(a.nodes.size() == 600);
    REQUIRE(a.ways.size() == 90);
    REQUIRE(a.relations.size() == 12);

    // Rebuild from a in reverse element order: same canonical data.
    OsmBuilder b;
    const auto tags = [&](std::span<const osm::Tag> ts) {
        OsmBuilder::TagList tl;
        for (auto it = ts.rbegin(); it != ts.rend(); ++it) tl.emplace_back(a.str(it->key), a.str(it->value));
        return tl;
    };
    for (auto it = a.relations.rbegin(); it != a.relations.rend(); ++it) {
        std::vector<OsmBuilder::MemberIn> m;
        for (const osm::Member& x : a.members_of(*it)) m.push_back({x.type, x.ref, std::string(a.str(x.role))});
        b.add_relation(it->id, std::move(m), tags(a.tags_of(*it)));
    }
    for (auto it = a.ways.rbegin(); it != a.ways.rend(); ++it) {
        const auto r = a.refs_of(*it);
        b.add_way(it->id, std::vector<std::int64_t>(r.begin(), r.end()), tags(a.tags_of(*it)));
    }
    for (auto it = a.nodes.rbegin(); it != a.nodes.rend(); ++it) b.add_node(it->id, it->lat, it->lon, tags(a.tags_of(*it)));
    auto rebuilt = b.build();
    REQUIRE(rebuilt);
    CHECK(*rebuilt == a);
}

TEST_CASE("osm: builder rejects invalid input", "[layer][osm]") {
    {
        OsmBuilder b;
        b.add_node(1, 0, 0);
        b.add_node(1, 0, 0);
        CHECK_FALSE(b.build());
    }
    {
        OsmBuilder b;
        b.add_node(1, 0, 0, {{"a", "x"}, {"a", "y"}});
        CHECK_FALSE(b.build());
    }
    {
        OsmBuilder b;
        b.add_node(1, 0, 0);
        b.add_way(2, {1, 3});
        CHECK_FALSE(b.build()); // ref to a missing node
    }
    {
        OsmBuilder b;
        b.add_way(2, {});
        CHECK_FALSE(b.build());
    }
    {
        OsmBuilder b;
        b.add_node(1, osm::kMaxLatE7 + 1, 0);
        CHECK_FALSE(b.build());
    }
    {
        OsmBuilder b;
        b.add_node(1, 0, 0, {{std::string(osm::kMaxStringLength + 1, 'a'), "x"}});
        CHECK_FALSE(b.build());
    }
    {
        OsmBuilder b; // members may be absent from the set
        b.add_node(1, 0, 0);
        b.add_relation(5, {{MemberType::Way, 77, "outer"}, {MemberType::Relation, 5, ""}});
        CHECK(b.build());
    }
}

TEST_CASE("osm: validate() catches non-canonical data", "[layer][osm]") {
    const OsmData good = g2m_test::make_osm_set(2, g2m_test::home_tile(), 50, 10, 3);
    REQUIRE(good.validate());
    OsmData d = good;
    std::swap(d.nodes[0], d.nodes[1]);
    CHECK_FALSE(d.validate());
    d = good;
    std::swap(d.strings[0], d.strings[1]);
    CHECK_FALSE(d.validate());
    d = good;
    d.way_refs[0] = 123; // not a node
    CHECK_FALSE(d.validate());
    d = good;
    d.ways[0].refs_count += 1000;
    CHECK_FALSE(d.validate());
    d = good;
    d.members.push_back(osm::Member{}); // pool longer than its users
    CHECK_FALSE(d.validate());
}

TEST_CASE("src.osm: round trip is lossless", "[layer][osm]") {
    for (std::uint64_t seed = 1; seed <= 4; ++seed) {
        OsmTile t{kDefaultOsmHaloM, g2m_test::make_osm_set(seed, g2m_test::home_tile())};
        auto back = osm_round_trip(t);
        REQUIRE(back);
        CHECK(*back == t);
        CHECK(back->data.validate());
    }
    // Empty tile.
    auto e = osm_round_trip(OsmTile{});
    REQUIRE(e);
    CHECK(*e == OsmTile{});

    // Extreme ids and coordinates (wrapping deltas).
    constexpr std::int64_t lo = std::numeric_limits<std::int64_t>::min();
    constexpr std::int64_t hi = std::numeric_limits<std::int64_t>::max();
    OsmBuilder b;
    b.add_node(lo, -osm::kMaxLatE7, -osm::kMaxLonE7, {{"", ""}});
    b.add_node(-1, osm::kMaxLatE7, osm::kMaxLonE7);
    b.add_node(0, 0, 0, {{"k", std::string(osm::kMaxStringLength, 'v')}});
    b.add_node(hi, -osm::kMaxLatE7, osm::kMaxLonE7);
    b.add_way(lo, {hi, lo, hi, 0, -1});
    b.add_way(hi, {0});
    b.add_relation(lo, {{MemberType::Relation, hi, "x"}, {MemberType::Node, lo, ""}, {MemberType::Way, 5, "y"}});
    b.add_relation(hi, {});
    auto d = b.build();
    REQUIRE(d);
    OsmTile t{0xffffffffu, *d};
    auto back = osm_round_trip(t);
    REQUIRE(back);
    CHECK(*back == t);
}

TEST_CASE("src.osm: decoder survives mutations", "[layer][osm]") {
    OsmTile t{kDefaultOsmHaloM, g2m_test::make_osm_set(9, g2m_test::home_tile(), 80, 15, 5)};
    auto body = encode_src_osm(t);
    REQUIRE(body);
    g2m_test::SplitMix64 r(17);
    int accepted = 0;
    for (int iter = 0; iter < 3000; ++iter) {
        TileBody m = *body;
        const std::size_t s = static_cast<std::size_t>(r.next() % m.sections.size());
        auto& data = m.sections[s].data;
        if (data.empty()) continue;
        const std::uint64_t op = r.next() % 3;
        const std::size_t pos = static_cast<std::size_t>(r.next() % data.size());
        if (op == 0) {
            data[pos] = static_cast<std::uint8_t>(r.next());
        } else if (op == 1) {
            data.resize(pos);
        } else {
            data.insert(data.begin() + static_cast<std::ptrdiff_t>(pos), static_cast<std::uint8_t>(r.next()));
        }
        auto d = decode_src_osm(m);
        if (d) {
            ++accepted;
            CHECK(d->data.validate()); // anything accepted is canonical
        }
    }
    CHECK(accepted < 3000);
}

TEST_CASE("osm clip: segment_touches_box matches an exact reference", "[layer][osm]") {
    g2m_test::SplitMix64 r(21);
    const auto v = [&](void) { return static_cast<std::int64_t>(r.next() % 41) - 20; };
    int touching = 0;
    for (int iter = 0; iter < 200000; ++iter) {
        const std::int64_t ax = v();
        const std::int64_t ay = v();
        const std::int64_t bx = v();
        const std::int64_t by = v();
        const std::int64_t xa = v();
        const std::int64_t xb = v();
        const std::int64_t ya = v();
        const std::int64_t yb = v();
        const std::int64_t x0 = std::min(xa, xb), x1 = std::max(xa, xb);
        const std::int64_t y0 = std::min(ya, yb), y1 = std::max(ya, yb);
        const bool ref = reference_touches(ax, ay, bx, by, x0, y0, x1, y1);
        touching += ref ? 1 : 0;
        REQUIRE(segment_touches_box(ax, ay, bx, by, x0, y0, x1, y1) == ref);
        // Scaled by 2^40 and shifted near the documented |v| < 2^62 limit:
        // the answer is invariant, and every product needs 128 bits.
        constexpr std::int64_t s = std::int64_t{1} << 40;
        constexpr std::int64_t off = std::int64_t{1} << 61;
        REQUIRE(segment_touches_box(ax * s + off, ay * s - off, bx * s + off, by * s - off, x0 * s + off,
                                    y0 * s - off, x1 * s + off, y1 * s - off) == ref);
    }
    CHECK(touching > 1000);
    CHECK(touching < 199000);
}

TEST_CASE("osm clip: selection contract", "[layer][osm]") {
    // Tile home_tile(): E [358400, 359424], N [5580800, 5581824]; with the
    // 128 m halo the box is E [358272, 359552], N [5580672, 5581952].
    OsmBuilder b;
    add_utm_node(b, 1, 358900, 5581300, {{"name", "inside"}});
    add_utm_node(b, 2, 358000, 5581300);
    add_utm_node(b, 3, 359900, 5581300);
    add_utm_node(b, 4, 358900, 5582500);
    add_utm_node(b, 5, 359000, 5582600, {{"name", "north"}});
    add_utm_node(b, 6, 359500, 5581300); // in the halo only
    add_utm_node(b, 7, 359500, 5582100); // with 8: passes outside the NE corner,
    add_utm_node(b, 8, 359700, 5581900); // though its bounding box overlaps B
    b.add_node(9, 500000000, 1000000000); // lon 100 deg: "far" from zone 32
    b.add_way(10, {2, 3}, {{"highway", "primary"}}); // crosses B, no node in it
    b.add_way(11, {4, 5}, {{"highway", "track"}});   // north of B
    b.add_way(12, {1, 4});                           // leaves B: keeps node 4
    b.add_way(13, {7, 8});                           // corner miss
    b.add_way(14, {1, 9});                           // IN node to a far node
    b.add_way(15, {9, 4});                           // far node to an outside node
    b.add_relation(100, {{MemberType::Way, 10, "outer"}, {MemberType::Way, 11, "inner"}});
    b.add_relation(101, {{MemberType::Relation, 100, "sub"}});
    b.add_relation(102, {{MemberType::Way, 11, ""}, {MemberType::Node, 5, "label"}});
    b.add_relation(103, {{MemberType::Node, 1, ""}});
    b.add_relation(104, {{MemberType::Relation, 101, ""}});
    b.add_relation(105, {{MemberType::Relation, 102, ""}});
    auto set = b.build();
    REQUIRE(set);

    auto clip = clip_osm(*set, g2m_test::home_tile(), kDefaultOsmHaloM);
    REQUIRE(clip);
    const OsmData& d = clip->data;
    CHECK(clip->halo_m == kDefaultOsmHaloM);
    REQUIRE(d.validate());
    CHECK(ids_of(d.nodes) == std::set<std::int64_t>{1, 2, 3, 4, 6, 9});
    CHECK(ids_of(d.ways) == std::set<std::int64_t>{10, 12, 14});
    CHECK(ids_of(d.relations) == std::set<std::int64_t>{100, 101, 103, 104});
    // Relation 100 keeps its complete member list, including way 11.
    const osm::Relation* r100 = d.find_relation(100);
    REQUIRE(r100);
    REQUIRE(r100->members_count == 2);
    CHECK(d.members_of(*r100)[1].ref == 11);
    CHECK(d.str(d.members_of(*r100)[1].role) == "inner");
    // Strings re-interned: "track" and "north" belong to excluded elements only.
    CHECK(std::find(d.strings.begin(), d.strings.end(), "track") == d.strings.end());
    CHECK(std::find(d.strings.begin(), d.strings.end(), "north") == d.strings.end());
    CHECK(d.tag(d.tags_of(*d.find_node(1)), "name") == "inside");

    // Without a halo node 6 is outside; the round trip holds.
    auto no_halo = clip_osm(*set, g2m_test::home_tile(), 0);
    REQUIRE(no_halo);
    CHECK_FALSE(ids_of(no_halo->data.nodes).contains(6));
    auto back = osm_round_trip(*clip);
    REQUIRE(back);
    CHECK(*back == *clip);
}

TEST_CASE("osm clip: tiles cover the set and halo only adds", "[layer][osm]") {
    const OsmData set = g2m_test::make_osm_set(4, g2m_test::home_tile());
    std::set<std::int64_t> nodes, ways, rels;
    for (int dy = -2; dy <= 2; ++dy) {
        for (int dx = -2; dx <= 2; ++dx) {
            auto c0 = clip_osm(set, g2m_test::home_tile(dx, dy), 0);
            auto c1 = clip_osm(set, g2m_test::home_tile(dx, dy), kDefaultOsmHaloM);
            REQUIRE(c0);
            REQUIRE(c1);
            REQUIRE(c0->data.validate());
            const auto n0 = ids_of(c0->data.nodes), n1 = ids_of(c1->data.nodes);
            const auto w0 = ids_of(c0->data.ways), w1 = ids_of(c1->data.ways);
            const auto r0 = ids_of(c0->data.relations), r1 = ids_of(c1->data.relations);
            CHECK(std::includes(n1.begin(), n1.end(), n0.begin(), n0.end()));
            CHECK(std::includes(w1.begin(), w1.end(), w0.begin(), w0.end()));
            CHECK(std::includes(r1.begin(), r1.end(), r0.begin(), r0.end()));
            nodes.insert(n0.begin(), n0.end());
            ways.insert(w0.begin(), w0.end());
            rels.insert(r0.begin(), r0.end());
        }
    }
    CHECK(nodes == ids_of(set.nodes));
    CHECK(ways == ids_of(set.ways));
    CHECK(rels == ids_of(set.relations));
    // A tile far away is empty.
    auto far = clip_osm(set, g2m_test::home_tile(50, 50));
    REQUIRE(far);
    CHECK(far->data.nodes.empty());
    CHECK(far->data.relations.empty());
}
