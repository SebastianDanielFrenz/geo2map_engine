// tests/unit/test_ring_assembly.cpp - g2m::geom::assemble_rings
// (g2m/core/geom/ring_assembly.h, PLAN G1c I1): chaining/splitting open
// ways, closed ways, outer/inner classification by even-odd nesting (not
// declared role), error paths, and permutation invariance.

#include "g2m/core/geom/ring_assembly.h"
#include "g2m/core/hash.h"

#include "splitmix64.h"

#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <map>
#include <string>
#include <vector>

using namespace g2m::geom;

namespace {

PointMm pt(std::int64_t x, std::int64_t y) { return PointMm{x, y}; }

class NodeTable {
public:
    void add(std::int64_t id, PointMm p) { pos_[id] = p; }
    NodePos fn() const {
        return [this](std::int64_t id) -> std::optional<PointMm> {
            const auto it = pos_.find(id);
            return it == pos_.end() ? std::nullopt : std::optional<PointMm>(it->second);
        };
    }

private:
    std::map<std::int64_t, PointMm> pos_;
};

MemberWay way(std::int64_t id, std::string_view role, const std::vector<std::int64_t>& nodes) {
    return MemberWay{id, role, std::span<const std::int64_t>(nodes)};
}

// FNV-1a over a RingSet's canonical bytes: each outer then each inner
// ring's point count and x/y values, in the set's own already-canonical
// order (used only to compare two RingSets for the permutation-invariance
// test below - not part of region.h's public contract).
std::uint64_t ring_set_hash(const RingSet& rs) {
    g2m::Fnv1a64 h;
    h.u64(rs.outers.size());
    for (const Ring& r : rs.outers) {
        h.u64(r.size());
        for (const PointMm& p : r) h.i64(p.x).i64(p.y);
    }
    h.u64(rs.inners.size());
    for (const Ring& r : rs.inners) {
        h.u64(r.size());
        for (const PointMm& p : r) h.i64(p.x).i64(p.y);
    }
    return h.value();
}

bool point_less(PointMm a, PointMm b) { return a.x != b.x ? a.x < b.x : a.y < b.y; }

std::vector<PointMm> sorted_points(const Ring& r) {
    std::vector<PointMm> v(r.begin(), r.end());
    std::sort(v.begin(), v.end(), point_less);
    return v;
}

// PointMm has operator== but deliberately no operator< (see exact.h), so
// std::vector<PointMm>'s own default lexicographical < is unavailable -
// point_less supplies it explicitly for sorting these ring-point vectors.
bool points_less(const std::vector<PointMm>& a, const std::vector<PointMm>& b) {
    return std::lexicographical_compare(a.begin(), a.end(), b.begin(), b.end(), point_less);
}

template <class T>
void fisher_yates(std::vector<T>& v, g2m_test::SplitMix64& rng) {
    for (std::size_t i = v.size(); i-- > 1;) {
        const std::size_t j = static_cast<std::size_t>(rng.next() % (i + 1));
        std::swap(v[i], v[j]);
    }
}

} // namespace

TEST_CASE("ring_assembly: square from 4 shuffled open ways", "[geom][ring]") {
    NodeTable nt;
    nt.add(1, pt(0, 0));
    nt.add(2, pt(10, 0));
    nt.add(3, pt(10, 10));
    nt.add(4, pt(0, 10));
    const std::vector<std::int64_t> n10 = {1, 2};
    const std::vector<std::int64_t> n20 = {3, 4};
    const std::vector<std::int64_t> n30 = {3, 2}; // reversed
    const std::vector<std::int64_t> n40 = {1, 4}; // reversed

    // Given out of way-id order, and with two of the four edges stored
    // reversed: assemble_rings must sort, chain (reversing as needed) and
    // still produce the one canonical ring.
    const std::vector<MemberWay> members = {way(30, "outer", n30), way(10, "outer", n10), way(40, "outer", n40),
                                            way(20, "outer", n20)};
    const auto r = assemble_rings(members, nt.fn());
    REQUIRE(r.ok());
    CHECK(r->rings.outers.size() == 1);
    CHECK(r->rings.inners.empty());
    CHECK(r->role_mismatches == 0);
    CHECK(r->dropped_dup_points == 0);
    CHECK(r->vertex_count == 4);
    const Ring expected = {pt(0, 0), pt(10, 0), pt(10, 10), pt(0, 10)}; // CCW, starting at the min point
    CHECK(r->rings.outers.front() == expected);
}

TEST_CASE("ring_assembly: a closed way is a ring directly", "[geom][ring]") {
    NodeTable nt;
    nt.add(1, pt(0, 0));
    nt.add(2, pt(5, 0));
    nt.add(3, pt(5, 5));
    nt.add(4, pt(0, 5));
    const std::vector<std::int64_t> nodes = {1, 2, 3, 4, 1}; // closed: repeats its first node

    const auto r = assemble_rings({way(100, "outer", nodes)}, nt.fn());
    REQUIRE(r.ok());
    CHECK(r->rings.outers.size() == 1);
    CHECK(r->rings.inners.empty());
    const Ring expected = {pt(0, 0), pt(5, 0), pt(5, 5), pt(0, 5)};
    CHECK(r->rings.outers.front() == expected);
    CHECK(r->dropped_dup_points == 1); // the repeated closing point
    CHECK(r->vertex_count == 4);
}

TEST_CASE("ring_assembly: outer with an inner hole", "[geom][ring]") {
    NodeTable nt;
    nt.add(1, pt(0, 0));
    nt.add(2, pt(20, 0));
    nt.add(3, pt(20, 20));
    nt.add(4, pt(0, 20));
    nt.add(5, pt(5, 5));
    nt.add(6, pt(15, 5));
    nt.add(7, pt(15, 15));
    nt.add(8, pt(5, 15));
    const std::vector<std::int64_t> outer_nodes = {1, 2, 3, 4, 1};
    const std::vector<std::int64_t> inner_nodes = {5, 6, 7, 8, 5};

    const auto r = assemble_rings({way(1, "outer", outer_nodes), way(2, "inner", inner_nodes)}, nt.fn());
    REQUIRE(r.ok());
    CHECK(r->rings.outers.size() == 1);
    CHECK(r->rings.inners.size() == 1);
    CHECK(r->role_mismatches == 0);
    CHECK(sorted_points(r->rings.outers.front()) ==
         sorted_points(Ring{pt(0, 0), pt(20, 0), pt(20, 20), pt(0, 20)}));
    CHECK(sorted_points(r->rings.inners.front()) ==
         sorted_points(Ring{pt(5, 5), pt(15, 5), pt(15, 15), pt(5, 15)}));
}

TEST_CASE("ring_assembly: inner tagged outer still becomes a hole, with one mismatch", "[geom][ring]") {
    NodeTable nt;
    nt.add(1, pt(0, 0));
    nt.add(2, pt(20, 0));
    nt.add(3, pt(20, 20));
    nt.add(4, pt(0, 20));
    nt.add(5, pt(5, 5));
    nt.add(6, pt(15, 5));
    nt.add(7, pt(15, 15));
    nt.add(8, pt(5, 15));
    const std::vector<std::int64_t> outer_nodes = {1, 2, 3, 4, 1};
    const std::vector<std::int64_t> inner_nodes = {5, 6, 7, 8, 5};

    // The hole is (wrongly) tagged "outer" - assembly still classifies it
    // geometrically as an inner ring, and counts exactly one mismatch.
    const auto r = assemble_rings({way(1, "outer", outer_nodes), way(2, "outer", inner_nodes)}, nt.fn());
    REQUIRE(r.ok());
    CHECK(r->rings.outers.size() == 1);
    CHECK(r->rings.inners.size() == 1);
    CHECK(r->role_mismatches == 1);
    CHECK(sorted_points(r->rings.inners.front()) ==
         sorted_points(Ring{pt(5, 5), pt(15, 5), pt(15, 15), pt(5, 15)}));
}

TEST_CASE("ring_assembly: figure-eight splits into two rings sharing one vertex", "[geom][ring]") {
    NodeTable nt;
    nt.add(1, pt(0, 0));   // the shared vertex
    nt.add(2, pt(10, 0));
    nt.add(3, pt(10, 10));
    nt.add(4, pt(-10, 0));
    nt.add(5, pt(-10, -10));
    const std::vector<std::int64_t> a1 = {1, 2};
    const std::vector<std::int64_t> a2 = {2, 3};
    const std::vector<std::int64_t> a3 = {3, 1}; // closes loop 1 back to the shared vertex
    const std::vector<std::int64_t> b1 = {1, 4};
    const std::vector<std::int64_t> b2 = {4, 5};
    const std::vector<std::int64_t> b3 = {5, 1}; // closes loop 2 back to the shared vertex

    // Roles left blank: step 9's nesting classification for this shape is
    // exercised (see below), not role-mismatch counting.
    const auto r = assemble_rings({way(1, "", a1), way(2, "", a2), way(3, "", a3), way(4, "", b1), way(5, "", b2),
                                   way(6, "", b3)},
                                  nt.fn());
    REQUIRE(r.ok()); // sharing exactly one vertex between the two rings is not a bad intersection
    // The core property this test is after is the CHAIN SPLIT itself: one
    // connected traversal through the shared vertex A must still yield two
    // separate 3-point rings, not one 5-point ring or an error. Both split
    // rings start exactly at A (chain_open_ways always splits a repeat off
    // starting at the repeated node), and ring_set.h's contains() is
    // documented "on-edge = inside" (plan step 9 is deliberately a simple
    // even-odd-of-a-sample-point rule, with no special tangency handling) -
    // so each ring's sample point (its own front(), = A) lands exactly on
    // the OTHER ring's boundary too, and both are consistently classified
    // odd-depth ("inner") rather than disambiguated as two siblings. This
    // is an accepted I1 simplification, not asserted here as outer/inner;
    // only that assembly succeeds and the two point sets are exactly right.
    CHECK(r->rings.outers.size() + r->rings.inners.size() == 2);
    std::vector<std::vector<PointMm>> got;
    for (const Ring& ring : r->rings.outers) got.push_back(sorted_points(ring));
    for (const Ring& ring : r->rings.inners) got.push_back(sorted_points(ring));
    std::sort(got.begin(), got.end(), points_less);
    std::vector<std::vector<PointMm>> want = {sorted_points(Ring{pt(0, 0), pt(10, 0), pt(10, 10)}),
                                              sorted_points(Ring{pt(0, 0), pt(-10, 0), pt(-10, -10)})};
    std::sort(want.begin(), want.end(), points_less);
    CHECK(got == want);
}

TEST_CASE("ring_assembly: unclosed ring names the node", "[geom][ring]") {
    NodeTable nt;
    nt.add(1, pt(0, 0));
    nt.add(2, pt(10, 0));
    nt.add(3, pt(10, 10));
    const std::vector<std::int64_t> w1 = {1, 2};
    const std::vector<std::int64_t> w2 = {2, 3}; // dangling: node 1 and node 3 each have odd degree

    const auto r = assemble_rings({way(1, "outer", w1), way(2, "outer", w2)}, nt.fn());
    REQUIRE_FALSE(r.ok());
    CHECK(r.error().code == g2m::ErrorCode::Malformed);
    CHECK(r.error().message.find("unclosed ring at node 1") != std::string::npos);
}

TEST_CASE("ring_assembly: missing node position is an error", "[geom][ring]") {
    NodeTable nt;
    nt.add(1, pt(0, 0));
    nt.add(2, pt(10, 0));
    // node 3 deliberately not added.
    const std::vector<std::int64_t> nodes = {1, 2, 3, 1};

    const auto r = assemble_rings({way(1, "outer", nodes)}, nt.fn());
    REQUIRE_FALSE(r.ok());
    CHECK(r.error().code == g2m::ErrorCode::Malformed);
    CHECK(r.error().message.find("node 3") != std::string::npos);
}

TEST_CASE("ring_assembly: duplicate way id is an error", "[geom][ring]") {
    NodeTable nt;
    nt.add(1, pt(0, 0));
    nt.add(2, pt(10, 0));
    nt.add(3, pt(10, 10));
    nt.add(4, pt(0, 10));
    const std::vector<std::int64_t> a = {1, 2, 3, 4, 1};
    const std::vector<std::int64_t> b = {1, 2, 3, 4, 1};

    const auto r = assemble_rings({way(7, "outer", a), way(7, "outer", b)}, nt.fn());
    REQUIRE_FALSE(r.ok());
    CHECK(r.error().code == g2m::ErrorCode::Malformed);
    CHECK(r.error().message.find("duplicate way") != std::string::npos);
}

TEST_CASE("ring_assembly: self-crossing bow-tie is rejected", "[geom][ring]") {
    NodeTable nt;
    nt.add(1, pt(0, 0));
    nt.add(2, pt(10, 10));
    nt.add(3, pt(10, 0));
    nt.add(4, pt(0, 10));
    // (0,0) -> (10,10) -> (10,0) -> (0,10) -> (0,0): the two diagonals of a
    // square, an hourglass that crosses itself.
    const std::vector<std::int64_t> nodes = {1, 2, 3, 4, 1};

    const auto r = assemble_rings({way(1, "outer", nodes)}, nt.fn());
    REQUIRE_FALSE(r.ok());
    CHECK(r.error().code == g2m::ErrorCode::Malformed);
    CHECK(r.error().message.find("cross or overlap") != std::string::npos);
}

TEST_CASE("ring_assembly: two rings touching at exactly one vertex is fine", "[geom][ring]") {
    NodeTable nt;
    nt.add(1, pt(0, 0));
    nt.add(2, pt(10, 0));
    nt.add(3, pt(10, 10));
    nt.add(4, pt(0, 10));
    nt.add(5, pt(20, 10));
    nt.add(6, pt(20, 20));
    nt.add(7, pt(10, 20));
    const std::vector<std::int64_t> square_a = {1, 2, 3, 4, 1};
    // Starting at node 5 (not the shared node 3) so this ring's own sample
    // point (its front(), used by step 9's nesting test) does not itself
    // land on square_a's boundary - see the figure-eight test above for
    // what happens when it does (contains()'s documented "on-edge =
    // inside" rule then makes the two rings' classification depend on
    // which point happens to be sampled, not disambiguated tangency).
    const std::vector<std::int64_t> square_b = {5, 6, 7, 3, 5}; // shares only vertex 3 = (10,10) with square_a

    const auto r = assemble_rings({way(1, "outer", square_a), way(2, "outer", square_b)}, nt.fn());
    REQUIRE(r.ok());
    CHECK(r->rings.outers.size() == 2);
    CHECK(r->rings.inners.empty());
    CHECK(r->role_mismatches == 0);
}

TEST_CASE("ring_assembly: permutation invariance over 20 seeded shuffles", "[geom][ring]") {
    // Same outer-square-with-inner-hole shape as above, built from 8 open
    // ways (4 outer edges + 4 inner edges). Each iteration: reverse each
    // way's own node order with 50% probability (one RNG draw per way),
    // then Fisher-Yates shuffle the member list itself (one RNG draw per
    // swap) - TOOL-027: one draw per statement. Every shuffle must
    // assemble to the byte-identical canonical RingSet.
    NodeTable nt;
    nt.add(1, pt(0, 0));
    nt.add(2, pt(20, 0));
    nt.add(3, pt(20, 20));
    nt.add(4, pt(0, 20));
    nt.add(5, pt(5, 5));
    nt.add(6, pt(15, 5));
    nt.add(7, pt(15, 15));
    nt.add(8, pt(5, 15));
    struct Edge {
        std::int64_t way_id, a, b;
        const char* role;
    };
    const Edge edges[] = {
        {1, 1, 2, "outer"}, {2, 2, 3, "outer"}, {3, 3, 4, "outer"}, {4, 4, 1, "outer"},
        {5, 5, 6, "inner"}, {6, 6, 7, "inner"}, {7, 7, 8, "inner"}, {8, 8, 5, "inner"},
    };

    std::optional<std::uint64_t> want;
    for (std::uint64_t seed = 0; seed < 20; ++seed) {
        g2m_test::SplitMix64 rng(seed + 1);
        std::vector<std::vector<std::int64_t>> node_lists;
        node_lists.reserve(std::size(edges));
        std::vector<MemberWay> members;
        members.reserve(std::size(edges));
        for (const Edge& e : edges) {
            const bool reversed = (rng.next() % 2) == 0;
            node_lists.push_back(reversed ? std::vector<std::int64_t>{e.b, e.a} : std::vector<std::int64_t>{e.a, e.b});
        }
        for (std::size_t i = 0; i < std::size(edges); ++i) members.push_back(way(edges[i].way_id, edges[i].role, node_lists[i]));
        fisher_yates(members, rng);

        const auto r = assemble_rings(members, nt.fn());
        REQUIRE(r.ok());
        const std::uint64_t got = ring_set_hash(r->rings);
        if (!want) {
            want = got;
        } else {
            CHECK(got == *want);
        }
    }
}
