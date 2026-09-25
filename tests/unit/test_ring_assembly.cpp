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
    // Item 0 fix: both split rings start exactly at the shared vertex A
    // (chain_open_ways always splits a repeat off starting at the repeated
    // node), so this is exactly the tangency case step 9's nesting test
    // must get right regardless of which point is sampled - two separate
    // 3-point triangles meeting at one corner, NEITHER nested inside the
    // other. Before the fix, both rings' sample point (front() == A)
    // landed exactly on the other ring's boundary, and contains()'s
    // documented "on-edge = inside" rule made both read as odd-depth
    // ("inner"), not disambiguated. The fix (ring_i_is_inside_ring_j: use
    // the first vertex NOT on the other ring's boundary) must now classify
    // both as outer, since neither triangle actually contains the other.
    CHECK(r->rings.outers.size() == 2);
    CHECK(r->rings.inners.empty());
    std::vector<std::vector<PointMm>> got;
    for (const Ring& ring : r->rings.outers) got.push_back(sorted_points(ring));
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
    const std::vector<std::int64_t> square_b = {5, 6, 7, 3, 5}; // shares only vertex 3 = (10,10) with square_a

    const auto r = assemble_rings({way(1, "outer", square_a), way(2, "outer", square_b)}, nt.fn());
    REQUIRE(r.ok());
    CHECK(r->rings.outers.size() == 2);
    CHECK(r->rings.inners.empty());
    CHECK(r->role_mismatches == 0);
}

TEST_CASE("ring_assembly: tangent vertex is the FIRST ring's front(), not the second's", "[geom][ring]") {
    // Item 0 fix regression: two squares sharing exactly one corner
    // (10,10), first square's closed way listed starting AT the shared
    // vertex (so rings[0].front() == the tangent point), second square's
    // way starting elsewhere. Before the fix, sampling rings[0].front()
    // against ring 1 would find it exactly on ring 1's boundary and
    // (on-edge = inside) misclassify depth. Neither square actually
    // contains the other - both must come out outer.
    NodeTable nt;
    nt.add(1, pt(0, 0));
    nt.add(2, pt(10, 0));
    nt.add(3, pt(10, 10)); // the shared/tangent vertex
    nt.add(4, pt(0, 10));
    nt.add(5, pt(20, 10));
    nt.add(6, pt(20, 20));
    nt.add(7, pt(10, 20));
    const std::vector<std::int64_t> square_a = {3, 4, 1, 2, 3}; // starts AT the shared vertex
    const std::vector<std::int64_t> square_b = {5, 6, 7, 3, 5}; // starts elsewhere

    const auto r = assemble_rings({way(1, "outer", square_a), way(2, "outer", square_b)}, nt.fn());
    REQUIRE(r.ok());
    CHECK(r->rings.outers.size() == 2);
    CHECK(r->rings.inners.empty());
}

TEST_CASE("ring_assembly: tangent vertex is the SECOND ring's front(), not the first's", "[geom][ring]") {
    // Same shape, roles of the two ways swapped: the shared vertex is now
    // the SECOND way's own first listed node - the mirror-image ordering
    // of the case above ("both orders").
    NodeTable nt;
    nt.add(1, pt(0, 0));
    nt.add(2, pt(10, 0));
    nt.add(3, pt(10, 10)); // the shared/tangent vertex
    nt.add(4, pt(0, 10));
    nt.add(5, pt(20, 10));
    nt.add(6, pt(20, 20));
    nt.add(7, pt(10, 20));
    const std::vector<std::int64_t> square_a = {1, 2, 3, 4, 1}; // starts elsewhere
    const std::vector<std::int64_t> square_b = {3, 5, 6, 7, 3}; // starts AT the shared vertex

    const auto r = assemble_rings({way(1, "outer", square_a), way(2, "outer", square_b)}, nt.fn());
    REQUIRE(r.ok());
    CHECK(r->rings.outers.size() == 2);
    CHECK(r->rings.inners.empty());
}

TEST_CASE("ring_assembly: inscribed triangle touching only at 3 shared vertices", "[geom][ring]") {
    // Item 0's "every vertex of i lies on j's boundary" fallback: a convex
    // hexagon (outer) and a triangle formed by every other one of its own
    // vertices (V0, V2, V4) - strictly inside the hexagon, touching its
    // boundary only at those 3 shared vertices (a diagonal chord of a
    // convex polygon never touches the boundary except at its own
    // endpoints). EVERY vertex of the triangle ring is therefore on the
    // hexagon ring's boundary, so ring_i_is_inside_ring_j must fall back to
    // the doubled-midpoint test to classify the triangle as an inner ring
    // (a hole) - and, symmetrically, the hexagon's own vertex V1 is NOT on
    // the triangle's boundary, so its classification (outer) uses the
    // ordinary per-vertex path.
    NodeTable nt;
    nt.add(10, pt(0, 0));  // V0
    nt.add(11, pt(4, 0));  // V1
    nt.add(12, pt(6, 3));  // V2
    nt.add(13, pt(4, 6));  // V3
    nt.add(14, pt(0, 6));  // V4
    nt.add(15, pt(-2, 3)); // V5
    const std::vector<std::int64_t> hexagon = {10, 11, 12, 13, 14, 15, 10};
    const std::vector<std::int64_t> triangle = {10, 12, 14, 10}; // V0, V2, V4 - alternating hexagon vertices

    const auto r = assemble_rings({way(1, "outer", hexagon), way(2, "inner", triangle)}, nt.fn());
    REQUIRE(r.ok());
    CHECK(r->rings.outers.size() == 1);
    CHECK(r->rings.inners.size() == 1);
    CHECK(r->role_mismatches == 0);
    CHECK(sorted_points(r->rings.inners.front()) ==
         sorted_points(Ring{pt(0, 0), pt(6, 3), pt(0, 6)}));
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
