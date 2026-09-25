// src/core/geom/ring_assembly.cpp - see g2m/core/geom/ring_assembly.h.

#include "g2m/core/geom/ring_assembly.h"

#include <algorithm>
#include <map>
#include <set>
#include <string>

namespace g2m::geom {
namespace {

std::string_view normalize_role(std::string_view role) {
    if (role == "outer" || role == "inner") return role;
    return {};
}

struct WayInfo {
    std::int64_t way_id = 0;
    std::string_view role; // normalized
    std::vector<std::int64_t> node_ids;
    std::vector<PointMm> points; // same length as node_ids
    bool used = false;

    std::int64_t first_id() const { return node_ids.front(); }
    std::int64_t last_id() const { return node_ids.back(); }
    bool closed() const { return node_ids.size() >= 1 && first_id() == last_id(); }
};

// A ring before dedup/classification/normalisation, plus which ways
// contributed to it (for role-mismatch counting).
struct RawRing {
    std::vector<PointMm> points; // implicitly closed, may contain consecutive duplicates
    std::vector<std::int64_t> way_ids;
};

Result<void> resolve_positions(std::vector<WayInfo>& ways, const NodePos& pos) {
    for (WayInfo& w : ways) {
        w.points.reserve(w.node_ids.size());
        for (std::int64_t id : w.node_ids) {
            const std::optional<PointMm> p = pos(id);
            if (!p) {
                return make_error(ErrorCode::Malformed, "missing node position for node " + std::to_string(id) +
                                                              " (way " + std::to_string(w.way_id) + ")");
            }
            w.points.push_back(*p);
        }
    }
    return {};
}

Result<void> check_even_degree(const std::vector<WayInfo>& open_ways) {
    std::map<std::int64_t, int> degree;
    std::map<std::int64_t, std::int64_t> first_way_at; // for the error message
    for (const WayInfo& w : open_ways) {
        for (std::int64_t id : {w.first_id(), w.last_id()}) {
            degree[id]++;
            first_way_at.try_emplace(id, w.way_id);
        }
    }
    for (const auto& [node_id, count] : degree) {
        if (count % 2 != 0) {
            return make_error(ErrorCode::Malformed, "unclosed ring at node " + std::to_string(node_id) + " (way " +
                                                          std::to_string(first_way_at[node_id]) + ")");
        }
    }
    return {};
}

// Chains every open way into raw rings (PLAN steps 5-6): starting from the
// lowest unused way id, walk from endpoint to endpoint (lowest id at a
// junction), splitting off a completed ring wherever a node id repeats
// within the current chain.
Result<void> chain_open_ways(std::vector<WayInfo>& open_ways, std::vector<RawRing>& out) {
    // Ordered by way_id ascending (the caller already sorted `open_ways`
    // that way, since it is a filtered view of the globally-sorted list).
    for (std::size_t start_idx = 0; start_idx < open_ways.size(); ++start_idx) {
        if (open_ways[start_idx].used) continue;
        WayInfo& start = open_ways[start_idx];
        start.used = true;

        std::vector<PointMm> chain_points;
        std::vector<std::int64_t> chain_ids;
        std::vector<std::int64_t> chain_way_ids;
        std::map<std::int64_t, std::size_t> id_to_index;

        const auto append_point = [&](std::int64_t id, PointMm p, std::int64_t way_id) {
            const auto it = id_to_index.find(id);
            if (it == id_to_index.end()) {
                id_to_index.emplace(id, chain_points.size());
                chain_points.push_back(p);
                chain_ids.push_back(id);
                chain_way_ids.push_back(way_id);
                return;
            }
            // Repeat: split off [i .. end] as a completed ring, keep [0 .. i].
            const std::size_t i = it->second;
            RawRing ring;
            ring.points.assign(chain_points.begin() + static_cast<std::ptrdiff_t>(i), chain_points.end());
            std::set<std::int64_t> distinct_ways(chain_way_ids.begin() + static_cast<std::ptrdiff_t>(i),
                                                  chain_way_ids.end());
            distinct_ways.insert(way_id);
            ring.way_ids.assign(distinct_ways.begin(), distinct_ways.end());
            out.push_back(std::move(ring));
            for (std::size_t k = i + 1; k < chain_ids.size(); ++k) id_to_index.erase(chain_ids[k]);
            chain_points.resize(i + 1);
            chain_ids.resize(i + 1);
            chain_way_ids.resize(i + 1);
        };

        chain_points.push_back(start.points.front());
        chain_ids.push_back(start.first_id());
        chain_way_ids.push_back(start.way_id);
        id_to_index.emplace(start.first_id(), 0);
        for (std::size_t k = 1; k < start.node_ids.size(); ++k) {
            append_point(start.node_ids[k], start.points[k], start.way_id);
        }

        while (chain_points.size() > 1) {
            const std::int64_t tail = chain_ids.back();
            std::size_t best = open_ways.size();
            for (std::size_t k = 0; k < open_ways.size(); ++k) {
                if (open_ways[k].used) continue;
                if (open_ways[k].first_id() != tail && open_ways[k].last_id() != tail) continue;
                if (best == open_ways.size() || open_ways[k].way_id < open_ways[best].way_id) best = k;
            }
            if (best == open_ways.size()) {
                return make_error(ErrorCode::Malformed,
                                   "unclosed ring at node " + std::to_string(tail) + " (way " +
                                       std::to_string(chain_way_ids.back()) + ")");
            }
            WayInfo& next = open_ways[best];
            next.used = true;
            if (next.first_id() == tail) {
                for (std::size_t k = 1; k < next.node_ids.size(); ++k) {
                    append_point(next.node_ids[k], next.points[k], next.way_id);
                }
            } else {
                const std::size_t n = next.node_ids.size();
                for (std::size_t k = 1; k < n; ++k) {
                    const std::size_t j = n - 1 - k;
                    append_point(next.node_ids[j], next.points[j], next.way_id);
                }
            }
        }
        // chain_points.size() == 1: fully closed back to the start (every
        // split above already emitted the rest as completed raw rings).
    }
    return {};
}

// Drops consecutive equal points (including the implicit close, last vs.
// first). Returns false (via `err`) if fewer than 3 distinct points remain.
Result<Ring> dedup_ring(const RawRing& raw, std::uint32_t& dropped_dup_points) {
    Ring out;
    for (const PointMm& p : raw.points) {
        if (!out.empty() && out.back() == p) {
            ++dropped_dup_points;
            continue;
        }
        out.push_back(p);
    }
    if (out.size() >= 2 && out.front() == out.back()) {
        out.pop_back();
        ++dropped_dup_points;
    }
    if (out.size() < 3) {
        const std::int64_t way_id = raw.way_ids.empty() ? 0 : raw.way_ids.front();
        return make_error(ErrorCode::Malformed,
                           "ring has fewer than 3 distinct points (way " + std::to_string(way_id) + ")");
    }
    return out;
}

bool shares_endpoint(PointMm a, PointMm b, PointMm c, PointMm d) { return a == c || a == d || b == c || b == d; }

enum class Touch { None, SharedVertex, Bad };

Touch classify_touch(PointMm a, PointMm b, PointMm c, PointMm d) {
    bool proper = false;
    if (!segments_intersect(a, b, c, d, &proper)) return Touch::None;
    if (proper) return Touch::Bad;
    if (!shares_endpoint(a, b, c, d)) return Touch::Bad; // a T-touch, or interior-interior overlap
    if (orient(a, b, c) == 0 && orient(a, b, d) == 0) {
        // Collinear: parametrize along ab (x, or y if ab is vertical) and
        // check the overlap is a single point, not a run.
        const auto param = [&](PointMm p) { return (a.x != b.x) ? p.x : p.y; };
        const std::int64_t lo = std::min(param(a), param(b)), hi = std::max(param(a), param(b));
        const std::int64_t plo = std::min(param(c), param(d)), phi = std::max(param(c), param(d));
        const std::int64_t overlap_lo = std::max(lo, plo), overlap_hi = std::min(hi, phi);
        if (overlap_lo < overlap_hi) return Touch::Bad;
    }
    return Touch::SharedVertex;
}

struct RingSeg {
    PointMm a, b;
    std::size_t ring_index;
    std::size_t edge_index;
};

// Step 8: any proper crossing or collinear overlap between two segments
// that are not consecutive edges of the same ring is an error. A
// bounding-box pre-filter (rather than a uniform grid bucket - see the
// G1c I1 report) keeps this fast in practice: ring segments are locally
// short, so only genuinely nearby pairs reach the exact test.
Result<void> check_no_bad_intersections(const std::vector<Ring>& rings) {
    std::vector<RingSeg> segs;
    for (std::size_t ri = 0; ri < rings.size(); ++ri) {
        const std::size_t n = rings[ri].size();
        for (std::size_t i = 0; i < n; ++i) segs.push_back({rings[ri][i], rings[ri][(i + 1) % n], ri, i});
    }
    for (std::size_t i = 0; i < segs.size(); ++i) {
        const RingSeg& s1 = segs[i];
        const std::int64_t s1w = std::min(s1.a.x, s1.b.x), s1e = std::max(s1.a.x, s1.b.x);
        const std::int64_t s1s = std::min(s1.a.y, s1.b.y), s1n = std::max(s1.a.y, s1.b.y);
        for (std::size_t j = i + 1; j < segs.size(); ++j) {
            const RingSeg& s2 = segs[j];
            if (s1.ring_index == s2.ring_index) {
                const std::size_t n = rings[s1.ring_index].size();
                const bool adjacent =
                    s2.edge_index == (s1.edge_index + 1) % n || s1.edge_index == (s2.edge_index + 1) % n;
                if (adjacent) continue; // consecutive edges legitimately share one endpoint
            }
            if (s1e < std::min(s2.a.x, s2.b.x) || s1w > std::max(s2.a.x, s2.b.x) ||
                s1n < std::min(s2.a.y, s2.b.y) || s1s > std::max(s2.a.y, s2.b.y)) {
                continue; // bounding boxes do not overlap: cannot touch
            }
            if (classify_touch(s1.a, s1.b, s2.a, s2.b) == Touch::Bad) {
                return make_error(ErrorCode::Malformed, "rings cross or overlap (ring " +
                                                              std::to_string(s1.ring_index) + " edge " +
                                                              std::to_string(s1.edge_index) + ", ring " +
                                                              std::to_string(s2.ring_index) + " edge " +
                                                              std::to_string(s2.edge_index) + ")");
            }
        }
    }
    return {};
}

bool ring_contains_point_only(const Ring& r, PointMm p) {
    RingSet tmp;
    tmp.outers.push_back(r);
    return contains(tmp, p);
}

void normalize_ring(Ring& ring, bool is_outer) {
    const std::size_t n = ring.size();
    std::size_t m = 0;
    for (std::size_t i = 1; i < n; ++i) {
        if (ring[i].x < ring[m].x || (ring[i].x == ring[m].x && ring[i].y < ring[m].y)) m = i;
    }
    const PointMm prev = ring[(m + n - 1) % n], cur = ring[m], next = ring[(m + 1) % n];
    const bool is_ccw = orient(prev, cur, next) > 0;
    if (is_ccw != is_outer) {
        std::reverse(ring.begin(), ring.end());
        m = n - 1 - m;
    }
    std::rotate(ring.begin(), ring.begin() + static_cast<std::ptrdiff_t>(m), ring.end());
}

} // namespace

Result<AssembledArea> assemble_rings(std::vector<MemberWay> members, const NodePos& pos) {
    // 1. sort by way_id, duplicate way = error, normalise roles.
    std::sort(members.begin(), members.end(),
              [](const MemberWay& a, const MemberWay& b) { return a.way_id < b.way_id; });
    std::vector<WayInfo> ways;
    ways.reserve(members.size());
    for (std::size_t i = 0; i < members.size(); ++i) {
        if (i > 0 && members[i].way_id == members[i - 1].way_id) {
            return make_error(ErrorCode::Malformed, "duplicate way " + std::to_string(members[i].way_id));
        }
        if (members[i].nodes.empty()) {
            return make_error(ErrorCode::Malformed, "way " + std::to_string(members[i].way_id) + " has no nodes");
        }
        WayInfo w;
        w.way_id = members[i].way_id;
        w.role = normalize_role(members[i].role);
        w.node_ids.assign(members[i].nodes.begin(), members[i].nodes.end());
        ways.push_back(std::move(w));
    }

    // 2. resolve node positions.
    if (auto r = resolve_positions(ways, pos); !r) return r.error();

    // way_id -> normalized role, captured before step 3 moves any WayInfo
    // out of `ways` (an open way's own `role` would otherwise read back
    // moved-from/empty in step 9 below).
    std::map<std::int64_t, std::string_view> role_by_way;
    for (const WayInfo& w : ways) role_by_way.emplace(w.way_id, w.role);

    // 3. closed ways -> rings directly; collect open ways separately.
    std::vector<RawRing> raw_rings;
    std::vector<WayInfo> open_ways;
    for (WayInfo& w : ways) {
        if (w.closed()) {
            RawRing ring;
            ring.points = w.points;
            ring.way_ids = {w.way_id};
            raw_rings.push_back(std::move(ring));
        } else {
            open_ways.push_back(std::move(w));
        }
    }

    // 4. even-degree check on open-way endpoints.
    if (auto r = check_even_degree(open_ways); !r) return r.error();

    // 5-6. chain + split open ways.
    if (auto r = chain_open_ways(open_ways, raw_rings); !r) return r.error();

    // 7. dedup consecutive equal points; < 3 distinct points = error.
    AssembledArea area;
    std::vector<Ring> rings;
    std::vector<std::vector<std::int64_t>> ring_way_ids;
    rings.reserve(raw_rings.size());
    ring_way_ids.reserve(raw_rings.size());
    for (const RawRing& raw : raw_rings) {
        Result<Ring> r = dedup_ring(raw, area.dropped_dup_points);
        if (!r) return r.error();
        rings.push_back(std::move(r).value());
        ring_way_ids.push_back(raw.way_ids);
    }

    // 8. no proper crossing or collinear overlap anywhere.
    if (auto r = check_no_bad_intersections(rings); !r) return r.error();

    // 9. classify by even-odd nesting depth; count role mismatches.
    std::vector<bool> is_outer(rings.size());
    for (std::size_t i = 0; i < rings.size(); ++i) {
        int depth = 0;
        const PointMm sample = rings[i].front();
        for (std::size_t j = 0; j < rings.size(); ++j) {
            if (j == i) continue;
            if (ring_contains_point_only(rings[j], sample)) ++depth;
        }
        is_outer[i] = (depth % 2) == 0;
        const std::string_view want = is_outer[i] ? "outer" : "inner";
        for (std::int64_t way_id : ring_way_ids[i]) {
            const auto it = role_by_way.find(way_id);
            const std::string_view role = (it != role_by_way.end()) ? it->second : std::string_view{};
            if (!role.empty() && role != want) ++area.role_mismatches;
        }
    }

    // 10. normalise orientation + starting point; partition into outers/inners.
    for (std::size_t i = 0; i < rings.size(); ++i) normalize_ring(rings[i], is_outer[i]);
    for (std::size_t i = 0; i < rings.size(); ++i) {
        if (is_outer[i]) {
            area.rings.outers.push_back(std::move(rings[i]));
        } else {
            area.rings.inners.push_back(std::move(rings[i]));
        }
    }
    area.rings.bbox = bounding_box(area.rings);
    for (const Ring& r : area.rings.outers) area.vertex_count += r.size();
    for (const Ring& r : area.rings.inners) area.vertex_count += r.size();

    return area;
}

} // namespace g2m::geom
