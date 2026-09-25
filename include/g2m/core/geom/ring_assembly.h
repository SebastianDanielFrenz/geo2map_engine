// g2m/core/geom/ring_assembly.h - build simple polygon rings from a set of
// OSM-relation-style way members (G1c plan I1): chains open ways at shared
// endpoints, splits at repeated node ids, rejects crossing/overlapping
// geometry, classifies outer/inner by even-odd nesting, and normalises
// each ring's orientation and starting point.
#pragma once

#include "g2m/core/geom/ring_set.h"
#include "g2m/core/result.h"

#include <cstdint>
#include <functional>
#include <optional>
#include <span>
#include <string_view>
#include <vector>

namespace g2m::geom {

// One relation member way: its OSM way id, its role string ("outer",
// "inner", or anything else - see assemble_rings() below), and its node id
// list in the way's own stored direction. A closed way repeats its first
// node id as its last.
struct MemberWay {
    std::int64_t way_id = 0;
    std::string_view role;
    std::span<const std::int64_t> nodes;
};

// Resolves an OSM node id to its position; empty if the node is not known
// to the caller (an error - see assemble_rings()).
using NodePos = std::function<std::optional<PointMm>(std::int64_t)>;

struct AssembledArea {
    RingSet rings;
    // Ways whose declared role ("outer"/"inner") disagrees with the ring's
    // actual even-odd classification. Purely informational: assembly
    // always uses the geometric classification, never the declared role -
    // a way with any other role text (including "") never counts here.
    std::uint32_t role_mismatches = 0;
    // Consecutive equal projected points collapsed into one point,
    // including a ring's own closing point when it repeats the start.
    std::uint32_t dropped_dup_points = 0;
    // Total vertices over every output ring (outers + inners), after
    // dedup.
    std::uint64_t vertex_count = 0;
};

// Assembles `members` into simple, non-crossing rings. `members` need not
// be sorted or unique (a duplicate way id is an error). Algorithm (see
// docs/formats/regions.md, "ring assembly rules"):
//  1. sort by way_id; a duplicate way id is an error; a role other than
//     "outer"/"inner"/"" is treated as "" (ignored, not a mismatch);
//  2. resolve every node id via `pos`; an unresolved id is an error;
//  3. a closed way (first node id == last) becomes a ring directly;
//  4. every open-way endpoint node id must have even degree among open
//     ways, else "unclosed ring at node N (way W)";
//  5. chain open ways end-to-end, starting from the lowest unused way id;
//     at a junction (a node id shared by several unused ways), take the
///    lowest id;
//  6. split a chain wherever a node id repeats (closes a sub-ring, and -
//     if the repeat is not the chain's own start - continues the rest of
//     the chain from that node);
//  7. drop consecutive equal projected points (including a repeated
//     closing point); a ring with fewer than 3 distinct points is an
//     error;
//  8. any proper crossing or collinear overlap between two (non-adjacent)
//     segments, in the same ring or different rings, is an error; a shared
//     vertex is fine;
//  9. classify each ring by even-odd nesting depth (even = outer, odd =
//     inner) and count role mismatches against it;
//  10. normalise: outer rings CCW, inner rings CW, each ring starting at
//      its lexicographically smallest point.
Result<AssembledArea> assemble_rings(std::vector<MemberWay> members, const NodePos& pos);

} // namespace g2m::geom
