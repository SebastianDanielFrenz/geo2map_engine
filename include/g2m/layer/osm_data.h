// g2m/layer/osm_data.h - an in-memory set of OSM nodes, ways and relations
// in canonical form: the decoded form of a g2m.src.osm tile
// (docs/formats/src_osm.md) and the input of the per-tile clip
// (osm_clip.h).
//
// Canonical form (validate() checks it; the builder produces it):
//  - strings: sorted strictly ascending by bytes; every key, value and role
//    is an index into it;
//  - nodes, ways, relations: sorted strictly ascending by id;
//  - tags: one pooled array, node tags first (in node order), then way
//    tags, then relation tags; each element's tags form one contiguous run
//    with strictly ascending keys (so at most one value per key);
//  - way_refs / members: pooled in way / relation order, order within an
//    element preserved exactly as in OSM (it is the geometry / the
//    relation's meaning);
//  - every way has at least one ref, and every ref names a node present in
//    this set (a way is always complete: G3 builds the road graph from it);
//  - relation members may name elements that are NOT in the set (a
//    relation's membership list is always complete, its members are not);
//  - coordinates are OSM's own fixed point: 1e-7 degree int32, lat within
//    +-90 deg, lon within +-180 deg.
// OSM metadata (version, timestamp, changeset, user) is not kept.
#pragma once

#include "g2m/core/result.h"

#include <cstdint>
#include <optional>
#include <span>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace g2m::osm {

inline constexpr std::int32_t kMaxLatE7 = 900000000;
inline constexpr std::int32_t kMaxLonE7 = 1800000000;
inline constexpr std::size_t kMaxStringLength = 65535;

enum class MemberType : std::uint8_t { Node = 0, Way = 1, Relation = 2 };

struct Tag {
    std::uint32_t key = 0;   // string index
    std::uint32_t value = 0; // string index
    friend bool operator==(const Tag&, const Tag&) = default;
};

struct Node {
    std::int64_t id = 0;
    std::int32_t lat = 0; // 1e-7 degree
    std::int32_t lon = 0;
    std::uint32_t tags_begin = 0, tags_count = 0;
    friend bool operator==(const Node&, const Node&) = default;
};

struct Way {
    std::int64_t id = 0;
    std::uint32_t refs_begin = 0, refs_count = 0;
    std::uint32_t tags_begin = 0, tags_count = 0;
    friend bool operator==(const Way&, const Way&) = default;
};

struct Member {
    MemberType type = MemberType::Node;
    std::int64_t ref = 0;
    std::uint32_t role = 0; // string index ("" is a string too)
    friend bool operator==(const Member&, const Member&) = default;
};

struct Relation {
    std::int64_t id = 0;
    std::uint32_t members_begin = 0, members_count = 0;
    std::uint32_t tags_begin = 0, tags_count = 0;
    friend bool operator==(const Relation&, const Relation&) = default;
};

struct OsmData {
    std::vector<std::string> strings;
    std::vector<Node> nodes;
    std::vector<Way> ways;
    std::vector<Relation> relations;
    std::vector<Tag> tags;
    std::vector<std::int64_t> way_refs;
    std::vector<Member> members;

    std::string_view str(std::uint32_t i) const { return strings[i]; }
    std::span<const Tag> tags_of(const Node& n) const { return {tags.data() + n.tags_begin, n.tags_count}; }
    std::span<const Tag> tags_of(const Way& w) const { return {tags.data() + w.tags_begin, w.tags_count}; }
    std::span<const Tag> tags_of(const Relation& r) const { return {tags.data() + r.tags_begin, r.tags_count}; }
    std::span<const std::int64_t> refs_of(const Way& w) const { return {way_refs.data() + w.refs_begin, w.refs_count}; }
    std::span<const Member> members_of(const Relation& r) const {
        return {members.data() + r.members_begin, r.members_count};
    }
    // Binary search by id; nullptr if absent.
    const Node* find_node(std::int64_t id) const;
    const Way* find_way(std::int64_t id) const;
    const Relation* find_relation(std::int64_t id) const;
    // Value of tag `key` on an element, if present.
    std::optional<std::string_view> tag(std::span<const Tag> element_tags, std::string_view key) const;

    // Checks every canonical-form rule above.
    Result<void> validate() const;

    friend bool operator==(const OsmData&, const OsmData&) = default;
};

// Builds canonical OsmData from elements added in any order.
class OsmBuilder {
public:
    using TagList = std::vector<std::pair<std::string, std::string>>;
    struct MemberIn {
        MemberType type;
        std::int64_t ref;
        std::string role;
    };

    void add_node(std::int64_t id, std::int32_t lat_e7, std::int32_t lon_e7, TagList tags = {});
    void add_way(std::int64_t id, std::vector<std::int64_t> refs, TagList tags = {});
    void add_relation(std::int64_t id, std::vector<MemberIn> members, TagList tags = {});

    // Fails on duplicate ids, duplicate tag keys on one element, a way
    // without refs or with a ref to a node that was not added, out-of-range
    // coordinates, or an over-long string.
    Result<OsmData> build() const;

private:
    struct NodeIn {
        std::int64_t id;
        std::int32_t lat, lon;
        TagList tags;
    };
    struct WayIn {
        std::int64_t id;
        std::vector<std::int64_t> refs;
        TagList tags;
    };
    struct RelIn {
        std::int64_t id;
        std::vector<MemberIn> members;
        TagList tags;
    };
    std::vector<NodeIn> nodes_;
    std::vector<WayIn> ways_;
    std::vector<RelIn> rels_;
};

} // namespace g2m::osm
