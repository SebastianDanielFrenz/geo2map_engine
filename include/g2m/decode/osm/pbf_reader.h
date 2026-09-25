// g2m/decode/osm/pbf_reader.h - OSM PBF streaming reader (PLAN.md 13,
// milestone G1b). Wraps libosmium + protozero; NO osmium (or protozero)
// type appears anywhere in this header or in any other g2m public header -
// callers see only the plain structs below.
//
// Determinism: nodes/ways/relations are delivered to the handler in FILE
// ORDER, always - libosmium's osmium::io::Reader may decompress PBF blocks
// on an internal thread pool, but its read() contract hands blocks back in
// the order they appear in the file, and this reader applies them to the
// handler in that same order as they arrive. Single-threaded decode
// (G1 scope).
#pragma once

#include "g2m/decode/byte_source.h"

#include <cstdint>
#include <string>
#include <vector>

namespace g2m::decode::osm {

struct Tag {
    std::string key;
    std::string value;
};

// lat/lon exactly as the PBF stores them: signed 1e-7 degree fixed point
// (libosmium's own osmium::Location::x()/y() - never round-tripped through
// a floating-point degree conversion).
struct OsmNode {
    std::int64_t id = 0;
    std::int32_t lon_1e7 = 0;
    std::int32_t lat_1e7 = 0;
    std::vector<Tag> tags;
};

struct OsmWay {
    std::int64_t id = 0;
    std::vector<std::int64_t> node_refs;
    std::vector<Tag> tags;
};

enum class OsmMemberType : std::uint8_t { Node, Way, Relation };

struct OsmMember {
    OsmMemberType type = OsmMemberType::Node;
    std::int64_t ref = 0;
    std::string role;
};

struct OsmRelation {
    std::int64_t id = 0;
    std::vector<OsmMember> members;
    std::vector<Tag> tags;
};

// A degrees (WGS84) bounding-box prefilter. Only NODES are filtered by
// location before reaching the handler (ways/relations always pass
// through - filtering them by member containment needs a second pass over
// their referenced nodes, out of G1b's scope; a caller wanting way/relation
// geometry inside a bbox reads every way/relation and cross-references the
// node ids it already streamed).
struct BoundingBox {
    double min_lat = -90.0;
    double max_lat = 90.0;
    double min_lon = -180.0;
    double max_lon = 180.0;

    bool contains(double lat, double lon) const {
        return lat >= min_lat && lat <= max_lat && lon >= min_lon && lon <= max_lon;
    }
};

// Streaming callback interface. Every method defaults to a no-op; override
// only what you need.
class PbfHandler {
public:
    virtual ~PbfHandler() = default;
    virtual void on_node(const OsmNode&) {}
    virtual void on_way(const OsmWay&) {}
    virtual void on_relation(const OsmRelation&) {}
};

struct PbfStats {
    std::uint64_t node_count = 0;
    std::uint64_t way_count = 0;
    std::uint64_t relation_count = 0;
};

// Which entity types a read_pbf() call should decode and deliver (I2,
// g2m_import's region_resolve.h/g2m_tiler): a multi-pass reader (find the
// admin relation among relations only, then its member ways, then their
// node positions) is both cheaper and simpler when each pass skips decoding
// the block types it does not need, rather than filtering entities the
// Bridge already spent time converting. Bitwise-combinable
// (`PbfEntities::Nodes | PbfEntities::Ways`).
enum class PbfEntities : std::uint8_t {
    Nodes = 1,
    Ways = 2,
    Relations = 4,
    All = Nodes | Ways | Relations,
};
constexpr PbfEntities operator|(PbfEntities a, PbfEntities b) {
    return static_cast<PbfEntities>(static_cast<std::uint8_t>(a) | static_cast<std::uint8_t>(b));
}
constexpr PbfEntities operator&(PbfEntities a, PbfEntities b) {
    return static_cast<PbfEntities>(static_cast<std::uint8_t>(a) & static_cast<std::uint8_t>(b));
}
constexpr bool any(PbfEntities a) { return static_cast<std::uint8_t>(a) != 0; }

// Streams `path` (a .osm.pbf file) to `handler` in file order. `bbox`, if
// non-null, prefilters nodes by location (see BoundingBox's own comment).
// `entities` restricts which of on_node/on_way/on_relation are ever called
// (and, for entity types entirely excluded, lets the underlying reader skip
// decoding those blocks - a real cost saving on a large file: I2's
// region_resolve.h reads a whole-country PBF once per entity type it
// actually needs per pass). Counts every entity actually delivered
// (post-filter) in the returned PbfStats. Throws DecodeError if the file
// cannot be opened or is malformed.
PbfStats read_pbf(const std::string& path, PbfHandler& handler, const BoundingBox* bbox = nullptr,
                   PbfEntities entities = PbfEntities::All);

} // namespace g2m::decode::osm
