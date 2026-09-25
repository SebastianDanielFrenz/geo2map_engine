// src/decode/osm/pbf_reader.cpp - see g2m/decode/osm/pbf_reader.h. Every
// osmium/protozero type is confined to this translation unit.

#include "g2m/decode/osm/pbf_reader.h"

// NOMINMAX/WIN32_LEAN_AND_MEAN/_CRT_SECURE_NO_WARNINGS are supplied by the
// g2m_dep_libosmium INTERFACE target (cmake/deps_decoders.cmake) so they
// apply consistently to every TU that includes osmium headers.
#include <osmium/handler.hpp>
#include <osmium/io/pbf_input.hpp> // registers ONLY the PBF format factory - avoids any_input.hpp's
                                    // transitive pull of bzip2_compression.hpp (bzlib.h, not an
                                    // approved dependency and not needed: PBF-only reading, PLAN.md 13)
#include <osmium/osm/entity_bits.hpp>
#include <osmium/osm/node.hpp>
#include <osmium/osm/relation.hpp>
#include <osmium/osm/way.hpp>
#include <osmium/visitor.hpp>

namespace g2m::decode::osm {
namespace {

std::vector<Tag> convert_tags(const osmium::TagList& tags) {
    std::vector<Tag> out;
    out.reserve(tags.size());
    for (const auto& t : tags) out.push_back(Tag{t.key(), t.value()});
    return out;
}

OsmMemberType convert_member_type(osmium::item_type t) {
    switch (t) {
        case osmium::item_type::node: return OsmMemberType::Node;
        case osmium::item_type::way: return OsmMemberType::Way;
        case osmium::item_type::relation: return OsmMemberType::Relation;
        default: throw DecodeError("osm: relation member with an unsupported type");
    }
}

class Bridge : public osmium::handler::Handler {
public:
    Bridge(PbfHandler& handler, const BoundingBox* bbox) : handler_(handler), bbox_(bbox) {}

    void node(const osmium::Node& n) {
        const osmium::Location loc = n.location();
        if (bbox_ != nullptr) {
            if (!loc.valid()) return;
            if (!bbox_->contains(loc.lat(), loc.lon())) return;
        }
        OsmNode out;
        out.id = n.id();
        if (loc.valid()) {
            out.lon_1e7 = loc.x();
            out.lat_1e7 = loc.y();
        }
        out.tags = convert_tags(n.tags());
        handler_.on_node(out);
        ++stats_.node_count;
    }

    void way(const osmium::Way& w) {
        OsmWay out;
        out.id = w.id();
        out.node_refs.reserve(w.nodes().size());
        for (const auto& nr : w.nodes()) out.node_refs.push_back(nr.ref());
        out.tags = convert_tags(w.tags());
        handler_.on_way(out);
        ++stats_.way_count;
    }

    void relation(const osmium::Relation& r) {
        OsmRelation out;
        out.id = r.id();
        out.members.reserve(r.members().size());
        for (const auto& m : r.members()) {
            out.members.push_back(OsmMember{convert_member_type(m.type()), m.ref(), std::string(m.role())});
        }
        out.tags = convert_tags(r.tags());
        handler_.on_relation(out);
        ++stats_.relation_count;
    }

    const PbfStats& stats() const { return stats_; }

private:
    PbfHandler& handler_;
    const BoundingBox* bbox_;
    PbfStats stats_;
};

} // namespace

PbfStats read_pbf(const std::string& path, PbfHandler& handler, const BoundingBox* bbox) {
    try {
        osmium::io::Reader reader(path, osmium::osm_entity_bits::node | osmium::osm_entity_bits::way |
                                             osmium::osm_entity_bits::relation);
        Bridge bridge(handler, bbox);
        while (osmium::memory::Buffer buffer = reader.read()) {
            osmium::apply(buffer, bridge);
        }
        reader.close();
        return bridge.stats();
    } catch (const DecodeError&) {
        throw;
    } catch (const std::exception& e) {
        throw DecodeError(std::string("osm: ") + e.what());
    }
}

} // namespace g2m::decode::osm
