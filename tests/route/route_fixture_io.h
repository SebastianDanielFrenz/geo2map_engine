// tests/route/route_fixture_io.h - reads a committed OSM fixture with its
// relations (turn restrictions), for the route tests.
#pragma once

#include "g2m/decode/osm/pbf_reader.h"
#include "g2m/layer/osm_data.h"

#include <catch2/catch_test_macros.hpp>

#include <string>
#include <utility>
#include <vector>

namespace g2m_test {

inline g2m::osm::OsmData read_route_fixture(const std::string& name) {
    namespace osm = g2m::osm;
    namespace dec = g2m::decode::osm;
    struct Reader final : dec::PbfHandler {
        osm::OsmBuilder builder;
        void on_node(const dec::OsmNode& n) override {
            osm::OsmBuilder::TagList tags;
            for (const auto& t : n.tags) tags.emplace_back(t.key, t.value);
            builder.add_node(n.id, n.lat_1e7, n.lon_1e7, std::move(tags));
        }
        void on_way(const dec::OsmWay& w) override {
            osm::OsmBuilder::TagList tags;
            for (const auto& t : w.tags) tags.emplace_back(t.key, t.value);
            builder.add_way(w.id, w.node_refs, std::move(tags));
        }
        void on_relation(const dec::OsmRelation& r) override {
            std::vector<osm::OsmBuilder::MemberIn> members;
            for (const auto& m : r.members)
                members.push_back({m.type == dec::OsmMemberType::Node  ? osm::MemberType::Node
                                   : m.type == dec::OsmMemberType::Way ? osm::MemberType::Way
                                                                       : osm::MemberType::Relation,
                                   m.ref, m.role});
            osm::OsmBuilder::TagList tags;
            for (const auto& t : r.tags) tags.emplace_back(t.key, t.value);
            builder.add_relation(r.id, std::move(members), std::move(tags));
        }
    } reader;
    dec::read_pbf(std::string(G2M_SOURCE_DIR) + "/testdata/fixtures/osm/" + name + ".osm.pbf", reader, nullptr);
    auto data = reader.builder.build();
    REQUIRE(data);
    return std::move(*data);
}

} // namespace g2m_test
