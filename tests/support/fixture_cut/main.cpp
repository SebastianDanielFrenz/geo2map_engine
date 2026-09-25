// tests/support/fixture_cut/main.cpp - test-only fixture-cutting CLI
// (target g2m_fixture_cut, PLAN.md G1c I2 tests/support/fixture_cut; keeps
// libosmium's PBF WRITER out of product code - g2m_decode only ever reads
// PBF, see cmake/deps_decoders.cmake - by confining it to this test-only
// tool, D20).
//
// Subcommand implemented in I2: cut-pbf (a small resolvable OSM PBF
// fixture, extracted from a full-size source extract). Later increments
// (I3+) are expected to add cut-worldcover/repack-dgm1, reusing
// tests/support/{zip_builder.h,tiff_builder.h} - not implemented here (out
// of I2 scope).
//
// cut-pbf --in PATH --relation ID [--relation ID...] [--bbox
//          lon0,lat0,lon1,lat1] --out PATH
//
// DEVIATION from the plan's literal "all objects in bbox + listed relations
// with member ways/nodes + ways referencing bbox nodes made complete":
// this implementation extracts exactly the listed relations' transitive
// closure - each relation's own tags/members, each Way-typed member's own
// tags/node-refs, and each of those ways' referenced nodes' own
// tags/location - and nothing else. It does NOT scan the source file for
// unrelated objects whose geometry merely intersects --bbox (that needs a
// node-location-indexed two-pass bbox scan, the "smart"/"complete_ways"
// strategy osmium's own `osmium extract` tool implements) - that
// background context is not needed for a small ADMIN-BOUNDARY fixture
// resolvable by g2m::import::resolve_region, only the requested relation's
// own closure is. --bbox is still accepted and recorded into the output
// PBF's header bounding box (provenance only) but does not otherwise
// filter what gets extracted. A relation-typed member (e.g. a sub-area of
// a multi-level admin boundary) is NOT recursed into - it is dropped from
// the member list with a warning on stderr (none of this project's own
// admin relations use nested relation members; recursing would need
// unbounded-depth cycle detection for a case this tool never actually
// hits).
#include "g2m/decode/osm/pbf_reader.h"

#include <osmium/builder/osm_object_builder.hpp>
// Only the PBF writer, NOT <osmium/io/any_output.hpp> - the "any" header
// registers every osmium output format (XML, O5M, bzip2-compressed
// variants, ...) and pulls in <bzlib.h>, a dependency this project does
// not declare (cmake/deps_decoders.cmake). pbf_output.hpp is libosmium's
// own documented minimal header for "if you want to write OSM PBF files".
#include <osmium/io/pbf_output.hpp>
#include <osmium/memory/buffer.hpp>
#include <osmium/osm/box.hpp>
#include <osmium/osm/item_type.hpp>
#include <osmium/osm/location.hpp>

#include <algorithm>
#include <cstdlib>
#include <iostream>
#include <map>
#include <optional>
#include <set>
#include <sstream>
#include <string>
#include <vector>

using g2m::decode::DecodeError;
using namespace g2m::decode::osm;

namespace {

struct Args {
    std::vector<std::string> positional;
    std::vector<std::pair<std::string, std::string>> flags;

    std::vector<std::string> get_all(const std::string& key) const {
        std::vector<std::string> out;
        for (const auto& [k, v] : flags)
            if (k == key) out.push_back(v);
        return out;
    }
    std::optional<std::string> get(const std::string& key) const {
        for (const auto& [k, v] : flags) {
            if (k == key) return v;
        }
        return std::nullopt;
    }
};

Args parse_args(int argc, char** argv, int from) {
    Args out;
    for (int i = from; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg.size() > 2 && arg[0] == '-' && arg[1] == '-') {
            const std::string key = arg.substr(2);
            if (i + 1 >= argc) {
                out.flags.emplace_back(key, "");
                continue;
            }
            out.flags.emplace_back(key, argv[++i]);
        } else {
            out.positional.push_back(arg);
        }
    }
    return out;
}

struct Bbox {
    double west, south, east, north;
};

std::optional<Bbox> parse_bbox(const std::string& s) {
    std::vector<double> vals;
    std::stringstream ss(s);
    std::string tok;
    while (std::getline(ss, tok, ',')) {
        try {
            vals.push_back(std::stod(tok));
        } catch (const std::exception&) {
            return std::nullopt;
        }
    }
    if (vals.size() != 4) return std::nullopt;
    return Bbox{vals[0], vals[1], vals[2], vals[3]};
}

// --- pass 1: relations ------------------------------------------------------

class RelationCollector : public PbfHandler {
public:
    explicit RelationCollector(std::vector<std::int64_t> wanted) : wanted_(std::move(wanted)) {
        std::sort(wanted_.begin(), wanted_.end());
    }
    void on_relation(const OsmRelation& r) override {
        if (std::binary_search(wanted_.begin(), wanted_.end(), r.id)) found_[r.id] = r;
    }
    std::map<std::int64_t, OsmRelation> found_;

private:
    std::vector<std::int64_t> wanted_;
};

// --- pass 2: ways ------------------------------------------------------------

class WayCollector : public PbfHandler {
public:
    explicit WayCollector(std::vector<std::int64_t> wanted) : wanted_(std::move(wanted)) {
        std::sort(wanted_.begin(), wanted_.end());
    }
    void on_way(const OsmWay& w) override {
        if (std::binary_search(wanted_.begin(), wanted_.end(), w.id)) found_[w.id] = w;
    }
    std::map<std::int64_t, OsmWay> found_;

private:
    std::vector<std::int64_t> wanted_;
};

// --- pass 3: nodes -----------------------------------------------------------

class NodeCollector : public PbfHandler {
public:
    explicit NodeCollector(std::vector<std::int64_t> wanted) : wanted_(std::move(wanted)) {
        std::sort(wanted_.begin(), wanted_.end());
    }
    void on_node(const OsmNode& n) override {
        if (std::binary_search(wanted_.begin(), wanted_.end(), n.id)) found_[n.id] = n;
    }
    std::map<std::int64_t, OsmNode> found_;

private:
    std::vector<std::int64_t> wanted_;
};

int cmd_cut_pbf(const Args& args) {
    const auto in_path = args.get("in");
    const auto out_path = args.get("out");
    const auto relation_strs = args.get_all("relation");
    if (!in_path || !out_path || relation_strs.empty()) {
        std::cerr << "g2m_fixture_cut cut-pbf: --in, --out and at least one --relation are required\n";
        return 1;
    }
    std::optional<Bbox> bbox;
    if (const auto bbox_str = args.get("bbox")) {
        bbox = parse_bbox(*bbox_str);
        if (!bbox) {
            std::cerr << "g2m_fixture_cut cut-pbf: --bbox must be lon0,lat0,lon1,lat1\n";
            return 1;
        }
    }
    std::vector<std::int64_t> wanted_relations;
    for (const auto& s : relation_strs) {
        try {
            wanted_relations.push_back(std::stoll(s));
        } catch (const std::exception&) {
            std::cerr << "g2m_fixture_cut cut-pbf: --relation '" << s << "' is not an integer\n";
            return 1;
        }
    }

    try {
        // Pass 1: relations.
        RelationCollector relations(wanted_relations);
        read_pbf(*in_path, relations, nullptr, PbfEntities::Relations);
        for (const auto id : wanted_relations) {
            if (!relations.found_.count(id)) {
                std::cerr << "g2m_fixture_cut cut-pbf: relation " << id << " not found in " << *in_path << "\n";
                return 1;
            }
        }

        // Wanted way ids: every Way-typed member of every found relation.
        // A Relation-typed member is dropped (not recursed into, see this
        // file's header comment); a Node-typed member's id goes straight
        // into wanted_nodes below.
        std::set<std::int64_t> wanted_ways;
        std::set<std::int64_t> wanted_nodes_from_members;
        for (const auto& [rid, rel] : relations.found_) {
            for (const auto& m : rel.members) {
                if (m.type == OsmMemberType::Way) {
                    wanted_ways.insert(m.ref);
                } else if (m.type == OsmMemberType::Node) {
                    wanted_nodes_from_members.insert(m.ref);
                } else {
                    std::cerr << "g2m_fixture_cut cut-pbf: warning: relation " << rid
                              << " has a relation-typed member (ref " << m.ref << ") - not recursed into, dropped\n";
                }
            }
        }

        // Pass 2: ways.
        WayCollector ways(std::vector<std::int64_t>(wanted_ways.begin(), wanted_ways.end()));
        if (!wanted_ways.empty()) read_pbf(*in_path, ways, nullptr, PbfEntities::Ways);
        for (const auto id : wanted_ways) {
            if (!ways.found_.count(id)) {
                std::cerr << "g2m_fixture_cut cut-pbf: way " << id << " (member of a requested relation) not found in "
                          << *in_path << "\n";
                return 1;
            }
        }

        std::set<std::int64_t> wanted_nodes(wanted_nodes_from_members.begin(), wanted_nodes_from_members.end());
        for (const auto& [wid, way] : ways.found_)
            for (const auto ref : way.node_refs) wanted_nodes.insert(ref);

        // Pass 3: nodes.
        NodeCollector nodes(std::vector<std::int64_t>(wanted_nodes.begin(), wanted_nodes.end()));
        if (!wanted_nodes.empty()) read_pbf(*in_path, nodes, nullptr, PbfEntities::Nodes);
        for (const auto id : wanted_nodes) {
            if (!nodes.found_.count(id)) {
                std::cerr << "g2m_fixture_cut cut-pbf: node " << id << " (referenced by a requested way/relation) not found in "
                          << *in_path << "\n";
                return 1;
            }
        }

        // --- write the fixture: nodes then ways then relations, each
        // block sorted by id ascending (std::map's own iteration order).
        osmium::io::Header header;
        header.set("generator", "g2m_fixture_cut");
        if (bbox) header.add_box(osmium::Box{bbox->west, bbox->south, bbox->east, bbox->north});

        osmium::io::File out_file{*out_path};
        out_file.set("add_metadata", false);
        out_file.set("pbf_dense_nodes", true);
        out_file.set("pbf_compression", "zlib");

        osmium::io::Writer writer{out_file, header, osmium::io::overwrite::allow};

        osmium::memory::Buffer buffer{1024 * 1024, osmium::memory::Buffer::auto_grow::yes};
        for (const auto& [id, n] : nodes.found_) {
            {
                osmium::builder::NodeBuilder builder{buffer};
                builder.set_id(n.id);
                builder.set_location(osmium::Location{n.lon_1e7, n.lat_1e7});
                if (!n.tags.empty()) {
                    osmium::builder::TagListBuilder tl{builder};
                    for (const auto& t : n.tags) tl.add_tag(t.key, t.value);
                }
            }
            buffer.commit();
        }
        for (const auto& [id, w] : ways.found_) {
            {
                osmium::builder::WayBuilder builder{buffer};
                builder.set_id(w.id);
                if (!w.tags.empty()) {
                    osmium::builder::TagListBuilder tl{builder};
                    for (const auto& t : w.tags) tl.add_tag(t.key, t.value);
                }
                {
                    osmium::builder::WayNodeListBuilder wnl{builder};
                    for (const auto ref : w.node_refs) wnl.add_node_ref(ref);
                }
            }
            buffer.commit();
        }
        for (const auto& [id, r] : relations.found_) {
            {
                osmium::builder::RelationBuilder builder{buffer};
                builder.set_id(r.id);
                if (!r.tags.empty()) {
                    osmium::builder::TagListBuilder tl{builder};
                    for (const auto& t : r.tags) tl.add_tag(t.key, t.value);
                }
                {
                    osmium::builder::RelationMemberListBuilder rml{builder};
                    for (const auto& m : r.members) {
                        if (m.type == OsmMemberType::Relation) continue; // dropped, see header comment
                        const auto item_type =
                            m.type == OsmMemberType::Way ? osmium::item_type::way : osmium::item_type::node;
                        rml.add_member(item_type, m.ref, m.role);
                    }
                }
            }
            buffer.commit();
        }
        writer(std::move(buffer));
        writer.close();

        std::cerr << "g2m_fixture_cut cut-pbf: wrote " << nodes.found_.size() << " nodes, " << ways.found_.size()
                  << " ways, " << relations.found_.size() << " relations to " << *out_path << "\n";
        return 0;
    } catch (const DecodeError& e) {
        std::cerr << "g2m_fixture_cut cut-pbf: " << e.what() << "\n";
        return 1;
    } catch (const std::exception& e) {
        std::cerr << "g2m_fixture_cut cut-pbf: " << e.what() << "\n";
        return 1;
    }
}

int usage() {
    std::cerr << "usage: g2m_fixture_cut <command> [options]\n"
                 "commands:\n"
                 "  cut-pbf --in PATH --relation ID [--relation ID...] [--bbox lon0,lat0,lon1,lat1] --out PATH\n";
    return 1;
}

} // namespace

int main(int argc, char** argv) {
    if (argc < 2) return usage();
    const std::string command = argv[1];
    const Args args = parse_args(argc, argv, 2);
    if (command == "cut-pbf") return cmd_cut_pbf(args);
    std::cerr << "g2m_fixture_cut: unknown command '" << command << "'\n";
    return usage();
}
