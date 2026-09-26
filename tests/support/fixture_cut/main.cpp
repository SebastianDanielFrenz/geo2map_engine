// tests/support/fixture_cut/main.cpp - test-only fixture-cutting CLI
// (target g2m_fixture_cut, PLAN.md G1c I2 tests/support/fixture_cut; keeps
// libosmium's PBF WRITER out of product code - g2m_decode only ever reads
// PBF, see cmake/deps_decoders.cmake - by confining it to this test-only
// tool, D20).
//
// Subcommand implemented in I2: cut-pbf (a small resolvable OSM PBF
// fixture, extracted from a full-size source extract). repack-dgm1 (I3) and
// cut-worldcover (I5) were added later, both reusing
// tests/support/{zip_builder.h,tiff_builder.h}.
//
// cut-pbf --in PATH --relation ID [--relation ID...] [--bbox
//          lon0,lat0,lon1,lat1] --out PATH
//
// PLAN.md G1c pre-item 2: with --bbox, this is a real "everything in bbox"
// cut, not just provenance metadata:
//  - every node inside --bbox is included;
//  - every way with at least one node ref inside --bbox is included, and
//    COMPLETED with every one of its own node refs (even ones outside
//    --bbox), so no included way ever has a dangling node reference;
//  - every --relation id (and, recursively, every Relation-typed member of
//    an already-included relation - see below) gets the SAME full
//    treatment: every Way-typed member is included and completed with all
//    its nodes, every Node-typed member is included;
//  - beyond that, any OTHER relation in the source file that references an
//    already-included node/way/relation is ALSO included ("closure
//    upward") - best-effort only: an upward-swept relation's OWN members
//    are not force-completed, so it may reference an object this cut does
//    not carry (the same "possibly incomplete at the edge" trade-off
//    osmium's own bbox-extraction strategies make; only the explicit
//    --relation closure above is guaranteed complete).
// Without --bbox, behaviour is unchanged from I2 except for the recursion
// fix below: --bbox is still accepted and recorded into the output PBF's
// header bounding box either way.
//
// FIXED since I2: a Relation-typed member of an included relation (e.g. a
// sub-area of a multi-level admin boundary) is now RECURSED into - given
// the exact same full-closure treatment - instead of being dropped from the
// member list with a warning. Implemented as an in-memory graph closure
// over every relation in the file (see all_relations below), so cycles
// (a relation referencing itself, directly or through others) terminate
// naturally: a relation id is added to the included set at most once.
//
// One full, unfiltered Relations pass loads every relation's id/members/
// tags into memory up front (`all_relations`) - this is what makes both the
// downward recursion above and the upward bbox sweep below pure in-memory
// graph problems needing no further re-reads of the (possibly huge) source
// file. This costs some memory (a country extract's relations are a small
// fraction of its total object count) but not extra wall-clock time: the
// PBF entity-bits reader (PbfEntities) already has to decode every
// Relations block whether or not a caller further filters by id.
#include "g2m/decode/byte_source.h"
#include "g2m/decode/dgm1.h"
#include "g2m/decode/osm/pbf_reader.h"
#include "g2m/decode/tiff/tiff_reader.h"
#include "g2m/decode/zip/zip_reader.h"
#include "g2m/layer/region.h"
#include "tiff_builder.h"
#include "zip_builder.h"

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
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <functional>
#include <iostream>
#include <limits>
#include <map>
#include <memory>
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

// --- pass: every relation in the file (unfiltered - see this file's own
// header comment for why) ----------------------------------------------------

class AllRelationCollector : public PbfHandler {
public:
    void on_relation(const OsmRelation& r) override { found_[r.id] = r; }
    std::map<std::int64_t, OsmRelation> found_;
};

// --- pass: ways, either explicitly wanted (by id) or swept in because at
// least one of their node refs lies inside --bbox (bbox_node_ids empty ==
// no bbox given == degrades to a plain by-id WayCollector) ------------------

class WaySweepCollector : public PbfHandler {
public:
    WaySweepCollector(std::vector<std::int64_t> wanted_ids, std::vector<std::int64_t> bbox_node_ids)
        : wanted_ids_(std::move(wanted_ids)), bbox_node_ids_(std::move(bbox_node_ids)) {
        std::sort(wanted_ids_.begin(), wanted_ids_.end());
        std::sort(bbox_node_ids_.begin(), bbox_node_ids_.end());
    }
    void on_way(const OsmWay& w) override {
        bool include = std::binary_search(wanted_ids_.begin(), wanted_ids_.end(), w.id);
        if (!include) {
            for (const auto ref : w.node_refs) {
                if (std::binary_search(bbox_node_ids_.begin(), bbox_node_ids_.end(), ref)) {
                    include = true;
                    break;
                }
            }
        }
        if (include) found_[w.id] = w;
    }
    std::map<std::int64_t, OsmWay> found_;

private:
    std::vector<std::int64_t> wanted_ids_;
    std::vector<std::int64_t> bbox_node_ids_;
};

// --- pass: every node inside --bbox (BoundingBox's own node-only prefilter
// already restricts delivery - nothing further to filter here) --------------

class BboxNodeCollector : public PbfHandler {
public:
    void on_node(const OsmNode& n) override { found_[n.id] = n; }
    std::map<std::int64_t, OsmNode> found_;
};

// --- pass: nodes by id --------------------------------------------------

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
        // Pass: every relation in the file, unfiltered (this file's own
        // header comment explains why).
        AllRelationCollector all_relations_pass;
        read_pbf(*in_path, all_relations_pass, nullptr, PbfEntities::Relations);
        const std::map<std::int64_t, OsmRelation>& all_relations = all_relations_pass.found_;

        for (const auto id : wanted_relations) {
            if (!all_relations.count(id)) {
                std::cerr << "g2m_fixture_cut cut-pbf: relation " << id << " not found in " << *in_path << "\n";
                return 1;
            }
        }

        // Explicit closure, downward: every --relation id, plus every
        // Relation-typed member of an already-included relation, recursively
        // - replaces I2's "drop with a warning" (this file's header
        // comment). A dangling Relation-typed member (ref not present in the
        // file at all) is warned about, not an error.
        std::set<std::int64_t> explicit_relations(wanted_relations.begin(), wanted_relations.end());
        {
            std::vector<std::int64_t> frontier(wanted_relations.begin(), wanted_relations.end());
            while (!frontier.empty()) {
                std::vector<std::int64_t> next;
                for (const auto rid : frontier) {
                    const auto it = all_relations.find(rid);
                    if (it == all_relations.end()) continue; // unreachable: checked above for explicit ids
                    for (const auto& m : it->second.members) {
                        if (m.type != OsmMemberType::Relation) continue;
                        if (explicit_relations.count(m.ref)) continue; // already queued/included
                        const auto child = all_relations.find(m.ref);
                        if (child == all_relations.end()) {
                            std::cerr << "g2m_fixture_cut cut-pbf: warning: relation " << rid
                                      << " has a relation-typed member (ref " << m.ref << ") not found in " << *in_path
                                      << " - skipped\n";
                            continue;
                        }
                        explicit_relations.insert(m.ref);
                        next.push_back(m.ref);
                    }
                }
                frontier = std::move(next);
            }
        }

        // Every Way-typed member of the explicit closure gets full
        // completion (fetched below, then completed with ALL its own node
        // refs); every Node-typed member is wanted directly.
        std::set<std::int64_t> wanted_ways_explicit;
        std::set<std::int64_t> wanted_nodes;
        for (const auto rid : explicit_relations) {
            for (const auto& m : all_relations.at(rid).members) {
                if (m.type == OsmMemberType::Way) {
                    wanted_ways_explicit.insert(m.ref);
                } else if (m.type == OsmMemberType::Node) {
                    wanted_nodes.insert(m.ref);
                }
                // Relation-typed members are exactly explicit_relations - no
                // separate bookkeeping needed.
            }
        }

        // Bbox node pass: every node inside --bbox (BoundingBox's own
        // node-only prefilter - see pbf_reader.h's own comment).
        std::map<std::int64_t, OsmNode> bbox_nodes;
        std::vector<std::int64_t> bbox_node_ids;
        if (bbox) {
            BoundingBox filter;
            filter.min_lon = bbox->west;
            filter.max_lon = bbox->east;
            filter.min_lat = bbox->south;
            filter.max_lat = bbox->north;
            BboxNodeCollector bbox_pass;
            read_pbf(*in_path, bbox_pass, &filter, PbfEntities::Nodes);
            bbox_nodes = std::move(bbox_pass.found_);
            bbox_node_ids.reserve(bbox_nodes.size());
            for (const auto& [id, n] : bbox_nodes) bbox_node_ids.push_back(id);
        }

        // Ways pass: explicit-closure ways, plus (bbox mode) every way with
        // at least one node ref inside --bbox (bbox_node_ids is empty
        // without --bbox, so this degrades to exactly the explicit set).
        WaySweepCollector way_sweep(std::vector<std::int64_t>(wanted_ways_explicit.begin(), wanted_ways_explicit.end()),
                                    bbox_node_ids);
        if (!wanted_ways_explicit.empty() || bbox) read_pbf(*in_path, way_sweep, nullptr, PbfEntities::Ways);
        for (const auto id : wanted_ways_explicit) {
            if (!way_sweep.found_.count(id)) {
                std::cerr << "g2m_fixture_cut cut-pbf: way " << id << " (member of a requested relation) not found in "
                          << *in_path << "\n";
                return 1;
            }
        }
        const std::map<std::int64_t, OsmWay>& ways_found = way_sweep.found_;

        // Every included way is completed with ALL its own node refs (even
        // ones outside --bbox) - "all ways with >= 1 node in bbox, completed
        // with all their nodes".
        for (const auto& [wid, w] : ways_found)
            for (const auto ref : w.node_refs) wanted_nodes.insert(ref);

        std::vector<std::int64_t> extra_node_ids;
        for (const auto id : wanted_nodes) {
            if (!bbox_nodes.count(id)) extra_node_ids.push_back(id);
        }
        std::map<std::int64_t, OsmNode> extra_nodes;
        if (!extra_node_ids.empty()) {
            NodeCollector node_pass(extra_node_ids);
            read_pbf(*in_path, node_pass, nullptr, PbfEntities::Nodes);
            extra_nodes = std::move(node_pass.found_);
        }
        for (const auto id : wanted_nodes) {
            if (bbox_nodes.count(id) || extra_nodes.count(id)) continue;
            std::cerr << "g2m_fixture_cut cut-pbf: node " << id
                       << " (referenced by a requested way/relation) not found in " << *in_path << "\n";
            return 1;
        }

        // Every bbox node is written too, even one no way/relation
        // references ("every node inside --bbox is included").
        std::map<std::int64_t, OsmNode> nodes_found = std::move(bbox_nodes);
        for (auto& [id, n] : extra_nodes) nodes_found.emplace(id, std::move(n));

        // Upward sweep (bbox mode only): any OTHER relation in the file that
        // references an already-included node/way/relation is included too
        // - best-effort ("closure upward"): its own members are NOT
        // force-completed, unlike the explicit closure above (this file's
        // header comment).
        std::set<std::int64_t> included_relations = explicit_relations;
        if (bbox) {
            std::set<std::int64_t> included_node_ids;
            for (const auto& [id, n] : nodes_found) included_node_ids.insert(id);
            std::set<std::int64_t> included_way_ids;
            for (const auto& [id, w] : ways_found) included_way_ids.insert(id);

            for (bool changed = true; changed;) {
                changed = false;
                for (const auto& [rid, rel] : all_relations) {
                    if (included_relations.count(rid)) continue;
                    bool member_included = false;
                    for (const auto& m : rel.members) {
                        if ((m.type == OsmMemberType::Way && included_way_ids.count(m.ref)) ||
                            (m.type == OsmMemberType::Node && included_node_ids.count(m.ref)) ||
                            (m.type == OsmMemberType::Relation && included_relations.count(m.ref))) {
                            member_included = true;
                            break;
                        }
                    }
                    if (member_included) {
                        included_relations.insert(rid);
                        changed = true;
                    }
                }
                // Downward: a newly swept relation's own Relation-typed
                // members are pulled in too (same closure rule as the
                // explicit set above).
                for (const auto& [rid, rel] : all_relations) {
                    if (!included_relations.count(rid)) continue;
                    for (const auto& m : rel.members) {
                        if (m.type == OsmMemberType::Relation && all_relations.count(m.ref) &&
                            !included_relations.count(m.ref)) {
                            included_relations.insert(m.ref);
                            changed = true;
                        }
                    }
                }
            }
        }

        // --- write the fixture: nodes then ways then relations, each
        // block sorted by id ascending (std::map/std::set's own iteration
        // order).
        osmium::io::Header header;
        header.set("generator", "g2m_fixture_cut");
        if (bbox) header.add_box(osmium::Box{bbox->west, bbox->south, bbox->east, bbox->north});

        osmium::io::File out_file{*out_path};
        out_file.set("add_metadata", false);
        out_file.set("pbf_dense_nodes", true);
        out_file.set("pbf_compression", "zlib");

        osmium::io::Writer writer{out_file, header, osmium::io::overwrite::allow};

        osmium::memory::Buffer buffer{1024 * 1024, osmium::memory::Buffer::auto_grow::yes};
        for (const auto& [id, n] : nodes_found) {
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
        for (const auto& [id, w] : ways_found) {
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
        for (const auto rid : included_relations) {
            const auto& r = all_relations.at(rid);
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
                        const auto item_type = m.type == OsmMemberType::Way     ? osmium::item_type::way
                                                : m.type == OsmMemberType::Node ? osmium::item_type::node
                                                                                : osmium::item_type::relation;
                        rml.add_member(item_type, m.ref, m.role);
                    }
                }
            }
            buffer.commit();
        }
        writer(std::move(buffer));
        writer.close();

        std::cerr << "g2m_fixture_cut cut-pbf: wrote " << nodes_found.size() << " nodes, " << ways_found.size()
                  << " ways, " << included_relations.size() << " relations to " << *out_path << "\n";
        return 0;
    } catch (const DecodeError& e) {
        std::cerr << "g2m_fixture_cut cut-pbf: " << e.what() << "\n";
        return 1;
    } catch (const std::exception& e) {
        std::cerr << "g2m_fixture_cut cut-pbf: " << e.what() << "\n";
        return 1;
    }
}

// --- repack-dgm1 (PLAN.md G1c I3): copies a handful of REAL DGM1 tiles out
// of the real (read-only) PKT_*.zip delivery into a small nested-STORED
// fixture zip, byte-for-byte (no re-encoding of the .tif/.tfw content - only
// the ZIP-level framing is rebuilt, via ZipBuilder, always STORED at every
// level for this fixture regardless of the source's own compression choice
// at either level). Reuses g2m::decode::dgm1::enumerate_tiles() to locate
// each requested (e_km, n_km) tile inside a given outer zip's inner zip
// (matched by a plain substring of its raw entry name, since municipality
// names are not ASCII-safe to pass on argv) instead of re-deriving the
// dgm1_<E>_<N> filename convention here.
//
// repack-dgm1 --out PATH --inner OUTER_ZIP#SUBSTRING [--inner ...] --key E,N
//             [--key E,N ...]
//
// Each --inner becomes one inner-zip entry ("innerN.zip") in the output
// fixture, containing every requested --key tile found inside it (+ its
// sibling .tfw, if present) with its ORIGINAL tif/tfw entry name preserved
// (dgm1::Catalog's own filename parsing depends on it) - only the
// CONTAINING inner-zip's own name is normalised (municipality names are not
// needed by anything that reads the fixture back).
namespace dgm1 = g2m::decode::dgm1;

struct RepackKey {
    int e_km = 0;
    int n_km = 0;
};

std::optional<RepackKey> parse_key(const std::string& s) {
    const auto comma = s.find(',');
    if (comma == std::string::npos) return std::nullopt;
    try {
        return RepackKey{std::stoi(s.substr(0, comma)), std::stoi(s.substr(comma + 1))};
    } catch (const std::exception&) {
        return std::nullopt;
    }
}

int cmd_repack_dgm1(const Args& args) {
    const auto out_path = args.get("out");
    const auto inner_specs = args.get_all("inner");
    const auto key_strs = args.get_all("key");
    if (!out_path || inner_specs.empty() || key_strs.empty()) {
        std::cerr << "g2m_fixture_cut repack-dgm1: --out, at least one --inner OUTER_ZIP#SUBSTRING and at least one "
                     "--key E,N are required\n";
        return 1;
    }
    std::vector<RepackKey> keys;
    for (const auto& s : key_strs) {
        const auto k = parse_key(s);
        if (!k) {
            std::cerr << "g2m_fixture_cut repack-dgm1: --key '" << s << "' must be 'E,N'\n";
            return 1;
        }
        keys.push_back(*k);
    }

    try {
        g2m::test_support::ZipBuilder outer_builder;
        std::size_t total_tif_bytes = 0;
        std::size_t total_tfw_bytes = 0;
        int inner_idx = 0;

        for (const auto& spec : inner_specs) {
            const auto hash = spec.find('#');
            if (hash == std::string::npos) {
                std::cerr << "g2m_fixture_cut repack-dgm1: --inner '" << spec << "' must be 'OUTER_ZIP#SUBSTRING'\n";
                return 1;
            }
            const std::string outer_path = spec.substr(0, hash);
            const std::string substring = spec.substr(hash + 1);

            // enumerate_tiles parses every inner zip's tif filenames using
            // the exact same rule dgm1::Catalog will later use to read this
            // fixture back - reusing it here means the fixture is
            // guaranteed self-consistent with the reader, not hand-derived.
            const auto located = dgm1::enumerate_tiles({outer_path});

            std::optional<std::string> inner_entry_name;
            std::vector<dgm1::TileLocation> wanted_locations;
            for (const auto& [key, loc] : located) {
                if (loc.inner_zip_entry_name.find(substring) == std::string::npos) continue;
                const bool wanted =
                    std::any_of(keys.begin(), keys.end(),
                                [&](const RepackKey& k) { return k.e_km == key.e_km && k.n_km == key.n_km; });
                if (!wanted) continue;
                if (!inner_entry_name) inner_entry_name = loc.inner_zip_entry_name;
                wanted_locations.push_back(loc);
            }
            if (wanted_locations.empty()) {
                std::cerr << "g2m_fixture_cut repack-dgm1: no requested --key found in " << outer_path
                          << " inner entry matching '" << substring << "'\n";
                return 1;
            }

            auto outer_src = std::make_shared<g2m::decode::FileByteSource>(outer_path);
            g2m::decode::ZipReader outer_reader(outer_src);
            const g2m::decode::ZipEntry* inner_entry = nullptr;
            for (const auto& e : outer_reader.entries()) {
                if (e.name == *inner_entry_name) {
                    inner_entry = &e;
                    break;
                }
            }
            if (inner_entry == nullptr) {
                std::cerr << "g2m_fixture_cut repack-dgm1: inner entry '" << *inner_entry_name << "' vanished from "
                          << outer_path << "\n";
                return 1;
            }
            // Generous cap: this is a one-off maintainer tool, not the
            // real importer's bounded-RSS path - the real DGM1 delivery's
            // outer-to-inner entries are STORED anyway (D8), so this never
            // actually inflates.
            constexpr std::uint64_t kMaxInflate = std::uint64_t{4} << 30;
            const auto inner_src = outer_reader.entry_source(*inner_entry, kMaxInflate);
            if (!inner_src) {
                std::cerr << "g2m_fixture_cut repack-dgm1: " << inner_src.error().message << "\n";
                return 1;
            }
            g2m::decode::ZipReader inner_reader(*inner_src);

            g2m::test_support::ZipBuilder inner_builder;
            for (const auto& loc : wanted_locations) {
                const g2m::decode::ZipEntry* tif_entry = nullptr;
                for (const auto& e : inner_reader.entries()) {
                    if (e.name == loc.tif_entry_name) {
                        tif_entry = &e;
                        break;
                    }
                }
                if (tif_entry == nullptr) {
                    std::cerr << "g2m_fixture_cut repack-dgm1: tif entry '" << loc.tif_entry_name << "' vanished\n";
                    return 1;
                }
                const auto tif_bytes = inner_reader.read_entry(*tif_entry);
                inner_builder.add(g2m::test_support::ZipBuilder::Item{tif_entry->name, tif_bytes, /*deflate=*/false});
                total_tif_bytes += tif_bytes.size();

                const auto dot = tif_entry->name.find_last_of('.');
                const std::string tif_base = dot == std::string::npos ? tif_entry->name : tif_entry->name.substr(0, dot);
                for (const auto& e : inner_reader.entries()) {
                    if (e.name.size() <= 4 || e.name.compare(e.name.size() - 4, 4, ".tfw") != 0) continue;
                    if (e.name.compare(0, tif_base.size(), tif_base) != 0) continue;
                    const auto tfw_bytes = inner_reader.read_entry(e);
                    inner_builder.add(g2m::test_support::ZipBuilder::Item{e.name, tfw_bytes, /*deflate=*/false});
                    total_tfw_bytes += tfw_bytes.size();
                    break;
                }
            }

            const auto inner_zip_bytes = inner_builder.build();
            const std::string out_inner_name = "inner" + std::to_string(inner_idx++) + ".zip";
            std::cerr << "g2m_fixture_cut repack-dgm1: " << out_inner_name << " <- " << outer_path << "#"
                      << *inner_entry_name << " (" << wanted_locations.size() << " tile(s), " << inner_zip_bytes.size()
                      << " bytes)\n";
            outer_builder.add(g2m::test_support::ZipBuilder::Item{out_inner_name, inner_zip_bytes, /*deflate=*/false});
        }

        const auto outer_bytes = outer_builder.build();
        std::ofstream out(*out_path, std::ios::binary);
        if (!out) {
            std::cerr << "g2m_fixture_cut repack-dgm1: cannot write " << *out_path << "\n";
            return 1;
        }
        out.write(reinterpret_cast<const char*>(outer_bytes.data()), static_cast<std::streamsize>(outer_bytes.size()));
        out.close();
        std::cerr << "g2m_fixture_cut repack-dgm1: wrote " << *out_path << " (" << outer_bytes.size()
                  << " bytes total, " << total_tif_bytes << " tif bytes, " << total_tfw_bytes << " tfw bytes)\n";
        return 0;
    } catch (const DecodeError& e) {
        std::cerr << "g2m_fixture_cut repack-dgm1: " << e.what() << "\n";
        return 1;
    } catch (const std::exception& e) {
        std::cerr << "g2m_fixture_cut repack-dgm1: " << e.what() << "\n";
        return 1;
    }
}

// WorldCover's own N=12000 factors as 2^5 * 3 * 5^3 = 32 * 375 - NOT a power
// of two, so `num/12000` as a double generally does NOT round-trip back to
// `num` through `(num/12000) * 12000` (the same reconstruction landcover.cpp's
// own open_and_validate performs, `corner_x * n`): found via
// test_import_landcover.cpp's own fixture-vs-source byte-equality check,
// which failed because the naive quotient for this project's actual fixture
// windows (e.g. 8.50875 for one window corner's longitude) reconstructs to
// 102104.99999999999 - and a search over the surrounding representable
// doubles (checked up to +-2000 ULPs) never lands on the exact integer
// either: 102105.0 sits exactly BETWEEN the two representable reconstructed
// products bracketing it (each one ULP-of-the-PRODUCT away, since the
// product's own ULP at this magnitude, ~2^-52*2^17 =~ 2.9e-11, is coarser
// than one ULP of the ~8.5-magnitude quotient scaled by 12000, ~2.1e-11) -
// no double whose product with 12000 is correctly rounded can land on it,
// so nudging the quotient can never work here, for ANY num not already
// divisible by 375.
//
// The fix is therefore upstream, not at the tiepoint value itself: pick a
// window whose corner IS divisible by 375 (`align_down_multiple`, used on
// col_lo/row_lo below, before the `c0`/`c1` reference values - the real
// source file's own tiepoint is a whole degree, 6.0/51.0, and 6*12000 =
// 72000 = 375*192 is itself already a multiple of 375, so aligning col_lo/
// row_lo to a multiple of 375 is exactly equivalent to aligning new_c0/
// new_c1 to a multiple of 375). For such a value, num/12000 = (num/375)/32
// - a division by a PURE power of two, which IEEE-754 double division
// performs exactly (provided num/375 fits the mantissa, true here) - so the
// value written round-trips bit-exactly, the same way the real source
// file's own whole-degree tiepoint always has.
constexpr std::int64_t kTiepointAlignment = 375;

std::int64_t align_down_multiple(std::int64_t x, std::int64_t m) {
    const std::int64_t r = ((x % m) + m) % m;
    return x - r;
}

// cut-worldcover (PLAN.md G1c I5): cuts one uncompressed, stripped uint8
// GeoTIFF window per CONNECTED CLUSTER of a resolved region's level-2 tile
// footprint out of a real ESA WorldCover COG, for
// tests/unit/test_import_landcover.cpp's fixture. testdata/fixtures/regions.json's
// "fixture" region resolves (against the real hessen-latest.osm.pbf) to two
// tile clusters about 11 km apart - the Höchst admin relation and the
// separate utm_bbox part - so a single bounding window would need to be far
// larger than either cluster on its own; this tool instead 8-connects the
// resolved tile set into clusters (any gap larger than one tile ends a
// cluster - the ~11 km gap here is nowhere close) and cuts one window per
// cluster, in ascending (northing, easting) order, matching that many --out
// PATHs supplied in the same order.
//
// Per cluster: the tile set's own UTM bounding box (TileKey::min_easting/
// min_northing/size_m, no resampling - exact integer metres) is converted to
// a lon/lat bounding box via the same g2m::geo::Utm::reverse the real
// importer uses (D16: no new numerics), then widened by --margin-px source
// pixels and mapped to the source file's own pixel grid using its
// validated (N, C0, C1) - the identical col/row formula
// import/landcover.cpp's own header comment documents (D17). The window is
// decoded via TiffReader::decode_window_u8 and re-encoded as a fresh
// classic TIFF (TiffBuilder: uncompressed, single strip, uint8) whose
// ModelPixelScaleTag is unchanged and whose ModelTiepointTag is moved to the
// window's own corner (new_C0 = col_lo + source C0, new_C1 = source C1 -
// row_lo - both exact integers, and both multiples of 375 by construction,
// see align_down_multiple's own comment above), so (new_C0|new_C1)/N
// round-trips bit-exactly back through the same `corner_x * N` reconstruction
// landcover.cpp's own open_and_validate performs (a plain division does NOT
// do this in general for N=12000 - only for a multiple of 375).
// GeoKeyDirectory carries just the two keys the reader consumes
// (RasterTypeGeoKey=PixelIsArea, GeographicTypeGeoKey=4326) plus
// GDAL_NODATA="0" - a byte-for-byte-minimal but fully self-describing
// GeoTIFF, not a copy of the source file's own (possibly larger) tag set.
int cmd_cut_worldcover(const Args& args) {
    const auto in_path = args.get("in");
    const auto resolved_path = args.get("resolved");
    const auto out_paths = args.get_all("out");
    if (!in_path || !resolved_path || out_paths.empty()) {
        std::cerr << "g2m_fixture_cut cut-worldcover: --in, --resolved and at least one --out are required\n";
        return 1;
    }
    int margin_px = 2;
    if (const auto m = args.get("margin-px")) {
        try {
            margin_px = std::stoi(*m);
        } catch (const std::exception&) {
            std::cerr << "g2m_fixture_cut cut-worldcover: --margin-px must be an integer\n";
            return 1;
        }
    }

    try {
        std::ifstream resolved_in(*resolved_path, std::ios::binary);
        if (!resolved_in) {
            std::cerr << "g2m_fixture_cut cut-worldcover: cannot read " << *resolved_path << "\n";
            return 1;
        }
        std::ostringstream resolved_ss;
        resolved_ss << resolved_in.rdbuf();
        const auto resolved = g2m::parse_resolved_region(resolved_ss.str());
        if (!resolved) {
            std::cerr << "g2m_fixture_cut cut-worldcover: " << resolved.error().message << "\n";
            return 1;
        }
        std::vector<g2m::TileKey> keys = g2m::tile_keys(*resolved);
        if (keys.empty()) {
            std::cerr << "g2m_fixture_cut cut-worldcover: resolved region has no tiles\n";
            return 1;
        }
        std::sort(keys.begin(), keys.end());

        // 8-connectivity union-find over the tile grid (x/y within 1 of an
        // already-clustered tile joins its cluster) - a small O(n^2) pass is
        // fine for the handful of tiles a fixture region resolves to.
        std::vector<int> parent(keys.size());
        for (std::size_t i = 0; i < keys.size(); ++i) parent[i] = static_cast<int>(i);
        std::function<int(int)> find = [&](int i) {
            while (parent[static_cast<std::size_t>(i)] != i) {
                parent[static_cast<std::size_t>(i)] = parent[static_cast<std::size_t>(parent[static_cast<std::size_t>(i)])];
                i = parent[static_cast<std::size_t>(i)];
            }
            return i;
        };
        auto unite = [&](int a, int b) {
            a = find(a);
            b = find(b);
            if (a != b) parent[static_cast<std::size_t>(a)] = b;
        };
        for (std::size_t i = 0; i < keys.size(); ++i) {
            for (std::size_t j = i + 1; j < keys.size(); ++j) {
                if (keys[i].zone != keys[j].zone || keys[i].level != keys[j].level) continue;
                if (std::abs(keys[i].x - keys[j].x) <= 1 && std::abs(keys[i].y - keys[j].y) <= 1) {
                    unite(static_cast<int>(i), static_cast<int>(j));
                }
            }
        }
        std::map<int, std::vector<std::size_t>> clusters;
        for (std::size_t i = 0; i < keys.size(); ++i) clusters[find(static_cast<int>(i))].push_back(i);

        // Cluster order: ascending (min northing, min easting) of each
        // cluster's own bbox - deterministic and independent of union-find's
        // internal representative choice.
        struct Cluster {
            std::int64_t min_e, min_n, max_e, max_n;
            g2m::geo::UtmZone zone;
            std::size_t tile_count = 0;
        };
        std::vector<Cluster> cluster_list;
        for (const auto& [_, idxs] : clusters) {
            Cluster c;
            c.zone = keys[idxs.front()].zone;
            c.tile_count = idxs.size();
            c.min_e = c.min_n = std::numeric_limits<std::int64_t>::max();
            c.max_e = c.max_n = std::numeric_limits<std::int64_t>::min();
            for (std::size_t i : idxs) {
                const g2m::TileKey& k = keys[i];
                c.min_e = std::min(c.min_e, k.min_easting());
                c.max_e = std::max(c.max_e, k.min_easting() + k.size_m());
                c.min_n = std::min(c.min_n, k.min_northing());
                c.max_n = std::max(c.max_n, k.min_northing() + k.size_m());
            }
            cluster_list.push_back(c);
        }
        std::sort(cluster_list.begin(), cluster_list.end(),
                  [](const Cluster& a, const Cluster& b) {
                      if (a.min_n != b.min_n) return a.min_n < b.min_n;
                      return a.min_e < b.min_e;
                  });

        if (cluster_list.size() != out_paths.size()) {
            std::cerr << "g2m_fixture_cut cut-worldcover: resolved region has " << cluster_list.size()
                      << " tile cluster(s) but " << out_paths.size() << " --out path(s) were given\n";
            return 1;
        }

        auto src_source = std::make_shared<g2m::decode::FileByteSource>(*in_path);
        g2m::decode::tiff::TiffReader src_reader(src_source);
        if (src_reader.ifds().empty()) {
            std::cerr << "g2m_fixture_cut cut-worldcover: " << *in_path << ": no IFD\n";
            return 1;
        }
        const auto& src_ifd = src_reader.ifds()[0];
        if (src_ifd.sample_format != g2m::decode::tiff::SampleFormat::UnsignedInt || src_ifd.bits_per_sample != 8 ||
            src_ifd.samples_per_pixel != 1) {
            std::cerr << "g2m_fixture_cut cut-worldcover: " << *in_path << ": IFD0 must be single-band uint8\n";
            return 1;
        }
        if (!src_ifd.geo_transform) {
            std::cerr << "g2m_fixture_cut cut-worldcover: " << *in_path << ": missing GeoTIFF georeference\n";
            return 1;
        }
        const double sx = src_ifd.geo_transform->pixel_width;
        if (!(sx > 0.0) || sx != src_ifd.geo_transform->pixel_height) {
            std::cerr << "g2m_fixture_cut cut-worldcover: " << *in_path << ": ModelPixelScale must be square\n";
            return 1;
        }
        const auto n = static_cast<std::int64_t>(std::floor(1.0 / sx + 0.5));
        if (n <= 0 || 1.0 / static_cast<double>(n) != sx) {
            std::cerr << "g2m_fixture_cut cut-worldcover: " << *in_path
                      << ": ModelPixelScale is not an exact 1/N degree\n";
            return 1;
        }
        const auto c0 = static_cast<std::int64_t>(src_ifd.geo_transform->corner_x * static_cast<double>(n));
        const auto c1 = static_cast<std::int64_t>(src_ifd.geo_transform->corner_y * static_cast<double>(n));
        const auto src_width = static_cast<std::int64_t>(src_ifd.width);
        const auto src_height = static_cast<std::int64_t>(src_ifd.height);

        const g2m::geo::Utm utm;
        for (std::size_t ci = 0; ci < cluster_list.size(); ++ci) {
            const Cluster& c = cluster_list[ci];
            double min_lon = 180.0, max_lon = -180.0, min_lat = 90.0, max_lat = -90.0;
            for (double e : {static_cast<double>(c.min_e), static_cast<double>(c.max_e)}) {
                for (double nn : {static_cast<double>(c.min_n), static_cast<double>(c.max_n)}) {
                    const auto rev = utm.reverse(c.zone, e, nn);
                    min_lon = std::min(min_lon, rev.lon);
                    max_lon = std::max(max_lon, rev.lon);
                    min_lat = std::min(min_lat, rev.lat);
                    max_lat = std::max(max_lat, rev.lat);
                }
            }
            std::int64_t col_lo = static_cast<std::int64_t>(std::floor(min_lon * static_cast<double>(n))) - c0 - margin_px;
            std::int64_t col_hi = static_cast<std::int64_t>(std::floor(max_lon * static_cast<double>(n))) - c0 + margin_px;
            std::int64_t row_lo = c1 - 1 - static_cast<std::int64_t>(std::floor(max_lat * static_cast<double>(n))) - margin_px;
            std::int64_t row_hi = c1 - 1 - static_cast<std::int64_t>(std::floor(min_lat * static_cast<double>(n))) + margin_px;
            // Widen (never narrow - align DOWN only) col_lo/row_lo to the
            // nearest multiple of kTiepointAlignment so the written tiepoint
            // round-trips bit-exactly (see align_down_multiple's comment).
            col_lo = align_down_multiple(col_lo, kTiepointAlignment);
            row_lo = align_down_multiple(row_lo, kTiepointAlignment);
            col_lo = std::max<std::int64_t>(col_lo, 0);
            row_lo = std::max<std::int64_t>(row_lo, 0);
            col_hi = std::min<std::int64_t>(col_hi, src_width - 1);
            row_hi = std::min<std::int64_t>(row_hi, src_height - 1);
            if (col_hi < col_lo || row_hi < row_lo) {
                std::cerr << "g2m_fixture_cut cut-worldcover: cluster " << ci << " lies entirely outside " << *in_path
                          << "\n";
                return 1;
            }
            const int win_w = static_cast<int>(col_hi - col_lo + 1);
            const int win_h = static_cast<int>(row_hi - row_lo + 1);

            const auto window = src_reader.decode_window_u8(0, col_lo, row_lo, win_w, win_h);

            const std::int64_t new_c0 = col_lo + c0;
            const std::int64_t new_c1 = c1 - row_lo;
            const double new_tiepoint_lon = static_cast<double>(new_c0) / static_cast<double>(n);
            const double new_tiepoint_lat = static_cast<double>(new_c1) / static_cast<double>(n);

            g2m::test_support::TiffBuilder b;
            b.add_short(256, static_cast<std::uint16_t>(win_w));  // ImageWidth
            b.add_short(257, static_cast<std::uint16_t>(win_h));  // ImageLength
            b.add_short(258, 8);                                  // BitsPerSample
            b.add_short(259, 1);                                  // Compression: none
            b.add_short(262, 1);                                  // PhotometricInterpretation: BlackIsZero
            b.add_short(277, 1);                                  // SamplesPerPixel
            b.add_long(278, static_cast<std::uint32_t>(win_h));   // RowsPerStrip: one strip
            b.add_short(284, 1);                                  // PlanarConfiguration: chunky
            b.add_short(339, 1);                                  // SampleFormat: unsigned int
            b.add_double_array(33550, {sx, sx, 0.0});             // ModelPixelScaleTag
            b.add_double_array(33922, {0.0, 0.0, 0.0, new_tiepoint_lon, new_tiepoint_lat, 0.0}); // ModelTiepointTag
            // GeoKeyDirectory: header{1,1,0,2} + RasterTypeGeoKey=1 (Area) +
            // GeographicTypeGeoKey=4326 (WGS84) - the only two keys
            // tiff_reader.cpp's second pass consumes (see this file's own
            // header comment).
            b.add_short_array(34735, {1, 1, 0, 2, 1025, 0, 1, 1, 2048, 0, 1, 4326});
            b.add_ascii(42113, "0"); // GDAL_NODATA

            std::vector<std::byte> strip(static_cast<std::size_t>(win_w) * static_cast<std::size_t>(win_h));
            for (int r = 0; r < win_h; ++r)
                for (int col = 0; col < win_w; ++col)
                    strip[static_cast<std::size_t>(r) * win_w + col] = static_cast<std::byte>(window.at(col, r));

            const auto bytes = b.build(strip);
            std::ofstream out(out_paths[ci], std::ios::binary);
            if (!out) {
                std::cerr << "g2m_fixture_cut cut-worldcover: cannot write " << out_paths[ci] << "\n";
                return 1;
            }
            out.write(reinterpret_cast<const char*>(bytes.data()), static_cast<std::streamsize>(bytes.size()));
            out.close();
            std::cerr << "g2m_fixture_cut cut-worldcover: cluster " << ci << " (" << c.tile_count << " tile(s)) -> "
                      << out_paths[ci] << ": window [" << col_lo << ".." << col_hi << "] x [" << row_lo << ".."
                      << row_hi << "] = " << win_w << "x" << win_h << " (" << bytes.size() << " bytes), lon ["
                      << min_lon << ".." << max_lon << "] lat [" << min_lat << ".." << max_lat << "]\n";
        }
        return 0;
    } catch (const DecodeError& e) {
        std::cerr << "g2m_fixture_cut cut-worldcover: " << e.what() << "\n";
        return 1;
    } catch (const std::exception& e) {
        std::cerr << "g2m_fixture_cut cut-worldcover: " << e.what() << "\n";
        return 1;
    }
}

// write-unsorted-nodes (PLAN.md G1c I4): a tiny synthetic PBF whose node
// ids are NOT strictly increasing (20, then 10, then 30 - a single
// out-of-order pair is enough), for import::extract_region_set's own
// "PBF not sorted" rejection test (test_import_osm.cpp) - a case
// ordinary real-world extracts never exercise (osmium/every real PBF
// writer emits nodes in ascending id order), so it has to be hand-built
// rather than cut from real data. Coordinates are arbitrary but inside
// Hessen's utm32n zone (matching testdata/fixtures/regions.json's
// "fixture" region) so a project_e7_to_mm call on them never returns
// nullopt ("far").
int cmd_write_unsorted_nodes(const Args& args) {
    const auto out_path = args.get("out");
    if (!out_path) {
        std::cerr << "g2m_fixture_cut write-unsorted-nodes: --out is required\n";
        return 1;
    }
    try {
        osmium::io::Header header;
        header.set("generator", "g2m_fixture_cut");

        osmium::io::File out_file{*out_path};
        out_file.set("add_metadata", false);
        out_file.set("pbf_dense_nodes", true);
        out_file.set("pbf_compression", "zlib");

        osmium::io::Writer writer{out_file, header, osmium::io::overwrite::allow};
        osmium::memory::Buffer buffer{1024 * 1024, osmium::memory::Buffer::auto_grow::yes};

        const struct {
            std::int64_t id;
            std::int32_t lon_1e7, lat_1e7;
        } nodes[] = {
            {20, 86500000, 501800000}, // 8.65E, 50.18N
            {10, 86510000, 501810000}, // out of order: id 10 after id 20
            {30, 86520000, 501820000},
        };
        for (const auto& n : nodes) {
            {
                osmium::builder::NodeBuilder builder{buffer};
                builder.set_id(n.id);
                builder.set_location(osmium::Location{n.lon_1e7, n.lat_1e7});
            }
            buffer.commit();
        }
        writer(std::move(buffer));
        writer.close();

        std::cerr << "g2m_fixture_cut write-unsorted-nodes: wrote 3 nodes (ids 20,10,30) to " << *out_path << "\n";
        return 0;
    } catch (const DecodeError& e) {
        std::cerr << "g2m_fixture_cut write-unsorted-nodes: " << e.what() << "\n";
        return 1;
    } catch (const std::exception& e) {
        std::cerr << "g2m_fixture_cut write-unsorted-nodes: " << e.what() << "\n";
        return 1;
    }
}

int usage() {
    std::cerr << "usage: g2m_fixture_cut <command> [options]\n"
                 "commands:\n"
                 "  cut-pbf --in PATH --relation ID [--relation ID...] [--bbox lon0,lat0,lon1,lat1] --out PATH\n"
                 "  repack-dgm1 --out PATH --inner OUTER_ZIP#SUBSTRING [--inner ...] --key E,N [--key ...]\n"
                 "  cut-worldcover --in PATH --resolved PATH [--margin-px N] --out PATH [--out PATH ...]\n"
                 "                 (one --out per connected tile cluster, in ascending (northing,easting) order)\n"
                 "  write-unsorted-nodes --out PATH\n";
    return 1;
}

} // namespace

int main(int argc, char** argv) {
    if (argc < 2) return usage();
    const std::string command = argv[1];
    const Args args = parse_args(argc, argv, 2);
    if (command == "cut-pbf") return cmd_cut_pbf(args);
    if (command == "repack-dgm1") return cmd_repack_dgm1(args);
    if (command == "cut-worldcover") return cmd_cut_worldcover(args);
    if (command == "write-unsorted-nodes") return cmd_write_unsorted_nodes(args);
    std::cerr << "g2m_fixture_cut: unknown command '" << command << "'\n";
    return usage();
}
