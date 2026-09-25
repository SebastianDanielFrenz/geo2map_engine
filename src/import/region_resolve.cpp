// src/import/region_resolve.cpp - see g2m/import/region_resolve.h.

#include "g2m/import/region_resolve.h"

#include "g2m/core/geom/project.h"
#include "g2m/core/geom/ring_assembly.h"
#include "g2m/core/hash.h"
#include "g2m/decode/byte_source.h"
#include "g2m/decode/osm/pbf_reader.h"
#include "g2m/layer/release.h"

#include <algorithm>
#include <cstdlib>
#include <filesystem>
#include <map>
#include <variant>

namespace g2m::import {
namespace {

using g2m::decode::DecodeError;
using g2m::decode::osm::OsmMember;
using g2m::decode::osm::OsmMemberType;
using g2m::decode::osm::OsmRelation;
using g2m::decode::osm::PbfEntities;
using g2m::decode::osm::PbfHandler;

// Pass 1: relations only - find one specific relation by id.
class RelationFinder final : public PbfHandler {
public:
    explicit RelationFinder(std::int64_t target_id) : target_id_(target_id) {}
    void on_relation(const OsmRelation& r) override {
        if (r.id == target_id_) found_ = r;
    }
    const std::optional<OsmRelation>& found() const { return found_; }

private:
    std::int64_t target_id_;
    std::optional<OsmRelation> found_;
};

// Pass 2: ways only - fetch node-ref lists for a fixed set of way ids.
class WayCollector final : public PbfHandler {
public:
    explicit WayCollector(std::vector<std::int64_t> ids) : ids_(std::move(ids)) {
        std::sort(ids_.begin(), ids_.end());
        ids_.erase(std::unique(ids_.begin(), ids_.end()), ids_.end());
    }
    void on_way(const g2m::decode::osm::OsmWay& w) override {
        if (std::binary_search(ids_.begin(), ids_.end(), w.id)) ways_[w.id] = w.node_refs;
    }
    const std::map<std::int64_t, std::vector<std::int64_t>>& ways() const { return ways_; }

private:
    std::vector<std::int64_t> ids_;
    std::map<std::int64_t, std::vector<std::int64_t>> ways_;
};

// Pass 3: nodes only - fetch positions for a fixed set of node ids.
class NodeCollector final : public PbfHandler {
public:
    explicit NodeCollector(std::vector<std::int64_t> ids) : ids_(std::move(ids)) {
        std::sort(ids_.begin(), ids_.end());
        ids_.erase(std::unique(ids_.begin(), ids_.end()), ids_.end());
    }
    void on_node(const g2m::decode::osm::OsmNode& n) override {
        if (std::binary_search(ids_.begin(), ids_.end(), n.id)) pos_[n.id] = {n.lat_1e7, n.lon_1e7};
    }
    const std::map<std::int64_t, std::pair<std::int32_t, std::int32_t>>& positions() const { return pos_; }

private:
    std::vector<std::int64_t> ids_;
    std::map<std::int64_t, std::pair<std::int32_t, std::int32_t>> pos_;
};

// Relations tagged boundary=administrative, optionally filtered (find_admin below).
class AdminScanner final : public PbfHandler {
public:
    AdminScanner(std::optional<std::string_view> name_contains, std::optional<int> admin_level)
        : name_contains_(name_contains), admin_level_(admin_level) {}

    void on_relation(const OsmRelation& r) override {
        std::optional<std::string> boundary;
        std::optional<std::string> name;
        std::optional<int> level;
        for (const auto& t : r.tags) {
            if (t.key == "boundary") boundary = t.value;
            else if (t.key == "name") name = t.value;
            else if (t.key == "admin_level") {
                char* end = nullptr;
                const long v = std::strtol(t.value.c_str(), &end, 10);
                if (end != t.value.c_str() && *end == '\0') level = static_cast<int>(v);
            }
        }
        if (!boundary || *boundary != "administrative") return;
        if (name_contains_ && (!name || name->find(*name_contains_) == std::string::npos)) return;
        if (admin_level_ && (!level || *level != *admin_level_)) return;
        AdminCandidate c;
        c.relation_id = r.id;
        c.name = name.value_or("");
        c.admin_level = level;
        out_.push_back(std::move(c));
    }
    std::vector<AdminCandidate> take() { return std::move(out_); }

private:
    std::optional<std::string_view> name_contains_;
    std::optional<int> admin_level_;
    std::vector<AdminCandidate> out_;
};

std::optional<int> parse_admin_level(std::string_view value) {
    // Best-effort: OSM admin_level values are meant to be plain small
    // integers, but the tag is free text in practice. A value that is not
    // exactly one (optionally signed) integer is left unset - it is only
    // ever used informationally (ResolvedPart::admin_level) or checked
    // against RegionSpec::expect_admin_level, never fed into geometry.
    if (value.empty()) return std::nullopt;
    std::string s(value);
    char* end = nullptr;
    const long v = std::strtol(s.c_str(), &end, 10);
    if (end == s.c_str() || *end != '\0') return std::nullopt;
    return static_cast<int>(v);
}

std::uint64_t geometry_hash_of(const geom::RingSet& rings) {
    Fnv1a64 h;
    const auto hash_ring_list = [&](const std::vector<geom::Ring>& list) {
        h.u64(list.size());
        for (const geom::Ring& r : list) {
            h.u64(r.size());
            for (const geom::PointMm& p : r) {
                h.i64(p.x);
                h.i64(p.y);
            }
        }
    };
    hash_ring_list(rings.outers);
    hash_ring_list(rings.inners);
    return h.value();
}

geom::RingSet ring_set_for_bbox(const UtmBboxPart& b) {
    geom::RingSet rs;
    geom::Ring outer{
        geom::PointMm{b.west * 1000, b.south * 1000},
        geom::PointMm{b.east * 1000, b.south * 1000},
        geom::PointMm{b.east * 1000, b.north * 1000},
        geom::PointMm{b.west * 1000, b.north * 1000},
    };
    rs.outers.push_back(std::move(outer));
    rs.bbox = geom::bounding_box(rs);
    return rs;
}

// Streams entire file through Sha256 without loading it into memory at once.
ContentHash sha256_of_file(const std::string& path) {
    g2m::decode::FileByteSource src(path); // throws DecodeError if it cannot be opened
    Sha256 sha;
    constexpr std::uint64_t kChunk = std::uint64_t{4} << 20; // 4 MiB
    const std::uint64_t total = src.size();
    std::vector<std::byte> bytes;
    for (std::uint64_t offset = 0; offset < total;) {
        const std::uint64_t len = std::min(kChunk, total - offset);
        // FileByteSource::read(offset, len, out) - the 3-arg override hides
        // IByteSource's 2-arg allocate-and-return convenience overload for a
        // FileByteSource-typed caller (no `using IByteSource::read;`), so
        // call the 3-arg form directly.
        src.read(offset, len, bytes);
        sha.update(bytes.data(), bytes.size());
        offset += len;
    }
    return sha.finish();
}

Result<ResolvedPart> resolve_osm_admin_part(const OsmAdminPart& part, const std::string& pbf_path,
                                             const geo::Utm& utm, geo::UtmZone zone) {
    RelationFinder finder(part.relation);
    try {
        g2m::decode::osm::read_pbf(pbf_path, finder, nullptr, PbfEntities::Relations);
    } catch (const DecodeError& e) {
        return make_error(ErrorCode::Io, std::string("region: reading ") + pbf_path + ": " + e.what());
    }
    if (!finder.found()) {
        return make_error(ErrorCode::NotFound, "region: osm_admin relation " + std::to_string(part.relation) +
                                                     " not found in " + pbf_path);
    }
    const OsmRelation& rel = *finder.found();

    std::vector<std::int64_t> way_ids;
    for (const OsmMember& m : rel.members) {
        if (m.type == OsmMemberType::Way && (m.role == "outer" || m.role == "inner")) way_ids.push_back(m.ref);
    }
    if (way_ids.empty()) {
        return make_error(ErrorCode::InvalidArgument, "region: osm_admin relation " + std::to_string(part.relation) +
                                                            " has no outer/inner way members");
    }

    WayCollector ways(way_ids);
    try {
        g2m::decode::osm::read_pbf(pbf_path, ways, nullptr, PbfEntities::Ways);
    } catch (const DecodeError& e) {
        return make_error(ErrorCode::Io, std::string("region: reading ") + pbf_path + ": " + e.what());
    }
    std::vector<std::int64_t> unique_way_ids = way_ids;
    std::sort(unique_way_ids.begin(), unique_way_ids.end());
    unique_way_ids.erase(std::unique(unique_way_ids.begin(), unique_way_ids.end()), unique_way_ids.end());
    for (std::int64_t id : unique_way_ids) {
        if (!ways.ways().count(id)) {
            return make_error(ErrorCode::NotFound, "region: way " + std::to_string(id) + " (member of relation " +
                                                         std::to_string(part.relation) + ") not found in " +
                                                         pbf_path);
        }
    }

    std::vector<std::int64_t> node_ids;
    for (std::int64_t id : unique_way_ids) {
        const auto& refs = ways.ways().at(id);
        node_ids.insert(node_ids.end(), refs.begin(), refs.end());
    }
    std::sort(node_ids.begin(), node_ids.end());
    node_ids.erase(std::unique(node_ids.begin(), node_ids.end()), node_ids.end());

    NodeCollector nodes(node_ids);
    try {
        g2m::decode::osm::read_pbf(pbf_path, nodes, nullptr, PbfEntities::Nodes);
    } catch (const DecodeError& e) {
        return make_error(ErrorCode::Io, std::string("region: reading ") + pbf_path + ": " + e.what());
    }

    std::vector<geom::MemberWay> members;
    members.reserve(way_ids.size());
    for (const OsmMember& m : rel.members) {
        if (m.type != OsmMemberType::Way) continue;
        if (m.role != "outer" && m.role != "inner") continue;
        const auto it = ways.ways().find(m.ref);
        if (it == ways.ways().end()) continue; // already verified present above
        members.push_back(geom::MemberWay{m.ref, std::string_view(m.role), std::span<const std::int64_t>(it->second)});
    }

    const auto& positions = nodes.positions();
    const geom::NodePos pos_fn = [&](std::int64_t id) -> std::optional<geom::PointMm> {
        const auto it = positions.find(id);
        if (it == positions.end()) return std::nullopt;
        return geom::project_e7_to_mm(utm, zone, it->second.first, it->second.second);
    };

    Result<geom::AssembledArea> assembled = geom::assemble_rings(std::move(members), pos_fn);
    if (!assembled) {
        return make_error(assembled.error().code,
                          "region: relation " + std::to_string(part.relation) + ": " + assembled.error().message);
    }

    ResolvedPart out;
    out.kind = ResolvedPartKind::OsmAdmin;
    out.id = part.relation;
    for (const auto& t : rel.tags) {
        if (t.key == "name") out.name = t.value;
    }
    std::optional<int> admin_level;
    for (const auto& t : rel.tags) {
        if (t.key == "admin_level") admin_level = parse_admin_level(t.value);
    }
    out.admin_level = admin_level;
    out.rings = assembled->rings;
    out.vertices = assembled->vertex_count;
    out.geometry_hash = geometry_hash_of(out.rings);

    if (part.expect_name && *part.expect_name != out.name) {
        return make_error(ErrorCode::InvalidArgument, "region: relation " + std::to_string(part.relation) +
                                                            ": expected name '" + *part.expect_name + "', got '" +
                                                            out.name + "'");
    }
    if (part.expect_admin_level) {
        if (!out.admin_level || *out.admin_level != *part.expect_admin_level) {
            return make_error(ErrorCode::InvalidArgument,
                              "region: relation " + std::to_string(part.relation) + ": expected admin_level " +
                                  std::to_string(*part.expect_admin_level) + ", got " +
                                  (out.admin_level ? std::to_string(*out.admin_level) : std::string("none")));
        }
    }
    return out;
}

} // namespace

Result<ResolvedRegion> resolve_region(const RegionSpec& spec, const std::string& pbf_path) {
    const std::optional<geo::UtmZone> zone_opt = geo::parse_utm_scheme(spec.scheme);
    if (!zone_opt) return make_error(ErrorCode::InvalidArgument, "region: scheme '" + spec.scheme + "'");
    const geo::UtmZone zone = *zone_opt;
    const geo::Utm utm;

    ResolvedRegion out;
    out.name = spec.name;
    out.scheme = spec.scheme;
    out.level = 2;
    out.halo_m = spec.halo_m;
    out.pbf_name = std::filesystem::path(pbf_path).filename().string();
    try {
        out.pbf_sha256 = sha256_of_file(pbf_path);
    } catch (const DecodeError& e) {
        return make_error(ErrorCode::Io, std::string("region: ") + e.what());
    }

    std::vector<TileKey> footprint_keys;
    std::vector<TileKey> interior_keys;
    for (const RegionPart& part : spec.parts) {
        ResolvedPart rp;
        geom::RingSet rings;
        if (const auto* bbox = std::get_if<UtmBboxPart>(&part)) {
            rings = ring_set_for_bbox(*bbox);
            rp.kind = ResolvedPartKind::UtmBbox;
            rp.id = 0;
            rp.name.clear();
            rp.admin_level.reset();
            rp.rings = rings;
            rp.vertices = rings.outers.empty() ? 0 : rings.outers.front().size();
            rp.geometry_hash = geometry_hash_of(rings);
        } else {
            const auto& admin = std::get<OsmAdminPart>(part);
            Result<ResolvedPart> resolved = resolve_osm_admin_part(admin, pbf_path, utm, zone);
            if (!resolved) return resolved.error();
            rp = std::move(resolved.value());
            rings = rp.rings;
        }

        const auto relations = geom::tiles_near(rings, zone, /*level=*/2, spec.halo_m);
        for (const auto& [key, relation] : relations) {
            footprint_keys.push_back(key);
            if (relation == geom::TileRelation::Interior) interior_keys.push_back(key);
        }
        out.parts.push_back(std::move(rp));
    }

    const Coverage footprint_cov = Coverage::from_level2_tiles(footprint_keys);
    out.tiles = footprint_cov.zones.empty() ? std::vector<CoverageRun>{} : footprint_cov.zones.front().runs;
    const Coverage interior_cov = Coverage::from_level2_tiles(interior_keys);
    out.interior = interior_cov.zones.empty() ? std::vector<CoverageRun>{} : interior_cov.zones.front().runs;

    std::uint64_t tile_count = 0;
    for (const CoverageRun& run : out.tiles) tile_count += static_cast<std::uint64_t>(run.x1 - run.x0 + 1);
    out.tile_count = tile_count;

    return out;
}

std::vector<AdminCandidate> find_admin(const std::string& pbf_path, std::optional<std::string_view> name_contains,
                                        std::optional<int> admin_level) {
    AdminScanner scanner(name_contains, admin_level);
    g2m::decode::osm::read_pbf(pbf_path, scanner, nullptr, PbfEntities::Relations);
    std::vector<AdminCandidate> out = scanner.take();
    std::sort(out.begin(), out.end(),
              [](const AdminCandidate& a, const AdminCandidate& b) { return a.relation_id < b.relation_id; });
    return out;
}

} // namespace g2m::import
