// tools/osm_tag_probe/scan.cpp - see scan.h.
#include "scan.h"

#include "g2m/core/geo/utm.h"
#include "g2m/core/geom/exact.h"
#include "g2m/core/geom/project.h"
#include "g2m/core/geom/ring_assembly.h"
#include "g2m/core/geom/ring_set.h"
#include "g2m/core/hash.h"
#include "g2m/core/tile_key.h"
#include "g2m/decode/osm/pbf_reader.h"
#include "g2m/layer/layer_id.h"
#include "g2m/layer/release.h"

#include <algorithm>
#include <cmath>
#include <exception>
#include <optional>
#include <span>
#include <string_view>
#include <utility>
#include <vector>

namespace g2m::tag_probe {
namespace {

using namespace g2m::decode::osm; // OsmNode, OsmWay, OsmRelation, OsmMemberType, PbfHandler, PbfEntities, read_pbf
using geom::PointMm;

constexpr std::int64_t kTileSizeMm = (TileKey::kLevel0SizeM << kRawTileLevel) * 1000;

// ---- region membership -------------------------------------------------

class RegionMask {
public:
    RegionMask(const ResolvedRegion& region, geo::UtmZone zone) : region_(region), zone_(zone) {
        for (const TileKey& k : tile_keys(region)) tiles_.push_back(k.packed());
        for (const TileKey& k : interior_tile_keys(region)) interior_.push_back(k.packed());
        std::sort(tiles_.begin(), tiles_.end());
        std::sort(interior_.begin(), interior_.end());
    }

    bool contains(PointMm p) const {
        const std::int64_t ix = floor_div(p.x, kTileSizeMm);
        const std::int64_t iy = floor_div(p.y, kTileSizeMm);
        if (ix < TileKey::kIndexMin || ix > TileKey::kIndexMax || iy < TileKey::kIndexMin || iy > TileKey::kIndexMax)
            return false;
        const TileKey key{zone_, kRawTileLevel, static_cast<std::int32_t>(ix), static_cast<std::int32_t>(iy)};
        if (!key.valid()) return false;
        const std::uint64_t packed = key.packed();
        if (!std::binary_search(tiles_.begin(), tiles_.end(), packed)) return false;
        if (std::binary_search(interior_.begin(), interior_.end(), packed)) return true;
        for (const ResolvedPart& part : region_.parts) {
            const geom::BoxMm& b = part.rings.bbox;
            if (p.x < b.w || p.x > b.e || p.y < b.s || p.y > b.n) continue;
            if (geom::contains(part.rings, p)) return true;
        }
        return false;
    }

private:
    const ResolvedRegion& region_;
    geo::UtmZone zone_;
    std::vector<std::uint64_t> tiles_;
    std::vector<std::uint64_t> interior_;
};

// ---- node index ----------------------------------------------------------

struct NodeRec {
    std::int64_t id = 0;
    std::int32_t lat = 0, lon = 0; // 1e-7 degree
};

const NodeRec* find_node(const std::vector<NodeRec>& nodes, std::int64_t id) {
    const auto it = std::lower_bound(nodes.begin(), nodes.end(), id, [](const NodeRec& r, std::int64_t v) { return r.id < v; });
    return it != nodes.end() && it->id == id ? &*it : nullptr;
}

// A tagged node whose usage by carriageway ways the way pass measures.
struct NodeInfo {
    std::int64_t id = 0;
    std::uint8_t mask = 0;
    unsigned degree = 0;
};

// ---- geometry helpers ---------------------------------------------------

std::int64_t round_to_i64(double v) { return static_cast<std::int64_t>(std::floor(v + 0.5)); }

// Shoelace area of one implicitly closed ring in mm^2 (deltas from the first
// vertex, so every product is exact in a double for polygons under ~90 km).
double ring_area_mm2(std::span<const PointMm> ring) {
    if (ring.size() < 3) return 0.0;
    double twice = 0.0;
    for (std::size_t i = 1; i + 1 < ring.size(); ++i) {
        const double ax = static_cast<double>(ring[i].x - ring[0].x);
        const double ay = static_cast<double>(ring[i].y - ring[0].y);
        const double bx = static_cast<double>(ring[i + 1].x - ring[0].x);
        const double by = static_cast<double>(ring[i + 1].y - ring[0].y);
        twice += ax * by - ay * bx;
    }
    return std::fabs(twice) / 2.0;
}

std::int64_t mm2_to_m2(double mm2) { return round_to_i64(mm2 / 1.0e6); }

// ---- pass 1: relations -------------------------------------------------

struct RelRec {
    std::int64_t id = 0;
    std::vector<Tag> tags;
    std::vector<std::pair<std::int64_t, std::string>> way_members; // (way id, role)
};

class RelationPass final : public PbfHandler {
public:
    RelationPass(std::vector<RelRec>& out, ScanStats& stats) : out_(out), stats_(stats) {}
    void on_relation(const OsmRelation& r) override {
        ++stats_.relations_total;
        if (!Accumulator::relation_is_relevant(r.tags)) return;
        ++stats_.relations_relevant;
        RelRec rec;
        rec.id = r.id;
        rec.tags = r.tags;
        for (const OsmMember& m : r.members)
            if (m.type == OsmMemberType::Way) rec.way_members.emplace_back(m.ref, m.role);
        out_.push_back(std::move(rec));
    }

private:
    std::vector<RelRec>& out_;
    ScanStats& stats_;
};

// ---- pass 2: nodes -------------------------------------------------------

class NodePass final : public PbfHandler {
public:
    NodePass(std::vector<NodeRec>& nodes, std::vector<NodeInfo>& infos, const RegionMask& mask, const geo::Utm& utm,
             geo::UtmZone zone, Accumulator& acc, ScanStats& stats)
        : nodes_(nodes), infos_(infos), mask_(mask), utm_(utm), zone_(zone), acc_(acc), stats_(stats) {}

    void on_node(const OsmNode& n) override {
        ++stats_.nodes_total;
        if (!nodes_.empty() && n.id <= nodes_.back().id) sorted_ = false;
        nodes_.push_back({n.id, n.lat_1e7, n.lon_1e7});
        if (n.tags.empty()) return;
        const std::uint8_t kind = Accumulator::node_kind_mask(n.tags);
        if (kind == 0) return;
        const std::optional<PointMm> p = geom::project_e7_to_mm(utm_, zone_, n.lat_1e7, n.lon_1e7);
        if (!p || !mask_.contains(*p)) return;
        ++stats_.tagged_nodes_in_region;
        acc_.add_node(kind, n.tags);
        if ((kind & kNodeKindsWithPlacement) != 0) infos_.push_back({n.id, kind, 0});
    }
    bool sorted() const { return sorted_; }

private:
    std::vector<NodeRec>& nodes_;
    std::vector<NodeInfo>& infos_;
    const RegionMask& mask_;
    const geo::Utm& utm_;
    geo::UtmZone zone_;
    Accumulator& acc_;
    ScanStats& stats_;
    bool sorted_ = true;
};

// ---- pass 3: ways ---------------------------------------------------------

class WayPass final : public PbfHandler {
public:
    WayPass(const std::vector<NodeRec>& nodes, std::vector<NodeInfo>& infos, const std::vector<std::int64_t>& wanted,
            std::vector<std::vector<std::int64_t>>& wanted_refs, const RegionMask& mask, const geo::Utm& utm,
            geo::UtmZone zone, Accumulator& acc, ScanStats& stats)
        : nodes_(nodes), infos_(infos), wanted_(wanted), wanted_refs_(wanted_refs), mask_(mask), utm_(utm), zone_(zone),
          acc_(acc), stats_(stats) {}

    void on_way(const OsmWay& w) override {
        ++stats_.ways_total;
        if (!infos_.empty() && Accumulator::is_carriageway(w.tags)) count_degrees(w);
        const auto wit = std::lower_bound(wanted_.begin(), wanted_.end(), w.id);
        if (wit != wanted_.end() && *wit == w.id)
            wanted_refs_[static_cast<std::size_t>(wit - wanted_.begin())] = w.node_refs;

        if (w.node_refs.empty() || !Accumulator::way_is_relevant(w.tags)) return;
        ++stats_.ways_relevant;
        const NodeRec* anchor = find_node(nodes_, w.node_refs[w.node_refs.size() / 2]);
        if (anchor == nullptr) {
            ++stats_.ways_unresolved_anchor;
            return;
        }
        const std::optional<PointMm> ap = geom::project_e7_to_mm(utm_, zone_, anchor->lat, anchor->lon);
        if (!ap || !mask_.contains(*ap)) return;
        ++stats_.ways_in_region;

        pts_.clear();
        ok_.clear();
        bool missing = false;
        for (const std::int64_t ref : w.node_refs) {
            const NodeRec* r = find_node(nodes_, ref);
            std::optional<PointMm> p;
            if (r != nullptr) p = geom::project_e7_to_mm(utm_, zone_, r->lat, r->lon);
            if (!p) {
                missing = true;
                pts_.push_back({});
                ok_.push_back(0);
            } else {
                pts_.push_back(*p);
                ok_.push_back(1);
            }
        }
        if (missing) ++stats_.ways_missing_nodes;

        double len_mm = 0.0;
        for (std::size_t i = 0; i + 1 < pts_.size(); ++i) {
            if (ok_[i] == 0 || ok_[i + 1] == 0) continue;
            const double dx = static_cast<double>(pts_[i + 1].x - pts_[i].x);
            const double dy = static_cast<double>(pts_[i + 1].y - pts_[i].y);
            len_mm += std::sqrt(dx * dx + dy * dy);
        }
        const bool closed = !missing && w.node_refs.size() >= 4 && w.node_refs.front() == w.node_refs.back();
        std::int64_t area = 0;
        if (closed) area = mm2_to_m2(ring_area_mm2(std::span<const PointMm>(pts_.data(), pts_.size() - 1)));
        acc_.add_way(w.tags, round_to_i64(len_mm), area, closed);
    }

private:
    void count_degrees(const OsmWay& w) {
        const std::size_t last = w.node_refs.size() - 1;
        for (std::size_t i = 0; i < w.node_refs.size(); ++i) {
            const auto it = std::lower_bound(infos_.begin(), infos_.end(), w.node_refs[i],
                                             [](const NodeInfo& n, std::int64_t v) { return n.id < v; });
            if (it == infos_.end() || it->id != w.node_refs[i]) continue;
            it->degree += (i == 0 || i == last) ? 1U : 2U;
        }
    }

    const std::vector<NodeRec>& nodes_;
    std::vector<NodeInfo>& infos_;
    const std::vector<std::int64_t>& wanted_;
    std::vector<std::vector<std::int64_t>>& wanted_refs_;
    const RegionMask& mask_;
    const geo::Utm& utm_;
    geo::UtmZone zone_;
    Accumulator& acc_;
    ScanStats& stats_;
    std::vector<PointMm> pts_;
    std::vector<char> ok_;
};

// Role-based ring areas without the exact assembler's crossing checks: closed
// member ways are rings, open ones are chained end to end by shared node id
// (lowest way id first, deterministic); a chain that does not close is
// dropped. A ring counts +1 (or -1 when its first way has role "inner").
struct LenientArea {
    double area_mm2 = 0.0;
    std::optional<PointMm> anchor; // first vertex of the first ring
};

LenientArea lenient_area(std::vector<geom::MemberWay> members, const geom::NodePos& pos) {
    std::sort(members.begin(), members.end(),
              [](const geom::MemberWay& a, const geom::MemberWay& b) { return a.way_id < b.way_id; });
    std::vector<char> used(members.size(), 0);
    LenientArea out;
    const auto add_ring = [&](const std::vector<std::int64_t>& nodes, bool inner) {
        std::vector<PointMm> pts;
        for (std::size_t i = 0; i + 1 < nodes.size(); ++i) { // the closing node repeats the first
            const std::optional<PointMm> p = pos(nodes[i]);
            if (!p) return;
            pts.push_back(*p);
        }
        if (pts.size() < 3) return;
        if (!out.anchor) out.anchor = pts.front();
        const double a = ring_area_mm2(pts);
        out.area_mm2 += inner ? -a : a;
    };
    for (std::size_t i = 0; i < members.size(); ++i) {
        if (used[i] != 0) continue;
        const auto& n0 = members[i].nodes;
        used[i] = 1;
        if (n0.size() < 2) continue;
        std::vector<std::int64_t> chain(n0.begin(), n0.end());
        const bool inner = members[i].role == "inner";
        while (chain.front() != chain.back()) {
            bool extended = false;
            for (std::size_t j = 0; j < members.size() && !extended; ++j) {
                if (used[j] != 0 || members[j].nodes.size() < 2) continue;
                const auto& nj = members[j].nodes;
                if (nj.front() == chain.back()) {
                    chain.insert(chain.end(), nj.begin() + 1, nj.end());
                    extended = true;
                } else if (nj.back() == chain.back()) {
                    chain.insert(chain.end(), nj.rbegin() + 1, nj.rend());
                    extended = true;
                }
                if (extended) used[j] = 1;
            }
            if (!extended) break;
        }
        if (chain.front() == chain.back() && chain.size() >= 4) add_ring(chain, inner);
    }
    return out;
}

template <class Pass>
std::optional<Error> run_pass(const std::string& path, Pass& pass, PbfEntities entities) {
    try {
        read_pbf(path, pass, nullptr, entities);
    } catch (const std::exception& e) {
        return make_error(ErrorCode::Io, std::string("osm_tag_probe: reading ") + path + ": " + e.what());
    }
    return std::nullopt;
}

std::string hex16(std::uint64_t v) { return to_hex(v); }

} // namespace

Result<ScanStats> scan_pbf(const std::string& pbf_path, const ResolvedRegion& region, Accumulator& acc) {
    const std::optional<geo::UtmZone> zone = geo::parse_utm_scheme(region.scheme);
    if (!zone) return make_error(ErrorCode::InvalidArgument, "osm_tag_probe: unparsable region scheme '" + region.scheme + "'");
    const geo::Utm utm(geo::kGRS80);
    const RegionMask mask(region, *zone);
    ScanStats stats;

    // Pass 1: relations.
    std::vector<RelRec> rels;
    {
        RelationPass pass(rels, stats);
        if (auto err = run_pass(pbf_path, pass, PbfEntities::Relations)) return *err;
    }
    std::vector<std::int64_t> wanted;
    for (const RelRec& r : rels)
        for (const auto& m : r.way_members) wanted.push_back(m.first);
    std::sort(wanted.begin(), wanted.end());
    wanted.erase(std::unique(wanted.begin(), wanted.end()), wanted.end());
    std::vector<std::vector<std::int64_t>> wanted_refs(wanted.size());

    // Pass 2: nodes.
    std::vector<NodeRec> nodes;
    nodes.reserve(40'000'000);
    std::vector<NodeInfo> infos;
    {
        NodePass pass(nodes, infos, mask, utm, *zone, acc, stats);
        if (auto err = run_pass(pbf_path, pass, PbfEntities::Nodes)) return *err;
        if (!pass.sorted()) return make_error(ErrorCode::Malformed, "osm_tag_probe: PBF nodes are not sorted by id");
    }

    // Pass 3: ways.
    {
        WayPass pass(nodes, infos, wanted, wanted_refs, mask, utm, *zone, acc, stats);
        if (auto err = run_pass(pbf_path, pass, PbfEntities::Ways)) return *err;
    }
    for (const NodeInfo& n : infos) acc.add_node_degree(n.mask, n.degree);

    // Relations: assemble, place, aggregate.
    const geom::NodePos pos = [&](std::int64_t id) -> std::optional<PointMm> {
        const NodeRec* r = find_node(nodes, id);
        if (r == nullptr) return std::nullopt;
        return geom::project_e7_to_mm(utm, *zone, r->lat, r->lon);
    };
    for (const RelRec& rel : rels) {
        std::vector<geom::MemberWay> members;
        bool missing = false;
        for (const auto& m : rel.way_members) {
            const auto it = std::lower_bound(wanted.begin(), wanted.end(), m.first);
            const std::vector<std::int64_t>& refs = wanted_refs[static_cast<std::size_t>(it - wanted.begin())];
            if (refs.empty()) {
                missing = true;
                continue;
            }
            members.push_back({m.first, m.second, std::span<const std::int64_t>(refs)});
        }
        // Exact assembly first (the region resolver's own routine). It rejects
        // any crossing or overlapping ring, which hand-mapped forests with many
        // rings hit often; then fall back to the lenient role-based ring areas.
        bool lenient = false;
        double area_mm2 = 0.0;
        std::optional<PointMm> anchor;
        bool exact = false;
        if (!missing && !members.empty()) {
            Result<geom::AssembledArea> a = geom::assemble_rings(members, pos);
            if (a.ok() && !a->rings.outers.empty() && !a->rings.outers.front().empty()) {
                exact = true;
                anchor = a->rings.outers.front().front();
                for (const geom::Ring& r : a->rings.outers) area_mm2 += ring_area_mm2(r);
                for (const geom::Ring& r : a->rings.inners) area_mm2 -= ring_area_mm2(r);
            }
        }
        if (!exact) {
            lenient = true;
            const LenientArea la = lenient_area(members, pos);
            area_mm2 = la.area_mm2;
            anchor = la.anchor;
            if (!anchor) {
                for (const geom::MemberWay& m : members) {
                    if (m.nodes.empty()) continue;
                    anchor = pos(m.nodes.front());
                    if (anchor) break;
                }
            }
        }
        if (!anchor) {
            ++stats.relations_no_geometry;
            continue;
        }
        if (!mask.contains(*anchor)) continue;
        ++stats.relations_in_region;
        if (lenient) ++stats.relations_lenient;
        const std::int64_t area = mm2_to_m2(area_mm2 < 0.0 ? 0.0 : area_mm2);
        if (area == 0) ++stats.relations_zero_area;
        acc.add_relation_polygon(rel.tags, area);
    }
    return stats;
}

ReportMeta make_meta(const ResolvedRegion& region, const ScanStats& stats) {
    ReportMeta meta;
    meta.region_name = region.name;
    meta.pbf_name = region.pbf_name;
    meta.pbf_sha256_hex = to_hex(region.pbf_sha256);
    for (const ResolvedPart& p : region.parts) {
        ReportPart rp;
        rp.id = p.id;
        rp.name = p.kind == ResolvedPartKind::UtmBbox ? std::string("(utm bbox)") : p.name;
        rp.admin_level = p.admin_level.value_or(0);
        rp.geometry_hash_hex = hex16(p.geometry_hash);
        meta.parts.push_back(std::move(rp));
    }
    meta.scan = stats;
    return meta;
}

} // namespace g2m::tag_probe
