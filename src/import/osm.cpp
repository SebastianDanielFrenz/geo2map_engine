// src/import/osm.cpp - see g2m/import/osm.h.
//
// extract_region_set() reads `pbf` in four passes, each restricted (via
// decode::osm::PbfEntities) to the one entity type it needs - I2's own
// region_resolve.h convention, applied here to build S_region instead of
// resolving a single admin relation:
//  N1 (Nodes only): every node's raw (id, lat, lon) into a 16 B/node SoA,
//     sorted by construction (ids must already be strictly increasing in
//     the PBF itself - checked as they stream in, else Malformed "PBF not
//     sorted"). Also computes node_in_ids: nodes whose projected position
//     lies in ANY of the region's own level-2 tiles, dilated by
//     kDefaultOsmHaloM - the same per-tile test osm_clip.cpp's clip_osm()
//     and osm_clip_index.cpp's OsmClipIndex::build() use, just evaluated
//     against the whole region's tile SET instead of one query tile.
//  W (Ways only): a way is kept if any ref is node_in_ids, or any
//     consecutive segment touches some region tile's dilated box (endpoints
//     re-projected on the fly from the N1 SoA - clip_osm's own way-
//     inclusion test, region-wide). A kept way must have every ref resolve
//     to a real node somewhere in the PBF (not necessarily node_in_ids -
//     just present at all); one that doesn't is dropped and counted in
//     ways_incomplete (a real data inconsistency, not a bug in this code).
//     Every ref of a kept way is added to keep_node_ids (node_in_ids plus
//     way-ref completions - clip_osm's own node_keep, region-wide).
//  R (Relations only, no bbox - relations aren't geometric): every
//     relation, closed transitively exactly like clip_osm's own step 4, but
//     seeded from the REGION-WIDE keep_node_ids/way ids above instead of
//     one tile's. This can keep a few more relations than any single tile's
//     own clip_osm(S_full, k) would need, which is fine: clip_osm(S_region,
//     k)'s reachability computation only marks a relation "in" for tile k
//     if it is actually reachable from k's own node_keep(k)/way_in(k) - an
//     "extra" relation present only for some OTHER tile's benefit is never
//     spuriously reachable from k, so the S_region equivalence contract
//     (osm.h's own doc comment) still holds exactly.
//  N2 (Nodes only, again): full tags for every id in keep_node_ids - a
//     second pass because N1 deliberately keeps only 16 B/node, not tags,
//     to bound memory over a country-scale file.
// Assembled via osm::OsmBuilder, which itself re-validates canonical form.
#include "g2m/import/osm.h"

#include "g2m/core/geom/exact.h"
#include "g2m/core/geom/project.h"
#include "g2m/decode/osm/pbf_reader.h"
#include "g2m/import/ordered_parallel.h"
#include "g2m/layer/layer_id.h"
#include "g2m/layer/osm_clip.h"
#include "g2m/layer/src_osm.h"
#include "g2m/layer/tile_container.h"

#include <algorithm>
#include <chrono>
#include <stdexcept>
#include <thread>
#include <unordered_map>
#include <unordered_set>
#include <utility>

namespace g2m::import {
namespace {

using namespace g2m::decode::osm; // Tag, OsmNode, OsmWay, OsmMember(Type), OsmRelation, PbfHandler, PbfEntities, read_pbf
using Pt = geom::PointMm;

// Same 1024 m bucket grid osm_clip_index.cpp uses (kBucketSizeMm/candidate_
// range/index_in_range - duplicated here rather than shared, since those are
// file-local helpers in a different translation unit serving a related but
// distinct purpose: "which region tiles could this touch" here, vs. "which
// ONE query tile" there).
constexpr std::int64_t kBucketSizeMm = (TileKey::kLevel0SizeM << kRawTileLevel) * 1000;
constexpr std::int64_t ceil_div(std::int64_t a, std::int64_t b) { return -floor_div(-a, b); }

struct AxisRange {
    std::int64_t lo, hi;
};

AxisRange candidate_range(std::int64_t c, std::int64_t halo_mm, std::int64_t size_mm) {
    return {ceil_div(c - halo_mm, size_mm) - 1, floor_div(c + halo_mm, size_mm)};
}

bool index_in_range(std::int64_t ix) { return ix >= TileKey::kIndexMin && ix <= TileKey::kIndexMax; }

// Is `p` within halo_mm of ANY tile in `region_tiles` (packed TileKeys)?
bool point_touches_region(Pt p, std::int64_t halo_mm, std::int64_t size_mm,
                           const std::unordered_set<std::uint64_t>& region_tiles, geo::UtmZone zone) {
    const AxisRange rx = candidate_range(p.x, halo_mm, size_mm);
    const AxisRange ry = candidate_range(p.y, halo_mm, size_mm);
    for (std::int64_t ix = rx.lo; ix <= rx.hi; ++ix) {
        if (!index_in_range(ix)) continue;
        for (std::int64_t iy = ry.lo; iy <= ry.hi; ++iy) {
            if (!index_in_range(iy)) continue;
            const TileKey key{zone, kRawTileLevel, static_cast<std::int32_t>(ix), static_cast<std::int32_t>(iy)};
            if (!key.valid()) continue;
            if (region_tiles.count(key.packed()) != 0) return true;
        }
    }
    return false;
}

// Does segment a-b touch ANY tile in `region_tiles`, dilated by halo_mm?
bool segment_touches_region(Pt a, Pt b, std::int64_t halo_mm, std::int64_t size_mm,
                             const std::unordered_set<std::uint64_t>& region_tiles, geo::UtmZone zone) {
    const AxisRange rxa = candidate_range(a.x, halo_mm, size_mm);
    const AxisRange rxb = candidate_range(b.x, halo_mm, size_mm);
    const AxisRange rya = candidate_range(a.y, halo_mm, size_mm);
    const AxisRange ryb = candidate_range(b.y, halo_mm, size_mm);
    const std::int64_t ix_lo = std::min(rxa.lo, rxb.lo);
    const std::int64_t ix_hi = std::max(rxa.hi, rxb.hi);
    const std::int64_t iy_lo = std::min(rya.lo, ryb.lo);
    const std::int64_t iy_hi = std::max(rya.hi, ryb.hi);
    for (std::int64_t ix = ix_lo; ix <= ix_hi; ++ix) {
        if (!index_in_range(ix)) continue;
        for (std::int64_t iy = iy_lo; iy <= iy_hi; ++iy) {
            if (!index_in_range(iy)) continue;
            const TileKey key{zone, kRawTileLevel, static_cast<std::int32_t>(ix), static_cast<std::int32_t>(iy)};
            if (!key.valid()) continue;
            if (region_tiles.count(key.packed()) == 0) continue;
            const std::int64_t x0 = ix * size_mm - halo_mm;
            const std::int64_t y0 = iy * size_mm - halo_mm;
            const std::int64_t x1 = ix * size_mm + size_mm + halo_mm;
            const std::int64_t y1 = iy * size_mm + size_mm + halo_mm;
            if (geom::segment_touches_box(a.x, a.y, b.x, b.y, x0, y0, x1, y1)) return true;
        }
    }
    return false;
}

// N1's 16 B/node SoA record.
struct NodeRec {
    std::int64_t id = 0;
    std::int32_t lat = 0, lon = 0; // 1e-7 degree, same convention as osm::Node
};

const NodeRec* find_node_rec(const std::vector<NodeRec>& nodes, std::int64_t id) {
    const auto it = std::lower_bound(nodes.begin(), nodes.end(), id,
                                      [](const NodeRec& r, std::int64_t v) { return r.id < v; });
    if (it != nodes.end() && it->id == id) return &*it;
    return nullptr;
}

// NOTE: an ordering violation is reported via sorted(), not an exception -
// g2m::decode::osm::read_pbf's own documented contract is to throw ONLY
// DecodeError (it wraps any other exception type a handler throws, losing
// the original type - see its own outer try/catch), so a distinct
// "not sorted" exception type thrown from on_node would arrive at
// extract_region_set's own catch site already turned into a generic
// DecodeError/std::exception, indistinguishable from a real I/O failure.
class NodeScanHandler final : public PbfHandler {
public:
    explicit NodeScanHandler(std::vector<NodeRec>& out) : out_(out) {}
    void on_node(const OsmNode& n) override {
        if (!sorted_) return; // already broken; stop accumulating, just let the read finish
        if (!out_.empty() && n.id <= out_.back().id) {
            sorted_ = false;
            return;
        }
        out_.push_back(NodeRec{n.id, n.lat_1e7, n.lon_1e7});
    }
    bool sorted() const { return sorted_; }

private:
    std::vector<NodeRec>& out_;
    bool sorted_ = true;
};

struct WayRec {
    std::int64_t id = 0;
    std::vector<std::int64_t> refs;
    osm::OsmBuilder::TagList tags;
};

class WayScanHandler final : public PbfHandler {
public:
    WayScanHandler(const std::vector<NodeRec>& nodes, const std::unordered_set<std::int64_t>& node_in_ids,
                    const std::unordered_set<std::uint64_t>& region_tiles, std::int64_t halo_mm, std::int64_t size_mm,
                    geo::UtmZone zone, const geo::Utm& utm, std::vector<WayRec>& ways_in,
                    std::unordered_set<std::int64_t>& keep_node_ids, std::uint64_t& ways_incomplete)
        : nodes_(nodes), node_in_ids_(node_in_ids), region_tiles_(region_tiles), halo_mm_(halo_mm), size_mm_(size_mm),
          zone_(zone), utm_(utm), ways_in_(ways_in), keep_node_ids_(keep_node_ids),
          ways_incomplete_(ways_incomplete) {}

    void on_way(const OsmWay& w) override {
        if (w.node_refs.empty()) return; // no valid geometry; never includable

        bool hit = false;
        for (const std::int64_t ref : w.node_refs) {
            if (node_in_ids_.count(ref) != 0) {
                hit = true;
                break;
            }
        }
        if (!hit) {
            for (std::size_t k = 0; k + 1 < w.node_refs.size(); ++k) {
                const NodeRec* a = find_node_rec(nodes_, w.node_refs[k]);
                const NodeRec* b = find_node_rec(nodes_, w.node_refs[k + 1]);
                if (a == nullptr || b == nullptr) continue;
                const std::optional<Pt> pa = geom::project_e7_to_mm(utm_, zone_, a->lat, a->lon);
                const std::optional<Pt> pb = geom::project_e7_to_mm(utm_, zone_, b->lat, b->lon);
                if (!pa || !pb) continue;
                if (segment_touches_region(*pa, *pb, halo_mm_, size_mm_, region_tiles_, zone_)) {
                    hit = true;
                    break;
                }
            }
        }
        if (!hit) return;

        bool complete = true;
        for (const std::int64_t ref : w.node_refs) {
            if (find_node_rec(nodes_, ref) == nullptr) {
                complete = false;
                break;
            }
        }
        if (!complete) {
            ++ways_incomplete_;
            return;
        }

        osm::OsmBuilder::TagList tags;
        tags.reserve(w.tags.size());
        for (const Tag& t : w.tags) tags.emplace_back(t.key, t.value);
        ways_in_.push_back(WayRec{w.id, w.node_refs, std::move(tags)});
        for (const std::int64_t ref : w.node_refs) keep_node_ids_.insert(ref);
    }

private:
    const std::vector<NodeRec>& nodes_;
    const std::unordered_set<std::int64_t>& node_in_ids_;
    const std::unordered_set<std::uint64_t>& region_tiles_;
    std::int64_t halo_mm_, size_mm_;
    geo::UtmZone zone_;
    const geo::Utm& utm_;
    std::vector<WayRec>& ways_in_;
    std::unordered_set<std::int64_t>& keep_node_ids_;
    std::uint64_t& ways_incomplete_;
};

struct RelRec {
    std::int64_t id = 0;
    std::vector<osm::OsmBuilder::MemberIn> members;
    osm::OsmBuilder::TagList tags;
};

class AllRelationHandler final : public PbfHandler {
public:
    explicit AllRelationHandler(std::vector<RelRec>& out) : out_(out) {}
    void on_relation(const OsmRelation& r) override {
        RelRec rec;
        rec.id = r.id;
        rec.members.reserve(r.members.size());
        for (const OsmMember& m : r.members) {
            osm::MemberType type = osm::MemberType::Node;
            if (m.type == OsmMemberType::Way) {
                type = osm::MemberType::Way;
            } else if (m.type == OsmMemberType::Relation) {
                type = osm::MemberType::Relation;
            }
            rec.members.push_back(osm::OsmBuilder::MemberIn{type, m.ref, m.role});
        }
        rec.tags.reserve(r.tags.size());
        for (const Tag& t : r.tags) rec.tags.emplace_back(t.key, t.value);
        out_.push_back(std::move(rec));
    }

private:
    std::vector<RelRec>& out_;
};

class NodeTagHandler final : public PbfHandler {
public:
    NodeTagHandler(const std::unordered_set<std::int64_t>& wanted, osm::OsmBuilder& builder)
        : wanted_(wanted), builder_(builder) {}
    void on_node(const OsmNode& n) override {
        if (wanted_.count(n.id) == 0) return;
        osm::OsmBuilder::TagList tags;
        tags.reserve(n.tags.size());
        for (const Tag& t : n.tags) tags.emplace_back(t.key, t.value);
        builder_.add_node(n.id, n.lat_1e7, n.lon_1e7, std::move(tags));
    }

private:
    const std::unordered_set<std::int64_t>& wanted_;
    osm::OsmBuilder& builder_;
};

double ms_between(std::chrono::steady_clock::time_point a, std::chrono::steady_clock::time_point b) {
    return std::chrono::duration<double, std::milli>(b - a).count();
}

} // namespace

Result<osm::OsmData> extract_region_set(const std::string& pbf, const ResolvedRegion& region,
                                         OsmExtractReport* report_out) {
    OsmExtractReport local_report;
    OsmExtractReport& report = report_out != nullptr ? *report_out : local_report;

    const std::optional<geo::UtmZone> zone = geo::parse_utm_scheme(region.scheme);
    if (!zone) {
        return make_error(ErrorCode::InvalidArgument,
                           "osm: extract_region_set: unparseable region scheme '" + region.scheme + "'");
    }

    const std::vector<TileKey> region_tile_keys = tile_keys(region);
    if (region_tile_keys.empty()) {
        osm::OsmBuilder empty_builder;
        return empty_builder.build();
    }
    std::unordered_set<std::uint64_t> region_tiles;
    region_tiles.reserve(region_tile_keys.size() * 2);
    for (const TileKey& k : region_tile_keys) region_tiles.insert(k.packed());

    const geo::Utm utm(geo::kGRS80);
    const std::int64_t halo_mm = static_cast<std::int64_t>(kDefaultOsmHaloM) * 1000;
    const std::int64_t size_mm = kBucketSizeMm;

    try {
        // --- N1: Nodes only ---
        const auto t0 = std::chrono::steady_clock::now();
        std::vector<NodeRec> nodes;
        {
            NodeScanHandler handler(nodes);
            read_pbf(pbf, handler, nullptr, PbfEntities::Nodes);
            if (!handler.sorted()) {
                return make_error(ErrorCode::Malformed, "osm: extract_region_set: PBF not sorted");
            }
        }
        report.nodes_total = nodes.size();
        report.index_bytes = nodes.size() * sizeof(NodeRec);

        std::unordered_set<std::int64_t> node_in_ids;
        for (const NodeRec& rec : nodes) {
            const std::optional<Pt> p = geom::project_e7_to_mm(utm, *zone, rec.lat, rec.lon);
            if (!p) continue;
            if (point_touches_region(*p, halo_mm, size_mm, region_tiles, *zone)) node_in_ids.insert(rec.id);
        }
        report.nodes_in = node_in_ids.size();
        std::unordered_set<std::int64_t> keep_node_ids = node_in_ids;
        const auto t1 = std::chrono::steady_clock::now();
        report.n1_ms = ms_between(t0, t1);

        // --- W: Ways only ---
        std::vector<WayRec> ways_in;
        std::uint64_t ways_incomplete = 0;
        {
            WayScanHandler handler(nodes, node_in_ids, region_tiles, halo_mm, size_mm, *zone, utm, ways_in,
                                    keep_node_ids, ways_incomplete);
            read_pbf(pbf, handler, nullptr, PbfEntities::Ways);
        }
        report.ways_in = ways_in.size();
        report.ways_incomplete = ways_incomplete;
        std::unordered_set<std::int64_t> way_in_ids;
        way_in_ids.reserve(ways_in.size() * 2);
        for (const WayRec& w : ways_in) way_in_ids.insert(w.id);
        const auto t2 = std::chrono::steady_clock::now();
        report.w_ms = ms_between(t1, t2);

        // --- R: Relations only (all of them; not geometric, so no bbox to filter by) ---
        std::vector<RelRec> all_relations;
        {
            AllRelationHandler handler(all_relations);
            read_pbf(pbf, handler, nullptr, PbfEntities::Relations);
        }
        std::unordered_map<std::int64_t, std::size_t> rel_index;
        rel_index.reserve(all_relations.size() * 2);
        for (std::size_t i = 0; i < all_relations.size(); ++i) rel_index.emplace(all_relations[i].id, i);

        std::vector<char> rel_in(all_relations.size(), 0);
        bool changed = true;
        while (changed) {
            changed = false;
            for (std::size_t i = 0; i < all_relations.size(); ++i) {
                if (rel_in[i] != 0) continue;
                bool inc = false;
                for (const osm::OsmBuilder::MemberIn& m : all_relations[i].members) {
                    if (m.type == osm::MemberType::Node) {
                        inc = keep_node_ids.count(m.ref) != 0;
                    } else if (m.type == osm::MemberType::Way) {
                        inc = way_in_ids.count(m.ref) != 0;
                    } else {
                        const auto it = rel_index.find(m.ref);
                        inc = it != rel_index.end() && rel_in[it->second] != 0;
                    }
                    if (inc) break;
                }
                if (inc) {
                    rel_in[i] = 1;
                    changed = true;
                }
            }
        }
        std::uint64_t relations_in = 0;
        for (const char v : rel_in) relations_in += (v != 0) ? 1u : 0u;
        report.relations_in = relations_in;
        const auto t3 = std::chrono::steady_clock::now();
        report.r_ms = ms_between(t2, t3);

        // --- N2: Nodes only, again (full tags for keep_node_ids) ---
        osm::OsmBuilder builder;
        {
            NodeTagHandler handler(keep_node_ids, builder);
            read_pbf(pbf, handler, nullptr, PbfEntities::Nodes);
        }
        for (const WayRec& w : ways_in) builder.add_way(w.id, w.refs, w.tags);
        for (std::size_t i = 0; i < all_relations.size(); ++i) {
            if (rel_in[i] == 0) continue;
            builder.add_relation(all_relations[i].id, all_relations[i].members, all_relations[i].tags);
        }
        const auto t4 = std::chrono::steady_clock::now();
        report.n2_ms = ms_between(t3, t4);

        return builder.build();
    } catch (const std::exception& e) {
        return make_error(ErrorCode::Io, std::string("osm: extract_region_set: ") + e.what());
    }
}

Result<OsmImportReport> import_osm(const ResolvedRegion& region, const osm::OsmData& region_set, ITileSink& sink,
                                    std::string_view scope, const OsmImportOptions& options) {
    (void)scope; // informational only - see osm.h's own doc comment (mirrors import_dem's `scope`)
    const auto t0 = std::chrono::steady_clock::now();

    const std::vector<TileKey> keys = tile_keys(region);
    OsmImportReport report;
    if (keys.empty()) return report;

    const std::optional<geo::UtmZone> zone = geo::parse_utm_scheme(region.scheme);
    if (!zone) {
        return make_error(ErrorCode::InvalidArgument,
                           "osm: import_osm: unparseable region scheme '" + region.scheme + "'");
    }

    unsigned jobs = options.jobs;
    if (jobs == 0) jobs = std::max(1u, std::thread::hardware_concurrency());

    const OsmClipIndex index = OsmClipIndex::build(region_set, *zone, kDefaultOsmHaloM);

    struct TileOutcome {
        bool empty = true;
        std::vector<std::uint8_t> encoded_body;
    };

    const auto produce = [&](std::size_t idx) -> TileOutcome {
        osm::OsmData clipped = index.clip(keys[idx]);
        TileOutcome outcome;
        if (clipped.nodes.empty() && clipped.ways.empty() && clipped.relations.empty()) {
            outcome.empty = true;
            return outcome;
        }
        OsmTile tile;
        tile.halo_m = index.halo_m();
        tile.data = std::move(clipped);
        const Result<TileBody> body = encode_src_osm(tile);
        if (!body) throw std::runtime_error("osm: encode_src_osm: " + body.error().message);
        const Result<std::vector<std::uint8_t>> encoded = encode_body(*body);
        if (!encoded) throw std::runtime_error("osm: encode_body: " + encoded.error().message);
        outcome.empty = false;
        outcome.encoded_body = *encoded;
        return outcome;
    };

    const auto consume = [&](std::size_t idx, TileOutcome&& outcome) {
        if (outcome.empty) {
            ++report.tiles_empty;
            return;
        }
        ++report.tiles_written;

        TileHeader header;
        header.layer = std::string(kSrcOsmLayer);
        header.layer_version = kSrcOsmEncodingVersion;
        header.key = keys[idx];
        header.datum = VerticalDatum::None;
        const Result<void> put_result = sink.put(header.layer, keys[idx], header, outcome.encoded_body);
        if (!put_result) throw std::runtime_error("osm: sink.put: " + put_result.error().message);
    };

    try {
        ordered_for(keys.size(), static_cast<int>(jobs), produce, consume);
    } catch (const std::exception& e) {
        return make_error(ErrorCode::Io, std::string("osm: ") + e.what());
    }

    const auto t1 = std::chrono::steady_clock::now();
    report.wall_ms = ms_between(t0, t1);
    return report;
}

} // namespace g2m::import
