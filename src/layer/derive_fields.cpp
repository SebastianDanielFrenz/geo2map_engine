// g2m.fields.geom deriver (docs/pipeline/fields.md). The geometry kernel is field_subdivide.cpp; this file finds the
// agricultural blocks in the OSM tiles, gathers what cuts them, and turns pieces into tile records.
//
// Tile invariance: the result for a block is a function of the block alone. Its inputs are the level-2 OSM tiles
// covering the block's bounding box grown by the largest splitter half width plus two raster cells (a way touching
// that box lies in one of those tiles, complete), and a relation block is completed first by fetching the tiles of
// its open chain ends. A block is therefore cut the same way in every tile it touches.
#include "g2m/core/geom/project.h"
#include "g2m/core/geom/ring_assembly.h"
#include "g2m/core/hash.h"
#include "g2m/layer/field_subdivide.h"
#include "g2m/layer/field_tile.h"
#include "g2m/layer/src_osm.h"
#include <algorithm>
#include <limits>
#include <map>
#include <set>

namespace g2m {
namespace {
using geom::PointMm;
using geom::Ring;

struct LineFeat { std::int64_t way_id; std::int32_t half_width_mm; std::vector<PointMm> pts; geom::BoxMm bbox; };
struct AreaFeat { std::int64_t id; FieldObstacle shape; };
struct RelFeat { std::int64_t id; std::vector<std::int64_t> ways; };

struct SrcTile {
    TileKey key;
    osm::OsmData data;
    std::vector<std::optional<PointMm>> pos; // per node, indexed like data.nodes
    // Features, built once per tile.
    std::vector<LineFeat> lines;
    std::vector<AreaFeat> areas;
    std::vector<RelFeat> obstacle_relations;
};

const PointMm* node_pos(const SrcTile& t, std::int64_t id) {
    const auto it = std::lower_bound(t.data.nodes.begin(), t.data.nodes.end(), id, [](const osm::Node& n, std::int64_t v) { return n.id < v; });
    if (it == t.data.nodes.end() || it->id != id) return nullptr;
    const auto& p = t.pos[static_cast<std::size_t>(it - t.data.nodes.begin())];
    return p ? &*p : nullptr;
}
bool way_points(const SrcTile& t, const osm::Way& w, std::vector<PointMm>& out) {
    out.clear();
    for (const auto id : t.data.refs_of(w)) {
        const PointMm* p = node_pos(t, id);
        if (!p) return false;
        out.push_back(*p);
    }
    return true;
}
geom::BoxMm points_bbox(const std::vector<PointMm>& pts) {
    geom::BoxMm b{std::numeric_limits<std::int64_t>::max(), std::numeric_limits<std::int64_t>::max(),
        std::numeric_limits<std::int64_t>::min(), std::numeric_limits<std::int64_t>::min()};
    for (const auto& p : pts) { b.w = std::min(b.w, p.x); b.e = std::max(b.e, p.x); b.s = std::min(b.s, p.y); b.n = std::max(b.n, p.y); }
    return b;
}
bool boxes_overlap(const geom::BoxMm& a, const geom::BoxMm& b) { return a.w <= b.e && b.w <= a.e && a.s <= b.n && b.s <= a.n; }
geom::BoxMm grown(geom::BoxMm b, std::int64_t m) { return {b.w - m, b.s - m, b.e + m, b.n + m}; }

std::optional<std::string_view> tag_value(const osm::OsmData& d, std::span<const osm::Tag> tags, std::string_view key) { return d.tag(tags, key); }

bool any_skip(const FieldRules& rules, const osm::OsmData& d, std::span<const osm::Tag> tags) {
    for (const auto& s : rules.skip) {
        const auto v = tag_value(d, tags, s.key);
        if (v && field_tag_matches(s, *v)) return true;
    }
    return false;
}

Result<std::optional<SrcTile>> load_tile(const InputTile& input, const geo::Utm& utm) {
    if (!input.header) {
        if (!input.body.empty()) return make_error(ErrorCode::Malformed, "fields.geom: absent input has bytes");
        return std::optional<SrcTile>{};
    }
    const auto& header = *input.header;
    if (header.key != input.ref.key || header.layer != kSrcOsmLayer || header.layer_version != 1 ||
        header.body_size != input.body.size() || header.body_hash != content_hash(input.body))
        return make_error(ErrorCode::Malformed, "fields.geom: source header mismatch");
    auto body = decode_body(input.body); if (!body) return body.error();
    auto osm_tile = decode_src_osm(*body); if (!osm_tile) return osm_tile.error();
    SrcTile t; t.key = input.ref.key; t.data = std::move(osm_tile->data);
    t.pos.reserve(t.data.nodes.size());
    for (const auto& n : t.data.nodes) t.pos.push_back(geom::project_e7_to_mm(utm, t.key.zone, n.lat, n.lon));
    return std::optional<SrcTile>{std::move(t)};
}

void build_features(const FieldRules& rules, SrcTile& t) {
    const auto& d = t.data;
    std::vector<PointMm> pts;
    for (const auto& w : d.ways) {
        const auto tags = d.tags_of(w);
        if (!way_points(t, w, pts) || pts.size() < 2) continue;
        // line features
        if (!any_skip(rules, d, tags)) {
            for (const auto& s : rules.splitters) {
                const auto v = tag_value(d, tags, s.match.key);
                if (v && field_tag_matches(s.match, *v)) {
                    t.lines.push_back({w.id, s.half_width_mm, pts, points_bbox(pts)});
                    break; // the first matching rule (table order) decides the width
                }
            }
        }
        // closed ways as obstacle areas
        if (pts.size() >= 4 && pts.front() == pts.back()) {
            for (const auto& o : rules.obstacles) {
                const auto v = tag_value(d, tags, o.match.key);
                if (v && field_tag_matches(o.match, *v)) {
                    Ring ring(pts.begin(), pts.end() - 1);
                    AreaFeat a; a.id = w.id; a.shape.bbox = points_bbox(pts); a.shape.rings.push_back(std::move(ring));
                    t.areas.push_back(std::move(a));
                    break;
                }
            }
        }
    }
    for (const auto& r : d.relations) {
        const auto tags = d.tags_of(r);
        const auto type = tag_value(d, tags, "type");
        if (!type || *type != "multipolygon") continue;
        for (const auto& o : rules.obstacles) {
            const auto v = tag_value(d, tags, o.match.key);
            if (v && field_tag_matches(o.match, *v)) {
                RelFeat f; f.id = r.id;
                for (const auto& m : d.members_of(r)) if (m.type == osm::MemberType::Way) f.ways.push_back(m.ref);
                std::sort(f.ways.begin(), f.ways.end());
                f.ways.erase(std::unique(f.ways.begin(), f.ways.end()), f.ways.end());
                t.obstacle_relations.push_back(std::move(f));
                break;
            }
        }
    }
}

// ---- blocks ------------------------------------------------------------------------------------------------
struct Member { std::int64_t way_id; std::string role; };
struct Block {
    FieldSource source = FieldSource::OsmWay;
    std::int64_t osm_id = 0;
    std::uint8_t landuse = 0;
    std::string crop;
    std::vector<Member> members; // a way block has exactly one
};
enum class BlockStatus : std::uint8_t { Ready, Pending, Declined };
struct BlockPlan {
    Block block;
    BlockStatus status = BlockStatus::Pending;
    FieldDeclineReason decline = FieldDeclineReason::InvalidGeometry;
    std::vector<TileKey> cover;          // the view: tiles covering the block (Ready)
    std::vector<TileKey> needed;         // tiles to add to the pool (Pending)
    geom::BoxMm bbox{};
};

struct Pool {
    std::vector<SrcTile> tiles;          // present tiles
    std::set<TileKey> keys;              // every resolved ref, present or absent
    const SrcTile* find(const TileKey& k) const {
        for (const auto& t : tiles) if (t.key == k) return &t;
        return nullptr;
    }
};

struct Located { const SrcTile* tile; const osm::Way* way; };
std::optional<Located> locate_way(const std::vector<const SrcTile*>& view, std::int64_t id) {
    for (const SrcTile* t : view) if (const auto* w = t->data.find_way(id)) return Located{t, w};
    return std::nullopt;
}

std::string crop_of(const osm::OsmData& d, std::span<const osm::Tag> tags) {
    const auto v = d.tag(tags, "crop");
    if (!v) return {};
    std::string s(*v);
    if (s.size() > kFieldsMaxCrop) s.resize(kFieldsMaxCrop);
    return s;
}

std::vector<Block> candidate_blocks(const FieldRules& rules, const SrcTile& t) {
    std::vector<Block> out;
    const auto& d = t.data;
    std::set<std::int64_t> relation_ways;
    for (const auto& r : d.relations) {
        const auto tags = d.tags_of(r);
        const auto type = d.tag(tags, "type"); const auto lu = d.tag(tags, "landuse");
        if (!type || *type != "multipolygon" || !lu) continue;
        const int code = rules.landuse_code(*lu);
        if (code < 0) continue;
        Block b; b.source = FieldSource::OsmRelation; b.osm_id = r.id; b.landuse = static_cast<std::uint8_t>(code); b.crop = crop_of(d, tags);
        for (const auto& m : d.members_of(r)) {
            if (m.type != osm::MemberType::Way) continue;
            const std::string_view role = d.str(m.role);
            if (role != "outer" && role != "inner" && !role.empty()) continue;
            b.members.push_back({m.ref, std::string(role)});
            relation_ways.insert(m.ref);
        }
        if (!b.members.empty()) out.push_back(std::move(b));
    }
    for (const auto& w : d.ways) {
        const auto tags = d.tags_of(w);
        const auto lu = d.tag(tags, "landuse");
        if (!lu) continue;
        const int code = rules.landuse_code(*lu);
        if (code < 0 || relation_ways.contains(w.id)) continue;
        const auto refs = d.refs_of(w);
        if (refs.size() < 4 || refs.front() != refs.back()) continue;
        Block b; b.source = FieldSource::OsmWay; b.osm_id = w.id; b.landuse = static_cast<std::uint8_t>(code); b.crop = crop_of(d, tags);
        b.members.push_back({w.id, "outer"});
        out.push_back(std::move(b));
    }
    return out;
}

TileKey tile_at_mm(const TileKey& like, std::int64_t x, std::int64_t y) {
    const std::int64_t size = like.size_m() * 1000;
    return TileKey{like.zone, 2, static_cast<std::int32_t>(floor_div(x, size)), static_cast<std::int32_t>(floor_div(y, size))};
}

// Where a block stands given the tiles resolved so far.
BlockPlan plan_block(const FieldRules& rules, const TileKey& target, const Pool& pool, Block block) {
    BlockPlan plan; plan.block = std::move(block);
    std::vector<const SrcTile*> all;
    for (const auto& t : pool.tiles) all.push_back(&t);
    std::set<std::int64_t> ids_found;
    std::map<std::int64_t, int> end_count; std::map<std::int64_t, PointMm> end_pos;
    geom::BoxMm box{std::numeric_limits<std::int64_t>::max(), std::numeric_limits<std::int64_t>::max(),
        std::numeric_limits<std::int64_t>::min(), std::numeric_limits<std::int64_t>::min()};
    bool missing = false, bad = false;
    std::vector<PointMm> pts;
    std::set<std::int64_t> seen_member;
    for (const auto& m : plan.block.members) {
        if (!seen_member.insert(m.way_id).second) continue;
        const auto loc = locate_way(all, m.way_id);
        if (!loc) { missing = true; continue; }
        if (!way_points(*loc->tile, *loc->way, pts) || pts.size() < 2) { bad = true; continue; }
        const auto b = points_bbox(pts);
        box.w = std::min(box.w, b.w); box.e = std::max(box.e, b.e); box.s = std::min(box.s, b.s); box.n = std::max(box.n, b.n);
        const auto refs = loc->tile->data.refs_of(*loc->way);
        if (refs.front() != refs.back()) {
            ++end_count[refs.front()]; end_pos[refs.front()] = pts.front();
            ++end_count[refs.back()]; end_pos[refs.back()] = pts.back();
        }
    }
    if (bad || box.w > box.e) { plan.status = BlockStatus::Declined; plan.decline = FieldDeclineReason::InvalidGeometry; return plan; }
    plan.bbox = box;
    const std::int64_t extent = std::max(box.e - box.w, box.n - box.s);
    if (extent > static_cast<std::int64_t>(rules.block_max_extent_m) * 1000) { plan.status = BlockStatus::Declined; plan.decline = FieldDeclineReason::TooLarge; return plan; }
    const std::int64_t margin = rules.max_half_width_mm() + 2 * static_cast<std::int64_t>(rules.cell_mm);
    const auto lo = tile_at_mm(target, box.w - margin, box.s - margin), hi = tile_at_mm(target, box.e + margin, box.n + margin);
    const std::int64_t count = (static_cast<std::int64_t>(hi.x) - lo.x + 1) * (static_cast<std::int64_t>(hi.y) - lo.y + 1);
    if (count > rules.block_max_cover_tiles) { plan.status = BlockStatus::Declined; plan.decline = FieldDeclineReason::TooManyTiles; return plan; }
    std::set<TileKey> needs;
    for (std::int32_t y = lo.y; y <= hi.y; ++y) for (std::int32_t x = lo.x; x <= hi.x; ++x) {
        const TileKey k{target.zone, 2, x, y};
        plan.cover.push_back(k);
        if (!pool.keys.contains(k)) needs.insert(k);
    }
    for (const auto& [node, n] : end_count) {
        if (n % 2 == 0) continue; // chained to another found member
        const TileKey k = tile_at_mm(target, end_pos[node].x, end_pos[node].y);
        if (!pool.keys.contains(k)) needs.insert(k);
    }
    plan.needed.assign(needs.begin(), needs.end());
    plan.status = (!plan.needed.empty() || missing) ? BlockStatus::Pending : BlockStatus::Ready;
    if (plan.status == BlockStatus::Pending && plan.needed.empty()) plan.decline = FieldDeclineReason::RelationIncomplete; // nothing left to fetch
    return plan;
}

struct Context {
    const FieldRules& rules;
    TileKey target;
    geo::Utm utm{geo::kGRS80};
};

Result<Pool> build_pool(const Context& cx, std::span<const InputTile> inputs) {
    Pool pool;
    std::size_t bytes = 0;
    for (const auto& input : inputs) {
        if (input.ref.layer != kSrcOsmLayer) return make_error(ErrorCode::InvalidArgument, "fields.geom: unexpected input layer");
        if (input.body.size() > kMaxBodyRawSize - bytes) return make_error(ErrorCode::LimitExceeded, "fields.geom: input byte budget");
        bytes += input.body.size();
        if (!pool.keys.insert(input.ref.key).second) return make_error(ErrorCode::InvalidArgument, "fields.geom: duplicate input");
        auto tile = load_tile(input, cx.utm); if (!tile) return tile.error();
        if (*tile) { build_features(cx.rules, **tile); pool.tiles.push_back(std::move(**tile)); }
    }
    std::sort(pool.tiles.begin(), pool.tiles.end(), [](const SrcTile& a, const SrcTile& b) { return a.key < b.key; });
    return pool;
}

std::vector<BlockPlan> plan_all(const Context& cx, const Pool& pool) {
    std::vector<BlockPlan> plans;
    const SrcTile* home = pool.find(cx.target);
    if (!home) return plans;
    for (auto& b : candidate_blocks(cx.rules, *home)) plans.push_back(plan_block(cx.rules, cx.target, pool, std::move(b)));
    return plans;
}

// ---- geometry of one block ---------------------------------------------------------------------------------
struct BlockGeometry {
    std::vector<FieldPolygon> polygons; // ascending by first outer vertex
};
Result<BlockGeometry> assemble_block(const Block& block, const std::vector<const SrcTile*>& view) {
    std::vector<geom::MemberWay> members;
    std::set<std::int64_t> seen;
    for (const auto& m : block.members) {
        if (!seen.insert(m.way_id).second) continue;
        const auto loc = locate_way(view, m.way_id);
        if (!loc) return make_error(ErrorCode::NotFound, "fields.geom: member way missing");
        members.push_back({m.way_id, m.role, loc->tile->data.refs_of(*loc->way)});
    }
    const geom::NodePos pos = [&](std::int64_t id) -> std::optional<PointMm> {
        for (const SrcTile* t : view) if (const PointMm* p = node_pos(*t, id)) return *p;
        return std::nullopt;
    };
    auto area = geom::assemble_rings(std::move(members), pos); if (!area) return area.error();
    BlockGeometry g;
    for (const auto& outer : area->rings.outers) {
        FieldPolygon poly; poly.outer = outer;
        normalise_field_ring(poly.outer, true);
        g.polygons.push_back(std::move(poly));
    }
    for (const auto& inner : area->rings.inners) {
        FieldPolygon* owner = nullptr; std::int64_t owner_area = 0;
        for (auto& poly : g.polygons) {
            const std::int64_t a = ring_twice_area_mm2(poly.outer);
            if (field_ring_contains(poly.outer, inner[0]) && (!owner || a < owner_area)) { owner = &poly; owner_area = a; }
        }
        if (!owner) continue;
        Ring h = inner; normalise_field_ring(h, false);
        owner->holes.push_back(std::move(h));
    }
    for (auto& poly : g.polygons)
        std::sort(poly.holes.begin(), poly.holes.end(), [](const Ring& a, const Ring& b) { return a[0].x != b[0].x ? a[0].x < b[0].x : a[0].y < b[0].y; });
    g.polygons.erase(std::remove_if(g.polygons.begin(), g.polygons.end(), [](const FieldPolygon& p) { return field_polygon_area_mm2(p) == 0; }), g.polygons.end());
    std::sort(g.polygons.begin(), g.polygons.end(), [](const FieldPolygon& a, const FieldPolygon& b) {
        return a.outer[0].x != b.outer[0].x ? a.outer[0].x < b.outer[0].x : a.outer[0].y < b.outer[0].y; });
    return g;
}

// Obstacles from the view: closed ways plus multipolygon relations that are complete inside the view.
void gather_features(const FieldRules& rules, const Block& block, const geom::BoxMm& box, const std::vector<const SrcTile*>& view,
    std::vector<FieldSplitSegment>& lines, std::vector<FieldObstacle>& obstacles) {
    const geom::BoxMm wide = grown(box, rules.max_half_width_mm() + 2 * static_cast<std::int64_t>(rules.cell_mm));
    std::set<std::int64_t> seen_lines, seen_areas, seen_rel;
    std::set<std::int64_t> own;
    for (const auto& m : block.members) own.insert(m.way_id);
    for (const SrcTile* t : view) {
        for (const auto& l : t->lines) {
            if (own.contains(l.way_id) || !boxes_overlap(grown(l.bbox, l.half_width_mm), wide) || !seen_lines.insert(l.way_id).second) continue;
            for (std::size_t i = 0; i + 1 < l.pts.size(); ++i) lines.push_back({l.pts[i], l.pts[i + 1], l.half_width_mm});
        }
        for (const auto& a : t->areas) {
            if (own.contains(a.id) || !boxes_overlap(a.shape.bbox, box) || !seen_areas.insert(a.id).second) continue;
            obstacles.push_back(a.shape);
        }
    }
    for (const SrcTile* t : view) {
        for (const auto& r : t->obstacle_relations) {
            if (!seen_rel.insert(r.id).second) continue;
            std::vector<geom::MemberWay> members; bool complete = true, touches = false;
            std::vector<PointMm> pts;
            for (const auto id : r.ways) {
                if (own.contains(id)) { complete = false; break; }
                const auto loc = locate_way(view, id);
                if (!loc) { complete = false; break; }
                if (way_points(*loc->tile, *loc->way, pts) && boxes_overlap(points_bbox(pts), box)) touches = true;
                members.push_back({id, "", loc->tile->data.refs_of(*loc->way)});
            }
            if (!complete || !touches) continue;
            const geom::NodePos pos = [&](std::int64_t id) -> std::optional<PointMm> {
                for (const SrcTile* v : view) if (const PointMm* p = node_pos(*v, id)) return *p;
                return std::nullopt;
            };
            auto area = geom::assemble_rings(std::move(members), pos);
            if (!area) continue; // an obstacle that does not assemble is ignored, never the field
            FieldObstacle o;
            for (const auto& ring : area->rings.outers) o.rings.push_back(ring);
            for (const auto& ring : area->rings.inners) o.rings.push_back(ring);
            o.bbox = area->rings.bbox;
            obstacles.push_back(std::move(o));
        }
    }
}

struct BlockResult {
    std::vector<FieldRecord> records;      // every piece of the block, ascending piece
    std::optional<FieldDeclineReason> declined;
    PointMm owner_vertex{};                // smallest outline vertex
    bool subdivided = false;
    std::uint32_t slivers_dropped = 0, obstacles_ignored = 0, sliver_blocks = 0;
};

BlockResult process_block(const FieldRules& rules, const BlockPlan& plan, const Pool& pool) {
    BlockResult out;
    std::vector<const SrcTile*> view;
    for (const auto& k : plan.cover) if (const SrcTile* t = pool.find(k)) view.push_back(t);
    auto geometry = assemble_block(plan.block, view);
    if (!geometry) {   // a member way absent from the covering tiles (the block reaches past the imported region) is an incomplete relation
        out.declined = geometry.error().code == ErrorCode::NotFound ? FieldDeclineReason::RelationIncomplete : FieldDeclineReason::InvalidGeometry;
        return out;
    }
    if (geometry->polygons.empty()) { out.declined = FieldDeclineReason::InvalidGeometry; return out; }
    out.owner_vertex = geometry->polygons[0].outer[0];
    std::vector<FieldSplitSegment> lines; std::vector<FieldObstacle> obstacles;
    gather_features(rules, plan.block, plan.bbox, view, lines, obstacles);
    std::uint32_t piece = 0;
    for (const auto& poly : geometry->polygons) {
        const auto pbox = ring_bbox(poly.outer);
        std::vector<const FieldObstacle*> hits;
        for (const auto& o : obstacles) if (boxes_overlap(o.bbox, pbox)) hits.push_back(&o);
        auto sub = subdivide_polygon(rules, poly, lines, hits);
        if (!sub) { out.declined = FieldDeclineReason::InvalidGeometry; return out; }
        if (sub->declined) { out.declined = sub->declined; return out; }
        out.slivers_dropped += sub->slivers_dropped;
        if (sub->obstacles_ignored) ++out.obstacles_ignored;
        if (sub->sliver_block) ++out.sliver_blocks;
        std::uint32_t flags = 0;
        if (sub->obstacles_ignored) flags |= field_flag::ObstaclesIgnored;
        if (sub->sliver_block) flags |= field_flag::SliverBlock;
        if (sub->simplify_fallback) flags |= field_flag::SimplifyFallback;
        std::vector<FieldPolygon> pieces;
        if (sub->verbatim) pieces.push_back(poly);
        else { pieces = std::move(sub->pieces); flags |= field_flag::Subdivided; out.subdivided = true; }
        for (auto& p : pieces) {
            const auto id = pack_field_id(plan.block.source, plan.block.osm_id, piece);
            if (!id) { out.declined = FieldDeclineReason::TooLarge; return out; }
            FieldRecord r;
            r.id = *id; r.source = plan.block.source; r.osm_id = plan.block.osm_id; r.piece = static_cast<std::uint16_t>(piece);
            r.landuse = plan.block.landuse; r.crop = plan.block.crop; r.flags = flags; r.polygon = std::move(p);
            finalize_field_record(r, rules);
            out.records.push_back(std::move(r));
            ++piece;
        }
    }
    return out;
}
} // namespace

ContentHash FieldsGeomDeriver::config_hash() const {
    const auto& sha = builtin_field_rules_de().content_sha256;
    return Sha256::of("g2m.fields.geom/1 enc=10 impl=2 rules=" + to_hex(std::span<const std::uint8_t>(sha.data(), sha.size())) +
        " raster=cell-centre-scanline-evenodd capsule=exact conn=4 trace=left-turn dp=exact piece-order=sw-cell anchor=9-lines"
        " block-owner=min-vertex view=cover(bbox+halfwidth+2cells) relation-walk=open-chain-ends stages=4");
}

std::vector<InputTileRef> FieldsGeomDeriver::inputs_for(const TileKey& target) const {
    if (!target.valid() || target.level != 2) return {};
    return {{std::string(kSrcOsmLayer), target}};
}

Result<std::vector<InputTileRef>> FieldsGeomDeriver::additional_inputs_for(const DeriveRequest& request) const {
    if (!request.target.valid() || request.target.level != 2) return make_error(ErrorCode::InvalidArgument, "fields.geom: level-2 target required");
    const Context cx{builtin_field_rules_de(), request.target, geo::Utm(geo::kGRS80)};
    auto pool = build_pool(cx, request.inputs); if (!pool) return pool.error();
    std::set<InputTileRef> refs;
    for (const auto& plan : plan_all(cx, *pool))
        if (plan.status == BlockStatus::Pending) for (const auto& k : plan.needed) refs.insert({std::string(kSrcOsmLayer), k});
    return std::vector<InputTileRef>(refs.begin(), refs.end());
}

Result<DeriveResult> FieldsGeomDeriver::derive(const DeriveRequest& request) const {
    if (!request.target.valid() || request.target.level != 2) return make_error(ErrorCode::InvalidArgument, "fields.geom: level-2 target required");
    const FieldRules& rules = builtin_field_rules_de();
    const Context cx{rules, request.target, geo::Utm(geo::kGRS80)};
    bool has_target = false;
    for (const auto& input : request.inputs) has_target = has_target || input.ref.key == request.target;
    if (!has_target) return make_error(ErrorCode::InvalidArgument, "fields.geom: incomplete input set");
    auto pool = build_pool(cx, request.inputs); if (!pool) return pool.error();
    FieldTile tile; tile.key = request.target;
    const std::int64_t size = request.target.size_m() * 1000;
    const geom::BoxMm box{request.target.min_easting() * 1000, request.target.min_northing() * 1000,
        request.target.min_easting() * 1000 + size, request.target.min_northing() * 1000 + size};
    std::map<std::pair<std::uint8_t, std::int64_t>, FieldDeclineReason> declines;
    for (auto& plan : plan_all(cx, *pool)) {
        const auto key = std::make_pair(static_cast<std::uint8_t>(plan.block.source), plan.block.osm_id);
        if (plan.status == BlockStatus::Declined) { declines.emplace(key, plan.decline); continue; }
        if (plan.status == BlockStatus::Pending) { declines.emplace(key, FieldDeclineReason::RelationIncomplete); continue; }
        BlockResult res = process_block(rules, plan, *pool);
        if (res.declined) { declines.emplace(key, *res.declined); continue; }
        const bool owned_block = res.owner_vertex.x >= box.w && res.owner_vertex.x < box.e && res.owner_vertex.y >= box.s && res.owner_vertex.y < box.n;
        if (owned_block) {
            ++tile.counts.blocks;
            if (res.subdivided) ++tile.counts.subdivided;
            tile.counts.slivers_dropped += res.slivers_dropped; tile.counts.obstacles_ignored += res.obstacles_ignored;
            tile.counts.sliver_blocks += res.sliver_blocks;
        }
        for (auto& r : res.records) {
            if (!field_polygon_touches_box(r.polygon, box)) continue;
            if (r.anchor.x >= box.w && r.anchor.x < box.e && r.anchor.y >= box.s && r.anchor.y < box.n) r.flags |= field_flag::Owned;
            tile.fields.push_back(std::move(r));
        }
    }
    std::sort(tile.fields.begin(), tile.fields.end(), [](const FieldRecord& a, const FieldRecord& b) { return a.id < b.id; });
    for (const auto& [key, reason] : declines)
        tile.declines.push_back({static_cast<FieldSource>(key.first), key.second, reason});
    auto body = encode_fields_geom(tile); if (!body) return body.error();
    auto bytes = encode_body(*body); if (!bytes) return bytes.error();
    return DeriveResult{std::move(*bytes), VerticalDatum::None, 0};
}
} // namespace g2m
