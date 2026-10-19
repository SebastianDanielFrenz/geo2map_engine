// tools/road_census/main.cpp - g2m_road_census: read-only censuses over the road graphs of a block of level-2 tiles of an
// existing immutable source store (never writes to it). The graphs of all tiles are derived in memory and merged into one
// world graph, so node degrees and join headings are the world's, not a tile's.
//
//   g2m_road_census arrows STORE SCOPE X0 X1 Y0 Y1
//   g2m_road_census way    STORE SCOPE X0 X1 Y0 Y1 WAY_ID...        (tags of single ways)
//   g2m_road_census bow    STORE SCOPE X0 X1 Y0 Y1 FACTOR MARGIN_M [WAY_ID...]
//
// X0..Y1 are inclusive level-2 tile indices on UTM zone 32 (tile edge 1024 m: E 455-490 km is x 444..478, N 5540-5585 km is
// y 5410..5454).
// `bow` (vault G2M-024): fits every stretch of every way lying wholly in the block twice, without and with the chord bound
// (ReferenceFitLimits::chord_half_width_factor / chord_margin_m) and reports how far the fitted lines leave their OSM
// polylines, how many stretches change their fit and by how much (motorway / trunk separately).
// `arrows`: the turn:lanes census of street fidelity (vault G2M-023/G2M-024): per way and lane which glyphs the ribbons used
// to paint (the old substring rule, reproduced here as `legacy_glyphs`) and which they paint now (lane_arrow_glyphs).
#include "g2m/core/tile_key.h"
#include "g2m/layer/road_cross_section.h"
#include "g2m/layer/road_graph_tile.h"
#include "g2m/layer/road_reference.h"
#include "g2m/layer/src_osm.h"
#include "g2m/server/tile_store.h"
#include "g2m/world/road_ribbons.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <atomic>
#include <iostream>
#include <map>
#include <mutex>
#include <set>
#include <string>
#include <thread>
#include <vector>

using namespace g2m;

namespace {

struct Block { int x0, x1, y0, y1; int zone = 32; };

// The block's source OSM tiles (plus a one-tile margin) are merged into ONE osm::OsmData and turned into the world road
// graph by build_road_graph: the same graph the tile deriver builds per tile, without the per-tile limits and without the
// turn restrictions (which this census does not need, and whose oversize relations make the tile deriver reject a few tiles).
bool load_world(const std::string& store_dir, const std::string& scope, const Block& b, RoadGraph& world) {
    TileStoreConfig config; config.dir = store_dir; config.read_only = true;
    auto store = TileStore::open(config);
    if (!store) { std::cerr << store.error().message << '\n'; return false; }
    const geo::UtmZone zone{static_cast<std::uint8_t>(b.zone), geo::Hemisphere::North};
    std::vector<TileKey> keys;
    for (int y = b.y0 - 1; y <= b.y1 + 1; ++y) for (int x = b.x0 - 1; x <= b.x1 + 1; ++x) keys.push_back({zone, 2, x, y});
    using Tags = osm::OsmBuilder::TagList;
    struct NodeInput { std::int32_t lat, lon; Tags tags; };
    struct WayInput { std::vector<std::int64_t> refs; Tags tags; };
    std::map<std::int64_t, NodeInput> nodes; std::map<std::int64_t, WayInput> ways;
    std::mutex merge_mutex; std::atomic<std::size_t> next{0}, present{0}; bool failed = false;
    const auto tags_of = [](const osm::OsmData& data, std::span<const osm::Tag> tags) {
        Tags result;
        for (const auto& tag : tags) result.emplace_back(data.str(tag.key), data.str(tag.value));
        return result;
    };
    const auto work = [&] {
        for (std::size_t i = next++; i < keys.size(); i = next++) {
            auto stored = (*store)->get({scope, std::string(kSrcOsmLayer), keys[i]});
            if (!stored) { std::lock_guard l(merge_mutex); std::cerr << stored.error().message << '\n'; failed = true; return; }
            if (!*stored) continue;
            auto body = decode_body((*stored)->body); if (!body) { failed = true; return; }
            auto tile = decode_src_osm(*body); if (!tile) { failed = true; return; }
            const auto& data = tile->data;
            std::lock_guard l(merge_mutex);
            ++present;
            for (const auto& node : data.nodes) nodes.emplace(node.id, NodeInput{node.lat, node.lon, tags_of(data, data.tags_of(node))});
            for (const auto& way : data.ways) {
                const auto refs = data.refs_of(way);
                ways.emplace(way.id, WayInput{{refs.begin(), refs.end()}, tags_of(data, data.tags_of(way))});
            }
        }
    };
    std::vector<std::thread> pool;
    for (unsigned t = 0; t < 6; ++t) pool.emplace_back(work);
    for (auto& t : pool) t.join();
    if (failed) return false;
    std::cerr << "source tiles present: " << present << " of " << keys.size() << " (block plus one tile margin)\n";
    osm::OsmBuilder builder;
    for (const auto& [id, node] : nodes) builder.add_node(id, node.lat, node.lon, node.tags);
    for (const auto& [id, way] : ways) builder.add_way(id, way.refs, way.tags);
    auto merged = builder.build(); if (!merged) { std::cerr << merged.error().message << '\n'; return false; }
    auto graph = build_road_graph(*merged, zone); if (!graph) { std::cerr << graph.error().message << '\n'; return false; }
    world = std::move(*graph);
    std::cerr << "world: ways=" << world.ways.size() << " nodes=" << world.nodes.size() << " edges=" << world.edges.size() << '\n';
    return true;
}

std::string tag_of(const RoadGraphWay& w, const std::string& key) {
    for (const auto& [k, v] : w.tags) if (k == key) return v;
    return {};
}
std::vector<std::string> split_lanes(const std::string& text) {
    std::vector<std::string> out; std::size_t begin = 0;
    while (true) {
        const auto end = text.find('|', begin);
        out.push_back(text.substr(begin, end == std::string::npos ? end : end - begin));
        if (end == std::string::npos) break;
        begin = end + 1;
    }
    return out;
}
// The glyph rule before the explicit token table: substring tests on the raw token (what road_ribbons.cpp painted).
std::uint16_t legacy_glyphs(const std::string& token) {
    std::uint16_t g = 0;
    if (token.find("through") != std::string::npos) g |= world::kArrowThrough;
    if (token.find("left") != std::string::npos) g |= world::kArrowLeft;
    if (token.find("right") != std::string::npos) g |= world::kArrowRight;
    return g;
}
// The TurnBit mask of a lane value (every value known to parse_turn_token).
std::uint16_t turns_of(const std::string& lane) {
    std::uint16_t turns = 0;
    for (std::size_t b = 0; b <= lane.size();) {
        const auto e = lane.find(';', b);
        if (const auto bit = parse_turn_token(std::string_view(lane).substr(b, e == std::string::npos ? std::string_view::npos : e - b))) turns |= *bit;
        if (e == std::string::npos) break;
        b = e + 1;
    }
    return turns;
}

int arrows(const RoadGraph& world) {
    std::size_t ways_turn_lanes = 0, ways_merge = 0, ways_wrong_before = 0, ways_wrong_after = 0, ways_mirror_before = 0,
        ways_converge_after = 0, ways_unknown = 0, ways_slight = 0, ways_sharp = 0, ways_compound_merge = 0;
    std::size_t lanes_merge = 0, lanes_wrong_before = 0, lanes_wrong_after = 0, unknown_tokens = 0;
    std::vector<std::int64_t> mirror_ids, converge_ids;
    std::map<std::string, std::size_t> merge_values;
    for (const auto& w : world.ways) {
        bool has = false, merge = false, wrong_before = false, wrong_after = false, mirror = false, converge = false, unknown = false,
            slight = false, sharp = false, compound = false;
        for (const char* key : {"turn:lanes", "turn:lanes:forward", "turn:lanes:backward"}) {
            const auto value = tag_of(w, key);
            if (value.empty()) continue;
            has = true;
            const auto lanes = split_lanes(value);
            if (value.find("merge_to_") != std::string::npos) { merge = true; ++merge_values[value]; }
            std::vector<std::uint16_t> before(lanes.size()), after(lanes.size());
            for (std::size_t i = 0; i < lanes.size(); ++i) {
                std::size_t unk = 0;
                before[i] = legacy_glyphs(lanes[i]);
                after[i] = world::lane_arrow_glyphs(lanes[i], &unk);
                unknown_tokens += unk; unknown = unknown || unk;
                const auto turns = turns_of(lanes[i]);
                // The legacy rule painted a bent arrow wherever the raw token contained left/right: wrong where the lane's
                // own values do not include exactly that turn (merge_to_*, slight_*, sharp_*).
                if (((before[i] & world::kArrowLeft) && !(turns & kTurnLeft)) || ((before[i] & world::kArrowRight) && !(turns & kTurnRight))) {
                    wrong_before = true; ++lanes_wrong_before;
                }
                if (turns & (kTurnMergeToLeft | kTurnMergeToRight)) {
                    ++lanes_merge;
                    if (turns & ~(kTurnMergeToLeft | kTurnMergeToRight | kTurnNone)) compound = true;
                }
                if (turns & (kTurnSlightLeft | kTurnSlightRight)) slight = true;
                if (turns & (kTurnSharpLeft | kTurnSharpRight)) sharp = true;
                // Wrong after: a full bent arrow for a lane that has no left/right value, or a merge arrow for a lane without
                // a merge value.
                if (((after[i] & world::kArrowLeft) && !(turns & kTurnLeft)) || ((after[i] & world::kArrowRight) && !(turns & kTurnRight)) ||
                    ((after[i] & (world::kArrowMergeLeft | world::kArrowMergeRight)) && !(turns & (kTurnMergeToLeft | kTurnMergeToRight)))) {
                    wrong_after = true; ++lanes_wrong_after;
                }
            }
            for (std::size_t i = 0; i + 1 < lanes.size(); ++i) {
                // Mirrored neighbours before: an arrow pointing right on a lane, an arrow pointing left on the lane to its right.
                if ((before[i] & world::kArrowRight) && (before[i + 1] & world::kArrowLeft)) mirror = true;
                // After: the same two lanes as converging merge arrows (both lanes claim to merge into the other).
                if ((after[i] & world::kArrowMergeRight) && (after[i + 1] & world::kArrowMergeLeft)) converge = true;
            }
        }
        ways_turn_lanes += has; ways_merge += merge; ways_wrong_before += wrong_before; ways_wrong_after += wrong_after;
        ways_mirror_before += mirror; ways_converge_after += converge; ways_unknown += unknown; ways_slight += slight;
        ways_sharp += sharp; ways_compound_merge += compound;
        if (mirror) mirror_ids.push_back(w.osm_id);
        if (converge) converge_ids.push_back(w.osm_id);
    }
    std::cout << "ways with turn:lanes (any of the three keys): " << ways_turn_lanes << '\n'
              << "ways with merge_to_*: " << ways_merge << " (merge lanes " << lanes_merge
              << ", ways with a merge next to another value on one lane: " << ways_compound_merge << ")\n"
              << "ways with slight_*: " << ways_slight << ", sharp_*: " << ways_sharp << ", unknown tokens: " << unknown_tokens
              << " in " << ways_unknown << " ways\n"
              << "BEFORE (substring rule): ways with a wrong bent arrow: " << ways_wrong_before << " (lanes " << lanes_wrong_before
              << "), ways with mirrored neighbours (right-bent lane left of a left-bent lane): " << ways_mirror_before << '\n'
              << "AFTER  (token table):    ways with a wrong bent or merge arrow: " << ways_wrong_after << " (lanes " << lanes_wrong_after
              << "), ways with converging merge arrows (merge_to_right | merge_to_left): " << ways_converge_after << '\n';
    std::cout << "mirrored before:"; for (auto id : mirror_ids) std::cout << ' ' << id; std::cout << '\n';
    std::cout << "converging after:"; for (auto id : converge_ids) std::cout << ' ' << id; std::cout << '\n';
    std::cout << "merge values (count):\n";
    for (const auto& [v, n] : merge_values) std::cout << "  " << v << " : " << n << '\n';
    return 0;
}

// ---- bow ----
struct Pt { double x, y; };
double seg_distance(Pt p, Pt a, Pt b) {
    const double dx = b.x - a.x, dy = b.y - a.y, n = dx * dx + dy * dy;
    const double t = n > 0 ? std::clamp(((p.x - a.x) * dx + (p.y - a.y) * dy) / n, 0.0, 1.0) : 0.0;
    const double ex = p.x - (a.x + t * dx), ey = p.y - (a.y + t * dy);
    return std::sqrt(ex * ex + ey * ey);
}
double polyline_distance(Pt p, const std::vector<Pt>& line) {
    double best = 1e18;
    for (std::size_t i = 0; i + 1 < line.size(); ++i) best = std::min(best, seg_distance(p, line[i], line[i + 1]));
    return best;
}
struct Fit {
    std::int64_t way = 0; std::uint32_t start = 0, end = 0;
    std::string highway; int rank = 0; bool ok = false; std::string error;
    double length = 0, dev = 0; std::size_t segments = 0, nodes = 0; std::uint64_t hash = 0; bool exception = false, relaxed = false;
    std::vector<Pt> samples;
};
std::vector<Fit> fit_block(const RoadGraph& world, const Block& b, const ReferenceFitLimits& limits) {
    const auto builder = RoadReferenceBuilder::make(world);
    if (!builder) { std::cerr << builder.error().message << '\n'; return {}; }
    const double lo_x = b.x0 * 1024.0 * 1000.0, hi_x = (b.x1 + 1) * 1024.0 * 1000.0, lo_y = b.y0 * 1024.0 * 1000.0, hi_y = (b.y1 + 1) * 1024.0 * 1000.0;
    std::map<std::int64_t, geom::PointMm> position;
    for (const auto& n : world.nodes) position.emplace(n.osm_id, n.position);
    std::vector<const RoadGraphWay*> mine;
    // A way is fitted by the sub-block that holds its FIRST node; all its nodes must lie in the world (the block plus a one
    // tile margin), so the degrees of its nodes are complete.
    const double ext = 1024.0 * 1000.0;
    for (const auto& w : world.ways) {
        const auto first = position.find(w.node_ids.front());
        if (first == position.end() || first->second.x < lo_x || first->second.x >= hi_x || first->second.y < lo_y || first->second.y >= hi_y) continue;
        bool inside = true;
        for (auto id : w.node_ids) {
            const auto it = position.find(id);
            if (it == position.end() || it->second.x < lo_x - ext || it->second.x > hi_x + ext || it->second.y < lo_y - ext || it->second.y > hi_y + ext) { inside = false; break; }
        }
        if (inside) mine.push_back(&w);
    }
    std::vector<std::vector<Fit>> parts(mine.size());
    std::atomic<std::size_t> next{0};
    const auto work = [&] {
        for (std::size_t i = next++; i < mine.size(); i = next++) {
            const auto& w = *mine[i];
            const auto fits = builder->fit_stretches(w.osm_id, limits);
            if (!fits) { Fit f; f.way = w.osm_id; f.highway = tag_of(w, "highway"); f.error = fits.error().message; parts[i].push_back(f); continue; }
            for (const auto& sf : *fits) {
                Fit f; f.way = w.osm_id; f.start = sf.stretch.start_ref; f.end = sf.stretch.end_ref; f.highway = tag_of(w, "highway");
                f.rank = w.rank; f.nodes = f.end - f.start + 1;
                if (!sf.reference) { f.error = sf.reference.error().message; parts[i].push_back(std::move(f)); continue; }
                f.ok = true; f.length = sf.reference->length_m; f.segments = sf.reference->segments.size(); f.exception = sf.reference->curvature_exception; f.relaxed = sf.reference->chord_bound_relaxed;
                std::vector<Pt> osm;
                for (std::uint32_t k = f.start; k <= f.end; ++k) {
                    const auto p = position.at(w.node_ids[k]);
                    osm.push_back({static_cast<double>(p.x) / 1000.0, static_cast<double>(p.y) / 1000.0});
                }
                std::uint64_t h = 1469598103934665603ull;
                for (double s = 0;; s += 1.0) {
                    const double st = std::min(s, f.length);
                    const auto pose = sf.reference->at(st);
                    if (pose) {
                        f.samples.push_back({pose->x, pose->y});
                        f.dev = std::max(f.dev, polyline_distance(f.samples.back(), osm));
                        h = (h ^ static_cast<std::uint64_t>(std::llround(pose->x * 1000))) * 1099511628211ull;
                        h = (h ^ static_cast<std::uint64_t>(std::llround(pose->y * 1000))) * 1099511628211ull;
                    }
                    if (st >= f.length) break;
                }
                f.hash = h;
                parts[i].push_back(std::move(f));
            }
        }
    };
    std::vector<std::thread> pool;
    for (unsigned t = 0; t < 8; ++t) pool.emplace_back(work);
    for (auto& t : pool) t.join();
    std::vector<Fit> out;
    for (auto& p : parts) for (auto& f : p) out.push_back(std::move(f));
    return out;
}
void report(const char* label, const std::vector<Fit>& fits) {
    std::size_t ok = 0, fail = 0, over3 = 0, over10 = 0, exceptions = 0, relaxed = 0; double max = 0; std::int64_t max_way = 0;
    std::map<std::string, std::array<std::size_t, 3>> by_class; // stretches, >3 m, >10 m
    std::vector<const Fit*> sorted;
    for (const auto& f : fits) {
        if (!f.ok) { ++fail; continue; }
        ++ok; exceptions += f.exception; relaxed += f.relaxed;
        auto& c = by_class[f.highway]; ++c[0];
        if (f.dev > 3) { ++over3; ++c[1]; }
        if (f.dev > 10) { ++over10; ++c[2]; }
        if (f.dev > max) { max = f.dev; max_way = f.way; }
        sorted.push_back(&f);
    }
    std::sort(sorted.begin(), sorted.end(), [](const Fit* a, const Fit* c) { return a->dev > c->dev; });
    std::cout << "== " << label << ": stretches fitted " << ok << ", failed " << fail << ", curvature exceptions " << exceptions << ", kept their unbounded fit (rounded fallback impossible) " << relaxed
              << "\n   lines more than 3 m from their OSM polyline: " << over3 << ", more than 10 m: " << over10 << ", max " << max
              << " m (way " << max_way << ")\n";
    std::vector<double> d;
    for (auto* f : sorted) d.push_back(f->dev);
    std::sort(d.begin(), d.end());
    const auto q = [&](double p) { return d.empty() ? 0.0 : d[std::min(d.size() - 1, static_cast<std::size_t>(p * static_cast<double>(d.size())))]; };
    std::cout << "   deviation quantiles: p50 " << q(.5) << "  p90 " << q(.9) << "  p99 " << q(.99) << "  p99.9 " << q(.999) << '\n';
    std::cout << "   >3 m / >10 m by class:";
    for (const auto& [h, c] : by_class) if (c[1]) std::cout << ' ' << h << '=' << c[1] << '/' << c[2] << " of " << c[0] << ';';
    std::cout << "\n   worst 12:";
    for (std::size_t i = 0; i < std::min<std::size_t>(12, sorted.size()); ++i)
        std::cout << "\n     way " << sorted[i]->way << " [" << sorted[i]->start << ".." << sorted[i]->end << "] " << sorted[i]->highway
                  << " nodes " << sorted[i]->nodes << " length " << sorted[i]->length << " dev " << sorted[i]->dev;
    std::cout << '\n';
}
int bow_report(const std::vector<Fit>& before, const std::vector<Fit>& after, double factor, double margin, const std::vector<std::int64_t>& ids) {
    std::cout << "ways with their first node in the block: stretches " << before.size() << '\n';
    report("BEFORE (no chord bound)", before);
    std::cout << "bound = " << factor << " x half width + " << margin << " m\n";
    report("AFTER (chord bound)", after);
    std::map<std::pair<std::int64_t, std::uint32_t>, const Fit*> old_by_key;
    for (const auto& f : before) old_by_key[{f.way, f.start}] = &f;
    std::size_t changed = 0, newly_failed = 0, newly_fitted = 0;
    std::map<std::string, std::size_t> changed_by_class;
    std::set<std::int64_t> changed_ways;
    struct Shift { std::int64_t way; std::uint32_t start; std::string highway; double max_shift; double length; };
    std::vector<Shift> major;
    for (const auto& f : after) {
        const auto it = old_by_key.find({f.way, f.start});
        if (it == old_by_key.end()) continue;
        const Fit& o = *it->second;
        if (o.ok == f.ok && o.hash == f.hash) continue;
        ++changed; ++changed_by_class[f.highway]; changed_ways.insert(f.way);
        if (o.ok && !f.ok) {
            ++newly_failed;
            std::cout << "   newly failed: way " << f.way << " [" << f.start << ".." << f.end << "] " << f.highway << " nodes " << f.nodes << " length-before " << o.length << " dev-before " << o.dev << " : " << f.error << '\n';
        }
        if (!o.ok && f.ok) ++newly_fitted;
        if (o.ok && f.ok) {
            // how far the new line moved: each new sample against the old line, and each old sample against the new one.
            double shift = 0;
            for (const auto& p : f.samples) shift = std::max(shift, polyline_distance(p, o.samples));
            for (const auto& p : o.samples) shift = std::max(shift, polyline_distance(p, f.samples));
            if (f.highway.rfind("motorway", 0) == 0 || f.highway.rfind("trunk", 0) == 0) major.push_back({f.way, f.start, f.highway, shift, f.length});
        }
    }
    std::cout << "== stretches whose fit changed: " << changed << " in " << changed_ways.size() << " ways (newly failed " << newly_failed << ", newly fitted " << newly_fitted << ") by class:";
    for (const auto& [h, n] : changed_by_class) std::cout << ' ' << h << '=' << n;
    std::sort(major.begin(), major.end(), [](const Shift& a, const Shift& c) { return a.max_shift > c.max_shift; });
    std::cout << "\n   motorway / trunk (and links) stretches changed: " << major.size();
    double mx = 0;
    for (const auto& m : major) mx = std::max(mx, m.max_shift);
    std::cout << ", max move of their reference line " << mx << " m\n";
    for (std::size_t i = 0; i < std::min<std::size_t>(10, major.size()); ++i)
        std::cout << "     way " << major[i].way << " [" << major[i].start << "] " << major[i].highway << " length " << major[i].length
                  << " moved " << major[i].max_shift << '\n';
    for (auto id : ids) for (const auto* set : {&before, &after})
        for (const auto& f : *set) if (f.way == id)
            std::cout << (set == &before ? "before" : "after ") << " way " << id << " [" << f.start << ".." << f.end << "] ok=" << f.ok
                      << " dev " << f.dev << " length " << f.length << " segments " << f.segments << " " << f.error << '\n';
    return 0;
}

int print_ways(const RoadGraph& world, const std::vector<std::int64_t>& ids) {
    for (auto id : ids) {
        auto it = std::lower_bound(world.ways.begin(), world.ways.end(), id, [](const auto& w, auto v) { return w.osm_id < v; });
        if (it == world.ways.end() || it->osm_id != id) { std::cout << id << ": absent\n"; continue; }
        std::cout << id << " nodes=" << it->node_ids.size() << '\n';
        for (const auto& [k, v] : it->tags) std::cout << "  " << k << "=" << v << '\n';
    }
    return 0;
}

} // namespace

int main(int argc, char** argv) {
    if (argc < 8) { std::cerr << "usage: g2m_road_census arrows|way|bow STORE SCOPE X0 X1 Y0 Y1 [WAY_ID...]\n"; return 2; }
    const std::string mode = argv[1];
    const Block b{std::atoi(argv[4]), std::atoi(argv[5]), std::atoi(argv[6]), std::atoi(argv[7])};
    if (mode == "bow") {
        // Sub-blocks of 8 x 8 tiles (the whole region's graph exceeds roads.graph's object limit): each is fitted on its own
        // world (sub-block plus one tile margin); a way counts in the sub-block that holds its first node.
        const double factor = argc > 8 ? std::atof(argv[8]) : 0.0, margin = argc > 9 ? std::atof(argv[9]) : 0.0;
        std::vector<std::int64_t> ids;
        for (int i = 10; i < argc; ++i) ids.push_back(std::atoll(argv[i]));
        ReferenceFitLimits base; base.allow_curvature_exceptions = true;
        auto bounded = base; bounded.chord_half_width_factor = factor; bounded.chord_margin_m = margin;
        std::vector<Fit> before, after;
        constexpr int kSub = 8;
        for (int sy = b.y0; sy <= b.y1; sy += kSub) for (int sx = b.x0; sx <= b.x1; sx += kSub) {
            const Block sub{sx, std::min(sx + kSub - 1, b.x1), sy, std::min(sy + kSub - 1, b.y1)};
            RoadGraph world;
            if (!load_world(argv[2], argv[3], sub, world)) return 1;
            auto fb = fit_block(world, sub, base), fa = fit_block(world, sub, bounded);
            for (auto& f : fb) before.push_back(std::move(f));
            for (auto& f : fa) after.push_back(std::move(f));
        }
        return bow_report(before, after, factor, margin, ids);
    }
    RoadGraph world;
    if (!load_world(argv[2], argv[3], b, world)) return 1;
    if (mode == "arrows") return arrows(world);
    if (mode == "way") {
        std::vector<std::int64_t> ids;
        for (int i = 8; i < argc; ++i) ids.push_back(std::atoll(argv[i]));
        return print_ways(world, ids);
    }
    std::cerr << "unknown mode\n";
    return 2;
}
