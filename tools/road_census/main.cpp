// tools/road_census/main.cpp - g2m_road_census: read-only censuses over the road graphs of a block of level-2 tiles of an
// existing immutable source store (never writes to it). The graphs of all tiles are derived in memory and merged into one
// world graph, so node degrees and join headings are the world's, not a tile's.
//
//   g2m_road_census arrows STORE SCOPE X0 X1 Y0 Y1
//   g2m_road_census way    STORE SCOPE X0 X1 Y0 Y1 WAY_ID...        (tags of single ways)
//
// X0..Y1 are inclusive level-2 tile indices on UTM zone 32 (tile edge 1024 m: E 455-490 km is x 444..478, N 5540-5585 km is
// y 5410..5454).
// `arrows`: the turn:lanes census of street fidelity (vault G2M-023/G2M-024): per way and lane which glyphs the ribbons used
// to paint (the old substring rule, reproduced here as `legacy_glyphs`) and which they paint now (lane_arrow_glyphs).
#include "g2m/core/tile_key.h"
#include "g2m/layer/road_cross_section.h"
#include "g2m/layer/road_graph_tile.h"
#include "g2m/layer/src_osm.h"
#include "g2m/server/tile_store.h"
#include "g2m/world/road_ribbons.h"

#include <algorithm>
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
    if (argc < 8) { std::cerr << "usage: g2m_road_census arrows|way STORE SCOPE X0 X1 Y0 Y1 [WAY_ID...]\n"; return 2; }
    const std::string mode = argv[1];
    const Block b{std::atoi(argv[4]), std::atoi(argv[5]), std::atoi(argv[6]), std::atoi(argv[7])};
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
