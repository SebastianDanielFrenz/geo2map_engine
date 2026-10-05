#include "g2m/layer/road_graph_tile.h"
#include "g2m/layer/src_osm.h"
#include "g2m/core/hash.h"
#include <algorithm>
#include <map>
#include <set>

namespace g2m {
namespace {
using Tags = osm::OsmBuilder::TagList;
struct NodeInput {
    std::int32_t lat, lon;
    Tags tags;
    friend bool operator==(const NodeInput&, const NodeInput&) = default;
};
struct WayInput {
    std::vector<std::int64_t> refs;
    Tags tags;
    friend bool operator==(const WayInput&, const WayInput&) = default;
};
Tags tags_of(const osm::OsmData& data, std::span<const osm::Tag> tags) {
    Tags result;
    for (const auto& tag : tags) result.emplace_back(data.str(tag.key), data.str(tag.value));
    return result;
}
const geom::PointMm& position(const RoadGraph& graph, std::int64_t id) {
    return std::lower_bound(graph.nodes.begin(), graph.nodes.end(), id,
        [](const auto& node, auto value) { return node.osm_id < value; })->position;
}
}

Result<RoadGraphTile> select_road_graph_tile(const RoadGraph& graph, const TileKey& key) {
    if (!key.valid() || key.level != 2) return make_error(ErrorCode::InvalidArgument, "roads.graph: level-2 target required");
    // Validate before exact predicates, additions or binary-search dereferences.
    const auto source_valid = validate_road_graph(graph); if (!source_valid) return source_valid.error();
    const auto exists = [&](auto id) {
        const auto it = std::lower_bound(graph.nodes.begin(), graph.nodes.end(), id,
            [](const auto& node, auto value) { return node.osm_id < value; });
        return it != graph.nodes.end() && it->osm_id == id;
    };
    for (const auto& edge : graph.edges) if (!exists(edge.from_node) || !exists(edge.to_node))
        return make_error(ErrorCode::Malformed, "roads.graph: unresolved source edge");
    const auto size = key.size_m() * 1000, halo = static_cast<std::int64_t>(kRoadGraphHaloM) * 1000;
    const auto west = key.min_easting() * 1000 - halo, south = key.min_northing() * 1000 - halo;
    std::set<std::int64_t> selected;
    for (const auto& edge : graph.edges) {
        const auto& a = position(graph, edge.from_node); const auto& b = position(graph, edge.to_node);
        if (geom::segment_touches_box(a.x, a.y, b.x, b.y, west, south, west + size + 2 * halo, south + size + 2 * halo))
            selected.insert(edge.way_id);
    }
    RoadGraphTile result; result.key = key;
    std::set<std::int64_t> node_ids;
    for (const auto& way : graph.ways) if (selected.contains(way.osm_id)) {
        result.graph.ways.push_back(way);
        for (auto id : way.node_ids) { if (!exists(id)) return make_error(ErrorCode::Malformed, "roads.graph: unresolved source ref"); node_ids.insert(id); }
        for (std::size_t i = 1; i < way.node_ids.size(); ++i) {
            if (way.node_ids[i - 1] == way.node_ids[i]) { ++result.graph.repeated_ref_pairs; continue; }
            result.graph.edges.push_back({way.osm_id, static_cast<std::uint32_t>(i - 1), way.node_ids[i - 1], way.node_ids[i]});
        }
    }
    for (const auto& node : graph.nodes) if (node_ids.contains(node.osm_id)) result.graph.nodes.push_back(node);
    for (std::size_t i = 0; i < result.graph.edges.size(); ++i) {
        const auto& edge = result.graph.edges[i];
        const auto& a = position(result.graph, edge.from_node); const auto& b = position(result.graph, edge.to_node);
        if (floor_div(floor_div(a.x + b.x, 2), size) == key.x && floor_div(floor_div(a.y + b.y, 2), size) == key.y)
            result.owned_edges.push_back(static_cast<std::uint32_t>(i));
    }
    const auto valid = validate_road_graph_tile(result); if (!valid) return valid.error();
    return result;
}

ContentHash RoadGraphDeriver::config_hash() const {
    return Sha256::of("g2m.roads.graph/1 complete-ways default-RoadStyle halo=128 midpoint-mm-owner raw-nine-neighbours node-tags=1 enc=5");
}
std::vector<InputTileRef> RoadGraphDeriver::inputs_for(const TileKey& target) const {
    std::vector<InputTileRef> result;
    if (!target.valid() || target.level != 2) return result;
    for (int dy = -1; dy <= 1; ++dy) for (int dx = -1; dx <= 1; ++dx) {
        auto key = target; key.x += dx; key.y += dy;
        if (key.valid()) result.push_back({std::string(kSrcOsmLayer), key});
    }
    std::sort(result.begin(), result.end()); return result;
}
Result<DeriveResult> RoadGraphDeriver::derive(const DeriveRequest& request) const {
    if (!request.target.valid() || request.target.level != 2)
        return make_error(ErrorCode::InvalidArgument, "roads.graph: level-2 target required");
    const auto expected = inputs_for(request.target);
    if (request.inputs.size() != expected.size()) return make_error(ErrorCode::InvalidArgument, "roads.graph: incomplete input set");
    std::set<InputTileRef> seen;
    std::map<std::int64_t, NodeInput> nodes;
    std::map<std::int64_t, WayInput> ways;
    std::size_t input_bytes = 0, ref_count = 0, tag_count = 0;
    for (const auto& input : request.inputs) {
        if (input.body.size() > kMaxBodyRawSize - input_bytes)
            return make_error(ErrorCode::LimitExceeded, "roads.graph: total source bytes");
        input_bytes += input.body.size();
        if (!std::binary_search(expected.begin(), expected.end(), input.ref) || !seen.insert(input.ref).second)
            return make_error(ErrorCode::InvalidArgument, "roads.graph: unexpected or duplicate input ref");
        if (!input.header) {
            if (!input.body.empty()) return make_error(ErrorCode::Malformed, "roads.graph: absent input has bytes");
            continue;
        }
        const auto& header = *input.header;
        if (header.key != input.ref.key || header.layer != kSrcOsmLayer || header.layer_version != 1 ||
            header.body_size != input.body.size() || header.body_hash != content_hash(input.body))
            return make_error(ErrorCode::Malformed, "roads.graph: source header mismatch");
        auto body = decode_body(input.body); if (!body) return body.error();
        auto osm_tile = decode_src_osm(*body); if (!osm_tile) return osm_tile.error();
        const auto& data = osm_tile->data;
        for (const auto& node : data.nodes) {
            NodeInput value{node.lat, node.lon, tags_of(data, data.tags_of(node))};
            const auto [it, inserted] = nodes.emplace(node.id, value);
            if (!inserted && it->second != value) return make_error(ErrorCode::Malformed, "roads.graph: conflicting node copies");
            if (inserted) tag_count += value.tags.size();
            if (nodes.size() > 1000000 || tag_count > 1000000)
                return make_error(ErrorCode::LimitExceeded, "roads.graph: merged nodes/tags");
        }
        for (const auto& way : data.ways) {
            const auto refs = data.refs_of(way);
            WayInput value{{refs.begin(), refs.end()}, tags_of(data, data.tags_of(way))};
            const auto [it, inserted] = ways.emplace(way.id, value);
            if (!inserted && it->second != value) return make_error(ErrorCode::Malformed, "roads.graph: conflicting way copies");
            if (inserted) { ref_count += value.refs.size(); tag_count += value.tags.size(); }
            if (ways.size() > 100000 || ref_count > 4000000 || tag_count > 1000000)
                return make_error(ErrorCode::LimitExceeded, "roads.graph: merged ways/refs/tags");
        }
    }
    osm::OsmBuilder builder;
    for (const auto& [id, node] : nodes) builder.add_node(id, node.lat, node.lon, node.tags);
    for (const auto& [id, way] : ways) builder.add_way(id, way.refs, way.tags);
    auto merged = builder.build(); if (!merged) return merged.error();
    auto graph = build_road_graph(*merged, request.target.zone); if (!graph) return graph.error();
    auto tile = select_road_graph_tile(*graph, request.target); if (!tile) return tile.error();
    auto body = encode_road_graph(*tile); if (!body) return body.error();
    auto bytes = encode_body(*body); if (!bytes) return bytes.error();
    return DeriveResult{std::move(*bytes), VerticalDatum::None, 0};
}
} // namespace g2m
