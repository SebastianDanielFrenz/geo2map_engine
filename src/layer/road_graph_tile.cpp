#include "g2m/layer/road_graph_tile.h"
#include "g2m/core/bytes.h"
#include "g2m/core/hash.h"
#include <algorithm>
#include <limits>

namespace g2m {
namespace {
constexpr std::size_t kMaxNodes = 1000000, kMaxWays = 100000, kMaxRefs = 4000000, kMaxTags = 1000000;
constexpr std::size_t kMaxString = 65535;
const RoadGraphNode* find_node(const RoadGraph& graph, std::int64_t id) {
    auto it = std::lower_bound(graph.nodes.begin(), graph.nodes.end(), id,
        [](const auto& n, auto value) { return n.osm_id < value; });
    return it != graph.nodes.end() && it->osm_id == id ? &*it : nullptr;
}
bool owned(const RoadGraphTile& tile, const RoadGraphEdge& edge) {
    const auto& a = find_node(tile.graph, edge.from_node)->position;
    const auto& b = find_node(tile.graph, edge.to_node)->position;
    const auto x = floor_div(a.x + b.x, 2), y = floor_div(a.y + b.y, 2);
    const auto size = tile.key.size_m() * 1000;
    return floor_div(x, size) == tile.key.x && floor_div(y, size) == tile.key.y;
}
Error malformed(std::string message) { return make_error(ErrorCode::Malformed, "roads.graph: " + message); }
void string(ByteWriter& w, const std::string& s) { w.varint(s.size()); w.bytes(s.data(), s.size()); }
Result<std::string> string(ByteReader& r) {
    const auto n = r.varint();
    if (!r.ok() || n > kMaxString || n > r.remaining()) return malformed("invalid string size");
    return std::string(r.string(static_cast<std::size_t>(n)));
}
}

Result<void> validate_road_graph(const RoadGraph& g) {
    if (g.nodes.size() > kMaxNodes || g.ways.size() > kMaxWays || g.edges.size() > kMaxRefs ||
        g.far_zone_ways > kMaxWays || g.repeated_ref_pairs > kMaxRefs)
        return make_error(ErrorCode::LimitExceeded, "roads.graph: object count");
    for (std::size_t i = 0; i < g.nodes.size(); ++i) {
        const auto& n = g.nodes[i];
        constexpr auto bound = std::int64_t{1} << 62;
        if ((i && g.nodes[i - 1].osm_id >= n.osm_id) || n.position.x <= -bound || n.position.x >= bound ||
            n.position.y <= -bound || n.position.y >= bound) return malformed("node order or coordinate range");
    }
    std::size_t refs = 0, tags = 0, tag_bytes = 0, edge_index = 0, repeats = 0;
    std::vector<bool> used(g.nodes.size(), false);
    for (std::size_t i = 0; i < g.ways.size(); ++i) {
        const auto& way = g.ways[i];
        if ((i && g.ways[i - 1].osm_id >= way.osm_id) || way.node_ids.size() < 2 || way.rank < 0 || way.rank > 255)
            return malformed("way order, rank or reference count");
        if (way.node_ids.size() > kMaxRefs - refs || way.tags.size() > kMaxTags - tags)
            return make_error(ErrorCode::LimitExceeded, "roads.graph: pooled count");
        refs += way.node_ids.size(); tags += way.tags.size();
        for (std::size_t j = 0; j < way.tags.size(); ++j) {
            const auto& [key, value] = way.tags[j];
            if (key.empty() || key.size() > kMaxString || value.size() > kMaxString ||
                (j && way.tags[j - 1].first >= key)) return malformed("noncanonical tags");
            if (key.size() + value.size() > kMaxSectionRawSize - tag_bytes)
                return make_error(ErrorCode::LimitExceeded, "roads.graph: pooled string bytes");
            tag_bytes += key.size() + value.size();
        }
        for (std::size_t j = 0; j < way.node_ids.size(); ++j) {
            const auto* n = find_node(g, way.node_ids[j]);
            if (!n) return malformed("unresolved node");
            used[static_cast<std::size_t>(n - g.nodes.data())] = true;
            if (!j) continue;
            if (way.node_ids[j - 1] == way.node_ids[j]) { ++repeats; continue; }
            const RoadGraphEdge expected{way.osm_id, static_cast<std::uint32_t>(j - 1), way.node_ids[j - 1], way.node_ids[j]};
            if (edge_index >= g.edges.size() || g.edges[edge_index++] != expected) return malformed("edge/reference mismatch");
        }
    }
    if (edge_index != g.edges.size() || repeats != g.repeated_ref_pairs ||
        std::find(used.begin(), used.end(), false) != used.end()) return malformed("extra edges/nodes or repeat count");
    return {};
}
Result<void> validate_road_graph_tile(const RoadGraphTile& tile) {
    if (!tile.key.valid() || tile.key.level != 2 || tile.halo_m != kRoadGraphHaloM)
        return malformed("invalid key or halo");
    const auto& g = tile.graph;
    const auto valid = validate_road_graph(g); if (!valid) return valid.error();
    std::size_t owned_index = 0;
    for (std::size_t i = 0; i < g.edges.size(); ++i) if (owned(tile, g.edges[i])) {
        if (owned_index >= tile.owned_edges.size() || tile.owned_edges[owned_index++] != i)
            return malformed("noncanonical ownership");
    }
    if (owned_index != tile.owned_edges.size()) return malformed("extra ownership indices");
    return {};
}

Result<TileBody> encode_road_graph(const RoadGraphTile& tile) {
    const auto valid = validate_road_graph_tile(tile); if (!valid) return valid.error();
    ByteWriter w;
    w.u64(tile.key.packed()); w.u32(tile.halo_m);
    const auto& g = tile.graph;
    w.varint(g.far_zone_ways); w.varint(g.repeated_ref_pairs);
    w.varint(g.nodes.size());
    for (const auto& n : g.nodes) { w.svarint(n.osm_id); w.svarint(n.position.x); w.svarint(n.position.y); }
    w.varint(g.ways.size());
    for (const auto& way : g.ways) {
        w.svarint(way.osm_id); w.u8(static_cast<std::uint8_t>(way.rank)); w.varint(way.node_ids.size());
        for (auto id : way.node_ids) w.svarint(id);
        w.varint(way.tags.size());
        for (const auto& [key, value] : way.tags) { string(w, key); string(w, value); }
    }
    // Edges are reconstructed canonically from original references. Only
    // ownership indices are stored; this avoids redundant, forgeable topology.
    w.varint(tile.owned_edges.size()); for (auto index : tile.owned_edges) w.varint(index);
    if (w.size() > kMaxSectionRawSize) return make_error(ErrorCode::LimitExceeded, "roads.graph: section size");
    return TileBody{kRoadGraphEncoding, 1, {{1, SectionCodec::Zstd, w.take()}}};
}

Result<RoadGraphTile> decode_road_graph(const TileBody& body) {
    if (body.encoding != kRoadGraphEncoding || body.encoding_version != 1)
        return make_error(ErrorCode::Unsupported, "roads.graph: encoding/version");
    if (body.sections.size() != 1 || body.sections[0].type != 1 || body.sections[0].data.size() > kMaxSectionRawSize)
        return malformed("sections");
    ByteReader r(body.sections[0].data);
    const auto key = TileKey::unpack(r.u64()); if (!key) return malformed("key");
    RoadGraphTile tile; tile.key = *key; tile.halo_m = r.u32();
    auto& g = tile.graph;
    const auto far = r.varint(), repeat = r.varint();
    if (!r.ok() || far > kMaxWays || repeat > kMaxRefs) return malformed("diagnostic counts");
    g.far_zone_ways = static_cast<std::size_t>(far); g.repeated_ref_pairs = static_cast<std::size_t>(repeat);
    const auto nodes = r.varint();
    if (!r.ok() || nodes > kMaxNodes || nodes > r.remaining() / 3) return malformed("node count");
    g.nodes.reserve(static_cast<std::size_t>(nodes));
    for (std::uint64_t i = 0; i < nodes; ++i) {
        RoadGraphNode n; n.osm_id = r.svarint(); n.position.x = r.svarint(); n.position.y = r.svarint();
        if (!r.ok()) return malformed("truncated nodes"); g.nodes.push_back(n);
    }
    const auto ways = r.varint();
    if (!r.ok() || ways > kMaxWays || ways > r.remaining() / 6) return malformed("way count");
    g.ways.reserve(static_cast<std::size_t>(ways));
    std::size_t refs_total = 0, tags_total = 0;
    for (std::uint64_t i = 0; i < ways; ++i) {
        RoadGraphWay way; way.osm_id = r.svarint(); way.rank = r.u8();
        const auto refs = r.varint();
        if (!r.ok() || refs < 2 || refs > kMaxRefs - refs_total || refs > r.remaining()) return malformed("reference count");
        refs_total += static_cast<std::size_t>(refs); way.node_ids.reserve(static_cast<std::size_t>(refs));
        for (std::uint64_t j = 0; j < refs; ++j) way.node_ids.push_back(r.svarint());
        const auto tags = r.varint();
        if (!r.ok() || tags > kMaxTags - tags_total || tags > r.remaining() / 2) return malformed("tag count");
        tags_total += static_cast<std::size_t>(tags);
        for (std::uint64_t j = 0; j < tags; ++j) {
            auto name = string(r); if (!name) return name.error();
            auto value = string(r); if (!value) return value.error();
            way.tags.emplace_back(std::move(*name), std::move(*value));
        }
        for (std::size_t j = 1; j < way.node_ids.size(); ++j) if (way.node_ids[j - 1] != way.node_ids[j])
            g.edges.push_back({way.osm_id, static_cast<std::uint32_t>(j - 1), way.node_ids[j - 1], way.node_ids[j]});
        g.ways.push_back(std::move(way));
    }
    const auto owners = r.varint();
    if (!r.ok() || owners > g.edges.size() || owners > r.remaining()) return malformed("ownership count");
    for (std::uint64_t i = 0; i < owners; ++i) {
        const auto index = r.varint(); if (!r.ok() || index > std::numeric_limits<std::uint32_t>::max()) return malformed("ownership index");
        tile.owned_edges.push_back(static_cast<std::uint32_t>(index));
    }
    if (!r.at_end()) return malformed("truncated or trailing bytes");
    const auto valid = validate_road_graph_tile(tile); if (!valid) return valid.error();
    return tile;
}

LayerDesc road_graph_desc() {
    return {{std::string(kRoadGraphLayer), 1}, LayerKind::Baked, kRoadGraphEncoding, 1, level_bit(2),
        {std::string(kSrcOsmLayer)}, "complete road ways, canonical OSM topology and midpoint edge ownership"};
}
} // namespace g2m
