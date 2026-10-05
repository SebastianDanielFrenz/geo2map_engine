#include "g2m/layer/road_graph.h"
#include "g2m/core/geom/project.h"
#include <algorithm>
#include <map>

namespace g2m {
bool road_node_tag_key_allowed(std::string_view key) {
    static constexpr std::string_view kKeys[] = {"barrier", "crossing", "crossing:markings", "direction", "height",
        "highway", "kerb", "length", "traffic_calming", "traffic_calming:length", "traffic_signals:direction", "width"};
    return std::binary_search(std::begin(kKeys), std::end(kKeys), key);
}
Result<RoadGraph> build_road_graph(const osm::OsmData& input, geo::UtmZone zone, const RoadStyle& style) {
    const auto valid = input.validate();
    if (!valid) return valid.error();
    if (zone.zone < 1 || zone.zone > 60)
        return make_error(ErrorCode::InvalidArgument, "road graph: invalid UTM zone");
    RoadGraph result;
    std::map<std::int64_t, RoadGraphNode> nodes;
    const geo::Utm utm(geo::kGRS80);
    for (const auto& way : input.ways) {
        const auto tags = input.tags_of(way);
        const auto highway = input.tag(tags, "highway");
        const auto area = input.tag(tags, "area");
        const auto* classification = highway ? style.find_highway(*highway) : nullptr;
        if (!classification || (area && *area == "yes")) continue;
        const auto refs = input.refs_of(way);
        if (refs.size() < 2) continue;
        std::vector<RoadGraphNode> projected;
        projected.reserve(refs.size());
        bool far = false;
        for (const auto id : refs) {
            const auto* node = input.find_node(id); // canonical validation proves presence
            const auto position = geom::project_e7_to_mm(utm, zone, node->lat, node->lon);
            if (!position) { far = true; break; }
            RoadGraphNode out{id, *position, {}};
            for (const auto& tag : input.tags_of(*node)) {
                const auto& key = input.str(tag.key);
                if (road_node_tag_key_allowed(key)) out.tags.emplace_back(key, input.str(tag.value));
            }
            std::sort(out.tags.begin(), out.tags.end());
            // A duplicated key (non-canonical input) keeps the smallest value so the list stays strictly ascending.
            out.tags.erase(std::unique(out.tags.begin(), out.tags.end(),
                [](const auto& a, const auto& b) { return a.first == b.first; }), out.tags.end());
            projected.push_back(std::move(out));
        }
        if (far) { ++result.far_zone_ways; continue; }
        RoadGraphWay road;
        road.osm_id = way.id;
        road.rank = classification->rank;
        road.node_ids.assign(refs.begin(), refs.end());
        for (const auto& tag : tags)
            road.tags.emplace_back(input.str(tag.key), input.str(tag.value));
        for (auto& node : projected) nodes.emplace(node.osm_id, std::move(node));
        for (std::size_t i = 0; i + 1 < refs.size(); ++i) {
            if (refs[i] == refs[i + 1]) { ++result.repeated_ref_pairs; continue; }
            result.edges.push_back({way.id, static_cast<std::uint32_t>(i), refs[i], refs[i + 1]});
        }
        result.ways.push_back(std::move(road));
    }
    result.nodes.reserve(nodes.size());
    for (auto& [id, node] : nodes) result.nodes.push_back(std::move(node));
    return result;
}
} // namespace g2m
