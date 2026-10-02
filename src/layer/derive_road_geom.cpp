#include "g2m/layer/road_geom_tile.h"
#include "g2m/layer/road_dem_dependencies.h"
#include "g2m/layer/terrain_layers.h"
#include "g2m/core/hash.h"
#include <algorithm>
#include <limits>
#include <set>
namespace g2m {
namespace {
Result<RoadGraphTile> graph_input(const DeriveRequest& request) {
    if (!request.target.valid() || request.target.level != 2) return make_error(ErrorCode::InvalidArgument, "roads.geom: level-2 target required");
    const InputTile* graph = nullptr;
    for (const auto& input : request.inputs) if (input.ref.layer == kRoadGraphLayer && input.ref.key == request.target) {
        if (graph) return make_error(ErrorCode::Malformed, "roads.geom: duplicate graph"); graph = &input;
    }
    if (!graph || !graph->header) return make_error(ErrorCode::NotFound, "roads.geom: graph dependency absent");
    if (graph->header->key != graph->ref.key || graph->header->layer != graph->ref.layer || graph->header->layer_version != 1 || graph->header->body_size != graph->body.size() || graph->header->body_hash != content_hash(graph->body))
        return make_error(ErrorCode::Malformed, "roads.geom: graph header mismatch");
    const auto body = decode_body(graph->body); if (!body) return body.error();
    const auto tile = decode_road_graph(*body); if (!tile) return tile.error();
    if (tile->key != request.target) return make_error(ErrorCode::Malformed, "roads.geom: graph key mismatch"); return tile;
}
}
LayerDesc road_geom_desc() {
    return {{std::string(kRoadGeomLayer), 1}, LayerKind::Baked, kRoadGeomEncoding, 1, level_bit(2),
        {std::string(kRoadGraphLayer), std::string(kElevBaseLayer)}, "complete parametric road profiles and explicit declined stretches"};
}
ContentHash RoadGeomDeriver::config_hash() const {
    return Sha256::of("g2m.roads.geom/1 class-policy-2026-10-02 spacing=5 max-samples=2048 iterations=4096 curvature-exceptions=true sloping-plane-r3 approach=32 height-correction=.25 merge-agreement=.5 incline-soft=5 dependency-margin=65 coverage=graph");
}
std::vector<InputTileRef> RoadGeomDeriver::inputs_for(const TileKey& target) const {
    if (!target.valid() || target.level != 2) return {}; return {{std::string(kRoadGraphLayer), target}};
}
Result<std::vector<InputTileRef>> RoadGeomDeriver::additional_inputs_for(const DeriveRequest& request) const {
    const auto graph = graph_input(request); if (!graph) return graph.error();
    return road_dem_dependencies(graph->graph, request.target.zone);
}
Result<DeriveResult> RoadGeomDeriver::derive(const DeriveRequest& request) const {
    const auto graph = graph_input(request); if (!graph) return graph.error();
    auto dependencies = road_dem_dependencies(graph->graph, request.target.zone); if (!dependencies) return dependencies.error();
    auto expected = *dependencies; expected.push_back({std::string(kRoadGraphLayer), request.target}); std::sort(expected.begin(), expected.end());
    if (request.inputs.size() != expected.size()) return make_error(ErrorCode::InvalidArgument, "roads.geom: incomplete dependency set");
    std::set<InputTileRef> seen; std::vector<HeightTile> heights; heights.reserve(dependencies->size());
    std::optional<VerticalDatum> datum; std::size_t bytes = 0;
    for (const auto& input : request.inputs) {
        if (!std::binary_search(expected.begin(), expected.end(), input.ref) || !seen.insert(input.ref).second)
            return make_error(ErrorCode::Malformed, "roads.geom: unexpected or duplicate dependency");
        if (input.body.size() > kMaxBodyRawSize - bytes) return make_error(ErrorCode::LimitExceeded, "roads.geom: input byte budget"); bytes += input.body.size();
        if (!input.header) return make_error(ErrorCode::NotFound, "roads.geom: required dependency absent");
        if (input.header->key != input.ref.key || input.header->layer != input.ref.layer || input.header->layer_version != 1 || input.header->body_size != input.body.size() || input.header->body_hash != content_hash(input.body))
            return make_error(ErrorCode::Malformed, "roads.geom: dependency header mismatch");
        if (input.ref.layer == kRoadGraphLayer) continue;
        if (input.header->datum != VerticalDatum::DHHN2016 && input.header->datum != VerticalDatum::EGM2008)
            return make_error(ErrorCode::Unsupported, "roads.geom: unknown elevation datum");
        if (datum && *datum != input.header->datum) return make_error(ErrorCode::Malformed, "roads.geom: mixed elevation datums"); datum = input.header->datum;
        const auto body = decode_body(input.body); if (!body) return body.error();
        heights.emplace_back(); const auto decoded = decode_height_tile(*body, input.ref.key, heights.back()); if (!decoded) return decoded.error();
        for (auto& height : heights.back().h) if (height != kHeightNoData) {
            const auto adjusted = static_cast<std::int64_t>(height) + input.header->height_offset;
            if (adjusted <= std::numeric_limits<std::int32_t>::min() || adjusted > std::numeric_limits<std::int32_t>::max())
                return make_error(ErrorCode::LimitExceeded, "roads.geom: elevation offset overflow");
            height = static_cast<std::int32_t>(adjusted);
        }
    }
    std::vector<const HeightTile*> views; for (const auto& height : heights) views.push_back(&height);
    const auto dem = RoadDemSampler::make(views); if (!dem) return dem.error();
    RoadProfileLimits limits; limits.reference.allow_curvature_exceptions = true;
    const auto geometry = build_road_geom_tile(*graph, *dem, limits); if (!geometry) return geometry.error();
    const auto body = encode_road_geom(*geometry); if (!body) return body.error();
    auto encoded = encode_body(*body); if (!encoded) return encoded.error();
    return DeriveResult{std::move(*encoded), datum.value_or(VerticalDatum::None), 0};
}
}
