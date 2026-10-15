#include "g2m/layer/road_geom_tile.h"
#include "g2m/layer/road_dem_dependencies.h"
#include "g2m/layer/terrain_layers.h"
#include "g2m/core/hash.h"
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <limits>
#include <optional>
#include <span>
#include <string>
#include <set>
namespace g2m {
namespace {
Result<RoadGraphTile> decode_graph_input(const InputTile& graph) {
    if (!graph.header) return make_error(ErrorCode::NotFound, "roads.geom: graph dependency absent");
    if (graph.header->key != graph.ref.key || graph.header->layer != graph.ref.layer || graph.header->layer_version != 1 || graph.header->body_size != graph.body.size() || graph.header->body_hash != content_hash(graph.body))
        return make_error(ErrorCode::Malformed, "roads.geom: graph header mismatch");
    const auto body = decode_body(graph.body); if (!body) return body.error();
    auto tile = decode_road_graph(*body); if (!tile) return tile.error();
    if (tile->key != graph.ref.key) return make_error(ErrorCode::Malformed, "roads.geom: graph key mismatch");
    return tile;
}
Result<RoadGraphTile> graph_input(const DeriveRequest& request) {
    if (!request.target.valid() || request.target.level != 2) return make_error(ErrorCode::InvalidArgument, "roads.geom: level-2 target required");
    const InputTile* graph = nullptr;
    for (const auto& input : request.inputs) if (input.ref.layer == kRoadGraphLayer && input.ref.key == request.target) {
        if (graph) return make_error(ErrorCode::Malformed, "roads.geom: duplicate graph"); graph = &input;
    }
    if (!graph) return make_error(ErrorCode::NotFound, "roads.geom: graph dependency absent");
    return decode_graph_input(*graph);
}
// Stage 1: DEM tiles under the complete ways plus the roads.graph tiles that hold
// every way incident to their nodes (stretch context), sorted and unique.
Result<std::vector<InputTileRef>> dependencies(const RoadGraphTile& graph, geo::UtmZone zone) {
    auto dem = road_dem_dependencies(graph.graph, zone); if (!dem) return dem.error();
    auto context = road_context_dependencies(graph.graph, zone, &graph.key); if (!context) return context.error();
    dem->insert(dem->end(), context->begin(), context->end()); std::sort(dem->begin(), dem->end());
    return dem;
}
// Stage 2 (decision D35): the stage-1 context tiles found among `inputs` (nullopt while any is still missing, i.e. in
// stage 1 itself; an explicitly absent tile counts as present and adds nothing), merged with the tile graph, give the
// bridge ways incident to the tile's nodes; road_bridge_dependencies names their DEM and their nodes' graph tiles.
Result<std::optional<std::vector<InputTileRef>>> bridge_dependencies(const RoadGraphTile& graph, geo::UtmZone zone,
    std::span<const InputTile> inputs) {
    const auto context_refs = road_context_dependencies(graph.graph, zone, &graph.key); if (!context_refs) return context_refs.error();
    std::vector<RoadGraphTile> tiles;
    for (const auto& ref : *context_refs) {
        const InputTile* found = nullptr;
        for (const auto& input : inputs) if (input.ref == ref) { found = &input; break; }
        if (!found) return std::optional<std::vector<InputTileRef>>{};
        if (!found->header) continue;
        auto tile = decode_graph_input(*found); if (!tile) return tile.error();
        tiles.push_back(std::move(*tile));
    }
    std::vector<const RoadGraph*> graphs{&graph.graph};
    for (const auto& tile : tiles) graphs.push_back(&tile.graph);
    const auto context = merge_road_graphs(graphs); if (!context) return context.error();
    auto bridges = road_bridge_dependencies(graph.graph, *context, zone, &graph.key); if (!bridges) return bridges.error();
    return std::optional<std::vector<InputTileRef>>(std::move(*bridges));
}
// Stage 1 plus, once the context is resolved, stage 2: sorted and unique.
Result<std::vector<InputTileRef>> all_dependencies(const RoadGraphTile& graph, geo::UtmZone zone, std::span<const InputTile> inputs,
    bool* complete = nullptr) {
    auto deps = dependencies(graph, zone); if (!deps) return deps.error();
    const auto bridges = bridge_dependencies(graph, zone, inputs); if (!bridges) return bridges.error();
    if (complete) *complete = bridges->has_value();
    if (*bridges) {
        deps->insert(deps->end(), (*bridges)->begin(), (*bridges)->end());
        std::sort(deps->begin(), deps->end()); deps->erase(std::unique(deps->begin(), deps->end()), deps->end());
    }
    return deps;
}
}
LayerDesc road_geom_desc() {
    return {{std::string(kRoadGeomLayer), 1}, LayerKind::Baked, kRoadGeomEncoding, 1, level_bit(2),
        {std::string(kRoadGraphLayer), std::string(kElevBaseLayer)}, "complete parametric road profiles and explicit declined stretches"};
}
namespace {
// An option value in the config text: integral values as integers, anything else with 17 significant digits.
std::string option_value(double v) {
    if (v > -1e15 && v < 1e15 && static_cast<double>(static_cast<long long>(v)) == v) return std::to_string(static_cast<long long>(v));
    char buf[64]; std::snprintf(buf, sizeof buf, "%.17g", v); return buf;
}
}
std::string road_geom_config_text() {
    // xs=1: the cross-section and feature sections (P1); enc=8: junctions, furniture and provenance (P2 slice 8) with
    // tunnel floors (D47). The
    // rules hash ties a release to the exact de.json bytes (junction and furniture blocks included); every value of the
    // served completion options is part of the text (the relaxed completion limits are code constants, completion=1).
    const auto& rules_hash = builtin_road_rules_de().content_sha256;
    const auto c = served_road_completion_options();
    return "g2m.roads.geom/1 class-policy-2026-10-02 spacing=5 max-samples=2048 iterations=4096 curvature-exceptions=true sloping-plane-r3 approach=32 height-correction=.25 merge-agreement=.5 incline-soft=5 dependency-margin=65 coverage=graph"
        " xs=1 enc=8 rules=de:" + to_hex(std::span<const std::uint8_t>(rules_hash.data(), rules_hash.size())) + " join-bisector=1 context=nodes"
        " junction=1 furniture=1 completion=1 completion-retry=" + option_value(c.retry_spacing_m) + "/" + std::to_string(c.retry_iterations) +
        " completion-abutment=" + option_value(c.abutment_ground_m) + "/" + option_value(c.abutment_path_m) +
        " completion-ground=" + option_value(c.ground_spacing_m) + "/" + option_value(c.ground_ramp_m) + " strict-anchor=1 ring-nesting=2mm"
        " bridge-far-end=1"    // D35: two discovery stages, the DEM and graph tiles of every incident bridge's far end
        " ground-ease=8/0.05"  // D43: ground completion eases contradicting end grades toward the chord in eighths, mismatch <= 0.05
        " corner-fillet=1"     // D44: kerb returns on block corners (radii from the rules), per-corner fallback to sharp
        " width-taper=5"       // D46: a width step between two continuing stubs tapers 1:5
        // D47 (terrain migration phase 4): tunnel floors (spacing/max grade/min radius/chord threshold, 0 = DEM floor
        // for every length) and the accepted-bridge anchors with the walk over every incident bridge edge.
        " tunnel-floor=" + std::string(c.tunnel_floors ? "" : "off/") + option_value(c.tunnel_spacing_m) + "/" + option_value(c.tunnel_max_grade) + "/" + option_value(c.tunnel_min_radius_m) +
        "/chord:" + option_value(c.tunnel_chord_min_length_m) +
        " bridge-anchor-reuse=" + (c.bridge_anchor_reuse ? "1" : "0") +
        // D48 (terrain migration phase 5): the walk budget above is 200 m; its DEM and graph reads are covered by the
        // stage-2 squares of this half-width around every node of the incident bridges.
        " bridge-walk-margin=" + std::to_string(kRoadBridgeWalkMarginMm / 1000);
}
ContentHash RoadGeomDeriver::config_hash() const { return Sha256::of(road_geom_config_text()); }
std::vector<InputTileRef> RoadGeomDeriver::inputs_for(const TileKey& target) const {
    if (!target.valid() || target.level != 2) return {}; return {{std::string(kRoadGraphLayer), target}};
}
Result<std::vector<InputTileRef>> RoadGeomDeriver::additional_inputs_for(const DeriveRequest& request) const {
    const auto graph = graph_input(request); if (!graph) return graph.error();
    return all_dependencies(*graph, request.target.zone, request.inputs);
}
Result<DeriveResult> RoadGeomDeriver::derive(const DeriveRequest& request) const {
    const auto graph = graph_input(request); if (!graph) return graph.error();
    bool complete = false;
    auto expected = all_dependencies(*graph, request.target.zone, request.inputs, &complete); if (!expected) return expected.error();
    if (!complete) return make_error(ErrorCode::InvalidArgument, "roads.geom: incomplete dependency set");
    expected->push_back({std::string(kRoadGraphLayer), request.target}); std::sort(expected->begin(), expected->end());
    if (request.inputs.size() != expected->size()) return make_error(ErrorCode::InvalidArgument, "roads.geom: incomplete dependency set");
    std::set<InputTileRef> seen; std::vector<HeightTile> heights; std::vector<RoadGraphTile> context_tiles;
    std::optional<VerticalDatum> datum; std::size_t bytes = 0;
    for (const auto& input : request.inputs) {
        if (!std::binary_search(expected->begin(), expected->end(), input.ref) || !seen.insert(input.ref).second)
            return make_error(ErrorCode::Malformed, "roads.geom: unexpected or duplicate dependency");
        if (input.body.size() > kMaxBodyRawSize - bytes) return make_error(ErrorCode::LimitExceeded, "roads.geom: input byte budget"); bytes += input.body.size();
        // A context graph tile outside the release's coverage is explicitly absent: no way is known there.
        if (input.ref.layer == kRoadGraphLayer && input.ref.key != request.target && !input.header) continue;
        if (!input.header) return make_error(ErrorCode::NotFound, "roads.geom: required dependency absent");
        if (input.header->key != input.ref.key || input.header->layer != input.ref.layer || input.header->layer_version != 1 || input.header->body_size != input.body.size() || input.header->body_hash != content_hash(input.body))
            return make_error(ErrorCode::Malformed, "roads.geom: dependency header mismatch");
        if (input.ref.layer == kRoadGraphLayer) {
            if (input.ref.key == request.target) continue;
            auto tile = decode_graph_input(input); if (!tile) return tile.error();
            context_tiles.push_back(std::move(*tile)); continue;
        }
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
    std::vector<const RoadGraph*> graphs{&graph->graph};
    for (const auto& tile : context_tiles) graphs.push_back(&tile.graph);
    const auto context = merge_road_graphs(graphs); if (!context) return context.error();
    std::vector<const HeightTile*> views; for (const auto& height : heights) views.push_back(&height);
    const auto dem = RoadDemSampler::make(views); if (!dem) return dem.error();
    RoadProfileLimits limits; limits.reference.allow_curvature_exceptions = true;
    const auto geometry = build_road_geom_tile(*graph, *dem, limits, &*context, builtin_road_rules_de(), served_road_completion_options());
    if (!geometry) return geometry.error();
    const auto body = encode_road_geom(*geometry); if (!body) return body.error();
    auto encoded = encode_body(*body); if (!encoded) return encoded.error();
    return DeriveResult{std::move(*encoded), datum.value_or(VerticalDatum::None), 0};
}
}
