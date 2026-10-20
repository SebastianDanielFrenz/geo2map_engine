// Railways R1/R2: the g2m.rail.geom deriver (inputs, DEM discovery, derive). See g2m/layer/rail_geom_tile.h.
#include "g2m/layer/rail_geom_tile.h"
#include "g2m/core/hash.h"
#include "g2m/layer/src_osm.h"
#include "g2m/layer/terrain_layers.h"
#include <algorithm>
#include <limits>
#include <set>

namespace g2m {
namespace {
Result<void> check_header(const InputTile& in, const char* what) {
    if (in.header->key != in.ref.key || in.header->layer != in.ref.layer || in.header->layer_version != 1 ||
        in.header->body_size != in.body.size() || in.header->body_hash != content_hash(in.body))
        return make_error(ErrorCode::Malformed, std::string("rail.geom: ") + what + " header mismatch");
    return {};
}
// The decoded src.osm tiles among `inputs` (absent tiles contribute nothing) and the world they hold.
struct Window {
    std::vector<OsmTile> tiles;
    RailWorld world;
};
Result<Window> make_window(const DeriveRequest& request) {
    if (!request.target.valid() || request.target.level != 2) return make_error(ErrorCode::InvalidArgument, "rail.geom: level-2 target required");
    Window w;
    w.tiles.reserve(request.inputs.size());
    std::size_t bytes = 0;
    for (const auto& input : request.inputs) {
        if (input.ref.layer != kSrcOsmLayer) continue;
        if (input.body.size() > kMaxBodyRawSize - bytes) return make_error(ErrorCode::LimitExceeded, "rail.geom: input byte budget");
        bytes += input.body.size();
        if (!input.header) {
            if (!input.body.empty()) return make_error(ErrorCode::Malformed, "rail.geom: absent input has bytes");
            continue;
        }
        if (const auto ok = check_header(input, "source"); !ok) return ok.error();
        const auto body = decode_body(input.body); if (!body) return body.error();
        auto tile = decode_src_osm(*body); if (!tile) return tile.error();
        w.tiles.push_back(std::move(*tile));
    }
    std::vector<const osm::OsmData*> data;
    for (const auto& t : w.tiles) data.push_back(&t.data);
    auto world = collect_rail_world(data, request.target.zone, builtin_rail_rules_de());
    if (!world) return world.error();
    w.world = std::move(*world);
    return w;
}
}

std::vector<InputTileRef> RailGeomDeriver::inputs_for(const TileKey& target) const {
    std::vector<InputTileRef> result;
    if (!target.valid() || target.level != 2) return result;
    for (int dy = -1; dy <= 1; ++dy) for (int dx = -1; dx <= 1; ++dx) {
        auto key = target; key.x += dx; key.y += dy;
        if (key.valid()) result.push_back({std::string(kSrcOsmLayer), key});
    }
    std::sort(result.begin(), result.end());
    return result;
}

Result<std::vector<InputTileRef>> RailGeomDeriver::additional_inputs_for(const DeriveRequest& request) const {
    auto window = make_window(request); if (!window) return window.error();
    return rail_dem_dependencies(window->world, request.target, builtin_rail_rules_de(), 1024, true);
}

Result<DeriveResult> RailGeomDeriver::derive(const DeriveRequest& request) const {
    auto window = make_window(request); if (!window) return window.error();
    auto expected = inputs_for(request.target);
    auto dem_refs = rail_dem_dependencies(window->world, request.target, builtin_rail_rules_de(), 1024, true); if (!dem_refs) return dem_refs.error();
    expected.insert(expected.end(), dem_refs->begin(), dem_refs->end());
    std::sort(expected.begin(), expected.end());
    if (request.inputs.size() != expected.size()) return make_error(ErrorCode::InvalidArgument, "rail.geom: incomplete dependency set");
    std::set<InputTileRef> seen; std::vector<HeightTile> heights; std::optional<VerticalDatum> datum;
    for (const auto& input : request.inputs) {
        if (!std::binary_search(expected.begin(), expected.end(), input.ref) || !seen.insert(input.ref).second)
            return make_error(ErrorCode::Malformed, "rail.geom: unexpected or duplicate dependency");
        if (input.ref.layer == kSrcOsmLayer) continue;
        if (!input.header) continue;   // outside the elevation coverage: the pieces needing it are declined
        if (const auto ok = check_header(input, "elevation"); !ok) return ok.error();
        if (input.header->datum != VerticalDatum::DHHN2016 && input.header->datum != VerticalDatum::EGM2008)
            return make_error(ErrorCode::Unsupported, "rail.geom: unknown elevation datum");
        if (datum && *datum != input.header->datum) return make_error(ErrorCode::Malformed, "rail.geom: mixed elevation datums");
        datum = input.header->datum;
        const auto body = decode_body(input.body); if (!body) return body.error();
        heights.emplace_back(); const auto decoded = decode_height_tile(*body, input.ref.key, heights.back()); if (!decoded) return decoded.error();
        for (auto& height : heights.back().h) if (height != kHeightNoData) {
            const auto adjusted = static_cast<std::int64_t>(height) + input.header->height_offset;
            if (adjusted <= std::numeric_limits<std::int32_t>::min() || adjusted > std::numeric_limits<std::int32_t>::max())
                return make_error(ErrorCode::LimitExceeded, "rail.geom: elevation offset overflow");
            height = static_cast<std::int32_t>(adjusted);
        }
    }
    std::vector<const HeightTile*> views; for (const auto& height : heights) views.push_back(&height);
    const auto dem = RoadDemSampler::make(views); if (!dem) return dem.error();
    const auto tile = build_rail_geom_tile(window->world, request.target, builtin_rail_rules_de(), *dem, true);
    if (!tile) return tile.error();
    const auto body = encode_rail_geom(*tile); if (!body) return body.error();
    auto bytes = encode_body(*body); if (!bytes) return bytes.error();
    return DeriveResult{std::move(*bytes), datum.value_or(VerticalDatum::None), 0};
}
}
