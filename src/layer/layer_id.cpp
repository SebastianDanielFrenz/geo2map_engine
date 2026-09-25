// src/layer/layer_id.cpp - see g2m/layer/layer_id.h and layer_registry.h.

#include "g2m/layer/layer_id.h"
#include "g2m/layer/layer_registry.h"

#include <algorithm>
#include <string>

namespace g2m {
namespace {

bool valid_component(std::string_view c) {
    if (c.empty() || c[0] < 'a' || c[0] > 'z') return false;
    return std::all_of(c.begin(), c.end(),
                       [](char ch) { return (ch >= 'a' && ch <= 'z') || (ch >= '0' && ch <= '9') || ch == '_'; });
}

} // namespace

bool is_valid_layer_name(std::string_view name) {
    if (name.empty() || name.size() > kMaxLayerNameLength) return false;
    int components = 0;
    std::size_t start = 0;
    while (true) {
        const std::size_t dot = name.find('.', start);
        const std::string_view comp = name.substr(start, dot == std::string_view::npos ? name.size() - start : dot - start);
        if (!valid_component(comp)) return false;
        ++components;
        if (dot == std::string_view::npos) break;
        start = dot + 1;
    }
    return components >= 2;
}

bool is_builtin_layer_name(std::string_view name) { return name.starts_with("g2m."); }

LayerDesc make_raw_elev_layer(std::string_view source) {
    LayerDesc d;
    if (!valid_component(source)) return d;
    d.id = LayerId{std::string(kSrcElevPrefix) + std::string(source), 1};
    if (!is_valid_layer_name(d.id.name)) {
        d.id.name.clear();
        return d;
    }
    d.kind = LayerKind::Raw;
    d.encoding = static_cast<std::uint16_t>(LayerEncoding::SrcElev);
    d.encoding_version = 1;
    d.levels = level_bit(kRawTileLevel);
    d.content = "native DEM samples (int32 1/256 m) + vertical datum id, source '" + std::string(source) + "'";
    return d;
}

LayerDesc make_raw_osm_layer() {
    LayerDesc d;
    d.id = LayerId{std::string(kSrcOsmLayer), 1};
    d.kind = LayerKind::Raw;
    d.encoding = static_cast<std::uint16_t>(LayerEncoding::SrcOsm);
    d.encoding_version = 1;
    d.levels = level_bit(kRawTileLevel);
    d.content = "OSM nodes/ways/relations clipped per 1 km tile with a halo; ids, tags, relation membership";
    return d;
}

LayerDesc make_raw_landcover_layer() {
    LayerDesc d;
    d.id = LayerId{std::string(kSrcLandcoverLayer), 1};
    d.kind = LayerKind::Raw;
    d.encoding = static_cast<std::uint16_t>(LayerEncoding::SrcLandcover);
    d.encoding_version = 1;
    d.levels = level_bit(kRawTileLevel);
    d.content = "land cover classes (uint8), nearest-neighbour on the UTM grid";
    return d;
}

const LayerRegistry& LayerRegistry::builtin() {
    static const LayerRegistry reg = [] {
        LayerRegistry r;
        // add() cannot fail for these constant descriptors; a failure would
        // be caught by tests/layer's registry test.
        (void)r.add(make_raw_elev_layer("dgm1"));
        (void)r.add(make_raw_landcover_layer());
        (void)r.add(make_raw_osm_layer());
        return r;
    }();
    return reg;
}

Result<void> LayerRegistry::add(LayerDesc desc) {
    if (!is_valid_layer_name(desc.id.name)) {
        return make_error(ErrorCode::InvalidArgument, "invalid layer name '" + desc.id.name + "'");
    }
    if (desc.levels == 0 || (desc.levels >> (TileKey::kMaxLevel + 1)) != 0) {
        return make_error(ErrorCode::InvalidArgument, "layer '" + desc.id.name + "': bad level mask");
    }
    for (std::size_t i = 0; i < desc.inputs.size(); ++i) {
        if (!is_valid_layer_name(desc.inputs[i]) || (i > 0 && !(desc.inputs[i - 1] < desc.inputs[i]))) {
            return make_error(ErrorCode::InvalidArgument, "layer '" + desc.id.name + "': inputs must be valid, sorted, unique");
        }
    }
    if ((desc.kind == LayerKind::Raw) != desc.inputs.empty()) {
        return make_error(ErrorCode::InvalidArgument,
                          "layer '" + desc.id.name + "': raw layers have no inputs, baked layers need some");
    }
    const auto it = std::lower_bound(layers_.begin(), layers_.end(), desc.id.name,
                                     [](const LayerDesc& d, const std::string& n) { return d.id.name < n; });
    if (it != layers_.end() && it->id.name == desc.id.name) {
        return make_error(ErrorCode::InvalidArgument, "duplicate layer '" + desc.id.name + "'");
    }
    layers_.insert(it, std::move(desc));
    return {};
}

const LayerDesc* LayerRegistry::find(std::string_view name) const {
    const auto it = std::lower_bound(layers_.begin(), layers_.end(), name,
                                     [](const LayerDesc& d, std::string_view n) { return d.id.name < n; });
    return (it != layers_.end() && it->id.name == name) ? &*it : nullptr;
}

} // namespace g2m
