// Street fidelity P2 slice 10 (terrain migration phase 6): g2m.terrain.class. See g2m/layer/terrain_class.h and
// docs/formats/terrain_class.md.
#include "g2m/layer/terrain_class.h"

#include "g2m/core/bytes.h"
#include "g2m/core/hash.h"
#include "g2m/layer/road_geom_tile.h"
#include "g2m/layer/terrain_layers.h"
#include "georef_io.h"

#include <algorithm>
#include <array>
#include <memory>

namespace g2m {
namespace {
constexpr int kN = kTerrainClassSamples;

Result<void> plane_check(const std::vector<std::uint8_t>& p, std::uint8_t max, const char* what) {
    if (p.size() != kTerrainClassNodes) return make_error(ErrorCode::Malformed, std::string("terrain.class: ") + what + " plane size");
    for (const auto v : p) if (v > max) return make_error(ErrorCode::Malformed, std::string("terrain.class: ") + what + " value out of range");
    return {};
}

Result<void> check_header(const InputTile& in) {
    if (in.header->key != in.ref.key || in.header->layer != in.ref.layer || in.header->layer_version != 1 ||
        in.header->body_size != in.body.size() || in.header->body_hash != content_hash(in.body))
        return make_error(ErrorCode::Malformed, "terrain.class: dependency header mismatch");
    return {};
}
} // namespace

void TerrainClassTile::reset(const TileKey& k) {
    key = k;
    for (auto* p : {&land_class, &surface, &role, &rank}) p->assign(kTerrainClassNodes, 0);
}

RasterGeoref terrain_class_georef(const TileKey& key) {
    RasterGeoref g;
    g.zone = key.zone;
    g.west_mm = key.min_easting() * 1000;
    g.south_mm = key.min_northing() * 1000;
    g.spacing_mm = (std::int64_t{1} << key.level) * 1000;
    g.registration = SampleRegistration::Node;
    g.row_order = RowOrder::SouthToNorth;
    g.width = kN; g.height = kN;
    return g;
}

Result<TileBody> encode_terrain_class(const TerrainClassTile& tile) {
    if (!tile.key.valid() || tile.key.level < 0 || tile.key.level > kTerrainMaxLevel)
        return make_error(ErrorCode::InvalidArgument, "terrain.class: key out of range");
    if (auto ok = plane_check(tile.land_class, kLandClassMax, "land_class"); !ok) return make_error(ErrorCode::InvalidArgument, ok.error().message);
    if (auto ok = plane_check(tile.surface, kSurfaceKindMax, "surface"); !ok) return make_error(ErrorCode::InvalidArgument, ok.error().message);
    if (auto ok = plane_check(tile.role, kTerrainRoleMax, "role"); !ok) return make_error(ErrorCode::InvalidArgument, ok.error().message);
    if (auto ok = plane_check(tile.rank, 255, "rank"); !ok) return make_error(ErrorCode::InvalidArgument, ok.error().message);
    TileBody body;
    body.encoding = static_cast<std::uint16_t>(LayerEncoding::TerrainClass);
    body.encoding_version = kTerrainClassEncodingVersion;
    ByteWriter meta;
    layer_detail::write_georef(meta, terrain_class_georef(tile.key));
    meta.u8(static_cast<std::uint8_t>(kTerrainClassScheme.size()));
    meta.bytes(kTerrainClassScheme.data(), kTerrainClassScheme.size());
    body.sections.push_back(Section{terrain_class::kMeta, SectionCodec::None, meta.take()});
    body.sections.push_back(Section{terrain_class::kLandClass, SectionCodec::Zstd, tile.land_class});
    body.sections.push_back(Section{terrain_class::kSurface, SectionCodec::Zstd, tile.surface});
    body.sections.push_back(Section{terrain_class::kRole, SectionCodec::Zstd, tile.role});
    body.sections.push_back(Section{terrain_class::kRank, SectionCodec::Zstd, tile.rank});
    return body;
}

Result<void> decode_terrain_class(const TileBody& body, const TileKey& expect, TerrainClassTile& out) {
    if (body.encoding != static_cast<std::uint16_t>(LayerEncoding::TerrainClass)) return make_error(ErrorCode::Unsupported, "not a terrain.class body");
    if (body.encoding_version != kTerrainClassEncodingVersion) return make_error(ErrorCode::Unsupported, "terrain.class version");
    if (body.sections.size() != 5) return make_error(ErrorCode::Malformed, "terrain.class: sections");
    const Section* meta = body.find(terrain_class::kMeta);
    const Section* planes[4] = {body.find(terrain_class::kLandClass), body.find(terrain_class::kSurface),
                                body.find(terrain_class::kRole), body.find(terrain_class::kRank)};
    if (!meta || !planes[0] || !planes[1] || !planes[2] || !planes[3]) return make_error(ErrorCode::Malformed, "terrain.class: sections");
    ByteReader r(meta->data);
    auto g = layer_detail::read_georef(r);
    if (!g) return g.error();
    const std::uint8_t len = r.u8();
    const std::string scheme(r.string(len));
    if (!r.ok()) return make_error(ErrorCode::Truncated, "terrain.class: meta truncated");
    if (!r.at_end()) return make_error(ErrorCode::Malformed, "terrain.class: meta has trailing bytes");
    if (!expect.valid() || expect.level < 0 || expect.level > kTerrainMaxLevel || !(*g == terrain_class_georef(expect)))
        return make_error(ErrorCode::Malformed, "terrain.class: georef does not match the expected key");
    if (scheme != kTerrainClassScheme) return make_error(ErrorCode::Unsupported, "terrain.class: unknown class scheme");
    TerrainClassTile t;
    t.key = expect;
    t.land_class = planes[0]->data; t.surface = planes[1]->data; t.role = planes[2]->data; t.rank = planes[3]->data;
    if (auto ok = plane_check(t.land_class, kLandClassMax, "land_class"); !ok) return ok;
    if (auto ok = plane_check(t.surface, kSurfaceKindMax, "surface"); !ok) return ok;
    if (auto ok = plane_check(t.role, kTerrainRoleMax, "role"); !ok) return ok;
    if (auto ok = plane_check(t.rank, 255, "rank"); !ok) return ok;
    out = std::move(t);
    return {};
}

void LandcoverBase::at(std::int64_t e, std::int64_t n, LandClass& land_class, SurfaceKind& surface) const {
    land_class = LandClass::Unknown; surface = SurfaceKind::Unknown;
    for (const LandcoverTile& t : tiles_) {
        const RasterGeoref& g = t.georef;
        if (g.spacing_mm <= 0) continue;
        // The cell holding the point, floor on both axes (Cell registration: cells start at the footprint corner; Node:
        // the sample nearest below-left, the same floor rule).
        const std::int64_t col = floor_div(e * 1000 - g.west_mm, g.spacing_mm);
        const std::int64_t r = floor_div(n * 1000 - g.south_mm, g.spacing_mm);
        if (col < 0 || r < 0 || col >= g.width || r >= g.height) continue;
        const std::int64_t row = g.row_order == RowOrder::SouthToNorth ? r : g.height - 1 - r;
        const std::uint8_t v = t.classes.data()[static_cast<std::size_t>(row * g.width + col)];
        ClassifierFacts facts;
        facts.landcover_scheme = t.class_scheme;
        facts.landcover_class = v;
        land_class = classifier_.land_class(facts);
        surface = classifier_.surface(facts);
        return;
    }
}

Result<void> classify_terrain_tile(TerrainClassTile& out, const TileKey& key, std::span<const HeightTile* const> dem,
    std::span<const CarveGeomInput> roads, const LandcoverBase& base, const CarveParams& params, const CarveOptions& options) {
    if (!key.valid() || key.level != 0) return make_error(ErrorCode::InvalidArgument, "terrain.class: classify needs a level-0 key");
    auto terrain = CarveTerrain::make(key, dem, params.smoothing);
    if (!terrain) return terrain.error();
    auto model = CarveModel::build(roads, params, *terrain, options, builtin_road_rules_de());
    if (!model) return model.error();
    out.reset(key);
    const std::int64_t e0 = key.min_easting(), n0 = key.min_northing();
    for (int j = 0; j < kN; ++j) for (int i = 0; i < kN; ++i) {
        const double x = static_cast<double>(e0 + i), y = static_cast<double>(n0 + j);
        const CarveSample c = model->evaluate(x, y);
        CarveClass k = model->classify(x, y, c);
        if (k.base) base.at(e0 + i, n0 + j, k.land_class, k.surface);
        const std::size_t idx = TerrainClassTile::index(i, j);
        out.land_class[idx] = static_cast<std::uint8_t>(k.land_class);
        out.surface[idx] = static_cast<std::uint8_t>(k.surface);
        out.role[idx] = static_cast<std::uint8_t>(k.role);
        out.rank[idx] = k.rank;
    }
    return {};
}

void downsample_terrain_class(const TileKey& parent, std::span<const TerrainClassTile* const> window, TerrainClassTile& out) {
    out.reset(parent);
    if (window.size() != 16) return;
    const std::int64_t cx0 = 2 * static_cast<std::int64_t>(parent.x) - 1, cy0 = 2 * static_cast<std::int64_t>(parent.y) - 1;
    // The child-level node (gi, gj) (global node indices at level L-1): the tile holding it, or, on a shared edge, the
    // neighbour that also holds it (equal by construction), first present wins.
    const auto node = [&](std::int64_t gi, std::int64_t gj, std::size_t& idx) -> const TerrainClassTile* {
        const std::int64_t cx = floor_div(gi, 256), cy = floor_div(gj, 256);
        const std::int64_t li = gi - 256 * cx, lj = gj - 256 * cy;
        const std::array<std::array<std::int64_t, 4>, 4> tries{{{cx, cy, li, lj}, {cx - 1, cy, li + 256, lj},
                                                               {cx, cy - 1, li, lj + 256}, {cx - 1, cy - 1, li + 256, lj + 256}}};
        for (std::size_t k = 0; k < 4; ++k) {
            const auto& t = tries[k];
            if (k == 1 && li != 0) continue;
            if (k == 2 && lj != 0) continue;
            if (k == 3 && (li != 0 || lj != 0)) continue;
            const std::int64_t sx = t[0] - cx0, sy = t[1] - cy0;
            if (sx < 0 || sy < 0 || sx > 3 || sy > 3) continue;
            const TerrainClassTile* tile = window[static_cast<std::size_t>(sy * 4 + sx)];
            if (!tile) continue;
            idx = TerrainClassTile::index(static_cast<int>(t[2]), static_cast<int>(t[3]));
            return tile;
        }
        return nullptr;
    };
    const std::int64_t gx0 = 512 * static_cast<std::int64_t>(parent.x), gy0 = 512 * static_cast<std::int64_t>(parent.y);
    for (int j = 0; j < kN; ++j) for (int i = 0; i < kN; ++i) {
        const std::int64_t gi = gx0 + 2 * i, gj = gy0 + 2 * j;
        std::size_t best_idx = 0;
        const TerrainClassTile* best = node(gi, gj, best_idx);   // ties go to the coincident centre
        for (int dy = -1; dy <= 1; ++dy) for (int dx = -1; dx <= 1; ++dx) {
            if (dx == 0 && dy == 0) continue;
            std::size_t idx = 0;
            const TerrainClassTile* t = node(gi + dx, gj + dy, idx);
            if (!t) continue;
            if (!best || t->rank[idx] > best->rank[best_idx]) { best = t; best_idx = idx; }
        }
        if (!best) continue;
        const std::size_t o = TerrainClassTile::index(i, j);
        out.land_class[o] = best->land_class[best_idx];
        out.surface[o] = best->surface[best_idx];
        out.role[o] = best->role[best_idx];
        out.rank[o] = best->rank[best_idx];
    }
}

std::vector<TileKey> terrain_class_landcover_keys(const TileKey& level0) {
    const std::int64_t size2 = TileKey::kLevel0SizeM << 2;
    const std::int64_t e0 = level0.min_easting(), n0 = level0.min_northing(), s = level0.size_m();
    std::vector<TileKey> keys;
    for (std::int64_t y = floor_div(n0, size2); y <= floor_div(n0 + s, size2); ++y)
        for (std::int64_t x = floor_div(e0, size2); x <= floor_div(e0 + s, size2); ++x)
            keys.push_back(TileKey{level0.zone, 2, static_cast<std::int32_t>(x), static_cast<std::int32_t>(y)});
    std::sort(keys.begin(), keys.end());
    return keys;
}

std::string terrain_class_config_text() {
    const auto& carve = builtin_carve_default().content_sha256;
    const auto table = builtin_esa_worldcover_classifier().table_hash();
    return "g2m.terrain.class/1 nodes=257 L0 carve-ownership roads.geom enc=9 radius=192 junction=D42 underpass-deck=1 carve=" +
        to_hex(std::span<const std::uint8_t>(carve.data(), carve.size())) + " classifier=" +
        to_hex(std::span<const std::uint8_t>(table.data(), table.size())) +
        " scheme=g2m.landclass.v1 landcover=floor sidewalk=paving_stones; L1-L6 rank-max 3x3 centre-then-row-major window=4x4";
}

TerrainClassDeriver::TerrainClassDeriver() : desc_(terrain_class_desc()) {}
ContentHash TerrainClassDeriver::config_hash() const { return Sha256::of(terrain_class_config_text()); }

std::vector<InputTileRef> TerrainClassDeriver::inputs_for(const TileKey& target) const {
    if (!target.valid() || target.level < 0 || target.level > kTerrainMaxLevel) return {};
    std::vector<InputTileRef> refs;
    if (target.level == 0) {
        for (int dy = -1; dy <= 1; ++dy) for (int dx = -1; dx <= 1; ++dx)
            refs.push_back({std::string(kElevBaseLayer), TileKey{target.zone, 0, target.x + dx, target.y + dy}});
        for (const auto& k : carve_road_geom_keys(target)) refs.push_back({std::string(kRoadGeomLayer), k});
        for (const auto& k : terrain_class_landcover_keys(target)) refs.push_back({std::string(kSrcLandcoverLayer), k});
    } else {
        for (int dy = -1; dy <= 2; ++dy) for (int dx = -1; dx <= 2; ++dx)
            refs.push_back({std::string(kTerrainClassLayer), TileKey{target.zone, target.level - 1,
                static_cast<std::int32_t>(2 * static_cast<std::int64_t>(target.x) + dx),
                static_cast<std::int32_t>(2 * static_cast<std::int64_t>(target.y) + dy)}});
    }
    std::sort(refs.begin(), refs.end());
    return refs;
}

bool TerrainClassDeriver::rejected_input_is_absent(const InputTileRef& ref) const { return ref.layer == kRoadGeomLayer; }

Result<DeriveResult> TerrainClassDeriver::derive(const DeriveRequest& request) const {
    const TileKey& target = request.target;
    if (!target.valid() || target.level < 0 || target.level > kTerrainMaxLevel)
        return make_error(ErrorCode::InvalidArgument, "terrain.class: level out of range [0, kTerrainMaxLevel]");
    const auto expected = inputs_for(target);
    if (request.inputs.size() != expected.size()) return make_error(ErrorCode::InvalidArgument, "terrain.class: unexpected inputs");
    for (std::size_t i = 0; i < expected.size(); ++i)
        if (!(request.inputs[i].ref == expected[i])) return make_error(ErrorCode::InvalidArgument, "terrain.class: unexpected inputs");
    auto tile = std::make_unique<TerrainClassTile>();
    if (target.level > 0) {
        std::vector<TerrainClassTile> children(16);
        std::array<const TerrainClassTile*, 16> window{};
        for (const InputTile& in : request.inputs) {
            if (!in.header) continue;
            if (auto ok = check_header(in); !ok) return ok.error();
            const auto body = decode_body(in.body); if (!body) return body.error();
            const std::int64_t sx = static_cast<std::int64_t>(in.ref.key.x) - (2 * static_cast<std::int64_t>(target.x) - 1);
            const std::int64_t sy = static_cast<std::int64_t>(in.ref.key.y) - (2 * static_cast<std::int64_t>(target.y) - 1);
            const std::size_t slot = static_cast<std::size_t>(sy * 4 + sx);
            if (auto ok = decode_terrain_class(*body, in.ref.key, children[slot]); !ok) return ok.error();
            window[slot] = &children[slot];
        }
        downsample_terrain_class(target, window, *tile);
    } else {
        CarveL0Inputs in;
        if (auto ok = decode_carve_l0_inputs(target, request.inputs, in, "terrain.class"); !ok) return ok.error();
        std::vector<LandcoverTile> landcover;
        for (const InputTile& li : request.inputs) {
            if (li.ref.layer != kSrcLandcoverLayer || !li.header) continue;
            if (auto ok = check_header(li); !ok) return ok.error();
            const auto body = decode_body(li.body); if (!body) return body.error();
            auto t = decode_src_landcover(*body); if (!t) return t.error();
            const auto want = tile_georef(li.ref.key, 256, SampleRegistration::Cell);
            if (!want || !(t->georef.zone == want->zone) || t->georef.west_mm != want->west_mm || t->georef.south_mm != want->south_mm ||
                t->georef.spacing_mm != want->spacing_mm || t->georef.width != want->width || t->georef.height != want->height ||
                t->georef.registration != SampleRegistration::Cell)
                return make_error(ErrorCode::Malformed, "terrain.class: src.landcover georef does not match its key");
            landcover.push_back(std::move(*t));
        }
        const LandcoverBase base(landcover, builtin_esa_worldcover_classifier());
        const auto geom = in.geom();
        if (auto ok = classify_terrain_tile(*tile, target, in.window, geom, base); !ok) return ok.error();
    }
    auto encoded = encode_terrain_class(*tile); if (!encoded) return encoded.error();
    auto body = encode_body(*encoded); if (!body) return body.error();
    return DeriveResult{std::move(*body), VerticalDatum::None, 0};
}
} // namespace g2m
