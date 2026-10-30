// g2m.fields.geom: rules, ids, geometry helpers, codec, subdivision kernel, deriver on synthetic OSM worlds
// (docs/formats/fields_geom.md, docs/pipeline/fields.md).
#include "g2m/core/geom/project.h"
#include "g2m/layer/field_tile.h"
#include "g2m/layer/field_rules.h"
#include "g2m/layer/field_subdivide.h"
#include "g2m/layer/osm_clip.h"
#include "g2m/layer/src_osm.h"
#include "g2m/core/bytes.h"
#include "g2m/core/hash.h"
#include "g2m/core/geo/utm.h"
#include "g2m/import/bake.h"
#include "g2m/server/builtin_release.h"
#include "g2m/server/server.h"
#include "g2m/server/upstream.h"
#include "road_fixture.h"
#include "synthetic.h"
#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <map>
#include <set>
#include <string>

using namespace g2m;
namespace {
using geom::PointMm;
using geom::Ring;
using Tags = osm::OsmBuilder::TagList;

// An OSM set with its clip index (the index keeps a pointer to the data, so a World never moves).
struct World {
    explicit World(osm::OsmData d, geo::UtmZone zone = g2m_test::kZone32N) : data(std::move(d)), index(OsmClipIndex::build(data, zone)) {}
    World(const World&) = delete; World& operator=(const World&) = delete;
    const osm::OsmData data;
    OsmClipIndex index;
};

// A synthetic OSM world in metres relative to the south-west corner of the home tile. Nodes are shared by position.
class Scene {
public:
    explicit Scene(const TileKey& home = g2m_test::home_tile()) : home_(home), utm_(geo::kGRS80) {}
    const TileKey& home() const { return home_; }
    std::int64_t node(double x_m, double y_m) {
        const auto key = std::make_pair(std::llround(x_m * 1000), std::llround(y_m * 1000));
        const auto it = nodes_.find(key);
        if (it != nodes_.end()) return it->second;
        const auto e = static_cast<double>(home_.min_easting()) + static_cast<double>(key.first) / 1000.0;
        const auto n = static_cast<double>(home_.min_northing()) + static_cast<double>(key.second) / 1000.0;
        const auto ll = utm_.reverse(home_.zone, e, n);
        const auto id = next_node_++;
        builder_.add_node(id, g2m_test::to_e7(ll.lat), g2m_test::to_e7(ll.lon));
        nodes_.emplace(key, id);
        return id;
    }
    // A way through the points (closed when the last point equals the first).
    std::int64_t way(std::int64_t id, const std::vector<std::pair<double, double>>& pts, Tags tags) {
        std::vector<std::int64_t> refs;
        for (const auto& p : pts) refs.push_back(node(p.first, p.second));
        builder_.add_way(id, std::move(refs), std::move(tags));
        return id;
    }
    static std::vector<std::pair<double, double>> rect(double x0, double y0, double x1, double y1) {
        return {{x0, y0}, {x1, y0}, {x1, y1}, {x0, y1}, {x0, y0}};
    }
    std::int64_t field(std::int64_t id, double x0, double y0, double x1, double y1, std::string landuse = "farmland", std::string crop = {}) {
        Tags tags{{"landuse", std::move(landuse)}};
        if (!crop.empty()) tags.emplace_back("crop", std::move(crop));
        return way(id, rect(x0, y0, x1, y1), std::move(tags));
    }
    void relation(std::int64_t id, std::vector<osm::OsmBuilder::MemberIn> members, Tags tags) {
        builder_.add_relation(id, std::move(members), std::move(tags));
    }
    inline World build() const;
private:
    TileKey home_;
    geo::Utm utm_;
    osm::OsmBuilder builder_;
    std::map<std::pair<std::int64_t, std::int64_t>, std::int64_t> nodes_;
    std::int64_t next_node_ = 1;
};
inline World Scene::build() const { const auto data = builder_.build(); REQUIRE(data); return World(*data); }

InputTile make_input(const InputTileRef& ref, const World& world) {
    const OsmTile clipped{world.index.halo_m(), world.index.clip(ref.key)};
    const auto tile_body = encode_src_osm(clipped); REQUIRE(tile_body);
    const auto bytes = encode_body(*tile_body); REQUIRE(bytes);
    TileHeader header; header.layer = ref.layer; header.layer_version = 1; header.key = ref.key;
    header.body_size = bytes->size(); header.body_hash = content_hash(*bytes);
    return {ref, header, *bytes};
}

struct Run {
    std::vector<InputTile> inputs;
    Result<DeriveResult> result = make_error(ErrorCode::NotFound, "not run");
    std::size_t stages_used = 0;
};
// The server's protocol: first-stage inputs, then up to discovery_stages() rounds of additional_inputs_for.
Run run_fields(const World& world, const TileKey& target, bool discovery = true, bool reversed = false, const std::set<TileKey>& absent = {}) {
    FieldsGeomDeriver deriver; Run run;
    const auto resolve = [&](const InputTileRef& ref) { return absent.contains(ref.key) ? InputTile{ref, {}, {}} : make_input(ref, world); };
    for (const auto& ref : deriver.inputs_for(target)) run.inputs.push_back(resolve(ref));
    const auto sort_inputs = [&] { std::sort(run.inputs.begin(), run.inputs.end(), [](const auto& a, const auto& b) { return a.ref < b.ref; }); };
    sort_inputs();
    for (std::size_t stage = 0; discovery && stage < deriver.discovery_stages(); ++stage) {
        const auto more = deriver.additional_inputs_for({target, 0, run.inputs}); REQUIRE(more);
        std::size_t added = 0;
        for (const auto& ref : *more) {
            if (std::any_of(run.inputs.begin(), run.inputs.end(), [&](const auto& in) { return in.ref == ref; })) continue;
            run.inputs.push_back(resolve(ref)); ++added;
        }
        sort_inputs();
        if (added == 0) break;
        ++run.stages_used;
    }
    if (reversed) std::reverse(run.inputs.begin(), run.inputs.end());
    run.result = deriver.derive({target, 0, run.inputs});
    return run;
}
FieldTile decode_run(const Run& run) {
    REQUIRE(run.result);
    const auto body = decode_body(run.result->body); REQUIRE(body);
    auto tile = decode_fields_geom(*body); REQUIRE(tile);
    REQUIRE(validate_field_tile(*tile));
    return *tile;
}
FieldTile fields_of(const World& world, const TileKey& target) { return decode_run(run_fields(world, target)); }

const FieldRecord* find_piece(const FieldTile& tile, std::int64_t osm_id, std::uint32_t piece, FieldSource source = FieldSource::OsmWay) {
    const auto id = pack_field_id(source, osm_id, piece); REQUIRE(id);
    for (const auto& f : tile.fields) if (f.id == *id) return &f;
    return nullptr;
}
std::size_t count_osm(const FieldTile& tile, std::int64_t osm_id) {
    return static_cast<std::size_t>(std::count_if(tile.fields.begin(), tile.fields.end(), [&](const auto& f) { return f.osm_id == osm_id; }));
}
PointMm at(const TileKey& key, double x_m, double y_m) {
    return {key.min_easting() * 1000 + std::llround(x_m * 1000), key.min_northing() * 1000 + std::llround(y_m * 1000)};
}
double near_ratio(const FieldRecord& f, double expected_m2) { return f.area_m2() / expected_m2; }
bool cdeg_near(std::uint16_t cdeg, int target, int tol = 100) {
    const int d = std::abs(static_cast<int>(cdeg) - target);
    return std::min(d, 18000 - d) <= tol;
}
Ring square_ring(std::int64_t x0, std::int64_t y0, std::int64_t x1, std::int64_t y1) { return {{x0, y0}, {x1, y0}, {x1, y1}, {x0, y1}}; }
} // namespace

// ---- rules ------------------------------------------------------------------------------------------------------

TEST_CASE("Field rules: the embedded file loads, matches by tag and rejects malformed rule files", "[layer][fields]") {
    const auto& rules = builtin_field_rules_de();
    CHECK(rules.landuse_code("farmland") == 0); CHECK(rules.landuse_code("meadow") == 1);
    CHECK(rules.landuse_code("orchard") == 2); CHECK(rules.landuse_code("vineyard") == 3); CHECK(rules.landuse_code("farmyard") == 4);
    CHECK(rules.landuse_code("forest") == -1); CHECK(rules.landuse_code("") == -1);
    CHECK(rules.cell_mm % 2 == 0); CHECK(rules.max_half_width_mm() == 7000);
    CHECK(rules.splitters.size() == 12);
    // The rules are data: the SHA-256 is that of the embedded bytes.
    CHECK(rules.content_sha256 == Sha256::of(builtin_field_rules_de_bytes()));
    const FieldTagMatch any{"bridge", {"*"}}, some{"highway", {"path", "track"}};
    CHECK(field_tag_matches(any, "yes")); CHECK_FALSE(field_tag_matches(any, "no"));
    CHECK(field_tag_matches(some, "track")); CHECK_FALSE(field_tag_matches(some, "primary"));

    const std::string text(reinterpret_cast<const char*>(builtin_field_rules_de_bytes().data()), builtin_field_rules_de_bytes().size());
    const auto load = [](const std::string& s) { return load_field_rules({reinterpret_cast<const std::byte*>(s.data()), s.size()}); };
    CHECK(load(text));
    const auto replaced = [&](const std::string& from, const std::string& to) {
        auto s = text; const auto p = s.find(from); REQUIRE(p != std::string::npos); s.replace(p, from.size(), to); return s;
    };
    CHECK_FALSE(load(""));
    CHECK_FALSE(load("[]"));
    CHECK_FALSE(load(replaced("g2m.field_rules/1", "g2m.field_rules/2")));
    CHECK_FALSE(load(replaced("\"cell_mm\": 1000", "\"cell_mm\": 1001")));          // odd cell
    CHECK_FALSE(load(replaced("\"cell_mm\": 1000", "\"cell_mm\": 10")));            // below the range
    CHECK_FALSE(load(replaced("\"cell_mm\": 1000", "\"cell_mm\": 1000.5")));        // not an integer
    CHECK_FALSE(load(replaced("\"farmland\": 0", "\"farmland\": 1")));              // duplicate code
    CHECK_FALSE(load(replaced("\"max_cells\"", "\"max_cellz\"")));                  // unknown and missing key
    CHECK_FALSE(load(replaced("\"half_width_mm\": 7000", "\"half_width_mm\": 200000"))); // wider than the OSM halo
    CHECK_FALSE(load(replaced("\"min_area_m2\": 200", "\"min_area_m2\": 2000000000"))); // min above max
    CHECK_FALSE(load(replaced("\"name\": \"track\"", "\"name\": \"path\"")));       // duplicate name
    CHECK_FALSE(load(text + " x"));
    // Every JSON edit changes the hash a release id is built from.
    const auto edited = load(replaced("\"tolerance_mm\": 1000", "\"tolerance_mm\": 1001")); REQUIRE(edited);
    CHECK(edited->content_sha256 != rules.content_sha256);
}

// ---- ids and geometry helpers --------------------------------------------------------------------------------------

TEST_CASE("Field ids pack source, OSM id and piece and reject what does not fit", "[layer][fields]") {
    for (const auto source : {FieldSource::OsmWay, FieldSource::OsmRelation, FieldSource::Parcel}) {
        for (const std::int64_t osm : {std::int64_t{0}, std::int64_t{1}, std::int64_t{123456789012}, kFieldMaxOsmId}) {
            for (const std::uint32_t piece : {0u, 1u, 65535u}) {
                const auto id = pack_field_id(source, osm, piece); REQUIRE(id);
                const auto parts = unpack_field_id(*id); REQUIRE(parts);
                CHECK(parts->source == source); CHECK(parts->osm_id == osm); CHECK(parts->piece == piece);
            }
        }
    }
    CHECK_FALSE(pack_field_id(FieldSource::OsmWay, -1, 0));
    CHECK_FALSE(pack_field_id(FieldSource::OsmWay, kFieldMaxOsmId + 1, 0));
    CHECK_FALSE(pack_field_id(FieldSource::OsmWay, 1, 65536));
    CHECK_FALSE(unpack_field_id(std::uint64_t{3} << 62)); // source 3 does not exist
    // Order: source first, then OSM id, then piece.
    CHECK(*pack_field_id(FieldSource::OsmWay, 5, 9) < *pack_field_id(FieldSource::OsmWay, 6, 0));
    CHECK(*pack_field_id(FieldSource::OsmWay, 5, 0) < *pack_field_id(FieldSource::OsmWay, 5, 1));
    CHECK(*pack_field_id(FieldSource::OsmWay, kFieldMaxOsmId, 65535) < *pack_field_id(FieldSource::OsmRelation, 0, 0));
}

TEST_CASE("Field geometry helpers are exact", "[layer][fields]") {
    const Ring ccw = square_ring(0, 0, 10000, 4000);
    CHECK(ring_twice_area_mm2(ccw) == 2 * 10000 * 4000);
    Ring cw(ccw.rbegin(), ccw.rend());
    CHECK(ring_twice_area_mm2(cw) == -2 * 10000 * 4000);
    // normalise: orientation and start vertex.
    Ring r = {{10000, 4000}, {0, 4000}, {0, 0}, {10000, 0}}; // clockwise, starts at (10000, 4000)
    normalise_field_ring(r, true);
    CHECK(r.front() == PointMm{0, 0}); CHECK(ring_twice_area_mm2(r) > 0);
    normalise_field_ring(r, false);
    CHECK(r.front() == PointMm{0, 0}); CHECK(ring_twice_area_mm2(r) < 0);

    FieldPolygon poly; poly.outer = ccw; poly.holes.push_back(square_ring(2000, 1000, 4000, 3000));
    normalise_field_ring(poly.outer, true); normalise_field_ring(poly.holes[0], false);
    CHECK(field_polygon_area_mm2(poly) == 10000ull * 4000 - 2000ull * 2000);
    CHECK(field_ring_contains(poly.outer, {5000, 2000}));
    CHECK(field_ring_contains(poly.outer, {0, 0}));       // corner counts
    CHECK(field_ring_contains(poly.outer, {10000, 2000})); // edge counts
    CHECK_FALSE(field_ring_contains(poly.outer, {10001, 2000}));
    CHECK(field_polygon_contains(poly, {8000, 2000}));
    CHECK_FALSE(field_polygon_contains(poly, {3000, 2000})); // in the hole
    CHECK(field_polygon_contains(poly, {2000, 2000}));       // on the hole's edge counts as inside
    const auto box = ring_bbox(poly.outer);
    CHECK(box.w == 0); CHECK(box.s == 0); CHECK(box.e == 10000); CHECK(box.n == 4000);

    std::uint16_t cdeg = 99; std::uint32_t len = 99;
    field_row_direction(poly.outer, cdeg, len);
    CHECK(cdeg == 0); CHECK(len == 10000);                 // the first of the two longest (equal) edges is the southern one, heading east
    const Ring tall = square_ring(0, 0, 3000, 8000);       // north-south edge is the longest
    field_row_direction(tall, cdeg, len);
    CHECK(cdeg == 9000); CHECK(len == 8000);
    const Ring west_going = {{0, 0}, {0, -5000}, {-6000, 0}}; // longest edge from (0,-5000) to (-6000,0): direction (-6000, 5000)
    field_row_direction(west_going, cdeg, len);
    CHECK(cdeg < 18000); CHECK(len == static_cast<std::uint32_t>(std::llround(std::sqrt(6000.0 * 6000 + 5000.0 * 5000))));
    field_row_direction(Ring{{0, 0}, {1, 1}}, cdeg, len); CHECK(cdeg == 0); CHECK(len == 0);

    // The anchor is inside the polygon, also when the centre is a hole, and is deterministic.
    FieldPolygon ring_shape; ring_shape.outer = square_ring(0, 0, 30000, 30000); ring_shape.holes.push_back(square_ring(5000, 5000, 25000, 25000));
    normalise_field_ring(ring_shape.outer, true); normalise_field_ring(ring_shape.holes[0], false);
    const auto anchor = field_anchor(ring_shape);
    CHECK(field_polygon_contains(ring_shape, anchor)); CHECK(anchor == field_anchor(ring_shape));
    CHECK(field_polygon_contains(poly, field_anchor(poly)));

    CHECK(field_polygon_touches_box(poly, {-5000, -5000, 100, 100}));   // a corner of the outline
    CHECK(field_polygon_touches_box(poly, {20000, 20000, 30000, 30000}) == false);
    CHECK(field_polygon_touches_box(poly, {4500, 1500, 5000, 2000}));   // inside the polygon, no outline segment touches the box
    CHECK(field_polygon_touches_box(poly, {-1000, -1000, 11000, 5000})); // the polygon inside the box
}

// ---- codec ------------------------------------------------------------------------------------------------------------

namespace {
FieldTile sample_tile() {
    Scene s; s.field(1000, 100, 100, 300, 200, "farmland", "wheat");
    s.way(2000, {{200, 50}, {200, 250}}, {{"highway", "track"}});
    s.field(1001, 400, 400, 600, 500, "meadow");
    return fields_of(s.build(), s.home());
}
} // namespace

TEST_CASE("Field tile codec round-trips and refuses every structural violation", "[layer][fields]") {
    const auto tile = sample_tile();
    REQUIRE(tile.fields.size() == 3);
    const auto body = encode_fields_geom(tile); REQUIRE(body);
    const auto decoded = decode_fields_geom(*body); REQUIRE(decoded);
    CHECK(*decoded == tile);
    const auto again = encode_fields_geom(*decoded); REQUIRE(again);
    CHECK(again->sections[0].data == body->sections[0].data);
    // An empty tile is valid.
    FieldTile empty; empty.key = tile.key;
    const auto empty_body = encode_fields_geom(empty); REQUIRE(empty_body); CHECK(decode_fields_geom(*empty_body));

    // Truncation at every byte, trailing bytes, the wrong encoding version.
    const auto& data = body->sections[0].data;
    for (std::size_t n = 0; n < data.size(); ++n) {
        auto cut = *body; cut.sections[0].data.resize(n);
        CHECK_FALSE(decode_fields_geom(cut));
    }
    auto extra = *body; extra.sections[0].data.push_back(0); CHECK_FALSE(decode_fields_geom(extra));
    auto wrong = *body; ++wrong.encoding_version; CHECK_FALSE(decode_fields_geom(wrong));
    // A single flipped bit never decodes to a tile that fails its own validation.
    for (std::size_t i = 0; i < data.size(); i += 7) {
        auto flipped = *body; flipped.sections[0].data[i] ^= 0x10;
        const auto result = decode_fields_geom(flipped);
        if (result) CHECK(validate_field_tile(*result));
    }
    // Count bomb: a million-million fields before any allocation.
    ByteWriter bomb; bomb.u64(tile.key.packed()); bomb.varint(1ull << 40);
    CHECK_FALSE(decode_fields_geom({kFieldsGeomEncoding, 1, {{1, SectionCodec::None, bomb.take()}}}));

    const auto reject = [&](const char* what, auto&& edit) {
        auto bad = tile; edit(bad);
        INFO(what);
        CHECK_FALSE(validate_field_tile(bad));
        CHECK_FALSE(encode_fields_geom(bad));
    };
    reject("area", [](FieldTile& t) { ++t.fields[0].area_mm2; });
    reject("row direction", [](FieldTile& t) { t.fields[0].row_dir_cdeg = static_cast<std::uint16_t>((t.fields[0].row_dir_cdeg + 1) % 18000); });
    reject("row edge", [](FieldTile& t) { ++t.fields[0].row_edge_mm; });
    reject("anchor", [](FieldTile& t) { t.fields[0].anchor.x += 1; });
    reject("owned flag", [](FieldTile& t) { t.fields[0].flags ^= field_flag::Owned; });
    reject("unknown flag", [](FieldTile& t) { t.fields[0].flags |= 1u << 20; });
    reject("id order", [](FieldTile& t) { std::swap(t.fields[0], t.fields[1]); });
    reject("duplicate id", [](FieldTile& t) { t.fields[1] = t.fields[0]; });
    reject("id vs parts", [](FieldTile& t) { ++t.fields[0].piece; });
    reject("clockwise outer", [](FieldTile& t) { std::reverse(t.fields[0].polygon.outer.begin(), t.fields[0].polygon.outer.end()); });
    reject("outer start", [](FieldTile& t) { std::rotate(t.fields[0].polygon.outer.begin(), t.fields[0].polygon.outer.begin() + 1, t.fields[0].polygon.outer.end()); });
    reject("two vertices", [](FieldTile& t) { t.fields[0].polygon.outer.resize(2); });
    reject("crop too long", [](FieldTile& t) { t.fields[0].crop.assign(kFieldsMaxCrop + 1, 'x'); });
    reject("decline order", [](FieldTile& t) { t.declines = {{FieldSource::OsmWay, 9, FieldDeclineReason::TooLarge}, {FieldSource::OsmWay, 3, FieldDeclineReason::TooLarge}}; });
    reject("decline reason", [](FieldTile& t) { t.declines = {{FieldSource::OsmWay, 9, static_cast<FieldDeclineReason>(77)}}; });
    reject("outside the tile", [](FieldTile& t) {
        for (auto& p : t.fields[0].polygon.outer) p.x += 5000000;
    });
    reject("hole outside the outer", [](FieldTile& t) {
        auto hole = square_ring(-9000, -9000, -8000, -8000); normalise_field_ring(hole, false);
        t.fields[0].polygon.holes.push_back(hole);
    });
}

TEST_CASE("A tile carrying a reserved Parcel source decodes", "[layer][fields]") {
    auto tile = sample_tile();
    // Re-id the first field as a parcel; recomputed fields keep the format valid.
    auto& f = tile.fields[0];
    f.source = FieldSource::Parcel; f.osm_id = 4242; f.piece = 0; f.id = *pack_field_id(f.source, f.osm_id, f.piece);
    std::sort(tile.fields.begin(), tile.fields.end(), [](const auto& a, const auto& b) { return a.id < b.id; });
    const auto body = encode_fields_geom(tile); REQUIRE(body);
    const auto back = decode_fields_geom(*body); REQUIRE(back);
    CHECK(*back == tile);
    CHECK(back->fields.back().source == FieldSource::Parcel);
}

// ---- subdivision kernel ------------------------------------------------------------------------------------------------

namespace {
FieldPolygon rect_polygon(std::int64_t x0, std::int64_t y0, std::int64_t x1, std::int64_t y1) {
    FieldPolygon p; p.outer = square_ring(x0, y0, x1, y1); normalise_field_ring(p.outer, true); return p;
}
FieldSplitSegment seg(double x0, double y0, double x1, double y1, std::int32_t half_width_mm) {
    return {{std::llround(x0 * 1000), std::llround(y0 * 1000)}, {std::llround(x1 * 1000), std::llround(y1 * 1000)}, half_width_mm};
}
FieldObstacle obstacle_rect(std::int64_t x0, std::int64_t y0, std::int64_t x1, std::int64_t y1) {
    FieldObstacle o; o.rings.push_back(square_ring(x0, y0, x1, y1)); o.bbox = {x0, y0, x1, y1}; return o;
}
double piece_m2(const FieldPolygon& p) { return static_cast<double>(field_polygon_area_mm2(p)) / 1e6; }
} // namespace

TEST_CASE("Subdivision kernel: an uncut polygon is returned verbatim", "[layer][fields][fields_kernel]") {
    const auto& rules = builtin_field_rules_de();
    const auto poly = rect_polygon(0, 0, 100000, 50000);
    const auto result = subdivide_polygon(rules, poly, {}, {}); REQUIRE(result);
    CHECK(result->verbatim); CHECK(result->pieces.empty()); CHECK_FALSE(result->declined);
    // A line outside the polygon cuts nothing.
    const std::vector<FieldSplitSegment> far{seg(200, 0, 200, 50, 2000)};
    const auto none = subdivide_polygon(rules, poly, far, {}); REQUIRE(none); CHECK(none->verbatim);
}

TEST_CASE("Subdivision kernel: a crossing line splits, the piece order is south-west first", "[layer][fields][fields_kernel]") {
    const auto& rules = builtin_field_rules_de();
    const auto poly = rect_polygon(0, 0, 100000, 50000);
    const std::vector<FieldSplitSegment> v{seg(50, -5, 50, 55, 2000)};
    const auto r = subdivide_polygon(rules, poly, v, {}); REQUIRE(r);
    REQUIRE_FALSE(r->verbatim); REQUIRE(r->pieces.size() == 2);
    CHECK(r->pieces[0].outer.front().x == 0);                       // west piece first
    CHECK(piece_m2(r->pieces[0]) == Catch::Approx(48.0 * 50).epsilon(0.03));
    CHECK(piece_m2(r->pieces[1]) == Catch::Approx(48.0 * 50).epsilon(0.03));
    for (const auto& p : r->pieces) { CHECK(field_polygon_simple(p)); CHECK(ring_twice_area_mm2(p.outer) > 0); }
    // A horizontal line orders the southern piece first.
    const std::vector<FieldSplitSegment> h{seg(-5, 25, 105, 25, 2000)};
    const auto rh = subdivide_polygon(rules, poly, h, {}); REQUIRE(rh); REQUIRE(rh->pieces.size() == 2);
    CHECK(ring_bbox(rh->pieces[0].outer).s < ring_bbox(rh->pieces[1].outer).s);
    // The cut is as wide as the splitter says: a wider half width eats more.
    const std::vector<FieldSplitSegment> wide{seg(50, -5, 50, 55, 7000)};
    const auto rw = subdivide_polygon(rules, poly, wide, {}); REQUIRE(rw); REQUIRE(rw->pieces.size() == 2);
    CHECK(piece_m2(rw->pieces[0]) < piece_m2(r->pieces[0]));
    // Deterministic.
    const auto again = subdivide_polygon(rules, poly, v, {}); REQUIRE(again);
    CHECK(again->pieces == r->pieces);
}

TEST_CASE("Subdivision kernel: a diagonal line gives straight, simple pieces", "[layer][fields][fields_kernel]") {
    const auto& rules = builtin_field_rules_de();
    const auto poly = rect_polygon(0, 0, 100000, 100000);
    const std::vector<FieldSplitSegment> d{seg(-10, -10, 110, 110, 2000)};
    const auto r = subdivide_polygon(rules, poly, d, {}); REQUIRE(r);
    REQUIRE(r->pieces.size() == 2);
    for (const auto& p : r->pieces) {
        CHECK(field_polygon_simple(p));
        CHECK(piece_m2(p) == Catch::Approx(5000.0 - 2.0 * 141.421356).epsilon(0.02));
        // Douglas-Peucker removed the raster staircase: far fewer vertices than the ~140 boundary cells.
        CHECK(p.outer.size() < 40);
    }
}

TEST_CASE("Subdivision kernel: holes, obstacles and the share cap", "[layer][fields][fields_kernel]") {
    const auto& rules = builtin_field_rules_de();
    const auto poly = rect_polygon(0, 0, 200000, 100000);
    // A pond removing 2.5 % leaves one piece of 97.5 % >= single_piece_keep_percent: the outline is kept.
    const auto pond = obstacle_rect(90000, 40000, 110000, 60000);
    const FieldObstacle* const small[] = {&pond};
    const auto keep = subdivide_polygon(rules, poly, {}, small); REQUIRE(keep);
    CHECK(keep->verbatim);
    // A big lake removing 25 %: one piece with a hole.
    const auto lake = obstacle_rect(50000, 25000, 150000, 75000);
    const FieldObstacle* const big[] = {&lake};
    const auto holed = subdivide_polygon(rules, poly, {}, big); REQUIRE(holed);
    REQUIRE_FALSE(holed->verbatim); REQUIRE(holed->pieces.size() == 1);
    CHECK(holed->pieces[0].holes.size() == 1);
    CHECK(piece_m2(holed->pieces[0]) == Catch::Approx(15000.0).epsilon(0.02));
    CHECK(ring_twice_area_mm2(holed->pieces[0].holes[0]) < 0);
    CHECK(field_polygon_simple(holed->pieces[0]));
    // A strip across the field splits it in two.
    const auto strip = obstacle_rect(90000, -10000, 110000, 110000);
    const FieldObstacle* const cutter[] = {&strip};
    const auto two = subdivide_polygon(rules, poly, {}, cutter); REQUIRE(two);
    CHECK(two->pieces.size() == 2);
    // An obstacle covering 80 % is treated as a mapping overlap: ignored, field kept whole.
    const auto cover = obstacle_rect(-1000, -1000, 161000, 101000);
    const FieldObstacle* const huge[] = {&cover};
    const auto capped = subdivide_polygon(rules, poly, {}, huge); REQUIRE(capped);
    CHECK(capped->obstacles_ignored); CHECK(capped->verbatim);
    // The cap is read from the rules.
    auto tight = rules; tight.obstacle_share_cap_percent = 20;
    const auto capped2 = subdivide_polygon(tight, poly, {}, big); REQUIRE(capped2);
    CHECK(capped2->obstacles_ignored);
}

TEST_CASE("Subdivision kernel: slivers are dropped and a block of slivers is kept whole", "[layer][fields][fields_kernel]") {
    const auto& rules = builtin_field_rules_de();
    const auto poly = rect_polygon(0, 0, 100000, 100000);
    // Tracks at x = 40 and x = 97: a 1 m strip (area 100 m2 < 300) is a sliver, the other two pieces stay.
    const std::vector<FieldSplitSegment> lines{seg(40, -5, 40, 105, 2000), seg(97, -5, 97, 105, 2000)};
    const auto r = subdivide_polygon(rules, poly, lines, {}); REQUIRE(r);
    CHECK(r->pieces.size() == 2); CHECK(r->slivers_dropped == 1); CHECK_FALSE(r->sliver_block);
    // A 10 m x 10 m field cut in two: both halves are slivers -> the block stays whole.
    const auto tiny = rect_polygon(0, 0, 10000, 10000);
    const std::vector<FieldSplitSegment> mid{seg(5, -2, 5, 12, 1000)};
    const auto s = subdivide_polygon(rules, tiny, mid, {}); REQUIRE(s);
    CHECK(s->sliver_block); CHECK(s->verbatim);
    // A long thin strip fails the width rule even when its area is large.
    const auto thin = rect_polygon(0, 0, 400000, 3000);
    const std::vector<FieldSplitSegment> across{seg(200, -2, 200, 5, 600)};
    const auto t = subdivide_polygon(rules, thin, across, {}); REQUIRE(t);
    CHECK(t->sliver_block);
    // Sliver thresholds are read from the rules.
    auto lenient = rules; lenient.sliver_min_area_m2 = 10; lenient.sliver_min_width_mm = 500;
    const auto l = subdivide_polygon(lenient, poly, lines, {}); REQUIRE(l);
    CHECK(l->pieces.size() == 3); CHECK(l->slivers_dropped == 0);
}

TEST_CASE("Subdivision kernel: the raster budget declines instead of allocating", "[layer][fields][fields_kernel]") {
    auto rules = builtin_field_rules_de();
    rules.max_cells = 10000;
    const auto poly = rect_polygon(0, 0, 500000, 500000);
    const std::vector<FieldSplitSegment> v{seg(250, -5, 250, 505, 2000)};
    const auto r = subdivide_polygon(rules, poly, v, {}); REQUIRE(r);
    REQUIRE(r->declined); CHECK(*r->declined == FieldDeclineReason::TooManyCells);
}

TEST_CASE("Subdivision kernel: pieces joined only at a corner stay apart and every piece is simple", "[layer][fields][fields_kernel]") {
    const auto& rules = builtin_field_rules_de();
    const auto poly = rect_polygon(0, 0, 100000, 100000);
    // Two crossing lines make four quadrants that touch only at the crossing.
    const std::vector<FieldSplitSegment> cross{seg(50, -5, 50, 105, 2000), seg(-5, 50, 105, 50, 2000)};
    const auto r = subdivide_polygon(rules, poly, cross, {}); REQUIRE(r);
    REQUIRE(r->pieces.size() == 4);
    for (const auto& p : r->pieces) { CHECK(field_polygon_simple(p)); CHECK(piece_m2(p) == Catch::Approx(48.0 * 48).epsilon(0.03)); }
    // Pieces are disjoint: no piece's anchor lies in another piece.
    for (std::size_t i = 0; i < r->pieces.size(); ++i)
        for (std::size_t j = 0; j < r->pieces.size(); ++j)
            if (i != j) CHECK_FALSE(field_polygon_contains(r->pieces[j], field_anchor(r->pieces[i])));
}

TEST_CASE("Outline simplification keeps simple rings simple", "[layer][fields][fields_kernel]") {
    // A staircase simplifies to its two end points' diagonal, within the tolerance.
    Ring stairs = {{0, 0}, {10000, 0}, {10000, 1000}, {9000, 1000}, {9000, 2000}, {8000, 2000}, {8000, 3000}, {0, 3000}};
    const auto simple = simplify_field_ring(stairs, 2000);
    CHECK(simple.size() <= stairs.size()); CHECK(simple.size() >= 3);
    FieldPolygon p; p.outer = simple; normalise_field_ring(p.outer, true);
    CHECK(field_polygon_simple(p));
    // Tolerance zero changes nothing but collinear points.
    const auto same = simplify_field_ring(stairs, 0); CHECK(same == stairs);
    // A bow tie is not simple.
    FieldPolygon bow; bow.outer = {{0, 0}, {10000, 10000}, {10000, 0}, {0, 10000}};
    CHECK_FALSE(field_polygon_simple(bow));
    // A hole crossing the outer ring is not simple.
    FieldPolygon crossing = rect_polygon(0, 0, 10000, 10000); crossing.holes.push_back(square_ring(8000, 2000, 12000, 4000));
    CHECK_FALSE(field_polygon_simple(crossing));
}

// ---- deriver on synthetic OSM worlds -----------------------------------------------------------------------------------

TEST_CASE("An uncut field keeps its OSM outline and carries landuse, crop, area and row direction", "[layer][fields][fields_derive]") {
    Scene s; s.field(1000, 100, 100, 300, 200, "farmland", "wheat");
    const auto tile = fields_of(s.build(), s.home());
    REQUIRE(tile.fields.size() == 1);
    const auto& f = tile.fields[0];
    CHECK(f.id == *pack_field_id(FieldSource::OsmWay, 1000, 0));
    CHECK(f.source == FieldSource::OsmWay); CHECK(f.osm_id == 1000); CHECK(f.piece == 0);
    CHECK(f.landuse == 0); CHECK(f.crop == "wheat");
    CHECK((f.flags & field_flag::Owned) != 0); CHECK((f.flags & field_flag::Subdivided) == 0);
    CHECK(near_ratio(f, 20000.0) == Catch::Approx(1.0).epsilon(0.001));
    CHECK(cdeg_near(f.row_dir_cdeg, 0)); CHECK(f.row_edge_mm == Catch::Approx(200000).epsilon(0.001));
    CHECK(f.polygon.outer.size() == 4); CHECK(f.polygon.holes.empty());
    CHECK((f.flags & (field_flag::SmallArea | field_flag::LargeArea | field_flag::Thin)) == 0);
    CHECK(tile.counts.blocks == 1); CHECK(tile.counts.subdivided == 0);
    CHECK(tile.declines.empty());
    // Point query.
    const auto hit = field_at(tile, at(s.home(), 200, 150)); REQUIRE(hit); CHECK(tile.fields[*hit].id == f.id);
    CHECK_FALSE(field_at(tile, at(s.home(), 50, 150)));
    CHECK_FALSE(field_at(tile, at(s.home(), 200, 250)));
    CHECK(field_polygon_contains(f.polygon, f.anchor));
}

TEST_CASE("Every landuse of the rules becomes a field and other landuse does not", "[layer][fields][fields_derive]") {
    Scene s; double x = 50;
    for (const char* lu : {"farmland", "meadow", "orchard", "vineyard", "farmyard"}) { s.field(static_cast<std::int64_t>(x), x, 100, x + 100, 200, lu); x += 150; }
    s.field(5000, 50, 300, 150, 400, "forest");
    s.field(5001, 250, 300, 350, 400, "residential");
    s.way(5002, {{450, 300}, {550, 300}, {550, 400}}, {{"landuse", "farmland"}}); // an open way is no area
    const auto tile = fields_of(s.build(), s.home());
    REQUIRE(tile.fields.size() == 5);
    std::vector<int> codes; for (const auto& f : tile.fields) codes.push_back(f.landuse);
    std::sort(codes.begin(), codes.end());
    CHECK(codes == std::vector<int>{0, 1, 2, 3, 4});
    CHECK(tile.counts.blocks == 5);
}

TEST_CASE("Linear features split a block; bridges and tunnels do not", "[layer][fields][fields_derive]") {
    struct Case { const char* name; Tags tags; bool splits; };
    const std::vector<Case> cases = {
        {"track", {{"highway", "track"}}, true},
        {"path", {{"highway", "path"}}, true},
        {"service road", {{"highway", "service"}}, true},
        {"residential", {{"highway", "residential"}}, true},
        {"primary", {{"highway", "primary"}}, true},
        {"railway", {{"railway", "rail"}}, true},
        {"ditch", {{"waterway", "ditch"}}, true},
        {"stream", {{"waterway", "stream"}}, true},
        {"river", {{"waterway", "river"}}, true},
        {"hedge", {{"barrier", "hedge"}}, true},
        {"wall", {{"barrier", "wall"}}, true},
        {"tree row", {{"natural", "tree_row"}}, true},
        {"fence is no splitter", {{"barrier", "fence"}}, false},
        {"power line is no splitter", {{"power", "line"}}, false},
        {"bridge", {{"highway", "track"}, {"bridge", "yes"}}, false},
        {"tunnel", {{"highway", "track"}, {"tunnel", "yes"}}, false},
        {"layer -1", {{"highway", "track"}, {"layer", "-1"}}, false},
        {"layer 1 still splits", {{"highway", "track"}, {"layer", "1"}}, true},
        {"bridge=no still splits", {{"highway", "track"}, {"bridge", "no"}}, true},
    };
    for (const auto& c : cases) {
        INFO(c.name);
        Scene s; s.field(1000, 100, 100, 300, 200);
        s.way(2000, {{200, 50}, {200, 250}}, c.tags);
        const auto tile = fields_of(s.build(), s.home());
        CHECK(count_osm(tile, 1000) == (c.splits ? 2u : 1u));
        if (c.splits) {
            const auto* a = find_piece(tile, 1000, 0); const auto* b = find_piece(tile, 1000, 1);
            REQUIRE(a); REQUIRE(b);
            CHECK((a->flags & field_flag::Subdivided) != 0); CHECK((b->flags & field_flag::Subdivided) != 0);
            CHECK(a->polygon.outer.front().x < b->polygon.outer.front().x);
            // The gap is not in either field.
            CHECK_FALSE(field_at(tile, at(s.home(), 200, 150)));
            CHECK(field_at(tile, at(s.home(), 150, 150))); CHECK(field_at(tile, at(s.home(), 250, 150)));
            CHECK(tile.counts.subdivided == 1);
        } else {
            CHECK((tile.fields[0].flags & field_flag::Subdivided) == 0);
            CHECK(near_ratio(tile.fields[0], 20000.0) == Catch::Approx(1.0).epsilon(0.001));
        }
    }
}

TEST_CASE("A track that does not reach across leaves the field whole", "[layer][fields][fields_derive]") {
    Scene s; s.field(1000, 100, 100, 300, 200);
    s.way(2000, {{200, 80}, {200, 150}}, {{"highway", "track"}}); // enters from the south, stops inside
    const auto tile = fields_of(s.build(), s.home());
    REQUIRE(tile.fields.size() == 1);
    CHECK((tile.fields[0].flags & field_flag::Subdivided) == 0);
    CHECK(near_ratio(tile.fields[0], 20000.0) == Catch::Approx(1.0).epsilon(0.001));
}

TEST_CASE("Piece sizes follow the splitter widths and the pieces are ordered deterministically", "[layer][fields][fields_derive]") {
    Scene s; s.field(1000, 100, 100, 400, 200);
    s.way(2000, {{200, 50}, {200, 250}}, {{"highway", "track"}});       // half width 2 m
    s.way(2001, {{300, 50}, {300, 250}}, {{"highway", "primary"}});     // half width 7 m
    const auto tile = fields_of(s.build(), s.home());
    REQUIRE(count_osm(tile, 1000) == 3);
    const auto* a = find_piece(tile, 1000, 0); const auto* b = find_piece(tile, 1000, 1); const auto* c = find_piece(tile, 1000, 2);
    REQUIRE(a); REQUIRE(b); REQUIRE(c);
    CHECK(near_ratio(*a, 98.0 * 100) == Catch::Approx(1.0).epsilon(0.03));
    CHECK(near_ratio(*b, (293.0 - 202.0) * 100) == Catch::Approx(1.0).epsilon(0.03));
    CHECK(near_ratio(*c, (400.0 - 307.0) * 100) == Catch::Approx(1.0).epsilon(0.03));
}

TEST_CASE("Water, buildings and other land use are cut out; a small pond keeps the outline; an overlap is capped", "[layer][fields][fields_derive]") {
    {   // a lake taking 25 % leaves a hole
        Scene s; s.field(1000, 100, 100, 300, 200);
        s.way(2000, Scene::rect(150, 125, 250, 175), {{"natural", "water"}});
        const auto tile = fields_of(s.build(), s.home());
        REQUIRE(tile.fields.size() == 1);
        CHECK(tile.fields[0].polygon.holes.size() == 1);
        CHECK(near_ratio(tile.fields[0], 15000.0) == Catch::Approx(1.0).epsilon(0.03));
        CHECK_FALSE(field_at(tile, at(s.home(), 200, 150)));
        CHECK(field_at(tile, at(s.home(), 120, 150)));
    }
    {   // a farm pond of 2 %: the OSM outline is kept
        Scene s; s.field(1000, 100, 100, 300, 200);
        s.way(2000, Scene::rect(190, 140, 210, 160), {{"natural", "water"}});
        const auto tile = fields_of(s.build(), s.home());
        REQUIRE(tile.fields.size() == 1);
        CHECK(tile.fields[0].polygon.holes.empty()); CHECK((tile.fields[0].flags & field_flag::Subdivided) == 0);
    }
    {   // a water body across the field splits it
        Scene s; s.field(1000, 100, 100, 300, 200);
        s.way(2000, Scene::rect(180, 50, 220, 250), {{"natural", "water"}});
        const auto tile = fields_of(s.build(), s.home());
        CHECK(count_osm(tile, 1000) == 2);
    }
    for (const auto& tags : std::vector<Tags>{{{"building", "yes"}}, {{"landuse", "residential"}}, {{"leisure", "pitch"}}, {{"amenity", "parking"}}}) {
        Scene s; s.field(1000, 100, 100, 300, 200);
        s.way(2000, Scene::rect(180, 50, 220, 250), tags);
        const auto tile = fields_of(s.build(), s.home());
        INFO(tags[0].first);
        CHECK(count_osm(tile, 1000) == 2);
    }
    {   // overlapping landuse (e.g. farmland inside a wood mapped on top): the cap ignores it
        Scene s; s.field(1000, 100, 100, 300, 200);
        s.way(2000, Scene::rect(90, 90, 310, 190), {{"landuse", "forest"}});
        const auto tile = fields_of(s.build(), s.home());
        REQUIRE(tile.fields.size() == 1);
        CHECK((tile.fields[0].flags & field_flag::ObstaclesIgnored) != 0);
        CHECK(near_ratio(tile.fields[0], 20000.0) == Catch::Approx(1.0).epsilon(0.001));
        CHECK(tile.counts.obstacles_ignored == 1);
    }
    {   // another farmland polygon is no obstacle
        Scene s; s.field(1000, 100, 100, 300, 200); s.field(1001, 150, 120, 250, 180);
        const auto tile = fields_of(s.build(), s.home());
        CHECK(tile.fields.size() == 2);
    }
}

TEST_CASE("Slivers between two close cutters are dropped and counted", "[layer][fields][fields_derive]") {
    Scene s; s.field(1000, 100, 100, 300, 200);
    s.way(2000, {{150, 50}, {150, 250}}, {{"highway", "track"}});
    s.way(2001, {{293, 50}, {293, 250}}, {{"highway", "track"}}); // leaves a 5 m strip east of the second cut: 5 * 100 = 500 m2 would stay, so use 297
    s.way(2002, {{297, 50}, {297, 250}}, {{"highway", "track"}});
    const auto tile = fields_of(s.build(), s.home());
    // Between x=295 (cut of 2001 ends at 295) and x=295 (cut of 2002 starts at 295) nothing remains, east of 299 a 1 m strip (sliver).
    CHECK(count_osm(tile, 1000) >= 2);
    CHECK(tile.counts.slivers_dropped >= 1);
    for (const auto& f : tile.fields) CHECK(f.area_m2() >= builtin_field_rules_de().sliver_min_area_m2);
}

TEST_CASE("A tiny block cut into slivers stays whole and is flagged", "[layer][fields][fields_derive]") {
    Scene s; s.field(1000, 100, 100, 110, 110);
    s.way(2000, {{105, 90}, {105, 120}}, {{"highway", "path"}});
    const auto tile = fields_of(s.build(), s.home());
    REQUIRE(tile.fields.size() == 1);
    CHECK((tile.fields[0].flags & field_flag::SliverBlock) != 0);
    CHECK((tile.fields[0].flags & field_flag::SmallArea) != 0);
    CHECK(tile.counts.sliver_blocks == 1);
    CHECK(near_ratio(tile.fields[0], 100.0) == Catch::Approx(1.0).epsilon(0.02));
}

TEST_CASE("Plausibility flags report implausible polygons but never drop them", "[layer][fields][fields_derive]") {
    Scene s;
    s.field(1000, 100, 100, 110, 110);     // 100 m2: small
    s.field(1001, 100, 200, 500, 202);     // 800 m2 but 2 m wide: thin
    s.field(1002, 100, 300, 300, 400);     // normal
    const auto tile = fields_of(s.build(), s.home());
    REQUIRE(tile.fields.size() == 3);
    CHECK((find_piece(tile, 1000, 0)->flags & field_flag::SmallArea) != 0);
    CHECK((find_piece(tile, 1001, 0)->flags & field_flag::Thin) != 0);
    CHECK((find_piece(tile, 1001, 0)->flags & field_flag::SmallArea) == 0);
    CHECK((find_piece(tile, 1002, 0)->flags & (field_flag::SmallArea | field_flag::Thin | field_flag::LargeArea)) == 0);
}

TEST_CASE("A multipolygon relation with a hole becomes one field with that hole", "[layer][fields][fields_derive]") {
    Scene s;
    const auto outer = s.way(10, Scene::rect(100, 100, 400, 300), {});
    const auto inner = s.way(11, Scene::rect(200, 150, 300, 200), {});
    using M = osm::OsmBuilder::MemberIn;
    s.relation(500, {M{osm::MemberType::Way, outer, "outer"}, M{osm::MemberType::Way, inner, "inner"}},
        {{"crop", "maize"}, {"landuse", "farmland"}, {"type", "multipolygon"}});
    const auto tile = fields_of(s.build(), s.home());
    REQUIRE(tile.fields.size() == 1);
    const auto& f = tile.fields[0];
    CHECK(f.source == FieldSource::OsmRelation); CHECK(f.osm_id == 500); CHECK(f.crop == "maize");
    CHECK(f.polygon.holes.size() == 1);
    CHECK(near_ratio(f, 60000.0 - 5000.0) == Catch::Approx(1.0).epsilon(0.002));
    CHECK_FALSE(field_at(tile, at(s.home(), 250, 175)));
    CHECK(field_at(tile, at(s.home(), 150, 175)));
    CHECK(f.id == *pack_field_id(FieldSource::OsmRelation, 500, 0));
}

TEST_CASE("A relation member way that is also tagged landuse is not a second field", "[layer][fields][fields_derive]") {
    Scene s;
    const auto outer = s.way(10, Scene::rect(100, 100, 400, 300), {{"landuse", "farmland"}});
    using M = osm::OsmBuilder::MemberIn;
    s.relation(500, {M{osm::MemberType::Way, outer, "outer"}}, {{"landuse", "farmland"}, {"type", "multipolygon"}});
    const auto tile = fields_of(s.build(), s.home());
    REQUIRE(tile.fields.size() == 1);
    CHECK(tile.fields[0].source == FieldSource::OsmRelation);
}

TEST_CASE("Declines are recorded: too large, invalid geometry", "[layer][fields][fields_derive]") {
    Scene s;
    s.field(1000, 100, 100, 3300, 300);                       // 3.2 km wide > 3000 m
    s.way(1001, {{100, 500}, {300, 700}, {300, 500}, {100, 700}, {100, 500}}, {{"landuse", "farmland"}}); // bow tie
    s.field(1002, 100, 800, 200, 900);
    const auto tile = fields_of(s.build(), s.home());
    REQUIRE(tile.fields.size() == 1); CHECK(tile.fields[0].osm_id == 1002);
    REQUIRE(tile.declines.size() == 2);
    CHECK(tile.declines[0].osm_id == 1000); CHECK(tile.declines[0].reason == FieldDeclineReason::TooLarge);
    CHECK(tile.declines[1].osm_id == 1001); CHECK(tile.declines[1].reason == FieldDeclineReason::InvalidGeometry);
}

TEST_CASE("A field across a tile border is the same record in both tiles and owned once", "[layer][fields][fields_derive]") {
    Scene s;
    const TileKey west = s.home(); auto east = west; ++east.x;
    s.field(1000, 900, 100, 1500, 200, "farmland", "barley");
    s.way(2000, {{1300, 50}, {1300, 250}}, {{"highway", "track"}});    // 276 m behind the border: beyond the 128 m halo of the west tile
    const auto world = s.build();
    const auto w = fields_of(world, west), e = fields_of(world, east);
    REQUIRE(count_osm(w, 1000) == 1); REQUIRE(count_osm(e, 1000) == 2);
    const auto* w0 = find_piece(w, 1000, 0); const auto* e0 = find_piece(e, 1000, 0); const auto* e1 = find_piece(e, 1000, 1);
    REQUIRE(w0); REQUIRE(e0); REQUIRE(e1);
    // Same record apart from the ownership flag.
    auto a = *w0, b = *e0; a.flags &= ~field_flag::Owned; b.flags &= ~field_flag::Owned;
    CHECK(a == b);
    CHECK(w0->crop == "barley");
    // The cut came from a feature the west tile never held itself.
    CHECK((w0->flags & field_flag::Subdivided) != 0);
    CHECK(near_ratio(*w0, (1298.0 - 900.0) * 100) == Catch::Approx(1.0).epsilon(0.03));
    // Owned exactly once, by the tile holding the anchor.
    int owners = 0;
    for (const auto* t : {&w, &e}) for (const auto& f : t->fields) if (f.osm_id == 1000 && f.piece == 0 && (f.flags & field_flag::Owned) != 0) ++owners;
    CHECK(owners == 1);
    CHECK(((w0->flags & field_flag::Owned) != 0) == (w0->anchor.x < east.min_easting() * 1000));
    // Block counts are owned by one tile.
    CHECK(w.counts.blocks + e.counts.blocks == 1);
    CHECK(w.counts.subdivided + e.counts.subdivided == 1);
    // A point query needs only the tile holding the point.
    CHECK(field_at(w, at(west, 1000, 150))); CHECK(field_at(e, at(west, 1100, 150)));
    CHECK(field_at(e, at(west, 1400, 150)));
    CHECK_FALSE(field_at(e, at(west, 1300, 150)));
}

TEST_CASE("A relation whose outline lies in another tile is completed by discovery", "[layer][fields][fields_derive]") {
    Scene s;
    const TileKey west = s.home(); auto east = west; ++east.x;
    using M = osm::OsmBuilder::MemberIn;
    // Way B (the far part of the ring) lies entirely beyond the west tile's 128 m halo (x > 1152).
    const auto a = s.way(10, {{1000, 100}, {1000, 300}, {1160, 300}}, {});
    const auto b = s.way(11, {{1160, 300}, {1400, 300}, {1400, 100}, {1160, 100}}, {});
    const auto c = s.way(12, {{1160, 100}, {1000, 100}}, {});
    s.relation(500, {M{osm::MemberType::Way, a, "outer"}, M{osm::MemberType::Way, b, "outer"}, M{osm::MemberType::Way, c, "outer"}},
        {{"landuse", "farmland"}, {"type", "multipolygon"}});
    const auto world = s.build();
    // Without discovery the west tile says so.
    const auto bare = decode_run(run_fields(world, west, false));
    CHECK(bare.fields.empty());
    REQUIRE(bare.declines.size() == 1);
    CHECK(bare.declines[0].source == FieldSource::OsmRelation); CHECK(bare.declines[0].reason == FieldDeclineReason::RelationIncomplete);
    // additional_inputs_for names the tile with the missing member.
    {
        FieldsGeomDeriver d; std::vector<InputTile> in{make_input(d.inputs_for(west)[0], world)};
        const auto more = d.additional_inputs_for({west, 0, in}); REQUIRE(more);
        REQUIRE_FALSE(more->empty());
        CHECK(std::any_of(more->begin(), more->end(), [&](const auto& r) { return r.key == east; }));
    }
    const auto full = run_fields(world, west); const auto w = decode_run(full);
    CHECK(full.stages_used >= 1);
    REQUIRE(w.fields.size() == 1); CHECK(w.declines.empty());
    CHECK(near_ratio(w.fields[0], 400.0 * 200) == Catch::Approx(1.0).epsilon(0.002));
    const auto e = fields_of(world, east);
    REQUIRE(e.fields.size() == 1);
    auto x = w.fields[0], y = e.fields[0]; x.flags &= ~field_flag::Owned; y.flags &= ~field_flag::Owned;
    CHECK(x == y);
}

TEST_CASE("A block reaching into a tile the store does not have is an incomplete relation", "[layer][fields][fields_derive]") {
    Scene s;
    const TileKey west = s.home(); auto east = west; ++east.x;
    using M = osm::OsmBuilder::MemberIn;
    // The only member way starts inside the west tile's halo but the block lies in the east tile.
    const auto outer = s.way(10, Scene::rect(1100, 100, 1400, 300), {});
    s.relation(500, {M{osm::MemberType::Way, outer, "outer"}}, {{"landuse", "farmland"}, {"type", "multipolygon"}});
    const auto world = s.build();
    const auto present = decode_run(run_fields(world, east));
    REQUIRE(present.fields.size() == 1);            // the east tile holds the field
    CHECK(decode_run(run_fields(world, west)).declines.empty());
    const auto gone = decode_run(run_fields(world, west, true, false, {east}));
    CHECK(gone.fields.empty());
    REQUIRE(gone.declines.size() == 1);
    CHECK(gone.declines[0].reason == FieldDeclineReason::RelationIncomplete);
}

TEST_CASE("Derivation is independent of input order and repeatable; far tiles are empty", "[layer][fields][fields_derive]") {
    Scene s;
    const TileKey west = s.home();
    s.field(1000, 900, 100, 1500, 200); s.way(2000, {{1300, 50}, {1300, 250}}, {{"highway", "track"}});
    s.field(1001, 100, 400, 300, 500, "meadow");
    const auto world = s.build();
    const auto a = run_fields(world, west), b = run_fields(world, west, true, true), c = run_fields(world, west);
    REQUIRE(a.result); REQUIRE(b.result); REQUIRE(c.result);
    CHECK(a.result->body == b.result->body); CHECK(a.result->body == c.result->body);
    auto far = west; far.x += 10;
    const auto none = decode_run(run_fields(world, far));
    CHECK(none.fields.empty()); CHECK(none.declines.empty()); CHECK(none.counts == FieldTileCounts{});
    // Bad requests fail instead of guessing.
    FieldsGeomDeriver d;
    CHECK(d.inputs_for(TileKey{g2m_test::kZone32N, 1, 1, 1}).empty());
    CHECK_FALSE(d.derive({TileKey{g2m_test::kZone32N, 1, 1, 1}, 0, {}}));
    CHECK_FALSE(d.derive({west, 0, {}}));
    auto inputs = a.inputs; for (auto& in : inputs) if (in.header) { in.header->body_hash[0] ^= 1; break; }
    CHECK_FALSE(d.derive({west, 0, inputs}));
    CHECK(d.discovery_stages() == IDeriver::kMaxDiscoveryStages);
    CHECK(d.output().id.name == kFieldsGeomLayer);
}

TEST_CASE("The config hash follows the rules file", "[layer][fields]") {
    FieldsGeomDeriver d;
    CHECK(d.config_hash() == FieldsGeomDeriver{}.config_hash());
    CHECK(d.output().id.name == "g2m.fields.geom");
    CHECK(d.output().inputs == std::vector<std::string>{"g2m.src.osm"});
}

// ---- matching saved fields ---------------------------------------------------------------------------------------------

TEST_CASE("match_field finds a saved field again after the data changed", "[layer][fields][fields_match]") {
    const auto& rules = builtin_field_rules_de();
    const TileKey home = g2m_test::home_tile();
    Scene before; before.field(1000, 100, 100, 400, 200); before.way(2000, {{250, 50}, {250, 250}}, {{"highway", "track"}});
    const auto tile = fields_of(before.build(), home);
    REQUIRE(tile.fields.size() == 2);
    const auto& east = tile.fields[1];
    const FieldRef saved{east.source, east.osm_id, east.piece, east.landuse, east.anchor};
    // Unchanged -> same id.
    auto m = match_field(saved, tile.fields, rules); CHECK(m.kind == FieldMatchKind::SameId); CHECK(m.index == 1);
    // The track was removed: the block is one piece 0 now; the anchor sits in it -> same block, other piece index.
    Scene gone; gone.field(1000, 100, 100, 400, 200);
    const auto t2 = fields_of(gone.build(), home);
    m = match_field(saved, t2.fields, rules); CHECK(m.kind == FieldMatchKind::SameBlock); CHECK(m.index == 0);
    // A new track on the west side shifts the piece indices: the saved piece 1 is now piece 2.
    Scene shifted; shifted.field(1000, 100, 100, 400, 200);
    shifted.way(2000, {{250, 50}, {250, 250}}, {{"highway", "track"}}); shifted.way(2001, {{150, 50}, {150, 250}}, {{"highway", "track"}});
    const auto t3 = fields_of(shifted.build(), home);
    REQUIRE(t3.fields.size() == 3);
    m = match_field(saved, t3.fields, rules); CHECK(m.kind == FieldMatchKind::SameBlock); CHECK(m.index == 2);
    // The way was redrawn under a new id: another element of the same landuse covers the anchor.
    Scene redrawn; redrawn.field(7777, 90, 90, 410, 210);
    const auto t4 = fields_of(redrawn.build(), home);
    m = match_field(saved, t4.fields, rules); CHECK(m.kind == FieldMatchKind::Moved); CHECK(m.index == 0);
    // Another landuse at that place is no match.
    Scene crop; crop.field(7777, 90, 90, 410, 210, "meadow");
    const auto t5 = fields_of(crop.build(), home);
    CHECK(match_field(saved, t5.fields, rules).kind == FieldMatchKind::None);
    // Nothing there.
    CHECK(match_field(saved, {}, rules).kind == FieldMatchKind::None);
    // Same block, anchor outside every piece: the nearest piece's anchor within the configured distance counts.
    Scene moved; moved.field(1000, 100, 100, 300, 200);
    const auto t6 = fields_of(moved.build(), home);
    m = match_field(saved, t6.fields, rules);
    CHECK(m.kind == FieldMatchKind::SameBlock);
    auto far_saved = saved; far_saved.anchor.x += 900000;
    CHECK(match_field(far_saved, t6.fields, rules).kind == FieldMatchKind::None);
}

// ---- server and bake -----------------------------------------------------------------------------------------------------

TEST_CASE("Field layer: server and parallel bake produce identical pinned containers", "[layer][fields][fields_bake]") {
    struct Temp {
        std::filesystem::path path = std::filesystem::temp_directory_path() /
            ("g2m_fields_bake_" + std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()));
        ~Temp() { std::error_code ec; std::filesystem::remove_all(path, ec); }
    } temp;
    auto sources = TileStore::open({temp.path, "sources"}); REQUIRE(sources);
    auto live = TileStore::open({temp.path, "live"}); REQUIRE(live);
    auto baked = TileStore::open({temp.path, "baked"}); REQUIRE(baked);
    Scene s; const TileKey west = s.home(); auto east = west; ++east.x;
    s.field(1000, 700, 300, 1350, 400, "farmland", "rye");
    s.way(2000, {{1200, 250}, {1200, 450}}, {{"highway", "track"}});
    const auto world = s.build();
    std::vector<TileKey> keys;
    for (int dx = -1; dx <= 2; ++dx) for (int dy = -1; dy <= 1; ++dy) { auto k = west; k.x += dx; k.y += dy; keys.push_back(k); }
    for (const auto& k : keys) {
        const auto source = make_input({std::string(kSrcOsmLayer), k}, world); REQUIRE(source.header);
        REQUIRE((*sources)->put({"fields_test", source.ref.layer, k}, *source.header, source.body, true));
    }
    BuiltinTerrainDerivers derivers;
    const auto manifest = make_local_release(**sources, builtin_local_release_params("fields_test", derivers, {}, false, false, false, false, true));
    REQUIRE(manifest);
    LocalSourceUpstream upstream(**sources);
    ServerConfig config; config.offline = true; config.derived_store = live->get();
    config.derivers = derivers.ordered(false, false, false, false, true); config.upstreams = {&upstream};
    Server server(config); REQUIRE(server.add_release(*manifest, "fields_test"));
    import::BakeOptions options; options.layers = {std::string(kFieldsGeomLayer)};
    options.min_level = options.max_level = 2; options.jobs = 4; options.pin = true;
    const auto report = import::bake(**sources, **baked, "fields_test", options); REQUIRE(report);
    CHECK(report->release_id_hex == to_hex(manifest->rid)); REQUIRE(report->layers.size() == 1);
    CHECK(report->layers[0].tiles_total == keys.size());
    for (const auto& k : {west, east}) {
        const auto response = server.tile({manifest->rid, std::string(kFieldsGeomLayer), k, {}});
        REQUIRE(response.meta.status == Status::Ok);
        const auto stored = (*baked)->get({report->release_id_hex, std::string(kFieldsGeomLayer), k});
        REQUIRE(stored); REQUIRE(*stored); CHECK((*stored)->pinned);
        CHECK((*stored)->container() == response.container);
        const auto body = decode_body((*stored)->body); REQUIRE(body);
        const auto tile = decode_fields_geom(*body); REQUIRE(tile);
        CHECK(count_osm(*tile, 1000) == (k == west ? 1u : 2u)); // piece 0 spans the border, piece 1 lies east
        // The same bytes as deriving straight from the source set.
        CHECK(*tile == fields_of(world, k));
    }
}

// ---- committed OSM fixtures ------------------------------------------------------------------------------------------------

namespace {
struct FixtureCensus {
    std::size_t tiles = 0, fields = 0, owned = 0, declines = 0, subdivided = 0, farmland_ways = 0;
    std::vector<FieldTile> all;
};
const FixtureCensus& census(const std::string& file) {
    static std::map<std::string, FixtureCensus> cache;
    if (const auto it = cache.find(file); it != cache.end()) return it->second;
    const World world(g2m_test::read_osm_fixture(std::string(G2M_SOURCE_DIR) + "/testdata/fixtures/osm/" + file));
    FixtureCensus c;
    for (const auto& w : world.data.ways) {
        const auto lu = world.data.tag(world.data.tags_of(w), "landuse");
        if (lu && builtin_field_rules_de().landuse_code(*lu) >= 0) ++c.farmland_ways;
    }
    std::set<TileKey> keys;
    const geo::Utm utm(geo::kGRS80);
    for (const auto& n : world.data.nodes) {
        const auto p = geom::project_e7_to_mm(utm, g2m_test::kZone32N, n.lat, n.lon);
        if (!p) continue;
        const auto k = tile_key_at(g2m_test::kZone32N, 2, static_cast<double>(p->x) / 1000.0, static_cast<double>(p->y) / 1000.0);
        if (k) keys.insert(*k);
    }
    for (const auto& key : keys) {
        auto tile = fields_of(world, key);
        ++c.tiles; c.fields += tile.fields.size(); c.declines += tile.declines.size(); c.subdivided += tile.counts.subdivided;
        for (const auto& f : tile.fields) if ((f.flags & field_flag::Owned) != 0) ++c.owned;
        c.all.push_back(std::move(tile));
    }
    return cache.emplace(file, std::move(c)).first->second;
}
} // namespace

TEST_CASE("Fixture census (prints field counts of the committed OSM fixtures)", "[.][diag][fields_fixture]") {
    for (const char* file : {"frankfurt_hoechst.osm.pbf", "hoechst_a66.osm.pbf"}) {
        const auto c = census(file);
        std::printf("%s: tiles %zu, landuse ways %zu, fields %zu (owned %zu), subdivided blocks %zu, declines %zu\n", file, c.tiles, c.farmland_ways, c.fields, c.owned, c.subdivided, c.declines);
    }
}

TEST_CASE("Fixture regions: every tile validates, a field is the same record in each tile it touches and is owned once", "[layer][fields][fields_fixture_check]") {
    for (const char* file : {"frankfurt_hoechst.osm.pbf", "hoechst_a66.osm.pbf"}) {
        INFO(file);
        const auto c = census(file);
        REQUIRE(c.fields > 0);
        std::map<std::uint64_t, FieldRecord> first;
        std::map<std::uint64_t, int> owners;
        std::set<std::pair<int, std::int64_t>> blocks;
        std::uint32_t counted_blocks = 0;
        for (const auto& tile : c.all) {
            counted_blocks += tile.counts.blocks;
            for (const auto& f : tile.fields) {
                auto stripped = f; stripped.flags &= ~field_flag::Owned;
                const auto [it, inserted] = first.emplace(f.id, stripped);
                if (!inserted) CHECK(it->second == stripped);
                if ((f.flags & field_flag::Owned) != 0) ++owners[f.id];
                blocks.insert({static_cast<int>(f.source), f.osm_id});
                // The anchor is a point of its own field, so the query in the owning tile finds a field there.
                if ((f.flags & field_flag::Owned) != 0) CHECK(field_at(tile, f.anchor));
            }
        }
        // Every field is owned by exactly one tile, as long as the tile holding its anchor was derived (it always is: the field touches it).
        CHECK(owners.size() == first.size());
        for (const auto& [id, n] : owners) CHECK(n == 1);
        // Declined blocks are the only blocks without fields; none are declined here, so the owned-block count is the number of blocks.
        CHECK(counted_blocks == blocks.size());
    }
}

TEST_CASE("Fixture regions: derivation is repeatable and independent of input order", "[layer][fields][fields_fixture_check]") {
    const World world(g2m_test::read_osm_fixture(std::string(G2M_SOURCE_DIR) + "/testdata/fixtures/osm/hoechst_a66.osm.pbf"));
    const auto c = census("hoechst_a66.osm.pbf");
    std::size_t checked = 0;
    for (const auto& tile : c.all) {
        if (tile.fields.empty() || checked >= 2) continue;
        ++checked;
        const auto a = run_fields(world, tile.key), b = run_fields(world, tile.key, true, true);
        REQUIRE(a.result); REQUIRE(b.result);
        CHECK(a.result->body == b.result->body);
    }
    CHECK(checked > 0);
}
