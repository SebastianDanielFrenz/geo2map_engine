// G4 slice 5: client map geometry (route_map.h). Synthetic scenes pin
// ownership, simplification, LOD filters, flags, strings and determinism;
// both OSM fixtures are tiled at every LOD with coverage and accuracy checks
// and a size/time print per LOD (never asserted).
#include "route_fixture_io.h"
#include "route_reference.h"
#include "route_scenes.h"
#include "splitmix64.h"
#include "synthetic.h"

#include "g2m/route/route_map.h"

#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <limits>
#include <map>
#include <set>
#include <string>
#include <vector>

using namespace g2m;
using namespace g2m::route;
using Tags = std::vector<std::pair<std::string, std::string>>;

namespace {

// Ranks from the default style and edges from the ways' refs (GraphBuilder
// leaves both to build_routing_graph).
RoadGraph finish(RoadGraph g) {
    for (auto& w : g.ways) {
        for (const auto& [k, v] : w.tags)
            if (k == "highway")
                if (const auto* e = RoadStyle::default_style().find_highway(v)) w.rank = e->rank;
        for (std::size_t i = 1; i < w.node_ids.size(); ++i)
            if (w.node_ids[i - 1] != w.node_ids[i])
                g.edges.push_back({w.osm_id, static_cast<std::uint32_t>(i - 1), w.node_ids[i - 1], w.node_ids[i]});
    }
    return g;
}

// Every level-2 tile holding a node or an edge midpoint, through the real codec.
std::vector<RoadGraphTile> tile_world(const RoadGraph& world) {
    std::set<TileKey> keys;
    const auto add = [&](std::int64_t x_mm, std::int64_t y_mm) {
        keys.insert(TileKey{g2m_test::kZone32N, 2, static_cast<std::int32_t>(floor_div(x_mm, 1024000)),
                            static_cast<std::int32_t>(floor_div(y_mm, 1024000))});
    };
    std::map<std::int64_t, geom::PointMm> pos;
    for (const auto& n : world.nodes) { add(n.position.x, n.position.y); pos[n.osm_id] = n.position; }
    for (const auto& e : world.edges)
        add(floor_div(pos[e.from_node].x + pos[e.to_node].x, 2), floor_div(pos[e.from_node].y + pos[e.to_node].y, 2));
    std::vector<RoadGraphTile> tiles;
    for (const auto& key : keys) {
        auto tile = select_road_graph_tile(world, key);
        REQUIRE(tile);
        auto body = encode_road_graph(*tile);
        REQUIRE(body);
        auto decoded = decode_road_graph(*body);
        REQUIRE(decoded);
        tiles.push_back(std::move(*decoded));
    }
    return tiles;
}

std::vector<const RoadGraphTile*> ptrs(const std::vector<RoadGraphTile>& tiles) {
    std::vector<const RoadGraphTile*> out;
    for (const auto& t : tiles) out.push_back(&t);
    return out;
}

TileKey up(TileKey k, int level) {
    while (k.level < level) k = k.parent();
    return k;
}

double point_polyline_distance(const geom::PointMm& p, std::span<const geom::PointMm> line) {
    double best = 1e300;
    for (std::size_t k = 0; k + 1 < line.size(); ++k) {
        const double vx = static_cast<double>(line[k + 1].x - line[k].x), vy = static_cast<double>(line[k + 1].y - line[k].y);
        const double wx = static_cast<double>(p.x - line[k].x), wy = static_cast<double>(p.y - line[k].y);
        const double len2 = vx * vx + vy * vy;
        double t = len2 > 0 ? (vx * wx + vy * wy) / len2 : 0.0;
        t = std::clamp(t, 0.0, 1.0);
        const double dx = wx - t * vx, dy = wy - t * vy;
        best = std::min(best, dx * dx + dy * dy);
    }
    if (line.size() == 1) {
        const double dx = static_cast<double>(p.x - line[0].x), dy = static_cast<double>(p.y - line[0].y);
        best = dx * dx + dy * dy;
    }
    return std::sqrt(best);
}

// A line's points back in absolute mm.
std::vector<geom::PointMm> line_mm(const MapTile& t, const MapLine& l) {
    std::vector<geom::PointMm> out;
    for (std::uint32_t k = 0; k < l.point_count; ++k) {
        const auto& p = t.points[l.point_first + k];
        out.push_back({t.key.min_easting() * 1000 + std::int64_t{p.x} * 100, t.key.min_northing() * 1000 + std::int64_t{p.y} * 100});
    }
    return out;
}

// Base aligned to level-2 tile (450, 5420); level-4 tile (112, 1355) holds L2 x 448..451.
constexpr std::int64_t kE0 = 460800, kN0 = 5550080;

RoadGraph scene() {
    g2m_test::GraphBuilder b;
    const auto node = [&](std::int64_t id, std::int64_t x, std::int64_t y) { b.node(id, kE0 + x, kN0 + y); };
    std::vector<std::int64_t> main;
    for (int k = 0; k < 19; ++k) { node(100 + k, 100 + 100 * k, 500); main.push_back(100 + k); }
    b.way(1, main, Tags{{"highway", "primary"}, {"name", "Hauptstrasse"}, {"ref", "B 8"}});
    node(201, 500, 100); node(202, 500, 250); node(203, 500, 400);
    b.way(2, {201, 202, 203}, Tags{{"highway", "residential"}, {"name", "Gasse"}, {"oneway", "yes"}});
    node(301, 1500, 100); node(302, 1500, 300);
    b.way(3, {301, 302}, Tags{{"highway", "motorway_link"}, {"oneway", "yes"}, {"bridge", "yes"}});
    node(401, 200, 900); node(402, 800, 900);
    b.way(4, {401, 402}, Tags{{"highway", "tertiary"}, {"tunnel", "yes"}, {"oneway", "-1"}, {"name", "Gasse"}});
    node(501, 1200, 700); node(502, 1260, 700); node(503, 1230, 750);
    b.way(5, {501, 502, 503, 501}, Tags{{"highway", "residential"}, {"junction", "roundabout"}});
    return finish(b.build());
}

} // namespace

TEST_CASE("simplify_polyline: Douglas-Peucker within tolerance", "[route][map]") {
    std::vector<geom::PointMm> straight;
    for (int k = 0; k <= 10; ++k) straight.push_back({k * 10000, 2 * k * 10000});
    CHECK(simplify_polyline(straight, 1000) == std::vector<geom::PointMm>{straight.front(), straight.back()});
    std::vector<geom::PointMm> zig;
    for (int k = 0; k <= 10; ++k) zig.push_back({k * 10000, (k % 2) * 800});
    CHECK(simplify_polyline(zig, 1000).size() == 2);
    CHECK(simplify_polyline(zig, 300).size() == zig.size());
    CHECK(simplify_polyline(std::span<const geom::PointMm>(zig.data(), 2), 1000).size() == 2);

    g2m_test::SplitMix64 rng(0x5a);
    for (int trial = 0; trial < 50; ++trial) {
        std::vector<geom::PointMm> walk{{0, 0}};
        for (int k = 0; k < 200; ++k)
            walk.push_back({walk.back().x + static_cast<std::int64_t>(rng.next() % 4001) - 1000,
                            walk.back().y + static_cast<std::int64_t>(rng.next() % 4001) - 2000});
        for (const std::int64_t tol : {1000, 8000}) {
            const auto s = simplify_polyline(walk, tol);
            REQUIRE(s.size() >= 2);
            CHECK(s.front() == walk.front());
            CHECK(s.back() == walk.back());
            for (const auto& p : walk) CHECK(point_polyline_distance(p, s) <= static_cast<double>(tol) + 1e-6);
            CHECK(simplify_polyline(walk, tol) == s);
        }
    }
}

TEST_CASE("Map tiles: ownership, LOD filters, flags, strings, determinism", "[route][map]") {
    const auto world = scene();
    const auto tiles = tile_world(world);
    const TileKey a{g2m_test::kZone32N, 2, 450, 5420}, b{g2m_test::kZone32N, 2, 451, 5420};
    const auto all = ptrs(tiles);

    MapTileStats sa, sb;
    const auto ta = build_map_tile(a, MapLod::Detail, all, default_map_config(), &sa);
    const auto tb = build_map_tile(b, MapLod::Detail, all, default_map_config(), &sb);
    REQUIRE(ta);
    REQUIRE(tb);
    CHECK(sa.source_tiles == 1);
    // The B 8 is split at x = 1024 m by edge midpoint: segments 0..8 in a, 9..17 in b, straight -> 2 points each.
    const auto main_a = std::find_if(ta->lines.begin(), ta->lines.end(), [](const MapLine& l) { return l.way_id == 1; });
    const auto main_b = std::find_if(tb->lines.begin(), tb->lines.end(), [](const MapLine& l) { return l.way_id == 1; });
    REQUIRE(main_a != ta->lines.end());
    REQUIRE(main_b != tb->lines.end());
    CHECK(main_a->seg_first == 0);
    CHECK(main_a->seg_last == 8);
    CHECK(main_b->seg_first == 9);
    CHECK(main_b->seg_last == 17);
    CHECK(main_a->point_count == 2);
    CHECK(ta->points[main_a->point_first] == MapPoint{1000, 5000});
    CHECK(ta->points[main_a->point_first + 1] == MapPoint{10000, 5000});
    CHECK(tb->points[main_b->point_first] == MapPoint{10000 - 10240, 5000}); // relative to b's corner
    CHECK(main_a->anchor == MapPoint{5500, 5000});
    CHECK(main_a->anchor_dx == 9000);
    CHECK(main_a->anchor_dy == 0);
    CHECK(ta->strings == std::vector<std::string>{"B 8", "Gasse", "Hauptstrasse"});
    CHECK(ta->strings[main_a->name] == "Hauptstrasse");
    CHECK(ta->strings[main_a->ref] == "B 8");

    const auto by_way = [](const MapTile& t, std::int64_t id) {
        const auto it = std::find_if(t.lines.begin(), t.lines.end(), [&](const MapLine& l) { return l.way_id == id; });
        REQUIRE(it != t.lines.end());
        return *it;
    };
    CHECK(by_way(*ta, 2).flags == kMapOneway);
    CHECK(by_way(*ta, 2).ref == kMapNoString);
    CHECK(by_way(*ta, 2).point_count == 2); // collinear middle point dropped
    CHECK(by_way(*tb, 3).flags == (kMapOneway | kMapBridge | kMapLink));
    CHECK(by_way(*ta, 4).flags == (kMapOnewayReverse | kMapTunnel));
    CHECK(by_way(*tb, 5).flags == (kMapOneway | kMapRoundabout));
    CHECK(by_way(*tb, 5).point_count == 4); // closed ring kept

    // Mid (L4): one tile holding both, one B 8 line over all 18 segments; residential gone.
    const TileKey mid = up(a, 4);
    CHECK(up(b, 4) == mid);
    MapTileStats sm;
    const auto tm = build_map_tile(mid, MapLod::Mid, all, default_map_config(), &sm);
    REQUIRE(tm);
    CHECK(sm.source_tiles == 2);
    std::vector<std::int64_t> ways;
    for (const auto& l : tm->lines) ways.push_back(l.way_id);
    CHECK(ways == std::vector<std::int64_t>{1, 3, 4}); // motorway_link shares the motorway rank
    CHECK(by_way(*tm, 1).seg_first == 0);
    CHECK(by_way(*tm, 1).seg_last == 17);
    CHECK(by_way(*tm, 1).point_count == 2);
    // Far (L6): primary and above.
    const auto tf = build_map_tile(up(a, 6), MapLod::Far, all);
    REQUIRE(tf);
    ways.clear();
    for (const auto& l : tf->lines) ways.push_back(l.way_id);
    CHECK(ways == std::vector<std::int64_t>{1, 3});
    CHECK(tf->strings == std::vector<std::string>{"B 8", "Hauptstrasse"});

    // Input order and unrelated tiles do not matter.
    auto shuffled = all;
    std::reverse(shuffled.begin(), shuffled.end());
    shuffled.push_back(nullptr);
    const auto tm2 = build_map_tile(mid, MapLod::Mid, shuffled);
    REQUIRE(tm2);
    CHECK(*tm2 == *tm);
    const std::vector<const RoadGraphTile*> only_b{&tiles[1]};
    const auto ta_without = build_map_tile(a, MapLod::Detail, only_b);
    REQUIRE(ta_without);
    CHECK(ta_without->lines.empty());
    CHECK(ta->byte_size() > 0);

    // Errors.
    CHECK(build_map_tile(a, MapLod::Mid, all).error().code == ErrorCode::InvalidArgument);
    RoadGraphTile l3 = tiles[0];
    l3.key = l3.key.parent();
    const std::vector<const RoadGraphTile*> bad{&l3};
    CHECK(build_map_tile(mid, MapLod::Mid, bad).error().code == ErrorCode::InvalidArgument);
    std::vector<const RoadGraphTile*> twice{&tiles[0], &tiles[0]};
    CHECK(build_map_tile(a, MapLod::Detail, twice).error().code == ErrorCode::Malformed);
    MapConfig tight = default_map_config();
    tight.max_lines = 1;
    CHECK(build_map_tile(a, MapLod::Detail, all, tight).error().code == ErrorCode::LimitExceeded);
}

TEST_CASE("route_polyline follows the route, cut at the snapped points", "[route][map][snap]") {
    const auto roads = g2m_test::cross();
    auto g = build_routing_graph(roads, {});
    REQUIRE(g);
    RouteSearcher searcher(*g);
    auto r = searcher.find(RouteQuery{2, 3}); // S -> C -> N
    REQUIRE(r);
    CHECK(route_polyline(*g, *r) == std::vector<geom::PointMm>{{0, -100000}, {0, 0}, {0, 100000}});
    SnapOptions o;
    auto from = g->snap({-2000, -30000}, o);
    auto to = g->snap({-60000, 3000}, o); // on C-W, 60 m west
    REQUIRE(from);
    REQUIRE(to);
    auto p = searcher.find(*from, *to);
    REQUIRE(p);
    REQUIRE(p->reachable);
    const auto line = route_polyline(*g, *p);
    CHECK(line == std::vector<geom::PointMm>{{0, -30000}, {0, 0}, {-60000, 0}});
    // The link way W-6-N, driven north to east: interior shape point kept.
    auto q = searcher.find(RouteQuery{4, 3});
    REQUIRE(q);
    const auto via = route_polyline(*g, *q);
    REQUIRE(via.size() >= 2);
    CHECK(via.front() == geom::PointMm{-100000, 0});
    CHECK(via.back() == geom::PointMm{0, 100000});
    CHECK(route_polyline(*g, Route{}).empty());
}

TEST_CASE("Map tiles on the real fixtures: coverage, accuracy, size and time", "[route][map][real-data]") {
    for (const char* name : {"frankfurt_hoechst", "hoechst_a66"}) {
        const auto data = g2m_test::read_route_fixture(name);
        const auto world = build_road_graph(data, g2m_test::kZone32N);
        REQUIRE(world);
        const auto tiles = tile_world(*world);
        const auto all = ptrs(tiles);
        std::map<std::int64_t, const RoadGraphWay*> way_of;
        for (const auto& w : world->ways) way_of[w.osm_id] = &w;
        std::map<std::int64_t, geom::PointMm> pos;
        for (const auto& n : world->nodes) pos[n.osm_id] = n.position;

        for (std::size_t li = 0; li < kMapLodCount; ++li) {
            const auto lod = static_cast<MapLod>(li);
            const MapLodConfig& lc = default_map_config().lods[li];
            std::set<TileKey> keys;
            for (const auto& t : tiles) keys.insert(up(t.key, lc.level));
            std::map<std::pair<std::int64_t, std::uint32_t>, int> covered;
            std::size_t lines = 0, points = 0, bytes = 0, max_bytes = 0, collapsed = 0, points_in = 0;
            double total_ms = 0, max_ms = 0;
            double worst_err = 0;
            for (const auto& key : keys) {
                MapTileStats st;
                const auto t0 = std::chrono::steady_clock::now();
                const auto tile = build_map_tile(key, lod, all, default_map_config(), &st);
                const double ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
                REQUIRE(tile);
                total_ms += ms;
                max_ms = std::max(max_ms, ms);
                lines += tile->lines.size();
                points += tile->points.size();
                points_in += st.points_in;
                bytes += tile->byte_size();
                max_bytes = std::max(max_bytes, tile->byte_size());
                collapsed += st.collapsed;
                auto reversed = all;
                std::reverse(reversed.begin(), reversed.end());
                const auto again = build_map_tile(key, lod, reversed);
                REQUIRE(again);
                CHECK(*again == *tile);
                for (const auto& l : tile->lines) {
                    REQUIRE(l.point_count >= 2);
                    CHECK(l.rank >= lc.min_rank);
                    const auto simple = line_mm(*tile, l);
                    const RoadGraphWay& w = *way_of.at(l.way_id);
                    for (std::uint32_t s = l.seg_first; s <= l.seg_last; ++s) {
                        if (w.node_ids[s] == w.node_ids[s + 1]) continue;
                        ++covered[{l.way_id, s}];
                        const double err = point_polyline_distance(pos.at(w.node_ids[s + 1]), simple);
                        worst_err = std::max(worst_err, err);
                    }
                    worst_err = std::max(worst_err, point_polyline_distance(pos.at(w.node_ids[l.seg_first]), simple));
                }
            }
            // Every drawn edge exactly once; every edge of a drawn class drawn (unless its line collapsed).
            std::size_t expected = 0, missing = 0;
            for (const auto& e : world->edges) {
                if (way_of.at(e.way_id)->rank < lc.min_rank) continue;
                ++expected;
                const auto it = covered.find({e.way_id, e.segment_index});
                if (it == covered.end()) ++missing;
                else CHECK(it->second == 1);
            }
            CHECK(covered.size() + missing == expected);
            CHECK(missing <= collapsed * 2);
            // Douglas-Peucker tolerance plus the decimetre rounding (half a decimetre per axis).
            CHECK(worst_err <= static_cast<double>(lc.tolerance_mm) + 71.0);
            std::printf("%s %-6s: %zu tiles (L%d), %zu lines, %zu -> %zu points, %zu collapsed, %zu/%zu edges, "
                        "%.1f KiB total, max %.1f KiB/tile, build mean %.3f ms max %.3f ms, worst error %.0f mm\n",
                        name, to_string(lod), keys.size(), lc.level, lines, points_in, points, collapsed, covered.size(),
                        expected, static_cast<double>(bytes) / 1024.0, static_cast<double>(max_bytes) / 1024.0,
                        total_ms / static_cast<double>(keys.size()), max_ms, worst_err);
        }
    }
}

TEST_CASE("Map LOD settings from the game: keys, selection, validation", "[route][map][lod_settings]") {
    const MapConfig& d = default_map_config();
    REQUIRE(validate_map_config(d));
    // Keys: deterministic, per LOD, sensitive to every content parameter only.
    const auto keys = [](const MapConfig& c) {
        return std::vector<std::uint64_t>{map_lod_key(c, MapLod::Detail), map_lod_key(c, MapLod::Mid), map_lod_key(c, MapLod::Far)};
    };
    const auto base = keys(d);
    CHECK(base == keys(MapConfig(d)));
    CHECK(base[0] != base[1]);
    CHECK(base[1] != base[2]);
    const auto changed = [&](auto edit) {
        MapConfig c = d;
        edit(c);
        const auto k = keys(c);
        std::vector<bool> diff;
        for (std::size_t i = 0; i < k.size(); ++i) diff.push_back(k[i] != base[i]);
        return diff;
    };
    using V = std::vector<bool>;
    CHECK(changed([](MapConfig& c) { c.lods[1].tolerance_mm = 2000; }) == V{false, true, false});
    CHECK(changed([](MapConfig& c) { c.lods[2].min_rank = 5; }) == V{false, false, true});
    CHECK(changed([](MapConfig& c) { c.lods[2].level = 7; }) == V{false, false, true});
    CHECK(changed([](MapConfig& c) { c.detail_scale = 2.0; }) == V{true, true, true});
    CHECK(changed([](MapConfig& c) { c.directions.implied_oneway.push_back("trunk"); }) == V{true, true, true});
    CHECK(changed([](MapConfig& c) { c.lods[0].switch_distance_m = 3000; }) == V{false, false, false});
    CHECK(changed([](MapConfig& c) { c.max_lines = 10; }) == V{false, false, false});
    // Scale 2 with doubled tolerances builds the same tiles as the defaults.
    MapConfig doubled = d;
    doubled.detail_scale = 2.0;
    for (auto& l : doubled.lods) l.tolerance_mm *= 2;
    CHECK(keys(doubled) == base);
    CHECK(effective_lod(doubled, MapLod::Mid).tolerance_mm == 8000);
    CHECK(effective_lod(doubled, MapLod::Detail).switch_distance_m == 4000.0);

    // LOD selection by view radius, scaled by the detail setting.
    CHECK(select_map_lod(d, 500.0) == MapLod::Detail);
    CHECK(select_map_lod(d, 2000.0) == MapLod::Detail);
    CHECK(select_map_lod(d, 5000.0) == MapLod::Mid);
    CHECK(select_map_lod(d, 80000.0) == MapLod::Far);
    MapConfig coarse = d;
    coarse.detail_scale = 0.5;
    CHECK(select_map_lod(coarse, 1500.0) == MapLod::Mid);
    CHECK(select_map_lod(doubled, 3000.0) == MapLod::Detail);

    // Validation.
    const auto invalid = [&](auto edit) {
        MapConfig c = d;
        edit(c);
        const auto r = validate_map_config(c);
        return !r && r.error().code == ErrorCode::InvalidArgument;
    };
    CHECK(invalid([](MapConfig& c) { c.detail_scale = 0.0; }));
    CHECK(invalid([](MapConfig& c) { c.detail_scale = std::numeric_limits<double>::quiet_NaN(); }));
    CHECK(invalid([](MapConfig& c) { c.lods[0].level = 1; }));
    CHECK(invalid([](MapConfig& c) { c.lods[1].level = 7; })); // above far's 6
    CHECK(invalid([](MapConfig& c) { c.lods[1].tolerance_mm = -1; }));
    CHECK(invalid([](MapConfig& c) { c.lods[2].switch_distance_m = 100.0; }));
    CHECK(invalid([](MapConfig& c) { c.lods[0].min_rank = 200; }));
    const std::vector<const RoadGraphTile*> none;
    MapConfig bad = d;
    bad.detail_scale = -1.0;
    CHECK(build_map_tile(TileKey{g2m_test::kZone32N, 2, 0, 0}, MapLod::Detail, none, bad).error().code == ErrorCode::InvalidArgument);
}

TEST_CASE("Map LOD settings change the tiles; the cache rebuilds only the affected LOD", "[route][map][lod_settings][real-data]") {
    const auto data = g2m_test::read_route_fixture("hoechst_a66");
    const auto world = build_road_graph(data, g2m_test::kZone32N);
    REQUIRE(world);
    const auto tiles = tile_world(*world);
    const auto all = ptrs(tiles);
    std::set<TileKey> mids, fars;
    for (const auto& t : tiles) { mids.insert(up(t.key, 4)); fars.insert(up(t.key, 6)); }
    // The mid tile with the most points (the fixture covers some mid tiles only at their edge).
    TileKey mid_key = *mids.begin();
    std::size_t most = 0;
    for (const auto& k : mids) {
        const auto t = build_map_tile(k, MapLod::Mid, all);
        REQUIRE(t);
        if (t->points.size() > most) { most = t->points.size(); mid_key = k; }
    }
    const TileKey detail_key = tiles.front().key, far_key = *fars.begin();

    const MapConfig& d = default_map_config();
    const auto mid = build_map_tile(mid_key, MapLod::Mid, all);
    REQUIRE(mid);
    CHECK(mid->params_key == map_lod_key(d, MapLod::Mid));
    // An explicit copy of the defaults gives the identical tile.
    const auto same = build_map_tile(mid_key, MapLod::Mid, all, MapConfig(d));
    REQUIRE(same);
    CHECK(*same == *mid);

    // Finer tolerance: more points; more detail_scale: the same effect.
    MapConfig fine = d;
    fine.lods[1].tolerance_mm = 1000;
    const auto mid_fine = build_map_tile(mid_key, MapLod::Mid, all, fine);
    REQUIRE(mid_fine);
    CHECK(mid_fine->points.size() > mid->points.size());
    CHECK(mid_fine->params_key != mid->params_key);
    MapConfig scaled = d;
    scaled.detail_scale = 8.0;
    const auto mid_scaled = build_map_tile(mid_key, MapLod::Mid, all, scaled);
    REQUIRE(mid_scaled);
    CHECK(mid_scaled->points == mid_fine->points);
    CHECK(mid_scaled->lines == mid_fine->lines);
    // Lower class cut-off on the far map: more lines.
    MapConfig busy = d;
    busy.lods[2].min_rank = 0;
    const auto far_default = build_map_tile(far_key, MapLod::Far, all);
    const auto far_busy = build_map_tile(far_key, MapLod::Far, all, busy);
    REQUIRE(far_default);
    REQUIRE(far_busy);
    CHECK(far_busy->lines.size() > far_default->lines.size());
    // Another level: the mid LOD at level 3 is built for level-3 keys only.
    MapConfig l3 = d;
    l3.lods[1].level = 3;
    CHECK(build_map_tile(mid_key, MapLod::Mid, all, l3).error().code == ErrorCode::InvalidArgument);
    const auto mid_l3 = build_map_tile(up(detail_key, 3), MapLod::Mid, all, l3);
    REQUIRE(mid_l3);
    CHECK(mid_l3->key.level == 3);

    // The cache follows the settings.
    MapTileCache cache;
    for (const auto& t : tiles) REQUIRE(cache.get(t.key, MapLod::Detail, all));
    for (const auto& k : mids) REQUIRE(cache.get(k, MapLod::Mid, all));
    for (const auto& k : fars) REQUIRE(cache.get(k, MapLod::Far, all));
    const std::size_t total = cache.size(), builds = cache.builds();
    CHECK(total == tiles.size() + mids.size() + fars.size());
    REQUIRE(cache.get(mid_key, MapLod::Mid, all));
    CHECK(cache.builds() == builds); // a hit
    CHECK(**cache.get(mid_key, MapLod::Mid, all) == *mid);

    const auto unchanged = cache.set_config(MapConfig(d));
    REQUIRE(unchanged);
    CHECK(*unchanged == 0);
    MapConfig switch_only = d;
    switch_only.lods[0].switch_distance_m = 1000; // selection only: no tile changes
    REQUIRE(cache.set_config(switch_only));
    CHECK(cache.size() == total);

    const auto dropped = cache.set_config(fine);
    REQUIRE(dropped);
    CHECK(*dropped == mids.size());
    CHECK(cache.find(mid_key, MapLod::Mid) == nullptr);
    CHECK(cache.find(detail_key, MapLod::Detail) != nullptr);
    CHECK(cache.find(far_key, MapLod::Far) != nullptr);
    const auto rebuilt = cache.get(mid_key, MapLod::Mid, all);
    REQUIRE(rebuilt);
    CHECK(**rebuilt == *mid_fine);

    MapConfig invalid = fine;
    invalid.detail_scale = 100.0;
    CHECK_FALSE(cache.set_config(invalid));
    CHECK(cache.config().detail_scale == 1.0);
    CHECK(cache.find(mid_key, MapLod::Mid) != nullptr);

    // Scale 8: detail and far tolerances change; mid's effective 1000 mm equals `fine`'s, so it stays.
    const auto rescaled = cache.set_config(scaled);
    REQUIRE(rescaled);
    CHECK(*rescaled == tiles.size() + fars.size());
    CHECK(cache.size() == 1);
    CHECK(cache.find(mid_key, MapLod::Mid) != nullptr);
}
