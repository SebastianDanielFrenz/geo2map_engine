// tools/ribbon_census/trimsteps.cpp - g2m_ribbon_census --trimsteps E0 N0 E1 N1 (ROADS stage 3c-3, diagnostics): the carve
// model's height step on the trim line between a junction core and the core of a used approach whose stretch lands (a
// Landing join) at its OTHER end - the design-T12 "trim" quantity of tests/layer/test_terrain_carve.cpp, at the far ends of
// landed ramp stretches whose junction node lies in the box. For each such junction the level-0 tile holding the node is
// carved as TerrainCarvedDeriver does (the roads.geom tiles of carve_road_geom_keys), on a FLAT terrain: the two cores' heights
// come from the served profiles and junction surfaces, not from the terrain, so a flat window keeps the run cheap. Every pair
// of 4-adjacent cell centres whose owners are that junction core and that approach core is bisected to its switch point; a
// switch within 5 cm of the trim line counts. Prints
//
//   TS junctionNode way at_end switches maxStepMm E N    one line per (junction, landed approach) seen in a carved tile
//
// (switches = owner switches on the trim line, maxStepMm = the largest |dz| over them in mm, E N = where), then a summary
// line TSUM. ROADS stage 5a: a switch counts only when the approach side at the switch point is the watched approach; a
// switch to another road's core at that station (a neighbour reaching the ring in a sliver past the trim line) is printed
// as NB (same fields) and summed in NBSUM. With the environment variable G2M_TRIMDUMP="node way" (ROADS stage 5a) every owner switch on that approach's trim
// line is printed as TD s t zJunction zApproach dz base paved full (the junction surface's approach heights at the point), and
// the approach's cross-section at the trim station and 1 m either side as TX s offset carriageway_l/r paved_l/r outer_l/r and
// one band per item kind:counted:width_mm (t positive left of the reference direction, metres).
#include "g2m/core/tile_key.h"
#include "g2m/layer/height_tile.h"
#include "g2m/layer/road_cross_section.h"
#include "g2m/layer/road_geom_tile.h"
#include "g2m/layer/road_junction.h"
#include "g2m/layer/road_rules.h"
#include "g2m/layer/road_surface.h"
#include "g2m/layer/carve_params.h"
#include "g2m/layer/terrain_carve.h"
#include "g2m/world/terrain_service.h"
#include "g2m/world/ticket.h"

#include <array>
#include <cstdlib>
#include <sstream>
#include <string>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <iostream>
#include <map>
#include <memory>
#include <set>
#include <tuple>
#include <vector>

using namespace g2m;
using namespace g2m::world;

namespace {
// getenv is deprecated under the MSVC CRT (C4996, warnings are errors); same pattern as test_junction_dump.cpp's env().
std::string g2m_getenv(const char* k) {
    std::string v;
#ifdef _MSC_VER
    char* p = nullptr; std::size_t n = 0;
    if (_dupenv_s(&p, &n, k) == 0 && p) { v = p; std::free(p); }
#else
    if (const char* p = std::getenv(k)) v = p;
#endif
    return v;
}
struct Step { std::size_t switches = 0; double max = 0, x = 0, y = 0; };
using Key = std::tuple<std::int64_t, std::int64_t, int>;   // junction node, way, approach at_end
} // namespace

int run_trimsteps(TerrainService& svc, const geo::UtmZone& zone, const double box[4]) {
    const auto k0 = tile_key_at(zone, 2, box[0], box[1]), k1 = tile_key_at(zone, 2, box[2], box[3]);
    if (!k0 || !k1) return 1;
    const auto in_box = [&](double e, double n) { return e >= box[0] && e <= box[2] && n >= box[1] && n <= box[3]; };
    std::map<TileKey, std::shared_ptr<const RoadGeomTile>> geoms;
    const auto geom = [&](const TileKey& k) -> std::shared_ptr<const RoadGeomTile> {
        auto it = geoms.find(k);
        if (it != geoms.end()) return it->second;
        auto ticket = svc.road_geom(k, Priority::Render);
        ticket.wait_for(std::chrono::minutes(30));
        auto g = ticket.value();
        if (!g && ticket.state() == TileState::Failed) std::cerr << "tile " << k.x << ' ' << k.y << " failed: " << ticket.message() << '\n';
        return geoms.emplace(k, g).first->second;
    };
    // the level-0 tiles holding a far-end junction node of a landed stretch (owner tiles only)
    std::set<TileKey> targets;
    for (std::int32_t ty = k0->y; ty <= k1->y; ++ty)
        for (std::int32_t tx = k0->x; tx <= k1->x; ++tx) {
            TileKey key = *k0; key.x = tx; key.y = ty;
            const auto g = geom(key);
            if (!g) continue;
            for (const auto& j : g->junctions) {
                if (j.decline != JunctionDecline::None) continue;
                bool landed = false;
                for (const auto& a : j.approaches) {
                    if (a.status != JunctionApproachStatus::Used || a.entry_index >= g->entries.size()) continue;
                    const auto& en = g->entries[a.entry_index];
                    if (en.cross_section && (a.at_end ? en.cross_section->start_join : en.cross_section->end_join).kind == JoinKind::Landing) landed = true;
                }
                if (!landed) continue;
                for (const auto id : j.nodes) {
                    const auto& G = g->source.graph;
                    const auto it = std::lower_bound(G.nodes.begin(), G.nodes.end(), id, [](const RoadGraphNode& n, std::int64_t v) { return n.osm_id < v; });
                    if (it == G.nodes.end() || it->osm_id != id) continue;
                    const double e = static_cast<double>(it->position.x) / 1000.0, n = static_cast<double>(it->position.y) / 1000.0;
                    if (!in_box(e, n)) continue;
                    if (const auto kk = tile_key_at(zone, 0, e, n); kk && kk->parent().parent() == key) targets.insert(*kk);
                }
            }
        }
    std::cerr << targets.size() << " level-0 tiles\n";
    auto flat = std::make_unique<std::array<HeightTile, 9>>();
    std::map<Key, Step> steps, neighbour;
    std::int64_t dump_node = 0, dump_way = 0;
    if (const std::string env = g2m_getenv("G2M_TRIMDUMP"); !env.empty()) { std::istringstream is(env); is >> dump_node >> dump_way; }
    for (const TileKey& key0 : targets) {
        std::array<const HeightTile*, 9> window{};
        for (int dy = -1; dy <= 1; ++dy)
            for (int dx = -1; dx <= 1; ++dx) {
                auto& t = (*flat)[static_cast<std::size_t>((dy + 1) * 3 + (dx + 1))];
                t.key = TileKey{key0.zone, 0, key0.x + dx, key0.y + dy};
                window[static_cast<std::size_t>((dy + 1) * 3 + (dx + 1))] = &t;
            }
        auto terrain = CarveTerrain::make(key0, window, builtin_carve_default().smoothing);
        if (!terrain) { std::cerr << "terrain: " << terrain.error().message << '\n'; continue; }
        std::vector<std::shared_ptr<const RoadGeomTile>> keep;
        std::vector<CarveGeomInput> roads;
        for (const auto& k : carve_road_geom_keys(key0)) {
            const auto g = geom(k);
            if (!g) continue;
            keep.push_back(g);
            roads.push_back({k, g.get()});
        }
        auto model = CarveModel::build(roads, builtin_carve_default(), *terrain, {}, builtin_road_rules_de());
        if (!model) { std::cerr << "model: " << model.error().message << '\n'; continue; }
        const RoadGeomTile& core = model->core_tile();
        // (junction, entry) -> the approach (landed at its other end) and the trim station on the entry
        std::map<std::pair<std::size_t, std::size_t>, std::tuple<Key, double>> watch;
        for (std::size_t jx = 0; jx < model->junction_count(); ++jx) {
            const RoadJunction& j = model->junction(jx);
            for (const auto& a : model->junction_surface(jx).approaches) {
                if (a.entry_index >= core.entries.size()) continue;
                const auto& en = core.entries[a.entry_index];
                if (!en.cross_section || (a.at_end ? en.cross_section->start_join : en.cross_section->end_join).kind != JoinKind::Landing) continue;
                watch[{jx, a.entry_index}] = {Key{j.nodes.front(), en.way_id, a.at_end ? 1 : 0}, a.at_end ? a.length_m - a.trim_m : a.trim_m};
                if (j.nodes.front() == dump_node && en.way_id == dump_way) {
                    const double st = a.at_end ? a.length_m - a.trim_m : a.trim_m;
                    const auto& xs = *en.cross_section;
                    for (const double s : {st - 1.0, st, st + 1.0}) {
                        const CarveFootprint f = carve_footprint(xs, s);
                        std::printf("TX %.3f at_end %d trim %.3f len %.3f offset %.4f carriageway %.4f %.4f paved %.4f %.4f outer %.4f %.4f bands",
                                    s, a.at_end ? 1 : 0, a.trim_m, a.length_m, xs.offset_m(s), f.carriageway_left, f.carriageway_right,
                                    f.paved_left, f.paved_right, f.outer_left, f.outer_right);
                        for (std::size_t b = 0; b < xs.bands.size(); ++b)
                            std::printf(" %d:%d:%.0f", static_cast<int>(xs.bands[b].kind), xs.bands[b].counted ? 1 : 0, xs.band_width_m(b, s) * 1000.0);
                        std::printf("\n");
                    }
                }
            }
        }
        if (watch.empty()) continue;
        const double e0 = static_cast<double>(key0.min_easting()), n0 = static_cast<double>(key0.min_northing());
        for (int jy = 0; jy < 256; ++jy)
            for (int ix = 0; ix < 256; ++ix)
                for (int dir = 0; dir < 2; ++dir) {
                    const int i2 = ix + (dir == 0 ? 1 : 0), j2 = jy + (dir == 1 ? 1 : 0);
                    if (i2 > 255 || j2 > 255) continue;
                    const double ax = e0 + ix + 0.5, ay = n0 + jy + 0.5, bx = e0 + i2 + 0.5, by = n0 + j2 + 0.5;
                    const CarveSample a = model->evaluate(ax, ay), b = model->evaluate(bx, by);
                    const bool ja = a.owner == CarveOwner::JunctionCore && b.owner == CarveOwner::ApproachCore;
                    const bool jb = b.owner == CarveOwner::JunctionCore && a.owner == CarveOwner::ApproachCore;
                    if (!ja && !jb) continue;
                    const std::size_t jx = ja ? a.element : b.element, ex = ja ? b.element : a.element;
                    const auto w = watch.find({jx, ex});
                    if (w == watch.end()) continue;
                    double lo = 0, hi = 1;
                    for (int it = 0; it < 34; ++it) {
                        const double mid = 0.5 * (lo + hi);
                        const CarveSample s = model->evaluate(ax + mid * (bx - ax), ay + mid * (by - ay));
                        if (s.owner == a.owner && s.element == a.element) lo = mid; else hi = mid;
                    }
                    const double px = ax + lo * (bx - ax), py = ay + lo * (by - ay);
                    const CarveSample sa = model->evaluate(px, py), sb = model->evaluate(ax + hi * (bx - ax), ay + hi * (by - ay));
                    if (sa.deck_clamped || sb.deck_clamped || sa.owner != a.owner || sb.owner != b.owner) continue;
                    // ROADS stage 5a: the approach side of the switch must be the watched approach itself; a neighbouring road's
                    // core reaching the junction ring in a sliver past the trim line is counted apart (NB), not as this trim's step
                    const CarveSample& side = ja ? sb : sa;
                    if (side.element != ex) { Step& nb = neighbour[std::get<0>(w->second)]; ++nb.switches; const double jn = std::fabs(sa.z - sb.z); if (jn > nb.max) { nb.max = jn; nb.x = ax + lo * (bx - ax); nb.y = ay + lo * (by - ay); } continue; }
                    double s = 0, t = 0;
                    if (!model->station_of(ex, px, py, s, t) || std::fabs(s - std::get<1>(w->second)) > 0.05) continue;
                    Step& st = steps[std::get<0>(w->second)];
                    if (std::get<0>(std::get<0>(w->second)) == dump_node && std::get<1>(std::get<0>(w->second)) == dump_way) {
                        const auto& surf = model->junction_surface(jx);
                        std::size_t ai = 0;
                        for (; ai < surf.approaches.size(); ++ai) if (surf.approaches[ai].entry_index == ex) break;
                        JunctionApproachHeights h{};
                        if (ai < surf.approaches.size()) h = junction_approach_heights(surf, core, ai, px, py);
                        const double zj = sa.owner == CarveOwner::JunctionCore ? sa.z : sb.z, za = sa.owner == CarveOwner::JunctionCore ? sb.z : sa.z;
                        std::printf("TD %.3f %.3f %.4f %.4f %.1f %.4f %.4f %.4f\n", s, t, zj, za, (zj - za) * 1000.0, h.base, h.paved, h.full); {
                            const auto& ref = core.entries[ex].profile->reference;
                            const auto lp = lateral_point(ref, s, t);
                            const RoadSurfaceParams sp = builtin_carve_default().surface_params();
                            std::printf("TDX core(s,t)->xy %.3f %.3f vs point %.3f %.3f | zsurf(s,t) %.4f | zsurf(s,0) %.4f\n", lp ? lp->x : 0.0, lp ? lp->y : 0.0,
                                        px, py, road_surface_height_m(core, ex, s, t, sp), road_surface_height_m(core, ex, s, 0.0, sp));
                            const double qx = ax + hi * (bx - ax), qy = ay + hi * (by - ay);
                            for (std::size_t k = 0; k < core.entries.size(); ++k) {
                                double s2 = 0, t2 = 0;
                                if (!core.entries[k].cross_section || !model->station_of(k, qx, qy, s2, t2)) continue;
                                const CarveFootprint f2 = carve_footprint(*core.entries[k].cross_section, s2);
                                if (t2 < f2.outer_right || t2 > f2.outer_left) continue;
                                std::printf("TDC entry %zu way %lld s %.3f t %.3f z %.4f paved %d%s%s", k, static_cast<long long>(core.entries[k].way_id), s2, t2,
                                            road_surface_height_m(core, k, s2, t2, sp), (t2 >= f2.paved_right && t2 <= f2.paved_left) ? 1 : 0, k == ex ? " <-approach" : "", "\n");
                            }
                            std::printf("TDO sb owner %d element %zu ex %zu z %.4f" "\n", static_cast<int>(sb.owner), sb.element, ex, sb.z);
                        }
                    }
                    ++st.switches;
                    const double jump = std::fabs(sa.z - sb.z);
                    if (jump > st.max) { st.max = jump; st.x = px; st.y = py; }
                }
    }
    std::size_t over1 = 0, over5 = 0;
    double worst = 0;
    for (const auto& [k, st] : steps) {
        std::printf("TS %lld %lld %d %zu %.1f %.2f %.2f\n", static_cast<long long>(std::get<0>(k)), static_cast<long long>(std::get<1>(k)), std::get<2>(k),
                    st.switches, st.max * 1000.0, st.x, st.y);
        if (st.max > 0.01) ++over1;
        if (st.max > 0.05) ++over5;
        worst = std::max(worst, st.max);
    }
    std::printf("TSUM approaches %zu over_10mm %zu over_50mm %zu worst_mm %.1f\n", steps.size(), over1, over5, worst * 1000.0);
    double nworst = 0;
    for (const auto& [k, st] : neighbour) {
        std::printf("NB %lld %lld %d %zu %.1f %.2f %.2f\n", static_cast<long long>(std::get<0>(k)), static_cast<long long>(std::get<1>(k)), std::get<2>(k),
                    st.switches, st.max * 1000.0, st.x, st.y);
        nworst = std::max(nworst, st.max);
    }
    std::printf("NBSUM approaches %zu worst_mm %.1f\n", neighbour.size(), nworst * 1000.0);
    return 0;
}
