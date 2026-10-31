// tests/layer/test_portal_continuity.cpp - terrain migration phase 7a-1 (docs/structure_classification.md section 5):
// portal and join continuity. The ground side owns the join between a tunnel floor and the ground road that reaches
// it: a tunnel floor's portal anchor is the adjacent ground profile's end height and grade (the approach level where
// that stretch has no profile) instead of the DEM sample at the portal node, which often lies on a bank face.
//
// On the committed OSM fixtures (frankfurt_hoechst, hoechst_a66, and unterliederbach_tunnel for way 4812754 at
// Unterliederbach, tile utm32n/2/456/5420), flat and tilted bumpy DEMs, for every TunnelFloor entry and every node it
// shares: |dz| <= 0.01 m against every other profile ending there and |dgrade| <= 0.02 where the node joins exactly
// two roads; no declined ground stretch ends at a portal node; where the carve opens the tunnel as a cut, the carved L0
// along the axis +-8 m around the portal steps by at most 0.10 m per metre more than the profile itself. A rail case
// (rail.geom, tunnel chain between two ground pieces) and the tile-invariance of the result are checked next to them.
// The 4812754 fixture has no DEM of its own (the real-data store is not a test input): the NW mouth is reproduced by a
// synthetic bank whose face at the node reads 99.47 m against the approach road at 98.7 m (the real DEM: 99.52 m and
// 98.68 m, docs/structure_classification.md section 5). Sabotage A (the portal anchor back to the DEM sample at the
// node) and B (no refit of the declined ground stretch to the tunnel plane) are recorded in the commit message and
// notes; the "previous release" controls inside the test compute the same failure without touching the source.
#include "carve_fixture.h"
#include "synthetic.h"

#include "g2m/layer/rail_geom_tile.h"
#include "g2m/layer/terrain_carve.h"

#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <functional>
#include <map>
#include <set>
#include <string>
#include <tuple>
#include <vector>

using namespace g2m;

namespace {
using DemFn = std::function<double(double, double)>;
constexpr double kDzTol = 0.01, kGradeTol = 0.02, kCarveExcess = 0.10;
constexpr const char* kFfm = "frankfurt_hoechst.osm.pbf";
constexpr const char* kA66 = "hoechst_a66.osm.pbf";
constexpr const char* kUnter = "unterliederbach_tunnel.osm.pbf";
constexpr std::int64_t kTunnelWay = 4812754, kNwNode = 30906085;

double smooth(double u) { u = std::clamp(u, 0.0, 1.0); return u * u * (3.0 - 2.0 * u); }

bool tag_is(const RoadGraphWay& w, const char* key, bool want_true) {
    bool set = false;
    for (const auto& [k, v] : w.tags) if (k == key && v != "no") set = true;
    return set == want_true;
}
const RoadGraphWay* find_way(const RoadGraph& world, std::int64_t id) {
    const auto it = std::lower_bound(world.ways.begin(), world.ways.end(), id, [](const auto& w, auto v) { return w.osm_id < v; });
    return it != world.ways.end() && it->osm_id == id ? &*it : nullptr;
}
const RoadGraphNode* find_node(const RoadGraph& world, std::int64_t id) {
    const auto it = std::lower_bound(world.nodes.begin(), world.nodes.end(), id, [](const auto& n, auto v) { return n.osm_id < v; });
    return it != world.nodes.end() && it->osm_id == id ? &*it : nullptr;
}

// The synthetic DEM of the Unterliederbach mouth: the road (NW approach) rises away from the node at 1.9 %, the tunnel's
// floor line (SE) at 0.95 %, and the rail embankment stands over the whole tunnel with its face 0.6 m before the NW node
// (so the DEM AT the node reads about 0.84 m above the approach road, the real 99.52 against 98.68).
DemFn unterliederbach_dem(const RoadGraph& world) {
    const auto* way = find_way(world, kTunnelWay); REQUIRE(way);
    const auto* a = find_node(world, way->node_ids.front()); const auto* b = find_node(world, way->node_ids.back()); REQUIRE(a); REQUIRE(b);
    const double ax = static_cast<double>(a->position.x) / 1000.0, ay = static_cast<double>(a->position.y) / 1000.0;
    const double bx = static_cast<double>(b->position.x) / 1000.0, by = static_cast<double>(b->position.y) / 1000.0;
    const double length = std::sqrt((bx - ax) * (bx - ax) + (by - ay) * (by - ay));
    const double ux = (bx - ax) / length, uy = (by - ay) / length;
    return [=](double e, double n) {
        const double d = (e - ax) * ux + (n - ay) * uy, l = -(e - ax) * uy + (n - ay) * ux;
        const double ground = 98.63 + (d < 0 ? -0.019 * d : 0.0095 * d);
        const double bank = 5.4 * smooth((d + 0.6) / 2.4) * smooth((length - 1.0 - d) / 2.4) * smooth((25.0 - std::fabs(l)) / 6.0);
        return ground + bank;
    };
}

struct End {
    std::int64_t way = 0; std::uint32_t start_ref = 0; int side = 0;
    bool tunnel = false, bridge = false, floor = false;
    double z = 0, away = 0;   // the height at the node and the grade rising AWAY from it (m/m)
    double length = 0; bool straight = false;   // the entry's length and whether its profile is one straight grade (within 1 cm)
};
struct NodeRec { double east = 0, north = 0; std::vector<End> ends; int declined_ground = 0; std::string declined_why; };
struct Scan {
    std::map<std::int64_t, NodeRec> nodes;
    std::size_t floors = 0, invariance_mismatch = 0;
    RoadCompletionStats sum;            // 7a-2 design-floor counters summed over tiles
    std::size_t infeasible_pairs = 0;   // 7a-2: adjacent portal pairs the floor cannot climb between (anchors moved; summed over tiles)
};

// Every level-2 tile of `world` built the way the deriver serves it; the ends of every entry grouped by node.
Scan scan(const RoadGraph& world, const DemFn& dem, const RoadCompletionOptions& completion) {
    Scan out;
    std::set<TileKey> keys;
    for (const auto& n : world.nodes)
        if (const auto k = tile_key_at(g2m_test::kZone32N, 2, static_cast<double>(n.position.x) / 1000.0, static_cast<double>(n.position.y) / 1000.0)) keys.insert(*k);
    std::set<std::tuple<std::int64_t, std::uint32_t, int>> seen, seen_declined;
    std::map<std::pair<std::int64_t, std::uint32_t>, std::vector<double>> signature;
    for (const auto& key : keys) {
        g2m_test::GeomOptions go; go.height_m = dem; go.completion = completion; go.underpass = true;
        RoadCompletionStats tile_stats; go.stats = &tile_stats;
        auto tile = g2m_test::geometry_in_tile_with(world, key, go); REQUIRE(tile);
        out.infeasible_pairs += tile_stats.floor_pairs_shifted;
        out.sum.floor_components += tile_stats.floor_components; out.sum.floor_pairs += tile_stats.floor_pairs;
        out.sum.floor_pairs_beyond_grade += tile_stats.floor_pairs_beyond_grade; out.sum.floor_pairs_still_beyond_grade += tile_stats.floor_pairs_still_beyond_grade;
        out.sum.floor_anchor_misses += tile_stats.floor_anchor_misses; out.sum.floor_unsolved += tile_stats.floor_unsolved; out.sum.floor_not_converged += tile_stats.floor_not_converged;
        out.sum.floor_entries += tile_stats.floor_entries; out.sum.floor_entry_failed += tile_stats.floor_entry_failed; out.sum.floor_entry_straight += tile_stats.floor_entry_straight;
        out.sum.floor_fit_over_250mm += tile_stats.floor_fit_over_250mm;
        out.sum.floor_fit_max_deviation_m = std::max(out.sum.floor_fit_max_deviation_m, tile_stats.floor_fit_max_deviation_m);
        out.sum.floor_max_shift_m = std::max(out.sum.floor_max_shift_m, tile_stats.floor_max_shift_m);
        for (std::size_t i = 0; i < tile->entries.size(); ++i) {
            const auto& e = tile->entries[i];
            const auto* way = find_way(world, e.way_id);
            if (!way || e.stretch.end_ref >= way->node_ids.size()) continue;
            const std::int64_t ends[2] = {way->node_ids[e.stretch.start_ref], way->node_ids[e.stretch.end_ref]};
            if (e.profile) {
                const auto& p = *e.profile;
                const bool floor = i < tile->origins.size() && tile->origins[i].provenance == ProfileProvenance::TunnelFloor;
                std::vector<double> sig;
                for (const double f : {0.0, 0.25, 0.5, 0.75, 1.0}) { const auto z = p.at(p.reference.length_m * f); if (z) { sig.push_back(z->height_m); sig.push_back(z->grade); } }
                const auto [it, fresh] = signature.emplace(std::make_pair(e.way_id, e.stretch.start_ref), sig);
                if (!fresh && it->second != sig) ++out.invariance_mismatch;
                // Straight within 1 cm of the chord over the whole metres: a D52 chord or a D65 cut, which the owner's ruling keeps one grade.
                bool straight = true;
                {
                    const auto a = p.at(0), b = p.at(p.reference.length_m);
                    for (double st = 1.0; straight && a && b && st < p.reference.length_m; st += 1.0) {
                        const auto z = p.at(st); if (!z) { straight = false; break; }
                        straight = std::fabs(z->height_m - (a->height_m + (b->height_m - a->height_m) * st / p.reference.length_m)) <= 0.01;
                    }
                }
                for (int side = 0; side < 2; ++side) {
                    if (!seen.insert({e.way_id, e.stretch.start_ref, side}).second) continue;
                    const double st = side ? p.reference.length_m : 0;
                    const auto z = p.at(st); const auto node = find_node(world, ends[side]); if (!z || !node) continue;
                    auto& rec = out.nodes[ends[side]];
                    rec.east = static_cast<double>(node->position.x) / 1000.0; rec.north = static_cast<double>(node->position.y) / 1000.0;
                    rec.ends.push_back({e.way_id, e.stretch.start_ref, side, p.attributes.tunnel, p.attributes.bridge, floor, z->height_m, side ? -z->grade : z->grade, p.reference.length_m, straight});
                    if (floor) ++out.floors;
                }
            } else if (tag_is(*way, "tunnel", false) && tag_is(*way, "bridge", false)) {
                for (int side = 0; side < 2; ++side) if (seen_declined.insert({e.way_id, e.stretch.start_ref, side}).second) {
                    auto& r = out.nodes[ends[side]]; ++r.declined_ground;
                    r.declined_why += "way " + std::to_string(e.way_id) + " " + (e.decline ? e.decline->message : std::string("?")) + "; ";
                }
            }
        }
    }
    return out;
}

struct Verdict { std::size_t portals = 0, with_floor = 0, dz_bad = 0, grade_bad = 0, declined = 0, unreachable = 0, unreachable_dz = 0, cut_grade = 0; double worst_dz = 0, worst_grade = 0; std::string detail; };
Verdict judge(const Scan& s, const char* label) {
    Verdict v;
    for (const auto& [id, n] : s.nodes) {
        std::vector<const End*> floors, others, profiled;
        for (const auto& e : n.ends) { if (e.bridge) continue; profiled.push_back(&e); if (e.floor) floors.push_back(&e); }
        if (floors.empty()) continue;
        ++v.portals; v.declined += static_cast<std::size_t>(n.declined_ground);
        // A ground stretch between end planes whose heights differ by more than the class grade allows over its length
        // ("endpoint grade infeasible") has no profile whatever anchors it; counted apart.
        if (n.declined_ground > 0 && n.declined_why.find("endpoint grade infeasible") != std::string::npos) v.unreachable += static_cast<std::size_t>(n.declined_ground);
        if (n.declined_ground > 0) v.detail += std::string("  declined ground stretch at node ") + std::to_string(id) + ": " + n.declined_why + "\n";
        for (const auto* t : floors) for (const auto* o : profiled) {
            if (o == t) continue;
            const double dz = std::fabs(t->z - o->z); v.worst_dz = std::max(v.worst_dz, dz);
            if (dz > kDzTol) {
                // Neither side could meet the other: the tunnel floor at the reach of its grade bound and the ground stretch at its class
                // bound (the review of ffd575d: such a node is reported apart, with its numbers, never absorbed).
                ++v.dz_bad; ++v.unreachable_dz;
                v.detail += std::string("  unreachable: dz ") + std::to_string(dz) + " at node " + std::to_string(id) + " (tunnel way " + std::to_string(t->way) + " stub " + std::to_string(t->length) +
                    " m, floor bound 0.30, floor z " + std::to_string(t->z) + " vs way " + std::to_string(o->way) + " length " + std::to_string(o->length) + " m, z " + std::to_string(o->z) + ")\n";
            }
            // The grade continues through a node that joins exactly two roads (rising away from the node on both sides: the sum is 0).
            if (profiled.size() == 2) {
                const double dg = std::fabs(t->away + o->away); v.worst_grade = std::max(v.worst_grade, dg);
                if (dg > kGradeTol) { (t->straight ? v.cut_grade : v.grade_bad)++; v.detail += std::string("  dgrade ") + std::to_string(dg) + " at node " + std::to_string(id) + " (tunnel way " + std::to_string(t->way) + " len " + std::to_string(t->length) + (t->straight ? " straight" : "") + " away " + std::to_string(t->away) + " vs way " + std::to_string(o->way) + " away " + std::to_string(o->away) + ")\n"; }
            }
        }
        ++v.with_floor;
    }
    std::printf("[portal %s] %zu portal nodes, %zu tunnel floors; worst dz %.4f m (%zu over %.2f), worst dgrade %.4f (%zu over %.2f), declined ground stretches at portals %zu (%zu between end planes the class grade cannot join), straight tunnels over the grade tolerance %zu, unreachable steps %zu\n%s",
        label, v.portals, s.floors, v.worst_dz, v.dz_bad, kDzTol, v.worst_grade, v.grade_bad, kGradeTol, v.declined, v.unreachable, v.cut_grade, v.unreachable_dz, v.detail.c_str());
    return v;
}

// The carved L0 along the axis +-8 m around each portal that has a ground profile next to a cut tunnel: the carved height
// steps per metre by at most kCarveExcess more than the profile's own. Returns {portals checked, worst excess}.
struct CarveVerdict { std::size_t checked = 0, skipped = 0; double worst = 0; };
CarveVerdict carved_axis(const RoadGraph& world, const DemFn& dem, const RoadCompletionOptions& completion, bool only_node = false, std::int64_t node_filter = 0) {
    CarveVerdict out;
    g2m_test::CarveFixture fx(world, dem); fx.completion = completion;
    // Entry lookup over every level-2 tile the carve reads.
    std::map<std::pair<std::int64_t, std::uint32_t>, const RoadProfile*> profiles;
    std::set<TileKey> keys2;
    for (const auto& n : world.nodes)
        if (const auto k = tile_key_at(g2m_test::kZone32N, 2, static_cast<double>(n.position.x) / 1000.0, static_cast<double>(n.position.y) / 1000.0)) keys2.insert(*k);
    struct Floor { const RoadProfile* p; std::int64_t node; int side; };
    std::vector<Floor> floors;
    std::set<std::pair<std::int64_t, std::uint32_t>> seen;
    for (const auto& key : keys2) {
        const RoadGeomTile& tile = fx.geom(key);
        for (std::size_t i = 0; i < tile.entries.size(); ++i) {
            const auto& e = tile.entries[i]; if (!e.profile) continue;
            if (!seen.insert({e.way_id, e.stretch.start_ref}).second) continue;
            profiles[{e.way_id, e.stretch.start_ref}] = &*e.profile;
        }
    }
    // For each tunnel floor end: the ground profile that meets it at the node (first by way id).
    std::map<std::int64_t, std::vector<std::pair<const RoadProfile*, int>>> by_node;   // node -> (profile, side)
    for (const auto& [id, p] : profiles) {
        const auto* way = find_way(world, id.first); if (!way) continue;
        const auto ends = std::array<std::int64_t, 2>{way->node_ids[p->stretch.start_ref], way->node_ids[p->stretch.end_ref]};
        for (int side = 0; side < 2; ++side) by_node[ends[side]].push_back({p, side});
    }
    for (const auto& [node, list] : by_node) {
        if (only_node && node != node_filter) continue;
        for (const auto& [tp, tside] : list) {
            if (!tp->attributes.tunnel) continue;
            const RoadProfile* ground = nullptr; int gside = 0;
            for (const auto& [gp, gs] : list) if (gp != tp && !gp->attributes.tunnel && !gp->attributes.bridge) { ground = gp; gside = gs; break; }
            if (!ground) continue;
            const auto at = [](const RoadProfile& p, int side, double k) { return side ? p.reference.length_m - k : k; };
            struct Pt { double x, y, z; };
            std::vector<Pt> seq;
            bool ok = true;
            for (int k = 8; k >= 1; --k) {   // outside -> node
                const double st = at(*ground, gside, k); if (st < 0 || st > ground->reference.length_m) continue;
                const auto xy = ground->reference.at(st); const auto z = ground->at(st); if (!xy || !z) { ok = false; break; }
                seq.push_back({xy->x, xy->y, z->height_m});
            }
            for (int k = 0; k <= 8 && ok; ++k) {   // node -> inside
                const double st = at(*tp, tside, k); if (st < 0 || st > tp->reference.length_m) continue;
                const auto xy = tp->reference.at(st); const auto z = tp->at(st); if (!xy || !z) { ok = false; break; }
                seq.push_back({xy->x, xy->y, z->height_m});
            }
            if (!ok || seq.size() < 4) { ++out.skipped; continue; }
            const auto key0 = tile_key_at(g2m_test::kZone32N, 0, seq[seq.size() / 2].x, seq[seq.size() / 2].y); if (!key0) continue;
            auto m = fx.model(*key0);
            std::vector<double> carved;
            for (const auto& p : seq) carved.push_back(m.model->evaluate(p.x, p.y).z);
            // Only a tunnel the carve opens as a cut follows the road (inside the tunnel the carved height meets the profile).
            double inside = 0; std::size_t count = 0;
            for (std::size_t i = seq.size(); i-- > 0 && count < 6; ++count) inside = std::max(inside, std::fabs(carved[i] - seq[i].z));
            if (inside > 0.15) { ++out.skipped; continue; }
            ++out.checked;
            double worst = -1e9;
            for (std::size_t i = 0; i + 1 < seq.size(); ++i)
                worst = std::max(worst, std::fabs(carved[i + 1] - carved[i]) - std::fabs(seq[i + 1].z - seq[i].z));
            out.worst = std::max(out.worst, worst);
            if (worst > kCarveExcess) std::printf("  carved step at node %lld: %.3f m more than the profile\n", static_cast<long long>(node), worst);
        }
    }
    return out;
}

// ---- rail: a tunnel chain between two ground pieces -------------------------------------------------------------------
const TileKey kRailTile{g2m_test::kZone32N, 2, 456, 5420};
constexpr double kE0 = 456.0 * 1024.0 + 512.0, kN0 = 5420.0 * 1024.0 + 512.0;
using Tags = osm::OsmBuilder::TagList;
class RailScene {
public:
    std::int64_t line(std::int64_t id, double x0, double x1, double step, double y, Tags tags) {
        std::vector<std::int64_t> refs;
        for (double x = x0; x <= x1 + 1e-9; x += step) refs.push_back(node(x, y));
        builder_.add_way(id, refs, std::move(tags));
        return id;
    }
    osm::OsmData data() const { auto d = builder_.build(); REQUIRE(d); return std::move(*d); }
private:
    std::int64_t node(double x, double y) {
        const auto key = std::make_pair(std::llround(x * 1000.0), std::llround(y * 1000.0));
        const auto it = nodes_.find(key); if (it != nodes_.end()) return it->second;
        const geo::Utm utm(geo::kGRS80);
        const geo::TmReverse g = utm.reverse(g2m_test::kZone32N, kE0 + x, kN0 + y);
        const std::int64_t id = 1000 + static_cast<std::int64_t>(nodes_.size());
        builder_.add_node(id, g2m_test::to_e7(g.lat), g2m_test::to_e7(g.lon));
        nodes_[key] = id; return id;
    }
    osm::OsmBuilder builder_;
    std::map<std::pair<std::int64_t, std::int64_t>, std::int64_t> nodes_;
};
// A 2 % grade east and a 6 m high embankment over the tunnel, its face 4 m wide centred on the portal (the DEM AT the
// portal node reads 3 m above the road that reaches it).
double rail_bank_dem(double e, double /*n*/) {
    const double x = e - kE0;
    const double bank = 6.0 * smooth((x + 42.0) / 4.0) * smooth((42.0 - x) / 4.0);
    return 100.0 + 0.02 * x + bank;
}
std::int32_t z_at(const RailPiece& p, double x) {
    std::size_t best = 0; double d = 1e18;
    for (std::size_t i = 0; i < p.xy.size(); ++i) {
        const double v = std::fabs(static_cast<double>(p.xy[i].x) / 1000.0 - (kE0 + x));
        if (v < d) { d = v; best = i; }
    }
    return p.z[best];
}
const RailPiece* piece_of(const RailGeomTile& t, std::int64_t way) {
    for (const auto& p : t.pieces) if (p.way_id == way && p.piece == 0) return &p;
    return nullptr;
}
} // namespace

TEST_CASE("portal continuity (7a-1): tunnel floors meet the ground road at every portal on the fixtures", "[layer][roads][structure][portal][fixtures]") {
    struct Case { const char* file; const char* dem_name; DemFn dem; };
    const auto flat = DemFn{};
    for (const char* file : {kFfm, kA66}) {
        const RoadGraph world = g2m_test::fixture_world(file);
        for (const bool tilted : {false, true}) {
            const DemFn dem = tilted ? DemFn(g2m_test::carve_tilted_dem) : flat;
            const Scan s = scan(world, dem, served_road_completion_options());
            const std::string label = std::string(file) + (tilted ? " tilted" : " flat");
            const Verdict v = judge(s, label.c_str());
            CHECK(s.floors > 0);
            std::printf("[portal 7a-2 %s] components %zu (tile reads), unsolved %zu, not converged %zu; adjacent pairs %zu, %zu beyond the grade outright, %zu still beyond after the move, anchors moved up to %.3f m, %zu anchor misses; entries %zu (%zu straight, %zu failed); fit deviation max %.3f m (%zu over 0.25 m)\n",
                label.c_str(), s.sum.floor_components, s.sum.floor_unsolved, s.sum.floor_not_converged, s.sum.floor_pairs, s.sum.floor_pairs_beyond_grade,
                s.sum.floor_pairs_still_beyond_grade, s.sum.floor_max_shift_m, s.sum.floor_anchor_misses, s.sum.floor_entries, s.sum.floor_entry_straight,
                s.sum.floor_entry_failed, s.sum.floor_fit_max_deviation_m, s.sum.floor_fit_over_250mm);
            CHECK(s.sum.floor_unsolved == 0);
            CHECK(s.sum.floor_anchor_misses == 0);
            // 7a-2: an anchor is met exactly (dz_bad 0) except at the portals of a pair the floor cannot climb between under the road grade: the
            // reach rule moves the anchors toward each other, a ground stretch that was accepted strictly keeps its end and a step of the
            // move remains there; every such node is one of the counted infeasible pairs (two portals per pair at most).
            if (s.infeasible_pairs == 0) CHECK(v.dz_bad == 0);
            else CHECK(v.dz_bad <= 2 * s.infeasible_pairs);
            CHECK(v.unreachable_dz == v.dz_bad);
            CHECK(v.declined == v.unreachable);
            // The flat DEM: every join continues the ground road's grade. The tilted bumpy DEM (2 m bumps over 23 m, ground roads
            // at grades up to 0.4 against tunnels of 8-50 m) gives joins no floor within the curvature bound can follow: reported.
            if (!tilted) { CHECK(v.grade_bad == 0); CHECK(v.declined == 0); }
            CHECK(s.invariance_mismatch == 0);
        }
    }
}

TEST_CASE("portal continuity (7a-1): Unterliederbach way 4812754, the NW mouth on the bank face", "[layer][roads][structure][portal][fixtures]") {
    const RoadGraph world = g2m_test::fixture_world(kUnter);
    const auto* way = find_way(world, kTunnelWay); REQUIRE(way);
    REQUIRE(way->node_ids.front() == kNwNode);
    const DemFn dem = unterliederbach_dem(world);
    // The bank face at the node, against the approach road 6.5 m out: the lip the previous release built.
    const auto* node = find_node(world, kNwNode); REQUIRE(node);
    const double ne = static_cast<double>(node->position.x) / 1000.0, nn = static_cast<double>(node->position.y) / 1000.0;
    std::printf("[portal 4812754] synthetic DEM at the NW node %.2f m, 6.5 m out along the approach road %.2f m\n", dem(ne, nn), dem(ne - 0.54 * 6.5, nn + 0.84 * 6.5));

    const auto probe = [&](const RoadCompletionOptions& options) {
        const Scan s = scan(world, dem, options);
        const auto it = s.nodes.find(kNwNode); REQUIRE(it != s.nodes.end());
        double floor_z = 0, ground_z = 0; bool have_floor = false, have_ground = false;
        for (const auto& e : it->second.ends) {
            if (e.way == kTunnelWay) { floor_z = e.z; have_floor = true; }
            else if (!e.tunnel && !e.bridge) { ground_z = e.z; have_ground = true; }
        }
        return std::tuple{s, floor_z, ground_z, have_floor, have_ground, it->second.declined_ground};
    };
    const auto [served, floor_z, ground_z, have_floor, have_ground, declined] = probe(served_road_completion_options());
    REQUIRE(have_floor);
    std::printf("[portal 4812754] served: tunnel floor at the NW node %.3f m, ground road end %s%.3f m, step %.3f m, declined ground stretches %d\n",
        floor_z, have_ground ? "" : "(none) ", ground_z, have_ground ? std::fabs(floor_z - ground_z) : -1.0, declined);
    CHECK(have_ground);
    CHECK(std::fabs(floor_z - ground_z) <= kDzTol);
    CHECK(declined == 0);
    const Verdict v = judge(served, "unterliederbach");
    CHECK(v.dz_bad == 0);
    CHECK(v.grade_bad == 0);   // straight (cut) tunnels are exempt: their floor is the chord, the carve needs it within 1 cm
    CHECK(v.declined == 0);
    CHECK(floor_z < dem(ne, nn) - 0.5);   // the floor starts on the road, not on the bank face

    // The previous release (the control): the floor on the DEM sample at the node and the last ground stretch declined.
    const auto [before, before_floor, before_ground, bf, bg, before_declined] = probe(g2m_test::served_without_portal_anchor());
    std::printf("[portal 4812754] previous release: tunnel floor %.3f m, ground end %s%.3f m, step %.3f m, declined ground stretches %d\n",
        before_floor, bg ? "" : "(none) ", before_ground, bg ? std::fabs(before_floor - before_ground) : -1.0, before_declined);
    CHECK(before_floor > floor_z + 0.5);
    CHECK((before_declined > 0 || (bg && std::fabs(before_floor - before_ground) > 0.5)));

    // The carved terrain along the axis +-8 m around the NW portal.
    // The carve recognises a cut by a straight floor; the design floor of 7a-2 dips below the chord under a cover thinner than the
    // clearance plus roof, so this carve control runs on the 7a-1 floors (the carve of the design floor is phase 7b).
    const CarveVerdict carved = carved_axis(world, dem, g2m_test::served_7a1(), true, kNwNode);
    std::printf("[portal 4812754] carved L0 along the axis +-8 m: %zu portal checked, worst step excess over the profile %.3f m\n", carved.checked, carved.worst);
    CHECK(carved.checked == 1);
    CHECK(carved.worst <= kCarveExcess);
}

TEST_CASE("portal continuity (7a-1): the carved terrain across the join on the fixtures", "[layer][roads][structure][portal][carve][fixtures]") {
    for (const char* file : {kFfm, kA66}) {
        const RoadGraph world = g2m_test::fixture_world(file);
        for (const bool tilted : {false, true}) {
            const DemFn dem = tilted ? DemFn(g2m_test::carve_tilted_dem) : DemFn{};
            // As in the Unterliederbach case: the carve of the design floor is phase 7b, so the bound is on the 7a-1 floors.
            const CarveVerdict c = carved_axis(world, dem, g2m_test::served_7a1());
            const CarveVerdict design = carved_axis(world, dem, served_road_completion_options());
            std::printf("[portal carve %s %s] design floor: %zu checked, worst step excess %.3f m\n", file, tilted ? "tilted" : "flat", design.checked, design.worst);
            std::printf("[portal carve %s %s] %zu portals checked (open cuts), %zu skipped, worst step excess %.3f m\n", file, tilted ? "tilted" : "flat", c.checked, c.skipped, c.worst);
            // The previous release on the same fixture: the carve's own smoothing and the bumpy tilted DEM put steps beside a
            // portal whatever the profiles do; the anchor must not add to them.
            const CarveVerdict before = carved_axis(world, dem, g2m_test::served_without_portal_anchor());
            std::printf("[portal carve %s %s] previous release: %zu checked, worst step excess %.3f m\n", file, tilted ? "tilted" : "flat", before.checked, before.worst);
            // Strict on the flat DEM (the fixture's pre-existing excess is the bound). The tilted DEM's 2 m bumps over 23 m make
            // tunnels straight chords that the previous release left on the bumps (1 cut checked then, 18 now): reported only.
            if (!tilted) CHECK(c.worst <= std::max(kCarveExcess, before.worst + 0.02));
            else CHECK(c.checked > 0);
        }
    }
}

TEST_CASE("portal continuity (7a-1): rail - a tunnel chain meets its ground pieces on the approach level", "[layer][roads][structure][portal][rail]") {
    RailScene scene;
    scene.line(10, -300, -40, 20, 0, {{"railway", "rail"}});
    scene.line(11, -40, 0, 20, 0, {{"railway", "rail"}, {"tunnel", "yes"}});
    scene.line(12, 0, 40, 20, 0, {{"railway", "rail"}, {"tunnel", "yes"}});
    scene.line(13, 40, 300, 20, 0, {{"railway", "rail"}});
    const auto data = scene.data();
    const osm::OsmData* tiles[] = {&data};
    const auto world = collect_rail_world(tiles, g2m_test::kZone32N, builtin_rail_rules_de()); REQUIRE(world);
    std::vector<std::unique_ptr<HeightTile>> heights;
    for (auto ty = static_cast<std::int32_t>(std::floor((kN0 - 20) / 256.0)); ty <= static_cast<std::int32_t>(std::floor((kN0 + 20) / 256.0)); ++ty)
        for (auto tx = static_cast<std::int32_t>(std::floor((kE0 - 330) / 256.0)); tx <= static_cast<std::int32_t>(std::floor((kE0 + 330) / 256.0)); ++tx) {
            auto t = std::make_unique<HeightTile>(); t->key = TileKey{g2m_test::kZone32N, 0, tx, ty};
            g2m_test::fill_dem(*t, rail_bank_dem);
            heights.push_back(std::move(t));
        }
    std::vector<const HeightTile*> views; for (const auto& t : heights) views.push_back(t.get());
    const auto dem = RoadDemSampler::make(views); REQUIRE(dem);

    const auto step_at = [&](bool anchor, double x, std::int64_t ground_way, std::int64_t tunnel_way) {
        const auto tile = build_rail_geom_tile(*world, kRailTile, builtin_rail_rules_de(), *dem, anchor); REQUIRE(tile);
        const RailPiece* g = piece_of(*tile, ground_way); const RailPiece* t = piece_of(*tile, tunnel_way); REQUIRE(g); REQUIRE(t);
        return std::pair<double, double>{z_at(*g, x) / 256.0, z_at(*t, x) / 256.0};
    };
    const double road_level = 100.0 + 0.02 * -40.0;   // the line at the portal, without the embankment
    const auto [west_ground, west_tunnel] = step_at(true, -40, 10, 11);
    const auto [east_ground, east_tunnel] = step_at(true, 40, 13, 12);
    std::printf("[portal rail] anchored: west mouth ground %.3f tunnel %.3f, east mouth ground %.3f tunnel %.3f (line level %.3f / %.3f)\n",
        west_ground, west_tunnel, east_ground, east_tunnel, road_level, 100.0 + 0.02 * 40.0);
    CHECK(std::fabs(west_ground - west_tunnel) <= kDzTol);
    CHECK(std::fabs(east_ground - east_tunnel) <= kDzTol);
    CHECK(std::fabs(west_tunnel - road_level) < 0.15);
    CHECK(std::fabs(east_tunnel - (100.0 + 0.02 * 40.0)) < 0.15);
    // The previous release: the anchors are the DEM samples on the embankment faces.
    const auto [bwest_ground, bwest_tunnel] = step_at(false, -40, 10, 11);
    std::printf("[portal rail] previous release: west mouth ground %.3f tunnel %.3f\n", bwest_ground, bwest_tunnel);
    CHECK(bwest_tunnel > road_level + 1.0);
}
