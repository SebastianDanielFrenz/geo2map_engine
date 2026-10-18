// g2m/world/road_structures.cpp - see include/g2m/world/road_structures.h. Ported from racing_game
// core/src/road_structures.cpp (0c027dc, approach decks from 945455a) statement for statement; libm calls are det math (std::sqrt is IEEE-exact).
// tests/world/test_road_meshes.cpp pins the output byte for byte against meshes recorded from the game's own code.
#include "g2m/world/road_structures.h"

#include "g2m/core/det_math.h"
#include "g2m/layer/road_surface.h"

#include <algorithm>
#include <cmath>
#include <map>
#include <set>
#include <utility>

namespace g2m::world {

namespace {

const RoadGraphWay* find_way(const RoadGraph& g, std::int64_t id) {
    auto it = std::lower_bound(g.ways.begin(), g.ways.end(), id, [](const auto& w, auto value) { return w.osm_id < value; });
    return it == g.ways.end() || it->osm_id != id ? nullptr : &*it;
}
geom::PointMm find_node(const RoadGraph& g, std::int64_t id) {
    auto it = std::lower_bound(g.nodes.begin(), g.nodes.end(), id, [](const auto& n, auto value) { return n.osm_id < value; });
    return it->position;
}
int osm_layer(const RoadGraphWay& w) {
    for (const auto& [key, value] : w.tags)
        if (key == "layer") {
            if (value == "-1") return -1;
            if (value == "-2") return -2;
            if (value == "1") return 1;
            if (value == "2") return 2;
        }
    return 0;
}

std::shared_ptr<const RoadDeck> deck(const RoadProfile& profile, std::int64_t id, std::uint32_t start_ref, double from, double to,
                                     bool inferred, const std::optional<RoadCrossSection>& xs = {}) {
    if (to - from < .5 || to - from > 1000) return {};
    auto output = std::make_shared<RoadDeck>();
    output->way_id = id;
    output->start_ref = start_ref;
    output->start_station = from;
    output->end_station = to;
    output->vertical = profile.vertical.segments;
    output->half_width = profile.attributes.width_mm / 2000.0;
    output->crown = profile.attributes.crown_per_mille / 1000.0;
    output->tunnel_floor = profile.attributes.tunnel;
    output->inferred = inferred;
    output->land_class = profile.attributes.land_class;
    output->cross_section = xs;
    auto first = profile.reference.at(from);
    if (!first) return {};
    output->mesh.origin[0] = first->x;
    output->mesh.origin[1] = first->y;
    const int count = std::max(1, static_cast<int>(std::ceil(to - from)));
    const double half = profile.attributes.width_mm / 2000.0;
    auto& m = output->mesh;
    for (int i = 0; i <= count; ++i) {
        // The last station is exactly `to`: from+(to-from)*count/count can land one ulp past the profile end, which a
        // served bridge profile rejects as "station outside profile".
        const double station = i == count ? to : from + (to - from) * i / count;
        auto xy = profile.reference.at(station);
        auto z = profile.at(station);
        if (!xy || !z) return {};
        const double sh = det::sin(xy->heading), ch = det::cos(xy->heading);
        const double nx = -sh, ny = ch;
        // Five vertices across the carriageway give a rounded crown and identical geometry to collision, rather than
        // a visible-only slab.
        for (int j = 0; j < 5; ++j) {
            const auto f = xs ? carve_footprint(*xs, station) : CarveFootprint{};
            const double offset = xs ? f.roadbed_right + (f.roadbed_left - f.roadbed_right) * j / 4 : half * (j - 2) / 2;
            if(xs) output->half_width = std::max(output->half_width, std::max(std::abs(f.roadbed_left),std::abs(f.roadbed_right)));
            const double crossfall = profile.attributes.crown_per_mille / 1000.0;
            const double camber = xs ? road_crown_m(f.crown,crossfall,-f.crown_pivot,.5)-road_crown_m(f.crown,crossfall,offset-f.crown_pivot,.5)
                                    : crossfall * (det::sqrt(offset * offset + .25) - .5);
            m.positions.insert(m.positions.end(), {static_cast<float>(xy->x + nx * offset - m.origin[0]),
                                                   static_cast<float>(xy->y + ny * offset - m.origin[1]),
                                                   static_cast<float>(z->height_m - camber)});
            const double cross_grade = xs ? (f.crown==CrownKind::OneSidedRight?crossfall:f.crown==CrownKind::OneSidedLeft?-crossfall:
                -crossfall*std::clamp((offset-f.crown_pivot)/.5,-1.0,1.0)) : -crossfall * offset / det::sqrt(offset * offset + .25);
            double gx = z->grade * ch + cross_grade * nx, gy = z->grade * sh + cross_grade * ny;
            const double scale = 1 / det::sqrt(1 + gx * gx + gy * gy);
            m.normals.insert(m.normals.end(), {static_cast<float>(-gx * scale), static_cast<float>(-gy * scale), static_cast<float>(scale)});
            m.land_class.push_back(static_cast<std::uint8_t>(output->land_class));
        }
    }
    for (int i = 0; i < count; ++i)
        for (int j = 0; j < 4; ++j) {
            const auto a = static_cast<std::uint16_t>(i * 5 + j), b = static_cast<std::uint16_t>((i + 1) * 5 + j);
            m.indices.insert(m.indices.end(), {a, b, static_cast<std::uint16_t>(a + 1), static_cast<std::uint16_t>(a + 1), b,
                                               static_cast<std::uint16_t>(b + 1)});
        }
    output->min_easting = output->max_easting = m.origin[0] + m.positions[0];
    output->min_northing = output->max_northing = m.origin[1] + m.positions[1];
    for (std::size_t v = 0; v < m.positions.size(); v += 3) {
        output->min_easting = std::min(output->min_easting, m.origin[0] + m.positions[v]);
        output->max_easting = std::max(output->max_easting, m.origin[0] + m.positions[v]);
        output->min_northing = std::min(output->min_northing, m.origin[1] + m.positions[v + 1]);
        output->max_northing = std::max(output->max_northing, m.origin[1] + m.positions[v + 1]);
    }
    // A half-metre structural slab supplies visible fascia and a ceiling collider beneath the same road surface. No
    // pillars are invented in the lower roadway's footprint.
    const auto top_vertices = m.positions.size() / 3;
    const auto top_indices = m.indices;
    for (std::size_t i = 0; i < top_vertices; ++i) {
        m.positions.insert(m.positions.end(), {m.positions[i * 3], m.positions[i * 3 + 1], m.positions[i * 3 + 2] - .5f});
        m.normals.insert(m.normals.end(), {0, 0, -1});
        m.land_class.push_back(static_cast<std::uint8_t>(output->land_class));
    }
    for (std::size_t i = 0; i < top_indices.size(); i += 3)
        m.indices.insert(m.indices.end(), {static_cast<std::uint16_t>(top_vertices + top_indices[i]),
                                           static_cast<std::uint16_t>(top_vertices + top_indices[i + 2]),
                                           static_cast<std::uint16_t>(top_vertices + top_indices[i + 1])});
    for (int i = 0; i < count; ++i)
        for (int side : {0, 4}) {
            const auto a = static_cast<std::uint16_t>(i * 5 + side), b = static_cast<std::uint16_t>((i + 1) * 5 + side);
            const auto c = static_cast<std::uint16_t>(b + top_vertices), d = static_cast<std::uint16_t>(a + top_vertices);
            if (side == 0) m.indices.insert(m.indices.end(), {a, c, b, a, d, c});
            else m.indices.insert(m.indices.end(), {a, b, c, a, c, d});
        }
    return output;
}

// A bridge continued onto its ground approach (see RoadSeparation). Every paved ground stretch is sampled at 1 m
// (position, surface height, half width + shoulder); an approach stretch (one end on a bridge way's end node) is walked
// from that end and the deck covers it up to the last station whose footprint overlaps a road more than clearance_m
// below it. The inferred deck carries the approach's own profile, so its heights do not change, only what supports them.
void add_approach_decks(const RoadGeomTile& geometry, const RoadSeparation& sep, std::vector<std::shared_ptr<const RoadDeck>>& result) {
    const auto& graph = geometry.source.graph;
    std::set<std::int64_t> bridge_ends;
    for (const auto& w : graph.ways) {
        auto attrs = resolve_road_attributes(w);
        if (attrs && attrs->bridge && !w.node_ids.empty()) {
            bridge_ends.insert(w.node_ids.front());
            bridge_ends.insert(w.node_ids.back());
        }
    }
    if (bridge_ends.empty()) return;
    const auto ground = [](const RoadGeomEntry& e) {
        return e.profile && !e.profile->attributes.bridge && !e.profile->attributes.tunnel && e.profile->attributes.land_class == LandClass::PavedRoad;
    };
    struct Sample {
        double x, y, z, core;
        std::size_t entry;
    };
    std::vector<Sample> samples;
    std::map<std::pair<int, int>, std::vector<std::size_t>> bins;
    constexpr double bin_m = 16;
    const auto bin = [](double v) { return static_cast<int>(std::floor(v / bin_m)); };
    for (std::size_t i = 0; i < geometry.entries.size(); ++i) {
        const auto& e = geometry.entries[i];
        if (!ground(e)) continue;
        const auto& p = *e.profile;
        const int count = std::max(1, static_cast<int>(std::ceil(p.reference.length_m)));
        for (int k = 0; k <= count; ++k) {
            const double station = k == count ? p.reference.length_m : p.reference.length_m * k / count;
            auto xy = p.reference.at(station);
            auto z = p.at(station);
            if (!xy || !z) continue;
            double core=p.attributes.width_mm/2000.0+p.attributes.shoulder_mm/1000.0;
            if(e.cross_section) {
                const auto f=carve_footprint(*e.cross_section,station);const double offset=(f.outer_left+f.outer_right)*.5;
                core=(f.outer_left-f.outer_right)*.5;
                xy->x-=det::sin(xy->heading)*offset;xy->y+=det::cos(xy->heading)*offset;
            }
            bins[{bin(xy->x), bin(xy->y)}].push_back(samples.size());
            samples.push_back({xy->x, xy->y, z->height_m, core, i});
        }
    }
    double reach_core = 0;
    for (const auto& s : samples) reach_core = std::max(reach_core, s.core);
    for (std::size_t i = 0; i < geometry.entries.size(); ++i) {
        const auto& e = geometry.entries[i];
        if (!ground(e)) continue;
        const auto* w = find_way(graph, e.way_id);
        if (!w || e.stretch.end_ref >= w->node_ids.size()) continue;
        const auto& p = *e.profile;
        const double length = p.reference.length_m;
        double core=p.attributes.width_mm/2000.0+p.attributes.shoulder_mm/1000.0;
        if(e.cross_section)for(double at:{0.0,length*.5,length}) {
            const auto f=carve_footprint(*e.cross_section,at);core=std::max(core,(f.outer_left-f.outer_right)*.5);
        }
        const double radius = core + reach_core + sep.margin_m;
        for (int end = 0; end < 2; ++end) {
            if (!bridge_ends.contains(w->node_ids[end ? e.stretch.end_ref : e.stretch.start_ref])) continue;
            const int steps = static_cast<int>(std::floor(std::min(sep.approach_reach_m, length)));
            double covered = -1;
            for (int k = 0; k <= steps; ++k) {
                const double station = end ? length - k : static_cast<double>(k);
                auto xy = p.reference.at(station);
                auto z = p.at(station);
                if (!xy || !z) continue;
                double station_core=core;
                if(e.cross_section) {
                    const auto f=carve_footprint(*e.cross_section,station);const double offset=(f.outer_left+f.outer_right)*.5;
                    station_core=(f.outer_left-f.outer_right)*.5;
                    xy->x-=det::sin(xy->heading)*offset;xy->y+=det::cos(xy->heading)*offset;
                }
                bool overlap = false;
                for (int by = bin(xy->y - radius); by <= bin(xy->y + radius) && !overlap; ++by)
                    for (int bx = bin(xy->x - radius); bx <= bin(xy->x + radius) && !overlap; ++bx) {
                        const auto it = bins.find({bx, by});
                        if (it == bins.end()) continue;
                        for (const auto index : it->second) {
                            const auto& s = samples[index];
                            if (s.entry == i) continue;
                            if (z->height_m - s.z <= sep.clearance_m) continue;
                            if (det::hypot(xy->x - s.x, xy->y - s.y) <= station_core + s.core + sep.margin_m) {
                                overlap = true;
                                break;
                            }
                        }
                    }
                if (overlap) covered = k;
            }
            if (covered < 0) continue;
            const double reach = std::min(length, covered + 1.0); // the sample spacing: the overlap may lie up to 1 m beyond
            if (auto d = end ? deck(p, e.way_id, e.stretch.start_ref, length - reach, length, true, e.cross_section) : deck(p, e.way_id, e.stretch.start_ref, 0, reach, true, e.cross_section))
                result.push_back(std::move(d));
        }
    }
}

} // namespace

std::optional<double> road_deck_height(const RoadDeck& deck, double station, double lateral) {
    if (station < deck.start_station - 1e-6 || station > deck.end_station + 1e-6) return {};
    auto it = std::upper_bound(deck.vertical.begin(), deck.vertical.end(), station, [](double s, const auto& c) { return s < c.station_m; });
    if (it == deck.vertical.begin()) return {};
    --it;
    auto pose = it->at(std::clamp(station, it->station_m, it->station_m + it->length_m));
    if (!pose) return {};
    if(deck.cross_section) {
        const auto f=carve_footprint(*deck.cross_section,station);
        return pose->height_m+road_crown_m(f.crown,deck.crown,lateral-f.crown_pivot,.5)-road_crown_m(f.crown,deck.crown,-f.crown_pivot,.5);
    }
    return pose->height_m - deck.crown * (det::sqrt(lateral * lateral + .25) - .5);
}

std::vector<std::shared_ptr<const RoadDeck>> build_road_decks(const RoadGeomTile& geometry, const RoadSeparation& separation) {
    std::vector<std::shared_ptr<const RoadDeck>> result;
    for (const auto& entry : geometry.entries)
        if (entry.profile && (entry.profile->attributes.bridge || entry.profile->attributes.tunnel)) {
            if (auto d = deck(*entry.profile, entry.way_id, entry.stretch.start_ref, 0, entry.profile->reference.length_m, false, entry.cross_section)) result.push_back(std::move(d));
        }
    // Underpass decks (roads.geom encoding 9, D56): the served span of a ground stretch over a lower road in a DEM
    // cutting, carrying the stretch's own (masked) profile; inferred like a roof.
    for (const auto& u : geometry.underpasses) {
        if (u.entry_index >= geometry.entries.size()) continue;
        const auto& entry = geometry.entries[u.entry_index];
        if (!entry.profile) continue;
        if (auto d = deck(*entry.profile, entry.way_id, entry.stretch.start_ref, u.start_m, u.end_m, true, entry.cross_section)) result.push_back(std::move(d));
    }
    const auto in_underpass = [&](std::size_t entry_index, double station) {
        for (const auto& u : geometry.underpasses)
            if (u.entry_index == entry_index && station >= u.start_m && station <= u.end_m) return true;
        return false;
    };
    const auto& graph = geometry.source.graph;
    for (const auto& lower : graph.ways) {
        auto attrs = resolve_road_attributes(lower);
        if (!attrs || !attrs->tunnel) continue;
        for (std::size_t entry_index = 0; entry_index < geometry.entries.size(); ++entry_index) {
            const auto& entry = geometry.entries[entry_index];
            if (!entry.profile || entry.profile->attributes.bridge || entry.profile->attributes.tunnel) continue;
            const auto* upper = find_way(graph, entry.way_id);
            if (!upper || upper->rank < 5 || osm_layer(*upper) <= osm_layer(lower)) continue;
            const auto& p = *entry.profile;
            for (std::size_t i = 1; i < lower.node_ids.size(); ++i) {
                const auto a = find_node(graph, lower.node_ids[i - 1]), b = find_node(graph, lower.node_ids[i]);
                const double ax = a.x / 1000.0, ay = a.y / 1000.0, dx = (b.x - a.x) / 1000.0, dy = (b.y - a.y) / 1000.0;
                for (const auto& segment : p.reference.segments) {
                    auto begin = segment.curve.at(0), finish = segment.curve.at(segment.curve.length);
                    if (!begin || !finish) continue;
                    const double ux = finish->x - begin->x, uy = finish->y - begin->y;
                    const double determinant = ux * dy - uy * dx;
                    if (std::abs(determinant) < 1e-6) continue;
                    const double vx = ax - begin->x, vy = ay - begin->y;
                    const double t = (vx * dy - vy * dx) / determinant, u = (vx * uy - vy * ux) / determinant;
                    if (t < 0 || t > 1 || u < 0 || u > 1) continue;
                    const double station = segment.station_m + t * segment.curve.length;
                    if (in_underpass(entry_index, station)) continue;   // the served underpass deck already carries it
                    const double sine = std::abs(determinant) / std::max(.001, det::hypot(ux, uy) * det::hypot(dx, dy));
                    const double extent = std::max(20.0, (attrs->width_mm / 2000.0 + 4) / std::max(.2, sine));
                    const double from = std::max(0.0, station - extent), to = std::min(p.reference.length_m, station + extent);
                    // Infer the roof from the upper road's supported ends, never from the lower tunnel's bare-earth floor.
                    auto z0 = p.at(from), z1 = p.at(to);
                    if (!z0 || !z1) continue;
                    auto upper_profile = p;
                    // Preserve approach grade as well as height: a straight roof inserted into a sloping profile
                    // creates a wheel-impact kink.
                    upper_profile.vertical.segments = {{from, to - from, z0->height_m, z1->height_m, z0->grade, z1->grade}};
                    if (auto d = deck(upper_profile, entry.way_id, entry.stretch.start_ref, from, to, true, entry.cross_section)) result.push_back(std::move(d));
                }
            }
        }
    }
    if (separation.enabled) add_approach_decks(geometry, separation, result);
    return result;
}

StructureSet build_structure_set(const RoadGeomTile& geometry, const RoadSeparation& separation) {
    StructureSet set;
    set.key = geometry.source.key;
    set.decks = build_road_decks(geometry, separation);
    return set;
}

} // namespace g2m::world
