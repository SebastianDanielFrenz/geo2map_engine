// tests/support/road_completion_reference.h - a FROZEN copy of racing_game's profile completion (design 6.6, test
// T18): `complete_road_profiles` and `abutment` of racing_game/core/src/road_structures.cpp and the 20 m retry of
// racing_game/core/src/world_terrain.cpp, as they stood on 2026-10-05, ported only as far as needed to compile here
// (rg:: -> g2m_test::game_reference, the RG_STRUCTURE log lines dropped). It is the reference the library tier is
// compared with: libm (std::sin, std::cos, std::hypot, std::sqrt) is allowed in tests and used on purpose, the
// anchors map is the game's own (filled in entry order, read by the ground step), and the walk has no budget. The
// tunnel-floor path is not ported (the library does not complete tunnels, design 6.6). Do not "fix" this file: it is
// what the game does today.
#pragma once

#include "g2m/layer/road_geom_tile.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <map>
#include <optional>
#include <string>
#include <vector>

namespace g2m_test::game_reference {

inline const g2m::RoadGraphWay* way(const g2m::RoadGraph& g, std::int64_t id) {
    auto it = std::lower_bound(g.ways.begin(), g.ways.end(), id, [](const auto& w, auto value) { return w.osm_id < value; });
    return it == g.ways.end() || it->osm_id != id ? nullptr : &*it;
}
inline g2m::geom::PointMm node(const g2m::RoadGraph& g, std::int64_t id) {
    auto it = std::lower_bound(g.nodes.begin(), g.nodes.end(), id, [](const auto& n, auto value) { return n.osm_id < value; });
    return it->position;
}

// Find supported ground away from the bare-earth hole. Degree-two bridges can be split over several OSM ways, and a
// nearby junction is not a reason to lose the whole structure: continue the most aligned ground approach.
inline std::optional<g2m::RoadJunctionPlane> abutment(const g2m::RoadGraph& g, const g2m::RoadDemSampler& dem,
    g2m::geo::UtmZone zone, std::int64_t id, double out_x, double out_y) {
    const auto origin = node(g, id); auto current = id; std::int64_t previous = 0; double remaining = 32;
    for (int work = 0; work < 100; ++work) {
        const g2m::RoadGraphEdge* selected = nullptr; double score = .25;
        for (const auto& edge : g.edges) {
            if (edge.from_node != current && edge.to_node != current) continue;
            const auto other = edge.from_node == current ? edge.to_node : edge.from_node; if (other == previous) continue;
            const auto* w = way(g, edge.way_id); if (!w) continue;
            auto attrs = g2m::resolve_road_attributes(*w); if (!attrs || attrs->tunnel) continue;
            const auto a = node(g, current), b = node(g, other);
            const double dx = (b.x - a.x) / 1000.0, dy = (b.y - a.y) / 1000.0, len = std::hypot(dx, dy);
            if (len < .001) continue;
            const double dot = (dx * out_x + dy * out_y) / len;
            if (dot > score) { score = dot; selected = &edge; }
        }
        if (!selected) break;
        const auto other = selected->from_node == current ? selected->to_node : selected->from_node;
        const auto a = node(g, current), b = node(g, other);
        const double dx = (b.x - a.x) / 1000.0, dy = (b.y - a.y) / 1000.0, len = std::hypot(dx, dy);
        const auto chosen_attrs = g2m::resolve_road_attributes(*way(g, selected->way_id));
        const bool on_bridge = chosen_attrs && chosen_attrs->bridge;
        if (!on_bridge && len >= remaining) {
            const double t = remaining / len;
            const g2m::geom::PointMm support{std::llround(a.x + t * (b.x - a.x)), std::llround(a.y + t * (b.y - a.y))};
            auto plane = dem.junction_plane(zone, support); if (!plane) return {};
            const double h = plane->height / 256.0 + plane->grade_x * (origin.x - support.x) / 1000.0 + plane->grade_y * (origin.y - support.y) / 1000.0;
            plane->height = static_cast<std::int32_t>(std::llround(h * 256)); return *plane;
        }
        if (!on_bridge) remaining -= len;
        out_x = dx / len; out_y = dy / len; previous = current; current = other;
    }
    return {};
}

// Bridge and ground completion of the game (its tunnel-floor loop is not ported). `geometry` carries the strict
// entries; the builder is made from the TILE graph, as the game does.
inline void complete_road_profiles(g2m::RoadGeomTile& geometry, const g2m::RoadDemSampler& dem) {
    const auto& graph = geometry.source.graph; auto builder = g2m::RoadReferenceBuilder::make(graph); if (!builder) return;
    const auto zone = geometry.source.key.zone;
    std::map<std::int64_t, g2m::RoadJunctionPlane> anchors;
    // Existing accepted bridge anchors are authoritative for adjoining ground.
    for (const auto& entry : geometry.entries) if (entry.profile && entry.profile->attributes.bridge) {
        const auto* w = way(graph, entry.way_id); if (!w) continue;
        const auto& p = *entry.profile;
        for (int end = 0; end < 2; ++end) {
            const double station = end ? p.reference.length_m : 0; auto xy = p.reference.at(station); auto z = p.at(station);
            if (xy && z) anchors[w->node_ids[end ? entry.stretch.end_ref : entry.stretch.start_ref]] = {static_cast<std::int32_t>(std::llround(z->height_m * 256)), z->grade * std::cos(xy->heading), z->grade * std::sin(xy->heading)};
        }
    }
    for (auto& entry : geometry.entries) {
        const auto* w = way(graph, entry.way_id); if (!w || entry.profile) continue;
        auto attrs = g2m::resolve_road_attributes(*w); if (!attrs || !attrs->bridge) continue;
        g2m::ReferenceFitLimits limits; limits.allow_curvature_exceptions = true;
        auto ref = builder->fit_stretch(entry.way_id, entry.stretch, limits); if (!ref) continue;
        auto start = ref->at(0), end = ref->at(ref->length_m); if (!start || !end) continue;
        auto a = abutment(graph, dem, zone, w->node_ids[entry.stretch.start_ref], -std::cos(start->heading), -std::sin(start->heading));
        auto b = abutment(graph, dem, zone, w->node_ids[entry.stretch.end_ref], std::cos(end->heading), std::sin(end->heading));
        if (anchors.contains(w->node_ids[entry.stretch.start_ref])) a = anchors[w->node_ids[entry.stretch.start_ref]];
        if (anchors.contains(w->node_ids[entry.stretch.end_ref])) b = anchors[w->node_ids[entry.stretch.end_ref]];
        if (!a || !b) continue;
        g2m::VerticalFitLimits fit; fit.max_grade = std::max(.12, attrs->max_grade_per_mille / 1000.0);
        fit.min_radius_m = std::min(250, attrs->min_vertical_radius_m); fit.max_iterations = 16384;
        fit.start_grade = a->grade_x * std::cos(start->heading) + a->grade_y * std::sin(start->heading);
        fit.end_grade = b->grade_x * std::cos(end->heading) + b->grade_y * std::sin(end->heading);
        const std::array<g2m::VerticalSample, 2> samples{{{0, a->height / 256.0}, {ref->length_m, b->height / 256.0}}};
        auto vertical = g2m::fit_vertical_profile(samples, fit);
        if (!vertical) {
            // Estimated DEM gradients at a short span can disagree. Its abutment rise remains measured; choose the common secant grade.
            fit.start_grade = fit.end_grade = (b->height - a->height) / 256.0 / ref->length_m;
            vertical = g2m::fit_vertical_profile(samples, fit); if (!vertical) continue;
            a->grade_x = fit.start_grade * std::cos(start->heading); a->grade_y = fit.start_grade * std::sin(start->heading);
            b->grade_x = fit.end_grade * std::cos(end->heading); b->grade_y = fit.end_grade * std::sin(end->heading);
        }
        g2m::RoadProfile p; p.stretch = entry.stretch; p.attributes = *attrs; p.reference = std::move(*ref); p.vertical = std::move(*vertical);
        p.start_anchor_inferred = p.end_anchor_inferred = true;
        anchors[w->node_ids[entry.stretch.start_ref]] = *a; anchors[w->node_ids[entry.stretch.end_ref]] = *b;
        entry.profile = std::move(p); entry.decline.reset();
    }
    for (auto& entry : geometry.entries) {
        const auto* w = way(graph, entry.way_id); if (!w) continue;
        auto attrs = g2m::resolve_road_attributes(*w); if (!attrs || attrs->bridge || attrs->tunnel) continue;
        const auto first_id = w->node_ids[entry.stretch.start_ref], last_id = w->node_ids[entry.stretch.end_ref];
        if (entry.profile && !anchors.contains(first_id) && !anchors.contains(last_id)) continue;
        g2m::ReferenceFitLimits limits; limits.allow_curvature_exceptions = true;
        auto ref = builder->fit_stretch(entry.way_id, entry.stretch, limits); if (!ref || ref->length_m < .5) continue;
        auto a = dem.junction_plane(zone, node(graph, first_id)), b = dem.junction_plane(zone, node(graph, last_id));
        if (anchors.contains(first_id)) a = anchors[first_id]; if (anchors.contains(last_id)) b = anchors[last_id];
        if (!a || !b) continue;
        auto start = ref->at(0), end = ref->at(ref->length_m); if (!start || !end) continue;
        g2m::VerticalFitLimits fit; fit.max_grade = std::max(.12, attrs->max_grade_per_mille / 1000.0);
        fit.min_radius_m = std::min(500, attrs->min_vertical_radius_m); fit.max_iterations = 16384;
        fit.start_grade = a->grade_x * std::cos(start->heading) + a->grade_y * std::sin(start->heading);
        fit.end_grade = b->grade_x * std::cos(end->heading) + b->grade_y * std::sin(end->heading);
        fit.start_grade = std::clamp(fit.start_grade, -.10, .10);
        fit.end_grade = std::clamp(fit.end_grade, -.10, .10);
        const int count = std::max(1, static_cast<int>(std::ceil(ref->length_m / 40)));
        if (count > 2047) continue;
        std::vector<g2m::VerticalSample> samples{{0, a->height / 256.0}}; bool complete = true;
        for (int i = 1; i < count; ++i) {
            const double station = ref->length_m * i / count; auto xy = ref->at(station); if (!xy) { complete = false; break; }
            auto height = dem.junction_height(zone, {std::llround(xy->x * 1000), std::llround(xy->y * 1000)});
            if (!height) { complete = false; break; }
            double z = *height / 256.0;
            // Carry inferred abutments to supported ground, keeping ramps out of the DEM's underpass depression on either side of the span.
            if (anchors.contains(first_id) && station < 32) z = a->height / 256.0 + fit.start_grade * station;
            if (anchors.contains(last_id) && ref->length_m - station < 32) z = b->height / 256.0 - fit.end_grade * (ref->length_m - station);
            samples.push_back({station, z});
        }
        if (!complete) continue;
        samples.push_back({ref->length_m, b->height / 256.0});
        auto vertical = g2m::fit_vertical_profile(samples, fit); if (!vertical) continue;
        g2m::RoadProfile p; p.stretch = entry.stretch; p.attributes = *attrs; p.reference = std::move(*ref); p.vertical = std::move(*vertical);
        entry.profile = std::move(p); entry.decline.reset();
    }
}

// The retry of WorldTerrain::road_surface_patch: explicit numerical declines are re-run at 20 m / 16384 iterations
// with the same limits. In the game it runs AFTER complete_road_profiles.
inline void retry_declines(g2m::RoadGeomTile& geometry, const g2m::RoadDemSampler& dem) {
    auto builder = g2m::RoadReferenceBuilder::make(geometry.source.graph); if (!builder) return;
    for (auto& entry : geometry.entries) {
        if (entry.profile || !entry.decline) continue;
        if (entry.decline->message.find("bounded solver") == std::string::npos &&
            entry.decline->message.find("endpoint curvature") == std::string::npos) continue;
        g2m::RoadProfileLimits limits; limits.sample_spacing_m = 20; limits.max_iterations = 16384;
        limits.reference.allow_curvature_exceptions = true;
        auto profile = g2m::build_road_profile(*builder, dem, geometry.source.key.zone, entry.way_id, entry.stretch, limits);
        if (profile) { entry.profile = std::move(*profile); entry.decline.reset(); }
    }
}

}  // namespace g2m_test::game_reference
