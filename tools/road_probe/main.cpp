#include "g2m/layer/road_graph_tile.h"
#include "g2m/layer/road_reference.h"
#include "g2m/layer/road_attributes.h"
#include "g2m/layer/road_profile.h"
#include "g2m/layer/road_dem_dependencies.h"
#include "g2m/layer/derive_terrain.h"
#include "g2m/core/det_math.h"
#include "g2m/server/tile_store.h"
#include <chrono>
#include <fstream>
#include <iostream>
#include <iomanip>
#include "g2m/core/geo/utm.h"
#include <string_view>
#include <map>
#include <algorithm>
using namespace g2m;
int main(int argc, char** argv) {
    bool exceptions_enabled = false, profiles_enabled = false;
    for (int i = 5; i < argc; ++i) {
        if (std::string_view(argv[i]) == "--exceptions" && !exceptions_enabled) exceptions_enabled = true;
        else if (std::string_view(argv[i]) == "--profiles" && !profiles_enabled) profiles_enabled = true;
        else { std::cerr << "unknown or repeated option\n"; return 1; }
    }
    if (argc < 5 || argc > 7) {
        std::cerr << "usage: g2m_road_probe STORE SCOPE TILE_KEY OUTPUT_CSV [--exceptions] [--profiles]\n"; return 1;
    }
    const auto key = parse_tile_path(argv[3]);
    if (!key || key->level != 2) { std::cerr << "level-2 tile key required\n"; return 1; }
    TileStoreConfig config; config.dir = argv[1]; config.read_only = true;
    auto store = TileStore::open(config); if (!store) { std::cerr << store.error().message << '\n'; return 1; }
    RoadGraphDeriver deriver; std::vector<InputTile> inputs;
    for (const auto& ref : deriver.inputs_for(*key)) {
        auto tile = (*store)->get({argv[2], ref.layer, ref.key});
        if (!tile) { std::cerr << tile.error().message << '\n'; return 1; }
        InputTile input; input.ref = ref;
        if (*tile) { input.header = std::move((*tile)->header); input.body = std::move((*tile)->body); }
        inputs.push_back(std::move(input));
    }
    if (std::none_of(inputs.begin(), inputs.end(), [](const auto& input) { return input.header.has_value(); })) {
        std::cerr << "no source OSM tiles for scope/target\n"; return 1;
    }
    using Clock = std::chrono::steady_clock;
    const auto begin = Clock::now(); const auto derived = deriver.derive({*key, 0, inputs});
    const auto graph_ms = std::chrono::duration<double, std::milli>(Clock::now() - begin).count();
    if (!derived) { std::cerr << derived.error().message << '\n'; return 1; }
    const auto body = decode_body(derived->body); if (!body) { std::cerr << body.error().message << '\n'; return 1; }
    const auto tile = decode_road_graph(*body); if (!tile) { std::cerr << tile.error().message << '\n'; return 1; }
    std::ofstream csv(argv[4]); if (!csv) { std::cerr << "cannot open output CSV\n"; return 1; }
    csv << "way_id,highway,rank,refs,min_class_radius_m,median_source_radius_m,owned_edges,reference_ok,stretches,failed_stretches,failed_owned_edges,fitted_length_m,reference_ms,error\n";
    const auto references_begin = Clock::now(); std::size_t accepted = 0; double length = 0;
    const auto builder = RoadReferenceBuilder::make(tile->graph); if (!builder) { std::cerr << builder.error().message << '\n'; return 1; }
    std::map<std::int64_t, std::size_t> owned;
    std::map<std::int64_t, std::vector<std::uint32_t>> owned_pairs;
    for (const auto index : tile->owned_edges) {
        const auto& edge = tile->graph.edges[index]; ++owned[edge.way_id]; owned_pairs[edge.way_id].push_back(edge.segment_index);
    }
    ReferenceFitLimits limits; limits.allow_curvature_exceptions = exceptions_enabled;
    std::size_t exceptions = 0;
    std::size_t total_stretches = 0, failed_stretches = 0, failed_owned_edges = 0;
    std::map<std::string, std::size_t> failures;
    for (const auto& way : tile->graph.ways) {
        const auto start = Clock::now(); const auto parts = builder->fit_stretches(way.osm_id, limits);
        const auto elapsed = std::chrono::duration<double, std::milli>(Clock::now() - start).count();
        if (!parts) { std::cerr << parts.error().message << '\n'; return 1; }
        total_stretches += parts->size(); std::size_t failed = 0, local_failed = 0; double fitted_length = 0; std::string reason;
        for (const auto& part : *parts) {
            if (part.reference) { if (part.reference->curvature_exception) ++exceptions; fitted_length += part.reference->length_m; continue; }
            ++failed; ++failures[part.reference.error().message];
            if (reason.empty()) reason = part.reference.error().message;
            for (const auto pair : owned_pairs[way.osm_id])
                if (pair >= part.stretch.start_ref && pair < part.stretch.end_ref) ++local_failed;
        }
        failed_stretches += failed; failed_owned_edges += local_failed; length += fitted_length;
        if (!failed) ++accepted;
        const auto highway = std::find_if(way.tags.begin(), way.tags.end(), [](const auto& tag) { return tag.first == "highway"; });
        const auto attrs = resolve_road_attributes(way); if (!attrs) { std::cerr << attrs.error().message << '\n'; return 1; }
        const auto point = [&](auto id) {
            return std::lower_bound(tile->graph.nodes.begin(), tile->graph.nodes.end(), id,
                [](const auto& n, auto search) { return n.osm_id < search; })->position;
        };
        std::vector<double> radii;
        for (std::size_t i = 1; i + 1 < way.node_ids.size(); ++i) {
            const auto a = point(way.node_ids[i - 1]), b = point(way.node_ids[i]), c = point(way.node_ids[i + 1]);
            const auto ax = (b.x - a.x) / 1000.0, ay = (b.y - a.y) / 1000.0;
            const auto bx = (c.x - b.x) / 1000.0, by = (c.y - b.y) / 1000.0;
            const auto cx = (c.x - a.x) / 1000.0, cy = (c.y - a.y) / 1000.0;
            const auto cross = std::fabs(ax * by - ay * bx);
            if (cross < 1e-8) continue;
            radii.push_back(det::sqrt((ax * ax + ay * ay) * (bx * bx + by * by) * (cx * cx + cy * cy)) / (2 * cross));
        }
        std::sort(radii.begin(), radii.end()); const auto median_radius = radii.empty() ? 0 : radii[radii.size() / 2];
        csv << way.osm_id << ',' << (highway == way.tags.end() ? "" : highway->second) << ',' << way.rank << ',' << way.node_ids.size()
            << ',' << attrs->min_horizontal_radius_m << ',' << median_radius << ',' << owned[way.osm_id] << ',' << (failed == 0) << ',' << parts->size() << ',' << failed << ',' << local_failed
            << ',' << fitted_length << ',' << elapsed << ',' << reason << '\n';
    }
    if (!csv) { std::cerr << "failed writing CSV\n"; return 1; }
    const auto reference_ms = std::chrono::duration<double, std::milli>(Clock::now() - references_begin).count();
    std::cout << "tile=" << to_string(*key) << " nodes=" << tile->graph.nodes.size() << " ways=" << tile->graph.ways.size()
        << " edges=" << tile->graph.edges.size() << " owned=" << tile->owned_edges.size()
        << " graph_body_bytes=" << derived->body.size() << " body_bytes_per_km2=" << derived->body.size() / 1.048576
        << " graph_ms=" << graph_ms << " reference_ok=" << accepted << " reference_km=" << length / 1000
        << " reference_ms=" << reference_ms << " stretches=" << total_stretches << " declined_stretches=" << failed_stretches
        << " curvature_exceptions=" << exceptions << " declined_owned_edges=" << failed_owned_edges << '\n';
    for (const auto& [reason, count] : failures) std::cout << "declined_stretches=" << count << " reason=" << reason << '\n';
    if (profiles_enabled) {
        const auto deps = road_dem_dependencies(tile->graph, key->zone); if (!deps) { std::cerr << deps.error().message << '\n'; return 1; }
        std::vector<HeightTile> heights; heights.reserve(deps->size()); ElevBaseDeriver elevation;
        const auto profile_begin = Clock::now();
        for (const auto& dep : *deps) {
            std::vector<InputTile> raw;
            for (const auto& ref : elevation.inputs_for(dep.key)) {
                auto stored = (*store)->get({argv[2], ref.layer, ref.key});
                if (!stored) { std::cerr << stored.error().message << '\n'; return 1; }
                InputTile input; input.ref = ref;
                if (*stored) { input.header = std::move((*stored)->header); input.body = std::move((*stored)->body); }
                raw.push_back(std::move(input));
            }
            const auto derived_height = elevation.derive({dep.key, 0, raw});
            if (!derived_height) { std::cerr << derived_height.error().message << '\n'; return 1; }
            const auto height_body = decode_body(derived_height->body); if (!height_body) { std::cerr << height_body.error().message << '\n'; return 1; }
            heights.emplace_back(); const auto decoded = decode_height_tile(*height_body, dep.key, heights.back());
            if (!decoded) { std::cerr << decoded.error().message << '\n'; return 1; }
        }
        std::vector<const HeightTile*> views; for (const auto& height : heights) views.push_back(&height);
        const auto dem = RoadDemSampler::make(views); if (!dem) { std::cerr << dem.error().message << '\n'; return 1; }
        std::map<std::string, std::size_t> declined; std::size_t fitted = 0, bridges = 0;
        RoadProfileLimits profile_limits; profile_limits.reference = limits;
        for (const auto& way : tile->graph.ways) {
            const auto stretches = builder->stretches(way.osm_id); if (!stretches) { std::cerr << stretches.error().message << '\n'; return 1; }
            for (const auto stretch : *stretches) {
                const auto profile = build_road_profile(*builder, *dem, key->zone, way.osm_id, stretch, profile_limits);
                if (profile) { ++fitted; if (profile->attributes.bridge) ++bridges; }
                else {
                    const auto count = ++declined[profile.error().message];
                    if (count <= 3) {
                        const auto point = [&](std::uint32_t index) {
                            return std::lower_bound(tile->graph.nodes.begin(), tile->graph.nodes.end(), way.node_ids[index],
                                [](const auto& node, auto id) { return node.osm_id < id; })->position;
                        };
                        const auto a = dem->junction_height(key->zone, point(stretch.start_ref));
                        const auto b = dem->junction_height(key->zone, point(stretch.end_ref));
                        const auto reference = builder->fit_stretch(way.osm_id, stretch, limits);
                        std::cout << "profile_example_way=" << way.osm_id << " start_ref=" << stretch.start_ref
                            << " end_ref=" << stretch.end_ref;
                        if (a && b && reference) std::cout << " length_m=" << reference->length_m
                            << " endpoint_rise_m=" << (static_cast<double>(*b) - *a) / 256
                            << " endpoint_mean_grade=" << (static_cast<double>(*b) - *a) / (256 * reference->length_m);
                        for (const auto index : {stretch.start_ref, stretch.end_ref}) {
                            const auto p = point(index); const auto ll = geo::Utm{}.reverse(key->zone, p.x / 1000.0, p.y / 1000.0);
                            std::cout << std::setprecision(10) << " node=" << way.node_ids[index] << " lat=" << ll.lat << " lon=" << ll.lon;
                        }
                        std::cout << " reason=" << profile.error().message << '\n';
                    }
                }
            }
        }
        std::cout << "dem_tiles=" << heights.size() << " profile_ok=" << fitted << " bridge_profiles=" << bridges
            << " profile_with_dem_ms=" << std::chrono::duration<double, std::milli>(Clock::now() - profile_begin).count() << '\n';
        for (const auto& [reason, count] : declined) std::cout << "declined_profiles=" << count << " reason=" << reason << '\n';
    }
    return 0;
}
