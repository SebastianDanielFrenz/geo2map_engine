#include "g2m/layer/road_geom_tile.h"
#include "g2m/layer/road_completion.h"
#include "g2m/layer/road_furniture.h"
#include "g2m/layer/road_junction.h"
#include "g2m/layer/road_underpass.h"
#include "g2m/core/bytes.h"
#include "g2m/core/det_math.h"
#include "g2m/core/tile_key.h"
#include <algorithm>
#include <bit>
#include <limits>
#include <cmath>
#include <cstdint>
#include <tuple>
#include <utility>
namespace g2m {
namespace {
constexpr std::size_t kMaxEntries = 100000, kMaxPieces = 1000000;
// Encoding 7 budgets (design 9.1). A ring's validity check is quadratic in its vertex count, so rings and the tile's
// total are capped (the fixtures' largest outer ring has 246 vertices).
constexpr std::size_t kMaxJunctions = 65536, kMaxJunctionNodes = 4096, kMaxJunctionApproaches = 4096;
constexpr std::size_t kMaxRingVertices = kJunctionMaxRingVertices, kMaxTileRingVertices = 262144, kMaxFurniture = 65536;
constexpr std::int64_t kMaxRingReachMm = 1000000;      // every ring vertex within 1 km of the owner node, per axis
constexpr std::int64_t kMaxCoordinateMm = std::int64_t{1} << 42;   // decode bound before any arithmetic
constexpr double kMaxFurnitureT = 200, kMaxFurnitureLength = 100, kMaxRaisedHeightM = 0.3;
constexpr std::size_t kMaxUnderpasses = 65536, kMaxUnderpassLowerWays = 64;   // encoding 9, section 8
constexpr std::int64_t kMaxUnderpassDepthMm = 1000000;
Error invalid(std::string message) { return make_error(ErrorCode::Malformed, "roads.geom: " + message); }
void number(ByteWriter& w, double d) { w.u64(std::bit_cast<std::uint64_t>(d)); }
double number(ByteReader& r) { return std::bit_cast<double>(r.u64()); }
bool finite(double d) { return std::isfinite(d) && std::fabs(d) <= 1e9; }

constexpr std::int32_t kMaxBandWidthMm = 20000, kMaxSectionWidthMm = 100000, kMaxOffsetMm = 200000;
constexpr std::size_t kMaxBands = 32, kMaxFeatures = 65536;
constexpr std::int32_t kMaxFeatureLengthMm = 50000, kMaxFeatureHeightMm = 300, kMaxFeatureWidthMm = 100000;

// Per-record rules of design 7.2 that need only the record and the reference length.
Result<void> validate_cross_section(const RoadCrossSection& xs, double length_m) {
    if (xs.bands.empty() || xs.bands.size() > kMaxBands) return invalid("band count");
    if ((xs.flags & ~static_cast<unsigned>(kXsDefinedMask)) != 0) return invalid("cross-section flags");
    if ((xs.field_declines & ~static_cast<std::uint32_t>(kDeclineDefinedMask)) != 0) return invalid("field declines");
    if (static_cast<unsigned>(xs.count_provenance) > 3 || static_cast<unsigned>(xs.placement_provenance) > 3) return invalid("section provenance");
    if (xs.length_m != length_m) return invalid("cross-section length");
    const std::int64_t limit = static_cast<std::int64_t>(std::ceil(length_m * 1000.0)) + 1;
    std::int64_t total = 0, max_start = 0, max_end = 0;
    for (const auto& b : xs.bands) {
        if (static_cast<unsigned>(b.kind) > 7 || static_cast<unsigned>(b.direction) > 3 || static_cast<unsigned>(b.change) > 4) return invalid("band enum");
        if ((b.turn & ~static_cast<unsigned>(kTurnDefinedMask)) != 0) return invalid("turn bits");
        for (auto p : {b.provenance.kind, b.provenance.direction, b.provenance.width, b.provenance.turn, b.provenance.change})
            if (static_cast<unsigned>(p) > 3) return invalid("band provenance");
        for (const auto v : {b.width_mm, b.start.from_width_mm, b.end.from_width_mm})
            if (v < 0 || v > kMaxBandWidthMm) return invalid("band width");
        for (const auto v : {b.start.length_mm, b.end.length_mm}) if (v < 0 || v > limit) return invalid("ramp length");
        total += b.width_mm; max_start = std::max<std::int64_t>(max_start, b.start.length_mm); max_end = std::max<std::int64_t>(max_end, b.end.length_mm);
    }
    if (total > kMaxSectionWidthMm) return invalid("section width");
    if (xs.anchor.band >= xs.bands.size() || !xs.bands[xs.anchor.band].counted) return invalid("anchor band");
    if (static_cast<unsigned>(xs.anchor.edge) > 2 || static_cast<unsigned>(xs.anchor.source) > 2) return invalid("anchor enum");
    const auto& o = xs.offset;
    if (static_cast<unsigned>(o.mode) > 3) return invalid("offset mode");
    if (std::abs(static_cast<std::int64_t>(o.start_mm)) > kMaxOffsetMm || std::abs(static_cast<std::int64_t>(o.end_mm)) > kMaxOffsetMm) return invalid("offset value");
    if (o.start_length_mm < 0 || o.end_length_mm < 0 || o.start_length_mm > limit || o.end_length_mm > limit) return invalid("offset length");
    if (o.mode == OffsetMode::Hold && (o.start_length_mm != 0 || o.end_length_mm != 0 || o.end_mm != 0)) return invalid("hold offset");
    if (o.mode == OffsetMode::Transition && (o.start_length_mm != 0 || o.end_length_mm != 0)) return invalid("transition offset");
    max_start = std::max<std::int64_t>(max_start, o.start_length_mm); max_end = std::max<std::int64_t>(max_end, o.end_length_mm);
    if (max_start + max_end > limit) return invalid("ramp lengths exceed the stretch");
    for (const auto* j : {&xs.start_join, &xs.end_join}) {
        if (static_cast<unsigned>(j->kind) > 5) return invalid("join kind");
        if ((j->kind == JoinKind::None) != (j->neighbour_way_id == 0)) return invalid("join record");
        if (j->kind == JoinKind::None && j->raw_delta_mm != 0) return invalid("join record");
    }
    return {};
}

Result<void> validate_features(const RoadGeomTile& tile) {
    if (tile.features.size() > kMaxFeatures) return invalid("feature budget");
    const ProfileFeature* previous = nullptr;
    std::size_t sorted_for = SIZE_MAX; std::vector<std::int64_t> nodes;
    for (const auto& f : tile.features) {
        if (f.entry_index >= tile.entries.size() || !tile.entries[f.entry_index].cross_section) return invalid("feature entry");
        const auto& entry = tile.entries[f.entry_index]; const double length = entry.profile->reference.length_m;
        if (static_cast<unsigned>(f.kind) > 9 || static_cast<unsigned>(f.provenance) > 3) return invalid("feature enum");
        if (!std::isfinite(f.station_m) || std::signbit(f.station_m) || f.station_m > length) return invalid("feature station");
        if (f.length_mm < 0 || f.length_mm > kMaxFeatureLengthMm || f.height_mm < 0 || f.height_mm > kMaxFeatureHeightMm ||
            f.width_mm < 0 || f.width_mm > kMaxFeatureWidthMm) return invalid("feature size");
        if (previous) {
            const auto a = std::tuple(previous->entry_index, std::bit_cast<std::uint64_t>(previous->station_m), previous->node_id);
            const auto b = std::tuple(f.entry_index, std::bit_cast<std::uint64_t>(f.station_m), f.node_id);
            if (!(a < b)) return invalid("feature order");
        }
        previous = &f;
        if (sorted_for != f.entry_index) {
            const auto way = std::lower_bound(tile.source.graph.ways.begin(), tile.source.graph.ways.end(), entry.way_id,
                [](const auto& w, auto id) { return w.osm_id < id; });
            if (way == tile.source.graph.ways.end() || way->osm_id != entry.way_id) return invalid("feature way");
            nodes.assign(way->node_ids.begin() + entry.stretch.start_ref, way->node_ids.begin() + entry.stretch.end_ref + 1);
            std::sort(nodes.begin(), nodes.end()); sorted_for = f.entry_index;
        }
        if (!std::binary_search(nodes.begin(), nodes.end(), f.node_id)) return invalid("feature node");
    }
    return {};
}

const RoadGraphNode* graph_node(const RoadGraph& graph, std::int64_t id) {
    const auto it = std::lower_bound(graph.nodes.begin(), graph.nodes.end(), id, [](const auto& n, auto v) { return n.osm_id < v; });
    return it == graph.nodes.end() || it->osm_id != id ? nullptr : &*it;
}
const RoadGraphWay* graph_way(const RoadGraph& graph, std::int64_t id) {
    const auto it = std::lower_bound(graph.ways.begin(), graph.ways.end(), id, [](const auto& w, auto v) { return w.osm_id < v; });
    return it == graph.ways.end() || it->osm_id != id ? nullptr : &*it;
}
// The node at one end of an entry's stretch (0 if the way is unknown; validate_road_geom_tile checked the entries first).
std::int64_t end_node(const RoadGeomTile& tile, std::size_t entry, bool at_end) {
    const auto& e = tile.entries[entry];
    const auto* way = graph_way(tile.source.graph, e.way_id);
    return way ? way->node_ids[at_end ? e.stretch.end_ref : e.stretch.start_ref] : 0;
}
bool zero(double d) { return d == 0 && !std::signbit(d); }
bool mm_exact(double metres) { return std::isfinite(metres) && std::fabs(metres) <= 1e6 && static_cast<double>(std::llround(metres * 1000.0)) / 1000.0 == metres; }

// Design 9.2, junction rules (section 5 of encoding 7).
Result<void> validate_junctions(const RoadGeomTile& tile) {
    if (tile.junctions.size() > kMaxJunctions) return invalid("junction budget");
    const auto& graph = tile.source.graph;
    std::size_t ring_vertices = 0;
    const RoadJunction* previous = nullptr;
    std::vector<std::pair<std::size_t, bool>> seen;
    for (const auto& j : tile.junctions) {
        // Ownership: the smallest node, in this tile's square.
        if (j.nodes.empty() || j.nodes.size() > kMaxJunctionNodes) return invalid("junction node count");
        for (std::size_t i = 0; i < j.nodes.size(); ++i) {
            if (i && j.nodes[i] <= j.nodes[i - 1]) return invalid("junction nodes not ascending");
            if (!graph_node(graph, j.nodes[i])) return invalid("junction node not in the graph");
        }
        const auto owner = graph_node(graph, j.nodes.front());
        if (!road_tile_contains(tile.source.key, owner->position)) return invalid("junction owner outside the tile");
        if (previous) {
            const auto a = previous->nodes.front(), b = j.nodes.front();
            const bool pair = a == b && previous->decline != JunctionDecline::ClusterTooLarge && j.decline == JunctionDecline::ClusterTooLarge;
            if (!(a < b || pair)) return invalid("junction order");
        }
        previous = &j;
        if ((j.flags & ~kJunctionFlagsDefinedMask) != 0) return invalid("junction flags");
        if (static_cast<unsigned>(j.decline) >= kJunctionDeclineCount) return invalid("junction decline");
        const bool cluster = (j.flags & static_cast<std::uint16_t>(JunctionFlags::Cluster)) != 0;
        if (cluster != (j.nodes.size() > 1)) return invalid("junction cluster flag");
        if (j.decline == JunctionDecline::ClusterTooLarge && j.nodes.size() < 2) return invalid("junction cluster record");
        const auto holds = [&](std::int64_t node) { return std::binary_search(j.nodes.begin(), j.nodes.end(), node); };
        // Approaches: indices, status against the entry, the end node, the trim.
        if (j.approaches.empty() || j.approaches.size() > kMaxJunctionApproaches) return invalid("junction approach count");
        seen.clear();
        for (const auto& a : j.approaches) {
            if (a.entry_index >= tile.entries.size()) return invalid("junction approach entry");
            const auto& e = tile.entries[a.entry_index];
            switch (a.status) {
            case JunctionApproachStatus::Used: if (!e.profile || !e.cross_section) return invalid("junction approach status"); break;
            case JunctionApproachStatus::NoProfile: if (e.profile) return invalid("junction approach status"); break;
            // A tunnel approach is declined, or (encoding 8) carries its TunnelFloor profile (D47).
            case JunctionApproachStatus::Tunnel: if (e.profile && !e.profile->attributes.tunnel) return invalid("junction approach status"); break;
            case JunctionApproachStatus::NoCrossSection: if (!e.profile || e.cross_section) return invalid("junction approach status"); break;
            default: return invalid("junction approach status");
            }
            if (!holds(end_node(tile, a.entry_index, a.at_end))) return invalid("junction approach end");
            const double cap = e.profile ? std::max(0.0, e.profile->reference.length_m - 0.5) : 0.0;
            if (!std::isfinite(a.trim_m) || std::signbit(a.trim_m) || a.trim_m > cap) return invalid("junction trim");
            seen.emplace_back(a.entry_index, a.at_end);
        }
        std::sort(seen.begin(), seen.end());
        if (std::adjacent_find(seen.begin(), seen.end()) != seen.end()) return invalid("junction approach repeated");
        for (std::size_t i = 0; i < j.absorbed.size(); ++i) {
            const auto e = j.absorbed[i];
            if (e >= tile.entries.size() || (i && e <= j.absorbed[i - 1])) return invalid("junction absorbed entry");
            if (!holds(end_node(tile, e, false)) || !holds(end_node(tile, e, true))) return invalid("junction absorbed ends");
            if (std::binary_search(seen.begin(), seen.end(), std::pair{e, false}) || std::binary_search(seen.begin(), seen.end(), std::pair{e, true}))
                return invalid("junction absorbed entry is an approach");
        }
        // Rings: all empty iff declined; otherwise counter-clockwise, simple, within reach of the owner, nested.
        const bool declined = j.decline != JunctionDecline::None;
        for (const auto& ring : j.rings) {
            if (ring.vertices.size() > kMaxRingVertices) return invalid("junction ring size");
            ring_vertices += ring.vertices.size();
            if (ring_vertices > kMaxTileRingVertices) return invalid("junction ring budget");
            if (declined ? !ring.vertices.empty() : ring.vertices.size() < 3) return invalid("junction ring count");
            for (const auto& p : ring.vertices)
                if (p.x < owner->position.x - kMaxRingReachMm || p.x > owner->position.x + kMaxRingReachMm ||
                    p.y < owner->position.y - kMaxRingReachMm || p.y > owner->position.y + kMaxRingReachMm) return invalid("junction ring reach");
            if (!declined && !junction_ring_valid(ring)) return invalid("junction ring not simple or not counter-clockwise");
        }
        if (!declined && !junction_rings_nested(j)) return invalid("junction rings not nested");
        // A raised junction: its height and the node that sets it.
        if (j.flags & static_cast<std::uint16_t>(JunctionFlags::RaisedTable)) {
            if (declined || !std::isfinite(j.raised_height_m) || !(j.raised_height_m > 0) || j.raised_height_m > kMaxRaisedHeightM || !holds(j.raised_node))
                return invalid("junction raised table");
        } else if (!zero(j.raised_height_m) || j.raised_node != 0) return invalid("junction raised table");
    }
    return {};
}

// Design 9.2, furniture rules (section 6 of encoding 7).
Result<void> validate_furniture(const RoadGeomTile& tile) {
    if (tile.furniture.size() > kMaxFurniture) return invalid("furniture budget");
    std::vector<std::pair<double, double>> groups;
    const RoadFurniture* previous = nullptr;
    for (const auto& f : tile.furniture) {
        if (static_cast<unsigned>(f.kind) > 3 || static_cast<std::size_t>(f.decline) >= kFurnitureDeclineCount ||
            static_cast<unsigned>(f.pattern) >= kCrossingPatternCount || static_cast<unsigned>(f.travel) > 1) return invalid("furniture enum");
        if ((f.flags & ~static_cast<unsigned>(kFurnitureDefinedMask)) != 0 || (f.flags & kFurnitureInsideJunction) != 0) return invalid("furniture flags");
        if (f.entry_index >= tile.entries.size() || f.junction_index > tile.junctions.size()) return invalid("furniture index");
        if (!finite(f.station_m) || !mm_exact(f.t_from) || !mm_exact(f.t_to) || !mm_exact(f.length_m) ||
            std::fabs(f.t_from) > kMaxFurnitureT || std::fabs(f.t_to) > kMaxFurnitureT || std::signbit(f.length_m) || f.length_m > kMaxFurnitureLength)
            return invalid("furniture value");
        if (previous) {
            const auto key = [](const RoadFurniture& r) {
                return std::make_tuple(r.junction_index, r.entry_index, static_cast<int>(r.kind), r.station_m, r.node_id, static_cast<int>(r.travel), r.t_from, r.flags);
            };
            if (!(key(*previous) < key(f))) return invalid("furniture order");
        }
        previous = &f;
        const RoadJunction* junction = f.junction_index ? &tile.junctions[f.junction_index - 1] : nullptr;
        if (junction ? !std::binary_search(junction->nodes.begin(), junction->nodes.end(), f.node_id) : !graph_node(tile.source.graph, f.node_id))
            return invalid("furniture node");
        if (f.decline != FurnitureDecline::None) {
            if (!zero(f.t_from) || !zero(f.t_to) || !zero(f.length_m) || f.pattern != CrossingPattern::None || f.flags != 0) return invalid("declined furniture extent");
            const bool junction_kind = f.decline == FurnitureDecline::JunctionDeclined || f.decline == FurnitureDecline::Ambiguous;
            if (junction_kind && (!junction || f.kind == FurnitureKind::Crossing)) return invalid("declined furniture junction");
            if (f.decline == FurnitureDecline::JunctionDeclined && junction->decline == JunctionDecline::None) return invalid("declined furniture junction");
            continue;
        }
        // Placed: on a profiled entry with a cross-section, near its reference line.
        const auto& e = tile.entries[f.entry_index];
        if (!e.profile || !e.cross_section) return invalid("furniture entry");
        const double length = e.profile->reference.length_m;
        if (f.station_m < -f.length_m || f.station_m > length + f.length_m) return invalid("furniture station");
        if (junction) {
            if (junction->decline != JunctionDecline::None || f.node_id != junction->nodes.front() || f.kind == FurnitureKind::Crossing) return invalid("junction furniture");
            const bool approach = std::any_of(junction->approaches.begin(), junction->approaches.end(), [&](const JunctionApproach& a) {
                return a.entry_index == f.entry_index && a.status == JunctionApproachStatus::Used; });
            if (!approach) return invalid("junction furniture entry");
        }
        switch (f.kind) {
        case FurnitureKind::Crossing:
            if (f.travel != LaneDirection::Forward || !(f.t_from < f.t_to) || !(f.length_m > 0) ||
                (f.flags & ~static_cast<unsigned>(kFurnitureSignalised | kFurnitureLoweredKerb | kFurnitureSpansJoin)) != 0) return invalid("crossing record");
            break;
        case FurnitureKind::StopLine: case FurnitureKind::GiveWayLine: {
            if (!(f.t_from < f.t_to) || !(f.length_m > 0) || f.pattern != CrossingPattern::None || f.flags != 0) return invalid("line record");
            // The acceptance rule of 7.2: the line spans exactly one lane group of its travel at its own station.
            furniture_lane_groups(*e.cross_section, length, f.station_m, f.travel, groups);
            const auto llr = [](double v) { return static_cast<double>(std::llround(v * 1000.0)) / 1000.0; };
            const bool spans = std::any_of(groups.begin(), groups.end(), [&](const auto& g) { return llr(g.first) == f.t_from && llr(g.second) == f.t_to; });
            if (!spans) return invalid("line lane span");
            break;
        }
        case FurnitureKind::Signal:
            if (f.t_from != f.t_to || !zero(f.length_m) || f.pattern != CrossingPattern::None || (f.flags & ~static_cast<unsigned>(kFurnitureLeftPost)) != 0)
                return invalid("signal record");
            break;
        }
    }
    return {};
}
// Encoding 9, section 8 (D56): underpass decks on ground entries, inside the stretch, sorted and disjoint per entry.
Result<void> validate_underpasses(const RoadGeomTile& tile) {
    if (tile.underpasses.size() > kMaxUnderpasses) return invalid("underpass budget");
    const RoadUnderpass* prev = nullptr;
    for (const auto& u : tile.underpasses) {
        if (u.entry_index >= tile.entries.size()) return invalid("underpass entry");
        const auto& e = tile.entries[u.entry_index];
        if (!e.profile || e.profile->attributes.bridge || e.profile->attributes.tunnel) return invalid("underpass on a non-ground entry");
        if (!mm_exact(u.start_m) || !mm_exact(u.end_m) || !(u.start_m > 0) || !(u.start_m < u.end_m) || !(u.end_m < e.profile->reference.length_m))
            return invalid("underpass extent");
        if (u.flags != 0) return invalid("underpass flags");
        if (u.depth_mm <= 0 || u.depth_mm > kMaxUnderpassDepthMm) return invalid("underpass depth");
        if (u.lower_ways.empty() || u.lower_ways.size() > kMaxUnderpassLowerWays) return invalid("underpass lower ways");
        for (std::size_t i = 0; i < u.lower_ways.size(); ++i)
            if (u.lower_ways[i] <= 0 || (i > 0 && u.lower_ways[i] <= u.lower_ways[i - 1])) return invalid("underpass lower way order");
        if (prev && (prev->entry_index > u.entry_index || (prev->entry_index == u.entry_index && !(prev->end_m < u.start_m))))
            return invalid("underpass order");
        prev = &u;
    }
    return {};
}
} // namespace
bool road_tile_contains(const TileKey& key, const geom::PointMm& p) noexcept {
    const auto size = static_cast<std::int64_t>(key.size_m()) * 1000;
    return floor_div(p.x, size) == key.x && floor_div(p.y, size) == key.y;
}
Result<RoadGeomTile> build_road_geom_tile(const RoadGraphTile& graph, const RoadDemSampler& dem,
    const RoadProfileLimits& limits, const RoadGraph* context, const RoadRules& rules,
    const RoadCompletionOptions& completion, RoadCompletionStats* completion_stats, const RoadUnderpassOptions& underpass) {
    const auto valid = validate_road_graph_tile(graph); if (!valid) return valid.error();
    const auto contains_tile_ways = [&](const RoadGraph& g) -> Result<void> {
        const auto contained = validate_road_graph(g); if (!contained) return contained.error();
        for (const auto& way : graph.graph.ways) {
            const auto it = std::lower_bound(g.ways.begin(), g.ways.end(), way.osm_id, [](const auto& w, auto id) { return w.osm_id < id; });
            if (it == g.ways.end() || !(*it == way)) return invalid("context lacks a tile way");
        }
        return {};
    };
    if (context) {
        // The context must contain every tile way unchanged, with all its nodes.
        const auto contained = validate_road_graph(*context); if (!contained) return contained.error();
        for (const auto& way : graph.graph.ways) {
            const auto it = std::lower_bound(context->ways.begin(), context->ways.end(), way.osm_id,
                [](const auto& w, auto id) { return w.osm_id < id; });
            if (it == context->ways.end() || !(*it == way)) return invalid("context lacks a tile way");
        }
    }
    const auto builder = RoadReferenceBuilder::make(context ? *context : graph.graph); if (!builder) return builder.error();
    // Underpass decks (D53): the finder masks every ground profile below (strict, retry, ground completion) through
    // the limits, and names the decks served afterwards.
    std::optional<RoadUnderpassFinder> finder;
    RoadProfileLimits profile_limits = limits;
    // G2M-024: the chord bound of the reference fits is de.json's reference block (zero = off), whatever the caller set.
    profile_limits.reference.chord_half_width_factor = static_cast<double>(rules.reference.chord_half_width_percent) / 100.0;
    profile_limits.reference.chord_margin_m = static_cast<double>(rules.reference.chord_margin_mm) / 1000.0;
    if (underpass.enabled && rules.underpass.enabled) {
        const RoadGraph& crossings = underpass.crossings ? *underpass.crossings : context ? *context : graph.graph;
        if (underpass.crossings) { const auto ok = contains_tile_ways(crossings); if (!ok) return invalid("crossing graph lacks a tile way"); }
        finder.emplace(crossings, dem, graph.key.zone, rules.underpass, context ? context : &graph.graph);   // D63: joins read the stretch context
        profile_limits.underpass = &*finder;
    }
    RoadGeomTile result; result.source = graph;
    if (underpass.enabled) result.encoding = kRoadGeomEncoding;
    // Strict pass: one accepted profile or one explicit decline per stretch.
    for (const auto& way : graph.graph.ways) {
        const auto stretches = builder->stretches(way.osm_id); if (!stretches) return stretches.error();
        for (const auto stretch : *stretches) {
            if (result.entries.size() == kMaxEntries) return make_error(ErrorCode::LimitExceeded, "roads.geom: entry budget");
            RoadGeomEntry entry; entry.way_id = way.osm_id; entry.stretch = stretch;
            auto profile = build_road_profile(*builder, dem, graph.key.zone, way.osm_id, stretch, profile_limits);
            if (profile) entry.profile = std::move(*profile); else entry.decline = profile.error();
            result.entries.push_back(std::move(entry));
        }
    }
    // Completion tier (design 6.6): off unless asked for; adds flagged profiles, never touches an accepted one.
    if (completion.enabled) {
        const auto done = complete_road_profiles(result.entries, result.origins, *builder, dem, graph.key.zone, profile_limits, completion, completion_stats);
        if (!done) return done.error();
    }
    // Cross-sections never read the DEM; they are stored only where a profile (hence a reference) exists.
    for (std::size_t index = 0; index < result.entries.size(); ++index) {
        auto& entry = result.entries[index];
        if (entry.profile) {
            auto xs = resolve_cross_section(*builder, entry.way_id, entry.stretch, entry.profile->reference.length_m, rules);
            if (!xs) return xs.error();
            entry.cross_section = std::move(xs->xs); entry.cross_section_decline = xs->decline;
        }
        const auto features = resolve_profile_features(graph.graph, entry, index, rules);
        if (!features) return features.error();
        result.features.insert(result.features.end(), features->begin(), features->end());
    }
    // Encoding 7 (design 5.1): the junctions the tile can resolve, kept where the owner node lies in this tile; then the
    // furniture of the tile's entries and of the owned junctions; then one origin per entry.
    auto junctions = build_road_junctions(result, *builder, rules); if (!junctions) return junctions.error();
    for (auto& j : *junctions) {
        const auto* owner = graph_node(graph.graph, j.nodes.front());
        if (owner && road_tile_contains(graph.key, owner->position)) result.junctions.push_back(std::move(j));
    }
    auto furniture = build_road_furniture(result, *builder, result.junctions, rules); if (!furniture) return furniture.error();
    result.furniture = std::move(*furniture);
    if (result.origins.empty()) result.origins.assign(result.entries.size(), RoadProfileOrigin{});
    // Encoding 9 (D56): the decks the masked ground profiles were fitted over, recomputed on the stored reference (the
    // very reference the profile pass masked on, so the same spans).
    if (finder) for (std::size_t index = 0; index < result.entries.size(); ++index) {
        const auto& entry = result.entries[index];
        if (!entry.profile || entry.profile->attributes.bridge || entry.profile->attributes.tunnel) continue;
        for (auto& fit : finder->spans(entry.way_id, entry.stretch, entry.profile->reference, underpass.stats)) {
            if (fit.covered) continue;   // D61: masked, no deck
            if (result.underpasses.size() == kMaxUnderpasses) return make_error(ErrorCode::LimitExceeded, "roads.geom: underpass budget");
            RoadUnderpass u; u.entry_index = index; u.start_m = fit.start_m; u.end_m = fit.end_m; u.depth_mm = fit.depth_mm;
            u.lower_ways = std::move(fit.lower_ways);
            if (u.lower_ways.size() > kMaxUnderpassLowerWays) u.lower_ways.resize(kMaxUnderpassLowerWays);
            result.underpasses.push_back(std::move(u));
        }
    }
    const auto check = validate_road_geom_tile(result); if (!check) return check.error();
    return result;
}
Result<void> validate_road_geom_tile(const RoadGeomTile& tile) {
    const auto graph_valid = validate_road_graph_tile(tile.source); if (!graph_valid) return graph_valid.error();
    if (tile.encoding != kRoadGeomEncoding && tile.encoding != kRoadGeomEncodingUnderpass && tile.encoding != kRoadGeomEncodingTunnels &&
        tile.encoding != kRoadGeomEncodingJunctions && tile.encoding != kRoadGeomEncodingPrevious && tile.encoding != kRoadGeomEncodingLegacy) return invalid("encoding");
    // `current`: encodings 10 to 7 (junctions, furniture, provenance); provenance 4 (TunnelFloor) only in 10 to 8;
    // underpass decks only in 10 and 9.
    const bool legacy = tile.encoding == kRoadGeomEncodingLegacy, with_decks = has_underpass_section(tile.encoding),
        current = with_decks || tile.encoding == kRoadGeomEncodingTunnels || tile.encoding == kRoadGeomEncodingJunctions;
    const unsigned max_provenance = with_decks || tile.encoding == kRoadGeomEncodingTunnels ? 4 : 3;
    if (!with_decks && !tile.underpasses.empty()) return invalid("underpass decks before encoding 9");
    if (tile.entries.size() > kMaxEntries) return invalid("entry budget");
    if (legacy && !tile.features.empty()) return invalid("legacy tile carries features");
    if (!current && (!tile.junctions.empty() || !tile.furniture.empty())) return invalid("encoding 6/5 tile carries junctions or furniture");
    // Encoding 7: one provenance record per entry (9.2). Encoding 6 may carry Strict-only origins (never stored).
    if (current ? tile.origins.size() != tile.entries.size()
                : (!tile.origins.empty() && (legacy || tile.origins.size() != tile.entries.size()))) return invalid("origins count");
    const auto builder = RoadReferenceBuilder::make(tile.source.graph); if (!builder) return builder.error();
    std::size_t index = 0, pieces = 0;
    for (const auto& way : tile.source.graph.ways) {
        // The tile cannot know world degrees, so stretch extents are checked
        // structurally: the way's entries partition [0, last ref], and no node
        // inside a stretch may be a junction even in the tile's own (partial) graph.
        std::uint32_t next = 0; bool any = false;
        while (index < tile.entries.size() && tile.entries[index].way_id == way.osm_id) {
            const auto& entry = tile.entries[index++]; const auto stretch = entry.stretch; any = true;
            if (stretch.start_ref != next || stretch.end_ref <= stretch.start_ref || stretch.end_ref >= way.node_ids.size() ||
                entry.profile.has_value() == entry.decline.has_value()) return invalid("stretch order or outcome");
            for (auto ref = stretch.start_ref + 1; ref < stretch.end_ref; ++ref)
                if (builder->degree(way.node_ids[ref]) != 2) return invalid("junction inside a stretch");
            next = stretch.end_ref;
            RoadProfileOrigin origin;
            if (!tile.origins.empty()) {
                origin = tile.origins[index - 1];
                if (static_cast<unsigned>(origin.provenance) > max_provenance) return invalid("provenance");
                if (origin.provenance == ProfileProvenance::Strict ? origin.strict_decline_code != 0
                    : (!entry.profile || origin.strict_decline_code < 1 || origin.strict_decline_code > static_cast<unsigned>(ErrorCode::ReadOnly)))
                    return invalid("provenance record");
            }
            if (legacy ? (entry.cross_section || entry.cross_section_decline)
                       : (entry.profile ? entry.cross_section.has_value() == entry.cross_section_decline.has_value()
                                        : (entry.cross_section || entry.cross_section_decline)))
                return invalid("cross-section outcome");
            if (entry.decline) {
                const auto code = static_cast<unsigned>(entry.decline->code);
                if (code < 1 || code > static_cast<unsigned>(ErrorCode::ReadOnly) || entry.decline->message.empty() || entry.decline->message.size() > 512)
                    return invalid("decline");
                continue;
            }
            const auto& p = *entry.profile;
            const auto attrs = resolve_road_attributes(way); if (!attrs) return attrs.error();
            const auto& ref = p.reference;
            // A tunnel carries a profile only as a TunnelFloor completion, and a TunnelFloor profile only on a tunnel (D47).
            if (p.attributes != *attrs || p.attributes.tunnel != (origin.provenance == ProfileProvenance::TunnelFloor) || p.stretch.start_ref != stretch.start_ref || p.stretch.end_ref != stretch.end_ref ||
                ref.way_id != way.osm_id || !finite(ref.length_m) || ref.length_m < 0.5 || ref.length_m > 100000 ||
                !finite(ref.requested_min_radius_m) || ref.requested_min_radius_m < 1 || !finite(ref.fitted_min_radius_m) || ref.fitted_min_radius_m < 1 ||
                (ref.curvature_exception ? ref.fitted_min_radius_m >= ref.requested_min_radius_m : ref.fitted_min_radius_m != ref.requested_min_radius_m) ||
                (p.start_anchor_adjusted && !p.start_anchor_inferred) || (p.end_anchor_adjusted && !p.end_anchor_inferred) ||
                ref.coincident_pairs > way.node_ids.size() || ref.segments.empty() || ref.anchors.size() > way.node_ids.size() ||
                p.vertical.segments.empty() || p.vertical.segments.size() > 2047 || p.vertical.iterations < 0 || p.vertical.iterations > 16384)
                return invalid("profile metadata");
            const auto total_pieces = ref.segments.size() + ref.anchors.size() + p.vertical.segments.size();
            if (total_pieces > kMaxPieces - pieces) return invalid("piece budget"); pieces += total_pieces;
            double station = 0;
            std::optional<geom::CurvePose> previous;
            for (const auto& s : ref.segments) {
                const auto& c = s.curve;
                if (!finite(s.station_m) || std::fabs(s.station_m - station) > 1e-7 || s.original_pair_index < stretch.start_ref ||
                    s.original_pair_index >= stretch.end_ref || s.piece_index > 1 || s.break_before || !finite(c.x0) || !finite(c.y0) ||
                    !finite(c.heading0) || !finite(c.length) || c.length <= 0 || !finite(c.curvature0) || !finite(c.curvature_rate) ||
                    std::max(std::fabs(c.curvature0), std::fabs(c.curvature0 + c.curvature_rate * c.length)) > 1 / ref.fitted_min_radius_m + 1e-7)
                    return invalid("reference piece");
                const auto a = c.at(0), b = c.at(c.length); if (!a || !b) return invalid("curve evaluation");
                if (previous && (std::fabs(previous->x - a->x) > 1e-5 || std::fabs(previous->y - a->y) > 1e-5 ||
                    std::fabs(det::sin(previous->heading - a->heading)) > 1e-5 || det::cos(previous->heading - a->heading) < 0)) return invalid("reference join");
                previous = *b; station += c.length;
            }
            if (std::fabs(station - ref.length_m) > 1e-7) return invalid("reference extent");
            // Per-stretch construction has exactly two anchors. Enforce that
            // directly, avoiding quadratic membership scans on hostile input.
            if (ref.anchors.size() != 2 || ref.anchors[0].node_id != way.node_ids[stretch.start_ref] ||
                ref.anchors[1].node_id != way.node_ids[stretch.end_ref] || ref.anchors[0].station_m != 0 ||
                ref.anchors[1].station_m != ref.length_m) return invalid("endpoint anchors");
            const auto position = [&](std::int64_t id) {
                return std::lower_bound(tile.source.graph.nodes.begin(), tile.source.graph.nodes.end(), id,
                    [](const auto& node, auto value) { return node.osm_id < value; })->position;
            };
            const auto a = position(ref.anchors[0].node_id), b = position(ref.anchors[1].node_id);
            const auto& first = ref.segments.front().curve;
            if (std::fabs(first.x0 - a.x / 1000.0) > 1e-5 || std::fabs(first.y0 - a.y / 1000.0) > 1e-5 ||
                !previous || std::fabs(previous->x - b.x / 1000.0) > 1e-5 || std::fabs(previous->y - b.y / 1000.0) > 1e-5)
                return invalid("reference endpoints");
            station = 0; std::optional<VerticalCubic> last;
            // A completed profile is certified at its own relaxed limits (design 6.6): bridge max(12 %, class) and min(250, class),
            // ground max(12 %, class) and min(500, class).
            double grade = attrs->max_grade_per_mille / 1000.0, radius = attrs->min_vertical_radius_m;
            if (origin.provenance == ProfileProvenance::BridgeCompleted) { grade = std::max(0.12, grade); radius = std::min(250.0, radius); }
            if (origin.provenance == ProfileProvenance::GroundCompleted) { grade = std::max(0.12, grade); radius = std::min(500.0, radius); }
            if (origin.provenance == ProfileProvenance::TunnelFloor) { grade = kRoadCompletionMaxTunnelGrade; radius = kRoadCompletionMinTunnelRadiusM; }
            const double curvature = 1.0 / radius;
            for (const auto& s : p.vertical.segments) {
                if (!finite(s.station_m) || std::fabs(s.station_m - station) > 1e-7 || !finite(s.length_m) || s.length_m < 0.5 ||
                    !finite(s.height0_m) || !finite(s.height1_m) || !finite(s.grade0) || !finite(s.grade1)) return invalid("vertical piece");
                const double delta = (s.height1_m - s.height0_m) / s.length_m;
                if (std::max({std::fabs(s.grade0), std::fabs(s.grade1), std::fabs(3 * delta - s.grade0 - s.grade1)}) > grade + 1e-7 ||
                    std::fabs((6 * delta - 4 * s.grade0 - 2 * s.grade1) / s.length_m) > curvature + 1e-7 ||
                    std::fabs((-6 * delta + 2 * s.grade0 + 4 * s.grade1) / s.length_m) > curvature + 1e-7) return invalid("vertical bounds");
                if (last && (std::fabs(last->height1_m - s.height0_m) > 1e-7 || std::fabs(last->grade1 - s.grade0) > 1e-7)) return invalid("vertical join");
                last = s; station += s.length_m;
            }
            if (std::fabs(station - ref.length_m) > 1e-7) return invalid("vertical extent");
            if (entry.cross_section) { const auto xs = validate_cross_section(*entry.cross_section, ref.length_m); if (!xs) return xs.error(); }
        }
        if (!any || next + 1 != way.node_ids.size()) return invalid("missing stretch");
    }
    if (index != tile.entries.size()) return invalid("extra stretches");
    const auto features = validate_features(tile); if (!features) return features.error();
    const auto junctions = validate_junctions(tile); if (!junctions) return junctions.error();
    const auto furniture = validate_furniture(tile); if (!furniture) return furniture;
    return validate_underpasses(tile);
}
Result<TileBody> encode_road_geom(const RoadGeomTile& tile) {
    if (tile.encoding != kRoadGeomEncoding && tile.encoding != kRoadGeomEncodingUnderpass && tile.encoding != kRoadGeomEncodingTunnels &&
        tile.encoding != kRoadGeomEncodingJunctions && tile.encoding != kRoadGeomEncodingPrevious)
        return make_error(ErrorCode::InvalidArgument, "roads.geom: only encodings 10, 9, 8, 7 and 6 are written");
    if (tile.encoding == kRoadGeomEncodingJunctions) for (const auto& o : tile.origins) if (o.provenance == ProfileProvenance::TunnelFloor)
        return make_error(ErrorCode::InvalidArgument, "roads.geom: encoding 7 cannot carry a tunnel floor");
    if (tile.encoding == kRoadGeomEncodingPrevious) for (const auto& o : tile.origins) if (o.provenance != ProfileProvenance::Strict)
        return make_error(ErrorCode::InvalidArgument, "roads.geom: encoding 6 cannot carry a completed profile");
    const auto valid = validate_road_geom_tile(tile); if (!valid) return valid.error();
    // Encodings 6 to 10 embed the encoding-5 graph section (with node tags), byte for byte.
    const auto graph = encode_road_graph(tile.source, kRoadGraphEncodingTags); if (!graph) return graph.error();
    ByteWriter w; w.varint(tile.entries.size());
    for (const auto& entry : tile.entries) {
        w.svarint(entry.way_id); w.u32(entry.stretch.start_ref); w.u32(entry.stretch.end_ref);
        w.u16(entry.decline ? static_cast<std::uint16_t>(entry.decline->code) : 0);
        if (entry.decline) { w.varint(entry.decline->message.size()); w.bytes(entry.decline->message.data(), entry.decline->message.size()); continue; }
        const auto& p = *entry.profile; const auto& ref = p.reference;
        w.u8(static_cast<std::uint8_t>((p.start_anchor_inferred ? 1 : 0) | (p.end_anchor_inferred ? 2 : 0) |
            (p.start_anchor_adjusted ? 4 : 0) | (p.end_anchor_adjusted ? 8 : 0) | (ref.curvature_exception ? 16 : 0)));
        number(w, ref.length_m); number(w, ref.requested_min_radius_m); number(w, ref.fitted_min_radius_m); w.varint(ref.coincident_pairs);
        w.varint(ref.segments.size());
        for (const auto& s : ref.segments) {
            w.u32(s.original_pair_index); w.u8(static_cast<std::uint8_t>(s.piece_index)); number(w, s.station_m);
            for (double d : {s.curve.x0, s.curve.y0, s.curve.heading0, s.curve.length, s.curve.curvature0, s.curve.curvature_rate}) number(w, d);
        }
        w.varint(ref.anchors.size()); for (const auto& a : ref.anchors) { w.svarint(a.node_id); number(w, a.station_m); }
        w.u32(static_cast<std::uint32_t>(p.vertical.iterations)); w.varint(p.vertical.segments.size());
        for (const auto& s : p.vertical.segments) for (double d : {s.station_m, s.length_m, s.height0_m, s.height1_m, s.grade0, s.grade1}) number(w, d);
        if (w.size() > kMaxSectionRawSize) return make_error(ErrorCode::LimitExceeded, "roads.geom: section budget");
    }
    ByteWriter xs; xs.varint(tile.entries.size());
    for (const auto& entry : tile.entries) {
        if (entry.cross_section_decline) {
            xs.u8(2); xs.u16(static_cast<std::uint16_t>(entry.cross_section_decline->code)); xs.svarint(entry.cross_section_decline->detail); continue;
        }
        if (!entry.cross_section) { xs.u8(0); continue; }
        const auto& c = *entry.cross_section;
        xs.u8(1); xs.u16(c.flags); xs.u32(c.field_declines);
        xs.u8(static_cast<std::uint8_t>(static_cast<unsigned>(c.count_provenance) | (static_cast<unsigned>(c.placement_provenance) << 2)));
        xs.varint(c.bands.size());
        for (const auto& b : c.bands) {
            xs.u8(static_cast<std::uint8_t>(b.kind)); xs.u8(static_cast<std::uint8_t>(b.direction)); xs.u8(static_cast<std::uint8_t>(b.change));
            xs.u8(static_cast<std::uint8_t>((b.counted ? 1 : 0) | (b.paved ? 2 : 0) | (b.marked_right ? 4 : 0)));
            xs.varint(static_cast<std::uint32_t>(b.width_mm)); xs.u16(b.turn);
            xs.u16(static_cast<std::uint16_t>(static_cast<unsigned>(b.provenance.kind) | (static_cast<unsigned>(b.provenance.direction) << 2) |
                (static_cast<unsigned>(b.provenance.width) << 4) | (static_cast<unsigned>(b.provenance.turn) << 6) | (static_cast<unsigned>(b.provenance.change) << 8)));
            xs.varint(static_cast<std::uint32_t>(b.start.length_mm)); xs.varint(static_cast<std::uint32_t>(b.start.from_width_mm));
            xs.varint(static_cast<std::uint32_t>(b.end.length_mm)); xs.varint(static_cast<std::uint32_t>(b.end.from_width_mm));
        }
        xs.u8(c.anchor.band); xs.u8(static_cast<std::uint8_t>(c.anchor.edge)); xs.u8(static_cast<std::uint8_t>(c.anchor.source));
        xs.u8(static_cast<std::uint8_t>(c.offset.mode));
        xs.svarint(c.offset.start_mm); xs.varint(static_cast<std::uint32_t>(c.offset.start_length_mm));
        xs.svarint(c.offset.end_mm); xs.varint(static_cast<std::uint32_t>(c.offset.end_length_mm));
        for (const auto* j : {&c.start_join, &c.end_join}) { xs.u8(static_cast<std::uint8_t>(j->kind)); xs.svarint(j->neighbour_way_id); xs.svarint(j->raw_delta_mm); }
        if (xs.size() > kMaxSectionRawSize) return make_error(ErrorCode::LimitExceeded, "roads.geom: cross-section budget");
    }
    ByteWriter fw; fw.varint(tile.features.size());
    for (const auto& f : tile.features) {
        fw.varint(f.entry_index); fw.svarint(f.node_id); number(fw, f.station_m);
        fw.u8(static_cast<std::uint8_t>(f.kind)); fw.u8(f.carve ? 1 : 0);
        fw.varint(static_cast<std::uint32_t>(f.length_mm)); fw.varint(static_cast<std::uint32_t>(f.height_mm)); fw.varint(static_cast<std::uint32_t>(f.width_mm));
        fw.u8(static_cast<std::uint8_t>(f.provenance));
    }
    if (fw.size() > kMaxSectionRawSize) return make_error(ErrorCode::LimitExceeded, "roads.geom: feature budget");
    TileBody body{tile.encoding, 1, {graph->sections[0], {2, SectionCodec::Zstd, w.take()}, {3, SectionCodec::Zstd, xs.take()}, {4, SectionCodec::Zstd, fw.take()}}};
    if (tile.encoding == kRoadGeomEncodingPrevious) return body;
    // Section 5: junctions (design 9.1; raised_node follows raised_height_m, decision D34).
    ByteWriter jw; jw.varint(tile.junctions.size());
    for (const auto& j : tile.junctions) {
        jw.svarint(j.nodes.front()); jw.u16(j.flags); jw.u16(static_cast<std::uint16_t>(j.decline));
        jw.varint(j.nodes.size()); for (const auto n : j.nodes) jw.svarint(n);
        jw.varint(j.approaches.size());
        for (const auto& a : j.approaches) { jw.varint(a.entry_index); jw.u8(a.at_end ? 1 : 0); jw.u8(static_cast<std::uint8_t>(a.status)); number(jw, a.trim_m); }
        jw.varint(j.absorbed.size()); for (const auto e : j.absorbed) jw.varint(e);
        for (const auto& ring : j.rings) {
            jw.varint(ring.vertices.size());
            geom::PointMm last{};
            for (std::size_t i = 0; i < ring.vertices.size(); ++i) {
                const auto p = ring.vertices[i];
                if (i == 0) { jw.svarint(p.x); jw.svarint(p.y); } else { jw.svarint(p.x - last.x); jw.svarint(p.y - last.y); }
                last = p;
            }
        }
        number(jw, j.raised_height_m); jw.svarint(j.raised_node);
        if (jw.size() > kMaxSectionRawSize) return make_error(ErrorCode::LimitExceeded, "roads.geom: junction budget");
    }
    // Section 6: furniture; t and length in integer mm (the records are mm-rounded, validated above).
    ByteWriter uw; uw.varint(tile.furniture.size());
    for (const auto& f : tile.furniture) {
        uw.u8(static_cast<std::uint8_t>(f.kind)); uw.u16(static_cast<std::uint16_t>(f.decline));
        uw.svarint(f.node_id); uw.varint(f.entry_index); uw.varint(f.junction_index);
        number(uw, f.station_m); uw.u8(static_cast<std::uint8_t>(f.travel));
        uw.svarint(std::llround(f.t_from * 1000.0)); uw.svarint(std::llround(f.t_to * 1000.0));
        uw.varint(static_cast<std::uint64_t>(std::llround(f.length_m * 1000.0)));
        uw.u8(static_cast<std::uint8_t>(f.pattern)); uw.u8(f.flags);
    }
    if (uw.size() > kMaxSectionRawSize) return make_error(ErrorCode::LimitExceeded, "roads.geom: furniture budget");
    // Section 7: provenance, one record per entry.
    ByteWriter pw; pw.varint(tile.origins.size());
    for (const auto& o : tile.origins) { pw.u8(static_cast<std::uint8_t>(o.provenance)); pw.u16(o.strict_decline_code); }
    body.sections.push_back({5, SectionCodec::Zstd, jw.take()});
    body.sections.push_back({6, SectionCodec::Zstd, uw.take()});
    body.sections.push_back({7, SectionCodec::Zstd, pw.take()});
    if (!has_underpass_section(tile.encoding)) return body;
    // Section 8 (encodings 9 and 10, D56): underpass decks; stations in integer mm, the lower ways as ascending deltas.
    ByteWriter dw; dw.varint(tile.underpasses.size());
    for (const auto& u : tile.underpasses) {
        const auto start = static_cast<std::uint64_t>(std::llround(u.start_m * 1000.0)), end = static_cast<std::uint64_t>(std::llround(u.end_m * 1000.0));
        dw.varint(u.entry_index); dw.varint(start); dw.varint(end - start);
        dw.varint(static_cast<std::uint64_t>(u.depth_mm)); dw.u16(u.flags);
        dw.varint(u.lower_ways.size());
        std::int64_t last = 0;
        for (const auto id : u.lower_ways) { dw.varint(static_cast<std::uint64_t>(id - last)); last = id; }
    }
    if (dw.size() > kMaxSectionRawSize) return make_error(ErrorCode::LimitExceeded, "roads.geom: underpass budget");
    body.sections.push_back({8, SectionCodec::Zstd, dw.take()});
    return body;
}
Result<RoadGeomTile> decode_road_geom(const TileBody& body) {
    if ((body.encoding != kRoadGeomEncoding && body.encoding != kRoadGeomEncodingUnderpass && body.encoding != kRoadGeomEncodingTunnels &&
         body.encoding != kRoadGeomEncodingJunctions && body.encoding != kRoadGeomEncodingPrevious && body.encoding != kRoadGeomEncodingLegacy) ||
        body.encoding_version != 1)
        return make_error(ErrorCode::Unsupported, "roads.geom: encoding/version");
    const bool legacy = body.encoding == kRoadGeomEncodingLegacy, with_decks = has_underpass_section(body.encoding),
        current = with_decks || body.encoding == kRoadGeomEncodingTunnels || body.encoding == kRoadGeomEncodingJunctions;
    const unsigned max_provenance = with_decks || body.encoding == kRoadGeomEncodingTunnels ? 4 : 3;
    const std::size_t section_count = legacy ? 2 : with_decks ? 8 : current ? 7 : 4;
    if (body.sections.size() != section_count) return invalid("sections");
    for (std::size_t i = 0; i < section_count; ++i) if (body.sections[i].type != i + 1) return invalid("sections");
    for (const auto& section : body.sections) if (section.data.size() > kMaxSectionRawSize) return invalid("section size");
    auto graph = decode_road_graph(TileBody{legacy ? kRoadGraphEncodingLegacy : kRoadGraphEncodingTags, 1, {body.sections[0]}}); if (!graph) return graph.error();
    RoadGeomTile tile; tile.source = std::move(*graph); tile.encoding = body.encoding; ByteReader r(body.sections[1].data);
    const auto count = r.varint(); if (!r.ok() || count > kMaxEntries || count > r.remaining() / 11) return invalid("entry count");
    std::size_t pieces = 0;
    for (std::uint64_t i = 0; i < count; ++i) {
        RoadGeomEntry e; e.way_id = r.svarint(); e.stretch = {r.u32(), r.u32()}; const auto code = r.u16();
        if (code) {
            const auto length = r.varint(); if (!r.ok() || length > 512 || length > r.remaining()) return invalid("decline length");
            e.decline = Error{static_cast<ErrorCode>(code), std::string(r.string(static_cast<std::size_t>(length)))};
        } else {
            RoadProfile p; p.stretch = e.stretch; auto& ref = p.reference; ref.way_id = e.way_id;
            const auto flags = r.u8(); if (flags > 31) return invalid("flags");
            p.start_anchor_inferred = flags & 1; p.end_anchor_inferred = flags & 2; p.start_anchor_adjusted = flags & 4; p.end_anchor_adjusted = flags & 8; ref.curvature_exception = flags & 16;
            ref.length_m = number(r); ref.requested_min_radius_m = number(r); ref.fitted_min_radius_m = number(r);
            const auto coincident = r.varint(); if (coincident > kMaxPieces) return invalid("coincident budget"); ref.coincident_pairs = static_cast<std::size_t>(coincident);
            const auto curves = r.varint(); if (!r.ok() || curves > kMaxPieces - pieces || curves > r.remaining() / 61) return invalid("curve count"); pieces += static_cast<std::size_t>(curves);
            for (std::uint64_t j = 0; j < curves; ++j) {
                RoadReferenceSegment s; s.original_pair_index = r.u32(); s.piece_index = r.u8(); s.station_m = number(r);
                s.curve = {number(r), number(r), number(r), number(r), number(r), number(r)}; ref.segments.push_back(s);
            }
            const auto anchors = r.varint(); if (!r.ok() || anchors > kMaxPieces - pieces || anchors > r.remaining() / 9) return invalid("anchor count"); pieces += static_cast<std::size_t>(anchors);
            for (std::uint64_t j = 0; j < anchors; ++j) ref.anchors.push_back({r.svarint(), number(r)});
            const auto iterations = r.u32(); if (iterations > 16384) return invalid("iteration count"); p.vertical.iterations = static_cast<int>(iterations);
            const auto cubics = r.varint(); if (!r.ok() || cubics > 2047 || cubics > kMaxPieces - pieces || cubics > r.remaining() / 48) return invalid("cubic count"); pieces += static_cast<std::size_t>(cubics);
            for (std::uint64_t j = 0; j < cubics; ++j) p.vertical.segments.push_back({number(r), number(r), number(r), number(r), number(r), number(r)});
            const auto way = std::lower_bound(tile.source.graph.ways.begin(), tile.source.graph.ways.end(), e.way_id,
                [](const auto& w, auto id) { return w.osm_id < id; });
            if (way == tile.source.graph.ways.end() || way->osm_id != e.way_id) return invalid("way absent");
            const auto attrs = resolve_road_attributes(*way); if (!attrs) return attrs.error(); p.attributes = *attrs; e.profile = std::move(p);
        }
        tile.entries.push_back(std::move(e));
    }
    if (!r.at_end()) return invalid("truncated or trailing bytes");
    if (!legacy) {
        ByteReader c(body.sections[2].data);
        const auto records = c.varint(); if (!c.ok() || records != tile.entries.size()) return invalid("cross-section record count");
        const auto u31 = [&](std::int32_t& out) { const auto v = c.varint(); if (v > 0x7fffffffu) return false; out = static_cast<std::int32_t>(v); return true; };
        const auto s31 = [&](std::int32_t& out) { const auto v = c.svarint(); if (v < -0x7fffffffLL || v > 0x7fffffffLL) return false; out = static_cast<std::int32_t>(v); return true; };
        for (auto& entry : tile.entries) {
            const auto kind = c.u8(); if (!c.ok() || kind > 2) return invalid("record kind");
            if (kind == 0) { if (entry.profile) return invalid("record kind"); continue; }
            if (!entry.profile) return invalid("record kind");
            if (kind == 2) {
                CrossSectionDecline d; const auto code = c.u16(); if (code < 1 || code > 3) return invalid("decline code");
                d.code = static_cast<CrossSectionDeclineCode>(code); if (!s31(d.detail)) return invalid("decline detail");
                entry.cross_section_decline = d; continue;
            }
            RoadCrossSection x; x.length_m = entry.profile->reference.length_m;
            x.flags = c.u16(); x.field_declines = c.u32();
            if ((x.flags & ~static_cast<unsigned>(kXsDefinedMask)) != 0 || (x.field_declines & ~static_cast<std::uint32_t>(kDeclineDefinedMask)) != 0) return invalid("reserved bits");
            const auto prov = c.u8(); if (prov > 15) return invalid("section provenance");
            x.count_provenance = static_cast<Provenance>(prov & 3); x.placement_provenance = static_cast<Provenance>(prov >> 2);
            const auto bands = c.varint(); if (!c.ok() || bands < 1 || bands > kMaxBands) return invalid("band count");
            for (std::uint64_t i = 0; i < bands; ++i) {
                CrossSectionBand b;
                const auto k = c.u8(), d = c.u8(), ch = c.u8(), bf = c.u8();
                if (!c.ok() || k > 7 || d > 3 || ch > 4 || bf > 7) return invalid("band enum");
                b.kind = static_cast<LaneKind>(k); b.direction = static_cast<LaneDirection>(d); b.change = static_cast<LaneChange>(ch);
                b.counted = bf & 1; b.paved = bf & 2; b.marked_right = bf & 4;
                if (!u31(b.width_mm)) return invalid("band width");
                b.turn = c.u16();
                const auto bp = c.u16(); if (bp > 0x3ff) return invalid("band provenance");
                b.provenance = {static_cast<Provenance>(bp & 3), static_cast<Provenance>((bp >> 2) & 3), static_cast<Provenance>((bp >> 4) & 3),
                    static_cast<Provenance>((bp >> 6) & 3), static_cast<Provenance>((bp >> 8) & 3)};
                if (!u31(b.start.length_mm) || !u31(b.start.from_width_mm) || !u31(b.end.length_mm) || !u31(b.end.from_width_mm)) return invalid("ramp");
                if (!c.ok()) return invalid("truncated cross-section");
                x.bands.push_back(b);
            }
            const auto ab = c.u8(), ae = c.u8(), as = c.u8(), om = c.u8();
            if (!c.ok() || ae > 2 || as > 2 || om > 3) return invalid("anchor or offset enum");
            x.anchor = {ab, static_cast<AnchorEdge>(ae), static_cast<PlacementSource>(as)}; x.offset.mode = static_cast<OffsetMode>(om);
            if (!s31(x.offset.start_mm) || !u31(x.offset.start_length_mm) || !s31(x.offset.end_mm) || !u31(x.offset.end_length_mm)) return invalid("offset");
            for (auto* j : {&x.start_join, &x.end_join}) {
                const auto jk = c.u8(); if (!c.ok() || jk > 5) return invalid("join kind");
                j->kind = static_cast<JoinKind>(jk); j->neighbour_way_id = c.svarint(); if (!s31(j->raw_delta_mm)) return invalid("join delta");
            }
            if (!c.ok()) return invalid("truncated cross-section");
            entry.cross_section = std::move(x);
        }
        if (!c.at_end()) return invalid("truncated or trailing cross-section bytes");
        ByteReader f(body.sections[3].data);
        const auto feature_count = f.varint(); if (!f.ok() || feature_count > kMaxFeatures || feature_count > f.remaining() / 16) return invalid("feature count");
        for (std::uint64_t i = 0; i < feature_count; ++i) {
            ProfileFeature p; const auto index = f.varint(); if (index >= tile.entries.size()) return invalid("feature entry");
            p.entry_index = static_cast<std::size_t>(index); p.node_id = f.svarint(); p.station_m = number(f);
            const auto kind = f.u8(), carve = f.u8(); if (!f.ok() || kind > 9 || carve > 1) return invalid("feature enum");
            p.kind = static_cast<ProfileFeatureKind>(kind); p.carve = carve == 1;
            const auto len = f.varint(), height = f.varint(), width = f.varint(); const auto prov = f.u8();
            if (!f.ok() || len > 0x7fffffffu || height > 0x7fffffffu || width > 0x7fffffffu || prov > 3) return invalid("feature size");
            p.length_mm = static_cast<std::int32_t>(len); p.height_mm = static_cast<std::int32_t>(height); p.width_mm = static_cast<std::int32_t>(width);
            p.provenance = static_cast<Provenance>(prov); tile.features.push_back(p);
        }
        if (!f.at_end()) return invalid("truncated or trailing feature bytes");
    }
    if (current) {
        const auto coordinate = [](std::int64_t v) { return v >= -kMaxCoordinateMm && v <= kMaxCoordinateMm; };
        ByteReader jr(body.sections[4].data);
        const auto junction_count = jr.varint(); if (!jr.ok() || junction_count > kMaxJunctions || junction_count > jr.remaining() / 24) return invalid("junction count");
        std::size_t ring_vertices = 0;
        for (std::uint64_t i = 0; i < junction_count; ++i) {
            RoadJunction j; const auto owner = jr.svarint(); j.flags = jr.u16(); const auto decline = jr.u16();
            if (!jr.ok() || decline >= kJunctionDeclineCount) return invalid("junction decline");
            j.decline = static_cast<JunctionDecline>(decline);
            const auto nodes = jr.varint(); if (!jr.ok() || nodes < 1 || nodes > kMaxJunctionNodes || nodes > jr.remaining()) return invalid("junction node count");
            for (std::uint64_t n = 0; n < nodes; ++n) j.nodes.push_back(jr.svarint());
            if (!jr.ok() || j.nodes.front() != owner) return invalid("junction owner");
            const auto approaches = jr.varint();
            if (!jr.ok() || approaches < 1 || approaches > kMaxJunctionApproaches || approaches > jr.remaining() / 11) return invalid("junction approach count");
            for (std::uint64_t a = 0; a < approaches; ++a) {
                JunctionApproach ap; const auto entry = jr.varint(); const auto at_end = jr.u8(), status = jr.u8(); ap.trim_m = number(jr);
                if (!jr.ok() || entry >= tile.entries.size() || at_end > 1 || status > 3) return invalid("junction approach");
                ap.entry_index = static_cast<std::size_t>(entry); ap.at_end = at_end == 1; ap.status = static_cast<JunctionApproachStatus>(status);
                j.approaches.push_back(ap);
            }
            const auto absorbed = jr.varint(); if (!jr.ok() || absorbed > tile.entries.size() || absorbed > jr.remaining()) return invalid("junction absorbed count");
            for (std::uint64_t a = 0; a < absorbed; ++a) {
                const auto entry = jr.varint(); if (!jr.ok() || entry >= tile.entries.size()) return invalid("junction absorbed entry");
                j.absorbed.push_back(static_cast<std::size_t>(entry));
            }
            for (auto& ring : j.rings) {
                const auto ring_count = jr.varint();
                if (!jr.ok() || ring_count > kMaxRingVertices || ring_count > jr.remaining() / 2 || (ring_vertices += ring_count) > kMaxTileRingVertices) return invalid("junction ring count");
                geom::PointMm p{};
                for (std::uint64_t v = 0; v < ring_count; ++v) {
                    const auto dx = jr.svarint(), dy = jr.svarint();
                    if (!jr.ok() || !coordinate(dx) || !coordinate(dy)) return invalid("junction ring vertex");
                    p = v == 0 ? geom::PointMm{dx, dy} : geom::PointMm{p.x + dx, p.y + dy};
                    if (!coordinate(p.x) || !coordinate(p.y)) return invalid("junction ring vertex");
                    ring.vertices.push_back(p);
                }
            }
            j.raised_height_m = number(jr); j.raised_node = jr.svarint();
            if (!jr.ok()) return invalid("truncated junction");
            tile.junctions.push_back(std::move(j));
        }
        if (!jr.at_end()) return invalid("truncated or trailing junction bytes");
        ByteReader ur(body.sections[5].data);
        const auto furniture_count = ur.varint(); if (!ur.ok() || furniture_count > kMaxFurniture || furniture_count > ur.remaining() / 20) return invalid("furniture count");
        for (std::uint64_t i = 0; i < furniture_count; ++i) {
            RoadFurniture furn; const auto kind = ur.u8(); const auto decline = ur.u16();
            furn.node_id = ur.svarint(); const auto entry = ur.varint(), junction = ur.varint();
            furn.station_m = number(ur); const auto travel = ur.u8();
            const auto t_from = ur.svarint(), t_to = ur.svarint(); const auto length = ur.varint();
            const auto pattern = ur.u8(); furn.flags = ur.u8();
            if (!ur.ok() || kind > 3 || decline >= kFurnitureDeclineCount || entry >= tile.entries.size() || junction > tile.junctions.size() ||
                travel > 1 || pattern >= kCrossingPatternCount || t_from < -200000 || t_from > 200000 || t_to < -200000 || t_to > 200000 || length > 100000)
                return invalid("furniture record");
            furn.kind = static_cast<FurnitureKind>(kind); furn.decline = static_cast<FurnitureDecline>(decline);
            furn.entry_index = static_cast<std::size_t>(entry); furn.junction_index = static_cast<std::uint32_t>(junction);
            furn.travel = static_cast<LaneDirection>(travel); furn.pattern = static_cast<CrossingPattern>(pattern);
            furn.t_from = static_cast<double>(t_from) / 1000.0; furn.t_to = static_cast<double>(t_to) / 1000.0; furn.length_m = static_cast<double>(length) / 1000.0;
            tile.furniture.push_back(furn);
        }
        if (!ur.at_end()) return invalid("truncated or trailing furniture bytes");
        ByteReader pr(body.sections[6].data);
        const auto origins = pr.varint(); if (!pr.ok() || origins != tile.entries.size() || origins > pr.remaining() / 3) return invalid("provenance count");
        for (std::uint64_t i = 0; i < origins; ++i) {
            const auto provenance = pr.u8(); const auto code = pr.u16();
            if (!pr.ok() || provenance > max_provenance) return invalid("provenance record");
            tile.origins.push_back({static_cast<ProfileProvenance>(provenance), code});
        }
        if (!pr.at_end()) return invalid("truncated or trailing provenance bytes");
    }
    if (has_underpass_section(body.encoding)) {
        ByteReader dr(body.sections[7].data);
        const auto underpass_count = dr.varint(); if (!dr.ok() || underpass_count > kMaxUnderpasses || underpass_count > dr.remaining() / 6) return invalid("underpass underpass_count");
        for (std::uint64_t i = 0; i < underpass_count; ++i) {
            RoadUnderpass u;
            const auto entry = dr.varint(), start = dr.varint(), length = dr.varint(), depth = dr.varint();
            u.flags = dr.u16();
            const auto ways = dr.varint();
            if (!dr.ok() || entry >= tile.entries.size() || start > static_cast<std::uint64_t>(kMaxCoordinateMm) ||
                length > static_cast<std::uint64_t>(kMaxCoordinateMm) || depth == 0 || depth > static_cast<std::uint64_t>(kMaxUnderpassDepthMm) ||
                ways == 0 || ways > kMaxUnderpassLowerWays) return invalid("underpass record");
            u.entry_index = static_cast<std::size_t>(entry);
            u.start_m = static_cast<double>(start) / 1000.0; u.end_m = static_cast<double>(start + length) / 1000.0;
            u.depth_mm = static_cast<std::int32_t>(depth);
            std::uint64_t last = 0;
            for (std::uint64_t k = 0; k < ways; ++k) {
                const auto delta = dr.varint();
                if (!dr.ok() || delta == 0 || delta > static_cast<std::uint64_t>(std::numeric_limits<std::int64_t>::max()) - last) return invalid("underpass lower way");
                last += delta; u.lower_ways.push_back(static_cast<std::int64_t>(last));
            }
            tile.underpasses.push_back(std::move(u));
        }
        if (!dr.at_end()) return invalid("truncated or trailing underpass bytes");
    }
    const auto valid = validate_road_geom_tile(tile); if (!valid) return valid.error(); return tile;
}
}
