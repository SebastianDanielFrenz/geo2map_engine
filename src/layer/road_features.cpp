// Street fidelity P1 slice 7: traffic-calming features of roads.geom section 4 (design 8.3, owner decision Q2).
// Tag reading and station lookup at build time, the carved height function for consumers. The height function
// uses only + - * / (road_smooth_step), so it is bit-identical on every platform.
#include "g2m/layer/road_geom_tile.h"
#include "g2m/core/det_math.h"
#include <algorithm>
#include <charconv>
#include <cmath>
#include <optional>

namespace g2m {

double feature_ramp_m(const ProfileFeature& f) noexcept {
    const double length = static_cast<double>(f.length_mm) / 1000.0;
    const double min_ramp = static_cast<double>(kFeatureMinRampMm) / 1000.0;
    double ramp = length / 2.0;
    if (f.kind == ProfileFeatureKind::Table) {
        const double slope = static_cast<double>(f.height_mm) * static_cast<double>(kFeatureTableRampRatio) / 1000.0;
        if (slope < ramp) ramp = slope;
    }
    return ramp > min_ramp ? ramp : min_ramp;
}

double feature_effective_length_m(const ProfileFeature& f) noexcept {
    const double length = static_cast<double>(f.length_mm) / 1000.0;
    const double twice = 2.0 * feature_ramp_m(f);
    return length > twice ? length : twice;
}

double feature_height_contribution_m(const ProfileFeature& f, double station_m) noexcept {
    if (!f.carve || f.height_mm <= 0) return 0.0;
    const double ramp = feature_ramp_m(f);
    const double half = feature_effective_length_m(f) / 2.0;
    double d = station_m - f.station_m;
    if (d < 0.0) d = -d;
    if (!(d < half)) return 0.0;
    const double plateau = half - ramp;
    const double height = (f.kind == ProfileFeatureKind::Dip ? -1.0 : 1.0) * static_cast<double>(f.height_mm) / 1000.0;
    if (d <= plateau) return height;
    return height * (1.0 - road_smooth_step((d - plateau) / ramp));
}

double feature_height_m(const RoadGeomTile& tile, std::size_t entry, double s) noexcept {
    const auto& features = tile.features;
    auto it = std::lower_bound(features.begin(), features.end(), entry,
        [](const ProfileFeature& f, std::size_t e) { return f.entry_index < e; });
    double sum = 0.0;
    for (; it != features.end() && it->entry_index == entry; ++it) sum += feature_height_contribution_m(*it, s);
    return sum;
}

namespace {

std::optional<ProfileFeatureKind> kind_of(std::string_view value) {
    struct Name { std::string_view tag; ProfileFeatureKind kind; };
    static constexpr Name kNames[] = {{"bump", ProfileFeatureKind::Bump}, {"mini_bumps", ProfileFeatureKind::MiniBumps},
        {"rumble_strip", ProfileFeatureKind::RumbleStrip}, {"cushion", ProfileFeatureKind::Cushion}, {"hump", ProfileFeatureKind::Hump},
        {"table", ProfileFeatureKind::Table}, {"chicane", ProfileFeatureKind::Chicane}, {"choker", ProfileFeatureKind::Choker},
        {"island", ProfileFeatureKind::Island}, {"painted_island", ProfileFeatureKind::Island}, {"dip", ProfileFeatureKind::Dip}};
    for (const auto& n : kNames) if (n.tag == value) return n.kind;
    return std::nullopt;
}

// A clean positive number of metres (no unit, no spaces), at most 50 m, as millimetres.
std::optional<std::int32_t> length_mm_of(std::string_view text) {
    double v = 0.0;
    const auto r = std::from_chars(text.data(), text.data() + text.size(), v);
    if (text.empty() || r.ec != std::errc{} || r.ptr != text.data() + text.size() || !std::isfinite(v) || !(v > 0.0) || v > 50.0) return std::nullopt;
    const auto mm = static_cast<std::int32_t>(std::llround(v * 1000.0));
    return mm >= 1 ? std::optional<std::int32_t>(mm) : std::nullopt;
}

const std::string* tag_of(const RoadGraphNode& node, std::string_view key) {
    for (const auto& [k, v] : node.tags) if (k == key) return &v;
    return nullptr;
}

double distance_to(const RoadReferenceLine& ref, double station, const geom::PointMm& node, bool& ok) {
    const auto pose = ref.at(station);
    if (!pose) { ok = false; return 0.0; }
    const double dx = pose->x - static_cast<double>(node.x) / 1000.0, dy = pose->y - static_cast<double>(node.y) / 1000.0;
    return det::sqrt(dx * dx + dy * dy);
}

// Station of the shape node at way ref index `index`: the start of the first straight piece of a pair at or after
// it (a clothoid knot is exactly there), or the middle of the rounded corner arc just before it (the corner arc
// of a rounded polyline carries the PREVIOUS pair's index, piece 1). The nearer to the node wins.
std::optional<double> station_of(const RoadReferenceLine& ref, std::uint32_t index, const geom::PointMm& node) {
    const auto it = std::find_if(ref.segments.begin(), ref.segments.end(),
        [&](const RoadReferenceSegment& s) { return s.piece_index == 0 && s.original_pair_index >= index; });
    if (it == ref.segments.end()) return std::nullopt;
    double best = it->station_m;
    bool ok = true;
    if (it != ref.segments.begin() && std::prev(it)->piece_index == 1) {
        const auto& arc = *std::prev(it);
        const double middle = arc.station_m + arc.curve.length / 2.0;
        const double d_arc = distance_to(ref, middle, node, ok), d_knot = distance_to(ref, best, node, ok);
        if (ok && d_arc < d_knot) best = middle;
    }
    return best;
}

} // namespace

Result<std::vector<ProfileFeature>> resolve_profile_features(const RoadGraph& graph, const RoadGeomEntry& entry,
    std::size_t entry_index, const RoadRules& rules) {
    std::vector<ProfileFeature> out;
    if (!entry.profile || !entry.cross_section) return out;
    const auto way = std::lower_bound(graph.ways.begin(), graph.ways.end(), entry.way_id,
        [](const RoadGraphWay& w, std::int64_t id) { return w.osm_id < id; });
    if (way == graph.ways.end() || way->osm_id != entry.way_id || entry.stretch.end_ref >= way->node_ids.size())
        return make_error(ErrorCode::Malformed, "road features: way absent or stretch out of range");
    const RoadReferenceLine& ref = entry.profile->reference;
    const double length = ref.length_m;
    for (std::uint32_t i = entry.stretch.start_ref + 1; i < entry.stretch.end_ref; ++i) {
        const std::int64_t node_id = way->node_ids[i];
        const auto node = std::lower_bound(graph.nodes.begin(), graph.nodes.end(), node_id,
            [](const RoadGraphNode& n, std::int64_t id) { return n.osm_id < id; });
        if (node == graph.nodes.end() || node->osm_id != node_id) return make_error(ErrorCode::Malformed, "road features: node absent");
        const std::string* value = tag_of(*node, "traffic_calming");
        if (!value) continue;
        const auto kind = kind_of(*value);
        if (!kind) continue;
        const auto station = station_of(ref, i, node->position);
        if (!station) continue;
        const TrafficCalmingRules& defaults = rules.calming(*kind);
        ProfileFeature f;
        f.entry_index = entry_index; f.node_id = node_id; f.station_m = *station; f.kind = *kind;
        f.carve = defaults.height_mm > 0;
        f.length_mm = defaults.length_mm; f.height_mm = defaults.height_mm; f.width_mm = defaults.width_mm;
        f.provenance = Provenance::Inferred;
        if (const std::string* tagged = tag_of(*node, "traffic_calming:length")) {
            if (const auto mm = length_mm_of(*tagged)) { f.length_mm = *mm; f.provenance = Provenance::Tagged; }
        }
        // The carved footprint stays inside the stretch: the junction beyond its end owns that part (P2).
        const double room = std::min(*station, length - *station);
        const double minimum = f.carve ? static_cast<double>(kFeatureMinRampMm) / 1000.0 : 0.5;
        if (!(room >= minimum)) continue;
        const double fits_mm = 2.0 * room * 1000.0;
        if (feature_effective_length_m(f) * 1000.0 > fits_mm) {
            f.length_mm = static_cast<std::int32_t>(fits_mm);
            f.provenance = Provenance::Derived;
        }
        out.push_back(f);
    }
    std::sort(out.begin(), out.end(), [](const ProfileFeature& a, const ProfileFeature& b) {
        return std::pair(a.station_m, a.node_id) < std::pair(b.station_m, b.node_id);
    });
    return out;
}

} // namespace g2m
