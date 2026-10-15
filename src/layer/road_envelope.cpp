// Street fidelity P1 slice 6: envelope_road_segments, the grip-raster input of a roads.geom tile
// (see g2m/layer/road_envelope.h and design 8.1).
#include "g2m/layer/road_envelope.h"
#include "g2m/core/det_math.h"
#include <algorithm>
#include <array>
#include <cmath>
#include <tuple>

namespace g2m {
namespace {

struct Keyed {
    RoadSegment seg;
    int layer = 0;            // 0 sidewalk, 1 road bed
    std::size_t entry = 0;
    std::size_t seq = 0;
};

struct WaySpeed { RoadSpeedLimit forward, backward; };

WaySpeed speed_of(const RoadGraphWay& way) {
    std::string_view maxspeed, forward, backward;
    bool conditional = false;
    for (const auto& [key, value] : way.tags) {
        if (key == "maxspeed") maxspeed = value;
        else if (key == "maxspeed:forward") forward = value;
        else if (key == "maxspeed:backward") backward = value;
        else if (key.starts_with("maxspeed:") && (key.find("conditional") != key.npos || key.find("lanes") != key.npos || key.find("variable") != key.npos)) conditional = true;
    }
    WaySpeed out;
    out.forward = parse_road_speed_limit(forward.empty() ? maxspeed : forward);
    out.backward = parse_road_speed_limit(backward.empty() ? maxspeed : backward);
    out.forward.conditional = out.backward.conditional = conditional;
    return out;
}

const RoadGraphWay* find_way(const RoadGraph& graph, std::int64_t id) {
    const auto it = std::lower_bound(graph.ways.begin(), graph.ways.end(), id,
        [](const RoadGraphWay& w, std::int64_t v) { return w.osm_id < v; });
    return it != graph.ways.end() && it->osm_id == id ? &*it : nullptr;
}

struct Box { std::int64_t w, s, e, n; };

bool touches(const Box& box, const geom::PointMm& a, const geom::PointMm& b, std::int64_t hw) {
    const std::int64_t lo_x = std::min(a.x, b.x) - hw, hi_x = std::max(a.x, b.x) + hw;
    const std::int64_t lo_y = std::min(a.y, b.y) - hw, hi_y = std::max(a.y, b.y) + hw;
    return hi_x >= box.w && lo_x <= box.e && hi_y >= box.s && lo_y <= box.n;
}

constexpr double kSidewalkOverlapM = 0.08;

std::int64_t mm(double metres) { return static_cast<std::int64_t>(std::llround(metres * 1000.0)); }

} // namespace

std::vector<RoadSegment> envelope_road_segments(const RoadGeomTile& tile, const TileKey& l0_key, double spacing_m) {
    std::vector<RoadSegment> result;
    if (!(spacing_m > 0.0) || !l0_key.valid() || !(l0_key.zone == tile.source.key.zone)) return result;
    const Box box{l0_key.min_easting() * 1000, l0_key.min_northing() * 1000,
                  (l0_key.min_easting() + l0_key.size_m()) * 1000, (l0_key.min_northing() + l0_key.size_m()) * 1000};

    std::vector<Keyed> keyed;
    std::size_t seq = 0;
    for (std::size_t ei = 0; ei < tile.entries.size(); ++ei) {
        const RoadGeomEntry& entry = tile.entries[ei];
        if (!entry.profile || !entry.cross_section) continue;
        const RoadProfile& profile = *entry.profile;
        if (profile.attributes.tunnel) continue;   // a TunnelFloor profile (D47) never paints the ground above it (osm_roads skips tunnels too)
        const RoadCrossSection& xs = *entry.cross_section;
        const RoadReferenceLine& ref = profile.reference;
        const double length = ref.length_m;
        if (!(length > 0.0) || xs.bands.empty() || xs.bands.size() > 32) continue;
        const RoadGraphWay* way = find_way(tile.source.graph, entry.way_id);
        WaySpeed speed;
        int rank = 0;
        if (way) { speed = speed_of(*way); rank = way->rank; }

        RoadSegment base;
        base.land_class = profile.attributes.land_class;
        base.surface = profile.attributes.surface;
        base.rank = rank;
        base.way_id = entry.way_id;
        base.bridge = profile.attributes.bridge;
        base.forward_limit = speed.forward;
        base.backward_limit = speed.backward;

        // Sidewalk band indices of this entry (a band's kind never changes along the stretch).
        std::array<std::size_t, 32> sidewalks{};
        std::size_t sidewalk_count = 0;
        for (std::size_t i = 0; i < xs.bands.size(); ++i) if (xs.bands[i].kind == LaneKind::Sidewalk) sidewalks[sidewalk_count++] = i;

        struct Sample { geom::CurvePose pose; CarveFootprint fp; std::array<double, 33> t{}; };
        const auto sample = [&](double s, Sample& out) {
            const auto pose = ref.at(s);
            if (!pose) return false;
            out.pose = *pose;
            out.fp = carve_footprint(xs, s);
            return xs.boundaries_at(s, std::span<double>(out.t.data(), xs.bands.size() + 1));
        };
        const auto point = [](const Sample& smp, double t) {
            double sn, cs;
            det::sincos(smp.pose.heading, sn, cs);
            return geom::PointMm{mm(smp.pose.x - t * sn), mm(smp.pose.y + t * cs)};
        };

        Sample s0, s1;
        double prev_s = 0.0;
        if (!sample(0.0, s0)) continue;
        for (std::size_t k = 1;; ++k) {
            double s = static_cast<double>(k) * spacing_m;
            const bool last = s >= length;
            if (last) s = length;
            if (s <= prev_s) break;
            if (!sample(s, s1)) break;

            // Road bed.
            {
                const double m0 = (s0.fp.roadbed_left + s0.fp.roadbed_right) / 2.0;
                const double m1 = (s1.fp.roadbed_left + s1.fp.roadbed_right) / 2.0;
                const double h0 = (s0.fp.roadbed_left - s0.fp.roadbed_right) / 2.0;
                const double h1 = (s1.fp.roadbed_left - s1.fp.roadbed_right) / 2.0;
                const std::int64_t hw = mm((h0 + h1) / 2.0);
                if (hw > 0) {
                    RoadSegment seg = base;
                    seg.a = point(s0, m0); seg.b = point(s1, m1); seg.half_width_mm = hw;
                    if (touches(box, seg.a, seg.b, hw)) keyed.push_back({seg, 1, ei, seq});
                }
            }
            // Sidewalks.
            for (std::size_t q = 0; q < sidewalk_count; ++q) {
                const std::size_t i = sidewalks[q];
                // The inner edge reaches kSidewalkOverlapM into the road bed, which is painted later and overwrites it:
                // two capsule chains with independent rounding must not leave a hairline gap of terrain between them.
                const auto span_of = [&](const Sample& smp, double& upper, double& lower) {
                    upper = smp.t[i]; lower = smp.t[i + 1];
                    if ((upper + lower) / 2.0 > (smp.fp.roadbed_left + smp.fp.roadbed_right) / 2.0) lower -= kSidewalkOverlapM;
                    else upper += kSidewalkOverlapM;
                };
                double u0, l0, u1, l1;
                span_of(s0, u0, l0); span_of(s1, u1, l1);
                const double m0 = (u0 + l0) / 2.0, m1 = (u1 + l1) / 2.0;
                const double h0 = (u0 - l0) / 2.0, h1 = (u1 - l1) / 2.0;
                const std::int64_t hw = mm((h0 + h1) / 2.0);
                if (hw <= 0) continue;
                RoadSegment seg = base;
                seg.land_class = LandClass::PavedRoad;
                seg.surface = SurfaceKind::PavingStones;
                seg.a = point(s0, m0); seg.b = point(s1, m1); seg.half_width_mm = hw;
                if (touches(box, seg.a, seg.b, hw)) keyed.push_back({seg, 0, ei, seq + 1 + q});
            }
            seq += 1 + sidewalk_count;
            s0 = s1;
            prev_s = s;
            if (last) break;
        }
    }
    std::sort(keyed.begin(), keyed.end(), [](const Keyed& x, const Keyed& y) {
        return std::tie(x.seg.rank, x.seg.way_id, x.layer, x.entry, x.seq) < std::tie(y.seg.rank, y.seg.way_id, y.layer, y.entry, y.seq);
    });
    result.reserve(keyed.size());
    for (const Keyed& k : keyed) result.push_back(k.seg);
    return result;
}

} // namespace g2m
