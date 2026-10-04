#include "g2m/layer/road_geom_tile.h"
#include "g2m/core/bytes.h"
#include "g2m/core/det_math.h"
#include <algorithm>
#include <bit>
#include <cmath>
namespace g2m {
namespace {
constexpr std::size_t kMaxEntries = 100000, kMaxPieces = 1000000;
Error invalid(std::string message) { return make_error(ErrorCode::Malformed, "roads.geom: " + message); }
void number(ByteWriter& w, double d) { w.u64(std::bit_cast<std::uint64_t>(d)); }
double number(ByteReader& r) { return std::bit_cast<double>(r.u64()); }
bool finite(double d) { return std::isfinite(d) && std::fabs(d) <= 1e9; }
}
Result<RoadGeomTile> build_road_geom_tile(const RoadGraphTile& graph, const RoadDemSampler& dem,
    const RoadProfileLimits& limits, const RoadGraph* context) {
    const auto valid = validate_road_graph_tile(graph); if (!valid) return valid.error();
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
    RoadGeomTile result; result.source = graph;
    for (const auto& way : graph.graph.ways) {
        const auto stretches = builder->stretches(way.osm_id); if (!stretches) return stretches.error();
        for (const auto stretch : *stretches) {
            if (result.entries.size() == kMaxEntries) return make_error(ErrorCode::LimitExceeded, "roads.geom: entry budget");
            RoadGeomEntry entry; entry.way_id = way.osm_id; entry.stretch = stretch;
            auto profile = build_road_profile(*builder, dem, graph.key.zone, way.osm_id, stretch, limits);
            if (profile) entry.profile = std::move(*profile); else entry.decline = profile.error();
            result.entries.push_back(std::move(entry));
        }
    }
    const auto check = validate_road_geom_tile(result); if (!check) return check.error();
    return result;
}
Result<void> validate_road_geom_tile(const RoadGeomTile& tile) {
    const auto graph_valid = validate_road_graph_tile(tile.source); if (!graph_valid) return graph_valid.error();
    if (tile.entries.size() > kMaxEntries) return invalid("entry budget");
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
            if (entry.decline) {
                const auto code = static_cast<unsigned>(entry.decline->code);
                if (code < 1 || code > static_cast<unsigned>(ErrorCode::ReadOnly) || entry.decline->message.empty() || entry.decline->message.size() > 512)
                    return invalid("decline");
                continue;
            }
            const auto& p = *entry.profile;
            const auto attrs = resolve_road_attributes(way); if (!attrs) return attrs.error();
            const auto& ref = p.reference;
            if (p.attributes != *attrs || p.attributes.tunnel || p.stretch.start_ref != stretch.start_ref || p.stretch.end_ref != stretch.end_ref ||
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
            const double grade = attrs->max_grade_per_mille / 1000.0, curvature = 1.0 / attrs->min_vertical_radius_m;
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
        }
        if (!any || next + 1 != way.node_ids.size()) return invalid("missing stretch");
    }
    if (index != tile.entries.size()) return invalid("extra stretches"); return {};
}
Result<TileBody> encode_road_geom(const RoadGeomTile& tile) {
    const auto valid = validate_road_geom_tile(tile); if (!valid) return valid.error();
    const auto graph = encode_road_graph(tile.source); if (!graph) return graph.error();
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
    return TileBody{kRoadGeomEncoding, 1, {graph->sections[0], {2, SectionCodec::Zstd, w.take()}}};
}
Result<RoadGeomTile> decode_road_geom(const TileBody& body) {
    if (body.encoding != kRoadGeomEncoding || body.encoding_version != 1) return make_error(ErrorCode::Unsupported, "roads.geom: encoding/version");
    if (body.sections.size() != 2 || body.sections[0].type != 1 || body.sections[1].type != 2 || body.sections[1].data.size() > kMaxSectionRawSize) return invalid("sections");
    auto graph = decode_road_graph(TileBody{kRoadGraphEncoding, 1, {body.sections[0]}}); if (!graph) return graph.error();
    RoadGeomTile tile; tile.source = std::move(*graph); ByteReader r(body.sections[1].data);
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
    const auto valid = validate_road_geom_tile(tile); if (!valid) return valid.error(); return tile;
}
}
