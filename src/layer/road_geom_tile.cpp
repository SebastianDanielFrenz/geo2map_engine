#include "g2m/layer/road_geom_tile.h"
#include "g2m/core/bytes.h"
#include "g2m/core/det_math.h"
#include <algorithm>
#include <bit>
#include <cmath>
#include <cstdint>
#include <tuple>
namespace g2m {
namespace {
constexpr std::size_t kMaxEntries = 100000, kMaxPieces = 1000000;
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
        if (static_cast<unsigned>(j->kind) > 4) return invalid("join kind");
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
}
Result<RoadGeomTile> build_road_geom_tile(const RoadGraphTile& graph, const RoadDemSampler& dem,
    const RoadProfileLimits& limits, const RoadGraph* context, const RoadRules& rules) {
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
            if (profile) {
                entry.profile = std::move(*profile);
                // Cross-sections never read the DEM; they are stored only where a profile (hence a reference) exists.
                auto xs = resolve_cross_section(*builder, way.osm_id, stretch, entry.profile->reference.length_m, rules);
                if (!xs) return xs.error();
                entry.cross_section = std::move(xs->xs); entry.cross_section_decline = xs->decline;
            } else entry.decline = profile.error();
            result.entries.push_back(std::move(entry));
        }
    }
    const auto check = validate_road_geom_tile(result); if (!check) return check.error();
    return result;
}
Result<void> validate_road_geom_tile(const RoadGeomTile& tile) {
    const auto graph_valid = validate_road_graph_tile(tile.source); if (!graph_valid) return graph_valid.error();
    if (tile.encoding != kRoadGeomEncoding && tile.encoding != kRoadGeomEncodingLegacy) return invalid("encoding");
    const bool legacy = tile.encoding == kRoadGeomEncodingLegacy;
    if (tile.entries.size() > kMaxEntries) return invalid("entry budget");
    if (legacy && !tile.features.empty()) return invalid("legacy tile carries features");
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
            if (entry.cross_section) { const auto xs = validate_cross_section(*entry.cross_section, ref.length_m); if (!xs) return xs.error(); }
        }
        if (!any || next + 1 != way.node_ids.size()) return invalid("missing stretch");
    }
    if (index != tile.entries.size()) return invalid("extra stretches");
    return validate_features(tile);
}
Result<TileBody> encode_road_geom(const RoadGeomTile& tile) {
    if (tile.encoding != kRoadGeomEncoding) return make_error(ErrorCode::InvalidArgument, "roads.geom: only encoding 6 is written");
    const auto valid = validate_road_geom_tile(tile); if (!valid) return valid.error();
    // Encoding 6 embeds the encoding-5 graph section (with node tags), byte for byte.
    const auto graph = encode_road_graph(tile.source, kRoadGraphEncoding); if (!graph) return graph.error();
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
    return TileBody{kRoadGeomEncoding, 1, {graph->sections[0], {2, SectionCodec::Zstd, w.take()}, {3, SectionCodec::Zstd, xs.take()}, {4, SectionCodec::Zstd, fw.take()}}};
}
Result<RoadGeomTile> decode_road_geom(const TileBody& body) {
    if ((body.encoding != kRoadGeomEncoding && body.encoding != kRoadGeomEncodingLegacy) || body.encoding_version != 1)
        return make_error(ErrorCode::Unsupported, "roads.geom: encoding/version");
    const bool legacy = body.encoding == kRoadGeomEncodingLegacy;
    if (body.sections.size() != (legacy ? 2u : 4u) || body.sections[0].type != 1 || body.sections[1].type != 2) return invalid("sections");
    for (const auto& section : body.sections) if (section.data.size() > kMaxSectionRawSize) return invalid("section size");
    if (!legacy && (body.sections[2].type != 3 || body.sections[3].type != 4)) return invalid("sections");
    auto graph = decode_road_graph(TileBody{legacy ? kRoadGraphEncodingLegacy : kRoadGraphEncoding, 1, {body.sections[0]}}); if (!graph) return graph.error();
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
                const auto jk = c.u8(); if (!c.ok() || jk > 4) return invalid("join kind");
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
    const auto valid = validate_road_geom_tile(tile); if (!valid) return valid.error(); return tile;
}
}
