// Street fidelity P1 slice 3: the cross-section of one way from its own tags and the country rules
// (design section 3), and the pure evaluation of a stored cross-section (design section 2.1).
// Everything here is deterministic integer arithmetic plus + - * / on doubles: no libm, no locale.
#include "g2m/layer/road_cross_section.h"
#include "g2m/layer/road_attributes.h"
#include <algorithm>
#include <charconv>
#include <string>

namespace g2m {

double road_smooth_step(double u) noexcept {
    if (!(u > 0.0)) return 0.0; // also NaN
    if (u >= 1.0) return 1.0;
    if (u <= 0.5) return 2.0 * u * u;
    const double v = 1.0 - u;
    return 1.0 - 2.0 * v * v;
}

double RoadCrossSection::band_width_m(std::size_t band, double s) const {
    const auto& b = bands[band];
    double w = static_cast<double>(b.width_mm);
    if (b.start.length_mm > 0) {
        const double a = static_cast<double>(b.start.length_mm) / 1000.0;
        w += static_cast<double>(b.start.from_width_mm - b.width_mm) * (1.0 - road_smooth_step(s / a));
    }
    if (b.end.length_mm > 0) {
        const double a = static_cast<double>(b.end.length_mm) / 1000.0;
        w += static_cast<double>(b.end.from_width_mm - b.width_mm) * (1.0 - road_smooth_step((length_m - s) / a));
    }
    return std::max(0.0, w) / 1000.0;
}

double RoadCrossSection::offset_m(double s) const {
    double mm = 0.0;
    switch (offset.mode) {
    case OffsetMode::None: break;
    case OffsetMode::Hold: mm = static_cast<double>(offset.start_mm); break;
    case OffsetMode::Ramps:
        if (offset.start_length_mm > 0 && offset.start_mm != 0) {
            const double a = static_cast<double>(offset.start_length_mm) / 1000.0;
            mm += static_cast<double>(offset.start_mm) * (1.0 - road_smooth_step(s / a));
        }
        if (offset.end_length_mm > 0 && offset.end_mm != 0) {
            const double a = static_cast<double>(offset.end_length_mm) / 1000.0;
            mm += static_cast<double>(offset.end_mm) * (1.0 - road_smooth_step((length_m - s) / a));
        }
        break;
    case OffsetMode::Transition:
        if (length_m > 0.0)
            mm = static_cast<double>(offset.start_mm) +
                 static_cast<double>(offset.end_mm - offset.start_mm) * road_smooth_step(s / length_m);
        else mm = static_cast<double>(offset.start_mm);
        break;
    }
    return mm / 1000.0;
}

bool RoadCrossSection::boundaries_at(double s, std::span<double> out) const {
    const std::size_t n = bands.size();
    if (out.size() != n + 1 || n == 0 || anchor.band >= n) return false;
    const std::size_t a = anchor.band;
    const double wa = band_width_m(a, s);
    const double t = offset_m(s);
    double left = t;
    if (anchor.edge == AnchorEdge::Centre) left = t + wa / 2.0;
    else if (anchor.edge == AnchorEdge::Right) left = t + wa;
    out[a] = left;
    out[a + 1] = left - wa;
    for (std::size_t i = a; i-- > 0;) out[i] = out[i + 1] + band_width_m(i, s);
    for (std::size_t i = a + 1; i < n; ++i) out[i + 1] = out[i] - band_width_m(i, s);
    return true;
}

namespace {
std::string_view trim(std::string_view text) {
    while (!text.empty() && (text.front() == ' ' || text.front() == '\t')) text.remove_prefix(1);
    while (!text.empty() && (text.back() == ' ' || text.back() == '\t')) text.remove_suffix(1);
    return text;
}
std::vector<std::string_view> split(std::string_view text, char separator) {
    std::vector<std::string_view> out;
    std::size_t start = 0;
    for (;;) {
        const auto at = text.find(separator, start);
        if (at == std::string_view::npos) { out.push_back(text.substr(start)); return out; }
        out.push_back(text.substr(start, at - start));
        start = at + 1;
    }
}
std::optional<std::int32_t> parse_int(std::string_view text, std::int32_t lo, std::int32_t hi) {
    text = trim(text);
    if (text.empty()) return std::nullopt;
    std::int32_t v = 0;
    const auto r = std::from_chars(text.data(), text.data() + text.size(), v);
    if (r.ec != std::errc{} || r.ptr != text.data() + text.size() || v < lo || v > hi) return std::nullopt;
    return v;
}

struct Group {
    int first = 0, n = 0;     // counted lanes first .. first+n-1 (way order)
    bool mirrored = false;    // travel order runs against way order
    std::string suffix;       // tag key suffix: "" (oneway), ":forward", ":backward"
    int band(int travel_index) const { return first + (mirrored ? n - 1 - travel_index : travel_index); }
};

struct PlacementValue {
    bool transition = false;
    int group_lane = 0;       // 1-based, in the group's travel order
    AnchorEdge edge = AnchorEdge::Centre; // in the group's travel frame
};

struct Resolver {
    const RoadGraphWay& way;
    std::uint32_t declines = 0;

    const std::string* find(std::string_view key) const {
        auto it = std::lower_bound(way.tags.begin(), way.tags.end(), key,
            [](const auto& item, std::string_view name) { return item.first < name; });
        return it != way.tags.end() && it->first == key ? &it->second : nullptr;
    }
    std::optional<std::string_view> get(std::string_view key) const {
        const auto* v = find(key);
        if (!v) return std::nullopt;
        return trim(*v);
    }
    bool starts_with_key(std::string_view prefix) const {
        auto it = std::lower_bound(way.tags.begin(), way.tags.end(), prefix,
            [](const auto& item, std::string_view name) { return item.first < name; });
        return it != way.tags.end() && std::string_view(it->first).starts_with(prefix);
    }
};

std::uint16_t parse_turn(std::string_view piece, bool& unknown) {
    std::uint16_t mask = 0;
    for (auto token : split(piece, ';')) {
        const auto bit = parse_turn_token(token);
        if (bit) mask |= *bit; else unknown = true;
    }
    return unknown ? 0 : mask;
}

constexpr std::uint16_t kTurnDirectional = kTurnLeft | kTurnSlightLeft | kTurnSharpLeft | kTurnRight |
                                           kTurnSlightRight | kTurnSharpRight | kTurnReverse;

// Placement value of one tag: "transition", "left_of:N", "right_of:N", "middle_of:N".
std::optional<PlacementValue> parse_placement(std::string_view text) {
    text = trim(text);
    PlacementValue out;
    if (text == "transition") { out.transition = true; return out; }
    std::string_view number;
    if (text.starts_with("left_of:")) { out.edge = AnchorEdge::Left; number = text.substr(8); }
    else if (text.starts_with("right_of:")) { out.edge = AnchorEdge::Right; number = text.substr(9); }
    else if (text.starts_with("middle_of:")) { out.edge = AnchorEdge::Centre; number = text.substr(10); }
    else return std::nullopt;
    const auto n = parse_int(number, 0, 1000000);
    if (!n) return std::nullopt;
    out.group_lane = *n; // range is checked against the group by the caller
    return out;
}

// What a cycleway/parking/sidewalk side value means.
enum class SideMeaning { Band, None, Bad };

CrossSectionBand make_band(LaneKind kind, std::int32_t width_mm, bool paved) {
    CrossSectionBand b;
    b.kind = kind; b.width_mm = width_mm; b.paved = paved; b.counted = false;
    b.direction = LaneDirection::None;
    b.provenance.kind = Provenance::Tagged; b.provenance.direction = Provenance::Inferred;
    b.provenance.width = Provenance::Inferred; b.provenance.turn = Provenance::Inferred;
    b.provenance.change = Provenance::Inferred;
    return b;
}
} // namespace

// The one table of turn:lanes value tokens (OSM wiki Key:turn): road_cross_section.h.
std::optional<std::uint16_t> parse_turn_token(std::string_view token) {
    token = trim(token);
    if (token.empty() || token == "none") return kTurnNone;
    if (token == "through") return kTurnThrough;
    if (token == "left") return kTurnLeft;
    if (token == "slight_left") return kTurnSlightLeft;
    if (token == "sharp_left") return kTurnSharpLeft;
    if (token == "right") return kTurnRight;
    if (token == "slight_right") return kTurnSlightRight;
    if (token == "sharp_right") return kTurnSharpRight;
    if (token == "reverse") return kTurnReverse;
    if (token == "merge_to_left") return kTurnMergeToLeft;
    if (token == "merge_to_right") return kTurnMergeToRight;
    return std::nullopt;
}

Result<CrossSectionBase> resolve_cross_section_base(const RoadGraphWay& way, const RoadRules& rules) {
    const auto attrs = resolve_road_attributes(way);
    if (!attrs) return attrs.error();
    Resolver rr{way, 0};
    const auto highway = rr.get("highway").value_or(std::string_view{});
    const auto* cls_ptr = rules.find_class(highway);
    if (!cls_ptr) return make_error(ErrorCode::InvalidArgument, "cross section: no rules for highway '" + std::string(highway) + "'");
    const RoadClassRules& cls = *cls_ptr;
    auto& declines = rr.declines;
    const bool unpaved = attrs->land_class == LandClass::UnpavedRoad;

    RoadCrossSection xs;

    // ---- 3.1 direction
    bool oneway = false, reversed = false, oneway_tagged = false;
    if (const auto ow = rr.get("oneway")) {
        if (*ow == "yes" || *ow == "true" || *ow == "1") { oneway = true; oneway_tagged = true; }
        else if (*ow == "-1" || *ow == "reverse") { oneway = true; reversed = true; oneway_tagged = true; }
        else if (*ow == "no" || *ow == "false" || *ow == "0") { oneway_tagged = true; /* explicit two-way */ }
    }
    if (!oneway_tagged) {
        const auto junction = rr.get("junction").value_or(std::string_view{});
        if (highway == "motorway" || highway == "motorway_link" || highway == "trunk_link" ||
            junction == "roundabout" || junction == "circular") oneway = true;
    }
    if (oneway) xs.flags |= kXsOneway;
    if (reversed) xs.flags |= kXsReversedOneway;

    // ---- 3.1 lane count and direction groups
    std::optional<std::int32_t> lanes_tag;
    if (const auto lt = rr.get("lanes")) {
        lanes_tag = parse_int(*lt, 1, 16);
        if (!lanes_tag) declines |= kDeclineLanesValue;
    }
    int n_bwd = 0, n_both = 0, n_fwd = 0;
    Provenance dir_prov = Provenance::Inferred;
    Provenance count_prov = Provenance::Inferred;
    if (oneway) {
        n_fwd = lanes_tag ? *lanes_tag : (cls.single_lane_both ? 1 : cls.oneway_lanes);
        count_prov = lanes_tag ? Provenance::Tagged : Provenance::Inferred;
        dir_prov = oneway_tagged ? Provenance::Tagged : Provenance::Inferred;
    } else {
        const auto read_count = [&](const char* key) -> std::optional<std::int32_t> {
            const auto v = rr.get(key);
            if (!v) return std::nullopt;
            const auto n = parse_int(*v, 0, 16);
            if (!n) declines |= kDeclineDirectionCounts;
            return n;
        };
        const auto f = read_count("lanes:forward");
        const auto b = read_count("lanes:backward");
        const auto bo = read_count("lanes:both_ways");
        const auto fallback_split = [&](std::int32_t total) {
            if (total == 1) { n_bwd = 0; n_both = 1; n_fwd = 0; }
            else { n_bwd = total / 2; n_both = 0; n_fwd = total - n_bwd; }
        };
        bool done = false;
        if (lanes_tag) {
            const std::int32_t total = *lanes_tag;
            count_prov = Provenance::Tagged;
            if (f && b) {
                if (*f + *b + bo.value_or(0) == total) {
                    n_fwd = *f; n_bwd = *b; n_both = bo.value_or(0); dir_prov = Provenance::Tagged; done = true;
                } else declines |= kDeclineDirectionCounts;
            } else if (f || b) {
                const std::int32_t other = total - (f ? *f : *b) - bo.value_or(0);
                if (other >= 0) {
                    n_fwd = f ? *f : other; n_bwd = b ? *b : other; n_both = bo.value_or(0);
                    dir_prov = Provenance::Derived; done = true;
                } else declines |= kDeclineDirectionCounts;
            } else if (bo) {
                const std::int32_t rest = total - *bo;
                if (rest >= 0) { n_both = *bo; n_bwd = rest / 2; n_fwd = rest - n_bwd; done = true; }
                else declines |= kDeclineDirectionCounts;
            }
            if (!done) { fallback_split(total); dir_prov = Provenance::Inferred; }
        } else {
            const std::int32_t total = f.value_or(0) + b.value_or(0) + bo.value_or(0);
            if ((f || b || bo) && total > 0) {
                n_fwd = f.value_or(0); n_bwd = b.value_or(0); n_both = bo.value_or(0);
                count_prov = Provenance::Derived; dir_prov = Provenance::Tagged; done = true;
            } else if (f || b || bo) declines |= kDeclineDirectionCounts;
            if (!done) {
                if (cls.single_lane_both) { n_both = 1; }
                else fallback_split(cls.twoway_lanes);
            }
        }
    }
    const int n_counted = n_bwd + n_both + n_fwd;
    if (n_counted <= 0) {
        CrossSectionBase base; base.decline = CrossSectionDecline{CrossSectionDeclineCode::NoCountedLane, 0};
        return base;
    }
    xs.count_provenance = count_prov;

    // Counted lanes in way order: backward, both ways, forward (oneway: all one direction).
    std::vector<CrossSectionBand> lanes(static_cast<std::size_t>(n_counted));
    std::vector<Group> groups;
    for (int i = 0; i < n_counted; ++i) {
        auto& b = lanes[static_cast<std::size_t>(i)];
        b.counted = true; b.paved = !unpaved;
        b.provenance.direction = dir_prov;
        if (!oneway && i >= n_bwd && i < n_bwd + n_both && dir_prov == Provenance::Inferred) b.provenance.direction = Provenance::Derived;
        if (oneway) b.direction = reversed ? LaneDirection::Backward : LaneDirection::Forward;
        else if (i < n_bwd) b.direction = LaneDirection::Backward;
        else if (i < n_bwd + n_both) b.direction = LaneDirection::Both;
        else b.direction = LaneDirection::Forward;
    }
    if (oneway) groups.push_back({0, n_counted, reversed, ""});
    else {
        if (n_bwd) groups.push_back({0, n_bwd, true, ":backward"});
        if (n_fwd) groups.push_back({n_bwd + n_both, n_fwd, false, ":forward"});
    }
    const auto key_of = [&](const char* base, const Group& g) { return std::string(base) + g.suffix; };

    // ---- 3.2 turn, change and bus tokens
    for (const auto& g : groups) {
        if (const auto v = rr.get(key_of("turn:lanes", g))) {
            const auto pieces = split(*v, '|');
            if (static_cast<int>(pieces.size()) != g.n) declines |= kDeclineTurnCount;
            else for (int i = 0; i < g.n; ++i) {
                bool unknown = false;
                const auto mask = parse_turn(pieces[static_cast<std::size_t>(i)], unknown);
                if (unknown) { declines |= kDeclineTurnToken; continue; }
                auto& lane = lanes[static_cast<std::size_t>(g.band(i))];
                lane.turn = mask; lane.provenance.turn = Provenance::Tagged;
            }
        }
        if (const auto v = rr.get(key_of("change:lanes", g))) {
            const auto pieces = split(*v, '|');
            if (static_cast<int>(pieces.size()) != g.n) declines |= kDeclineChangeCount;
            else for (int i = 0; i < g.n; ++i) {
                const auto token = trim(pieces[static_cast<std::size_t>(i)]);
                auto& lane = lanes[static_cast<std::size_t>(g.band(i))];
                if (token.empty()) continue;
                if (token == "yes") lane.change = LaneChange::Yes;
                else if (token == "no") lane.change = LaneChange::No;
                else if (token == "not_left") lane.change = LaneChange::NotLeft;
                else if (token == "not_right") lane.change = LaneChange::NotRight;
                else { declines |= kDeclineChangeCount; continue; }
                lane.provenance.change = Provenance::Tagged;
            }
        }
        // Bus: per-lane lists first, else a count of outermost lanes.
        bool per_lane = false;
        for (const char* base : {"bus:lanes", "psv:lanes"}) {
            const auto v = rr.get(key_of(base, g));
            if (!v) continue;
            per_lane = true;
            const auto pieces = split(*v, '|');
            if (static_cast<int>(pieces.size()) != g.n) { declines |= kDeclineBusLanesCount; continue; }
            for (int i = 0; i < g.n; ++i)
                for (const auto token : split(pieces[static_cast<std::size_t>(i)], ';'))
                    if (trim(token) == "designated") {
                        auto& lane = lanes[static_cast<std::size_t>(g.band(i))];
                        lane.kind = LaneKind::Bus; lane.provenance.kind = Provenance::Tagged;
                    }
        }
        if (!per_lane) {
            for (const char* base : {"lanes:bus", "lanes:psv"}) {
                const auto v = rr.get(key_of(base, g));
                if (!v) continue;
                const auto count = parse_int(*v, 0, g.n);
                if (!count) { declines |= kDeclineBusLanesCount; continue; }
                for (int k = 0; k < *count; ++k) { // outermost = last in travel order
                    auto& lane = lanes[static_cast<std::size_t>(g.band(g.n - 1 - k))];
                    if (lane.kind != LaneKind::Bus) { lane.kind = LaneKind::Bus; lane.provenance.kind = Provenance::Derived; }
                }
            }
        }
    }
    for (auto& lane : lanes) {
        if (lane.kind == LaneKind::Bus) continue;
        if ((lane.turn & kTurnDirectional) && !(lane.turn & (kTurnThrough | kTurnNone))) {
            lane.kind = LaneKind::Turn; lane.provenance.kind = Provenance::Tagged;
        }
    }

    // ---- 3.4 (first half) non-counted bands: needed now because `width` is the whole carriageway.
    // Each side lists its bands from the carriageway outward.
    std::vector<CrossSectionBand> side_bands[2]; // 0 = left, 1 = right (way direction)
    const auto first_lane_dir = lanes.front().direction;
    const auto last_lane_dir = lanes.back().direction;
    const auto side_dir = [&](int side) { return side == 0 ? first_lane_dir : last_lane_dir; };
    const auto kerb_side_is_inner = [&](int side) { // the median side of a oneway carriageway
        return oneway && (reversed ? side == 1 : side == 0);
    };
    const auto classify = [&](std::string_view value, std::initializer_list<std::string_view> band_values,
                              std::initializer_list<std::string_view> none_values) {
        for (const auto v : band_values) if (value == v) return SideMeaning::Band;
        for (const auto v : none_values) if (value == v) return SideMeaning::None;
        return SideMeaning::Bad;
    };
    // The value a side reads: left -> base:left, base:both; right -> base:right, base:both.
    const auto side_value = [&](const std::string& base, int side) -> std::optional<std::string_view> {
        if (const auto v = rr.get(base + (side == 0 ? ":left" : ":right"))) return v;
        return rr.get(base + ":both");
    };

    // Cycle lanes.
    {
        const std::initializer_list<std::string_view> kNoBand = {"no", "none", "track", "separate", "shared_lane",
            "share_busway", "shared_busway", "opposite", "opposite_track", "opposite_share_busway"};
        const auto bare = rr.get("cycleway");
        for (int side = 0; side < 2; ++side) {
            auto v = side_value("cycleway", side);
            if (!v && bare) {
                // Bare cycleway=*: both sides of a two-way road, the travel-right side of a oneway.
                const bool travel_right = reversed ? side == 0 : side == 1;
                if (!oneway || travel_right) v = bare;
            }
            if (!v) continue;
            switch (classify(*v, {"lane"}, kNoBand)) {
            case SideMeaning::Band: {
                auto b = make_band(LaneKind::Cycle, rules.cycle_lane_mm, !unpaved);
                b.direction = side_dir(side); side_bands[side].push_back(b); break; }
            case SideMeaning::None: break;
            case SideMeaning::Bad: declines |= kDeclineCyclewayValue; break;
            }
        }
    }
    // Parking (the new parking:<side> scheme only).
    {
        if (rr.starts_with_key("parking:lane")) declines |= kDeclineParkingScheme;
        for (int side = 0; side < 2; ++side) {
            const auto v = side_value("parking", side);
            if (!v) continue;
            switch (classify(*v, {"lane"}, {"no", "separate", "street_side", "on_kerb", "half_on_kerb"})) {
            case SideMeaning::Band: {
                auto orientation = ParkingOrientation::Parallel;
                auto o = rr.get(std::string("parking:") + (side == 0 ? "left" : "right") + ":orientation");
                if (!o) o = rr.get("parking:both:orientation");
                if (o) {
                    if (*o == "parallel") orientation = ParkingOrientation::Parallel;
                    else if (*o == "diagonal") orientation = ParkingOrientation::Diagonal;
                    else if (*o == "perpendicular") orientation = ParkingOrientation::Perpendicular;
                    else declines |= kDeclineParkingScheme;
                }
                auto b = make_band(LaneKind::Parking, rules.parking_width_mm(orientation), !unpaved);
                b.direction = side_dir(side); side_bands[side].push_back(b); break; }
            case SideMeaning::None: break;
            case SideMeaning::Bad: declines |= kDeclineParkingScheme; break;
            }
        }
    }
    // Carriageway width of the on-carriageway bands (everything pushed so far).
    std::int64_t onroad_mm = 0;
    for (const auto& side : side_bands) for (const auto& b : side) onroad_mm += b.width_mm;

    // ---- 3.3 widths of counted lanes
    std::vector<std::optional<std::int32_t>> lane_width(static_cast<std::size_t>(n_counted));
    for (const auto& g : groups) {
        const auto v = rr.get(key_of("width:lanes", g));
        if (!v) continue;
        const auto pieces = split(*v, '|');
        if (static_cast<int>(pieces.size()) != g.n) { declines |= kDeclineWidthLanesCount; continue; }
        std::vector<std::optional<std::int32_t>> parsed(pieces.size());
        bool bad = false;
        for (std::size_t i = 0; i < pieces.size() && !bad; ++i) {
            const auto piece = trim(pieces[i]);
            if (piece.empty()) continue;
            parsed[i] = parse_road_width_mm(piece);
            if (!parsed[i]) bad = true;
        }
        if (bad) { declines |= kDeclineWidthValue; continue; }
        for (int i = 0; i < g.n; ++i) {
            const auto& p = parsed[static_cast<std::size_t>(i)];
            if (!p) continue;
            lane_width[static_cast<std::size_t>(g.band(i))] = p;
            lanes[static_cast<std::size_t>(g.band(i))].provenance.width = Provenance::Tagged;
        }
    }
    if (const auto wtag = rr.get("width")) {
        const auto carriageway = parse_road_width_mm(*wtag);
        if (!carriageway) declines |= kDeclineWidthValue;
        else {
            std::vector<std::size_t> open;
            std::int64_t known = 0;
            for (std::size_t i = 0; i < lane_width.size(); ++i) {
                if (lane_width[i]) known += *lane_width[i]; else open.push_back(i);
            }
            if (!open.empty()) {
                const std::int64_t total = *carriageway - onroad_mm - known;
                const std::int64_t per = total / static_cast<std::int64_t>(open.size());
                if (total < 0 || per < rules.min_lane_mm) declines |= kDeclineWidthInconsistent;
                else {
                    const std::int64_t remainder = total - per * static_cast<std::int64_t>(open.size());
                    for (std::size_t k = 0; k < open.size(); ++k) {
                        const bool gets_extra = static_cast<std::int64_t>(open.size() - k) <= remainder; // rightmost first
                        lane_width[open[k]] = static_cast<std::int32_t>(per + (gets_extra ? 1 : 0));
                        lanes[open[k]].provenance.width = Provenance::Derived;
                    }
                }
            }
        }
    }
    for (std::size_t i = 0; i < lane_width.size(); ++i) {
        if (lane_width[i]) { lanes[i].width_mm = *lane_width[i]; continue; }
        std::int32_t w = cls.lane_mm;
        if (cls.single_lane_both) w = n_counted > 1 ? std::max(rules.min_lane_mm, cls.single_lane_mm / n_counted) : cls.single_lane_mm;
        else if (oneway && cls.outer_lane_mm > 0 && static_cast<int>(i) == (reversed ? 0 : n_counted - 1)) w = cls.outer_lane_mm;
        lanes[i].width_mm = w; lanes[i].provenance.width = Provenance::Inferred;
    }

    // ---- 3.4 (second half) shoulder, sidewalk, verge
    std::optional<std::string_view> shoulder_tag = rr.get("shoulder");
    bool shoulder_present[2];
    for (int side = 0; side < 2; ++side)
        shoulder_present[side] = (kerb_side_is_inner(side) ? cls.shoulder_inner_mm : cls.shoulder_outer_mm) > 0;
    if (shoulder_tag) {
        if (*shoulder_tag == "yes" || *shoulder_tag == "both") shoulder_present[0] = shoulder_present[1] = true;
        else if (*shoulder_tag == "left") { shoulder_present[0] = true; shoulder_present[1] = false; }
        else if (*shoulder_tag == "right") { shoulder_present[0] = false; shoulder_present[1] = true; }
        else if (*shoulder_tag == "no") shoulder_present[0] = shoulder_present[1] = false;
    }
    const bool shoulder_kind_tagged = shoulder_tag && (*shoulder_tag == "yes" || *shoulder_tag == "both" || *shoulder_tag == "left" ||
                                                       *shoulder_tag == "right" || *shoulder_tag == "no");
    bool sidewalk_present[2] = {false, false};
    {
        const auto bare = rr.get("sidewalk");
        bool bare_bad = false;
        bool bare_side[2] = {false, false};
        if (bare) {
            if (*bare == "both") bare_side[0] = bare_side[1] = true;
            else if (*bare == "left") bare_side[0] = true;
            else if (*bare == "right") bare_side[1] = true;
            else if (*bare == "no" || *bare == "none" || *bare == "separate") {}
            else bare_bad = true;
        }
        bool any_bad = bare_bad;
        for (int side = 0; side < 2; ++side) {
            const auto v = side_value("sidewalk", side);
            if (v) {
                switch (classify(*v, {"yes"}, {"no", "none", "separate"})) {
                case SideMeaning::Band: sidewalk_present[side] = true; break;
                case SideMeaning::None: break;
                case SideMeaning::Bad: any_bad = true; break;
                }
            } else sidewalk_present[side] = bare_side[side];
        }
        if (any_bad) declines |= kDeclineSidewalkValue;
    }
    for (int side = 0; side < 2; ++side) {
        if (shoulder_present[side]) {
            std::int32_t w = kerb_side_is_inner(side) ? cls.shoulder_inner_mm : cls.shoulder_outer_mm;
            if (w == 0) w = rules.tagged_shoulder_mm;
            auto b = make_band(LaneKind::Shoulder, w, !unpaved);
            b.provenance.kind = shoulder_kind_tagged ? Provenance::Tagged : Provenance::Inferred;
            side_bands[side].push_back(b);
        }
        if (sidewalk_present[side]) {
            side_bands[side].push_back(make_band(LaneKind::Sidewalk, rules.sidewalk_mm, !unpaved));
        } else if (cls.verge_mm > 0) {
            auto b = make_band(LaneKind::Verge, cls.verge_mm, false);
            b.provenance.kind = Provenance::Inferred;
            side_bands[side].push_back(b);
        }
    }

    // ---- markings
    {
        const auto marking = rr.get("lane_markings");
        if (marking) xs.flags |= kXsMarkedTagged;
        const bool marked = marking ? *marking != "no" && (*marking == "yes" || cls.markings_default)
                                    : cls.markings_default;
        for (int i = 0; i + 1 < n_counted; ++i) lanes[static_cast<std::size_t>(i)].marked_right = marked;
    }

    // ---- assemble bands: left side outermost first, counted lanes, right side outward
    const int left_count = static_cast<int>(side_bands[0].size());
    xs.bands.reserve(side_bands[0].size() + lanes.size() + side_bands[1].size());
    for (auto it = side_bands[0].rbegin(); it != side_bands[0].rend(); ++it) xs.bands.push_back(*it);
    for (const auto& l : lanes) xs.bands.push_back(l);
    for (const auto& b : side_bands[1]) xs.bands.push_back(b);

    // ---- 3.5 placement anchor
    {
        PlacementAnchor anchor;
        const int k = (n_counted % 2 == 1) ? (n_counted - 1) / 2 : n_counted / 2 - 1;
        anchor.band = static_cast<std::uint8_t>(left_count + k);
        anchor.edge = (n_counted % 2 == 1) ? AnchorEdge::Centre : AnchorEdge::Right;
        anchor.source = PlacementSource::Absent;

        struct Resolved { bool ok = false; bool transition = false; int band = 0; AnchorEdge edge = AnchorEdge::Centre; };
        const auto resolve_for = [&](const std::string& key, const Group* g) -> Resolved {
            Resolved out;
            const auto v = rr.get(key);
            if (!v) return out;
            const auto p = parse_placement(*v);
            if (!p) { declines |= kDeclinePlacementValue; return out; }
            if (p->transition) { out.ok = true; out.transition = true; return out; }
            if (!g || p->group_lane < 1 || p->group_lane > g->n) { declines |= kDeclinePlacementRange; return out; }
            out.ok = true;
            out.band = left_count + g->band(p->group_lane - 1);
            out.edge = p->edge;
            if (g->mirrored) out.edge = p->edge == AnchorEdge::Left ? AnchorEdge::Right
                                         : p->edge == AnchorEdge::Right ? AnchorEdge::Left : AnchorEdge::Centre;
            return out;
        };
        Resolved chosen;
        if (oneway) chosen = resolve_for("placement", &groups.front());
        else {
            const Group* fwd = nullptr; const Group* bwd = nullptr;
            for (const auto& g : groups) (g.mirrored ? bwd : fwd) = &g;
            const auto f = resolve_for("placement:forward", fwd);
            const auto b = resolve_for("placement:backward", bwd);
            const auto bare = rr.get("placement");
            Resolved plain;
            if (bare) {
                if (*bare == "transition") { plain.ok = true; plain.transition = true; }
                else declines |= kDeclinePlacementValue;
            }
            chosen = f.ok ? f : (b.ok ? b : plain);
            // The same lateral position seen from both groups (left of band i = right of band i-1) is no conflict.
            const auto position = [](const Resolved& x) { return 2 * x.band + (x.edge == AnchorEdge::Left ? 0 : x.edge == AnchorEdge::Centre ? 1 : 2); };
            if (f.ok && b.ok && (f.transition != b.transition || (!f.transition && position(f) != position(b))))
                declines |= kDeclinePlacementValue;
        }
        if (chosen.ok) {
            if (chosen.transition) anchor.source = PlacementSource::Transition;
            else {
                anchor.source = PlacementSource::Explicit;
                anchor.band = static_cast<std::uint8_t>(chosen.band);
                anchor.edge = chosen.edge;
            }
        }
        xs.anchor = anchor;
        xs.placement_provenance = anchor.source == PlacementSource::Explicit ? Provenance::Tagged : Provenance::Inferred;
    }
    xs.field_declines = declines;

    // ---- 3.6 whole-section declines
    CrossSectionBase base;
    if (xs.bands.size() > 32) {
        base.decline = CrossSectionDecline{CrossSectionDeclineCode::TooManyBands, static_cast<std::int32_t>(xs.bands.size())};
        return base;
    }
    std::int64_t total_mm = 0;
    for (const auto& b : xs.bands) total_mm += b.width_mm;
    if (total_mm > 100000) {
        base.decline = CrossSectionDecline{CrossSectionDeclineCode::TotalWidthLimit, static_cast<std::int32_t>(std::min<std::int64_t>(total_mm, 2147483647))};
        return base;
    }
    base.xs = std::move(xs);
    return base;
}
} // namespace g2m
