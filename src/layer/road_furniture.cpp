// Street fidelity P2 slice 7: junction furniture (design section 7). See road_furniture.h for the contract.
// Everything here is + - * / on mm-rounded inputs plus det::sincos for a post's facing; no unordered iteration decides
// an output (maps are ordered, the result is sorted by a total key).
#include "g2m/layer/road_furniture.h"
#include "g2m/core/det_math.h"
#include "g2m/layer/road_attributes.h"
#include "g2m/layer/road_cross_section.h"
#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <optional>
#include <string>
#include <string_view>
#include <tuple>

namespace g2m {
namespace {
constexpr std::size_t kNoEntry = std::numeric_limits<std::size_t>::max();
constexpr double kLaneGapM = 0.0005;     // lanes closer than this form one contiguous group
constexpr std::size_t kMaxLanes = 32;

double mm_round(double metres) { return static_cast<double>(std::llround(metres * 1000.0)) / 1000.0; }
double mm_to_m(std::int32_t mm) { return static_cast<double>(mm) / 1000.0; }
LaneDirection opposite(LaneDirection d) { return d == LaneDirection::Forward ? LaneDirection::Backward : LaneDirection::Forward; }

const std::string* tag_of(const RoadGraphNode& node, std::string_view key) {
    for (const auto& [k, v] : node.tags) if (k == key) return &v;
    return nullptr;
}
const RoadGraphNode* find_node(const RoadGraph& g, std::int64_t id) {
    const auto it = std::lower_bound(g.nodes.begin(), g.nodes.end(), id, [](const RoadGraphNode& n, std::int64_t v) { return n.osm_id < v; });
    return it == g.nodes.end() || it->osm_id != id ? nullptr : &*it;
}
const RoadGraphWay* find_way(const RoadGraph& g, std::int64_t id) {
    const auto it = std::lower_bound(g.ways.begin(), g.ways.end(), id, [](const RoadGraphWay& w, std::int64_t v) { return w.osm_id < v; });
    return it == g.ways.end() || it->osm_id != id ? nullptr : &*it;
}

enum class Control : std::uint8_t { None, Signal, Stop, GiveWay };
struct NodeInfo {
    const RoadGraphNode* node = nullptr;
    bool crossing = false;
    Control control = Control::None;
};
NodeInfo classify(const RoadGraphNode& node) {
    NodeInfo info; info.node = &node;
    const std::string* highway = tag_of(node, "highway");
    if (!highway) return info;
    if (*highway == "crossing") {
        info.crossing = true;
        const std::string* kind = tag_of(node, "crossing");
        if (kind && *kind == "traffic_signals") info.control = Control::Signal;   // a signalised crossing is also a signal (7.2)
    } else if (*highway == "traffic_signals") info.control = Control::Signal;
    else if (*highway == "stop") info.control = Control::Stop;
    else if (*highway == "give_way") info.control = Control::GiveWay;
    return info;
}

struct Item { std::int64_t way = 0; std::uint32_t ref = 0; bool at_end = false; };
enum class Role : std::uint8_t { Absent, Interior, Boundary, Junction };
struct Chosen {
    Role role = Role::Absent;
    std::size_t entry = kNoEntry;     // kNoEntry: the chosen stretch is not an entry of this tile
    double station = 0;
    bool at_end = false;              // Boundary/Junction: the chosen stretch ends (not starts) at the node
    std::uint32_t degree = 0;
};

struct Group { double lo = 0, hi = 0; };

class Builder {
public:
    Builder(const RoadGeomTile& tile, const RoadReferenceBuilder& context, std::span<const RoadJunction> junctions,
        const RoadRules& rules, FurnitureBuildStats* stats)
        : tile_(tile), graph_(tile.source.graph), ctx_(context), junctions_(junctions), fr_(rules.furniture), stats_(stats) {}

    Result<std::vector<RoadFurniture>> run() {
        index_entries();
        std::vector<NodeInfo> nodes;
        for (const auto& n : graph_.nodes) {
            const NodeInfo info = classify(n);
            if (info.crossing || info.control != Control::None) nodes.push_back(info);
        }
        index_incident(nodes);
        for (const auto& info : nodes) if (info.crossing) crossing(info);
        for (const auto& info : nodes) if (info.control != Control::None) entry_control(info);
        junction_controls();
        std::stable_sort(out_.begin(), out_.end(), [](const RoadFurniture& a, const RoadFurniture& b) {
            return std::make_tuple(a.junction_index, a.entry_index, static_cast<int>(a.kind), a.station_m, a.node_id, static_cast<int>(a.travel), a.t_from, a.flags) <
                   std::make_tuple(b.junction_index, b.entry_index, static_cast<int>(b.kind), b.station_m, b.node_id, static_cast<int>(b.travel), b.t_from, b.flags);
        });
        return std::move(out_);
    }

private:
    const RoadGeomTile& tile_;
    const RoadGraph& graph_;
    const RoadReferenceBuilder& ctx_;
    std::span<const RoadJunction> junctions_;
    const FurnitureRules& fr_;
    FurnitureBuildStats* stats_;
    std::map<std::int64_t, std::vector<std::size_t>> by_way_;
    std::map<std::int64_t, std::vector<Item>> incident_;
    std::vector<RoadFurniture> out_;
    struct PlacedCrossing { std::size_t entry; double station, length; };
    std::vector<PlacedCrossing> crossings_;
    struct ControlLine { std::size_t entry; LaneDirection travel; double station; bool signal; };
    std::vector<ControlLine> control_lines_;

    void index_entries() {
        for (std::size_t i = 0; i < tile_.entries.size(); ++i) by_way_[tile_.entries[i].way_id].push_back(i);
    }
    void index_incident(const std::vector<NodeInfo>& nodes) {
        for (const auto& n : nodes) incident_[n.node->osm_id];
        for (const auto& way : ctx_.graph().ways) {
            for (std::size_t i = 0; i < way.node_ids.size(); ++i) {
                const auto it = incident_.find(way.node_ids[i]);
                if (it == incident_.end()) continue;
                if (i > 0) it->second.push_back({way.osm_id, static_cast<std::uint32_t>(i), true});
                if (i + 1 < way.node_ids.size()) it->second.push_back({way.osm_id, static_cast<std::uint32_t>(i), false});
            }
        }
    }

    // ---- entry helpers ----
    FurnitureDecline entry_state(std::size_t e) const {
        const auto& entry = tile_.entries[e];
        if (const auto* way = find_way(graph_, entry.way_id)) {
            const auto attrs = resolve_road_attributes(*way);
            if (attrs && attrs->tunnel) return FurnitureDecline::Tunnel;
        }
        return entry.profile && entry.cross_section ? FurnitureDecline::None : FurnitureDecline::StretchDeclined;
    }
    double length_of(std::size_t e) const { return tile_.entries[e].profile->reference.length_m; }
    std::size_t find_entry(std::int64_t way, std::uint32_t ref, bool at_end) const {
        const auto it = by_way_.find(way);
        if (it == by_way_.end()) return kNoEntry;
        for (const auto e : it->second) {
            const auto& s = tile_.entries[e].stretch;
            if (at_end ? s.end_ref == ref : s.start_ref == ref) return e;
        }
        return kNoEntry;
    }

    Chosen choose(std::int64_t id) const {
        Chosen c;
        const auto it = incident_.find(id);
        if (it == incident_.end() || it->second.empty()) return c;
        const auto& items = it->second;
        c.degree = ctx_.degree(id);
        if (c.degree == 0) return c;
        if (c.degree == 2 && items.size() == 2 && items[0].way == items[1].way && items[0].ref == items[1].ref) {
            c.role = Role::Interior;
            const auto ways = by_way_.find(items[0].way);
            if (ways == by_way_.end()) return c;
            for (const auto e : ways->second) {
                const auto& s = tile_.entries[e].stretch;
                if (s.start_ref < items[0].ref && items[0].ref < s.end_ref) { c.entry = e; break; }
            }
            return c;
        }
        c.role = c.degree >= 3 ? Role::Junction : Role::Boundary;
        const Item* best = nullptr;
        const auto key = [&](const Item& item) {
            const int prefer = c.degree == 2 ? (item.at_end ? 1 : 0) : 0;   // a degree-two join: the stretch that starts at the node
            return std::make_tuple(prefer, item.way, item.ref, item.at_end ? 1 : 0);
        };
        for (const auto& item : items) if (!best || key(item) < key(*best)) best = &item;
        c.at_end = best->at_end;
        c.entry = find_entry(best->way, best->ref, best->at_end);
        if (c.entry != kNoEntry && tile_.entries[c.entry].profile) c.station = best->at_end ? length_of(c.entry) : 0.0;
        return c;
    }

    // ---- lanes ----
    struct LaneInfo { std::size_t count = 0; std::vector<Group> groups; };
    LaneInfo lanes_at(std::size_t e, double station, LaneDirection travel) const {
        LaneInfo info;
        const auto& entry = tile_.entries[e];
        const double s = std::clamp(station, 0.0, entry.profile->reference.length_m);
        std::array<LaneQuery, kMaxLanes> buf{};
        info.count = driving_lanes_at(*entry.cross_section, s, travel, buf);
        std::vector<Group> spans;
        for (std::size_t i = 0; i < info.count; ++i) spans.push_back({buf[i].centre_t - buf[i].width_m / 2, buf[i].centre_t + buf[i].width_m / 2});
        std::sort(spans.begin(), spans.end(), [](const Group& a, const Group& b) { return a.lo < b.lo; });
        for (const auto& span : spans) {
            if (!info.groups.empty() && span.lo - info.groups.back().hi <= kLaneGapM) info.groups.back().hi = std::max(info.groups.back().hi, span.hi);
            else info.groups.push_back(span);
        }
        return info;
    }

    void emit(const RoadFurniture& f) { out_.push_back(f); }
    void count_line_decline(FurnitureDecline d) { if (stats_) ++stats_->line_declined[static_cast<std::size_t>(d)]; }

    // ---- crossings (7.1) ----
    void crossing(const NodeInfo& info) {
        const auto& node = *info.node;
        if (stats_) ++stats_->crossing_nodes;
        const Chosen c = choose(node.osm_id);
        if (c.role == Role::Absent || c.entry == kNoEntry) { if (stats_) ++stats_->crossing_not_ours; return; }
        RoadFurniture r;
        r.kind = FurnitureKind::Crossing; r.node_id = node.osm_id; r.entry_index = c.entry; r.travel = LaneDirection::Forward;
        const auto decline = [&](FurnitureDecline d) { r.decline = d; emit(r); if (stats_) ++stats_->crossing_declined[static_cast<std::size_t>(d)]; };
        if (const auto state = entry_state(c.entry); state != FurnitureDecline::None) { decline(state); return; }
        const auto& entry = tile_.entries[c.entry];
        if (c.role == Role::Junction) { r.station_m = c.station; decline(FurnitureDecline::JunctionNode); return; }
        double station = c.station;
        if (c.role == Role::Interior) {
            const auto* way = find_way(graph_, entry.way_id);
            const auto s = way ? road_node_station(entry.profile->reference, ownership_ref(*way, node.osm_id, entry), node.position) : std::nullopt;
            if (!s) { decline(FurnitureDecline::StretchDeclined); return; }
            station = *s;
        }
        const double length = entry.profile->reference.length_m;
        const auto edges = carriageway_edges(*entry.cross_section, std::clamp(station, 0.0, length));
        // Pattern: crossing:markings first, then the crossing=* kind (design 7.1 rules 1-5); anything else paints nothing.
        CrossingPattern pattern = CrossingPattern::None;
        bool decided = false;
        if (const std::string* v = tag_of(node, "crossing:markings")) {
            if (const CrossingPattern* p = fr_.find_marking(*v)) { pattern = *p; decided = true; }
        }
        const std::string* kind = tag_of(node, "crossing");
        if (!decided && kind) { if (const CrossingPattern* p = fr_.find_kind(*kind)) pattern = *p; }
        const bool signalised = kind && *kind == "traffic_signals";
        std::int32_t length_mm = pattern == CrossingPattern::Zebra ? fr_.crossing_zebra_mm : signalised ? fr_.crossing_signalised_mm : fr_.crossing_other_mm;
        if (const std::string* w = tag_of(node, "width")) {
            if (const auto parsed = parse_road_width_mm(*w); parsed && *parsed >= 1000 && *parsed <= 10000) length_mm = *parsed;
        }
        r.station_m = station; r.t_from = mm_round(edges.right); r.t_to = mm_round(edges.left); r.length_m = mm_to_m(length_mm);
        r.pattern = pattern;
        if (signalised) r.flags |= kFurnitureSignalised;
        if (const std::string* k = tag_of(node, "kerb"); k && (*k == "lowered" || *k == "flush")) r.flags |= kFurnitureLoweredKerb;
        if (c.role == Role::Boundary && c.degree == 2) r.flags |= kFurnitureSpansJoin;
        emit(r);
        crossings_.push_back({c.entry, station, r.length_m});
        if (stats_) { ++stats_->crossing_placed; ++stats_->crossing_patterns[static_cast<std::size_t>(pattern)]; }
    }
    // The way ref index of an interior node inside the entry's stretch.
    static std::uint32_t ownership_ref(const RoadGraphWay& way, std::int64_t node, const RoadGeomEntry& entry) {
        for (std::uint32_t i = entry.stretch.start_ref + 1; i < entry.stretch.end_ref; ++i) if (way.node_ids[i] == node) return i;
        return entry.stretch.start_ref + 1;
    }

    // ---- directions (7.2) ----
    struct Directions { bool forward = false, backward = false; bool resolved = false; };
    static std::optional<Directions> direction_tag(const RoadGraphNode& node) {
        for (const char* key : {"traffic_signals:direction", "direction"}) {
            if (const std::string* v = tag_of(node, key)) {
                if (*v == "forward") return Directions{true, false, true};
                if (*v == "backward") return Directions{false, true, true};
                if (*v == "both") return Directions{true, true, true};
            }
        }
        return std::nullopt;
    }
    Directions resolve_directions(const RoadGraphNode& node, std::size_t e, double station, bool sign) const {
        if (const auto tagged = direction_tag(node)) return *tagged;
        const auto& entry = tile_.entries[e];
        if (entry.cross_section->flags & kXsOneway) {
            const bool reversed = (entry.cross_section->flags & kXsReversedOneway) != 0;
            return Directions{!reversed, reversed, true};
        }
        const double assoc = static_cast<double>(sign ? fr_.sign_assoc_max_m : fr_.signal_assoc_max_m);
        const auto* way = find_way(graph_, entry.way_id);
        const double length = length_of(e);
        double d_start = std::numeric_limits<double>::infinity(), d_end = d_start;
        if (way) {
            if (ctx_.degree(way->node_ids[entry.stretch.start_ref]) >= 3 && station <= assoc) d_start = station;
            if (ctx_.degree(way->node_ids[entry.stretch.end_ref]) >= 3 && length - station <= assoc) d_end = length - station;
        }
        const bool near_start = d_start <= assoc, near_end = d_end <= assoc;
        if (near_start && near_end) {
            if (d_end < d_start) return Directions{true, false, true};     // towards the end junction
            if (d_start < d_end) return Directions{false, true, true};
            return Directions{true, true, true};
        }
        if (near_end) return Directions{true, false, true};
        if (near_start) return Directions{false, true, true};
        return Directions{true, true, !sign};                               // signals default to both, a sign stays unresolved
    }

    // ---- lines and signals (7.2, 7.3) ----
    struct LineSpec {
        std::size_t entry = 0;
        LaneDirection travel = LaneDirection::Forward;
        double station = 0;
        FurnitureKind kind = FurnitureKind::StopLine;
        double width_m = 0;
        bool signal = false;          // also a signal post
        bool crossing_rule = true;
        std::int64_t node = 0;
        std::uint32_t junction = 0;
    };
    double before_crossing(const LineSpec& s, double station) {
        const double before = mm_to_m(fr_.stop_line_before_crossing_mm);
        double result = station;
        for (const auto& c : crossings_) {
            if (c.entry != s.entry) continue;
            const double c0 = c.station - c.length / 2, c1 = c.station + c.length / 2;
            if (s.travel == LaneDirection::Forward) {
                if (c0 - station <= before + s.width_m && c1 > station) result = std::min(result, c0 - before);
            } else {
                if (station - c1 <= before + s.width_m && c0 < station) result = std::max(result, c1 + before);
            }
        }
        return result;
    }
    // Places the line (one record per contiguous lane group) and its signal posts. Returns true iff a line was placed.
    bool place_line_set(const LineSpec& s) {
        const auto& entry = tile_.entries[s.entry];
        const double length = length_of(s.entry);
        double station = std::clamp(s.station, 0.0, length);
        RoadFurniture base;
        base.kind = s.kind; base.node_id = s.node; base.entry_index = s.entry; base.travel = s.travel; base.junction_index = s.junction;
        LaneInfo lanes = lanes_at(s.entry, station, s.travel);
        if (lanes.count == 0) {
            base.station_m = station; base.decline = FurnitureDecline::NoLanes; emit(base); count_line_decline(FurnitureDecline::NoLanes);
            return false;
        }
        if (s.crossing_rule) {
            const double moved = std::clamp(before_crossing(s, station), 0.0, length);
            // The line moves only onto a station that has lanes of its travel (D31): a record's [t_from, t_to] must be a
            // lane group AT its own station (the acceptance rule of 7.2, checked at decode by 9.2). Slice 7 kept the
            // moved station with the lanes of the original one. Without lanes there the line stays where it was.
            if (moved != station) {
                LaneInfo at_moved = lanes_at(s.entry, moved, s.travel);
                if (at_moved.count != 0) { station = moved; lanes = std::move(at_moved); if (stats_) ++stats_->moved_before_crossing; }
                else if (stats_) ++stats_->kept_without_lanes;
            }
        }
        const double width = mm_round(s.width_m);
        for (const auto& g : lanes.groups) {
            RoadFurniture r = base;
            r.station_m = station; r.t_from = mm_round(g.lo); r.t_to = mm_round(g.hi); r.length_m = width;
            emit(r);
            if (stats_) {
                if (s.signal) ++stats_->lines_signal; else if (s.kind == FurnitureKind::GiveWayLine) ++stats_->lines_give_way; else ++stats_->lines_stop;
                if (s.junction) ++stats_->junction_lines;
            }
        }
        if (stats_) ++stats_->line_sets;
        if (s.signal) {
            const auto edges = carriageway_edges(*entry.cross_section, station);
            const double offset = mm_to_m(fr_.signal_lateral_offset_mm);
            const bool forward = s.travel == LaneDirection::Forward;
            RoadFurniture post = base;
            post.kind = FurnitureKind::Signal; post.station_m = station; post.length_m = 0;
            post.t_from = post.t_to = mm_round(forward ? edges.right - offset : edges.left + offset);
            emit(post);
            if (stats_) ++stats_->signals;
            if (static_cast<std::int32_t>(lanes.count) >= fr_.signal_left_min_lanes && lanes_at(s.entry, station, opposite(s.travel)).count == 0) {
                RoadFurniture left = post;
                left.t_from = left.t_to = mm_round(forward ? edges.left + offset : edges.right - offset);
                left.flags |= kFurnitureLeftPost;
                emit(left);
                if (stats_) { ++stats_->signals; ++stats_->left_posts; }
            }
        }
        control_lines_.push_back({s.entry, s.travel, station, s.signal});
        return true;
    }

    FurnitureKind line_kind(Control c) const { return c == Control::GiveWay ? FurnitureKind::GiveWayLine : FurnitureKind::StopLine; }
    double width_of(Control c) const {
        return mm_to_m(c == Control::Signal ? fr_.line_signal_mm : c == Control::Stop ? fr_.line_stop_mm : fr_.line_give_way_mm);
    }

    void entry_control(const NodeInfo& info) {
        const auto& node = *info.node;
        const bool sign = info.control != Control::Signal;
        const Chosen c = choose(node.osm_id);
        if (c.role == Role::Junction) {                             // the junction pass handles (and counts) it
            if (stats_ && !listed_junction_holds(node.osm_id)) ++stats_->junction_controls_not_owned;
            return;
        }
        if (stats_) { if (sign) ++stats_->sign_nodes; else ++stats_->signal_nodes; }
        if (c.role == Role::Absent || c.entry == kNoEntry) { if (stats_) ++stats_->controls_not_ours; return; }
        RoadFurniture r;
        r.kind = info.control == Control::Signal ? FurnitureKind::Signal : line_kind(info.control);
        r.node_id = node.osm_id; r.entry_index = c.entry;
        const auto decline = [&](FurnitureDecline d) { r.decline = d; emit(r); count_line_decline(d); };
        if (const auto state = entry_state(c.entry); state != FurnitureDecline::None) { decline(state); return; }
        const auto& entry = tile_.entries[c.entry];
        double station = c.station;
        if (c.role == Role::Interior) {
            const auto* way = find_way(graph_, entry.way_id);
            const auto s = way ? road_node_station(entry.profile->reference, ownership_ref(*way, node.osm_id, entry), node.position) : std::nullopt;
            if (!s) { decline(FurnitureDecline::StretchDeclined); return; }
            station = *s;
        }
        r.station_m = station;
        const Directions d = resolve_directions(node, c.entry, station, sign);
        if (!d.resolved) { decline(FurnitureDecline::DirectionUnresolved); return; }
        for (const bool forward : {true, false}) {
            if (!(forward ? d.forward : d.backward)) continue;
            LineSpec spec;
            spec.entry = c.entry; spec.travel = forward ? LaneDirection::Forward : LaneDirection::Backward; spec.station = station;
            spec.kind = line_kind(info.control); spec.width_m = width_of(info.control); spec.signal = info.control == Control::Signal;
            spec.node = node.osm_id;
            place_line_set(spec);
        }
    }

    // ---- junction-derived lines and signals ----
    bool covered_by_entry_node(std::size_t entry, LaneDirection travel, bool at_end, bool signal_case) const {
        const double length = length_of(entry);
        for (const auto& l : control_lines_) {
            if (l.entry != entry || l.travel != travel) continue;
            const double dist = at_end ? length - l.station : l.station;
            const double assoc = static_cast<double>(l.signal ? fr_.signal_assoc_max_m : fr_.sign_assoc_max_m);
            if (signal_case && !l.signal) continue;
            if (dist <= assoc) return true;
        }
        return false;
    }
    bool listed_junction_holds(std::int64_t node) const {
        for (const auto& jn : junctions_)
            if (jn.decline != JunctionDecline::ClusterTooLarge && std::binary_search(jn.nodes.begin(), jn.nodes.end(), node)) return true;
        return false;
    }
    void junction_controls() {
        for (std::size_t j = 0; j < junctions_.size(); ++j) {
            const RoadJunction& jn = junctions_[j];
            // A ClusterTooLarge record is not the junction of its nodes: every member keeps its own single-node record (D19),
            // which places or declines the member's controls. Reading the controls here as well emitted a second,
            // contradicting JunctionDeclined record per control node (D31).
            if (jn.decline == JunctionDecline::ClusterTooLarge) continue;
            std::vector<NodeInfo> controls;
            for (const auto id : jn.nodes) {
                const auto* node = find_node(graph_, id);
                if (!node) continue;
                const NodeInfo info = classify(*node);
                if (info.control != Control::None) controls.push_back(info);
            }
            if (controls.empty()) continue;
            const auto junction_index = static_cast<std::uint32_t>(j + 1);
            const std::int64_t owner = jn.nodes.front();
            if (stats_) for (const auto& c : controls) { if (c.control == Control::Signal) ++stats_->signal_nodes; else ++stats_->sign_nodes; }
            if (jn.decline != JunctionDecline::None) {
                for (const auto& c : controls) {
                    RoadFurniture r;
                    r.kind = c.control == Control::Signal ? FurnitureKind::Signal : line_kind(c.control);
                    r.decline = FurnitureDecline::JunctionDeclined; r.node_id = c.node->osm_id; r.junction_index = junction_index;
                    emit(r); count_line_decline(FurnitureDecline::JunctionDeclined);
                }
                continue;
            }
            std::vector<const JunctionApproach*> used;
            for (const auto& a : jn.approaches) {
                if (a.status != JunctionApproachStatus::Used) continue;
                const auto& e = tile_.entries[a.entry_index];
                if (e.profile && e.cross_section) used.push_back(&a);
            }
            const bool signals = std::any_of(controls.begin(), controls.end(), [](const NodeInfo& c) { return c.control == Control::Signal; });
            if (signals) {
                for (const auto* a : used) approach_line(jn, *a, junction_index, owner, Control::Signal, mm_to_m(fr_.junction_stop_setback_mm), true);
                continue;
            }
            int max_rank = std::numeric_limits<int>::min();
            for (const auto* a : used) {
                const auto* way = find_way(graph_, tile_.entries[a->entry_index].way_id);
                if (way) max_rank = std::max(max_rank, way->rank);
            }
            const bool any_lower = std::any_of(used.begin(), used.end(), [&](const JunctionApproach* a) {
                const auto* way = find_way(graph_, tile_.entries[a->entry_index].way_id);
                return way && way->rank < max_rank;
            });
            if (!any_lower) {
                for (const auto& c : controls) {
                    RoadFurniture r;
                    r.kind = line_kind(c.control); r.decline = FurnitureDecline::Ambiguous; r.node_id = c.node->osm_id; r.junction_index = junction_index;
                    emit(r); count_line_decline(FurnitureDecline::Ambiguous);
                }
                continue;
            }
            const Control kind = std::any_of(controls.begin(), controls.end(), [](const NodeInfo& c) { return c.control == Control::Stop; }) ? Control::Stop : Control::GiveWay;
            for (const auto* a : used) {
                const auto* way = find_way(graph_, tile_.entries[a->entry_index].way_id);
                if (!way || !(way->rank < max_rank)) continue;
                approach_line(jn, *a, junction_index, owner, kind, mm_to_m(fr_.sign_line_setback_mm), false);
            }
        }
    }
    void approach_line(const RoadJunction& jn, const JunctionApproach& a, std::uint32_t junction_index, std::int64_t owner, Control control,
            double setback_m, bool crossing_rule) {
        (void)jn;
        const double length = length_of(a.entry_index);
        const LaneDirection travel = a.at_end ? LaneDirection::Forward : LaneDirection::Backward;
        const double sigma = std::min(a.trim_m + setback_m, length);
        const double station = a.at_end ? length - sigma : sigma;
        // One-way leaving the junction: nothing to control (7.2 case (b)).
        if (lanes_at(a.entry_index, station, travel).count == 0 && lanes_at(a.entry_index, station, opposite(travel)).count != 0) return;
        if (covered_by_entry_node(a.entry_index, travel, a.at_end, control == Control::Signal)) {
            if (stats_) ++stats_->suppressed_junction_lines;
            return;
        }
        LineSpec spec;
        spec.entry = a.entry_index; spec.travel = travel; spec.station = station; spec.kind = line_kind(control); spec.width_m = width_of(control);
        spec.signal = control == Control::Signal; spec.crossing_rule = crossing_rule; spec.node = owner; spec.junction = junction_index;
        place_line_set(spec);
    }
};

// A point of the entry's frame, continued along the end tangent outside [0, L].
Result<LateralPoint> extended_point(const RoadReferenceLine& ref, double s, double t, double* heading = nullptr) {
    const double sc = std::clamp(s, 0.0, ref.length_m);
    const auto pose = ref.at(sc);
    if (!pose) return pose.error();
    double sn = 0, cs = 1;
    det::sincos(pose->heading, sn, cs);
    if (heading) *heading = pose->heading;
    const double extra = s - sc;
    return LateralPoint{pose->x - t * sn + extra * cs, pose->y + t * cs + extra * sn};
}
} // namespace

Result<std::vector<RoadFurniture>> build_road_furniture(const RoadGeomTile& tile, const RoadReferenceBuilder& context,
    std::span<const RoadJunction> junctions, const RoadRules& rules, FurnitureBuildStats* stats) {
    Builder builder(tile, context, junctions, rules, stats);
    return builder.run();
}

Result<std::array<LateralPoint, 4>> furniture_quad(const RoadGeomTile& tile, const RoadFurniture& f) {
    if (f.decline != FurnitureDecline::None) return make_error(ErrorCode::InvalidArgument, "furniture quad: declined record");
    if (f.entry_index >= tile.entries.size() || !tile.entries[f.entry_index].profile)
        return make_error(ErrorCode::InvalidArgument, "furniture quad: entry has no profile");
    const auto& ref = tile.entries[f.entry_index].profile->reference;
    const double s0 = f.station_m - f.length_m / 2, s1 = f.station_m + f.length_m / 2;
    std::array<LateralPoint, 4> q{};
    const double ss[4] = {s0, s1, s1, s0}, ts[4] = {f.t_from, f.t_from, f.t_to, f.t_to};
    for (int i = 0; i < 4; ++i) {
        const auto p = extended_point(ref, ss[i], ts[i]);
        if (!p) return p.error();
        q[static_cast<std::size_t>(i)] = *p;
    }
    return q;
}

Result<SignalPost> signal_post(const RoadGeomTile& tile, const RoadFurniture& f) {
    if (f.kind != FurnitureKind::Signal || f.decline != FurnitureDecline::None)
        return make_error(ErrorCode::InvalidArgument, "signal post: not a placed signal");
    if (f.entry_index >= tile.entries.size() || !tile.entries[f.entry_index].profile)
        return make_error(ErrorCode::InvalidArgument, "signal post: entry has no profile");
    const auto& ref = tile.entries[f.entry_index].profile->reference;
    double heading = 0;
    const auto p = extended_point(ref, f.station_m, f.t_from, &heading);
    if (!p) return p.error();
    double sn = 0, cs = 1;
    det::sincos(heading, sn, cs);
    const double sign = f.travel == LaneDirection::Forward ? -1.0 : 1.0;   // facing opposite to the travel direction
    return SignalPost{p->x, p->y, sign * cs, sign * sn};
}
} // namespace g2m
