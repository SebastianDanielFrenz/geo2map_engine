// tools/osm_tag_probe/tag_probe.cpp - see tag_probe.h.
#include "tag_probe.h"

#include <algorithm>
#include <cstdio>
#include <set>
#include <string_view>

namespace g2m::tag_probe {
namespace {

// ---------------------------------------------------------------- road metrics

struct MetricSpec {
    const char* name;
    std::vector<std::string_view> keys;     // exact keys (any present)
    std::vector<std::string_view> prefixes; // key starts with (any present)
    const char* value_equals;               // non-null: the FIRST of `keys` must have exactly this value
};

const std::vector<MetricSpec>& road_metrics() {
    static const std::vector<MetricSpec> m = {
        {"lanes", {"lanes"}, {}, nullptr},
        {"lanes_fwd_bwd", {"lanes:forward", "lanes:backward"}, {}, nullptr},
        {"turn_lanes", {"turn:lanes", "turn:lanes:forward", "turn:lanes:backward", "turn:lanes:both_ways"}, {}, nullptr},
        {"change_lanes",
         {"change:lanes", "change:lanes:forward", "change:lanes:backward", "change:lanes:both_ways"},
         {},
         nullptr},
        {"placement", {"placement", "placement:forward", "placement:backward"}, {}, nullptr},
        {"lane_markings", {"lane_markings"}, {}, nullptr},
        {"width", {"width"}, {}, nullptr},
        {"est_width", {"est_width"}, {}, nullptr},
        {"width_lanes", {"width:lanes", "width:lanes:forward", "width:lanes:backward"}, {}, nullptr},
        {"sidewalk", {"sidewalk", "sidewalk:left", "sidewalk:right", "sidewalk:both"}, {}, nullptr},
        {"cycleway", {"cycleway", "cycleway:left", "cycleway:right", "cycleway:both"}, {}, nullptr},
        {"parking_lane", {}, {"parking:lane"}, nullptr},
        {"parking_any", {}, {"parking:"}, nullptr},
        {"surface", {"surface"}, {}, nullptr},
        {"smoothness", {"smoothness"}, {}, nullptr},
        {"maxspeed", {"maxspeed", "maxspeed:forward", "maxspeed:backward"}, {}, nullptr},
        {"lit", {"lit"}, {}, nullptr},
        {"oneway", {"oneway"}, {}, nullptr},
        {"junction_roundabout", {"junction"}, {}, "roundabout"},
        {"divider", {"divider"}, {}, nullptr},
        {"overtaking", {"overtaking", "overtaking:forward", "overtaking:backward"}, {}, nullptr},
    };
    return m;
}

bool starts_with(std::string_view s, std::string_view prefix) { return s.substr(0, prefix.size()) == prefix; }

bool metric_present(const MetricSpec& m, TagSpan tags) {
    if (m.value_equals != nullptr) {
        const std::string* v = find_tag(tags, m.keys.front());
        return v != nullptr && *v == m.value_equals;
    }
    for (const Tag& t : tags) {
        if (t.value.empty()) continue;
        for (const std::string_view k : m.keys)
            if (t.key == k) return true;
        for (const std::string_view p : m.prefixes)
            if (starts_with(t.key, p)) return true;
    }
    return false;
}

// Report grouping of the road metrics (names must exist in road_metrics()).
struct ReportTable {
    const char* title;
    std::vector<const char*> metrics;
};
const std::vector<ReportTable>& road_tables() {
    static const std::vector<ReportTable> t = {
        {"Lanes", {"lanes", "lanes_fwd_bwd", "turn_lanes", "change_lanes", "placement", "lane_markings"}},
        {"Width, edges and parking",
         {"width", "est_width", "width_lanes", "sidewalk", "cycleway", "parking_lane", "parking_any"}},
        {"Surface and rules",
         {"surface", "smoothness", "maxspeed", "lit", "oneway", "junction_roundabout", "divider", "overtaking"}},
    };
    return t;
}

// ---------------------------------------------------------------- feature groups

struct GroupSpec {
    const char* name;
    std::vector<const char*> keys;
};

const std::vector<GroupSpec>& node_specs() {
    static const std::vector<GroupSpec> s = {
        {"crossing", {"crossing", "crossing:markings", "crossing_ref"}},
        {"traffic_signals", {"traffic_signals:direction", "direction"}},
        {"stop", {"direction"}},
        {"give_way", {"direction"}},
        {"traffic_calming", {}},
        {"kerb", {}},
        {"tree", {"leaf_type", "leaf_cycle", "genus", "species", "taxon", "denotation", "height"}},
    };
    return s;
}
const std::vector<GroupSpec>& way_specs() {
    static const std::vector<GroupSpec> s = {
        {"traffic_calming", {}},
        {"kerb", {}},
        {"footway_crossing", {"crossing", "crossing:markings"}},
        {"building",
         {"height", "building:levels", "roof:shape", "roof:levels", "roof:colour", "roof:material", "building:colour",
          "building:material", "roof:height", "min_height"}},
        {"tree_row", {"leaf_type", "leaf_cycle", "genus", "species"}},
        {"forest", {"leaf_type", "leaf_cycle"}},
    };
    return s;
}

const GroupSpec& find_spec(const std::vector<GroupSpec>& specs, std::string_view name) {
    for (const GroupSpec& s : specs)
        if (name == s.name) return s;
    static const GroupSpec none{"", {}};
    return none;
}

const char* node_group_name(std::uint8_t kind_bit) {
    switch (kind_bit) {
    case kNodeCrossing: return "crossing";
    case kNodeSignals: return "traffic_signals";
    case kNodeStop: return "stop";
    case kNodeGiveWay: return "give_way";
    case kNodeCalming: return "traffic_calming";
    case kNodeKerb: return "kerb";
    case kNodeTree: return "tree";
    default: return "";
    }
}

void track(Group& g, const GroupSpec& spec, TagSpan tags, std::int64_t len_mm, std::int64_t area_m2) {
    g.all.add(len_mm, area_m2);
    for (const char* k : spec.keys)
        if (find_tag(tags, k) != nullptr) g.with[k].add(len_mm, area_m2);
}

bool is_value(TagSpan tags, std::string_view key, std::string_view value) {
    const std::string* v = find_tag(tags, key);
    return v != nullptr && *v == value;
}

bool is_building(TagSpan tags) {
    const std::string* v = find_tag(tags, "building");
    return v != nullptr && *v != "no";
}
bool is_forest(TagSpan tags) {
    return is_value(tags, "natural", "wood") || is_value(tags, "landuse", "forest");
}

const char* const kPlaceNames[] = {"off_carriageway", "dead_end", "mid_way", "junction"};
const char* const kPlaceExplain[] = {"no carriageway way uses the node", "degree 1 (way end)",
                                     "degree 2 (mid-way or a way split, not a junction)", "degree >= 3 (junction)"};

// ---------------------------------------------------------------- formatting

std::string u64(std::uint64_t v) { return std::to_string(v); }

std::string json_escape(std::string_view s) {
    std::string o;
    o.reserve(s.size() + 2);
    for (const char ch : s) {
        const auto c = static_cast<unsigned char>(ch);
        switch (c) {
        case '"': o += "\\\""; break;
        case '\\': o += "\\\\"; break;
        case '\n': o += "\\n"; break;
        case '\r': o += "\\r"; break;
        case '\t': o += "\\t"; break;
        default:
            if (c < 0x20) {
                char buf[8];
                std::snprintf(buf, sizeof buf, "\\u%04x", c);
                o += buf;
            } else {
                o += ch;
            }
        }
    }
    return o;
}
std::string jstr(std::string_view s) { return "\"" + json_escape(s) + "\""; }

std::string md_cell(std::string_view s) {
    std::string o;
    for (const char c : s) {
        if (c == '|') o += "\\|";
        else if (c == '`') o += '\'';
        else if (c == '\n' || c == '\r') o += ' ';
        else o += c;
    }
    return o;
}
std::string md_code(std::string_view s) { return "`" + md_cell(s) + "`"; }

std::string hex_or(std::string_view s) { return s.empty() ? "-" : std::string(s); }

// "w% / km%" cell, one decimal each.
std::string cell_pair(const Counts& with, const Counts& all) {
    return format_pct(with.n, all.n, 1) + " / " + format_pct(static_cast<std::uint64_t>(with.length_mm),
                                                            static_cast<std::uint64_t>(all.length_mm), 1);
}

void sum_into(Counts& a, const Counts& b) {
    a.n += b.n;
    a.length_mm += b.length_mm;
    a.area_m2 += b.area_m2;
}

// A tally section as a markdown table.
void md_tally(std::string& out, const Accumulator& acc, const std::string& title, const std::string& tally_name,
              std::size_t limit) {
    const auto all = acc.top_values(tally_name, 0);
    std::uint64_t total = 0;
    for (const auto& kv : all) total += kv.second;
    out += "\n**" + title + "** (" + u64(total) + " tagged, " + u64(all.size()) + " distinct values)\n\n";
    if (all.empty()) {
        out += "_none_\n";
        return;
    }
    out += "| # | value | count | % of tagged |\n|---:|---|---:|---:|\n";
    std::uint64_t shown = 0;
    const std::size_t n = std::min(limit, all.size());
    for (std::size_t i = 0; i < n; ++i) {
        out += "| " + std::to_string(i + 1) + " | " + md_code(all[i].first) + " | " + u64(all[i].second) + " | " +
               format_pct(all[i].second, total, 1) + " |\n";
        shown += all[i].second;
    }
    if (all.size() > n)
        out += "| | _other (" + u64(all.size() - n) + " values)_ | " + u64(total - shown) + " | " +
               format_pct(total - shown, total, 1) + " |\n";
}

std::string json_tally(const Accumulator& acc, const std::string& tally_name, std::size_t limit) {
    const auto all = acc.top_values(tally_name, 0);
    std::uint64_t total = 0;
    for (const auto& kv : all) total += kv.second;
    std::uint64_t shown = 0;
    const std::size_t n = std::min(limit, all.size());
    std::string o = "{\"tagged\":" + u64(total) + ",\"distinct\":" + u64(all.size()) + ",\"top\":[";
    for (std::size_t i = 0; i < n; ++i) {
        if (i != 0) o += ',';
        o += "{\"value\":" + jstr(all[i].first) + ",\"count\":" + u64(all[i].second) + ",\"pct\":" +
             format_pct(all[i].second, total, 2) + "}";
        shown += all[i].second;
    }
    o += "],\"other_count\":" + u64(total - shown) + "}";
    return o;
}

std::string json_counts(const Counts& c, bool length, bool area) {
    std::string o = "{\"n\":" + u64(c.n);
    if (length) o += ",\"length_km\":" + format_km(c.length_mm);
    if (area) o += ",\"area_ha\":" + format_ha(c.area_m2);
    return o + "}";
}

std::string json_group(const Group& g, const GroupSpec& spec, bool length, bool area) {
    std::string o = "{\"total\":" + json_counts(g.all, length, area) + ",\"with\":{";
    bool first = true;
    for (const char* k : spec.keys) {
        if (!first) o += ',';
        first = false;
        const auto it = g.with.find(k);
        const Counts c = it == g.with.end() ? Counts{} : it->second;
        o += jstr(k) + ":{\"n\":" + u64(c.n) + ",\"pct\":" + format_pct(c.n, g.all.n, 2);
        if (length) o += ",\"length_km\":" + format_km(c.length_mm) + ",\"pct_length\":" +
                         format_pct(static_cast<std::uint64_t>(c.length_mm), static_cast<std::uint64_t>(g.all.length_mm), 2);
        if (area) o += ",\"area_ha\":" + format_ha(c.area_m2) + ",\"pct_area\":" +
                       format_pct(static_cast<std::uint64_t>(c.area_m2), static_cast<std::uint64_t>(g.all.area_m2), 2);
        o += "}";
    }
    return o + "}}";
}

void md_group(std::string& out, const std::string& heading, const Group& g, const GroupSpec& spec, bool length,
              bool area) {
    out += "\n**" + heading + "**: " + u64(g.all.n);
    if (length) out += ", " + format_km(g.all.length_mm) + " km";
    if (area) out += ", " + format_ha(g.all.area_m2) + " ha";
    out += "\n\n";
    if (spec.keys.empty()) return;
    out += "| key present | count | % of count";
    if (length) out += " | % of km";
    if (area) out += " | % of area";
    out += " |\n|---|---:|---:";
    if (length) out += "|---:";
    if (area) out += "|---:";
    out += "|\n";
    for (const char* k : spec.keys) {
        const auto it = g.with.find(k);
        const Counts c = it == g.with.end() ? Counts{} : it->second;
        out += "| " + md_code(k) + " | " + u64(c.n) + " | " + format_pct(c.n, g.all.n, 1);
        if (length)
            out += " | " + format_pct(static_cast<std::uint64_t>(c.length_mm), static_cast<std::uint64_t>(g.all.length_mm), 1);
        if (area)
            out += " | " + format_pct(static_cast<std::uint64_t>(c.area_m2), static_cast<std::uint64_t>(g.all.area_m2), 1);
        out += " |\n";
    }
}

std::string json_placement(const std::array<std::uint64_t, kPlaceClassCount>& p) {
    std::uint64_t total = 0;
    for (const auto v : p) total += v;
    std::string o = "{";
    for (int i = 0; i < kPlaceClassCount; ++i) {
        if (i != 0) o += ',';
        o += jstr(kPlaceNames[i]) + ":{\"n\":" + u64(p[static_cast<std::size_t>(i)]) + ",\"pct\":" +
             format_pct(p[static_cast<std::size_t>(i)], total, 2) + "}";
    }
    return o + "}";
}

void md_placement(std::string& out, const std::array<std::uint64_t, kPlaceClassCount>& p) {
    std::uint64_t total = 0;
    for (const auto v : p) total += v;
    out += "\nPlacement (carriageway ways through the node):\n\n| class | count | % | meaning |\n|---|---:|---:|---|\n";
    for (int i = 0; i < kPlaceClassCount; ++i)
        out += std::string("| ") + kPlaceNames[i] + " | " + u64(p[static_cast<std::size_t>(i)]) + " | " +
               format_pct(p[static_cast<std::size_t>(i)], total, 1) + " | " + kPlaceExplain[i] + " |\n";
}

} // namespace

// ---------------------------------------------------------------- public helpers

const std::string* find_tag(TagSpan tags, std::string_view key) {
    for (const Tag& t : tags)
        if (t.key == key) return t.value.empty() ? nullptr : &t.value;
    return nullptr;
}

std::string format_pct(std::uint64_t num, std::uint64_t den, int decimals) {
    if (decimals < 0) decimals = 0;
    if (decimals > 2) decimals = 2;
    std::uint64_t scale = 1; // value is in units of 10^-decimals percent
    for (int i = 0; i < decimals; ++i) scale *= 10;
    std::uint64_t v = 0;
    if (den != 0) {
        // round(num * 100 * scale / den), half up; num is bounded far below 2^64 / 20000.
        v = (num * 200 * scale + den) / (2 * den);
    }
    std::string s = std::to_string(v / scale);
    if (decimals > 0) {
        std::string frac = std::to_string(v % scale);
        while (frac.size() < static_cast<std::size_t>(decimals)) frac.insert(frac.begin(), '0');
        s += "." + frac;
    }
    return s;
}

std::string format_km(std::int64_t mm) {
    const std::int64_t m = mm < 0 ? 0 : (mm + 500) / 1000;
    std::string frac = std::to_string(m % 1000);
    while (frac.size() < 3) frac.insert(frac.begin(), '0');
    return std::to_string(m / 1000) + "." + frac;
}

std::string format_ha(std::int64_t m2) {
    const std::int64_t t = m2 < 0 ? 0 : (m2 + 500) / 1000; // tenths of a hectare (0.1 ha = 1000 m^2)
    return std::to_string(t / 10) + "." + std::to_string(t % 10);
}

// ---------------------------------------------------------------- Accumulator: classification

const std::vector<std::string>& Accumulator::road_class_names() {
    static const std::vector<std::string> n = {
        "motorway", "motorway_link", "trunk",     "trunk_link",   "primary",   "primary_link", "secondary",
        "secondary_link", "tertiary", "tertiary_link", "unclassified", "residential", "service",
    };
    return n;
}

int Accumulator::road_class_index(TagSpan tags) {
    const std::string* h = find_tag(tags, "highway");
    if (h == nullptr) return -1;
    const auto& names = road_class_names();
    for (std::size_t i = 0; i < names.size(); ++i)
        if (*h == names[i]) return static_cast<int>(i);
    return -1;
}

bool Accumulator::is_carriageway(TagSpan tags) {
    if (road_class_index(tags) >= 0) return true;
    return is_value(tags, "highway", "living_street");
}

std::uint8_t Accumulator::node_kind_mask(TagSpan tags) {
    std::uint8_t m = 0;
    const std::string* h = find_tag(tags, "highway");
    if (h != nullptr) {
        if (*h == "crossing") m |= kNodeCrossing;
        else if (*h == "traffic_signals") m |= kNodeSignals;
        else if (*h == "stop") m |= kNodeStop;
        else if (*h == "give_way") m |= kNodeGiveWay;
    }
    if (find_tag(tags, "traffic_calming") != nullptr) m |= kNodeCalming;
    if (find_tag(tags, "kerb") != nullptr) m |= kNodeKerb;
    if (is_value(tags, "natural", "tree")) m |= kNodeTree;
    return m;
}

bool Accumulator::way_is_relevant(TagSpan tags) {
    if (road_class_index(tags) >= 0) return true;
    if (find_tag(tags, "traffic_calming") != nullptr || find_tag(tags, "kerb") != nullptr) return true;
    if (is_building(tags) || is_forest(tags)) return true;
    if (is_value(tags, "natural", "tree_row")) return true;
    return is_value(tags, "footway", "crossing");
}

bool Accumulator::relation_is_relevant(TagSpan tags) {
    if (!is_value(tags, "type", "multipolygon")) return false;
    return is_building(tags) || is_forest(tags);
}

// ---------------------------------------------------------------- Accumulator: adding

void Accumulator::tally(std::string_view tally_name, std::string_view value) {
    ++tallies_[std::string(tally_name)][std::string(value)];
}

void Accumulator::add_node(std::uint8_t mask, TagSpan tags) {
    for (std::uint8_t bit = 1; bit != 0 && bit <= kNodeTree; bit = static_cast<std::uint8_t>(bit << 1)) {
        if ((mask & bit) == 0) continue;
        const std::string name = node_group_name(bit);
        track(node_groups_[name], find_spec(node_specs(), name), tags, 0, 0);
        if (bit == kNodeCrossing) {
            if (const std::string* v = find_tag(tags, "crossing")) tally("crossing.crossing", *v);
            if (const std::string* v = find_tag(tags, "crossing:markings")) tally("crossing.crossing:markings", *v);
        } else if (bit == kNodeCalming) {
            if (const std::string* v = find_tag(tags, "traffic_calming")) tally("traffic_calming.nodes", *v);
        } else if (bit == kNodeKerb) {
            if (const std::string* v = find_tag(tags, "kerb")) tally("kerb.nodes", *v);
        } else if (bit == kNodeTree) {
            if (const std::string* v = find_tag(tags, "leaf_type")) tally("tree.leaf_type", *v);
            if (const std::string* v = find_tag(tags, "leaf_cycle")) tally("tree.leaf_cycle", *v);
            if (const std::string* v = find_tag(tags, "denotation")) tally("tree.denotation", *v);
        }
    }
}

void Accumulator::add_node_degree(std::uint8_t mask, unsigned degree) {
    const int cls = degree == 0 ? kPlaceOffRoad : degree == 1 ? kPlaceDeadEnd : degree == 2 ? kPlaceMidWay : kPlaceJunction;
    for (std::uint8_t bit = 1; bit != 0 && bit <= kNodeKerb; bit = static_cast<std::uint8_t>(bit << 1)) {
        if ((mask & bit) == 0) continue;
        ++placement_[node_group_name(bit)][static_cast<std::size_t>(cls)];
    }
}

void Accumulator::add_building(TagSpan tags, std::int64_t area_m2, bool via_relation) {
    track(way_groups_["building"], find_spec(way_specs(), "building"), tags, 0, area_m2);
    if (via_relation) building_relations_.add(0, area_m2);
    if (const std::string* v = find_tag(tags, "roof:shape")) tally("building.roof:shape", *v);
    if (const std::string* v = find_tag(tags, "building:colour")) tally("building.building:colour", *v);
}

void Accumulator::add_forest(TagSpan tags, std::int64_t area_m2) {
    track(way_groups_["forest"], find_spec(way_specs(), "forest"), tags, 0, area_m2);
    if (is_value(tags, "natural", "wood")) forest_by_tag_["natural=wood"].add(0, area_m2);
    if (is_value(tags, "landuse", "forest")) forest_by_tag_["landuse=forest"].add(0, area_m2);
    if (const std::string* v = find_tag(tags, "leaf_type")) tally("forest.leaf_type", *v);
    if (const std::string* v = find_tag(tags, "leaf_cycle")) tally("forest.leaf_cycle", *v);
}

void Accumulator::add_way(TagSpan tags, std::int64_t length_mm, std::int64_t area_m2, bool closed) {
    const int ri = road_class_index(tags);
    if (ri >= 0) {
        RoadClass& rc = roads_[road_class_names()[static_cast<std::size_t>(ri)]];
        rc.all.add(length_mm, 0);
        for (const MetricSpec& m : road_metrics())
            if (metric_present(m, tags)) rc.with[m.name].add(length_mm, 0);
        for (const Tag& t : tags) {
            if (t.value.empty()) continue;
            if (t.key == "turn:lanes" || t.key == "turn:lanes:forward" || t.key == "turn:lanes:backward" ||
                t.key == "turn:lanes:both_ways")
                tally("road.turn:lanes", t.value);
        }
    }
    if (const std::string* v = find_tag(tags, "traffic_calming")) {
        track(way_groups_["traffic_calming"], find_spec(way_specs(), "traffic_calming"), tags, length_mm, 0);
        tally("traffic_calming.ways", *v);
    }
    if (const std::string* v = find_tag(tags, "kerb")) {
        track(way_groups_["kerb"], find_spec(way_specs(), "kerb"), tags, length_mm, 0);
        tally("kerb.ways", *v);
    }
    if (is_value(tags, "footway", "crossing"))
        track(way_groups_["footway_crossing"], find_spec(way_specs(), "footway_crossing"), tags, length_mm, 0);
    if (is_building(tags)) {
        if (closed) add_building(tags, area_m2, false);
        else building_open_ways_.add(length_mm, 0);
    }
    if (is_value(tags, "natural", "tree_row"))
        track(way_groups_["tree_row"], find_spec(way_specs(), "tree_row"), tags, length_mm, 0);
    if (is_forest(tags)) {
        if (closed) add_forest(tags, area_m2);
        else forest_open_ways_.add(length_mm, 0);
    }
}

void Accumulator::add_relation_polygon(TagSpan tags, std::int64_t area_m2) {
    if (is_building(tags)) add_building(tags, area_m2, true);
    if (is_forest(tags)) add_forest(tags, area_m2);
}

// ---------------------------------------------------------------- Accumulator: read access

const Accumulator::RoadClass& Accumulator::road(std::string_view highway) const {
    static const RoadClass empty;
    const auto it = roads_.find(std::string(highway));
    return it == roads_.end() ? empty : it->second;
}
const Group& Accumulator::node_group(std::string_view name) const {
    static const Group empty;
    const auto it = node_groups_.find(std::string(name));
    return it == node_groups_.end() ? empty : it->second;
}
const Group& Accumulator::way_group(std::string_view name) const {
    static const Group empty;
    const auto it = way_groups_.find(std::string(name));
    return it == way_groups_.end() ? empty : it->second;
}
Counts Accumulator::forest_by_tag(std::string_view tag) const {
    const auto it = forest_by_tag_.find(std::string(tag));
    return it == forest_by_tag_.end() ? Counts{} : it->second;
}
std::array<std::uint64_t, kPlaceClassCount> Accumulator::placement(std::string_view node_group) const {
    const auto it = placement_.find(std::string(node_group));
    return it == placement_.end() ? std::array<std::uint64_t, kPlaceClassCount>{} : it->second;
}

std::vector<std::pair<std::string, std::uint64_t>> Accumulator::top_values(std::string_view tally_name,
                                                                           std::size_t limit) const {
    std::vector<std::pair<std::string, std::uint64_t>> v;
    const auto it = tallies_.find(std::string(tally_name));
    if (it == tallies_.end()) return v;
    v.assign(it->second.begin(), it->second.end());
    std::sort(v.begin(), v.end(), [](const auto& a, const auto& b) {
        if (a.second != b.second) return a.second > b.second;
        return a.first < b.first;
    });
    if (limit != 0 && v.size() > limit) v.resize(limit);
    return v;
}

// ---------------------------------------------------------------- reports

namespace {

Counts road_with(const Accumulator::RoadClass& rc, const char* metric) {
    const auto it = rc.with.find(metric);
    return it == rc.with.end() ? Counts{} : it->second;
}

} // namespace

std::string Accumulator::to_markdown(const ReportMeta& meta) const {
    std::string o;
    o += "# OSM tag coverage: " + meta.region_name + "\n\n";
    o += "Phase P0 of the street-level fidelity plan (`out/street_fidelity_plan.md` section 6). Produced by "
         "`g2m_osm_tag_probe`; counts are deterministic (same input, same bytes).\n\n";
    o += "## Input\n\n";
    o += "- PBF: `" + md_cell(meta.pbf_name) + "`, sha256 `" + hex_or(meta.pbf_sha256_hex) + "`\n";
    o += "- Region `" + md_cell(meta.region_name) + "`: union of these OSM administrative boundaries\n\n";
    o += "| relation | name | admin_level | geometry hash |\n|---:|---|---:|---|\n";
    for (const ReportPart& p : meta.parts)
        o += "| " + std::to_string(p.id) + " | " + md_cell(p.name) + " | " +
             (p.admin_level == 0 ? std::string("-") : std::to_string(p.admin_level)) + " | `" + hex_or(p.geometry_hash_hex) +
             "` |\n";
    const ScanStats& s = meta.scan;
    o += "\nScan: " + u64(s.nodes_total) + " nodes, " + u64(s.ways_total) + " ways, " + u64(s.relations_total) +
         " relations in the PBF. Tagged nodes of interest inside the region: " + u64(s.tagged_nodes_in_region) + ". Relevant ways: " +
         u64(s.ways_relevant) + ", of which in the region " + u64(s.ways_in_region) + " (anchor node not in the PBF: " +
         u64(s.ways_unresolved_anchor) + "; in-region ways with a node ref missing from the PBF: " +
         u64(s.ways_missing_nodes) + "). Relevant relations (multipolygon building/forest): " + u64(s.relations_relevant) +
         ", in the region " + u64(s.relations_in_region) + " (exact ring assembly rejected, lenient ring areas used: " +
         u64(s.relations_lenient) + "; counted with area 0: " + u64(s.relations_zero_area) + "; no geometry, skipped: " +
         u64(s.relations_no_geometry) + ").\n\n";
    o += "## Method\n\n";
    o += "- A way or polygon is in the region when its **anchor** lies inside the union: the node at index n/2 of "
         "a way's node list (a relation: the first vertex of its first assembled ring). Exact integer "
         "point-in-polygon on UTM zone 32 (GRS80) millimetres, the same ring assembly and region cut the tiler "
         "uses. A way crossing the border is counted whole, once.\n";
    o += "- Relation areas: outer rings minus inner rings from the region resolver's exact ring assembly; where "
         "that rejects the geometry (crossing or overlapping rings), from a lenient role-based chaining of the "
         "member ways (the scan line above counts those).\n";
    o += "- Lengths are summed segment lengths in the UTM grid (not corrected for the 0.9996 scale factor, error "
         "under 0.1 percent). Areas are shoelace areas in the UTM grid.\n";
    o += "- A key is **present** when it exists with a non-empty value (so `sidewalk=no` counts as mapped).\n";
    o += "- Percent cells in the road tables read `% of ways / % of km`.\n";
    o += "- Metric groups: `lanes_fwd_bwd` = `lanes:forward` or `lanes:backward`; `turn_lanes` = `turn:lanes` "
         "with or without `:forward`, `:backward`, `:both_ways`; `change_lanes` likewise; `placement` = "
         "`placement`, `placement:forward`, `placement:backward`; `width_lanes` = `width:lanes[:forward|:backward]`; "
         "`sidewalk`/`cycleway` = the bare key or `:left`/`:right`/`:both`; `maxspeed` = bare or "
         "`:forward`/`:backward`; `overtaking` likewise; `parking_lane` = any key starting `parking:lane`; "
         "`parking_any` = any key starting `parking:`; `junction_roundabout` = `junction=roundabout`.\n";
    o += "- Node placement is derived from the node's degree in carriageway ways (the 13 reported classes plus "
         "`living_street`), counted over the whole PBF: interior occurrence 2, way end 1.\n";
    o += "- Buildings: `building=*` other than `no`, closed ways plus multipolygon relations (old-style relations "
         "whose outer way also carries `building` count twice; rare). Forests: closed ways and multipolygon "
         "relations tagged `natural=wood` or `landuse=forest`, counted once even when both are set.\n\n";

    // ---- roads
    o += "## 1. Road classes\n\n";
    std::map<std::string, Counts> all_with;
    Counts all_total;
    o += "| class | ways | km |\n|---|---:|---:|\n";
    for (const std::string& cls : road_class_names()) {
        const RoadClass& rc = road(cls);
        o += "| " + cls + " | " + u64(rc.all.n) + " | " + format_km(rc.all.length_mm) + " |\n";
        sum_into(all_total, rc.all);
        for (const auto& kv : rc.with) sum_into(all_with[kv.first], kv.second);
    }
    o += "| **all listed** | " + u64(all_total.n) + " | " + format_km(all_total.length_mm) + " |\n";
    for (const ReportTable& t : road_tables()) {
        o += std::string("\n### ") + t.title + " (% of ways / % of km)\n\n| class |";
        for (const char* m : t.metrics) o += std::string(" ") + m + " |";
        o += "\n|---|";
        for (std::size_t i = 0; i < t.metrics.size(); ++i) o += "---:|";
        o += "\n";
        for (const std::string& cls : road_class_names()) {
            const RoadClass& rc = road(cls);
            o += "| " + cls + " |";
            for (const char* m : t.metrics) o += " " + cell_pair(road_with(rc, m), rc.all) + " |";
            o += "\n";
        }
        o += "| **all listed** |";
        for (const char* m : t.metrics) {
            const auto it = all_with.find(m);
            o += " **" + cell_pair(it == all_with.end() ? Counts{} : it->second, all_total) + "** |";
        }
        o += "\n";
    }
    md_tally(o, *this, "Top values of turn:lanes (all turn:lanes keys merged)", "road.turn:lanes", 15);

    // ---- nodes
    o += "\n## 2. Nodes\n";
    for (const char* name : {"crossing", "traffic_signals", "stop", "give_way", "traffic_calming", "kerb"}) {
        const std::string n = name;
        const char* label = n == "crossing"           ? "highway=crossing nodes"
                            : n == "traffic_signals"  ? "highway=traffic_signals nodes"
                            : n == "stop"             ? "highway=stop nodes"
                            : n == "give_way"         ? "highway=give_way nodes"
                            : n == "traffic_calming"  ? "traffic_calming=* nodes"
                                                      : "kerb=* nodes";
        md_group(o, label, node_group(n), find_spec(node_specs(), n), false, false);
        md_placement(o, placement(n));
        if (n == "crossing") {
            md_tally(o, *this, "crossing=* values", "crossing.crossing", 15);
            md_tally(o, *this, "Top values of crossing:markings", "crossing.crossing:markings", 15);
        } else if (n == "traffic_calming") {
            md_tally(o, *this, "traffic_calming by type (nodes)", "traffic_calming.nodes", 15);
        } else if (n == "kerb") {
            md_tally(o, *this, "kerb=* values (nodes)", "kerb.nodes", 15);
        }
    }
    o += "\nWays carrying the same families:\n";
    md_group(o, "traffic_calming=* ways", way_group("traffic_calming"), find_spec(way_specs(), "traffic_calming"), true, false);
    md_tally(o, *this, "traffic_calming by type (ways)", "traffic_calming.ways", 15);
    md_group(o, "kerb=* ways", way_group("kerb"), find_spec(way_specs(), "kerb"), true, false);
    md_group(o, "footway=crossing ways", way_group("footway_crossing"), find_spec(way_specs(), "footway_crossing"), true,
             false);

    // ---- buildings
    o += "\n## 3. Buildings\n";
    md_group(o, "Buildings (closed ways and multipolygon relations)", way_group("building"),
             find_spec(way_specs(), "building"), false, true);
    o += "\nOf these, from relations: " + u64(building_relations_.n) + ". Open ways tagged `building` (not counted above): " +
         u64(building_open_ways_.n) + ".\n";
    md_tally(o, *this, "Top values of roof:shape", "building.roof:shape", 15);
    md_tally(o, *this, "Top values of building:colour", "building.building:colour", 15);

    // ---- trees
    o += "\n## 4. Trees and forest\n";
    md_group(o, "natural=tree nodes", node_group("tree"), find_spec(node_specs(), "tree"), false, false);
    md_tally(o, *this, "tree leaf_type", "tree.leaf_type", 15);
    md_tally(o, *this, "tree leaf_cycle", "tree.leaf_cycle", 15);
    md_tally(o, *this, "tree denotation", "tree.denotation", 15);
    md_group(o, "natural=tree_row ways", way_group("tree_row"), find_spec(way_specs(), "tree_row"), true, false);
    md_group(o, "Forest polygons (natural=wood or landuse=forest)", way_group("forest"), find_spec(way_specs(), "forest"),
             false, true);
    o += "\nBy tag (a polygon with both appears in both rows): natural=wood " + u64(forest_by_tag("natural=wood").n) + " (" +
         format_ha(forest_by_tag("natural=wood").area_m2) + " ha), landuse=forest " + u64(forest_by_tag("landuse=forest").n) +
         " (" + format_ha(forest_by_tag("landuse=forest").area_m2) + " ha). Open ways with these tags (not counted): " +
         u64(forest_open_ways_.n) + ".\n";
    md_tally(o, *this, "forest leaf_type", "forest.leaf_type", 15);
    md_tally(o, *this, "forest leaf_cycle", "forest.leaf_cycle", 15);
    return o;
}

std::string Accumulator::to_json(const ReportMeta& meta) const {
    std::string o = "{\n";
    o += "\"format\":\"g2m.osm_tag_coverage/1\",\n";
    o += "\"region\":" + jstr(meta.region_name) + ",\n";
    o += "\"pbf\":{\"name\":" + jstr(meta.pbf_name) + ",\"sha256\":" + jstr(meta.pbf_sha256_hex) + "},\n";
    o += "\"parts\":[";
    for (std::size_t i = 0; i < meta.parts.size(); ++i) {
        const ReportPart& p = meta.parts[i];
        if (i != 0) o += ',';
        o += "{\"relation\":" + std::to_string(p.id) + ",\"name\":" + jstr(p.name) + ",\"admin_level\":" +
             std::to_string(p.admin_level) + ",\"geometry_hash\":" + jstr(p.geometry_hash_hex) + "}";
    }
    o += "],\n";
    const ScanStats& s = meta.scan;
    o += "\"scan\":{\"nodes_total\":" + u64(s.nodes_total) + ",\"tagged_nodes_in_region\":" + u64(s.tagged_nodes_in_region) +
         ",\"ways_total\":" + u64(s.ways_total) + ",\"ways_relevant\":" + u64(s.ways_relevant) +
         ",\"ways_in_region\":" + u64(s.ways_in_region) + ",\"ways_unresolved_anchor\":" + u64(s.ways_unresolved_anchor) +
         ",\"ways_missing_nodes\":" + u64(s.ways_missing_nodes) + ",\"relations_total\":" + u64(s.relations_total) +
         ",\"relations_relevant\":" + u64(s.relations_relevant) + ",\"relations_in_region\":" + u64(s.relations_in_region) +
         ",\"relations_lenient\":" + u64(s.relations_lenient) + ",\"relations_zero_area\":" + u64(s.relations_zero_area) +
         ",\"relations_no_geometry\":" + u64(s.relations_no_geometry) + "},\n";

    // roads
    o += "\"roads\":{\n\"metrics\":[";
    for (std::size_t i = 0; i < road_metrics().size(); ++i) o += (i ? "," : "") + jstr(road_metrics()[i].name);
    o += "],\n\"classes\":{\n";
    std::map<std::string, Counts> all_with;
    Counts all_total;
    const auto road_json = [&](const RoadClass& rc) {
        std::string r = "{\"ways\":" + u64(rc.all.n) + ",\"length_km\":" + format_km(rc.all.length_mm) + ",\"with\":{";
        bool first = true;
        for (const MetricSpec& m : road_metrics()) {
            const auto it = rc.with.find(m.name);
            const Counts c = it == rc.with.end() ? Counts{} : it->second;
            if (!first) r += ',';
            first = false;
            r += jstr(m.name) + ":{\"ways\":" + u64(c.n) + ",\"pct_ways\":" + format_pct(c.n, rc.all.n, 2) +
                 ",\"length_km\":" + format_km(c.length_mm) + ",\"pct_length\":" +
                 format_pct(static_cast<std::uint64_t>(c.length_mm), static_cast<std::uint64_t>(rc.all.length_mm), 2) + "}";
        }
        return r + "}}";
    };
    for (const std::string& cls : road_class_names()) {
        const RoadClass& rc = road(cls);
        o += jstr(cls) + ":" + road_json(rc) + ",\n";
        sum_into(all_total, rc.all);
        for (const auto& kv : rc.with) sum_into(all_with[kv.first], kv.second);
    }
    RoadClass all_rc;
    all_rc.all = all_total;
    all_rc.with = all_with;
    o += "\"all_listed\":" + road_json(all_rc) + "\n},\n";
    o += "\"turn_lanes_top\":" + json_tally(*this, "road.turn:lanes", 15) + "\n},\n";

    // nodes
    o += "\"nodes\":{\n";
    const char* node_names[] = {"crossing", "traffic_signals", "stop", "give_way", "traffic_calming", "kerb", "tree"};
    for (std::size_t i = 0; i < 7; ++i) {
        const std::string n = node_names[i];
        o += jstr(n) + ":{\"group\":" + json_group(node_group(n), find_spec(node_specs(), n), false, false);
        if (n != "tree") o += ",\"placement\":" + json_placement(placement(n));
        if (n == "crossing")
            o += ",\"crossing_values\":" + json_tally(*this, "crossing.crossing", 15) + ",\"crossing_markings_top\":" +
                 json_tally(*this, "crossing.crossing:markings", 15);
        if (n == "traffic_calming") o += ",\"by_type\":" + json_tally(*this, "traffic_calming.nodes", 15);
        if (n == "kerb") o += ",\"values\":" + json_tally(*this, "kerb.nodes", 15);
        if (n == "tree")
            o += ",\"leaf_type\":" + json_tally(*this, "tree.leaf_type", 15) + ",\"leaf_cycle\":" +
                 json_tally(*this, "tree.leaf_cycle", 15) + ",\"denotation\":" + json_tally(*this, "tree.denotation", 15);
        o += i + 1 < 7 ? "},\n" : "}\n";
    }
    o += "},\n";

    // ways
    o += "\"ways\":{\n";
    o += "\"traffic_calming\":{\"group\":" + json_group(way_group("traffic_calming"), find_spec(way_specs(), "traffic_calming"), true, false) +
         ",\"by_type\":" + json_tally(*this, "traffic_calming.ways", 15) + "},\n";
    o += "\"kerb\":{\"group\":" + json_group(way_group("kerb"), find_spec(way_specs(), "kerb"), true, false) + "},\n";
    o += "\"footway_crossing\":{\"group\":" +
         json_group(way_group("footway_crossing"), find_spec(way_specs(), "footway_crossing"), true, false) + "},\n";
    o += "\"tree_row\":{\"group\":" + json_group(way_group("tree_row"), find_spec(way_specs(), "tree_row"), true, false) + "}\n";
    o += "},\n";

    // buildings
    o += "\"buildings\":{\"group\":" + json_group(way_group("building"), find_spec(way_specs(), "building"), false, true) +
         ",\"from_relations\":" + json_counts(building_relations_, false, true) +
         ",\"open_ways_not_counted\":" + u64(building_open_ways_.n) +
         ",\"roof_shape_top\":" + json_tally(*this, "building.roof:shape", 15) +
         ",\"building_colour_top\":" + json_tally(*this, "building.building:colour", 15) + "},\n";

    // forest
    o += "\"forest\":{\"group\":" + json_group(way_group("forest"), find_spec(way_specs(), "forest"), false, true) +
         ",\"natural_wood\":" + json_counts(forest_by_tag("natural=wood"), false, true) +
         ",\"landuse_forest\":" + json_counts(forest_by_tag("landuse=forest"), false, true) +
         ",\"open_ways_not_counted\":" + u64(forest_open_ways_.n) +
         ",\"leaf_type\":" + json_tally(*this, "forest.leaf_type", 15) +
         ",\"leaf_cycle\":" + json_tally(*this, "forest.leaf_cycle", 15) + "}\n";
    o += "}\n";
    return o;
}

} // namespace g2m::tag_probe
