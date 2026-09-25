// src/layer/osm_data.cpp - see g2m/layer/osm_data.h.

#include "g2m/layer/osm_data.h"

#include <algorithm>
#include <string>

namespace g2m::osm {
namespace {

template <class T>
const T* find_by_id(const std::vector<T>& v, std::int64_t id) {
    const auto it = std::lower_bound(v.begin(), v.end(), id, [](const T& e, std::int64_t x) { return e.id < x; });
    return (it != v.end() && it->id == id) ? &*it : nullptr;
}

template <class T>
bool ids_ascending(const std::vector<T>& v) {
    for (std::size_t i = 1; i < v.size(); ++i) {
        if (!(v[i - 1].id < v[i].id)) return false;
    }
    return true;
}

} // namespace

const Node* OsmData::find_node(std::int64_t id) const { return find_by_id(nodes, id); }
const Way* OsmData::find_way(std::int64_t id) const { return find_by_id(ways, id); }
const Relation* OsmData::find_relation(std::int64_t id) const { return find_by_id(relations, id); }

std::optional<std::string_view> OsmData::tag(std::span<const Tag> element_tags, std::string_view key) const {
    for (const Tag& t : element_tags) {
        if (strings[t.key] == key) return std::string_view(strings[t.value]);
    }
    return std::nullopt;
}

Result<void> OsmData::validate() const {
    const auto bad = [](const char* what) { return make_error(ErrorCode::Malformed, std::string("osm: ") + what); };
    for (std::size_t i = 0; i < strings.size(); ++i) {
        if (strings[i].size() > kMaxStringLength) return bad("string too long");
        if (i > 0 && !(strings[i - 1] < strings[i])) return bad("strings not sorted/unique");
    }
    if (!ids_ascending(nodes) || !ids_ascending(ways) || !ids_ascending(relations)) return bad("ids not sorted/unique");
    const std::size_t ns = strings.size();
    std::size_t tag_pos = 0;
    const auto check_tags = [&](std::uint32_t begin, std::uint32_t count) -> bool {
        if (begin != tag_pos || count > tags.size() - tag_pos) return false;
        for (std::uint32_t k = 0; k < count; ++k) {
            const Tag& t = tags[begin + k];
            if (t.key >= ns || t.value >= ns) return false;
            if (k > 0 && !(tags[begin + k - 1].key < t.key)) return false;
        }
        tag_pos += count;
        return true;
    };
    for (const Node& n : nodes) {
        if (n.lat < -kMaxLatE7 || n.lat > kMaxLatE7 || n.lon < -kMaxLonE7 || n.lon > kMaxLonE7) {
            return bad("coordinate out of range");
        }
        if (!check_tags(n.tags_begin, n.tags_count)) return bad("node tags");
    }
    std::size_t ref_pos = 0;
    for (const Way& w : ways) {
        if (!check_tags(w.tags_begin, w.tags_count)) return bad("way tags");
        if (w.refs_begin != ref_pos || w.refs_count == 0 || w.refs_count > way_refs.size() - ref_pos) {
            return bad("way refs layout");
        }
        for (std::int64_t ref : refs_of(w)) {
            if (!find_node(ref)) return bad("way ref to a node not in the set");
        }
        ref_pos += w.refs_count;
    }
    if (ref_pos != way_refs.size()) return bad("unreferenced way refs");
    std::size_t mem_pos = 0;
    for (const Relation& r : relations) {
        if (!check_tags(r.tags_begin, r.tags_count)) return bad("relation tags");
        if (r.members_begin != mem_pos || r.members_count > members.size() - mem_pos) return bad("members layout");
        for (const Member& m : members_of(r)) {
            if (static_cast<std::uint8_t>(m.type) > 2 || m.role >= ns) return bad("member");
        }
        mem_pos += r.members_count;
    }
    if (mem_pos != members.size()) return bad("unreferenced members");
    if (tag_pos != tags.size()) return bad("unreferenced tags");
    return {};
}

// ---- builder ----

void OsmBuilder::add_node(std::int64_t id, std::int32_t lat_e7, std::int32_t lon_e7, TagList tags) {
    nodes_.push_back(NodeIn{id, lat_e7, lon_e7, std::move(tags)});
}
void OsmBuilder::add_way(std::int64_t id, std::vector<std::int64_t> refs, TagList tags) {
    ways_.push_back(WayIn{id, std::move(refs), std::move(tags)});
}
void OsmBuilder::add_relation(std::int64_t id, std::vector<MemberIn> members, TagList tags) {
    rels_.push_back(RelIn{id, std::move(members), std::move(tags)});
}

Result<OsmData> OsmBuilder::build() const {
    const auto bad = [](const std::string& what) { return make_error(ErrorCode::InvalidArgument, "osm builder: " + what); };
    // Intern every string, sorted.
    std::vector<std::string> strs;
    const auto collect = [&](const TagList& tl) {
        for (const auto& [k, v] : tl) {
            strs.push_back(k);
            strs.push_back(v);
        }
    };
    for (const auto& n : nodes_) collect(n.tags);
    for (const auto& w : ways_) collect(w.tags);
    for (const auto& r : rels_) {
        collect(r.tags);
        for (const auto& m : r.members) strs.push_back(m.role);
    }
    std::sort(strs.begin(), strs.end());
    strs.erase(std::unique(strs.begin(), strs.end()), strs.end());
    for (const auto& s : strs) {
        if (s.size() > kMaxStringLength) return bad("string longer than 65535 bytes");
    }
    const auto idx = [&](const std::string& s) {
        return static_cast<std::uint32_t>(std::lower_bound(strs.begin(), strs.end(), s) - strs.begin());
    };

    OsmData d;
    d.strings = strs;
    const auto sorted_order = [](const auto& v) {
        std::vector<std::size_t> o(v.size());
        for (std::size_t i = 0; i < o.size(); ++i) o[i] = i;
        std::stable_sort(o.begin(), o.end(), [&](std::size_t a, std::size_t b) { return v[a].id < v[b].id; });
        return o;
    };
    const auto add_tags = [&](const TagList& tl, std::uint32_t& begin, std::uint32_t& count) -> bool {
        std::vector<Tag> t;
        for (const auto& [k, v] : tl) t.push_back(Tag{idx(k), idx(v)});
        std::sort(t.begin(), t.end(), [](const Tag& a, const Tag& b) { return a.key < b.key; });
        for (std::size_t i = 1; i < t.size(); ++i) {
            if (t[i - 1].key == t[i].key) return false;
        }
        begin = static_cast<std::uint32_t>(d.tags.size());
        count = static_cast<std::uint32_t>(t.size());
        d.tags.insert(d.tags.end(), t.begin(), t.end());
        return true;
    };

    for (std::size_t i : sorted_order(nodes_)) {
        const NodeIn& in = nodes_[i];
        if (!d.nodes.empty() && d.nodes.back().id == in.id) return bad("duplicate node " + std::to_string(in.id));
        if (in.lat < -kMaxLatE7 || in.lat > kMaxLatE7 || in.lon < -kMaxLonE7 || in.lon > kMaxLonE7) {
            return bad("node " + std::to_string(in.id) + ": coordinate out of range");
        }
        Node n{in.id, in.lat, in.lon, 0, 0};
        if (!add_tags(in.tags, n.tags_begin, n.tags_count)) return bad("duplicate tag key on node " + std::to_string(in.id));
        d.nodes.push_back(n);
    }
    for (std::size_t i : sorted_order(ways_)) {
        const WayIn& in = ways_[i];
        if (!d.ways.empty() && d.ways.back().id == in.id) return bad("duplicate way " + std::to_string(in.id));
        if (in.refs.empty()) return bad("way " + std::to_string(in.id) + " has no refs");
        Way w{in.id, static_cast<std::uint32_t>(d.way_refs.size()), static_cast<std::uint32_t>(in.refs.size()), 0, 0};
        for (std::int64_t ref : in.refs) {
            if (!d.find_node(ref)) return bad("way " + std::to_string(in.id) + " refers to missing node " + std::to_string(ref));
        }
        d.way_refs.insert(d.way_refs.end(), in.refs.begin(), in.refs.end());
        if (!add_tags(in.tags, w.tags_begin, w.tags_count)) return bad("duplicate tag key on way " + std::to_string(in.id));
        d.ways.push_back(w);
    }
    for (std::size_t i : sorted_order(rels_)) {
        const RelIn& in = rels_[i];
        if (!d.relations.empty() && d.relations.back().id == in.id) {
            return bad("duplicate relation " + std::to_string(in.id));
        }
        Relation r{in.id, static_cast<std::uint32_t>(d.members.size()), static_cast<std::uint32_t>(in.members.size()), 0, 0};
        for (const MemberIn& m : in.members) {
            if (static_cast<std::uint8_t>(m.type) > 2) return bad("bad member type");
            d.members.push_back(Member{m.type, m.ref, idx(m.role)});
        }
        if (!add_tags(in.tags, r.tags_begin, r.tags_count)) {
            return bad("duplicate tag key on relation " + std::to_string(in.id));
        }
        d.relations.push_back(r);
    }
    return d;
}

} // namespace g2m::osm
