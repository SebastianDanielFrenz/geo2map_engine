// src/route/turn_restriction.cpp - see g2m/route/turn_restriction.h.
#include "g2m/route/turn_restriction.h"

#include <algorithm>
#include <array>
#include <string_view>

namespace g2m::route {
namespace {

struct ValueInfo {
    std::string_view value;
    RestrictionKind kind;
};
constexpr std::array<ValueInfo, 10> kValues{{
    {"no_left_turn", RestrictionKind::No},     {"no_right_turn", RestrictionKind::No},
    {"no_straight_on", RestrictionKind::No},   {"no_u_turn", RestrictionKind::No},
    {"no_entry", RestrictionKind::No},         {"no_exit", RestrictionKind::No},
    {"only_left_turn", RestrictionKind::Only}, {"only_right_turn", RestrictionKind::Only},
    {"only_straight_on", RestrictionKind::Only}, {"only_u_turn", RestrictionKind::Only},
}};

// true if a ';'-separated list names a vehicle class a car belongs to.
bool lists_car(std::string_view list) {
    while (!list.empty()) {
        const auto cut = list.find(';');
        std::string_view item = list.substr(0, cut);
        while (!item.empty() && item.front() == ' ') item.remove_prefix(1);
        while (!item.empty() && item.back() == ' ') item.remove_suffix(1);
        if (item == "motorcar" || item == "motor_vehicle" || item == "vehicle") return true;
        if (cut == std::string_view::npos) break;
        list.remove_prefix(cut + 1);
    }
    return false;
}

} // namespace

Result<RestrictionSet> extract_turn_restrictions(const osm::OsmData& data, std::size_t max_restrictions) {
    RestrictionSet out;
    for (const osm::Relation& rel : data.relations) {
        const auto tags = data.tags_of(rel);
        const auto type = data.tag(tags, "type");
        if (!type || *type != "restriction") continue;
        ++out.relations_seen;
        auto value = data.tag(tags, "restriction:motorcar");
        if (!value) value = data.tag(tags, "restriction");
        if (!value) {
            bool other = false, conditional = false;
            for (const osm::Tag& t : tags) {
                const std::string_view key = data.str(t.key);
                if (key == "restriction:conditional") conditional = true;
                else if (key.starts_with("restriction:")) other = true;
            }
            if (conditional && !other) ++out.conditional_only;
            else if (other) ++out.not_for_cars;
            else ++out.malformed;
            continue;
        }
        if (const auto except = data.tag(tags, "except"); except && lists_car(*except)) {
            ++out.not_for_cars;
            continue;
        }
        const auto info = std::find_if(kValues.begin(), kValues.end(), [&](const ValueInfo& v) { return v.value == *value; });
        if (info == kValues.end()) {
            ++out.unknown_value;
            continue;
        }
        std::vector<std::int64_t> from, to, via_ways;
        std::vector<std::int64_t> via_nodes;
        bool bad = false;
        for (const osm::Member& m : data.members_of(rel)) {
            const std::string_view role = data.str(m.role);
            if (role == "from" || role == "to") {
                if (m.type != osm::MemberType::Way) { bad = true; break; }
                (role == "from" ? from : to).push_back(m.ref);
            } else if (role == "via") {
                if (m.type == osm::MemberType::Node) via_nodes.push_back(m.ref);
                else if (m.type == osm::MemberType::Way) via_ways.push_back(m.ref);
                else { bad = true; break; }
            } // other roles (location_hint, ...) are ignored
        }
        const bool multi_from = info->value == "no_entry", multi_to = info->value == "no_exit";
        if (bad || from.empty() || to.empty() || (from.size() > 1 && !multi_from) || (to.size() > 1 && !multi_to) ||
            via_nodes.size() + (via_ways.empty() ? 0u : 1u) != 1u || via_nodes.size() > 1) {
            ++out.malformed;
            continue;
        }
        for (const std::int64_t f : from) {
            for (const std::int64_t t : to) {
                if (out.restrictions.size() >= max_restrictions)
                    return make_error(ErrorCode::LimitExceeded, "turn restrictions: more than max_restrictions rules");
                TurnRestriction r;
                r.relation_id = rel.id;
                r.kind = info->kind;
                r.value = std::string(info->value);
                r.from_way = f;
                r.to_way = t;
                r.via_node = via_nodes.empty() ? 0 : via_nodes.front();
                r.via_ways = via_ways;
                out.restrictions.push_back(std::move(r));
            }
        }
    }
    // Relations are already ascending by id; sort within a relation and drop
    // repeated (from, to) pairs of one relation.
    std::sort(out.restrictions.begin(), out.restrictions.end(), [](const TurnRestriction& a, const TurnRestriction& b) {
        if (a.relation_id != b.relation_id) return a.relation_id < b.relation_id;
        if (a.from_way != b.from_way) return a.from_way < b.from_way;
        return a.to_way < b.to_way;
    });
    out.restrictions.erase(std::unique(out.restrictions.begin(), out.restrictions.end(),
                                       [](const TurnRestriction& a, const TurnRestriction& b) {
                                           return a.relation_id == b.relation_id && a.from_way == b.from_way &&
                                                  a.to_way == b.to_way;
                                       }),
                           out.restrictions.end());
    return out;
}

} // namespace g2m::route
