// g2m/layer/road_tier.h - road clearing tiers (owner decision, racing_game): the physics surface classes of paved
// roads are one class per clearing tier, decided per road segment from the way's `highway` value and `motorroad=yes`.
// The tier set and the highway/motorroad mapping are DATA owned by the caller: it hands geo2map an ordered rule list
// (RoadTierMap) and the first rule that matches a way gives its tier; a paved way that matches no rule gets
// `default_tier` (by convention the last tier). Tiers are zero-based (tier 0 = T1 ... tier 5 = T6). Unpaved roads and
// non-road land have no tier (kNoRoadTier); RoadSegment::tier is then 0 and ignored by RoadValueLut. geo2map itself
// knows no highway names here - there is no built-in map. Without a map every segment has tier 0 (the old behaviour).
#pragma once

#include "g2m/layer/classifier.h"
#include "g2m/layer/road_graph.h"

#include <cstdint>
#include <string>
#include <string_view>
#include <vector>

namespace g2m {

// "No tier": an unpaved road, non-road land, or no tier map supplied (ribbon vertices).
inline constexpr std::uint8_t kNoRoadTier = 255;
// RoadValueLut holds this many tier slots (tier 0..7). A larger tier is clamped to the last slot by the LUT.
inline constexpr std::size_t kRoadTierSlots = 8;

struct RoadTierRule {
    std::uint8_t tier = 0;
    // `highway` tag values this rule matches, e.g. {"motorway", "motorway_link"}; empty = any highway value.
    std::vector<std::string> highways;
    // motorroad=yes (Kraftfahrstrasse): require it / exclude it. Absent tag or any value but "yes" = not a motorroad.
    bool require_motorroad_yes = false;
    bool exclude_motorroad_yes = false;
    friend bool operator==(const RoadTierRule&, const RoadTierRule&) = default;
};

struct RoadTierMap {
    std::vector<RoadTierRule> rules; // ordered, first match wins
    std::uint8_t default_tier = 0;   // a paved way matching no rule
    friend bool operator==(const RoadTierMap&, const RoadTierMap&) = default;

    // Pure and allocation-free. The tier of a way with this `highway` value; no paved/unpaved decision here.
    std::uint8_t tier_for(std::string_view highway, bool motorroad_yes) const noexcept {
        for (const RoadTierRule& rule : rules) {
            if (rule.require_motorroad_yes && !motorroad_yes) continue;
            if (rule.exclude_motorroad_yes && motorroad_yes) continue;
            if (!rule.highways.empty()) {
                bool hit = false;
                for (const std::string& h : rule.highways) {
                    if (h == highway) { hit = true; break; }
                }
                if (!hit) continue;
            }
            return rule.tier;
        }
        return default_tier;
    }

    // The tier of a way: kNoRoadTier for an unpaved road (`land_class != PavedRoad`), else tier_for.
    std::uint8_t tier_of(std::string_view highway, bool motorroad_yes, LandClass land_class) const noexcept {
        return land_class == LandClass::PavedRoad ? tier_for(highway, motorroad_yes) : kNoRoadTier;
    }
};

// The tier of a graph way (tags read: highway, motorroad). `land_class` is the way's resolved class
// (RoadAttributes::land_class: the highway default with any `surface` override applied).
inline std::uint8_t road_tier_of(const RoadTierMap& map, const RoadGraphWay& way, LandClass land_class) noexcept {
    std::string_view highway;
    bool motorroad = false;
    for (const auto& [key, value] : way.tags) {
        if (key == "highway") highway = value;
        else if (key == "motorroad") motorroad = value == "yes";
    }
    return map.tier_of(highway, motorroad, land_class);
}

} // namespace g2m
