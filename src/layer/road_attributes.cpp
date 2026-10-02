#include "g2m/layer/road_attributes.h"
#include <algorithm>
#include <charconv>

namespace g2m {
namespace {
std::string_view trim(std::string_view text) {
    while (!text.empty() && (text.front() == ' ' || text.front() == '\t')) text.remove_prefix(1);
    while (!text.empty() && (text.back() == ' ' || text.back() == '\t')) text.remove_suffix(1);
    return text;
}
// Decimal-to-fixed-point without locale, libc floating point or overflow.
// Round half away from zero; at most six fractional digits are accepted.
std::optional<std::int32_t> decimal(std::string_view text, std::int32_t scale) {
    text = trim(text); bool negative = false;
    if (!text.empty() && (text.front() == '+' || text.front() == '-')) {
        negative = text.front() == '-'; text.remove_prefix(1);
    }
    std::int64_t value = 0, divisor = 1; bool fraction = false, digit = false;
    for (const char c : text) {
        if (c == '.' && !fraction) { fraction = true; continue; }
        if (c < '0' || c > '9' || value > 1000000000 || (fraction && divisor == 1000000)) return {};
        digit = true; value = value * 10 + c - '0'; if (fraction) divisor *= 10;
    }
    if (!digit) return {};
    value = (value * scale + divisor / 2) / divisor;
    if (value > 2147483647) return {};
    return static_cast<std::int32_t>(negative ? -value : value);
}
std::string_view tag(const RoadGraphWay& way, std::string_view key) {
    auto it = std::lower_bound(way.tags.begin(), way.tags.end(), key,
        [](const auto& item, auto name) { return item.first < name; });
    return it != way.tags.end() && it->first == key ? std::string_view(it->second) : std::string_view{};
}
bool structure(std::string_view value) { return !value.empty() && value != "no" && value != "0" && value != "false"; }
}
Result<RoadAttributes> resolve_road_attributes(const RoadGraphWay& way, const RoadStyle& style) {
    for (std::size_t i = 0; i < way.tags.size(); ++i)
        if (way.tags[i].first.empty() || (i && way.tags[i - 1].first >= way.tags[i].first))
            return make_error(ErrorCode::Malformed, "road attributes: noncanonical tags");
    const auto* highway = style.find_highway(tag(way, "highway"));
    if (!highway || tag(way, "area") == "yes")
        return make_error(ErrorCode::Unsupported, "road attributes: unsupported highway/area");
    RoadAttributes out;
    // The built-in defaults are exact at mm resolution. A custom table must
    // provide physically bounded finite widths before integer conversion.
    if (!(highway->default_width_m >= 0.5 && highway->default_width_m <= 100.0))
        return make_error(ErrorCode::InvalidArgument, "road attributes: default width range");
    out.width_mm = static_cast<std::int32_t>(highway->default_width_m * 1000.0 + 0.5);
    out.land_class = highway->land_class;
    const auto lane_text = trim(tag(way, "lanes"));
    std::int32_t lane_count = 0;
    if (!lane_text.empty()) {
        const auto parsed = std::from_chars(lane_text.data(), lane_text.data() + lane_text.size(), lane_count);
        if (parsed.ec == std::errc{} && parsed.ptr == lane_text.data() + lane_text.size() &&
            lane_count >= 1 && lane_count <= 30) out.lanes = lane_count;
    }
    if (out.lanes && out.lanes * 3250 <= 100000) out.width_mm = out.lanes * 3250;
    auto width_text = trim(tag(way, "width"));
    if (width_text.ends_with(" m")) width_text = trim(width_text.substr(0, width_text.size() - 2));
    if (const auto width = decimal(width_text, 1000); width && *width >= 500 && *width <= 100000) {
        out.width_mm = *width; out.width_inferred = false;
    }
    if (const auto* surface = style.find_surface(tag(way, "surface"))) {
        out.surface = surface->surface_kind; out.land_class = surface->land_class; out.surface_inferred = false;
    }
    out.bridge = structure(tag(way, "bridge")); out.tunnel = structure(tag(way, "tunnel"));
    // Class data: motorway/trunk 6%, primary/secondary/tertiary 12%,
    // local roads 15%, service/track 20%; blend grows with class importance.
    constexpr std::int32_t grades[] = {200, 200, 150, 150, 150, 120, 120, 120, 60, 60};
    constexpr std::int32_t blends[] = {3000, 3000, 5000, 5000, 5000, 8000, 10000, 12000, 15000, 15000};
    constexpr std::int32_t vertical_radii[] = {50, 80, 100, 150, 200, 300, 500, 750, 1000, 1500};
    constexpr std::int32_t horizontal_radii[] = {6, 6, 10, 15, 25, 40, 60, 100, 250, 500};
    if (highway->rank < 0 || highway->rank > 9)
        return make_error(ErrorCode::InvalidArgument, "road attributes: unsupported class rank");
    out.max_grade_per_mille = grades[highway->rank]; out.blend_mm = blends[highway->rank];
    out.min_vertical_radius_m = vertical_radii[highway->rank];
    out.min_horizontal_radius_m = horizontal_radii[highway->rank];
    auto incline = trim(tag(way, "incline"));
    if (incline.ends_with('%')) incline = trim(incline.substr(0, incline.size() - 1));
    if (const auto slope = decimal(incline, 10); slope && *slope >= -1000 && *slope <= 1000)
        out.incline_per_mille = *slope;
    return out;
}
} // namespace g2m
