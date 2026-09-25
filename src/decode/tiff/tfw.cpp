// src/decode/tiff/tfw.cpp - see g2m/decode/tiff/tfw.h.

#include "g2m/decode/tiff/tfw.h"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdlib>
#include <vector>

namespace g2m::decode {

std::optional<GeoTransform> parse_tfw(const std::string& text) {
    std::vector<double> values;
    values.reserve(6);
    std::size_t i = 0;
    const std::size_t n = text.size();
    while (i < n && values.size() < 6) {
        while (i < n && std::isspace(static_cast<unsigned char>(text[i]))) ++i;
        if (i >= n) break;
        const std::size_t start = i;
        while (i < n && !std::isspace(static_cast<unsigned char>(text[i]))) ++i;
        const std::string tok = text.substr(start, i - start);
        char* end = nullptr;
        const double v = std::strtod(tok.c_str(), &end);
        if (end == tok.c_str() || *end != '\0') return std::nullopt; // not a clean number
        values.push_back(v);
    }
    if (values.size() != 6) return std::nullopt;

    const double a = values[0]; // pixel width
    const double d = values[1]; // rotation (must be 0)
    const double b = values[2]; // rotation (must be 0)
    const double e = values[3]; // pixel height, negative for north-up
    const double c = values[4]; // origin x (upper-left pixel centre)
    const double f = values[5]; // origin y (upper-left pixel centre)

    constexpr double kRotationTol = 1e-12;
    if (std::fabs(d) > kRotationTol || std::fabs(b) > kRotationTol) return std::nullopt;
    if (!(a > 0.0) || !(e < 0.0)) return std::nullopt; // only north-up, right-handed x supported
    constexpr double kSpacingTol = 1e-9;
    if (std::fabs(a - (-e)) > kSpacingTol * std::max(1.0, a)) return std::nullopt;

    GeoTransform t;
    t.origin_x = c;
    t.origin_y = f;
    t.pixel_width = a;
    t.pixel_height = -e;
    return t;
}

} // namespace g2m::decode
