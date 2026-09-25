// g2m/decode/tiff/tfw.h - Esri "world file" (.tfw/.wld) parser (PLAN.md 13,
// milestone G1b): the fallback georeference for a DGM1 GeoTIFF that ships
// without GeoTIFF GeoKeys.
#pragma once

#include "g2m/decode/tiff/geotransform.h"

#include <optional>
#include <string>

namespace g2m::decode {

// A world file is six whitespace/newline-separated ASCII numbers, one per
// line, in this fixed order (the usual convention; no rotation is expected
// in any source this library targets, so a nonzero rotation term throws
// rather than being silently dropped):
//   line 1: pixel size in the x-direction (A)
//   line 2: rotation about y-axis (D) - must be 0
//   line 3: rotation about x-axis (B) - must be 0
//   line 4: pixel size in the y-direction, NEGATIVE for a north-up image (E)
//   line 5: x-coordinate of the CENTRE of the upper-left pixel (C)
//   line 6: y-coordinate of the CENTRE of the upper-left pixel (F)
//
// Returns nullopt if the text does not parse as six finite numbers, or if
// a rotation term is nonzero, or if the pixel sizes are not exactly equal
// in magnitude (RasterGeoref requires one spacing for both axes).
std::optional<GeoTransform> parse_tfw(const std::string& text);

} // namespace g2m::decode
