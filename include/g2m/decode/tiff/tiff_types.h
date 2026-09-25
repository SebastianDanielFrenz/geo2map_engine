// g2m/decode/tiff/tiff_types.h - TIFF/GeoTIFF tag value enums (PLAN.md 13,
// milestone G1b). TIFF 6.0 + BigTIFF (Adobe PageMaker 6.0 extension) +
// GeoTIFF 1.0/1.1 subset - only the values this reader actually supports.
#pragma once

#include <cstdint>

namespace g2m::decode::tiff {

enum class Compression : std::uint16_t {
    None = 1,
    Lzw = 5,
    PackBits = 32773,
    Deflate = 8,     // Adobe/zip-in-tiff (the common one)
    DeflateOld = 32946, // pre-standardisation tag value some encoders still emit
};

enum class SampleFormat : std::uint16_t {
    UnsignedInt = 1, // default if the tag is absent
    SignedInt = 2,
    Float = 3,
};

enum class Predictor : std::uint16_t {
    None = 1,
    HorizontalDifferencing = 2,
    FloatingPoint = 3,
};

enum class PlanarConfig : std::uint16_t {
    Chunky = 1,
    Planar = 2,
};

// GeoTIFF GeoKeyDirectory key ids this reader recognises (GeoTIFF 1.1
// section B).
namespace geokey {
inline constexpr std::uint16_t kGTModelTypeGeoKey = 1024;
inline constexpr std::uint16_t kGTRasterTypeGeoKey = 1025; // 1 = PixelIsArea, 2 = PixelIsPoint
inline constexpr std::uint16_t kGeographicTypeGeoKey = 2048;
inline constexpr std::uint16_t kProjectedCSTypeGeoKey = 3072;
} // namespace geokey

enum class RasterType : std::uint16_t {
    Unknown = 0,
    PixelIsArea = 1,
    PixelIsPoint = 2,
};

// Baseline TIFF tag ids used by this reader.
namespace tag {
inline constexpr std::uint16_t kImageWidth = 256;
inline constexpr std::uint16_t kImageLength = 257;
inline constexpr std::uint16_t kBitsPerSample = 258;
inline constexpr std::uint16_t kCompression = 259;
inline constexpr std::uint16_t kPhotometricInterpretation = 262;
inline constexpr std::uint16_t kStripOffsets = 273;
inline constexpr std::uint16_t kSamplesPerPixel = 277;
inline constexpr std::uint16_t kRowsPerStrip = 278;
inline constexpr std::uint16_t kStripByteCounts = 279;
inline constexpr std::uint16_t kPlanarConfiguration = 284;
inline constexpr std::uint16_t kPredictor = 317;
inline constexpr std::uint16_t kTileWidth = 322;
inline constexpr std::uint16_t kTileLength = 323;
inline constexpr std::uint16_t kTileOffsets = 324;
inline constexpr std::uint16_t kTileByteCounts = 325;
inline constexpr std::uint16_t kSampleFormat = 339;
inline constexpr std::uint16_t kModelPixelScaleTag = 33550;
inline constexpr std::uint16_t kModelTiepointTag = 33922;
inline constexpr std::uint16_t kModelTransformationTag = 34264;
inline constexpr std::uint16_t kGeoKeyDirectoryTag = 34735;
inline constexpr std::uint16_t kGeoDoubleParamsTag = 34736;
inline constexpr std::uint16_t kGeoAsciiParamsTag = 34737;
inline constexpr std::uint16_t kGdalNoData = 42113;
} // namespace tag

} // namespace g2m::decode::tiff
