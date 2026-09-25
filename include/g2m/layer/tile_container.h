// g2m/layer/tile_container.h - the g2m.tile/1 container (PLAN.md 3.4;
// byte layout: docs/formats/tile_container.md).
//
// A container is HEADER || BODY:
//  - the BODY is the layer data: an encoding id/version and typed sections,
//    each stored raw or as one zstd frame. It depends only on the tile's
//    content, never on the release, so identical content across releases
//    has identical bytes. body_hash = SHA-256(BODY) is the ETag and the
//    dedup key of every store.
//  - the HEADER names the tile (layer, version, release id, scheme/level/
//    x/y), its height datum, the content hashes of the inputs it was derived
//    from, and the body hash/size. It ends with an FNV-1a 64 checksum of the
//    header bytes before it.
//
// Encoders are deterministic (same input -> same bytes on every supported
// compiler/OS). Decoders take untrusted bytes: they never read out of
// bounds, never allocate more than the documented limits, and report every
// problem as an Error (no exceptions). Every decoder accepts the canonical
// form only where that is cheap to check (e.g. ascending section types),
// so that decode -> encode -> decode is stable.
#pragma once

#include "g2m/core/result.h"
#include "g2m/core/tile_key.h"

#include <array>
#include <cstdint>
#include <span>
#include <string>
#include <vector>

namespace g2m {

using ContentHash = std::array<std::uint8_t, 32>; // SHA-256
using ReleaseId = ContentHash;

ContentHash content_hash(std::span<const std::uint8_t> bytes);

inline constexpr std::uint8_t kContainerMajor = 1;
inline constexpr std::uint8_t kContainerMinor = 0;

// Decoder limits (docs/formats/tile_container.md).
inline constexpr std::size_t kMaxSections = 64;
inline constexpr std::size_t kMaxSectionRawSize = std::size_t{64} << 20;   // 64 MiB
inline constexpr std::size_t kMaxBodyRawSize = std::size_t{256} << 20;     // sum over sections
inline constexpr std::size_t kMaxInputHashes = 4096;

enum class VerticalDatum : std::uint16_t {
    None = 0,      // not a height layer
    DHHN2016 = 1,  // German normal heights (DGM1)
    EGM2008 = 2,   // global geoid (GLO-30)
};

enum class SectionCodec : std::uint8_t {
    None = 0,
    Zstd = 1,
};

struct Section {
    std::uint16_t type = 0;
    SectionCodec codec = SectionCodec::Zstd; // how encode_body stores it
    std::vector<std::uint8_t> data;          // raw (uncompressed) bytes

    friend bool operator==(const Section&, const Section&) = default;
};

struct TileBody {
    std::uint16_t encoding = 0;
    std::uint16_t encoding_version = 0;
    std::vector<Section> sections; // strictly ascending type

    const Section* find(std::uint16_t type) const;
    friend bool operator==(const TileBody&, const TileBody&) = default;
};

// Fails if section types are not strictly ascending, there are too many
// sections, or a size limit is exceeded.
Result<std::vector<std::uint8_t>> encode_body(const TileBody& body);
Result<TileBody> decode_body(std::span<const std::uint8_t> bytes);

struct TileHeader {
    std::string layer;                   // valid layer name
    std::uint32_t layer_version = 0;
    ReleaseId rid{};                     // all zero = not yet assigned to a release (source store rows)
    TileKey key;                         // scheme utm{zone}{n|s}, level, x, y
    VerticalDatum datum = VerticalDatum::None;
    std::int32_t height_offset = 0;      // 1/256 m, added to every stored height (0 for raw layers)
    std::vector<ContentHash> inputs;     // body hashes of the input tiles; sorted, unique
    ContentHash body_hash{};
    std::uint64_t body_size = 0;

    friend bool operator==(const TileHeader&, const TileHeader&) = default;
};

// Fails on an invalid layer name, key, or unsorted/duplicate inputs.
Result<std::vector<std::uint8_t>> encode_header(const TileHeader& header);
// Decodes a header at the start of bytes; *consumed gets its size.
Result<TileHeader> decode_header(std::span<const std::uint8_t> bytes, std::size_t* consumed = nullptr);

// HEADER || BODY. header.body_hash/body_size are set from body here.
Result<std::vector<std::uint8_t>> assemble_container(TileHeader header, std::span<const std::uint8_t> body);

struct ContainerView {
    TileHeader header;
    std::span<const std::uint8_t> body; // points into the parsed bytes
};
// Parses HEADER || BODY and verifies body size and SHA-256. Does not decode
// the body (see decode_body).
Result<ContainerView> parse_container(std::span<const std::uint8_t> bytes);

} // namespace g2m
