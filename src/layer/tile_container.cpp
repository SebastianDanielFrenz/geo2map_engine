// src/layer/tile_container.cpp - see g2m/layer/tile_container.h and
// docs/formats/tile_container.md.

#include "g2m/layer/tile_container.h"

#include "g2m/core/bytes.h"
#include "g2m/core/hash.h"
#include "g2m/layer/layer_id.h"
#include "zstd_codec.h"

#include <algorithm>
#include <cstring>

namespace g2m {
namespace {

constexpr std::uint8_t kMagic[4] = {'G', '2', 'M', 'T'};
constexpr std::uint8_t kSchemeUtm = 1;
constexpr std::size_t kSectionEntrySize = 12;

bool strictly_ascending(const std::vector<ContentHash>& v) {
    for (std::size_t i = 1; i < v.size(); ++i) {
        if (!(v[i - 1] < v[i])) return false;
    }
    return true;
}

} // namespace

ContentHash content_hash(std::span<const std::uint8_t> bytes) {
    return Sha256::of(std::as_bytes(bytes));
}

const Section* TileBody::find(std::uint16_t type) const {
    for (const Section& s : sections) {
        if (s.type == type) return &s;
    }
    return nullptr;
}

Result<std::vector<std::uint8_t>> encode_body(const TileBody& body) {
    if (body.sections.size() > kMaxSections) return make_error(ErrorCode::LimitExceeded, "too many sections");
    std::size_t total_raw = 0;
    for (std::size_t i = 0; i < body.sections.size(); ++i) {
        const Section& s = body.sections[i];
        if (i > 0 && !(body.sections[i - 1].type < s.type)) {
            return make_error(ErrorCode::InvalidArgument, "section types must be strictly ascending");
        }
        if (s.codec != SectionCodec::None && s.codec != SectionCodec::Zstd) {
            return make_error(ErrorCode::InvalidArgument, "unknown section codec");
        }
        if (s.data.size() > kMaxSectionRawSize) return make_error(ErrorCode::LimitExceeded, "section too large");
        total_raw += s.data.size();
    }
    if (total_raw > kMaxBodyRawSize) return make_error(ErrorCode::LimitExceeded, "body too large");

    std::vector<std::vector<std::uint8_t>> stored;
    stored.reserve(body.sections.size());
    for (const Section& s : body.sections) {
        stored.push_back(s.codec == SectionCodec::Zstd ? layer_detail::zstd_compress(s.data) : s.data);
    }
    ByteWriter w;
    w.u16(body.encoding);
    w.u16(body.encoding_version);
    w.u16(static_cast<std::uint16_t>(body.sections.size()));
    w.u16(0);
    for (std::size_t i = 0; i < body.sections.size(); ++i) {
        const Section& s = body.sections[i];
        w.u16(s.type);
        w.u8(static_cast<std::uint8_t>(s.codec));
        w.u8(0);
        w.u32(static_cast<std::uint32_t>(s.data.size()));
        w.u32(static_cast<std::uint32_t>(stored[i].size()));
    }
    for (const auto& st : stored) w.bytes(st);
    return w.take();
}

Result<TileBody> decode_body(std::span<const std::uint8_t> bytes) {
    ByteReader r(bytes);
    TileBody body;
    body.encoding = r.u16();
    body.encoding_version = r.u16();
    const std::uint16_t n = r.u16();
    const std::uint16_t reserved = r.u16();
    if (!r.ok()) return make_error(ErrorCode::Truncated, "body header truncated");
    if (reserved != 0) return make_error(ErrorCode::Malformed, "body reserved field != 0");
    if (n > kMaxSections) return make_error(ErrorCode::LimitExceeded, "too many sections");
    if (static_cast<std::size_t>(n) * kSectionEntrySize > r.remaining()) {
        return make_error(ErrorCode::Truncated, "section table truncated");
    }
    struct Entry {
        std::uint16_t type;
        std::uint8_t codec;
        std::uint32_t raw, stored;
    };
    std::vector<Entry> table(n);
    std::size_t total_raw = 0;
    std::size_t total_stored = 0;
    for (std::size_t i = 0; i < n; ++i) {
        Entry& e = table[i];
        e.type = r.u16();
        e.codec = r.u8();
        const std::uint8_t res = r.u8();
        e.raw = r.u32();
        e.stored = r.u32();
        if (res != 0) return make_error(ErrorCode::Malformed, "section reserved field != 0");
        if (i > 0 && !(table[i - 1].type < e.type)) {
            return make_error(ErrorCode::Malformed, "section types not strictly ascending");
        }
        if (e.codec > static_cast<std::uint8_t>(SectionCodec::Zstd)) {
            return make_error(ErrorCode::Unsupported, "unknown section codec");
        }
        if (e.codec == static_cast<std::uint8_t>(SectionCodec::None) && e.raw != e.stored) {
            return make_error(ErrorCode::Malformed, "uncompressed section: raw size != stored size");
        }
        if (e.raw > kMaxSectionRawSize) return make_error(ErrorCode::LimitExceeded, "section too large");
        total_raw += e.raw;
        total_stored += e.stored;
    }
    if (total_raw > kMaxBodyRawSize) return make_error(ErrorCode::LimitExceeded, "body too large");
    if (total_stored != r.remaining()) {
        return make_error(total_stored > r.remaining() ? ErrorCode::Truncated : ErrorCode::Malformed,
                          "section sizes do not add up to the body size");
    }
    body.sections.reserve(n);
    for (const Entry& e : table) {
        const auto payload = r.bytes(e.stored);
        Section s;
        s.type = e.type;
        s.codec = static_cast<SectionCodec>(e.codec);
        if (s.codec == SectionCodec::Zstd) {
            auto raw = layer_detail::zstd_decompress(payload, e.raw);
            if (!raw) return raw.error();
            s.data = std::move(raw).value();
        } else {
            s.data.assign(payload.begin(), payload.end());
        }
        body.sections.push_back(std::move(s));
    }
    return body;
}

Result<std::vector<std::uint8_t>> encode_header(const TileHeader& h) {
    if (!is_valid_layer_name(h.layer)) return make_error(ErrorCode::InvalidArgument, "invalid layer name");
    if (!h.key.valid()) return make_error(ErrorCode::InvalidArgument, "invalid tile key");
    if (static_cast<std::uint16_t>(h.datum) > static_cast<std::uint16_t>(VerticalDatum::EGM2008)) {
        return make_error(ErrorCode::InvalidArgument, "unknown vertical datum");
    }
    if (h.inputs.size() > kMaxInputHashes || !strictly_ascending(h.inputs)) {
        return make_error(ErrorCode::InvalidArgument, "input hashes must be sorted, unique and at most 4096");
    }
    ByteWriter w;
    w.bytes(kMagic, 4);
    w.u8(kContainerMajor);
    w.u8(kContainerMinor);
    w.u16(0); // flags
    w.u8(static_cast<std::uint8_t>(h.layer.size()));
    w.bytes(h.layer.data(), h.layer.size());
    w.u32(h.layer_version);
    w.bytes(h.rid);
    w.u8(kSchemeUtm);
    w.u8(static_cast<std::uint8_t>(h.key.zone.zone));
    w.u8(static_cast<std::uint8_t>(h.key.zone.hemisphere));
    w.u8(static_cast<std::uint8_t>(h.key.level));
    w.i32(h.key.x);
    w.i32(h.key.y);
    w.u16(static_cast<std::uint16_t>(h.datum));
    w.i32(h.height_offset);
    w.u16(static_cast<std::uint16_t>(h.inputs.size()));
    for (const ContentHash& in : h.inputs) w.bytes(in);
    w.bytes(h.body_hash);
    w.u64(h.body_size);
    const std::uint64_t checksum = Fnv1a64().bytes(w.data().data(), w.size()).value();
    w.u64(checksum);
    return w.take();
}

Result<TileHeader> decode_header(std::span<const std::uint8_t> bytes, std::size_t* consumed) {
    ByteReader r(bytes);
    const auto magic = r.bytes(4);
    if (!r.ok()) return make_error(ErrorCode::Truncated, "header truncated");
    if (std::memcmp(magic.data(), kMagic, 4) != 0) return make_error(ErrorCode::BadMagic, "not a g2m.tile container");
    const std::uint8_t major = r.u8();
    const std::uint8_t minor = r.u8();
    const std::uint16_t flags = r.u16();
    if (!r.ok()) return make_error(ErrorCode::Truncated, "header truncated");
    if (major != kContainerMajor || minor != kContainerMinor) {
        return make_error(ErrorCode::Unsupported, "container version " + std::to_string(major) + "." + std::to_string(minor));
    }
    if (flags != 0) return make_error(ErrorCode::Unsupported, "unknown container flags");
    TileHeader h;
    const std::uint8_t name_len = r.u8();
    h.layer = std::string(r.string(name_len));
    h.layer_version = r.u32();
    const auto rid = r.bytes(32);
    const std::uint8_t scheme = r.u8();
    const std::uint8_t zone = r.u8();
    const std::uint8_t hemi = r.u8();
    const std::uint8_t level = r.u8();
    h.key.x = r.i32();
    h.key.y = r.i32();
    const std::uint16_t datum = r.u16();
    h.height_offset = r.i32();
    const std::uint16_t n_inputs = r.u16();
    if (!r.ok()) return make_error(ErrorCode::Truncated, "header truncated");
    std::copy(rid.begin(), rid.end(), h.rid.begin());
    if (!is_valid_layer_name(h.layer)) return make_error(ErrorCode::Malformed, "invalid layer name");
    if (scheme != kSchemeUtm) return make_error(ErrorCode::Unsupported, "unknown tile scheme");
    if (hemi > 1) return make_error(ErrorCode::Malformed, "bad hemisphere");
    h.key.zone = geo::UtmZone{zone, static_cast<geo::Hemisphere>(hemi)};
    h.key.level = level;
    if (!h.key.valid()) return make_error(ErrorCode::Malformed, "invalid tile key");
    if (datum > static_cast<std::uint16_t>(VerticalDatum::EGM2008)) {
        return make_error(ErrorCode::Unsupported, "unknown vertical datum");
    }
    h.datum = static_cast<VerticalDatum>(datum);
    if (n_inputs > kMaxInputHashes) return make_error(ErrorCode::LimitExceeded, "too many input hashes");
    if (static_cast<std::size_t>(n_inputs) * 32 > r.remaining()) return make_error(ErrorCode::Truncated, "header truncated");
    h.inputs.resize(n_inputs);
    for (ContentHash& in : h.inputs) {
        const auto b = r.bytes(32);
        std::copy(b.begin(), b.end(), in.begin());
    }
    if (!strictly_ascending(h.inputs)) return make_error(ErrorCode::Malformed, "input hashes not sorted/unique");
    const auto bh = r.bytes(32);
    h.body_size = r.u64();
    const std::size_t checked = r.pos();
    const std::uint64_t checksum = r.u64();
    if (!r.ok()) return make_error(ErrorCode::Truncated, "header truncated");
    std::copy(bh.begin(), bh.end(), h.body_hash.begin());
    if (Fnv1a64().bytes(bytes.data(), checked).value() != checksum) {
        return make_error(ErrorCode::HashMismatch, "header checksum mismatch");
    }
    if (consumed) *consumed = r.pos();
    return h;
}

Result<std::vector<std::uint8_t>> assemble_container(TileHeader header, std::span<const std::uint8_t> body) {
    header.body_hash = content_hash(body);
    header.body_size = body.size();
    auto head = encode_header(header);
    if (!head) return head.error();
    std::vector<std::uint8_t> out = std::move(head).value();
    out.insert(out.end(), body.begin(), body.end());
    return out;
}

Result<ContainerView> parse_container(std::span<const std::uint8_t> bytes) {
    std::size_t n = 0;
    auto h = decode_header(bytes, &n);
    if (!h) return h.error();
    ContainerView v{std::move(h).value(), bytes.subspan(n)};
    if (v.body.size() != v.header.body_size) {
        return make_error(v.body.size() < v.header.body_size ? ErrorCode::Truncated : ErrorCode::Malformed,
                          "body size != header body_size");
    }
    if (content_hash(v.body) != v.header.body_hash) return make_error(ErrorCode::HashMismatch, "body hash mismatch");
    return v;
}

} // namespace g2m
