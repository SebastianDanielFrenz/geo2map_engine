// src/layer/src_osm.cpp - see g2m/layer/src_osm.h and
// docs/formats/src_osm.md.
//
// Every delta chain uses wrap-around uint64 arithmetic, so it is a
// bijection on int64 and never overflows (no UB on hostile input); range
// rules (coordinates, id order) are checked on the decoded values.

#include "g2m/layer/src_osm.h"

#include "g2m/core/bytes.h"

#include <limits>
#include <string>

namespace g2m {
namespace {

using namespace g2m::osm;

std::uint64_t u(std::int64_t v) { return static_cast<std::uint64_t>(v); }
std::int64_t s(std::uint64_t v) { return static_cast<std::int64_t>(v); }

void write_ids(ByteWriter& w, std::span<const std::int64_t> ids) {
    std::uint64_t prev = 0;
    for (std::size_t i = 0; i < ids.size(); ++i) {
        if (i == 0) {
            w.svarint(ids[0]);
        } else {
            w.varint(u(ids[i]) - prev); // >= 1: ids strictly ascending
        }
        prev = u(ids[i]);
    }
}

template <class T>
std::vector<std::int64_t> ids_of(const std::vector<T>& v) {
    std::vector<std::int64_t> ids;
    ids.reserve(v.size());
    for (const T& e : v) ids.push_back(e.id);
    return ids;
}

void write_tags(ByteWriter& w, std::span<const Tag> tags) {
    w.varint(tags.size());
    for (const Tag& t : tags) {
        w.varint(t.key);
        w.varint(t.value);
    }
}

Error bad(const char* what) { return make_error(ErrorCode::Malformed, std::string("src.osm: ") + what); }

// Reads n strictly ascending ids.
bool read_ids(ByteReader& r, std::size_t n, std::vector<std::int64_t>& out) {
    out.clear();
    out.reserve(n);
    for (std::size_t i = 0; i < n; ++i) {
        if (i == 0) {
            out.push_back(r.svarint());
        } else {
            const std::uint64_t d = r.varint();
            const std::int64_t prev = out.back();
            const std::uint64_t room = u(std::numeric_limits<std::int64_t>::max()) - u(prev);
            if (d == 0 || d > room) return false;
            out.push_back(s(u(prev) + d));
        }
        if (!r.ok()) return false;
    }
    return r.at_end();
}

// Reads one element's tags, appending to d.tags.
bool read_tags(ByteReader& r, OsmData& d, std::uint32_t& begin, std::uint32_t& count) {
    const std::uint64_t n = r.varint();
    if (!r.ok() || n > r.remaining() / 2) return false; // each tag >= 2 bytes
    begin = static_cast<std::uint32_t>(d.tags.size());
    count = static_cast<std::uint32_t>(n);
    const std::uint64_t ns = d.strings.size();
    for (std::uint64_t k = 0; k < n; ++k) {
        const std::uint64_t key = r.varint();
        const std::uint64_t val = r.varint();
        if (!r.ok() || key >= ns || val >= ns) return false;
        if (k > 0 && !(d.tags.back().key < key)) return false;
        d.tags.push_back(Tag{static_cast<std::uint32_t>(key), static_cast<std::uint32_t>(val)});
    }
    return true;
}

} // namespace

Result<TileBody> encode_src_osm(const OsmTile& tile) {
    const OsmData& d = tile.data;
    if (auto v = d.validate(); !v) return make_error(ErrorCode::InvalidArgument, v.error().message);
    if (d.strings.size() > 0xffffffffu || d.nodes.size() > 0xffffffffu || d.ways.size() > 0xffffffffu ||
        d.relations.size() > 0xffffffffu) {
        return make_error(ErrorCode::LimitExceeded, "src.osm: too many elements");
    }
    TileBody body;
    body.encoding = static_cast<std::uint16_t>(LayerEncoding::SrcOsm);
    body.encoding_version = kSrcOsmEncodingVersion;
    const auto add = [&](std::uint16_t type, SectionCodec codec, ByteWriter& w) {
        body.sections.push_back(Section{type, codec, w.take()});
    };

    ByteWriter meta;
    meta.u32(tile.halo_m);
    meta.u32(static_cast<std::uint32_t>(d.strings.size()));
    meta.u32(static_cast<std::uint32_t>(d.nodes.size()));
    meta.u32(static_cast<std::uint32_t>(d.ways.size()));
    meta.u32(static_cast<std::uint32_t>(d.relations.size()));
    add(src_osm::kMeta, SectionCodec::None, meta);

    ByteWriter strings;
    for (const std::string& str : d.strings) {
        strings.varint(str.size());
        strings.bytes(str.data(), str.size());
    }
    add(src_osm::kStrings, SectionCodec::Zstd, strings);

    ByteWriter node_ids, lat, lon, node_tags;
    write_ids(node_ids, ids_of(d.nodes));
    std::int64_t plat = 0, plon = 0;
    for (const Node& n : d.nodes) {
        lat.svarint(n.lat - plat);
        lon.svarint(n.lon - plon);
        plat = n.lat;
        plon = n.lon;
        write_tags(node_tags, d.tags_of(n));
    }
    add(src_osm::kNodeIds, SectionCodec::Zstd, node_ids);
    add(src_osm::kNodeLat, SectionCodec::Zstd, lat);
    add(src_osm::kNodeLon, SectionCodec::Zstd, lon);
    add(src_osm::kNodeTags, SectionCodec::Zstd, node_tags);

    ByteWriter way_ids, refs, way_tags;
    write_ids(way_ids, ids_of(d.ways));
    std::uint64_t pref = 0;
    for (const Way& w : d.ways) {
        refs.varint(w.refs_count);
        for (std::int64_t ref : d.refs_of(w)) {
            refs.svarint(s(u(ref) - pref));
            pref = u(ref);
        }
        write_tags(way_tags, d.tags_of(w));
    }
    add(src_osm::kWayIds, SectionCodec::Zstd, way_ids);
    add(src_osm::kWayRefs, SectionCodec::Zstd, refs);
    add(src_osm::kWayTags, SectionCodec::Zstd, way_tags);

    ByteWriter rel_ids, mem, rel_tags;
    write_ids(rel_ids, ids_of(d.relations));
    std::uint64_t pmem = 0;
    for (const Relation& r : d.relations) {
        mem.varint(r.members_count);
        for (const Member& m : d.members_of(r)) {
            mem.varint(static_cast<std::uint8_t>(m.type));
            mem.svarint(s(u(m.ref) - pmem));
            mem.varint(m.role);
            pmem = u(m.ref);
        }
        write_tags(rel_tags, d.tags_of(r));
    }
    add(src_osm::kRelationIds, SectionCodec::Zstd, rel_ids);
    add(src_osm::kRelationMembers, SectionCodec::Zstd, mem);
    add(src_osm::kRelationTags, SectionCodec::Zstd, rel_tags);
    return body;
}

Result<OsmTile> decode_src_osm(const TileBody& body) {
    if (body.encoding != static_cast<std::uint16_t>(LayerEncoding::SrcOsm)) {
        return make_error(ErrorCode::Unsupported, "not a src.osm body");
    }
    if (body.encoding_version != kSrcOsmEncodingVersion) return make_error(ErrorCode::Unsupported, "src.osm version");
    if (body.sections.size() != 12) return bad("expected sections 1..12");
    for (std::size_t i = 0; i < 12; ++i) {
        if (body.sections[i].type != i + 1) return bad("expected sections 1..12");
    }
    const auto sec = [&](std::uint16_t type) { return std::span<const std::uint8_t>(body.sections[type - 1u].data); };

    OsmTile tile;
    OsmData& d = tile.data;
    ByteReader meta(sec(src_osm::kMeta));
    tile.halo_m = meta.u32();
    const std::uint32_t n_strings = meta.u32();
    const std::uint32_t n_nodes = meta.u32();
    const std::uint32_t n_ways = meta.u32();
    const std::uint32_t n_rels = meta.u32();
    if (!meta.at_end()) return bad("meta size");
    // Every element costs at least one byte in its id column, every string
    // at least one byte: bound the counts before reserving anything.
    if (n_strings > sec(src_osm::kStrings).size() || n_nodes > sec(src_osm::kNodeIds).size() ||
        n_ways > sec(src_osm::kWayIds).size() || n_rels > sec(src_osm::kRelationIds).size()) {
        return bad("element count exceeds its column");
    }

    {
        ByteReader r(sec(src_osm::kStrings));
        d.strings.reserve(n_strings);
        for (std::uint32_t i = 0; i < n_strings; ++i) {
            const std::uint64_t len = r.varint();
            if (!r.ok() || len > kMaxStringLength || len > r.remaining()) return bad("string");
            d.strings.emplace_back(r.string(static_cast<std::size_t>(len)));
            if (i > 0 && !(d.strings[i - 1] < d.strings[i])) return bad("strings not sorted/unique");
        }
        if (!r.at_end()) return bad("strings trailing bytes");
    }

    std::vector<std::int64_t> ids;
    // Nodes.
    {
        ByteReader rid(sec(src_osm::kNodeIds));
        if (!read_ids(rid, n_nodes, ids)) return bad("node ids");
        ByteReader rlat(sec(src_osm::kNodeLat)), rlon(sec(src_osm::kNodeLon)), rtag(sec(src_osm::kNodeTags));
        d.nodes.reserve(n_nodes);
        std::int64_t plat = 0, plon = 0;
        for (std::uint32_t i = 0; i < n_nodes; ++i) {
            const std::int64_t la = s(u(plat) + u(rlat.svarint()));
            const std::int64_t lo = s(u(plon) + u(rlon.svarint()));
            if (!rlat.ok() || !rlon.ok() || la < -kMaxLatE7 || la > kMaxLatE7 || lo < -kMaxLonE7 || lo > kMaxLonE7) {
                return bad("node coordinate");
            }
            plat = la;
            plon = lo;
            Node n{ids[i], static_cast<std::int32_t>(la), static_cast<std::int32_t>(lo), 0, 0};
            if (!read_tags(rtag, d, n.tags_begin, n.tags_count)) return bad("node tags");
            d.nodes.push_back(n);
        }
        if (!rlat.at_end() || !rlon.at_end() || !rtag.at_end()) return bad("node columns trailing bytes");
    }
    // Ways.
    {
        ByteReader rid(sec(src_osm::kWayIds));
        if (!read_ids(rid, n_ways, ids)) return bad("way ids");
        ByteReader rref(sec(src_osm::kWayRefs)), rtag(sec(src_osm::kWayTags));
        d.ways.reserve(n_ways);
        std::uint64_t pref = 0;
        for (std::uint32_t i = 0; i < n_ways; ++i) {
            const std::uint64_t cnt = rref.varint();
            if (!rref.ok() || cnt == 0 || cnt > rref.remaining()) return bad("way ref count");
            Way w{ids[i], static_cast<std::uint32_t>(d.way_refs.size()), static_cast<std::uint32_t>(cnt), 0, 0};
            for (std::uint64_t k = 0; k < cnt; ++k) {
                pref += u(rref.svarint());
                if (!rref.ok()) return bad("way refs");
                if (!d.find_node(s(pref))) return bad("way ref to a node not in the tile");
                d.way_refs.push_back(s(pref));
            }
            if (!read_tags(rtag, d, w.tags_begin, w.tags_count)) return bad("way tags");
            d.ways.push_back(w);
        }
        if (!rref.at_end() || !rtag.at_end()) return bad("way columns trailing bytes");
    }
    // Relations.
    {
        ByteReader rid(sec(src_osm::kRelationIds));
        if (!read_ids(rid, n_rels, ids)) return bad("relation ids");
        ByteReader rmem(sec(src_osm::kRelationMembers)), rtag(sec(src_osm::kRelationTags));
        d.relations.reserve(n_rels);
        std::uint64_t pmem = 0;
        for (std::uint32_t i = 0; i < n_rels; ++i) {
            const std::uint64_t cnt = rmem.varint();
            if (!rmem.ok() || cnt > rmem.remaining() / 3) return bad("member count"); // each member >= 3 bytes
            Relation rel{ids[i], static_cast<std::uint32_t>(d.members.size()), static_cast<std::uint32_t>(cnt), 0, 0};
            for (std::uint64_t k = 0; k < cnt; ++k) {
                const std::uint64_t type = rmem.varint();
                pmem += u(rmem.svarint());
                const std::uint64_t role = rmem.varint();
                if (!rmem.ok() || type > 2 || role >= d.strings.size()) return bad("member");
                d.members.push_back(Member{static_cast<MemberType>(type), s(pmem), static_cast<std::uint32_t>(role)});
            }
            if (!read_tags(rtag, d, rel.tags_begin, rel.tags_count)) return bad("relation tags");
            d.relations.push_back(rel);
        }
        if (!rmem.at_end() || !rtag.at_end()) return bad("relation columns trailing bytes");
    }
    return tile;
}

} // namespace g2m
